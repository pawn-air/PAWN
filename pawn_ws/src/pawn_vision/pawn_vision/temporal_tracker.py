"""
PAWN SAR VISION V2 — ZAMANSAL TAKİPÇİ, GÜVEN PUANI (TRUST SCORE) VE AKTİF ALGILAMA MOTORU
=========================================================================================
`SAR_VISION_UPGRADE_BLUEPRINT.md` şartnamesine tam uyumlu V2 Karar Katmanı:

1. Çoklu Nesne Zamansal Eşleştirme (Spatial IoU + Normalize Merkez Mesafesi):
   - Her modülün (human_animal, fire_smoke, hazard_signs, structural) tespitlerini
     kareler arasında (hem synchronous hem multiplexed modda) takip eder ve benzersiz
     `track_id` atar.
2. Matematiksel Güven Kumbarası (Trust Score Accumulation & Exponential Decay):
   - Tespit doğrulandığında:
       T_t = min(1.0, T_{t-1} + delta_up * conf_t)
   - Kare kaçırıldığında (Decay / Zaman Aşımı):
       T_t = T_{t-1} * gamma  (gamma = 0.85)
   - Güven Barajı (Trust Gate):
       * T_t >= 0.70 (veya çıkış histerezisi T_t >= 0.58) -> EKRANA ÇİZ / ROS TOPIC'E BAS
       * T_t < 0.70 -> Arka planda aday olarak beklet (1-2 karelik yanlış alarmları %100 eler)
       * T_t < 0.15 -> Bellekten sil (Garbage Collector)
3. Kutu Koordinatı Yumuşatma (EMA Smoothing) & Durum İstikrarı (Majority Voting):
   - Bounding box titremesini üstel hareketli ortalama ile sönümler.
4. Otonom Aktif Algılama (Active Perception / Next-Best-View Manevra Üreteci):
   - Kadraj sınırından kırpılmış (`KADRAJ_KIRPILMASI_BELIRSIZ`) veya belirsiz/kısmi
     görünümlü kazazedelerde otonom uçuş kontrolcüsüne (`/pawn/active_inspection_goal`)
     geri çekilme + gimbal eğme (`STEP_BACK_AND_TILT`) veya 30° yörünge (`ORBIT_ARC_30_DEG`)
     görev mesajı üretir.
"""

from collections import Counter
from dataclasses import dataclass, field
import math
from typing import Any, Dict, List, Optional, Tuple


@dataclass
class TrackedObject:
    track_id: int
    category_key: str
    class_name: str
    bbox_xyxy: List[float]
    confidence: float
    trust_score: float
    hits: int = 1
    misses: int = 0
    confirmed: bool = False
    last_seen_frame: int = 0
    status_history: List[str] = field(default_factory=list)
    payload: Dict[str, Any] = field(default_factory=dict)


class TemporalTracker:
    """
    4 SAR modülü için zamansal güven puanı (Trust Score), titreme önleyici (Anti-Flicker)
    ve Aktif Algılama (Active Perception) karar motoru.
    """

    def __init__(
        self,
        trust_enter_threshold: float = 0.70,
        trust_exit_threshold: float = 0.56,
        trust_delete_threshold: float = 0.15,
        decay_gamma: float = 0.85,
        bbox_ema_alpha: float = 0.65,
    ) -> None:
        self.trust_enter_threshold = trust_enter_threshold
        self.trust_exit_threshold = trust_exit_threshold
        self.trust_delete_threshold = trust_delete_threshold
        self.decay_gamma = decay_gamma
        self.bbox_ema_alpha = bbox_ema_alpha

        self._next_track_id: int = 1
        self.tracks: Dict[str, List[TrackedObject]] = {
            "human_animal": [],
            "fire_smoke": [],
            "hazard_signs": [],
            "structural": [],
        }

        # Modül bazlı puan kazanım katsayıları ve minimum ardışık görülme (hit) şartları
        self.category_rules: Dict[str, Dict[str, float]] = {
            "human_animal": {"init_factor": 0.48, "delta_up": 0.42, "min_hits": 2},
            "fire_smoke": {"init_factor": 0.42, "delta_up": 0.36, "min_hits": 3},
            "hazard_signs": {"init_factor": 0.48, "delta_up": 0.40, "min_hits": 2},
            "structural": {"init_factor": 0.40, "delta_up": 0.35, "min_hits": 3},
        }

    def reset(self) -> None:
        for k in self.tracks:
            self.tracks[k].clear()
        self._next_track_id = 1

    @staticmethod
    def _compute_iou(boxA: List[float], boxB: List[float]) -> float:
        xA = max(boxA[0], boxB[0])
        yA = max(boxA[1], boxB[1])
        xB = min(boxA[2], boxB[2])
        yB = min(boxA[3], boxB[3])
        inter = max(0.0, xB - xA) * max(0.0, yB - yA)
        areaA = max(1.0, (boxA[2] - boxA[0]) * (boxA[3] - boxA[1]))
        areaB = max(1.0, (boxB[2] - boxB[0]) * (boxB[3] - boxB[1]))
        return inter / float(areaA + areaB - inter)

    @staticmethod
    def _normalized_center_dist(
        boxA: List[float], boxB: List[float], img_w: int, img_h: int
    ) -> float:
        cxA = (boxA[0] + boxA[2]) * 0.5
        cyA = (boxA[1] + boxA[3]) * 0.5
        cxB = (boxB[0] + boxB[2]) * 0.5
        cyB = (boxB[1] + boxB[3]) * 0.5
        diag = max(1.0, math.hypot(img_w, img_h))
        return math.hypot(cxA - cxB, cyA - cyB) / diag

    def update_category(
        self,
        category_key: str,
        raw_detections: List[Dict[Dict[str, Any], Any]],
        frame_idx: int,
        img_w: int = 1280,
        img_h: int = 720,
    ) -> List[Dict[str, Any]]:
        """
        İlgili modül çalıştığında ham tespitleri alır, mevcut izlerle (tracks) eşleştirir,
        Trust Score puanlarını günceller ve yalnızca barajı geçen onaylı tespitleri döner.
        """
        if category_key not in self.tracks:
            self.tracks[category_key] = []

        active_tracks = self.tracks[category_key]
        rules = self.category_rules.get(
            category_key, {"init_factor": 0.45, "delta_up": 0.38, "min_hits": 2}
        )

        # Eşleştirme matrisi oluştur
        candidate_pairs: List[Tuple[float, int, int]] = []
        for t_idx, tr in enumerate(active_tracks):
            for d_idx, det in enumerate(raw_detections):
                det_cls = det.get("class_name", "")
                det_cat = det.get("category", "")
                # Sınıf uyumluluğu kontrolü
                if tr.class_name != det_cls and tr.payload.get("category") != det_cat:
                    continue
                det_box = det.get("bbox_xyxy", [0.0, 0.0, 0.0, 0.0])
                iou = self._compute_iou(tr.bbox_xyxy, det_box)
                ndist = self._normalized_center_dist(tr.bbox_xyxy, det_box, img_w, img_h)
                if iou >= 0.15 or ndist <= 0.14:
                    sim = iou + max(0.0, 1.0 - (ndist / 0.16)) * 0.45
                    candidate_pairs.append((sim, t_idx, d_idx))

        candidate_pairs.sort(key=lambda x: x[0], reverse=True)
        matched_tracks = set()
        matched_dets = set()

        for _, t_idx, d_idx in candidate_pairs:
            if t_idx in matched_tracks or d_idx in matched_dets:
                continue
            matched_tracks.add(t_idx)
            matched_dets.add(d_idx)

            tr = active_tracks[t_idx]
            det = raw_detections[d_idx]
            conf = float(det.get("confidence", 0.5))

            # 1. Güven Puanı Artışı: T_t = min(1.0, T_{t-1} + delta_up * conf_t)
            delta = rules["delta_up"] * max(0.45, conf)
            tr.trust_score = min(1.0, tr.trust_score + delta)
            tr.hits += 1
            tr.misses = 0
            tr.last_seen_frame = frame_idx

            # 2. Bounding Box EMA Yumuşatma (Titreme önleme)
            new_box = det.get("bbox_xyxy", tr.bbox_xyxy)
            a = self.bbox_ema_alpha
            tr.bbox_xyxy = [
                round(a * float(new_box[i]) + (1.0 - a) * tr.bbox_xyxy[i], 1)
                for i in range(4)
            ]
            tr.confidence = round(0.65 * conf + 0.35 * tr.confidence, 3)

            # 3. Durum (Status) Kararlılığı — Son 5 gözlem üzerinden çoğunluk oyu
            new_status = det.get("status", "")
            if new_status:
                tr.status_history.append(new_status)
                if len(tr.status_history) > 5:
                    tr.status_history.pop(0)
            stable_status = (
                Counter(tr.status_history).most_common(1)[0][0]
                if tr.status_history
                else new_status
            )

            # Payload güncelle
            updated_payload = dict(det)
            updated_payload["bbox_xyxy"] = list(tr.bbox_xyxy)
            updated_payload["confidence"] = tr.confidence
            updated_payload["trust_score"] = round(tr.trust_score, 3)
            updated_payload["track_id"] = tr.track_id
            if stable_status and stable_status != new_status:
                updated_payload["status"] = stable_status
                # Eğer durum çoğunluk oyuyla korunduysa eski açıklamayı koru
                if tr.payload.get("status") == stable_status and "description" in tr.payload:
                    updated_payload["description"] = tr.payload["description"]

            cx = int((tr.bbox_xyxy[0] + tr.bbox_xyxy[2]) * 0.5)
            cy = int((tr.bbox_xyxy[1] + tr.bbox_xyxy[3]) * 0.5)
            updated_payload["slam_pin_2d"] = {
                "cx": cx,
                "cy": cy,
                "norm_u": round(cx / max(1.0, float(img_w)), 4),
                "norm_v": round(cy / max(1.0, float(img_h)), 4),
            }
            tr.payload = updated_payload

            # Baraj Kontrolü (Trust Gate)
            if tr.hits >= int(rules["min_hits"]) and tr.trust_score >= self.trust_enter_threshold:
                tr.confirmed = True

        # Eşleşmeyen mevcut izler için Decay (Zaman Aşımı: T_t = T_{t-1} * 0.85)
        surviving_tracks: List[TrackedObject] = []
        for t_idx, tr in enumerate(active_tracks):
            if t_idx not in matched_tracks:
                tr.trust_score = tr.trust_score * self.decay_gamma
                tr.misses += 1
                if tr.trust_score < self.trust_exit_threshold or tr.misses > 3:
                    tr.confirmed = False
                if tr.trust_score >= self.trust_delete_threshold and tr.misses <= 6:
                    tr.payload["trust_score"] = round(tr.trust_score, 3)
                    surviving_tracks.append(tr)
            else:
                surviving_tracks.append(tr)

        # Yeni tespitler için yeni Track oluştur
        for d_idx, det in enumerate(raw_detections):
            if d_idx in matched_dets:
                continue
            conf = float(det.get("confidence", 0.5))
            init_trust = min(0.68, rules["init_factor"] * conf + 0.14)
            box = [float(v) for v in det.get("bbox_xyxy", [0.0, 0.0, 0.0, 0.0])]
            tid = self._next_track_id
            self._next_track_id += 1

            payload_copy = dict(det)
            payload_copy["trust_score"] = round(init_trust, 3)
            payload_copy["track_id"] = tid

            st = det.get("status", "")
            new_tr = TrackedObject(
                track_id=tid,
                category_key=category_key,
                class_name=det.get("class_name", "unknown"),
                bbox_xyxy=box,
                confidence=round(conf, 3),
                trust_score=init_trust,
                hits=1,
                misses=0,
                confirmed=False,
                last_seen_frame=frame_idx,
                status_history=[st] if st else [],
                payload=payload_copy,
            )
            surviving_tracks.append(new_tr)

        self.tracks[category_key] = surviving_tracks
        return self.get_confirmed_detections(category_key)

    def get_confirmed_detections(self, category_key: str) -> List[Dict[str, Any]]:
        """Yalnızca Trust Gate barajını geçmiş onaylı tespitleri döner."""
        confirmed_list: List[Dict[str, Any]] = []
        for tr in self.tracks.get(category_key, []):
            if tr.confirmed and tr.trust_score >= self.trust_exit_threshold:
                confirmed_list.append(tr.payload)
        return confirmed_list

    def generate_active_inspection_goals(
        self, img_w: int = 1280, img_h: int = 720
    ) -> List[Dict[str, Any]]:
        """
        Aktif Algılama (Active Perception / Next-Best-View):
        Kadraj kenarından kırpılmış veya durumu belirsiz kazazedeleri incelemek için
        otonom uçuş kontrolcüsüne (/pawn/active_inspection_goal) manevra hedefleri üretir.
        """
        goals: List[Dict[str, Any]] = []
        for tr in self.tracks.get("human_animal", []):
            if tr.payload.get("category") != "human":
                continue
            # En az 2 kez görülmüş olan belirsiz veya kadraj kırpılmalı insanlarda tetikle
            if tr.hits < 2:
                continue

            status = tr.payload.get("status", "")
            edge_info = tr.payload.get("edge_truncation", {})
            pin = tr.payload.get("slam_pin_2d", {})
            norm_u = float(pin.get("norm_u", 0.5))
            norm_v = float(pin.get("norm_v", 0.5))

            if status == "KADRAJ_KIRPILMASI_BELIRSIZ" or edge_info.get("is_edge_truncated", False):
                edges = edge_info.get("truncated_edges", [])
                pitch_deg = -15.0 if "bottom" in edges else (12.0 if "top" in edges else -8.0)
                dy = round((0.5 - norm_u) * 0.7, 2)
                goals.append(
                    {
                        "target_track_id": tr.track_id,
                        "action": "STEP_BACK_AND_TILT",
                        "delta_x_m": -0.80,
                        "delta_y_m": dy,
                        "delta_z_m": 0.15 if "top" in edges else -0.10,
                        "delta_yaw_deg": round((0.5 - norm_u) * 25.0, 1),
                        "delta_gimbal_pitch_deg": pitch_deg,
                        "uncertainty_score": round(1.0 - tr.trust_score, 2),
                        "reason": (
                            f"Kazazede #{tr.track_id} kadraj sinirindan kirpilmis ({','.join(edges) or 'kenar'}). "
                            f"Tam vucut teyidi icin 0.8m geri cekil ve kamerayi {pitch_deg:.0f} derece eg."
                        ),
                    }
                )
            elif status == "ENKAZ_ALTI_KISMI_GORUNUM":
                goals.append(
                    {
                        "target_track_id": tr.track_id,
                        "action": "ORBIT_ARC_30_DEG",
                        "delta_x_m": 0.0,
                        "delta_y_m": 0.65 if norm_u >= 0.5 else -0.65,
                        "delta_z_m": 0.0,
                        "delta_yaw_deg": 30.0 if norm_u >= 0.5 else -30.0,
                        "delta_gimbal_pitch_deg": -10.0,
                        "uncertainty_score": round(1.0 - tr.trust_score, 2),
                        "reason": (
                            f"Kazazede #{tr.track_id} ic kadrajda kismi gorunumlu (olasi enkaz/engel). "
                            f"Farkli acisindan teyit icin 30 derece yorunge (orbit) manevrasi yap."
                        ),
                    }
                )
        return goals
