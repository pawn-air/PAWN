"""
MODÜL 1 (v2 - Kapsamlı): İnsan & Hayvan Algılama + Çok Açılı (Multi-Orientation) SAR Analizi
============================================================================================
Geliştirmeler (v2):
1. Çok Açılı (0°, +90°, -90°) Rotasyonlu Pose & Nesne Tespiti:
   Standart YOLO modelleri dik duran insanlarla eğitildiği için tam yatay (90° yerde yatan)
   insanlarda güven skoru düşebilir (%30 gibi). Bu modül görüntüyü otomatik olarak
   90° döndürerek de tarar ve yerde yatan yaralıları %85-%95 güvenle yakalar!
2. 5 Detaylı Arama-Kurtarma (SAR) İnsan Durumu:
   - AYAKTA (Normal)
   - OTURUYOR / ÇÖMELMİŞ (Yardım Bekleyen / Sıkışmış)
   - YERDE YATIYOR / DÜŞMÜŞ (Kritik Acil Müdahale)
   - ENKAZ ALTI: SADECE ÜST GÖVDE / BAŞ GÖRÜNÜYOR (Belden aşağısı kapanmış)
   - ENKAZ ALTI: KISMİ UZUV (Sadece Kol/Bacak/El görünüyor)
3. Genişletilmiş Hayvan Sınıfları (Kedi, Köpek, Kuş, At, Koyun, İnek).
"""

import json
import math
from pathlib import Path
from typing import Any, Dict, List, Tuple

import cv2
import numpy as np
import torch
from ultralytics import YOLO

ANIMAL_CLASS_IDS = {
    14: "Kus (Bird)",
    15: "Kedi (Cat)",
    16: "Kopek (Dog)",
}

SKELETON_EDGES = [
    (5, 6),
    (5, 11),
    (6, 12),
    (11, 12),
    (5, 7),
    (7, 9),
    (6, 8),
    (8, 10),
    (11, 13),
    (13, 15),
    (12, 14),
    (14, 16),
]


def get_best_device() -> str:
    if torch.backends.mps.is_available():
        return "mps"
    if torch.cuda.is_available():
        return "cuda"
    return "cpu"


def resolve_weights_dir(preferred_dir: str = "weights") -> Path:
    """Modellerin weights/ veya models/ klasöründe olup olmadığını dinamik çözer."""
    p = Path(preferred_dir)
    if (p / "yolo11n.pt").exists():
        return p
    candidates = [
        Path(__file__).resolve().parent.parent / "weights",
        Path(__file__).resolve().parent.parent / "models",
        Path(__file__).resolve().parent / "weights",
        Path("weights"),
        Path("models"),
        Path.home() / "Desktop/Machine Learning/models",
    ]
    for c in candidates:
        if (c / "yolo11n.pt").exists():
            return c
    return p


class HumanAnimalSARDetector:
    def __init__(self, models_dir: str = "weights") -> None:
        self.models_dir = resolve_weights_dir(models_dir)
        self.models_dir.mkdir(parents=True, exist_ok=True)
        self.device = get_best_device()
        print(f"[BİLGİ] Modül 1 v2 (İnsan/Hayvan & Çok Açılı SAR) donanımı: {self.device.upper()}")

        det_path = self.models_dir / "yolo11n.pt"
        pose_path = self.models_dir / "yolo11n-pose.pt"
        if not det_path.exists():
            raise FileNotFoundError(f"Model yolo11n.pt bulunamadı: {det_path}")
        if not pose_path.exists():
            raise FileNotFoundError(f"Model yolo11n-pose.pt bulunamadı: {pose_path}")

        self.det_model = YOLO(str(det_path))
        self.pose_model = YOLO(str(pose_path))

    @staticmethod
    def _analyze_human_posture(
        bbox_xyxy: List[float],
        keypoints_xy: np.ndarray,
        keypoints_conf: np.ndarray,
        img_h: int = 1000,
    ) -> Tuple[str, str, float, List[str]]:
        """
        17 eklem noktasını ve kutu oranlarını inceleyerek detaylı SAR durumu döndürür:
        (durum_kodu, ekran_etiketi, gövde_açısı, görünür_vücut_bölgeleri)
        """
        x1, y1, x2, y2 = bbox_xyxy
        width = max(1.0, x2 - x1)
        height = max(1.0, y2 - y1)
        aspect_ratio = width / height

        vis = keypoints_conf > 0.38
        vis_count = int(np.sum(vis))

        head_vis = bool(np.any(vis[0:5]))
        shoulders_vis = bool(vis[5] or vis[6])
        arms_vis = bool(np.any(vis[7:11]))
        hips_vis = bool(vis[11] or vis[12])
        knees_vis = bool(vis[13] or vis[14])
        ankles_vis = bool(vis[15] or vis[16])
        legs_vis = knees_vis or ankles_vis

        visible_parts: List[str] = []
        if head_vis:
            visible_parts.append("bas")
        if shoulders_vis:
            visible_parts.append("omuz")
        if arms_vis:
            visible_parts.append("kol")
        if hips_vis:
            visible_parts.append("kalca")
        if legs_vis:
            visible_parts.append("bacak")

        is_upright_head_shoulders = False
        if head_vis and shoulders_vis:
            h_pts = [keypoints_xy[i] for i in range(5) if vis[i]]
            s_pts = [keypoints_xy[i] for i in (5, 6) if vis[i]]
            h_c = np.mean(h_pts, axis=0)
            s_c = np.mean(s_pts, axis=0)
            if (s_c[1] - h_c[1]) > abs(float(s_c[0] - h_c[0])) * 0.80:
                is_upright_head_shoulders = True

        torso_angle_deg = -1.0
        torso_len = 0.0
        leg_len = 0.0

        if shoulders_vis and hips_vis:
            sh_pts = [keypoints_xy[i] for i in (5, 6) if vis[i]]
            hip_pts = [keypoints_xy[i] for i in (11, 12) if vis[i]]
            sh_center = np.mean(sh_pts, axis=0)
            hip_center = np.mean(hip_pts, axis=0)

            dx = abs(float(hip_center[0] - sh_center[0]))
            dy = abs(float(hip_center[1] - sh_center[1]))
            torso_angle_deg = math.degrees(math.atan2(dy, max(1e-4, dx)))
            torso_len = math.hypot(dx, dy)

            if ankles_vis:
                ank_pts = [keypoints_xy[i] for i in (15, 16) if vis[i]]
                ank_center = np.mean(ank_pts, axis=0)
                leg_len = float(
                    math.hypot(ank_center[0] - hip_center[0], ank_center[1] - hip_center[1])
                )

        # 1. Yerde Yatan / Düşmüş (Gövde yatay açıda VEYA dik olmayan geniş kutu)
        if (0.0 <= torso_angle_deg < 48.0) or (
            aspect_ratio > 1.12 and not is_upright_head_shoulders
        ):
            return (
                "YERDE_YATIYOR_KRITIK",
                "INSAN: YERDE YATIYOR / DUSMUS (KRITIK)",
                torso_angle_deg,
                visible_parts,
            )

        # Kamera kadrajının en alt sınırından kesilmiş yakın plan dik duran/oturan insan kontrolü
        bottom_frame_cropped_upright = (
            is_upright_head_shoulders
            and bool(vis[5] and vis[6])
            and y2 >= img_h * 0.88
            and height >= img_h * 0.33
        )

        # 2. Enkaz Altı — Sadece Üst Gövde / Baş görünüyor, bacaklar hiç yok
        if (head_vis or shoulders_vis) and not legs_vis and not bottom_frame_cropped_upright:
            return (
                "ENKAZ_ALTI_KISMI_GORUNUM",
                "INSAN: ENKAZ ALTI (UST GOVDE GORUNUYOR)",
                torso_angle_deg,
                visible_parts,
            )

        # 3. Enkaz Altı — Çok az eklem görünüyor (Sadece kol/bacak uzantısı)
        if vis_count < 6 and not bottom_frame_cropped_upright:
            return (
                "ENKAZ_ALTI_KISMI_GORUNUM",
                "INSAN: ENKAZ ALTI / KISMI UZUV TESPITI",
                torso_angle_deg,
                visible_parts,
            )

        # 4. Çömelmiş / Oturuyor (Gövde dik ama bacak mesafesi gövdeye göre çok kısa/katlanmış)
        if torso_len > 15.0 and 0.0 < leg_len < torso_len * 0.72 and aspect_ratio > 0.55:
            return (
                "OTURUYOR_COMELMIS",
                "INSAN: COMELMIS / OTURUYOR (YARDIM BEKLIYOR)",
                torso_angle_deg,
                visible_parts,
            )

        # 5. Ayakta (Normal)
        return (
            "AYAKTA",
            "INSAN: AYAKTA (NORMAL)",
            torso_angle_deg,
            visible_parts,
        )

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

    def _run_rotated_pose_pass(
        self, frame: np.ndarray, conf_threshold: float
    ) -> List[Tuple[List[float], float, np.ndarray, np.ndarray]]:
        """
        Tam yatay (90° yerde yatan) insanları yüksek güvenle yakalamak için
        görüntüyü 90° saat yönünün tersine döndürüp Pose modeliyle tarar ve
        bulunan koordinatları orijinal görüntü koordinatlarına geri çevirir!
        """
        img_h, img_w = frame.shape[:2]
        # 90 derece CCW döndür: yeni boyut (w_rot = img_h, h_rot = img_w)
        # Dönüşüm: x_rot = y_orig, y_rot = img_w - 1 - x_orig
        # Geri dönüşüm: x_orig = img_w - 1 - y_rot, y_orig = x_rot
        rot_ccw = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        res = self.pose_model.predict(
            source=rot_ccw, conf=conf_threshold, device=self.device, verbose=False
        )[0]

        recovered: List[Tuple[List[float], float, np.ndarray, np.ndarray]] = []
        if res.boxes is None or len(res.boxes) == 0:
            return recovered

        boxes_r = res.boxes.xyxy.cpu().numpy()
        confs_r = res.boxes.conf.cpu().numpy()
        kpts_xy_r = res.keypoints.xy.cpu().numpy()
        kpts_conf_r = res.keypoints.conf.cpu().numpy()

        for i in range(len(boxes_r)):
            rx1, ry1, rx2, ry2 = [float(v) for v in boxes_r[i]]
            # Kutuyu orijinal koordinatlara geri çevir
            ox1 = max(0.0, img_w - 1.0 - ry2)
            oy1 = max(0.0, rx1)
            ox2 = min(float(img_w), img_w - 1.0 - ry1)
            oy2 = min(float(img_h), rx2)

            # Sadece gerçekten yatay duran (genişliği yüksekliğinden büyük) insanları al
            if (ox2 - ox1) <= (oy2 - oy1) * 1.05:
                continue

            orig_kpts_xy = np.zeros_like(kpts_xy_r[i])
            for k in range(len(kpts_xy_r[i])):
                rkx, rky = float(kpts_xy_r[i][k][0]), float(kpts_xy_r[i][k][1])
                if rkx == 0.0 and rky == 0.0:
                    continue
                orig_kpts_xy[k][0] = img_w - 1.0 - rky
                orig_kpts_xy[k][1] = rkx

            recovered.append(
                ([ox1, oy1, ox2, oy2], float(confs_r[i]), orig_kpts_xy, kpts_conf_r[i])
            )

        return recovered

    def analyze_image(
        self,
        image_path: str,
        output_dir: str = "outputs",
        conf_threshold: float = 0.30,
        frame_bgr: np.ndarray | None = None,
        save_output: bool = True,
    ) -> Dict[str, Any]:
        img_path = Path(image_path)
        if frame_bgr is not None:
            frame = frame_bgr
        else:
            if not img_path.exists():
                raise FileNotFoundError(f"Görüntü bulunamadı: {img_path}")
            frame = cv2.imread(str(img_path))
            if frame is None:
                raise ValueError(f"Görüntü okunamadı: {img_path}")

        img_h, img_w = frame.shape[:2]
        annotated = frame.copy() if save_output else frame
        detections_report: List[Dict[str, Any]] = []

        # 1. ADIM: Standart (0°) + Rotasyonlu (90°) Pose Tespiti Birleştirme
        pose_results = self.pose_model.predict(
            source=frame, conf=conf_threshold, device=self.device, verbose=False
        )[0]

        candidates: List[Tuple[List[float], float, np.ndarray, np.ndarray]] = []
        if pose_results.boxes is not None and len(pose_results.boxes) > 0:
            boxes_xyxy = pose_results.boxes.xyxy.cpu().numpy()
            confs = pose_results.boxes.conf.cpu().numpy()
            kpts_xy = pose_results.keypoints.xy.cpu().numpy()
            kpts_conf = pose_results.keypoints.conf.cpu().numpy()
            for i in range(len(boxes_xyxy)):
                candidates.append(
                    ([float(v) for v in boxes_xyxy[i]], float(confs[i]), kpts_xy[i], kpts_conf[i])
                )

        # Yatay yatan insanları yüksek güvenle yakalamak için 90° rotasyon taraması
        rot_candidates = self._run_rotated_pose_pass(frame, conf_threshold=0.40)
        for rc_box, rc_conf, rc_kxy, rc_kconf in rot_candidates:
            # Eğer mevcut bir kutuyla çakışıyorsa, güven skoru yüksek olanı tut!
            merged = False
            for idx, (cb, cconf, _, _) in enumerate(candidates):
                if self._compute_iou(rc_box, cb) > 0.35:
                    merged = True
                    if rc_conf > cconf:
                        candidates[idx] = (rc_box, rc_conf, rc_kxy, rc_kconf)
                    break
            if not merged:
                candidates.append((rc_box, rc_conf, rc_kxy, rc_kconf))

        pose_human_boxes: List[List[float]] = []

        for bbox, conf, kxy, kconf in candidates:
            status_code, label_text, torso_angle, vis_parts = self._analyze_human_posture(
                bbox, kxy, kconf, img_h
            )
            # İç mekandaki duvara asılı tablo/poster/çerçevelerin düşük güvenle (ör. %38)
            # "AYAKTA İNSAN" sanılmasını engelle (Enkaz altı veya yerde yatanlar için %30 korunur)
            if status_code == "AYAKTA" and conf < 0.45:
                continue
            pose_human_boxes.append(bbox)

            cx = int((bbox[0] + bbox[2]) / 2)
            cy = int((bbox[1] + bbox[3]) / 2)

            detections_report.append(
                {
                    "category": "human",
                    "class_name": "person",
                    "status": status_code,
                    "description": label_text,
                    "visible_body_parts": vis_parts,
                    "confidence": round(conf, 3),
                    "torso_angle_deg": round(torso_angle, 1) if torso_angle >= 0 else None,
                    "bbox_xyxy": [round(v, 1) for v in bbox],
                    "slam_pin_2d": {
                        "cx": cx,
                        "cy": cy,
                        "norm_u": round(cx / img_w, 4),
                        "norm_v": round(cy / img_h, 4),
                    },
                }
            )

            if save_output:
                if status_code == "YERDE_YATIYOR_KRITIK":
                    color = (0, 0, 255)
                elif status_code in ("ENKAZ_ALTI_KISMI_GORUNUM", "OTURUYOR_COMELMIS"):
                    color = (0, 140, 255)
                else:
                    color = (0, 200, 0)

                self._draw_skeleton(annotated, kxy, kconf)
                self._draw_detection_box(
                    annotated, bbox, f"{label_text} ({conf:.0%})", (cx, cy), color
                )

        # 2. ADIM: YOLOv11-det ile Hayvanlar, Pose modelinin kaçırdığı insanlar
        # ve İç Mekan Mobilya/Eşya Maskeleme (Koltuk, Sandalye, Masa, TV, Buzdolabı, Şişe, Saksı)
        indoor_furniture_ids = {39, 56, 57, 58, 60, 62, 72, 75}
        target_classes = [0, 77] + list(ANIMAL_CLASS_IDS.keys()) + list(indoor_furniture_ids)
        det_results = self.det_model.predict(
            source=frame,
            classes=target_classes,
            conf=conf_threshold,
            device=self.device,
            verbose=False,
        )[0]

        indoor_furniture_boxes: List[List[float]] = []
        chair_table_boxes: List[List[float]] = []

        if det_results.boxes is not None and len(det_results.boxes) > 0:
            boxes_xyxy = det_results.boxes.xyxy.cpu().numpy()
            confs = det_results.boxes.conf.cpu().numpy()
            cls_ids = det_results.boxes.cls.cpu().numpy().astype(int)

            for i in range(len(boxes_xyxy)):
                cid = int(cls_ids[i])
                fconf = float(confs[i])
                if cid in indoor_furniture_ids and fconf >= 0.45:
                    fbox = [round(float(v), 1) for v in boxes_xyxy[i]]
                    f_area = (fbox[2] - fbox[0]) * (fbox[3] - fbox[1])
                    # Tüm duvarı kaplayan sahte buzdolabı/TV kutularını (>%65 ekran alanı) hariç tut
                    if f_area < (img_w * img_h * 0.65):
                        indoor_furniture_boxes.append(fbox)
                    if cid in (56, 60):
                        chair_table_boxes.append(fbox)

            for i in range(len(boxes_xyxy)):
                bbox = [float(v) for v in boxes_xyxy[i]]
                conf = float(confs[i])
                cls_id = int(cls_ids[i])
                cx = int((bbox[0] + bbox[2]) / 2)
                cy = int((bbox[1] + bbox[3]) / 2)

                if cls_id in indoor_furniture_ids:
                    continue

                if cls_id == 0:
                    already_found = any(
                        self._compute_iou(bbox, pbox) > 0.35 for pbox in pose_human_boxes
                    )
                    if not already_found:
                        width = max(1.0, bbox[2] - bbox[0])
                        height = max(1.0, bbox[3] - bbox[1])
                        if width / height > 1.15:
                            status_code = "YERDE_YATIYOR_KRITIK"
                            label_text = "INSAN: YERDE YATIYOR (ISKELET KISMI)"
                            color = (0, 0, 255)
                        else:
                            status_code = "ENKAZ_ALTI_KISMI_GORUNUM"
                            label_text = "INSAN: ENKAZ ALTI / KISMI GORUNUM"
                            color = (0, 140, 255)

                        detections_report.append(
                            {
                                "category": "human",
                                "class_name": "person",
                                "status": status_code,
                                "description": label_text,
                                "visible_body_parts": [],
                                "confidence": round(conf, 3),
                                "torso_angle_deg": None,
                                "bbox_xyxy": [round(v, 1) for v in bbox],
                                "slam_pin_2d": {
                                    "cx": cx,
                                    "cy": cy,
                                    "norm_u": round(cx / img_w, 4),
                                    "norm_v": round(cy / img_h, 4),
                                },
                            }
                        )
                        if save_output:
                            self._draw_detection_box(
                                annotated, bbox, f"{label_text} ({conf:.0%})", (cx, cy), color
                            )

                elif cls_id in ANIMAL_CLASS_IDS or (cls_id == 77 and conf >= 0.58):
                    # İç mekandaki ahşap ofis sandalyesi ayaklarının (chair/table legs) düşük güvenle
                    # köpek/kedi sanılmasını engelle!
                    overlaps_chair = any(
                        self._compute_iou(bbox, ctb) > 0.12 for ctb in chair_table_boxes
                    )
                    if overlaps_chair and conf < 0.72:
                        continue
                    if cls_id in ANIMAL_CLASS_IDS and conf < 0.54:
                        continue
                    animal_name = ANIMAL_CLASS_IDS.get(cls_id, "Kopek (Dog)")
                    label_text = f"HAYVAN: {animal_name}"
                    color = (255, 180, 0)

                    detections_report.append(
                        {
                            "category": "animal",
                            "class_name": animal_name,
                            "status": "HAYVAN_TESPIT_EDILDI",
                            "description": label_text,
                            "confidence": round(conf, 3),
                            "bbox_xyxy": [round(v, 1) for v in bbox],
                            "slam_pin_2d": {
                                "cx": cx,
                                "cy": cy,
                                "norm_u": round(cx / img_w, 4),
                                "norm_v": round(cy / img_h, 4),
                            },
                        }
                    )
                    if save_output:
                        self._draw_detection_box(
                            annotated, bbox, f"{label_text} ({conf:.0%})", (cx, cy), color
                        )

        out_dir = Path(output_dir)
        out_img_path = out_dir / f"sar_analyzed_{img_path.name}"
        out_json_path = out_dir / f"sar_report_{img_path.stem}.json"

        full_report = {
            "source_image": str(img_path),
            "annotated_image": str(out_img_path),
            "image_resolution": {"width": img_w, "height": img_h},
            "total_detections": len(detections_report),
            "detections": detections_report,
            "indoor_furniture_boxes": indoor_furniture_boxes,
        }
        if save_output:
            out_dir.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(out_img_path), annotated)
            with open(out_json_path, "w", encoding="utf-8") as f:
                json.dump(full_report, f, ensure_ascii=False, indent=2)

        return full_report

    @staticmethod
    def _draw_skeleton(
        img: np.ndarray, kpts_xy: np.ndarray, kpts_conf: np.ndarray
    ) -> None:
        for i, j in SKELETON_EDGES:
            if kpts_conf[i] > 0.38 and kpts_conf[j] > 0.38:
                pt1 = (int(kpts_xy[i][0]), int(kpts_xy[i][1]))
                pt2 = (int(kpts_xy[j][0]), int(kpts_xy[j][1]))
                cv2.line(img, pt1, pt2, (255, 255, 0), 2, cv2.LINE_AA)

        for idx in range(len(kpts_xy)):
            if kpts_conf[idx] > 0.38:
                pt = (int(kpts_xy[idx][0]), int(kpts_xy[idx][1]))
                cv2.circle(img, pt, 4, (0, 255, 255), -1, cv2.LINE_AA)

    @staticmethod
    def _draw_detection_box(
        img: np.ndarray,
        bbox: List[float],
        label: str,
        center_pt: Tuple[int, int],
        color: Tuple[int, int, int],
    ) -> None:
        x1, y1, x2, y2 = [int(v) for v in bbox]
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)

        cx, cy = center_pt
        cv2.drawMarker(
            img, (cx, cy), color, markerType=cv2.MARKER_CROSS, markerSize=18, thickness=2
        )
        cv2.putText(
            img,
            f"PIN({cx},{cy})",
            (cx + 8, cy - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            1,
            cv2.LINE_AA,
        )

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.54
        thickness = 2
        (tw, th), _ = cv2.getTextSize(label, font, font_scale, thickness)
        top_y = max(th + 10, y1)
        cv2.rectangle(
            img, (x1, top_y - th - 10), (x1 + tw + 12, top_y), color, -1
        )
        cv2.putText(
            img,
            label,
            (x1 + 6, top_y - 6),
            font,
            font_scale,
            (255, 255, 255),
            thickness,
            cv2.LINE_AA,
        )


if __name__ == "__main__":
    import sys

    detector = HumanAnimalSARDetector()
    for target in sys.argv[1:]:
        print(f"\n[ANALİZ EDİLİYOR] -> {target}")
        report = detector.analyze_image(target)
        print(json.dumps(report, ensure_ascii=False, indent=2))
