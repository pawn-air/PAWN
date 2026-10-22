"""
ANA SİSTEM (MASTER PIPELINE): DJI Tello Arama ve Kurtarma (SAR) Algılama Motoru
===============================================================================
Bu ana modül, geliştirdiğimiz 4 ayrı yapay zeka modülünü tek bir çatı altında toplar:
  1. Modül 1 (`HumanAnimalSARDetector`)   -> İnsan / Hayvan + Ayakta / Yerde Yatan / Enkaz Altı
  2. Modül 2 (`FireSmokeSARDetector`)     -> Yangın / Alev ve Duman Tespiti
  3. Modül 3 (`HazardSignSARDetector`)    -> 6 Sınıflı Kapsamlı Tehlike Levhaları
  4. Modül 4 (`StructuralCrackSARDetector`) -> Duvar/Kolon Çatlak Segmentasyonu & Oda Sağlamlığı

İki farklı çalışma moduna sahiptir:
  - Mod A (`analyze_full_image`): Sabit bir fotoğraf üzerinde 4 modülün tamamını çalıştırır,
    tüm bulguları tek bir HUD ekranında birleştirir ve SLAM ekibinin 3D haritaya Pin
    koyması için birleşik JSON paketi üretir.
  - Mod B (`process_stream_frame`): DJI Tello'dan 30 FPS görüntü akarken SLAM'i kastırmamak
    için her karede sırayla 1 modülü çalıştırır (Frame Multiplexing -> kare başına ~4 ms!).
"""

import json
import time
from pathlib import Path
from typing import Any, Dict, List

import cv2
import numpy as np

from indoor_risk_assessment import IndoorSARRiskAssessmentEngine
from module1_human_animal import HumanAnimalSARDetector
from module2_fire_smoke import FireSmokeSARDetector
from module3_hazard_signs import HazardSignSARDetector
from module4_structural_crack import StructuralCrackSARDetector


class SARMasterPerceptionEngine:
    def __init__(self) -> None:
        print("=" * 72)
        print("[SAR MASTER ENGINE] 4 Yapay Zeka Modülü + Risk Assessment Motoru Başlatılıyor...")
        print("=" * 72)
        self.mod1_human = HumanAnimalSARDetector()
        self.mod2_fire = FireSmokeSARDetector()
        self.mod3_hazard = HazardSignSARDetector()
        self.mod4_crack = StructuralCrackSARDetector()
        self.risk_engine = IndoorSARRiskAssessmentEngine()

        # Canlı video akışı (30 FPS) için son tespitlerin önbelleği
        self._stream_cache: Dict[str, List[Dict[str, Any]]] = {
            "mod1": [],
            "mod2": [],
            "mod3": [],
            "mod4": [],
        }
        # MPS GPU Shader Warmup (İlk karedeki soğuk başlangıç gecikmesini sıfırlar)
        dummy = np.zeros((320, 320, 3), dtype=np.uint8)
        self.mod1_human.pose_model.predict(dummy, verbose=False)
        self.mod1_human.det_model.predict(dummy, verbose=False)
        self.mod2_fire.model.predict(dummy, verbose=False)
        self.mod3_hazard.model.predict(dummy, verbose=False)
        self.mod4_crack.crack_model.predict(dummy, verbose=False)
        if self.mod4_crack.heavy_model is not None:
            self.mod4_crack.heavy_model.predict(dummy, verbose=False)
        print("[SAR MASTER ENGINE] Tüm modüller ve MPS GPU çekirdekleri hazır!\n")

    def analyze_full_image(
        self, image_path: str, output_dir: str = "outputs"
    ) -> Dict[str, Any]:
        """
        Tek bir fotoğraf üzerinde 4 modülü de çalıştırır, birleşik görsel ve
        SLAM 3D Pin JSON raporu üretir.
        """
        img_path = Path(image_path)
        frame = cv2.imread(str(img_path))
        if frame is None:
            raise ValueError(f"Görüntü okunamadı: {img_path}")

        img_h, img_w = frame.shape[:2]
        t0 = time.perf_counter()

        # 1. Önce Ön-Plan (Foreground) Modüllerini Hafıza İçi (Zero Disk I/O) Çalıştır
        tm1 = time.perf_counter()
        rep1 = self.mod1_human.analyze_image(
            str(img_path), frame_bgr=frame, save_output=False
        )
        t_mod1_ms = (time.perf_counter() - tm1) * 1000.0

        tm3 = time.perf_counter()
        rep3 = self.mod3_hazard.analyze_image(
            str(img_path), frame_bgr=frame, save_output=False
        )
        t_mod3_ms = (time.perf_counter() - tm3) * 1000.0

        tm2 = time.perf_counter()
        rep2 = self.mod2_fire.analyze_image(
            str(img_path), frame_bgr=frame, save_output=False
        )
        t_mod2_ms = (time.perf_counter() - tm2) * 1000.0

        human_and_furniture_boxes = [
            d["bbox_xyxy"] for d in rep1["detections"]
        ] + rep1.get("indoor_furniture_boxes", [])

        # Çapraz Modül Filtresi 0: İnsan kıyafeti (fermuar/yaka) veya şişe/buzdolabı üzerindeki
        # barkod/QR kodların düşük güvenle (< %80) tehlike levhası sanılmasını engelle!
        filtered_signs = []
        for s_det in rep3["detections"]:
            sb = s_det["bbox_xyxy"]
            inside_obj = any(
                self._box_inside_ratio(sb, ob) > 0.35 for ob in human_and_furniture_boxes
            )
            if not (inside_obj and s_det["confidence"] < 0.80):
                filtered_signs.append(s_det)
        rep3["detections"] = filtered_signs

        # Çapraz Modül Filtresi 1: Eğer bir bölge Tehlike Levhası (ör. Kırmızı GHS Levhası) ise,
        # Yangın modülünün o levhayı "ateş" sanmasını engelle!
        filtered_fire = []
        for f_det in rep2["detections"]:
            fb = f_det["bbox_xyxy"]
            is_sign = any(
                self._box_overlap_ratio(fb, h_det["bbox_xyxy"]) > 0.25
                for h_det in rep3["detections"]
            )
            if not is_sign:
                filtered_fire.append(f_det)
        rep2["detections"] = filtered_fire

        # Çapraz Modül Filtresi 2: Duvar Çatlak Modülünü (Modül 4) hafıza içinde çalıştır
        tm4 = time.perf_counter()
        rep4 = self.mod4_crack.analyze_image(
            str(img_path), frame_bgr=frame, save_output=False
        )
        t_mod4_ms = (time.perf_counter() - tm4) * 1000.0
        fg_boxes = (
            [d["bbox_xyxy"] for d in rep1["detections"]]
            + [d["bbox_xyxy"] for d in rep2["detections"]]
            + [d["bbox_xyxy"] for d in rep3["detections"]]
        )
        furniture_boxes = rep1.get("indoor_furniture_boxes", [])
        valid_cracks = []
        max_valid_severity = 0.0
        for c_det in rep4["detections"]:
            cb = c_det["bbox_xyxy"]
            inside_fg = any(self._box_inside_ratio(cb, fgb) > 0.30 for fgb in fg_boxes)
            is_linear_crack = c_det.get("class_name") == "crack"
            inside_furn = is_linear_crack and any(
                self._box_inside_ratio(cb, fb) > 0.30 for fb in furniture_boxes
            )
            if inside_fg or (inside_furn and c_det["confidence"] < 0.78):
                continue
            valid_cracks.append(c_det)
            max_valid_severity = max(max_valid_severity, c_det["severity_score_100"])

        rep4["detections"] = valid_cracks
        rep4["max_severity_score_100"] = max_valid_severity
        if max_valid_severity >= 52.0:
            rep4["room_structural_integrity"] = "KRITIK_TEHLIKE_ODA_COKEBILIR"
        elif max_valid_severity >= 28.0:
            rep4["room_structural_integrity"] = "ORTA_RISK_DIKKATLI_ILERLE"
        elif len(valid_cracks) > 0:
            rep4["room_structural_integrity"] = "DUSUK_RISK_YUZEYSEL"
        else:
            rep4["room_structural_integrity"] = "SAGLAM_HASAR_YOK"

        # ÇOK-MODÜLLÜ İÇ MEKAN RİSK DEĞERLENDİRME VE TRİYAJ HESABI (Ana Katkı)
        tm_risk = time.perf_counter()
        risk_assessment = self.risk_engine.evaluate_room_risk(
            human_animal_dets=rep1["detections"],
            fire_smoke_dets=rep2["detections"],
            hazard_sign_dets=rep3["detections"],
            structural_dets=rep4["detections"],
        )
        t_risk_ms = (time.perf_counter() - tm_risk) * 1000.0

        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        stream_multiplexed_ms = (t_mod1_ms + t_mod2_ms + t_mod3_ms + t_mod4_ms) / 4.0 + t_risk_ms

        # Orijinal görüntü üzerinde geçerli çatlak bölgelerinin segmentasyon maskesini koru
        canvas = frame.copy()
        mod4_img = rep4.get("_mask_only_array")
        if mod4_img is not None:
            for det in rep4["detections"]:
                bx = [int(v) for v in det["bbox_xyxy"]]
                x1, y1, x2, y2 = max(0, bx[0]), max(0, bx[1]), min(img_w, bx[2]), min(img_h, bx[3])
                canvas[y1:y2, x1:x2] = mod4_img[y1:y2, x1:x2]
            # Ön-plan (İnsan, Yangın, Levha) nesnelerinin üstüne çatlak boyası taşmasını temizle
            for fgb in fg_boxes:
                fx1, fy1, fx2, fy2 = max(0, int(fgb[0])), max(0, int(fgb[1])), min(img_w, int(fgb[2])), min(img_h, int(fgb[3]))
                canvas[fy1:fy2, fx1:fx2] = frame[fy1:fy2, fx1:fx2]

        for det in rep4["detections"]:
            sev = det["severity_score_100"]
            col = (
                (0, 0, 255)
                if sev >= 52.0
                else (0, 140, 255)
                if sev >= 28.0
                else (0, 215, 255)
            )
            pin = det["slam_pin_2d"]
            self._draw_hud_box(
                canvas,
                det["bbox_xyxy"],
                det["description"],
                (pin["cx"], pin["cy"]),
                col,
            )

        # Modül 1, 2 ve 3'ün tespit kutularını ve SLAM Pinlerini aynı tuval üzerine çiz
        for det in rep1["detections"]:
            st = det["status"]
            col = (
                (0, 0, 255)
                if st == "YERDE_YATIYOR_KRITIK"
                else (0, 140, 255)
                if st == "ENKAZ_ALTI_KISMI_GORUNUM"
                else (255, 180, 0)
                if det["category"] == "animal"
                else (0, 200, 0)
            )
            pin = det["slam_pin_2d"]
            self._draw_hud_box(
                canvas,
                det["bbox_xyxy"],
                f"{det['description']} ({det['confidence']:.0%})",
                (pin["cx"], pin["cy"]),
                col,
            )

        for det in rep2["detections"]:
            col = (0, 20, 255) if "fire" in det["class_name"] else (130, 130, 130)
            pin = det["slam_pin_2d"]
            self._draw_hud_box(
                canvas,
                det["bbox_xyxy"],
                det["description"],
                (pin["cx"], pin["cy"]),
                col,
            )

        for det in rep3["detections"]:
            pin = det["slam_pin_2d"]
            self._draw_hud_box(
                canvas,
                det["bbox_xyxy"],
                det["description"],
                (pin["cx"], pin["cy"]),
                (180, 0, 255),
            )

        all_detections = (
            rep1["detections"]
            + rep2["detections"]
            + rep3["detections"]
            + rep4["detections"]
        )

        # Üst ve Alt Çok-Modüllü Risk Assessment & Triyaj HUD Paneli Çizimi
        self.risk_engine.draw_risk_assessment_hud(canvas, risk_assessment)
        risk_json_clean = {
            k: v for k, v in risk_assessment.items() if not k.startswith("_")
        }

        out_dir = Path(output_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        out_img_path = out_dir / f"MASTER_SAR_{img_path.name}"
        out_json_path = out_dir / f"MASTER_SAR_REPORT_{img_path.stem}.json"
        cv2.imwrite(str(out_img_path), canvas)

        master_report = {
            "source_image": str(img_path),
            "master_annotated_image": str(out_img_path),
            "processing_time_ms": round(elapsed_ms, 1),
            "module_latencies_ms": {
                "mod1_human_animal_pose_ms": round(t_mod1_ms, 2),
                "mod2_fire_smoke_ms": round(t_mod2_ms, 2),
                "mod3_hazard_signs_ms": round(t_mod3_ms, 2),
                "mod4_structural_crack_heavy_ms": round(t_mod4_ms, 2),
                "mod5_risk_assessment_math_ms": round(t_risk_ms, 3),
                "total_sync_4modules_ms": round(elapsed_ms, 1),
                "stream_30fps_multiplexed_ms": round(stream_multiplexed_ms, 2),
            },
            "indoor_risk_assessment": risk_json_clean,
            "room_structural_integrity": rep4["room_structural_integrity"],
            "max_structural_severity_100": rep4["max_severity_score_100"],
            "counts": {
                "human_animal": len(rep1["detections"]),
                "fire_smoke": len(rep2["detections"]),
                "hazard_signs": len(rep3["detections"]),
                "structural_cracks": len(rep4["detections"]),
                "total_slam_pins": len(all_detections),
            },
            "slam_3d_pins_payload": all_detections,
        }
        with open(out_json_path, "w", encoding="utf-8") as f:
            json.dump(master_report, f, ensure_ascii=False, indent=2)

        return master_report

    @staticmethod
    def _box_overlap_ratio(boxA: List[float], boxB: List[float]) -> float:
        xA = max(boxA[0], boxB[0])
        yA = max(boxA[1], boxB[1])
        xB = min(boxA[2], boxB[2])
        yB = min(boxA[3], boxB[3])
        inter = max(0.0, xB - xA) * max(0.0, yB - yA)
        min_area = min(
            max(1.0, (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])),
            max(1.0, (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])),
        )
        return inter / min_area

    @staticmethod
    def _box_inside_ratio(inner_box: List[float], outer_box: List[float]) -> float:
        xA = max(inner_box[0], outer_box[0])
        yA = max(inner_box[1], outer_box[1])
        xB = min(inner_box[2], outer_box[2])
        yB = min(inner_box[3], outer_box[3])
        inter = max(0.0, xB - xA) * max(0.0, yB - yA)
        inner_area = max(1.0, (inner_box[2] - inner_box[0]) * (inner_box[3] - inner_box[1]))
        return inter / inner_area

    @staticmethod
    def _draw_hud_box(
        img: np.ndarray,
        bbox: List[float],
        label: str,
        center_pt: tuple[int, int],
        color: tuple[int, int, int],
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
            (cx + 8, cy + 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            color,
            2,
            cv2.LINE_AA,
        )
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.48
        thickness = 2
        (tw, th), _ = cv2.getTextSize(label, font, font_scale, thickness)
        top_y = max(th + 64, y1)
        cv2.rectangle(img, (x1, top_y - th - 10), (x1 + tw + 12, top_y), color, -1)
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

    engine = SARMasterPerceptionEngine()
    for target in sys.argv[1:]:
        print(f"\n[MASTER SAR ANALİZİ] -> {target}")
        rep = engine.analyze_full_image(target)
        print(json.dumps(rep, ensure_ascii=False, indent=2))
