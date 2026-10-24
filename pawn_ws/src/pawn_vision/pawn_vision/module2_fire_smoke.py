"""
MODÜL 2: Yangın (Ateş) ve Duman Algılama Modülü (Fire & Smoke Detection)
========================================================================
Bu modül, D-Fire veri setiyle özel eğitilmiş (fine-tuned) YOLO modelini
(models/fire_smoke_yolo.pt -> {0: 'fire', 1: 'smoke'}) kullanarak:
1. Görüntüdeki Alev (Fire) ve Duman (Smoke) bölgelerini tespit eder.
2. OpenCV HSV renk/parlaklık analiziyle sahte alarmları (kırmızı kıyafet vb.) filtreler
   ve yangının büyüklüğüne göre Tehlike Seviyesi (Başlangıç Ateşi / Kritik Yangın / Yoğun Duman) üretir.
3. SLAM ekibinin 3D haritaya Yangın/Duman Pini koyabilmesi için merkez piksel (cx, cy)
   ve normalize (u, v) koordinatlarını JSON olarak kaydeder.
"""

import json
from pathlib import Path
from typing import Any, Dict, List, Tuple

import cv2
import numpy as np
import torch
from ultralytics import YOLO


def get_best_device() -> str:
    if torch.backends.mps.is_available():
        return "mps"
    if torch.cuda.is_available():
        return "cuda"
    return "cpu"


def resolve_model_file(filename_or_path: str) -> Path:
    """Modellerin weights/ veya models/ klasöründe olup olmadığını dinamik çözer."""
    p = Path(filename_or_path)
    if p.exists():
        return p
    name = p.name
    candidates = [
        Path(__file__).resolve().parent.parent / "weights" / name,
        Path(__file__).resolve().parent.parent / "models" / name,
        Path(__file__).resolve().parent / "weights" / name,
        Path("weights") / name,
        Path("models") / name,
        Path.home() / "Desktop/Machine Learning/models" / name,
    ]
    for c in candidates:
        if c.exists():
            return c
    return p


class FireSmokeSARDetector:
    def __init__(self, model_path: str = "weights/fire_smoke_yolo.pt") -> None:
        self.model_path = resolve_model_file(model_path)
        if not self.model_path.exists():
            raise FileNotFoundError(
                f"Yangın/Duman model dosyası bulunamadı: {self.model_path}"
            )
        self.device = get_best_device()
        print(f"[BİLGİ] Modül 2 (Yangın & Duman) donanımı: {self.device.upper()}")
        print(f"[BİLGİ] Model yükleniyor: {self.model_path}")
        self.model = YOLO(str(self.model_path))

    @staticmethod
    def _verify_fire_hsv(frame_bgr: np.ndarray, bbox: List[float]) -> float:
        """
        Tespit edilen kutunun içindeki piksellerin gerçekten alev/ateş rengi ve
        parlaklığında (HSV uzayında Kırmızı-Turuncu-Sarı + Yüksek Parlaklık) olup
        olmadığını kontrol eder.
        Döndürdüğü değer: Alev pikseli oranı (0.0 - 1.0)
        """
        h_img, w_img = frame_bgr.shape[:2]
        x1 = max(0, int(bbox[0]))
        y1 = max(0, int(bbox[1]))
        x2 = min(w_img, int(bbox[2]))
        y2 = min(h_img, int(bbox[3]))

        crop = frame_bgr[y1:y2, x1:x2]
        if crop.size == 0:
            return 0.0

        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        # Gerçek alev/ateşin HSV aralığı: Turuncu-Sarı-Kırmızı parlak çekirdek + Koyu Turuncu/Kırmızı alev dili
        # (İç mekandaki düz sarı tavan ampullerini/abajurları ateş sanmayı engeller!)
        lower_warm = np.array([4, 95, 175], dtype=np.uint8)
        upper_warm = np.array([36, 255, 255], dtype=np.uint8)
        mask_warm = cv2.inRange(hsv, lower_warm, upper_warm)

        mask_flame_edge = cv2.inRange(
            hsv, np.array([0, 105, 135], dtype=np.uint8), np.array([19, 255, 255], dtype=np.uint8)
        )
        total_pixels = max(1, crop.shape[0] * crop.shape[1])
        warm_ratio = float(np.count_nonzero(mask_warm)) / total_pixels
        edge_ratio = float(np.count_nonzero(mask_flame_edge)) / total_pixels
        if edge_ratio < 0.025:
            return 0.0
        return warm_ratio

    def analyze_image(
        self,
        image_path: str,
        output_dir: str = "outputs",
        conf_threshold: float = 0.38,
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
        total_img_area = float(img_h * img_w)
        annotated = frame.copy() if save_output else frame
        detections_report: List[Dict[str, Any]] = []

        results = self.model.predict(
            source=frame,
            conf=conf_threshold,
            iou=0.35,
            device=self.device,
            verbose=False,
        )[0]

        if results.boxes is not None and len(results.boxes) > 0:
            boxes_xyxy = results.boxes.xyxy.cpu().numpy()
            confs = results.boxes.conf.cpu().numpy()
            cls_ids = results.boxes.cls.cpu().numpy().astype(int)

            for i in range(len(boxes_xyxy)):
                bbox = [float(v) for v in boxes_xyxy[i]]
                conf = float(confs[i])
                cls_id = int(cls_ids[i])
                cls_name = self.model.names.get(cls_id, "unknown").lower()

                box_area = max(1.0, (bbox[2] - bbox[0]) * (bbox[3] - bbox[1]))
                area_ratio = box_area / total_img_area

                cx = int((bbox[0] + bbox[2]) / 2)
                cy = int((bbox[1] + bbox[3]) / 2)

                if "fire" in cls_name:
                    hsv_ratio = self._verify_fire_hsv(frame, bbox)
                    # Kutunun içinde en az %12 oranında gerçek turuncu/sarı parlak alev pikseli olmalı
                    if hsv_ratio < 0.12:
                        continue

                    if area_ratio > 0.08:
                        status = "KRITIK_YANGIN"
                        label = f"YANGIN: KRITIK ALEV ({conf:.0%})"
                        color = (0, 0, 255)  # Kırmızı
                    else:
                        status = "BASLANGIC_ATESI"
                        label = f"YANGIN: ATES / ALEV ({conf:.0%})"
                        color = (0, 90, 255)  # Koyu Turuncu
                else:
                    hsv_ratio = None
                    status = "DUMAN_TEHLIKESI"
                    label = f"YANGIN: DUMAN TESPITI ({conf:.0%})"
                    color = (130, 130, 130)  # Gri / Duman Rengi

                detections_report.append(
                    {
                        "category": "fire_hazard",
                        "class_name": cls_name,
                        "status": status,
                        "description": label,
                        "confidence": round(conf, 3),
                        "frame_area_ratio": round(area_ratio, 4),
                        "fire_hsv_score": round(hsv_ratio, 3) if hsv_ratio is not None else None,
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
                    self._draw_box(annotated, bbox, label, (cx, cy), color)

        out_dir = Path(output_dir)
        out_img_path = out_dir / f"sar_fire_{img_path.name}"
        out_json_path = out_dir / f"sar_fire_report_{img_path.stem}.json"

        full_report = {
            "module": "module2_fire_smoke",
            "source_image": str(img_path),
            "annotated_image": str(out_img_path),
            "total_detections": len(detections_report),
            "detections": detections_report,
        }
        if save_output:
            out_dir.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(out_img_path), annotated)
            with open(out_json_path, "w", encoding="utf-8") as f:
                json.dump(full_report, f, ensure_ascii=False, indent=2)

        return full_report

    @staticmethod
    def _draw_box(
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
            0.48,
            color,
            1,
            cv2.LINE_AA,
        )

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.58
        thickness = 2
        (tw, th), _ = cv2.getTextSize(label, font, font_scale, thickness)
        top_y = max(th + 10, y1)
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

    detector = FireSmokeSARDetector()
    for target in sys.argv[1:]:
        print(f"\n[YANGIN ANALİZİ] -> {target}")
        rep = detector.analyze_image(target)
        print(json.dumps(rep, ensure_ascii=False, indent=2))
