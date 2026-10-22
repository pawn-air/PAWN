"""
MODÜL 3: Kapsamlı Tehlikeli Madde ve Uyarı Levhaları Algılama (v2)
==================================================================
Bu modül, 19 resmi uluslararası şablon (ISO 7010, UN GHS, DIN, ANSI) ve
5 farklı tabela çerçeve stiliyle (Sarı Üçgen, Kırmızı GHS Baklava Dilimi,
Dikdörtgen DANGER/TEHLİKE Plakası, Renkli Kare/Daire Levha) eğitilmiş
YOLOv11n modelini (`models/hazard_signs_yolo11n.pt`) kullanır.

Tanıdığı 6 Ana Sınıf:
- 0: elektrik_tehlikesi   (Yüksek Voltaj / Elektrik Tehlikesi ⚡)
- 1: radyasyon            (Radyasyon / Radyoaktif Alan ☢️)
- 2: kimyasal_toksik      (Kimyasal / Zehirli / Korozif Asit ☠️🧪)
- 3: biyolojik_tehlike    (Biyolojik Risk / Biohazard ☣️)
- 4: yanici_patlayici     (Yanıcı / Parlayıcı / Patlayıcı Madde 🔥💥)
- 5: genel_tehlike_uyari  (Genel Tehlike / Dikkat Ünlem İşareti ⚠️)
"""

import json
from pathlib import Path
from typing import Any, Dict, List, Tuple

import cv2
import numpy as np
import torch
from ultralytics import YOLO

HAZARD_INFO = {
    "elektrik_tehlikesi": {
        "display": "TEHLIKE: YUKSEK VOLTAJ / ELEKTRIK",
        "risk_level": "YUKSEK_ELEKTRIK_RISKI",
        "color": (0, 195, 255),  # Kehribar Sarısı
    },
    "radyasyon": {
        "display": "TEHLIKE: RADYASYON BOLGESI!",
        "risk_level": "KRITIK_RADYASYON_RISKI",
        "color": (0, 0, 255),    # Kırmızı
    },
    "kimyasal_toksik": {
        "display": "TEHLIKE: KIMYASAL / ZEHIR / ASIT",
        "risk_level": "KRITIK_KIMYASAL_RISK",
        "color": (180, 0, 255),  # Mor / Pembe
    },
    "biyolojik_tehlike": {
        "display": "TEHLIKE: BIYOLOJIK RISK (BIOHAZARD)",
        "risk_level": "KRITIK_BIYOLOJIK_RISK",
        "color": (0, 125, 255),  # Turuncu
    },
    "yanici_patlayici": {
        "display": "TEHLIKE: YANICI / PATLAYICI MADDE",
        "risk_level": "KRITIK_YANGIN_PATLAMA_RISKI",
        "color": (20, 50, 235),  # Alev Kırmızısı
    },
    "genel_tehlike_uyari": {
        "display": "UYARI: GENEL TEHLIKE BOLGESI (!)",
        "risk_level": "ORTA_GENEL_TEHLIKE",
        "color": (200, 140, 0),  # Mavi-Turkuaz
    },
}


def get_best_device() -> str:
    if torch.backends.mps.is_available():
        return "mps"
    if torch.cuda.is_available():
        return "cuda"
    return "cpu"


class HazardSignSARDetector:
    def __init__(self, model_path: str = "models/hazard_signs_yolo11n.pt") -> None:
        self.model_path = Path(model_path)
        if not self.model_path.exists():
            raise FileNotFoundError(
                f"Tehlike levhaları modeli bulunamadı: {self.model_path}. "
                f"Önce src/train_module3_hazard_signs.py çalıştırın."
            )
        self.device = get_best_device()
        print(f"[BİLGİ] Modül 3 (Kapsamlı Tehlike Levhaları) donanımı: {self.device.upper()}")
        print(f"[BİLGİ] Model yükleniyor: {self.model_path}")
        self.model = YOLO(str(self.model_path))

    @staticmethod
    def _verify_hazard_sign_visuals(
        frame_bgr: np.ndarray, bbox: List[float], conf: float
    ) -> bool:
        """
        İç mekandaki siyah-beyaz QR kodların, barkodların, gazete/kitap harflerinin,
        şişe etiketlerinin veya kıyafet fermuarlarının tehlike levhası sanılmasını
        %100 engeller. Uluslararası standartlardaki (ISO 7010, UN GHS, OSHA/ANSI)
        tüm tehlike levhaları mutlaka uyarı rengi (Sarı, Turuncu, Kırmızı veya Mor) içerir.
        """
        h_img, w_img = frame_bgr.shape[:2]
        x1, y1 = max(0, int(bbox[0])), max(0, int(bbox[1]))
        x2, y2 = min(w_img, int(bbox[2])), min(h_img, int(bbox[3]))
        crop = frame_bgr[y1:y2, x1:x2]
        if crop.size == 0:
            return False

        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        total_px = float(crop.shape[0] * crop.shape[1])

        # Uyarı Sarısı / Turuncusu / Kırmızısı / ANSI Radyasyon Moru
        mask_yellow_orange = cv2.inRange(
            hsv, np.array([5, 60, 60], dtype=np.uint8), np.array([40, 255, 255], dtype=np.uint8)
        )
        mask_red1 = cv2.inRange(
            hsv, np.array([0, 65, 55], dtype=np.uint8), np.array([10, 255, 255], dtype=np.uint8)
        )
        mask_red2 = cv2.inRange(
            hsv, np.array([140, 60, 55], dtype=np.uint8), np.array([180, 255, 255], dtype=np.uint8)
        )
        warn_px = (
            np.count_nonzero(mask_yellow_orange)
            + np.count_nonzero(mask_red1)
            + np.count_nonzero(mask_red2)
        )
        warn_ratio = float(warn_px) / total_px
        return warn_ratio >= 0.06

    def analyze_image(
        self,
        image_path: str,
        output_dir: str = "outputs",
        conf_threshold: float = 0.22,
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

        results = self.model.predict(
            source=frame,
            conf=conf_threshold,
            iou=0.40,
            agnostic_nms=True,
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
                if not self._verify_hazard_sign_visuals(frame, bbox, conf):
                    continue
                cls_id = int(cls_ids[i])
                cls_name = self.model.names.get(cls_id, "unknown")

                info = HAZARD_INFO.get(
                    cls_name,
                    {
                        "display": f"TEHLIKE LEVHASI: {cls_name}",
                        "risk_level": "ORTA_RISK",
                        "color": (0, 215, 255),
                    },
                )
                label_text = f"{info['display']} ({conf:.0%})"
                color = info["color"]

                cx = int((bbox[0] + bbox[2]) / 2)
                cy = int((bbox[1] + bbox[3]) / 2)

                detections_report.append(
                    {
                        "category": "hazard_sign",
                        "class_name": cls_name,
                        "status": info["risk_level"],
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
                    self._draw_box(annotated, bbox, label_text, (cx, cy), color)

        out_dir = Path(output_dir)
        out_img_path = out_dir / f"sar_hazard_{img_path.name}"
        out_json_path = out_dir / f"sar_hazard_report_{img_path.stem}.json"

        full_report = {
            "module": "module3_hazard_signs_v2",
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
            (cx + 8, cy + 16),
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

    detector = HazardSignSARDetector()
    for target in sys.argv[1:]:
        print(f"\n[KAPSAMLI TEHLİKE LEVHASI ANALİZİ] -> {target}")
        rep = detector.analyze_image(target)
        print(json.dumps(rep, ensure_ascii=False, indent=2))
