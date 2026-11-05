"""
MODÜL 2 (V2 - Dinamik Akış & Doku Doğrulamalı): Yangın (Ateş) ve Duman Algılama Modülü
======================================================================================
Geliştirmeler (`SAR_VISION_UPGRADE_BLUEPRINT.md` Adım 3 Uyumlu):
1. Alev (Fire) Doğrulaması (`_verify_fire_hsv`):
   - OpenCV HSV renk/parlaklık analiziyle iç mekandaki sarı ampulleri veya kırmızı eşyaları eler.
2. Duman İç Doku & Fraktal Türbülans Analizi (`_verify_smoke_plume_texture`):
   - Düz gri/krem alçıpan duvarların, parlak yemek masası yüzeylerinin, tavan gölgelerinin
     veya beton zeminlerin duman sanılmasını engeller (iç bölgede Laplacian varyansı ve
     yerel kontrast dağılımı kontrolü).
3. Yoğun Optik Akış (Dense Optical Flow - `cv2.calcOpticalFlowFarneback`):
   - Video akışında arka plan kamera hareketini (ego-motion) çıkarır; hareketsiz düz duvarları
     veya zeminleri eler, yalnızca yukarı yönlü konvektif akış veya türbülanslı dağılım
     gösteren gerçek duman bulutlarını onaylar.
"""

import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

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

        # Video akışlarında (Dense Optical Flow) önceki gri kare önbelleği
        self._prev_gray_small: Optional[np.ndarray] = None

    def reset_temporal_state(self) -> None:
        self._prev_gray_small = None

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

    @staticmethod
    def _verify_smoke_plume_texture(
        frame_bgr: np.ndarray, bbox: List[float], has_active_fire: bool = False
    ) -> bool:
        """
        Düz gri/bej duvarların, masa yansımalarının, tavan gölgelerinin ve beton zeminlerin
        'duman' sanılmasını engeller.
        Gerçek duman bulutu iç bölgesinde (merkez %64) türbülanslı yoğunluk değişimi
        (inner_std >= 18.0 ve inner_lap >= 16.0) barındırır; düz duvar/masa yüzeyinde
        ise iç doku pürüzsüzdür (inner_lap < 16.0).
        """
        h_img, w_img = frame_bgr.shape[:2]
        x1, y1 = max(0, int(bbox[0])), max(0, int(bbox[1]))
        x2, y2 = min(w_img, int(bbox[2])), min(h_img, int(bbox[3]))
        bw, bh = x2 - x1, y2 - y1
        if bw < 24 or bh < 24:
            return False

        # Kutu sınırlarındaki masa/duvar kenar çizgilerinden etkilenmemek için iç %64'lük çekirdeği al
        ix1 = x1 + int(bw * 0.18)
        iy1 = y1 + int(bh * 0.18)
        ix2 = max(ix1 + 8, x2 - int(bw * 0.18))
        iy2 = max(iy1 + 8, y2 - int(bh * 0.18))

        inner = frame_bgr[iy1:iy2, ix1:ix2]
        if inner.size == 0:
            return False

        gray = cv2.cvtColor(inner, cv2.COLOR_BGR2GRAY)
        inner_std = float(np.std(gray))
        inner_lap = float(cv2.Laplacian(gray, cv2.CV_64F).var())

        # Aktif alev varsa eşiği biraz esnet, yoksa sıkı tut
        min_std = 14.0 if has_active_fire else 18.5
        min_lap = 11.0 if has_active_fire else 16.5

        if inner_std < min_std or inner_lap < min_lap:
            return False

        # Sarı/bej boyalı iç mekan duvarlarını (H in [12, 34], S > 42) ele
        hsv = cv2.cvtColor(inner, cv2.COLOR_BGR2HSV)
        mean_h = float(np.mean(hsv[:, :, 0]))
        mean_s = float(np.mean(hsv[:, :, 1]))
        if not has_active_fire and (12.0 <= mean_h <= 34.0) and mean_s > 42.0:
            return False

        return True

    def _verify_smoke_optical_flow(
        self,
        curr_gray_small: np.ndarray,
        bbox: List[float],
        img_w: int,
        img_h: int,
        has_active_fire: bool = False,
    ) -> bool:
        """
        Adım 3 (`cv2.calcOpticalFlowFarneback`):
        Ardışık video kareleri arasında kamera hareketini (medyan arka plan akışı) çıkarır.
        Duman adayı tamamen hareketsiz bir yüzeyse (ne yukarı konvektif akış ne de
        içsel türbülans varsa) elenir.
        """
        if self._prev_gray_small is None or self._prev_gray_small.shape != curr_gray_small.shape:
            return True

        if has_active_fire:
            return True

        flow = cv2.calcOpticalFlowFarneback(
            self._prev_gray_small,
            curr_gray_small,
            None,
            0.5,
            2,
            13,
            2,
            5,
            1.1,
            0,
        )
        sh, sw = curr_gray_small.shape[:2]
        sx = sw / max(1.0, float(img_w))
        sy = sh / max(1.0, float(img_h))

        rx1 = max(0, int(bbox[0] * sx))
        ry1 = max(0, int(bbox[1] * sy))
        rx2 = min(sw, max(rx1 + 4, int(bbox[2] * sx)))
        ry2 = min(sh, max(ry1 + 4, int(bbox[3] * sy)))

        roi_flow = flow[ry1:ry2, rx1:rx2]
        if roi_flow.size == 0:
            return False

        bg_vx = float(np.median(flow[:, :, 0]))
        bg_vy = float(np.median(flow[:, :, 1]))

        rel_vx = roi_flow[:, :, 0] - bg_vx
        rel_vy = roi_flow[:, :, 1] - bg_vy

        mean_rel_vy = float(np.mean(rel_vy))
        turb_std = float(np.std(np.hypot(rel_vx, rel_vy)))

        # Gerçek duman ya yukarı doğru yükselir (mean_rel_vy < -0.18) ya da içsel türbülansa sahiptir
        is_dynamic_plume = (mean_rel_vy < -0.18) or (turb_std >= 0.42)
        return is_dynamic_plume

    def analyze_image(
        self,
        image_path: str,
        output_dir: str = "outputs",
        conf_threshold: float = 0.38,
        frame_bgr: np.ndarray | None = None,
        save_output: bool = True,
        is_video_stream: bool = False,
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

        curr_gray_small: Optional[np.ndarray] = None
        if is_video_stream:
            curr_gray_small = cv2.resize(
                cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (320, 180)
            )

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

            # İlk geçiş: Gerçek alev (fire) var mı kontrol et
            verified_fires: List[Tuple[List[float], float, str, float]] = []
            smoke_candidates: List[Tuple[List[float], float, str]] = []

            for i in range(len(boxes_xyxy)):
                bbox = [float(v) for v in boxes_xyxy[i]]
                conf = float(confs[i])
                cls_id = int(cls_ids[i])
                cls_name = self.model.names.get(cls_id, "unknown").lower()

                if "fire" in cls_name:
                    hsv_ratio = self._verify_fire_hsv(frame, bbox)
                    if hsv_ratio >= 0.12:
                        verified_fires.append((bbox, conf, cls_name, hsv_ratio))
                else:
                    smoke_candidates.append((bbox, conf, cls_name))

            has_active_fire = len(verified_fires) > 0

            # Alevleri ekle
            for bbox, conf, cls_name, hsv_ratio in verified_fires:
                box_area = max(1.0, (bbox[2] - bbox[0]) * (bbox[3] - bbox[1]))
                area_ratio = box_area / total_img_area
                cx = int((bbox[0] + bbox[2]) / 2)
                cy = int((bbox[1] + bbox[3]) / 2)

                if area_ratio > 0.08:
                    status = "KRITIK_YANGIN"
                    label = f"YANGIN: KRITIK ALEV ({conf:.0%})"
                    color = (0, 0, 255)
                else:
                    status = "BASLANGIC_ATESI"
                    label = f"YANGIN: ATES / ALEV ({conf:.0%})"
                    color = (0, 90, 255)

                detections_report.append(
                    {
                        "category": "fire_hazard",
                        "class_name": cls_name,
                        "status": status,
                        "description": label,
                        "confidence": round(conf, 3),
                        "frame_area_ratio": round(area_ratio, 4),
                        "fire_hsv_score": round(hsv_ratio, 3),
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

            # Duman adaylarını doku + optik akış süzgecinden geçir
            for bbox, conf, cls_name in smoke_candidates:
                if not has_active_fire and conf < 0.48:
                    continue
                if not self._verify_smoke_plume_texture(frame, bbox, has_active_fire):
                    continue
                if is_video_stream and curr_gray_small is not None:
                    if not self._verify_smoke_optical_flow(
                        curr_gray_small, bbox, img_w, img_h, has_active_fire
                    ):
                        continue

                box_area = max(1.0, (bbox[2] - bbox[0]) * (bbox[3] - bbox[1]))
                area_ratio = box_area / total_img_area
                cx = int((bbox[0] + bbox[2]) / 2)
                cy = int((bbox[1] + bbox[3]) / 2)

                status = "DUMAN_TEHLIKESI"
                label = f"YANGIN: DUMAN TESPITI ({conf:.0%})"
                color = (130, 130, 130)

                detections_report.append(
                    {
                        "category": "fire_hazard",
                        "class_name": cls_name,
                        "status": status,
                        "description": label,
                        "confidence": round(conf, 3),
                        "frame_area_ratio": round(area_ratio, 4),
                        "fire_hsv_score": None,
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

        if is_video_stream and curr_gray_small is not None:
            self._prev_gray_small = curr_gray_small

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
