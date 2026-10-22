"""
MODÜL 4 (v2 - Kapsamlı): Yapısal Hasar, Çatlak Segmentasyonu, Kolon Demiri & Beton Kırığı Analizi
=================================================================================================
Bu modül 2 ayrı derin öğrenme modelini + Matematiksel Hasar Ölçüm Motorunu birleştirir:
  1. Model 4A (`models/crack_seg_yolo.pt` - Instance Segmentation):
     Duvar ve kolonlardaki ince/kalın çizgisel çatlakları piksel piksel boyar,
     uzunluğunu (`minAreaRect`), kalınlığını (`distanceTransform`) ve deprem kesme
     açısını (`25°-65° Shear Crack`) ölçer.
  2. Model 4B (`models/heavy_structural_yolo11n.pt` - Kendi Eğittiğimiz Ağır Hasar Modeli):
     Çizgisel olmayan ağır yıkım belirtilerini tespit eder:
     - `beton_dokulmesi_spalling`  (Beton Dökülmesi / Kolon Ezilmesi / Derin Kırık Oyuk)
     - `aciga_cikmis_kolon_demiri` (Açığa Çıkmış Kolon/İnşaat Demiri - ÇÖKME TEHLİKESİ!)
     - `duvar_gocugu_tugla`        (Sıva Patlaması / Açığa Çıkmış Tuğla Duvar Göçüğü)
"""

import json
import math
from pathlib import Path
from typing import Any, Dict, List, Tuple

import cv2
import numpy as np
import torch
from ultralytics import YOLO

HEAVY_DAMAGE_INFO = {
    "aciga_cikmis_kolon_demiri": {
        "status": "KRITIK_ACIGA_CIKMIS_DEMIR_COKME_RISKI",
        "label": "AGIR HASAR: ACIGA CIKMIS KOLON DEMIRI (COKME TEHLIKESI!)",
        "base_severity": 88.0,
        "color": (0, 0, 255),      # Kırmızı
    },
    "beton_dokulmesi_spalling": {
        "status": "KRITIK_BETON_KIRIGI_SPALLING",
        "label": "AGIR HASAR: BETON DOKULMESI / KOLON KIRIGI",
        "base_severity": 72.0,
        "color": (0, 40, 255),     # Koyu Kırmızı-Turuncu
    },
    "duvar_gocugu_tugla": {
        "status": "ORTA_DUVAR_SIVA_PATLAMASI_TUGLA",
        "label": "YAPISAL HASAR: SIVA COKMESI / ACIGA CIKMIS TUGLA",
        "base_severity": 54.0,
        "color": (0, 140, 255),    # Turuncu
    },
}


def get_best_device() -> str:
    if torch.backends.mps.is_available():
        return "mps"
    if torch.cuda.is_available():
        return "cuda"
    return "cpu"


class StructuralCrackSARDetector:
    def __init__(
        self,
        crack_seg_path: str = "models/crack_seg_yolo.pt",
        heavy_damage_path: str = "models/heavy_structural_yolo11n.pt",
    ) -> None:
        self.crack_seg_path = Path(crack_seg_path)
        self.heavy_damage_path = Path(heavy_damage_path)
        if not self.crack_seg_path.exists():
            raise FileNotFoundError(f"Çatlak modeli bulunamadı: {self.crack_seg_path}")

        self.device = get_best_device()
        print(f"[BİLGİ] Modül 4 v2 (Çatlak Segmentasyonu + Ağır Yapısal Hasar) donanımı: {self.device.upper()}")
        self.crack_model = YOLO(str(self.crack_seg_path))
        self.heavy_model = (
            YOLO(str(self.heavy_damage_path))
            if self.heavy_damage_path.exists()
            else None
        )

    @staticmethod
    def _verify_heavy_damage_material(
        frame_bgr: np.ndarray, bbox: List[float], cname: str
    ) -> bool:
        """
        Ağır hasar ve lokalize hasar tespitlerinde fiziksel materyal doğrulaması yapar:
        - `duvar_gocugu_tugla`: Gerçekten kiremit/tuğla rengi (kırmızı-turuncu-kahve) içeriyor mu?
        - `aciga_cikmis_kolon_demiri`: Paslı/metalik demir dokusu ve pürüzlü kenar içeriyor mu?
        - `beton_dokulmesi_spalling`: Pürüzlü kırık beton/oyuk kenar dokusu içeriyor mu?
        
        Lokalize / küçük kutularda (area_ratio < 0.02) eşikleri esnetir; böylece tek tük
        açıkta kalmış demir uçları veya küçük sıva dökülmeleri yanlışlıkla elenmez.
        """
        h_img, w_img = frame_bgr.shape[:2]
        x1, y1 = max(0, int(bbox[0])), max(0, int(bbox[1]))
        x2, y2 = min(w_img, int(bbox[2])), min(h_img, int(bbox[3]))
        crop = frame_bgr[y1:y2, x1:x2]
        if crop.size == 0:
            return False

        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        total_px = float(crop.shape[0] * crop.shape[1])
        area_ratio = total_px / max(1.0, float(w_img * h_img))
        is_small = area_ratio < 0.020  # Lokalize küçük hasar kontrolü

        if cname == "duvar_gocugu_tugla":
            # Kiremit / Kızıl-Turuncu Tuğla HSV aralığı
            brick_mask = cv2.inRange(
                hsv,
                np.array([0, 35, 30], dtype=np.uint8),
                np.array([22, 255, 240], dtype=np.uint8),
            )
            brick_ratio = float(np.count_nonzero(brick_mask)) / total_px
            edges = cv2.Canny(cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY), 70, 180)
            edge_ratio = float(np.count_nonzero(edges)) / total_px
            min_brick = 0.08 if is_small else 0.12
            min_edge = 0.09 if is_small else 0.14
            return brick_ratio >= min_brick and edge_ratio >= min_edge

        if cname == "aciga_cikmis_kolon_demiri":
            # Paslı metal dokusu ve yüksek pürüzlülük kontrolü
            rust_mask = cv2.inRange(
                hsv,
                np.array([0, 25, 20], dtype=np.uint8),
                np.array([28, 255, 210], dtype=np.uint8),
            )
            edges = cv2.Canny(cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY), 70, 180)
            rust_ratio = float(np.count_nonzero(rust_mask)) / total_px
            edge_ratio = float(np.count_nonzero(edges)) / total_px
            min_rust = 0.025 if is_small else 0.045
            min_edge = 0.08 if is_small else 0.12
            return rust_ratio >= min_rust and edge_ratio >= min_edge

        if cname == "beton_dokulmesi_spalling":
            # Kırık beton/sıva oyuğu doğrulaması: Pürüzlü kenar dokusu
            edges = cv2.Canny(cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY), 60, 160)
            edge_ratio = float(np.count_nonzero(edges)) / total_px
            min_edge = 0.06 if is_small else 0.09
            return edge_ratio >= min_edge

        return True

    @staticmethod
    def _analyze_crack_geometry(
        binary_mask: np.ndarray, img_diag: float, bbox: List[float] | None = None
    ) -> Dict[str, Any]:
        h_full, w_full = binary_mask.shape[:2]
        if bbox is not None:
            x0 = max(0, int(bbox[0]) - 4)
            y0 = max(0, int(bbox[1]) - 4)
            x1 = min(w_full, int(bbox[2]) + 4)
            y1 = min(h_full, int(bbox[3]) + 4)
            roi_mask = binary_mask[y0:y1, x0:x1]
        else:
            x0, y0 = 0, 0
            roi_mask = binary_mask

        ys_r, xs_r = np.where(roi_mask > 0)
        if len(xs_r) < 10:
            return {
                "length_px": 0.0,
                "length_ratio": 0.0,
                "max_width_px": 0.0,
                "angle_deg": 0.0,
                "is_diagonal_shear": False,
                "severity_score": 0.0,
                "status": "DUSUK_RISK_YUZEYSEL",
                "label": "YUZEYSEL KILCAL CATLAK (DUSUK RISK)",
                "color": (0, 215, 255),
                "centroid": (0, 0),
            }

        xs = xs_r + x0
        ys = ys_r + y0
        cx = int(np.mean(xs))
        cy = int(np.mean(ys))

        pts = np.column_stack((xs_r, ys_r)).astype(np.float32)
        rect = cv2.minAreaRect(pts)
        (_, _), (rw, rh), _ = rect
        length_px = float(max(rw, rh))
        length_ratio = length_px / max(1.0, img_diag)

        dx = float(np.max(xs_r) - np.min(xs_r))
        dy = float(np.max(ys_r) - np.min(ys_r))
        angle_deg = math.degrees(math.atan2(dy, max(1.0, dx)))
        is_diagonal_shear = 25.0 <= angle_deg <= 65.0

        dist_map = cv2.distanceTransform(roi_mask, cv2.DIST_L2, 5)
        max_width_px = float(np.max(dist_map) * 2.0)

        score = (length_ratio * 95.0) + (min(max_width_px, 40.0) * 1.1)
        if is_diagonal_shear and length_ratio > 0.20:
            score += 18.0
        severity_score = min(100.0, round(score, 1))

        if severity_score >= 52.0 or (length_ratio > 0.38 and max_width_px > 12.0):
            status = "KRITIK_YAPISAL_KIRIK"
            shear_tag = " [KESME KIRIGI]" if is_diagonal_shear else ""
            label = f"HASAR: KRITIK KOLON/DUVAR KIRIGI{shear_tag}"
            color = (0, 0, 255)
        elif severity_score >= 28.0:
            status = "ORTA_YAPISAL_CATLAK"
            label = "HASAR: DERIN DUVAR CATLAGI (ORTA RISK)"
            color = (0, 140, 255)
        else:
            status = "DUSUK_RISK_YUZEYSEL"
            label = "HASAR: YUZEYSEL SIVA CATLAGI (DUSUK RISK)"
            color = (0, 215, 255)

        return {
            "length_px": round(length_px, 1),
            "length_ratio": round(length_ratio, 3),
            "max_width_px": round(max_width_px, 1),
            "angle_deg": round(angle_deg, 1),
            "is_diagonal_shear": is_diagonal_shear,
            "severity_score": severity_score,
            "status": status,
            "label": label,
            "color": color,
            "centroid": (cx, cy),
        }

    def analyze_image(
        self,
        image_path: str,
        output_dir: str = "outputs",
        conf_threshold: float = 0.25,
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
        img_diag = math.hypot(img_w, img_h)
        annotated = frame.copy()
        mask_overlay = frame.copy()
        detections_report: List[Dict[str, Any]] = []

        max_room_severity = 0.0

        # 1. KATMAN: Piksel Bazlı Çizgisel Çatlak Segmentasyonu (`crack_seg_yolo.pt`)
        # İç mekan (ev/ofis) sarı/krem/renkli boyalı alçı duvarlarda çatlak kontrastını artırmak için
        # hem orijinal görüntüde hem de nötr gri-CLAHE (beton tonu) normalize görüntüde tarama yapar.
        clahe_gray = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(
            cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        )
        neutral_indoor_frame = cv2.cvtColor(clahe_gray, cv2.COLOR_GRAY2BGR)

        results_list = [
            self.crack_model.predict(
                source=frame,
                conf=conf_threshold,
                iou=0.45,
                retina_masks=True,
                device=self.device,
                verbose=False,
            )[0],
            self.crack_model.predict(
                source=neutral_indoor_frame,
                conf=0.22,
                iou=0.45,
                retina_masks=True,
                device=self.device,
                verbose=False,
            )[0],
        ]

        seen_crack_boxes: List[List[float]] = []
        for results in results_list:
            if results.boxes is None or len(results.boxes) == 0:
                continue
            boxes_xyxy = results.boxes.xyxy.cpu().numpy()
            confs = results.boxes.conf.cpu().numpy()
            has_masks = results.masks is not None and results.masks.data is not None
            masks_np = results.masks.data.cpu().numpy() if has_masks else None

            for i in range(len(boxes_xyxy)):
                bbox = [float(v) for v in boxes_xyxy[i]]
                if (
                    bbox[3] < img_h * 0.03
                    or bbox[1] > img_h * 0.97
                    or bbox[2] < img_w * 0.03
                    or bbox[0] > img_w * 0.97
                ):
                    continue
                bw = bbox[2] - bbox[0]
                bh = bbox[3] - bbox[1]
                # İç mekandaki dümdüz dikey kapı kasası / duvar köşesi birleşim çizgilerini (dar dikey şerit) ele
                if bh > bw * 6.5:
                    continue
                # İç mekandaki koyu kahverengi ahşap kapı panellerini / mobilya gölgelerini çatlak sanmayı engelle
                cx1, cy1 = max(0, int(bbox[0])), max(0, int(bbox[1]))
                cx2, cy2 = min(img_w, int(bbox[2])), min(img_h, int(bbox[3]))
                if cx2 > cx1 and cy2 > cy1:
                    mean_v = float(np.mean(cv2.cvtColor(frame[cy1:cy2, cx1:cx2], cv2.COLOR_BGR2HSV)[:, :, 2]))
                    if mean_v < 82.0:
                        continue
                # Mükerrer kutu kontrolü
                is_dup = False
                for sb in seen_crack_boxes:
                    ix1, iy1 = max(bbox[0], sb[0]), max(bbox[1], sb[1])
                    ix2, iy2 = min(bbox[2], sb[2]), min(bbox[3], sb[3])
                    inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
                    if inter / max(1.0, min(bw * bh, (sb[2] - sb[0]) * (sb[3] - sb[1]))) > 0.35:
                        is_dup = True
                        break
                if is_dup:
                    continue
                seen_crack_boxes.append(bbox)
                conf = float(confs[i])

                if masks_np is not None and i < len(masks_np):
                    raw_mask = masks_np[i]
                    if raw_mask.shape[:2] != (img_h, img_w):
                        raw_mask = cv2.resize(
                            raw_mask, (img_w, img_h), interpolation=cv2.INTER_NEAREST
                        )
                    bin_mask = (raw_mask > 0.5).astype(np.uint8)
                else:
                    bin_mask = np.zeros((img_h, img_w), dtype=np.uint8)
                    x1, y1, x2, y2 = [int(v) for v in bbox]
                    bin_mask[y1:y2, x1:x2] = 1

                geom = self._analyze_crack_geometry(bin_mask, img_diag, bbox)
                # İç mekan filtre 1: Gerçek çizgisel çatlak ince ve uzundur; kalın cisimleri (avize kolu, vantilatör kanadı, yastık köşesi vb.) ele
                if geom["max_width_px"] / max(1.0, geom["length_px"]) > 0.22 or geom["max_width_px"] > 30.5:
                    continue
                # İç mekan filtre 2: Alt süpürgelik / dolap tabanı gölgesi (zemine bitişik yatay köşe gölgesi)
                if bbox[3] > img_h * 0.88 and geom["angle_deg"] < 26.0 and geom["length_ratio"] < 0.15:
                    continue
                max_room_severity = max(max_room_severity, geom["severity_score"])

                color = geom["color"]
                cx, cy = geom["centroid"]
                if cx == 0 and cy == 0:
                    cx = int((bbox[0] + bbox[2]) / 2)
                    cy = int((bbox[1] + bbox[3]) / 2)

                mask_bool = bin_mask > 0
                mask_overlay[mask_bool] = color
                contours, _ = cv2.findContours(
                    bin_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )
                cv2.drawContours(annotated, contours, -1, color, 2, cv2.LINE_AA)

                full_label = f"{geom['label']} | Skor:{geom['severity_score']:.0f}/100 ({conf:.0%})"
                detections_report.append(
                    {
                        "category": "structural_damage",
                        "class_name": "crack",
                        "status": geom["status"],
                        "description": full_label,
                        "confidence": round(conf, 3),
                        "severity_score_100": geom["severity_score"],
                        "crack_metrics": {
                            "length_px": geom["length_px"],
                            "length_to_frame_ratio": geom["length_ratio"],
                            "max_width_px": geom["max_width_px"],
                            "angle_deg": geom["angle_deg"],
                            "is_diagonal_shear_crack": geom["is_diagonal_shear"],
                        },
                        "bbox_xyxy": [round(v, 1) for v in bbox],
                        "slam_pin_2d": {
                            "cx": cx,
                            "cy": cy,
                            "norm_u": round(cx / img_w, 4),
                            "norm_v": round(cy / img_h, 4),
                        },
                    }
                )

        # 2. KATMAN: Ağır Yapısal Hasar Modeli (Beton Dökülmesi, Kolon Demiri, Duvar Göçüğü)
        if self.heavy_model is not None:
            heavy_res = self.heavy_model.predict(
                source=frame,
                conf=0.28,
                iou=0.40,
                agnostic_nms=True,
                device=self.device,
                verbose=False,
            )[0]

            if heavy_res.boxes is not None and len(heavy_res.boxes) > 0:
                h_boxes = heavy_res.boxes.xyxy.cpu().numpy()
                h_confs = heavy_res.boxes.conf.cpu().numpy()
                h_clss = heavy_res.boxes.cls.cpu().numpy().astype(int)

                for i in range(len(h_boxes)):
                    bbox = [float(v) for v in h_boxes[i]]
                    conf = float(h_confs[i])
                    cid = int(h_clss[i])
                    cname = self.heavy_model.names.get(cid, "structural_damage")
                    if not self._verify_heavy_damage_material(frame, bbox, cname):
                        continue
                    area_ratio = ((bbox[2] - bbox[0]) * (bbox[3] - bbox[1])) / max(
                        1.0, float(img_w * img_h)
                    )
                    is_localized = area_ratio < 0.025

                    if is_localized:
                        if cname == "aciga_cikmis_kolon_demiri":
                            det_label = "YAPISAL HASAR: LOKAL ACIGA CIKMIS DEMIR UCU"
                            det_status = "ORTA_LOKAL_ACIK_DEMIR_UCU"
                            sev = min(80.0, round(65.0 + area_ratio * 400.0, 1))
                        elif cname == "beton_dokulmesi_spalling":
                            det_label = "YAPISAL HASAR: LOKAL BETON DOKULMESI / OYUK"
                            det_status = "ORTA_LOKAL_BETON_DOKULMESI"
                            sev = min(60.0, round(42.0 + area_ratio * 350.0, 1))
                        elif cname == "duvar_gocugu_tugla":
                            det_label = "YAPISAL HASAR: LOKAL SIVA DOKULMESI / TUGLA"
                            det_status = "DUSUK_LOKAL_SIVA_PATLAMASI"
                            sev = min(50.0, round(30.0 + area_ratio * 300.0, 1))
                        else:
                            det_label = f"LOKAL HASAR: {cname}"
                            det_status = "ORTA_LOKAL_HASAR"
                            sev = 40.0
                    else:
                        info = HEAVY_DAMAGE_INFO.get(
                            cname,
                            {
                                "status": "KRITIK_YAPISAL_HASAR",
                                "label": f"AGIR HASAR: {cname}",
                                "base_severity": 70.0,
                                "color": (0, 0, 255),
                            },
                        )
                        det_label = info["label"]
                        det_status = info["status"]
                        sev = min(100.0, round(info["base_severity"] + area_ratio * 25.0, 1))

                    max_room_severity = max(max_room_severity, sev)

                    cx = int((bbox[0] + bbox[2]) / 2)
                    cy = int((bbox[1] + bbox[3]) / 2)
                    full_label = f"{det_label} | Skor:{sev:.0f}/100 ({conf:.0%})"

                    detections_report.append(
                        {
                            "category": "structural_damage",
                            "class_name": cname,
                            "status": det_status,
                            "description": full_label,
                            "confidence": round(conf, 3),
                            "severity_score_100": sev,
                            "crack_metrics": None,
                            "bbox_xyxy": [round(v, 1) for v in bbox],
                            "slam_pin_2d": {
                                "cx": cx,
                                "cy": cy,
                                "norm_u": round(cx / img_w, 4),
                                "norm_v": round(cy / img_h, 4),
                            },
                        }
                    )

        annotated = cv2.addWeighted(mask_overlay, 0.45, annotated, 0.55, 0)
        mask_only_canvas = annotated.copy()

        for det in detections_report:
            bbox = det["bbox_xyxy"]
            pin = det["slam_pin_2d"]
            sev = det["severity_score_100"]
            col = (
                (0, 0, 255)
                if sev >= 52.0
                else (0, 140, 255)
                if sev >= 28.0
                else (0, 215, 255)
            )
            self._draw_box(
                annotated, bbox, det["description"], (pin["cx"], pin["cy"]), col
            )

        if max_room_severity >= 52.0:
            room_integrity = "KRITIK_TEHLIKE_ODA_COKEBILIR"
            banner_txt = f"ODA SAGLAMLIGI: KRITIK TEHLIKE! (Hasar Skoru: {max_room_severity:.0f}/100)"
            banner_col = (0, 0, 200)
        elif max_room_severity >= 28.0:
            room_integrity = "ORTA_RISK_DIKKATLI_ILERLE"
            banner_txt = f"ODA SAGLAMLIGI: ORTA YAPISAL RISK (Hasar Skoru: {max_room_severity:.0f}/100)"
            banner_col = (0, 110, 230)
        elif len(detections_report) > 0:
            room_integrity = "DUSUK_RISK_YUZEYSEL"
            banner_txt = f"ODA SAGLAMLIGI: DUSUK RISK / YUZEYSEL CATLAK ({max_room_severity:.0f}/100)"
            banner_col = (0, 160, 180)
        else:
            room_integrity = "SAGLAM_HASAR_YOK"
            banner_txt = "ODA SAGLAMLIGI: GUVENLI (Yapisal Catlak Yok)"
            banner_col = (0, 150, 0)

        cv2.rectangle(annotated, (0, 0), (img_w, 42), banner_col, -1)
        cv2.putText(
            annotated,
            banner_txt,
            (14, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.68,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        out_dir = Path(output_dir)
        out_img_path = out_dir / f"sar_crack_{img_path.name}"
        mask_only_path = out_dir / f"sar_crack_maskonly_{img_path.name}"
        out_json_path = out_dir / f"sar_crack_report_{img_path.stem}.json"

        full_report = {
            "module": "module4_structural_crack_v2",
            "source_image": str(img_path),
            "annotated_image": str(out_img_path),
            "mask_only_image": str(mask_only_path),
            "room_structural_integrity": room_integrity,
            "max_severity_score_100": max_room_severity,
            "total_detections": len(detections_report),
            "detections": detections_report,
            "_mask_only_array": mask_only_canvas,
        }
        if save_output:
            out_dir.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(out_img_path), annotated)
            cv2.imwrite(str(mask_only_path), mask_only_canvas)
            clean_rep = {k: v for k, v in full_report.items() if not k.startswith("_")}
            with open(out_json_path, "w", encoding="utf-8") as f:
                json.dump(clean_rep, f, ensure_ascii=False, indent=2)

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
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

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
            2,
            cv2.LINE_AA,
        )

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.48
        thickness = 2
        (tw, th), _ = cv2.getTextSize(label, font, font_scale, thickness)
        top_y = max(th + 52, y1)
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

    detector = StructuralCrackSARDetector()
    for target in sys.argv[1:]:
        print(f"\n[YAPISAL HASAR ANALİZİ v2] -> {target}")
        rep = detector.analyze_image(target)
        print(json.dumps(rep, ensure_ascii=False, indent=2))
