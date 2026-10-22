"""
SAR VİDEO İŞLEME VE CANLI AKIŞ MOTORU (Video & Live Stream Pipeline)
===================================================================
Bu script, DJI Tello dronundan gelen canlı kamera akışını (RTSP/UDP),
web kamerasını veya kaydedilmiş MP4/MOV video dosyalarını işler:

Temel Özellikler:
  1. Zaman Paylaşımlı Görev Dağıtıcı (Frame Multiplexing Scheduler):
     - Her karede 4 modeli birden çalıştırmak yerine görevleri dağıtır:
       Kare 4k+0 -> Modül 1 (İnsan/Hayvan + Duruş/Pose)
       Kare 4k+1 -> Modül 2 (Yangın & Duman)
       Kare 4k+2 -> Modül 3 (Tehlike Levhaları)
       Kare 4k+3 -> Modül 4 (Yapısal Çatlak & Lokal/Ağır Hasar)
     - Tespit önbelleği (Detection Cache) sayesinde her karede tüm aktif
       tehlikeler ve SLAM Pinleri canlı gösterilir.
  2. Gerçek Zamanlı Risk & Triyaj HUD Paneli:
     - Üst ve alt panelde anlık INSARAG/AFAD Triyaj Kodu, Risk Skoru,
       Güvenlik Skoru, Giriş İzni ve Gerekli KKD (PPE) listelenir.
  3. Canlı FPS & Gecikme Telemetrisi:
     - Anlık işleme hızı (FPS) ve modül gecikmeleri (ms) ekranda akar.
"""

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

# Proje kök dizinini ve src/ klasörünü import yoluna ekle
current_dir = Path(__file__).resolve().parent
if str(current_dir) not in sys.path:
    sys.path.insert(0, str(current_dir))

from indoor_risk_assessment import IndoorSARRiskAssessmentEngine
from module1_human_animal import HumanAnimalSARDetector
from module2_fire_smoke import FireSmokeSARDetector
from module3_hazard_signs import HazardSignSARDetector
from module4_structural_crack import StructuralCrackSARDetector


class SARVideoPipeline:
    def __init__(self, mode: str = "multiplexed") -> None:
        """
        mode:
          - 'multiplexed': 30 FPS akıcı canlı yayın için modülleri sırayla çalıştırır.
          - 'synchronous': Her karede 4 modülü birden çalıştırır (Kayıtlı videolar için maksimum doğruluk).
        """
        self.mode = mode
        print("=" * 72)
        print(f"[SAR VİDEO MOTORU] Başlatılıyor... Mod: {self.mode.upper()}")
        print("=" * 72)

        self.mod1_human = HumanAnimalSARDetector()
        self.mod2_fire = FireSmokeSARDetector()
        self.mod3_hazard = HazardSignSARDetector()
        self.mod4_crack = StructuralCrackSARDetector()
        self.risk_engine = IndoorSARRiskAssessmentEngine()

        # Tespit Önbelleği (Kareler arası tespit sürekliliği)
        self.cache: Dict[str, List[Dict[str, Any]]] = {
            "human_animal": [],
            "fire_smoke": [],
            "hazard_signs": [],
            "structural": [],
        }

        # Modül 4 maskesi önbelleği
        self._last_crack_mask: Optional[np.ndarray] = None

        # GPU Isınma (Warmup)
        dummy = np.zeros((320, 320, 3), dtype=np.uint8)
        self.mod1_human.pose_model.predict(dummy, verbose=False)
        self.mod1_human.det_model.predict(dummy, verbose=False)
        self.mod2_fire.model.predict(dummy, verbose=False)
        self.mod3_hazard.model.predict(dummy, verbose=False)
        self.mod4_crack.crack_model.predict(dummy, verbose=False)
        if self.mod4_crack.heavy_model is not None:
            self.mod4_crack.heavy_model.predict(dummy, verbose=False)

        print("[SAR VİDEO MOTORU] Tüm modeller ve GPU hazır!\n")

    def process_frame(
        self,
        frame: np.ndarray,
        frame_idx: int,
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """
        Tek bir video karesini işler, HUD çizimlerini yapar ve anlık risk raporunu döner.
        """
        h_img, w_img = frame.shape[:2]
        t0 = time.perf_counter()

        mod_idx = frame_idx % 4

        if self.mode == "synchronous":
            # Her karede 4 modülü de çalıştır
            rep1 = self.mod1_human.analyze_image("", frame_bgr=frame, save_output=False)
            rep2 = self.mod2_fire.analyze_image("", frame_bgr=frame, save_output=False)
            rep3 = self.mod3_hazard.analyze_image("", frame_bgr=frame, save_output=False)
            rep4 = self.mod4_crack.analyze_image("", frame_bgr=frame, save_output=False)

            self.cache["human_animal"] = rep1.get("detections", [])
            self.cache["fire_smoke"] = rep2.get("detections", [])
            self.cache["hazard_signs"] = rep3.get("detections", [])
            self.cache["structural"] = rep4.get("detections", [])
            self._last_crack_mask = rep4.get("_mask_only_array")

        else:
            # FRAME MULTIPLEXING: Sırayla tek bir modülü çalıştır, diğerlerini önbellekten oku
            if mod_idx == 0:
                rep1 = self.mod1_human.analyze_image("", frame_bgr=frame, save_output=False)
                self.cache["human_animal"] = rep1.get("detections", [])
            elif mod_idx == 1:
                rep2 = self.mod2_fire.analyze_image("", frame_bgr=frame, save_output=False)
                self.cache["fire_smoke"] = rep2.get("detections", [])
            elif mod_idx == 2:
                rep3 = self.mod3_hazard.analyze_image("", frame_bgr=frame, save_output=False)
                self.cache["hazard_signs"] = rep3.get("detections", [])
            elif mod_idx == 3:
                rep4 = self.mod4_crack.analyze_image("", frame_bgr=frame, save_output=False)
                self.cache["structural"] = rep4.get("detections", [])
                self._last_crack_mask = rep4.get("_mask_only_array")

        # 5. Risk Assessment Motoru (Her karede 0.01 ms'de çalışır)
        risk_assessment = self.risk_engine.evaluate_room_risk(
            human_animal_dets=self.cache["human_animal"],
            fire_smoke_dets=self.cache["fire_smoke"],
            hazard_sign_dets=self.cache["hazard_signs"],
            structural_dets=self.cache["structural"],
        )

        canvas = frame.copy()

        # Çatlak pikselleri önbellekten bindir
        if self._last_crack_mask is not None and self._last_crack_mask.shape[:2] == (h_img, w_img):
            for det in self.cache["structural"]:
                bx = [int(v) for v in det["bbox_xyxy"]]
                x1, y1, x2, y2 = max(0, bx[0]), max(0, bx[1]), min(w_img, bx[2]), min(h_img, bx[3])
                canvas[y1:y2, x1:x2] = self._last_crack_mask[y1:y2, x1:x2]

        # Tespit kutularını ve SLAM pinlerini çiz
        # Modül 4 (Hasar & Çatlak)
        for det in self.cache["structural"]:
            sev = det.get("severity_score_100", 50.0)
            col = (0, 0, 255) if sev >= 52.0 else (0, 140, 255) if sev >= 28.0 else (0, 215, 255)
            pin = det.get("slam_pin_2d", {"cx": int((det["bbox_xyxy"][0] + det["bbox_xyxy"][2]) / 2), "cy": int((det["bbox_xyxy"][1] + det["bbox_xyxy"][3]) / 2)})
            self._draw_hud_box(canvas, det["bbox_xyxy"], det.get("description", "HASAR"), (pin["cx"], pin["cy"]), col)

        # Modül 1 (İnsan / Hayvan)
        for det in self.cache["human_animal"]:
            st = det.get("status", "NORMAL")
            col = (
                (0, 0, 255)
                if st == "YERDE_YATIYOR_KRITIK"
                else (0, 140, 255)
                if st == "ENKAZ_ALTI_KISMI_GORUNUM"
                else (255, 180, 0)
                if det.get("category") == "animal"
                else (0, 200, 0)
            )
            pin = det.get("slam_pin_2d", {"cx": int((det["bbox_xyxy"][0] + det["bbox_xyxy"][2]) / 2), "cy": int((det["bbox_xyxy"][1] + det["bbox_xyxy"][3]) / 2)})
            conf = det.get("confidence", 0.9)
            self._draw_hud_box(canvas, det["bbox_xyxy"], f"{det.get('description', 'INSAN')} ({conf:.0%})", (pin["cx"], pin["cy"]), col)

        # Modül 2 (Yangın / Duman)
        for det in self.cache["fire_smoke"]:
            col = (0, 20, 255) if "fire" in det.get("class_name", "") else (130, 130, 130)
            pin = det.get("slam_pin_2d", {"cx": int((det["bbox_xyxy"][0] + det["bbox_xyxy"][2]) / 2), "cy": int((det["bbox_xyxy"][1] + det["bbox_xyxy"][3]) / 2)})
            self._draw_hud_box(canvas, det["bbox_xyxy"], det.get("description", "YANGIN"), (pin["cx"], pin["cy"]), col)

        # Modül 3 (Tehlike Levhaları)
        for det in self.cache["hazard_signs"]:
            pin = det.get("slam_pin_2d", {"cx": int((det["bbox_xyxy"][0] + det["bbox_xyxy"][2]) / 2), "cy": int((det["bbox_xyxy"][1] + det["bbox_xyxy"][3]) / 2)})
            self._draw_hud_box(canvas, det["bbox_xyxy"], det.get("description", "LEVHA"), (pin["cx"], pin["cy"]), (180, 0, 255))

        # Risk & Triyaj HUD Panelleri
        self.risk_engine.draw_risk_assessment_hud(canvas, risk_assessment)

        # Telemetri Bilgisi (FPS & İşleme Modu)
        elapsed_sec = max(1e-5, time.perf_counter() - t0)
        current_fps = 1.0 / elapsed_sec
        cv2.putText(
            canvas,
            f"MOD: {self.mode.upper()} (M{mod_idx+1}) | {current_fps:.1f} FPS",
            (w_img - 280, 42),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        all_dets = (
            self.cache["human_animal"]
            + self.cache["fire_smoke"]
            + self.cache["hazard_signs"]
            + self.cache["structural"]
        )

        frame_report = {
            "frame_idx": frame_idx,
            "fps": round(current_fps, 1),
            "active_module": f"mod{mod_idx+1}",
            "risk_assessment": {k: v for k, v in risk_assessment.items() if not k.startswith("_")},
            "active_detections_count": len(all_dets),
        }

        return canvas, frame_report

    def run_video(
        self,
        source: str | int,
        output_path: Optional[str] = "outputs/SAR_VIDEO_OUTPUT.mp4",
        max_frames: Optional[int] = None,
        display: bool = False,
    ) -> Dict[str, Any]:
        """
        Bir video dosyasını veya web kamerasını baştan sona işler ve sonuç videosunu kaydeder.
        """
        cap = cv2.VideoCapture(source)
        if not cap.isOpened():
            raise RuntimeError(f"Video kaynağı açılamadı: {source}")

        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        input_fps = cap.get(cv2.CAP_PROP_FPS)
        if input_fps <= 0 or math.isnan(input_fps):
            input_fps = 30.0
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

        print(f"[VİDEO BİLGİSİ] Kaynak: {source} | Boyut: {w}x{h} | Giriş FPS: {input_fps:.1f} | Toplam Kare: {total_frames}")

        writer = None
        if output_path:
            out_p = Path(output_path)
            out_p.parent.mkdir(parents=True, exist_ok=True)
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer = cv2.VideoWriter(str(out_p), fourcc, min(30.0, input_fps), (w, h))

        frame_count = 0
        t_start = time.time()
        fps_history: List[float] = []

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break

                annotated_frame, report = self.process_frame(frame, frame_count)
                fps_history.append(report["fps"])

                if writer is not None:
                    writer.write(annotated_frame)

                if display:
                    cv2.imshow("DJI Tello SAR Master Perception Stream", annotated_frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        print("[BİLGİ] Kullanıcı 'q' tuşuna basarak durdurdu.")
                        break

                frame_count += 1
                if frame_count % 30 == 0:
                    avg_fps = np.mean(fps_history[-30:])
                    risk = report["risk_assessment"]["overall_risk_score_100"]
                    code = report["risk_assessment"]["triage_code"]
                    print(f"  -> Kare {frame_count:04d} | Hız: {avg_fps:.1f} FPS | Risk: {risk:.0f}/100 | Triyaj: {code}")

                if max_frames and frame_count >= max_frames:
                    print(f"[BİLGİ] Maksimum kare sınırına ulaşıldı: {max_frames}")
                    break

        finally:
            cap.release()
            if writer is not None:
                writer.release()
            if display:
                cv2.destroyAllWindows()

        total_elapsed = time.time() - t_start
        overall_avg_fps = frame_count / max(1e-5, total_elapsed)

        print("\n" + "=" * 72)
        print("VİDEO İŞLEME TAMAMLANDI!")
        print(f"  Toplam İşlenen Kare : {frame_count}")
        print(f"  Toplam Süre         : {total_elapsed:.2f} saniye")
        print(f"  Ortalama Akış Hızı  : {overall_avg_fps:.1f} FPS")
        if output_path and Path(output_path).exists():
            print(f"  Kayıt Edilen Video  : {Path(output_path).resolve()} ({os.path.getsize(output_path)/1e6:.2f} MB)")
        print("=" * 72)

        return {
            "source": str(source),
            "output_video": str(output_path) if output_path else None,
            "processed_frames": frame_count,
            "elapsed_seconds": round(total_elapsed, 2),
            "average_fps": round(overall_avg_fps, 1),
        }

    @staticmethod
    def _draw_hud_box(
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
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.46
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
    parser = argparse.ArgumentParser(description="SAR Canlı Video & Video Dosyası İşleme")
    parser.add_argument("--source", type=str, default="0", help="Video dosyası yolu veya kamera ID (ör: 0 veya test.mp4)")
    parser.add_argument("--output", type=str, default="outputs/SAR_VIDEO_OUTPUT.mp4", help="Çıktı video dosyası yolu")
    parser.add_argument("--mode", type=str, default="multiplexed", choices=["multiplexed", "synchronous"], help="İşleme modu")
    parser.add_argument("--max_frames", type=int, default=None, help="Maksimum işlenecek kare sayısı")
    parser.add_argument("--display", action="store_true", help="Canlı pencerede göster")
    args = parser.parse_args()

    src = int(args.source) if args.source.isdigit() else args.source
    pipeline = SARVideoPipeline(mode=args.mode)
    pipeline.run_video(source=src, output_path=args.output, max_frames=args.max_frames, display=args.display)
