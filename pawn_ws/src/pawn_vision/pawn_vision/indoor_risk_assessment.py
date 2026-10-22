"""
İÇ MEKAN ARAMA-KURTARMA (INDOOR SAR) ÇOK-MODÜLLÜ RİSK DEĞERLENDİRME VE TRİYAJ MOTORU
====================================================================================
Bitirme Projesi Ana Mühendislik Katkısı (Core Novel Contribution):
Bu modül, 4 ayrı derin öğrenme ve görüntü işleme modülünden gelen çıktıları
(İnsan/Hayvan Postürü, Yangın/Duman, Tehlike Levhaları, Duvar/Tavan/Kolon Hasarları)
tekil tespitler olmaktan çıkarıp matematiksel bir "Kapalı Ortam Risk, Sinerji ve
Kurtarma Triyaj Karar Destek Sistemi"ne dönüştürür.

Hesaplanan Parametreler:
  1. 4 Boyutlu Alt Risk Skorları (0-100):
     - R_structural : Tavan/Kolon/Duvar kümülatif çökme riski (konum ve kesme açısı ağırlıklı)
     - R_thermal    : Kapalı ortam (Indoor Enclosure) yangın, duman boğulması ve Flashover riski
     - R_hazmat     : CBRN (Radyasyon, Biyolojik, Kimyasal, Patlayıcı, Yüksek Voltaj) riski
     - R_victim     : İçerideki kazazede/canlı hayati kritiklik skoru
  2. Çapraz Modül Mekansal Sinerji Matrisi (Cross-Modal Spatial Coupling - Delta_R_synergy):
     - Kazazede ile aktif tehlike (ateş, duman, çöken kolon) arasındaki Öklid mesafesini (d_ij)
       ölçerek bileşik felaket riskini (Compound Disaster Risk) hesaplar.
  3. INSARAG / AFAD Uyumlu Kurtarma Triyaj Kodu (P1 Kırmızı .. P5 Yeşil),
     Kurtarma Ekibi Odaya Giriş İzni, Zorunlu KKD (PPE) ve Taktik Eylem Planı.
"""

import math
from typing import Any, Dict, List, Tuple

import cv2
import numpy as np


HAZMAT_BASE_SCORES: Dict[str, float] = {
    "radyasyon": 90.0,
    "biyolojik_tehlike": 85.0,
    "kimyasal_toksik": 82.0,
    "yanici_patlayici": 78.0,
    "elektrik_tehlikesi": 68.0,
    "genel_tehlike_uyari": 42.0,
}

VICTIM_BASE_SCORES: Dict[str, float] = {
    "YERDE_YATIYOR_KRITIK": 92.0,
    "ENKAZ_ALTI_KISMI_GORUNUM": 90.0,
    "OTURUYOR_COMELMIS": 65.0,
    "HAYVAN_TESPIT_EDILDI": 48.0,
    "AYAKTA": 34.0,
}


class IndoorSARRiskAssessmentEngine:
    """
    Kapalı mekan (ev, ofis, apartman, koridor, tesisat odası) arama-kurtarma
    operasyonları için çok-modüllü risk analizi ve triyaj motoru.
    """

    @staticmethod
    def _euclidean_norm_dist(det_a: Dict[str, Any], det_b: Dict[str, Any]) -> float:
        pa = det_a.get("slam_pin_2d", {})
        pb = det_b.get("slam_pin_2d", {})
        ua, va = float(pa.get("norm_u", 0.5)), float(pa.get("norm_v", 0.5))
        ub, vb = float(pb.get("norm_u", 0.5)), float(pb.get("norm_v", 0.5))
        return math.hypot(ua - ub, va - vb)

    def evaluate_room_risk(
        self,
        human_animal_dets: List[Dict[str, Any]],
        fire_smoke_dets: List[Dict[str, Any]],
        hazard_sign_dets: List[Dict[str, Any]],
        structural_dets: List[Dict[str, Any]],
    ) -> Dict[str, Any]:
        # ---------------------------------------------------------------------
        # 1. YAPISAL ÇÖKME RİSKİ (R_structural: 0 - 100)
        # ---------------------------------------------------------------------
        weighted_struct_scores: List[float] = []
        has_exposed_rebar = False
        has_spalling_or_brick = False
        has_shear_crack = False
        has_ceiling_damage = False

        for s_det in structural_dets:
            base_s = float(s_det.get("severity_score_100", 0.0))
            cname = s_det.get("class_name", "crack")
            norm_v = float(s_det.get("slam_pin_2d", {}).get("norm_v", 0.5))

            # İç mekanda tavan/üst kiriş (norm_v < 0.35) hasarları kafaya düşme riski taşır
            loc_mult = 1.15 if norm_v < 0.35 else (1.05 if norm_v < 0.72 else 1.00)
            if norm_v < 0.35 and base_s >= 25.0:
                has_ceiling_damage = True

            if cname == "aciga_cikmis_kolon_demiri":
                has_exposed_rebar = True
                loc_mult = max(loc_mult, 1.12)
            elif cname in ("beton_dokulmesi_spalling", "duvar_gocugu_tugla"):
                has_spalling_or_brick = True

            metrics = s_det.get("crack_metrics") or {}
            if metrics.get("is_diagonal_shear_crack"):
                has_shear_crack = True

            weighted_struct_scores.append(min(100.0, base_s * loc_mult))

        if weighted_struct_scores:
            weighted_struct_scores.sort(reverse=True)
            max_s = weighted_struct_scores[0]
            # Çoklu hasar kümülatif etkisi (aynı odada birden fazla çatlak/göçük varsa risk artar)
            cumulative_bonus = sum(weighted_struct_scores[1:]) * 0.22
            r_structural = min(100.0, round(max_s + cumulative_bonus, 1))
        else:
            r_structural = 0.0

        # ---------------------------------------------------------------------
        # 2. TERMAL / KAPALI ORTAM YANGIN & DUMAN RİSKİ (R_thermal: 0 - 100)
        # ---------------------------------------------------------------------
        fire_scores: List[float] = []
        has_active_flame = False
        has_indoor_smoke = False

        for f_det in fire_smoke_dets:
            st = f_det.get("status", "")
            area_r = float(f_det.get("frame_area_ratio", 0.02))
            if st == "KRITIK_YANGIN":
                has_active_flame = True
                fire_scores.append(min(100.0, 84.0 + area_r * 110.0))
            elif st == "BASLANGIC_ATESI":
                has_active_flame = True
                fire_scores.append(min(92.0, 68.0 + area_r * 140.0))
            else:
                has_indoor_smoke = True
                fire_scores.append(min(88.0, 60.0 + area_r * 95.0))

        if fire_scores:
            r_thermal = max(fire_scores)
            # Kapalı odada hem alev hem yoğun duman varsa Flashover / CO boğulma etkisi
            if has_active_flame and has_indoor_smoke:
                r_thermal = min(100.0, r_thermal + 12.0)
            elif len(fire_scores) > 1:
                r_thermal = min(100.0, r_thermal + 6.0)
            r_thermal = round(r_thermal, 1)
        else:
            r_thermal = 0.0

        # ---------------------------------------------------------------------
        # 3. CBRN & ENDÜSTRİYEL TEHLİKE LEVHASI RİSKİ (R_hazmat: 0 - 100)
        # ---------------------------------------------------------------------
        haz_scores: List[float] = []
        detected_hazmat_classes: List[str] = []
        for h_det in hazard_sign_dets:
            cname = h_det.get("class_name", "genel_tehlike_uyari")
            detected_hazmat_classes.append(cname)
            haz_scores.append(HAZMAT_BASE_SCORES.get(cname, 45.0))

        if haz_scores:
            haz_scores.sort(reverse=True)
            Unique_bonus = 8.0 * max(0, len(set(detected_hazmat_classes)) - 1)
            r_hazmat = min(100.0, round(haz_scores[0] + Unique_bonus, 1))
        else:
            r_hazmat = 0.0

        # ---------------------------------------------------------------------
        # 4. KAZAZEDE & CANLI KRİTİKLİK SKORU (R_victim: 0 - 100)
        # ---------------------------------------------------------------------
        victim_scores: List[float] = []
        critical_humans = 0
        conscious_humans = 0
        animals_count = 0

        for v_det in human_animal_dets:
            st = v_det.get("status", "AYAKTA")
            cat = v_det.get("category", "human")
            victim_scores.append(VICTIM_BASE_SCORES.get(st, 40.0))
            if cat == "animal":
                animals_count += 1
            elif st in ("YERDE_YATIYOR_KRITIK", "ENKAZ_ALTI_KISMI_GORUNUM"):
                critical_humans += 1
            else:
                conscious_humans += 1

        if victim_scores:
            victim_scores.sort(reverse=True)
            multi_victim_bonus = min(12.0, 5.0 * (len(victim_scores) - 1))
            r_victim = min(100.0, round(victim_scores[0] + multi_victim_bonus, 1))
        else:
            r_victim = 0.0

        # ---------------------------------------------------------------------
        # 5. ÇAPRAZ MODÜL MEKANSAL SİNERJİ MATRİSİ (Delta_R_synergy)
        # ---------------------------------------------------------------------
        synergy_bonus = 0.0
        synergy_alerts: List[str] = []

        # Sinerji A: İçerideki Canlı (İnsan/Hayvan) + Aktif Yangın/Duman Yakınlığı
        if human_animal_dets and fire_smoke_dets:
            min_d = min(
                self._euclidean_norm_dist(v, f)
                for v in human_animal_dets
                for f in fire_smoke_dets
            )
            prox_factor = max(0.55, 1.35 - min_d)
            add_s = round(16.0 * prox_factor, 1)
            synergy_bonus += add_s
            synergy_alerts.append(
                f"KRITIK SINERJI: Kazazede ile Oda Ici Yangin/Duman Ayni Ortamda (Mesafe:{min_d:.2f}, +{add_s:.0f} Risk)"
            )

        # Sinerji B: İçerideki Canlı + Ağır/Orta Yapısal Çökme Riski
        if human_animal_dets and r_structural >= 32.0:
            min_d = min(
                self._euclidean_norm_dist(v, s)
                for v in human_animal_dets
                for s in structural_dets
            )
            prox_factor = max(0.55, 1.30 - min_d)
            add_s = round(14.0 * prox_factor, 1)
            synergy_bonus += add_s
            synergy_alerts.append(
                f"YAPISAL SINERJI: Hasarli/Cokme Riskli Odada Canli Mahsur (Mesafe:{min_d:.2f}, +{add_s:.0f} Risk)"
            )

        # Sinerji C: Aktif Yangın/Duman + Yanıcı/Patlayıcı veya Kimyasal Levha (BLEVE / Toksik Gaz)
        if fire_smoke_dets and any(
            c in ("yanici_patlayici", "kimyasal_toksik", "radyasyon")
            for c in detected_hazmat_classes
        ):
            synergy_bonus += 20.0
            synergy_alerts.append(
                "PATLAMA SINERJISI: Aktif Ates/Duman + Yanici/Kimyasal Madde Levhasi (Ikincil Patlama Riski +20)"
            )

        # Sinerji D: Yapısal Duvar/Tavan Göçüğü + Yüksek Voltaj Levhası (Enkazda Elektrik Kaçağı)
        if r_structural >= 28.0 and "elektrik_tehlikesi" in detected_hazmat_classes:
            synergy_bonus += 12.0
            synergy_alerts.append(
                "ELEKTRIK SINERJISI: Yapisal Hasar + Yuksek Voltaj Bolgesi (Kablo Kopmasi / Carpma Riski +12)"
            )

        synergy_bonus = min(35.0, round(synergy_bonus, 1))

        # ---------------------------------------------------------------------
        # 6. BİRLEŞİK ODA RİSK İNDEKSİ (R_total: 0 - 100) VE GÜVENLİK SKORU
        # ---------------------------------------------------------------------
        sub_list = sorted([r_structural, r_thermal, r_hazmat, r_victim], reverse=True)
        r_base = (
            0.70 * sub_list[0]
            + 0.20 * sub_list[1]
            + 0.07 * sub_list[2]
            + 0.03 * sub_list[3]
        )
        r_total = min(100.0, round(r_base + synergy_bonus, 1))
        room_safety_score = round(100.0 - r_total, 1)

        # 5 Kademeli Genel Risk Seviyesi
        if r_total >= 78.0:
            risk_level = "SEVIYE_5_EKSTREM_FELAKET_RISKI"
            risk_color_bgr = (0, 0, 225)
        elif r_total >= 56.0:
            risk_level = "SEVIYE_4_KRITIK_ACIL_TEHLIKE"
            risk_color_bgr = (0, 45, 240)
        elif r_total >= 34.0:
            risk_level = "SEVIYE_3_YUKSEK_RISK_KONTROLLU"
            risk_color_bgr = (0, 130, 255)
        elif r_total >= 14.0:
            risk_level = "SEVIYE_2_DUSUK_ORTA_RISK"
            risk_color_bgr = (0, 195, 245)
        else:
            risk_level = "SEVIYE_1_GUVENLI_TEMIZ_ODA"
            risk_color_bgr = (40, 175, 40)

        # ---------------------------------------------------------------------
        # 7. INSARAG / AFAD KURTARMA TRİYAJ KODU VE OPERASYONEL PROTOKOL
        # ---------------------------------------------------------------------
        if critical_humans > 0 or (
            (conscious_humans > 0 or animals_count > 0)
            and (r_thermal >= 55.0 or r_structural >= 52.0 or r_hazmat >= 75.0)
        ):
            triage_code = "KIRMIZI_KOD_P1_ACIL_CAN_KURTARMA"
            triage_summary = "HAYATI ONCELIK (P1): Iceride kritik yarali veya aktif tehlike altinda canli var!"
        elif conscious_humans > 0 or animals_count > 0:
            triage_code = "TURUNCU_KOD_P2_ONCELIKLI_TAHLIYE"
            triage_summary = "ONCELIKLI TAHLIYE (P2): Odada kurtarilmayi bekleyen canli (insan/hayvan) tespit edildi."
        elif r_thermal >= 50.0 or r_structural >= 50.0 or r_hazmat >= 65.0:
            triage_code = "SARI_KOD_P3_TEHLIKE_IZOLASYONU_GIRIS_YASAK"
            triage_summary = "TEHLIKE IZOLASYONU (P3): Canli yok ancak oda cokme/yangin/CBRN acisindan yuksek tehlikeli!"
        elif r_structural > 0.0 or r_hazmat > 0.0:
            triage_code = "MAVI_KOD_P4_DIKKATLI_GOZLEM"
            triage_summary = "DIKKATLI GOZLEM (P4): Odada yuzeysel/orta yapisal hasar veya genel uyari mevcut."
        else:
            triage_code = "YESIL_KOD_P5_TEMIZ_GUVENLI_ODA"
            triage_summary = "TEMIZ ALAN (P5): Odada yapisal hasar, yangin, tehlike levhasi veya yarali yok."

        # Zorunlu KKD (PPE) ve Taktik Eylem Önerileri
        required_ppe: List[str] = ["Standart SAR Kaski & Eldiven"]
        tactical_actions: List[str] = []

        if critical_humans > 0:
            tactical_actions.append(
                f"ACIL MEDIKAL: {critical_humans} kritik kazazede (yerde/enkaz altinda) icin travma sedyesi hazirla."
            )
        if conscious_humans > 0 or animals_count > 0:
            tactical_actions.append(
                f"TAHLIYE: Icerideki {conscious_humans} kisi ve {animals_count} evcil hayvani guvenli koridordan cikar."
            )
        if has_active_flame or has_indoor_smoke:
            required_ppe.extend(["SCBA Bagimsiz Oksijen Tupu", "Yanmaz Nomex Kiyafet", "Termal Kamera"])
            tactical_actions.append(
                "YANGIN MUDAHALESI: Kapali oda Flashover riski! Kapiyi kontrolsuz acma, once sogutma/sondurme uygula."
            )
        if "elektrik_tehlikesi" in detected_hazmat_classes:
            required_ppe.append("Dielektrik Yalitkan Cizme & Eldiven")
            tactical_actions.append(
                "ELEKTRIK GUVENLIGI: Odaya girmeden once bina ana elektrik salterini kapat."
            )
        if any(c in ("kimyasal_toksik", "biyolojik_tehlike", "radyasyon") for c in detected_hazmat_classes):
            required_ppe.append("CBRN Koruyucu Tulum & Tam Yuz Gaz Maskesi")
            tactical_actions.append(
                "CBRN PROTOKOLU: Tehlikeli madde levhasi tespit edildi; gaz/radyasyon olcumu yapmadan girme."
            )
        if "yanici_patlayici" in detected_hazmat_classes:
            tactical_actions.append(
                "PATLAMA RISKI: Odada yanici/patlayici madde uyarisi var; kivilcim cikaracak kesici alet kullanma."
            )
        if has_exposed_rebar or r_structural >= 52.0:
            required_ppe.append("Agir Enkaz Titresim Sensoru")
            tactical_actions.append(
                "YAPISAL TAHKIMAT: Tavan/kolon cokme riski kritik! Teleskopik dikme (shoring) kurmadan odaya girme."
            )
        elif r_structural >= 28.0:
            tactical_actions.append(
                "YAPISAL KONTROL: Duvar/kolonda orta olcekli catlak mevcut; artci sarsintida tasiyici kolondan uzak dur."
            )

        if not tactical_actions:
            tactical_actions.append(
                "RUTIN TARAMA: Oda guvenli gorunuyor, SLAM haritasinda YESIL (Temiz) olarak isaretle."
            )

        # Kurtarma Ekibi Giriş İzni
        if r_total >= 75.0 and critical_humans == 0:
            entry_permission = "INSAN_GIRISI_YASAK_SADECE_DRONE_ROBOT"
        elif r_total >= 55.0:
            entry_permission = "SADECE_TAM_DONANIMLI_ACIL_KURTARMA_TIMI"
        elif r_total >= 25.0:
            entry_permission = "KONTROLLU_VE_KKD_ILE_DIKKATLI_GIRIS"
        else:
            entry_permission = "GUVENLI_SERBEST_GIRIS"

        return {
            "overall_risk_score_100": r_total,
            "room_safety_score_100": room_safety_score,
            "overall_risk_level": risk_level,
            "triage_code": triage_code,
            "triage_summary": triage_summary,
            "entry_permission": entry_permission,
            "sub_risk_scores": {
                "structural_collapse_risk": r_structural,
                "thermal_fire_smoke_risk": r_thermal,
                "hazmat_cbrn_risk": r_hazmat,
                "victim_criticality_score": r_victim,
                "compound_synergy_bonus": synergy_bonus,
            },
            "structural_flags": {
                "has_exposed_rebar": has_exposed_rebar,
                "has_spalling_or_brick": has_spalling_or_brick,
                "has_diagonal_shear_crack": has_shear_crack,
                "has_ceiling_overhead_hazard": has_ceiling_damage,
            },
            "synergy_alerts": synergy_alerts,
            "required_ppe": list(dict.fromkeys(required_ppe)),
            "tactical_actions": tactical_actions,
            "_hud_color_bgr": risk_color_bgr,
        }

    @staticmethod
    def draw_risk_assessment_hud(
        canvas: np.ndarray, risk_report: Dict[str, Any]
    ) -> np.ndarray:
        """
        Görüntünün üst ve alt kısmına Bitirme Projesi'nin ana katkısı olan
        'İç Mekan Çok-Modüllü Risk Değerlendirme ve Triyaj Telemetri Paneli'ni çizer.
        """
        h, w = canvas.shape[:2]
        r_tot = risk_report["overall_risk_score_100"]
        safety = risk_report["room_safety_score_100"]
        r_lvl = risk_report["overall_risk_level"]
        triage = risk_report["triage_code"]
        subs = risk_report["sub_risk_scores"]
        col = tuple(int(c) for c in risk_report.get("_hud_color_bgr", (0, 140, 255)))

        # 1. ÜST ANA TRİYAJ & RİSK BANNER'I (54px yüksekliğinde)
        top_h = 54
        overlay_top = canvas.copy()
        cv2.rectangle(overlay_top, (0, 0), (w, top_h), (18, 20, 26), -1)
        cv2.addWeighted(overlay_top, 0.88, canvas, 0.12, 0, canvas)
        cv2.rectangle(canvas, (0, 0), (10, top_h), col, -1)
        cv2.line(canvas, (0, top_h), (w, top_h), col, 2)

        line1 = f"INDOOR SAR RISK ENGINE | GENEL RISK: {r_tot:.0f}/100 ({r_lvl}) | ODA GUVENLIGI: {safety:.0f}/100"
        line2 = f"TRIYAJ: {triage} | GIRIS: {risk_report['entry_permission']}"
        font = cv2.FONT_HERSHEY_SIMPLEX
        s1 = min(0.54, max(0.38, w / 1650.0))
        s2 = min(0.46, max(0.35, w / 1850.0))

        cv2.putText(canvas, line1, (18, 22), font, s1, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(canvas, line2, (18, 44), font, s2, (0, 235, 255), 1, cv2.LINE_AA)

        # 2. ALT 4-BOYUTLU ALT RİSK TELEMETRİ PANELİ (56px yüksekliğinde)
        bot_h = 56
        y0 = max(top_h + 10, h - bot_h)
        overlay_bot = canvas.copy()
        cv2.rectangle(overlay_bot, (0, y0), (w, h), (18, 20, 26), -1)
        cv2.addWeighted(overlay_bot, 0.86, canvas, 0.14, 0, canvas)
        cv2.line(canvas, (0, y0), (w, y0), (90, 95, 110), 1)

        metrics_txt = (
            f"ALT RISKLER -> Yapisal:{subs['structural_collapse_risk']:.0f} | "
            f"Termal/Yangin:{subs['thermal_fire_smoke_risk']:.0f} | "
            f"CBRN/Levha:{subs['hazmat_cbrn_risk']:.0f} | "
            f"Kazazede:{subs['victim_criticality_score']:.0f} | "
            f"Sinerji:+{subs['compound_synergy_bonus']:.0f}"
        )
        action_txt = (
            risk_report["synergy_alerts"][0]
            if risk_report["synergy_alerts"]
            else f"ONERI: {risk_report['tactical_actions'][0]}"
        )
        if len(action_txt) > 108:
            action_txt = action_txt[:105] + "..."

        cv2.putText(
            canvas, metrics_txt, (14, y0 + 21), font, s2, (210, 250, 210), 1, cv2.LINE_AA
        )
        cv2.putText(
            canvas, action_txt, (14, y0 + 44), font, s2, (80, 205, 255), 1, cv2.LINE_AA
        )

        return canvas
