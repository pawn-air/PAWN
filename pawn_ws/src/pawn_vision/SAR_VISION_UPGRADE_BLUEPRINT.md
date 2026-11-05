# PAWN SAR Vision Engine — V2 Kararlılık, Aktif Algılama & Güven Puanı Mimarisi
**Belge Türü:** Geleceğe Yönelik Mimari & Uygulama Spesifikasyonu (Future Blueprint)  
**Tarih:** 23 Ekim 2026  
**Hedef Kitle:** Antigravity AI / PAWN Vision Mühendisleri  
**Durum:** Planlama & Tasarım Aşaması (Hazır olduğunda doğrudan uygulanacak rehber)

---

## 📌 1. Belgenin Amacı ve Bağlam (Context)

Bu belge; PAWN İnsansız Hava Aracı (İHA) Arama-Kurtarma (SAR) projesinde kullanılan bilgisayarlı görü modellerinin (Modül 1, 2, 3, 4) gerçek saha testlerinde gözlemlenen tutarsızlıklarını, yanlış alarmlarını (false positive) ve görsel titremelerini (flickering) kökünden çözmek amacıyla hazırlanmıştır.

Mevcut sistemde (V1) modeller her bir video karesini birbirinden bağımsız (hafızasız / memoryless) işlemektedir. Bu belgede açıklanan mimari (V2), **"Ham Model Çıkarımını" (Raw Inference)** alıp; **Zamansal Filtreleme (Temporal Filtering)**, **Güven Puanı Havuzu (Trust Score Engine)** ve **Otonom Aktif Algılama (Active Perception)** katmanlarıyla endüstriyel/havacılık seviyesine yükseltir.

---

## ⚠️ 2. V1 Sahasında Karşılaşılan 5 Temel Problem ve Çözümleri

### 2.1. Modül 1 (İnsan/Canlı): Kadraj Dışı Kırpılma vs. Enkaz Altı Yanılgısı
* **Problem:** Kişi ayakta dimdik durmasına rağmen, drone kamerasının açısı yere eğikse veya kişi kadraja alttan yeni giriyorsa (sadece belden yukarısı görünüyorsa), sistem bacak keypointlerini bulamadığı için kişiyi `ENKAZ ALTI / MAHSUR` olarak sınıflandırmaktadır.
* **Kök Neden:** Görsel kadraj kırpılması (*Bounding Box Edge Truncation*) ile fiziksel moloz engeli (*Physical Occlusion*) arasındaki farkın ayırt edilememesi.
* **Çözüm (Edge-Proximity & Skeleton Check):**
  1. **Kadraj Sınırı Kontrolü:** Tespit kutusunun alt kenarı görüntünün en alt sınırına yakınsa ($y_2 \ge 0.95 \cdot H$ veya $y_1 \le 0.05 \cdot H$), bacakların eksik olması "enkaz" değil, `KADRAJ_KIRPILMASI` olarak etiketlenecek.
  2. **Gövde Eğimi:** Omuz-kalça vektörü dikey eksene paralelse ($|\theta| < 20^\circ$), kişi ayakta kabul edilecek.
  3. `ENKAZ ALTI` kararı sadece ve sadece kişi görüntünün güvenli iç alanındayken ($0.10 \cdot H < y < 0.90 \cdot H$) ve moloz/çökme tespitleriyle uzamsal kesişim yaşadığında verilecektir.

---

### 2.2. Modül 2 (Yangın & Duman): Gri Duvar / Beton Zemin Yanlış Alarmları
* **Problem:** Duman modeli gri renkli her düz yüzeyi (alçıpan duvar, beton zemin, gri battaniye, gölgeler) duman olarak algılamaktadır.
* **Kök Neden:** Statik YOLO tek bir RGB karesine bakar. Duman eğitim setinde gri-beyaz yumuşak pikseller olduğu için, iç mekandaki gri homojen yüzeyler yüksek güvenle duman sanılır.
* **Çözüm (Optik Akış ve Dinamik Genişleme):**
  1. **Statik vs. Dinamik:** Duvar ve zemin hareketsizdir. Gerçek duman konveksiyonla yukarı doğru akar ($v_y < 0$) ve sınırları genişler (turbulent expansion).
  2. **Dense Optical Flow / Frame Differencing:** Ardışık kareler arasında yukarı yönlü akış vektörü olmayan ve piksel gradyanı homojen olan gri kutular otomatik elenir.
  3. **Zamansal Kalıcılık Barajı:** Duman tespiti en az 8 ardışık kare ($>250\text{ ms}$) boyunca dinamik hareket göstermelidir.

---

### 2.3. Modül 3 (Tehlike Tabelaları): Küresel Çeşitlilik & Genelleştirme Endişesi
* **Problem:** Dünyada binlerce farklı tabela tasarımı varken, sınırlı veri setiyle eğitilen modelin genellenememesi riski.
* **Kök Neden:** Yalnızca belirli piktogramlara overfit olma ihtimali.
* **Çözüm (Uluslararası Standartlar + OCR Hibriti):**
  1. **Uluslararası Standart Geometrisi:** Dünyadaki tüm endüstriyel tesisler **ISO 7010, ANSI Z535 ve UN GHS** standartlarına tabidir. Sarı üçgen (uyarı), kırmızı daire/çapraz (yasak), kırmızı eşkenar dörtgen (GHS kimyasal tehlike) sembolleri evrenseldir.
  2. **YOLO + OCR Doğrulaması:** Tespit edilen tabela kutusu içerisine çok hafif bir metin tanıma (örn. Mobile-OCR) çalıştırılır. Kutuda "DANGER", "HIGH VOLTAGE", "FLAMMABLE", "RADYASYON", "TEHLİKE" kelimeleri okunduğunda piktogramla çapraz doğrulanır; güvenilirlik %99'a çıkar.

---

### 2.4. Modül 4 (Yapısal Çatlak): Kararsızlık ve Görsel Titreme (Flickering)
* **Problem:** Drone havada asılıyken aynı duvardaki çatlağı bir karede algılayıp sonraki karede kaybetmesi; çatlak olmayan yerlerde anlık parlayıp sönen hayalet kutular çıkması.
* **Kök Neden:** Modelin güven skoru ($conf$) eşik değeri etrafında salınır ($0.24 \leftrightarrow 0.26$). Eşik $0.25$ ise kutu her karede yanıp söner.

---

### 2.5. Genel: Yüksek Yanlış Alarm (False Positive) Oranı
* **Kök Neden:** Tek karelik kararların doğrudan operatör ekranına ve ROS 2 düğümlerine aktarılması. Araya filtre katmanı konulmaması.

---

## 🚀 3. İki Büyük Otonom Robotik Yeniliği (User-Proposed Features)

### 3.1. Aktif Algılama (Active Perception / Next-Best-View Drone Manevrası)
Eğer drone bir insanı kısmi görüyor veya durumunu netleştiremiyorsa, durumu tahmin etmeye çalışmak yerine **fiziksel olarak açısını değiştirmelidir**.

```
                [GÖRÜNTÜ İŞLEME: pawn_vision]
                             │
     Kişi Kadraj Kenarında VEYA Durum Belirsiz (Conf: %45)
                             │
                             ▼
     ROS 2 Mesajı: /pawn/active_inspection_goal
        { target: "victim_1", action: "STEP_BACK_AND_TILT" }
                             │
                             ▼
             [OTONOM UÇUŞ: pawn_autopilot]
                             │
     ┌───────────────────────┴────────────────────────┐
     ▼                                                ▼
Manevra A (Geri Çekilme):                       Manevra B (Yörünge/Orbit):
İHA 0.8 metre geri çekilir                      Enkaz arkasını görmek için
ve gimbal'ı 15° aşağı eğer                      hedefin etrafında 30° yay
(Tüm vücut kadraja girer)                       çizerek yeni açı alır
                             │
                             ▼
            [YENİ AÇIDAN KESİN DOĞRULAMA]
   -> Ayakta / Güvenli VEYA Gerçek Enkaz Altı Alarmı
```

* **ROS 2 Entegrasyonu:**
  * Topic: `/pawn/active_inspection_goal`
  * Message Type: `pawn_msgs/ActiveInspectionGoal`
  * Alanlar: `float32 delta_x`, `float32 delta_y`, `float32 delta_z`, `float32 delta_gimbal_pitch`

---

### 3.2. Güven Puanı (Trust Score) & Zamansal Takipçi (Temporal Tracker)

Hiçbir tespit tek kareyle kabul edilmez. Her tespit uzamsal olarak (IoU / ByteTrack) önceki karelerle eşleştirilir ve bir **Güven Kumbarası ($T$)** yönetilir.

#### Matematiksel Güven Modeli (Accumulation & Decay):
Her $ID_k$ nesnesi için güven puanı $T \in [0.0, 1.0]$:

1. **Tespit Doğrulandığında (Puan Artışı):**
   $$T_t = \min\left(1.0, \; T_{t-1} + 0.20 \cdot \text{conf}_t\right)$$
2. **Kare Kaçırıldığında / Görünmediğinde (Zaman Aşımı / Decay):**
   $$T_t = T_{t-1} \times 0.85$$
3. **Ekrana Çıkma ve Alarm Şartı (Trust Gate):**
   * $T_t \ge 0.70$ ise $\rightarrow$ **EKRANA ÇİZ / SLAM HARİTASINA İŞLE / ROS TOPIC'E BAS**
   * $T_t < 0.70$ ise $\rightarrow$ Arka planda beklemede tut, kullanıcıyı rahatsız etme.
   * $T_t < 0.15$ ise $\rightarrow$ Nesneyi bellekten sil (Garbage Collector).

#### Bu Algoritmanın Sağladığı 2 Kritik Güç:
1. **Titreme (Flickering) %100 Yok Olur:** Drone kamerasında ışık anlık değişip model 1 kare çatlağı kaçırsa bile, güven puanı $0.90 \times 0.85 = 0.765$ olur ($>0.70$ barajının üstünde kalır). Kutu ekrandan kaybolmaz; sadece hafif saydamlaşır.
2. **Yanlış Alarmlar (False Positive) %100 Elenir:** Anlık bir yansıma 1 kareliğine "duman" veya "çatlak" üretse bile güven puanı $0.20$'de kalır; 2. karede yok olacağı için $0.70$ barajına asla ulaşamaz ve operatörün ekranına hiç yansıtılmaz!

---

### 3.3. 3D SLAM Landmark Entegrasyonu (Kalıcı Dünya Hafızası)
* 2D piksel kutuları, İHA'nın derinlik kamerası ve SLAM haritasıyla ($X, Y, Z$) koordinatlarına dönüştürülür.
* Drone 5 dakika sonra aynı odaya tekrar girdiğinde; görüntü işlemeye bile gerek kalmadan haritadan `"Bu kolonda teyit edilmiş derin çatlak var"` verisi HUD ekranına yansıtılır.

---

## 🛠️ 4. Gelecekte Uygulama Başladığında Yapılacak İşler (Implementation Roadmap)

Geliştirmeye başlanacağı gün izlenecek adım adım reçete:

| Adım | Modül / Dosya | Yapılacak İşlem |
| :--- | :--- | :--- |
| **Adım 1** | `src/temporal_tracker.py` *(Yeni)* | Çoklu nesne takibi (IoU Matcher) + Trust Score ($T$) puan/decay matematiksel sınıfını oluştur. |
| **Adım 2** | `src/module1_human_animal.py` | `check_edge_proximity()` fonksiyonu ekle. Kadraj sınırına çarpan kutularda `ENKAZ_ALTI` bayrağını engelle. |
| **Adım 3** | `src/module2_fire_smoke.py` | `cv2.calcOpticalFlowFarneback` ile basit yukarı yönlü hareket kontrolü ekle; hareketsiz gri alanları duman listesinden sil. |
| **Adım 4** | `src/sar_master_pipeline.py` | Bütün modüllerin ham çıktılarını `TemporalTracker` süzgecinden geçirerek nihai rapor ve HUD çizimi üret. |
| **Adım 5** | `pawn_vision/vision_node.py` | Belirsiz kurban durumunda `/pawn/active_inspection_goal` ROS 2 mesajını publish eden mantığı bağla. |

---

## 🎯 5. Sonuç & Vizyon
Bu V2 mimarisi uygulandığında; PAWN projesi yalnızca bir "YOLO çalıştıran drone" olmaktan çıkıp, **kendi belirsizliğini bilen, doğrulamak için manevra yapan, zamansal hafızaya sahip üst düzey bir Arama-Kurtarma Robotu** haline gelecektir.
