# Search & rescue features (v2) — how they work

PAWN's main task: fly a Tello through an unknown building **with only the 2D FPV video coming down and 4-axis
sticks going up**, find a red triangle / a person, show **where** it is on the map and **how to get back to it**.
This page explains the algorithms behind each feature so the team can reason about them, tune them and replace
them (e.g. with the SLAM module) without reading all the code.

Frames everywhere: **x = forward at takeoff, y = left, z = up (m), yaw CCW+ (deg)**. Images: normalised `u, v ∈ [0,1]`.

---

## 1. Target localisation — `core/targets.py`, `core/geometry.py`

*Türkçe özet:* Tek kamera derinlik ölçemez. Her tespiti, drone'un o anki konumundan çıkan bir **ışın** olarak
alıyoruz. Farklı yerlerden görülen ışınlar hedefte kesişir (**triangülasyon**). Hedefin gerçek boyu biliniyorsa
(kırmızı üçgen 0.85 m, insan 1.7 m) pikseldeki boyundan **mesafe** de çıkar. İkisini tek bir en küçük kareler
probleminde birleştiriyoruz; sonuç hedefin 3D konumu + belirsizlik çemberi.

1. **Ray**: pixel `(u, v)` → camera ray `((u·W − cx)/fx, (v·H − cy)/fy, 1)` → body (camera tilt) → world (yaw).
   `fx = (W/2) / tan(HFOV/2)`, HFOV = `camera.hfov_deg` (70° for the 960×720 Tello stream).
2. **Known-size range** (optional): depth `Z = fy · H_real / h_px`, `H_real` from `targets.known_height_m`.
   Ignored when the box touches the image border (cut-off objects lie about their size).
3. **Fusion** — weighted linear least squares, closed form:

   ```
   ray i      :  (I − dᵢdᵢᵀ)(X − pᵢ) ≈ 0      σ⊥ = 0.035 rad × distance
   range i    :  dᵢdᵢᵀ (X − (pᵢ + rᵢdᵢ)) ≈ 0   σr = 15 % of range
   (Σ Wᵢ) X = Σ Wᵢ cᵢ      →  X,   Cov = (Σ Wᵢ)⁻¹   →   σ = √λmax(Cov_xy)
   ```
   Two passes: the second re-weights each ray with the estimated distance.
   *method* = `bearing` (one ray / parallel rays, shown as a dashed ray on the map), `triangulation`,
   `known-size range`, or `range+triangulation`.
4. **Data association**: a new detection joins the target of the same label whose estimate lies closest to its
   ray (gate = max(`targets.gate_m`, 0.35·distance)); otherwise a new target starts. → two people stay two targets.
5. **Confirmation**: `targets.confirm_hits` hits **and** σ ≤ `targets.confirm_sigma_m` → `CONFIRMED`
   (event, voice call-out, `target` bus topic, written to `targets.json` in the flight log).

Operator **✚ Mark** mode (key `K`) feeds clicks on the video through exactly the same code: one click = one
bearing; slide sideways and click again → triangulated. Useful when a detector misses a person the pilot sees.

Measured in the simulator: red triangle localised to **≈ 0.1–0.2 m** from ~6 m away with dead-reckoning poses.
With real drift expect more — feeding a SLAM pose to `/api/drones/{id}/pose` improves it directly.

## 2. Backtracking & return-to-home — `core/navigation.py`

*Türkçe özet:* Bina içinde GPS ve harita yok; eve düz çizgi duvardan geçebilir. Ama drone'un **zaten uçtuğu yol
boştur**. Bu yüzden geri dönüş, uçulan iz (breadcrumb) üzerinden yapılır.

* The odometry track (a point every ≥ 3 cm) is the breadcrumb trail.
* **Rescue route** of a target (map → click target → *route*): target → point where it was **first seen** →
  trail → home. This is what you hand to the rescue team ("go through the door, left, 7.2 m").
* **RTH** (`⌂ RTH`, Shift+H, or automatic): current position → nearest trail point → trail → home, simplified
  with Ramer–Douglas–Peucker (`navigation.rdp_eps_m`), legs < 0.25 m merged, converted to a normal mission:
  `speed`, then per leg *rotate to face the leg* (camera looks where it flies) + `forward` (chunks ≤ 5 m),
  `hover`, `land`. Pilot stick input aborts it like any mission.
* **Go to target**: same, to the first-seen point, ending with a rotation that faces the target.
* **Smart RTH battery** (`safety.smart_rth`: `off|warn|rth`): needed % = (trail length / speed + 2 s per leg + 6 s)
  × drain rate + `navigation.reserve_pct`. The drain rate is **learned during the flight** from the battery
  trend (fallback `default_drain_pct_s`). When `battery ≤ needed + battery_land` → warning (and RTH if `rth`).

## 3. Coverage map — `core/coverage.py`

Every 0.4 s in flight the camera's horizontal FOV wedge (radius `coverage.range_m`) is stamped into a 0.25 m grid;
the map shows it as a green heat map and the SAR panel shows explored m². Gaps = not yet searched. No occlusion
test (no depth) — a SLAM module can replace `update()` with ray casting against its map.

## 4. Search patterns — `core/patterns.py`

Mission generators after IAMSAR (scaled to rooms): **expanding square** (s, s, 2s, 2s, 3s…), **parallel track**
(lawnmower), **sector** (3 × 120° triangles rotated 30°), **orbit** (polygon around a point R m ahead while
facing it — photogrammetry capture for the 3D twin), **scan grid** (360° scans at grid points). Optional
`detect_plugin`: a short `wait_detection` before each leg. Output is a normal mission → preview, edit, save, run.

## 5. Collision guard — `vision/plugins/builtin.py: CollisionTTC`, `core/safety.py`

*Türkçe özet:* Bir yüzeye yaklaşırken görüntü **büyür** (looming). Büyüme hızından çarpmaya kalan süre
(time-to-contact, TTC) hesaplanır — mesafe ya da hız bilmeden. TTC eşik altına inerse drone frenlenir.

* Corners (Shi–Tomasi) in the central ROI are tracked with pyramidal Lucas–Kanade between frames
  `baseline_s` apart, forward-backward checked. Pairwise distance ratios (translation-invariant → yaw and
  sideways motion don't matter) give the image scale `s`; **TTC = Δt / (s − 1)** (Lee's τ).
* Texture lost while closing in (blank wall fills the view) → the last TTC keeps counting down for ≤ 2 s
  (constant-velocity extrapolation) instead of going blind exactly when it matters.
* Safety monitor: TTC < `ttc_warn_s` while moving forward (pilot pitch > 10 or a move command running or AUTO)
  → `hover(hard=True)`: mission/AUTO aborted, `rc 0 0 0 0` and SDK `stop` (interrupts a running move).
* Optional `safety.dynamic_obstacle_brake`: also brake for a large moving blob (plugin `motion`) in the path.
* Simulator check: approaching a wall at 0.6 m/s, the TTC estimate tracks ground truth within ~10 % and the
  drone stops short of the wall (`tests/test_collision_e2e.py`). Limits: textureless surfaces, glass, very thin
  obstacles; the guard is an assistant, not a guarantee.

## 6. 3D digital-twin dataset — `core/keyframes.py`

Every flight log stores keyframes (sharp — Laplacian variance — and moved ≥ 0.25 m or turned ≥ 12°) with their
poses and writes `transforms.json` (nerfstudio / instant-ngp / Gaussian splatting, OpenGL camera convention) and a
COLMAP text model (`colmap/cameras.txt`, `images.txt` with the poses as priors, empty `points3D.txt`).

```bash
# Gaussian splat / NeRF
ns-train splatfacto --data recordings/<session>
# COLMAP with the GCS poses as priors (then dense: image_undistorter + patch_match_stereo, or OpenMVS)
colmap feature_extractor --database_path db.db --image_path keyframes
colmap exhaustive_matcher --database_path db.db
colmap point_triangulator --database_path db.db --image_path keyframes --input_path colmap --output_path sparse
```
Download from **Flight review → ⇩ 3D dataset**. Poses are dead-reckoning unless a SLAM pose is posted, so for
best results let COLMAP refine them (`mapper` / `bundle_adjuster`).

## 7. Post-flight report

**Flight review → 📄 Flight report**: summary (duration, distance, battery, max alt/range/temperature), trajectory
with targets and uncertainty, altitude/battery/speed/temperature charts, target table, warnings timeline,
snapshots. Self-contained HTML, light/dark, print → PDF for the CS491 reports.

## Where to plug in the team's work

| You build | Replace / feed |
|---|---|
| Monocular SLAM (Şebnem, Doğa) | POST `/pose` (better targets, RTH, coverage, dataset poses), `/map/clouds/{name}` |
| Detector (Nijat) | a `VisionPlugin` with `is_target = True` → localisation, confirmation, map, report for free |
| Obstacle avoidance | a `VisionPlugin` like `collision_ttc`, or an autonomy controller |
| Exploration / autonomy | an autonomy `Controller` (AUTO mode); `nav.path_home()` and coverage are available |
