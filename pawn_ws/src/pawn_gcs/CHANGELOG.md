# Changelog

## 2.0.0 — search & rescue release
* **3D target localisation**: detection rays triangulated across viewpoints + known-size ranging, data association,
  1σ uncertainty, tentative → confirmed; operator click-to-mark on the video (`K`).
* **Backtracking**: rescue route per target along the flown trail; **RTH along breadcrumbs** (`Shift+H`),
  go-to-target; **smart RTH battery** with learned drain rate (`safety.smart_rth`).
* **Coverage heat map** + explored area; **IAMSAR search-pattern generator** (expanding square, parallel track,
  sector, orbit, scan grid); bookmarks (`M`); SAR summary panel.
* **Collision guard**: monocular time-to-contact (LK looming) plugin `collision_ttc` + automatic brake (`stop`).
* **3D twin dataset** in every flight log: keyframes + `transforms.json` (nerfstudio/3DGS) + COLMAP model.
* **Post-flight report** (HTML/PDF), session & dataset zip export.
* Voice call-outs (EN/TR), gamepad / RC-transmitter axis learning, PWA manifest.
* Simulator: textured walls (posters/panels) for feature tracking & SLAM; room shell drawn first.
* Standalone builds (PyInstaller) for Windows/macOS/Linux + release workflow; 43 tests.

## 1.0.0
* Live video + HUD, telemetry, missions, vision plugins, autonomy controllers, flight logs & review,
  multi-drone, safety failsafes, simulator, REST/WS API.
