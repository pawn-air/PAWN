# Changelog

## 3.1.0 — recordings usable as SLAM datasets
* `video.mp4` holds every decoded frame exactly once with its real time (variable frame rate); frames are no
  longer duplicated or skipped to fake a constant 30 fps.
* New `frames.csv` per video: frame index → decode time `t` (same clock as `telemetry.csv`) → decoder frame id.
* `meta.json` gets a `video` summary: frames, dropped frames, first/last frame time, encoder.
* Frames the writer has to drop are counted (shown in the "Video saved" event) instead of vanishing silently.
* New setting `recording.video_crf` (default 18, was a fixed 23) for less re-encoding loss.

## 3.0.0 — minimal GCS
* Stripped to video, keyboard control, raw telemetry and logs. The GCS never flies on its own.
* Removed: missions, map, vision plugins, autonomy, target localisation, RTH/backtracking, coverage, search patterns,
  collision guard, 3D dataset, reports, voice, gamepad/touch sticks, fleet sync, pre-flight takeoff blocking,
  battery/temperature/geofence/client-loss failsafes, odometry, HUD.
* Kept as empty tabs: Map, Mission, Vision, Autonomy (module slots, see docs/ARCHITECTURE.md).
* Safety = stick limit clamped on the GCS, dead-man, link-lost timeout.
* Telemetry shows only the fields the drone sends, one chart each, plus the raw packet stream.
* Unsolicited drone messages (late replies, `error Auto land`, …) are shown and logged.
* Telemetry CSV now carries the raw state string and the rc sent.
* Old settings.json / config.toml keys of removed features are ignored.

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
