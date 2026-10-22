# Architecture

```
 Browser(s)  ─── HTTP/REST ───┐                         ┌── UDP 8889  commands / replies ──┐
  web/ UI    ─── WS /ws  ─────┤      pawn_gcs (Python)   ├── UDP 8890  state 10 Hz  ◄──────┤  DJI Tello
             ─── WS /ws/video ┤                         ├── UDP 11111 H.264 video ◄───────┘  (or sim/)
 Scripts / SLAM / ROS ────────┘                         └── UDP relay → SLAM node (optional)
```

## Layers

| Layer | Module | Responsibility |
|---|---|---|
| Transport | `core/tello.py` – `TelloLink` | UDP sockets, serialised command/reply matching with per-command timeouts, fire-and-forget `rc`, state parsing, raw video packets |
| Video | `core/video.py` | PyAV H.264 decoder thread → `FrameStore` (latest frame only; consumers never queue up latency), optional raw UDP relay |
| Drone node | `core/node.py` – `DroneNode` | One per drone: SDK connect/stream, flight state, **control arbitration** (pilot > mission > AUTO), 20 Hz rc loop with dead-man, keep-alive, link health |
| Estimation | `core/odometry.py` | Dead-reckoning from vgx/vgy + height, shared-map home offset, external (SLAM) pose override |
| Safety | `core/safety.py` | Pre-flight checks & takeoff blocking, battery/temperature/link/geofence/client-loss failsafes (5 Hz) |
| Missions | `core/mission.py` | Validation, kinematic preview, executor (pause/resume/abort, pilot override), JSON store |
| Recording | `core/recorder.py` | Session folders: telemetry CSV, events JSONL, MP4 (H.264 via PyAV, mp4v fallback), snapshots |
| Map | `core/mapstore.py` | Target markers (pose + bearing at detection), external point clouds |
| Vision | `vision/` | Plugin discovery (built-in + `plugins/`), one worker thread per drone on the latest frame, detections → bus, target markers |
| Autonomy | `autonomy/` | Controller plugins called from the rc loop in AUTO mode |
| Bus | `core/bus.py` | Thread-safe pub/sub; every module communicates through topics, the WebSocket forwards them all |
| API | `server.py`, `app.py` | FastAPI REST + WebSockets, static UI, lifespan wiring |
| UI | `web/` | Vanilla ES modules: `core.js` (WS/REST/store), `video.js` (HUD), `control.js` (keyboard/gamepad/touch), `mapview.js` (2D/3D canvas), `tabs/*.js` |

## Bus topics

`telemetry`, `link`, `event`, `command`, `mode`, `flight`, `mission`, `detections`, `map`, `recording`, `autonomy`.
Any module can subscribe (`bus.subscribe("detections", fn)`), so e.g. a ROS bridge or a database logger is one new file.

## Control authority

```
emergency / safety actions  >  pilot sticks (any non-zero input)  >  mission step  >  AUTO controller
```

* Pilot sticks are streamed at 20 Hz; the server zeroes them after `rc.deadman_ms` without packets.
* Any non-zero pilot input switches MISSION/AUTO back to MANUAL and aborts the mission.
* Discrete commands (`forward 100`) block the link until the Tello replies; the rc loop pauses meanwhile so the two never fight.
* Emergency is sent raw (bypasses the command queue) to every drone.

## Threads

Event loop (asyncio): UDP I/O, rc loops, safety, missions, API. Separate threads: H.264 decode (per drone), vision (per drone), video file writer, simulator video encoder. Cross-thread communication only through `FrameStore` and `bus.publish` (thread-safe).

## Extending

* New vision algorithm → `plugins/my_detector.py` (see PLUGINS.md)
* New autonomy policy → `Controller` subclass in `plugins/`
* SLAM / mapping → external process using the API (`examples/slam_bridge.py`)
* New UI tab → add a button + pane in `index.html`, a module in `web/js/tabs/`, import it in `main.js`; listen with `on('telemetry', ...)` etc.
* New drone type → implement a link with the same surface as `TelloLink` (`command`, `send_raw`, `on_state`, `on_video`) and keep `DroneNode` unchanged.

## v2 modules (search & rescue)

```
vision runner ──detections (is_target)──▶ MapStore.add_observation ──▶ TargetLocalizer (rays + known size, LSQ)
                                                   │                          └─▶ bus "map" / "target"
telemetry ─▶ Odometry.track (breadcrumbs) ─▶ Navigator: backtrack_path / rth / goto_target ─▶ MissionExecutor
          └▶ CoverageGrid (FOV wedge stamps) ─▶ bus "coverage"
frames ─▶ KeyframeRecorder (sharp + parallax) ─▶ session/keyframes + transforms.json + colmap/
SafetyMonitor ─▶ collision guard (collision_ttc / motion detections) · smart RTH (BatteryModel) · existing failsafes
SessionIndex ─▶ report.html (core/report.py) · export.zip
```
Algorithms and tuning: [SAR.md](SAR.md).
