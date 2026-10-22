# API

Interactive OpenAPI docs: **http://127.0.0.1:8080/docs** (try every endpoint from the browser).

## REST (JSON)

| Method | Path | Body / notes |
|---|---|---|
| GET | `/api/status` | all drones: connection, link rates, mode, mission, recording |
| GET | `/api/meta` | telemetry field names/units |
| GET | `/api/events?limit=300` | recent events + commands |
| POST | `/api/drones/{id}/connect` · `/disconnect` | |
| POST | `/api/drones/{id}/command` | `{"text": "forward 100"}` → `{ok, resp, dt}` (any Tello SDK command) |
| POST | `/api/drones/{id}/emergency` · `/hover` | |
| POST | `/api/drones/{id}/mode` | `{"mode": "MANUAL|MISSION|AUTO"}` |
| GET | `/api/drones/{id}/preflight` | `{checks, blockers}` |
| POST | `/api/drones/{id}/pose` | external pose `{x, y, z, yaw}` (map frame, m/deg) — overrides dead-reckoning for 1 s |
| POST | `/api/drones/{id}/home` | reset odometry home |
| GET | `/api/drones/{id}/track` | trajectory points |
| GET | `/api/drones/{id}/frame.jpg` | latest video frame |
| POST | `/api/drones/{id}/snapshot` | save JPEG into the session |
| POST | `/api/drones/{id}/record` | `{"what": "video"|"log", "on": true}` |
| POST | `/api/fleet/command` | `{"text": "takeoff", "drones": ["a","b"]}` (all if omitted) |
| GET/POST/DELETE | `/api/missions[/{file}]` | list / load / save / delete |
| POST | `/api/missions/preview` | mission → `{points, distance_m, est_time_s, max_range_m, fence_ok, error}` |
| POST | `/api/drones/{id}/mission/{start|pause|resume|abort}` | start takes the mission JSON |
| GET | `/api/vision` | plugins, params, enabled per drone |
| POST | `/api/vision/{name}/enable` | `{"drone": "tello1", "on": true}` |
| POST | `/api/vision/{name}/params` | `{"threshold": 0.4}` |
| GET | `/api/autonomy` · POST `/api/drones/{id}/autonomy` | `{"name": "track_target", "values": {...}, "engage": true}` |
| GET | `/api/map` | markers, clouds, tracks |
| POST | `/api/map/markers` · DELETE `/api/map/markers/{id}` · POST `/api/map/clear` | `{label, x, y, z, source}` |
| POST | `/api/map/clouds/{name}` | `{"points": [[x,y,z], ...], "color": "#94a3b8"}` |
| GET | `/api/recordings` · `/api/recordings/{session}/{file}` · DELETE | flight logs |
| GET/POST | `/api/settings` · POST `/api/settings/reset` | partial updates are merged |
| **SAR / navigation (v2)** | | |
| GET | `/api/drones/{id}/rth` | RTH estimate along the trail: `{path_m, straight_m, time_s, bat_needed, drain_pct_s, legs, path}` |
| POST | `/api/drones/{id}/rth` | `{"land": true}` → fly the breadcrumb trail home as a mission |
| POST | `/api/drones/{id}/goto_target/{tid}` | fly the trail back to where target `tid` was seen, finish facing it |
| GET | `/api/targets/{tid}/backtrack` | rescue route: target → first-seen point → trail → home, `{path, length_m}` |
| POST | `/api/drones/{id}/mark` | operator mark on the video `{"u": 0.62, "v": 0.48, "label": "victim"}` (normalised); repeated marks triangulate |
| POST | `/api/drones/{id}/bookmark` | `{"note": "check door"}` → map flag + snapshot |
| GET | `/api/coverage` | `{res, cells: [[i, j, looks], ...], area_m2}` |
| GET | `/api/patterns` · POST `/api/patterns/{name}` | search-pattern catalogue / generate a mission from params |
| GET | `/api/recordings/{session}/report.html` | printable post-flight report |
| GET | `/api/recordings/{session}/export.zip?what=all|dataset` | whole session, or only the 3D dataset (keyframes, transforms.json, COLMAP) |

## WebSocket `/ws`

Server → client: `{"topic": "hello", "data": {...}}` once, then `{"topic": "batch", "data": [{"topic", "data"}, ...]}`.
Topics: `telemetry` (≤15 Hz/drone, `status.nav` = RTH estimate, `status.collision` = TTC), `link`, `event`, `command`, `mode`, `flight`, `mission`, `detections`, `map`, `recording`, `autonomy`, `coverage` (cell deltas, 1 Hz), `target` (a target was confirmed), `collision` (guard braked).

Client → server:
```json
{"type": "sticks", "drone": "tello1", "v": [lr, fb, ud, yaw]}   // -100..100, send at 10-20 Hz
{"type": "cmd", "drone": "tello1", "text": "land"}
{"type": "ping", "t": 123.4}                                     // -> {"topic": "pong"}
```

## WebSocket `/ws/video/{id}`

Binary messages: 12-byte little-endian header (`uint32 frame_id`, `float64 unix_ms`) + JPEG.

## Raw video for other programs

Settings → video → `relay_udp: ["127.0.0.1:5600"]` forwards the untouched Tello H.264 packets. Open with PyAV/FFmpeg/GStreamer (`udp://127.0.0.1:5600`), ROS `ffmpeg_image_transport`, ORB-SLAM wrappers, etc.
