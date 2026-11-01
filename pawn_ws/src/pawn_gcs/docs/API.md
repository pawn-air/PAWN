# API

Interactive OpenAPI docs: **http://127.0.0.1:8080/docs**.

## REST (JSON)

| Method | Path | Body / notes |
|---|---|---|
| GET | `/api/status` | all drones: connection, link/video rates, last rc sent, recording |
| GET | `/api/meta` | telemetry field names/units |
| GET | `/api/events?limit=500` | recent log: commands + replies, drone messages, GCS events |
| POST | `/api/drones/{id}/connect` · `/disconnect` | |
| POST | `/api/drones/{id}/command` | `{"text": "forward 100"}` → `{cmd, ok, resp, dt}` — any Tello SDK command, sent unchanged |
| POST | `/api/drones/{id}/emergency` | sent immediately, no reply awaited |
| POST | `/api/drones/{id}/hover` | sticks to zero (`rc 0 0 0 0`) |
| GET | `/api/drones/{id}/frame.jpg` | latest video frame |
| POST | `/api/drones/{id}/snapshot` | save JPEG into the session |
| POST | `/api/drones/{id}/record` | `{"what": "video"|"log", "on": true}` |
| GET | `/api/recordings` · `/api/recordings/{session}/{file}` · DELETE `/api/recordings/{session}` | sessions |
| GET/POST | `/api/settings` · POST `/api/settings/reset` | partial updates are merged; unknown keys are ignored |

## WebSocket `/ws`

Server → client: `{"topic": "hello", "data": {...status, settings, events}}` once, then `{"topic": "batch", "data": [{"topic", "data"}, ...]}`.
Topics: `telemetry` (every state packet), `command`, `tello`, `event`, `link`, `recording` — see ARCHITECTURE.md.

Client → server:
```json
{"type": "sticks", "drone": "tello1", "v": [lr, fb, ud, yaw]}   // -100..100 (clamped to safety.stick_max), send at 10-20 Hz
{"type": "ping", "t": 123.4}                                     // -> {"topic": "pong"}
```

## WebSocket `/ws/video/{id}`

Binary messages: 12-byte little-endian header (`uint32 frame_id`, `float64 unix_ms`) + JPEG.

## Raw video for other programs

Settings → video → `relay_udp: ["127.0.0.1:5600"]` forwards the untouched Tello H.264 packets. Open with PyAV/FFmpeg/GStreamer (`udp://127.0.0.1:5600`), ROS `ffmpeg_image_transport`, ORB-SLAM wrappers, etc.
