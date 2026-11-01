# Architecture

```
 Browser(s)  ─── HTTP/REST ───┐                          ┌── UDP 8889  commands / replies ──┐
  web/ UI    ─── WS /ws  ─────┤      pawn_gcs (Python)    ├── UDP 8890  state 10 Hz  ◄──────┤  DJI Tello
             ─── WS /ws/video ┤                          ├── UDP 11111 H.264 video ◄───────┘  (or sim/)
 Scripts / SLAM / ROS ────────┘                          └── UDP relay → SLAM node (optional)
```

## Layers

| Layer | Module | Responsibility |
|---|---|---|
| Transport | `core/tello.py` – `TelloLink` | UDP sockets, one command at a time with per-command reply timeouts, fire-and-forget `rc`, state parsing, raw video packets, unsolicited replies |
| Video | `core/video.py` | PyAV H.264 decoder thread → `FrameStore` (latest frame only), optional raw UDP relay |
| Drone node | `core/node.py` – `DroneNode` | One per drone: connect/stream, pilot sticks (clamped, dead-man), rc loop + keepalive, link status |
| Recording | `core/recorder.py` | Session folders: raw telemetry CSV, events JSONL, MP4, snapshots |
| Bus | `core/bus.py` | Thread-safe pub/sub; the WebSocket forwards every topic to the browser |
| API | `server.py`, `app.py` | FastAPI REST + WebSockets, static UI, wiring |
| UI | `web/` | Vanilla ES modules: `core.js` (WS/REST/store), `video.js`, `control.js` (keyboard), `main.js` (shell, state table, log), `tabs/*.js` |

## Control

Only a human commands the drone:

* Buttons / keys / console → `POST /api/drones/{id}/command` → `TelloLink.command` (reply awaited, shown in the log).
* Keyboard sticks → WebSocket `sticks` at 20 Hz → clamped to `safety.stick_max` → `rc a b c d`.
* No key input for `safety.deadman_ms` → sticks 0. No stick movement while the motors spin → `rc 0 0 0 0` every 0.5 s (keeps the Tello from its 15 s no-command landing).
* Motors off (the drone's `time` field not advancing): no rc at all, a quiet `command` after 5 s of silence. The drone gets `takeoff` with no rc traffic around it.
* While a discrete command (`takeoff`, `forward 100`) waits for its reply, rc is paused unless a stick is moved.
* Emergency is sent raw (bypasses the command queue) to every drone.

## Bus topics

| Topic | Payload |
|---|---|
| `telemetry` | `{drone, state: {field: value}, raw: "pitch:0;...", rc: [lr, fb, ud, yw], t}` — every state packet |
| `command` | `{drone, cmd, ok, resp, dt, t}` — every command and the drone's reply (`rc` is not forwarded to the UI) |
| `tello` | `{drone, msg, t}` — text the drone sent while no command was waiting (late replies, `error Auto land` …) |
| `event` | `{drone, level, msg, src}` — GCS events (connect, logging, link) |
| `link` | `{drone, connected, link_alive, ...status}` — 1 Hz |
| `recording` | `{drone, logging, video, session, frames}` |

## Adding a module

The Map, Mission, Vision and Autonomy tabs are empty slots.

**UI** — create `web/js/tabs/<name>.js`, render into `#pane-<name>` (replace the `.empty` placeholder), and add `import './tabs/<name>.js';` in `main.js`:

```js
import { $, on, h, store, api, sendCmd } from '../core.js';
on('telemetry', (d) => { /* d.state, d.raw, d.rc */ });
$('#pane-vision').replaceChildren(h('div', {}, 'my module'));
```

**Backend** — subscribe to the bus and/or decoded frames in `app.py`, expose endpoints in `server.py`:

```python
node.frame_listeners.append(lambda node, frame: ...)   # frame.img = BGR numpy array (decoder thread!)
gcs.bus.subscribe("telemetry", lambda topic, d: ...)
gcs.bus.publish("my_topic", {...})                       # reaches every browser via /ws
```

**Anything that sends commands** must go through `DroneNode.cmd` / `set_pilot_sticks`, so the log and the stick limit keep applying.

**External processes** (SLAM, ROS): use the REST/WS API, or `video.relay_udp` for the untouched H.264 stream.

## Threads

Event loop (asyncio): UDP I/O, rc loops, API. Separate threads: H.264 decode (per drone), video file writer, simulator video encoder. Cross-thread communication only through `FrameStore` and `bus.publish` (thread-safe).
