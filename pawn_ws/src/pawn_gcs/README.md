# PAWN Ground Control Station

Minimal web ground control station for the **DJI Tello** (PAWN, Bilkent CS491). One Python process talks to the drone(s) over UDP and serves a browser UI, on Windows, macOS and Linux.

It does three things: shows the video, lets a human fly with the keyboard, and shows/logs everything the drone sends. **The GCS never flies on its own** — no auto-land, no takeoff checks, no geofence, no autonomy. The menu keeps empty slots (Map, Mission, Vision, Autonomy) for modules to be added later.

| Area | What there is |
|---|---|
| **Video** | H.264 → PyAV decode → JPEG to the browser, fullscreen. No overlay. |
| **Control** | Takeoff / Hover / Land / **Emergency**, keyboard sticks (Mode 2), SDK console for any command |
| **Telemetry** | Every field the drone sends, as received: live table, one chart per field, raw packet stream |
| **Log** | Every command with the drone's reply and RTT, every unsolicited drone message, GCS events |
| **Safety** | Stick limit (clamped on the GCS), dead-man (sticks → 0 when key input stops), link-lost timeout |
| **Recording** | While connected: telemetry CSV (parsed fields + raw string + rc sent), events JSONL, `udp.log` (every command-port datagram); video MP4 and snapshots on demand. Flight review tab. |
| **Simulator** | Software-in-the-loop Tello (real UDP protocol, synthetic FPV video) |
| **Integration** | REST + WebSocket API (`/docs`), raw H.264 UDP relay for SLAM / ROS |

## Quick start

Requires **Python 3.9+**.

```bash
./start.sh --sim          # simulator (no drone needed)
./start.sh                # real Tello: join the TELLO-XXXX Wi-Fi first
start-sim.bat / start.bat # Windows
```

The launcher creates `.venv`, installs `requirements.txt` once, starts the server and opens **http://127.0.0.1:8080**.
Manual: `pip install -r requirements.txt` then `python -m pawn_gcs [--sim [N]] [--host 0.0.0.0] [--port 8080]`.

> **Firewall:** the Tello sends telemetry to UDP **8890** and video to UDP **11111** on your PC. See [docs/TELLO_NETWORK.md](docs/TELLO_NETWORK.md).

## Keyboard (Mode 2)

| Key | Action |
|---|---|
| `W` / `S` | up / down (throttle) |
| `A` / `D` | yaw left / right |
| `↑` / `↓` | forward / back (pitch) |
| `←` / `→` | left / right (roll) |
| `T` / `L` | takeoff / land |
| `Space` | hover (sticks to zero) |
| `Shift+Space` | **emergency motor stop** (all drones) |
| `F` / `R` | snapshot / video recording |
| `1`…`4` | select drone |

## What the GCS sends on its own

* While the motors spin (the drone's `time` state field advances): `rc 0 0 0 0` every 0.5 s when no stick is moved — a Tello lands by itself after 15 s without any command, so this is what keeps it hovering.
* While the motors are off: no `rc` packets; `command` after 5 s without traffic (keeps SDK mode; not logged).
* Stick values fall back to `0` after `safety.deadman_ms` without key input (closed tab, laptop Wi-Fi to the browser lost).
* `command`, `streamon`, `sdk?`, `sn?`, `wifi?` once, when Connect is pressed.

Nothing else. The drone's own firmware can still land or refuse commands (low battery voltage, temperature); its replies appear in the log exactly as received.

## Türkçe

1. `./start.sh --sim` ile drone'suz dene; gerçek Tello için önce `TELLO-XXXX` Wi-Fi'ına bağlan, `./start.sh`, tarayıcıda **Connect**.
2. Klavye Mode 2: W/S yükseklik · A/D yaw · ok tuşları ileri-geri / sağ-sol. `T` kalkış, `L` iniş, `Space` hover, `Shift+Space` acil durdurma.
3. GCS kendi başına hiçbir şey yapmaz: otomatik iniş, kalkış engeli, geofence yok. Stick bırakılınca drone olduğu yerde hover eder.
4. Sağ sütun ve Telemetry sekmesi drone'un gönderdiği ham veriyi gösterir; Log, her komutu drone'un cevabıyla birlikte gösterir.

## Project layout

```
pawn_gcs/
  core/     tello.py (UDP link)  video.py (H.264)  node.py (per-drone: sticks, keepalive)  recorder.py  bus.py
  sim/      Tello emulator + synthetic camera renderer
  web/      UI (vanilla ES modules, no build step, works offline on the Tello Wi-Fi)
  server.py FastAPI REST + WebSocket     app.py wiring     config.py
examples/   api_client.py
tools/      raw_takeoff.py (bare SDK takeoff test, no GCS)
docs/       ARCHITECTURE.md (how to add modules), API.md, TELLO_NETWORK.md
tests/      unit + end-to-end tests against the simulator
```

Adding a module (map, mission, vision, autonomy …): see [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Standalone app

```bash
pip install -r requirements.txt pyinstaller
python packaging/build.py        # -> dist/PAWN-GCS/ and dist/PAWN-GCS-<os>-<arch>.zip
```

## Tests

```bash
pip install pytest httpx
python -m pytest
```
