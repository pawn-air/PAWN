"""HTTP + WebSocket API (FastAPI). Interactive docs at /docs.

WebSockets
  /ws                 JSON: server pushes bus topics; client sends sticks / cmd / ping
  /ws/video/{drone}   binary: 12-byte header (uint32 frame_id, float64 unix_ms, LE) + JPEG
"""
from __future__ import annotations

import asyncio
import json
import struct
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Optional

from fastapi import Body, FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles

from .app import VERSION, GcsApp
from .config import AppConfig
from .core import patterns
from .core.mission import MissionError, preview, validate
from .core.report import build_report
from .core.tello import STATE_FIELDS

WEB = Path(__file__).resolve().parent / "web"


def create_app(cfg: AppConfig) -> FastAPI:
    gcs = GcsApp(cfg)

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        await gcs.start()
        yield
        await gcs.stop()

    api = FastAPI(title="PAWN Ground Control", version=VERSION, lifespan=lifespan,
                  description="Ground control station API for DJI Tello (PAWN project).")
    api.state.gcs = gcs

    def node(drone: str):
        try:
            return gcs.node(drone)
        except KeyError:
            raise HTTPException(404, f"unknown drone {drone}")

    # ------------------------------------------------------------- status
    @api.get("/api/status")
    def status():
        return gcs.status()

    @api.get("/api/meta")
    def meta():
        return {"state_fields": {k: {"unit": u, "label": l} for k, (u, l) in STATE_FIELDS.items()},
                "version": VERSION, "sim": cfg.sim}

    @api.get("/api/events")
    def events(limit: int = 300):
        return list(gcs.events)[-limit:]

    # ------------------------------------------------------------- drones
    @api.post("/api/drones/{drone}/connect")
    async def connect(drone: str):
        ok = await node(drone).connect()
        return {"ok": ok, "status": node(drone).status()}

    @api.post("/api/drones/{drone}/disconnect")
    async def disconnect(drone: str):
        await node(drone).disconnect()
        return {"ok": True}

    @api.post("/api/drones/{drone}/command")
    async def command(drone: str, body: dict = Body(...)):
        text = str(body.get("text", "")).strip()
        if not text:
            raise HTTPException(400, "empty command")
        r = await node(drone).cmd(text, source=body.get("source", "api"))
        return r.as_dict()

    @api.post("/api/drones/{drone}/emergency")
    async def emergency(drone: str):
        return (await node(drone).cmd("emergency")).as_dict()

    @api.post("/api/drones/{drone}/hover")
    async def hover(drone: str):
        await node(drone).hover("operator")
        return {"ok": True}

    @api.post("/api/drones/{drone}/mode")
    async def set_mode(drone: str, body: dict = Body(...)):
        n = node(drone)
        n.set_mode(str(body.get("mode", "MANUAL")), reason="operator")
        return {"mode": n.mode}

    @api.get("/api/drones/{drone}/preflight")
    def preflight(drone: str):
        n = node(drone)
        return {"checks": n.safety.preflight(), "blockers": n.safety.takeoff_blockers()}

    @api.post("/api/drones/{drone}/pose")
    def ext_pose(drone: str, body: dict = Body(...)):
        """External pose (SLAM/VIO): {x, y, z, yaw} in metres / degrees, GCS map frame."""
        node(drone).odom.set_external(body)
        return {"ok": True}

    @api.post("/api/drones/{drone}/home")
    async def reset_home(drone: str):
        n = node(drone)
        n.odom.set_home(n.state.get("yaw"))
        gcs.bus.event("Home / odometry reset", "info", drone)
        return {"ok": True}

    @api.get("/api/drones/{drone}/track")
    def track(drone: str):
        return {"track": list(node(drone).odom.track)}

    @api.get("/api/drones/{drone}/frame.jpg")
    def frame(drone: str):
        n = node(drone)
        fr = n.video.store.latest()
        if fr is None:
            raise HTTPException(404, "no video")
        return Response(gcs.jpeg(n, fr), media_type="image/jpeg")

    @api.post("/api/drones/{drone}/snapshot")
    async def snapshot(drone: str):
        name = node(drone).recorder.snapshot()
        if not name:
            raise HTTPException(409, "no video frame")
        return {"file": name}

    @api.post("/api/drones/{drone}/record")
    async def record(drone: str, body: dict = Body(...)):
        r = node(drone).recorder
        what, on = body.get("what", "video"), bool(body.get("on", True))
        if what == "video":
            r.start_video() if on else r.stop_video()
        else:
            r.start_log() if on else r.stop_log()
        return r.status()

    # ------------------------------------------------------------- navigation / SAR
    @api.post("/api/drones/{drone}/rth")
    async def rth(drone: str, body: dict = Body(default={})):
        """Return home along the breadcrumb trail (rotate + forward legs), then land."""
        try:
            m = node(drone).nav.rth(land=bool(body.get("land", True)))
        except (RuntimeError, MissionError) as e:
            raise HTTPException(409, str(e))
        return {"ok": True, "steps": len(m["steps"]), "mission": m}

    @api.get("/api/drones/{drone}/rth")
    def rth_estimate(drone: str):
        n = node(drone)
        return {**n.nav.estimate(), "path": n.nav.path_home()}

    @api.post("/api/drones/{drone}/goto_target/{tid}")
    async def goto_target(drone: str, tid: int):
        try:
            m = node(drone).nav.goto_target(tid)
        except KeyError:
            raise HTTPException(404, "unknown target")
        except (RuntimeError, MissionError) as e:
            raise HTTPException(409, str(e))
        return {"ok": True, "steps": len(m["steps"])}

    @api.get("/api/targets/{tid}/backtrack")
    def backtrack(tid: int, drone: Optional[str] = None):
        """Breadcrumb path from where the target was first seen back to home (rescue route)."""
        m = gcs.map.target(tid)
        if not m:
            raise HTTPException(404, "unknown target")
        n = node(drone or m.get("drone") or next(iter(gcs.nodes)))
        from .core.geometry import path_length
        path = n.nav.backtrack_path(tid)
        return {"target": m, "path": path, "length_m": round(path_length(path), 2)}

    @api.post("/api/drones/{drone}/mark")
    async def mark(drone: str, body: dict = Body(...)):
        """Operator marks a target on the video at normalised (u, v). Two marks from different spots triangulate."""
        n = node(drone)
        from .core.geometry import Camera
        cam = Camera.from_settings(gcs.settings, n.video.width or 960, n.video.height or 720)
        det = {"center": [float(body["u"]), float(body["v"])], "conf": 1.0}
        m = gcs.map.add_observation(drone, str(body.get("label") or "manual"), n.odom.pose(), det, "operator",
                                    source="manual", cam=cam, force=True)
        gcs.bus.event(f"Operator mark '{m['label']}' #{m['id']} ({m['hits']} rays, {m['method']})", "info", drone, src="operator")
        return m

    @api.post("/api/drones/{drone}/bookmark")
    async def bookmark(drone: str, body: dict = Body(default={})):
        n = node(drone)
        p = n.odom.pose()
        note = str(body.get("note") or "bookmark")
        m = gcs.map.add_marker({"label": note, "source": "bookmark", "drone": drone, "x": p["x"], "y": p["y"],
                                "z": p["z"], "bearing": p["yaw"], "note": note})
        gcs.bus.event(f"Bookmark '{note}' at ({p['x']:.1f}, {p['y']:.1f}, {p['z']:.1f})", "info", drone, src="operator")
        if n.recorder:
            n.recorder.snapshot(tag="bookmark")
        return m

    @api.get("/api/coverage")
    def coverage():
        return gcs.coverage.snapshot()

    @api.get("/api/patterns")
    def patterns_list():
        return patterns.describe()

    @api.post("/api/patterns/{name}")
    def patterns_gen(name: str, body: dict = Body(default={})):
        try:
            m = patterns.generate(name, body)
            validate(m)
        except (ValueError, MissionError) as e:
            raise HTTPException(400, str(e))
        return m

    # ------------------------------------------------------------- fleet
    @api.post("/api/fleet/command")
    async def fleet_command(body: dict = Body(...)):
        """Send the same command to several drones at once (tandem takeoff/land etc.)."""
        text = str(body.get("text", "")).strip()
        ids = body.get("drones") or list(gcs.nodes)
        res = await asyncio.gather(*(node(i).cmd(text, source="fleet") for i in ids))
        return {i: r.as_dict() for i, r in zip(ids, res)}

    # ------------------------------------------------------------- missions
    @api.get("/api/missions")
    def missions():
        return gcs.missions.list()

    @api.get("/api/missions/{file}")
    def mission_get(file: str):
        try:
            return gcs.missions.load(file)
        except FileNotFoundError:
            raise HTTPException(404, "not found")

    @api.post("/api/missions")
    def mission_save(body: dict = Body(...)):
        try:
            return {"file": gcs.missions.save(body)}
        except MissionError as e:
            raise HTTPException(400, str(e))

    @api.delete("/api/missions/{file}")
    def mission_delete(file: str):
        gcs.missions.delete(file)
        return {"ok": True}

    @api.post("/api/missions/preview")
    def mission_preview(body: dict = Body(...)):
        try:
            validate(body)
            err = None
        except MissionError as e:
            err = str(e)
        p = preview(body)
        p["error"] = err
        fence = gcs.settings.get("safety.geofence_radius_m", 8.0), gcs.settings.get("safety.geofence_max_alt_m", 3.0)
        p["fence_ok"] = p["max_range_m"] <= fence[0] and p["max_alt_m"] <= fence[1]
        return p

    @api.post("/api/drones/{drone}/mission/{action}")
    async def mission_action(drone: str, action: str, body: dict = Body(default={})):
        ex = node(drone).mission
        try:
            if action == "start":
                ex.start(body)
            elif action == "pause":
                ex.pause()
            elif action == "resume":
                ex.resume()
            elif action == "abort":
                ex.abort("operator")
            else:
                raise HTTPException(404, "unknown action")
        except MissionError as e:
            raise HTTPException(400, str(e))
        return ex.status()

    # ------------------------------------------------------------- vision / autonomy
    @api.get("/api/vision")
    def vision():
        return gcs.vision.describe()

    @api.post("/api/vision/{name}/enable")
    async def vision_enable(name: str, body: dict = Body(...)):
        try:
            gcs.vision.set_enabled(body["drone"], name, bool(body.get("on", True)))
        except KeyError:
            raise HTTPException(404, "unknown plugin/drone")
        return {"ok": True}

    @api.post("/api/vision/{name}/params")
    def vision_params(name: str, body: dict = Body(...)):
        try:
            return gcs.vision.set_params(name, body)
        except KeyError:
            raise HTTPException(404, "unknown plugin")

    @api.get("/api/autonomy")
    def autonomy():
        return gcs.autonomy.describe()

    @api.post("/api/drones/{drone}/autonomy")
    async def autonomy_select(drone: str, body: dict = Body(...)):
        node(drone)
        try:
            gcs.autonomy.select(drone, body.get("name"), body.get("values"))
        except KeyError:
            raise HTTPException(404, "unknown controller")
        if body.get("engage"):
            node(drone).set_mode("AUTO", reason="operator")
        return gcs.autonomy.describe()

    # ------------------------------------------------------------- map
    @api.get("/api/map")
    def map_get():
        return {**gcs.map.snapshot(), "tracks": {i: list(n.odom.track) for i, n in gcs.nodes.items()},
                "coverage": gcs.coverage.snapshot()}

    @api.post("/api/map/markers")
    async def map_marker(body: dict = Body(...)):
        return gcs.map.add_marker(body)

    @api.delete("/api/map/markers/{mid}")
    async def map_marker_del(mid: int):
        gcs.map.remove_marker(mid)
        return {"ok": True}

    @api.post("/api/map/clear")
    async def map_clear():
        gcs.map.clear()
        gcs.coverage.clear()
        for n in gcs.nodes.values():
            n.odom.track.clear()
        return {"ok": True}

    @api.post("/api/map/clouds/{name}")
    async def map_cloud(name: str, body: dict = Body(...)):
        """Push a point cloud (e.g. sparse SLAM map): {points: [[x,y,z],...], color}."""
        gcs.map.set_cloud(name, body.get("points", []), body.get("color", "#94a3b8"))
        return {"ok": True, "n": len(body.get("points", []))}

    # ------------------------------------------------------------- recordings
    @api.get("/api/recordings")
    def recordings():
        return gcs.sessions.list()

    @api.get("/api/recordings/{sid}/report.html", response_class=HTMLResponse)
    def recording_report(sid: str):
        try:
            p = gcs.sessions.path(sid)
        except ValueError:
            raise HTTPException(400, "bad path")
        if not p.is_dir():
            raise HTTPException(404, "not found")
        return HTMLResponse(build_report(p))

    @api.get("/api/recordings/{sid}/export.zip")
    async def recording_zip(sid: str, what: str = "all"):
        """Whole session as a zip (what=all) or only the 3D dataset (what=dataset)."""
        try:
            p = gcs.sessions.path(sid)
        except ValueError:
            raise HTTPException(400, "bad path")
        if not p.is_dir():
            raise HTTPException(404, "not found")
        out = await asyncio.to_thread(gcs.sessions.export_zip, p.name, what)
        return FileResponse(out, filename=out.name, media_type="application/zip")

    @api.get("/api/recordings/{sid}/{file:path}")
    def recording_file(sid: str, file: str):
        try:
            p = gcs.sessions.path(sid, file)
        except ValueError:
            raise HTTPException(400, "bad path")
        if not p.is_file():
            raise HTTPException(404, "not found")
        return FileResponse(p)

    @api.delete("/api/recordings/{sid}")
    def recording_delete(sid: str):
        gcs.sessions.delete(sid)
        return {"ok": True}

    # ------------------------------------------------------------- settings
    @api.get("/api/settings")
    def settings_get():
        return gcs.settings.all()

    @api.post("/api/settings")
    async def settings_set(body: dict = Body(...)):
        s = gcs.settings.update(body)
        if "video" in body:
            for n in gcs.nodes.values():
                n.video.set_relay(gcs.settings.get("video.relay_udp", []))
        gcs.bus.event("Settings updated", "info")
        return s

    @api.post("/api/settings/reset")
    def settings_reset():
        return gcs.settings.reset()

    # ------------------------------------------------------------- websockets
    @api.websocket("/ws")
    async def ws_main(ws: WebSocket):
        await ws.accept()
        gcs.clients += 1
        q: asyncio.Queue = asyncio.Queue(maxsize=2000)
        last_tel: dict[str, float] = {}

        def on_any(topic: str, data: dict):
            if topic == "telemetry":  # throttle to 15 Hz per drone
                now = time.monotonic()
                if now - last_tel.get(data["drone"], 0) < 0.066:
                    return
                last_tel[data["drone"]] = now
            if topic == "command" and data.get("cmd", "").startswith("rc "):
                return
            try:
                q.put_nowait({"topic": topic, "data": data})
            except asyncio.QueueFull:
                pass

        unsub = gcs.bus.subscribe("*", on_any)
        await ws.send_text(json.dumps({"topic": "hello", "data": {**gcs.status(), "settings": gcs.settings.all(),
                                                                    "events": list(gcs.events)[-200:]}}, default=str))

        async def sender():
            while True:
                msg = await q.get()
                batch = [msg]
                while not q.empty() and len(batch) < 50:
                    batch.append(q.get_nowait())
                await ws.send_text(json.dumps({"topic": "batch", "data": batch}, default=str))

        send_task = asyncio.ensure_future(sender())
        try:
            while True:
                msg = json.loads(await ws.receive_text())
                typ, drone = msg.get("type"), msg.get("drone")
                if typ == "sticks" and drone in gcs.nodes:
                    gcs.nodes[drone].set_pilot_sticks(*msg.get("v", [0, 0, 0, 0])[:4])
                elif typ == "cmd" and drone in gcs.nodes:
                    asyncio.ensure_future(gcs.nodes[drone].cmd(str(msg.get("text", "")), source="ws"))
                elif typ == "ping":
                    await ws.send_text(json.dumps({"topic": "pong", "data": {"t": msg.get("t")}}))
        except (WebSocketDisconnect, RuntimeError):
            pass
        finally:
            send_task.cancel()
            unsub()
            gcs.clients -= 1

    @api.websocket("/ws/video/{drone}")
    async def ws_video(ws: WebSocket, drone: str):
        await ws.accept()
        if drone not in gcs.nodes:
            await ws.close(code=4404)
            return
        n = gcs.nodes[drone]
        last = 0
        try:
            while True:
                fr = await asyncio.to_thread(n.video.store.wait_newer, last, 1.0)
                if fr is None:
                    continue
                min_dt = 1.0 / max(1, int(gcs.settings.get("video.max_fps", 30)))
                last = fr.id
                data = await asyncio.to_thread(gcs.jpeg, n, fr)
                await ws.send_bytes(struct.pack("<Id", fr.id, fr.t * 1000) + data)
                await asyncio.sleep(min_dt * 0.5)
        except (WebSocketDisconnect, RuntimeError, ConnectionError):
            pass

    # ------------------------------------------------------------- static UI
    @api.get("/")
    def index():
        return FileResponse(WEB / "index.html", headers={"Cache-Control": "no-cache"})

    api.mount("/", StaticFiles(directory=WEB), name="web")

    @api.exception_handler(KeyError)
    async def keyerr(_, exc):
        return JSONResponse({"detail": f"not found: {exc}"}, status_code=404)

    return api
