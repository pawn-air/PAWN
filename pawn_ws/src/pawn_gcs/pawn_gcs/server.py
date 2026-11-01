"""HTTP + WebSocket API (FastAPI). Interactive docs at /docs.

WebSockets
  /ws                 JSON: server pushes bus topics; client sends sticks / ping
  /ws/video/{drone}   binary: 12-byte header (uint32 frame_id, float64 unix_ms, LE) + JPEG
"""
from __future__ import annotations

import asyncio
import json
import struct
import time
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import Body, FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles

from .app import VERSION, GcsApp
from .config import AppConfig
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
    def events(limit: int = 500):
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
        node(drone).hover()
        return {"ok": True}

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

    # ------------------------------------------------------------- recordings
    @api.get("/api/recordings")
    def recordings():
        return gcs.sessions.list()

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
        gcs.bus.event("Settings updated: " + json.dumps(body, separators=(",", ":")), "info")
        return s

    @api.post("/api/settings/reset")
    def settings_reset():
        return gcs.settings.reset()

    # ------------------------------------------------------------- websockets
    @api.websocket("/ws")
    async def ws_main(ws: WebSocket):
        await ws.accept()
        gcs.clients += 1
        q: asyncio.Queue = asyncio.Queue(maxsize=4000)

        def on_any(topic: str, data: dict):
            if topic == "command" and data.get("cmd", "").startswith("rc "):
                return
            try:
                q.put_nowait({"topic": topic, "data": data})
            except asyncio.QueueFull:
                pass

        unsub = gcs.bus.subscribe("*", on_any)
        await ws.send_text(json.dumps({"topic": "hello", "data": {**gcs.status(), "settings": gcs.settings.all(),
                                                                    "events": list(gcs.events)[-500:]}}, default=str))

        async def sender():
            while True:
                msg = await q.get()
                batch = [msg]
                while not q.empty() and len(batch) < 100:
                    batch.append(q.get_nowait())
                await ws.send_text(json.dumps({"topic": "batch", "data": batch}, default=str))

        send_task = asyncio.ensure_future(sender())
        try:
            while True:
                msg = json.loads(await ws.receive_text())
                typ, drone = msg.get("type"), msg.get("drone")
                if typ == "sticks" and drone in gcs.nodes:
                    gcs.nodes[drone].set_pilot_sticks(*msg.get("v", [0, 0, 0, 0])[:4])
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
    # The JS/CSS are served under a per-start URL prefix, so a browser can never keep running
    # modules cached from an older version (ES-module imports ignore a normal reload in some browsers).
    ui = f"/ui-{int(time.time())}"

    @api.get("/")
    def index():
        html = (WEB / "index.html").read_text("utf-8")
        html = html.replace('href="/css/', f'href="{ui}/css/').replace('src="/js/', f'src="{ui}/js/')
        return HTMLResponse(html, headers={"Cache-Control": "no-cache"})

    api.mount(ui, StaticFiles(directory=WEB), name="ui")
    api.mount("/", StaticFiles(directory=WEB), name="web")

    @api.exception_handler(KeyError)
    async def keyerr(_, exc):
        return JSONResponse({"detail": f"not found: {exc}"}, status_code=404)

    return api
