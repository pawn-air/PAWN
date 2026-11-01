"""GcsApp: wires the subsystems together. Owned by the web server's lifespan.

Add a new backend module here: give it the bus (``self.bus``) and/or a node
(``node.frame_listeners`` for video frames) and expose it in server.py.
"""
from __future__ import annotations

import asyncio
import logging
import threading
import time
from collections import deque

import cv2

from .config import AppConfig, Settings
from .core.bus import EventBus
from .core.node import DroneNode
from .core.recorder import Recorder, SessionIndex

log = logging.getLogger("pawn.app")
VERSION = "3.0.0"


class GcsApp:
    def __init__(self, cfg: AppConfig) -> None:
        self.cfg = cfg
        self.bus = EventBus()
        self.settings = Settings(cfg.data_dir / "settings.json")
        self.sessions = SessionIndex(cfg.recordings_dir)
        self.nodes: dict[str, DroneNode] = {}
        self.sims: list = []
        self.clients = 0
        self.events: deque = deque(maxlen=2000)   # everything the log panel shows, for late joiners
        self.started = time.time()
        self._jpeg: dict[str, tuple[int, bytes]] = {}
        self._jpeg_lock = threading.Lock()
        self.bus.subscribe("event", lambda t, d: self.events.append({"type": "event", **d, "t": time.time()}))
        self.bus.subscribe("tello", lambda t, d: self.events.append({"type": "tello", **d}))
        self.bus.subscribe("command", self._log_cmd)

    def _log_cmd(self, t, d):
        if not d.get("cmd", "").startswith("rc "):
            self.events.append({"type": "command", **d})

    async def start(self) -> None:
        self.bus.bind_loop(asyncio.get_running_loop())
        if self.cfg.sim:
            from .sim.tello_sim import SimTello
            for i, d in enumerate(self.cfg.drones):
                sim = SimTello(cmd_port=d.cmd_port, state_port=d.state_port, video_port=d.video_port,
                               start_xy=(0.0, 1.5 * i), name=d.id)
                await sim.start("127.0.0.1")
                self.sims.append(sim)
        for d in self.cfg.drones:
            node = DroneNode(d, self.bus, self.settings)
            node.recorder = Recorder(node, self.cfg.recordings_dir, self.settings)
            await node.start()
            self.nodes[d.id] = node
        self.bus.event(f"PAWN GCS {VERSION} ready - {len(self.nodes)} drone(s)" + (" [SIMULATION]" if self.cfg.sim else ""),
                       "success")
        if self.cfg.sim:
            for n in self.nodes.values():
                asyncio.ensure_future(n.connect())

    async def stop(self) -> None:
        for n in self.nodes.values():
            if n.recorder:
                n.recorder.stop_log()
                n.recorder.flush()
            await n.stop()
        for s in self.sims:
            await s.stop()

    def node(self, drone_id: str) -> DroneNode:
        if drone_id not in self.nodes:
            raise KeyError(drone_id)
        return self.nodes[drone_id]

    def jpeg(self, node: DroneNode, fr) -> bytes:
        """Encode a frame once, share it with every viewer."""
        with self._jpeg_lock:
            cached = self._jpeg.get(node.id)
            if cached and cached[0] == fr.id:
                return cached[1]
        q = int(self.settings.get("video.jpeg_quality", 75))
        ok, buf = cv2.imencode(".jpg", fr.img, [cv2.IMWRITE_JPEG_QUALITY, q])
        data = buf.tobytes() if ok else b""
        with self._jpeg_lock:
            self._jpeg[node.id] = (fr.id, data)
        return data

    def status(self) -> dict:
        return {"version": VERSION, "sim": self.cfg.sim, "uptime": round(time.time() - self.started),
                "clients": self.clients, "drones": [n.status() for n in self.nodes.values()]}
