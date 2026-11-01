"""Recording: raw telemetry CSV, event/command JSONL, MP4 video, snapshots.

Each session is a folder ``recordings/<YYYYmmdd-HHMMSS>_<drone>/``:
    meta.json          drone, start/end time, SDK info
    telemetry.csv      every state packet as received: parsed fields + the raw string + rc sent by the GCS
    events.jsonl       events, commands (with reply + RTT), unsolicited drone messages
    udp.log            every datagram on the command port, both directions, rc and keepalives included
    video.mp4          FPV video (while video recording is on)
    snapshots/*.jpg
The web UI lists, downloads and replays sessions (Flight review tab).
"""
from __future__ import annotations

import csv
import json
import logging
import queue
import shutil
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING

import cv2

from .tello import STATE_FIELDS

if TYPE_CHECKING:
    from .node import DroneNode

log = logging.getLogger("pawn.rec")

TELEM_COLS = ["t", *STATE_FIELDS, "rc_lr", "rc_fb", "rc_ud", "rc_yw", "raw"]


class _VideoWriter(threading.Thread):
    def __init__(self, path: Path) -> None:
        super().__init__(daemon=True, name="vidwriter")
        self.path = path
        self.q: queue.Queue = queue.Queue(maxsize=120)
        self.frames = 0
        self._w = None
        self._t0 = None

    def _open(self, w: int, h: int) -> None:
        """Prefer H.264 via PyAV (plays in every browser); fall back to OpenCV mp4v."""
        try:
            import av
            self._c = av.open(str(self.path), "w")
            self._s = self._c.add_stream("libx264", rate=30)
            self._s.width, self._s.height, self._s.pix_fmt = w, h, "yuv420p"
            self._s.options = {"preset": "veryfast", "crf": "23"}
            self._av = av
            self._w = "av"
        except Exception as e:
            log.info("PyAV H.264 writer unavailable (%s) - using OpenCV mp4v", e)
            self._w = cv2.VideoWriter(str(self.path), cv2.VideoWriter_fourcc(*"mp4v"), 30.0, (w, h))

    def _write(self, img) -> None:
        if self._w == "av":
            fr = self._av.VideoFrame.from_ndarray(img, format="bgr24")
            for pkt in self._s.encode(fr):
                self._c.mux(pkt)
        else:
            self._w.write(img)

    def run(self) -> None:
        while True:
            item = self.q.get()
            if item is None:
                break
            img, t = item
            if self._w is None:
                h, w = img.shape[:2]
                self._open(w, h)
                self._t0 = t
            # keep wall-clock timing at 30 fps: duplicate/skip frames as needed
            target = int((t - self._t0) * 30) + 1
            while self.frames < target:
                self._write(img)
                self.frames += 1
        if self._w == "av":
            for pkt in self._s.encode():
                self._c.mux(pkt)
            self._c.close()
        elif self._w is not None:
            self._w.release()


class Recorder:
    def __init__(self, node: "DroneNode", root: Path, settings) -> None:
        self.node = node
        self.root = root
        self.settings = settings
        self.session: Path | None = None
        self._csv = None
        self._csv_f = None
        self._ev_f = None
        self._wire_f = None
        self._t0 = 0.0
        self._vw: _VideoWriter | None = None
        self._meta: dict = {}
        self._lock = threading.Lock()
        self._finishers: list[threading.Thread] = []
        self._was_connected = False
        b = node.bus
        b.subscribe("telemetry", self._on_telem)
        for topic in ("event", "command", "tello"):
            b.subscribe(topic, self._on_event)
        b.subscribe("link", self._on_link)
        node.frame_listeners.append(self._on_frame)
        node.link.on_wire = self._on_wire

    # ------------------------------------------------------------- sessions
    def status(self) -> dict:
        return {"logging": self.session is not None, "video": self._vw is not None,
                "session": self.session.name if self.session else None,
                "frames": self._vw.frames if self._vw else 0,
                "elapsed": round(time.time() - self._t0, 1) if self.session else 0}

    def _pub(self) -> None:
        self.node.bus.publish("recording", {"drone": self.node.id, **self.status()})

    def start_log(self) -> str:
        with self._lock:
            if self.session:
                return self.session.name
            name = datetime.now().strftime("%Y%m%d-%H%M%S") + f"_{self.node.id}"
            self.session = self.root / name
            (self.session / "snapshots").mkdir(parents=True, exist_ok=True)
            self._csv_f = open(self.session / "telemetry.csv", "w", newline="", encoding="utf-8")
            self._csv = csv.writer(self._csv_f)
            self._csv.writerow(TELEM_COLS)
            self._ev_f = open(self.session / "events.jsonl", "w", encoding="utf-8")
            self._wire_f = open(self.session / "udp.log", "w", encoding="utf-8")
            self._t0 = time.time()
            self._meta = {"drone": self.node.id, "name": self.node.cfg.name, "start": self._t0, "info": self.node.info}
            (self.session / "meta.json").write_text(json.dumps(self._meta, indent=2), "utf-8")
        self.node.bus.event(f"Logging started: {name}", "info", self.node.id, src="recorder")
        self._pub()
        return name

    def stop_log(self) -> None:
        self.stop_video()
        with self._lock:
            if not self.session:
                return
            self._meta.update({"end": time.time(), "duration_s": round(time.time() - self._t0, 1), "info": self.node.info})
            (self.session / "meta.json").write_text(json.dumps(self._meta, indent=2), "utf-8")
            for f in (self._csv_f, self._ev_f, self._wire_f):
                f.close()
            name = self.session.name
            self.session = None
            self._csv = self._csv_f = self._ev_f = self._wire_f = None
        self.node.bus.event(f"Logging stopped: {name}", "info", self.node.id, src="recorder")
        self._pub()

    def start_video(self) -> None:
        if self._vw:
            return
        if not self.session:
            self.start_log()
        self._vw = _VideoWriter(self.session / "video.mp4")
        self._vw.start()
        self.node.bus.event("Video recording started", "info", self.node.id, src="recorder")
        self._pub()

    def stop_video(self) -> None:
        vw, self._vw = self._vw, None
        if vw:
            vw.q.put(None)
            self._pub()

            def _finish():  # flush the encoder off the event loop
                vw.join(timeout=30)
                self.node.bus.event(f"Video saved ({vw.frames} frames)", "success", self.node.id, src="recorder")
                self._pub()
            t = threading.Thread(target=_finish, daemon=True)
            self._finishers.append(t)
            t.start()

    def flush(self, timeout: float = 30) -> None:
        """Wait for pending video files to be finalised (called on shutdown)."""
        for t in self._finishers:
            t.join(timeout)
        self._finishers.clear()

    def snapshot(self) -> str | None:
        fr = self.node.video.store.latest()
        if fr is None:
            self.node.bus.event("Snapshot failed: no video", "warn", self.node.id, src="recorder")
            return None
        folder = (self.session / "snapshots") if self.session else (self.root / "snapshots")
        folder.mkdir(parents=True, exist_ok=True)
        name = datetime.now().strftime("%H%M%S_%f")[:-3] + ".jpg"
        cv2.imwrite(str(folder / name), fr.img, [cv2.IMWRITE_JPEG_QUALITY, 95])
        self.node.bus.event(f"Snapshot saved: {name}", "success", self.node.id, src="recorder")
        return name

    # ------------------------------------------------------------- handlers
    def _on_link(self, topic: str, d: dict) -> None:
        """Start the log when a connection is made, stop it on disconnect (LOG button still works in between)."""
        if d.get("drone") != self.node.id:
            return
        was, self._was_connected = self._was_connected, d["connected"]
        if not self.settings.get("recording.log_on_connect", True) or was == d["connected"]:
            return
        if d["connected"] and not self.session:
            self.start_log()
        elif not d["connected"] and self.session:
            self.stop_log()

    def _on_telem(self, topic: str, d: dict) -> None:
        if d.get("drone") != self.node.id or not self._csv:
            return
        st = d["state"]
        row = [f"{d['t']:.3f}", *(st.get(k, "") for k in STATE_FIELDS), *d["rc"], d["raw"]]
        with self._lock:
            if self._csv:
                self._csv.writerow(row)

    def _on_event(self, topic: str, d: dict) -> None:
        if d.get("drone") not in (self.node.id, None) or not self._ev_f:
            return
        if topic == "command" and d.get("cmd", "").startswith("rc "):
            return
        self._write_ev({"type": topic, **d})

    def _write_ev(self, obj: dict) -> None:
        with self._lock:
            if self._ev_f:
                obj.setdefault("t", time.time())
                self._ev_f.write(json.dumps(obj, default=str) + "\n")
                self._ev_f.flush()

    def _on_wire(self, direction: str, text: str) -> None:
        with self._lock:
            if self._wire_f:
                self._wire_f.write(f"{time.time():.3f} {direction} {text}\n")

    def _on_frame(self, node, fr) -> None:
        vw = self._vw
        if vw:
            try:
                vw.q.put_nowait((fr.img, fr.t))
            except queue.Full:
                pass


class SessionIndex:
    def __init__(self, root: Path) -> None:
        self.root = root
        root.mkdir(parents=True, exist_ok=True)

    def list(self) -> list[dict]:
        out = []
        for p in sorted(self.root.iterdir(), reverse=True):
            if not p.is_dir() or p.name in ("snapshots", "_exports"):
                continue
            meta = {}
            if (p / "meta.json").exists():
                try:
                    meta = json.loads((p / "meta.json").read_text("utf-8"))
                except Exception:
                    pass
            files = sorted(f.name for f in p.iterdir() if f.is_file())
            snaps = sorted(f.name for f in (p / "snapshots").glob("*.jpg")) if (p / "snapshots").exists() else []
            size = sum(f.stat().st_size for f in p.rglob("*") if f.is_file())
            out.append({"id": p.name, "meta": meta, "files": files, "snapshots": snaps, "size": size,
                        "open": "end" not in meta})
        return out

    def path(self, session: str, file: str | None = None) -> Path:
        p = (self.root / Path(session).name)
        if file:
            parts = Path(file).parts
            p = p.joinpath(*[Path(x).name for x in parts])
        if not p.resolve().is_relative_to(self.root.resolve()):
            raise ValueError("bad path")
        return p

    def delete(self, session: str) -> None:
        p = self.path(session)
        if p.exists():
            shutil.rmtree(p)
