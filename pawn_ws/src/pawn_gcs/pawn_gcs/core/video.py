"""H.264 video pipeline: UDP packets -> PyAV decoder thread -> latest frame.

Consumers never block the decoder: they read ``FrameStore.latest()`` (or wait
for the next frame id), so a slow vision plugin or a slow browser can't add
latency to everyone else.
"""
from __future__ import annotations

import logging
import queue
import socket
import threading
import time
from dataclasses import dataclass
from typing import Callable

import numpy as np

log = logging.getLogger("pawn.video")

try:
    import av  # PyAV ships FFmpeg wheels for Windows/macOS/Linux
except Exception:  # pragma: no cover
    av = None


@dataclass
class Frame:
    id: int
    t: float            # time.time() when decoded
    img: np.ndarray     # BGR uint8, HxWx3


class FrameStore:
    def __init__(self) -> None:
        self._frame: Frame | None = None
        self._cond = threading.Condition()

    def put(self, frame: Frame) -> None:
        with self._cond:
            self._frame = frame
            self._cond.notify_all()

    def latest(self) -> Frame | None:
        return self._frame

    def wait_newer(self, last_id: int, timeout: float = 1.0) -> Frame | None:
        with self._cond:
            self._cond.wait_for(lambda: self._frame is not None and self._frame.id > last_id, timeout)
            f = self._frame
            return f if f and f.id > last_id else None

    def clear(self) -> None:
        with self._cond:
            self._frame = None


class VideoPipeline:
    """Feed raw UDP payloads with :meth:`feed`; decoded frames land in ``store``."""

    def __init__(self, name: str, on_frame: Callable[[Frame], None] | None = None) -> None:
        self.name = name
        self.store = FrameStore()
        self.on_frame = on_frame
        self._q: queue.Queue[bytes | None] = queue.Queue(maxsize=4096)
        self._thread: threading.Thread | None = None
        self._running = False
        self._frame_id = 0
        self.fps = 0.0
        self.width = 0
        self.height = 0
        self.decode_errors = 0
        self._relay_sock: socket.socket | None = None
        self.relay_targets: list[tuple[str, int]] = []

    # ----------------------------------------------------------- relay (SLAM)
    def set_relay(self, targets: list[str]) -> None:
        out = []
        for t in targets or []:
            try:
                host, port = t.rsplit(":", 1)
                out.append((host, int(port)))
            except ValueError:
                log.warning("bad relay target %r (want host:port)", t)
        self.relay_targets = out
        if out and self._relay_sock is None:
            self._relay_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # ------------------------------------------------------------------- feed
    def feed(self, data: bytes) -> None:
        if self._relay_sock:
            for tgt in self.relay_targets:
                try:
                    self._relay_sock.sendto(data, tgt)
                except OSError:
                    pass
        try:
            self._q.put_nowait(data)
        except queue.Full:  # decoder can't keep up: drop oldest
            try:
                self._q.get_nowait()
                self._q.put_nowait(data)
            except queue.Empty:
                pass

    def start(self) -> None:
        if self._running:
            return
        if av is None:
            log.error("PyAV not installed - video disabled (pip install av)")
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, name=f"video-{self.name}", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        self._q.put(None)
        if self._thread:
            self._thread.join(timeout=2)
        self._thread = None

    def _run(self) -> None:
        codec = av.CodecContext.create("h264", "r")
        try:  # lower latency: no frame threading
            codec.thread_type = "SLICE"
        except Exception:
            pass
        times: list[float] = []
        while self._running:
            try:
                data = self._q.get(timeout=0.5)
            except queue.Empty:
                continue
            if data is None:
                break
            try:
                packets = codec.parse(data)
            except Exception:
                self.decode_errors += 1
                continue
            for pkt in packets:
                try:
                    frames = codec.decode(pkt)
                except Exception:
                    self.decode_errors += 1
                    continue
                for vf in frames:
                    try:
                        img = vf.to_ndarray(format="bgr24")
                    except Exception:
                        self.decode_errors += 1
                        continue
                    self._frame_id += 1
                    now = time.time()
                    self.height, self.width = img.shape[:2]
                    times.append(now)
                    times = [t for t in times if now - t < 1.0]
                    self.fps = float(len(times))
                    fr = Frame(self._frame_id, now, img)
                    self.store.put(fr)
                    if self.on_frame:
                        try:
                            self.on_frame(fr)
                        except Exception:
                            log.exception("on_frame failed")
        log.info("video thread %s stopped", self.name)
