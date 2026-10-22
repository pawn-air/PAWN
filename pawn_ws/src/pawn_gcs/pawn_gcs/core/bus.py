"""Tiny async publish/subscribe event bus.

Every subsystem (driver, safety, missions, plugins, recorder, web server)
talks through this bus, so new modules can be added without touching the
existing ones. Publishing is thread-safe: worker threads (video decoder,
vision plugins) can publish too.

Topics used across the app (payloads are plain dicts, JSON-serialisable):
    telemetry      {"drone": id, "state": {...}, "odom": {...}}
    event          {"drone": id|None, "level": "info|warn|error|success", "msg": str, "src": str}
    command        {"drone": id, "cmd": str, "resp": str|None, "ok": bool, "dt": float}
    link           {"drone": id, "connected": bool, ...}
    detections     {"drone": id, "plugin": name, "items": [...], "frame_id": int}
    mission        {"drone": id, "status": ..., "step": int, ...}
    mode           {"drone": id, "mode": "MANUAL|MISSION|AUTO"}
    map            {"type": "points|marker|pose", ...}
    recording      {"drone": id, "video": bool, "session": str}
"""
from __future__ import annotations

import asyncio
import fnmatch
import logging
import threading
from collections import defaultdict
from typing import Any, Awaitable, Callable, Optional, Union

log = logging.getLogger("pawn.bus")

Handler = Callable[[str, dict], Union[Awaitable[None], None]]


class EventBus:
    def __init__(self) -> None:
        self._subs: dict[str, list[Handler]] = defaultdict(list)
        self._loop: asyncio.AbstractEventLoop | None = None
        self._lock = threading.Lock()

    def bind_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop

    def subscribe(self, pattern: str, handler: Handler) -> Callable[[], None]:
        """Subscribe to a topic pattern (fnmatch wildcards, e.g. ``"*"``)."""
        with self._lock:
            self._subs[pattern].append(handler)

        def unsubscribe() -> None:
            with self._lock:
                if handler in self._subs.get(pattern, []):
                    self._subs[pattern].remove(handler)

        return unsubscribe

    def publish(self, topic: str, data: dict[str, Any]) -> None:
        """Publish from any thread."""
        loop = self._loop
        if loop is None:
            return
        try:
            running = asyncio.get_running_loop()
        except RuntimeError:
            running = None
        if running is loop:
            self._dispatch(topic, data)
        else:
            try:
                loop.call_soon_threadsafe(self._dispatch, topic, data)
            except RuntimeError:  # loop closed during shutdown
                pass

    def _dispatch(self, topic: str, data: dict) -> None:
        with self._lock:
            targets = [h for p, hs in self._subs.items() if fnmatch.fnmatchcase(topic, p) for h in hs]
        for h in targets:
            try:
                res = h(topic, data)
                if asyncio.iscoroutine(res):
                    asyncio.ensure_future(res)
            except Exception:  # never let one bad subscriber break the bus
                log.exception("bus handler failed for %s", topic)

    # convenience
    def event(self, msg: str, level: str = "info", drone: str | None = None, src: str = "gcs") -> None:
        getattr(log, {"warn": "warning", "success": "info"}.get(level, level), log.info)(f"[{drone or '-'}] {msg}")
        self.publish("event", {"drone": drone, "level": level, "msg": msg, "src": src})
