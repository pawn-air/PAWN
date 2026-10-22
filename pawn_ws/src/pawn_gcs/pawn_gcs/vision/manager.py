"""Discovers vision plugins and runs the enabled ones on each drone's video.

One worker thread per drone pulls the *latest* frame (never a backlog) and
runs every enabled plugin that is due according to its ``rate_hz``.
"""
from __future__ import annotations

import importlib
import importlib.util
import inspect
import json
import logging
import pkgutil
import threading
import time
from pathlib import Path
from typing import TYPE_CHECKING

from .base import VisionPlugin
from ..core.geometry import Camera

if TYPE_CHECKING:
    from ..core.node import DroneNode

log = logging.getLogger("pawn.vision")


def discover(extra_dirs: list[Path]) -> dict[str, type[VisionPlugin]]:
    found: dict[str, type[VisionPlugin]] = {}
    mods = []
    from . import plugins as builtin_pkg
    names = {m.name for m in pkgutil.iter_modules(builtin_pkg.__path__)} | {"builtin"}  # frozen builds may not list
    for name in sorted(names):
        try:
            mods.append(importlib.import_module(f"{builtin_pkg.__name__}.{name}"))
        except Exception as e:
            log.warning("plugin module %s failed: %s", name, e)
    for d in extra_dirs:
        if not d.exists():
            continue
        for f in sorted(d.glob("*.py")):
            if f.name.startswith("_"):
                continue
            try:
                spec = importlib.util.spec_from_file_location(f"pawn_user_plugins.{f.stem}", f)
                mod = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(mod)
                mods.append(mod)
            except Exception as e:
                log.warning("user plugin %s failed: %s", f, e)
    for mod in mods:
        for _, obj in inspect.getmembers(mod, inspect.isclass):
            if issubclass(obj, VisionPlugin) and obj is not VisionPlugin and obj.name != "base":
                found[obj.name] = obj
    return found


class _Runner(threading.Thread):
    def __init__(self, mgr: "VisionManager", node: "DroneNode") -> None:
        super().__init__(daemon=True, name=f"vision-{node.id}")
        self.mgr, self.node = mgr, node
        self.instances: dict[str, VisionPlugin] = {}
        self.last_run: dict[str, float] = {}
        self.running = True

    def run(self) -> None:
        last_id = 0
        while self.running:
            enabled = self.mgr.enabled.get(self.node.id, set())
            if not enabled:
                time.sleep(0.2)
                continue
            fr = self.node.video.store.wait_newer(last_id, timeout=0.5)
            if fr is None:
                continue
            last_id = fr.id
            ctx = {"drone": self.node.id, "state": dict(self.node.state), "pose": self.node.odom.pose(),
                   "frame_id": fr.id, "t": fr.t}
            for name in list(enabled):
                cls = self.mgr.classes.get(name)
                if cls is None:
                    continue
                if time.time() - self.last_run.get(name, 0) < 1.0 / max(cls.rate_hz, 0.1):
                    continue
                inst = self.instances.get(name)
                if inst is None:
                    inst = cls(self.mgr.params.get(name))
                    self.instances[name] = inst
                    if not inst.available:
                        self.node.bus.event(f"Vision plugin '{name}' unavailable: {inst.unavailable_reason}",
                                            "warn", self.node.id, src="vision")
                if not inst.available:
                    continue
                self.last_run[name] = time.time()
                t0 = time.perf_counter()
                try:
                    items = inst.process(fr.img, ctx)
                except Exception as e:
                    log.exception("plugin %s failed", name)
                    self.node.bus.event(f"Plugin '{name}' error: {e}", "error", self.node.id, src="vision")
                    self.mgr.set_enabled(self.node.id, name, False)
                    continue
                ms = (time.perf_counter() - t0) * 1000
                det = {"drone": self.node.id, "plugin": name, "items": items, "t": time.time(), "ms": round(ms, 1),
                       "frame_id": fr.id, "color": cls.color}
                self.node.detections[name] = det
                self.node.bus.publish("detections", det)
                if cls.is_target and self.node.flying:
                    H, W = fr.img.shape[:2]
                    cam = Camera.from_settings(self.node.settings, W, H)
                    min_conf = float(self.node.settings.get("targets.min_conf", 0.3))
                    for it in items:
                        if it.get("center") and it.get("conf", 0) >= min_conf:
                            self.mgr.map.add_observation(self.node.id, it["label"], ctx["pose"], it, name, cam=cam)


class VisionManager:
    def __init__(self, extra_dirs: list[Path], store_path: Path, mapstore) -> None:
        self.classes = discover(extra_dirs)
        self.store_path = store_path
        self.map = mapstore
        self.enabled: dict[str, set[str]] = {}
        self.params: dict[str, dict] = {}
        self.runners: dict[str, _Runner] = {}
        if store_path.exists():
            try:
                d = json.loads(store_path.read_text("utf-8"))
                self.params = d.get("params", {})
                self.enabled = {k: set(v) for k, v in d.get("enabled", {}).items()}
            except Exception:
                pass
        self.probe: dict[str, str] = {}
        for name, cls in self.classes.items():
            try:
                inst = cls()
                if not inst.available:
                    self.probe[name] = inst.unavailable_reason
            except Exception as e:
                self.probe[name] = str(e)
        log.info("vision plugins: %s", ", ".join(sorted(self.classes)))

    def attach(self, node: "DroneNode") -> None:
        r = _Runner(self, node)
        self.runners[node.id] = r
        r.start()

    def stop(self) -> None:
        for r in self.runners.values():
            r.running = False

    def _save(self) -> None:
        self.store_path.parent.mkdir(parents=True, exist_ok=True)
        self.store_path.write_text(json.dumps({"params": self.params,
                                               "enabled": {k: sorted(v) for k, v in self.enabled.items()}}, indent=2))

    def describe(self, drone: str | None = None) -> list[dict]:
        out = []
        for name, cls in sorted(self.classes.items()):
            d = cls.describe()
            vals = {k: v.get("default") for k, v in cls.params.items()}
            vals.update(self.params.get(name, {}))
            d["values"] = vals
            d["enabled"] = {did: name in s for did, s in self.enabled.items()}
            avail, reason = name not in self.probe, self.probe.get(name, "")
            for r in self.runners.values():
                inst = r.instances.get(name)
                if inst is not None and not inst.available:
                    avail, reason = False, inst.unavailable_reason
            d["available"], d["unavailable_reason"] = avail, reason
            out.append(d)
        return out

    def set_enabled(self, drone: str, name: str, on: bool) -> None:
        if name not in self.classes:
            raise KeyError(name)
        s = self.enabled.setdefault(drone, set())
        (s.add if on else s.discard)(name)
        if not on:
            r = self.runners.get(drone)
            node = r.node if r else None
            if node:
                node.detections.pop(name, None)
                node.bus.publish("detections", {"drone": drone, "plugin": name, "items": [], "t": time.time(), "ms": 0})
        self._save()

    def set_params(self, name: str, values: dict) -> dict:
        if name not in self.classes:
            raise KeyError(name)
        cur = self.params.setdefault(name, {})
        cur.update(values)
        for r in self.runners.values():
            inst = r.instances.get(name)
            if inst:
                inst.update(values)
        self._save()
        return cur
