"""Autonomy controllers: closed-loop algorithms that output rc sticks in AUTO mode.

This is where PAWN's research algorithms plug in (vision-only exploration,
target search, visual servoing). A controller only *proposes* sticks; the
DroneNode applies them, the pilot can override any time, and safety
monitors stay in charge.

Write one:  subclass ``Controller``, implement ``step(node) -> (lr, fb, ud, yw) | None``,
put the file in ``plugins/`` (same folder as vision plugins) and restart.
"""
from __future__ import annotations

import importlib.util
import inspect
import logging
import time
from pathlib import Path
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from ..core.node import DroneNode

log = logging.getLogger("pawn.auto")


class Controller:
    name = "base"
    title = "Base controller"
    description = ""
    params: dict[str, dict] = {}

    def __init__(self, values: dict | None = None) -> None:
        self.values = {k: v.get("default") for k, v in self.params.items()}
        conv = {"int": lambda x: int(float(x)), "float": float, "str": str, "select": str,
                "bool": lambda x: x in (True, "true", "1", 1, "on")}
        for k, v in (values or {}).items():
            spec = self.params.get(k)
            if spec:
                try:
                    self.values[k] = conv.get(spec.get("type", "str"), str)(v)
                except (TypeError, ValueError):
                    pass
        self.debug: dict[str, Any] = {}

    def reset(self) -> None:
        """Called when the controller is (re)activated."""

    def step(self, node: "DroneNode") -> tuple[int, int, int, int] | None:
        return None

    @classmethod
    def describe(cls) -> dict:
        return {"name": cls.name, "title": cls.title, "description": cls.description, "params": cls.params}


def _best(node: "DroneNode", plugin: str, label: str | None, max_age: float = 0.5):
    d = node.detections.get(plugin)
    if not d or time.time() - d["t"] > max_age:
        return None
    items = [it for it in d.get("items", []) if it.get("box") and (not label or it.get("label") == label)]
    if not items:
        return None
    return max(items, key=lambda it: it["box"][2] * it["box"][3])


class _PID:
    def __init__(self, kp, ki, kd, lim=100):
        self.kp, self.ki, self.kd, self.lim = kp, ki, kd, lim
        self.i = 0.0
        self.prev = None
        self.t = None

    def __call__(self, err):
        now = time.monotonic()
        dt = 0.05 if self.t is None else max(1e-3, now - self.t)
        self.t = now
        self.i = max(-50, min(50, self.i + err * dt))
        d = 0.0 if self.prev is None else (err - self.prev) / dt
        self.prev = err
        u = self.kp * err + self.ki * self.i + self.kd * d
        return int(max(-self.lim, min(self.lim, u)))


class TrackTarget(Controller):
    name = "track_target"
    title = "Visual servo: track target"
    description = ("Image-based visual servoing (IBVS): yaw to center the target horizontally, climb/descend to "
                   "center it vertically, move forward/back to keep its apparent size. Uses any vision plugin.")
    params = {
        "plugin": {"type": "str", "default": "red_triangle", "label": "Vision plugin"},
        "label": {"type": "str", "default": "", "label": "Label (empty = any)"},
        "target_size": {"type": "float", "default": 0.12, "min": 0.01, "max": 0.8, "step": 0.01, "label": "Target box height (0-1)"},
        "kp_yaw": {"type": "float", "default": 120, "min": 0, "max": 400, "label": "Kp yaw"},
        "kp_ud": {"type": "float", "default": 90, "min": 0, "max": 400, "label": "Kp up/down"},
        "kp_fb": {"type": "float", "default": 200, "min": 0, "max": 800, "label": "Kp fwd/back"},
        "max": {"type": "int", "default": 40, "min": 5, "max": 100, "label": "Max stick"},
        "lost_action": {"type": "select", "default": "hover", "options": ["hover", "spin"], "label": "When lost"},
    }

    def reset(self):
        v = self.values
        self.py, self.pz, self.px = _PID(v["kp_yaw"], 0, 8, v["max"]), _PID(v["kp_ud"], 0, 5, v["max"]), _PID(v["kp_fb"], 0, 10, v["max"])

    def step(self, node):
        it = _best(node, self.values["plugin"], self.values["label"] or None)
        if it is None:
            self.debug = {"target": None}
            return (0, 0, 0, 25) if self.values["lost_action"] == "spin" else (0, 0, 0, 0)
        cx, cy = it["center"] if it.get("center") else (it["box"][0] + it["box"][2] / 2, it["box"][1] + it["box"][3] / 2)
        size = it["box"][3]
        ex, ey, es = cx - 0.5, 0.5 - cy, float(self.values["target_size"]) - size
        yw, ud, fb = self.py(ex), self.pz(ey), self.px(es)
        self.debug = {"target": it["label"], "ex": round(ex, 3), "ey": round(ey, 3), "es": round(es, 3)}
        return (0, fb, ud, yw)


class SearchSpin(Controller):
    name = "search_spin"
    title = "Search: spin & scan, then track"
    description = ("Rescue-style search primitive: yaw slowly scanning the room; when the vision plugin reports the "
                   "target it switches to visual-servo tracking. A baseline for PAWN's exploration policy.")
    params = {
        "plugin": {"type": "str", "default": "red_triangle", "label": "Vision plugin"},
        "label": {"type": "str", "default": "", "label": "Label (empty = any)"},
        "yaw_rate": {"type": "int", "default": 25, "min": 5, "max": 100, "label": "Scan yaw stick"},
        "creep_fb": {"type": "int", "default": 0, "min": 0, "max": 40, "label": "Forward creep while scanning"},
    }

    def reset(self):
        self.tracker = TrackTarget({"plugin": self.values["plugin"], "label": self.values["label"]})
        self.tracker.reset()
        self.found = False

    def step(self, node):
        it = _best(node, self.values["plugin"], self.values["label"] or None)
        if it is not None:
            if not self.found:
                node.bus.event("Search: target acquired -> tracking", "success", node.id, src="autonomy")
            self.found = True
            out = self.tracker.step(node)
            self.debug = {"phase": "track", **self.tracker.debug}
            return out
        if self.found:
            node.bus.event("Search: target lost -> scanning", "warn", node.id, src="autonomy")
        self.found = False
        self.debug = {"phase": "scan"}
        return (0, int(self.values["creep_fb"]), 0, int(self.values["yaw_rate"]))


class AutonomyManager:
    def __init__(self, extra_dirs: list[Path]) -> None:
        self.classes: dict[str, type[Controller]] = {c.name: c for c in (TrackTarget, SearchSpin)}
        for d in extra_dirs:
            if not d.exists():
                continue
            for f in sorted(d.glob("*.py")):
                if f.name.startswith("_"):
                    continue
                try:
                    spec = importlib.util.spec_from_file_location(f"pawn_user_auto.{f.stem}", f)
                    mod = importlib.util.module_from_spec(spec)
                    spec.loader.exec_module(mod)
                    for _, obj in inspect.getmembers(mod, inspect.isclass):
                        if issubclass(obj, Controller) and obj is not Controller and obj.name != "base":
                            self.classes[obj.name] = obj
                except Exception as e:
                    log.warning("autonomy plugin %s failed: %s", f, e)
        self.active: dict[str, Controller] = {}
        self.params: dict[str, dict] = {}
        self._dbg_t: dict[str, float] = {}
        self.bus = None

    def describe(self) -> dict:
        return {"controllers": [dict(c.describe(), values={**{k: v.get("default") for k, v in c.params.items()},
                                                           **self.params.get(n, {})})
                                for n, c in self.classes.items()],
                "active": {d: c.name for d, c in self.active.items()}}

    def active_name(self, drone: str) -> str | None:
        c = self.active.get(drone)
        return c.name if c else None

    def select(self, drone: str, name: str | None, values: dict | None = None) -> None:
        if not name:
            self.active.pop(drone, None)
            return
        if name not in self.classes:
            raise KeyError(name)
        if values:
            self.params.setdefault(name, {}).update(values)
        c = self.classes[name](self.params.get(name))
        c.reset()
        self.active[drone] = c

    def step(self, node: "DroneNode"):
        c = self.active.get(node.id)
        if not c:
            return None
        try:
            out = c.step(node)
        except Exception as e:
            log.exception("controller failed")
            node.bus.event(f"Controller '{c.name}' crashed: {e} -> MANUAL", "error", node.id, src="autonomy")
            node.set_mode("MANUAL", "controller error")
            return (0, 0, 0, 0)
        now = time.monotonic()
        if now - self._dbg_t.get(node.id, 0) > 0.25:
            self._dbg_t[node.id] = now
            node.bus.publish("autonomy", {"drone": node.id, "controller": c.name, "out": out, "debug": c.debug})
        return out
