"""Backtracking & return-to-home along the breadcrumb trail.

Indoors there is no GPS and no obstacle map, so a straight line home can go
through a wall. The path the drone already flew *is* known to be free, so:

* **Backtrack path** of a target = trail from the point where the target was
  first seen back to home (what a rescue team would walk), simplified with RDP.
* **RTH** = fly the trail in reverse from the current position to home as a
  normal mission (rotate to face each leg -> camera looks where it flies -> forward).
* **Go to target** = fly the trail to the point the target was seen from.
* **Smart RTH battery check** = battery needed to fly the trail home, using a
  drain rate learned online during this flight (+ landing reserve).
"""
from __future__ import annotations

import math
import time
from collections import deque
from typing import TYPE_CHECKING

from .geometry import path_along_track, path_length, wrap180

if TYPE_CHECKING:
    from .node import DroneNode


def plan_steps(pose: dict, path: list, speed_cms: int = 40, land: bool = True, face_travel: bool = True) -> list[dict]:
    """Convert a world path (m) into Tello mission steps (rotate + forward legs, final altitude)."""
    steps: list[dict] = [{"type": "speed", "cms": int(max(10, min(100, speed_cms))), "note": "nav speed"}]
    yaw = pose["yaw"]
    x, y, z = pose["x"], pose["y"], pose["z"]
    carry = 0.0
    for wp in path[1:]:
        dx, dy = wp[0] - x, wp[1] - y
        dist = math.hypot(dx, dy) + carry
        if dist < 0.2:
            carry = dist
            continue
        carry = 0.0
        if face_travel:
            heading = math.degrees(math.atan2(dy, dx))
            dyaw = wrap180(heading - yaw)
            if abs(dyaw) >= 4:
                steps.append({"type": "rotate", "dir": "ccw" if dyaw > 0 else "cw", "deg": int(round(abs(dyaw)))})
                yaw += round(abs(dyaw)) * (1 if dyaw > 0 else -1)
            cm = int(round(dist * 100))
            while cm >= 20:
                c = min(cm, 500)
                if cm - c and cm - c < 20:  # avoid an illegal short tail
                    c = cm - 20
                steps.append({"type": "move", "dir": "forward", "cm": c})
                cm -= c
        else:
            c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
            bx, by = c * dx + s * dy, -s * dx + c * dy
            steps.append({"type": "go", "x": int(bx * 100), "y": int(by * 100), "z": 0, "speed": speed_cms})
        x, y = wp[0], wp[1]
    steps.append({"type": "hover", "s": 1, "note": "arrived"})
    if land:
        steps.append({"type": "land"})
    return steps


class BatteryModel:
    """Online %/s drain estimate from the battery trend of the current flight."""

    def __init__(self, default_rate: float = 0.11) -> None:
        self.default = default_rate
        self.samples: deque = deque(maxlen=600)

    def update(self, flying: bool, bat: float | None) -> None:
        if not flying or bat is None:
            if not flying:
                self.samples.clear()
            return
        now = time.monotonic()
        if not self.samples or now - self.samples[-1][0] >= 1.0:
            self.samples.append((now, float(bat)))

    @property
    def rate(self) -> float:
        if len(self.samples) < 30:
            return self.default
        t0, b0 = self.samples[0]
        t1, b1 = self.samples[-1]
        if t1 - t0 < 30 or b0 - b1 <= 0:
            return self.default
        return max(0.03, min(0.5, (b0 - b1) / (t1 - t0)))


class Navigator:
    def __init__(self, node: "DroneNode", mapstore, settings) -> None:
        self.node = node
        self.map = mapstore
        self.s = settings
        self.battery = BatteryModel(float(settings.get("navigation.default_drain_pct_s", 0.11)))

    # ---------------------------------------------------------------- paths
    def home_xyz(self) -> tuple:
        tr = self.node.odom.track
        return tuple(tr[0]) if tr else (self.node.cfg.home_x, self.node.cfg.home_y, 0.0)

    def path_home(self) -> list:
        p = self.node.odom.pose()
        return path_along_track(self.node.odom.track, (p["x"], p["y"], p["z"]), self.home_xyz(),
                                eps=float(self.s.get("navigation.rdp_eps_m", 0.15)))

    def backtrack_path(self, tid: int) -> list:
        """Trail from where target ``tid`` was first seen back to home (for display / rescue team)."""
        m = self.map.target(tid)
        if not m:
            raise KeyError(tid)
        start = tuple(m.get("seen_from") or (m["x"], m["y"], m["z"]))
        path = path_along_track(self.node.odom.track, start, self.home_xyz(),
                                eps=float(self.s.get("navigation.rdp_eps_m", 0.15)))
        return [(m["x"], m["y"], m["z"])] + path

    def estimate(self) -> dict:
        """Distance/time/battery needed to fly the trail home now."""
        path = self.path_home()
        L = path_length(path)
        v = float(self.s.get("navigation.speed_cms", 40)) / 100
        turns = len(path)
        t = L / max(v, 0.1) + turns * 2.0 + 6.0  # per-leg rotate/settle + landing
        need = t * self.battery.rate + float(self.s.get("navigation.reserve_pct", 5))
        return {"path_m": round(L, 2), "time_s": round(t, 1), "bat_needed": round(need, 1),
                "drain_pct_s": round(self.battery.rate, 3), "legs": max(0, turns - 1),
                "straight_m": round(math.dist(path[0][:2], path[-1][:2]), 2) if path else 0.0}

    # ---------------------------------------------------------------- actions
    def _fly(self, name: str, path: list, land: bool, face_xy: tuple | None = None) -> dict:
        n = self.node
        if not n.flying:
            raise RuntimeError("not flying")
        pose = n.odom.pose()
        steps = plan_steps(pose, path, int(self.s.get("navigation.speed_cms", 40)), land=land)
        if face_xy is not None:  # finish facing a point (e.g. the target)
            yaw = pose["yaw"] + sum(st["deg"] * (1 if st["dir"] == "ccw" else -1) for st in steps if st["type"] == "rotate")
            end = path[-1]
            face = wrap180(math.degrees(math.atan2(face_xy[1] - end[1], face_xy[0] - end[0])) - yaw)
            if abs(face) > 5:
                steps.insert(len(steps) - (2 if land else 1), {"type": "rotate", "dir": "ccw" if face > 0 else "cw", "deg": int(abs(face))})
        mission = {"name": name, "description": f"auto-generated {name} along breadcrumbs ({path_length(path):.1f} m)",
                   "steps": steps, "generated": True}
        if n.mission.state in ("running", "paused"):
            n.mission.abort(name)
        n.mission.start(mission)
        n.bus.event(f"{name}: following breadcrumb trail {path_length(path):.1f} m, {len(path) - 1} legs", "warn", n.id, src="nav")
        return mission

    def rth(self, land: bool = True) -> dict:
        return self._fly("RTH", self.path_home(), land)

    def goto_target(self, tid: int) -> dict:
        m = self.map.target(tid)
        if not m:
            raise KeyError(tid)
        p = self.node.odom.pose()
        goal = tuple(m.get("seen_from") or (m["x"], m["y"], m["z"]))
        path = path_along_track(self.node.odom.track, (p["x"], p["y"], p["z"]), goal)
        return self._fly(f"GOTO-{m['label']}", path, land=False, face_xy=(m["x"], m["y"]))
