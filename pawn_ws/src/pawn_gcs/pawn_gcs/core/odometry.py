"""Pose estimation for the map view.

Two sources:
  * ``dead_reckoning`` - integrates Tello's vgx/vgy velocities + height ``h``
    (drifts, but needs nothing extra; good for geofence and the live map),
  * ``external`` - any SLAM / VIO module can POST a pose to
    ``/api/drones/{id}/pose`` (PAWN's vision-only SLAM goes here). When an
    external pose is fresh (< 1 s) it wins.

Frame: x = forward at takeoff, y = left, z = up (metres), yaw in degrees (CCW+).
"""
from __future__ import annotations

import math
import time
from collections import deque


class Odometry:
    def __init__(self, settings, offset=(0.0, 0.0, 0.0)) -> None:
        self.settings = settings
        self.offset = offset  # (x, y, yaw) of this drone's home in the shared map frame
        self.reset()

    def reset(self) -> None:
        self.x = self.y = self.z = 0.0
        self.yaw0: float | None = None
        self.yaw = 0.0
        self.speed = 0.0
        self._t: float | None = None
        self.distance = 0.0
        self.ext: dict | None = None
        self.ext_t = 0.0
        self.track: deque = deque(maxlen=20000)
        self._last_track = 0.0

    def set_home(self, raw_yaw: float | None = None) -> None:
        self.x = self.y = 0.0
        self.distance = 0.0
        self.yaw0 = raw_yaw
        self.track.clear()

    def update(self, st: dict) -> dict:
        now = time.monotonic()
        scale = float(self.settings.get("odometry.velocity_scale_cm", 10.0)) / 100.0
        frame = self.settings.get("odometry.velocity_frame", "world")
        raw_yaw = st.get("yaw", 0.0)
        if self.yaw0 is None:
            self.yaw0 = raw_yaw
        # Tello yaw is CW-positive; convert to CCW-positive relative to home
        self.yaw = _wrap(-(raw_yaw - self.yaw0))
        vx = st.get("vgx", 0.0) * scale
        vy = st.get("vgy", 0.0) * scale
        if frame == "body":
            c, s = math.cos(math.radians(self.yaw)), math.sin(math.radians(self.yaw))
            vx, vy = c * vx - s * vy, s * vx + c * vy
        else:
            # Tello world frame: +vgy is to the right -> our +y is left
            vy = -vy
        self.speed = math.hypot(vx, vy)
        if self._t is not None:
            dt = min(now - self._t, 0.5)
            self.x += vx * dt
            self.y += vy * dt
            self.distance += self.speed * dt
        self._t = now
        self.z = st.get("h", 0.0) / 100.0
        pose = self.pose()
        if now - self._last_track > 0.2 and (not self.track or _moved(self.track[-1], pose)):
            self.track.append((round(pose["x"], 3), round(pose["y"], 3), round(pose["z"], 3)))
            self._last_track = now
        return pose

    def set_external(self, pose: dict) -> None:
        self.ext = pose
        self.ext_t = time.monotonic()

    def pose(self) -> dict:
        if self.ext and time.monotonic() - self.ext_t < 1.0:
            p = self.ext
            return {"x": p.get("x", 0.0), "y": p.get("y", 0.0), "z": p.get("z", self.z),
                    "yaw": p.get("yaw", self.yaw), "speed": self.speed, "dist": self.distance, "source": "external"}
        ox, oy, oyaw = self.offset
        c, s = math.cos(math.radians(oyaw)), math.sin(math.radians(oyaw))
        return {"x": ox + c * self.x - s * self.y, "y": oy + s * self.x + c * self.y, "z": self.z,
                "yaw": _wrap(self.yaw + oyaw), "speed": self.speed, "dist": self.distance, "source": "dead_reckoning"}

    @property
    def horizontal_range(self) -> float:
        """Distance from this drone's own home (geofence)."""
        p = self.pose()
        return math.hypot(p["x"] - self.offset[0], p["y"] - self.offset[1])


def _wrap(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def _moved(a, p) -> bool:
    return abs(a[0] - p["x"]) + abs(a[1] - p["y"]) + abs(a[2] - p["z"]) > 0.03
