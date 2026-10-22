"""Search coverage map ("where has the camera looked?").

Search-and-rescue GCSs show which area a sensor has already swept so the
operator doesn't re-search the same room and can see the gaps. Here each
telemetry update stamps the camera's horizontal field-of-view wedge (up to
``coverage.range_m``) into a sparse 2D grid. Without depth there is no
occlusion test - a SLAM module can later replace this with ray-casting.
Cells store a "look count"; the UI renders it as a heat map.
"""
from __future__ import annotations

import math
import threading
import time


class CoverageGrid:
    def __init__(self, bus, settings) -> None:
        self.bus = bus
        self.s = settings
        self.cells: dict[tuple[int, int], int] = {}
        self._dirty: set[tuple[int, int]] = set()
        self._lock = threading.Lock()
        self._last: dict[str, float] = {}
        self._last_pub = 0.0

    @property
    def res(self) -> float:
        return float(self.s.get("coverage.cell_m", 0.25))

    def update(self, drone: str, pose: dict, flying: bool) -> None:
        if not flying or not self.s.get("coverage.enabled", True):
            return
        now = time.monotonic()
        if now - self._last.get(drone, 0) < 0.4:
            return
        self._last[drone] = now
        R = float(self.s.get("coverage.range_m", 3.0))
        half = math.radians(float(self.s.get("camera.hfov_deg", 70.0)) / 2)
        res = self.res
        px, py, yaw = pose["x"], pose["y"], math.radians(pose["yaw"])
        i0, i1 = math.floor((px - R) / res), math.floor((px + R) / res)
        j0, j1 = math.floor((py - R) / res), math.floor((py + R) / res)
        with self._lock:
            for i in range(i0, i1 + 1):
                cx = (i + 0.5) * res - px
                for j in range(j0, j1 + 1):
                    cy = (j + 0.5) * res - py
                    d = math.hypot(cx, cy)
                    if d > R or d < 1e-6:
                        continue
                    a = math.atan2(cy, cx) - yaw
                    a = (a + math.pi) % (2 * math.pi) - math.pi
                    if abs(a) <= half or d < res:
                        k = (i, j)
                        self.cells[k] = min(self.cells.get(k, 0) + 1, 999)
                        self._dirty.add(k)
            if now - self._last_pub > 1.0 and self._dirty:
                self._last_pub = now
                delta = [[k[0], k[1], self.cells[k]] for k in self._dirty]
                self._dirty.clear()
            else:
                delta = None
        if delta:
            self.bus.publish("coverage", {"res": res, "cells": delta, "area_m2": self.area()})

    def area(self) -> float:
        return round(len(self.cells) * self.res ** 2, 2)

    def snapshot(self) -> dict:
        with self._lock:
            return {"res": self.res, "cells": [[i, j, c] for (i, j), c in self.cells.items()], "area_m2": self.area()}

    def clear(self) -> None:
        with self._lock:
            self.cells.clear()
            self._dirty.clear()
        self.bus.publish("coverage", {"res": self.res, "cells": [], "area_m2": 0.0, "cleared": True})
