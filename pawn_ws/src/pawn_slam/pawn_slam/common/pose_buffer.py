"""Time-stamped poses with lookup at any time between two samples.

Camera frames and telemetry arrive at different times; a consumer asks "where was the drone at t?"
and gets the pose interpolated between the two neighbouring samples. No extrapolation: a time
outside the buffered range, or inside a gap longer than ``max_gap``, returns None.
Thread-safe (ROS callbacks may add and look up from different threads).
"""
from __future__ import annotations

import bisect
import threading

from .transforms import Pose, interpolate


class PoseBuffer:
    def __init__(self, max_age: float | None = None, max_gap: float | None = None) -> None:
        """max_age: drop samples older than this many seconds behind the newest one.
        max_gap: refuse to interpolate across a gap longer than this (s), e.g. tracking lost."""
        self.max_age = max_age
        self.max_gap = max_gap
        self._t: list[float] = []
        self._p: list[Pose] = []
        self._lock = threading.Lock()

    def add(self, t: float, pose: Pose) -> None:
        """Insert a sample; out-of-order times are fine, a repeated time replaces the old sample."""
        with self._lock:
            i = bisect.bisect_left(self._t, t)
            if i < len(self._t) and self._t[i] == t:
                self._p[i] = pose
            else:
                self._t.insert(i, t)
                self._p.insert(i, pose)
            if self.max_age is not None:
                k = bisect.bisect_left(self._t, self._t[-1] - self.max_age)
                del self._t[:k], self._p[:k]

    def lookup(self, t: float) -> Pose | None:
        with self._lock:
            i = bisect.bisect_left(self._t, t)
            if i < len(self._t) and self._t[i] == t:
                return self._p[i]
            if i == 0 or i == len(self._t):
                return None
            t0, t1 = self._t[i - 1], self._t[i]
            if self.max_gap is not None and t1 - t0 > self.max_gap:
                return None
            return interpolate(self._p[i - 1], self._p[i], (t - t0) / (t1 - t0))

    def span(self) -> tuple[float, float] | None:
        with self._lock:
            return (self._t[0], self._t[-1]) if self._t else None

    def clear(self) -> None:
        with self._lock:
            self._t.clear()
            self._p.clear()

    def __len__(self) -> int:
        return len(self._t)
