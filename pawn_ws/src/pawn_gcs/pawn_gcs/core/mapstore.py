"""Shared map layer: localised targets, bookmarks, external markers and point clouds (SLAM).

Targets come from :class:`~pawn_gcs.core.targets.TargetLocalizer` (detection rays
triangulated + known-size ranging). External modules (SLAM, a second detector)
can still POST absolute markers to ``/api/map/markers``.
"""
from __future__ import annotations

import threading
import time

from .geometry import Camera
from .targets import TargetLocalizer


class MapStore:
    def __init__(self, bus, settings=None) -> None:
        self.bus = bus
        self.settings = settings
        self._lock = threading.Lock()
        self.markers: list[dict] = []          # api / bookmark / slam markers
        self.clouds: dict[str, dict] = {}      # name -> {"points": [[x,y,z],...], "color": "#..."}
        self.loc = TargetLocalizer(settings) if settings is not None else None
        self._next = 1
        self._last_obs: dict[tuple, float] = {}
        self._last_pub: dict[int, float] = {}

    def _alloc(self) -> int:
        i = self._next
        self._next += 1
        return i

    # ------------------------------------------------------------ targets
    def add_observation(self, drone: str, label: str, pose: dict, det: dict, plugin: str,
                        source: str = "detection", cam: Camera | None = None, force: bool = False) -> dict | None:
        """Feed one detection (normalised ``center`` + optional ``box``) into the localiser."""
        if self.loc is None or not det.get("center"):
            return None
        now = time.time()
        key = (drone, label, plugin)
        hz = float(self.settings.get("targets.max_obs_hz", 5.0))
        if not force and now - self._last_obs.get(key, 0) < 1.0 / max(hz, 0.1):
            return None
        self._last_obs[key] = now
        cam = cam or Camera.from_settings(self.settings)
        with self._lock:
            t, created, confirmed = self.loc.observe(self._alloc, drone, label, pose, det, plugin, source, cam)
            m = t.as_dict()
        if created:
            self.bus.event(f"Target candidate '{label}' near ({m['x']:.1f}, {m['y']:.1f}, {m['z']:.1f}) m", "info", drone, src="map")
        if confirmed:
            self.bus.event(f"TARGET '{label}' confirmed at ({m['x']:.1f}, {m['y']:.1f}, {m['z']:.1f}) m "
                           f"±{m['sigma']:.1f} m [{m['method']}]", "success", drone, src="map")
            self.bus.publish("target", {"drone": drone, "status": "confirmed", "marker": m})
        if created or confirmed or force or now - self._last_pub.get(t.id, 0) > 0.25:
            self._last_pub[t.id] = now
            self.bus.publish("map", {"type": "marker", "marker": m})
        return m

    def target(self, tid: int) -> dict | None:
        for m in self.all_markers():
            if m["id"] == tid:
                return m
        return None

    # ------------------------------------------------------------ plain markers
    def add_marker(self, data: dict) -> dict:
        with self._lock:
            m = {"id": self._alloc(), "label": str(data.get("label", "marker")), "drone": data.get("drone"),
                 "source": data.get("source", "api"), "x": float(data.get("x", 0)), "y": float(data.get("y", 0)),
                 "z": float(data.get("z", 0)), "bearing": data.get("bearing"), "conf": float(data.get("conf", 1)),
                 "hits": 1, "t": time.time(), "t_last": time.time(), "plugin": data.get("plugin"),
                 "status": data.get("status", "confirmed" if data.get("source", "api") != "bookmark" else "bookmark"),
                 "sigma": float(data.get("sigma", 0.0)), "method": data.get("method", data.get("source", "api")),
                 "note": data.get("note", ""), "seen_from": data.get("seen_from")}
            self.markers.append(m)
        self.bus.publish("map", {"type": "marker", "marker": dict(m)})
        return m

    def remove_marker(self, mid: int) -> None:
        with self._lock:
            self.markers = [m for m in self.markers if m["id"] != mid]
            if self.loc:
                self.loc.remove(mid)
        self.bus.publish("map", {"type": "marker_removed", "id": mid})

    def clear(self) -> None:
        with self._lock:
            self.markers.clear()
            self.clouds.clear()
            if self.loc:
                self.loc.clear()
        self.bus.publish("map", {"type": "cleared"})

    def set_cloud(self, name: str, points: list, color: str = "#94a3b8") -> None:
        pts = [[round(float(p[0]), 3), round(float(p[1]), 3), round(float(p[2]), 3)] for p in points[:50000]]
        with self._lock:
            self.clouds[name] = {"points": pts, "color": color}
        self.bus.publish("map", {"type": "cloud", "name": name, "points": pts, "color": color})

    def all_markers(self) -> list[dict]:
        with self._lock:
            out = [t.as_dict() for t in self.loc.targets] if self.loc else []
            return out + [dict(m) for m in self.markers]

    def snapshot(self) -> dict:
        markers = self.all_markers()
        with self._lock:
            return {"markers": markers, "clouds": {k: dict(v) for k, v in self.clouds.items()}}

    # backwards compatible helper (v1 API): bearing-only marker
    def add_detection_marker(self, drone: str, label: str, pose: dict, bearing: float, conf: float, plugin: str):
        import math
        cam = Camera.from_settings(self.settings) if self.settings is not None else Camera()
        u = 0.5 - math.tan(math.radians(bearing)) / (2 * math.tan(math.radians(cam.hfov_deg / 2)))
        return self.add_observation(drone, label, pose, {"center": [u, 0.5], "conf": conf}, plugin, force=True)
