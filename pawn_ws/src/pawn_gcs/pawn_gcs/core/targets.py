"""Monocular target localisation: turn 2D detections into 3D map positions.

A single FPV camera has no depth, so every detection is treated as a *ray*
from the drone's estimated camera position through the detection's pixel.
Two independent cues are fused in one weighted least-squares problem:

1. **Bearing triangulation** - rays seen from different positions intersect
   at the target. Each ray contributes ``(I - d dᵀ)(X - p) ≈ 0`` (perpendicular
   residual, σ ≈ angular noise × distance).
2. **Known-size ranging** - if the real height of the object class is known
   (PAWN red triangle, a person ≈ 1.7 m) the pixel height gives depth
   ``Z = f · H_real / h_px``; this adds ``d dᵀ (X - (p + r d)) ≈ 0`` with
   σ ≈ 15 % of the range.

Normal equations ``(Σ Wᵢ) X = Σ Wᵢ cᵢ`` give the estimate, ``(Σ Wᵢ)⁻¹`` its
covariance (shown as the uncertainty circle on the map). Detections are
associated to existing targets of the same label with a ray-distance gate.
A target becomes *confirmed* after N hits with a bounded uncertainty.

Manual marks (operator clicks on the video) go through exactly the same path,
so a person spotted by the pilot is localised from 2+ clicks at different spots.
"""
from __future__ import annotations

import math
import time

import numpy as np

from .geometry import Camera

ANG_SIGMA = 0.035          # rad, pixel/attitude/heading noise of one ray
RANGE_SIGMA_FRAC = 0.15    # known-size ranging error (fraction of range)
DEFAULT_RANGE = 3.0        # m, used only to weight rays before a range is known
MAX_OBS = 300


class Target:
    def __init__(self, tid: int, label: str, drone: str, plugin: str, source: str) -> None:
        self.id = tid
        self.label = label
        self.drone = drone
        self.plugin = plugin
        self.source = source
        self.obs: list[tuple] = []      # (p, d, r|None, t, conf)
        self.X = np.zeros(3)
        self.cov = np.eye(3) * 99.0
        self.method = "bearing"
        self.hits = 0
        self.conf = 0.0
        self.status = "tentative"
        self.t = time.time()
        self.t_last = self.t
        self.seen_from: tuple | None = None
        self.bearing = None

    # ------------------------------------------------------------------ fit
    def add(self, p: np.ndarray, d: np.ndarray, r: float | None, conf: float) -> None:
        self.obs.append((p, d, r, time.time(), conf))
        if len(self.obs) > MAX_OBS:  # keep the first ones (widest baseline) and the latest
            self.obs = self.obs[:50] + self.obs[-(MAX_OBS - 50):]
        self.hits += 1
        self.conf = max(self.conf, conf)
        self.t_last = time.time()
        if self.seen_from is None:
            self.seen_from = tuple(float(v) for v in p)
        self.solve()

    def solve(self) -> None:
        for _ in range(2):  # second pass re-weights rays with the estimated distance
            M = np.zeros((3, 3))
            b = np.zeros(3)
            n_rng = 0
            for p, d, r, _t, _c in self.obs:
                P = np.eye(3) - np.outer(d, d)
                dist = r if r else (np.linalg.norm(self.X - p) if self.hits > 1 and self.method != "bearing" else DEFAULT_RANGE)
                w_perp = 1.0 / max(ANG_SIGMA * max(dist, 0.3), 0.01) ** 2
                M += w_perp * P
                b += w_perp * P @ p
                if r:
                    n_rng += 1
                    w_r = 1.0 / max(RANGE_SIGMA_FRAC * r, 0.05) ** 2
                    D = np.outer(d, d)
                    M += w_r * D
                    b += w_r * D @ (p + r * d)
            ev = np.linalg.eigvalsh(M)
            spread = self._angular_spread()
            if ev[0] > 1e-9 * ev[-1] and (n_rng or spread > 3.0):
                self.X = np.linalg.solve(M, b)
                self.cov = np.linalg.inv(M)
                self.method = ("range+triangulation" if spread > 5 else "known-size range") if n_rng else "triangulation"
            else:
                p, d, r, *_ = self.obs[-1]
                self.X = p + (r or DEFAULT_RANGE) * d
                self.cov = np.diag([4.0, 4.0, 1.0])
                self.method = "bearing"
        p, d, *_ = self.obs[-1]
        self.bearing = math.degrees(math.atan2(d[1], d[0]))

    def _angular_spread(self) -> float:
        ds = np.array([o[1] for o in self.obs])
        m = ds.mean(axis=0)
        m /= np.linalg.norm(m) or 1
        return float(np.degrees(np.arccos(np.clip(ds @ m, -1, 1))).max() * 2)

    @property
    def sigma(self) -> float:
        """1-σ horizontal uncertainty radius (m)."""
        return float(math.sqrt(max(np.linalg.eigvalsh(self.cov[:2, :2])[-1], 0)))

    def ray_distance(self, p: np.ndarray, d: np.ndarray) -> tuple[float, float]:
        """(perpendicular distance of the estimate to a ray, distance along the ray)."""
        v = self.X - p
        t = float(v @ d)
        return float(np.linalg.norm(v - t * d)), t

    def as_dict(self) -> dict:
        return {"id": self.id, "label": self.label, "drone": self.drone, "plugin": self.plugin, "source": self.source,
                "x": round(float(self.X[0]), 3), "y": round(float(self.X[1]), 3), "z": round(float(self.X[2]), 3),
                "sigma": round(self.sigma, 2), "method": self.method, "status": self.status, "hits": self.hits,
                "n_range": sum(1 for o in self.obs if o[2]), "conf": round(self.conf, 2),
                "bearing": round(self.bearing, 1) if self.bearing is not None else None,
                "seen_from": [round(v, 3) for v in self.seen_from] if self.seen_from else None,
                "t": self.t, "t_last": self.t_last}


class TargetLocalizer:
    def __init__(self, settings) -> None:
        self.settings = settings
        self.targets: list[Target] = []

    def known_height(self, label: str) -> float | None:
        sizes = self.settings.get("targets.known_height_m", {}) or {}
        v = sizes.get(label)
        return float(v) if v else None

    def observe(self, tid_alloc, drone: str, label: str, pose: dict, det: dict, plugin: str, source: str,
                cam: Camera) -> tuple[Target, bool, bool]:
        """Add one detection. Returns (target, created, newly_confirmed)."""
        u, v = det["center"]
        p = np.array([pose["x"], pose["y"], pose["z"]], float)
        d = cam.world_ray(u, v, pose["yaw"])
        r = None
        H = self.known_height(label)
        box = det.get("box")
        if H and box and box[3] > 0.02 and 0.04 < box[1] and box[1] + box[3] < 0.96:  # not cut by the image border
            depth = cam.fy * H / (box[3] * cam.height)
            if 0.2 < depth < 25:
                r = cam.depth_to_ray_length(u, v, depth)
        if det.get("info", {}).get("range_m"):  # e.g. ArUco gives metric range itself
            r = float(det["info"]["range_m"])
        gate_min = float(self.settings.get("targets.gate_m", 1.0))
        best, best_score = None, 1e9
        for t in self.targets:
            if t.label != label:
                continue
            perp, along = t.ray_distance(p, d)
            if along < 0:
                continue
            gate = max(gate_min, 0.35 * along, 2.5 * t.sigma if t.method == "bearing" else 0)
            if r is not None and t.method != "bearing":
                if abs(math.dist(p, t.X) - r) > max(1.5, 0.6 * r):
                    continue
            if perp < gate and perp < best_score:
                best, best_score = t, perp
        created = best is None
        if created:
            best = Target(tid_alloc(), label, drone, plugin, source)
            self.targets.append(best)
        was = best.status
        best.add(p, d, r, float(det.get("conf", 1.0)))
        need = int(self.settings.get("targets.confirm_hits", 5))
        if best.status == "tentative" and best.hits >= need and best.sigma <= float(self.settings.get("targets.confirm_sigma_m", 1.5)):
            best.status = "confirmed"
        return best, created, was != best.status

    def remove(self, tid: int) -> None:
        self.targets = [t for t in self.targets if t.id != tid]

    def clear(self) -> None:
        self.targets.clear()
