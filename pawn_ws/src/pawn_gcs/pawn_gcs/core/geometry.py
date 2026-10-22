"""Camera & path geometry shared by target localisation, coverage, backtracking and dataset export.

Frames
  world  : x = forward at takeoff, y = left, z = up (metres) - same as Odometry
  body   : x = forward, y = left, z = up, rotated by yaw (deg, CCW+)
  camera : OpenCV convention, x = right, y = down, z = optical axis
Image coordinates are normalised (u, v in 0..1, origin top-left), like vision plugin output.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class Camera:
    """Pinhole model of the Tello FPV camera (defaults match the 960x720 stream, ~70 deg HFOV)."""
    width: int = 960
    height: int = 720
    hfov_deg: float = 70.0
    tilt_deg: float = 0.0          # camera pitched down (+) relative to the body x axis

    @classmethod
    def from_settings(cls, settings, width: int | None = None, height: int | None = None) -> "Camera":
        return cls(width=int(width or 960), height=int(height or 720),
                   hfov_deg=float(settings.get("camera.hfov_deg", 70.0)),
                   tilt_deg=float(settings.get("camera.tilt_deg", 0.0)))

    @property
    def fx(self) -> float:
        return (self.width / 2) / math.tan(math.radians(self.hfov_deg) / 2)

    @property
    def fy(self) -> float:
        return self.fx  # square pixels

    @property
    def cx(self) -> float:
        return self.width / 2

    @property
    def cy(self) -> float:
        return self.height / 2

    @property
    def vfov_deg(self) -> float:
        return math.degrees(2 * math.atan(self.cy / self.fy))

    def cam_ray(self, u: float, v: float) -> np.ndarray:
        """Un-normalised camera-frame direction (x right, y down, z forward) for image point (u, v)."""
        return np.array([(u * self.width - self.cx) / self.fx, (v * self.height - self.cy) / self.fy, 1.0])

    def world_ray(self, u: float, v: float, yaw_deg: float) -> np.ndarray:
        """Unit world-frame direction of the ray through (u, v) for a drone with heading ``yaw_deg``."""
        c = self.cam_ray(u, v)
        fwd, left, up = c[2], -c[0], -c[1]            # camera -> body
        t = math.radians(self.tilt_deg)               # pitch the camera down
        fwd, up = fwd * math.cos(t) + up * math.sin(t), -fwd * math.sin(t) + up * math.cos(t)
        y = math.radians(yaw_deg)
        d = np.array([math.cos(y) * fwd - math.sin(y) * left, math.sin(y) * fwd + math.cos(y) * left, up])
        return d / np.linalg.norm(d)

    def depth_to_ray_length(self, u: float, v: float, depth: float) -> float:
        """Convert depth along the optical axis to distance along the pixel ray."""
        return float(depth * np.linalg.norm(self.cam_ray(u, v)))

    def R_world_cam(self, yaw_deg: float) -> np.ndarray:
        """Rotation whose columns are the camera x/y/z axes expressed in world coordinates (OpenCV camera)."""
        y, t = math.radians(yaw_deg), math.radians(self.tilt_deg)
        fwd = np.array([math.cos(y) * math.cos(t), math.sin(y) * math.cos(t), -math.sin(t)])
        right = np.array([math.sin(y), -math.cos(y), 0.0])
        down = np.cross(fwd, right)
        return np.stack([right, down, fwd], axis=1)

    def intrinsics(self) -> dict:
        return {"w": self.width, "h": self.height, "fl_x": round(self.fx, 3), "fl_y": round(self.fy, 3),
                "cx": self.cx, "cy": self.cy, "hfov_deg": self.hfov_deg}


# --------------------------------------------------------------------------- paths
def rdp(points: list, eps: float) -> list:
    """Ramer-Douglas-Peucker polyline simplification (iterative, works on 2D or 3D points)."""
    if len(points) < 3:
        return list(points)
    P = np.asarray(points, float)
    keep = np.zeros(len(P), bool)
    keep[0] = keep[-1] = True
    stack = [(0, len(P) - 1)]
    while stack:
        a, b = stack.pop()
        if b <= a + 1:
            continue
        seg = P[b] - P[a]
        L = np.linalg.norm(seg)
        rel = P[a + 1:b] - P[a]
        if L < 1e-9:
            d = np.linalg.norm(rel, axis=1)
        else:
            d = np.linalg.norm(np.cross(rel, seg / L), axis=1) if P.shape[1] == 3 else \
                np.abs(rel[:, 0] * seg[1] - rel[:, 1] * seg[0]) / L
        i = int(np.argmax(d))
        if d[i] > eps:
            m = a + 1 + i
            keep[m] = True
            stack += [(a, m), (m, b)]
    return [tuple(map(float, p)) for p in P[keep]]


def path_length(points) -> float:
    return float(sum(math.dist(points[i], points[i + 1]) for i in range(len(points) - 1))) if len(points) > 1 else 0.0


def nearest_index(points, p) -> int:
    if not len(points):
        return -1
    P = np.asarray(points, float)[:, :2]
    return int(np.argmin(np.sum((P - np.asarray(p[:2], float)) ** 2, axis=1)))


def path_along_track(track, start, goal, eps: float = 0.15, min_seg: float = 0.25) -> list:
    """Path from ``start`` to ``goal`` that follows the already-flown track (known obstacle-free).

    Both ends snap to the nearest track point; the track between them is
    simplified with RDP and very short segments are merged.
    """
    track = list(track)
    if len(track) < 2:
        return [tuple(start), tuple(goal)]
    i, j = nearest_index(track, start), nearest_index(track, goal)
    seg = track[i:j + 1] if i <= j else track[j:i + 1][::-1]
    pts = [tuple(start)] + [tuple(p) for p in seg] + [tuple(goal)]
    pts = rdp(pts, eps)
    out = [pts[0]]
    for p in pts[1:-1]:
        if math.dist(p[:2], out[-1][:2]) >= min_seg:
            out.append(p)
    if math.dist(pts[-1][:2], out[-1][:2]) >= 0.05 or len(out) == 1:
        out.append(pts[-1])
    return out


def wrap180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0
