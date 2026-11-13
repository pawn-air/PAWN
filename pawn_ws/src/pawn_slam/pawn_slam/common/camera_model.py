"""Pinhole camera with OpenCV ``plumb_bob`` distortion (k1, k2, p1, p2, k3), in camera_optical_frame.

Calibration files use the ROS ``camera_info`` YAML layout (what ``camera_calibration`` writes):
``image_width``, ``image_height``, ``camera_matrix.data`` (3x3, row-major), ``distortion_model``,
``distortion_coefficients.data``.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


@dataclass(frozen=True, eq=False)
class PinholeCamera:
    width: int
    height: int
    K: np.ndarray
    D: np.ndarray = field(default_factory=lambda: np.zeros(5))
    name: str = ""

    def __post_init__(self) -> None:
        object.__setattr__(self, "K", np.asarray(self.K, dtype=float).reshape(3, 3))
        D = np.zeros(5)
        d = np.asarray(self.D, dtype=float).ravel()
        if d.size > 5:
            raise ValueError(f"plumb_bob has at most 5 coefficients, got {d.size}")
        D[:d.size] = d
        object.__setattr__(self, "D", D)

    # ------------------------------------------------------------------ loading
    @classmethod
    def from_camera_info(cls, info: dict) -> PinholeCamera:
        model = info.get("distortion_model", "plumb_bob")
        if model != "plumb_bob":
            raise ValueError(f"unsupported distortion model {model!r} (only plumb_bob)")
        return cls(width=int(info["image_width"]), height=int(info["image_height"]),
                   K=info["camera_matrix"]["data"], D=info["distortion_coefficients"]["data"],
                   name=info.get("camera_name", ""))

    @classmethod
    def from_yaml(cls, path: str | Path) -> PinholeCamera:
        import yaml
        return cls.from_camera_info(yaml.safe_load(Path(path).read_text("utf-8")))

    @property
    def fx(self) -> float:
        return self.K[0, 0]

    @property
    def fy(self) -> float:
        return self.K[1, 1]

    @property
    def cx(self) -> float:
        return self.K[0, 2]

    @property
    def cy(self) -> float:
        return self.K[1, 2]

    # --------------------------------------------------------------- distortion
    def distort(self, xy) -> np.ndarray:
        """Ideal normalized coordinates (N, 2) -> distorted normalized coordinates."""
        xy = np.atleast_2d(np.asarray(xy, dtype=float))
        k1, k2, p1, p2, k3 = self.D
        x, y = xy[:, 0], xy[:, 1]
        r2 = x * x + y * y
        radial = 1 + r2 * (k1 + r2 * (k2 + r2 * k3))
        xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
        return np.stack([xd, yd], axis=1)

    def undistort(self, xy_d, iterations: int = 20) -> np.ndarray:
        """Inverse of :meth:`distort` by fixed-point iteration (OpenCV's undistortPoints method)."""
        xy_d = np.atleast_2d(np.asarray(xy_d, dtype=float))
        k1, k2, p1, p2, k3 = self.D
        x, y = xy_d[:, 0].copy(), xy_d[:, 1].copy()
        for _ in range(iterations):
            r2 = x * x + y * y
            icdist = 1 / (1 + r2 * (k1 + r2 * (k2 + r2 * k3)))
            dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
            dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
            x = (xy_d[:, 0] - dx) * icdist
            y = (xy_d[:, 1] - dy) * icdist
        return np.stack([x, y], axis=1)

    # --------------------------------------------------------------- projection
    def project(self, points) -> np.ndarray:
        """Points in camera_optical_frame (N, 3) -> pixels (N, 2). Points not in front of the camera -> NaN."""
        p = np.atleast_2d(np.asarray(points, dtype=float))
        z = p[:, 2]
        out = np.full((len(p), 2), np.nan)
        front = z > 1e-9
        if front.any():
            xd = self.distort(p[front, :2] / z[front, None])
            out[front] = xd @ self.K[:2, :2].T + self.K[:2, 2]
        return out

    def unproject(self, pixels) -> np.ndarray:
        """Pixels (N, 2) -> unit rays (N, 3) in camera_optical_frame."""
        px = np.atleast_2d(np.asarray(pixels, dtype=float))
        xd = np.stack([(px[:, 0] - self.cx) / self.fx, (px[:, 1] - self.cy) / self.fy], axis=1)
        rays = np.column_stack([self.undistort(xd), np.ones(len(px))])
        return rays / np.linalg.norm(rays, axis=1, keepdims=True)

    def in_image(self, pixels) -> np.ndarray:
        px = np.atleast_2d(np.asarray(pixels, dtype=float))
        return (px[:, 0] >= 0) & (px[:, 0] < self.width) & (px[:, 1] >= 0) & (px[:, 1] < self.height)
