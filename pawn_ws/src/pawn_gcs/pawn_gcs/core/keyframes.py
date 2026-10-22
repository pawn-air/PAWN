"""Keyframe capture + 3D-reconstruction dataset export (the "digital twin" input).

While a flight log is open, sharp frames are kept whenever the drone has moved
or turned enough since the last keyframe (photogrammetry needs parallax, not
30 near-identical frames per second). On log close the session gets:

    keyframes/kf_000001.jpg ...     the images
    keyframes.csv                   t, pose (x y z yaw), sharpness per image
    transforms.json                 nerfstudio / instant-ngp / 3D Gaussian Splatting format
    colmap/cameras.txt              COLMAP text model with the Tello pinhole intrinsics
    colmap/images.txt               ... and the GCS pose of every image as a prior
    colmap/points3D.txt             (empty)

So the folder can go straight into nerfstudio (``ns-train splatfacto --data``),
or into COLMAP (feature extraction + ``point_triangulator`` with the prior poses,
or a fresh ``mapper`` run) and then OpenMVS / Meshroom for a mesh. Poses are the
GCS estimate (dead reckoning or the external SLAM pose when one is posted).
"""
from __future__ import annotations

import csv
import json
import logging
import math
import queue
import threading
import time
from pathlib import Path

import cv2
import numpy as np

from .geometry import Camera

log = logging.getLogger("pawn.kf")


def rot_to_quat(R: np.ndarray) -> tuple[float, float, float, float]:
    """Rotation matrix -> (qw, qx, qy, qz)."""
    tr = np.trace(R)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return (0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s)
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return ((R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s)
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s)
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s)


def c2w_opencv(cam: Camera, pose: dict) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = cam.R_world_cam(pose["yaw"])
    T[:3, 3] = [pose["x"], pose["y"], pose["z"]]
    return T


def write_dataset(folder: Path, frames: list[dict], cam: Camera) -> dict:
    """Write transforms.json + COLMAP text model for ``frames`` [{file, pose, t, sharpness}]."""
    folder = Path(folder)
    if not frames:
        return {"frames": 0}
    flip = np.diag([1.0, -1.0, -1.0, 1.0])  # OpenCV camera -> OpenGL camera (nerfstudio convention)
    tf = {"camera_model": "OPENCV", **{k: v for k, v in cam.intrinsics().items() if k != "hfov_deg"},
          "k1": 0.0, "k2": 0.0, "p1": 0.0, "p2": 0.0, "pose_source": "PAWN GCS estimate",
          "world_frame": "x forward at takeoff, y left, z up (metres)", "frames": []}
    for f in frames:
        T = c2w_opencv(cam, f["pose"]) @ flip
        tf["frames"].append({"file_path": f"keyframes/{f['file']}", "transform_matrix": np.round(T, 6).tolist(),
                             "time": f["t"], "sharpness": f.get("sharpness")})
    (folder / "transforms.json").write_text(json.dumps(tf, indent=1), "utf-8")
    cm = folder / "colmap"
    cm.mkdir(exist_ok=True)
    (cm / "cameras.txt").write_text(
        "# Camera list with one line of data per camera:\n#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n"
        f"1 PINHOLE {cam.width} {cam.height} {cam.fx:.4f} {cam.fy:.4f} {cam.cx:.4f} {cam.cy:.4f}\n", "utf-8")
    lines = ["# Image list with two lines of data per image:",
             "#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME", "#   POINTS2D[] as (X, Y, POINT3D_ID)"]
    for i, f in enumerate(frames, 1):
        Rwc = cam.R_world_cam(f["pose"]["yaw"])
        C = np.array([f["pose"]["x"], f["pose"]["y"], f["pose"]["z"]])
        Rcw = Rwc.T
        t = -Rcw @ C
        q = rot_to_quat(Rcw)
        lines.append(f"{i} {q[0]:.8f} {q[1]:.8f} {q[2]:.8f} {q[3]:.8f} {t[0]:.6f} {t[1]:.6f} {t[2]:.6f} 1 {f['file']}")
        lines.append("")
    (cm / "images.txt").write_text("\n".join(lines) + "\n", "utf-8")
    (cm / "points3D.txt").write_text("# 3D point list (empty - triangulate with COLMAP point_triangulator)\n", "utf-8")
    return {"frames": len(frames)}


class KeyframeRecorder:
    def __init__(self, node, settings) -> None:
        self.node = node
        self.s = settings
        self.folder: Path | None = None
        self.frames: list[dict] = []
        self._last_pose: dict | None = None
        self._last_t = 0.0
        self._q: queue.Queue = queue.Queue(maxsize=8)
        self._lock = threading.Lock()
        self._cam: Camera | None = None
        self._n = 0
        self._worker = threading.Thread(target=self._run, daemon=True, name=f"kf-{node.id}")
        self._worker.start()
        node.frame_listeners.append(self._on_frame)

    def start(self, session: Path) -> None:
        with self._lock:
            self.folder = session
            self.frames = []
            self._last_pose = None
            self._n = 0

    def stop(self) -> dict:
        with self._lock:
            folder, frames, self.folder = self.folder, list(self.frames), None
        if not folder or not frames:
            return {"frames": 0}
        self._q.join()
        with (folder / "keyframes.csv").open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["file", "t", "x", "y", "z", "yaw", "pose_src", "sharpness"])
            for k in frames:
                p = k["pose"]
                w.writerow([k["file"], f"{k['t']:.3f}", p["x"], p["y"], p["z"], p["yaw"], p.get("source"), k["sharpness"]])
        res = write_dataset(folder, frames, self._cam or Camera.from_settings(self.s))
        self.node.bus.event(f"3D dataset: {res['frames']} keyframes -> transforms.json + COLMAP model", "success",
                            self.node.id, src="keyframes")
        return res

    @property
    def count(self) -> int:
        return len(self.frames)

    def _on_frame(self, node, fr) -> None:
        if self.folder is None or not node.flying or not self.s.get("keyframes.enabled", True):
            return
        now = fr.t
        if now - self._last_t < float(self.s.get("keyframes.min_interval_s", 0.3)):
            return
        pose = node.odom.pose()
        lp = self._last_pose
        moved = lp is None or math.dist((pose["x"], pose["y"], pose["z"]), (lp["x"], lp["y"], lp["z"])) >= \
            float(self.s.get("keyframes.min_move_m", 0.25))
        turned = lp is not None and abs(((pose["yaw"] - lp["yaw"]) + 180) % 360 - 180) >= float(self.s.get("keyframes.min_turn_deg", 12))
        stale = now - self._last_t >= float(self.s.get("keyframes.max_interval_s", 4.0))
        if not (moved or turned or stale):
            return
        small = cv2.cvtColor(cv2.resize(fr.img, (320, 240)), cv2.COLOR_BGR2GRAY)
        sharp = float(cv2.Laplacian(small, cv2.CV_64F).var())
        if sharp < float(self.s.get("keyframes.min_sharpness", 40)):
            return
        self._last_t, self._last_pose = now, pose
        H, W = fr.img.shape[:2]
        if self._cam is None or self._cam.width != W:
            self._cam = Camera.from_settings(self.s, W, H)
        with self._lock:
            if self.folder is None:
                return
            self._n += 1
            name = f"kf_{self._n:06d}.jpg"
            self.frames.append({"file": name, "t": fr.t, "pose": {k: (round(v, 4) if isinstance(v, float) else v)
                                                                   for k, v in pose.items()}, "sharpness": round(sharp, 1)})
            folder = self.folder
        try:
            self._q.put((folder / "keyframes" / name, fr.img.copy()), timeout=0.05)
        except queue.Full:  # disk too slow - drop this keyframe instead of stalling the decoder
            with self._lock:
                self.frames = [f for f in self.frames if f["file"] != name]

    def _run(self) -> None:
        while True:
            path, img = self._q.get()
            try:
                path.parent.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(path), img, [cv2.IMWRITE_JPEG_QUALITY, 92])
            except Exception:  # pragma: no cover
                log.exception("keyframe write failed")
            finally:
                self._q.task_done()
