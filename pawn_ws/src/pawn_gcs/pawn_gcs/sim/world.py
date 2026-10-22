"""Very small software renderer for the simulator's synthetic FPV camera.

Renders an indoor "building" (room with walls, pillars, a doorway frame), a red
triangle target, an ArUco marker and a person-like figure from the drone's
pose, so every vision plugin can be exercised without hardware.
Pure NumPy + OpenCV; painter's algorithm + near-plane clipping.
"""
from __future__ import annotations

import math

import cv2
import numpy as np

W, H = 960, 720
FX = FY = 690.0          # ~70 deg horizontal FOV, close to the Tello camera
CX, CY = W / 2, H / 2
NEAR = 0.05
LIGHT = np.array([0.4, 0.3, 0.85]) / np.linalg.norm([0.4, 0.3, 0.85])


def _box(cx, cy, w, d, h, color, z0=0.0):
    x0, x1, y0, y1, z1 = cx - w / 2, cx + w / 2, cy - d / 2, cy + d / 2, z0 + h
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0), (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    faces = [(0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7), (4, 5, 6, 7)]
    return [(np.array([v[i] for i in f], float), color) for f in faces]


SHELL = 5  # first polygons = 4 walls + ceiling (drawn first: nothing is ever behind them)


def _wall_decor(R, HGT, n_per_wall=34, seed=7):
    """Posters / frames / panels on the walls: texture for feature trackers, looming and SLAM.

    Keeps clear of the red triangle, the ArUco marker and the doorway; avoids red hues so the
    red-triangle detector only fires on the real target.
    """
    rng = np.random.default_rng(seed)
    keep_out = {0: [], 1: [(-1.7, -0.1, 0.7, 1.95)], 2: [(-1.7, -0.5, 0.9, 2.1)], 3: [(-1.2, 0.4, 0.0, 2.3)]}
    # wall i spans corners[i] -> corners[i+1]; local coordinate s along the wall, z up
    walls = [((-R, -R), (R, -R)), ((R, -R), (R, R)), ((R, R), (-R, R)), ((-R, R), (-R, -R))]
    palette = [(150, 120, 60), (60, 140, 160), (70, 150, 90), (180, 180, 170), (40, 50, 60), (120, 90, 140),
               (200, 200, 205), (90, 110, 60), (50, 90, 130), (170, 150, 110)]
    out = []
    for wi, ((ax, ay), (bx, by)) in enumerate(walls):
        L = math.hypot(bx - ax, by - ay)
        ux, uy = (bx - ax) / L, (by - ay) / L
        nx, ny = -uy, ux  # inward normal (room is CCW)
        placed = 0
        while placed < n_per_wall:
            w, h = rng.uniform(0.15, 0.8), rng.uniform(0.12, 0.6)
            s0, z0 = rng.uniform(0.3, L - 0.3 - w), rng.uniform(0.15, HGT - 0.2 - h)
            # wall-local s -> world coordinate along the wall for the keep-out tests
            wx0, wy0 = ax + ux * s0, ay + uy * s0
            along = wy0 if wi in (1, 3) else wx0
            along1 = along + (w if (uy > 0 or ux > 0) else -w)
            lo, hi = min(along, along1), max(along, along1)
            if any(lo < k1 and hi > k0 and z0 < kz1 and z0 + h > kz0 for k0, k1, kz0, kz1 in keep_out[wi]):
                continue
            col = palette[int(rng.integers(len(palette)))]
            off = 0.012
            p0 = (ax + ux * s0 + nx * off, ay + uy * s0 + ny * off)
            p1 = (ax + ux * (s0 + w) + nx * off, ay + uy * (s0 + w) + ny * off)
            out.append((np.array([(p0[0], p0[1], z0), (p1[0], p1[1], z0), (p1[0], p1[1], z0 + h), (p0[0], p0[1], z0 + h)], float), col))
            if rng.random() < 0.5:  # inner panel -> more corners
                m = min(w, h) * 0.18
                q0 = (ax + ux * (s0 + m) + nx * off * 1.6, ay + uy * (s0 + m) + ny * off * 1.6)
                q1 = (ax + ux * (s0 + w - m) + nx * off * 1.6, ay + uy * (s0 + w - m) + ny * off * 1.6)
                c2 = tuple(int(c * 0.55) for c in col)
                out.append((np.array([(q0[0], q0[1], z0 + m), (q1[0], q1[1], z0 + m), (q1[0], q1[1], z0 + h - m), (q0[0], q0[1], z0 + h - m)], float), c2))
            placed += 1
    return out


def build_world():
    polys = []
    R, HGT = 7.0, 3.0  # room half-size (m), height
    wall_cols = [(170, 160, 150), (150, 165, 175), (165, 150, 170), (160, 170, 150)]
    corners = [(-R, -R), (R, -R), (R, R), (-R, R)]
    for i in range(4):
        (ax, ay), (bx, by) = corners[i], corners[(i + 1) % 4]
        polys.append((np.array([(ax, ay, 0), (bx, by, 0), (bx, by, HGT), (ax, ay, HGT)], float), wall_cols[i]))
    polys.append((np.array([(-R, -R, HGT), (R, -R, HGT), (R, R, HGT), (-R, R, HGT)], float), (95, 95, 100)))
    polys += _wall_decor(R, HGT)
    # pillars / furniture
    polys += _box(3.0, 2.5, 0.6, 0.6, 3.0, (70, 110, 160))
    polys += _box(-3.0, -2.5, 0.6, 0.6, 3.0, (70, 110, 160))
    polys += _box(4.5, -3.5, 1.6, 0.8, 0.9, (60, 90, 120))     # table
    polys += _box(-4.0, 3.5, 1.0, 1.0, 1.2, (40, 120, 90))     # crate
    # "person" (target for person detectors): legs, torso, head
    px, py = 5.0, 4.6
    polys += _box(px, py - 0.12, 0.18, 0.18, 0.85, (60, 50, 40))
    polys += _box(px, py + 0.12, 0.18, 0.18, 0.85, (60, 50, 40))
    polys += _box(px, py, 0.3, 0.5, 0.65, (150, 90, 40), z0=0.85)
    polys += _box(px, py, 0.22, 0.22, 0.25, (140, 170, 210), z0=1.52)
    # red triangle target on the +x wall (the PAWN mission target)
    tri = np.array([(R - 0.01, -1.4, 0.9), (R - 0.01, -0.4, 0.9), (R - 0.01, -0.9, 1.75)], float)
    polys.append((tri, (30, 30, 230)))
    # dark "doorway" on the -y wall
    polys.append((np.array([(-1.0, -R + 0.01, 0), (0.2, -R + 0.01, 0), (0.2, -R + 0.01, 2.1), (-1.0, -R + 0.01, 2.1)]), (45, 40, 35)))
    return polys, R


def _aruco_texture():
    try:
        d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        m = cv2.aruco.generateImageMarker(d, 7, 200)
    except Exception:
        m = np.zeros((200, 200), np.uint8)
        m[40:160, 40:160] = 255
    m = cv2.copyMakeBorder(m, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255)
    return cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)


class Renderer:
    def __init__(self) -> None:
        self.polys, self.R = build_world()
        self.aruco = _aruco_texture()
        R = self.R
        # ArUco id 7 on the +y wall, 1.4 m above floor
        self.aruco_quad = np.array([(-1.5, R - 0.01, 1.9), (-0.7, R - 0.01, 1.9), (-0.7, R - 0.01, 1.1), (-1.5, R - 0.01, 1.1)], float)
        shades = []
        for pts, col in self.polys:
            n = np.cross(pts[1] - pts[0], pts[2] - pts[0])
            nn = np.linalg.norm(n)
            k = 0.65 + 0.35 * abs(float(np.dot(n / nn, LIGHT))) if nn > 0 else 1.0
            shades.append(tuple(int(c * k) for c in col))
        self.shaded = shades

    def _cam(self, x, y, z, yaw, pitch, roll):
        """World->camera rotation. World: x fwd, y left, z up. Camera: x right, y down, z fwd."""
        cy, sy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])           # body->world yaw
        cp, sp = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        cr, sr = math.cos(math.radians(roll)), math.sin(math.radians(roll))
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        Rbw = Rz @ Ry @ Rx
        B2C = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], float)    # body(x fwd,y left,z up)->cam
        Rwc = B2C @ Rbw.T
        return Rwc, np.array([x, y, z])

    @staticmethod
    def _clip(pc):
        out = []
        n = len(pc)
        for i in range(n):
            a, b = pc[i], pc[(i + 1) % n]
            ain, bin_ = a[2] >= NEAR, b[2] >= NEAR
            if ain:
                out.append(a)
            if ain != bin_:
                t = (NEAR - a[2]) / (b[2] - a[2])
                out.append(a + t * (b - a))
        return np.array(out) if len(out) >= 3 else None

    @staticmethod
    def _proj(pc):
        return np.stack([FX * pc[:, 0] / pc[:, 2] + CX, FY * pc[:, 1] / pc[:, 2] + CY], 1)

    def render(self, x, y, z, yaw, pitch=0.0, roll=0.0, t=0.0, overlay: str = "") -> np.ndarray:
        img = np.empty((H, W, 3), np.uint8)
        img[:] = (110, 110, 115)
        Rwc, C = self._cam(x, y, max(z, 0.08), yaw, pitch, roll)
        R = self.R
        # floor
        floor = np.array([(-R, -R, 0), (R, -R, 0), (R, R, 0), (-R, R, 0)], float)
        self._fill(img, floor, (85, 95, 105), Rwc, C)
        # floor grid (1 m)
        for k in np.arange(-R, R + 0.01, 1.0):
            for a, b in (((k, -R, 0), (k, R, 0)), ((-R, k, 0), (R, k, 0))):
                self._line(img, np.array(a, float), np.array(b, float), (120, 130, 140), Rwc, C)
        # polys sorted far->near
        items = []
        for k, ((pts, _), col) in enumerate(zip(self.polys, self.shaded)):
            pc = (Rwc @ (pts - C).T).T
            d = float(np.mean(np.linalg.norm(pc, axis=1)))
            items.append((1e6 + d if k < SHELL else d, pc, col))  # room shell always first
        aru_c = (Rwc @ (self.aruco_quad - C).T).T
        items.append((float(np.mean(np.linalg.norm(aru_c, axis=1))) - 0.02, aru_c, "aruco"))
        items.sort(key=lambda it: -it[0])
        for _, pc, col in items:
            if isinstance(col, str):
                self._aruco(img, pc)
                continue
            cp = self._clip(pc)
            if cp is None:
                continue
            p2 = self._proj(cp)
            if np.all(p2[:, 0] < -2000) or np.all(p2[:, 0] > W + 2000):
                continue
            cv2.fillPoly(img, [np.clip(p2, -1e5, 1e5).astype(np.int32)], col, cv2.LINE_AA)
        # sensor noise + slight vignette-ish darkening for realism
        noise = np.random.randint(0, 6, (H // 4, W // 4, 1), np.uint8)
        img = cv2.add(img, cv2.resize(noise, (W, H), interpolation=cv2.INTER_NEAREST)[..., None].repeat(3, 2))
        if overlay:
            cv2.putText(img, overlay, (W - 12 - 9 * len(overlay), 84), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (230, 230, 230), 1, cv2.LINE_AA)
        return img

    def _fill(self, img, pts, col, Rwc, C):
        cp = self._clip((Rwc @ (pts - C).T).T)
        if cp is not None:
            cv2.fillPoly(img, [np.clip(self._proj(cp), -1e5, 1e5).astype(np.int32)], col, cv2.LINE_AA)

    def _line(self, img, a, b, col, Rwc, C):
        ac, bc = Rwc @ (a - C), Rwc @ (b - C)
        if ac[2] < NEAR and bc[2] < NEAR:
            return
        if ac[2] < NEAR:
            ac = ac + (NEAR - ac[2]) / (bc[2] - ac[2]) * (bc - ac)
        if bc[2] < NEAR:
            bc = bc + (NEAR - bc[2]) / (ac[2] - bc[2]) * (ac - bc)
        p = self._proj(np.stack([ac, bc]))
        p = np.clip(p, -1e5, 1e5).astype(int)
        cv2.line(img, tuple(p[0]), tuple(p[1]), col, 1, cv2.LINE_AA)

    def _aruco(self, img, pc):
        if np.any(pc[:, 2] < 0.2):
            return
        dst = self._proj(pc).astype(np.float32)
        h, w = self.aruco.shape[:2]
        src = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
        M = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(self.aruco, M, (W, H), flags=cv2.INTER_LINEAR)
        mask = cv2.warpPerspective(np.full((h, w), 255, np.uint8), M, (W, H))
        img[mask > 0] = warped[mask > 0]
