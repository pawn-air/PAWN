"""Built-in vision plugins (pure OpenCV, no extra downloads)."""
from __future__ import annotations

import math

import cv2
import numpy as np

from ..base import VisionPlugin


class RedTriangle(VisionPlugin):
    name = "red_triangle"
    title = "Red triangle target"
    description = "HSV red threshold + polygon approximation. PAWN's search target marker."
    color = "#f43f5e"
    rate_hz = 15
    is_target = True
    params = {
        "s_min": {"type": "int", "default": 110, "min": 0, "max": 255, "label": "Saturation min"},
        "v_min": {"type": "int", "default": 70, "min": 0, "max": 255, "label": "Value min"},
        "hue_tol": {"type": "int", "default": 10, "min": 1, "max": 30, "label": "Hue tolerance"},
        "min_area": {"type": "int", "default": 250, "min": 20, "max": 50000, "label": "Min area px"},
        "eps": {"type": "float", "default": 0.06, "min": 0.01, "max": 0.2, "step": 0.01, "label": "Poly epsilon"},
    }

    def process(self, img, ctx):
        H, W = img.shape[:2]
        hsv = cv2.cvtColor(cv2.GaussianBlur(img, (5, 5), 0), cv2.COLOR_BGR2HSV)
        v = self.values
        t = v["hue_tol"]
        m = cv2.inRange(hsv, (0, v["s_min"], v["v_min"]), (t, 255, 255)) | \
            cv2.inRange(hsv, (180 - t, v["s_min"], v["v_min"]), (180, 255, 255))
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        out = []
        for c in cnts:
            a = cv2.contourArea(c)
            if a < v["min_area"]:
                continue
            ap = cv2.approxPolyDP(c, v["eps"] * cv2.arcLength(c, True), True)
            if len(ap) != 3 or not cv2.isContourConvex(ap):
                continue
            x, y, w, h = cv2.boundingRect(ap)
            tri_area = cv2.contourArea(ap)
            conf = float(min(1.0, a / max(tri_area, 1)))
            pts = ap.reshape(-1, 2)
            out.append({"label": "red_triangle", "conf": round(conf, 2), "box": self.norm_box(x, y, w, h, W, H),
                        "poly": [[round(px / W, 4), round(py / H, 4)] for px, py in pts],
                        "center": [round((x + w / 2) / W, 4), round((y + h / 2) / H, 4)],
                        "info": {"area_px": int(a)}})
        return out


class Aruco(VisionPlugin):
    name = "aruco"
    title = "ArUco markers"
    description = "Fiducial markers (cv2.aruco) - landing pads, waypoints, ground truth for SLAM evaluation."
    color = "#a3e635"
    rate_hz = 15
    is_target = False
    params = {
        "dictionary": {"type": "select", "default": "DICT_4X4_50",
                       "options": ["DICT_4X4_50", "DICT_5X5_100", "DICT_6X6_250", "DICT_ARUCO_ORIGINAL", "DICT_APRILTAG_36h11"],
                       "label": "Dictionary"},
        "marker_cm": {"type": "float", "default": 15.0, "min": 1, "max": 200, "label": "Marker size (cm)"},
    }

    def setup(self):
        if not hasattr(cv2, "aruco"):
            raise RuntimeError("cv2.aruco missing (pip install opencv-contrib-python)")
        self._dict_name = None

    def process(self, img, ctx):
        if self._dict_name != self.values["dictionary"]:
            d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self.values["dictionary"]))
            self._det = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
            self._dict_name = self.values["dictionary"]
        H, W = img.shape[:2]
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._det.detectMarkers(gray)
        out = []
        if ids is None:
            return out
        f = 690.0 * W / 960.0  # approximate Tello focal length (px)
        for c, i in zip(corners, ids.flatten()):
            pts = c.reshape(-1, 2)
            x, y, w, h = cv2.boundingRect(pts.astype(np.float32))
            side = float(np.mean([np.linalg.norm(pts[k] - pts[(k + 1) % 4]) for k in range(4)]))
            dist = self.values["marker_cm"] / 100 * f / max(side, 1)
            out.append({"label": f"aruco:{int(i)}", "conf": 1.0, "box": self.norm_box(x, y, w, h, W, H),
                        "poly": [[round(px / W, 4), round(py / H, 4)] for px, py in pts],
                        "center": [round(float(pts[:, 0].mean()) / W, 4), round(float(pts[:, 1].mean()) / H, 4)],
                        "info": {"id": int(i), "range_m": round(dist, 2)}})
        return out


class PersonHOG(VisionPlugin):
    name = "person_hog"
    title = "Person (HOG)"
    description = "OpenCV HOG+SVM pedestrian detector. CPU-only baseline; swap for YOLO when available."
    color = "#fbbf24"
    rate_hz = 4
    is_target = True
    params = {
        "scale_width": {"type": "int", "default": 480, "min": 240, "max": 960, "label": "Process width"},
        "hit_threshold": {"type": "float", "default": 0.3, "min": -1, "max": 3, "step": 0.1, "label": "Hit threshold"},
    }

    def setup(self):
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def process(self, img, ctx):
        H, W = img.shape[:2]
        sw = self.values["scale_width"]
        k = sw / W
        small = cv2.resize(img, (sw, int(H * k)))
        rects, weights = self.hog.detectMultiScale(small, hitThreshold=self.values["hit_threshold"],
                                                   winStride=(8, 8), padding=(8, 8), scale=1.05)
        out = []
        for (x, y, w, h), s in zip(rects, np.array(weights).flatten() if len(weights) else []):
            out.append({"label": "person", "conf": round(float(min(1, s / 2)), 2),
                        "box": self.norm_box(x / k, y / k, w / k, h / k, W, H),
                        "center": [round((x + w / 2) / k / W, 4), round((y + h / 2) / k / H, 4)]})
        return out


class FaceHaar(VisionPlugin):
    name = "face"
    title = "Faces (Haar)"
    description = "Classic Viola-Jones face detector - quick sanity check of the video pipeline."
    color = "#c084fc"
    rate_hz = 8
    params = {"min_size": {"type": "int", "default": 40, "min": 10, "max": 300, "label": "Min face px"}}

    def setup(self):
        self.cc = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")
        if self.cc.empty():
            raise RuntimeError("haar cascade not found")

    def process(self, img, ctx):
        H, W = img.shape[:2]
        g = cv2.equalizeHist(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
        ms = self.values["min_size"]
        faces = self.cc.detectMultiScale(g, 1.15, 5, minSize=(ms, ms))
        return [{"label": "face", "conf": 0.8, "box": self.norm_box(x, y, w, h, W, H),
                 "center": [round((x + w / 2) / W, 4), round((y + h / 2) / H, 4)]} for x, y, w, h in faces]


class ColorBlob(VisionPlugin):
    name = "color_blob"
    title = "Color blob (tunable)"
    description = "Generic HSV range tracker - tune live for any coloured target."
    color = "#38bdf8"
    rate_hz = 15
    params = {
        "h_min": {"type": "int", "default": 90, "min": 0, "max": 180, "label": "H min"},
        "h_max": {"type": "int", "default": 130, "min": 0, "max": 180, "label": "H max"},
        "s_min": {"type": "int", "default": 120, "min": 0, "max": 255, "label": "S min"},
        "v_min": {"type": "int", "default": 60, "min": 0, "max": 255, "label": "V min"},
        "min_area": {"type": "int", "default": 400, "min": 10, "max": 100000, "label": "Min area"},
        "max_blobs": {"type": "int", "default": 3, "min": 1, "max": 20, "label": "Max blobs"},
    }

    def process(self, img, ctx):
        H, W = img.shape[:2]
        v = self.values
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        m = cv2.inRange(hsv, (v["h_min"], v["s_min"], v["v_min"]), (v["h_max"], 255, 255))
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = sorted((c for c in cnts if cv2.contourArea(c) >= v["min_area"]), key=cv2.contourArea, reverse=True)
        out = []
        for c in cnts[: v["max_blobs"]]:
            x, y, w, h = cv2.boundingRect(c)
            out.append({"label": "blob", "conf": 1.0, "box": self.norm_box(x, y, w, h, W, H),
                        "center": [round((x + w / 2) / W, 4), round((y + h / 2) / H, 4)],
                        "info": {"area_px": int(cv2.contourArea(c))}})
        return out


class FrameQuality(VisionPlugin):
    name = "frame_quality"
    title = "Frame quality (photogrammetry)"
    description = ("Sharpness (variance of Laplacian), brightness, exposure clipping, feature count. "
                   "Use it to gate keyframes for SLAM / 3D reconstruction and spot motion blur.")
    color = "#94a3b8"
    rate_hz = 5
    params = {"blur_threshold": {"type": "float", "default": 60.0, "min": 1, "max": 1000, "label": "Blur threshold"}}

    def setup(self):
        self.orb = cv2.ORB_create(500)

    def process(self, img, ctx):
        g = cv2.cvtColor(cv2.resize(img, (480, 360)), cv2.COLOR_BGR2GRAY)
        sharp = float(cv2.Laplacian(g, cv2.CV_64F).var())
        bright = float(g.mean())
        clip = float(((g < 5) | (g > 250)).mean() * 100)
        feats = len(self.orb.detect(g, None))
        ok = sharp >= self.values["blur_threshold"] and 30 < bright < 225 and feats > 80
        return [{"label": "quality", "conf": 1.0 if ok else 0.0,
                 "info": {"sharpness": round(sharp, 1), "brightness": round(bright, 1), "clipped_pct": round(clip, 1),
                          "orb_features": feats, "keyframe_ok": ok}}]


class MotionObstacles(VisionPlugin):
    name = "motion"
    title = "Moving obstacles (optical flow)"
    description = ("Dense Farneback flow on a low-res frame; the median flow approximates ego-motion, "
                   "cells that deviate strongly are flagged as independently moving objects.")
    color = "#fb923c"
    rate_hz = 8
    params = {
        "threshold": {"type": "float", "default": 2.5, "min": 0.5, "max": 20, "step": 0.5, "label": "Deviation px"},
        "grid": {"type": "int", "default": 12, "min": 4, "max": 32, "label": "Grid cells"},
    }

    def setup(self):
        self._prev = None

    def process(self, img, ctx):
        H, W = img.shape[:2]
        g = cv2.cvtColor(cv2.resize(img, (320, 240)), cv2.COLOR_BGR2GRAY)
        prev, self._prev = self._prev, g
        if prev is None:
            return []
        flow = cv2.calcOpticalFlowFarneback(prev, g, None, 0.5, 2, 13, 2, 5, 1.1, 0)
        ego = np.median(flow.reshape(-1, 2), axis=0)
        dev = np.linalg.norm(flow - ego, axis=2)
        n = self.values["grid"]
        ch, cw = 240 // n, 320 // n
        cells = dev[: ch * n, : cw * n].reshape(n, ch, n, cw).mean(axis=(1, 3))
        mask = (cells > self.values["threshold"]).astype(np.uint8)
        out = []
        num, lab, stats, _ = cv2.connectedComponentsWithStats(mask, 4)
        for i in range(1, num):
            x, y, w, h, a = stats[i]
            if a < 2:
                continue
            out.append({"label": "moving", "conf": round(float(min(1, cells[lab == i].mean() / 10)), 2),
                        "box": [round(x / n, 4), round(y / n, 4), round(w / n, 4), round(h / n, 4)],
                        "center": [round((x + w / 2) / n, 4), round((y + h / 2) / n, 4)]})
        if out or abs(ego).sum() > 0.5:
            out.append({"label": "ego_flow", "conf": 1.0,
                        "info": {"dx": round(float(ego[0]), 2), "dy": round(float(ego[1]), 2)}})
        return out


class Yolo(VisionPlugin):
    name = "yolo"
    title = "YOLO (ultralytics, optional)"
    description = "Any Ultralytics YOLO model. Needs `pip install ultralytics` (+ model download on first use)."
    color = "#f97316"
    rate_hz = 6
    is_target = True
    params = {
        "model": {"type": "str", "default": "yolov8n.pt", "label": "Model file"},
        "conf": {"type": "float", "default": 0.4, "min": 0.05, "max": 0.95, "step": 0.05, "label": "Confidence"},
        "classes": {"type": "str", "default": "person", "label": "Classes (comma, empty=all)"},
        "imgsz": {"type": "int", "default": 480, "min": 160, "max": 1280, "label": "Inference size"},
    }

    def setup(self):
        import ultralytics  # noqa: F401  (raises -> unavailable)
        self._model = None
        self._model_name = None

    def process(self, img, ctx):
        from ultralytics import YOLO
        if self._model_name != self.values["model"]:
            self._model = YOLO(self.values["model"])
            self._model_name = self.values["model"]
        H, W = img.shape[:2]
        res = self._model.predict(img, conf=self.values["conf"], imgsz=self.values["imgsz"], verbose=False)[0]
        wanted = {c.strip() for c in self.values["classes"].split(",") if c.strip()}
        out = []
        for b in res.boxes:
            label = res.names[int(b.cls)]
            if wanted and label not in wanted:
                continue
            x1, y1, x2, y2 = (float(v) for v in b.xyxy[0])
            out.append({"label": label, "conf": round(float(b.conf), 2),
                        "box": self.norm_box(x1, y1, x2 - x1, y2 - y1, W, H),
                        "center": [round((x1 + x2) / 2 / W, 4), round((y1 + y2) / 2 / H, 4)]})
        return out


class CollisionTTC(VisionPlugin):
    name = "collision_ttc"
    title = "Collision guard (looming / time-to-contact)"
    description = ("Monocular time-to-contact from image expansion in the centre of view (Lee 1976 'tau', "
                   "used for bio-inspired MAV obstacle avoidance): corners are tracked with pyramidal LK, the robust "
                   "median scale ratio s gives TTC = dt / (s - 1). The safety monitor brakes when TTC drops below "
                   "the threshold while flying forward.")
    color = "#ef4444"
    rate_hz = 8
    params = {
        "roi": {"type": "float", "default": 0.6, "min": 0.2, "max": 0.95, "step": 0.05, "label": "Centre ROI size"},
        "ttc_warn_s": {"type": "float", "default": 1.5, "min": 0.3, "max": 5, "step": 0.1, "label": "Brake TTC (s)"},
        "baseline_s": {"type": "float", "default": 0.25, "min": 0.08, "max": 1.0, "step": 0.01, "label": "Frame gap (s)"},
        "min_points": {"type": "int", "default": 15, "min": 5, "max": 200, "label": "Min tracked corners"},
    }

    def setup(self):
        self._hist = []      # [(t, gray)]
        self._ttc = None
        self._last_valid = None  # (t, ttc) - for extrapolation when texture is lost

    def process(self, img, ctx):
        g = cv2.cvtColor(cv2.resize(img, (480, 360)), cv2.COLOR_BGR2GRAY)
        t = float(ctx.get("t") or 0.0)
        self._hist = [(tt, gg) for tt, gg in self._hist if t - tt < 1.5] + [(t, g)]
        base = self.values["baseline_s"]
        old = [h for h in self._hist[:-1] if t - h[0] >= base]
        if not old:
            return []
        t0, g0 = old[-1]
        dt = t - t0
        r = self.values["roi"]
        x0, x1, y0, y1 = int(240 - 240 * r), int(240 + 240 * r), int(180 - 180 * r), int(180 + 180 * r)
        mask = np.zeros_like(g0)
        mask[y0:y1, x0:x1] = 255
        p0 = cv2.goodFeaturesToTrack(g0, 300, 0.01, 6, mask=mask)
        info = {"ttc_s": None, "points": 0, "danger": False}
        if p0 is not None and len(p0) >= self.values["min_points"]:
            p1, st, _ = cv2.calcOpticalFlowPyrLK(g0, g, p0, None, winSize=(21, 21), maxLevel=3)
            pb, st2, _ = cv2.calcOpticalFlowPyrLK(g, g0, p1, None, winSize=(21, 21), maxLevel=3)
            good = (st.ravel() == 1) & (st2.ravel() == 1) & (np.linalg.norm((pb - p0).reshape(-1, 2), axis=1) < 1.0)
            a, b = p0.reshape(-1, 2)[good], p1.reshape(-1, 2)[good]
            info["points"] = int(len(a))
            if len(a) >= self.values["min_points"]:
                # pairwise distance ratios are invariant to translation (yaw / sideways motion)
                idx = np.random.default_rng(0).integers(0, len(a), size=(min(600, len(a) * 4), 2))
                d0 = np.linalg.norm(a[idx[:, 0]] - a[idx[:, 1]], axis=1)
                d1 = np.linalg.norm(b[idx[:, 0]] - b[idx[:, 1]], axis=1)
                keep = d0 > 15
                if keep.sum() >= 10:
                    s = float(np.median(d1[keep] / d0[keep]))
                    info["scale"] = round(s, 4)
                    ttc = dt / (s - 1) if s > 1.002 else None
                    if ttc is not None:
                        self._ttc = ttc if self._ttc is None else 0.5 * self._ttc + 0.5 * ttc
                    else:
                        self._ttc = None
                    info["ttc_s"] = round(self._ttc, 2) if self._ttc is not None else None
        if info["ttc_s"] is not None:
            self._last_valid = (t, info["ttc_s"])
        elif self._last_valid and t - self._last_valid[0] < 2.0 and self._last_valid[1] < 8.0:
            # Texture vanished while closing in (blank wall filling the view): keep counting down
            # under a constant-velocity assumption instead of going blind exactly when it matters.
            info["ttc_s"] = round(max(0.0, self._last_valid[1] - (t - self._last_valid[0])), 2)
            info["extrapolated"] = True
        danger = bool(info["ttc_s"] is not None and info["ttc_s"] < self.values["ttc_warn_s"])
        info["danger"] = danger
        box = [round(x0 / 480, 3), round(y0 / 360, 3), round((x1 - x0) / 480, 3), round((y1 - y0) / 360, 3)]
        return [{"label": "ttc", "conf": 1.0 if danger else 0.0, "box": box if danger else None, "info": info}]

def bearing_deg(cx: float, hfov: float = 70.0) -> float:
    """Horizontal bearing of a normalised x coordinate relative to the camera axis (CCW+)."""
    return -math.degrees(math.atan((cx - 0.5) * 2 * math.tan(math.radians(hfov / 2))))
