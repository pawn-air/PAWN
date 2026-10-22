"""Starter for PAWN's vision-only SLAM: consume the GCS video, publish pose + sparse map back.

Shows the whole integration loop with a deliberately simple monocular visual
odometry (ORB features + essential matrix). Replace `MonoVO` with ORB-SLAM3 /
DPVO / your own pipeline - the GCS side stays the same:

  video in   : raw H.264 relayed by the GCS over UDP  (Settings -> video -> relay_udp = 127.0.0.1:5600)
               or JPEG snapshots from GET /api/drones/<id>/frame.jpg (slower, no setup)
  pose out   : POST /api/drones/<id>/pose     {x, y, z, yaw}   (map frame, metres / degrees)
  map out    : POST /api/map/clouds/<name>    {points: [[x,y,z],...], color}
  targets out: POST /api/map/markers          {label, x, y, z, source: "slam"}

Run:  python examples/slam_bridge.py --drone sim1 --source relay --port 5600
Note: monocular VO has no metric scale; --scale is a crude constant. The GCS
map frame is x forward, y left, z up (camera: x right, y down, z forward).
"""
import argparse
import json
import math
import time
import urllib.request

import cv2
import numpy as np

K = np.array([[690.0, 0, 480], [0, 690.0, 360], [0, 0, 1]])  # approx. Tello intrinsics at 960x720 - calibrate yours!


def post(base, path, body):
    req = urllib.request.Request(base + path, data=json.dumps(body).encode(), method="POST",
                                 headers={"content-type": "application/json"})
    urllib.request.urlopen(req, timeout=5).read()


def frames_relay(port):
    import socket

    import av
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", port))
    codec = av.CodecContext.create("h264", "r")
    while True:
        data, _ = sock.recvfrom(65535)
        try:
            for pkt in codec.parse(data):
                for f in codec.decode(pkt):
                    yield f.to_ndarray(format="bgr24")
        except av.error.InvalidDataError:
            continue  # joined mid-GOP: wait for the next keyframe


def frames_http(base, drone, fps=8):
    while True:
        with urllib.request.urlopen(f"{base}/drones/{drone}/frame.jpg", timeout=5) as r:
            img = cv2.imdecode(np.frombuffer(r.read(), np.uint8), cv2.IMREAD_COLOR)
        yield img
        time.sleep(1 / fps)


class MonoVO:
    def __init__(self, scale=0.05):
        self.orb = cv2.ORB_create(1500)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.prev = None
        self.R = np.eye(3)
        self.t = np.zeros((3, 1))
        self.scale = scale
        self.cloud = []

    def step(self, img):
        g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        kp, des = self.orb.detectAndCompute(g, None)
        if self.prev is None or des is None:
            self.prev = (kp, des)
            return None
        kp0, des0 = self.prev
        self.prev = (kp, des)
        if des0 is None:
            return None
        m = sorted(self.bf.match(des0, des), key=lambda x: x.distance)[:500]
        if len(m) < 30:
            return None
        p0 = np.float32([kp0[x.queryIdx].pt for x in m])
        p1 = np.float32([kp[x.trainIdx].pt for x in m])
        if np.median(np.linalg.norm(p1 - p0, axis=1)) < 1.5:  # not enough parallax -> keep pose
            return self.pose()
        E, mask = cv2.findEssentialMat(p1, p0, K, cv2.RANSAC, 0.999, 1.0)
        if E is None or E.shape != (3, 3):
            return None
        _, R, t, mask = cv2.recoverPose(E, p1, p0, K, mask=mask)
        self.t = self.t + self.scale * (self.R @ t)
        self.R = R @ self.R
        # sparse points (triangulated in the previous camera frame, then moved to world)
        P0 = K @ np.hstack([np.eye(3), np.zeros((3, 1))])
        P1 = K @ np.hstack([R, t])
        good = mask.ravel() > 0
        if good.sum() > 10:
            X = cv2.triangulatePoints(P0, P1, p0[good].T, p1[good].T)
            X = (X[:3] / X[3]).T
            X = X[(X[:, 2] > 0.5) & (X[:, 2] < 40)] * self.scale
            self.cloud.extend(cam_to_map(self.R.T @ x.reshape(3, 1) + self.t) for x in X[::5])
            self.cloud = self.cloud[-5000:]
        return self.pose()

    def pose(self):
        x, y, z = cam_to_map(self.t)
        fwd = self.R.T @ np.array([[0], [0], [1.0]])  # camera z in world
        yaw = math.degrees(math.atan2(-fwd[0, 0], fwd[2, 0]))
        return {"x": x, "y": y, "z": z, "yaw": yaw}


def cam_to_map(v):
    v = np.asarray(v).ravel()
    return [float(v[2]), float(-v[0]), float(-v[1])]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://127.0.0.1:8080")
    ap.add_argument("--drone", default="sim1")
    ap.add_argument("--source", choices=["relay", "http"], default="http")
    ap.add_argument("--port", type=int, default=5600)
    ap.add_argument("--scale", type=float, default=0.05)
    a = ap.parse_args()
    base = a.url + "/api"
    if a.source == "relay":
        post(base, "/settings", {"video": {"relay_udp": [f"127.0.0.1:{a.port}"]}})
        frames = frames_relay(a.port)
    else:
        frames = frames_http(base, a.drone)
    vo = MonoVO(a.scale)
    last_cloud = 0.0
    for img in frames:
        p = vo.step(img)
        if p:
            post(base, f"/drones/{a.drone}/pose", p)
        if time.time() - last_cloud > 2 and vo.cloud:
            post(base, "/map/clouds/vo_points", {"points": vo.cloud, "color": "#64748b"})
            last_cloud = time.time()


if __name__ == "__main__":
    main()
