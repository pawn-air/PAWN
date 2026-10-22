"""Example user plugins - copy this file to start your own.

Anything in this folder is loaded at start-up:
  * subclasses of VisionPlugin  -> Vision tab (run on every drone's video)
  * subclasses of Controller    -> Autonomy tab (produce rc sticks in AUTO mode)

Restart the GCS after editing (python -m pawn_gcs --sim to try it without a drone).
"""
import cv2
import numpy as np

from pawn_gcs.autonomy import Controller
from pawn_gcs.vision.base import VisionPlugin


class BrightSpot(VisionPlugin):
    """Finds the brightest blob - e.g. a person signalling with a phone flashlight in a dark room."""

    name = "bright_spot"
    title = "Bright spot (example)"
    description = "Template plugin: brightest region after blur. Copy plugins/example_plugins.py to write your own."
    color = "#fde047"
    rate_hz = 10
    is_target = False          # True -> detections drop markers on the map while flying
    params = {
        "blur": {"type": "int", "default": 21, "min": 3, "max": 61, "step": 2, "label": "Blur kernel"},
        "min_value": {"type": "int", "default": 240, "min": 0, "max": 255, "label": "Min brightness"},
    }

    def setup(self):
        # load models / allocate buffers here; raise an exception to mark the plugin unavailable
        pass

    def process(self, img, ctx):
        H, W = img.shape[:2]
        k = self.values["blur"] | 1
        g = cv2.GaussianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (k, k), 0)
        _, maxv, _, (x, y) = cv2.minMaxLoc(g)
        if maxv < self.values["min_value"]:
            return []
        r = 30
        return [{"label": "bright_spot", "conf": round(maxv / 255, 2),
                 "box": self.norm_box(x - r, y - r, 2 * r, 2 * r, W, H),
                 "center": [round(x / W, 4), round(y / H, 4)],
                 "info": {"value": int(maxv)}}]


class AltitudeHold(Controller):
    """Holds a target height with a P-controller on the throttle stick; yaw/roll/pitch stay at zero.

    `step()` is called at the rc rate (default 20 Hz). Return (lr, fb, ud, yaw) in -100..100,
    or None to output nothing. `node` gives you: node.state (raw Tello telemetry),
    node.odom.pose() (x, y, z, yaw), node.detections[plugin] (latest vision results),
    node.video.store.latest() (latest decoded frame) and node.bus.event(...) for logging.
    """

    name = "altitude_hold"
    title = "Altitude hold (example)"
    description = "Template controller: P-control on height. Shows the Controller API."
    params = {
        "target_m": {"type": "float", "default": 1.2, "min": 0.3, "max": 3.0, "step": 0.1, "label": "Target height m"},
        "kp": {"type": "float", "default": 80, "min": 0, "max": 300, "label": "Kp"},
    }

    def step(self, node):
        z = node.odom.pose()["z"]
        err = float(self.values["target_m"]) - z
        ud = int(np.clip(float(self.values["kp"]) * err, -40, 40))
        self.debug = {"z": round(z, 2), "err": round(err, 2)}
        return (0, 0, ud, 0)
