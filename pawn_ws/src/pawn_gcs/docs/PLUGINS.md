# Writing plugins

Put a `.py` file in `plugins/` (or any folder listed in `config.toml → paths.plugin_dirs`) and restart. Start from `plugins/example_plugins.py`.

## Vision plugin

```python
from pawn_gcs.vision.base import VisionPlugin

class MyDetector(VisionPlugin):
    name = "my_detector"          # unique id (used in missions & controllers)
    title = "My detector"
    description = "What it does"
    color = "#22d3ee"             # overlay colour
    rate_hz = 10                  # max calls per second
    is_target = True              # detections drop map markers while flying
    params = {                    # rendered as a form in the Vision tab, editable live
        "threshold": {"type": "float", "default": 0.5, "min": 0, "max": 1, "step": 0.05, "label": "Threshold"},
        "mode": {"type": "select", "default": "fast", "options": ["fast", "accurate"]},
    }

    def setup(self):              # load models; raise -> shown as "unavailable"
        self.model = ...

    def process(self, img, ctx):  # img: BGR uint8 HxWx3 (960x720 on Tello)
        # ctx = {"drone", "state" (raw telemetry), "pose" (x,y,z,yaw), "frame_id", "t"}
        H, W = img.shape[:2]
        return [{
            "label": "person", "conf": 0.9,
            "box": self.norm_box(x, y, w, h, W, H),      # normalised 0..1
            "center": [cx / W, cy / H],                  # used for bearing + visual servoing
            "poly": [[x1/W, y1/H], ...],                  # optional outline
            "info": {"anything": "json-serialisable"},   # shown in the Vision tab / logs
        }]
```

Param types: `int`, `float` (slider when `min`/`max` given), `bool`, `str`, `select`. `self.values[...]` holds current values.

Plugins run in a worker thread on the **latest** frame; if a plugin is slow it just runs less often, it never adds video latency. A crash disables that plugin and logs the error.

Detections are:
* drawn on the HUD,
* published on the bus (`detections`) and written to the flight log,
* available to missions (`wait_detection`, `scan` steps use the plugin `name` and optional `label`),
* available to controllers (`node.detections[name]`),
* turned into map markers when `is_target = True` and the drone is flying.

## Autonomy controller

```python
from pawn_gcs.autonomy import Controller

class Explore(Controller):
    name = "explore"
    title = "Frontier exploration"
    description = "..."
    params = {"speed": {"type": "int", "default": 30, "min": 0, "max": 100}}

    def reset(self):              # called when selected / re-engaged
        self.t0 = None

    def step(self, node):         # called at rc rate (20 Hz) while in AUTO mode and flying
        det = node.detections.get("red_triangle")      # latest vision output
        pose = node.odom.pose()                          # x, y, z, yaw (SLAM pose if one is posted)
        frame = node.video.store.latest()                # .img, .t, .id
        self.debug = {"anything": 1}                     # shown live in the Autonomy tab
        return (lr, fb, ud, yaw)                         # -100..100 each, or None
```

Rules the GCS enforces for you: pilot input overrides instantly, safety failsafes stay active, an exception in `step()` drops back to MANUAL.

Keep `step()` fast (< 5 ms). Heavy work belongs in a vision plugin (its own thread); the controller just reads the results.

## Targets → 3D map positions (v2)

If your plugin sets `is_target = True`, every detection with a `center` (and ideally a `box`) is turned into a
3D target automatically: rays from several viewpoints are triangulated, and if the class has a real-world height
in Settings → `targets.known_height_m` (e.g. `{"person": 1.7}`) the box height also gives range. Put your
label there (exactly the `label` your plugin returns). A detection may also carry `info.range_m` (metric range,
as the ArUco plugin does) which is used directly. Details: [SAR.md](SAR.md#1-target-localisation--coretargetspy-coregeometrypy).
