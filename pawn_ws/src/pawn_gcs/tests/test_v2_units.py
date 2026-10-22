"""Unit tests for the v2 modules: geometry, target localisation, navigation, patterns, coverage, dataset export."""
import math

import numpy as np
import pytest

from pawn_gcs.config import DEFAULT_SETTINGS, Settings
from pawn_gcs.core.coverage import CoverageGrid
from pawn_gcs.core.geometry import Camera, path_along_track, path_length, rdp
from pawn_gcs.core.keyframes import rot_to_quat, write_dataset
from pawn_gcs.core.mission import preview, validate
from pawn_gcs.core.navigation import BatteryModel, plan_steps
from pawn_gcs.core.patterns import describe, generate
from pawn_gcs.core.targets import TargetLocalizer


class _Bus:
    def __init__(self):
        self.msgs = []

    def publish(self, t, d):
        self.msgs.append((t, d))

    def event(self, *a, **k):
        pass


@pytest.fixture
def settings(tmp_path):
    return Settings(tmp_path / "s.json")


def _project(cam, X, pose):
    R = cam.R_world_cam(pose["yaw"])
    pc = R.T @ (np.asarray(X) - np.array([pose["x"], pose["y"], pose["z"]]))
    return (cam.fx * pc[0] / pc[2] + cam.cx) / cam.width, (cam.fy * pc[1] / pc[2] + cam.cy) / cam.height, pc[2]


# ----------------------------------------------------------------- geometry
def test_world_ray_matches_rotation():
    cam = Camera(tilt_deg=10)
    for yaw in (0, 37, -120):
        for u, v in ((0.5, 0.5), (0.1, 0.8), (0.9, 0.2)):
            d = cam.world_ray(u, v, yaw)
            c = cam.cam_ray(u, v)
            d2 = cam.R_world_cam(yaw) @ (c / np.linalg.norm(c))
            assert np.allclose(d, d2, atol=1e-9)


def test_projection_roundtrip():
    cam = Camera()
    pose = {"x": 1, "y": -2, "z": 1.1, "yaw": 25}
    X = np.array([5.0, 0.5, 1.6])
    u, v, z = _project(cam, X, pose)
    d = cam.world_ray(u, v, pose["yaw"])
    p = np.array([1, -2, 1.1])
    assert np.allclose(p + d * np.linalg.norm(X - p), X, atol=1e-6)


def test_rdp_and_track_path():
    line = [(i * 0.1, 0.0, 1.0) for i in range(50)] + [(4.9, i * 0.1, 1.0) for i in range(1, 30)]
    s = rdp(line, 0.05)
    assert len(s) == 3
    path = path_along_track(line, (4.9, 2.9, 1.0), (0, 0, 1.0))
    assert path[0] == (4.9, 2.9, 1.0) and path[-1] == (0, 0, 1.0)
    assert path_length(path) == pytest.approx(4.9 + 2.9, abs=0.05)  # follows the L, not the diagonal


# ----------------------------------------------------------------- targets
@pytest.mark.parametrize("with_size", [True, False])
def test_localizer_triangulates(settings, with_size):
    cam = Camera()
    loc = TargetLocalizer(settings)
    X = np.array([6.0, 1.0, 1.2])
    n = iter(range(1, 999))
    rng = np.random.default_rng(1)
    for i in range(12):
        pose = {"x": 0.25 * i, "y": -0.2 * i, "z": 0.9, "yaw": 5 + 2 * i}
        u, v, z = _project(cam, X, pose)
        det = {"center": [u + rng.normal(0, 0.003), v + rng.normal(0, 0.003)], "conf": 0.9}
        if with_size:
            h = cam.fy * 0.85 / z / cam.height
            det["box"] = [u - 0.02, v - h / 2, 0.04, h]
        t, *_ = loc.observe(lambda: next(n), "d", "red_triangle", pose, det, "p", "detection", cam)
    assert len(loc.targets) == 1
    assert np.linalg.norm(t.X - X) < 0.25
    assert t.status == "confirmed"
    assert t.method in ("triangulation", "range+triangulation")


def test_localizer_separates_two_targets(settings):
    cam = Camera()
    loc = TargetLocalizer(settings)
    n = iter(range(1, 999))
    for X in ([5, 2, 1], [5, -2, 1]):
        for i in range(4):
            pose = {"x": 0.3 * i, "y": 0.0, "z": 1.0, "yaw": 0.0}
            u, v, _ = _project(cam, X, pose)
            loc.observe(lambda: next(n), "d", "person", pose, {"center": [u, v]}, "p", "detection", cam)
    assert len(loc.targets) == 2


# ----------------------------------------------------------------- navigation
def test_plan_steps_reaches_goal():
    pose = {"x": 3.0, "y": 2.0, "z": 1.0, "yaw": 90.0}
    path = [(3.0, 2.0, 1.0), (3.0, 0.0, 1.0), (0.0, 0.0, 1.0)]
    steps = plan_steps(pose, path, land=True)
    m = {"steps": [{"type": "takeoff"}] + steps}
    validate(m)
    # replay with the kinematic previewer (starts at origin facing +x): shift by the pose
    x, y, yaw = pose["x"], pose["y"], pose["yaw"]
    for st in steps:
        if st["type"] == "rotate":
            yaw += st["deg"] * (1 if st["dir"] == "ccw" else -1)
        elif st["type"] == "move" and st["dir"] == "forward":
            x += math.cos(math.radians(yaw)) * st["cm"] / 100
            y += math.sin(math.radians(yaw)) * st["cm"] / 100
    assert math.hypot(x, y) < 0.05
    assert steps[-1]["type"] == "land"


def test_battery_model_learns_rate():
    bm = BatteryModel(0.1)
    assert bm.rate == 0.1
    import time
    t0 = time.monotonic()
    for i in range(60):
        bm.samples.append((t0 + i, 90 - 0.2 * i))
    assert bm.rate == pytest.approx(0.2, abs=0.01)


# ----------------------------------------------------------------- patterns
@pytest.mark.parametrize("name", list(describe()))
def test_patterns_valid(name):
    m = generate(name, {"detect_plugin": "red_triangle"})
    validate(m)
    p = preview(m)
    assert p["distance_m"] > 1
    assert m["steps"][0]["type"] == "takeoff" and m["steps"][-1]["type"] == "land"


def test_expanding_square_geometry():
    m = generate("expanding_square", {"leg_m": 1.0, "legs": 4, "takeoff": False, "finish": "none", "turn": "left"})
    pts = preview(m)["points"]
    xy = [(round(p[0], 2), round(p[1], 2)) for p in pts]
    assert xy[:5] == [(0, 0), (1, 0), (1, 1), (-1, 1), (-1, -1)]


def test_orbit_stays_on_circle():
    m = generate("orbit", {"radius_m": 1.5, "sides": 12, "takeoff": False, "finish": "none"})
    for p in preview(m)["points"]:
        assert math.dist(p[:2], (1.5, 0)) == pytest.approx(1.5, abs=0.1)


# ----------------------------------------------------------------- coverage
def test_coverage_wedge(settings):
    bus = _Bus()
    g = CoverageGrid(bus, settings)
    g.update("d", {"x": 0, "y": 0, "yaw": 0}, True)
    area = g.area()
    expected = 0.5 * math.radians(70) * 3.0 ** 2      # sector area
    assert area == pytest.approx(expected, rel=0.25)
    cells = g.snapshot()["cells"]
    assert all(i >= -1 for i, j, c in cells)          # nothing behind the camera
    g.update("d", {"x": 0, "y": 0, "yaw": 0}, False)  # not flying -> no change
    assert g.area() == area


# ----------------------------------------------------------------- dataset
def test_rot_to_quat_identity_and_yaw():
    assert np.allclose(rot_to_quat(np.eye(3)), (1, 0, 0, 0))
    a = math.radians(90)
    R = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
    q = rot_to_quat(R)
    assert np.allclose(q, (math.cos(a / 2), 0, 0, math.sin(a / 2)), atol=1e-9)


def test_write_dataset(tmp_path):
    import json
    cam = Camera()
    frames = [{"file": f"kf_{i:06d}.jpg", "t": i, "pose": {"x": i * 0.3, "y": 0, "z": 1, "yaw": 10 * i}, "sharpness": 99}
              for i in range(1, 4)]
    write_dataset(tmp_path, frames, cam)
    tf = json.loads((tmp_path / "transforms.json").read_text())
    T = np.array(tf["frames"][1]["transform_matrix"])
    assert np.allclose(T[:3, 3], [0.6, 0, 1])
    # OpenGL camera looks down -z: -T[:3,2] must be the drone's forward direction
    assert np.allclose(-T[:3, 2], [math.cos(math.radians(20)), math.sin(math.radians(20)), 0], atol=1e-6)
    lines = [l for l in (tmp_path / "colmap/images.txt").read_text().splitlines() if l and not l.startswith("#")]
    assert len(lines) == 3


def test_new_settings_defaults_present():
    for sec in ("camera", "targets", "navigation", "coverage", "keyframes"):
        assert sec in DEFAULT_SETTINGS
