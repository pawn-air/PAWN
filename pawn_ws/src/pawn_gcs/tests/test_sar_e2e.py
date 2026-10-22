"""End-to-end PAWN search-and-rescue loop against the simulator:
search flight -> red triangle localised in 3D -> backtrack path -> RTH along breadcrumbs -> report + 3D dataset."""
import math
import time
import zipfile
import io

import pytest
from fastapi.testclient import TestClient

from pawn_gcs.config import AppConfig, DroneConfig
from pawn_gcs.server import create_app

TRIANGLE = (7.0, -0.9, 1.18)   # centroid of the sim's red triangle (+x wall)


def _wait(fn, timeout=10.0, every=0.1):
    end = time.time() + timeout
    while time.time() < end:
        v = fn()
        if v:
            return v
        time.sleep(every)
    return None


@pytest.fixture(scope="module")
def client(tmp_path_factory):
    base = tmp_path_factory.mktemp("sar")
    cfg = AppConfig(sim=True, data_dir=base / "data", recordings_dir=base / "rec", missions_dir=base / "missions",
                    extra_plugin_dirs=[])
    cfg.drones = [DroneConfig(id="s1", name="S1", ip="127.0.0.1", cmd_port=51889, bind_ip="127.0.0.1",
                              local_cmd_port=52889, state_port=52890, video_port=53111)]
    with TestClient(create_app(cfg)) as c:
        c.post("/api/settings", json={"safety": {"client_loss_action": "none", "smart_rth": "off"},
                                      "keyframes": {"min_sharpness": 1}})
        assert _wait(lambda: client_ready(c), 15)
        yield c


def client_ready(c):
    s = c.get("/api/status").json()["drones"][0]
    return s["connected"] and s["video_alive"]


def cmd(c, text):
    r = c.post("/api/drones/s1/command", json={"text": text}).json()
    assert r["ok"], (text, r)


def test_search_localises_target_in_3d(client):
    client.post("/api/vision/red_triangle/enable", json={"drone": "s1", "on": True})
    cmd(client, "takeoff")
    cmd(client, "up 40")
    # fly a short zig-zag so detections come from several positions (baseline for triangulation)
    for c in ("forward 150", "left 80", "forward 100", "right 160"):
        cmd(client, c)
        time.sleep(0.8)
    tg = _wait(lambda: [m for m in client.get("/api/map").json()["markers"]
                        if m["label"] == "red_triangle" and m["status"] == "confirmed"], 10)
    assert tg, "triangle should be confirmed"
    t = tg[0]
    err = math.dist((t["x"], t["y"], t["z"]), TRIANGLE)
    # dead reckoning + 1.2 m tall image detection -> sub-metre is the realistic target here
    assert err < 1.0, (t, err)
    assert t["sigma"] < 1.0 and t["method"] != "bearing"
    client.post("/api/vision/red_triangle/enable", json={"drone": "s1", "on": False})


def test_backtrack_and_rth(client):
    tg = [m for m in client.get("/api/map").json()["markers"] if m["label"] == "red_triangle"][0]
    bt = client.get(f"/api/targets/{tg['id']}/backtrack").json()
    assert len(bt["path"]) >= 2 and bt["length_m"] > 1.0
    assert math.hypot(*bt["path"][-1][:2]) < 0.3, "backtrack path must end at home"
    est = client.get("/api/drones/s1/rth").json()
    assert est["path_m"] > 1 and est["bat_needed"] > 0
    r = client.post("/api/drones/s1/rth", json={"land": True}).json()
    assert r["ok"] and r["steps"] > 3
    st = _wait(lambda: (s := client.get("/api/status").json()["drones"][0]) and not s["flying"] and s, 90, 0.3)
    assert st, "RTH should end with a landing"
    tr = client.get("/api/drones/s1/track").json()["track"]
    assert math.hypot(tr[-1][0], tr[-1][1]) < 0.5, tr[-1]


def test_operator_mark_triangulates(client):
    # the flight log closes 3 s after landing; a new takeoff before that would continue the same session
    assert _wait(lambda: [x for x in client.get("/api/recordings").json() if not x["open"]], 15)
    client.post("/api/map/clear", json={})
    cmd(client, "takeoff")
    a = client.post("/api/drones/s1/mark", json={"u": 0.6, "v": 0.5, "label": "victim"}).json()
    assert a["method"] == "bearing"
    cmd(client, "left 100")
    b = client.post("/api/drones/s1/mark", json={"u": 0.75, "v": 0.5, "label": "victim"}).json()
    assert b["id"] == a["id"] and b["hits"] == 2 and b["method"] == "triangulation"
    cmd(client, "land")


def test_coverage_and_patterns(client):
    cov = client.get("/api/coverage").json()
    assert cov["area_m2"] > 1.0
    pats = client.get("/api/patterns").json()
    assert {"expanding_square", "parallel_track", "sector", "orbit", "scan_points"} <= set(pats)
    m = client.post("/api/patterns/expanding_square", json={"leg_m": 0.5, "legs": 4}).json()
    assert m["steps"][0]["type"] == "takeoff" and m["steps"][-1]["type"] == "land"
    pv = client.post("/api/missions/preview", json=m).json()
    assert pv["error"] is None and pv["distance_m"] > 2


def test_report_and_dataset(client):
    sess = _wait(lambda: [s for s in client.get("/api/recordings").json() if not s["open"] and s["dataset"]], 15)
    assert sess, "closed session with a 3D dataset expected"
    sid = sess[-1]["id"]
    tf = client.get(f"/api/recordings/{sid}/transforms.json").json()
    assert tf["fl_x"] > 600 and len(tf["frames"]) >= 5
    assert len(tf["frames"][0]["transform_matrix"]) == 4
    rep = client.get(f"/api/recordings/{sid}/report.html")
    assert rep.status_code == 200 and "PAWN flight report" in rep.text and "red_triangle" in rep.text
    z = client.get(f"/api/recordings/{sid}/export.zip?what=dataset")
    names = zipfile.ZipFile(io.BytesIO(z.content)).namelist()
    assert any(n.endswith("colmap/images.txt") for n in names) and any(n.endswith(".jpg") for n in names)
