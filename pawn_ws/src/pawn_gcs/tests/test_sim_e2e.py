"""End-to-end: real UDP driver + H.264 decoder + API, against the built-in Tello simulator."""
import time

import pytest
from fastapi.testclient import TestClient

from pawn_gcs.config import AppConfig, DroneConfig
from pawn_gcs.server import create_app


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
    base = tmp_path_factory.mktemp("gcs")
    cfg = AppConfig(sim=True, data_dir=base / "data", recordings_dir=base / "rec", missions_dir=base / "missions",
                    extra_plugin_dirs=[])
    cfg.drones = [DroneConfig(id="t1", name="T1", ip="127.0.0.1", cmd_port=41889, bind_ip="127.0.0.1",
                              local_cmd_port=42889, state_port=42890, video_port=43111)]
    with TestClient(create_app(cfg)) as c:
        c.post("/api/settings", json={"safety": {"client_loss_action": "none"}})
        yield c


def test_connects_and_streams(client):
    st = _wait(lambda: (s := client.get("/api/status").json()["drones"][0])["connected"] and s["state_hz"] > 5 and s)
    assert st, "simulator did not connect"
    assert _wait(lambda: client.get("/api/drones/t1/frame.jpg").status_code == 200, 15), "no decoded video"
    st = client.get("/api/status").json()["drones"][0]
    assert st["video_size"] == [960, 720]
    assert st["info"]["sdk"] == "20"


def test_preflight_and_flight(client):
    pf = client.get("/api/drones/t1/preflight").json()
    assert not pf["blockers"], pf
    r = client.post("/api/drones/t1/command", json={"text": "takeoff"}).json()
    assert r["ok"], r
    assert client.post("/api/drones/t1/command", json={"text": "forward 100"}).json()["ok"]
    assert client.post("/api/drones/t1/command", json={"text": "flip x"}).json()["ok"] is False  # bad arg -> error
    tr = _wait(lambda: (t := client.get("/api/drones/t1/track").json()["track"]) and t[-1][0] > 0.9 and t)
    assert tr and tr[-1][2] == pytest.approx(0.8, abs=0.1)


def test_vision_marks_red_triangle(client):
    client.post("/api/vision/red_triangle/enable", json={"drone": "t1", "on": True})
    m = _wait(lambda: [x for x in client.get("/api/map").json()["markers"] if x["label"] == "red_triangle"], 10)
    assert m, "red triangle (on the +x wall) should be seen from the start position"
    client.post("/api/vision/red_triangle/enable", json={"drone": "t1", "on": False})


def test_mission_runs_and_lands(client):
    mission = {"name": "e2e", "steps": [{"type": "move", "dir": "left", "cm": 50}, {"type": "rotate", "dir": "ccw", "deg": 45},
                                        {"type": "snapshot"}, {"type": "land"}]}
    assert client.post("/api/missions/preview", json=mission).json()["error"] is None
    assert client.post("/api/drones/t1/mission/start", json=mission).status_code == 200
    done = _wait(lambda: client.get("/api/status").json()["drones"][0]["mission"]["state"] in ("done", "failed"), 20)
    st = client.get("/api/status").json()["drones"][0]
    assert done and st["mission"]["state"] == "done", st["mission"]
    assert not st["flying"]


def test_log_session_written(client):
    sess = _wait(lambda: [s for s in client.get("/api/recordings").json() if not s["open"]], 8)
    assert sess, "auto flight log should close a few seconds after landing"
    s = sess[0]
    assert "telemetry.csv" in s["files"] and "events.jsonl" in s["files"] and s["snapshots"]
    csv = client.get(f"/api/recordings/{s['id']}/telemetry.csv").text.splitlines()
    assert csv[0].startswith("t,t_rel,pitch") and len(csv) > 20


def test_websocket_hello_and_ping(client):
    with client.websocket_connect("/ws") as ws:
        hello = ws.receive_json()
        assert hello["topic"] == "hello" and hello["data"]["drones"][0]["id"] == "t1"
        ws.send_json({"type": "ping", "t": 1})
        for _ in range(50):
            m = ws.receive_json()
            if m["topic"] == "pong":
                break
        assert m["topic"] == "pong"
