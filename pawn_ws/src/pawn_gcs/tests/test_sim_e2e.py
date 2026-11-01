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
    cfg = AppConfig(sim=True, data_dir=base / "data", recordings_dir=base / "rec")
    cfg.drones = [DroneConfig(id="t1", name="T1", ip="127.0.0.1", cmd_port=41889, bind_ip="127.0.0.1",
                              local_cmd_port=42889, state_port=42890, video_port=43111)]
    with TestClient(create_app(cfg)) as c:
        yield c


def _st(client):
    return client.get("/api/status").json()["drones"][0]


def _sim(client):
    return client.app.state.gcs.sims[0]


def _cmds(client):
    return [e["cmd"] for e in client.get("/api/events").json() if e["type"] == "command"]


def test_connects_and_streams(client):
    st = _wait(lambda: (s := _st(client))["connected"] and s["state_hz"] > 5 and s)
    assert st, "simulator did not connect"
    assert _wait(lambda: client.get("/api/drones/t1/frame.jpg").status_code == 200, 15), "no decoded video"
    st = _st(client)
    assert st["video_size"] == [960, 720] and st["info"]["sdk"] == "20"


def test_no_rc_while_motors_are_off(client):
    sim = _sim(client)
    t0 = sim.rc_t
    time.sleep(1.5)
    assert sim.rc_t == t0, "rc packets must not be sent to a drone on the ground"
    assert not _st(client)["motors_running"]


def test_commands_go_through_unchanged(client):
    assert client.post("/api/drones/t1/command", json={"text": "takeoff"}).json()["ok"]
    assert _sim(client).flying
    assert client.post("/api/drones/t1/command", json={"text": "forward 50"}).json()["ok"]
    r = client.post("/api/drones/t1/command", json={"text": "flip x"}).json()
    assert r["ok"] is False and r["resp"].startswith("error")    # the drone's reply, not a GCS opinion


def test_sticks_are_clamped_and_deadman_hovers(client):
    client.post("/api/settings", json={"safety": {"stick_max": 30}})
    gcs = client.app.state.gcs
    gcs.nodes["t1"].set_pilot_sticks(100, -100, 0, 50)
    assert _wait(lambda: _sim(client).rc == (30, -30, 0, 30), 2)
    # no more input -> after the dead-man time the GCS itself sends zero sticks
    assert _wait(lambda: _sim(client).rc == (0, 0, 0, 0), 2)
    client.post("/api/settings", json={"safety": {"stick_max": 60}})


def test_released_stick_stops_even_while_a_command_runs(client):
    """The drone keeps the last rc values: releasing a key must send zero at once, not after the move's reply."""
    import threading
    sim, node = _sim(client), client.app.state.gcs.nodes["t1"]
    th = threading.Thread(target=lambda: client.post("/api/drones/t1/command", json={"text": "forward 500"}))
    th.start()
    assert _wait(lambda: node.link.busy, 2)
    node.set_pilot_sticks(0, 0, 0, 50)
    assert _wait(lambda: sim.rc == (0, 0, 0, 50), 1)
    # key released: no more stick packets -> dead-man -> zero, while "forward 500" still waits for its reply
    assert _wait(lambda: sim.rc == (0, 0, 0, 0), 1.0) and node.link.busy
    th.join(30)


def test_keepalive_keeps_hovering_and_gcs_never_lands_by_itself(client):
    sim = _sim(client)
    sim.battery = 8                       # would have triggered the old GCS battery auto-land
    time.sleep(17)                        # the simulated Tello, like the real one, lands after 15 s without commands
    assert sim.flying, "keepalive rc must keep the drone in the air"
    assert "land" not in _cmds(client)
    assert client.post("/api/drones/t1/command", json={"text": "land"}).json()["ok"]
    assert not sim.flying
    sim.battery = 90


def test_takeoff_is_never_blocked_by_the_gcs(client):
    sim = _sim(client)
    sim.battery = 5                       # the sim refuses below 10 % - that reply must reach the operator as-is
    r = client.post("/api/drones/t1/command", json={"text": "takeoff"}).json()
    assert r["resp"] == "error Not enough battery"
    sim.battery = 90


def test_emergency_is_sent_immediately(client):
    assert client.post("/api/drones/t1/command", json={"text": "takeoff"}).json()["ok"]
    assert client.post("/api/drones/t1/emergency").json()["ok"]
    assert _wait(lambda: not _sim(client).flying, 2)


def test_log_session_is_raw(client):
    s = _wait(lambda: [x for x in client.get("/api/recordings").json() if x["open"]], 3)
    assert s, "a log runs while connected"
    sid = s[0]["id"]
    client.post("/api/drones/t1/record", json={"what": "log", "on": False})
    rows = client.get(f"/api/recordings/{sid}/telemetry.csv").text.splitlines()
    assert rows[0].startswith("t,pitch,roll,yaw") and rows[0].endswith("rc_lr,rc_fb,rc_ud,rc_yw,raw")
    assert "pitch:" in rows[-1] and len(rows) > 20
    ev = client.get(f"/api/recordings/{sid}/events.jsonl").text
    assert '"cmd": "takeoff"' in ev and '"cmd": "rc ' not in ev
    wire = client.get(f"/api/recordings/{sid}/udp.log").text
    assert " TX takeoff" in wire and " RX ok" in wire and " TX rc " in wire


def test_log_button_stays_off_while_connected(client):
    client.post("/api/drones/t1/record", json={"what": "log", "on": False})
    time.sleep(2.5)                       # several 1 Hz link updates
    assert not _st(client)["recording"]["logging"]


def test_websocket_hello_and_ping(client):
    with client.websocket_connect("/ws") as ws:
        hello = ws.receive_json()
        assert hello["topic"] == "hello" and hello["data"]["drones"][0]["id"] == "t1"
        assert set(hello["data"]["settings"]) == {"safety", "rc", "link", "video", "recording"}
        ws.send_json({"type": "ping", "t": 1})
        for _ in range(200):
            m = ws.receive_json()
            if m["topic"] == "pong":
                break
        assert m["topic"] == "pong"
