"""Collision guard end-to-end: fly at the simulator's +x wall, the looming (TTC) plugin must brake the drone."""
import time

import pytest
from fastapi.testclient import TestClient

from pawn_gcs.config import AppConfig, DroneConfig
from pawn_gcs.server import create_app


@pytest.fixture(scope="module")
def client(tmp_path_factory):
    base = tmp_path_factory.mktemp("col")
    cfg = AppConfig(sim=True, data_dir=base / "data", recordings_dir=base / "rec", missions_dir=base / "missions",
                    extra_plugin_dirs=[])
    cfg.drones = [DroneConfig(id="c1", name="C1", ip="127.0.0.1", cmd_port=61889, bind_ip="127.0.0.1",
                              local_cmd_port=62889, state_port=62890, video_port=63111)]
    with TestClient(create_app(cfg)) as c:
        c.post("/api/settings", json={"safety": {"client_loss_action": "none", "geofence_radius_m": 20}})
        end = time.time() + 15
        while time.time() < end and not c.get("/api/status").json()["drones"][0]["video_alive"]:
            time.sleep(0.2)
        yield c


def test_ttc_brakes_before_wall(client):
    cmd = lambda t: client.post("/api/drones/c1/command", json={"text": t}).json()  # noqa: E731
    client.post("/api/vision/collision_ttc/params", json={"ttc_warn_s": 2.0})
    client.post("/api/vision/collision_ttc/enable", json={"drone": "c1", "on": True})
    assert cmd("takeoff")["ok"]
    assert cmd("speed 60")["ok"]
    assert cmd("right 250")["ok"]             # y = -2.5: posters on the wall ahead, no pillar in the way
    assert cmd("forward 400")["ok"]           # x = 4.0 m, wall at 7 m
    r = cmd("forward 290")                     # would end at 6.9 m -> into the wall
    time.sleep(0.5)
    sim = client.app.state.gcs.sims[0]            # ground truth from the simulator (odometry drifts)
    evs = [e["msg"] for e in client.get("/api/events?limit=80").json() if "COLLISION GUARD" in e["msg"]]
    assert evs, "collision guard should have fired"
    assert sim.x < 6.5, f"drone should stop well short of the wall at 7 m, got x={sim.x:.2f} ({r}, {evs})"
    cmd("land")
