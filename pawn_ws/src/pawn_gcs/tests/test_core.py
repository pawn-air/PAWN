"""Unit tests: protocol parsing, missions, odometry, settings. No network needed."""
from pathlib import Path

import pytest

from pawn_gcs.config import Settings
from pawn_gcs.core.mission import MissionError, preview, to_command, validate
from pawn_gcs.core.odometry import Odometry
from pawn_gcs.core.tello import parse_state


def test_parse_state_full_string():
    raw = ("pitch:1;roll:-2;yaw:45;vgx:3;vgy:0;vgz:-1;templ:60;temph:63;tof:85;h:70;bat:87;"
           "baro:112.34;time:12;agx:-10.00;agy:5.00;agz:-998.00;\r\n")
    st = parse_state(raw)
    assert st["pitch"] == 1 and st["roll"] == -2 and st["yaw"] == 45
    assert st["bat"] == 87 and st["h"] == 70 and st["baro"] == pytest.approx(112.34)
    assert len(st) == 16


def test_parse_state_edu_fields_and_garbage():
    st = parse_state("mid:-1;x:0;y:0;z:0;mpry:0,0,0;pitch:0;junk;bad:abc;")
    assert st["mid"] == -1 and st["mpry_p"] == 0 and "bad" not in st


def test_mission_validate_and_commands():
    m = {"steps": [{"type": "takeoff"}, {"type": "move", "dir": "forward", "cm": 100},
                   {"type": "rotate", "dir": "cw", "deg": 90}, {"type": "go", "x": 50, "y": 0, "z": 0, "speed": 30},
                   {"type": "wait", "s": 1}, {"type": "land"}]}
    validate(m)
    assert [to_command(s) for s in m["steps"]] == ["takeoff", "forward 100", "cw 90", "go 50 0 0 30", None, "land"]


@pytest.mark.parametrize("bad", [
    {"steps": []},
    {"steps": [{"type": "move", "dir": "forward", "cm": 10}]},     # < 20 cm
    {"steps": [{"type": "rotate", "dir": "spin", "deg": 90}]},
    {"steps": [{"type": "go", "x": 5, "y": 5, "z": 5, "speed": 50}]},  # all |v| < 20
    {"steps": [{"type": "teleport"}]},
])
def test_mission_validate_rejects(bad):
    with pytest.raises(MissionError):
        validate(bad)


def test_mission_preview_square():
    sq = {"steps": [{"type": "takeoff"}] + [s for _ in range(4) for s in (
        {"type": "move", "dir": "forward", "cm": 200}, {"type": "rotate", "dir": "ccw", "deg": 90})] + [{"type": "land"}]}
    p = preview(sq)
    end = p["points"][-1]
    assert abs(end[0]) < 1e-6 and abs(end[1]) < 1e-6 and end[2] == 0  # closed square, landed at home
    assert p["distance_m"] == pytest.approx(0.8 + 8 + 0.8, abs=1e-6)
    assert p["max_range_m"] == pytest.approx(2 * 2 ** 0.5, abs=0.01)


class _S:
    def __init__(self, d=None):
        self.d = d or {}

    def get(self, k, default=None):
        return self.d.get(k, default)


def test_odometry_integrates_world_velocity(monkeypatch):
    t = [100.0]
    monkeypatch.setattr("pawn_gcs.core.odometry.time.monotonic", lambda: t[0])
    od = Odometry(_S())
    for _ in range(11):  # 1 s at 10 Hz, vgx = 5 dm/s -> 0.5 m forward
        od.update({"vgx": 5, "vgy": 0, "h": 100, "yaw": 0})
        t[0] += 0.1
    p = od.pose()
    assert p["x"] == pytest.approx(0.5, abs=0.02) and p["z"] == pytest.approx(1.0)
    assert od.horizontal_range == pytest.approx(0.5, abs=0.02)


def test_odometry_home_offset_and_external_pose():
    od = Odometry(_S(), offset=(2.0, 1.0, 90.0))
    od.update({"yaw": 0, "h": 0})
    p = od.pose()
    assert p["x"] == pytest.approx(2.0) and p["y"] == pytest.approx(1.0) and p["yaw"] == pytest.approx(90.0)
    od.set_external({"x": 5, "y": 6, "z": 1, "yaw": 10})
    assert od.pose()["source"] == "external" and od.pose()["x"] == 5


def test_settings_merge_and_persist(tmp_path: Path):
    s = Settings(tmp_path / "s.json")
    assert s.get("safety.battery_land") == 15
    s.update({"safety": {"battery_land": 20}})
    s2 = Settings(tmp_path / "s.json")
    assert s2.get("safety.battery_land") == 20 and s2.get("safety.battery_warn") == 30
