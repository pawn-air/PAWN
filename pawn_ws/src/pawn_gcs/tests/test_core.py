"""Unit tests: protocol parsing, settings, unsolicited drone replies. No network needed."""
import json
from pathlib import Path

import pytest

from pawn_gcs.config import DroneConfig, Settings, load_config
from pawn_gcs.core.tello import TelloLink, parse_state


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


def test_settings_merge_and_persist(tmp_path: Path):
    s = Settings(tmp_path / "s.json")
    assert s.get("safety.stick_max") == 60
    s.update({"safety": {"stick_max": 40}})
    s2 = Settings(tmp_path / "s.json")
    assert s2.get("safety.stick_max") == 40 and s2.get("safety.deadman_ms") == 400


def test_settings_drop_removed_options(tmp_path: Path):
    """settings.json from the full-featured version must not bring back auto-land & co."""
    p = tmp_path / "s.json"
    p.write_text(json.dumps({"safety": {"battery_land": 10, "battery_autoland": True, "stick_max": 30},
                             "odometry": {"velocity_frame": "world"}}))
    s = Settings(p)
    assert s.all()["safety"] == {"stick_max": 30, "deadman_ms": 400}
    assert "odometry" not in s.all()
    s.update({"geofence": {"on": True}})
    assert "geofence" not in s.all()


def test_old_config_toml_still_loads(tmp_path: Path):
    p = tmp_path / "config.toml"
    p.write_text('[paths]\nplugin_dirs = ["plugins"]\nmissions_dir = "missions"\n'
                 '[[drones]]\nid = "a"\nhome_x = 1.0\nhome_yaw = 0.0\n')
    cfg = load_config(p)
    assert cfg.drones[0].id == "a"


def test_unsolicited_and_late_replies_are_surfaced():
    got = []
    link = TelloLink(DroneConfig(), lambda *a: None, lambda *a: None, on_unsolicited=got.append)
    link._on_reply(b"error Auto land", None)       # nothing pending -> surfaced
    link._pending = "battery?"
    link._on_reply(b"87", None)                    # pending -> queued as the reply
    assert got == ["error Auto land"] and link._replies.get_nowait() == "87"


def test_binary_packets_are_never_taken_as_a_reply():
    got = []
    link = TelloLink(DroneConfig(), lambda *a: None, lambda *a: None, on_unsolicited=got.append)
    link._pending = "command"
    link._on_reply(bytes.fromhex("cc18010b9a560071010000"), None)   # Tello app protocol packet
    link._on_reply(b"ok", None)
    assert link._replies.get_nowait() == "ok" and link._replies.empty()
    assert got == ["binary packet, 11 bytes"]
