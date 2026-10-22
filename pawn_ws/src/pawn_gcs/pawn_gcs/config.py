"""Configuration: static config from TOML + runtime-editable settings (JSON).

Static (config.toml):   server host/port, drone list & network ports, folders.
Runtime (settings.json): safety thresholds, RC tuning, odometry, video — editable
                         from the web UI Settings tab and persisted to disk.
"""
from __future__ import annotations

import copy
import json
import sys
import threading
try:
    import tomllib
except ModuleNotFoundError:  # Python 3.10
    import tomli as tomllib
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

# Project root = where config.toml, missions/, plugins/, data/ and recordings/ live.
# In a PyInstaller build it is the folder of the executable (writable, next to the app).
ROOT = Path(sys.executable).resolve().parent if getattr(sys, "frozen", False) else Path(__file__).resolve().parent.parent


@dataclass
class DroneConfig:
    id: str = "tello1"
    name: str = "Tello 1"
    ip: str = "192.168.10.1"         # drone address (normal Tello in AP mode is always this)
    cmd_port: int = 8889             # drone command port
    bind_ip: str = "0.0.0.0"         # local NIC to bind (use a 2nd Wi-Fi adapter IP for tandem flight)
    local_cmd_port: int = 8889       # local port we send commands from
    state_port: int = 8890           # local port the drone sends state to
    video_port: int = 11111          # local port the drone sends H.264 to
    color: str = "#38bdf8"           # UI accent colour for this drone
    home_x: float = 0.0              # where this drone takes off in the shared map frame (m) -
    home_y: float = 0.0              #   lets tandem drones share one map
    home_yaw: float = 0.0            # initial heading in the shared map frame (deg, CCW+)


@dataclass
class AppConfig:
    host: str = "127.0.0.1"
    port: int = 8080
    open_browser: bool = True
    data_dir: Path = ROOT / "data"
    recordings_dir: Path = ROOT / "recordings"
    missions_dir: Path = ROOT / "missions"
    extra_plugin_dirs: list[Path] = field(default_factory=lambda: [ROOT / "plugins"])
    drones: list[DroneConfig] = field(default_factory=lambda: [DroneConfig()])
    sim: bool = False


DEFAULT_SETTINGS: dict[str, Any] = {
    "safety": {
        "battery_warn": 30,            # %
        "battery_land": 15,            # % -> automatic landing
        "temp_warn": 85,               # deg C (temph)
        "link_timeout_s": 2.0,         # no state packets -> link lost
        "geofence_enabled": True,
        "geofence_radius_m": 8.0,      # horizontal radius from home (dead reckoning / external pose)
        "geofence_max_alt_m": 3.0,
        "geofence_action": "hover",    # hover | land
        "client_loss_action": "hover", # last browser disconnected while flying: hover | land | none (API/scripts only)
        "client_loss_land_s": 10.0,
        "require_preflight": True,
        "collision_guard": True,       # brake when collision_ttc plugin reports time-to-contact < threshold
        "dynamic_obstacle_brake": False,  # also brake for large moving blobs ('motion' plugin) in the flight path
        "smart_rth": "warn",           # battery needed to fly home along the trail: off | warn | rth
    },
    "rc": {
        "rate_hz": 20,
        "max": 60,                     # stick scale limit (0-100) for keyboard / gamepad
        "expo": 0.3,                   # gamepad expo curve
        "deadzone": 0.08,
        "deadman_ms": 400,             # stick packets older than this -> sticks zero
    },
    "odometry": {
        "velocity_scale_cm": 10.0,     # Tello vgx/vgy/vgz units -> cm/s (firmware reports dm/s)
        "velocity_frame": "world",     # world | body
    },
    "video": {
        "jpeg_quality": 75,
        "max_fps": 30,
        "relay_udp": [],               # e.g. ["127.0.0.1:5000"] -> raw H.264 forwarded for SLAM / ROS nodes
    },
    "camera": {
        "hfov_deg": 70.0,              # Tello 960x720 stream ~70 deg horizontal (82.6 deg diagonal spec)
        "tilt_deg": 0.0,               # camera pitched down (deg); 0 for a stock Tello
    },
    "targets": {
        "known_height_m": {"red_triangle": 0.85, "person": 1.7, "aruco": 0},  # real heights -> range from pixel size
        "confirm_hits": 5,             # detections before a target is CONFIRMED
        "confirm_sigma_m": 1.5,        # ... and max 1-sigma position uncertainty
        "gate_m": 1.0,                 # association gate (ray-to-estimate distance)
        "max_obs_hz": 5.0,
        "min_conf": 0.3,
    },
    "navigation": {
        "speed_cms": 40,               # RTH / go-to-target speed
        "rdp_eps_m": 0.15,             # breadcrumb trail simplification
        "reserve_pct": 5,              # extra battery margin for smart RTH
        "default_drain_pct_s": 0.11,   # until learned in flight (Tello ~13 min hover)
    },
    "coverage": {
        "enabled": True,
        "cell_m": 0.25,
        "range_m": 3.0,                # how far the camera is assumed to "see" a target
    },
    "keyframes": {
        "enabled": True,               # 3D reconstruction dataset in every flight log
        "min_move_m": 0.25,
        "min_turn_deg": 12,
        "min_interval_s": 0.3,
        "max_interval_s": 4.0,
        "min_sharpness": 40,
    },
    "recording": {
        "auto_log_on_takeoff": True,
        "auto_video_on_takeoff": False,
    },
}


def _deep_merge(base: dict, upd: dict) -> dict:
    out = copy.deepcopy(base)
    for k, v in (upd or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = v
    return out


class Settings:
    """Thread-safe runtime settings with persistence."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self._lock = threading.Lock()
        data = {}
        if path.exists():
            try:
                data = json.loads(path.read_text("utf-8"))
            except Exception:
                data = {}
        self._data = _deep_merge(DEFAULT_SETTINGS, data)

    def get(self, dotted: str, default: Any = None) -> Any:
        cur: Any = self._data
        for part in dotted.split("."):
            if not isinstance(cur, dict) or part not in cur:
                return default
            cur = cur[part]
        return cur

    def all(self) -> dict:
        with self._lock:
            return copy.deepcopy(self._data)

    def update(self, patch: dict) -> dict:
        with self._lock:
            self._data = _deep_merge(self._data, patch)
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.path.write_text(json.dumps(self._data, indent=2), "utf-8")
            return copy.deepcopy(self._data)

    def reset(self) -> dict:
        with self._lock:
            self._data = copy.deepcopy(DEFAULT_SETTINGS)
            if self.path.exists():
                self.path.unlink()
            return copy.deepcopy(self._data)


def load_config(path: Path | None, sim: bool = False, sim_count: int = 1) -> AppConfig:
    cfg = AppConfig(sim=sim)
    if path and path.exists():
        raw = tomllib.loads(path.read_text("utf-8"))
        srv = raw.get("server", {})
        cfg.host = srv.get("host", cfg.host)
        cfg.port = int(srv.get("port", cfg.port))
        cfg.open_browser = bool(srv.get("open_browser", cfg.open_browser))
        paths = raw.get("paths", {})
        for key in ("data_dir", "recordings_dir", "missions_dir"):
            if key in paths:
                p = Path(paths[key])
                setattr(cfg, key, p if p.is_absolute() else ROOT / p)
        if "plugin_dirs" in paths:
            cfg.extra_plugin_dirs = [Path(p) if Path(p).is_absolute() else ROOT / p for p in paths["plugin_dirs"]]
        if raw.get("drones"):
            cfg.drones = [DroneConfig(**d) for d in raw["drones"]]
    if sim:
        # Simulated drones live on localhost with unique ports so several can run at once.
        palette = ["#38bdf8", "#f472b6", "#a3e635", "#fbbf24"]
        cfg.drones = [
            DroneConfig(
                id=f"sim{i + 1}", name=f"SimTello {i + 1}", ip="127.0.0.1",
                cmd_port=18889 + i * 10, bind_ip="127.0.0.1", local_cmd_port=28889 + i * 10,
                state_port=28890 + i * 10, video_port=31111 + i * 10, color=palette[i % len(palette)], home_y=1.5 * i,
            )
            for i in range(sim_count)
        ]
    return cfg
