"""Configuration: static config from TOML + runtime-editable settings (JSON).

Static (config.toml):   server host/port, drone list & network ports, folders.
Runtime (settings.json): stick limit, dead-man, link timeout, video, recording —
                         editable from the web UI and persisted to disk.
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
from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import Any

# Project root = where config.toml, data/ and recordings/ live.
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


@dataclass
class AppConfig:
    host: str = "127.0.0.1"
    port: int = 8080
    open_browser: bool = True
    data_dir: Path = ROOT / "data"
    recordings_dir: Path = ROOT / "recordings"
    drones: list[DroneConfig] = field(default_factory=lambda: [DroneConfig()])
    sim: bool = False


DEFAULT_SETTINGS: dict[str, Any] = {
    "safety": {
        "stick_max": 60,               # % - pilot sticks are clamped to +-this on the GCS before they reach the drone
        "deadman_ms": 400,             # no stick input for this long -> sticks zero (= hover)
    },
    "rc": {
        "rate_hz": 20,                 # rc packet rate while sticks are moving
    },
    "link": {
        "timeout_s": 2.0,              # no state packet for this long -> link shown as lost
    },
    "video": {
        "jpeg_quality": 75,
        "max_fps": 30,
        "relay_udp": [],               # e.g. ["127.0.0.1:5000"] -> raw H.264 forwarded for SLAM / ROS nodes
    },
    "recording": {
        "log_on_connect": True,        # telemetry/event log runs while connected
    },
}


def _known(base: dict, data: dict) -> dict:
    """Keep only the keys DEFAULT_SETTINGS knows (old settings files carry removed options)."""
    return {k: (_known(base[k], v) if isinstance(base[k], dict) and isinstance(v, dict) else v)
            for k, v in (data or {}).items() if k in base}


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
        self._data = _deep_merge(DEFAULT_SETTINGS, _known(DEFAULT_SETTINGS, data))

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
            self._data = _deep_merge(self._data, _known(DEFAULT_SETTINGS, patch))
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
        for key in ("data_dir", "recordings_dir"):
            if key in paths:
                p = Path(paths[key])
                setattr(cfg, key, p if p.is_absolute() else ROOT / p)
        if raw.get("drones"):
            known = {f.name for f in fields(DroneConfig)}  # ignore keys of removed features (home_x ...)
            cfg.drones = [DroneConfig(**{k: v for k, v in d.items() if k in known}) for d in raw["drones"]]
    if sim:
        # Simulated drones live on localhost with unique ports so several can run at once.
        palette = ["#38bdf8", "#f472b6", "#a3e635", "#fbbf24"]
        cfg.drones = [
            DroneConfig(
                id=f"sim{i + 1}", name=f"SimTello {i + 1}", ip="127.0.0.1",
                cmd_port=18889 + i * 10, bind_ip="127.0.0.1", local_cmd_port=28889 + i * 10,
                state_port=28890 + i * 10, video_port=31111 + i * 10, color=palette[i % len(palette)],
            )
            for i in range(sim_count)
        ]
    return cfg
