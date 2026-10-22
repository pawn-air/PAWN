"""Entry point:  python -m pawn_gcs [--sim [N]] [--config config.toml] [--host 0.0.0.0] [--port 8080]"""
from __future__ import annotations

import argparse
import logging
import threading
import time
import webbrowser
from pathlib import Path

import uvicorn

from .config import ROOT, load_config
from .server import create_app


def main() -> None:
    ap = argparse.ArgumentParser(prog="pawn_gcs", description="PAWN Ground Control Station for DJI Tello")
    ap.add_argument("--sim", nargs="?", const=1, type=int, default=0, metavar="N",
                    help="run with N simulated Tellos (default 1) instead of real hardware")
    ap.add_argument("--config", type=Path, default=ROOT / "config.toml")
    ap.add_argument("--host", help="bind address (0.0.0.0 to open the UI from a tablet/phone on the same network)")
    ap.add_argument("--port", type=int)
    ap.add_argument("--no-browser", action="store_true")
    ap.add_argument("--log-level", default="info")
    a = ap.parse_args()

    logging.basicConfig(level=a.log_level.upper(), format="%(asctime)s %(levelname)-5s %(name)s: %(message)s",
                        datefmt="%H:%M:%S")
    cfg = load_config(a.config, sim=bool(a.sim), sim_count=a.sim or 1)
    if a.host:
        cfg.host = a.host
    if a.port:
        cfg.port = a.port
    for p in (cfg.data_dir, cfg.recordings_dir, cfg.missions_dir):
        p.mkdir(parents=True, exist_ok=True)

    url = f"http://{'127.0.0.1' if cfg.host in ('0.0.0.0', '::') else cfg.host}:{cfg.port}"
    print(f"\n  PAWN Ground Control  ->  {url}   ({'SIMULATION' if cfg.sim else 'real Tello'})\n")
    if cfg.open_browser and not a.no_browser:
        threading.Thread(target=lambda: (time.sleep(1.5), webbrowser.open(url)), daemon=True).start()
    uvicorn.run(create_app(cfg), host=cfg.host, port=cfg.port, log_level="warning", ws_ping_interval=10)


if __name__ == "__main__":
    main()
