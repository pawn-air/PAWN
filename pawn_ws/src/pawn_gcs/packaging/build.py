"""Build a standalone PAWN GCS for the current OS (run once on each of Windows / macOS / Linux).

    pip install -r requirements.txt pyinstaller
    python packaging/build.py            ->  dist/PAWN-GCS/  +  dist/PAWN-GCS-<os>-<arch>.zip

The folder contains the executable plus config.toml, missions/ and plugins/ next to it, so the
team can edit settings and drop in vision plugins without Python installed. Start with
``PAWN-GCS --sim`` to try it without a drone.
"""
import platform
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DIST = ROOT / "dist" / "PAWN-GCS"


def main() -> None:
    subprocess.check_call([sys.executable, "-m", "PyInstaller", "--noconfirm", "--clean",
                           "--distpath", str(ROOT / "dist"), "--workpath", str(ROOT / "build"),
                           str(ROOT / "packaging" / "pawn_gcs.spec")])
    for item in ("config.toml", "README.md"):
        shutil.copy2(ROOT / item, DIST / item)
    for folder in ("missions", "plugins", "docs"):
        if (DIST / folder).exists():
            shutil.rmtree(DIST / folder)
        shutil.copytree(ROOT / folder, DIST / folder, ignore=shutil.ignore_patterns("__pycache__"))
    exe = "PAWN-GCS.exe" if sys.platform == "win32" else "PAWN-GCS"
    (DIST / ("start-sim.bat" if sys.platform == "win32" else "start-sim.sh")).write_text(
        f'@echo off\r\ncd /d "%~dp0"\r\n{exe} --sim %*\r\n' if sys.platform == "win32"
        else f'#!/bin/sh\ncd "$(dirname "$0")"\nexec ./{exe} --sim "$@"\n', "utf-8")
    if sys.platform != "win32":
        (DIST / "start-sim.sh").chmod(0o755)
    osname = {"win32": "windows", "darwin": "macos"}.get(sys.platform, "linux")
    archive = shutil.make_archive(str(ROOT / "dist" / f"PAWN-GCS-{osname}-{platform.machine().lower()}"), "zip",
                                  ROOT / "dist", "PAWN-GCS")
    print("built", DIST, "->", archive)


if __name__ == "__main__":
    main()
