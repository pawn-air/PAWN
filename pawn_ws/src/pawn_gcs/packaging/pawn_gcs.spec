# PyInstaller spec - builds a self-contained PAWN GCS folder (no Python needed on the target PC).
#   pip install pyinstaller && python packaging/build.py
# Output: dist/PAWN-GCS/  (PAWN-GCS.exe on Windows, PAWN-GCS on Linux, PAWN-GCS on macOS)
# -*- mode: python ; coding: utf-8 -*-
from pathlib import Path
from PyInstaller.utils.hooks import collect_submodules, collect_data_files

ROOT = Path(SPECPATH).parent
hidden = (collect_submodules("pawn_gcs") + collect_submodules("uvicorn") + collect_submodules("av")
          + ["websockets", "httptools", "anyio._backends._asyncio"])
datas = [(str(ROOT / "pawn_gcs" / "web"), "pawn_gcs/web")] + collect_data_files("cv2")

a = Analysis([str(ROOT / "packaging" / "entry.py")], pathex=[str(ROOT)], binaries=[], datas=datas,
             hiddenimports=hidden, hookspath=[], runtime_hooks=[], excludes=["tkinter", "matplotlib", "torch",
             "ultralytics", "pytest", "playwright"], noarchive=False)
pyz = PYZ(a.pure)
exe = EXE(pyz, a.scripts, [], exclude_binaries=True, name="PAWN-GCS", console=True,
          icon=None, disable_windowed_traceback=False)
coll = COLLECT(exe, a.binaries, a.datas, strip=False, upx=False, name="PAWN-GCS")
