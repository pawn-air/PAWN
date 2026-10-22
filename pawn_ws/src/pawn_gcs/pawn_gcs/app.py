import os
import sys
import pathlib
import ctypes

os.environ["QT_API"] = "pyside6"

import PySide6
_pyside_dir = pathlib.Path(PySide6.__file__).resolve().parent
_plugins_dir = _pyside_dir / "Qt" / "plugins"
_platforms_dir = _plugins_dir / "platforms"
_lib_dir = _pyside_dir / "Qt" / "lib"

# 1. Pre-load Qt frameworks with RTLD_GLOBAL to guarantee all symbols resolve on macOS
for _fw in ["QtCore", "QtGui", "QtWidgets", "QtDBus", "QtOpenGL"]:
    _fw_file = _lib_dir / f"{_fw}.framework" / "Versions" / "A" / _fw
    if _fw_file.exists():
        try:
            ctypes.CDLL(str(_fw_file), mode=ctypes.RTLD_GLOBAL)
        except Exception:
            pass

# 2. Pre-load Cocoa platform plugin into memory
_cocoa_file = _platforms_dir / "libqcocoa.dylib"
if _cocoa_file.exists():
    try:
        ctypes.CDLL(str(_cocoa_file), mode=ctypes.RTLD_GLOBAL)
    except Exception as _e:
        print(f"[PAWN GCS] Preload cocoa error: {_e}", file=sys.stderr)

os.environ["QT_PLUGIN_PATH"] = str(_plugins_dir)
os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = str(_platforms_dir)

import PySide6.QtCore
import PySide6.QtGui
import PySide6.QtWidgets
from PySide6.QtWidgets import QApplication

def main():
    # 3. CRITICAL: Initialize QApplication FIRST before any other modules
    app = QApplication(sys.argv)

    # 4. Apply palette and import main window only after QApplication is active
    from pawn_gcs.theme import apply_palette
    apply_palette(app, "dark")

    from pawn_gcs.main_window import PawnGroundStation
    win = PawnGroundStation()
    win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()