"""Entry point for frozen (PyInstaller) builds: same CLI as ``python -m pawn_gcs``."""
from pawn_gcs.__main__ import main

if __name__ == "__main__":
    main()
