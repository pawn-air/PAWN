@echo off
REM PAWN GCS launcher (Windows). Usage: start.bat [--sim [N]] [other args]
cd /d "%~dp0"
if not exist .venv (
  echo [pawn] creating virtualenv ^(.venv^) ...
  py -3 -m venv .venv || python -m venv .venv
  .venv\Scripts\python -m pip install --upgrade pip
  .venv\Scripts\python -m pip install -r requirements.txt
)
.venv\Scripts\python -m pawn_gcs %*
