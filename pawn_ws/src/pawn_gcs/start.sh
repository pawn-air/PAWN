#!/usr/bin/env bash
# PAWN GCS launcher (Linux / macOS). Usage: ./start.sh [--sim [N]] [other args]
set -e
cd "$(dirname "$0")"
PY=${PYTHON:-python3}
if [ ! -d .venv ]; then
  echo "[pawn] creating virtualenv (.venv) ..."
  $PY -m venv .venv
  .venv/bin/python -m pip install --upgrade pip >/dev/null
  .venv/bin/python -m pip install -r requirements.txt
fi
exec .venv/bin/python -m pawn_gcs "$@"
