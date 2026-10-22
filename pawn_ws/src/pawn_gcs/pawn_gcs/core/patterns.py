"""Search-pattern mission generators (IAMSAR-style, scaled down to indoor Tello flight).

Every generator returns a normal mission dict (``missions/*.json`` format), so
the result can be previewed, edited step by step, saved and run like any
hand-made mission. Legs are flown as *rotate + forward* so the forward camera
always looks along the search track.

Patterns
  expanding_square  datum-centred square spiral, legs s, s, 2s, 2s, 3s ... (IAMSAR "SS")
  parallel_track    creeping-line / lawnmower over a W x L rectangle with track spacing S ("PS"/"CS")
  sector            three 120° triangles around the datum, rotated 30° each ("VS")
  orbit             polygon around a point R m ahead, always facing it (photogrammetry / 3D twin capture)
  scan_points       360° scan at each grid point (look around corners / doorways)
"""
from __future__ import annotations

import math

PATTERNS = {
    "expanding_square": {
        "title": "Expanding square (SS)", "params": {
            "leg_m": {"type": "float", "default": 1.0, "min": 0.3, "max": 5, "label": "first leg (m)"},
            "legs": {"type": "int", "default": 8, "min": 2, "max": 30, "label": "number of legs"},
            "turn": {"type": "select", "default": "right", "options": ["right", "left"], "label": "turn direction"}}},
    "parallel_track": {
        "title": "Parallel track / lawnmower (PS)", "params": {
            "length_m": {"type": "float", "default": 4.0, "min": 0.5, "max": 20, "label": "track length (m)"},
            "width_m": {"type": "float", "default": 3.0, "min": 0.5, "max": 20, "label": "area width (m)"},
            "spacing_m": {"type": "float", "default": 1.0, "min": 0.3, "max": 5, "label": "track spacing (m)"},
            "turn": {"type": "select", "default": "right", "options": ["right", "left"], "label": "first turn"}}},
    "sector": {
        "title": "Sector search (VS)", "params": {
            "radius_m": {"type": "float", "default": 2.0, "min": 0.5, "max": 10, "label": "radius (m)"},
            "turn": {"type": "select", "default": "right", "options": ["right", "left"], "label": "turn direction"}}},
    "orbit": {
        "title": "Orbit point (photogrammetry)", "params": {
            "radius_m": {"type": "float", "default": 1.5, "min": 0.5, "max": 6, "label": "radius (m)"},
            "sides": {"type": "int", "default": 12, "min": 4, "max": 36, "label": "polygon sides"},
            "dwell_s": {"type": "float", "default": 0.5, "min": 0, "max": 5, "label": "dwell per vertex (s)"},
            "snapshots": {"type": "bool", "default": True, "label": "snapshot at vertices"}}},
    "scan_points": {
        "title": "Grid of 360° scans", "params": {
            "rows": {"type": "int", "default": 2, "min": 1, "max": 6, "label": "rows"},
            "cols": {"type": "int", "default": 2, "min": 1, "max": 6, "label": "columns"},
            "spacing_m": {"type": "float", "default": 2.0, "min": 0.5, "max": 6, "label": "spacing (m)"},
            "scan_step": {"type": "int", "default": 45, "min": 15, "max": 120, "label": "scan step (deg)"}}},
}

COMMON = {
    "takeoff": {"type": "bool", "default": True, "label": "start with takeoff"},
    "alt_cm": {"type": "int", "default": 100, "min": 30, "max": 300, "label": "search altitude (cm)"},
    "speed_cms": {"type": "int", "default": 40, "min": 10, "max": 100, "label": "speed (cm/s)"},
    "detect_plugin": {"type": "str", "default": "", "label": "stop when plugin detects (empty = no)"},
    "finish": {"type": "select", "default": "land", "options": ["land", "hover", "none"], "label": "at the end"},
}


def describe() -> dict:
    return {k: {"title": v["title"], "params": {**v["params"], **COMMON}} for k, v in PATTERNS.items()}


def _move(m: float) -> list[dict]:
    cm, out = int(round(m * 100)), []
    while cm >= 20:
        c = min(cm, 500)
        if 0 < cm - c < 20:
            c = cm - 20
        out.append({"type": "move", "dir": "forward", "cm": c})
        cm -= c
    return out


def _turn(deg: float, right: bool) -> list[dict]:
    deg = int(round(deg))
    return [{"type": "rotate", "dir": "cw" if right else "ccw", "deg": deg}] if deg else []


def generate(pattern: str, params: dict | None = None) -> dict:
    if pattern not in PATTERNS:
        raise ValueError(f"unknown pattern {pattern!r}")
    spec = {**PATTERNS[pattern]["params"], **COMMON}
    p = {k: v["default"] for k, v in spec.items()}
    p.update({k: v for k, v in (params or {}).items() if k in spec})
    right = p.get("turn", "right") == "right"
    body: list[dict] = []
    plug = (p.get("detect_plugin") or "").strip()

    def leg(m):
        if plug:  # short check before each leg; mission continues if nothing is seen
            body.append({"type": "wait_detection", "plugin": plug, "timeout": 0.6, "on_timeout": "continue",
                         "note": "look"})
        body.extend(_move(m))

    if pattern == "expanding_square":
        s = float(p["leg_m"])
        for i in range(int(p["legs"])):
            leg(s * (i // 2 + 1))
            body += _turn(90, right)
    elif pattern == "parallel_track":
        L, W, S = float(p["length_m"]), float(p["width_m"]), float(p["spacing_m"])
        n = max(1, int(math.floor(W / S + 1e-6)) + 1)
        r = right
        for i in range(n):
            leg(L)
            if i < n - 1:
                body += _turn(90, r)
                body += _move(S)
                body += _turn(90, r)
                r = not r
    elif pattern == "sector":
        R = float(p["radius_m"])
        for k in range(3):
            for j in range(3):
                leg(R)
                body += _turn(120 if j < 2 else 150 if k < 2 else 120, right)
    elif pattern == "orbit":
        R, n = float(p["radius_m"]), int(p["sides"])
        a = 360.0 / n
        chord = 2 * R * math.sin(math.radians(a) / 2)
        # the object to orbit is R metres in FRONT of the drone; the drone keeps facing it
        for i in range(n):
            if p.get("snapshots"):
                body.append({"type": "snapshot"})
            if p.get("dwell_s"):
                body.append({"type": "hover", "s": float(p["dwell_s"])})
            body.append({"type": "rotate", "dir": "cw", "deg": int(round(a / 2))})
            body.append({"type": "move", "dir": "left", "cm": max(20, int(round(chord * 100)))})
            body.append({"type": "rotate", "dir": "cw", "deg": int(round(a / 2))})
    elif pattern == "scan_points":
        rows, cols, S = int(p["rows"]), int(p["cols"]), float(p["spacing_m"])
        r = right
        for i in range(rows):
            for j in range(cols):
                body.append({"type": "scan", "deg": 360, "step": int(p["scan_step"]), "dwell": 0.8,
                             "plugin": plug, "label": "", "stop_on_detect": bool(plug)})
                if j < cols - 1:
                    body += _move(S)
            if i < rows - 1:
                body += _turn(90, r)
                body += _move(S)
                body += _turn(90, r)
                r = not r

    steps: list[dict] = []
    if p.get("takeoff"):
        steps.append({"type": "takeoff"})
        extra = int(p["alt_cm"]) - 80  # Tello hovers at ~80 cm after takeoff
        if extra >= 20:
            steps.append({"type": "move", "dir": "up", "cm": min(extra, 500)})
        elif extra <= -20:
            steps.append({"type": "move", "dir": "down", "cm": min(-extra, 500)})
    steps.append({"type": "speed", "cms": int(p["speed_cms"])})
    steps += body
    if p["finish"] == "land":
        steps.append({"type": "land"})
    elif p["finish"] == "hover":
        steps.append({"type": "hover", "s": 2})
    title = PATTERNS[pattern]["title"]
    return {"name": f"{pattern}", "description": f"{title} - generated {', '.join(f'{k}={v}' for k, v in p.items())}",
            "steps": steps, "pattern": {"type": pattern, "params": p}}
