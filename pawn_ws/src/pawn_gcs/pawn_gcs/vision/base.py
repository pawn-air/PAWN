"""Vision plugin API.

Write a plugin = subclass :class:`VisionPlugin`, drop the .py file into
``pawn_gcs/vision/plugins/`` or the top-level ``plugins/`` folder, restart.
It shows up in the Vision tab with its parameters auto-rendered as a form.

Detections use *normalised* image coordinates (0..1) so the UI can draw them
at any size:

    {"label": "red_triangle", "conf": 0.9,
     "box": [x, y, w, h],            # optional, normalised
     "poly": [[x, y], ...],          # optional, normalised
     "center": [cx, cy],             # optional, normalised
     "info": {...}}                  # optional, shown in tooltip / log
"""
from __future__ import annotations

from typing import Any

import numpy as np


class VisionPlugin:
    name: str = "base"              # unique id
    title: str = "Base plugin"
    description: str = ""
    color: str = "#22d3ee"          # overlay colour
    rate_hz: float = 10.0           # max processing rate
    is_target: bool = False         # detections are mission targets -> drop markers on the map
    params: dict[str, dict] = {}    # {"key": {"type": "int|float|bool|str|select", "default": ..., "min", "max", "step", "options", "label"}}

    def __init__(self, values: dict[str, Any] | None = None) -> None:
        self.values = {k: v.get("default") for k, v in self.params.items()}
        if values:
            self.update(values)
        self.available, self.unavailable_reason = True, ""
        try:
            self.setup()
        except Exception as e:  # missing optional dependency etc.
            self.available, self.unavailable_reason = False, str(e)

    # override -------------------------------------------------------------
    def setup(self) -> None:
        """Load models etc. Raise to mark the plugin unavailable."""

    def process(self, img: np.ndarray, ctx: dict) -> list[dict]:
        """Return detections for one BGR frame. ``ctx`` has drone state & pose."""
        return []

    # helpers --------------------------------------------------------------
    def update(self, values: dict[str, Any]) -> None:
        for k, v in values.items():
            spec = self.params.get(k)
            if not spec:
                continue
            t = spec.get("type", "float")
            try:
                v = {"int": int, "float": float, "bool": lambda x: x in (True, "true", "1", 1, "on"), "str": str,
                     "select": str}[t](v)
            except (KeyError, ValueError, TypeError):
                continue
            if t in ("int", "float"):
                if "min" in spec:
                    v = max(spec["min"], v)
                if "max" in spec:
                    v = min(spec["max"], v)
            self.values[k] = v

    @staticmethod
    def norm_box(x, y, w, h, W, H) -> list[float]:
        return [round(x / W, 4), round(y / H, 4), round(w / W, 4), round(h / H, 4)]

    @classmethod
    def describe(cls) -> dict:
        return {"name": cls.name, "title": cls.title, "description": cls.description, "color": cls.color,
                "rate_hz": cls.rate_hz, "is_target": cls.is_target, "params": cls.params}
