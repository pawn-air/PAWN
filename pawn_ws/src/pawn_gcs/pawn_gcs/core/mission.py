"""Mission planning & execution.

A mission is JSON (saved in ``missions/``) with a list of steps. Steps map to
Tello SDK commands plus GCS-level actions (wait, snapshot, record, wait for a
detection, scan-for-target). The executor runs them one by one, can be
paused/resumed/aborted, and any pilot stick input aborts it (pilot override).

Step types
----------
takeoff | land
move      {dir: forward|back|left|right|up|down, cm: 20..500}
rotate    {dir: cw|ccw, deg: 1..360}
go        {x, y, z (cm, body frame), speed (10..100)}
speed     {cms: 10..100}
flip      {dir: f|b|l|r}
wait      {s}
hover     {s}                      - rc 0 0 0 0 for s seconds
snapshot  {}
record    {on: bool}
wait_detection {plugin, label?, timeout, on_timeout: continue|abort|land}
scan      {deg: 360, step: 30, dwell: 1.0, plugin?, label?, stop_on_detect: true}
command   {text}                   - raw SDK command (escape hatch)
"""
from __future__ import annotations

import asyncio
import json
import math
import time
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .node import DroneNode

DIRS = {"forward", "back", "left", "right", "up", "down"}


class MissionError(ValueError):
    pass


def validate(mission: dict) -> list[dict]:
    steps = mission.get("steps")
    if not isinstance(steps, list) or not steps:
        raise MissionError("mission has no steps")
    out = []
    for i, st in enumerate(steps):
        t = st.get("type")
        try:
            if t == "move":
                assert st["dir"] in DIRS and 20 <= int(st["cm"]) <= 500
            elif t == "rotate":
                assert st["dir"] in ("cw", "ccw") and 1 <= int(st["deg"]) <= 360
            elif t == "go":
                assert all(-500 <= int(st[k]) <= 500 for k in "xyz") and 10 <= int(st.get("speed", 50)) <= 100
                assert max(abs(int(st[k])) for k in "xyz") >= 20
            elif t == "speed":
                assert 10 <= int(st["cms"]) <= 100
            elif t == "flip":
                assert st["dir"] in ("f", "b", "l", "r")
            elif t in ("wait", "hover"):
                assert 0 <= float(st["s"]) <= 600
            elif t == "wait_detection":
                assert st.get("plugin")
            elif t == "scan":
                assert 1 <= int(st.get("step", 30)) <= 360
            elif t == "command":
                assert str(st.get("text", "")).strip()
            elif t not in ("takeoff", "land", "snapshot", "record"):
                raise AssertionError(f"unknown type {t!r}")
        except (AssertionError, KeyError, ValueError, TypeError) as e:
            raise MissionError(f"step {i + 1} ({t}): invalid {e or ''}".strip()) from None
        out.append(st)
    return out


def to_command(st: dict) -> str | None:
    t = st["type"]
    if t in ("takeoff", "land"):
        return t
    if t == "move":
        return f"{st['dir']} {int(st['cm'])}"
    if t == "rotate":
        return f"{st['dir']} {int(st['deg'])}"
    if t == "go":
        return f"go {int(st['x'])} {int(st['y'])} {int(st['z'])} {int(st.get('speed', 50))}"
    if t == "speed":
        return f"speed {int(st['cms'])}"
    if t == "flip":
        return f"flip {st['dir']}"
    if t == "command":
        return str(st["text"]).strip()
    return None


def preview(mission: dict) -> dict:
    """Kinematic preview: list of 3D waypoints (m) + total distance / est. time."""
    x = y = z = 0.0
    yaw = 0.0  # CCW+ deg, 0 = initial forward
    speed = 0.5  # m/s default for moves
    pts = [(0.0, 0.0, 0.0)]
    dist = 0.0
    t = 0.0
    labels = []
    for i, st in enumerate(mission.get("steps", [])):
        typ = st.get("type")
        nx, ny, nz = x, y, z
        if typ == "takeoff":
            nz = 0.8
            t += 5
        elif typ == "land":
            nz = 0.0
            t += 4
        elif typ == "move":
            d = int(st.get("cm", 0)) / 100
            fx, fy, fz = {"forward": (1, 0, 0), "back": (-1, 0, 0), "left": (0, 1, 0), "right": (0, -1, 0),
                          "up": (0, 0, 1), "down": (0, 0, -1)}.get(st.get("dir"), (0, 0, 0))
            c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
            nx, ny, nz = x + (c * fx - s * fy) * d, y + (s * fx + c * fy) * d, z + fz * d
        elif typ == "go":
            c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
            gx, gy, gz = int(st.get("x", 0)) / 100, int(st.get("y", 0)) / 100, int(st.get("z", 0)) / 100
            nx, ny, nz = x + c * gx - s * gy, y + s * gx + c * gy, z + gz
        elif typ == "rotate":
            yaw += int(st.get("deg", 0)) * (-1 if st.get("dir") == "cw" else 1)
            t += abs(int(st.get("deg", 0))) / 60
        elif typ == "speed":
            speed = int(st.get("cms", 50)) / 100
        elif typ in ("wait", "hover"):
            t += float(st.get("s", 0))
        elif typ == "scan":
            t += 360 / max(1, int(st.get("step", 30))) * (float(st.get("dwell", 1)) + 1)
        elif typ == "wait_detection":
            t += float(st.get("timeout", 10))
        seg = math.dist((x, y, z), (nx, ny, nz))
        if seg > 0:
            dist += seg
            t += seg / max(speed, 0.1) + 0.5
            pts.append((round(nx, 3), round(ny, 3), round(nz, 3)))
            labels.append(i)
        x, y, z = nx, ny, nz
    rng = max((math.hypot(p[0], p[1]) for p in pts), default=0)
    alt = max((p[2] for p in pts), default=0)
    return {"points": pts, "step_index": labels, "distance_m": round(dist, 2), "est_time_s": round(t, 1),
            "max_range_m": round(rng, 2), "max_alt_m": round(alt, 2)}


class MissionExecutor:
    def __init__(self, node: "DroneNode") -> None:
        self.node = node
        self.mission: dict | None = None
        self.state = "idle"     # idle | running | paused | done | aborted | failed
        self.step = -1
        self.msg = ""
        self._task: asyncio.Task | None = None
        self._resume = asyncio.Event()
        self._resume.set()
        self.t0 = 0.0

    def status(self) -> dict:
        return {"state": self.state, "step": self.step, "msg": self.msg,
                "name": (self.mission or {}).get("name"), "total": len((self.mission or {}).get("steps", [])),
                "elapsed": round(time.monotonic() - self.t0, 1) if self.state in ("running", "paused") else 0}

    def _pub(self) -> None:
        self.node.bus.publish("mission", {"drone": self.node.id, **self.status()})

    def start(self, mission: dict) -> None:
        if self.state in ("running", "paused"):
            raise MissionError("a mission is already running")
        validate(mission)
        if not self.node.connected:
            raise MissionError("drone not connected")
        self.mission = mission
        self.state, self.step, self.msg = "running", -1, ""
        self.t0 = time.monotonic()
        self._resume.set()
        self.node.set_mode("MISSION", reason=f"mission '{mission.get('name', '?')}'")
        self._task = asyncio.ensure_future(self._run())
        self._pub()

    def pause(self) -> None:
        if self.state == "running":
            self.state = "paused"
            self._resume.clear()
            self.node.link.send_raw("rc 0 0 0 0")
            self.node.bus.event("Mission paused (after current step)", "info", self.node.id, src="mission")
            self._pub()

    def resume(self) -> None:
        if self.state == "paused":
            self.state = "running"
            self._resume.set()
            self._pub()

    def abort(self, reason: str = "aborted") -> None:
        if self.state not in ("running", "paused"):
            return
        try:
            if asyncio.current_task() is self._task:
                return  # side effect of the mission's own step (e.g. its 'land')
        except RuntimeError:
            pass
        self.state, self.msg = "aborted", reason
        if self._task and not self._task.done():
            self._task.cancel()
        self.node.bus.event(f"Mission aborted: {reason}", "warn", self.node.id, src="mission")
        if self.node.mode == "MISSION":
            self.node.set_mode("MANUAL", reason="mission aborted")
        self._pub()

    async def _run(self) -> None:
        n = self.node
        steps = self.mission["steps"]
        try:
            for i, st in enumerate(steps):
                await self._resume.wait()
                self.step = i
                self.msg = st.get("note") or st.get("type")
                self._pub()
                ok = await self._exec(st)
                if not ok:
                    self.state = "failed"
                    n.bus.event(f"Mission failed at step {i + 1} ({st.get('type')}) - holding", "error", n.id, src="mission")
                    await n.hover("mission failed")
                    self._pub()
                    return
            self.state, self.msg = "done", "completed"
            n.bus.event(f"Mission '{self.mission.get('name', '')}' completed", "success", n.id, src="mission")
        except asyncio.CancelledError:
            return
        finally:
            if n.mode == "MISSION":
                n.set_mode("MANUAL", reason="mission end")
            self._pub()

    async def _exec(self, st: dict) -> bool:
        n, t = self.node, st["type"]
        cmd = to_command(st)
        if cmd:
            r = await n.cmd(cmd, source="mission")
            if not r.ok and t not in ("land",):
                r = await n.cmd(cmd, source="mission")  # one retry
            return r.ok or t == "land"
        if t == "wait":
            await asyncio.sleep(float(st["s"]))
            return True
        if t == "hover":
            end = time.monotonic() + float(st["s"])
            while time.monotonic() < end:
                n.link.send_raw("rc 0 0 0 0")
                await asyncio.sleep(0.2)
            return True
        if t == "snapshot":
            if n.recorder:
                n.recorder.snapshot(tag="mission")
            return True
        if t == "record":
            if n.recorder:
                (n.recorder.start_video if st.get("on", True) else n.recorder.stop_video)()
            return True
        if t == "wait_detection":
            found = await self._wait_det(st.get("plugin"), st.get("label"), float(st.get("timeout", 10)))
            if found:
                n.bus.event(f"Detection '{st.get('label') or st.get('plugin')}' confirmed", "success", n.id, src="mission")
                return True
            act = st.get("on_timeout", "continue")
            n.bus.event(f"wait_detection timed out -> {act}", "warn", n.id, src="mission")
            if act == "land":
                await n.cmd("land", source="mission")
                return True
            return act == "continue"
        if t == "scan":
            step, total = int(st.get("step", 30)), int(st.get("deg", 360))
            done = 0
            while done < total:
                if st.get("plugin") and st.get("stop_on_detect", True):
                    if await self._wait_det(st["plugin"], st.get("label"), float(st.get("dwell", 1.0))):
                        n.bus.event(f"Scan: target found after {done}°", "success", n.id, src="mission")
                        return True
                else:
                    await asyncio.sleep(float(st.get("dwell", 1.0)))
                r = await n.cmd(f"ccw {min(step, total - done)}", source="mission")
                if not r.ok:
                    return False
                done += step
            return True
        return False

    async def _wait_det(self, plugin: str, label: str | None, timeout: float) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            d = self.node.detections.get(plugin)
            if d and time.time() - d["t"] < 0.6:
                items = d.get("items", [])
                if any((not label) or it.get("label") == label for it in items):
                    return True
            await asyncio.sleep(0.1)
        return False


class MissionStore:
    def __init__(self, folder: Path) -> None:
        self.folder = folder
        folder.mkdir(parents=True, exist_ok=True)

    def list(self) -> list[dict]:
        out = []
        for p in sorted(self.folder.glob("*.json")):
            try:
                m = json.loads(p.read_text("utf-8"))
                out.append({"file": p.name, "name": m.get("name", p.stem), "steps": len(m.get("steps", [])),
                            "description": m.get("description", "")})
            except Exception:
                continue
        return out

    def load(self, file: str) -> dict:
        p = (self.folder / Path(file).name)
        return json.loads(p.read_text("utf-8"))

    def save(self, mission: dict) -> str:
        validate(mission)
        name = "".join(c if c.isalnum() or c in "-_" else "_" for c in mission.get("name", "mission"))[:60] or "mission"
        p = self.folder / f"{name}.json"
        p.write_text(json.dumps(mission, indent=2), "utf-8")
        return p.name

    def delete(self, file: str) -> None:
        p = self.folder / Path(file).name
        if p.exists():
            p.unlink()
