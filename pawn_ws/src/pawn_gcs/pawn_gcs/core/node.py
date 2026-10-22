"""DroneNode: one drone = link + video + odometry + control arbitration.

Control authority (highest first):
  1. Emergency / safety actions
  2. Pilot sticks (keyboard / gamepad / on-screen) - any non-zero pilot input
     immediately takes over from MISSION or AUTO ("pilot override")
  3. Mission executor (discrete SDK commands)
  4. AUTO controller (autonomy plugin producing rc sticks)
"""
from __future__ import annotations

import asyncio
import logging
import time
from typing import TYPE_CHECKING, Any

from ..config import DroneConfig
from .odometry import Odometry
from .tello import CommandResult, TelloLink
from .video import Frame, VideoPipeline

if TYPE_CHECKING:
    from .bus import EventBus

log = logging.getLogger("pawn.node")

MODES = ("MANUAL", "MISSION", "AUTO")
MOTION_CMDS = {"takeoff", "land", "up", "down", "left", "right", "forward", "back", "cw", "ccw", "flip", "go", "curve"}


class Sticks:
    __slots__ = ("lr", "fb", "ud", "yw", "t")

    def __init__(self) -> None:
        self.lr = self.fb = self.ud = self.yw = 0
        self.t = 0.0

    def set(self, lr, fb, ud, yw) -> None:
        c = lambda v: int(max(-100, min(100, round(float(v)))))  # noqa: E731
        self.lr, self.fb, self.ud, self.yw = c(lr), c(fb), c(ud), c(yw)
        self.t = time.monotonic()

    def fresh(self, ms: float) -> bool:
        return (time.monotonic() - self.t) * 1000 < ms

    def nonzero(self) -> bool:
        return any((self.lr, self.fb, self.ud, self.yw))

    def tuple(self):
        return self.lr, self.fb, self.ud, self.yw


class DroneNode:
    def __init__(self, cfg: DroneConfig, bus: "EventBus", settings) -> None:
        self.cfg = cfg
        self.id = cfg.id
        self.bus = bus
        self.settings = settings
        self.state: dict[str, float] = {}
        self.state_raw = ""
        self.odom = Odometry(settings, offset=(cfg.home_x, cfg.home_y, cfg.home_yaw))
        self.video = VideoPipeline(cfg.id, on_frame=self._on_frame)
        self.link = TelloLink(cfg, self._on_state, self.video.feed, self._on_command)
        self.connected = False          # SDK mode entered ("command" -> ok)
        self.flying = False
        self.mode = "MANUAL"
        self.pilot = Sticks()
        self.auto = Sticks()
        self.last_rc: tuple = (0, 0, 0, 0)
        self.info: dict[str, Any] = {}
        self.takeoff_t: float | None = None
        self.state_hz = 0.0
        self._state_times: list[float] = []
        self._tasks: list[asyncio.Task] = []
        self._link_alive = False
        self._vid_kbps = 0.0
        self._vid_bytes_prev = 0
        self.last_rtt: float | None = None
        # hooks set by the app (missions, safety, recorder, plugins, autonomy)
        self.mission = None
        self.safety = None
        self.recorder = None
        self.autonomy = None
        self.detections: dict[str, dict] = {}
        self.frame_listeners: list = []
        self.nav = None                 # Navigator (RTH / backtracking)
        self.nav_est: dict = {}         # latest RTH estimate (safety monitor, 1 Hz)
        self.coverage = None

    # ------------------------------------------------------------- lifecycle
    async def start(self) -> None:
        self.video.set_relay(self.settings.get("video.relay_udp", []))
        self.video.start()
        try:
            await self.link.open()
        except OSError:
            self.bus.event(self.link.open_error or "UDP open failed", "error", self.id)
        self._tasks = [asyncio.ensure_future(self._rc_loop()), asyncio.ensure_future(self._house_loop())]

    async def stop(self) -> None:
        for t in self._tasks:
            t.cancel()
        self.video.stop()
        await self.link.close()

    async def connect(self) -> bool:
        if not self.link.is_open:
            try:
                await self.link.open()
            except OSError:
                self.bus.event(self.link.open_error or "UDP open failed", "error", self.id)
                return False
        self.bus.event(f"Connecting to {self.cfg.ip}:{self.cfg.cmd_port} ...", "info", self.id)
        r = await self.link.command("command", timeout=3, retries=2)
        if not r.ok:
            self.connected = False
            self.bus.event("No reply to 'command'. Are you on the Tello Wi-Fi? (firewall may block UDP 8890/11111)", "error", self.id)
            self._publish_link()
            return False
        self.connected = True
        self.bus.event("SDK mode entered", "success", self.id)
        await self.link.command("streamon")
        for q in ("sdk?", "sn?", "battery?", "wifi?"):
            r = await self.link.command(q)
            if r.ok and r.resp:
                self.info[q.rstrip("?")] = r.resp
        self._publish_link()
        return True

    async def disconnect(self) -> None:
        if self.connected:
            await self.link.command("streamoff")
        self.connected = False
        self.video.store.clear()
        self._publish_link()
        self.bus.event("Disconnected", "info", self.id)

    # --------------------------------------------------------------- callbacks
    def _on_state(self, st: dict, raw: str) -> None:
        now = time.monotonic()
        self.state, self.state_raw = st, raw
        self._state_times.append(now)
        self._state_times = [t for t in self._state_times if now - t < 1.0]
        self.state_hz = float(len(self._state_times))
        h = st.get("h", 0)
        if not self.flying and h > 20 and st.get("tof", 0) > 30 and self.connected:
            self.flying = True  # took off by other means (e.g. throw-and-go)
        pose = self.odom.update(st)
        if self.coverage is not None:
            self.coverage.update(self.id, pose, self.flying)
        self.bus.publish("telemetry", {"drone": self.id, "state": st, "odom": pose, "t": time.time(),
                                       "status": self.status(compact=True)})

    def _on_frame(self, fr: Frame) -> None:
        for fn in list(self.frame_listeners):
            try:
                fn(self, fr)
            except Exception:
                log.exception("frame listener failed")

    def _on_command(self, r: CommandResult) -> None:
        if r.ok:
            self.last_rtt = r.dt
        self.bus.publish("command", {"drone": self.id, **r.as_dict(), "t": time.time()})

    # ----------------------------------------------------------------- control
    async def cmd(self, text: str, source: str = "ui") -> CommandResult:
        """Send an SDK command with GCS-side checks and side effects."""
        text = " ".join(text.strip().split())
        head = text.split()[0].lower() if text else ""
        if not self.link.is_open:
            return CommandResult(text, False, "link not open", 0.0)
        if head == "emergency":
            self.link.send_raw("emergency")  # don't wait behind the command lock
            self._on_motors_off("EMERGENCY - motors stopped", "error")
            return CommandResult(text, True, "sent", 0.0)
        if head == "takeoff" and self.safety:
            blockers = self.safety.takeoff_blockers()
            if blockers:
                msg = "Takeoff blocked: " + "; ".join(blockers)
                self.bus.event(msg, "warn", self.id, src="safety")
                return CommandResult(text, False, msg, 0.0)
        if head == "land" and self.link.busy:
            # Land must not wait for a long move to finish: kick it through raw
            self.link.send_raw("land")
            self.bus.event("Land requested (sent immediately)", "warn", self.id)
        if head in MOTION_CMDS and head not in ("takeoff", "land") and not self.flying:
            self.bus.event(f"'{head}' ignored: not flying", "warn", self.id)
            return CommandResult(text, False, "not flying", 0.0)
        if head == "takeoff":
            self.odom.set_home(self.state.get("yaw"))
        res = await self.link.command(text)
        if res.ok and head == "takeoff":
            self.flying = True
            self.takeoff_t = time.monotonic()
            self.bus.event("Takeoff complete", "success", self.id)
            self.bus.publish("flight", {"drone": self.id, "flying": True})
        elif head == "land" and (res.ok or self.state.get("h", 0) < 10):
            self._on_motors_off("Landed", "success")
        elif not res.ok:
            self.bus.event(f"'{text}' failed: {res.resp or 'timeout'}", "warn", self.id)
        return res

    def _on_motors_off(self, msg: str, level: str) -> None:
        was = self.flying
        self.flying = False
        self.takeoff_t = None
        self.set_mode("MANUAL", reason="motors off")
        if self.mission:
            self.mission.abort("motors off")
        self.bus.event(msg, level, self.id)
        if was:
            self.bus.publish("flight", {"drone": self.id, "flying": False})

    def set_pilot_sticks(self, lr, fb, ud, yw) -> None:
        self.pilot.set(lr, fb, ud, yw)
        if self.pilot.nonzero() and self.mode != "MANUAL":
            self.set_mode("MANUAL", reason="pilot override")
            if self.mission:
                self.mission.abort("pilot override")

    def set_mode(self, mode: str, reason: str = "") -> None:
        mode = mode.upper()
        if mode not in MODES or mode == self.mode:
            return
        if mode == "AUTO" and not (self.autonomy and self.autonomy.active_name(self.id)):
            self.bus.event("AUTO needs an autonomy controller selected (Autonomy tab)", "warn", self.id)
            return
        self.mode = mode
        self.bus.publish("mode", {"drone": self.id, "mode": mode, "reason": reason})
        self.bus.event(f"Mode -> {mode}" + (f" ({reason})" if reason else ""), "info", self.id)

    async def hover(self, reason: str = "", hard: bool = False) -> None:
        """Best-effort 'stop everything and hold position'.

        ``hard`` also interrupts a running SDK move with ``stop`` (SDK 2.0+: "hovers in the air,
        works at any time") - used by the collision guard, where finishing the move is not an option.
        """
        if self.mission:
            self.mission.abort(reason or "hover")
        self.set_mode("MANUAL", reason or "hover")
        self.pilot.set(0, 0, 0, 0)
        if hard and self.link.busy:
            self.link.send_raw("stop")
        # rc 0 0 0 0 gets no reply, so it can't be mistaken for the reply of a
        # running command (a raw "stop" would be).
        self.link.send_raw("rc 0 0 0 0")

    # -------------------------------------------------------------------- loops
    async def _rc_loop(self) -> None:
        while True:
            rate = max(5, int(self.settings.get("rc.rate_hz", 20)))
            await asyncio.sleep(1.0 / rate)
            if not (self.connected and self.flying):
                continue
            dead = float(self.settings.get("rc.deadman_ms", 400))
            sticks = (0, 0, 0, 0)
            pilot_active = self.pilot.fresh(dead) and self.pilot.nonzero()
            if pilot_active:
                sticks = self.pilot.tuple()
            elif self.mode == "AUTO" and self.autonomy:
                out = self.autonomy.step(self)
                if out is not None:
                    self.auto.set(*out)
                    sticks = self.auto.tuple()
            if self.link.busy and not pilot_active:
                continue  # a discrete motion command is running - let it finish
            if sticks != (0, 0, 0, 0) or self.last_rc != (0, 0, 0, 0) or time.monotonic() - self.link.last_tx_t > 0.5:
                self.link.send_raw("rc %d %d %d %d" % sticks)
                self.last_rc = sticks

    async def _house_loop(self) -> None:
        """1 Hz housekeeping: link status, keep-alive, Wi-Fi SNR, video bitrate."""
        tick = 0
        while True:
            await asyncio.sleep(1.0)
            tick += 1
            vb = self.link.video_bytes
            self._vid_kbps = (vb - self._vid_bytes_prev) * 8 / 1000
            self._vid_bytes_prev = vb
            alive = (time.monotonic() - self.link.last_state_t) < float(self.settings.get("safety.link_timeout_s", 2.0))
            if alive != self._link_alive:
                self._link_alive = alive
                if self.connected:
                    self.bus.event("Telemetry link " + ("restored" if alive else "LOST"),
                                   "success" if alive else "error", self.id, src="link")
            if self.connected and not self.link.busy:
                idle = time.monotonic() - self.link.last_tx_t
                if not self.flying and idle > 5:
                    asyncio.ensure_future(self.link.command("command", quiet=True))  # keep SDK mode alive
                elif tick % 4 == 0 and self.mode == "MANUAL":
                    async def _snr():
                        r = await self.link.command("wifi?", quiet=True)
                        if r.ok and r.resp:
                            self.info["wifi"] = r.resp
                    asyncio.ensure_future(_snr())
            self._publish_link()

    def _publish_link(self) -> None:
        self.bus.publish("link", {"drone": self.id, **self.status()})

    # ------------------------------------------------------------------ status
    def status(self, compact: bool = False) -> dict:
        now = time.monotonic()
        d = {
            "id": self.id, "connected": self.connected,
            "link_alive": (now - self.link.last_state_t) < float(self.settings.get("safety.link_timeout_s", 2.0)),
            "video_alive": (now - self.link.last_video_t) < 2.0,
            "flying": self.flying, "mode": self.mode, "busy": self.link.busy,
            "flight_time": round(now - self.takeoff_t, 1) if self.takeoff_t else 0.0,
            "rc": self.last_rc,
            "nav": self.nav_est,
            "collision": self.safety.collision if self.safety else {},
        }
        if compact:
            return d
        d.update({
            "name": self.cfg.name, "ip": self.cfg.ip, "home": [self.cfg.home_x, self.cfg.home_y, self.cfg.home_yaw], "color": self.cfg.color, "bind_ip": self.cfg.bind_ip,
            "info": self.info, "state_hz": self.state_hz, "video_fps": self.video.fps,
            "video_kbps": round(self._vid_kbps, 1), "video_size": [self.video.width, self.video.height],
            "decode_errors": self.video.decode_errors, "rtt": round(self.last_rtt, 3) if self.last_rtt else None,
            "open_error": self.link.open_error,
            "recording": self.recorder.status() if self.recorder else {},
            "mission": self.mission.status() if self.mission else {},
        })
        return d
