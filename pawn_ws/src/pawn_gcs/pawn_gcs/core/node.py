"""DroneNode: one drone = UDP link + video decoder + pilot sticks.

The GCS never flies the drone on its own. The only things it sends without a
human pressing something are:
  * while the motors spin: ``rc 0 0 0 0`` every 0.5 s when no stick is moved - this is what
    keeps a Tello hovering (the firmware lands by itself after 15 s without any command),
  * while the motors are off: no rc at all, only ``command`` after 5 s of silence (keeps SDK mode),
  * ``command`` / ``streamon`` / ``sdk?`` / ``sn?`` / ``wifi?`` once, when you press Connect.
"Motors spin" is read from the drone's own ``time`` state field (motor-on seconds), which only
advances while the motors run.
Pilot sticks are clamped to ``safety.stick_max`` and fall back to zero (= hover)
when the browser stops sending them for ``safety.deadman_ms``.

Extension point: ``frame_listeners`` (decoded video frames) and the event bus
(``telemetry``, ``command``, ``event``, ``link``, ``tello`` topics).
"""
from __future__ import annotations

import asyncio
import logging
import time
from typing import TYPE_CHECKING, Any

from ..config import DroneConfig
from .tello import CommandResult, TelloLink
from .video import Frame, VideoPipeline

if TYPE_CHECKING:
    from .bus import EventBus

log = logging.getLogger("pawn.node")

KEEPALIVE_S = 0.5
GROUND_KEEPALIVE_S = 5.0
ZERO = (0, 0, 0, 0)
ZERO_REPEATS = 3           # extra "rc 0 0 0 0" after a stick is released (UDP can drop one)
BUSY_QUIET_S = 12.0        # keepalive pauses this long for a running takeoff / SDK move, then resumes
                           # (a lost reply must not leave the drone without commands past its 15 s limit)
MOTORS_OFF_AFTER_S = 2.5   # "time" field unchanged this long -> motors are off


class Sticks:
    __slots__ = ("lr", "fb", "ud", "yw", "t")

    def __init__(self) -> None:
        self.lr = self.fb = self.ud = self.yw = 0
        self.t = 0.0

    def set(self, lr, fb, ud, yw, limit: int = 100) -> None:
        c = lambda v: int(max(-limit, min(limit, round(float(v)))))  # noqa: E731
        self.lr, self.fb, self.ud, self.yw = c(lr), c(fb), c(ud), c(yw)
        self.t = time.monotonic()

    def fresh(self, ms: float) -> bool:
        return (time.monotonic() - self.t) * 1000 < ms

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
        self.video = VideoPipeline(cfg.id, on_frame=self._on_frame)
        self.link = TelloLink(cfg, self._on_state, self.video.feed, self._on_command, self._on_unsolicited)
        self.connected = False          # SDK mode entered ("command" -> ok)
        self.pilot = Sticks()
        self.last_rc: tuple = (0, 0, 0, 0)
        self.info: dict[str, Any] = {}
        self.state_hz = 0.0
        self._state_times: list[float] = []
        self._tasks: list[asyncio.Task] = []
        self._link_alive = False
        self._vid_kbps = 0.0
        self._vid_bytes_prev = 0
        self.last_rtt: float | None = None
        self._zero_repeats = 0
        self._motor_time: float | None = None
        self._motor_time_t = float("-inf")  # monotonic time the "time" field last changed (monotonic may start at 0)
        self.recorder = None
        self.frame_listeners: list = []

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
        for q in ("sdk?", "sn?", "wifi?"):
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
        self.state, self.state_raw = st, raw.strip()
        self._state_times.append(now)
        self._state_times = [t for t in self._state_times if now - t < 1.0]
        self.state_hz = float(len(self._state_times))
        if st.get("time") != self._motor_time:
            if self._motor_time is not None:  # the first packet only sets the baseline
                self._motor_time_t = now
            self._motor_time = st.get("time")
        self.bus.publish("telemetry", {"drone": self.id, "state": st, "raw": self.state_raw, "t": time.time(),
                                       "rc": self.last_rc})

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

    def _on_unsolicited(self, txt: str) -> None:
        self.bus.publish("tello", {"drone": self.id, "msg": txt, "t": time.time()})

    # ----------------------------------------------------------------- control
    async def cmd(self, text: str, source: str = "ui") -> CommandResult:
        """Send an SDK command exactly as given and return the drone's reply."""
        text = " ".join(text.strip().split())
        head = text.split()[0].lower() if text else ""
        if not self.link.is_open:
            return CommandResult(text, False, "link not open", 0.0)
        if head == "emergency":
            self.link.send_raw("emergency")  # don't wait behind the command lock
            self.bus.publish("command", {"drone": self.id, "cmd": "emergency", "ok": True, "resp": "(sent, no reply awaited)",
                                         "dt": 0.0, "t": time.time()})
            return CommandResult(text, True, "sent", 0.0)
        if head == "land" and self.link.busy:
            # land must not wait behind a running move
            self.link.send_raw("land")
            self.bus.event("land sent immediately (another command was still running)", "info", self.id)
        return await self.link.command(text)

    def set_pilot_sticks(self, lr, fb, ud, yw) -> None:
        self.pilot.set(lr, fb, ud, yw, limit=int(self.settings.get("safety.stick_max", 60)))

    @property
    def motors_running(self) -> bool:
        """From the drone's motor-on timer. True when unknown (no timer field, or no fresh state
        packet): then rc keeps flowing, so stick control and the keepalive never stop mid-air."""
        if "time" not in self.state or time.monotonic() - self.link.last_state_t > 1.0:
            return True
        return time.monotonic() - self._motor_time_t < MOTORS_OFF_AFTER_S

    def hover(self) -> None:
        """Sticks to zero now. ``rc 0 0 0 0`` gets no reply, so it never collides with a running command."""
        self.pilot.set(0, 0, 0, 0)
        self._send_rc(ZERO)
        self._zero_repeats = ZERO_REPEATS

    # -------------------------------------------------------------------- loops
    def _send_rc(self, sticks: tuple) -> None:
        self.link.send_raw("rc %d %d %d %d" % sticks)
        self.last_rc = sticks

    async def _rc_loop(self) -> None:
        while True:
            rate = max(5, int(self.settings.get("rc.rate_hz", 20)))
            await asyncio.sleep(1.0 / rate)
            if not self.connected:
                continue
            dead = float(self.settings.get("safety.deadman_ms", 400))
            sticks = self.pilot.tuple() if self.pilot.fresh(dead) else ZERO
            moving = sticks != ZERO
            now = time.monotonic()
            # A released stick stops the drone at once - also while a command waits for its reply
            # (the drone keeps the last rc values until it gets new ones).
            if not moving and self.last_rc != ZERO:
                self._send_rc(ZERO)
                self._zero_repeats = ZERO_REPEATS
                continue
            if not moving and self._zero_repeats:
                self._send_rc(ZERO)
                self._zero_repeats -= 1
                continue
            if not self.motors_running:
                # on the ground: no rc packets (the drone has to accept "takeoff" undisturbed)
                if now - self.link.last_tx_t > GROUND_KEEPALIVE_S and not self.link.busy:
                    asyncio.ensure_future(self.link.command("command", quiet=True))
                continue
            if moving:
                self._send_rc(sticks)
            elif self.link.busy and self.link.busy_for < BUSY_QUIET_S:
                continue  # a takeoff / SDK move is running - let it finish
            elif now - self.link.last_tx_t > KEEPALIVE_S:
                self._send_rc(ZERO)

    async def _house_loop(self) -> None:
        """1 Hz: link status and video bitrate."""
        while True:
            await asyncio.sleep(1.0)
            vb = self.link.video_bytes
            self._vid_kbps = (vb - self._vid_bytes_prev) * 8 / 1000
            self._vid_bytes_prev = vb
            alive = (time.monotonic() - self.link.last_state_t) < float(self.settings.get("link.timeout_s", 2.0))
            if alive != self._link_alive:
                self._link_alive = alive
                if self.connected:
                    self.bus.event("State packets " + ("arriving" if alive else "STOPPED"),
                                   "success" if alive else "error", self.id, src="link")
            self._publish_link()

    def _publish_link(self) -> None:
        self.bus.publish("link", {"drone": self.id, **self.status()})

    # ------------------------------------------------------------------ status
    def status(self) -> dict:
        now = time.monotonic()
        return {
            "id": self.id, "name": self.cfg.name, "ip": self.cfg.ip, "bind_ip": self.cfg.bind_ip, "color": self.cfg.color,
            "connected": self.connected,
            "link_alive": (now - self.link.last_state_t) < float(self.settings.get("link.timeout_s", 2.0)),
            "video_alive": (now - self.link.last_video_t) < 2.0,
            "busy": self.link.busy, "rc": self.last_rc, "info": self.info, "motors_running": self.motors_running,
            "state_hz": self.state_hz, "state_packets": self.link.state_count,
            "video_fps": self.video.fps, "video_kbps": round(self._vid_kbps, 1),
            "video_size": [self.video.width, self.video.height], "decode_errors": self.video.decode_errors,
            "rtt": round(self.last_rtt, 3) if self.last_rtt else None, "open_error": self.link.open_error,
            "recording": self.recorder.status() if self.recorder else {},
        }
