"""Low-level DJI Tello UDP link (SDK 1.3 / 2.0 text protocol).

This is the "drone I/O" layer: it knows nothing about the UI, missions or
safety. It only:
  * sends text commands to <ip>:8889 and matches the "ok"/"error"/value reply,
  * fires-and-forgets ``rc`` stick packets,
  * receives the 10 Hz state string on :8890 and parses it,
  * receives the raw H.264 stream on :11111 and hands packets to a sink.

Every port and the local bind address are configurable, which is what makes
two normal (non-EDU) Tellos possible from one PC: give each drone its own
Wi-Fi adapter and set ``bind_ip`` to that adapter's address.
"""
from __future__ import annotations

import asyncio
import logging
import socket
import sys
import time
from dataclasses import dataclass
from typing import Callable

from ..config import DroneConfig

log = logging.getLogger("pawn.tello")

# Per-command reply timeouts (seconds). Tello replies to motion commands only
# after the motion has finished, so these need to be generous.
TIMEOUTS = {
    "command": 3, "takeoff": 20, "land": 20, "emergency": 3, "streamon": 5, "streamoff": 5,
    "up": 20, "down": 20, "left": 20, "right": 20, "forward": 20, "back": 20,
    "cw": 20, "ccw": 20, "flip": 10, "go": 30, "curve": 30, "stop": 5, "speed": 5,
    "wifi": 5, "setbitrate": 5, "setresolution": 5, "setfps": 5,
}
QUERY_TIMEOUT = 3.0

# Every field of the Tello state string, with unit + human label (used by the UI).
STATE_FIELDS = {
    "pitch": ("deg", "Pitch"), "roll": ("deg", "Roll"), "yaw": ("deg", "Yaw"),
    "vgx": ("dm/s", "Velocity X"), "vgy": ("dm/s", "Velocity Y"), "vgz": ("dm/s", "Velocity Z"),
    "templ": ("°C", "Temp low"), "temph": ("°C", "Temp high"), "tof": ("cm", "ToF distance"),
    "h": ("cm", "Height"), "bat": ("%", "Battery"), "baro": ("m", "Barometer"),
    "time": ("s", "Motor time"), "agx": ("mg", "Accel X"), "agy": ("mg", "Accel Y"), "agz": ("mg", "Accel Z"),
    # Tello EDU only (mission pads) - parsed if present
    "mid": ("", "Mission pad"), "x": ("cm", "Pad X"), "y": ("cm", "Pad Y"), "z": ("cm", "Pad Z"),
}


def parse_state(raw: str) -> dict[str, float]:
    """``"pitch:0;roll:1;...;\\r\\n"`` -> ``{"pitch": 0.0, "roll": 1.0, ...}``."""
    out: dict[str, float] = {}
    for part in raw.strip().split(";"):
        if ":" not in part:
            continue
        k, v = part.split(":", 1)
        k = k.strip()
        if k == "mpry":  # "a,b,c" (EDU) -> keep as three fields
            try:
                a, b, c = (float(x) for x in v.split(","))
                out.update({"mpry_p": a, "mpry_r": b, "mpry_y": c})
            except ValueError:
                pass
            continue
        try:
            out[k] = float(v)
        except ValueError:
            continue
    return out


@dataclass
class CommandResult:
    cmd: str
    ok: bool
    resp: str | None
    dt: float

    def as_dict(self) -> dict:
        return {"cmd": self.cmd, "ok": self.ok, "resp": self.resp, "dt": round(self.dt, 3)}


class _Proto(asyncio.DatagramProtocol):
    def __init__(self, on_data: Callable[[bytes, tuple], None]) -> None:
        self.on_data = on_data

    def datagram_received(self, data: bytes, addr) -> None:  # noqa: D401
        self.on_data(data, addr)

    def error_received(self, exc: Exception) -> None:
        log.debug("udp error: %s", exc)


def _udp_socket(host: str, port: int, rcvbuf: int | None = None) -> socket.socket:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if rcvbuf:
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, rcvbuf)
        except OSError:
            pass
    if sys.platform == "win32" and hasattr(socket, "SIO_UDP_CONNRESET"):
        # Windows: an ICMP "port unreachable" (drone not up yet) would otherwise
        # raise WinError 10054 on the next receive and can stall the socket.
        try:
            s.ioctl(socket.SIO_UDP_CONNRESET, False)
        except OSError:
            pass
    s.bind((host, port))
    s.setblocking(False)
    return s


class TelloLink:
    def __init__(self, cfg: DroneConfig,
                 on_state: Callable[[dict, str], None],
                 on_video: Callable[[bytes], None],
                 on_command: Callable[[CommandResult], None] | None = None) -> None:
        self.cfg = cfg
        self.on_state = on_state
        self.on_video = on_video
        self.on_command = on_command
        self._cmd_tr: asyncio.DatagramTransport | None = None
        self._transports: list[asyncio.BaseTransport] = []
        self._replies: asyncio.Queue[str] = asyncio.Queue()
        self._lock = asyncio.Lock()
        self._pending: str | None = None
        self.last_state_t = 0.0
        self.last_video_t = 0.0
        self.last_tx_t = 0.0
        self.state_count = 0
        self.video_bytes = 0
        self.open_error: str | None = None

    # ------------------------------------------------------------------ setup
    async def open(self) -> None:
        loop = asyncio.get_running_loop()
        c = self.cfg
        try:
            cmd_sock = _udp_socket(c.bind_ip, c.local_cmd_port)
            self._cmd_tr, _ = await loop.create_datagram_endpoint(lambda: _Proto(self._on_reply), sock=cmd_sock)
            st_sock = _udp_socket(c.bind_ip, c.state_port)
            tr, _ = await loop.create_datagram_endpoint(lambda: _Proto(self._on_state), sock=st_sock)
            self._transports.append(tr)
            vid_sock = _udp_socket(c.bind_ip, c.video_port, rcvbuf=4 * 1024 * 1024)
            tr, _ = await loop.create_datagram_endpoint(lambda: _Proto(self._on_video), sock=vid_sock)
            self._transports.append(tr)
            self.open_error = None
        except OSError as e:
            self.open_error = f"UDP bind failed ({e}). Is another GCS/app using ports {c.local_cmd_port}/{c.state_port}/{c.video_port}?"
            await self.close()
            raise

    async def close(self) -> None:
        for tr in self._transports:
            tr.close()
        self._transports.clear()
        if self._cmd_tr:
            self._cmd_tr.close()
            self._cmd_tr = None

    @property
    def is_open(self) -> bool:
        return self._cmd_tr is not None

    # -------------------------------------------------------------- receivers
    def _on_reply(self, data: bytes, addr) -> None:
        txt = data.decode("utf-8", "replace").strip()
        if self._pending is None:
            log.debug("[%s] unsolicited reply: %r", self.cfg.id, txt)
            return
        self._replies.put_nowait(txt)

    def _on_state(self, data: bytes, addr) -> None:
        raw = data.decode("ascii", "replace")
        st = parse_state(raw)
        if st:
            self.last_state_t = time.monotonic()
            self.state_count += 1
            self.on_state(st, raw)

    def _on_video(self, data: bytes, addr) -> None:
        self.last_video_t = time.monotonic()
        self.video_bytes += len(data)
        self.on_video(data)

    # ---------------------------------------------------------------- sending
    def send_raw(self, cmd: str) -> None:
        """Fire and forget (used for ``rc`` which never gets a reply)."""
        if self._cmd_tr:
            self._cmd_tr.sendto(cmd.encode("ascii"), (self.cfg.ip, self.cfg.cmd_port))
            self.last_tx_t = time.monotonic()

    async def command(self, cmd: str, timeout: float | None = None, retries: int = 0, quiet: bool = False) -> CommandResult:
        """Send a command and wait for its reply. Serialised: Tello handles one at a time."""
        cmd = cmd.strip()
        head = cmd.split()[0] if cmd else ""
        if timeout is None:
            timeout = QUERY_TIMEOUT if head.endswith("?") else TIMEOUTS.get(head, 8.0)
        async with self._lock:
            res = None
            for attempt in range(retries + 1):
                while not self._replies.empty():  # drop stale late replies
                    self._replies.get_nowait()
                t0 = time.monotonic()
                self._pending = cmd
                self.send_raw(cmd)
                try:
                    resp = await asyncio.wait_for(self._replies.get(), timeout)
                    ok = not resp.lower().startswith(("error", "out of range", "unknown"))
                    res = CommandResult(cmd, ok, resp, time.monotonic() - t0)
                except asyncio.TimeoutError:
                    res = CommandResult(cmd, False, None, time.monotonic() - t0)
                finally:
                    self._pending = None
                if res.ok:
                    break
        if self.on_command and not quiet:
            self.on_command(res)
        return res

    @property
    def busy(self) -> bool:
        return self._lock.locked()
