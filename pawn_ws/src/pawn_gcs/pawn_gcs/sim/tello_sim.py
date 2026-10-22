"""Software-in-the-loop DJI Tello emulator.

Speaks the *real* Tello UDP protocol (commands, 10 Hz state string, H.264
video on UDP), so the GCS runs exactly the same code path as with hardware.
Start with ``python -m pawn_gcs --sim`` (embedded) or stand-alone:

    python -m pawn_gcs.sim.tello_sim --cmd-port 18889 --state-port 28890 --video-port 31111
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import math
import random
import threading
import time

import numpy as np

from .world import Renderer

log = logging.getLogger("pawn.sim")

MOVE_DIRS = {"forward": (1, 0, 0), "back": (-1, 0, 0), "left": (0, 1, 0), "right": (0, -1, 0),
             "up": (0, 0, 1), "down": (0, 0, -1)}


class SimTello(asyncio.DatagramProtocol):
    def __init__(self, cmd_port=18889, gcs_ip="127.0.0.1", state_port=28890, video_port=31111,
                 start_xy=(0.0, 0.0), start_yaw=0.0, battery=96, name="sim"):
        self.cmd_port, self.gcs_ip = cmd_port, gcs_ip
        self.state_port, self.video_port = state_port, video_port
        self.name = name
        # pose: world x fwd, y left, z up (m); yaw deg CCW+
        self.x, self.y, self.z = start_xy[0], start_xy[1], 0.0
        self.yaw = start_yaw
        self.vx = self.vy = self.vz = 0.0     # world m/s
        self.wz = 0.0                           # deg/s
        self.pitch = self.roll = 0.0
        self.flying = False
        self.sdk = False
        self.stream = False
        self.battery = float(battery)
        self.temp = 55.0
        self.motor_time = 0.0
        self.speed_cmps = 100
        self.rc = (0, 0, 0, 0)
        self.rc_t = 0.0
        self.busy: asyncio.Task | None = None
        self.transport: asyncio.DatagramTransport | None = None
        self.gcs_addr = None
        self._sock_out = None
        self._stop = False
        self.renderer = Renderer()
        self._video_thread: threading.Thread | None = None

    # ------------------------------------------------------------ networking
    def connection_made(self, transport):
        self.transport = transport

    def datagram_received(self, data, addr):
        self.gcs_addr = addr
        cmd = data.decode("ascii", "replace").strip()
        asyncio.ensure_future(self._handle(cmd, addr))

    def _reply(self, msg, addr):
        if self.transport:
            self.transport.sendto(msg.encode(), addr)

    # -------------------------------------------------------------- commands
    async def _handle(self, cmd: str, addr):
        parts = cmd.split()
        if not parts:
            return
        head, args = parts[0], parts[1:]
        if head == "rc":
            try:
                self.rc = tuple(max(-100, min(100, int(float(a)))) for a in args[:4])
                self.rc_t = time.monotonic()
            except Exception:
                pass
            return
        if head == "command":
            self.sdk = True
            return self._reply("ok", addr)
        if not self.sdk:
            return  # real Tello ignores everything before "command"
        if head.endswith("?"):
            return self._reply(self._query(head), addr)
        if head == "emergency":
            self._cancel()
            self.flying = False
            self.vx = self.vy = self.vz = 0.0
            self.z = 0.0
            return self._reply("ok", addr)
        if head in ("streamon", "streamoff"):
            self.stream = head == "streamon"
            return self._reply("ok", addr)
        if head in ("speed",):
            try:
                v = int(args[0])
                assert 10 <= v <= 100
                self.speed_cmps = v
                return self._reply("ok", addr)
            except Exception:
                return self._reply("error", addr)
        if head in ("wifi", "setbitrate", "setresolution", "setfps", "mon", "moff", "mdirection", "downvision"):
            return self._reply("ok", addr)
        if head == "stop":
            self._cancel()
            self.vx = self.vy = self.vz = self.wz = 0.0
            return self._reply("ok", addr)
        if head == "land":
            self._cancel()
            if not self.flying:
                return self._reply("error", addr)
            await self._go_to(self.x, self.y, 0.0, 0.6)
            self.flying = False
            return self._reply("ok", addr)
        if self.busy and not self.busy.done():
            return self._reply("error Not joystick", addr)
        self.busy = asyncio.ensure_future(self._motion(head, args, addr))

    def _cancel(self):
        if self.busy and not self.busy.done():
            self.busy.cancel()

    async def _motion(self, head, args, addr):
        try:
            if head == "takeoff":
                if self.flying:
                    return self._reply("error", addr)
                if self.battery < 10:
                    return self._reply("error Not enough battery", addr)
                self.flying = True
                await self._go_to(self.x, self.y, 0.8, 0.8)
                return self._reply("ok", addr)
            if not self.flying:
                return self._reply("error Not joystick", addr)
            if head in MOVE_DIRS:
                d = int(args[0])
                if not 20 <= d <= 500:
                    return self._reply("out of range", addr)
                fx, fy, fz = MOVE_DIRS[head]
                c, s = math.cos(math.radians(self.yaw)), math.sin(math.radians(self.yaw))
                dx, dy = (c * fx - s * fy) * d / 100, (s * fx + c * fy) * d / 100
                await self._go_to(self.x + dx, self.y + dy, max(0.2, self.z + fz * d / 100), self.speed_cmps / 100)
                return self._reply("ok", addr)
            if head in ("cw", "ccw"):
                a = int(args[0])
                if not 1 <= a <= 360:
                    return self._reply("out of range", addr)
                await self._rotate(-a if head == "cw" else a)
                return self._reply("ok", addr)
            if head == "flip":
                if not args or args[0] not in ("l", "r", "f", "b"):
                    return self._reply("error", addr)
                if self.battery < 50:
                    return self._reply("error Not enough battery", addr)
                await asyncio.sleep(1.2)
                self.battery -= 3
                return self._reply("ok", addr)
            if head == "go":
                gx, gy, gz, sp = (int(a) for a in args[:4])
                c, s = math.cos(math.radians(self.yaw)), math.sin(math.radians(self.yaw))
                dx, dy = (c * gx - s * gy) / 100, (s * gx + c * gy) / 100
                await self._go_to(self.x + dx, self.y + dy, max(0.2, self.z + gz / 100), sp / 100)
                return self._reply("ok", addr)
            if head == "curve":
                x1, y1, z1, x2, y2, z2, sp = (int(a) for a in args[:7])
                for gx, gy, gz in ((x1, y1, z1), (x2 - x1, y2 - y1, z2 - z1)):
                    c, s = math.cos(math.radians(self.yaw)), math.sin(math.radians(self.yaw))
                    await self._go_to(self.x + (c * gx - s * gy) / 100, self.y + (s * gx + c * gy) / 100,
                                      max(0.2, self.z + gz / 100), sp / 100)
                return self._reply("ok", addr)
            return self._reply("unknown command: " + head, addr)
        except asyncio.CancelledError:
            pass
        except Exception as e:  # malformed args
            log.debug("sim cmd error %s", e)
            self._reply("error", addr)

    async def _go_to(self, tx, ty, tz, speed):
        speed = max(0.1, speed)
        while True:
            dx, dy, dz = tx - self.x, ty - self.y, tz - self.z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist < 0.02:
                self.vx = self.vy = self.vz = 0.0
                self.x, self.y, self.z = tx, ty, tz
                return
            v = min(speed, dist * 3 + 0.05)
            self.vx, self.vy, self.vz = dx / dist * v, dy / dist * v, dz / dist * v
            await asyncio.sleep(0.02)

    async def _rotate(self, deg):
        target = self.yaw + deg
        while abs(target - self.yaw) > 0.5:
            self.wz = math.copysign(min(90.0, abs(target - self.yaw) * 4 + 5), target - self.yaw)
            await asyncio.sleep(0.02)
        self.wz = 0.0
        self.yaw = target

    def _query(self, q):
        return {
            "battery?": f"{int(self.battery)}", "speed?": f"{self.speed_cmps}.0", "time?": f"{int(self.motor_time)}s",
            "wifi?": f"{random.randint(85, 95)}", "sdk?": "20", "sn?": f"0TQSIM{self.cmd_port}",
            "height?": f"{int(self.z * 10)}dm", "temp?": f"{int(self.temp)}~{int(self.temp) + 2}C",
            "attitude?": f"pitch:{int(self.pitch)};roll:{int(self.roll)};yaw:{int(self._tello_yaw())};",
            "tof?": f"{int(self.z * 1000) if self.z > 0.1 else 100}mm", "baro?": f"{112.5 + self.z:.2f}",
            "acceleration?": "agx:0.00;agy:0.00;agz:-1000.00;", "hardware?": "TELLO", "active?": "ok",
        }.get(q, "error")

    def _tello_yaw(self):
        return ((-self.yaw) + 180) % 360 - 180

    # --------------------------------------------------------------- physics
    async def physics(self):
        dt = 0.02
        while not self._stop:
            now = time.monotonic()
            rc_on = self.flying and (self.busy is None or self.busy.done())
            if rc_on:
                lr, fb, ud, yw = self.rc
                vmax = 1.2
                c, s = math.cos(math.radians(self.yaw)), math.sin(math.radians(self.yaw))
                bf, bl = fb / 100 * vmax, -lr / 100 * vmax
                tvx, tvy = c * bf - s * bl, s * bf + c * bl
                a = 1 - math.exp(-dt / 0.25)  # first-order response
                self.vx += (tvx - self.vx) * a
                self.vy += (tvy - self.vy) * a
                self.vz += (ud / 100 * 0.8 - self.vz) * a
                self.wz += (-yw / 100 * 100 - self.wz) * a
            pvx, pvy = self.vx, self.vy
            self.x += self.vx * dt
            self.y += self.vy * dt
            self.z = max(0.0, min(3.0, self.z + self.vz * dt)) if self.flying else 0.0
            self.yaw += self.wz * dt
            R = 6.75  # walls
            self.x, self.y = max(-R, min(R, self.x)), max(-R, min(R, self.y))
            # attitude from body-frame velocity (tilt to move)
            c, s = math.cos(math.radians(self.yaw)), math.sin(math.radians(self.yaw))
            bvx, bvy = c * pvx + s * pvy, -s * pvx + c * pvy
            self.pitch += ((bvx * 9 if self.flying else 0) - self.pitch) * 0.15
            self.roll += ((-bvy * 9 if self.flying else 0) - self.roll) * 0.15
            if self.flying:
                self.motor_time += dt
                self.battery = max(0.0, self.battery - dt / 9.0)
                self.temp = min(92.0, self.temp + dt * 0.03)
                if self.battery <= 3:
                    self.flying = False  # forced landing
                    self.z = 0
            else:
                self.battery = max(0.0, self.battery - dt / 120.0)
                self.temp = max(45.0, self.temp - dt * 0.01)
            await asyncio.sleep(dt - (time.monotonic() - now) if dt > (time.monotonic() - now) else 0)

    async def state_loop(self):
        while not self._stop:
            if self.sdk and self.transport:
                c, s = math.cos(math.radians(self.yaw)), math.sin(math.radians(self.yaw))
                # Tello world frame (relative to power-on heading): vgx fwd, vgy right; dm/s
                vgx, vgy, vgz = self.vx * 10, -self.vy * 10, -self.vz * 10
                h = int(self.z * 100)
                st = (f"pitch:{int(self.pitch)};roll:{int(self.roll)};yaw:{int(self._tello_yaw())};"
                      f"vgx:{int(round(vgx))};vgy:{int(round(vgy))};vgz:{int(round(vgz))};"
                      f"templ:{int(self.temp)};temph:{int(self.temp) + 2};tof:{max(10, h + 8) if h > 5 else 10};"
                      f"h:{h};bat:{int(self.battery)};baro:{112.5 + self.z:.2f};time:{int(self.motor_time)};"
                      f"agx:{-self.pitch * 17:.2f};agy:{self.roll * 17:.2f};agz:-998.00;\r\n")
                self.transport.sendto(st.encode(), (self.gcs_ip, self.state_port))
            await asyncio.sleep(0.1)

    # ----------------------------------------------------------------- video
    def _video_worker(self):
        import av
        import socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        enc = None
        fps = 30
        t0 = time.monotonic()
        n = 0
        while not self._stop:
            if not self.stream:
                enc = None
                time.sleep(0.1)
                continue
            if enc is None:
                enc = av.CodecContext.create("libx264", "w")
                enc.width, enc.height, enc.pix_fmt = 960, 720, "yuv420p"
                enc.time_base = __import__("fractions").Fraction(1, fps)
                enc.bit_rate = 2_500_000
                enc.options = {"preset": "ultrafast", "tune": "zerolatency",
                               "x264-params": "keyint=30:min-keyint=30:repeat-headers=1:scenecut=0"}
                n = 0
            img = self.renderer.render(self.x, self.y, self.z + 0.05, self.yaw, self.pitch, self.roll,
                                       overlay=f"SIM {self.name}  bat {int(self.battery)}%")
            frame = av.VideoFrame.from_ndarray(img, format="bgr24").reformat(format="yuv420p")
            frame.pts = n
            n += 1
            try:
                for pkt in enc.encode(frame):
                    data = bytes(pkt)
                    for i in range(0, len(data), 1460):
                        sock.sendto(data[i:i + 1460], (self.gcs_ip, self.video_port))
            except Exception as e:
                log.warning("sim encode error: %s", e)
                enc = None
            t0 += 1 / fps
            delay = t0 - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                t0 = time.monotonic()

    async def start(self, host="127.0.0.1"):
        loop = asyncio.get_running_loop()
        await loop.create_datagram_endpoint(lambda: self, local_addr=(host, self.cmd_port))
        self._tasks = [asyncio.ensure_future(self.physics()), asyncio.ensure_future(self.state_loop())]
        self._video_thread = threading.Thread(target=self._video_worker, daemon=True, name=f"simvideo-{self.name}")
        self._video_thread.start()
        log.info("SimTello %s listening on %s:%d", self.name, host, self.cmd_port)

    async def stop(self):
        self._stop = True
        for t in getattr(self, "_tasks", []):
            t.cancel()
        if self.transport:
            self.transport.close()


def main():
    ap = argparse.ArgumentParser(description="Stand-alone Tello simulator")
    ap.add_argument("--cmd-port", type=int, default=18889)
    ap.add_argument("--state-port", type=int, default=28890)
    ap.add_argument("--video-port", type=int, default=31111)
    ap.add_argument("--gcs-ip", default="127.0.0.1")
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO)

    async def run():
        sim = SimTello(a.cmd_port, a.gcs_ip, a.state_port, a.video_port)
        await sim.start()
        await asyncio.Event().wait()

    asyncio.run(run())


if __name__ == "__main__":
    main()
