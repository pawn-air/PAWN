"""Failsafes and pre-flight checks.

Runs at 5 Hz per drone. All thresholds come from runtime settings (Settings tab).
Implemented failsafes (as in QGroundControl / PX4 style GCSs):
  * battery warning and automatic landing at critical level
  * over-temperature warning
  * telemetry link loss detection (Tello itself auto-lands after ~15 s without commands)
  * virtual geofence (radius + max altitude) on the estimated pose
  * GCS client loss (all browsers closed while flying) -> hover, then optional land
  * collision guard: monocular time-to-contact (looming) and moving obstacles in the
    flight path -> brake (hover) - see vision plugins ``collision_ttc`` / ``motion``
  * smart RTH: battery needed to fly the breadcrumb trail home (learned drain rate)
    -> warn or start RTH automatically (DJI-style "return-to-home battery")
"""
from __future__ import annotations

import asyncio
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .node import DroneNode


class SafetyMonitor:
    def __init__(self, node: "DroneNode", settings, client_count) -> None:
        self.node = node
        self.s = settings
        self.client_count = client_count    # callable -> number of connected UI clients
        self._latched: set[str] = set()
        self._client_lost_t: float | None = None
        self._task: asyncio.Task | None = None
        self.alarms: list[dict] = []
        self._brake_t = 0.0
        self._est_t = 0.0
        self.collision: dict = {}

    def start(self) -> None:
        self._task = asyncio.ensure_future(self._loop())

    def _once(self, key: str, msg: str, level: str = "warn") -> bool:
        if key in self._latched:
            return False
        self._latched.add(key)
        self.node.bus.event(msg, level, self.node.id, src="safety")
        return True

    def _clear(self, key: str) -> None:
        self._latched.discard(key)

    # ---------------------------------------------------------------- checks
    def preflight(self) -> list[dict]:
        n, st = self.node, self.node.state
        bat = st.get("bat")
        temp = st.get("temph")
        checks = [
            {"name": "SDK connection", "ok": n.connected, "detail": "in SDK mode" if n.connected else "press Connect"},
            {"name": "Telemetry", "ok": n.status(True)["link_alive"], "detail": f"{n.state_hz:.0f} Hz"},
            {"name": "Video stream", "ok": n.status(True)["video_alive"], "detail": f"{n.video.fps:.0f} fps"},
            {"name": "Battery", "ok": bat is not None and bat > max(self.s.get("safety.battery_warn", 30), 20),
             "detail": f"{bat:.0f}%" if bat is not None else "n/a"},
            {"name": "Temperature", "ok": temp is not None and temp < self.s.get("safety.temp_warn", 85),
             "detail": f"{temp:.0f}°C" if temp is not None else "n/a"},
            {"name": "Attitude level", "ok": abs(st.get("pitch", 0)) < 15 and abs(st.get("roll", 0)) < 15,
             "detail": f"p {st.get('pitch', 0):.0f}° r {st.get('roll', 0):.0f}°"},
            {"name": "Not flying", "ok": not n.flying, "detail": "on ground" if not n.flying else "airborne"},
        ]
        return checks

    def takeoff_blockers(self) -> list[str]:
        if not self.s.get("safety.require_preflight", True):
            return []
        crit = {"SDK connection", "Telemetry", "Battery", "Temperature", "Not flying"}
        return [f"{c['name']} ({c['detail']})" for c in self.preflight() if c["name"] in crit and not c["ok"]]

    # ------------------------------------------------------------ collision
    def _collision_guard(self) -> None:
        n, s = self.node, self.s
        now = time.monotonic()
        info, reason = {}, None
        d = n.detections.get("collision_ttc")
        if d and time.time() - d["t"] < 0.6 and d.get("items"):
            info = d["items"][0].get("info", {})
            if info.get("danger"):
                reason = f"time-to-contact {info.get('ttc_s')} s"
        if reason is None and s.get("safety.dynamic_obstacle_brake", False):
            m = n.detections.get("motion")
            if m and time.time() - m["t"] < 0.6:
                for it in m.get("items", []):
                    b = it.get("box")
                    if it.get("label") == "moving" and b and b[2] * b[3] > 0.08 and 0.25 < b[0] + b[2] / 2 < 0.75:
                        reason = "moving obstacle in flight path"
                        break
        self.collision = {"ttc_s": info.get("ttc_s"), "danger": reason is not None, "reason": reason}
        if not (n.flying and s.get("safety.collision_guard", True) and reason):
            return
        forward = n.last_rc[1] > 10 or (n.link.busy and n.odom.speed > 0.15) or (n.mode == "AUTO" and n.odom.speed > 0.15)
        if forward and now - self._brake_t > 3.0:
            self._brake_t = now
            n.bus.event(f"COLLISION GUARD: {reason} - braking", "error", n.id, src="safety")
            n.bus.publish("collision", {"drone": n.id, "reason": reason, "ttc_s": info.get("ttc_s")})
            asyncio.ensure_future(n.hover("collision guard", hard=True))

    # ------------------------------------------------------------ smart RTH
    async def _smart_rth(self, bat: float) -> None:
        n, s = self.node, self.s
        nav = getattr(n, "nav", None)
        if nav is None:
            return
        nav.battery.update(n.flying, bat)
        now = time.monotonic()
        if now - self._est_t < 1.0:
            return
        self._est_t = now
        est = nav.estimate() if n.flying else {}
        n.nav_est = est
        mode = s.get("safety.smart_rth", "warn")
        if not n.flying or mode == "off" or not est:
            return
        if bat <= est["bat_needed"] + s.get("safety.battery_land", 15):
            msg = (f"Smart RTH: {bat:.0f}% left, ~{est['bat_needed']:.0f}% needed to fly {est['path_m']:.1f} m home"
                   f" (+{s.get('safety.battery_land', 15)}% land reserve)")
            if mode == "rth" and n.mission.status()["name"] != "RTH":
                if self._once("srth", msg + " - RETURNING HOME", "error"):
                    try:
                        nav.rth(land=True)
                    except Exception as e:  # pragma: no cover
                        n.bus.event(f"Smart RTH failed: {e}", "error", n.id, src="safety")
            else:
                self._once("srth", msg + " - return now", "warn")

    # ------------------------------------------------------------------ loop
    async def _loop(self) -> None:
        while True:
            await asyncio.sleep(0.2)
            try:
                await self._tick()
            except Exception as e:  # pragma: no cover
                self.node.bus.event(f"safety monitor error: {e}", "error", self.node.id, src="safety")

    async def _tick(self) -> None:
        n, s, st = self.node, self.s, self.node.state
        if not n.connected or not st:
            return
        bat = st.get("bat", 100)
        if bat <= s.get("safety.battery_warn", 30):
            self._once("bat_warn", f"Battery low: {bat:.0f}%")
        else:
            self._clear("bat_warn")
        if n.flying and bat <= s.get("safety.battery_land", 15):
            if self._once("bat_land", f"Battery CRITICAL {bat:.0f}% - auto landing", "error"):
                await n.hover("battery critical")
                asyncio.ensure_future(n.cmd("land", source="safety"))
        if not n.flying:
            self._clear("bat_land")
            self._clear("srth")
        self._collision_guard()
        await self._smart_rth(bat)
        temp = st.get("temph", 0)
        if temp >= s.get("safety.temp_warn", 85):
            self._once("temp", f"High temperature {temp:.0f}°C - land soon / add airflow")
        elif temp < s.get("safety.temp_warn", 85) - 3:
            self._clear("temp")

        if n.flying and s.get("safety.geofence_enabled", True):
            rng = n.odom.horizontal_range
            alt = n.odom.pose()["z"]
            r, amax = s.get("safety.geofence_radius_m", 8.0), s.get("safety.geofence_max_alt_m", 3.0)
            if rng > r or alt > amax:
                what = f"range {rng:.1f}/{r} m" if rng > r else f"alt {alt:.1f}/{amax} m"
                if self._once("fence", f"GEOFENCE breach ({what}) - {s.get('safety.geofence_action', 'hover')}", "error"):
                    await n.hover("geofence")
                    if s.get("safety.geofence_action") == "land":
                        asyncio.ensure_future(n.cmd("land", source="safety"))
            elif rng < r * 0.9 and alt < amax * 0.9:
                self._clear("fence")

        # GCS client loss
        if n.flying and self.client_count() == 0 and s.get("safety.client_loss_action") != "none":
            if self._client_lost_t is None:
                self._client_lost_t = time.monotonic()
                n.bus.event("No GCS client connected - holding position", "warn", n.id, src="safety")
                await n.hover("client lost")
            elif (s.get("safety.client_loss_action") == "land"
                  and time.monotonic() - self._client_lost_t > s.get("safety.client_loss_land_s", 10)):
                if self._once("client_land", "GCS client lost too long - landing", "error"):
                    asyncio.ensure_future(n.cmd("land", source="safety"))
        else:
            self._client_lost_t = None
            self._clear("client_land")
