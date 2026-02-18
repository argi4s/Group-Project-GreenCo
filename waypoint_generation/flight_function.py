#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
pymavlink mission skeleton (course-style):
- connect + wait_heartbeat
- background RX thread updates HEARTBEAT + GLOBAL_POSITION_INT + SYS_STATUS
- heartbeat watchdog -> auto RTL
- state machine mission controller
- camera-footprint-aware lawnmower planner:
    footprint scales with altitude (given ref 50m -> 52.4m x 39.3m)
    spacing/step derived from overlaps (NOT hard-coded)

Designed to be imported and called by other scripts.
"""

from __future__ import annotations
import time
import math
import threading
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Sequence, Tuple

from pymavlink import mavutil


# -----------------------------
# Basic geo helpers
# -----------------------------
EARTH_R_M = 6371000.0

@dataclass(frozen=True)
class LatLon:
    lat: float
    lon: float

def haversine_m(a: LatLon, b: LatLon) -> float:
    lat1, lon1, lat2, lon2 = map(math.radians, [a.lat, a.lon, b.lat, b.lon])
    dlat, dlon = lat2 - lat1, lon2 - lon1
    s = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    return 2 * EARTH_R_M * math.asin(math.sqrt(s))

def latlon_to_enu_m(p: LatLon, ref: LatLon) -> Tuple[float, float]:
    """Local tangent approximation: x east (m), y north (m)."""
    lat0 = math.radians(ref.lat)
    dlat = math.radians(p.lat - ref.lat)
    dlon = math.radians(p.lon - ref.lon)
    x = EARTH_R_M * dlon * math.cos(lat0)
    y = EARTH_R_M * dlat
    return x, y

def enu_to_latlon(x_m: float, y_m: float, ref: LatLon) -> LatLon:
    lat0 = math.radians(ref.lat)
    dlat = y_m / EARTH_R_M
    dlon = x_m / (EARTH_R_M * math.cos(lat0))
    return LatLon(ref.lat + math.degrees(dlat), ref.lon + math.degrees(dlon))

def point_in_poly(x: float, y: float, poly_xy: Sequence[Tuple[float, float]]) -> bool:
    """Ray casting."""
    inside = False
    n = len(poly_xy)
    for i in range(n):
        x1, y1 = poly_xy[i]
        x2, y2 = poly_xy[(i + 1) % n]
        hit = ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1 + 1e-12) + x1)
        if hit:
            inside = not inside
    return inside

def rotate_to_track(x: float, y: float, heading_deg: float) -> Tuple[float, float]:
    """
    heading: 0=N, 90=E.
    Convert ENU -> track frame (x' along heading, y' left).
    """
    psi = math.radians(heading_deg)
    xp = math.sin(psi)*x + math.cos(psi)*y
    yp = math.cos(psi)*x - math.sin(psi)*y
    return xp, yp

def rotate_from_track(xp: float, yp: float, heading_deg: float) -> Tuple[float, float]:
    psi = math.radians(heading_deg)
    x = math.sin(psi)*xp + math.cos(psi)*yp
    y = math.cos(psi)*xp - math.sin(psi)*yp
    return x, y


# -----------------------------
# Camera footprint model
# -----------------------------
@dataclass
class CameraFootprintModel:
    """
    Your given reference:
      at 50m: 52.4m x 39.3m ground coverage
    We assume linear scaling with altitude (fixed FOV).
    """
    ref_alt_m: float = 50.0
    ref_right_m: float = 52.4   # across-track (camera right)
    ref_forward_m: float = 39.3 # along-track (camera forward)

    def footprint(self, alt_m: float) -> Tuple[float, float]:
        s = alt_m / self.ref_alt_m
        return self.ref_forward_m * s, self.ref_right_m * s  # (along, across)


# -----------------------------
# Planner: lawnmower
# -----------------------------
@dataclass
class Waypoint:
    lat: float
    lon: float
    rel_alt_m: float
    yaw_deg: Optional[float] = None
    speed_mps: Optional[float] = None

class LawnmowerPlanner:
    def __init__(self, camera: CameraFootprintModel):
        self.camera = camera

    @staticmethod
    def _auto_heading(area_xy: Sequence[Tuple[float, float]]) -> float:
        xs = [p[0] for p in area_xy]
        ys = [p[1] for p in area_xy]
        dx = max(xs) - min(xs)
        dy = max(ys) - min(ys)
        return 90.0 if dx >= dy else 0.0

    def plan(
        self,
        area_poly: Sequence[LatLon],
        alt_m: float,
        overlap_along: float = 0.7,
        overlap_across: float = 0.6,
        heading_deg: Optional[float] = None,
        speed_mps: float = 4.0,
        yaw_deg: Optional[float] = None,
        margin_m: float = 1.0,
        max_points: int = 4000,
    ) -> List[Waypoint]:
        """
        Returns waypoints that (approximately) cover area_poly by camera footprint.

        - NOT hard-coded: spacing computed from footprint(alt_m) and overlaps.
        - Only boundary constraint (SSSI ignored because outside boundary).
        """
        if len(area_poly) < 3:
            raise ValueError("area_poly must have >=3 vertices")

        ref = area_poly[0]
        area_xy = [latlon_to_enu_m(p, ref) for p in area_poly]

        if heading_deg is None:
            heading_deg = self._auto_heading(area_xy)

        # Rotate to track frame
        area_tf = [rotate_to_track(x, y, heading_deg) for x, y in area_xy]
        xs = [p[0] for p in area_tf]
        ys = [p[1] for p in area_tf]
        minx, maxx = min(xs) + margin_m, max(xs) - margin_m
        miny, maxy = min(ys) + margin_m, max(ys) - margin_m
        if minx >= maxx or miny >= maxy:
            raise ValueError("Polygon too small after margin; reduce margin_m")

        along_m, across_m = self.camera.footprint(alt_m)
        # spacing derived from overlaps
        line_spacing = max(0.5, across_m * max(0.05, (1.0 - overlap_across)))
        step_m = max(0.5, along_m * max(0.05, (1.0 - overlap_along)))

        wps_tf: List[Tuple[float, float]] = []
        y = miny
        direction = 1
        while y <= maxy + 1e-6:
            # sample along x
            inside_x = []
            x = minx
            while x <= maxx + 1e-6:
                if point_in_poly(x, y, area_tf):
                    inside_x.append(x)
                x += step_m

            if inside_x:
                x0, x1 = min(inside_x), max(inside_x)
                x_list = self._frange(x0, x1, step_m)
                if direction < 0:
                    x_list = list(reversed(x_list))
                for xx in x_list:
                    wps_tf.append((xx, y))
                    if len(wps_tf) >= max_points:
                        y = maxy + 1e9
                        break
                direction *= -1

            y += line_spacing

        # back to latlon
        wps: List[Waypoint] = []
        for xp, yp in wps_tf:
            x_enu, y_enu = rotate_from_track(xp, yp, heading_deg)
            ll = enu_to_latlon(x_enu, y_enu, ref)
            wps.append(Waypoint(ll.lat, ll.lon, alt_m, yaw_deg=yaw_deg or heading_deg, speed_mps=speed_mps))
        return wps

    @staticmethod
    def _frange(a: float, b: float, step: float) -> List[float]:
        out = []
        x = a
        while x <= b + 1e-6:
            out.append(x)
            x += step
        return out if out else [a]


# -----------------------------
# Drone control: pymavlink (course style)
# -----------------------------
class DronePymavlink:
    def __init__(self):
        self.master: Optional[mavutil.mavlink_connection] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._rx_stop = threading.Event()

        self._last_heartbeat = 0.0
        self._latest_gpos = None   # GLOBAL_POSITION_INT
        self._latest_sys = None    # SYS_STATUS

    # --- connection ---
    def connect(self, connection_str: str, baud: Optional[int] = None, timeout_s: float = 10.0) -> None:
        self.master = mavutil.mavlink_connection(connection_str, baud=baud)
        self.wait_heartbeat(timeout_s=timeout_s)

    def wait_heartbeat(self, timeout_s: float = 10.0) -> None:
        if not self.master:
            raise RuntimeError("Not connected")
        hb = self.master.wait_heartbeat(timeout=timeout_s)
        if hb is None:
            raise TimeoutError("No HEARTBEAT received")
        self._last_heartbeat = time.time()

    def start_rx(self) -> None:
        if not self.master:
            raise RuntimeError("Not connected")
        if self._rx_thread and self._rx_thread.is_alive():
            return
        self._rx_stop.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def stop_rx(self) -> None:
        self._rx_stop.set()
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)

    def heartbeat_age_s(self) -> float:
        return time.time() - self._last_heartbeat if self._last_heartbeat else 1e9

    def _rx_loop(self) -> None:
        assert self.master is not None
        while not self._rx_stop.is_set():
            msg = self.master.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                continue
            mtype = msg.get_type()
            if mtype == "HEARTBEAT":
                self._last_heartbeat = time.time()
            elif mtype == "GLOBAL_POSITION_INT":
                self._latest_gpos = msg
            elif mtype == "SYS_STATUS":
                self._latest_sys = msg

    # --- state getters ---
    def get_latest_position(self) -> Optional[LatLon]:
        if self._latest_gpos is None:
            return None
        lat = self._latest_gpos.lat / 1e7
        lon = self._latest_gpos.lon / 1e7
        return LatLon(lat, lon)

    # --- actions ---
    def set_mode(self, mode: str) -> None:
        if not self.master:
            raise RuntimeError("Not connected")
        self.master.set_mode(mode)

    def arm(self) -> None:
        if not self.master:
            raise RuntimeError("Not connected")
        self.master.arducopter_arm()
        self.master.motors_armed_wait()

    def disarm(self) -> None:
        if not self.master:
            raise RuntimeError("Not connected")
        self.master.arducopter_disarm()
        self.master.motors_disarmed_wait()

    def takeoff(self, rel_alt_m: float) -> None:
        """
        GUIDED takeoff (ArduCopter):
        MAV_CMD_NAV_TAKEOFF param7 = altitude (meters)
        """
        if not self.master:
            raise RuntimeError("Not connected")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            float(rel_alt_m)
        )

    def condition_yaw(self, yaw_deg: float, relative: bool = False, yaw_rate_dps: float = 20.0) -> None:
        if not self.master:
            raise RuntimeError("Not connected")
        is_relative = 1.0 if relative else 0.0
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            float(yaw_deg),
            float(yaw_rate_dps),
            1.0,  # direction: 1=cw, -1=ccw (you can change)
            is_relative,
            0, 0, 0
        )

    def goto(self, lat_deg: float, lon_deg: float, rel_alt_m: float,
             yaw_deg: Optional[float] = None, speed_mps: Optional[float] = None) -> None:
        """
        GUIDED position setpoint:
        SET_POSITION_TARGET_GLOBAL_INT with RELATIVE_ALT frame.
        """
        if not self.master:
            raise RuntimeError("Not connected")

        # type_mask: ignore velocity/accel/yaw_rate; optionally include yaw
        IGNORE_VX = 1 << 3
        IGNORE_VY = 1 << 4
        IGNORE_VZ = 1 << 5
        IGNORE_AFX = 1 << 6
        IGNORE_AFY = 1 << 7
        IGNORE_AFZ = 1 << 8
        FORCE = 1 << 9
        IGNORE_YAW_RATE = 1 << 11

        if yaw_deg is None:
            IGNORE_YAW = 1 << 10
            type_mask = (IGNORE_VX | IGNORE_VY | IGNORE_VZ |
                         IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ |
                         FORCE | IGNORE_YAW | IGNORE_YAW_RATE)
            yaw = 0.0
        else:
            type_mask = (IGNORE_VX | IGNORE_VY | IGNORE_VZ |
                         IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ |
                         FORCE | IGNORE_YAW_RATE)
            yaw = math.radians(float(yaw_deg))

        # optional speed: set via PARAM / DO_CHANGE_SPEED is more "correct".
        # Here we provide a lightweight command if you want it.
        if speed_mps is not None:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                0,
                1, float(speed_mps), -1, 0, 0, 0, 0
            )

        self.master.mav.set_position_target_global_int_send(
            int(time.time() * 1e3) & 0xFFFFFFFF,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            type_mask,
            int(lat_deg * 1e7),
            int(lon_deg * 1e7),
            float(rel_alt_m),
            0, 0, 0,     # vx,vy,vz
            0, 0, 0,     # ax,ay,az
            yaw,
            0.0          # yaw_rate
        )

    def rtl(self) -> None:
        self.set_mode("RTL")

    def set_home(self, lat_deg: float, lon_deg: float, alt_msl: float = 0.0) -> None:
        """
        Set vehicle HOME so RTL returns to your TOL.
        MAV_CMD_DO_SET_HOME param1=0 -> use specified location
        """
        if not self.master:
            raise RuntimeError("Not connected")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            0,
            0, 0, 0, 0,
            float(lat_deg), float(lon_deg), float(alt_msl)
        )


    def land(self) -> None:
        self.set_mode("LAND")

    def kill_motors(self) -> None:
        """
        Emergency: MAV_CMD_COMPONENT_ARM_DISARM with param2=21196 (kill).
        Use with caution.
        """
        if not self.master:
            raise RuntimeError("Not connected")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,    # disarm
            21196,# magic for force/killswitch
            0, 0, 0, 0, 0
        )


# -----------------------------
# Mission controller (state machine + watchdog)
# -----------------------------
class MissionState(Enum):
    DISCONNECTED = "DISCONNECTED"
    CONNECTED = "CONNECTED"
    READY = "READY"
    ARMED = "ARMED"
    TAKEOFF = "TAKEOFF"
    MISSION = "MISSION"
    RTL = "RTL"
    LAND = "LAND"
    FAILSAFE = "FAILSAFE"

class MissionController:
    def __init__(self, flight_area_poly: Sequence[LatLon], camera: CameraFootprintModel):
        if len(flight_area_poly) < 3:
            raise ValueError("flight_area_poly must have >=3 vertices")
        self.flight_area = list(flight_area_poly)
        self.camera = camera
        self.planner = LawnmowerPlanner(camera)

        self.drone = DronePymavlink()
        self.state = MissionState.DISCONNECTED

        self._watchdog_stop = threading.Event()
        self._watchdog_thread: Optional[threading.Thread] = None

    # ---- core callable APIs ----
    def connect_and_start(self, connection_str: str, timeout_s: float = 10.0) -> None:
        self.drone.connect(connection_str, timeout_s=timeout_s)
        self.drone.start_rx()
        self.state = MissionState.CONNECTED
        self._start_watchdog()

    def arm_takeoff(self, rel_alt_m: float) -> None:
        self._require_connected()
        self._require_within_boundary_takeoff()

        self.drone.set_mode("GUIDED")
        self.drone.arm()
        self.state = MissionState.ARMED

        self.drone.takeoff(rel_alt_m)
        self.state = MissionState.TAKEOFF
        # simple wait: wait position rel alt rising if available
        time.sleep(3.0)
        self.state = MissionState.MISSION

    def plan_lawnmower(
        self,
        area_poly: Optional[Sequence[LatLon]] = None,
        alt_m: float = 30.0,
        overlap_along: float = 0.7,
        overlap_across: float = 0.6,
        heading_deg: Optional[float] = None,
        speed_mps: float = 4.0,
        yaw_deg: Optional[float] = None,
    ) -> List[Waypoint]:
        poly = list(area_poly) if area_poly is not None else self.flight_area
        return self.planner.plan(
            area_poly=poly,
            alt_m=alt_m,
            overlap_along=overlap_along,
            overlap_across=overlap_across,
            heading_deg=heading_deg,
            speed_mps=speed_mps,
            yaw_deg=yaw_deg,
        )

    def run_waypoints(self, waypoints: Sequence[Waypoint], arrive_radius_m: float = 3.0, timeout_each_s: float = 120.0) -> None:
        self._require_connected()
        self.state = MissionState.MISSION

        for i, wp in enumerate(waypoints, 1):
            self._assert_within_boundary(LatLon(wp.lat, wp.lon))
            self.drone.goto(wp.lat, wp.lon, wp.rel_alt_m, yaw_deg=wp.yaw_deg, speed_mps=wp.speed_mps)
            self._wait_arrival(LatLon(wp.lat, wp.lon), arrive_radius_m, timeout_each_s)

    def run_search(self, alt_m: float = 30.0, overlap_along: float = 0.7, overlap_across: float = 0.6, heading_deg: Optional[float] = None) -> None:
        wps = self.plan_lawnmower(alt_m=alt_m, overlap_along=overlap_along, overlap_across=overlap_across, heading_deg=heading_deg)
        self.run_waypoints(wps)

    def rth(self) -> None:
        self._require_connected()
        self.state = MissionState.RTL
        self.drone.rtl()

    def land(self) -> None:
        self._require_connected()
        self.state = MissionState.LAND
        self.drone.land()

    def abort_to_rtl(self, reason: str) -> None:
        # called by watchdog
        self.state = MissionState.FAILSAFE
        try:
            self.drone.rtl()
        except Exception:
            # last resort
            self.drone.kill_motors()

    def stop(self) -> None:
        self._watchdog_stop.set()
        if self._watchdog_thread:
            self._watchdog_thread.join(timeout=1.0)
        self.drone.stop_rx()

    # ---- internal helpers ----
    def _start_watchdog(self, hb_timeout_s: float = 2.5, check_period_s: float = 0.2) -> None:
        def _loop():
            while not self._watchdog_stop.is_set():
                age = self.drone.heartbeat_age_s()
                if age > hb_timeout_s:
                    self.abort_to_rtl(f"heartbeat lost age={age:.1f}s")
                    return
                time.sleep(check_period_s)

        self._watchdog_stop.clear()
        self._watchdog_thread = threading.Thread(target=_loop, daemon=True)
        self._watchdog_thread.start()

    def _require_connected(self) -> None:
        if self.state in (MissionState.DISCONNECTED,):
            raise RuntimeError("Not connected")

    def _assert_within_boundary(self, p: LatLon) -> None:
        # boundary check in ENU
        ref = self.flight_area[0]
        poly_xy = [latlon_to_enu_m(x, ref) for x in self.flight_area]
        px, py = latlon_to_enu_m(p, ref)
        if not point_in_poly(px, py, poly_xy):
            raise ValueError("Target point is outside flight boundary polygon")

    def _require_within_boundary_takeoff(self) -> None:
        pos = self.drone.get_latest_position()
        if pos is None:
            # If no position yet, just allow (you can tighten this later)
            return
        self._assert_within_boundary(pos)

    def _wait_arrival(self, target: LatLon, radius_m: float, timeout_s: float) -> None:
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            cur = self.drone.get_latest_position()
            if cur is not None:
                if haversine_m(cur, target) <= radius_m:
                    return
            time.sleep(0.2)
        raise TimeoutError("Arrival timeout")


# -----------------------------
# Example usage
# -----------------------------
if __name__ == "__main__":
    # Replace with your flight boundary polygon
    flight_area = [
        LatLon(51.423561, -2.671297),
        LatLon(51.422842, -2.670066),
        LatLon(51.424173, -2.668757),
        LatLon(51.423444, -2.668178),
    ]

    camera = CameraFootprintModel(ref_alt_m=50.0, ref_right_m=52.4, ref_forward_m=39.3)
    mc = MissionController(flight_area_poly=flight_area, camera=camera)

    mc.connect_and_start("tcp:127.0.0.1:14550")

    mc.arm_takeoff(rel_alt_m=30.0)

    # Search with altitude-dependent footprint spacing
    wps = mc.plan_lawnmower(alt_m=30.0, overlap_along=0.7, overlap_across=0.6)
    mc.run_waypoints(wps)

    mc.rth()
    mc.stop()
