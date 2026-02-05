#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
pymavlink 可运行“起飞 -> 航点 -> 降落”模板（偏 ArduPilot Copter / MissionPlanner / GUIDED 风格）

默认连接串：tcp:127.0.0.1:14550
（和 BristolFlightLab/PymavlinkDemo README 里 MissionPlanner 通过 TCP Host 14550 输出 MAVLink 的方式一致）:contentReference[oaicite:0]{index=0}

✅ 流程：
1) 连接 MAVLink + wait_heartbeat
2) (可选) 等 GPS 3D fix
3) 切 GUIDED
4) ARM
5) MAV_CMD_NAV_TAKEOFF 到相对高度 alt（相对 Home）
6) 逐个航点：SET_POSITION_TARGET_GLOBAL_INT (GLOBAL_RELATIVE_ALT_INT)
7) 切 LAND 并等待解锁（best-effort）

⚠️ 建议先在模拟器跑通，再上真机。
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

from pymavlink import mavutil


EARTH_RADIUS_M = 6378137.0  # WGS84


@dataclass
class Waypoint:
    lat_deg: float
    lon_deg: float
    rel_alt_m: float  # meters above home


def is_network_link(conn: str) -> bool:
    return conn.startswith(("tcp:", "udp:", "udpin:", "udpout:"))


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in meters."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return EARTH_RADIUS_M * c


def offset_latlon(lat_deg: float, lon_deg: float, d_north_m: float, d_east_m: float) -> Tuple[float, float]:
    """把局部 N/E (米) 偏移换算到新的 lat/lon（小范围近似）。"""
    lat_rad = math.radians(lat_deg)
    dlat = d_north_m / EARTH_RADIUS_M
    dlon = d_east_m / (EARTH_RADIUS_M * math.cos(lat_rad) + 1e-12)
    return lat_deg + math.degrees(dlat), lon_deg + math.degrees(dlon)


def start_gcs_heartbeat(master: mavutil.mavfile, stop_evt: threading.Event, hz: float = 1.0) -> threading.Thread:
    """
    给飞控发 GCS heartbeat（约 1Hz）。有些场景下组件需要持续 heartbeat，避免 failsafe。:contentReference[oaicite:1]{index=1}
    """
    period = 1.0 / max(hz, 0.1)

    def loop():
        while not stop_evt.is_set():
            try:
                master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,  # base_mode
                    0,  # custom_mode
                    mavutil.mavlink.MAV_STATE_ACTIVE,
                )
            except Exception:
                pass
            stop_evt.wait(period)

    t = threading.Thread(target=loop, name="gcs_heartbeat", daemon=True)
    t.start()
    return t


def request_data_streams(master: mavutil.mavfile, rate_hz: int = 10) -> None:
    """请求飞控按一定频率推送数据，确保能收到 GLOBAL_POSITION_INT 等。"""
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        rate_hz,
        1,
    )


def wait_heartbeat(master: mavutil.mavfile, timeout_s: float = 30.0) -> None:
    master.wait_heartbeat(timeout=timeout_s)


def wait_gps_fix(master: mavutil.mavfile, timeout_s: float = 60.0, min_fix_type: int = 3, min_sats: int = 6) -> None:
    """等待 GPS 3D fix（best-effort）。README 也提到刚开模拟器时可能需要等定位/滤波稳定再 ARM。:contentReference[oaicite:2]{index=2}"""
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        msg = master.recv_match(type="GPS_RAW_INT", blocking=True, timeout=2.0)
        if msg is None:
            continue
        fix_type = int(getattr(msg, "fix_type", 0))
        sats = int(getattr(msg, "satellites_visible", 0))
        if fix_type >= min_fix_type and sats >= min_sats:
            return
    raise TimeoutError(f"GPS fix not ready after {timeout_s:.0f}s (need fix_type>={min_fix_type}, sats>={min_sats})")


def set_mode(master: mavutil.mavfile, mode: str, timeout_s: float = 10.0) -> None:
    """
    通过 set_mode_send 设置飞行模式（例如 GUIDED/LAND/AUTO）。
    这类写法在 pymavlink 实战里很常见：custom_flag + mode_mapping()["GUIDED"]。:contentReference[oaicite:3]{index=3}
    """
    mapping = master.mode_mapping() or {}
    if mode not in mapping:
        raise ValueError(f"Mode {mode!r} not in mode_mapping(): {sorted(mapping.keys())}")

    mode_id = mapping[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )

    t0 = time.time()
    while time.time() - t0 < timeout_s:
        master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if master.flightmode == mode:
            return
    raise TimeoutError(f"Failed to switch to mode {mode!r} within {timeout_s:.0f}s (current: {master.flightmode!r})")


def command_long_ack(
    master: mavutil.mavfile,
    command: int,
    params: Tuple[float, float, float, float, float, float, float],
    timeout_s: float = 10.0,
) -> None:
    """发送 COMMAND_LONG，并等待对应 COMMAND_ACK（best-effort）。"""
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        command,
        0,
        *params,
    )

    t0 = time.time()
    while time.time() - t0 < timeout_s:
        ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=1.0)
        if ack is None:
            continue
        if int(getattr(ack, "command", -1)) == int(command):
            result = int(getattr(ack, "result", -1))
            if result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                raise RuntimeError(f"COMMAND_ACK {command} rejected, result={result}")
            return
    return


def arm(master: mavutil.mavfile, timeout_s: float = 15.0) -> None:
    command_long_ack(
        master,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        (1, 0, 0, 0, 0, 0, 0),
        timeout_s=timeout_s,
    )
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if master.motors_armed():
            return
    raise TimeoutError("Vehicle did not arm in time")


def takeoff(master: mavutil.mavfile, target_alt_m: float, timeout_s: float = 60.0) -> None:
    """
    GUIDED 起飞：MAV_CMD_NAV_TAKEOFF，param7 为高度。
    注：不同飞控栈对 takeoff altitude 的解释可能有差异；ArduPilot/常见 Copter 场景通常能按相对高度工作。:contentReference[oaicite:4]{index=4}
    """
    command_long_ack(
        master,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        (0, 0, 0, 0, 0, 0, float(target_alt_m)),
        timeout_s=10.0,
    )

    t0 = time.time()
    while time.time() - t0 < timeout_s:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2.0)
        if msg is None:
            continue
        rel_alt_m = float(getattr(msg, "relative_alt", 0)) / 1000.0  # mm -> m :contentReference[oaicite:5]{index=5}
        if rel_alt_m >= 0.95 * target_alt_m:
            return
    raise TimeoutError(f"Takeoff did not reach {target_alt_m} m within {timeout_s:.0f}s")


def goto_global_rel_alt(
    master: mavutil.mavfile,
    lat_deg: float,
    lon_deg: float,
    rel_alt_m: float,
    yaw_deg: Optional[float] = None,
) -> None:
    """
    发送 SET_POSITION_TARGET_GLOBAL_INT（GLOBAL_RELATIVE_ALT_INT），相当于“指哪飞哪”。
    这种用法在 DroneKit 的 guided 例子里也对应同一条 MAVLink 消息。:contentReference[oaicite:6]{index=6}
    """
    # type_mask bits (ignore fields)
    IGNORE_VX = 1 << 3
    IGNORE_VY = 1 << 4
    IGNORE_VZ = 1 << 5
    IGNORE_AX = 1 << 6
    IGNORE_AY = 1 << 7
    IGNORE_AZ = 1 << 8
    IGNORE_YAW = 1 << 10
    IGNORE_YAW_RATE = 1 << 11

    type_mask = IGNORE_VX | IGNORE_VY | IGNORE_VZ | IGNORE_AX | IGNORE_AY | IGNORE_AZ | IGNORE_YAW_RATE

    if yaw_deg is None:
        type_mask |= IGNORE_YAW
        yaw = 0.0
    else:
        yaw = math.radians(float(yaw_deg))

    master.mav.set_position_target_global_int_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        type_mask,
        int(lat_deg * 1e7),
        int(lon_deg * 1e7),
        float(rel_alt_m),
        0, 0, 0,
        0, 0, 0,
        yaw,
        0.0,
    )


def wait_reach_waypoint(
    master: mavutil.mavfile,
    target: Waypoint,
    pos_tol_m: float = 2.0,
    alt_tol_m: float = 1.0,
    timeout_s: float = 120.0,
) -> None:
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2.0)
        if msg is None:
            continue
        lat = float(getattr(msg, "lat", 0)) / 1e7
        lon = float(getattr(msg, "lon", 0)) / 1e7
        rel_alt = float(getattr(msg, "relative_alt", 0)) / 1000.0
        d = haversine_m(lat, lon, target.lat_deg, target.lon_deg)
        if d <= pos_tol_m and abs(rel_alt - target.rel_alt_m) <= alt_tol_m:
            return
    raise TimeoutError(f"Did not reach waypoint within {timeout_s:.0f}s")


def get_current_global(master: mavutil.mavfile, timeout_s: float = 10.0) -> Tuple[float, float, float]:
    """从 GLOBAL_POSITION_INT 读当前 (lat, lon, rel_alt)。"""
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2.0)
        if msg is None:
            continue
        lat = float(getattr(msg, "lat", 0)) / 1e7
        lon = float(getattr(msg, "lon", 0)) / 1e7
        rel_alt = float(getattr(msg, "relative_alt", 0)) / 1000.0
        if abs(lat) > 1e-6 or abs(lon) > 1e-6:
            return lat, lon, rel_alt
    raise TimeoutError("No valid GLOBAL_POSITION_INT received")


def land(master: mavutil.mavfile, timeout_s: float = 120.0) -> None:
    """切 LAND 并等待解锁（best-effort）。"""
    try:
        set_mode(master, "LAND", timeout_s=10.0)
    except Exception:
        command_long_ack(
            master,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            (0, 0, 0, 0, 0, 0, 0),
            timeout_s=5.0,
        )

    t0 = time.time()
    while time.time() - t0 < timeout_s:
        master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if not master.motors_armed():
            return
    return


def parse_wp_ne_list(wp_ne: List[str]) -> List[Tuple[float, float]]:
    out = []
    for s in wp_ne:
        parts = [p.strip() for p in s.split(",")]
        if len(parts) != 2:
            raise ValueError(f"--wp-ne expects 'north_m,east_m', got: {s!r}")
        out.append((float(parts[0]), float(parts[1])))
    return out


def parse_wp_lla_list(wp_lla: List[str]) -> List[Tuple[float, float, float]]:
    out = []
    for s in wp_lla:
        parts = [p.strip() for p in s.split(",")]
        if len(parts) != 3:
            raise ValueError(f"--wp-lla expects 'lat,lon,rel_alt_m', got: {s!r}")
        out.append((float(parts[0]), float(parts[1]), float(parts[2])))
    return out


def build_default_square_ne(size_m: float = 30.0) -> List[Tuple[float, float]]:
    """默认绕当前点飞一个 30m 正方形，再回到起点。"""
    return [(size_m, 0.0), (size_m, size_m), (0.0, size_m), (0.0, 0.0)]


def main():
    ap = argparse.ArgumentParser(description="pymavlink: takeoff -> waypoints -> land template (GUIDED).")
    ap.add_argument("--connect", default="tcp:127.0.0.1:14550", help="e.g. tcp:127.0.0.1:14550 or /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200, help="baudrate for serial")
    ap.add_argument("--alt", type=float, default=10.0, help="takeoff/waypoint relative altitude (m above home)")
    ap.add_argument("--wp-ne", action="append", default=[], help="repeatable: 'north_m,east_m'")
    ap.add_argument("--wp-lla", action="append", default=[], help="repeatable: 'lat,lon,rel_alt_m'")
    ap.add_argument("--pos-tol", type=float, default=2.0, help="position tolerance (m)")
    ap.add_argument("--alt-tol", type=float, default=1.0, help="altitude tolerance (m)")
    ap.add_argument("--no-gps-wait", action="store_true", help="skip GPS fix wait")
    ap.add_argument("--no-heartbeat", action="store_true", help="do not send GCS heartbeat")
    ap.add_argument("--verbose", action="store_true", help="print debug details")
    args = ap.parse_args()

    # connect
    if is_network_link(args.connect):
        master = mavutil.mavlink_connection(args.connect, autoreconnect=True)
    else:
        master = mavutil.mavlink_connection(args.connect, baud=args.baud, autoreconnect=True)

    print(f"[1/6] Connecting: {args.connect}")
    wait_heartbeat(master, timeout_s=30.0)
    print(f"[OK] Heartbeat received. system={master.target_system} component={master.target_component}")

    hb_stop = threading.Event()
    hb_thread = None
    if not args.no_heartbeat:
        hb_thread = start_gcs_heartbeat(master, hb_stop, hz=1.0)

    request_data_streams(master, rate_hz=10)

    # Ctrl+C -> LAND
    def _sigint(_sig, _frame):
        print("\n[!] Ctrl+C -> switching to LAND (best-effort) ...")
        try:
            land(master)
        except Exception as e:
            print(f"[!] LAND failed: {e}")
        finally:
            hb_stop.set()
            sys.exit(0)

    signal.signal(signal.SIGINT, _sigint)

    # GPS wait
    if not args.no_gps_wait:
        print("[2/6] Waiting for GPS fix ...")
        try:
            wait_gps_fix(master, timeout_s=60.0)
            print("[OK] GPS fix ready.")
        except Exception as e:
            print(f"[WARN] GPS wait skipped/failed: {e}")

    # current position
    print("[3/6] Reading current position ...")
    home_lat, home_lon, home_alt = get_current_global(master, timeout_s=15.0)
    if args.verbose:
        print(f"[DBG] Current: lat={home_lat:.7f}, lon={home_lon:.7f}, rel_alt={home_alt:.1f}m")

    # build waypoints
    wps: List[Waypoint] = []
    if args.wp_lla:
        for lat, lon, rel_alt in parse_wp_lla_list(args.wp_lla):
            wps.append(Waypoint(lat, lon, rel_alt))
    else:
        ne_list = parse_wp_ne_list(args.wp_ne) if args.wp_ne else build_default_square_ne(size_m=30.0)
        for dn, de in ne_list:
            lat, lon = offset_latlon(home_lat, home_lon, dn, de)
            wps.append(Waypoint(lat, lon, float(args.alt)))

    # guided + arm + takeoff
    print("[4/6] Setting GUIDED ...")
    set_mode(master, "GUIDED", timeout_s=10.0)
    print("[OK] Mode = GUIDED")

    print("[5/6] Arming ...")
    arm(master, timeout_s=20.0)
    print("[OK] Armed")

    print(f"[6/6] Takeoff to {args.alt:.1f} m ...")
    takeoff(master, target_alt_m=float(args.alt), timeout_s=90.0)
    print("[OK] Reached takeoff altitude")

    # fly waypoints
    for i, wp in enumerate(wps, 1):
        print(f"[WP {i}/{len(wps)}] Go to lat={wp.lat_deg:.7f}, lon={wp.lon_deg:.7f}, alt={wp.rel_alt_m:.1f}m")
        goto_global_rel_alt(master, wp.lat_deg, wp.lon_deg, wp.rel_alt_m)
        wait_reach_waypoint(master, wp, pos_tol_m=float(args.pos_tol), alt_tol_m=float(args.alt_tol), timeout_s=180.0)
        print(f"[OK] Reached WP {i}")

    # land
    print("[DONE] Landing ...")
    land(master, timeout_s=180.0)
    print("[OK] Landed / Disarmed (or LAND mode set)")

    hb_stop.set()
    if hb_thread:
        hb_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()


"""
怎么跑（按你给的 repo 思路）

MissionPlanner 里把模拟器的 MAVLink 输出设成 TCP Host - 14550，并勾上 Write access，脚本用 tcp:127.0.0.1:14550 连接。

安装：python -m pip install pymavlink（repo 也这么装）。

运行（默认飞 30m 正方形）：

python mission_template.py --connect tcp:127.0.0.1:14550 --alt 10


自定义航点（相对当前点 N/E 偏移，单位米）：

python mission_template.py --connect tcp:127.0.0.1:14550 --alt 15 \
  --wp-ne 30,0 --wp-ne 30,30 --wp-ne 0,30 --wp-ne 0,0


直接给经纬度航点（lat,lon,相对高度m）：

python mission_template.py --connect tcp:127.0.0.1:14550 \
  --wp-lla 51.4235000,-2.6715000,15 --wp-lla 51.4236000,-2.6713000,15
"""