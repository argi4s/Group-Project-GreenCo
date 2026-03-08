# Importable class for FSM integration
# Handles MAVLink connection, telemetry, blocking flight commands, RTL and emergency interrupts

from pymavlink import mavutil
import threading
import time
import math
from typing import Tuple, Optional

LatLonAlt = Tuple[float, float, float]  # (lat, lon, alt_m)

class Drone:
    def __init__(self, mavlink_uri: str = "udp:127.0.0.1:14552"):
        # MAVLink
        self.mav = mavutil.mavlink_connection(mavlink_uri)
        self.target_system = 1
        self.target_component = 1

        # Flight state / telemetry
        self._armed = False
        self._mode = ""
        self._gps_fix = 0
        self._lat = None
        self._lon = None
        self._alt = None
        self._heading = None
        self._wp_index = None

        # Interrupt flags
        self.emergency_flag = False
        self.rtl_flag = False

        # Internal
        self.running = True
        self.busy = False
        self._lock = threading.Lock()

        # Start telemetry updater
        threading.Thread(target=self._telemetry_loop, daemon=True).start()

        # Wait for heartbeat
        print(f"[INFO] Connecting to MAVLink on {mavlink_uri}...")
        self.mav.wait_heartbeat()
        print("[INFO] Heartbeat received.")

    # ----------------------------
    # Telemetry updater
    # ----------------------------
    def _telemetry_loop(self):
        while self.running:
            try:
                msg = self.mav.recv_match(blocking=False)
                if msg is None:
                    time.sleep(0.5)   # HOW OFTEN THE PI POLLS THE CUBE
                    continue

                with self._lock:
                    if msg.get_type() == "HEARTBEAT":
                        self._mode = mavutil.mode_string_v10(msg)
                        self._armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                    elif msg.get_type() == "GLOBAL_POSITION_INT":
                        self._lat = msg.lat / 1e7
                        self._lon = msg.lon / 1e7
                        self._alt = msg.relative_alt / 1000.0
                    elif msg.get_type() == "NAV_CONTROLLER_OUTPUT":
                        self._heading = msg.heading
                        self._wp_index = msg.wp_dist  # approximate distance to WP
                    elif msg.get_type() == "GPS_RAW_INT":
                        self._gps_fix = msg.fix_type
            except Exception:
                continue
            time.sleep(0.05)

    # ----------------------------
    # Properties
    # ----------------------------
    @property
    def armed(self):
        with self._lock:
            return self._armed

    @property
    def mode(self):
        with self._lock:
            return self._mode

    @property
    def gps_fix(self):
        with self._lock:
            return self._gps_fix

    @property
    def position(self):
        with self._lock:
            return (self._lat, self._lon, self._alt)

    @property
    def heading(self):
        with self._lock:
            return self._heading

    # ----------------------------
    # Flight Control
    # ----------------------------
    def set_mode(self, mode: str):
        """Set the drone mode to GUIDED."""
        if mode != "GUIDED":
            print("[INFO] Only 'GUIDED' mode will support automation functions.")
        if mode != "GUIDED"|"AUTO":
            raise ValueError ("This mode is not supported")
        self.mav.set_mode(mode)
        time.sleep(0.2)  # small delay for mode switch
        print(f"[INFO] Set mode to {mode}")

    def arm(self):
        self.mav.mav.command_long_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        while not self.emergency_flag and not self.armed:
            time.sleep(0.1)

    def disarm(self):
        self.mav.mav.command_long_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        while self.armed and not self.emergency_flag:
            time.sleep(0.1)

    def takeoff(self, target_altitude_m: float):
        """Take off to a given altitude."""
	self._check_interrupts()
        self.set_mode("GUIDED")
        self.busy = True
        self.mav.mav.command_long_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, target_altitude_m
        )
        self._wait_until_altitude(target_altitude_m)
        self.busy = False

    def land(self):
        """Land the drone."""
        self.set_mode("GUIDED")
        self.busy = True
        self.mav.mav.command_long_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        self._wait_until_altitude(0.5)
        self.busy = False

    def rtl(self):
        """Return to launch (RTL)."""
	self._check_interrupts()
        self.set_mode("GUIDED")
        self.rtl_flag = True
        self.mav.mav.command_long_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        while self.rtl_flag and not self.emergency_flag:
            time.sleep(0.1)

    def emergency(self):
        """Trigger emergency stop."""
        self.emergency_flag = True
        self.mav.mav.command_long_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 211, 0, 0, 0, 0, 0
        )

    def move_to_wp(self, lat, lon, alt):
        """Move to a specific waypoint using GUIDED mode."""
        #self.set_mode("GUIDED")                # Setting mode every time leads to clustered UDP instructions
	self._check_interrupts()        
	self.busy = True
        while True:
            self.send_position_target(lat, lon, alt)

            if self.distance_to(lat, lon) < 2 and abs(self.alt - alt) < 1:
                break

            time.sleep(0.2)

        self.busy = False

    def send_position_target(self, lat, lon, alt):
        """Send position target to move the drone."""
        self.mav.mav.set_position_target_global_int_send(
            0,                                  # time_boot_ms (ignored)
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,                 # ignore vel/acc/yaw
            int(lat * 1e7),
            int(lon * 1e7),
            alt,
            0, 0, 0,                            # velocity
            0, 0, 0,                            # acceleration
            0, 0                                # yaw, yaw_rate
        )
        self._wait_until_wp_reached(lat, lon, alt)

    def distance_to(self, lat, lon):
        """Calculate distance to the target location."""
        if self._lat is None or self._lon is None:
            return float("inf")

        dlat = self._lat - lat
        dlon = self._lon - lon
        return math.sqrt(dlat * dlat + dlon * dlon) * 111139  # Approximate distance in meters

    def _wait_until_altitude(self, target_alt: float, tolerance: float = 0.5):
        """Wait until the drone reaches a target altitude."""
        while not self.emergency_flag and not self.rtl_flag:
            _, _, alt = self.position
            if alt is not None and abs(alt - target_alt) <= tolerance:
                break
            time.sleep(0.1)

    def _wait_until_wp_reached(self, lat: float, lon: float, alt: float, tolerance_m: float = 1.0):
        while not self.emergency_flag and not self.rtl_flag:
            pos = self.position
            if pos[0] is None:
                time.sleep(0.1)
                continue
            dist = self._distance_m(pos, (lat, lon, alt))
            if dist <= tolerance_m:
                break
            time.sleep(0.1)

    def _distance_m(self, a: LatLonAlt, b: LatLonAlt) -> float:
        from math import radians, cos, sin, sqrt, atan2
        R = 6371000.0
        lat1, lon1, _ = a
        lat2, lon2, _ = b
        dlat = radians(lat2 - lat1)
        dlon = radians(lon2 - lon1)
        lat1r = radians(lat1)
        lat2r = radians(lat2)
        x = dlon * cos((lat1r + lat2r)/2.0)
        y = dlat
        return R * sqrt(x*x + y*y)

    def _check_interrupts(self):
        if self.emergency_flag:
            raise RuntimeError("Emergency active: command blocked")
        if self.rtl_flag:
            raise RuntimeError("RTL active: command blocked")

    def stop(self):
        """Stop the drone and cleanup."""
        self.running = False
