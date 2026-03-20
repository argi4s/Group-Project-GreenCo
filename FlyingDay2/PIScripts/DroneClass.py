# Importable class for FSM integration
# Handles MAVLink connection, telemetry, blocking flight commands, RTL and emergency interrupts

from pymavlink import mavutil
import threading
import time
import math
from typing import Tuple, Optional

LonLatAlt = Tuple[float, float, float]  # (lon, lat, alt_m) - CHANGED: lon first

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
        self._ground_speed = None
        self._air_speed = None
        self._vz = None
        self._roll = None
        self._pitch = None
        self._yaw = None
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
                    time.sleep(0.05)
                    continue

                with self._lock:
                    if msg.get_type() == "HEARTBEAT":
                        self._mode = mavutil.mode_string_v10(msg)
                        self._armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0

                    elif msg.get_type() == "GLOBAL_POSITION_INT":
                        self._lat = msg.lat / 1e7
                        self._lon = msg.lon / 1e7
                        self._alt = msg.relative_alt / 1000.0
                        self._heading = msg.hdg / 100.0 if msg.hdg != 65535 else 0

                    elif msg.get_type() == "NAV_CONTROLLER_OUTPUT":
                        self._wp_index = msg.wp_dist  # approximate distance to WP

                    elif msg.get_type() == "GPS_RAW_INT":
                        self._gps_fix = msg.fix_type

                    elif msg.get_type() == "VFR_HUD":
                        self._air_speed = msg.airspeed
                        self._ground_speed = msg.groundspeed
                        self._vz = msg.climb  # vertical speed

                    elif msg.get_type() == "ATTITUDE":
                        self._roll = msg.roll
                        self._pitch = msg.pitch
                        self._yaw = msg.yaw

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
            return (self._lon, self._lat, self._alt)  # CHANGED: lon first

    @property
    def heading(self):
        with self._lock:
            return self._heading

    @property
    def ground_speed(self):
        with self._lock:
            return self._ground_speed

    @property
    def air_speed(self):
        with self._lock:
            return self._air_speed

    @property
    def vertical_speed(self):
        with self._lock:
            return self._vz  # vertical velocity
    
    # ----------------------------
    # Flight Control
    # ----------------------------
    def set_mode(self, mode: str):
        """Set the drone mode."""
        if mode not in ["GUIDED", "AUTO"]:
            raise ValueError("This mode is not supported")

        self.mav.set_mode(mode)
        time.sleep(0.2)
        print(f"[INFO] Set mode to {mode}")

    def arm(self):
        self._check_interrupts()
        self.busy = True

        self.mav.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0)
        self.busy = False
        
    def disarm(self):
        self._check_interrupts()
        self.busy = True

        self.mav.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0
        )

    def takeoff(self, target_altitude_m: float):
        """Take off to a given altitude."""
        self._check_interrupts()

        if self.mode != "GUIDED":
            raise RuntimeError("Takeoff requires GUIDED mode")

        self.busy = True

        self.mav.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            target_altitude_m
        )

        self._wait_until_altitude(target_altitude_m)
        self.busy = False

    def land(self):
        """Land the drone."""
        self._check_interrupts()

        self.busy = True

        self.mav.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0, 0, 0, 0
        )

        self._wait_until_altitude(0.3)
        self.busy = False
    
    def landed(self) -> bool:
        """Check if drone is landed (altitude low and disarmed)."""
        with self._lock:
            return (self._alt is not None and 
                    self._alt < 0.5 and 
                    not self._armed)

    def rtl(self):
        """Return to launch (RTL)."""
        self._check_interrupts()

        self.busy = True
        self.rtl_flag = True

        self.mav.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,
            0, 0, 0, 0, 0, 0, 0
        )

        # Wait for RTL to complete
        while True:
            self._check_interrupts()
            lon, lat, alt = self.position  # CHANGED: lon first

            if alt is not None and alt < 0.3 and not self.armed:
                break

            time.sleep(0.5)

        # Clear the RTL flag AFTER successfully completing
        with self._lock:
            self.rtl_flag = False
    
        self.busy = False
        print("[INFO] RTL completed, flag cleared")

    def emergency(self):
        """Trigger emergency stop."""
        self.emergency_flag = True

        self.mav.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            211, 0, 0, 0, 0, 0
        )

    def set_speed(self, speed_m_s):
        """Set ground speed in meters per second."""
        self.mav.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0,
            1,  # Speed type (1 = groundspeed)
            speed_m_s,  # Speed value
            -1,  # Throttle (no change)
            0, 0, 0, 0, 0
        )
        time.sleep(0.1)
        print(f"[INFO] Speed set to {speed_m_s} m/s")

    def move_to_wp(self, lon, lat, alt, speed=None, heading=None):  # CHANGED: added heading
        """Move to a specific waypoint using GUIDED mode.
           If speed is provided, sets groundspeed before moving.
           If heading is provided, maintains that heading during movement.
        """
        self._check_interrupts()

        if self.mode != "GUIDED":
            raise RuntimeError("Waypoint navigation requires GUIDED mode")

        self.busy = True
        
        # Set speed if provided
        if speed is not None:
            self.set_speed(speed)

        while True:
            self._check_interrupts()

            # Pass heading to position target
            self.send_position_target(lon, lat, alt, heading)  # CHANGED: pass heading

            # Use the public distance_to method with (lon, lat) order
            dist = self.distance_to(lon, lat)  # CHANGED: lon first
            _, _, current_alt = self.position

            if dist < 2 and current_alt is not None and abs(current_alt - alt) < 1:
                break

            time.sleep(0.2)

        self.busy = False
        
    def move_to_wp_queue(self, waypoint_generator):
        """Move through a queue of waypoints fetched one at a time from the generator."""
        self._check_interrupts()

        # Process waypoints one at a time
        for i, wp in waypoint_generator():
            lon, lat, alt = wp  # CHANGED: lon first

            print(f"Moving to waypoint {i+1} -> ({lon:.7f},{lat:.7f},{alt:.2f}m)")  # CHANGED: lon first

            # Move to the current waypoint
            self.move_to_wp(lon, lat, alt)  # CHANGED: lon first

        print("All waypoints have been completed.")
        
    def upload_mission(self, waypoints):
        """
        Upload a list of waypoints to the autopilot so AUTO mode can execute them.
        Each waypoint is a tuple (lon, lat, alt_m).  # CHANGED: lon first
        """
        self._check_interrupts()
        self.busy = True

        # Clear existing mission
        self.mav.waypoint_clear_all_send()
        time.sleep(0.5)

        print(f"[INFO] Uploading {len(waypoints)} waypoints to mission...")

        for i, (lon, lat, alt) in enumerate(waypoints):  # CHANGED: lon first
            # MAVLink expects lon in x, lat in y
            self.mav.mav.mission_item_send(
                self.target_system,
                self.target_component,
                i,                          # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 0,                       # current, autocontinue
                0, 0, 0,                     # params 1-3 (hold, radius, yaw)
                int(lon * 1e7),               # CHANGED: lon first
                int(lat * 1e7),               # lat second
                alt
            )

            # Wait for acknowledgement
            ack = None
            retries = 5
            while retries > 0:
                msg = self.mav.recv_match(type='MISSION_ACK', blocking=True, timeout=1)
                if msg:
                    ack = msg
                    break
                retries -= 1
                print(f"[WARN] Waiting for MISSION_ACK for waypoint {i}... retries left {retries}")
            if ack is None:
                print(f"[ERROR] No ACK received for waypoint {i}, mission upload may fail")
        
        print("[INFO] Mission upload complete.")
        self.busy = False

    def send_position_target(self, lon, lat, alt, heading=None):  # CHANGED: added heading
        """Send position target to move the drone with optional fixed heading."""
        
        # Set up position target flags
        if heading is not None:
            # Position + yaw (ignore velocity, acceleration, yaw rate)
            type_mask = 0b0000110000111000  # Position + yaw
        else:
            # Position only, no yaw control
            type_mask = 0b0000111111111000  # Position only
    
        # Send the position target
        self.mav.mav.set_position_target_global_int_send(
            0,
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            type_mask,
            int(lat * 1e7),
            int(lon * 1e7),
            alt,
            0, 0, 0,  # velocities
            0, 0, 0,  # accelerations
            heading if heading is not None else 0,  # yaw
            0  # yaw rate
        )
    
        # Small delay to allow processing
        time.sleep(0.1)
    
        # Request the current target from autopilot
        self.mav.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT,
            0, 0, 0, 0, 0, 0
        )
    
        # Listen for response
        msg = self.mav.recv_match(type='POSITION_TARGET_GLOBAL_INT', blocking=True, timeout=2)
    
        # Write to debug file
        with open("waypoint_debug.txt", "a") as f:
            f.write(f"\n--- Position Target Debug ---\n")
            f.write(f"Command sent to: ({lon:.7f}, {lat:.7f}, {alt:.2f})\n")
            f.write(f"Heading: {heading if heading is not None else 'AUTO'}\n")
            f.write(f"Sent as ints: lon={int(lon*1e7)}, lat={int(lat*1e7)}\n")
       
            if msg:
                ap_lon = msg.lon_int / 1e7
                ap_lat = msg.lat_int / 1e7
                ap_alt = msg.alt
                f.write(f"Autopilot target: ({ap_lon:.7f}, {ap_lat:.7f}, {ap_alt:.2f})\n")
           
                # Calculate difference
                lon_diff = abs(lon - ap_lon) * 111320 * math.cos(math.radians(lat))
                lat_diff = abs(lat - ap_lat) * 111320
                total_diff = math.sqrt(lon_diff**2 + lat_diff**2)
                f.write(f"Difference: {lon_diff:.2f}m E/W, {lat_diff:.2f}m N/S, {total_diff:.2f}m total\n")
           
                if abs(lon - ap_lon) > 0.0001 or abs(lat - ap_lat) > 0.0001:
                    f.write(f"⚠️ MISMATCH DETECTED! Coordinates differ\n")
            else:
                f.write(f"❌ No POSITION_TARGET_GLOBAL_INT response from autopilot\n")
       
            f.write(f"--- End Debug ---\n\n")
   
        return msg
    
    def distance_to(self, lon, lat):  # CHANGED: lon first
        """Public wrapper for _distance_to - expects (lon, lat) order."""
        return self._distance_to(lon, lat)

    def _distance_to(self, lon, lat):  # CHANGED: lon first
        """Calculate distance to target using Haversine formula (accurate)."""
        current_lon, current_lat, _ = self.position  # CHANGED: lon first from position

        if current_lat is None or current_lon is None:
            return float("inf")

        # Convert to radians
        lat1 = math.radians(current_lat)
        lon1 = math.radians(current_lon)
        lat2 = math.radians(lat)
        lon2 = math.radians(lon)

        # Haversine formula
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        R = 6371000  # Earth's radius in meters

        distance = R * c

        # Debug
        with open("waypoint_debug.txt", "a") as f:
            f.write(f"distance_to: current=({current_lon:.7f},{current_lat:.7f}) ")  # CHANGED: lon first
            f.write(f"target=({lon:.7f},{lat:.7f}) ")  # CHANGED: lon first
            f.write(f"dist={distance:.2f}m\n")

        return distance

    def _wait_until_altitude(self, target_alt: float, tolerance: float = 0.5):
        """Wait until the drone reaches a target altitude."""
        while True:
            self._check_interrupts()

            _, _, alt = self.position
            if alt is not None and abs(alt - target_alt) <= tolerance:
                break

            time.sleep(0.1)

    def _distance_m(self, a: LonLatAlt, b: LonLatAlt) -> float:  # CHANGED: LonLatAlt type
        from math import radians, cos, sqrt
        R = 6371000.0

        lon1, lat1, _ = a  # CHANGED: lon first
        lon2, lat2, _ = b  # CHANGED: lon first

        dlat = radians(lat2 - lat1)
        dlon = radians(lon2 - lon1)

        lat1r = radians(lat1)
        lat2r = radians(lat2)

        x = dlon * cos((lat1r + lat2r) / 2.0)
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