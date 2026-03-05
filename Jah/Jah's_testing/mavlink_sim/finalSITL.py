from pymavlink import mavutil
import time
import threading
from collections import deque
import math


class DroneController:

    def __init__(self, connection_string="tcp:127.0.0.1:14550"):

        # Connection
        self.mav = mavutil.mavlink_connection(connection_string)
        self.mav.wait_heartbeat()
        print("Connected")

        # Waypoints
        self.WAYPOINTS = [
            (51.4233850, -2.6716371, 49.35),
            (51.4237044, -2.6669526, 10.0),
            (51.4227243, -2.6662177, 10.0),
            (51.4217375, -2.6699513, 10.0),
            (51.4233848, -2.6716378, 10.0),
        ]

        self.wp_index = 0

        # Position
        self.current_lat = None
        self.current_lon = None
        self.current_alt = None

        # State
        self.current_mode = "UNKNOWN"
        self.armed = False

        self.status_buffer = deque(maxlen=10)

        self.lock = threading.Lock()
        self.running = True

        # ACK
        self.ack_event = threading.Event()
        self.ack_result = None

        # Start MAVLink RX thread
        self.rx_thread = threading.Thread(
            target=self.mavlink_rx_loop,
            daemon=True
        )
        self.rx_thread.start()

    # ===============================
    # MAVLink RECEIVE LOOP
    # ===============================

    def mavlink_rx_loop(self):

        while self.running:

            msg = self.mav.recv_match(blocking=True, timeout=1)

            if not msg:
                continue

            t = msg.get_type()

            with self.lock:

                if t == "HEARTBEAT":

                    self.current_mode = mavutil.mode_string_v10(msg)

                    self.armed = bool(
                        msg.base_mode &
                        mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )

                elif t == "STATUSTEXT":

                    self.status_buffer.append(msg.text)
                    print("[STATUSTEXT]", msg.text)

                elif t == "COMMAND_ACK":

                    self.ack_result = msg.result
                    self.ack_event.set()

                    print("[ACK]", msg.command, msg.result)

                elif t == "GLOBAL_POSITION_INT":

                    self.current_lat = msg.lat / 1e7
                    self.current_lon = msg.lon / 1e7
                    self.current_alt = msg.relative_alt / 1000.0

    # ===============================
    # ARM
    # ===============================

    def arm(self):

        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )

        print("[CMD] ARM")

    def arm_with_ack_timeout(self, timeout=3):

        self.ack_event.clear()
        self.ack_result = None

        self.arm()

        if self.ack_event.wait(timeout):
            return self.ack_result

        return None

    # ===============================
    # MODE
    # ===============================

    def set_mode_ack(self, mode, timeout=5):

        self.mav.set_mode_apm(mode)

        start = time.time()

        while time.time() - start < timeout:

            if self.current_mode == mode:

                print("[MODE]", mode, "confirmed")
                return True

            time.sleep(0.1)

        print("[MODE FAIL]", mode)

        return False

    # ===============================
    # POSITION TARGET
    # ===============================

    def send_position_target(self, lat, lon, alt):

        self.mav.mav.set_position_target_global_int_send(
            0,
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,
            int(lat * 1e7),
            int(lon * 1e7),
            alt,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )

    # ===============================
    # DISTANCE
    # ===============================

    def distance_to(self, lat, lon):

        if self.current_lat is None:
            return float("inf")

        dlat = self.current_lat - lat
        dlon = self.current_lon - lon

        return math.sqrt(dlat*dlat + dlon*dlon) * 111139

    # ===============================
    # NEXT WAYPOINT
    # ===============================

    def next_waypoint(self):

        if self.wp_index >= len(self.WAYPOINTS):

            print("[MISSION] Completed")
            return

        lat, lon, alt = self.WAYPOINTS[self.wp_index]

        print("[MISSION] Next WP", self.wp_index)
        print("LAT:", lat)
        print("LON:", lon)
        print("ALT:", alt)

        self.send_position_target(lat, lon, alt)

        self.wp_index += 1

    # ===============================
    # STATUS
    # ===============================

    def status(self):

        print("Mode:", self.current_mode)
        print("Armed:", self.armed)
        print("Lat:", self.current_lat)
        print("Lon:", self.current_lon)
        print("Alt:", self.current_alt)

    # ===============================
    # STOP
    # ===============================

    def stop(self):

        self.running = False