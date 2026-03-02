from pymavlink import mavutil
import time
import threading
from collections import deque
import math

# Connection
mav = mavutil.mavlink_connection("tcp:127.0.0.1:14550")
mav.wait_heartbeat()
print("Connected")


# Waypoints (lat, lon, alt)
WAYPOINTS = [
    (51.4233850, -2.6716371, 49.35),
    (51.4237044, -2.6669526, 10.0),
    (51.4227243, -2.6662177, 10.0),
    (51.4217375, -2.6699513, 10.0),
    (51.4233848, -2.6716378, 10.0),
]

current_lat = None
current_lon = None
current_alt = None

# Shared state
current_mode = "UNKNOWN"
armed = False
status_buffer = deque(maxlen=10)
last_ack = None

lock = threading.Lock()
running = True

# ADDED: ACK synchronization primitives
ack_event = threading.Event()      # Signals when an ACK is received
ack_result = None                  # Stores result of last ACK

# MAVLink receive loop
def mavlink_rx_loop():
    global current_mode, armed, last_ack, ack_result
    global current_lat, current_lon, current_alt

    while running:
        msg = mav.recv_match(blocking=True, timeout=1)
        if not msg:
            continue

        t = msg.get_type()

        with lock:
            if t == "HEARTBEAT":
                current_mode = mavutil.mode_string_v10(msg)
                armed = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )

            elif t == "STATUSTEXT":
                status_buffer.append(msg.text)
                print(f"[STATUSTEXT] {msg.text}")

            elif t == "COMMAND_ACK":
                last_ack = msg
                ack_result = msg.result          # ADDED: store ACK result
                ack_event.set()                  # ADDED: signal ACK arrival
                print(f"[ACK] command={msg.command} result={msg.result}")

            elif t == "GLOBAL_POSITION_INT":
                current_lat = msg.lat / 1e7
                current_lon = msg.lon / 1e7
                current_alt = msg.relative_alt / 1000.0


# Commands
def arm():
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    print("[CMD] ARM")

# ADDED: ACK timeout–based arm wrapper
def arm_with_ack_timeout(timeout=3):
    global ack_result

    ack_event.clear()       # Clear previous ACK state
    ack_result = None

    arm()                   # Send arm command

    # Wait for ACK event instead of sleeping
    if ack_event.wait(timeout):
        return ack_result
    else:
        return None         # Timeout occurred

def set_mode_ack(mode, timeout=5):
    """
    Set mode and wait until HEARTBEAT confirms it.
    ACK is best-effort, HEARTBEAT is truth.
    """
    ack_event.clear()

    mav.set_mode_apm(mode)

    start = time.time()
    while time.time() - start < timeout:
        if current_mode == mode:
            print(f"[MODE] {mode} confirmed")
            return True
        time.sleep(0.1)

    print(f"[MODE FAIL] {mode} not confirmed")
    return False

def send_position_target(lat, lon, alt):
    mav.mav.set_position_target_global_int_send(
        0,                                  # time_boot_ms (ignored)
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,                 # ignore vel/acc/yaw
        int(lat * 1e7),
        int(lon * 1e7),
        alt,
        0, 0, 0,                            # velocity
        0, 0, 0,                            # acceleration
        0, 0                                # yaw, yaw_rate
    )

def distance_to(lat, lon):
    if current_lat is None or current_lon is None:
        return float("inf")

    dlat = current_lat - lat
    dlon = current_lon - lon
    return math.sqrt(dlat*dlat + dlon*dlon) * 111139


def travel_waypoints():
    print("[TRAVEL] Switching to GUIDED")
    if not set_mode_ack("GUIDED"):
        print("[TRAVEL] Failed to enter GUIDED")
        return

    for i, (lat, lon, alt) in enumerate(WAYPOINTS):
        print(f"[TRAVEL] Going to WP {i}")
        send_position_target(lat, lon, alt)

        last_send = 0  # timestamp of last position command

        while True:
            # 1. Safety: operator override
            if current_mode != "GUIDED":
                print("[TRAVEL] Interrupted (mode changed)")
                return

            # 2. Periodically re-send target (GUIDED needs this)
            if time.time() - last_send > 1.0:
                send_position_target(lat, lon, alt)
                last_send = time.time()

            # 3. Check convergence
            dist = distance_to(lat, lon)

            if dist < 2.0 and abs(current_alt - alt) < 1.0:
                print(f"[TRAVEL] Reached WP {i}")
                break

            time.sleep(0.2)


    print("[TRAVEL] Completed → RTL")
    set_mode_ack("RTL")

# Start RX thread
rx_thread = threading.Thread(target=mavlink_rx_loop, daemon=True)
rx_thread.start()


# CLI loop
print("Type: mode | travel | status | arm | exit")  # main menu

while True:
    cmd = input("> ").strip().lower()

    if cmd == "exit":
        with lock:
            running = False
        break

    elif cmd == "status":
        with lock:
            print(f"Mode: {current_mode}")
            print(f"Armed: {armed}")
            print(f"Lat: {current_lat}")
            print(f"Lon: {current_lon}")
            print(f"Alt: {current_alt}")

    elif cmd == "travel":
        # IMPORTANT: do NOT hold the lock while traveling
        travel_waypoints()

    elif cmd.startswith("mode"):
        _, mode = cmd.split()
        set_mode_ack(mode.upper())

    elif cmd == "arm":
        with lock:
            status_buffer.clear()
            last_ack = None

        result = arm_with_ack_timeout(timeout=3)

        if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("[RESULT] Arm accepted")
        elif result is None:
            print("[RESULT] Arm failed: ACK timeout")
            with lock:
                for s in status_buffer:
                    print(" -", s)
        else:
            print("[RESULT] Arm failed")
            with lock:
                for s in status_buffer:
                    print(" -", s)
