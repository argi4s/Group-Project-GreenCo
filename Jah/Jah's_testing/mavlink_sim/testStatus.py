from pymavlink import mavutil
import time
import threading
from collections import deque

# ------------------------
# Connection
# ------------------------
mav = mavutil.mavlink_connection("tcp:127.0.0.1:14550")
mav.wait_heartbeat()
print("Connected")

# ------------------------
# Shared state
# ------------------------
current_mode = "UNKNOWN"
armed = False
status_buffer = deque(maxlen=10)
last_ack = None

lock = threading.Lock()
running = True

# ------------------------
# MAVLink receive loop (CRITICAL)
# ------------------------
def mavlink_rx_loop():
    global current_mode, armed, last_ack

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
                print(f"[ACK] command={msg.command} result={msg.result}")

# ------------------------
# Commands
# ------------------------
def arm():
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    print("[CMD] ARM")

# ------------------------
# Start RX thread
# ------------------------
rx_thread = threading.Thread(target=mavlink_rx_loop, daemon=True)
rx_thread.start()

# ------------------------
# CLI loop
# ------------------------
print("Type: status | arm | exit")

while True:
    cmd = input("> ").strip().lower()

    with lock:
        if cmd == "exit":
            running = False
            break

        elif cmd == "status":
            print(f"Mode: {current_mode}")
            print(f"Armed: {armed}")

        elif cmd == "arm":
            status_buffer.clear()
            last_ack = None
            arm()

            time.sleep(1.5)  # allow messages to arrive

            if last_ack and last_ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("[RESULT] Arm accepted")
            else:
                print("[RESULT] Arm failed")
                for s in status_buffer:
                    print(" -", s)
