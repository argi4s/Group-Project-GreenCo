from pymavlink import mavutil
import threading
import time
from collections import deque
from enum import Enum, auto

# Mission States, so that FSM is "applied"
class MissionState(Enum):
    AUTO_MISSION = auto()
    DETECTED = auto()
    LANDING = auto()
    ON_GROUND = auto()
    PAYLOAD = auto()
    TAKEOFF = auto()
    RESUME = auto()

# Connection
mav = mavutil.mavlink_connection("tcp:127.0.0.1:14550") # SITL 
mav.wait_heartbeat()
print("Connected")

# Shared state
current_mode = "UNKNOWN"
armed = False
altitude = None

last_ack = None
ack_event = threading.Event()
status_buffer = deque(maxlen=10)

state = MissionState.AUTO_MISSION
running = True
lock = threading.Lock()

# MAVLink RX loop
def mavlink_rx():
    global current_mode, armed, altitude, last_ack

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

            elif t == "GLOBAL_POSITION_INT":
                altitude = msg.relative_alt / 1000.0

            elif t == "COMMAND_ACK":
                last_ack = msg
                ack_event.set()
                print(f"[ACK] cmd={msg.command} result={msg.result}")

            elif t == "STATUSTEXT":
                status_buffer.append(msg.text)
                print(f"[STATUSTEXT] {msg.text}")

# functions
def send_with_ack(send_fn, timeout=3):
    ack_event.clear()
    send_fn()
    return ack_event.wait(timeout)

def set_mode(mode):
    return send_with_ack(lambda: mav.mav.set_mode_send(
        mav.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mav.mode_mapping()[mode]
    ))

def land():
    return send_with_ack(lambda: mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    ))

def takeoff(alt=5):
    return send_with_ack(lambda: mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, alt
    ))

# Start RX thread
rx_thread = threading.Thread(target=mavlink_rx, daemon=True)
rx_thread.start()

# CLI   menu
print("""   
Commands:
 detect
 confirm_land
 release1
 release2
 confirm_takeoff
 resume
 status
 exit
""")

while True:
    cmd = input("> ").strip().lower()

    with lock:
        if cmd == "exit":
            running = False
            break

        elif cmd == "status":
            print(f"State: {state.name}")
            print(f"Mode: {current_mode}")
            print(f"Armed: {armed}")
            print(f"Alt: {altitude}")

        elif cmd == "detect" and state == MissionState.AUTO_MISSION:
            print("[EVENT] Target detected → stopping mission")
            set_mode("GUIDED")
            state = MissionState.DETECTED

        elif cmd == "confirm_land" and state == MissionState.DETECTED:
            print("[HITL] Landing approved")
            land()
            state = MissionState.LANDING

        elif cmd == "release1" and state == MissionState.LANDING:   #this is fake
            print("[PAYLOAD] Hook 1 released")

        elif cmd == "release2" and state == MissionState.LANDING:   #and this
            print("[PAYLOAD] Hook 2 released")
            state = MissionState.PAYLOAD

        elif cmd == "confirm_takeoff" and state == MissionState.PAYLOAD:    #this doesn't work for some reason
            print("[HITL] Takeoff approved")
            takeoff(5)
            state = MissionState.TAKEOFF

        elif cmd == "resume" and state == MissionState.TAKEOFF:
            print("[MISSION] Resuming / RTH")
            set_mode("RTL")
            state = MissionState.RESUME
