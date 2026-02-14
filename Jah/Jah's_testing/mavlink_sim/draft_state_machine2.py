from pymavlink import mavutil
import time
from enum import Enum

# ------------------------
# FSM States
# ------------------------
class State(Enum):
    INIT = 0
    IDLE = 1
    ARMED = 2
    TAKEOFF = 3
    MISSION = 4
    LOITER = 5
    RTL = 6
    FAILSAFE = 7

# ------------------------
# MAVLink connection
# ------------------------
master = mavutil.mavlink_connection("tcp:127.0.0.1:14550")
master.wait_heartbeat()
print("Connected to vehicle")

# ------------------------
# Helpers
# ------------------------
def set_mode(mode):
    mapping = master.mode_mapping()
    if mode not in mapping:
        print(f"Mode {mode} not supported")
        return False

    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mapping[mode]
    )
    print(f"Mode → {mode}")
    return True


def is_armed():
    return master.motors_armed()


def get_altitude():
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    if msg:
        return msg.relative_alt / 1000.0
    return None


def read_operator_command():
    msg = master.recv_match(type='STATUSTEXT', blocking=False)
    if msg:
        return msg.text.decode('utf-8').strip().upper()
    return None

# ------------------------
# FSM variables
# ------------------------
state = State.INIT
last_heartbeat = time.time()

# ------------------------
# FSM Loop
# ------------------------
while True:

    # --- Heartbeat monitoring ---
    hb = master.recv_match(type='HEARTBEAT', blocking=False)
    if hb:
        last_heartbeat = time.time()

    if time.time() - last_heartbeat > 3:
        state = State.FAILSAFE

    alt = get_altitude()
    cmd = read_operator_command()

    # ------------------------
    # Operator Commands (global)
    # ------------------------
    if cmd:
        print(f"Operator → {cmd}")

        if cmd == "HOLD":
            set_mode("LOITER")
            state = State.LOITER

        elif cmd == "RESUME" and state == State.LOITER:
            set_mode("AUTO")
            state = State.MISSION

        elif cmd == "RTL":
            state = State.RTL

        elif cmd == "ABORT":
            state = State.FAILSAFE

    # ------------------------
    # FSM Logic
    # ------------------------
    if state == State.INIT:
        print("FSM → IDLE")
        state = State.IDLE

    elif state == State.IDLE:
        if is_armed():
            print("FSM → ARMED")
            state = State.ARMED

    elif state == State.ARMED:
        if alt and alt > 1.0:
            print("FSM → TAKEOFF")
            state = State.TAKEOFF

    elif state == State.TAKEOFF:
        if alt and alt > 10.0:
            print("FSM → MISSION")
            set_mode("AUTO")
            state = State.MISSION

    elif state == State.MISSION:
        # Normal autonomous flight
        pass

    elif state == State.LOITER:
        # Hold position, wait for RESUME / RTL
        pass

    elif state == State.RTL:
        print("FSM → RTL")
        set_mode("RTL")
        break

    elif state == State.FAILSAFE:
        print("FAILSAFE → RTL")
        set_mode("RTL")
        break

    time.sleep(0.2)
