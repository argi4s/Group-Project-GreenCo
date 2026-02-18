from pymavlink import mavutil
import time

# ----------------------
# Connection
# ----------------------
mav = mavutil.mavlink_connection("tcp:127.0.0.1:14550")
mav.wait_heartbeat()
print("[OK] Connected to FC")

# ----------------------
# ACK utility
# ----------------------
def wait_command_ack(command, timeout=3):
    start = time.time()
    while time.time() - start < timeout:
        msg = mav.recv_match(type="COMMAND_ACK", blocking=False)
        if msg and msg.command == command:
            return msg.result
        time.sleep(0.05)
    return None

# ----------------------
# Commands
# ----------------------
def arm():
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    ack = wait_command_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
    print("[ARM]", ack)

def set_mode_ack(mode):
    mav.set_mode_apm(mode)
    ack = wait_command_ack(mavutil.mavlink.MAV_CMD_DO_SET_MODE)
    if ack == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print(f"[MODE] {mode}")
        return True
    print("[MODE FAIL]", ack)
    return False

def goto_global(lat, lon, alt):
    mav.mav.set_position_target_global_int_send(
        0,
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,
        int(lat * 1e7),
        int(lon * 1e7),
        alt,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )
    print("[GUIDED] Going to target")

def rtl():
    set_mode_ack("RTL")

# ----------------------
# FSM loop
# ----------------------
print("""
Commands:
 arm
 guided <lat> <lon> <alt>
 rtl
 exit
""")

while True:
    cmd = input("> ").strip().lower()

    if cmd == "exit":
        break

    elif cmd == "arm":
        arm()

    elif cmd.startswith("guided"):
        _, lat, lon, alt = cmd.split()
        if set_mode_ack("GUIDED"):
            goto_global(float(lat), float(lon), float(alt))

    elif cmd == "rtl":
        rtl()

    else:
        print("Unknown command")
