from pymavlink import mavutil
import time

mav = mavutil.mavlink_connection("tcp:127.0.0.1:14550") # check comunication protocol and port
mav.wait_heartbeat()

print("Connected to FC")    # check

def set_mode(mode):
    mav.set_mode_apm(mode)
    print(f"[CMD] Mode set to {mode}")

def arm():          # arming function, use arm in cmd to call it
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    print("[CMD] Arm")

def send_velocity(vx, vy, vz):  # velocity function, type x,y,z coords for the drone to follow
    mav.mav.set_position_target_local_ned_send(
        0,
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111, #ignore everything except Velocity (Mask)
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )
    print(f"[CMD] Velocity {vx} {vy} {vz}")

while True:
    cmd = input("> ").strip().lower()

    if cmd == "exit":
        break

    elif cmd.startswith("mode"):
        _, mode = cmd.split()
        set_mode(mode.upper())

    elif cmd == "arm":
        arm()

    elif cmd.startswith("vel"):
        _, vx, vy, vz = cmd.split()
        send_velocity(float(vx), float(vy), float(vz))

    elif cmd == "stop":
        send_velocity(0, 0, 0)

    elif cmd == "status":
        hb = mav.recv_match(type="HEARTBEAT", blocking=True)
        print("Mode:", mavutil.mode_string_v10(hb))

    else:
        print("Unknown command")
