from pymavlink import mavutil
import time

mav = mavutil.mavlink_connection("tcp:127.0.0.1:14550") # check comunication protocol and port
mav.wait_heartbeat()

print("Connected to FC")    # check

def set_mode(mode):
    mav.set_mode_apm(mode)
    print(f"[CMD] Mode set to {mode}")

def arm():          # arming function, use "arm" in cmd to call it
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    print("[CMD] Arm")

def disarm():       # disarming function, use "disarm"
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    print("[CMD] Disarm")


def takeOff(height):  # take off at p7 meters
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,    # p1=pitch, p2-p3=empty, p4=yaw, p5=lat, p6=lon, p7=alt
        0,
        0, 0, 0, 0,
        0, 0,
        height  #p7
    )
    print("[CMD] Taking Off")


def start_mission():
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0,
        0,      # start index
        0,      # end index (0 = full mission)
        0, 0, 0, 0, 0
    )
    print("[CMD] Start Mission")

def send_velocity(vx, vy, vz):  # velocity function, type x,y,z coords for the drone to follow
    mav.mav.set_position_target_local_ned_send(
        0,
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111, #ignore position, acceleration, yaw focus on Velocity (Mask)
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )
    print(f"[CMD] Velocity {vx} {vy} {vz}")

def changeSpeed():
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0,      #number of attempts
        1,      #groundspeed
        2.0,    #speed
        -1,
        0,0,0,0,
    )
    print("[CMD] Speed Change")

def wait_ack(command, timeout=3):
    start = time.time()
    while time.time() - start < timeout:
        msg = mav.recv_match(type="COMMAND_ACK", blocking=False)
        if msg and msg.command == command:
            return msg.result
        time.sleep(0.05)
    return None

while True:
    cmd = input("> ").strip().lower()

    if cmd == "exit":
        break

    elif cmd.startswith("mode"):
        _, mode = cmd.split()
        set_mode(mode.upper())

    elif cmd == "arm":
        arm()
        ack = wait_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)

        if ack == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("[ACK] Arm accepted")
        else:
            print("[ACK] Arm failed:", ack)
    
    elif cmd == "disarm":
        disarm()

    elif cmd.startswith("takeoff"):
        _, height = cmd.split()
        takeOff(float(height))

    elif cmd == "changespeed":
        changeSpeed()

    elif cmd == "startmission":
        start_mission()

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
