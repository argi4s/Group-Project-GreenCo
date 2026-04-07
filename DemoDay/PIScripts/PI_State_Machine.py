#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from DroneClass import Drone
from queue import Queue
import threading
import time
import socket
import os
import atexit
from vision_receiver import VisionReceiver



from lawnmower import plan_lawnmower, CameraFootprintRef, Waypoint


# -------------------------
# INITIALISE CAMERA
# -------------------------
# Check if we should run in development mode
DEV_MODE = os.environ.get('DRONE_DEV_MODE', 'false').lower() == 'true'

if DEV_MODE:
    print("[DEV MODE] Running in development mode - camera disabled")
    # Create a dummy camera object that does nothing
    class DummyCamera:
        def start(self): print("[DEV MODE] Dummy camera start")
        def stop(self): print("[DEV MODE] Dummy camera stop")
    camera = DummyCamera()
    #from camera_stream_file import CameraStreamFile
    #camera = CameraStreamFile(
    #    filename="/home/matlok/Desktop/FlyingDay4/GCSScripts/drone_recording_20260330_142116.avi",
    #    vec_port=5801
    #)
else:
    from camera_stream import CameraStream
    camera = CameraStream(
        video_ip="pop-os.local",
        video_port=5800,
        vec_port=5801
    )

# -------------------------
# STATES
# -------------------------
STATE_INIT = "INIT"
STATE_TAKEOFF = "TAKEOFF"
STATE_FLIGHT = "FLIGHT"
STATE_LAND = "LAND"
STATE_EMERGENCY = "EMERGENCY"
STATE_AOI = "AOI"
STATE_PLB = "PLB"
STATE_RETURN = "RETURN"
STATE_LAWNMOWER = "LAWN"
STATE_RTL = "RTL"
STATE_UPL = "UPL"
STATE_ALIGN = "ALIGN"
STATE_DESCEND = "DESCEND"



current_state = STATE_INIT

# -------------------------
# HARDCODED PATHS
# ------------------------

RETURN_WPS = [
    (-2.667548,  51.423516),
    (-2.6669526, 51.4237044),
    (-2.6662177, 51.4227243),
    (-2.6699513, 51.4217375),
    (-2.6716378, 51.4233848)
]
# Not following ISO 6709 the typical convention as this confuses us as developers and users of mavlink. We can expose at the front end conventional lon, lat order as expected but will not do so internally - Kory


AOI_WPS = RETURN_WPS[::-1]

SEARCH_AREA = [
    (-2.670948345438704, 51.42326956502679),
    (-2.670045428650557, 51.42287025017865),
    (-2.668169295906676, 51.42336622593724),
    (-2.668809768621569, 51.42421477437771),
    (-2.671277780473196, 51.42354069739116),
    (-2.670948345438704, 51.42326956502679)
]

PLB_WPS = []

TEST_SQUARE = [
    (-2.6713921,51.4234248, ),
    ( -2.6712371,51.4232738,),
    ( -2.6710807,51.4233227),
    ( -2.6712107,51.4234691)
]

LAWN_WPS = []


lawn_resume_index = 0
lawn_completed = False
lawn_debug_index = 0


# -------------------------
# AUTO-DETECT PI'S IP
# -------------------------
def get_local_ip():
    """Get the local IP address of this machine"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"

MAVPROXY_IP = "127.0.0.1"
print(f"[INFO] Pi IP address: {MAVPROXY_IP}")
MAVPROXY_PORT = 14401

# Global heading setting
FIXED_HEADING = -1  # -1 = auto rotate

# -------------------------
# DRONE INITIALISATION
# -------------------------
def on_drone_event(event):
    global current_state

    if event == "RC_OVERRIDE":
        print("[SAFETY] LOITER detected — RC override active")

        # Kill all missions
        current_state = STATE_INIT

        # Stop GCS from sending
        send_status("RC override — automation paused", type_="FSM")

print("[INFO] Initialising drone connection...")
drone = Drone(f"udp:127.0.0.1:{MAVPROXY_PORT}")
print("[INFO] Drone connected via MAVLink")
drone.fsm_callback = on_drone_event
# -------------------------
# STATE RECOVERY
# -------------------------
# If the drone is already flying when the FSM starts,
# recover the correct state automatically.
_, _, alt = drone.position

if drone.armed and alt is not None and alt > 1.0:
    print("[INFO] Drone already airborne — recovering FLIGHT state")
    current_state = STATE_FLIGHT
else:
    current_state = STATE_INIT

# -------------------------
# START CAMERA
# -------------------------
camera.start()  # <-- START CAMERA HERE
from vision_receiver import VisionReceiver
vision = VisionReceiver(host="127.0.0.1", port=5801)
vision.start()


# -------------------------
# UDP COMMUNICATION
# -------------------------
PI_UDP_PORT = 9050
global GCS_ADDR
GCS_ADDR = ("pop-os.local", 9051)

udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_sock.bind(("", PI_UDP_PORT))
udp_sock.settimeout(0.2)

# -------------------------
# EVENT QUEUE
# -------------------------
event_queue = Queue()

def udp_listener():
    while True:
        try:
            data, _ = udp_sock.recvfrom(1024)
            text = data.decode().strip()
            parts = text.split()
            event = parts[0]
            param = parts[1] if len(parts) > 1 else None
            event_queue.put((event, param))
        except socket.timeout:
            continue

threading.Thread(target=udp_listener, daemon=True).start()

def vision_monitor_loop():
    global current_state
    while True:
        try:
            if current_state == STATE_FLIGHT:
                if vision.mostly_visible_for(5.0, 0.9):
                    print("Target visible ≥90% for 5s → ALIGN")
                    drone.set_mode("GUIDED")
                    current_state = STATE_ALIGN
                    time.sleep(0.2)
            time.sleep(0.05)
        except Exception as e:
            print(f"[vision monitor error] {e}")
            time.sleep(0.1)

threading.Thread(target=vision_monitor_loop, daemon=True).start()

lawn_active = False
# -------------------------
# STATUS SENDER
# -------------------------
seq_counter = 0
ACK_TIMEOUT = 0.5
MAX_RETRIES = 5

gcs_ack_lost = False
gcs_connected = False
last_ack_time = 0
GCS_TIMEOUT = 2

def send_status(msg, type_="STATUS"):
    global seq_counter, gcs_ack_lost, gcs_connected, last_ack_time

    seq_counter += 1
    seq = seq_counter
    payload = f"SEQ={seq} TYPE={type_} MSG={msg}"

    ack_received = False
    for _ in range(MAX_RETRIES):
        udp_sock.sendto(payload.encode(), GCS_ADDR)
        try:
            udp_sock.settimeout(ACK_TIMEOUT)
            data, _ = udp_sock.recvfrom(1024)
            if data.decode().strip() == f"ACK SEQ={seq} TYPE={type_}":
                ack_received = True
                last_ack_time = time.time()
                gcs_connected = True
                break
        except socket.timeout:
            continue

    if ack_received:
        gcs_ack_lost = False
    else:
        gcs_ack_lost = True
        if time.time() - last_ack_time > GCS_TIMEOUT:
            gcs_connected = False



def start_auto_mission(mission_wps, mission_name="MISSION"):
    global current_state

    # Use current altitude already baked into mission_wps
    if not mission_wps:
        send_status(f"{mission_name}: no waypoints", type_="FSM")
        return

    # Reset interrupt flags at mission start
    drone.emergency_flag = False
    drone.rtl_flag = False

    # Switch to GUIDED
    drone.set_mode("GUIDED")
    time.sleep(1)
    send_status(f"{mission_name}: mode GUIDED", type_="FSM")

    # Upload mission
    send_status(f"{mission_name}: uploading {len(mission_wps)} waypoints", type_="FSM")
    drone.upload_mission(mission_wps)
    send_status(f"{mission_name}: upload complete", type_="FSM")

    # Switch to AUTO
    drone.set_mode("AUTO")
    send_status(f"{mission_name}: mode AUTO", type_="FSM")

    current_state = STATE_FLIGHT

def update_lawn_progress(drone, waypoints):
    global lawn_resume_index, lawn_completed

    # Find nearest waypoint index
    lon, lat, _ = drone.position
    if lon is None or lat is None:
        return

    # Compute nearest waypoint
    dists = [(i, (wp[0]-lon)**2 + (wp[1]-lat)**2) for i, wp in enumerate(waypoints)]
    nearest_i = min(dists, key=lambda x: x[1])[0]

    lawn_resume_index = nearest_i

    # If we reached the last waypoint
    if nearest_i >= len(waypoints) - 1:
        lawn_completed = True


# -------------------------
# CLEANUP HANDLER
# -------------------------


def cleanup():
    """Stop camera on exit"""
    print("\n[INFO] Cleaning up...")
    camera.stop()

atexit.register(cleanup)

# -------------------------
# FSM EVENT HANDLER
# -------------------------

def handle_event(event, param):
    global current_state, FIXED_HEADING
    global lawn_resume_index, lawn_completed, lawn_active

    send_status(f"State={current_state} Event={event}", type_="FSM")

    if event == "override_clear":
        lawn_active = False
        drone.emergency_flag = False
        drone.rtl_flag = False

        send_status("Override cleared — operator resumed control", type_="FSM")

        # If drone is airborne → return to FLIGHT
        _, _, alt = drone.position
        if drone.armed and alt is not None and alt > 2.1:
            current_state = STATE_FLIGHT
        else:
            current_state = STATE_INIT

        return


    if event == "TAKEOFF_STARTED":
        lawn_active = False
        send_status("Takeoff started", "FSM")
        return

    if event == "TAKEOFF_COMPLETE":
        lawn_active = False
        current_state = STATE_FLIGHT
        send_status("Takeoff complete — entering FLIGHT", "FSM")
        return

    if event == "LANDING":
        lawn_active = False
        send_status("Landing sequence detected", "FSM")
        current_state = STATE_LAND
        return

    if event == "LANDED":
        lawn_active = False
        send_status("Landing complete — drone on ground", "FSM")
        current_state = STATE_INIT
        return


    # -------------------------
    # UPL (AOI upload-only)
    # -------------------------
    if event == "upl":

        if not AOI_WPS:
            send_status("No AOI waypoints available", type_="FSM")
            return

        _, _, current_alt = drone.position
        if current_alt is None or current_alt < 5:
            current_alt = 20

        mission_wps = [(lon, lat, current_alt) for lon, lat in AOI_WPS]

        send_status(f"Uploading {len(mission_wps)} AOI waypoints at {current_alt:.1f}m", type_="FSM")

        try:
            drone.upload_mission(mission_wps)
            send_status("AOI mission upload complete", type_="FSM")
        except Exception as e:
            send_status(f"Mission upload failed: {e}", type_="FSM")

        return


    # -------------------------
    # EMERGENCY
    # -------------------------
    if event == "emergency":
        lawn_active = False
        drone.emergency()
        #current_state = STATE_EMERGENCY
        send_status("EMERGENCY - Drone killed", type_="EMERGENCY")
        return

    # -------------------------
    # RTL
    # -------------------------
    if event == "rtl":
        lawn_active = False
        current_state = STATE_INIT
        send_status("RTL triggered", type_="RTL")
        drone.rtl()
        return

    # -------------------------
    # TAKEOFF
    # -------------------------
    if event == "takeoff":
        lawn_active = False
        if current_state == STATE_EMERGENCY:
            send_status("In EMERGENCY - cannot take off", type_="FSM")
            return

        if not drone.armed:
            drone.arm()
            send_status("Drone armed", type_="FSM")
            time.sleep(1)

        takeoff_alt = float(param) if param else 30

        try:
            drone.takeoff(takeoff_alt)
            current_state = STATE_FLIGHT
            # If drone is already airborne, skip takeoff
            if drone.armed and drone.position[2] is not None and drone.position[2] > 1.0:
                send_status("Skipping takeoff — drone already airborne", "FSM")
                current_state = STATE_FLIGHT
                return
            send_status(f"Takeoff to {takeoff_alt}m — hovering", type_="FSM")
        except RuntimeError as e:
            if "RTL active" in str(e) or "Emergency active" in str(e):
                print(f"Takeoff interrupted: {e}")
            else:
                raise
        return


    if event == "TAKEOFF_COMPLETE":
        lawn_active = False
        current_state = STATE_FLIGHT
        send_status("Takeoff complete — entering FLIGHT", "FSM")


    if event == "alt":
        lawn_active = False
        current_state = STATE_FLIGHT
        new_alt = float(param) if param else 30

        try:
            drone.auto_set_altitude(new_alt)

            send_status(f"Changing altitude to {new_alt}m", type_="FSM")
            return

        except RuntimeError as e:
            send_status("Problem with altitude change")
        return

    # -------------------------
    # SPEED
    # -------------------------
    if event == "speed":

        if param:
            try:
                drone.set_speed(float(param))
                send_status(f"Speed set to {param} m/s", type_="FSM")
            except:
                send_status(f"Invalid speed: {param}", type_="FSM")
        return

    # -------------------------
    # HEADING
    # -------------------------
    if event == "heading":
        if param:
            try:
                FIXED_HEADING = float(param)
                if FIXED_HEADING == -1:
                    send_status("Heading set to AUTO ROTATE", type_="FSM")
                else:
                    send_status(f"Fixed heading set to {FIXED_HEADING}°", type_="FSM")
            except:
                send_status(f"Invalid heading: {param}", type_="FSM")
        return

    # -------------------------
    # BLOCK ALL OTHER COMMANDS IN EMERGENCY
    # -------------------------
    if current_state == STATE_EMERGENCY:
        lawn_active = False
        send_status("In EMERGENCY state - command ignored", type_="FSM")
        return

    if event == "fence_upload":
        if not param:
            send_status("No geofence provided", "FSM")
            return

        try:
            coords = [float(x) for x in param.split(',')]
            if len(coords) % 2 != 0:
                raise ValueError("Odd number of coordinates")

            fence = [(coords[i], coords[i+1]) for i in range(0, len(coords), 2)]

            drone.upload_geofence(fence)
            send_status(f"Geofence uploaded with {len(fence)} points", "FSM")

        except Exception as e:
            send_status(f"Geofence upload failed: {e}", "FSM")

        return


    # -------------------------
    # PLB UPLOAD (just storing)
    # -------------------------
    if event == "plb_upload":
        if not param:
            send_status("No PLB waypoints provided", type_="FSM")
            return

        try:
            coords = [float(x) for x in param.split(',')]
            if len(coords) % 2 != 0:
                raise ValueError("Odd number of coordinates")

            _, _, current_alt = drone.position
            if current_alt is None or current_alt < 5:
                current_alt = 20

            global PLB_WPS
            PLB_WPS = [(coords[i], coords[i+1], current_alt)
                       for i in range(0, len(coords), 2)]

            send_status(f"PLB mission loaded with {len(PLB_WPS)} waypoints", type_="FSM")

        except Exception as e:
            send_status(f"Failed to parse PLB waypoints: {e}", type_="FSM")
        return


    if event == "preview_request":
        # Send lawnmower preview (regenerate at current altitude)
        origin_lon, origin_lat = drone.position[:2]
        altitude_m = drone.position[2] if drone.position[2] else 20

        lawn = plan_lawnmower(
            SEARCH_AREA,
            altitude_m,
            origin_lonlat=(origin_lon, origin_lat)
        )
        lawn_pts = ";".join([f"{wp.lon},{wp.lat}" for wp in lawn])
        send_status(f"LAWN {lawn_pts}", type_="PREVIEW")

        # AOI
        aoi_pts = ";".join([f"{lon},{lat}" for lon, lat in AOI_WPS])
        send_status(f"AOI {aoi_pts}", type_="PREVIEW")

        # RETURN
        ret_pts = ";".join([f"{lon},{lat}" for lon, lat in RETURN_WPS])
        send_status(f"RETURN {ret_pts}", type_="PREVIEW")

        # PLB
        plb_pts = ";".join([f"{lon},{lat}" for lon, lat, _ in PLB_WPS])
        send_status(f"PLB {plb_pts}", type_="PREVIEW")

        # Geofence (if you store it)
        try:
            fence_pts = ";".join([f"{lon},{lat}" for lon, lat in GEOFENCE_WPS])
            send_status(f"GEOFENCE {fence_pts}", type_="PREVIEW")
        except:
            pass

        return


    # -------------------------
    # LAND
    # -------------------------
    if event == "land":

        lawn_active = False
        send_status("Landing initiated", type_="LAND")
        try:
            drone.land()
        except RuntimeError as e:
            send_status(f"Landing interrupted: {e}", "LAND")

        # Immediately return to INIT — do NOT wait for landed()
        current_state = STATE_INIT
        send_status("Returned to INIT", type_="FSM")
        return


    # -------------------------
    # INIT (RESET)
    # -------------------------
    if current_state in (STATE_INIT, STATE_RTL):
        if event == "init":
            lawn_active = False
            drone.emergency_flag = False
            drone.rtl_flag = False
            lawn_resume_index = 0
            lawn_completed = False


            drone.set_mode("GUIDED")
            time.sleep(1)
            send_status("Mode set to GUIDED", type_="FSM")
            # Automatic recovery if FSM restarted mid-flight
            if current_state == STATE_INIT:
                _, _, alt = drone.position
                if drone.armed and alt is not None and alt > 1.0:
                    current_state = STATE_FLIGHT
        return

    # ============================================================
    #                     FLIGHT STATE MISSIONS
    # ============================================================
    if current_state == STATE_FLIGHT:

        # -------------------------
        # RESTART
        # -------------------------
        if event == "restart":
            lawn_active = False
            current_state = STATE_INIT
            return


        elif event == "test_square":
            lawn_active = False

            print("Uploading TEST SQUARE mission")

            # Use current altitude or default to 20m
            alt = drone.position[2] if drone.position else 20

            mission = [(lon, lat, alt) for (lon, lat) in TEST_SQUARE]

            start_auto_mission(mission, "TEST_SQUARE")

        # -------------------------
        # LAWNMOWER
        # -------------------------
        elif event == "lawn":
            lawn_active = True
            lawn_completed = False
            global LAWN_WPS



            # Always generate lawn pattern anchored to SEARCH_AREA, not current position
            altitude_m = drone.position[2] if drone.position[2] else 20

            waypoints = plan_lawnmower(
                search_polygon=SEARCH_AREA,
                altitude_m=altitude_m,
                origin_lonlat=SEARCH_AREA[0],
                camera_ref=CameraFootprintRef(
                    ref_alt_m=50,
                    ref_width_m=52.4,
                    ref_height_m=39.3
                ),
                side_overlap=0.7,
                forward_overlap=0.7
            )



            # If finished → restart from beginning
            if lawn_completed:
                start_index = 0
            else:
                start_index = lawn_resume_index

            mission_wps = [(wp.lon, wp.lat, altitude_m) for wp in waypoints]
            LAWN_WPS = [(wp.lon, wp.lat, altitude_m) for wp in waypoints]

            current_state = STATE_LAWNMOWER
            send_status(
                f"Lawnmower mission with {len(mission_wps)} waypoints (resume index {start_index})",
                type_="FSM"
            )

            try:
                start_auto_mission(mission_wps, mission_name="LAWN")
                drone.set_mission_index(start_index)
            except Exception as e:
                send_status(f"Lawnmower mission failed: {e}", type_="FSM")
                current_state = STATE_FLIGHT



        # -------------------------
        # AOI
        # -------------------------
        elif event == "aoi":
            lawn_active = False
            _, _, current_alt = drone.position
            if current_alt is None or current_alt < 5:
                current_alt = 20

            mission_wps = [(lon, lat, current_alt) for lon, lat in AOI_WPS]

            current_state = STATE_AOI
            send_status(f"AOI mission started", type_="FSM")

            try:
                start_auto_mission(mission_wps, mission_name="AOI")
            except Exception as e:
                send_status(f"AOI mission failed: {e}", type_="FSM")
                current_state = STATE_FLIGHT
            return

        # -------------------------
        # PLB START
        # -----------------------
        elif event == "plb_start":
            lawn_active = False
            if not PLB_WPS:
                send_status("No PLB waypoints loaded", type_="FSM")
                return

            altitude_m = drone.position[2] if drone.position[2] else 20

            # FIX: strip altitude → 2D polygon
            plb_poly = [(lon, lat) for (lon, lat, _) in PLB_WPS]

            # FIX: origin must also be 2D
            origin = plb_poly[0]

            waypoints = plan_lawnmower(
                search_polygon=plb_poly,
                altitude_m=altitude_m,
                origin_lonlat=origin,
                camera_ref=CameraFootprintRef(
                    ref_alt_m=50,
                    ref_width_m=52.4,
                    ref_height_m=39.3
                ),
                side_overlap=0.7,
                forward_overlap=0.7
            )

            mission_wps = [(wp.lon, wp.lat, altitude_m) for wp in waypoints]

            current_state = STATE_PLB
            send_status(f"PLB mission started", type_="FSM")

            try:
                start_auto_mission(mission_wps, mission_name="PLB")
            except Exception as e:
                send_status(f"PLB mission failed: {e}", type_="FSM")
                current_state = STATE_FLIGHT


        # -------------------------
        # RETURN
        # -------------------------
        elif event == "return":
            lawn_active = False
            _, _, current_alt = drone.position
            if current_alt is None or current_alt < 5:
                current_alt = 20

            mission_wps = [(lon, lat, current_alt) for lon, lat in RETURN_WPS]

            current_state = STATE_RETURN
            send_status(f"RETURN mission started", type_="FSM")

            try:
                start_auto_mission(mission_wps, mission_name="RETURN")
            except Exception as e:
                send_status(f"RETURN mission failed: {e}", type_="FSM")
                current_state = STATE_FLIGHT
            return

    # ALIGN state
    elif current_state == STATE_ALIGN:
        lawn_active = False
        latest = vision.get_latest()
        height_m = drone.position[2]

        # If target lost → exit ALIGN
        if latest is None or latest.get("det", 0) == 0:
            drone.stop_in_place()
            send_status("ALIGN: target lost → FLIGHT", "FSM")
            current_state = STATE_FLIGHT
            return

        # Compute correction
        cmd = correction_high.compute_align_command(latest, height_m)

        # If valid but not centered → send velocity
        if cmd["valid"] and not cmd["aligned"]:
            drone.send_body_velocity(cmd["vx"], cmd["vy"], cmd["vz"])
            return

        # If centered → stop and descend
        if cmd["aligned"]:
            drone.stop_in_place()
            send_status("ALIGN: centered → DESCEND", "FSM")
            current_state = STATE_DESCEND
            return

    # DESCEND state
    elif current_state == STATE_DESCEND:
        lawn_active = False
        try:
            drone.descend_to_altitude(5.0, descent_rate=0.2)
            send_status("DESCEND complete → FLIGHT", "FSM")
            current_state = STATE_FLIGHT
        except RuntimeError as e:
            send_status(f"DESCEND interrupted: {e}", "FSM")
            current_state = STATE_FLIGHT
        return



    # -------------------------
    # DEPLOY (servo)
    # -------------------------
    if event == "deploy":
        set_servo_pwm(14, 2050)
        return

    # -------------------------
    # RTL STATE MONITOR
    # -------------------------
#    if current_state == STATE_RTL:
#       if not drone.landed():
#            time.sleep(0.5)
#        else:
#            current_state = STATE_INIT


# -------------------------
# HEARTBEAT (Disabled)
# -------------------------
def heartbeat_loop():
    while True:
        time.sleep(1)

# -------------------------
# TELEMETRY DISPLAY
# -------------------------
def telemetry_display():
    global current_state, gcs_ack_lost, lawn_debug_index
    while True:
        lon, lat, alt = drone.position
        mode = drone.mode
        armed = drone.armed
        gps = drone.gps_fix
        heading = drone.heading
        ground_speed = drone.ground_speed
        air_speed = drone.air_speed
        vz = drone.vertical_speed
        conn_status = "CONNECTED" if gcs_connected else "DISCONNECTED"

        alt_str = f"{alt:.2f} m" if alt is not None else "--"
        lon_str = f"{lon:.7f}" if lon is not None else "--"
        lat_str = f"{lat:.7f}" if lat is not None else "--"
        gps_str = str(gps) if gps is not None else "--"
        mode_str = mode if mode else "--"
        heading_str = f"{heading:.1f}°" if heading is not None else "--"
        gnd_spd_str = f"{ground_speed:.1f} m/s" if ground_speed is not None else "--"
        air_spd_str = f"{air_speed:.1f} m/s" if air_speed is not None else "--"
        vz_str = f"{vz:.1f} m/s" if vz is not None else "--"

        os.system("clear")
        print("====================================")
        print("        DRONE TELEMETRY PANEL       ")
        print("====================================")
        print(f"STATE : {current_state}")
        print(f"MODE  : {mode_str}")
        print(f"ARMED : {armed}")
        print(f"ALT   : {alt_str}")
        print(f"LON   : {lon_str}")
        print(f"LAT   : {lat_str}")
        print(f"GPS   : {gps_str}")
        print(f"LAWN IDX : {drone.mission_index}")
        print(f"DEBUG IDX : {lawn_debug_index}")
        print("------------------------------------")
        print(f"HEADING : {heading_str}")
        print(f"GND SPD : {gnd_spd_str}")
        print(f"AIR SPD : {air_spd_str}")
        print(f"VZ SPD  : {vz_str}")
        print("------------------------------------")
        print(f"GCS   : {conn_status}")
        print(f"FIXED HDG : {FIXED_HEADING}° ({'AUTO' if FIXED_HEADING==-1 else 'FIXED'})")
        if gcs_ack_lost:
            print("\n[WARNING] ACK not received — check GCS connection!")
        print("====================================")
        time.sleep(0.5)

# -------------------------
# THREADS
# -------------------------

def lawn_progress_loop():
    global lawn_resume_index, lawn_completed, lawn_active, lawn_debug_index

    last_seen = -1

    while True:
        if lawn_active:
            current = drone.mission_index
            lawn_debug_index = current   # <-- LIVE VALUE

            if current != last_seen:
                completed = max(0, current - 1)
                lawn_resume_index = completed
                last_seen = current

                if LAWN_WPS and completed >= len(LAWN_WPS) - 1:
                    lawn_completed = True
                    lawn_resume_index = 0
                    lawn_active = False

        time.sleep(0.2)






def keepalive_loop():
    while True:
        send_status("ping", type_="KEEPALIVE")
        time.sleep(2.5)

threading.Thread(target=keepalive_loop, daemon=True).start()
threading.Thread(target=telemetry_display, daemon=True).start()
threading.Thread(target=lawn_progress_loop, daemon=True).start()


# Auto-discovery thread
def discovery_listener():
    disc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    disc_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    disc_sock.bind(('', 9052))
    disc_sock.settimeout(1)

    local_ip = MAVPROXY_IP
    fsm_port = PI_UDP_PORT

    global GCS_ADDR
    while True:
        try:
            data, addr = disc_sock.recvfrom(1024)
            if data.decode() == "WHO_IS_PI?":
                response = f"I_AM_PI:{local_ip}:{fsm_port}"
                disc_sock.sendto(response.encode(), addr)
                GCS_ADDR = (addr[0], 9051)
                print(f"\n[!] Auto-discovered GCS at {addr[0]}:9051")
        except socket.timeout:
            continue
        except Exception as e:
            print(f"Discovery error: {e}")

threading.Thread(target=discovery_listener, daemon=True).start()

# -------------------------
# MAIN LOOP
# -------------------------

_, _, alt = drone.position

if drone.armed and alt is not None and alt > 1.0:
    print("[INFO] Drone already airborne — recovering FLIGHT state")
    current_state = STATE_FLIGHT
else:
    current_state = STATE_INIT

print("[INFO] FSM running. Waiting for GCS commands.")
print("[INFO] Emergency and RTL will interrupt any active mission")



while True:
    try:
        event, param = event_queue.get(timeout=0.5)
        handle_event(event, param)
    except:
        pass

    # Monitor RTL progress
#    if current_state == STATE_RTL and drone.landed():
#        print(" RTL complete - drone landed")
#
#        # CRITICAL: Set mode back to GUIDED for future commands
#        drone.set_mode("GUIDED")
#        time.sleep(1)  # Give it time to switch
#
#        send_status("RTL complete - landed", type_="LAND")
#        current_state = STATE_INIT
#        send_status("Returned to INIT - Mode set to GUIDED", type_="FSM")

    # Monitor LAND progress
    #if current_state == STATE_LAND:
    #    lon, lat, alt = drone.position
    #    vz = drone.vertical_speed
    #    armed = drone.armed
    #    if drone.landed():
    #        landed = True#

    #    elif alt is not None and alt < 2.0 and abz(vz) < 0.2 and not armed:
    #        print("Landing fallback triggered (alt < 2m, vz approx. 0, disarmed)")
    #        landed = True
    #    else:
    #        landed = False

    #    if landed:
    #        print(" Landing complete (FSM exit)")
    #        drone.set_mode("GUIDED")
    #        time.sleep(1)
    #        send_status("Landing complete", type_="LAND")
    #        current_state = STATE_INIT
    #        send_status("Returned to INIT - Mode set to GUIDED", type_="FSM")

        # ⚠️ CRITICAL: Set mode back to GUIDED for future commands
    #    drone.set_mode("GUIDED")
    #    time.sleep(1)

    #    send_status("Landing complete", type_="LAND")
    #    current_state = STATE_INIT
    #    send_status("Returned to INIT - Mode set to GUIDED", type_="FSM")

    time.sleep(0.05)
