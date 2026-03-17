#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from DroneClass import Drone
from queue import Queue
import threading
import time
import socket
import os

from lawnmower import plan_lawnmower, CameraFootprintRef, Waypoint

from camera_stream import CameraStream  # <-- NEW IMPORT
from vision_receiver import VisionReceiver


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
else:
    from camera_stream import CameraStream
    camera = CameraStream(
        video_ip="192.168.1.17",
        video_port=5800,
        vec_port=5801
    )
=======
camera = CameraStream(
    video_ip="192.168.1.17",  # Your PC IP
    video_port=5800,
    vec_remote_ip="192.168.1.17",
    vec_local_ip="127.0.0.1",
    vec_port=5801
)
>>>>>>> cd62866 (Update (aiming after detected)- framework and interfaces)

# Hearing the vector to local
vision = VisionReceiver(host="127.0.0.1", port=5801)

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

STATE_ALIGN = "ALIGN"

current_state = STATE_INIT
align_entered = False
target_acquire_position = None

# -------------------------
# HARDCODED PATHS
# ------------------------

RETURN_WPS = [
    (-2.6716371, 51.4233850),
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

PLB = []


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

MAVPROXY_IP = get_local_ip()
print(f"[INFO] Pi IP address: {MAVPROXY_IP}")
MAVPROXY_PORT = 14401

# Global heading setting
FIXED_HEADING = -1  # -1 = auto rotate

# -------------------------
# DRONE INITIALISATION
# -------------------------
print("[INFO] Initialising drone connection...")
drone = Drone(f"udp:{MAVPROXY_IP}:{MAVPROXY_PORT}")
print("[INFO] Drone connected via MAVLink")

# -------------------------
# START CAMERA
# -------------------------
camera.start()  # <-- START CAMERA HERE
vision.start()  # start vector hearing thread

# -------------------------
# UDP COMMUNICATION
# -------------------------
PI_UDP_PORT = 9050
global GCS_ADDR
GCS_ADDR = ("127.0.0.1", 9051)

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

# -------------------------
# CLEANUP HANDLER
# -------------------------
import atexit

def cleanup():
    """Stop camera on exit"""
    print("\n[INFO] Cleaning up...")
    camera.stop()
    vision.stop()

atexit.register(cleanup)

# -------------------------
# FSM EVENT HANDLER
# -------------------------

def handle_event(event, param):
    global current_state, FIXED_HEADING, target_acquire_position, align_entered

    send_status(f"State={current_state} Event={event}", type_="FSM")

    # EMERGENCY - Highest priority, kills drone immediately
    if event == "emergency":
        print("\n EMERGENCY! Killing drone...")
        drone.emergency()
        current_state = STATE_EMERGENCY
        send_status("EMERGENCY - Drone killed", type_="EMERGENCY")
        return

    # RTL - Return to launch (interrupts anything)
    if event == "rtl":
        print("\n RTL triggered - returning to launch...")
        send_status("RTL triggered", type_="RTL")
        current_state = STATE_RTL
        drone.rtl()  # This just sets the flag and sends command
        return

    #  TAKEOFF - Can be called from any state (except emergency)
    if event == "takeoff":
        # Don't take off if in emergency
        if current_state == STATE_EMERGENCY:
            send_status("In EMERGENCY - cannot take off", type_="FSM")
            return
            
        print("\n TAKEOFF triggered...")
        
        # Arm if not already armed
        if not drone.armed:
            drone.arm()
            send_status("Drone armed", type_="FSM")
            time.sleep(1)
        
        # Get altitude
        takeoff_alt = float(param) if param is not None else 10
        
        try:
            # Takeoff can be interrupted by RTL/emergency
            drone.takeoff(takeoff_alt)
            current_state = STATE_FLIGHT
            send_status(f"Takeoff to {takeoff_alt}m — hovering", type_="FSM")
        except RuntimeError as e:
            if "RTL active" in str(e) or "Emergency active" in str(e):
                print(f"Takeoff interrupted: {e}")
            else:
                raise
        return

    # Speed command
    if event == "speed":
        if param:
            try:
                speed_val = float(param)
                drone.set_speed(speed_val)
                send_status(f"Speed set to {speed_val} m/s", type_="FSM")
            except ValueError:
                send_status(f"Invalid speed: {param}", type_="FSM")
        return

    # Heading command
    if event == "heading":
        if param:
            try:
                heading_val = float(param)
                FIXED_HEADING = heading_val
                if heading_val == -1:
                    send_status("Heading set to AUTO ROTATE", type_="FSM")
                else:
                    send_status(f"Fixed heading set to {heading_val}°", type_="FSM")
            except ValueError:
                send_status(f"Invalid heading: {param}", type_="FSM")
        return

    # Don't process other commands if in emergency
    if current_state == STATE_EMERGENCY:
        send_status("In EMERGENCY state - command ignored", type_="FSM")
        return
    
    if event == "plb_upload":
        print("\n PLB waypoints received")
    
        if not param:
            send_status("No PLB waypoints provided", type_="FSM")
            return
        
        try:
            # Parse the waypoints from param (format: "lon1,lat1,lon2,lat2,...")
            coords = [float(x) for x in param.split(',')]
            if len(coords) % 2 != 0:
                raise ValueError("Odd number of coordinates")
            
            # Create waypoints list (lon, lat, current altitude)
            _, _, current_alt = drone.position
            if current_alt is None or current_alt < 5:
                current_alt = 20  # Default altitude if not available
            
            global PLB_WPS  # Make it accessible globally
            PLB_WPS = []
            for i in range(0, len(coords), 2):
                lon, lat = coords[i], coords[i+1]
                PLB_WPS.append((lon, lat, current_alt))
            
            send_status(f"PLB mission loaded with {len(PLB_WPS)} waypoints", type_="FSM")
            print(f"[INFO] Loaded {len(PLB_WPS)} PLB waypoints")
        
        except Exception as e:
            send_status(f"Failed to parse PLB waypoints: {e}", type_="FSM")
            print(f"[ERROR] PLB parsing failed: {e}")
    
    if event == "land":
        current_state = STATE_LAND
        send_status("Descending", type_="LAND")
        try:
            drone.land()
            send_status("Landing complete", type_="LAND")
            current_state = STATE_INIT
            send_status("Returned to INIT", type_="FSM")
        except RuntimeError as e:
            if "RTL active" in str(e) or "Emergency active" in str(e):
                print(f"Land interrupted: {e}")

    # INIT state
    if current_state == STATE_INIT:
        if event == "init":
            drone.set_mode("GUIDED")
            time.sleep(1)
            send_status("Mode set to GUIDED", type_="FSM")
            current_state = STATE_TAKEOFF

    # FLIGHT state
    elif current_state == STATE_FLIGHT:
        if event == "restart":
            current_state = STATE_INIT
            
        
        elif event == "lawn":
            # --- Generate waypoints for lawnmower pattern ---
            origin_lon, origin_lat = drone.position[:2]
            altitude_m = drone.position[2] if drone.position[2] > 0 else 10

            waypoints = plan_lawnmower(
                search_polygon=SEARCH_AREA,
                altitude_m=altitude_m,
                origin_lonlat=(origin_lon, origin_lat),
                camera_ref=CameraFootprintRef(
                    ref_alt_m=50,
                    ref_width_m=52.4,
                    ref_height_m=39.3
                ),
                side_overlap=0.3,
                forward_overlap=0.7
            )

            current_state = STATE_LAWNMOWER
            send_status(f"Generated {len(waypoints)} waypoints", type_="FSM")
    
            with open("waypoint_debug.txt", "w") as debug_file:
                debug_file.write(f"=== Lawnmower Mission Debug ===\n")
                debug_file.write(f"Total waypoints: {len(waypoints)}\n")
                debug_file.write(f"Start position: {drone.position}\n\n")
                debug_file.write("=== ALL GENERATED WAYPOINTS ===\n")
                for j, wp in enumerate(waypoints):
                    debug_file.write(f"WP{j+1:2d}: ({wp.lon:.7f}, {wp.lat:.7f}, {wp.alt_m:.2f})\n")
                debug_file.write("=" * 40 + "\n\n")
        
                # Fly the waypoints - this can be interrupted by RTL/emergency
                for i, wp in enumerate(waypoints):
                    # Check for interrupts before each waypoint
                    if current_state != STATE_LAWNMOWER:
                        debug_file.write(f"State changed to {current_state}, breaking\n")
                        break

                    send_status(f"WP {i+1}/{len(waypoints)} -> ({wp.lon:.7f},{wp.lat:.7f},{wp.alt_m:.2f}m)", type_="FSM")

                    try:
                        if FIXED_HEADING != -1:
                            drone.move_to_wp(wp.lon, wp.lat, wp.alt_m, heading=FIXED_HEADING)
                        else:
                            drone.move_to_wp(wp.lon, wp.lat, wp.alt_m)
                        debug_file.write(f"move_to_wp completed successfully\n")
                    except RuntimeError as e:
                        error_str = str(e)
                        debug_file.write(f"EXCEPTION in move_to_wp: {error_str}\n")

                        if "RTL active" in error_str:
                            print("🔄 RTL interrupted lawn mission")
                            current_state = STATE_RTL
                            send_status("Lawnmower interrupted by RTL", type_="FSM")
                            return
                        elif "Emergency active" in error_str:
                            print(" Emergency interrupted lawn mission")
                            current_state = STATE_EMERGENCY
                            send_status("Lawnmower interrupted by EMERGENCY", type_="FSM")
                            return

                        elif "Visual target found" in error_str:
                            print("Visual target found, exiting lawn mission")
                            target_acquire_position = drone.position
                            align_entered = False
                            send_status("Stable target detected, entering ALIGN", type_="FSM")
                            current_state = STATE_ALIGN
                            drone.clear_visual_interrupt()
                            debug_file.write(f"Visual target acquired at position: {target_acquire_position}\n")
                            debug_file.flush()
                            break

                        else:
                            debug_file.write(f"Unhandled exception: {error_str}\n")
                            raise
            
                    debug_file.write(f"Waypoint {i+1} complete\n")
                    debug_file.flush()

                debug_file.write("Loop finished\n")
    
            if current_state == STATE_LAWNMOWER:
                send_status("Lawnmower mission complete", type_="FSM")
                current_state = STATE_FLIGHT
        
        elif event == "aoi":  # New AOI command
            send_status(f"AOI mission started with {len(AOI_WPS)} waypoints", type_="FSM")
            origin_lon, origin_lat = drone.position[:2]
            altitude_m = drone.position[2] if drone.position[2] > 0 else 10
        
            with open("waypoint_debug.txt", "a") as debug_file:
                debug_file.write(f"\n=== AOI Mission Debug ===\n")
                debug_file.write(f"Total waypoints: {len(AOI_WPS)}\n")
                debug_file.write(f"Start position: {drone.position}\n\n")
                current_state = STATE_AOI
                # Fly the AOI waypoints
                for i, wp in enumerate(AOI_WPS):
                    # Check for interrupts
                    if current_state != STATE_AOI:
                        debug_file.write(f"State changed to {current_state}, breaking\n")
                        break

                    lon, lat, alt = wp
                    send_status(f"AOI WP {i+1}/{len(AOI_WPS)} -> ({lon:.7f},{lat:.7f},{alt:.2f}m)", type_="FSM")

                    try:
                        if FIXED_HEADING != -1:
                            drone.move_to_wp(lon, lat, drone.position[:2], heading=FIXED_HEADING)
                        else:
                            drone.move_to_wp(lon, lat, drone.position[:2])
                        debug_file.write(f"AOI WP{i+1} completed successfully\n")
                    except RuntimeError as e:
                        error_str = str(e)
                        debug_file.write(f"EXCEPTION in AOI mission: {error_str}\n")

                        if "RTL active" in error_str:
                            print(" RTL interrupted AOI mission")
                            current_state = STATE_RTL
                            send_status("AOI mission interrupted by RTL", type_="FSM")
                            return
                        elif "Emergency active" in error_str:
                            print(" Emergency interrupted AOI mission")
                            current_state = STATE_EMERGENCY
                            send_status("AOI mission interrupted by EMERGENCY", type_="FSM")
                            return
                        else:
                            raise
                
                    debug_file.flush()

                debug_file.write("AOI mission complete\n")
    
            if current_state == STATE_AOI:
                send_status("AOI mission complete", type_="FSM")
                current_state = STATE_FLIGHT

        elif event == "plb_start":  # New PLB command
            current_state = STATE_PLB
            send_status(f"PLB mission started with {len(PLB_WPS)} waypoints", type_="FSM")
        
            with open("waypoint_debug.txt", "a") as debug_file:
                debug_file.write(f"\n=== PLB Mission Debug ===\n")
                debug_file.write(f"Total waypoints: {len(PLB_WPS)}\n")
                debug_file.write(f"Start position: {drone.position}\n\n")
            
                # Fly the AOI waypoints
                for i, wp in enumerate(PLB_WPS):
                    # Check for interrupts
                    if current_state != STATE_PLB:
                        debug_file.write(f"State changed to {current_state}, breaking\n")
                        break

                    lon, lat, alt = wp
                    send_status(f"PLB WP {i+1}/{len(PLB_WPS)} -> ({lon:.7f},{lat:.7f},{alt:.2f}m)", type_="FSM")

                    try:
                        if FIXED_HEADING != -1:
                            drone.move_to_wp(lon, lat, drone.position[:2], heading=FIXED_HEADING)
                        else:
                            drone.move_to_wp(lon, lat, drone.position[:2])
                        debug_file.write(f"PLB WP{i+1} completed successfully\n")
                    except RuntimeError as e:
                        error_str = str(e)
                        debug_file.write(f"EXCEPTION in PLB mission: {error_str}\n")

                        if "RTL active" in error_str:
                            print(" RTL interrupted PLB mission")
                            current_state = STATE_RTL
                            send_status("PLB mission interrupted by RTL", type_="FSM")
                            return
                        elif "Emergency active" in error_str:
                            print(" Emergency interrupted PLB mission")
                            current_state = STATE_EMERGENCY
                            send_status("PLB mission interrupted by EMERGENCY", type_="FSM")
                            return
                        else:
                            raise
                
                    debug_file.flush()

                debug_file.write("PLB mission complete\n")
    
            if current_state == STATE_AOI:
                send_status("AOI mission complete", type_="FSM")
                current_state = STATE_FLIGHT

        elif event == "return":  # New RETURN command
            current_state = STATE_RETURN
            send_status(f"RETURN mission started with {len(RETURN_WPS)} waypoints", type_="FSM")
        
            with open("waypoint_debug.txt", "a") as debug_file:
                debug_file.write(f"\n=== RETURN Mission Debug ===\n")
                debug_file.write(f"Total waypoints: {len(RETURN_WPS)}\n")
                debug_file.write(f"Start position: {drone.position}\n\n")
            
                # Fly the RETURN waypoints
                for i, wp in enumerate(RETURN_WPS):
                    # Check for interrupts
                    if current_state != STATE_RETURN:
                        debug_file.write(f"State changed to {current_state}, breaking\n")
                        break

                    lon, lat, alt = wp
                    send_status(f"RETURN WP {i+1}/{len(RETURN_WPS)} -> ({lon:.7f},{lat:.7f},{alt:.2f}m)", type_="FSM")

                    try:
                        if FIXED_HEADING != -1:
                            drone.move_to_wp(lon, lat, drone.position[:2], heading=FIXED_HEADING)
                        else:
                            drone.move_to_wp(lon, lat, drone.position[:2])
                        debug_file.write(f"RETURN WP{i+1} completed successfully\n")
                    except RuntimeError as e:
                        error_str = str(e)
                        debug_file.write(f"EXCEPTION in RETURN mission: {error_str}\n")

                        if "RTL active" in error_str:
                            print(" RTL interrupted RETURN mission")
                            current_state = STATE_RTL
                            send_status("RETURN mission interrupted by RTL", type_="FSM")
                            return
                        elif "Emergency active" in error_str:
                            print(" Emergency interrupted RETURN mission")
                            current_state = STATE_EMERGENCY
                            send_status("RETURN mission interrupted by EMERGENCY", type_="FSM")
                            return
                        else:
                            raise
                
                    debug_file.flush()

                debug_file.write("RETURN mission complete\n")
    
            if current_state == STATE_RETURN:
                send_status("RETURN mission complete", type_="FSM")
                current_state = STATE_FLIGHT

    # LAND state
    elif current_state == STATE_LAND:
        if event == 'deploy':
            set_servo_pwm(1050, 1550, 2050)
        pass

    # RTL state
    elif current_state == STATE_RTL:
        if not drone.landed():
            time.sleep(0.5)
        else:
            current_state = STATE_INIT
            
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
    global current_state, gcs_ack_lost
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
        time.sleep(1)

# -------------------------
# THREADS
# -------------------------

def keepalive_loop():
    while True:
        if gcs_connected:
            send_status("ping", type_="KEEPALIVE")
        time.sleep(5)

threading.Thread(target=keepalive_loop, daemon=True).start()
threading.Thread(target=telemetry_display, daemon=True).start()

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


# monitoring vision stream
def vision_monitor_loop():
    while True:
        try:
            if current_state == STATE_LAWNMOWER:
                if vision.mostly_visible_for(5.0, 0.9):
                    print("Target visible in >=90% of last 5s -> trigger visual interrupt")
                    drone.trigger_visual_interrupt()
                    time.sleep(0.2)
                else:
                    time.sleep(0.05)
            else:
                time.sleep(0.05)

        except Exception as e:
            print(f"[WARNING] vision_monitor_loop error: {e}")
            time.sleep(0.1)

threading.Thread(target=vision_monitor_loop, daemon=True).start()


# -------------------------
# MAIN LOOP
# -------------------------

print("[INFO] FSM running. Waiting for GCS commands.")
print("[INFO] Emergency and RTL will interrupt any active mission")

while True:
    try:
        event, param = event_queue.get(timeout=0.5)
        handle_event(event, param)
    except:
        pass
    
    # Monitor RTL progress
    if current_state == STATE_RTL and drone.landed():
        print(" RTL complete - drone landed")
    
        # ⚠️ CRITICAL: Set mode back to GUIDED for future commands
        drone.set_mode("GUIDED")
        time.sleep(1)  # Give it time to switch
    
        send_status("RTL complete - landed", type_="LAND")
        current_state = STATE_INIT
        send_status("Returned to INIT - Mode set to GUIDED", type_="FSM")
    
    # Monitor LAND progress
    if current_state == STATE_LAND and drone.landed():
        print(" Landing complete")
    
        # ⚠️ CRITICAL: Set mode back to GUIDED for future commands
        drone.set_mode("GUIDED")
        time.sleep(1)
    
        send_status("Landing complete", type_="LAND")
        current_state = STATE_INIT
        send_status("Returned to INIT - Mode set to GUIDED", type_="FSM")

    # Executed only once when entering ALIGN
    if current_state == STATE_ALIGN and not align_entered:
        print("Entering ALIGN state")
        send_status("Entered ALIGN state", type_="FSM")
        print(f"Target acquired at: {target_acquire_position}")
        align_entered = True
        drone.set_speed(0.5)

    # The logic for exiting ALIGN
    if current_state == STATE_ALIGN:
        latest = vision.get_latest()

        if latest is not None:
            print(f"[ALIGN] latest vision = {latest}")

        if vision.lost_for(0.5):
            print("[ALIGN] target lost for 0.5s -> exit ALIGN")
            send_status("Target lost in ALIGN, returning to FLIGHT", type_="FSM")
            current_state = STATE_FLIGHT
            align_entered = False

    time.sleep(0.05)