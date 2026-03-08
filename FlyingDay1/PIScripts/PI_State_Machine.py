#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from DroneClass import Drone
from queue import Queue
import threading
import time
import socket
import os

from lawnmower import plan_lawnmower, CameraFootprintRef, Waypoint

# -------------------------
# STATES
# -------------------------
STATE_INIT = "INIT"
STATE_TAKEOFF = "TAKEOFF"
STATE_FLIGHT = "FLIGHT"
STATE_LAND = "LAND"
STATE_EMERGENCY = "EMERGENCY"
STATE_LAWNMOWER = "LAWN"
STATE_MOVE_TO_POINT = "MOVE"

current_state = STATE_INIT


# -------------------------
# HARD-CODED LAWNMOWER AREA (lat, lon, alt=0)
# -------------------------
lawn_polygon = [
    (-2.670948345438704, 51.42326956502679),
    (-2.670045428650557, 51.42287025017865),
    (-2.668169295906676, 51.42336622593724),
    (-2.668809768621569, 51.42421477437771),
    (-2.671277780473196, 51.42354069739116),
    (-2.670948345438704, 51.42326956502679)
]

# -------------------------
# DRONE INITIALISATION
# -------------------------

print("[INFO] Initialising drone connection...")
drone = Drone("udp:192.168.0.249:14450")
print("[INFO] Drone connected via MAVLink")

# -------------------------
# UDP COMMUNICATION
# -------------------------

PI_UDP_PORT = 9050
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

# Global flags
gcs_ack_lost = False
gcs_connected = False
last_ack_time = 0
GCS_TIMEOUT = 2  # seconds before marking disconnected

def send_status(msg, type_="STATUS"):
    """
    Send a status message to the GCS with a type.
    type_ should be one of: FSM, HEARTBEAT, TAKEOFF, LAND, EMERGENCY, RTL
    """
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
# FSM EVENT HANDLER
# -------------------------

# -------------------------
# FSM EVENT HANDLER
# -------------------------
def handle_event(event):
    global current_state

    send_status(f"State={current_state} Event={event}", type_="FSM")

    if event == "emergency":
        drone.emergency()
        current_state = STATE_EMERGENCY
        send_status("Drone disarmed", type_="EMERGENCY")
        return

    if event == "rtl":
        drone.rtl()
        send_status("RTL triggered — returning to launch", type_="RTL")
        current_state = STATE_LAND  # mark as landing

        # Wait until drone lands
        while not drone.landed():
            time.sleep(0.5)
        send_status("RTL complete — landed", type_="LAND")
        current_state = STATE_INIT
        send_status("Returned to INIT", type_="FSM")
        return

    # INIT state
    if current_state == STATE_INIT:
        if event == "init":
            drone.set_mode("GUIDED")
            time.sleep(1)
            send_status("Mode set to GUIDED", type_="FSM")
            current_state = STATE_TAKEOFF

    # TAKEOFF state
    elif current_state == STATE_TAKEOFF:
        if event == "takeoff":
            drone.arm()
            send_status("Drone armed", type_="FSM")
            time.sleep(1)
            drone.takeoff(10)  # example altitude
            current_state = STATE_FLIGHT
            send_status("Takeoff attempt — hovering", type_="FSM")


    # FLIGHT state
    elif current_state == STATE_FLIGHT:
        if event == "restart":
            current_state = STATE_INIT
        if event == "land":
            current_state = STATE_LAND
            send_status("Descending", type_="LAND")
            drone.land()
            send_status("Landing complete", type_="LAND")
            current_state = STATE_INIT
            send_status("Returned to INIT", type_="FSM")
        elif event == "move_point":
            # Go to first point of lawn polygon
            target_lat, target_lon = lawn_polygon[0]
            altitude_m = drone.position[2] if drone.position[2] > 0 else 10

            current_state = STATE_MOVE_TO_POINT
            send_status(f"Moving to test point ({target_lat},{target_lon},{altitude_m})", type_="FSM")

            # Use proper move_to_wp function
            drone.move_to_wp(target_lat, target_lon, altitude_m)

            send_status("Arrived at test point", type_="FSM")
            current_state = STATE_FLIGHT

        elif event == "lawn":
            # --- Generate waypoints for lawnmower pattern ---
            origin = drone.position[:2]  # current lat/lon
            altitude_m = drone.position[2] if drone.position[2] > 0 else 10

            waypoints = plan_lawnmower(
                search_polygon=lawn_polygon,
                altitude_m=altitude_m,
                origin_latlon=origin,
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

            # Fly the waypoints
            for i, wp in enumerate(waypoints):

                if current_state != STATE_LAWNMOWER:
                    break  # interrupted by emergency / rtl

                send_status(
                    f"WP {i+1}/{len(waypoints)} -> ({wp.lat:.7f},{wp.lon:.7f},{wp.alt_m:.2f}m)",
                    type_="FSM"
                )

                # Use DroneClass navigation
                drone.move_to_wp_queue(wp.lat, wp.lon, wp.alt_m)

            send_status("Lawnmower mission complete", type_="FSM")
            current_state = STATE_FLIGHT
     
  
        

# -------------------------
# HEARTBEAT
# -------------------------

def heartbeat_loop():
    global current_state
    while True:
        lat, lon, alt = drone.position
        mode = drone.mode
        armed = drone.armed
        gps = drone.gps_fix

        status_msg = (
            f"State={current_state} "
            f"Mode={mode} Armed={armed} "
            f"Pos=({lat},{lon},{alt}) "
            f"GPS={gps}"
        )
        send_status(status_msg, type_="HEARTBEAT")
        time.sleep(1)

# -------------------------
# TELEMETRY DISPLAY
# -------------------------

def telemetry_display():
    global current_state, gcs_ack_lost
    while True:
        lat, lon, alt = drone.position
        mode = drone.mode
        armed = drone.armed
        gps = drone.gps_fix
        conn_status = "CONNECTED" if gcs_connected else "DISCONNECTED"

        alt_str = f"{alt:.2f} m" if alt is not None else "--"
        lat_str = f"{lat:.7f}" if lat is not None else "--"
        lon_str = f"{lon:.7f}" if lon is not None else "--"
        gps_str = str(gps) if gps is not None else "--"
        mode_str = mode if mode else "--"

        os.system("clear")
        print("====================================")
        print("        DRONE TELEMETRY PANEL       ")
        print("====================================")
        print(f"STATE : {current_state}")
        print(f"MODE  : {mode_str}")
        print(f"ARMED : {armed}")
        print(f"ALT   : {alt_str}")
        print(f"LAT   : {lat_str}")
        print(f"LON   : {lon_str}")
        print(f"GPS   : {gps_str}")
        print(f"GCS   : {conn_status}")
        if gcs_ack_lost:
            print("\n[WARNING] ACK not received — check GCS connection!")
        print("====================================")
        time.sleep(1)

# -------------------------
# THREADS
# -------------------------

threading.Thread(target=heartbeat_loop, daemon=True).start()
threading.Thread(target=telemetry_display, daemon=True).start()

# -------------------------
# MAIN LOOP
# -------------------------

print("[INFO] FSM running. Waiting for GCS commands.")
while True:
    try:
        event, _ = event_queue.get(timeout=0.5)
        handle_event(event)
    except:
        pass
    time.sleep(0.05)