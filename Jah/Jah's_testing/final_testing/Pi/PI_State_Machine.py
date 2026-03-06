#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pi-side FSM Companion Computer

- Receives FSM events from GCS over a dedicated UDP socket
- Executes FSM actions (takeoff, waypoints, payload steps, etc.) via Drone class
- Sends FSM status updates back to GCS over UDP
"""

from drone_control import Drone  # <- your new class
from queue import Queue
import threading
import time
import socket

# -------------------------
# FSM States
# -------------------------
STATE_INIT = "INIT"
STATE_FIND_AOI = "FIND_AOI"
STATE_SEARCH = "SEARCH"
STATE_SPECIFIC_SEARCH = "SPECIFIC_SEARCH"
STATE_DETECTION = "DETECTION"
STATE_APPROACH = "APPROACH"
STATE_PAYLOAD_DEPLOY = "PAYLOAD_DEPLOY"
STATE_RETURN = "RETURN"
STATE_EMERGENCY = "EMERGENCY"

current_state = STATE_INIT
payload_step = 0
detection_param = None
return_approved = False
aoi_approved = False

# -------------------------
# FSM Transition Table
# -------------------------
TRANSITIONS = {
    STATE_INIT: {"start_mission": STATE_FIND_AOI},
    STATE_SEARCH: {"detect": STATE_DETECTION, "specific_search": STATE_SPECIFIC_SEARCH},
    STATE_SPECIFIC_SEARCH: {"detect": STATE_DETECTION},
    STATE_DETECTION: {"approve_approach": STATE_APPROACH},
    STATE_APPROACH: {"begin_payload": STATE_PAYLOAD_DEPLOY},
    STATE_PAYLOAD_DEPLOY: {"payload_done": STATE_RETURN},
    STATE_RETURN: {"approve_return": STATE_RETURN},
}

# -------------------------
# Waypoints
# -------------------------
RETURN_WPS = [
    (51.4233850, -2.6716371, 49.35),
    (51.4237044, -2.6669526, 10.0),
    (51.4227243, -2.6662177, 10.0),
    (51.4217375, -2.6699513, 10.0),
    (51.4233848, -2.6716378, 10.0),
]

AOI_WPS = [
    (51.4233850, -2.6716371, 49.35),
    (0.0, 0.0, 10.0),
    (51.4215703, -2.6699030, 10.0),
    (51.4227544, -2.6660031, 10.0),
    (51.4244403, -2.6673871, 10.0),
]

# -------------------------
# Drone Initialization
# -------------------------
drone = Drone("udp:127.0.0.1:14552")
print("[INFO] Drone class initialized.")

# -------------------------
# UDP FSM Comms (Pi -> GCS)
# -------------------------
PI_UDP_PORT = 9050
GCS_ADDR = ("192.168.1.17", 9051)

udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_sock.bind(("", PI_UDP_PORT))
udp_sock.settimeout(0.2)

# -------------------------
# Event Queue
# -------------------------
event_queue = Queue()

def udp_listener():
    """Listen for FSM events from GCS over UDP"""
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
# FSM Helpers
# -------------------------
seq_counter = 0
ACK_TIMEOUT = 0.5
MAX_RETRIES = 5

def send_status(msg):
    global seq_counter
    seq_counter += 1
    seq = seq_counter
    payload = f"SEQ={seq} TYPE=STATUS MSG={msg}"
    for _ in range(MAX_RETRIES):
        udp_sock.sendto(payload.encode(), GCS_ADDR)
        try:
            udp_sock.settimeout(ACK_TIMEOUT)
            data, _ = udp_sock.recvfrom(1024)
            if data.decode().strip() == f"ACK SEQ={seq} TYPE=STATUS":
                return True
        except socket.timeout:
            continue
    print(f"[WARNING] Status message not acknowledged: {msg}")
    return False

def send_waypoints(wps):
    """Upload waypoints using drone.move_to_wp"""
    for lat, lon, alt in wps:
        try:
            drone.move_to_wp(lat, lon, alt)
        except RuntimeError as e:
            send_status(f"[WAYPOINT INTERRUPTED] {e}")
            break

# -------------------------
# FSM Event Handlers
# -------------------------
def on_event(state, event, param=None):
    global payload_step, detection_param, return_approved

    send_status(f"[FSM] State={state} Event={event} Param={param}")

    if state == STATE_INIT:
        send_status("INIT: Waiting for start_search")

    elif state == STATE_FIND_AOI:
        if not aoi_approved:
            send_status("FIND_AOI: Awaiting AOI approval")
        else:
            send_status("FIND_AOI: Approved -> flying to AOI")

    elif state == STATE_SEARCH:
        if event == "detect":
            send_status("SEARCH: Detection -> holding position")
        elif event == "specific_search":
            send_status("SEARCH: Entering specific search")

    elif state == STATE_SPECIFIC_SEARCH:
        if event == "detect":
            send_status("SPECIFIC_SEARCH: Detection -> holding position")

    elif state == STATE_DETECTION:
        send_status("DETECTION: Target locked -> awaiting approve_approach")
        if param:
            detection_param = param.upper()
            send_status(f"DETECTION PARAM SET: {detection_param}")

    elif state == STATE_APPROACH:
        send_status("APPROACH: Holding near target")

    elif state == STATE_PAYLOAD_DEPLOY:
        steps = [
            "Confirm payload deployment",
            "Detach first loop",
            "Detach payload",
            "Take-off"
        ]
        if payload_step < len(steps):
            send_status(f"PAYLOAD STEP {payload_step+1}: {steps[payload_step]}")

    elif state == STATE_RETURN:
        if not return_approved:
            send_status("RETURN: Awaiting approve_return")
        else:
            send_status("RETURN: Returning home")

    elif state == STATE_EMERGENCY:
        send_status("EMERGENCY: Pilot Control")

def handle_event(event, param=None):
    global current_state, payload_step, return_approved, aoi_approved

    if event == "emergency":
        drone.emergency()
        current_state = STATE_EMERGENCY
        on_event(current_state, event)
        return

    if event == "rtl":
        drone.rtl()
        return

    on_event(current_state, event, param)

    if current_state == STATE_PAYLOAD_DEPLOY and event == "confirm":
        payload_step += 1
        if payload_step >= 4:
            send_status("PAYLOAD: Complete")
            current_state = STATE_RETURN
        return

    if current_state == STATE_DETECTION and event == "approve_approach":
        current_state = STATE_APPROACH
        return

    if current_state == STATE_RETURN:
        if event == "approve_return":
            return_approved = True
        return

    if current_state == STATE_FIND_AOI and event == "gcs_confirm":
        aoi_approved = True
        send_status("AOI APPROVED -> uploading waypoints")
        send_waypoints(AOI_WPS)
        current_state = STATE_SEARCH
        return

    allowed = TRANSITIONS.get(current_state, {})
    if event in allowed:
        next_state = allowed[event]
        send_status(f"TRANSITION: {current_state} -> {next_state}")
        current_state = next_state

# -------------------------
# Heartbeat / Status Thread
# -------------------------
def heartbeat_loop():
    """Send FSM status periodically"""
    global current_state
    while True:
        lat, lon, alt = drone.position
        next_wp = AOI_WPS[0] if current_state in [STATE_SEARCH, STATE_SPECIFIC_SEARCH] else RETURN_WPS[0]
        status_msg = f"[HEARTBEAT] State={current_state} Pos=({lat},{lon},{alt}) NextWP={next_wp}"
        send_status(status_msg)
        time.sleep(1)

threading.Thread(target=heartbeat_loop, daemon=True).start()

# -------------------------
# FSM Main Loop
# -------------------------
print("[INFO] FSM running. Waiting for UDP events from GCS...")

while True:
    try:
        event, param = event_queue.get(timeout=0.5)
        handle_event(event, param)
    except:
        pass
    time.sleep(0.05)
