#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pi-side FSM Companion Computer

- Receives FSM events via MAVLink STATUSTEXT from GCS
- Executes FSM actions (takeoff, waypoints, payload steps, etc.)
- Sends back FSM status updates via STATUSTEXT
"""

from pymavlink import mavutil
from queue import Queue
import threading
import time

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
    STATE_INIT: {"start_search": STATE_FIND_AOI},  # waits for GCS approval
    STATE_SEARCH: {"detect": STATE_DETECTION, "specific_search": STATE_SPECIFIC_SEARCH},
    STATE_SPECIFIC_SEARCH: {"detect": STATE_DETECTION},
    STATE_DETECTION: {"approve_approach": STATE_APPROACH},
    STATE_APPROACH: {"begin_payload": STATE_PAYLOAD_DEPLOY},
    STATE_PAYLOAD_DEPLOY: {"payload_done": STATE_RETURN},
    STATE_RETURN: {"approve_return": STATE_RETURN},
}

#--------------------------
# Waypoints
#--------------------------

# Return home points
RETURN_WPS = [
    (51.4233850, -2.6716371, 49.35),
    (51.4237044, -2.6669526, 10.0),
    (51.4227243, -2.6662177, 10.0),
    (51.4217375, -2.6699513, 10.0),
    (51.4233848, -2.6716378, 10.0),
]

# AOI points
AOI_WPS = [
    (51.4233850, -2.6716371, 49.35),
    (0.0, 0.0, 10.0),  # placeholder for dummy WP
    (51.4215703, -2.6699030, 10.0),
    (51.4227544, -2.6660031, 10.0),
    (51.4244403, -2.6673871, 10.0),
]

# -------------------------
# MAVLink Connection
# -------------------------
MAVLINK_CONN = "udp:127.0.0.1:14552"  # local MAVProxy port
drone = mavutil.mavlink_connection(MAVLINK_CONN)
print(f"[INFO] Connecting to MAVLink on {MAVLINK_CONN}...")
drone.wait_heartbeat()
print("[INFO] Heartbeat received. FSM ready.")

# -------------------------
# Event Queue
# -------------------------
event_queue = Queue()

def mavlink_listener():
    """Listen for STATUSTEXT messages from GCS and push to FSM queue"""
    while True:
        msg = drone.recv_match(type='STATUSTEXT', blocking=True)
        if msg:
            text = msg.text.strip()
            allowed_events = ["start_search","begin_search","detect","specific_search",
                              "approve_approach","confirm","approve_return","emergency","gcs_confirm"]
            for evt in allowed_events:
                if text.startswith(evt):
                    parts = text.split()
                    param = parts[1] if len(parts) > 1 else None
                    event_queue.put((evt, param))
                    break

listener_thread = threading.Thread(target=mavlink_listener, daemon=True)
listener_thread.start()

# -------------------------
# FSM Event Handler
# -------------------------
def send_status(msg, severity=mavutil.mavlink.MAV_SEVERITY_INFO):
    """Send FSM status message via MAVLink"""
    drone.mav.statustext_send(severity, msg.encode())

def on_event(state, event, param=None):
    global payload_step, detection_param, return_approved

    # Report state to GCS
    send_status(f"[FSM] Current state: {state}, Event: {event}, Param: {param}")

    if state == STATE_INIT:
        send_status("INIT: Waiting for start_search command")

    elif state == STATE_FIND_AOI:
        if not aoi_approved:
            send_status("FIND_AOI: Awaiting AOI approval (gcs_confirm)")
        else:
            send_status("FIND_AOI: Approved — flying to AOI")

    elif state == STATE_SEARCH:
        if event == "detect":
            send_status("SEARCH: Sensor detected object — HOLDING position")
        elif event == "specific_search":
            send_status("SEARCH: Entering specific search region")

    elif state == STATE_SPECIFIC_SEARCH:
        if event == "detect":
            send_status("SPECIFIC_SEARCH: Sensor detected object — HOLDING position")

    elif state == STATE_DETECTION:
        send_status("DETECTION: Target locked — waiting for approve_approach")
        if param:
            detection_param = param.upper()
            send_status(f"DETECTION PARAMETER SET: {detection_param}")

    elif state == STATE_APPROACH:
        send_status("APPROACH: Hovering near target — waiting for payload deployment")

    elif state == STATE_PAYLOAD_DEPLOY:
        msgs = [
            "Confirm payload deployment",
            "Detach first loop",
            "Detach payload",
            "Take-off"
        ]
        if payload_step < len(msgs):
            send_status(f"PAYLOAD STEP {payload_step+1}: {msgs[payload_step]}")

    elif state == STATE_RETURN:
        if not return_approved:
            send_status("RETURN: Awaiting approve_return")
        else:
            send_status("RETURN: Returning home")

    elif state == STATE_EMERGENCY:
        send_status("EMERGENCY: Interrupt triggered! Landing immediately", severity=mavutil.mavlink.MAV_SEVERITY_CRITICAL)

def handle_event(event, param=None):
    global current_state, payload_step, detection_param, return_approved, aoi_approved

    if event == "emergency":
        current_state = STATE_EMERGENCY
        on_event(current_state, event)
        return

    # Execute FSM action
    on_event(current_state, event, param)

    # Payload step handling
    if current_state == STATE_PAYLOAD_DEPLOY and event == "confirm":
        payload_step += 1
        if payload_step >= 4:
            send_status("PAYLOAD: Deployment complete")
            current_state = STATE_RETURN
        return

    # Detection approval
    if current_state == STATE_DETECTION and event == "approve_approach":
        current_state = STATE_APPROACH
        return

    # RETURN requires approval
    if current_state == STATE_RETURN:
        if event == "approve_return":
            return_approved = True
        return
    
    if current_state == STATE_FIND_AOI:
        if event == "gcs_confirm":
            aoi_approved = True
            send_status("AOI APPROVED — uploading AOI waypoints")
            send_waypoints(AOI_WPS)
            current_state = STATE_SEARCH
        return
    
    # Standard transitions
    allowed = TRANSITIONS.get(current_state, {})
    if event in allowed:
        next_state = allowed[event]
        send_status(f"[FSM] Transition: {current_state} -> {next_state}")
        current_state = next_state

def send_waypoints(wps):
    """
    Sends a list of (lat, lon, alt) tuples as a mission to the vehicle.
    Assumes vehicle is in GUIDED mode.
    """
    # Clear existing mission
    drone.mav.mission_clear_all_send(drone.target_system, drone.target_component)
    time.sleep(1)

    # Send each waypoint
    for i, (lat, lon, alt) in enumerate(wps):
        drone.mav.mission_item_send(
            drone.target_system,
            drone.target_component,
            i,                   # seq
            0,                   # frame: 0 = global relative
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0,                   # current
            0,                   # autocontinue
            0, 0, 0, 0,          # params 1-4 (hold, radius, etc.)
            lat,
            lon,
            alt
        )
    # Send a mission count
    drone.mav.mission_count_send(drone.target_system, drone.target_component, len(wps))


# -------------------------
# FSM Main Loop
# -------------------------
print("[INFO] FSM running. Waiting for events from GCS...")

while True:
    try:
        event, param = event_queue.get(timeout=0.5)
        handle_event(event, param)
    except:
        # No event, just loop and continue
        pass
    time.sleep(0.05)
