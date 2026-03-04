#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GCS FSM Command Sender
Send FSM events to companion computer over MAVLink
"""

from pymavlink import mavutil

# --------------------------
# MAVLink connection setup
# --------------------------
# Replace with Pi's IP and MAVProxy port
MAVLINK_CONN = "udp:192.168.0.249:14551"  

print(f"[INFO] Connecting to MAVLink at {MAVLINK_CONN}...")
drone = mavutil.mavlink_connection(MAVLINK_CONN)
drone.wait_heartbeat()
print("[INFO] Heartbeat received from vehicle. Ready to send FSM commands.\n")

# --------------------------
# CLI loop for sending FSM events
# --------------------------
VALID_EVENTS = [
    "start_search",
    "begin_search",
    "detect",
    "specific_search",
    "approve_approach",
    "confirm",
    "approve_return",
    "emergency"
]

def send_fsm_event(event, param=None):
    """Send FSM command as MAVLink STATUSTEXT to companion computer"""
    if param:
        text = f"{event} {param}"
    else:
        text = event
    drone.mav.statustext_send(
        mavutil.mavlink.MAV_SEVERITY_INFO,
        text.encode()
    )
    print(f"[SENT] {text}")

def cli_loop():
    print("=== FSM GCS Command Sender ===")
    print("Type commands to trigger FSM on companion computer.")
    print("Valid commands:", ", ".join(VALID_EVENTS))
    print("Add parameter if needed, e.g., approve_approach N")
    print("Type 'exit' to quit.\n")

    while True:
        cmd_input = input("> ").strip()
        if not cmd_input:
            continue
        if cmd_input.lower() == "exit":
            print("Exiting.")
            break

        parts = cmd_input.split()
        event = parts[0]
        param = parts[1] if len(parts) > 1 else None

        if event not in VALID_EVENTS:
            print(f"[ERROR] Invalid event: {event}")
            continue

        send_fsm_event(event, param)

if __name__ == "__main__":
    cli_loop()
