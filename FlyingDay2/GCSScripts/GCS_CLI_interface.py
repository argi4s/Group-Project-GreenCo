#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import threading
import sys
import readline
import os
import time
import struct
from pymavlink import mavutil

# Pi FSM communication
PI_ADDR = ("192.168.1.69", 9050)  # Pi's FSM UDP port
GCS_UDP_PORT = 9051

# MAVLink telemetry from MAVProxy
MAVPROXY_IP = "192.168.1.17"
MAVPROXY_PORT = 14402  # Your MAVProxy UDP port

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", GCS_UDP_PORT))
sock.settimeout(0.2)

# MAVLink connection for telemetry
mav_conn = mavutil.mavlink_connection(f"udp:{MAVPROXY_IP}:{MAVPROXY_PORT}")

shutdown_event = threading.Event()

VALID_EVENTS = [
    "init",
    "takeoff",
    "land",
    "emergency",
    "rtl",
    "lawn",
    "restart",
    "move_point",
    "auto_lawn"
]

# -------------------------
# Globals
# -------------------------
pi_state = "--"              # FSM state from Pi
last_pi_msg_time = 0
PI_TIMEOUT = 2                # seconds before marking disconnected
print_lock = threading.Lock()

# MAVLink telemetry data
mav_telemetry = {
    "mode": "--",
    "armed": "--",
    "lat": "--",
    "lon": "--",
    "alt": "--",
    "gps_fix": "--",
    "heading": "--",
    "last_update": 0
}
MAV_TIMEOUT = 3               # seconds before marking MAVLink stale

# Heartbeat blink toggle
heartbeat_state = True

# ANSI colors
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
RESET = "\033[0m"
BOLD = "\033[1m"

# -------------------------
# Helper to print above CLI input
# -------------------------
def print_above_input(msg):
    with print_lock:
        saved = readline.get_line_buffer()
        sys.stdout.write('\033[2K\r')  # Clear current line
        sys.stdout.write(f"{msg}\n")
        sys.stdout.write(f"> {saved}")
        sys.stdout.flush()

# -------------------------
# Listen for Pi FSM status messages (only state info now)
# -------------------------
def pi_status_listener():
    global last_pi_msg_time, pi_state

    seq_received = set()

    while not shutdown_event.is_set():
        try:
            data, addr = sock.recvfrom(1024)
            text = data.decode().strip()

            if text.startswith("SEQ="):
                parts = text.split(" ", 2)
                seq = int(parts[0].split("=")[1])
                msg_type = parts[1].split("=")[1]
                content = parts[2].split("=", 1)[1]

                # ACK back to Pi
                ack = f"ACK SEQ={seq} TYPE={msg_type}"
                sock.sendto(ack.encode(), addr)

                # Deduplicate
                if seq in seq_received:
                    continue
                seq_received.add(seq)

                # Update Pi state from FSM messages
                if msg_type == "FSM" and "State=" in content:
                    # Extract state from FSM message
                    for part in content.split():
                        if part.startswith("State="):
                            pi_state = part.split("=")[1]
                            break
                
                last_pi_msg_time = time.time()

            else:
                print_above_input(f"[UNKNOWN] {text}")

        except socket.timeout:
            continue

# -------------------------
# MAVLink telemetry listener (direct from MAVProxy)
# -------------------------
def mavlink_listener():
    while not shutdown_event.is_set():
        try:
            # Non-blocking receive
            msg = mav_conn.recv_match(blocking=False, timeout=0.1)
            
            if msg is None:
                time.sleep(0.05)
                continue

            msg_type = msg.get_type()
            
            # Update telemetry data
            if msg_type == "HEARTBEAT":
                mav_telemetry["mode"] = mavutil.mode_string_v10(msg)
                mav_telemetry["armed"] = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                mav_telemetry["last_update"] = time.time()
                
            elif msg_type == "GLOBAL_POSITION_INT":
                mav_telemetry["lat"] = msg.lat / 1e7
                mav_telemetry["lon"] = msg.lon / 1e7
                mav_telemetry["alt"] = msg.relative_alt / 1000.0
                mav_telemetry["heading"] = msg.hdg / 100.0 if msg.hdg != 65535 else 0
                mav_telemetry["last_update"] = time.time()
                
            elif msg_type == "GPS_RAW_INT":
                mav_telemetry["gps_fix"] = msg.fix_type
                mav_telemetry["last_update"] = time.time()
                
            elif msg_type == "VFR_HUD":
                # Optional: get airspeed, groundspeed, etc.
                pass
                
        except Exception as e:
            # Silently ignore errors in telemetry thread
            time.sleep(0.1)

# -------------------------
# Dashboard loop
# -------------------------
def dashboard_loop():
    global heartbeat_state
    while not shutdown_event.is_set():
        with print_lock:
            os.system("clear")
            print(f"{BOLD}======= GCS TELEMETRY ======={RESET}")

            # Pi FSM Connection status
            if time.time() - last_pi_msg_time > PI_TIMEOUT:
                pi_conn_status = f"{RED}DISCONNECTED{RESET}"
            else:
                pi_conn_status = f"{GREEN}CONNECTED{RESET}"
            print(f"PI FSM  : {pi_conn_status}")

            # MAVLink Connection status
            if time.time() - mav_telemetry["last_update"] > MAV_TIMEOUT:
                mav_conn_status = f"{RED}STALE{RESET}"
            else:
                mav_conn_status = f"{GREEN}ACTIVE{RESET}"
            print(f"MAVLink : {mav_conn_status}")

            # Pi FSM State
            print(f"PI STATE: {pi_state}")

            # MAVLink Telemetry
            print(f"\n{BOLD}--- Drone Telemetry (from MAVProxy) ---{RESET}")
            print(f"MODE  : {mav_telemetry['mode']}")
            print(f"ARMED : {mav_telemetry['armed']}")
            
            lat = mav_telemetry['lat']
            lon = mav_telemetry['lon']
            alt = mav_telemetry['alt']
            
            print(f"LAT   : {lat:.7f}" if lat != "--" else "LAT   : --")
            print(f"LON   : {lon:.7f}" if lon != "--" else "LON   : --")
            print(f"ALT   : {alt:.2f} m" if alt != "--" else "ALT   : --")
            print(f"GPS   : {mav_telemetry['gps_fix']}")
            print(f"HDG   : {mav_telemetry['heading']:.1f}°" if mav_telemetry['heading'] != "--" else "HDG   : --")

            # Heartbeat indicator
            heartbeat_symbol = f"{GREEN}+{RESET}" if heartbeat_state else f"{YELLOW}-{RESET}"
            heartbeat_state = not heartbeat_state
            print(f"\nHEARTBEAT : {heartbeat_symbol}")

            # Instructions / valid events
            print(f"\n{BOLD}Instructions :{RESET} {', '.join(VALID_EVENTS)}")

            print(f"{BOLD}=============================={RESET}")
            print("> ", end="", flush=True)

        time.sleep(1)

# -------------------------
# Send FSM events to Pi
# -------------------------
def send_fsm_event(event, param=None):
    msg = f"{event} {param}" if param else event
    sock.sendto(msg.encode(), PI_ADDR)
    print_above_input(f"{CYAN}[SENT]{RESET} {msg} to Pi")

# -------------------------
# CLI loop
# -------------------------
def cli_loop():
    print("=== Drone GCS CLI ===")
    print("Commands:", ", ".join(VALID_EVENTS))
    print("Type 'exit' to quit.\n")

    while True:
        try: 
            cmd = input("> ").strip()
        except EOFError:
            break

        if not cmd:
            continue

        if cmd.lower() == "exit":
            shutdown_event.set()
            sock.close()
            break

        parts = cmd.split()
        event = parts[0]
        param = parts[1] if len(parts) > 1 else None

        if event not in VALID_EVENTS:
            print_above_input(f"{RED}[ERROR]{RESET} Invalid command: {event}")
            continue

        send_fsm_event(event, param)

# -------------------------
# Main
# -------------------------
if __name__ == "__main__":
    # Start listener threads
    threading.Thread(target=pi_status_listener, daemon=True).start()
    threading.Thread(target=mavlink_listener, daemon=True).start()
    threading.Thread(target=dashboard_loop, daemon=True).start()
    
    print(f"{GREEN}[INFO]{RESET} Connected to MAVLink at {MAVPROXY_IP}:{MAVPROXY_PORT}")
    print(f"{GREEN}[INFO]{RESET} Listening for Pi FSM at 0.0.0.0:{GCS_UDP_PORT}")
    
    cli_loop()