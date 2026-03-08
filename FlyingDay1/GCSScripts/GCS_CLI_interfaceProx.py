#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import threading
import sys
import readline
import os
import time
import struct

# -------------------------
# Addresses
# -------------------------
PI_ADDR = ("127.0.0.1", 9050)  # Pi command port
MAVPROXY_ADDR = ("192.168.0.249", 14888)  # MAVProxy telemetry port
GCS_UDP_PORT = 9051

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", GCS_UDP_PORT))
sock.settimeout(0.2)

shutdown_event = threading.Event()

VALID_EVENTS = [
    "init",
    "takeoff",
    "land",
    "emergency",
    "rtl",
    "lawn",
    "restart",
    "move_point"
]

# -------------------------
# Globals
# -------------------------
status_messages = {}       # Latest messages keyed by TYPE
last_pi_msg_time = 0
last_mavproxy_msg_time = 0
PI_TIMEOUT = 2             # seconds before marking disconnected
MAVPROXY_TIMEOUT = 2       # seconds before marking MAVProxy disconnected
print_lock = threading.Lock()

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
# Listen for Pi status messages
# -------------------------
def status_listener():
    global last_pi_msg_time

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

                # Update latest message
                status_messages[msg_type] = content
                last_pi_msg_time = time.time()

            else:
                print_above_input(f"[UNKNOWN] {text}")

        except socket.timeout:
            continue

# -------------------------
# Listen for MAVLink telemetry from MAVProxy
# -------------------------
def mavproxy_listener():
    global last_mavproxy_msg_time

    mav_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    mav_socket.bind(MAVPROXY_ADDR)
    mav_socket.settimeout(0.2)

    while not shutdown_event.is_set():
        try:
            data, addr = mav_socket.recvfrom(1024)
            # Parse MAVLink message here (simplified version)
            # MAVLink messages have specific formats, depending on message type (e.g., GPS, battery)
            msg_type = data[0]  # Example for message type detection
            content = data[1:].decode().strip()

            # Example: Assuming msg_type is 1 for GPS data (this is a simplification)
            if msg_type == 1:  # Mocking GPS data
                lat, lon, alt = struct.unpack('fff', content.encode('utf-8'))  # Simplified unpacking
                status_messages["GPS"] = f"Lat: {lat}, Lon: {lon}, Alt: {alt}"
                last_mavproxy_msg_time = time.time()

        except socket.timeout:
            continue

# -------------------------
# Dashboard loop
# -------------------------
def dashboard_loop():
    global heartbeat_state
    while not shutdown_event.is_set():
        with print_lock:
            os.system("clear")
            print(f"{BOLD}======= GCS TELEMETRY ======={RESET}")

            # Connection status to Pi
            if time.time() - last_pi_msg_time > PI_TIMEOUT:
                conn_status = f"{RED}DISCONNECTED{RESET}"
            else:
                conn_status = f"{GREEN}CONNECTED{RESET}"

            print(f"PI : {conn_status}")

            # MAVProxy connection status
            if time.time() - last_mavproxy_msg_time > MAVPROXY_TIMEOUT:
                mav_conn_status = f"{RED}DISCONNECTED{RESET}"
            else:
                mav_conn_status = f"{GREEN}CONNECTED{RESET}"

            print(f"MAVPROXY : {mav_conn_status}")

            # Parse Pi heartbeat for lat/lon/alt/state
            state = "--"
            mode = "--"
            armed = "--"
            lat = "--"
            lon = "--"
            alt = "--"
            gps_fix = "--"

            heartbeat = status_messages.get("HEARTBEAT", "")
            if heartbeat:
                parts = heartbeat.split(" ")
                for p in parts:
                    if p.startswith("State="):
                        state = p.split("=")[1]
                    elif p.startswith("Mode="):
                        mode = p.split("=")[1]
                    elif p.startswith("Armed="):
                        armed = p.split("=")[1]
                    elif p.startswith("Pos="):
                        pos = p.split("=")[1].strip("()")
                        lat_val, lon_val, alt_val = pos.split(",")
                        lat = f"{float(lat_val):.7f}"
                        lon = f"{float(lon_val):.7f}"
                        alt = f"{float(alt_val):.2f} m"
                    elif p.startswith("GPS="):
                        gps_fix = p.split("=")[1]

            # MAVProxy telemetry (simplified)
            gps_data = status_messages.get("GPS", "No GPS data")
            if gps_data != "No GPS data":
                # Example: "Lat: 51.123456, Lon: -0.123456, Alt: 100.0"
                lat, lon, alt = gps_data.split(", ")
                lat = lat.split(":")[1].strip()
                lon = lon.split(":")[1].strip()
                alt = alt.split(":")[1].strip()

            # Heartbeat blinking
            heartbeat_symbol = f"{GREEN}+{RESET}" if heartbeat_state else f"{YELLOW}-{RESET}"
            heartbeat_state = not heartbeat_state

            print(f"STATE : {state}")
            print(f"MODE  : {mode}")
            print(f"ARMED : {armed}")
            print(f"ALT   : {alt}")
            print(f"LAT   : {lat}")
            print(f"LON   : {lon}")
            print(f"GPS   : {gps_fix}")
            print(f"HEARTBEAT : {heartbeat_symbol}")

            # Instructions / valid events
            print(f"Instructions : {', '.join(VALID_EVENTS)}")

            print(f"{BOLD}=============================={RESET}")
            print("> ", end="", flush=True)

        time.sleep(1)

# -------------------------
# Send FSM events to Pi
# -------------------------
def send_fsm_event(event, param=None):
    msg = f"{event} {param}" if param else event
    sock.sendto(msg.encode(), PI_ADDR)

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
        print_above_input(f"{CYAN}[SENT]{RESET} {cmd}")

# -------------------------
# Main
# -------------------------
if __name__ == "__main__":
    threading.Thread(target=status_listener, daemon=True).start()  # Pi listener
    threading.Thread(target=mavproxy_listener, daemon=True).start()  # MAVProxy listener
    threading.Thread(target=dashboard_loop, daemon=True).start()  # Dashboard loop
    cli_loop()  # Command line interface loop