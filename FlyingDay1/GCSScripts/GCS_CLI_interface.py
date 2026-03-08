#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import threading
import sys
import readline
import os
import time

PI_ADDR = ("127.0.0.1", 9050)
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
PI_TIMEOUT = 2             # seconds before marking disconnected
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
# Dashboard loop
# -------------------------
def dashboard_loop():
    global heartbeat_state
    while not shutdown_event.is_set():
        with print_lock:
            os.system("clear")
            print(f"{BOLD}======= GCS TELEMETRY ======={RESET}")

            # Connection status
            if time.time() - last_pi_msg_time > PI_TIMEOUT:
                conn_status = f"{RED}DISCONNECTED{RESET}"
            else:
                conn_status = f"{GREEN}CONNECTED{RESET}"

            print(f"PI : {conn_status}")

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
                        try:
                            lat_val, lon_val, alt_val = pos.split(",")
                            # Check if lat_val, lon_val, alt_val are valid and convert to float
                            lat = f"{float(lat_val):.7f}" if lat_val else "--"
                            lon = f"{float(lon_val):.7f}" if lon_val else "--"
                            alt = f"{float(alt_val):.2f} m" if alt_val else "--"
                        except ValueError:
                            # Handle the case where values cannot be converted to float
                            lat = "--"
                            lon = "--"
                            alt = "--"
                    elif p.startswith("GPS="):
                        gps_fix = p.split("=")[1]

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
    threading.Thread(target=status_listener, daemon=True).start()
    threading.Thread(target=dashboard_loop, daemon=True).start()
    cli_loop()