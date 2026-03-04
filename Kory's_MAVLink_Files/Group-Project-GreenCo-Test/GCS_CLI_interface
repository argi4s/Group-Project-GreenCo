#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import threading
import sys
import readline

PI_ADDR = ("192.168.1.50", 9050)
GCS_UDP_PORT = 9051

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", GCS_UDP_PORT))
sock.settimeout(0.2)

shutdown_event = threading.Event()

VALID_EVENTS = [
    "start_mission",
    "begin_search",
    "detect",
    "specific_search",
    "approve_approach",
    "begin_payload",
    "confirm",
    "approve_return",
    "emergency",
    "gcs_confirm"
]

def send_fsm_event(event, param=None):
    msg = f"{event} {param}" if param else event
    sock.sendto(msg.encode(), PI_ADDR)
    print_above_input(f"[SENT] {msg}")

# -------------------------
# Print messages above current input
# -------------------------
print_lock = threading.Lock()

def print_above_input(msg):
    """Print a message above the input line without disturbing typing"""
    with print_lock:
        # Save current input
        saved = readline.get_line_buffer()
        sys.stdout.write('\033[2K\r')  # Clear current line
        sys.stdout.write(f"{msg}\n")   # Print new message
        sys.stdout.write(f"> {saved}") # Rewrite input prompt
        sys.stdout.flush()

def status_listener():
    while not shutdown_event.is_set():
        try:
            data, _ = sock.recvfrom(1024)
            print_above_input(f"[FSM] {data.decode().strip()}")
        except socket.timeout:
            continue
        except OSError:
            break

def cli_loop():
    print("=== FSM GCS Command Sender (UDP) ===")
    print("Type 'exit' to quit.\n")
    print("Valid events:", ", ".join(VALID_EVENTS))

    while True:
        try:
            cmd = input("> ").strip()
        except EOFError:
            break

        if not cmd:
            continue

        if cmd.lower() == "exit":
            print("Shutting down...")
            # Send quit message to Pi
            try:
                sock.sendto("quit".encode(), PI_ADDR)
            except:
                pass
            shutdown_event.set()
            sock.close()
            break

        parts = cmd.split()
        event = parts[0]
        param = parts[1] if len(parts) > 1 else None

        if event not in VALID_EVENTS:
            print_above_input(f"[ERROR] Invalid event: {event}")
            continue

        send_fsm_event(event, param)

if __name__ == "__main__":
    listener = threading.Thread(target=status_listener, daemon=True)
    listener.start()
    cli_loop()
    listener.join()
