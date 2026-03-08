#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import threading
import sys
import readline

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
    "emergency"
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
    with print_lock:
        saved = readline.get_line_buffer()
        sys.stdout.write('\033[2K\r')
        sys.stdout.write(f"{msg}\n")
        sys.stdout.write(f"> {saved}")
        sys.stdout.flush()

# -------------------------
# Listen for status from Pi
# -------------------------
def status_listener():
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

                ack = f"ACK SEQ={seq} TYPE={msg_type}"
                sock.sendto(ack.encode(), addr)

                if seq in seq_received:
                    continue

                seq_received.add(seq)

                print_above_input(f"[{msg_type}] {content}")

            else:
                print_above_input(f"[UNKNOWN] {text}")

        except socket.timeout:
            continue

# -------------------------
# CLI
# -------------------------
def cli_loop():

    print("=== Drone CLI (UDP) ===")
    print("Commands: init, takeoff, land, emergency")
    print("Type 'exit' to quit.\n")

    while True:

        try:
            cmd = input("> ").strip()
        except EOFError:
            break

        if not cmd:
            continue

        if cmd.lower() == "exit":

            print("Shutting down...")

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
            print_above_input(f"[ERROR] Invalid command: {event}")
            continue

        send_fsm_event(event, param)

# -------------------------
# Main
# -------------------------
if __name__ == "__main__":

    listener = threading.Thread(target=status_listener, daemon=True)
    listener.start()

    cli_loop()

    listener.join()
