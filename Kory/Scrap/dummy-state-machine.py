#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 11 16:19:45 2026

@author: kory campbell
"""

# ============================================================
# Interactive / Sensor-Driven Mission FSM with Emergency
# ============================================================

STATE_INIT = "INIT"
STATE_SEARCH = "SEARCH"
STATE_SPECIFIC_SEARCH = "SPECIFIC_SEARCH"
STATE_DETECTION = "DETECTION"
STATE_APPROACH = "APPROACH"
STATE_PAYLOAD_DEPLOY = "PAYLOAD_DEPLOY"
STATE_RETURN = "RETURN"
STATE_EMERGENCY = "EMERGENCY"

current_state = STATE_INIT
payload_step = 0       # tracks multi-step payload deployment
detection_param = None # stores user-specified parameter after detection
return_approved = False

# ----------------------------
# Transition Table
# ----------------------------
TRANSITIONS = {
    STATE_INIT: {"start_search": STATE_SEARCH},
    STATE_SEARCH: {"detect": STATE_DETECTION, "specific_search": STATE_SPECIFIC_SEARCH},
    STATE_SPECIFIC_SEARCH: {"detect": STATE_DETECTION},
    STATE_DETECTION: {"approve_approach": STATE_APPROACH},
    STATE_APPROACH: {"begin_payload": STATE_PAYLOAD_DEPLOY},
    STATE_PAYLOAD_DEPLOY: {"payload_done": STATE_RETURN},
    STATE_RETURN: {"approve_return": STATE_RETURN},
}

# ----------------------------
# Behavior Handler
# ----------------------------
def on_event(state, event, param=None):
    global payload_step, detection_param, return_approved

    if state == STATE_INIT:
        print("INIT: Waiting for user to start search")

    elif state == STATE_SEARCH:
        if event == "detect":
            print("SEARCH: Sensor detected object — HOLDING position")
        elif event == "specific_search":
            print("SEARCH: Entering specific search region")

    elif state == STATE_SPECIFIC_SEARCH:
        if event == "detect":
            print("SPECIFIC SEARCH: Sensor detected object — HOLDING position")

    elif state == STATE_DETECTION:
        print(f"DETECTION: Target locked — waiting for user approval")
        if param:
            detection_param = param.upper()
            print(f"DETECTION PARAMETER SET: {detection_param}")

    elif state == STATE_APPROACH:
        print("APPROACH: Hovering near target — waiting for user to start payload deployment")

    elif state == STATE_PAYLOAD_DEPLOY:
        msgs = [
            "Confirm payload deployment",
            "Detach first loop",
            "Detach payload",
            "Take-off"
        ]
        if payload_step < len(msgs):
            print(f"PAYLOAD STEP {payload_step+1}: {msgs[payload_step]}")

    elif state == STATE_RETURN:
        if not return_approved:
            print("RETURN: Awaiting user approval to return")
        else:
            print("RETURN: Returning home")

    elif state == STATE_EMERGENCY:
        print("EMERGENCY: Interrupt triggered! Exiting mission immediately")

# ----------------------------
# FSM Event Router
# ----------------------------
def handle_event(event, param=None):
    global current_state, payload_step, detection_param, return_approved

    if event == "emergency":
        current_state = STATE_EMERGENCY
        on_event(current_state, event)
        return

    # Run behavior first
    on_event(current_state, event, param)

    # Special payload handling
    if current_state == STATE_PAYLOAD_DEPLOY:
        if event == "confirm":
            payload_step += 1
            if payload_step >= 4:
                print("PAYLOAD: Deployment complete")
                current_state = STATE_RETURN
            return

    # Detection with parameter
    if current_state == STATE_DETECTION and event.startswith("approve_approach"):
        current_state = STATE_APPROACH
        return

    # RETURN requires approval
    if current_state == STATE_RETURN:
        if event == "approve_return":
            return_approved = True
        return

    # Normal FSM transitions
    allowed = TRANSITIONS.get(current_state, {})
    if event in allowed:
        next_state = allowed[event]
        print(f"[FSM] {current_state} -> {next_state}")
        current_state = next_state

# ----------------------------
# CLI / Sensor Loop
# ----------------------------
def print_prompt():
    print("\nCurrent State:", current_state)
    print("Commands:")
    if current_state == STATE_INIT:
        print(" - start_search")
    elif current_state == STATE_SEARCH:
        print(" - specific_search")
        print("Sensor may automatically detect object (simulate with 'y')")
    elif current_state == STATE_SPECIFIC_SEARCH:
        print("Sensor may automatically detect object (simulate with 'y')")
    elif current_state == STATE_DETECTION:
        print(" - approve_approach <N/E/S/W>")
    elif current_state == STATE_APPROACH:
        print(" - begin_payload")
    elif current_state == STATE_PAYLOAD_DEPLOY:
        print(" - confirm (x4)")
    elif current_state == STATE_RETURN:
        print(" - approve_return")
    print(" - emergency (anywhere)")

def cli_loop():
    global current_state

    print("=== Mission FSM CLI ===")

    while True:
        if current_state == STATE_EMERGENCY:
            print("Mission terminated due to emergency!")
            break

        print_prompt()

        # simulate sensor trigger
        if current_state in [STATE_SEARCH, STATE_SPECIFIC_SEARCH]:
            sensor_trigger = input("Sensor event? (y/n) > ").strip().lower()
            if sensor_trigger == "y":
                handle_event("detect")
                continue

        cmd_input = input("> ").strip()

        if not cmd_input:
            continue

        if cmd_input == "exit":
            print("Exiting.")
            break

        # handle approve_approach with parameter
        if cmd_input.startswith("approve_approach"):
            parts = cmd_input.split()
            param = parts[1] if len(parts) > 1 else None
            handle_event(cmd_input, param)
            continue

        handle_event(cmd_input)

if __name__ == "__main__":
    cli_loop()
