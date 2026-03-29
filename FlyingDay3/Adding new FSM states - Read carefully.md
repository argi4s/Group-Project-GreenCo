   Adding new FSM states - Read carefully

Adding new FSM states - Read carefully

in PI_State_Machine.py

Add a state constant

```python
STATE_NEW_MISSION = "NEW_MISSION"
```

Add handler in handle_event()

```python
if event == "new_mission":
    current_state = STATE_NEW_MISSION
    send_status(f"New mission started", type_="FSM")
    
    # Execute mission
    try:
        # Your mission code here
        for item in mission_items:
            if current_state != STATE_NEW_MISSION:
                break  # Allow interrupts
            drone.do_something()
    except RuntimeError as e:
        if "RTL active" in str(e):
            current_state = STATE_RTL
            return
        elif "Emergency active" in str(e):
            current_state = STATE_EMERGENCY
            return
    
    # Return to FLIGHT when done
    if current_state == STATE_NEW_MISSION:
        current_state = STATE_FLIGHT
```

in drone_backend.py add to VALID_EVENTS

```python
self.VALID_EVENTS = [
    # ... existing events ...
    "new_mission"  # Add your new command
]
```

in drone_gui.py add button to appropriate frame. This may be a little tricky please just message me and I'll point you to where if unsure but for temporary testing just place after any button you think it should go next to and we can fix later.

```python
ttk.Button(btn_frameX, text="NEW MISSION", 
           command=lambda: self.send_cmd("new_mission"), 
           width=10).pack(side="left", padx=2)
```