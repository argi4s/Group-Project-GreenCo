   Adding new functions to DroneClass.py

Adding new functions to DroneClass.py

Template used for creating new commands

```python
def new_command(self, param1, param2=None):
    """Description of what this command does"""
    # Always check interrupts first!
    self._check_interrupts()
    
    self.busy = True
    
    try:
        # Send MAVLink command
        self.mav.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_YOUR_COMMAND,  # Find correct enum
            0,  # confirmation
            param1, param2, 0, 0, 0, 0, 0  # Fill params as needed
        )
        
        # If blocking, wait for completion
        if blocking:
            self._wait_for_condition()
            
    except Exception as e:
        # Handle errors appropriately
        print(f"[ERROR] new_command failed: {e}")
        raise
    finally:
        self.busy = False


```

Adding new properties that are called from mavlink

```python
@property
def new_telemetry_value(self):
    with self._lock:
        return self._new_value

# In _telemetry_loop:
elif msg.get_type() == "NEW_MESSAGE":
    self._new_value = msg.field_name
```