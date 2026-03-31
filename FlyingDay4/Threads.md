   Threads

Threads

## Threads and Their Responsibilities

### Raspberry Pi Threads

#### PI_State_Machine.py

| Thread | Function | Purpose |
| --- | --- | --- |
| Main Loop | `while True` | FSM main loop, processes events from queue |
| UDP Listener | `udp_listener()` | Listens on port 9051 for GCS commands |
| Status Sender | `send_status()` | Sends ACK'd status messages to GCS |
| Keepalive | `keepalive_loop()` | Periodic ping to GCS (every 5s) |
| Telemetry Display | `telemetry_display()` | Console UI updates (clears screen) |
| Discovery | `discovery_listener()` | Responds to "WHO_IS_PI?" broadcasts |

#### camera_stream.py

| Thread | Function | Purpose |
| --- | --- | --- |
| Camera Loop | `_run_camera()` | Captures frames, runs YOLO every N frames |
| GStreamer | `_setup_gstreamer()` | Subprocess for H.264 encoding/UDP streaming |

#### DroneClass.py

| Thread | Function | Purpose |
| --- | --- | --- |
| Telemetry | `_telemetry_loop()` | Polls MAVLink messages, updates internal state |

### Ground Station Threads

#### drone_backend.py

| Thread | Function | Purpose |
| --- | --- | --- |
| Pi Status | `pi_status_listener()` | Listens on 9051, ACKs messages, tracks FSM state |
| MAVLink | `mavlink_listener()` | Connects to MAVProxy on 14402, decodes telemetry |
| Vector | `vector_listener()` | Listens on 5801 for JSON detection data |

#### drone_gui.py

| Thread | Function | Purpose |
| --- | --- | --- |
| Video | `video_receiver()` | GStreamer pipeline, decodes H.264, updates frame buffer |