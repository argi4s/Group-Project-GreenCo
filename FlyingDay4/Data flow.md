   Data flow

Data flow

## FROM GUI

drone_gui.py (button click)  
→ backend.send_command("takeoff", "10")  
→ UDP packet to 192.168.1.69:9051  
→ PI_State_Machine.udp_listener()  
→ event_queue.put(("takeoff", "10"))  
→ handle_event() → drone.takeoff(10)  
→ send_status() to GCS (ACK + state update)

## FROM FSM

PI_State_Machine.send_status("State=FLIGHT Event=takeoff")  
→ UDP packet to GCS_ADDR:9051  
→ drone_backend.pi_status_listener()  
→ Sends ACK back to Pi  
→ Calls callback("on_pi_message")  
→ drone_gui updates FSM state, logs to console

## FROM CAMERA STREAM - VIDEO

camera_stream.\_run_camera()  
→ picam2.capture_array() (RGB)  
→ Convert to BGR  
→ YOLOv10 detection (every N frames)  
→ Draw bounding box  
→ proc.stdin.write(frame_bgr.tobytes())  
→ GStreamer pipeline: fdsrc → x264enc → rtph264pay → udpsink  
→ UDP to 192.168.1.17:5800  
→ drone_gui.video_receiver() (GStreamer decode)  
→ Display in tkinter label

## FROM CAMERA STREAM - VECTOR

camera_stream.\_run_camera()  
→ yolov10_detector.detect_bbox_on_bgr()  
→ Calculate dx/dy from image center  
→ msg = {"t": time.time(), "det": 1, "dx": dx, ...}  
→ vec_sock.sendto(json.dumps(msg), (VIDEO_IP, VEC_PORT))  
→ UDP to 192.168.1.17:5801  
→ drone_backend.vector_listener()  
→ Calls callback("on_detection")  
→ drone_gui draws bounding box overlay

## FROM CUBE - TELEMETRY

Orange Cube (ArduPilot)  
→ Serial to Pi (/dev/ttyAMA0)  
→ MAVProxy (on Pi) forwards to multiple outputs:  
→ udp:192.168.1.17:14400 (Mission Planner)  
→ udp:192.168.1.17:14402 (drone_backend)  
→ udp:192.168.1.69:14401 (DroneClass)  
→ drone_backend.mavlink_listener() decodes messages  
→ Calls callback("on_mav_update")  
→ drone_gui updates telemetry display