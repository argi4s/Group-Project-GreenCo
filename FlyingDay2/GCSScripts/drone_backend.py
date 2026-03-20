#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# drone_backend.py - Handles all network communication and data

import socket
import threading
import time
import json
from pymavlink import mavutil
import socket

def get_local_ip():
    """Get the local IP address of this machine"""
    try:
        # Create a socket connection to an external IP (doesn't actually connect)
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"  # Fallback to localhost

class DroneBackend:

    
    def __init__(self, callbacks=None):
        # Configuration
        
        self.LOCAL_IP = get_local_ip()
    
        # Pi addresses (destination for sending)
        self.PI_FSM_ADDR = ("192.168.1.69", 9050)      # Pi's FSM listener
        self.PI_VEC_ADDR = ("192.168.0.249", 5801)      # Pi's vector detection listener
    
        # GCS listening ports (where we receive)
        self.GCS_FSM_PORT = 9051                        # Pi sends FSM messages here
        self.GCS_VEC_PORT = 5801                        # Pi sends detection data here
    
        # MAVProxy connection (listening for drone telemetry)
        self.MAVPROXY_IP = self.LOCAL_IP                # Our IP to receive on
        self.MAVPROXY_PORT = 14402                      # Port MAVProxy sends to
        
        # Call this when GCS starts
        pi_addr = self.discover_pi()
        if pi_addr:
            self.PI_FSM_ADDR = pi_addr    
        
        # Valid commands
        self.VALID_EVENTS = [
            "init", "takeoff", "land", "emergency",
            "rtl", "lawn", "restart", "aoi", "return"  # Added aoi and return
        ]
        
        # Callbacks
        self.callbacks = callbacks or {}
        
        # Data stores
        self.pi_state = "--"
        self.last_pi_msg_time = 0
        self.PI_TIMEOUT = 2
        
        self.mav_telemetry = {
            "mode": "--", "armed": "--", "lat": "--", "lon": "--",
            "alt": "--", "gps_fix": "--", "heading": "--",
            "ground_speed": "--", "last_update": 0
        }
        self.MAV_TIMEOUT = 3
        
        self.detection_data = {
            "detected": False, "score": 0, "dx": 0, "dy": 0,
            "dxn": 0, "dyn": 0, "bbox": [0, 0, 0, 0], "last_update": 0
        }
        
        # Sockets
        self.udp_sock = None
        self.vec_sock = None
        self.shutdown_event = threading.Event()
        
        # Start networking
        self.init_sockets()
        self.start_threads()
    
    def discover_pi(self, timeout=3):
        """Broadcast discovery request and wait for Pi responses"""
        # Create socket for discovery
        disc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        disc_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        disc_sock.settimeout(timeout)
    
        # Send discovery request
        request = "WHO_IS_PI?"
        disc_sock.sendto(request.encode(), ('<broadcast>', 9052))
    
        # Listen for responses
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                data, addr = disc_sock.recvfrom(1024)
                response = data.decode()
                if response.startswith("I_AM_PI:"):
                    _, pi_ip, pi_port = response.split(':')
                    print(f"Found Pi at {pi_ip}:{pi_port}")
                    return (pi_ip, int(pi_port))
            except socket.timeout:
                continue
        return None


    
    
    def init_sockets(self):
        """Initialize UDP sockets"""
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind(("", self.GCS_FSM_PORT))
        self.udp_sock.settimeout(0.2)
        
        self.vec_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.vec_sock.bind(("", self.GCS_VEC_PORT))
        self.vec_sock.settimeout(0.2)
    
    def start_threads(self):
        """Start all background threads"""
        threading.Thread(target=self.pi_status_listener, daemon=True).start()
        
        self.mav_conn = mavutil.mavlink_connection(f"udp:{self.MAVPROXY_IP}:{self.MAVPROXY_PORT}")
        threading.Thread(target=self.mavlink_listener, daemon=True).start()
        
        threading.Thread(target=self.vector_listener, daemon=True).start()
    
    def pi_status_listener(self):
        """Listen for Pi FSM status messages"""
        seq_received = set()
    
        while not self.shutdown_event.is_set():
            try:
                data, addr = self.udp_sock.recvfrom(1024)
                text = data.decode().strip()
            
                if text.startswith("SEQ="):
                    parts = text.split(" ", 2)
                    seq = int(parts[0].split("=")[1])
                    msg_type = parts[1].split("=")[1]
                    content = parts[2].split("=", 1)[1]
                
                    # ACK back to Pi
                    ack = f"ACK SEQ={seq} TYPE={msg_type}"
                    self.udp_sock.sendto(ack.encode(), addr)
                
                    if seq in seq_received:
                        continue
                    seq_received.add(seq)
                
                    # 🔧 Extract state and print only on changes
                    if "State=" in content:
                        for part in content.split():
                            if part.startswith("State="):
                                new_state = part.split("=")[1]
                                # Only print if state actually changed
                                if new_state != self.pi_state:
                                    old_state = self.pi_state
                                    self.pi_state = new_state
                                    print(f"[{time.strftime('%H:%M:%S')}] State changed: {old_state} → {new_state}")
                                break
                
                    # Notify GUI - filter out keepalives
                    # Notify GUI - filter out keepalives
                    if msg_type != "KEEPALIVE" and "___KEEPALIVE___" not in content:
                        if "on_pi_message" in self.callbacks:
                            self.callbacks["on_pi_message"](msg_type, content)
                
                    # Still update the last message time for connection status
                    self.last_pi_msg_time = time.time()
                
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Pi listener error: {e}")
                time.sleep(0.1)
    
    def mavlink_listener(self):
        """Listen for MAVLink telemetry"""
        while not self.shutdown_event.is_set():
            try:
                msg = self.mav_conn.recv_match(blocking=False, timeout=0.1)
                
                if msg is None:
                    time.sleep(0.05)
                    continue
                
                msg_type = msg.get_type()
                
                if msg_type == "HEARTBEAT":
                    # Get the raw mode value (0x00000004 = 4)
                    mode_val = msg.custom_mode
    
                    # Simple lookup table for common ArduPilot modes
                    mode_names = {
                        0: "STABILIZE",
                        1: "ACRO", 
                        2: "ALT_HOLD",
                        3: "AUTO",
                        4: "GUIDED",      # 0x00000004 = GUIDED
                        5: "LOITER",
                        6: "RTL",
                        7: "CIRCLE",
                        9: "LAND",
                        16: "POSHOLD"
                    }
    
                    # Convert to readable name, or show raw value if unknown
                    mode_str = mode_names.get(mode_val, f"Mode({mode_val})")
    
                    self.mav_telemetry["mode"] = mode_str
                    self.mav_telemetry["armed"] = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                    self.mav_telemetry["last_update"] = time.time()
                    
                elif msg_type == "GLOBAL_POSITION_INT":
                    self.mav_telemetry["lat"] = msg.lat / 1e7
                    self.mav_telemetry["lon"] = msg.lon / 1e7
                    self.mav_telemetry["alt"] = msg.relative_alt / 1000.0
                    self.mav_telemetry["heading"] = msg.hdg / 100.0 if msg.hdg != 65535 else 0
                    self.mav_telemetry["last_update"] = time.time()
                    
                elif msg_type == "GPS_RAW_INT":
                    self.mav_telemetry["gps_fix"] = msg.fix_type
                    self.mav_telemetry["last_update"] = time.time()
                    
                elif msg_type == "VFR_HUD":
                    self.mav_telemetry["ground_speed"] = msg.groundspeed
                    self.mav_telemetry["last_update"] = time.time()
                    
                # Notify GUI of update
                if "on_mav_update" in self.callbacks:
                    self.callbacks["on_mav_update"](self.mav_telemetry)
                    
            except Exception as e:
                print(f"MAVLink listener error: {e}")
                time.sleep(0.1)
    
    def vector_listener(self):
        """Listen for detection vector data"""
        while not self.shutdown_event.is_set():
            try:
                data, addr = self.vec_sock.recvfrom(1024)
                self.detection_data = json.loads(data.decode())
                self.detection_data["last_update"] = time.time()
                
                if "on_detection" in self.callbacks:
                    self.callbacks["on_detection"](self.detection_data)
                    
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Vector listener error: {e}")
                time.sleep(0.1)
    
    def send_command(self, event, param=None):
        """Send command to Pi FSM"""
        msg = f"{event} {param}" if param else event
        self.udp_sock.sendto(msg.encode(), self.PI_FSM_ADDR)
        return msg
    
    def get_connection_status(self):
        """Get connection status for all links"""
        return {
            "pi_fsm": time.time() - self.last_pi_msg_time < self.PI_TIMEOUT,
            "mavlink": time.time() - self.mav_telemetry["last_update"] < self.MAV_TIMEOUT,
            "detection": time.time() - self.detection_data["last_update"] < 1.0
        }
    
    def shutdown(self):
        """Clean shutdown"""
        self.shutdown_event.set()
        if self.udp_sock:
            self.udp_sock.close()
        if self.vec_sock:
            self.vec_sock.close()