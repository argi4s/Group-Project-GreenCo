#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# drone_gui.py - Tkinter GUI with embedded video and ACK status bar

import tkinter as tk
from tkinter import ttk, scrolledtext
import threading
import cv2
from PIL import Image, ImageTk
import subprocess
import numpy as np
import socket
import struct
import time
from datetime import datetime
from drone_backend import DroneBackend

class DroneGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Drone Ground Control Station")
        self.root.geometry("1400x800")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Video stream config
        self.VIDEO_IP = "192.168.1.17"
        self.VIDEO_PORT = 5800
        self.video_running = False
        self.video_thread = None
        self.current_frame = None
        self.recording = False
        self.video_writer = None
        
        self._last_state_msg = ""
        
        # Initialize backend with callbacks
        self.backend = DroneBackend(callbacks={
            "on_pi_message": self.on_pi_message,
            "on_mav_update": self.on_mav_update,
            "on_detection": self.on_detection
        })
        
        # Create GUI
        self.setup_ui()
        
        # Start periodic updates
        self.update_connection_status()
        self.update_video_display()
        
    def setup_ui(self):
        """Create all GUI elements"""
        # Configure grid - add bottom row for status bar
        self.root.grid_rowconfigure(0, weight=1)  # Main content
        self.root.grid_rowconfigure(1, weight=0)  # Status bar (fixed height)
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=1)
        self.root.grid_columnconfigure(2, weight=2)
        
        # Create panels
        self.create_control_panel()   # Left
        self.create_telemetry_panel() # Center
        self.create_video_panel()     # Right
        
        # Add status bar at bottom
        self.create_status_bar()
        
    def create_control_panel(self):
        """Left panel - Controls and command buttons"""
        frame = ttk.LabelFrame(self.root, text="Drone Controls", padding=10)
        frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        
        # Connection status
        ttk.Label(frame, text="Connection Status:", font=("Arial", 10, "bold")).grid(row=0, column=0, sticky="w", pady=5)
        self.pi_status_var = tk.StringVar(value="DISCONNECTED")
        self.pi_status_label = ttk.Label(frame, textvariable=self.pi_status_var, foreground="red")
        self.pi_status_label.grid(row=0, column=1, sticky="w", pady=5)
        
        # MAVLink status
        ttk.Label(frame, text="MAVLink Status:", font=("Arial", 10, "bold")).grid(row=1, column=0, sticky="w", pady=5)
        self.mav_status_var = tk.StringVar(value="STALE")
        self.mav_status_label = ttk.Label(frame, textvariable=self.mav_status_var, foreground="red")
        self.mav_status_label.grid(row=1, column=1, sticky="w", pady=5)
        
        # Current FSM State
        ttk.Label(frame, text="FSM State:", font=("Arial", 10, "bold")).grid(row=2, column=0, sticky="w", pady=5)
        self.fsm_state_var = tk.StringVar(value="--")
        ttk.Label(frame, textvariable=self.fsm_state_var, font=("Arial", 12, "bold")).grid(row=2, column=1, sticky="w", pady=5)
        
        # Separator
        ttk.Separator(frame, orient='horizontal').grid(row=3, column=0, columnspan=2, sticky="ew", pady=10)
        
        # Command buttons
        ttk.Label(frame, text="Commands:", font=("Arial", 10, "bold")).grid(row=4, column=0, columnspan=2, pady=5)
        
        # Button rows
        row = 5
        btn_frame1 = ttk.Frame(frame)
        btn_frame1.grid(row=row, column=0, columnspan=2, pady=2)
        
        ttk.Button(btn_frame1, text="INIT", command=lambda: self.send_cmd("init"), width=10).pack(side="left", padx=2)
        ttk.Button(btn_frame1, text="TAKEOFF", command=self.takeoff_dialog, width=10).pack(side="left", padx=2)
        ttk.Button(btn_frame1, text="LAND", command=lambda: self.send_cmd("land"), width=10).pack(side="left", padx=2)
        
        row += 1
        btn_frame2 = ttk.Frame(frame)
        btn_frame2.grid(row=row, column=0, columnspan=2, pady=2)
        
        ttk.Button(btn_frame2, text="RTL", command=lambda: self.send_cmd("rtl"), width=10).pack(side="left", padx=2)
        ttk.Button(btn_frame2, text="LAWN", command=lambda: self.send_cmd("lawn"), width=10).pack(side="left", padx=2)
        ttk.Button(btn_frame2, text="AOI", command=lambda: self.send_cmd("aoi"), width=10).pack(side="left", padx=2)      
        ttk.Button(btn_frame2, text="RETURN", command=lambda: self.send_cmd("return"), width=10).pack(side="left", padx=2) 
        
        ttk.Button(btn_frame2, text="RESTART", command=lambda: self.send_cmd("restart"), width=10).pack(side="left", padx=2)
        
        row += 1
        btn_frame3 = ttk.Frame(frame)
        btn_frame3.grid(row=row, column=0, columnspan=2, pady=2)
        
        ttk.Button(btn_frame3, text="EMERGENCY", command=lambda: self.send_cmd("emergency"), width=10).pack(side="left", padx=2)
        
        # Speed Control
        row += 1
        speed_frame = ttk.LabelFrame(frame, text="Speed Control", padding=5)
        speed_frame.grid(row=row, column=0, columnspan=2, sticky="ew", pady=5)
        
        ttk.Label(speed_frame, text="Speed (m/s):").grid(row=0, column=0, padx=2, pady=2)
        self.speed_entry = ttk.Entry(speed_frame, width=10)
        self.speed_entry.grid(row=0, column=1, padx=2, pady=2)
        self.speed_entry.insert(0, "5.0")
        
        ttk.Button(speed_frame, text="Set Speed", command=self.set_speed, width=10).grid(row=0, column=2, padx=2, pady=2)
        
        # Heading Control
        row += 1
        heading_frame = ttk.LabelFrame(frame, text="Heading Control", padding=5)
        heading_frame.grid(row=row, column=0, columnspan=2, sticky="ew", pady=5)
        
        ttk.Label(heading_frame, text="Heading (°):").grid(row=0, column=0, padx=2, pady=2)
        self.heading_entry = ttk.Entry(heading_frame, width=10)
        self.heading_entry.grid(row=0, column=1, padx=2, pady=2)
        self.heading_entry.insert(0, "0")
        
        ttk.Button(heading_frame, text="Set Heading", command=self.set_heading, width=10).grid(row=0, column=2, padx=2, pady=2)
        
        # Default heading button (auto-rotate)
        ttk.Button(heading_frame, text="Default (Auto Rotate)", 
                  command=self.set_default_heading, width=20).grid(row=1, column=0, columnspan=3, pady=2)
        
        ttk.Label(heading_frame, text="(0=N, 90=E, -1 = auto rotate)", font=("Arial", 8)).grid(row=2, column=0, columnspan=3)
        
        # Command log
        row += 1
        ttk.Label(frame, text="Command Log:", font=("Arial", 10, "bold")).grid(row=row, column=0, columnspan=2, pady=5)
        
        self.log_text = scrolledtext.ScrolledText(frame, height=8, width=35)
        self.log_text.grid(row=row+1, column=0, columnspan=2, pady=5)
        
    def create_telemetry_panel(self):
        """Center panel - Telemetry data"""
        frame = ttk.LabelFrame(self.root, text="Drone Telemetry", padding=10)
        frame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
    
        # Mode and Armed - FIXED WIDTH
        ttk.Label(frame, text="Mode:", font=("Arial", 10, "bold")).grid(row=0, column=0, sticky="w", pady=2)
        self.mode_var = tk.StringVar(value="--")
        # Add width=15 to keep it constant
        ttk.Label(frame, textvariable=self.mode_var, font=("Arial", 12), width=15, anchor="w").grid(row=0, column=1, sticky="w", pady=2)
    
        ttk.Label(frame, text="Armed:", font=("Arial", 10, "bold")).grid(row=1, column=0, sticky="w", pady=2)
        self.armed_var = tk.StringVar(value="--")
        # Add width=15 here too for consistency
        ttk.Label(frame, textvariable=self.armed_var, font=("Arial", 12), width=15, anchor="w").grid(row=1, column=1, sticky="w", pady=2)
    
        # Position
        ttk.Label(frame, text="Latitude:", font=("Arial", 10, "bold")).grid(row=2, column=0, sticky="w", pady=2)
        self.lat_var = tk.StringVar(value="--")
        ttk.Label(frame, textvariable=self.lat_var, width=20, anchor="w").grid(row=2, column=1, sticky="w", pady=2)
    
        ttk.Label(frame, text="Longitude:", font=("Arial", 10, "bold")).grid(row=3, column=0, sticky="w", pady=2)
        self.lon_var = tk.StringVar(value="--")
        ttk.Label(frame, textvariable=self.lon_var, width=20, anchor="w").grid(row=3, column=1, sticky="w", pady=2)
        
        ttk.Label(frame, text="Altitude:", font=("Arial", 10, "bold")).grid(row=4, column=0, sticky="w", pady=2)
        self.alt_var = tk.StringVar(value="--")
        ttk.Label(frame, textvariable=self.alt_var).grid(row=4, column=1, sticky="w", pady=2)
        
        # GPS and Heading
        ttk.Label(frame, text="GPS Fix:", font=("Arial", 10, "bold")).grid(row=5, column=0, sticky="w", pady=2)
        self.gps_var = tk.StringVar(value="--")
        ttk.Label(frame, textvariable=self.gps_var).grid(row=5, column=1, sticky="w", pady=2)
        
        ttk.Label(frame, text="Heading:", font=("Arial", 10, "bold")).grid(row=6, column=0, sticky="w", pady=2)
        self.heading_var = tk.StringVar(value="--")
        ttk.Label(frame, textvariable=self.heading_var).grid(row=6, column=1, sticky="w", pady=2)
        
        ttk.Label(frame, text="Ground Speed:", font=("Arial", 10, "bold")).grid(row=7, column=0, sticky="w", pady=2)
        self.speed_var = tk.StringVar(value="--")
        ttk.Label(frame, textvariable=self.speed_var).grid(row=7, column=1, sticky="w", pady=2)
        
        # Separator
        ttk.Separator(frame, orient='horizontal').grid(row=8, column=0, columnspan=2, sticky="ew", pady=10)
        
        # Detection info
        ttk.Label(frame, text="Object Detection", font=("Arial", 12, "bold")).grid(row=9, column=0, columnspan=2, pady=5)
        
        self.detection_var = tk.StringVar(value="No detection")
        ttk.Label(frame, textvariable=self.detection_var, foreground="blue").grid(row=10, column=0, columnspan=2, pady=2)
        
        self.detection_score_var = tk.StringVar(value="")
        ttk.Label(frame, textvariable=self.detection_score_var).grid(row=11, column=0, columnspan=2, pady=2)
        
    def create_video_panel(self):
        """Right panel - Embedded video with recording"""
        frame = ttk.LabelFrame(self.root, text="Video Feed", padding=10)
        frame.grid(row=0, column=2, sticky="nsew", padx=5, pady=5)
        
        # Video controls
        controls = ttk.Frame(frame)
        controls.pack(fill="x", pady=5)
        
        ttk.Label(controls, text="Stream:").pack(side="left", padx=5)
        
        self.video_ip_var = tk.StringVar(value=self.VIDEO_IP)
        ttk.Entry(controls, textvariable=self.video_ip_var, width=15).pack(side="left", padx=2)
        
        ttk.Label(controls, text=":").pack(side="left")
        
        self.video_port_var = tk.StringVar(value=str(self.VIDEO_PORT))
        ttk.Entry(controls, textvariable=self.video_port_var, width=6).pack(side="left", padx=2)
        
        # Video control buttons frame
        video_buttons = ttk.Frame(controls)
        video_buttons.pack(side="left", padx=5)
        
        self.video_start_btn = ttk.Button(video_buttons, text="▶ Start", command=self.start_video, width=8)
        self.video_start_btn.pack(side="left", padx=2)
        
        self.video_record_btn = ttk.Button(video_buttons, text="● Record", command=self.toggle_recording, width=8, state="disabled")
        self.video_record_btn.pack(side="left", padx=2)
        
        self.video_stop_btn = ttk.Button(video_buttons, text="■ Stop", command=self.stop_video, width=8, state="disabled")
        self.video_stop_btn.pack(side="left", padx=2)
        
        # Video display label
        self.video_label = ttk.Label(frame, relief="sunken")
        self.video_label.pack(expand=True, fill="both", pady=5)
        
        # Status
        self.video_status_var = tk.StringVar(value="Ready")
        ttk.Label(frame, textvariable=self.video_status_var).pack(pady=2)
        
        # Recording indicator
        self.recording_var = tk.StringVar(value="")
        ttk.Label(frame, textvariable=self.recording_var, foreground="red").pack(pady=2)
        
    def create_status_bar(self):
        """Create a status bar at the bottom for ACKs"""
        status_frame = ttk.Frame(self.root, relief="sunken", padding=2)
        status_frame.grid(row=1, column=0, columnspan=3, sticky="ew", padx=5, pady=2)
        
        # Label for ACK messages
        ttk.Label(status_frame, text="ACK:", font=("Arial", 9, "bold")).pack(side="left", padx=5)
        
        self.ack_var = tk.StringVar(value="Ready")
        ack_label = ttk.Label(status_frame, textvariable=self.ack_var, font=("Arial", 9))
        ack_label.pack(side="left", padx=5)
        
        # Clear button
        ttk.Button(status_frame, text="Clear", command=lambda: self.ack_var.set("Ready"), width=5).pack(side="left", padx=5)
        
        # Timestamp
        self.ack_time_var = tk.StringVar(value="")
        ttk.Label(status_frame, textvariable=self.ack_time_var, font=("Arial", 8), foreground="gray").pack(side="right", padx=5)
        
    def start_video(self):
        """Start receiving video stream"""
        if self.video_running:
            return
            
        self.video_running = True
        self.video_thread = threading.Thread(target=self.video_receiver, daemon=True)
        self.video_thread.start()
        self.video_status_var.set(f"Connecting to {self.video_ip_var.get()}:{self.video_port_var.get()}...")
        
        # Update button states
        self.video_start_btn.config(state="disabled")
        self.video_record_btn.config(state="normal")
        self.video_stop_btn.config(state="normal")
    
    def stop_video(self):
        """Stop video stream"""
        self.video_running = False
        if self.recording:
            self.toggle_recording()  # Stop recording if active
        
        if self.video_thread:
            self.video_thread.join(timeout=1.0)
        self.video_label.config(image='')
        self.video_status_var.set("Stopped")
        self.recording_var.set("")
        
        # Update button states
        self.video_start_btn.config(state="normal")
        self.video_record_btn.config(state="disabled", text="● Record")
        self.video_stop_btn.config(state="disabled")
    
    def toggle_recording(self):
        """Start or stop recording with datetime filename"""
        if not self.recording:
            # Start recording
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"drone_recording_{timestamp}.avi"
            
            # Initialize video writer
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.video_writer = cv2.VideoWriter(filename, fourcc, 20.0, (640, 480))
            self.recording = True
            self.video_record_btn.config(text="◼ Recording")
            self.recording_var.set(f"● RECORDING {timestamp}")
            self.log_message(f"🎥 Started recording: {filename}")
        else:
            # Stop recording
            self.recording = False
            if self.video_writer:
                self.video_writer.release()
                self.video_writer = None
            self.video_record_btn.config(text="● Record")
            self.recording_var.set("")
            self.log_message("🎥 Recording stopped")
    
    def video_receiver(self):
        """Receive H264 stream and decode with GStreamer"""
        import gi
        gi.require_version('Gst', '1.0')
        from gi.repository import Gst, GLib
    
        Gst.init(None)
    
        # Create GStreamer pipeline
        pipeline_str = (
            f"udpsrc port={self.video_port_var.get()} ! "
            "application/x-rtp, media=video, encoding-name=H264 ! "
            "rtph264depay ! "
            "h264parse ! "
            "avdec_h264 ! "
            "videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink name=sink emit-signals=True max-buffers=1 drop=True"
        )
    
        pipeline = Gst.parse_launch(pipeline_str)
        sink = pipeline.get_by_name('sink')
    
        def new_sample(sink):
            sample = sink.emit('pull-sample')
            buf = sample.get_buffer()
            caps = sample.get_caps()
        
            # Get frame info
            structure = caps.get_structure(0)
            width = structure.get_value('width')
            height = structure.get_value('height')
        
            # Extract frame data
            success, map_info = buf.map(Gst.MapFlags.READ)
            if success:
                frame_data = map_info.data
                # Convert to numpy array
                frame = np.frombuffer(frame_data, dtype=np.uint8).reshape((height, width, 3))
                self.current_frame = frame
            
                # Save frame if recording
                if self.recording and self.video_writer:
                    self.video_writer.write(frame)
            
                buf.unmap(map_info)
        
            return Gst.FlowReturn.OK
    
        sink.connect('new-sample', new_sample)
    
        # Start pipeline
        pipeline.set_state(Gst.State.PLAYING)
    
        # Create a GLib main loop
        loop = GLib.MainLoop()
    
        # Define a function to check if we should quit
        def check_quit():
            if not self.video_running:
                loop.quit()
                return False
            return True
    
        # Add a timeout to check quit condition
        GLib.timeout_add(100, check_quit)  # Check every 100ms
    
        # Start the main loop
        try:
            loop.run()
        except:
            pass
        finally:
            # Cleanup
            pipeline.set_state(Gst.State.NULL)
        
        def new_sample(sink):
            sample = sink.emit('pull-sample')
            buf = sample.get_buffer()
            caps = sample.get_caps()
            
            # Get frame info
            structure = caps.get_structure(0)
            width = structure.get_value('width')
            height = structure.get_value('height')
            
            # Extract frame data
            success, map_info = buf.map(Gst.MapFlags.READ)
            if success:
                frame_data = map_info.data
                # Convert to numpy array
                frame = np.frombuffer(frame_data, dtype=np.uint8).reshape((height, width, 3))
                self.current_frame = frame
                
                # Save frame if recording
                if self.recording and self.video_writer:
                    self.video_writer.write(frame)
                
                buf.unmap(map_info)
            
            return Gst.FlowReturn.OK
        
        sink.connect('new-sample', new_sample)
        
        # Start pipeline
        pipeline.set_state(Gst.State.PLAYING)
        
        # Main loop
        loop = GLib.MainLoop()
        context = loop.get_context()
        
        while self.video_running:
            context.iteration(False)
            time.sleep(0.01)
        
        # Cleanup
        pipeline.set_state(Gst.State.NULL)
    
    def update_video_display(self):
        """Update the video label with current frame"""
        if self.current_frame is not None:
            # Resize frame to fit label
            frame = self.current_frame.copy()
            h, w = frame.shape[:2]
            
            # Calculate new size to fit label while maintaining aspect ratio
            label_width = self.video_label.winfo_width()
            label_height = self.video_label.winfo_height()
            
            if label_width > 1 and label_height > 1:
                scale = min(label_width/w, label_height/h)
                new_w = int(w * scale)
                new_h = int(h * scale)
                
                if new_w > 0 and new_h > 0:
                    frame = cv2.resize(frame, (new_w, new_h))
                    
                    # Convert to PhotoImage
                    image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    image = Image.fromarray(image)
                    photo = ImageTk.PhotoImage(image)
                    
                    self.video_label.config(image=photo)
                    self.video_label.image = photo  # Keep a reference
        
        # Schedule next update
        self.root.after(30, self.update_video_display)
    
    # Callback methods
    def on_pi_message(self, msg_type, content):
        """Handle messages from Pi FSM"""
        self.fsm_state_var.set(self.backend.pi_state)
        
        # Check for ACK messages - show in status bar
        if "ack" in content.lower() or "ACK" in msg_type or "Event=ack" in content:
            self.ack_var.set(f"✓ {content[:40]}")
            self.ack_time_var.set(time.strftime("%H:%M:%S"))
            return  # Don't log to main window
        
        # Handle keepalives
        if msg_type == "KEEPALIVE":
            return  # Completely ignore
        
        # Check for speed responses
        if "speed" in content.lower() and "set to" in content.lower():
            self.log_message(f"⚡ {content}")
            return
            
        # Check for heading responses
        if "heading" in content.lower() and "set to" in content.lower():
            self.log_message(f"🧭 {content}")
            return
        
        # Log other messages to main window
        if msg_type == "FSM":
            if "State=" in content and "Event=" in content:
                # State transitions
                self.log_message(f"🔄 {content}")
            elif "WP" in content:
                # Waypoint updates
                self.log_message(f"🎯 {content}")
            elif content and len(content.strip()) > 10:
                # Other FSM messages
                self.log_message(f"Pi: {msg_type} - {content[:50]}")
        else:
            # Non-FSM messages
            self.log_message(f"Pi: {msg_type} - {content[:50]}")
    
    def on_mav_update(self, telemetry):
        """Handle MAVLink telemetry updates"""
        self.mode_var.set(str(telemetry["mode"]))
        self.armed_var.set(str(telemetry["armed"]))
        
        if telemetry["lat"] != "--":
            self.lat_var.set(f"{telemetry['lat']:.7f}")
            self.lon_var.set(f"{telemetry['lon']:.7f}")
            self.alt_var.set(f"{telemetry['alt']:.2f} m")
        
        self.gps_var.set(str(telemetry["gps_fix"]))
        
        if telemetry["heading"] != "--":
            self.heading_var.set(f"{telemetry['heading']:.1f}°")
        
        if telemetry["ground_speed"] != "--":
            self.speed_var.set(f"{telemetry['ground_speed']:.1f} m/s")
    
    def on_detection(self, detection):
        """Handle detection data"""
        if detection.get("det", 0) == 1:
            self.detection_var.set(f"Person detected!")
            self.detection_score_var.set(f"Confidence: {detection['score']:.2f}")
            
            # Draw bounding box on video frame
            if self.current_frame is not None and "bbox" in detection:
                x1, y1, x2, y2 = detection["bbox"]
                cv2.rectangle(self.current_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(self.current_frame, f"{detection['score']:.2f}", 
                           (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        else:
            self.detection_var.set("No detection")
            self.detection_score_var.set("")
    
    def update_connection_status(self):
        """Update connection status display"""
        status = self.backend.get_connection_status()
        
        # Update colors
        if status["pi_fsm"]:
            self.pi_status_var.set("CONNECTED")
            self.pi_status_label.config(foreground="green")
        else:
            self.pi_status_var.set("DISCONNECTED")
            self.pi_status_label.config(foreground="red")
        
        if status["mavlink"]:
            self.mav_status_var.set("ACTIVE")
            self.mav_status_label.config(foreground="green")
        else:
            self.mav_status_var.set("STALE")
            self.mav_status_label.config(foreground="red")
        
        # Schedule next update
        self.root.after(1000, self.update_connection_status)
    
    # Command methods
    def send_cmd(self, event, param=None):
        """Send command to backend"""
        msg = self.backend.send_command(event, param)
        self.log_message(f"Sent: {msg}")
    
    def set_speed(self):
        """Set speed from entry field"""
        speed = self.speed_entry.get()
        try:
            # Validate it's a number
            float(speed)
            self.send_cmd("speed", speed)
        except ValueError:
            self.log_message(f"❌ Invalid speed: {speed}")
    
    def set_heading(self):
        """Set fixed heading from entry field"""
        heading = self.heading_entry.get()
        try:
            # Validate it's a number
            float(heading)
            self.send_cmd("heading", heading)
        except ValueError:
            self.log_message(f"❌ Invalid heading: {heading}")
    
    def set_default_heading(self):
        """Set heading to default (auto-rotate)"""
        self.heading_entry.delete(0, tk.END)
        self.heading_entry.insert(0, "-1")
        self.send_cmd("heading", "-1")
        self.log_message("🧭 Heading set to AUTO ROTATE")
    
    def takeoff_dialog(self):
        """Show takeoff altitude dialog"""
        dialog = tk.Toplevel(self.root)
        dialog.title("Takeoff")
        dialog.geometry("300x150")
        
        ttk.Label(dialog, text="Enter takeoff altitude (meters):").pack(pady=10)
        
        alt_var = tk.StringVar(value="10")
        ttk.Entry(dialog, textvariable=alt_var, width=10).pack(pady=5)
        
        def do_takeoff():
            self.send_cmd("takeoff", alt_var.get())
            dialog.destroy()
        
        ttk.Button(dialog, text="Takeoff", command=do_takeoff).pack(pady=10)
    
    def log_message(self, msg):
        """Add message to log"""
        self.log_text.insert(tk.END, f"{msg}\n")
        self.log_text.see(tk.END)
    
    def on_closing(self):
        """Clean shutdown"""
        if self.recording:
            self.toggle_recording()
        self.stop_video()
        self.backend.shutdown()
        self.root.destroy()

# Main entry point
if __name__ == "__main__":
    root = tk.Tk()
    app = DroneGUI(root)
    root.mainloop()