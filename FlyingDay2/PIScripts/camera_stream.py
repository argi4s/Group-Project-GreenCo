#!/usr/bin/env python3
# camera_stream.py - Camera streaming module for FSM integration

from picamera2 import Picamera2
import cv2
import subprocess
import yolov10_detector
import socket
import json
import time
import threading
import os
import signal

class CameraStream:
    def __init__(self, video_ip="192.168.1.17", video_port=5800, vec_port=5801,
                 width=640, height=480, fps=30, detect_every_n=3):
        # Configuration
        self.VIDEO_IP = video_ip
        self.VIDEO_PORT = video_port
        self.VEC_PORT = vec_port
        self.W = width
        self.H = height
        self.FPS = fps
        self.DETECT_EVERY_N_FRAMES = detect_every_n
        
        # State
        self.running = False
        self.thread = None
        self.frame_idx = 0
        self.last_det = None
        
        # GStreamer process
        self.proc = None
        self.vec_sock = None
        self.picam2 = None
        
    def start(self):
        """Start the camera stream in a background thread"""
        if self.running:
            print("[WARNING] Camera already running")
            return False
            
        self.running = True
        self.thread = threading.Thread(target=self._run_camera, daemon=True)
        self.thread.start()
        print(f"[INFO] Camera stream started -> {self.VIDEO_IP}:{self.VIDEO_PORT}")
        return True
    
    def stop(self):
        """Stop the camera stream"""
        print("[INFO] Stopping camera stream...")
        self.running = False
        if self.thread:
            self.thread.join(timeout=5)
        self._cleanup()
        print("[INFO] Camera stopped")
    
    def _setup_gstreamer(self):
        """Setup GStreamer pipeline"""
        gst_cmd = [
            "gst-launch-1.0", "-q",
            "fdsrc", "!",
            "videoparse", f"width={self.W}", f"height={self.H}", "format=bgr", f"framerate={self.FPS}/1", "!",
            "queue", "!",
            "videoconvert", "!",
            "x264enc", "tune=zerolatency", "speed-preset=ultrafast",
            "bitrate=1500", f"key-int-max={self.FPS}", "!",
            "h264parse", "!",
            "rtph264pay", "pt=96", "config-interval=1", "!",
            "udpsink", f"host={self.VIDEO_IP}", f"port={self.VIDEO_PORT}", "sync=false", "async=false"
        ]
        
        self.proc = subprocess.Popen(gst_cmd, stdin=subprocess.PIPE)
        
        # Setup vector socket
        self.vec_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    def _setup_camera(self):
        """Setup Picamera2"""
        self.picam2 = Picamera2()
        self.picam2.configure(
            self.picam2.create_video_configuration(
                main={"size": (self.W, self.H), "format": "RGB888"},
                controls={"FrameRate": self.FPS}
            )
        )
        self.picam2.start()
    
    def _cleanup(self):
        """Clean up resources"""
        try:
            if self.proc:
                self.proc.stdin.close()
                self.proc.terminate()
        except:
            pass
        self.proc = None
        
        if self.vec_sock:
            self.vec_sock.close()
            self.vec_sock = None
    
    def _run_camera(self):
        """Main camera loop (runs in thread)"""
        try:
            self._setup_camera()
            self._setup_gstreamer()
            
            while self.running:
                # 1 Capture RGB frame
                frame_rgb = self.picam2.capture_array()
                
                # 2 Convert to BGR
                frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
                
                # 3 Control detection frequency
                self.frame_idx += 1
                if self.frame_idx % self.DETECT_EVERY_N_FRAMES == 0:
                    self.last_det = yolov10_detector.detect_bbox_on_bgr(frame_bgr)
                
                # 4 Get detection
                bbox = None
                score = None
                if self.last_det is not None:
                    bbox, score = self.last_det
                
                # 5 Draw bbox (optional - comment out if you want raw stream)
                if bbox is not None:
                    x1, y1, x2, y2 = bbox
                    cv2.rectangle(frame_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(
                        frame_bgr,
                        f"person {score:.2f}",
                        (x1, max(0, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2
                    )
                
                # 6 Send vector data
                msg = {"t": time.time(), "det": 0}
                
                if bbox is not None:
                    cx = (x1 + x2) / 2.0
                    cy = (y1 + y2) / 2.0
                    dx = cx - (self.W / 2.0)
                    dy = cy - (self.H / 2.0)
                    dxn = dx / (self.W / 2.0)
                    dyn = dy / (self.H / 2.0)
                    msg = {
                        "t": time.time(),
                        "det": 1,
                        "score": float(score),
                        "dx": dx, "dy": dy,
                        "dxn": dxn, "dyn": dyn,
                        "bbox": [int(x1), int(y1), int(x2), int(y2)]
                    }
                
                try:
                    self.vec_sock.sendto(json.dumps(msg).encode("utf-8"), 
                                       (self.VIDEO_IP, self.VEC_PORT))
                except:
                    pass
                
                # 7 Push to livestream
                self.proc.stdin.write(frame_bgr.tobytes())
                
                # 8 Flush buffer periodically
                if self.frame_idx % 30 == 0:
                    self.proc.stdin.flush()
                    
        except Exception as e:
            print(f"[ERROR] Camera thread error: {e}")
        finally:
            self._cleanup()
    
    def get_detection(self):
        """Get the latest detection (if needed by FSM)"""
        return self.last_det

# For standalone testing
if __name__ == "__main__":
    import time
    
    camera = CameraStream()
    try:
        camera.start()
        print("Camera running. Press Ctrl+C to stop")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        camera.stop()