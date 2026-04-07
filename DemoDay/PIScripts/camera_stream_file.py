import cv2
import socket
import json
import time
import threading
import yolov10_detector

class CameraStreamFile:
    def __init__(self, filename, vec_port=5801, detect_every_n=3):
        self.filename = filename
        self.vec_port = vec_port
        self.detect_every_n = detect_every_n

        self.running = False
        self.thread = None
        self.vec_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()

    def _run(self):
        cap = cv2.VideoCapture(self.filename)
        frame_idx = 0

        while self.running:
            ret, frame = cap.read()
            if not ret:
                break

            frame_idx += 1

            # Run YOLO every N frames
            det = None
            if frame_idx % self.detect_every_n == 0:
                det = yolov10_detector.detect_bbox_on_bgr(frame)

            # Build vector message
            msg = {"t": time.time(), "det": 0}

            if det:
                (x1, y1, x2, y2), score = det
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                dx = cx - frame.shape[1]/2
                dy = cy - frame.shape[0]/2

                msg = {
                    "t": time.time(),
                    "det": 1,
                    "score": float(score),
                    "dx": dx,
                    "dy": dy,
                    "dxn": dx / (frame.shape[1]/2),
                    "dyn": dy / (frame.shape[0]/2),
                    "bbox": [x1, y1, x2, y2]
                }

            payload = json.dumps(msg).encode()

            # Send to VisionReceiver (same as real camera)
            self.vec_sock.sendto(payload, ("127.0.0.1", self.vec_port))

            # Simulate real-time playback
            time.sleep(1/30.0)
