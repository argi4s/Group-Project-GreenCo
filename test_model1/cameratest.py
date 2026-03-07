#!/usr/bin/env python3
from picamera2 import Picamera2
import cv2
import subprocess
import yolov10_detector
import socket, json, time

# Config
VIDEO_IP = "192.168.1.32"
VIDEO_PORT = 5800
#W, H, FPS = 640, 480, 30

W, H, FPS = 640, 480, 30
DETECT_EVERY_N_FRAMES = 3   # 5 frame 1 detection
frame_idx = 0
#last_bbox = None
last_det = None

# Vector text (can be change later when making combine)
VEC_IP     = "192.168.1.32"   #
VEC_PORT   = 5801             #



# GStreamer: stdin(BGR raw) -> x264 -> RTP/UDP
gst_cmd = [
    "gst-launch-1.0", "-q",
    "fdsrc", "!",
    "videoparse", f"width={W}", f"height={H}", "format=bgr", f"framerate={FPS}/1", "!",
    "queue", "!",
    "videoconvert", "!",
    "x264enc", "tune=zerolatency", "speed-preset=ultrafast",
    "bitrate=1500", f"key-int-max={FPS}", "!",
    "h264parse", "!",
    "rtph264pay", "pt=96", "config-interval=1", "!",
    "udpsink", f"host={VIDEO_IP}", f"port={VIDEO_PORT}", "sync=false", "async=false"
]

proc = subprocess.Popen(gst_cmd, stdin=subprocess.PIPE)

vec_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Camera
picam2 = Picamera2()
picam2.configure(
    picam2.create_video_configuration(
        main={"size": (W, H), "format": "RGB888"},
        controls={"FrameRate": FPS}
    )
)
picam2.start()

print(f"Streaming (RTP/H264) to {VIDEO_IP}:{VIDEO_PORT}")



try:
    while True:
        #1 （RGB）
        frame_rgb = picam2.capture_array()

        #2 change to BGR
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        # 3 control detection freq
        frame_idx += 1
        if frame_idx % DETECT_EVERY_N_FRAMES == 0:
            last_det = yolov10_detector.detect_bbox_on_bgr(frame_bgr)

        # bbox = last_bbox
        bbox = None
        score = None
        if last_det is not None:
            bbox, score = last_det  # bbox=(x1,y1,x2,y2), score=float

        # 4) draw bbox
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

        # 5 add vector UDP
        msg = {"t": time.time(), "det": 0}

        if bbox is not None:
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            dx = cx - (W / 2.0)
            dy = cy - (H / 2.0)
            dxn = dx / (W / 2.0)
            dyn = dy / (H / 2.0)
            msg = {
                "t": time.time(),
                "det": 1,
                "score": float(score),
                "dx": dx, "dy": dy,
                "dxn": dxn, "dyn": dyn,
                "bbox": [int(x1), int(y1), int(x2), int(y2)]
            }

        try:
            vec_sock.sendto(json.dumps(msg).encode("utf-8"), (VEC_IP, VEC_PORT))
        except:
            pass

        # 6 pushing the livestream
        proc.stdin.write(frame_bgr.tobytes())

        # 7 buffer
        if frame_idx % 30 == 0:
            proc.stdin.flush()

except KeyboardInterrupt:
    pass
finally:
    try:
        proc.stdin.close()
    except:
        pass
    proc.terminate()
