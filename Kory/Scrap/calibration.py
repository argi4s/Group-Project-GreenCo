#!/usr/bin/env python3
import numpy as np
import cv2 as cv
from flask import Flask, Response, render_template_string
from picamera2 import Picamera2
import time

# --------------------------
# Calibration settings
# --------------------------

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

GRID_SHAPE = (8, 13)       # inner corners
SQUARE_SIZE = 28.5e-3      # 1.65 cm squares

# Object points template
objp = np.zeros((GRID_SHAPE[0] * GRID_SHAPE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:GRID_SHAPE[0], 0:GRID_SHAPE[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints = []
imgpoints = []

calibration_done = False
calib_results = "Calibration not performed yet."

# --------------------------
# Camera setup
# --------------------------

use_picam2 = True

try:
    picam2 = Picamera2()
    config = picam2.create_video_configuration(main={"size": (640, 480)})
    picam2.configure(config)
    picam2.start()
except Exception:
    print("Picamera2 unavailable, falling back to USB webcam.")
    use_picam2 = False
    cam = cv.VideoCapture(0)

# --------------------------
# Flask App
# --------------------------

app = Flask(__name__)

HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>Camera Calibration Stream</title>
</head>
<body style="background:#111; color:white; text-align:center;">
    <h1>Camera Calibration Stream</h1>

    {% if not calibrated %}
        <p>Collected: {{count}} / 100 chessboard samples</p>
    {% else %}
        <h2>Calibration Completed</h2>
        <pre style="text-align:left; margin:auto; width:80%; border:1px solid white; padding:10px;">
{{results}}
        </pre>
    {% endif %}

    <img src="/video_feed" style="width:80%; border:3px solid white;">
</body>
</html>
"""

@app.route("/")
def index():
    return render_template_string(
        HTML_PAGE,
        count=len(objpoints),
        calibrated=calibration_done,
        results=calib_results
    )

# --------------------------
# Streaming Generator
# --------------------------

def generate_frames():
    global objpoints, imgpoints, calibration_done, calib_results

    frame = None

    while True:

        # --------------------------
        # Capture frame
        # --------------------------
        if use_picam2:
            frame = picam2.capture_array()
        else:
            ret, frame = cam.read()
            if not ret:
                continue

        # Only attempt data collection before calibration
        if not calibration_done:

            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            ret, corners = cv.findChessboardCorners(gray, GRID_SHAPE, None)

            if ret:
                objpoints.append(objp)
                corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners2)
                cv.drawChessboardCorners(frame, GRID_SHAPE, corners2, ret)

                print(f"[INFO] Collected {len(objpoints)} / 100")

            # --------------------------
            # When 100 samples are collected → CALIBRATE
            # --------------------------
            if len(objpoints) >= 100:
                print("[INFO] Running camera calibration...")
                t0 = time.time()

                ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
                    objpoints, imgpoints, gray.shape[::-1], None, None
                )

                elapsed = time.time() - t0
                print(f"[INFO] Calibration done in {elapsed:.1f} seconds")

                # Compute reprojection error
                mean_error = 0
                for i in range(len(objpoints)):
                    imgpts2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
                    err = cv.norm(imgpoints[i], imgpts2, cv.NORM_L2) / len(imgpts2)
                    mean_error += err
                mean_error /= len(objpoints)

                calib_results = (
                    f"Camera Matrix:\n{mtx}\n\n"
                    f"Distortion Coefficients:\n{dist}\n\n"
                    f"Mean Reprojection Error: {mean_error:.6f}\n"
                )
                np.savez('camera_calibration.npz', mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)

                calibration_done = True

        # --------------------------
        # Output frame as JPEG
        # --------------------------
        ret2, buffer = cv.imencode(".jpg", frame)
        jpg = buffer.tobytes()

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
        )


@app.route("/video_feed")
def video_feed():
    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )

# --------------------------
# Run Server
# --------------------------

if __name__ == "__main__":
    print("Open on your LAN: http://<raspberry-pi-ip>:5000")
    app.run(host="0.0.0.0", port=5000, debug=False)
