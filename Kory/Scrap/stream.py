#!/usr/bin/env python3
from flask import Flask, Response, render_template_string
from picamera2 import Picamera2
import cv2
import numpy as np


app = Flask(__name__)

# Initialize camera
picam2 = Picamera2()
config = picam2.create_video_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

# Simple HTML page
HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>Raspberry Pi Camera Stream</title>
</head>
<body style="background: #111; color: white; text-align: center;">
    <h1>Raspberry Pi Camera Stream</h1>
    <img src="/video_feed" style="width: 80%; border: 3px solid white;">
</body>
</html>
"""
@app.route('/')
def index():
    return render_template_string(HTML_PAGE)



def generate_frames():
    while True:
        frame = picam2.capture_array()
        #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        lower_green = np.array([40,50,50])
        upper_green = np.array([80,255,255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        result = cv2.bitwise_and(frame, frame, mask= mask)
        ret, buffer = cv2.imencode('.jpg', result)
        frame = buffer.tobytes()

        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n'
        )


@app.route('/video_feed')






def video_feed():
    return Response(
        generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

if __name__ == '__main__':
    # host='0.0.0.0' makes it accessible only within the local network
    app.run(host='0.0.0.0', port=5000, debug=False)

