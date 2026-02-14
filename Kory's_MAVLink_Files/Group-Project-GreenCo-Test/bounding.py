#!/usr/bin/env python3
from flask import Flask, Response, render_template_string
from picamera2 import Picamera2
import cv2

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
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        grid_rows = 4  # Number of rows (horizontal divisions)
        grid_cols = 4  # Number of columns (vertical divisions)

        height, width, _ = frame.shape  # Get frame dimensions
        box_height = height // grid_rows  # Height of each box
        box_width = width // grid_cols  # Width of each box

        for i in range(0,grid_rows+1):
            for j in range(0,grid_cols+1):
                cv2.rectangle(frame, (j*box_width,i*box_height), (box_width, box_height), (0,200,0), 1)


        ret, buffer = cv2.imencode('.jpg', frame)
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

