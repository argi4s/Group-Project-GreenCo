# yolov10_detector.py
import numpy as np

# pip install ultralytics
from ultralytics import YOLO

# person = 0 in COCO
_PERSON_CLASS_ID = 0

# load model only once
_MODEL = YOLO("yolo26n_openvino_model/")  # or change to yolov10n.pt/ or other model

def detect_bbox_on_bgr(frame_bgr, conf=0.25, imgsz=640):
    """
    input：BGR ndarray (H,W,3)
    output：bbox (x1,y1,x2,y2) in original image，or None
    """
    results = _MODEL.predict(
        source=frame_bgr,
        imgsz=imgsz,
        conf=conf,
        classes=[_PERSON_CLASS_ID],  # only detect person
        verbose=False
    )
    r = results[0]
    if r.boxes is None or len(r.boxes) == 0:
        return None

    boxes = r.boxes.xyxy.cpu().numpy()
    confs = r.boxes.conf.cpu().numpy()
    i = int(np.argmax(confs))
    score = float(confs[i])
    x1, y1, x2, y2 = boxes[i].astype(int).tolist()

    h, w = frame_bgr.shape[:2]
    x1 = max(0, min(w - 1, x1))
    y1 = max(0, min(h - 1, y1))
    x2 = max(0, min(w - 1, x2))
    y2 = max(0, min(h - 1, y2))
    if x2 <= x1 or y2 <= y1:
        return None
    return (x1, y1, x2, y2), score
