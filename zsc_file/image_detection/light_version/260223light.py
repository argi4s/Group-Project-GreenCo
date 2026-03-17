import cv2
import numpy as np
from collections import deque

# -----------------------------
# Utilities
# -----------------------------
def is_headless():
    import os
    return (os.name != "nt") and (os.environ.get("DISPLAY", "") == "")

def clean_mask(mask, k=5, it_open=1, it_close=1):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    m = mask.astype(np.uint8) * 255
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel, iterations=it_open)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel, iterations=it_close)
    return (m > 0)

def grass_mask_hsv(bgr, h_lo=30, h_hi=95, s_min=30):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h = hsv[:, :, 0]
    s = hsv[:, :, 1]
    return (h >= h_lo) & (h <= h_hi) & (s >= s_min)

def cloth_seed_hsv(bgr, grass_mask,
                   v_percentile=70, v_max=200,
                   s_percentile=70, s_max=160,
                   grass_h_lo=30, grass_h_hi=95, grass_s_min=30):
    """衣服/人体seed：非草 + (偏暗 or 低饱和)，并排除绿色(阴影草)"""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h = hsv[:, :, 0]
    s = hsv[:, :, 1]
    v = hsv[:, :, 2]

    v_thr = int(np.clip(np.percentile(v, v_percentile), 0, v_max))
    s_thr = int(np.clip(np.percentile(s, s_percentile), 0, s_max))

    is_grass_color = (h >= grass_h_lo) & (h <= grass_h_hi) & (s >= grass_s_min)
    non_grass = ~grass_mask

    dark = (v < v_thr)
    low_sat = (s < s_thr)
    seed = non_grass & (~is_grass_color) & (dark | low_sat)
    return seed

def expand_rect(x, y, w, h, pad, W, H):
    x0 = max(0, x - pad)
    y0 = max(0, y - pad)
    x1 = min(W, x + w + pad)
    y1 = min(H, y + h + pad)
    return x0, y0, x1 - x0, y1 - y0

# -----------------------------
# HOG person detector (no training / no download)
# -----------------------------
_HOG = None
def hog_detect_person(frame_bgr, downscale=0.5):
    """
    返回：bbox (x,y,w,h) in original frame coords，或 None
    """
    global _HOG
    if _HOG is None:
        _HOG = cv2.HOGDescriptor()
        _HOG.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    H, W = frame_bgr.shape[:2]
    if downscale != 1.0:
        small = cv2.resize(frame_bgr, (int(W * downscale), int(H * downscale)), interpolation=cv2.INTER_AREA)
    else:
        small = frame_bgr

    # detectMultiScale 很吃CPU：参数尽量温和
    rects, weights = _HOG.detectMultiScale(
        small,
        winStride=(8, 8),
        padding=(8, 8),
        scale=1.05
    )

    if len(rects) == 0:
        return None

    # 选权重最大那个人
    best_i = int(np.argmax(weights))
    x, y, w, h = rects[best_i]
    if downscale != 1.0:
        x = int(x / downscale); y = int(y / downscale)
        w = int(w / downscale); h = int(h / downscale)

    # 简单过滤太小的框
    if w * h < (H * W) * 0.02:
        return None
    return (x, y, w, h)

# -----------------------------
# Temporal gate (optional)
# -----------------------------
class SimpleTemporalGate:
    def __init__(self, keep_frames=3, max_dist=60):
        self.keep_frames = keep_frames
        self.max_dist = max_dist
        self.history = deque(maxlen=keep_frames)

    def update(self, centers):
        self.history.append(centers)
        if len(self.history) < self.keep_frames:
            return None
        last = self.history[-1]
        if not last:
            return None

        def has_match(pt, pts):
            for q in pts:
                if (pt[0]-q[0])**2 + (pt[1]-q[1])**2 <= self.max_dist**2:
                    return True
            return False

        stable = []
        for pt in last:
            ok = True
            for prev in list(self.history)[:-1]:
                if not has_match(pt, prev):
                    ok = False
                    break
            if ok:
                stable.append(pt)
        return stable[0] if stable else None

# -----------------------------
# GrabCut refine inside ROI
# -----------------------------
def refine_grabcut(frame_bgr, roi_rect, grass_mask, seed_mask, iters=2):
    """
    roi_rect: (x,y,w,h)
    grass_mask: bool (sure background)
    seed_mask: bool (sure/probable foreground seed)
    """
    H, W = frame_bgr.shape[:2]
    x, y, w, h = roi_rect

    gc = np.full((H, W), cv2.GC_PR_BGD, np.uint8)
    gc[grass_mask] = cv2.GC_BGD

    # ROI 内先都设成 PR_FG，允许长出来
    gc[y:y+h, x:x+w] = cv2.GC_PR_FGD

    # seed 设为 sure FG（再轻微膨胀，别太碎）
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    seed_u8 = (seed_mask.astype(np.uint8) * 255)
    seed_d = cv2.dilate(seed_u8, k, iterations=1) > 0
    gc[seed_d] = cv2.GC_FGD

    bgdModel = np.zeros((1, 65), np.float64)
    fgdModel = np.zeros((1, 65), np.float64)

    cv2.grabCut(frame_bgr, gc, (x, y, w, h), bgdModel, fgdModel, iters, cv2.GC_INIT_WITH_MASK)
    out = (gc == cv2.GC_FGD) | (gc == cv2.GC_PR_FGD)
    # 强制不吃草
    out = out & (~grass_mask)
    return out

# -----------------------------
# Main entry for tests
# -----------------------------
def process_frame(frame_bgr, w=640, h=360, use_hog=True, debug=False):
    """
    返回：vis(BGR), mask_u8, dbg(dict)
    """
    frame = cv2.resize(frame_bgr, (w, h), interpolation=cv2.INTER_AREA)
    H, W = frame.shape[:2]

    if debug:
        print("[DEBUG] USING NEW LIGHT_VERSION (HOG+GrabCut)")

    grass = grass_mask_hsv(frame, h_lo=30, h_hi=95, s_min=30)

    # 1) 先用HOG找人ROI（找不到再fallback）
    rect = hog_detect_person(frame, downscale=0.5) if use_hog else None

    if rect is None:
        # fallback：用 seed 连通域找一个大致区域（比你原来强一点）
        seed = cloth_seed_hsv(frame, grass_mask=grass, v_percentile=70, s_percentile=70)
        seed = clean_mask(seed, k=3)
        # 把碎片连接起来方便成一个区域
        k7 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        seed_cc = cv2.dilate((seed.astype(np.uint8) * 255), k7, iterations=2) > 0
        ys, xs = np.where(seed_cc)
        if len(xs) == 0:
            out = np.zeros((H, W), dtype=bool)
            vis = frame.copy()
            return vis, (out.astype(np.uint8) * 255), {"grass": grass, "seed": seed, "rect": None}
        x0, x1 = xs.min(), xs.max()
        y0, y1 = ys.min(), ys.max()
        rect = (int(x0), int(y0), int(x1-x0+1), int(y1-y0+1))

    # 2) 扩大ROI，保证衣服都在里面
    x, y, ww, hh = rect
    pad = int(0.60 * max(ww, hh))
    rx, ry, rw, rh = expand_rect(x, y, ww, hh, pad, W, H)
    roi_rect = (rx, ry, rw, rh)

    # 3) ROI内构建 seed（暗 or 低饱和），然后GrabCut补全
    seed = cloth_seed_hsv(frame, grass_mask=grass, v_percentile=70, s_percentile=70)
    seed = clean_mask(seed, k=3)

    out = refine_grabcut(frame, roi_rect=roi_rect, grass_mask=grass, seed_mask=seed, iters=2)
    out = clean_mask(out, k=5)

    vis = frame.copy()
    vis[out] = (0, 0, 255)

    if debug:
        cv2.rectangle(vis, (rx, ry), (rx+rw, ry+rh), (255, 0, 0), 2)  # ROI框

    return vis, (out.astype(np.uint8) * 255), {"grass": grass, "seed": seed, "rect": roi_rect}

# -----------------------------
# Keep your original video loop
# -----------------------------
def main(video_path=0):
    cap = cv2.VideoCapture(video_path)
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        vis, mask_u8, _ = process_frame(frame, w=640, h=360, use_hog=True, debug=False)
        cv2.imshow("vis", vis)
        cv2.imshow("mask", mask_u8)
        if cv2.waitKey(1) & 0xFF == 27:
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(0)