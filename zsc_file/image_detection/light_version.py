import cv2
import numpy as np
from collections import deque

def exg_grass_mask(bgr, thr_mode="percentile", p=65):
    # ExG = 2G - R - B  (int16 防止溢出)
    b = bgr[:, :, 0].astype(np.int16)
    g = bgr[:, :, 1].astype(np.int16)
    r = bgr[:, :, 2].astype(np.int16)
    exg = 2 * g - r - b

    if thr_mode == "otsu":
        exg_u8 = cv2.normalize(exg, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        _, m = cv2.threshold(exg_u8, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        return (m > 0)
    else:
        thr = np.percentile(exg, p)  # 自适应：取分位数做阈值
        return (exg > thr)

def dark_mask_hsv(bgr, v_percentile=25, v_min=0, v_max=80):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    v = hsv[:, :, 2]
    # 自适应：更暗的那一部分
    thr = int(np.percentile(v, v_percentile))
    thr = np.clip(thr, v_min, v_max)  # 防止过亮/过暗导致阈值飘
    return (v < thr)

def clean_mask(mask, k=5):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    m = mask.astype(np.uint8) * 255
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel, iterations=1)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel, iterations=1)
    return (m > 0)

def filter_components(mask, min_area=150, max_area=20000, ar_min=0.2, ar_max=5.0):
    # 返回：过滤后的mask，以及候选blob信息
    m = mask.astype(np.uint8)
    num, labels, stats, centroids = cv2.connectedComponentsWithStats(m, connectivity=8)
    out = np.zeros_like(m, dtype=np.uint8)
    blobs = []
    for i in range(1, num):
        x, y, w, h, area = stats[i]
        if area < min_area or area > max_area:
            continue
        ar = w / (h + 1e-6)
        if ar < ar_min or ar > ar_max:
            continue
        out[labels == i] = 1
        blobs.append({"id": i, "bbox": (x, y, w, h), "area": area, "c": tuple(centroids[i])})
    return out.astype(bool), blobs

class SimpleTemporalGate:
    """连续出现 >= N 帧才输出；用中心点近邻做关联"""
    def __init__(self, keep_frames=3, max_dist=60):
        self.keep_frames = keep_frames
        self.max_dist = max_dist
        self.history = deque(maxlen=keep_frames)  # 存每帧的中心点列表

    def update(self, blobs):
        centers = [b["c"] for b in blobs]
        self.history.append(centers)

        if len(self.history) < self.keep_frames:
            return None  # 暂时不输出

        # 简单策略：找在最近 keep_frames 帧里都“有附近中心点”的目标
        # 以最后一帧中心点为候选，往前找是否都能匹配到
        last = self.history[-1]
        if not last:
            return None

        def has_match(pt, pts, md):
            for q in pts:
                if (pt[0]-q[0])**2 + (pt[1]-q[1])**2 <= md*md:
                    return True
            return False

        stable_centers = []
        for pt in last:
            ok = True
            for prev in list(self.history)[:-1]:
                if not has_match(pt, prev, self.max_dist):
                    ok = False
                    break
            if ok:
                stable_centers.append(pt)

        # 只返回一个最稳定的（你也可以返回多个）
        if not stable_centers:
            return None
        return stable_centers[0]

def main(video_path=0):
    cap = cv2.VideoCapture(video_path)
    gate = SimpleTemporalGate(keep_frames=3, max_dist=60)

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        # 1) 降采样
        frame_small = cv2.resize(frame, (640, 360), interpolation=cv2.INTER_AREA)

        # 2) 草地mask
        grass = exg_grass_mask(frame_small, thr_mode="percentile", p=65)

        # 3) 非草地
        non_grass = ~grass

        # 4) 暗色mask（黑衣服）
        dark = dark_mask_hsv(frame_small, v_percentile=25, v_min=0, v_max=90)

        # 5) 候选
        cand = non_grass & dark

        # 6) 清理
        cand = clean_mask(cand, k=5)

        # 7) 连通域筛选
        filt, blobs = filter_components(cand, min_area=120, max_area=25000, ar_min=0.15, ar_max=6.0)

        # 8) 时间门控（稳定3帧才认为是真人）
        stable_c = gate.update(blobs)

        # 输出mask：默认是 filt；如果你要只输出“稳定目标”，就只保留最近的一个blob
        out_mask = filt.copy()
        if stable_c is not None:
            # 只保留离 stable_c 最近的blob
            best = None
            best_d = 1e18
            for b in blobs:
                cx, cy = b["c"]
                d = (cx - stable_c[0])**2 + (cy - stable_c[1])**2
                if d < best_d:
                    best_d = d
                    best = b
            if best is not None:
                out_mask[:] = False
                x, y, w, h = best["bbox"]
                out_mask[y:y+h, x:x+w] = True  # 先用bbox当mask范围（更快）
                # 如果你想用真实连通域mask：可用 labels==best["id"]（需要上面返回labels）

        # 可视化
        vis = frame_small.copy()
        vis[out_mask] = (0, 0, 255)  # 红色叠加（只是显示用）
        cv2.imshow("vis", vis)
        cv2.imshow("mask", (out_mask.astype(np.uint8)*255))

        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(0)  # 0=摄像头；或替换为视频文件路径
