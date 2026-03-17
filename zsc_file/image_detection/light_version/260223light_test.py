#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import glob
import argparse
import importlib.util
from pathlib import Path

import cv2
import numpy as np


def is_headless():
    return (os.name != "nt") and (os.environ.get("DISPLAY", "") == "")


def load_module_from_path(py_path: str, module_name: str = "light_module"):
    py_path = str(Path(py_path).resolve())
    if not Path(py_path).exists():
        raise FileNotFoundError(f"Cannot find script: {py_path}")
    spec = importlib.util.spec_from_file_location(module_name, py_path)
    mod = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(mod)
    return mod


def list_images(img_dir: str, pattern: str):
    exts = [p.strip() for p in pattern.split(",") if p.strip()]
    files = []
    for ext in exts:
        files.extend(glob.glob(str(Path(img_dir) / ext)))
    return sorted(files)


def ensure_dir(d: Path):
    d.mkdir(parents=True, exist_ok=True)


def to_u8_mask(mask):
    if mask is None:
        return None
    if mask.dtype != np.uint8:
        mask = mask.astype(np.uint8)
    if mask.max() <= 1:
        mask = mask * 255
    return mask


def overlay_red(bgr, mask_u8):
    vis = bgr.copy()
    if mask_u8 is None:
        return vis
    m = mask_u8 > 0
    vis[m] = (0, 0, 255)
    return vis


def call_process_frame(mod, frame_bgr, w, h, use_hog, debug):
    """
    兼容多种 process_frame() 签名：
    - (vis, mask_u8, dbg)
    - (vis, mask_u8, blobs, stable_c)
    - 或只返回 (vis, mask_u8)
    """
    if not hasattr(mod, "process_frame"):
        raise AttributeError("Target script has no process_frame(). Please add one or use a script that provides it.")

    fn = mod.process_frame

    # 尝试带参数调用（不同版本可能不支持 use_hog/debug）
    try:
        out = fn(frame_bgr, w=w, h=h, use_hog=use_hog, debug=debug)
    except TypeError:
        try:
            out = fn(frame_bgr, w=w, h=h)
        except TypeError:
            out = fn(frame_bgr)

    if isinstance(out, tuple):
        if len(out) == 3:
            vis, mask_u8, dbg = out
            return vis, to_u8_mask(mask_u8), dbg
        if len(out) == 4:
            vis, mask_u8, blobs, stable_c = out
            dbg = {"blobs": blobs, "stable_center": stable_c}
            return vis, to_u8_mask(mask_u8), dbg
        if len(out) == 2:
            vis, mask_u8 = out
            return vis, to_u8_mask(mask_u8), {}
    # 非预期返回
    return None, None, {"raw": out}


def save_debug(dbg, out_base: Path, w, h):
    """
    可选保存中间结果（如果 dbg 里有）
    支持：grass(bool)、seed(bool)、rect(tuple)
    """
    if not isinstance(dbg, dict):
        return

    if "grass" in dbg and dbg["grass"] is not None:
        g = dbg["grass"]
        if g.dtype != np.uint8:
            g = (g.astype(np.uint8) * 255)
        cv2.imwrite(str(out_base.with_name(out_base.name + "_dbg_grass.png")), g)

    if "seed" in dbg and dbg["seed"] is not None:
        s = dbg["seed"]
        if s.dtype != np.uint8:
            s = (s.astype(np.uint8) * 255)
        cv2.imwrite(str(out_base.with_name(out_base.name + "_dbg_seed.png")), s)

    # rect 不单独存图；由主程序画到 overlay 上


def main():
    ap = argparse.ArgumentParser("260223test_light: test image folder using imported light_version.py (headless-safe)")

    ap.add_argument("--script", type=str, default="./light_version.py",
                    help="path to your algorithm script (should provide process_frame)")
    ap.add_argument("--img", type=str, default="", help="single image path")
    ap.add_argument("--dir", type=str, default="", help="directory of images")
    ap.add_argument("--pattern", type=str, default="*.jpg,*.png,*.jpeg,*.JPG,*.PNG,*.JPEG",
                    help="glob patterns for --dir, comma-separated")
    ap.add_argument("--save", type=str, default="./out_vis",
                    help="output directory to save overlay+mask")
    ap.add_argument("--save-debug", action="store_true",
                    help="also save debug masks if available in dbg dict")

    ap.add_argument("--w", type=int, default=640)
    ap.add_argument("--h", type=int, default=360)

    ap.add_argument("--use-hog", action="store_true",
                    help="pass use_hog=True to process_frame if supported")
    ap.add_argument("--debug", action="store_true",
                    help="pass debug=True to process_frame if supported")

    ap.add_argument("--show", action="store_true",
                    help="try cv2.imshow (NOT recommended on Codespace/headless)")
    ap.add_argument("--no-show", action="store_true",
                    help="force no GUI window (recommended)")

    args = ap.parse_args()

    mod = load_module_from_path(args.script)

    # 输入补全
    if not args.img and not args.dir:
        args.dir = input("请输入图片文件夹路径（或用 --img 指定单图）：").strip()

    save_dir = Path(args.save)
    ensure_dir(save_dir)

    # GUI策略
    headless = is_headless()
    show = args.show and (not args.no_show) and (not headless)
    if args.show and headless:
        print("[WARN] Headless environment detected (no DISPLAY). --show will be ignored.")
        show = False

    # 单张图
    if args.img:
        img_path = Path(args.img)
        if not img_path.exists():
            raise FileNotFoundError(f"Image not found: {img_path}")

        frame = cv2.imread(str(img_path))
        if frame is None:
            raise RuntimeError(f"Failed to read image: {img_path}")

        vis, mask_u8, dbg = call_process_frame(mod, frame, args.w, args.h, args.use_hog, args.debug)

        # 如果 vis 没给，就自己做 overlay
        if vis is None:
            frame_small = cv2.resize(frame, (args.w, args.h), interpolation=cv2.INTER_AREA)
            vis = overlay_red(frame_small, mask_u8)

        # 如果 dbg 里有 rect，画出来方便看 ROI
        if isinstance(dbg, dict) and "rect" in dbg and dbg["rect"] is not None:
            x, y, w, h = dbg["rect"]
            cv2.rectangle(vis, (int(x), int(y)), (int(x+w), int(y+h)), (255, 0, 0), 2)

        out_base = save_dir / img_path.stem
        cv2.imwrite(str(out_base.with_name(out_base.name + "_overlay.png")), vis)
        if mask_u8 is not None:
            cv2.imwrite(str(out_base.with_name(out_base.name + "_mask.png")), mask_u8)

        if args.save_debug:
            save_debug(dbg, out_base, args.w, args.h)

        print(f"[DONE] saved to {save_dir}")
        if show:
            cv2.imshow("vis", vis)
            cv2.imshow("mask", mask_u8 if mask_u8 is not None else np.zeros((args.h, args.w), np.uint8))
            cv2.waitKey(0)
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass
        return

    # 文件夹
    if args.dir:
        files = list_images(args.dir, args.pattern)
        if not files:
            raise RuntimeError(f"No images found in {args.dir} with pattern {args.pattern}")

        print(f"[INFO] found {len(files)} images in {args.dir}")

        for i, fp in enumerate(files):
            p = Path(fp)
            frame = cv2.imread(str(p))
            if frame is None:
                print(f"[WARN] skip unreadable: {p.name}")
                continue

            vis, mask_u8, dbg = call_process_frame(mod, frame, args.w, args.h, args.use_hog, args.debug)

            if vis is None:
                frame_small = cv2.resize(frame, (args.w, args.h), interpolation=cv2.INTER_AREA)
                vis = overlay_red(frame_small, mask_u8)

            if isinstance(dbg, dict) and "rect" in dbg and dbg["rect"] is not None:
                x, y, w, h = dbg["rect"]
                cv2.rectangle(vis, (int(x), int(y)), (int(x+w), int(y+h)), (255, 0, 0), 2)

            out_base = save_dir / p.stem
            cv2.imwrite(str(out_base.with_name(out_base.name + "_overlay.png")), vis)
            if mask_u8 is not None:
                cv2.imwrite(str(out_base.with_name(out_base.name + "_mask.png")), mask_u8)

            if args.save_debug:
                save_debug(dbg, out_base, args.w, args.h)

            print(f"[{i+1}/{len(files)}] {p.name} saved")

            if show:
                cv2.imshow("vis", vis)
                cv2.imshow("mask", mask_u8 if mask_u8 is not None else np.zeros((args.h, args.w), np.uint8))
                if (cv2.waitKey(1) & 0xFF) == 27:
                    break

        if show:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass

        print(f"[DONE] all saved to {save_dir}")


if __name__ == "__main__":
    main()