#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
260221lawnmover_test.py (NO third-party deps)
- Exports CSV + QGC WPL 110
- Saves SVG (local XY meters) for quick visual check (no matplotlib needed)
- Saves Leaflet HTML map (no folium needed; uses CDN)
"""

from __future__ import annotations

import os
import sys
import math
import argparse
import importlib.util
from typing import List, Tuple

LatLon = Tuple[float, float]
XY = Tuple[float, float]

# -------------------------
# load planner by file path (dataclass-safe for py3.12)
# -------------------------
HERE = os.path.dirname(os.path.abspath(__file__))

# your planner file is present as 260220lawnmower_only.py in this folder
PLANNER_PATH = os.path.join(HERE, "260220lawnmower_only.py")
if not os.path.exists(PLANNER_PATH):
    raise FileNotFoundError(
        f"[ERROR] planner file not found: {PLANNER_PATH}\n"
        f"Current dir: {HERE}\n"
        f"Files here: {os.listdir(HERE)}"
    )

spec = importlib.util.spec_from_file_location("lawnmower_planner", PLANNER_PATH)
planner = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = planner  # IMPORTANT for dataclasses in py3.12
spec.loader.exec_module(planner)

plan_lawnmower = planner.plan_lawnmower
waypoints_to_tuples = planner.waypoints_to_tuples
CameraFootprintRef = planner.CameraFootprintRef
footprint_at_alt = planner.footprint_at_alt
latlon_to_xy_m = planner.latlon_to_xy_m


# -------------------------
# small utils
# -------------------------
def polyline_length_xy(path: List[XY]) -> float:
    if len(path) < 2:
        return 0.0
    s = 0.0
    for (x1, y1), (x2, y2) in zip(path[:-1], path[1:]):
        s += math.hypot(x2 - x1, y2 - y1)
    return s

def export_csv(path_csv: str, wps: List[Tuple[float, float, float]]):
    with open(path_csv, "w", encoding="utf-8") as f:
        f.write("lat,lon,alt_m\n")
        for lat, lon, alt in wps:
            f.write(f"{lat:.8f},{lon:.8f},{alt:.2f}\n")

def export_qgc_wpl110(path_wpl: str, wps: List[Tuple[float, float, float]]):
    """
    QGC WPL 110 format
    frame=3 => MAV_FRAME_GLOBAL_RELATIVE_ALT
    cmd=16 => MAV_CMD_NAV_WAYPOINT
    """
    with open(path_wpl, "w", encoding="utf-8") as f:
        f.write("QGC WPL 110\n")
        for i, (lat, lon, alt) in enumerate(wps):
            current = 1 if i == 0 else 0
            f.write(f"{i}\t{current}\t3\t16\t0\t0\t0\t0\t{lat}\t{lon}\t{alt}\t1\n")


# -------------------------
# SVG visual (no matplotlib)
# -------------------------
def save_svg_xy(path_svg: str,
                origin: LatLon,
                flight_area: List[LatLon],
                wps_latlon: List[Tuple[float, float, float]]):
    poly_xy = [latlon_to_xy_m(p, origin) for p in flight_area]
    path_xy = [latlon_to_xy_m((lat, lon), origin) for (lat, lon, _alt) in wps_latlon]

    xs = [p[0] for p in poly_xy] + ([p[0] for p in path_xy] if path_xy else [])
    ys = [p[1] for p in poly_xy] + ([p[1] for p in path_xy] if path_xy else [])
    if not xs or not ys:
        raise ValueError("empty geometry")

    minx, maxx = min(xs), max(xs)
    miny, maxy = min(ys), max(ys)

    pad = 20.0
    W = (maxx - minx) + 2 * pad
    H = (maxy - miny) + 2 * pad

    def tx(x: float) -> float:
        return (x - minx) + pad

    def ty(y: float) -> float:
        # SVG y axis goes down; invert to keep north up
        return (maxy - y) + pad

    poly_pts = " ".join([f"{tx(x):.2f},{ty(y):.2f}" for x, y in poly_xy] + [f"{tx(poly_xy[0][0]):.2f},{ty(poly_xy[0][1]):.2f}"])
    path_pts = " ".join([f"{tx(x):.2f},{ty(y):.2f}" for x, y in path_xy])

    start = path_xy[0] if path_xy else None
    end = path_xy[-1] if path_xy else None

    svg = []
    svg.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="800" viewBox="0 0 {W:.2f} {H:.2f}">')
    svg.append("""
<style>
  .area { fill: rgba(30,144,255,0.12); stroke: #1e90ff; stroke-width: 2; }
  .path { fill: none; stroke: #ff7f0e; stroke-width: 2; }
  .ptS  { fill: #2ca02c; }
  .ptE  { fill: #d62728; }
  .txt  { font-family: ui-sans-serif, system-ui; font-size: 16px; fill: #333; }
</style>
""")
    svg.append(f'<polyline class="area" points="{poly_pts}" />')
    if path_xy:
        svg.append(f'<polyline class="path" points="{path_pts}" />')
        svg.append(f'<circle class="ptS" cx="{tx(start[0]):.2f}" cy="{ty(start[1]):.2f}" r="5" />')
        svg.append(f'<circle class="ptE" cx="{tx(end[0]):.2f}" cy="{ty(end[1]):.2f}" r="5" />')
    svg.append(f'<text class="txt" x="10" y="24">Local XY (m): Flight Area + LawnMower Path (Start=green, End=red)</text>')
    svg.append('</svg>')

    with open(path_svg, "w", encoding="utf-8") as f:
        f.write("\n".join(svg))


# -------------------------
# Leaflet HTML (no folium)
# -------------------------
def save_leaflet_html(path_html: str,
                      origin: LatLon,
                      flight_area: List[LatLon],
                      wps_latlon: List[Tuple[float, float, float]]):
    poly_js = "[" + ",".join([f"[{lat:.8f},{lon:.8f}]" for lat, lon in flight_area] + [f"[{flight_area[0][0]:.8f},{flight_area[0][1]:.8f}]"]) + "]"
    path_js = "[" + ",".join([f"[{lat:.8f},{lon:.8f}]" for (lat, lon, _alt) in wps_latlon]) + "]"

    start_js = f"[{wps_latlon[0][0]:.8f},{wps_latlon[0][1]:.8f}]" if wps_latlon else None
    end_js   = f"[{wps_latlon[-1][0]:.8f},{wps_latlon[-1][1]:.8f}]" if wps_latlon else None

    html = f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Lawnmower Path</title>
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
  <style>
    html, body, #map {{ height: 100%; margin: 0; }}
    .info {{ position: absolute; top: 10px; left: 10px; z-index: 999; background: white; padding: 8px 10px; border-radius: 8px; box-shadow: 0 2px 10px rgba(0,0,0,.15); font-family: system-ui; }}
  </style>
</head>
<body>
<div class="info">Flight Area + LawnMower Path</div>
<div id="map"></div>
<script>
  const map = L.map('map').setView([{origin[0]:.8f}, {origin[1]:.8f}], 17);
  L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
    maxZoom: 20,
    attribution: '&copy; OpenStreetMap contributors'
  }}).addTo(map);

  const area = L.polyline({poly_js}, {{color: '#1e90ff', weight: 4}}).addTo(map);
  const path = L.polyline({path_js}, {{color: '#ff7f0e', weight: 4}}).addTo(map);

  const group = L.featureGroup([area, path]).addTo(map);
  map.fitBounds(group.getBounds().pad(0.1));

  {"L.marker(" + start_js + ").addTo(map).bindPopup('Start');" if start_js else ""}
  {"L.marker(" + end_js + ").addTo(map).bindPopup('End');" if end_js else ""}
</script>
</body>
</html>
"""
    with open(path_html, "w", encoding="utf-8") as f:
        f.write(html)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--alt", type=float, default=40.0)
    ap.add_argument("--side_overlap", type=float, default=0.30)
    ap.add_argument("--forward_overlap", type=float, default=0.70)
    ap.add_argument("--max_alt", type=float, default=50.0)
    ap.add_argument("--out_prefix", type=str, default="lawnmower_260220")
    ap.add_argument("--html", action="store_true")
    ap.add_argument("--svg", action="store_true")
    args = ap.parse_args()

    # ---- your inputs ----
    TOL = (51.4234722, -2.6715556)
    flight_area = [
        (51.423561, -2.671297),
        (51.422842, -2.670066),
        (51.424173, -2.668757),
        (51.423444, -2.668178),
    ]

    cam_ref = CameraFootprintRef(ref_alt_m=50.0, ref_width_m=52.4, ref_height_m=39.3)
    fp_w, fp_h = footprint_at_alt(min(args.alt, args.max_alt), cam_ref)

    wps = plan_lawnmower(
        search_polygon=flight_area,
        altitude_m=args.alt,
        origin_latlon=TOL,
        camera_ref=cam_ref,
        side_overlap=args.side_overlap,
        forward_overlap=args.forward_overlap,
        max_alt_m=args.max_alt,
        holes=None,
        start_latlon=TOL,
        use_along_step_points=False,
    )

    wps_tuples = waypoints_to_tuples(wps)
    print(f"[INFO] altitude={min(args.alt, args.max_alt):.1f}m footprint≈({fp_w:.1f}m x {fp_h:.1f}m)")
    print(f"[INFO] waypoints = {len(wps_tuples)}")
    print("[INFO] first 5:", wps_tuples[:5])

    path_xy = [latlon_to_xy_m((lat, lon), TOL) for (lat, lon, _alt) in wps_tuples]
    print(f"[INFO] path length ≈ {polyline_length_xy(path_xy):.1f} m")

    export_csv(args.out_prefix + ".csv", wps_tuples)
    export_qgc_wpl110(args.out_prefix + ".waypoints", wps_tuples)
    print(f"[INFO] exported: {args.out_prefix}.csv , {args.out_prefix}.waypoints")

    if args.svg:
        save_svg_xy(args.out_prefix + ".svg", TOL, flight_area, wps_tuples)
        print(f"[INFO] saved SVG: {args.out_prefix}.svg")

    if args.html:
        save_leaflet_html(args.out_prefix + ".html", TOL, flight_area, wps_tuples)
        print(f"[INFO] saved HTML: {args.out_prefix}.html")


if __name__ == "__main__":
    main()
