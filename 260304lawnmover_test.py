#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test script for a lawnmower planner loaded by FILE PATH (works even if filename starts with digits).
Outputs:
- CSV waypoints
- optional QGC WPL 110 mission
- GeoJSON (no deps)
- HTML Leaflet map (no python deps; loads Leaflet from CDN)

Usage:
  python 260304lawnmover_test.py --mode pca
  python 260304lawnmover_test.py --mode manual --manual_deg 90
  python 260304lawnmover_test.py --mode auto_min_turns
"""

from __future__ import annotations
import argparse
import os
import math
import json
import sys
import hashlib
import importlib.util
from typing import List, Tuple, Optional

LatLon = Tuple[float, float]
XY = Tuple[float, float]

# ---- builtin fallback (used when tol_file is missing) ----
DEFAULT_TOL = (51.42347222222222, -2.6715555555555555)  # from take off locatoin.txt
DEFAULT_FLIGHT_POLY = [
    (51.423561, -2.671297),
    (51.422842, -2.670066),
    (51.424173, -2.668757),
    (51.423444, -2.668178),
]


# ------------------ dynamic import by path (py3.12 dataclasses-safe) ------------------

def load_planner_module(planner_path: str):
    planner_path = os.path.abspath(planner_path)
    if not os.path.exists(planner_path):
        raise FileNotFoundError(f"planner_path not found: {planner_path}")

    mod_name = "planner_mod_" + hashlib.md5(planner_path.encode("utf-8")).hexdigest()[:10]
    spec = importlib.util.spec_from_file_location(mod_name, planner_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to create module spec from: {planner_path}")

    mod = importlib.util.module_from_spec(spec)
    # critical: register before exec_module (fixes dataclasses on py3.12)
    sys.modules[mod_name] = mod
    spec.loader.exec_module(mod)
    return mod


# ------------------ helpers: parsing + IO ------------------

def dms_to_decimal(dms: str) -> float:
    s = dms.strip()
    hemi = s[-1].upper()
    body = s[:-1]
    body = body.replace("°", " ").replace("'", " ").replace('"', " ")
    parts = [p for p in body.split() if p]
    if len(parts) < 2:
        raise ValueError(f"Bad DMS: {dms}")
    deg = float(parts[0])
    minute = float(parts[1])
    sec = float(parts[2]) if len(parts) >= 3 else 0.0
    val = deg + minute / 60.0 + sec / 3600.0
    if hemi in ("S", "W"):
        val = -val
    return val


def load_tol_and_flight_area(txt_path: str) -> Tuple[LatLon, List[LatLon]]:
    """
    Preferred: read TOL + flight area polygon from txt_path.
    Fallback: if file doesn't exist, use builtin DEFAULT_TOL / DEFAULT_FLIGHT_POLY.
    """
    if not txt_path or not os.path.exists(txt_path):
        print(f"[WARN] tol_file not found: {txt_path!r} -> using builtin DEFAULT_TOL/DEFAULT_FLIGHT_POLY")
        return DEFAULT_TOL, DEFAULT_FLIGHT_POLY[:]

    tol: Optional[LatLon] = None
    poly: List[LatLon] = []

    with open(txt_path, "r", encoding="utf-8") as f:
        lines = [ln.strip() for ln in f.readlines() if ln.strip()]

    for ln in lines:
        if ln.upper().startswith("TOL"):
            parts = ln.split()
            if len(parts) < 3:
                raise ValueError(f"Bad TOL line: {ln}")
            tol = (dms_to_decimal(parts[1]), dms_to_decimal(parts[2]))
            break
    if tol is None:
        raise ValueError("Failed to parse TOL from file")

    in_area = False
    for ln in lines:
        if "flight" in ln.lower() and "aera" in ln.lower():
            in_area = True
            continue
        if in_area and "," in ln:
            a, b = [x.strip() for x in ln.split(",")[:2]]
            poly.append((float(a), float(b)))

    if len(poly) < 3:
        raise ValueError("Failed to parse flight area polygon (need >=3 points)")

    return tol, poly


def load_polygon_csv(path: str) -> List[LatLon]:
    pts: List[LatLon] = []
    with open(path, "r", encoding="utf-8") as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith("#"):
                continue
            if "," not in ln:
                continue
            a, b = [x.strip() for x in ln.split(",")[:2]]
            pts.append((float(a), float(b)))
    if len(pts) < 3:
        raise ValueError(f"Polygon file must have >=3 points: {path}")
    return pts


def write_csv(path: str, wps: List[Tuple[float, float, float]]) -> None:
    with open(path, "w", encoding="utf-8") as f:
        f.write("lat,lon,alt_m\n")
        for lat, lon, alt in wps:
            f.write(f"{lat:.8f},{lon:.8f},{alt:.3f}\n")


def write_qgc_wpl110(path: str,
                    wps: List[Tuple[float, float, float]],
                    *,
                    home_lat: float,
                    home_lon: float,
                    home_alt: float = 0.0) -> None:
    FRAME_GLOBAL_REL_ALT = 3
    MAV_CMD_NAV_WAYPOINT = 16

    def line(i: int, current: int, lat: float, lon: float, alt: float) -> str:
        return f"{i}\t{current}\t{FRAME_GLOBAL_REL_ALT}\t{MAV_CMD_NAV_WAYPOINT}\t0\t0\t0\t0\t{lat:.8f}\t{lon:.8f}\t{alt:.3f}\t1\n"

    with open(path, "w", encoding="utf-8") as f:
        f.write("QGC WPL 110\n")
        f.write(line(0, 1, home_lat, home_lon, home_alt))
        for i, (lat, lon, alt) in enumerate(wps, start=1):
            f.write(line(i, 0, lat, lon, alt))


# ------------------ metrics ------------------

def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def path_turns_and_length_xy(path: List[XY], *, turn_thresh_deg: float = 60.0) -> Tuple[int, float]:
    if len(path) < 2:
        return 0, 0.0

    angles: List[float] = []
    length = 0.0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        seg_len = math.hypot(dx, dy)
        length += seg_len
        if seg_len > 1e-6:
            angles.append(math.atan2(dy, dx))

    if len(angles) < 2:
        return 0, length

    thr = math.radians(turn_thresh_deg)
    turns = 0
    for a1, a2 in zip(angles[:-1], angles[1:]):
        if abs(wrap_pi(a2 - a1)) >= thr:
            turns += 1
    return turns, length


# ------------------ GeoJSON + HTML (NEW, no deps) ------------------

def _closed_ring_lonlat(poly: List[LatLon]) -> List[List[float]]:
    """Return closed ring [ [lon,lat], ... ]"""
    ring = [[lon, lat] for (lat, lon) in poly]
    if ring and (ring[0][0] != ring[-1][0] or ring[0][1] != ring[-1][1]):
        ring.append(ring[0])
    return ring

def write_geojson(path: str,
                  *,
                  tol: LatLon,
                  polygon: List[LatLon],
                  holes: List[List[LatLon]],
                  wps: List[Tuple[float, float, float]],
                  mode: str) -> None:
    features = []

    # outer polygon
    outer = _closed_ring_lonlat(polygon)
    poly_coords = [outer]
    # holes as interior rings (optional)
    for h in holes:
        poly_coords.append(_closed_ring_lonlat(h))

    features.append({
        "type": "Feature",
        "properties": {"name": "search_area"},
        "geometry": {"type": "Polygon", "coordinates": poly_coords}
    })

    # path line
    line = [[lon, lat] for (lat, lon, _alt) in wps]
    features.append({
        "type": "Feature",
        "properties": {"name": f"path_{mode}", "mode": mode},
        "geometry": {"type": "LineString", "coordinates": line}
    })

    # start/end/tol points
    if wps:
        (lat0, lon0, _a0) = wps[0]
        (lat1, lon1, _a1) = wps[-1]
        features.append({
            "type": "Feature",
            "properties": {"name": "start"},
            "geometry": {"type": "Point", "coordinates": [lon0, lat0]}
        })
        features.append({
            "type": "Feature",
            "properties": {"name": "end"},
            "geometry": {"type": "Point", "coordinates": [lon1, lat1]}
        })

    features.append({
        "type": "Feature",
        "properties": {"name": "TOL"},
        "geometry": {"type": "Point", "coordinates": [tol[1], tol[0]]}
    })

    fc = {"type": "FeatureCollection", "features": features}
    with open(path, "w", encoding="utf-8") as f:
        json.dump(fc, f, ensure_ascii=False, indent=2)

def write_leaflet_html(path: str,
                       *,
                       geojson_obj: dict,
                       title: str = "Lawnmower Path Viewer") -> None:
    # Embed GeoJSON as JS object to avoid fetch/CORS.
    geojson_str = json.dumps(geojson_obj, ensure_ascii=False)

    html = f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>{title}</title>
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"
        integrity="sha256-p4NxAoJBhIIN+hmNHrzRCf9tD/miZyoHS5obTRR9BMY=" crossorigin=""/>
  <style>
    html, body {{ height: 100%; margin: 0; }}
    #map {{ height: 100%; }}
    .legend {{
      position: absolute; z-index: 999;
      left: 10px; top: 10px;
      background: rgba(255,255,255,0.9);
      padding: 10px 12px; border-radius: 8px;
      font-family: ui-sans-serif, system-ui, -apple-system;
      font-size: 13px;
      box-shadow: 0 2px 10px rgba(0,0,0,0.15);
    }}
  </style>
</head>
<body>
<div class="legend">
  <div><b>{title}</b></div>
  <div>• Polygon: search_area</div>
  <div>• Line: path</div>
  <div>• Points: TOL / start / end</div>
</div>
<div id="map"></div>

<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"
        integrity="sha256-20nQCchB9co0qIjJZRGuk2/Z9VM+kNiyxNV1lvTlZBo=" crossorigin=""></script>
<script>
  const GEO = {geojson_str};

  const map = L.map('map', {{ zoomControl: true }});

  // base layer
  L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
    maxZoom: 20,
    attribution: '&copy; OpenStreetMap contributors'
  }}).addTo(map);

  function styleFn(feature) {{
    const name = (feature.properties && feature.properties.name) || "";
    if (feature.geometry.type === "Polygon") {{
      return {{ weight: 2, fillOpacity: 0.15 }};
    }}
    if (feature.geometry.type === "LineString") {{
      return {{ weight: 3 }};
    }}
    return {{}};
  }}

  function pointToLayer(feature, latlng) {{
    const name = (feature.properties && feature.properties.name) || "pt";
    if (name === "TOL") return L.circleMarker(latlng, {{ radius: 7, weight: 2 }});
    if (name === "start") return L.circleMarker(latlng, {{ radius: 7, weight: 2 }});
    if (name === "end") return L.circleMarker(latlng, {{ radius: 7, weight: 2 }});
    return L.circleMarker(latlng, {{ radius: 6, weight: 2 }});
  }}

  const layer = L.geoJSON(GEO, {{
    style: styleFn,
    pointToLayer: pointToLayer,
    onEachFeature: (feature, lyr) => {{
      const props = feature.properties || {{}};
      const name = props.name || feature.geometry.type;
      const extra = props.mode ? ` (mode=${{props.mode}})` : "";
      lyr.bindPopup(`<b>${{name}}</b>${{extra}}`);
    }}
  }}).addTo(map);

  const bounds = layer.getBounds();
  if (bounds && bounds.isValid()) {{
    map.fitBounds(bounds.pad(0.15));
  }} else {{
    map.setView([51.4545, -2.5879], 13); // Bristol fallback
  }}
</script>
</body>
</html>"""
    with open(path, "w", encoding="utf-8") as f:
        f.write(html)


# ------------------ main ------------------

def main():
    ap = argparse.ArgumentParser()

    ap.add_argument(
        "--planner_path",
        default="./260304lawnmover.py",
        help="Path to the planner .py file (default: ./260304lawnmover.py in this folder)"
    )

    ap.add_argument("--mode", default="pca",
                    choices=["pca", "manual", "auto_min_turns"])
    ap.add_argument("--manual_deg", type=float, default=0.0)
    ap.add_argument("--offset_deg", type=float, default=0.0)

    ap.add_argument("--alt", type=float, default=40.0)
    ap.add_argument("--max_alt", type=float, default=50.0)
    ap.add_argument("--side_overlap", type=float, default=0.30)
    ap.add_argument("--forward_overlap", type=float, default=0.70)
    ap.add_argument("--use_along_step_points", action="store_true")

    ap.add_argument("--auto_step_deg", type=float, default=5.0)
    ap.add_argument("--auto_fine_step_deg", type=float, default=1.0)
    ap.add_argument("--turn_thresh_deg", type=float, default=60.0)

    ap.add_argument("--margin_m", type=float, default=1.0)

    ap.add_argument("--out_dir", default="./runs/lawnmower_test")
    ap.add_argument("--write_qgc", action="store_true")

    # geojson/html outputs (no deps)
    ap.add_argument("--no_geojson", dest="write_geojson", action="store_false", help="disable geojson output")
    ap.add_argument("--no_html", dest="write_html", action="store_false", help="disable html output")
    ap.set_defaults(write_geojson=True, write_html=True)

    # polygon inputs
    ap.add_argument("--tol_file", default="take off locatoin.txt")
    ap.add_argument("--polygon_file", default="")
    ap.add_argument("--hole_file", action="append", default=[])

    ap.add_argument("--start_from_tol", action="store_true")

    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    # Load planner module by path
    mod = load_planner_module(args.planner_path)

    # Expect these symbols exist in your planner file
    plan_lawnmower = getattr(mod, "plan_lawnmower")
    waypoints_to_tuples = getattr(mod, "waypoints_to_tuples")
    latlon_to_xy_m_func = getattr(mod, "latlon_to_xy_m")
    CameraFootprintRef = getattr(mod, "CameraFootprintRef")

    # Load TOL + polygon (fallback if missing)
    tol, flight_poly = load_tol_and_flight_area(args.tol_file)
    polygon = load_polygon_csv(args.polygon_file) if args.polygon_file else flight_poly
    holes: List[List[LatLon]] = [load_polygon_csv(hf) for hf in args.hole_file]
    start_latlon = tol if args.start_from_tol else None

    # Plan
    wps = plan_lawnmower(
        search_polygon=polygon,
        altitude_m=args.alt,
        origin_latlon=tol,
        camera_ref=CameraFootprintRef(),
        side_overlap=args.side_overlap,
        forward_overlap=args.forward_overlap,
        max_alt_m=args.max_alt,
        holes=holes if holes else None,
        margin_m=args.margin_m,
        start_latlon=start_latlon,
        use_along_step_points=args.use_along_step_points,

        heading_mode=args.mode,
        manual_heading_deg=args.manual_deg,
        heading_offset_deg=args.offset_deg,
        auto_step_deg=args.auto_step_deg,
        auto_fine_step_deg=args.auto_fine_step_deg,
        turn_thresh_deg=args.turn_thresh_deg,
    )

    wps_tuples = waypoints_to_tuples(wps)

    # Metrics in XY
    path_xy = [latlon_to_xy_m_func((lat, lon), tol) for (lat, lon, _alt) in wps_tuples]
    turns, length_m = path_turns_and_length_xy(path_xy, turn_thresh_deg=args.turn_thresh_deg)

    print("=== Lawnmower Test ===")
    print(f"planner_path    : {os.path.abspath(args.planner_path)}")
    print(f"mode            : {args.mode}")
    print(f"manual_deg      : {args.manual_deg:.2f}")
    print(f"offset_deg      : {args.offset_deg:.2f}")
    print(f"points          : {len(wps_tuples)}")
    print(f"path length     : {length_m:.1f} m")
    print(f"turns (>=thr)   : {turns}  (thr={args.turn_thresh_deg:.1f} deg)")
    print(f"out_dir         : {os.path.abspath(args.out_dir)}")

    # Export CSV
    out_csv = os.path.join(args.out_dir, f"waypoints_{args.mode}.csv")
    write_csv(out_csv, wps_tuples)
    print(f"[OK] wrote {out_csv}")

    # Export QGC mission file (optional)
    if args.write_qgc:
        out_wpl = os.path.join(args.out_dir, f"mission_{args.mode}.waypoints")
        write_qgc_wpl110(out_wpl, wps_tuples, home_lat=tol[0], home_lon=tol[1], home_alt=0.0)
        print(f"[OK] wrote {out_wpl}")

    # GeoJSON + HTML (default ON)
    if args.write_geojson or args.write_html:
        # build geojson object in-memory (so HTML can embed it)
        # reuse the same structure as write_geojson() would dump
        features = []

        outer = _closed_ring_lonlat(polygon)
        poly_coords = [outer]
        for h in holes:
            poly_coords.append(_closed_ring_lonlat(h))
        features.append({
            "type": "Feature",
            "properties": {"name": "search_area"},
            "geometry": {"type": "Polygon", "coordinates": poly_coords}
        })

        line = [[lon, lat] for (lat, lon, _alt) in wps_tuples]
        features.append({
            "type": "Feature",
            "properties": {"name": f"path_{args.mode}", "mode": args.mode},
            "geometry": {"type": "LineString", "coordinates": line}
        })

        if wps_tuples:
            (lat0, lon0, _a0) = wps_tuples[0]
            (lat1, lon1, _a1) = wps_tuples[-1]
            features.append({
                "type": "Feature",
                "properties": {"name": "start"},
                "geometry": {"type": "Point", "coordinates": [lon0, lat0]}
            })
            features.append({
                "type": "Feature",
                "properties": {"name": "end"},
                "geometry": {"type": "Point", "coordinates": [lon1, lat1]}
            })
        features.append({
            "type": "Feature",
            "properties": {"name": "TOL"},
            "geometry": {"type": "Point", "coordinates": [tol[1], tol[0]]}
        })

        geo = {"type": "FeatureCollection", "features": features}

        if args.write_geojson:
            out_geojson = os.path.join(args.out_dir, f"plan_{args.mode}.geojson")
            with open(out_geojson, "w", encoding="utf-8") as f:
                json.dump(geo, f, ensure_ascii=False, indent=2)
            print(f"[OK] wrote {out_geojson}")

        if args.write_html:
            out_html = os.path.join(args.out_dir, f"plan_{args.mode}.html")
            title = f"Lawnmower Plan ({args.mode})"
            write_leaflet_html(out_html, geojson_obj=geo, title=title)
            print(f"[OK] wrote {out_html}")
            print("      Open the HTML in VSCode (click) or download and open in a browser.")

if __name__ == "__main__":
    main()
