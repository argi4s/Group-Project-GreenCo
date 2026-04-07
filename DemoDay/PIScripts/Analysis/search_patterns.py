# search_patterns.py
# Interchangeable SAR search patterns compatible with your lawnmower generator.
# All return MAVLink-ready (lon, lat, alt) tuples.

from __future__ import annotations
from dataclasses import dataclass
from typing import List, Tuple
import math

LonLat = Tuple[float, float]
LonLatAlt = Tuple[float, float, float]

# -----------------------------
# Helpers
# -----------------------------

def meters_per_deg_lat(lat):
    return 111_320.0

def meters_per_deg_lon(lat):
    return 111_320.0 * math.cos(math.radians(lat))

def lonlat_to_xy(p: LonLat, origin: LonLat):
    lon, lat = p
    lon0, lat0 = origin
    x = (lon - lon0) * meters_per_deg_lon(lat0)
    y = (lat - lat0) * meters_per_deg_lat(lat0)
    return x, y

def xy_to_lonlat(xy, origin: LonLat):
    x, y = xy
    lon0, lat0 = origin
    lat = lat0 + y / meters_per_deg_lat(lat0)
    lon = lon0 + x / meters_per_deg_lon(lat0)
    return lon, lat

def rotate(x, y, ang_rad):
    c, s = math.cos(ang_rad), math.sin(ang_rad)
    return c*x - s*y, s*x + c*y

# -----------------------------
# Config
# -----------------------------

@dataclass
class SearchPatternConfig:
    altitude_m: float
    track_spacing_m: float
    origin_lonlat: LonLat
    heading_deg: float = 0.0
    legs: int = 10  # for expanding/square/sector
    sweep_width_m: float = None   # <--- ADD THIS

# -----------------------------
# 1. Parallel Track Search
# -----------------------------

def parallel_track(search_polygon: List[LonLat], cfg: SearchPatternConfig) -> List[LonLatAlt]:
    """
    Classic parallel track (like lawnmower but simpler).
    """
    origin = cfg.origin_lonlat
    heading = math.radians(cfg.heading_deg)

    # Convert polygon to XY
    poly_xy = [lonlat_to_xy(p, origin) for p in search_polygon]
    xs = [p[0] for p in poly_xy]
    ys = [p[1] for p in poly_xy]

    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)

    dy = cfg.track_spacing_m
    rows = int((y_max - y_min) / dy) + 1

    wps = []
    reverse = False

    for i in range(rows):
        y = y_min + i * dy
        x0, x1 = x_min, x_max
        if reverse:
            x0, x1 = x1, x0
        wps.append((x0, y))
        wps.append((x1, y))
        reverse = not reverse

    # Rotate + convert back to lon/lat
    out = []
    for x, y in wps:
        xr, yr = rotate(x, y, heading)
        lon, lat = xy_to_lonlat((xr, yr), origin)
        out.append((lon, lat, cfg.altitude_m))

    return out

# -----------------------------
# 2. Creeping Line Search
# -----------------------------

def creeping_line(search_polygon: List[LonLat], cfg: SearchPatternConfig) -> List[LonLatAlt]:
    """
    Same as parallel track but legs run perpendicular to major axis.
    """
    # Swap x/y roles by rotating 90 degrees
    cfg2 = SearchPatternConfig(
        altitude_m=cfg.altitude_m,
        track_spacing_m=cfg.track_spacing_m,
        origin_lonlat=cfg.origin_lonlat,
        heading_deg=cfg.heading_deg + 90
    )
    return parallel_track(search_polygon, cfg2)

# -----------------------------
# 3. Expanding Square Search
# -----------------------------

def expanding_square(datum: LonLat, cfg: SearchPatternConfig) -> List[LonLatAlt]:
    """
    Expanding square around datum.
    """
    origin = datum
    step = cfg.track_spacing_m
    legs = cfg.legs

    x, y = 0.0, 0.0
    wps_xy = [(x, y)]

    direction = 0  # 0=E,1=N,2=W,3=S
    length = step

    for i in range(1, legs+1):
        for _ in range(2):
            dx = step if direction == 0 else -step if direction == 2 else 0
            dy = step if direction == 1 else -step if direction == 3 else 0
            for _ in range(i):
                x += dx
                y += dy
                wps_xy.append((x, y))
            direction = (direction + 1) % 4

    # Convert back
    out = []
    for x, y in wps_xy:
        lon, lat = xy_to_lonlat((x, y), origin)
        out.append((lon, lat, cfg.altitude_m))
    return out

# -----------------------------
# 4. Sector Search
# -----------------------------

def sector_search(datum: LonLat, cfg: SearchPatternConfig) -> List[LonLatAlt]:
    """
    120° sector search with repeated sweeps.
    """
    origin = datum
    radius = cfg.track_spacing_m
    legs = cfg.legs

    out = []
    for i in range(legs):
        ang = math.radians((i % 3) * 120)
        x = radius * math.cos(ang)
        y = radius * math.sin(ang)
        lon, lat = xy_to_lonlat((x, y), origin)
        out.append((lon, lat, cfg.altitude_m))
        out.append(origin + (cfg.altitude_m,))

    return out

# -----------------------------
# 5. Track Line Search
# -----------------------------

def track_line(start: LonLat, end: LonLat, cfg: SearchPatternConfig) -> List[LonLatAlt]:
    """
    Straight line between two points.
    """
    lon1, lat1 = start
    lon2, lat2 = end
    return [(lon1, lat1, cfg.altitude_m), (lon2, lat2, cfg.altitude_m)]

# -----------------------------
# 6. Shoreline Search
# -----------------------------

def shoreline_search(polyline: List[LonLat], cfg: SearchPatternConfig) -> List[LonLatAlt]:
    """
    Follow coastline polyline.
    """
    return [(lon, lat, cfg.altitude_m) for lon, lat in polyline]

# -----------------------------
# 7. Barrier Search
# -----------------------------

def barrier_search(start: LonLat, end: LonLat, cfg: SearchPatternConfig) -> List[LonLatAlt]:
    """
    Patrol back and forth between two fixed points.
    """
    wps = []
    for i in range(cfg.legs):
        if i % 2 == 0:
            wps.append((*start, cfg.altitude_m))
        else:
            wps.append((*end, cfg.altitude_m))
    return wps
