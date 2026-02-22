# lawnmower_planner.py
# Altitude-aware lawnmower (boustrophedon) waypoint generator for a polygon search area.
# - spacing is computed from camera footprint that scales ~linearly with altitude
# - works with a general simple polygon (and optional "holes" to avoid no-fly areas)
# - outputs lat/lon/alt waypoints you can feed into your MAVLink GUIDED loop or mission uploader

from __future__ import annotations
from dataclasses import dataclass
from typing import List, Tuple, Optional, Iterable
import math

LatLon = Tuple[float, float]   # (lat_deg, lon_deg)
XY = Tuple[float, float]       # (x_m East, y_m North)

@dataclass(frozen=True)
class Waypoint:
    lat: float
    lon: float
    alt_m: float

# ---------- geo helpers (flat-earth local tangent for small areas) ----------

def _meters_per_deg_lat(lat_deg: float) -> float:
    # good enough for < few km areas
    return 111_320.0

def _meters_per_deg_lon(lat_deg: float) -> float:
    return 111_320.0 * math.cos(math.radians(lat_deg))

def latlon_to_xy_m(p: LatLon, origin: LatLon) -> XY:
    lat, lon = p
    lat0, lon0 = origin
    x = (lon - lon0) * _meters_per_deg_lon(lat0)
    y = (lat - lat0) * _meters_per_deg_lat(lat0)
    return (x, y)

def xy_to_latlon(p: XY, origin: LatLon) -> LatLon:
    x, y = p
    lat0, lon0 = origin
    lat = lat0 + y / _meters_per_deg_lat(lat0)
    lon = lon0 + x / _meters_per_deg_lon(lat0)
    return (lat, lon)

def _rotate(p: XY, ang_rad: float) -> XY:
    x, y = p
    c, s = math.cos(ang_rad), math.sin(ang_rad)
    return (c*x - s*y, s*x + c*y)

def _ensure_closed(poly: List[XY]) -> List[XY]:
    if not poly:
        raise ValueError("polygon is empty")
    if poly[0] != poly[-1]:
        return poly + [poly[0]]
    return poly

# ---------- camera footprint + spacing ----------

@dataclass(frozen=True)
class CameraFootprintRef:
    ref_alt_m: float = 50.0
    # NOTE: set these to your calibrated footprint at ref_alt_m
    # (you said before ~52.4m x 39.3m at 50m AGL; keep as defaults)
    ref_width_m: float = 52.4   # cross-track width
    ref_height_m: float = 39.3  # along-track height

def footprint_at_alt(alt_m: float, ref: CameraFootprintRef) -> Tuple[float, float]:
    """
    Simple nadir-camera approximation: footprint scales linearly with altitude.
    Returns (width_m, height_m).
    """
    if alt_m <= 0:
        raise ValueError("alt_m must be > 0")
    scale = alt_m / ref.ref_alt_m
    return (ref.ref_width_m * scale, ref.ref_height_m * scale)

def track_spacing_m(footprint_width_m: float, side_overlap: float) -> float:
    """
    side_overlap in [0, 0.95]. 0.3 means 30% overlap between adjacent swaths.
    """
    side_overlap = float(side_overlap)
    side_overlap = max(0.0, min(0.95, side_overlap))
    return footprint_width_m * (1.0 - side_overlap)

def along_step_m(footprint_height_m: float, forward_overlap: float) -> float:
    """
    forward_overlap in [0, 0.95]. Used if you want intermediate points along each row.
    If you only fly row endpoints (continuous video), you can ignore this.
    """
    forward_overlap = float(forward_overlap)
    forward_overlap = max(0.0, min(0.95, forward_overlap))
    return footprint_height_m * (1.0 - forward_overlap)

# ---------- geometry: line-polygon intersection at y = const ----------

def _poly_x_intersections_at_y(poly: List[XY], y: float) -> List[float]:
    """
    Returns sorted x-intersections between polygon edges and horizontal line y=const.
    Uses the classic (y1>y)!=(y2>y) rule to avoid double-counting vertices.
    """
    xs: List[float] = []
    poly = _ensure_closed(poly)
    for (x1, y1), (x2, y2) in zip(poly[:-1], poly[1:]):
        # ignore horizontal edges
        if y1 == y2:
            continue
        # include edge if it straddles y
        if (y1 > y) != (y2 > y):
            t = (y - y1) / (y2 - y1)
            x = x1 + t * (x2 - x1)
            xs.append(x)
    xs.sort()
    return xs

def _x_to_intervals(xs: List[float]) -> List[Tuple[float, float]]:
    """
    Pair sorted intersection xs into inside intervals [x0,x1], [x2,x3], ...
    """
    if len(xs) < 2:
        return []
    if len(xs) % 2 != 0:
        # numerical edge case; drop last
        xs = xs[:-1]
    return [(xs[i], xs[i+1]) for i in range(0, len(xs), 2) if xs[i+1] > xs[i]]

def _subtract_intervals(base: List[Tuple[float, float]],
                        cut: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """
    base minus cut, in 1D. All intervals are (a<b).
    """
    if not base:
        return []
    if not cut:
        return base

    cut_sorted = sorted(cut)
    out: List[Tuple[float, float]] = []
    for a, b in base:
        cur_a, cur_b = a, b
        for c, d in cut_sorted:
            if d <= cur_a:
                continue
            if c >= cur_b:
                break
            # overlap exists
            if c > cur_a:
                out.append((cur_a, min(c, cur_b)))
            cur_a = max(cur_a, d)
            if cur_a >= cur_b:
                break
        if cur_a < cur_b:
            out.append((cur_a, cur_b))
    return [(a, b) for a, b in out if b - a > 1e-6]

def _segment_points_1d(x0: float, x1: float, step: Optional[float]) -> List[float]:
    """
    Create points along [x0,x1]. Always includes endpoints.
    If step is None or step<=0 -> only endpoints.
    """
    if x1 < x0:
        x0, x1 = x1, x0
    if step is None or step <= 0:
        return [x0, x1]
    L = x1 - x0
    if L <= 1e-6:
        return [x0]
    n = max(1, int(math.floor(L / step)))
    xs = [x0 + i * (L / n) for i in range(n + 1)]
    xs[-1] = x1
    return xs

# ---------- choose sweep direction (PCA) ----------

def _pca_heading_rad(points: List[XY]) -> float:
    """
    Returns angle of the principal axis (largest variance) relative to +x (East).
    """
    if len(points) < 2:
        return 0.0
    mx = sum(p[0] for p in points) / len(points)
    my = sum(p[1] for p in points) / len(points)
    Sxx = sum((p[0]-mx)**2 for p in points) / len(points)
    Syy = sum((p[1]-my)**2 for p in points) / len(points)
    Sxy = sum((p[0]-mx)*(p[1]-my) for p in points) / len(points)

    # eigenvector for largest eigenvalue of [[Sxx,Sxy],[Sxy,Syy]]
    # angle = 0.5*atan2(2Sxy, Sxx-Syy)
    ang = 0.5 * math.atan2(2.0*Sxy, (Sxx - Syy))
    return ang

# ---------- main planner ----------

def plan_lawnmower(
    search_polygon: List[LatLon],
    altitude_m: float,
    *,
    origin_latlon: LatLon,  # usually TOL
    camera_ref: CameraFootprintRef = CameraFootprintRef(),
    side_overlap: float = 0.30,
    forward_overlap: float = 0.70,
    max_alt_m: Optional[float] = 50.0,  # cap to comply with project requirement if desired
    holes: Optional[List[List[LatLon]]] = None,  # exclusion polygons (e.g., SSSI)
    margin_m: float = 1.0,  # trim segment ends inward to avoid boundary touch
    start_latlon: Optional[LatLon] = None,  # choose first row direction closer to this
    use_along_step_points: bool = False,     # True -> insert intermediate points along rows
) -> List[Waypoint]:
    """
    Returns a boustrophedon (lawnmower) path covering the polygon, spacing set by altitude.
    """
    if max_alt_m is not None:
        altitude_m = min(float(altitude_m), float(max_alt_m))
    altitude_m = float(altitude_m)
    if altitude_m <= 0:
        raise ValueError("altitude_m must be > 0")

    # convert polygons to local XY
    outer_xy = [latlon_to_xy_m(p, origin_latlon) for p in search_polygon]
    outer_xy = _ensure_closed(outer_xy)[:-1]  # keep open list for PCA etc.

    holes_xy: List[List[XY]] = []
    if holes:
        for h in holes:
            hx = [latlon_to_xy_m(p, origin_latlon) for p in h]
            holes_xy.append(_ensure_closed(hx)[:-1])

    # footprint -> spacing
    fp_w, fp_h = footprint_at_alt(altitude_m, camera_ref)
    dy = track_spacing_m(fp_w, side_overlap)  # row spacing
    dx_step = along_step_m(fp_h, forward_overlap) if use_along_step_points else None

    if dy <= 0.2:
        raise ValueError(f"track spacing too small (dy={dy:.3f}m); check overlaps/footprint")

    # choose sweep heading by PCA and rotate to sweep-frame (rows along x, stepping y)
    heading = _pca_heading_rad(outer_xy)
    # rotate by -heading so principal axis aligns with +x
    rot = -heading
    outer_r = [_rotate(p, rot) for p in outer_xy]
    holes_r = [[_rotate(p, rot) for p in h] for h in holes_xy]

    ys = [p[1] for p in outer_r]
    y_min, y_max = min(ys), max(ys)

    # pick y lines; we include endpoints
    n_rows = max(1, int(math.ceil((y_max - y_min) / dy)) + 1)

    # optional start point (in rotated frame)
    start_r: Optional[XY] = None
    if start_latlon is not None:
        start_xy = latlon_to_xy_m(start_latlon, origin_latlon)
        start_r = _rotate(start_xy, rot)

    waypoints_xy_r: List[XY] = []
    reverse = False

    # Build rows from y_min to y_max
    for k in range(n_rows):
        y = y_min + k * dy
        if y > y_max:
            y = y_max

        xs_outer = _poly_x_intersections_at_y(outer_r, y)
        intervals = _x_to_intervals(xs_outer)
        if not intervals:
            continue

        # subtract holes
        for h in holes_r:
            xs_h = _poly_x_intersections_at_y(h, y)
            cut = _x_to_intervals(xs_h)
            if cut:
                intervals = _subtract_intervals(intervals, cut)
                if not intervals:
                    break
        if not intervals:
            continue

        # For each interval, create a segment (possibly multiple segments per row if holes split it)
        # We stitch them in a direction that alternates across rows.
        segs = intervals[:]  # (x0,x1)
        if reverse:
            segs = [(b, a) for (a, b) in reversed(segs)]  # traverse from high-x to low-x

        row_points: List[XY] = []
        for x0, x1 in segs:
            # trim ends inward by margin_m
            if (x1 - x0) > 2.0 * margin_m:
                if x1 >= x0:
                    x0 += margin_m
                    x1 -= margin_m
                else:
                    x0 -= margin_m
                    x1 += margin_m

            xs = _segment_points_1d(x0, x1, dx_step)
            # keep direction: if reverse row, segment_points_1d returns increasing; flip if needed
            if x1 < x0:
                xs = list(reversed(xs))
            for x in xs:
                row_points.append((x, y))

        if not row_points:
            continue

        # For the first row only: choose direction closer to start_r if provided
        if start_r is not None and not waypoints_xy_r:
            d_first = (row_points[0][0] - start_r[0])**2 + (row_points[0][1] - start_r[1])**2
            d_last  = (row_points[-1][0] - start_r[0])**2 + (row_points[-1][1] - start_r[1])**2
            if d_last < d_first:
                row_points.reverse()
                reverse = not reverse  # keep alternation consistent after flipping

        # append row points, avoid duplicate consecutive points
        for p in row_points:
            if not waypoints_xy_r or (abs(p[0]-waypoints_xy_r[-1][0]) > 1e-6 or abs(p[1]-waypoints_xy_r[-1][1]) > 1e-6):
                waypoints_xy_r.append(p)

        reverse = not reverse

        if y >= y_max - 1e-9:
            break

    # rotate back, convert to lat/lon
    waypoints: List[Waypoint] = []
    for p_r in waypoints_xy_r:
        p_xy = _rotate(p_r, -rot)  # undo rot
        lat, lon = xy_to_latlon(p_xy, origin_latlon)
        waypoints.append(Waypoint(lat=lat, lon=lon, alt_m=altitude_m))
    return waypoints

# ---------- tiny helper for your GUIDED controller (optional) ----------

def waypoints_to_tuples(wps: Iterable[Waypoint]) -> List[Tuple[float, float, float]]:
    return [(w.lat, w.lon, w.alt_m) for w in wps]