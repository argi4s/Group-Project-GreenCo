# search_analysis.py

from typing import List, Tuple
import math

LonLatAlt = Tuple[float, float, float]

def total_distance(wps: List[LonLatAlt]) -> float:
    """Meters."""
    def hav(a, b):
        lon1, lat1, _ = a
        lon2, lat2, _ = b
        R = 6371000
        dlat = math.radians(lat2-lat1)
        dlon = math.radians(lon2-lon1)
        lat1r = math.radians(lat1)
        lat2r = math.radians(lat2)
        h = math.sin(dlat/2)**2 + math.cos(lat1r)*math.cos(lat2r)*math.sin(dlon/2)**2
        return 2*R*math.asin(math.sqrt(h))
    return sum(hav(wps[i], wps[i+1]) for i in range(len(wps)-1))

def estimated_time(distance_m: float, speed_mps: float) -> float:
    return distance_m / speed_mps

def coverage_area(search_polygon) -> float:
    """Polygon area in m²."""
    # Shoelace formula
    area = 0
    for i in range(len(search_polygon)):
        lon1, lat1 = search_polygon[i]
        lon2, lat2 = search_polygon[(i+1)%len(search_polygon)]
        area += lon1*lat2 - lon2*lat1
    return abs(area)/2

def coverage_factor(sweep_width, track_spacing):
    return sweep_width / track_spacing

def probability_of_detection(C):
    """Simple POD approximation."""
    return 1 - math.exp(-C)
