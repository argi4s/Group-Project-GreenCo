# analyse_all_patterns.py

import math
from search_patterns import (
    parallel_track, creeping_line, expanding_square,
    sector_search, shoreline_search, barrier_search,
    track_line, SearchPatternConfig
)
from search_visualisation import rasterise_coverage, plot_heatmap, plot_gaps_and_overlaps
from lawnmower import lonlat_to_xy_m

# -----------------------------
# CONFIG
# -----------------------------
SEARCH_AREA = [
    (-2.670948345438704, 51.42326956502679),
    (-2.670045428650557, 51.42287025017865),
    (-2.668169295906676, 51.42336622593724),
    (-2.668809768621569, 51.42421477437771),
    (-2.671277780473196, 51.42354069739116),
]

cfg = SearchPatternConfig(
    altitude_m=20,
    track_spacing_m=12,
    origin_lonlat=SEARCH_AREA[0],
    heading_deg=0,
    legs=20
)

origin = SEARCH_AREA[0]

# -----------------------------
# TURN COUNTER
# -----------------------------
def count_turns(wps):
    def bearing(a, b):
        lon1, lat1 = a[:2]
        lon2, lat2 = b[:2]
        dlon = math.radians(lon2 - lon1)
        lat1 = math.radians(lat1)
        lat2 = math.radians(lat2)
        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dlon)
        return math.degrees(math.atan2(x, y)) % 360

    turns = 0
    last_bearing = None

    for i in range(1, len(wps)):
        b = bearing(wps[i-1], wps[i])
        if last_bearing is not None:
            if abs(b - last_bearing) > 5:
                turns += 1
        last_bearing = b

    return turns

# -----------------------------
# DISTANCE
# -----------------------------
def total_distance(wps_xy):
    dist = 0
    for i in range(len(wps_xy)-1):
        x1,y1 = wps_xy[i]
        x2,y2 = wps_xy[i+1]
        dist += math.hypot(x2-x1, y2-y1)
    return dist

# -----------------------------
# ANALYSIS FUNCTION
# -----------------------------
def analyse(name, wps):
    print("\n==============================")
    print(f"   {name}")
    print("==============================")

    # Convert to XY
    wps_xy = [lonlat_to_xy_m((lon,lat), origin) for lon,lat,_ in wps]

    # Headings
    headings = []
    for i in range(len(wps_xy)-1):
        x1,y1 = wps_xy[i]
        x2,y2 = wps_xy[i+1]
        headings.append(math.degrees(math.atan2(y2-y1, x2-x1)))

    # Rasterise
    #grid, bounds = rasterise_coverage(wps_xy, headings, resolution=1.0)

    # Visualise
    plot_heatmap(grid, bounds, title=f"{name} Coverage")
    plot_gaps_and_overlaps(grid, bounds)

    # Stats
    print(f"Waypoints: {len(wps)}")
    print(f"Turns: {count_turns(wps)}")
    print(f"Distance: {total_distance(wps_xy)/1000:.2f} km")

# -----------------------------
# RUN ALL PATTERNS
# -----------------------------
analyse("Parallel Track", parallel_track(SEARCH_AREA, cfg))
analyse("Creeping Line", creeping_line(SEARCH_AREA, cfg))
analyse("Expanding Square", expanding_square(SEARCH_AREA[0], cfg))
analyse("Sector Search", sector_search(SEARCH_AREA[0], cfg))
analyse("Shoreline Search", shoreline_search(SEARCH_AREA, cfg))
analyse("Barrier Search", barrier_search(SEARCH_AREA[0], SEARCH_AREA[2], cfg))
analyse("Track Line", track_line(SEARCH_AREA[0], SEARCH_AREA[2], cfg))
