from search_patterns import (
    parallel_track,
    creeping_line,
    expanding_square,
    sector_search,
    track_line,
    shoreline_search,
    barrier_search,
    SearchPatternConfig
)

from search_analysis import (
    total_distance,
    estimated_time,
    coverage_factor,
    probability_of_detection
)

SEARCH_AREA = [
    (-2.670948345438704, 51.42326956502679),
    (-2.670045428650557, 51.42287025017865),
    (-2.668169295906676, 51.42336622593724),
    (-2.668809768621569, 51.42421477437771),
    (-2.671277780473196, 51.42354069739116),
    (-2.670948345438704, 51.42326956502679)
]

ALT = 20.0
SPEED = 2.5  # m/s
SWEEP_WIDTH = 16.2
TRACK_SPACING = 12.0

cfg = SearchPatternConfig(
    altitude_m=ALT,
    track_spacing_m=TRACK_SPACING,
    sweep_width_m=SWEEP_WIDTH,
    origin_lonlat=SEARCH_AREA[0],
    heading_deg=0,
    legs=12
)

patterns = {
    "Parallel Track": lambda: parallel_track(SEARCH_AREA, cfg),
    "Creeping Line": lambda: creeping_line(SEARCH_AREA, cfg),
    "Expanding Square": lambda: expanding_square(cfg.origin_lonlat, cfg),
    "Sector Search": lambda: sector_search(cfg.origin_lonlat, cfg),
    "Track Line": lambda: track_line(SEARCH_AREA[0], SEARCH_AREA[2], cfg),
    "Shoreline": lambda: shoreline_search(SEARCH_AREA, cfg),
    "Barrier": lambda: barrier_search(SEARCH_AREA[0], SEARCH_AREA[3], cfg),
}

print("\n=== SEARCH PATTERN ANALYSIS ===\n")

for name, fn in patterns.items():
    wps = fn()
    dist = total_distance(wps)
    time_s = estimated_time(dist, SPEED)
    C = coverage_factor(SWEEP_WIDTH, TRACK_SPACING)
    POD = probability_of_detection(C)

    print(f"{name}:")
    print(f"  Waypoints: {len(wps)}")
    print(f"  Distance: {dist/1000:.2f} km")
    print(f"  Time: {time_s/60:.1f} min")
    print(f"  Coverage Factor: {C:.2f}")
    print(f"  POD: {POD*100:.1f}%")
    print("")
