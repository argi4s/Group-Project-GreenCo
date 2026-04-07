# run_all_patterns.py

import math
import os

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

from search_visualisation import (
    rasterise_coverage,
    plot_heatmap,
    plot_gaps_and_overlaps
)

from lawnmower import lonlat_to_xy_m

# -----------------------------
# Search area (your real polygon)
# -----------------------------
SEARCH_AREA = [
    (-2.670948345438704, 51.42326956502679),
    (-2.670045428650557, 51.42287025017865),
    (-2.668169295906676, 51.42336622593724),
    (-2.668809768621569, 51.42421477437771),
    (-2.671277780473196, 51.42354069739116),
]

# -----------------------------
# Camera + flight parameters
# -----------------------------
ALT = 20.0
SPEED = 2.5
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

# -----------------------------
# Output directory
# -----------------------------
os.makedirs("pattern_outputs", exist_ok=True)

# -----------------------------
# Patterns to run
# -----------------------------
patterns = {
    "Parallel_Track": lambda: parallel_track(SEARCH_AREA, cfg),
    "Creeping_Line": lambda: creeping_line(SEARCH_AREA, cfg),
    "Expanding_Square": lambda: expanding_square(cfg.origin_lonlat, cfg),
    "Sector_Search": lambda: sector_search(cfg.origin_lonlat, cfg),
    "Track_Line": lambda: track_line(SEARCH_AREA[0], SEARCH_AREA[2], cfg),
    "Shoreline": lambda: shoreline_search(SEARCH_AREA, cfg),
    "Barrier": lambda: barrier_search(SEARCH_AREA[0], SEARCH_AREA[3], cfg),
}

# -----------------------------
# Run all patterns
# -----------------------------
for name, fn in patterns.items():
    print(f"\n=== Running {name} ===")

    # Generate waypoints
    wps = fn()

    # Convert to XY
    origin = SEARCH_AREA[0]
    wps_xy = [lonlat_to_xy_m((lon, lat), origin) for lon, lat, _ in wps]

    # Compute headings
    headings = []
    for i in range(len(wps_xy) - 1):
        x1, y1 = wps_xy[i]
        x2, y2 = wps_xy[i + 1]
        hdg = math.degrees(math.atan2(y2 - y1, x2 - x1))
        headings.append(hdg)

    # Rasterise coverage
    grid, bounds = rasterise_coverage(wps_xy, headings, resolution=1.0)

    # Save heatmap
    heatmap_path = f"pattern_outputs/{name}_heatmap.png"
    plot_heatmap(grid, bounds, title=f"{name} Coverage Heatmap")
    import matplotlib.pyplot as plt
    plt.savefig(heatmap_path)
    plt.close()
    print(f"Saved: {heatmap_path}")

    # Save gaps + overlaps
    gaps = (grid == 0)
    overlaps = (grid > 1)

    plt.figure(figsize=(10, 8))
    plt.imshow(gaps, extent=bounds, origin='lower', cmap='Blues')
    plt.title(f"{name} - Gaps")
    plt.savefig(f"pattern_outputs/{name}_gaps.png")
    plt.close()

    plt.figure(figsize=(10, 8))
    plt.imshow(overlaps, extent=bounds, origin='lower', cmap='Greens')
    plt.title(f"{name} - Overlaps")
    plt.savefig(f"pattern_outputs/{name}_overlaps.png")
    plt.close()

    print(f"Saved: pattern_outputs/{name}_gaps.png")
    print(f"Saved: pattern_outputs/{name}_overlaps.png")

print("\nAll patterns processed.")
