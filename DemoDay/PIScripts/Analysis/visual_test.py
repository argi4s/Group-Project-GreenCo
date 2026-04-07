from search_patterns import parallel_track, SearchPatternConfig
from search_visualisation import rasterise_coverage, plot_heatmap, plot_gaps_and_overlaps
from lawnmower import lonlat_to_xy_m

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
    heading_deg=0
)

# Generate pattern
wps = parallel_track(SEARCH_AREA, cfg)

# Convert to XY
origin = SEARCH_AREA[0]
wps_xy = [lonlat_to_xy_m((lon,lat), origin) for lon,lat,_ in wps]

# Compute headings between points
import math
headings = []
for i in range(len(wps_xy)-1):
    x1,y1 = wps_xy[i]
    x2,y2 = wps_xy[i+1]
    hdg = math.degrees(math.atan2(y2-y1, x2-x1))
    headings.append(hdg)

# Rasterise
grid, bounds = rasterise_coverage(wps_xy, headings, resolution=1.0)

# Visualise
plot_heatmap(grid, bounds, title="Parallel Track Coverage")
plot_gaps_and_overlaps(grid, bounds)
