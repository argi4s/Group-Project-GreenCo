import sys
from lawnmower import plan_lawnmower, CameraFootprintRef

# Define the lawn polygon
lawn_polygon = [
    (-2.670948345438704, 51.42326956502679),
    (-2.670045428650557, 51.42287025017865),
    (-2.668169295906676, 51.42336622593724),
    (-2.668809768621569, 51.42421477437771),
    (-2.671277780473196, 51.42354069739116),
    (-2.670948345438704, 51.42326956502679)
]

# Define origin and altitude (using arbitrary values, replace with real drone position if needed)
origin = (51.42326956502679, -2.670948345438704)  # Example lat/lon (should come from drone position)
altitude_m = 10  # Example altitude (change to your drone's current altitude if needed)

# Camera reference parameters (based on your setup)
camera_ref = CameraFootprintRef(
    ref_alt_m=50,
    ref_width_m=52.4,
    ref_height_m=39.3
)

# Overlap settings
side_overlap = 0.3  # 30% side overlap
forward_overlap = 0.7  # 70% forward overlap

# Call the lawnmower waypoint planner
waypoints = plan_lawnmower(
    search_polygon=lawn_polygon,
    altitude_m=altitude_m,
    origin_latlon=origin,
    camera_ref=camera_ref,
    side_overlap=side_overlap,
    forward_overlap=forward_overlap
)

# Output the generated waypoints
print(f"Generated {len(waypoints)} waypoints:")
for i, wp in enumerate(waypoints):
    print(f"WP {i+1}: Latitude={wp.lat:.7f}, Longitude={wp.lon:.7f}, Altitude={wp.alt_m:.2f}m")