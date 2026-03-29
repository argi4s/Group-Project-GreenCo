   Adding new waypoint generators (alternative to lawnmower)

Adding new waypoint generators (alternative to lawnmower)

in lawnmower.py

```python
def plan_new_pattern(
    polygon: List[LonLat],
    altitude_m: float,
    origin_lonlat: LonLat,
    **kwargs
) -> List[Waypoint]:
    """
    Generate waypoints for new search pattern
    
    Args:
        polygon: Search area boundary (lon, lat)
        altitude_m: Flight altitude
        origin_lonlat: Reference point for local coordinates
    
    Returns:
        List of Waypoint objects
    """
    # Convert to local XY
    outer_xy = [lonlat_to_xy_m(p, origin_lonlat) for p in polygon]
    
    # Generate pattern logic
    waypoints_xy = []
    
    # Your pattern generation code here
    # Example: spiral pattern, star pattern, etc.
    
    # Convert back to lon/lat
    waypoints = []
    for x, y in waypoints_xy:
        lon, lat = xy_to_lonlat((x, y), origin_lonlat)
        waypoints.append(Waypoint(lon=lon, lat=lat, alt_m=altitude_m))
    
    return waypoints
```

then in PI_State_Machine.py

```python
elif event == "new_pattern":
    # Generate waypoints
    origin_lon, origin_lat = drone.position[:2]
    altitude = drone.position[2]
    
    waypoints = plan_new_pattern(
        search_polygon,
        altitude,
        origin_lonlat=(origin_lon, origin_lat)
    )
    
    current_state = STATE_NEW_PATTERN
    
    # Fly the pattern
    for wp in waypoints:
        if current_state != STATE_NEW_PATTERN:
            break
        drone.move_to_wp(wp.lon, wp.lat, wp.alt_m)
```