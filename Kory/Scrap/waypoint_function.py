#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Feb 14 11:06:59 2026

@author: Kory Campbell
"""

def export_to_mission_planner_with_home_and_terrain(points_latlon, home_latlon,
                                                   elevation_func=None,
                                                   filename="grid_waypoints.waypoints",
                                                   base_altitude=100,
                                                   altitude_offset=20):
    """
    Export waypoints with terrain-based altitudes.
    
    Parameters:
    - elevation_func: function(lat, lon) -> elevation in meters
    - base_altitude: minimum altitude above ground level (AGL)
    - altitude_offset: additional safety margin above terrain
    """
    home_lat, home_lon = home_latlon

    with open(filename, "w") as f:
        f.write("QGC WPL 110\n")

        # Home waypoint
        if elevation_func:
            home_elevation = elevation_func(home_lat, home_lon)
            home_altitude = home_elevation + base_altitude + altitude_offset
        else:
            home_altitude = base_altitude + altitude_offset
            
        f.write(f"0\t1\t0\t16\t0\t0\t0\t0\t{home_lat:.7f}\t{home_lon:.7f}\t{home_altitude:.2f}\t1\n")

        # Grid waypoints (skip first and last points which are home)
        waypoint_index = 1
        for i, (lat, lon) in enumerate(points_latlon):
            # Skip if this is the home point (first or last in route)
            if (np.isclose(lat, home_lat) and np.isclose(lon, home_lon)):
                continue
                
            if elevation_func:
                terrain_elevation = elevation_func(lat, lon)
                altitude = terrain_elevation + base_altitude + altitude_offset
            else:
                altitude = base_altitude + altitude_offset
            
            f.write(
                f"{waypoint_index}\t0\t3\t16\t0\t0\t0\t0\t"
                f"{lat:.7f}\t{lon:.7f}\t{altitude:.2f}\t1\n"
            )
            waypoint_index += 1

    print(f"✔ Exported {waypoint_index} waypoints with terrain elevations to {filename}")
    return filename