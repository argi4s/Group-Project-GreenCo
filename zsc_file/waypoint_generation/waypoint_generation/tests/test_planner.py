from waypoint_generation.flight_function import LatLon, CameraFootprintModel, LawnmowerPlanner


def test_planner_scales_with_altitude():
    flight_area = [
        LatLon(51.423561, -2.671297),
        LatLon(51.422842, -2.670066),
        LatLon(51.424173, -2.668757),
        LatLon(51.423444, -2.668178),
    ]
    camera = CameraFootprintModel(ref_alt_m=50.0, ref_right_m=52.4, ref_forward_m=39.3)
    planner = LawnmowerPlanner(camera)

    wps_20 = planner.plan(flight_area, alt_m=20, overlap_along=0.7, overlap_across=0.6)
    wps_40 = planner.plan(flight_area, alt_m=40, overlap_along=0.7, overlap_across=0.6)

    # 高度更高 -> footprint更大 -> 航线更稀疏 -> 航点数通常更少（一般应 wps_40 < wps_20）
    assert len(wps_40) < len(wps_20)
