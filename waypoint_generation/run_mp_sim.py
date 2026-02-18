# run_mp_sim.py
import argparse
from flight_function import LatLon, CameraFootprintModel, MissionController, haversine_m
import sys
sys.path.append(r"C:\path\to\folder\that\contains\flight_function.py")
from flight_function import LatLon, CameraFootprintModel, MissionController



# ---- HARD-CODE from your txt ----
# TOL: 51°25'24.5"N 2°40'17.6"W -> decimal approx:
TOL = LatLon(51.423472, -2.671556)

FLIGHT_AREA = [
    LatLon(51.423561, -2.671297),
    LatLon(51.422842, -2.670066),
    LatLon(51.424173, -2.668757),
    LatLon(51.423444, -2.668178),
]

def reorder_waypoints_start_near_tol(tol: LatLon, wps):
    if not wps:
        return wps
    best_i = min(range(len(wps)), key=lambda i: haversine_m(tol, LatLon(wps[i].lat, wps[i].lon)))
    return wps[best_i:] + wps[:best_i]

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mavlink", default="tcp:127.0.0.1:5760",
                    help="Mission Planner SITL often uses tcp:5760. If yours is UDP, try udp:127.0.0.1:14550")
    ap.add_argument("--alt", type=float, default=30.0)
    ap.add_argument("--overlap-along", type=float, default=0.7)
    ap.add_argument("--overlap-across", type=float, default=0.6)
    ap.add_argument("--heading", type=float, default=None)
    ap.add_argument("--speed", type=float, default=4.0)
    args = ap.parse_args()

    if args.alt > 50.0:
        raise ValueError("alt must be <= 50m")

    camera = CameraFootprintModel(ref_alt_m=50.0, ref_right_m=52.4, ref_forward_m=39.3)
    mc = MissionController(flight_area_poly=FLIGHT_AREA, camera=camera)

    # 1) connect + rx + watchdog (your original function)
    mc.connect_and_start(args.mavlink)

    # 2) set HOME to TOL so RTL returns to TOL
    mc.drone.set_home(TOL.lat, TOL.lon, 0.0)

    # 3) takeoff
    mc.arm_takeoff(rel_alt_m=args.alt)

    # 4) plan lawnmower (your original function)
    wps = mc.plan_lawnmower(
        alt_m=args.alt,
        overlap_along=args.overlap_along,
        overlap_across=args.overlap_across,
        heading_deg=args.heading,
        speed_mps=args.speed,
        yaw_deg=args.heading,  # optional: keep yaw aligned
    )
    wps = reorder_waypoints_start_near_tol(TOL, wps)

    # 5) fly lawnmower (your original function)
    mc.run_waypoints(wps, arrive_radius_m=3.0, timeout_each_s=120.0)

    # 6) return home + stop
    mc.rth()
    mc.stop()

if __name__ == "__main__":
    main()
