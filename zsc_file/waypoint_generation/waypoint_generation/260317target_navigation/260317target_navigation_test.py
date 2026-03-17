import time

from target_navigation_260317 import (
    PixelTarget,
    GeoPoint,
    VehiclePose,
    CameraModel,
    TargetClosedLoopFSM,
    build_default_nadir_pixel_to_global,
    ne_offset_m_between_points,
    offset_latlon_m,
    ne_to_body,
    distance_vehicle_to_goal_m,
)


class FakeVehicle:
    def __init__(self, lat: float, lon: float, alt_m: float, yaw_deg: float):
        self.lat = lat
        self.lon = lon
        self.alt_m = alt_m
        self.yaw_deg = yaw_deg
        self.goal = None

    def get_pose(self) -> VehiclePose:
        return VehiclePose(
            lat=self.lat,
            lon=self.lon,
            alt_m=self.alt_m,
            yaw_deg=self.yaw_deg,
        )

    def navigate_to(self, goal: GeoPoint) -> None:
        self.goal = goal
        print(f"[NAVIGATE_TO] lat={goal.lat:.8f}, lon={goal.lon:.8f}, alt={goal.alt_m}")

    def stop(self) -> None:
        print("[STOP] vehicle stop called")
        self.goal = None

    def update(self, dt: float, speed_mps: float = 2.0) -> None:
        if self.goal is None:
            return

        cur = GeoPoint(self.lat, self.lon, self.alt_m)
        dist_m = distance_vehicle_to_goal_m(self.get_pose(), self.goal)
        if dist_m < 1e-6:
            return

        step_m = min(speed_mps * dt, dist_m)
        north_m, east_m = ne_offset_m_between_points(cur, self.goal)
        scale = step_m / max(dist_m, 1e-9)
        dn = north_m * scale
        de = east_m * scale
        self.lat, self.lon = offset_latlon_m(self.lat, self.lon, dn, de)


class FakeDetector:
    def __init__(self, vehicle, world_target, camera, get_height_agl_fn):
        self.vehicle = vehicle
        self.world_target = world_target
        self.camera = camera
        self.get_height_agl_fn = get_height_agl_fn

    def __call__(self, frame):
        pose = self.vehicle.get_pose()
        agl_m = float(self.get_height_agl_fn())
        if agl_m <= 0.0:
            return None

        origin = GeoPoint(pose.lat, pose.lon, pose.alt_m)
        north_m, east_m = ne_offset_m_between_points(origin, self.world_target)
        yaw_total_deg = pose.yaw_deg + self.camera.camera_yaw_deg
        forward_m, right_m = ne_to_body(north_m, east_m, yaw_total_deg)

        du = (right_m / agl_m) * self.camera.fx
        dv = -(forward_m / agl_m) * self.camera.fy
        u = self.camera.cx + du
        v = self.camera.cy + dv

        if 0.0 <= u < self.camera.width and 0.0 <= v < self.camera.height:
            return PixelTarget(u=u, v=v, confidence=0.99, label="target")
        return None


def main():
    lat0 = 51.4500
    lon0 = -2.6000
    alt_m = 12.0
    yaw_deg = 0.0

    vehicle = FakeVehicle(lat=lat0, lon=lon0, alt_m=alt_m, yaw_deg=yaw_deg)

    tgt_lat, tgt_lon = offset_latlon_m(lat0, lon0, north_m=3.0, east_m=2.0)
    world_target = GeoPoint(lat=tgt_lat, lon=tgt_lon, alt_m=alt_m)

    def get_height_agl():
        return 12.0

    camera = CameraModel.from_fov(
        width=1280,
        height=720,
        hfov_deg=70.0,
        vfov_deg=43.0,
        camera_yaw_deg=0.0,
    )

    detector = FakeDetector(
        vehicle=vehicle,
        world_target=world_target,
        camera=camera,
        get_height_agl_fn=get_height_agl,
    )

    pixel_to_global = build_default_nadir_pixel_to_global(
        image_width=1280,
        image_height=720,
        hfov_deg=70.0,
        vfov_deg=43.0,
        get_height_agl_fn=get_height_agl,
        camera_yaw_deg=0.0,
    )

    fsm = TargetClosedLoopFSM(
        detect_target_fn=detector,
        get_vehicle_pose_fn=vehicle.get_pose,
        pixel_to_global_fn=pixel_to_global,
        navigate_to_fn=vehicle.navigate_to,
        stop_navigation_fn=vehicle.stop,
        arrival_radius_m=0.8,
        nav_reissue_period_s=0.2,
        target_lost_timeout_s=1.0,
        continue_to_last_goal_on_loss=True,
    )

    print("=== Offline FSM test start ===")
    print(f"World target: lat={world_target.lat:.8f}, lon={world_target.lon:.8f}")

    dt = 0.1
    max_steps = 300

    for k in range(max_steps):
        vehicle.update(dt=dt, speed_mps=2.0)
        status = fsm.step(frame=None)

        dist = status.distance_to_goal_m
        dist_str = "None" if dist is None else f"{dist:.2f}"
        print(
            f"[{k:03d}] state={status.state:<12} sub={str(status.substate):<12} "
            f"visible={status.target_visible!s:<5} dist={dist_str:<6} msg={status.message}"
        )

        if status.arrived:
            print("=== SUCCESS: Arrived and stop command was issued ===")
            break

        time.sleep(0.02)
    else:
        print("=== TEST ENDED: did not arrive within max_steps ===")

    print(f"Final vehicle pose: lat={vehicle.lat:.8f}, lon={vehicle.lon:.8f}")


if __name__ == "__main__":
    main()