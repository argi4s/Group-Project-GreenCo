from __future__ import annotations

import math
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Any, Callable, Optional


@dataclass
class PixelTarget:
    """Detection result in image coordinates."""
    u: float
    v: float
    confidence: float = 1.0
    label: str = "target"
    bbox: Optional[tuple[float, float, float, float]] = None


@dataclass
class GeoPoint:
    """Global geographic point."""
    lat: float
    lon: float
    alt_m: Optional[float] = None


@dataclass
class VehiclePose:
    """Vehicle pose/state used by the FSM."""
    lat: float
    lon: float
    alt_m: float
    yaw_deg: float


@dataclass
class CameraModel:
    """Simple pinhole camera model."""
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    camera_yaw_deg: float = 0.0

    @classmethod
    def from_fov(
        cls,
        width: int,
        height: int,
        hfov_deg: float,
        vfov_deg: float,
        camera_yaw_deg: float = 0.0,
    ) -> "CameraModel":
        hfov_rad = math.radians(hfov_deg)
        vfov_rad = math.radians(vfov_deg)
        fx = (width / 2.0) / math.tan(hfov_rad / 2.0)
        fy = (height / 2.0) / math.tan(vfov_rad / 2.0)
        cx = width / 2.0
        cy = height / 2.0
        return cls(
            width=width,
            height=height,
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
            camera_yaw_deg=camera_yaw_deg,
        )


@dataclass
class FSMStatus:
    state: str
    substate: Optional[str]
    target_visible: bool
    target_pixel: Optional[PixelTarget]
    target_global: Optional[GeoPoint]
    distance_to_goal_m: Optional[float]
    command_sent: bool
    stop_sent: bool
    arrived: bool
    message: str


class MainState(Enum):
    WAITING_FOR_TARGET = auto()
    CLOSED_LOOP = auto()
    FINISHED = auto()


class ClosedLoopSubState(Enum):
    ACQUIRE_TARGET = auto()
    NAVIGATING = auto()
    STOPPED = auto()


EARTH_RADIUS_M = 6378137.0


def wrap_angle_deg(angle: float) -> float:
    return (angle + 360.0) % 360.0


def _meters_per_deg_lat(lat_deg: float) -> float:
    return 111_320.0


def _meters_per_deg_lon(lat_deg: float) -> float:
    return 111_320.0 * math.cos(math.radians(lat_deg))


def offset_latlon_m(lat_deg: float, lon_deg: float, north_m: float, east_m: float) -> tuple[float, float]:
    new_lat = lat_deg + north_m / _meters_per_deg_lat(lat_deg)
    new_lon = lon_deg + east_m / _meters_per_deg_lon(lat_deg)
    return new_lat, new_lon


def ne_offset_m_between_points(origin: GeoPoint, target: GeoPoint) -> tuple[float, float]:
    """Returns (north_m, east_m) from origin to target using small-area approximation."""
    north_m = (target.lat - origin.lat) * _meters_per_deg_lat(origin.lat)
    east_m = (target.lon - origin.lon) * _meters_per_deg_lon(origin.lat)
    return north_m, east_m


def haversine_distance_m(a: GeoPoint, b: GeoPoint) -> float:
    lat1 = math.radians(a.lat)
    lon1 = math.radians(a.lon)
    lat2 = math.radians(b.lat)
    lon2 = math.radians(b.lon)

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    s = (
        math.sin(dlat / 2.0) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    )
    c = 2.0 * math.atan2(math.sqrt(s), math.sqrt(max(1e-12, 1.0 - s)))
    return EARTH_RADIUS_M * c


def body_to_ne(forward_m: float, right_m: float, yaw_deg: float) -> tuple[float, float]:
    """
    Body frame -> N/E
    yaw_deg:
        0   = facing north
        90  = facing east
        180 = south
        270 = west
    """
    yaw = math.radians(wrap_angle_deg(yaw_deg))
    north = forward_m * math.cos(yaw) - right_m * math.sin(yaw)
    east = forward_m * math.sin(yaw) + right_m * math.cos(yaw)
    return north, east


def ne_to_body(north_m: float, east_m: float, yaw_deg: float) -> tuple[float, float]:
    """N/E -> body frame, returns (forward_m, right_m)."""
    yaw = math.radians(wrap_angle_deg(yaw_deg))
    forward = north_m * math.cos(yaw) + east_m * math.sin(yaw)
    right = -north_m * math.sin(yaw) + east_m * math.cos(yaw)
    return forward, right


def distance_vehicle_to_goal_m(vehicle_pose: VehiclePose, goal: GeoPoint) -> float:
    cur = GeoPoint(lat=vehicle_pose.lat, lon=vehicle_pose.lon, alt_m=vehicle_pose.alt_m)
    return haversine_distance_m(cur, goal)


class NadirPixelToGlobalConverter:
    """
    Default converter for:
    - nadir/downward-looking camera
    - flat-ground assumption
    - AGL obtained from external function interface
    """

    def __init__(
        self,
        camera: CameraModel,
        get_height_agl_fn: Callable[[], float],
    ):
        self.camera = camera
        self.get_height_agl_fn = get_height_agl_fn

    def __call__(self, detection: PixelTarget, vehicle_pose: VehiclePose) -> Optional[GeoPoint]:
        agl_m = float(self.get_height_agl_fn())
        if agl_m <= 0.0:
            return None

        du = detection.u - self.camera.cx
        dv = detection.v - self.camera.cy

        right_m = (du / self.camera.fx) * agl_m
        forward_m = -(dv / self.camera.fy) * agl_m

        yaw_total_deg = vehicle_pose.yaw_deg + self.camera.camera_yaw_deg
        north_m, east_m = body_to_ne(forward_m, right_m, yaw_total_deg)

        tgt_lat, tgt_lon = offset_latlon_m(
            vehicle_pose.lat,
            vehicle_pose.lon,
            north_m=north_m,
            east_m=east_m,
        )

        return GeoPoint(lat=tgt_lat, lon=tgt_lon, alt_m=vehicle_pose.alt_m)


def build_default_nadir_pixel_to_global(
    image_width: int,
    image_height: int,
    hfov_deg: float,
    vfov_deg: float,
    get_height_agl_fn: Callable[[], float],
    camera_yaw_deg: float = 0.0,
) -> NadirPixelToGlobalConverter:
    camera = CameraModel.from_fov(
        width=image_width,
        height=image_height,
        hfov_deg=hfov_deg,
        vfov_deg=vfov_deg,
        camera_yaw_deg=camera_yaw_deg,
    )
    return NadirPixelToGlobalConverter(camera=camera, get_height_agl_fn=get_height_agl_fn)


class TargetClosedLoopFSM:
    """
    Importable closed-loop target navigation FSM.

    External interfaces to inject:
    - detect_target_fn(frame) -> Optional[PixelTarget]
    - get_vehicle_pose_fn() -> VehiclePose
    - pixel_to_global_fn(detection, pose) -> Optional[GeoPoint]
    - navigate_to_fn(goal) -> None
    - stop_navigation_fn() -> None
    """

    def __init__(
        self,
        detect_target_fn: Callable[[Any], Optional[PixelTarget]],
        get_vehicle_pose_fn: Callable[[], VehiclePose],
        pixel_to_global_fn: Callable[[PixelTarget, VehiclePose], Optional[GeoPoint]],
        navigate_to_fn: Callable[[GeoPoint], None],
        stop_navigation_fn: Callable[[], None],
        arrival_radius_m: float = 1.0,
        nav_reissue_period_s: float = 0.2,
        target_lost_timeout_s: float = 1.0,
        continue_to_last_goal_on_loss: bool = True,
    ):
        self.detect_target_fn = detect_target_fn
        self.get_vehicle_pose_fn = get_vehicle_pose_fn
        self.pixel_to_global_fn = pixel_to_global_fn
        self.navigate_to_fn = navigate_to_fn
        self.stop_navigation_fn = stop_navigation_fn

        self.arrival_radius_m = float(arrival_radius_m)
        self.nav_reissue_period_s = float(nav_reissue_period_s)
        self.target_lost_timeout_s = float(target_lost_timeout_s)
        self.continue_to_last_goal_on_loss = bool(continue_to_last_goal_on_loss)

        self.main_state = MainState.WAITING_FOR_TARGET
        self.substate: Optional[ClosedLoopSubState] = None

        self.current_target_pixel: Optional[PixelTarget] = None
        self.current_target_global: Optional[GeoPoint] = None

        self.last_seen_time: Optional[float] = None
        self.last_nav_cmd_time: Optional[float] = None

    def reset(self) -> None:
        self.main_state = MainState.WAITING_FOR_TARGET
        self.substate = None
        self.current_target_pixel = None
        self.current_target_global = None
        self.last_seen_time = None
        self.last_nav_cmd_time = None

    def is_finished(self) -> bool:
        return self.main_state == MainState.FINISHED

    def step(
        self,
        frame: Any = None,
        *,
        external_detection: Optional[PixelTarget] = None,
        now: Optional[float] = None,
    ) -> FSMStatus:
        now = time.monotonic() if now is None else now
        pose = self.get_vehicle_pose_fn()

        detection = external_detection if external_detection is not None else self.detect_target_fn(frame)

        if detection is not None:
            self.current_target_pixel = detection
            self.last_seen_time = now

            goal = self.pixel_to_global_fn(detection, pose)
            if goal is not None:
                self.current_target_global = goal

        command_sent = False
        stop_sent = False
        arrived = False
        message = ""

        if self.main_state == MainState.WAITING_FOR_TARGET:
            if detection is not None and self.current_target_global is not None:
                self.main_state = MainState.CLOSED_LOOP
                self.substate = ClosedLoopSubState.ACQUIRE_TARGET
                message = "Target detected. Entering closed-loop control."
            else:
                message = "Waiting for target."

        if self.main_state == MainState.CLOSED_LOOP:
            if self.substate is None:
                self.substate = ClosedLoopSubState.ACQUIRE_TARGET

            if self.substate == ClosedLoopSubState.ACQUIRE_TARGET:
                if self.current_target_global is not None:
                    self.navigate_to_fn(self.current_target_global)
                    self.last_nav_cmd_time = now
                    command_sent = True
                    self.substate = ClosedLoopSubState.NAVIGATING
                    message = "Initial navigate command sent."
                else:
                    if self._target_lost(now):
                        self._back_to_waiting()
                        message = "Target lost before goal was computed."
                    else:
                        message = "Acquiring target global position..."

            elif self.substate == ClosedLoopSubState.NAVIGATING:
                if self.current_target_global is not None:
                    if (
                        self.last_nav_cmd_time is None
                        or (now - self.last_nav_cmd_time) >= self.nav_reissue_period_s
                    ):
                        self.navigate_to_fn(self.current_target_global)
                        self.last_nav_cmd_time = now
                        command_sent = True
                        message = "Closed-loop navigate update sent."

                if self._target_lost(now):
                    if self.current_target_global is None:
                        self._back_to_waiting()
                        message = "Target lost and no valid goal exists."
                    elif not self.continue_to_last_goal_on_loss:
                        self.stop_navigation_fn()
                        stop_sent = True
                        self._back_to_waiting()
                        message = "Target lost. Stopped and returned to waiting state."
                    else:
                        if not message:
                            message = "Target lost; continuing toward last known goal."

                if self.current_target_global is not None:
                    dist_m = distance_vehicle_to_goal_m(pose, self.current_target_global)
                    if dist_m <= self.arrival_radius_m:
                        self.stop_navigation_fn()
                        stop_sent = True
                        arrived = True
                        self.substate = ClosedLoopSubState.STOPPED
                        self.main_state = MainState.FINISHED
                        message = f"Arrived at target within {dist_m:.2f} m. Stop command sent."

                if not message:
                    message = "Navigating toward target."

            elif self.substate == ClosedLoopSubState.STOPPED:
                message = "Vehicle stopped at target."

        elif self.main_state == MainState.FINISHED:
            if self.substate is None:
                self.substate = ClosedLoopSubState.STOPPED
            message = "FSM finished."

        distance_m = None
        if self.current_target_global is not None:
            distance_m = distance_vehicle_to_goal_m(pose, self.current_target_global)

        return FSMStatus(
            state=self.main_state.name,
            substate=self.substate.name if self.substate is not None else None,
            target_visible=(detection is not None),
            target_pixel=self.current_target_pixel,
            target_global=self.current_target_global,
            distance_to_goal_m=distance_m,
            command_sent=command_sent,
            stop_sent=stop_sent,
            arrived=arrived,
            message=message,
        )

    def _target_lost(self, now: float) -> bool:
        if self.last_seen_time is None:
            return True
        return (now - self.last_seen_time) > self.target_lost_timeout_s

    def _back_to_waiting(self) -> None:
        self.main_state = MainState.WAITING_FOR_TARGET
        self.substate = None
        self.current_target_pixel = None
        self.current_target_global = None
        self.last_seen_time = None
        self.last_nav_cmd_time = None