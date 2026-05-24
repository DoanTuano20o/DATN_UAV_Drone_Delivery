from __future__ import annotations

import json
from math import asin, cos, radians, sin, sqrt
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .state_machine import MissionState


def haversine_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = 6371000.0
    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)
    a = (
        sin(dlat / 2) ** 2
        + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon / 2) ** 2
    )
    return 2 * r * asin(sqrt(a))


def offset_latlon_m(lat_deg: float, lon_deg: float, north_m: float, east_m: float) -> tuple[float, float]:
    dlat = north_m / 111320.0
    dlon = east_m / (111320.0 * max(cos(radians(lat_deg)), 1e-6))
    return lat_deg + dlat, lon_deg + dlon


class MissionManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("mission_manager_node")

        self.declare_parameter("takeoff_alt_m", 5.0)
        self.declare_parameter("hold_before_goto_s", 7.0)
        self.declare_parameter("goto_speed_mps", 1.0)

        self.declare_parameter("marker_ref_lat", 10.8449699)
        self.declare_parameter("marker_ref_lon", 106.7962627)

        self.declare_parameter("search_enter_radius_m", 3.0)

        self.declare_parameter("search_pattern_radius_m", 2.0)
        self.declare_parameter("search_pattern_radius_step_m", 1.0)
        self.declare_parameter("search_pattern_loops_max", 3)
        self.declare_parameter("search_waypoint_reached_m", 1.0)
        self.declare_parameter("search_step_hold_s", 3.0)
        self.declare_parameter("search_waypoint_timeout_s", 8.0)
        self.declare_parameter("search_waypoint_speed_mps", 0.3)

        self.declare_parameter("alt_reached_tol_m", 0.30)

        self.declare_parameter("big_marker_id", 150)
        self.declare_parameter("small_marker_id", 40)
        self.declare_parameter("small_marker_enable_alt_m", 5.0)

        self.declare_parameter("marker_lost_timeout_s", 1.2)
        self.declare_parameter("search_timeout_s", 60.0)

        self.declare_parameter("big_align_deadband_px", 25.0)
        self.declare_parameter("big_lock_tolerance_px", 45.0)
        self.declare_parameter("big_lock_hold_s", 1.2)
        self.declare_parameter("big_fast_error_px", 120.0)
        self.declare_parameter("big_slow_error_px", 35.0)
        self.declare_parameter("big_max_body_speed_mps", 0.09)
        self.declare_parameter("big_min_body_speed_mps", 0.02)

        self.declare_parameter("vx_from_err_y_gain", -0.004)
        self.declare_parameter("vy_from_err_x_gain", 0.002)

        self.declare_parameter("enable_descend_after_lock_big", True)
        self.declare_parameter("descend_to_small_alt_m", 3.0)
        self.declare_parameter("descend_speed_mps", 0.10)
        self.declare_parameter("descend_alt_reached_tol_m", 0.15)
        self.declare_parameter("descend_recenter_tolerance_px", 60.0)

        self.declare_parameter("small_search_timeout_s", 8.0)
        self.declare_parameter("small_align_deadband_px", 15.0)
        self.declare_parameter("small_lock_tolerance_px", 20.0)
        self.declare_parameter("small_lock_hold_s", 1.0)
        self.declare_parameter("small_vx_from_err_y_gain", -0.003)
        self.declare_parameter("small_vy_from_err_x_gain", 0.002)
        self.declare_parameter("small_max_body_speed_mps", 0.05)
        self.declare_parameter("small_min_body_speed_mps", 0.01)

        self.declare_parameter("enable_land_after_lock_small", True)
        self.declare_parameter("land_on_small_alt_m", 1.0)
        self.declare_parameter("land_use_aruco_z", True)
        self.declare_parameter("land_on_small_aruco_z_m", 0.90)
        self.declare_parameter("land_aruco_z_max_err_px", 120.0)
        self.declare_parameter("land_descend_speed_mps", 0.06)
        self.declare_parameter("land_recenter_tolerance_px", 120.0)
        self.declare_parameter("land_final_tolerance_px", 80.0)
        self.declare_parameter("land_small_hold_s", 0.3)

        # New flow: drop payload, then command RTL.
        self.declare_parameter("enable_drop_after_lock_small", True)
        self.declare_parameter("drop_alt_m", 1.0)
        self.declare_parameter("drop_use_aruco_z", True)
        self.declare_parameter("drop_aruco_z_m", 0.90)
        self.declare_parameter("drop_aruco_z_max_err_px", 120.0)
        self.declare_parameter("drop_descend_speed_mps", 0.06)
        self.declare_parameter("drop_recenter_tolerance_px", 120.0)
        self.declare_parameter("drop_final_tolerance_px", 80.0)
        self.declare_parameter("drop_hold_s", 0.3)
        self.declare_parameter("servo_cmd_topic", "/servo_cmd")
        self.declare_parameter("drop_done_topic", "/drop_done")
        self.declare_parameter("servo_cmd_payload", "DROP")
        self.declare_parameter("drop_timeout_s", 3.0)
        self.declare_parameter("enable_rtl_after_drop", True)
        self.declare_parameter("rtl_cmd_topic", "/flight/cmd_rtl")
        self.declare_parameter("rtl_cmd_payload", "RTL")

        self.declare_parameter("align_yaw_hold_resend_s", 1.0)
        self.declare_parameter("align_yaw_rate_dps", 10.0)

        self.declare_parameter("auto_start_on_guided", True)
        self.declare_parameter("loop_hz", 10.0)

        self.takeoff_alt_m = float(self.get_parameter("takeoff_alt_m").value)
        self.hold_before_goto_s = float(self.get_parameter("hold_before_goto_s").value)
        self.goto_speed_mps = float(self.get_parameter("goto_speed_mps").value)

        self.marker_ref_lat = float(self.get_parameter("marker_ref_lat").value)
        self.marker_ref_lon = float(self.get_parameter("marker_ref_lon").value)

        self.search_enter_radius_m = float(self.get_parameter("search_enter_radius_m").value)

        self.search_pattern_radius_m = float(self.get_parameter("search_pattern_radius_m").value)
        self.search_pattern_radius_step_m = float(self.get_parameter("search_pattern_radius_step_m").value)
        self.search_pattern_loops_max = int(self.get_parameter("search_pattern_loops_max").value)
        self.search_waypoint_reached_m = float(self.get_parameter("search_waypoint_reached_m").value)
        self.search_step_hold_s = float(self.get_parameter("search_step_hold_s").value)
        self.search_waypoint_timeout_s = float(self.get_parameter("search_waypoint_timeout_s").value)
        self.search_waypoint_speed_mps = float(self.get_parameter("search_waypoint_speed_mps").value)

        self.alt_reached_tol_m = float(self.get_parameter("alt_reached_tol_m").value)

        self.big_marker_id = int(self.get_parameter("big_marker_id").value)
        self.small_marker_id = int(self.get_parameter("small_marker_id").value)
        self.small_marker_enable_alt_m = float(self.get_parameter("small_marker_enable_alt_m").value)

        self.marker_lost_timeout_s = float(self.get_parameter("marker_lost_timeout_s").value)
        self.search_timeout_s = float(self.get_parameter("search_timeout_s").value)

        self.big_align_deadband_px = float(self.get_parameter("big_align_deadband_px").value)
        self.big_lock_tolerance_px = float(self.get_parameter("big_lock_tolerance_px").value)
        self.big_lock_hold_s = float(self.get_parameter("big_lock_hold_s").value)
        self.big_fast_error_px = float(self.get_parameter("big_fast_error_px").value)
        self.big_slow_error_px = float(self.get_parameter("big_slow_error_px").value)
        self.big_max_body_speed_mps = float(self.get_parameter("big_max_body_speed_mps").value)
        self.big_min_body_speed_mps = float(self.get_parameter("big_min_body_speed_mps").value)

        self.vx_from_err_y_gain = float(self.get_parameter("vx_from_err_y_gain").value)
        self.vy_from_err_x_gain = float(self.get_parameter("vy_from_err_x_gain").value)

        self.enable_descend_after_lock_big = bool(self.get_parameter("enable_descend_after_lock_big").value)
        self.descend_to_small_alt_m = float(self.get_parameter("descend_to_small_alt_m").value)
        self.descend_speed_mps = float(self.get_parameter("descend_speed_mps").value)
        self.descend_alt_reached_tol_m = float(self.get_parameter("descend_alt_reached_tol_m").value)
        self.descend_recenter_tolerance_px = float(self.get_parameter("descend_recenter_tolerance_px").value)

        self.small_search_timeout_s = float(self.get_parameter("small_search_timeout_s").value)
        self.small_align_deadband_px = float(self.get_parameter("small_align_deadband_px").value)
        self.small_lock_tolerance_px = float(self.get_parameter("small_lock_tolerance_px").value)
        self.small_lock_hold_s = float(self.get_parameter("small_lock_hold_s").value)
        self.small_vx_from_err_y_gain = float(self.get_parameter("small_vx_from_err_y_gain").value)
        self.small_vy_from_err_x_gain = float(self.get_parameter("small_vy_from_err_x_gain").value)
        self.small_max_body_speed_mps = float(self.get_parameter("small_max_body_speed_mps").value)
        self.small_min_body_speed_mps = float(self.get_parameter("small_min_body_speed_mps").value)

        self.enable_land_after_lock_small = bool(self.get_parameter("enable_land_after_lock_small").value)
        self.land_on_small_alt_m = float(self.get_parameter("land_on_small_alt_m").value)
        self.land_use_aruco_z = bool(self.get_parameter("land_use_aruco_z").value)
        self.land_on_small_aruco_z_m = float(self.get_parameter("land_on_small_aruco_z_m").value)
        self.land_aruco_z_max_err_px = float(self.get_parameter("land_aruco_z_max_err_px").value)
        self.land_descend_speed_mps = float(self.get_parameter("land_descend_speed_mps").value)
        self.land_recenter_tolerance_px = float(self.get_parameter("land_recenter_tolerance_px").value)
        self.land_final_tolerance_px = float(self.get_parameter("land_final_tolerance_px").value)
        self.land_small_hold_s = float(self.get_parameter("land_small_hold_s").value)

        self.enable_drop_after_lock_small = bool(self.get_parameter("enable_drop_after_lock_small").value)
        self.drop_alt_m = float(self.get_parameter("drop_alt_m").value)
        self.drop_use_aruco_z = bool(self.get_parameter("drop_use_aruco_z").value)
        self.drop_aruco_z_m = float(self.get_parameter("drop_aruco_z_m").value)
        self.drop_aruco_z_max_err_px = float(self.get_parameter("drop_aruco_z_max_err_px").value)
        self.drop_descend_speed_mps = float(self.get_parameter("drop_descend_speed_mps").value)
        self.drop_recenter_tolerance_px = float(self.get_parameter("drop_recenter_tolerance_px").value)
        self.drop_final_tolerance_px = float(self.get_parameter("drop_final_tolerance_px").value)
        self.drop_hold_s = float(self.get_parameter("drop_hold_s").value)
        self.servo_cmd_topic = str(self.get_parameter("servo_cmd_topic").value)
        self.drop_done_topic = str(self.get_parameter("drop_done_topic").value)
        self.servo_cmd_payload = str(self.get_parameter("servo_cmd_payload").value)
        self.drop_timeout_s = float(self.get_parameter("drop_timeout_s").value)
        self.enable_rtl_after_drop = bool(self.get_parameter("enable_rtl_after_drop").value)
        self.rtl_cmd_topic = str(self.get_parameter("rtl_cmd_topic").value)
        self.rtl_cmd_payload = str(self.get_parameter("rtl_cmd_payload").value)

        self.align_yaw_hold_resend_s = float(self.get_parameter("align_yaw_hold_resend_s").value)
        self.align_yaw_rate_dps = float(self.get_parameter("align_yaw_rate_dps").value)

        self.auto_start_on_guided = bool(self.get_parameter("auto_start_on_guided").value)
        loop_hz = float(self.get_parameter("loop_hz").value)

        self.telemetry: dict[str, Any] = {}
        self.goal: dict[str, Any] | None = None
        self.big_marker: dict[str, Any] | None = None
        self.small_marker: dict[str, Any] | None = None

        self.prev_auto_state: MissionState | None = None
        self.hold_start_time_s: float | None = None
        self.lock_start_time_s: float | None = None
        self.big_last_seen_time_s: float | None = None
        self.small_last_seen_time_s: float | None = None
        self.search_big_start_time_s: float | None = None
        self.search_small_start_time_s: float | None = None
        self.small_search_timeout_warned = False
        self.last_align_log_time_s = 0.0

        self.goto_sent = False
        self.land_sent = False
        self.zero_vel_sent = False

        self.drop_done_received = False
        self.drop_cmd_sent = False
        self.drop_start_time_s: float | None = None
        self.rtl_cmd_sent = False
        self.pending_event: dict[str, Any] | None = None

        self.search_points: list[tuple[float, float]] = []
        self.search_loop_index = 0
        self.search_point_index = 0
        self.search_nav_sent = False
        self.search_point_hold_start_s: float | None = None
        self.search_waypoint_sent_time_s: float | None = None

        self.align_heading_deg: float | None = None
        self.align_yaw_hold_last_sent_s: float | None = None

        self.state = MissionState.WAIT_GUIDED if self.auto_start_on_guided else MissionState.IDLE

        self.create_subscription(String, "/flight/telemetry_json", self.on_telemetry, 10)
        self.create_subscription(String, "/mission/goal_json", self.on_goal, 10)
        self.create_subscription(String, "/mission/control", self.on_control, 10)
        self.create_subscription(String, "/vision/aruco_detections_json", self.on_aruco, 10)

        self.state_pub = self.create_publisher(String, "/mission/state", 10)
        self.takeoff_pub = self.create_publisher(String, "/flight/cmd_takeoff_json", 10)
        self.goto_pub = self.create_publisher(String, "/flight/cmd_goto_global_json", 10)
        self.vel_pub = self.create_publisher(String, "/flight/cmd_vel_body_json", 10)
        self.vel_local_pub = self.create_publisher(String, "/flight/cmd_vel_local_json", 10)
        self.hold_yaw_pub = self.create_publisher(String, "/flight/cmd_hold_yaw_json", 10)
        self.land_pub = self.create_publisher(String, "/flight/cmd_land", 10)

        self.servo_pub = self.create_publisher(String, self.servo_cmd_topic, 10)
        self.rtl_pub = self.create_publisher(String, self.rtl_cmd_topic, 10)
        self.create_subscription(String, self.drop_done_topic, self.on_drop_done, 10)

        self.timer = self.create_timer(1.0 / max(loop_hz, 1.0), self.on_timer)

        self.get_logger().info("MissionManagerNode started")

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def current_mode(self) -> str:
        return str(self.telemetry.get("mode", "")).upper()

    def is_guided(self) -> bool:
        return self.current_mode() == "GUIDED"

    def is_armed(self) -> bool:
        return bool(self.telemetry.get("armed", False))

    def auto_allowed(self) -> bool:
        return self.is_guided() and self.is_armed()

    def current_rel_alt(self) -> float | None:
        val = self.telemetry.get("rel_alt_m")
        if val is None:
            return None
        try:
            return float(val)
        except Exception:
            return None

    def current_latlon(self) -> tuple[float | None, float | None]:
        lat = self.telemetry.get("lat")
        lon = self.telemetry.get("lon")
        try:
            lat = None if lat is None else float(lat)
            lon = None if lon is None else float(lon)
        except Exception:
            return None, None
        return lat, lon

    def current_heading_deg(self) -> float | None:
        val = self.telemetry.get("heading_deg")
        if val is None:
            val = self.telemetry.get("yaw_deg")
        if val is None:
            return None
        try:
            return float(val)
        except Exception:
            return None

    def in_small_marker_phase(self) -> bool:
        return self.state in {
            MissionState.SEARCH_SMALL,
            MissionState.ALIGN_SMALL,
            MissionState.LOCK_SMALL,
            MissionState.LAND_ON_SMALL_ARUCO,
            MissionState.DESCEND_TO_DROP_ALT,
            MissionState.DROP_PAYLOAD,
            MissionState.WAIT_DROP_DONE,
            MissionState.RTL_RETURN,
        }

    def small_marker_allowed_by_alt(self) -> bool:
        rel_alt = self.current_rel_alt()
        if rel_alt is None:
            return self.in_small_marker_phase()
        return rel_alt <= self.small_marker_enable_alt_m

    def active_marker_id(self) -> int:
        return self.small_marker_id if self.in_small_marker_phase() else self.big_marker_id

    def active_marker_seen(self) -> bool:
        if self.in_small_marker_phase():
            return self.small_marker_fresh()
        return self.big_marker_fresh()

    def active_marker(self) -> dict[str, Any] | None:
        if self.in_small_marker_phase():
            return self.small_marker if self.small_marker_fresh() else None
        return self.big_marker if self.big_marker_fresh() else None

    def target_marker_ids_for_current_state(self) -> set[int]:
        if self.in_small_marker_phase():
            if self.small_marker_allowed_by_alt():
                return {self.small_marker_id}
            return set()

        # Trong pha DESCEND_TO_SMALL_ALT, vẫn ưu tiên bám ID lớn để hạ.
        # Tuy nhiên nếu đã xuống đủ thấp theo small_marker_enable_alt_m,
        # cho phép nhận thêm ID nhỏ làm neo phụ khi ID lớn bị mất tạm thời.
        # Lưu ý: thấy ID nhỏ ở đây KHÔNG làm chuyển pha sớm sang ALIGN_SMALL.
        if self.state == MissionState.DESCEND_TO_SMALL_ALT:
            if self.small_marker_allowed_by_alt():
                return {self.big_marker_id, self.small_marker_id}
            return {self.big_marker_id}

        return {self.big_marker_id}

    def clear_inactive_marker_cache(self) -> None:
        if self.in_small_marker_phase():
            self.big_marker = None
            self.big_last_seen_time_s = None
        else:
            self.small_marker = None
            self.small_last_seen_time_s = None

    def marker_ref_valid(self) -> bool:
        return abs(self.marker_ref_lat) > 1e-9 and abs(self.marker_ref_lon) > 1e-9

    def search_center_latlon(self) -> tuple[float | None, float | None]:
        if self.marker_ref_valid():
            return self.marker_ref_lat, self.marker_ref_lon

        if self.goal is not None:
            return float(self.goal["lat"]), float(self.goal["lon"])

        return None, None

    def entered_search_region(self) -> bool:
        target_lat, target_lon = self.search_center_latlon()
        if target_lat is None or target_lon is None:
            return False

        lat, lon = self.current_latlon()
        if lat is None or lon is None:
            return False

        dist = haversine_distance_m(lat, lon, target_lat, target_lon)
        return dist <= self.search_enter_radius_m

    def build_search_points(self, radius_m: float) -> list[tuple[float, float]]:
        center_lat, center_lon = self.search_center_latlon()
        if center_lat is None or center_lon is None:
            return []

        # Không bắt đầu bằng (0,0), tránh drone đứng ở tâm GPS quá lâu.
        offsets = [
            (radius_m, 0.0),
            (radius_m, radius_m),
            (0.0, radius_m),
            (-radius_m, radius_m),
            (-radius_m, 0.0),
            (-radius_m, -radius_m),
            (0.0, -radius_m),
            (radius_m, -radius_m),
            (radius_m, 0.0),
        ]

        pts = []
        for north_m, east_m in offsets:
            lat, lon = offset_latlon_m(center_lat, center_lon, north_m, east_m)
            pts.append((lat, lon))
        return pts

    def start_search_loop(self, loop_index: int) -> None:
        radius = self.search_pattern_radius_m + self.search_pattern_radius_step_m * loop_index
        self.search_points = self.build_search_points(radius)
        self.search_loop_index = loop_index
        self.search_point_index = 0
        self.search_nav_sent = False
        self.search_point_hold_start_s = None
        self.search_waypoint_sent_time_s = None
        self.get_logger().info(f"Search loop {loop_index + 1} started, radius={radius:.2f}m")

    def current_descend_target_alt_m(self) -> float:
        if self.goal is not None:
            try:
                return float(self.goal.get("small_alt_m", self.descend_to_small_alt_m))
            except Exception:
                pass
        return self.descend_to_small_alt_m

    def descend_target_reached(self) -> bool:
        rel_alt = self.current_rel_alt()
        if rel_alt is None:
            return False
        return rel_alt <= (self.current_descend_target_alt_m() + self.descend_alt_reached_tol_m)

    def marker_inside_descend_window(self) -> bool:
        if not self.big_marker_fresh() or self.big_marker is None:
            return False
        err_x = abs(float(self.big_marker["err_x"]))
        err_y = abs(float(self.big_marker["err_y"]))
        return err_x <= self.descend_recenter_tolerance_px and err_y <= self.descend_recenter_tolerance_px

    def restart_search_pattern(self) -> None:
        self.search_big_start_time_s = self.now_s()
        self.lock_start_time_s = None
        self.align_heading_deg = None
        self.align_yaw_hold_last_sent_s = None
        self.publish_zero_vel()
        self.start_search_loop(0)
        self.transition_to(MissionState.SEARCH_BIG_PATTERN)

    def current_search_point(self) -> tuple[float | None, float | None]:
        if not self.search_points:
            return None, None
        if self.search_point_index >= len(self.search_points):
            return None, None
        return self.search_points[self.search_point_index]

    def distance_to_point_m(self, lat_t: float, lon_t: float) -> float | None:
        lat, lon = self.current_latlon()
        if lat is None or lon is None:
            return None
        return haversine_distance_m(lat, lon, lat_t, lon_t)

    def reset_runtime(self) -> None:
        self.prev_auto_state = None
        self.hold_start_time_s = None
        self.lock_start_time_s = None
        self.big_last_seen_time_s = None
        self.small_last_seen_time_s = None
        self.search_big_start_time_s = None
        self.search_small_start_time_s = None
        self.small_search_timeout_warned = False
        self.big_marker = None
        self.small_marker = None
        self.last_align_log_time_s = 0.0
        self.goto_sent = False
        self.land_sent = False
        self.zero_vel_sent = False
        self.drop_done_received = False
        self.drop_cmd_sent = False
        self.drop_start_time_s = None
        self.rtl_cmd_sent = False
        self.pending_event = None
        self.search_points = []
        self.search_loop_index = 0
        self.search_point_index = 0
        self.search_nav_sent = False
        self.search_point_hold_start_s = None
        self.search_waypoint_sent_time_s = None
        self.align_heading_deg = None
        self.align_yaw_hold_last_sent_s = None

    def transition_to(self, new_state: MissionState) -> None:
        if self.state == new_state:
            return

        old_state = self.state
        self.get_logger().info(f"{self.state.value} -> {new_state.value}")
        self.state = new_state
        self.hold_start_time_s = None
        self.lock_start_time_s = None
        self.zero_vel_sent = False

        if new_state == MissionState.ALIGN_BIG:
            self.get_logger().info("ALIGN_BIG started")
        elif new_state == MissionState.LOCK_BIG:
            self.get_logger().info("LOCK_BIG acquired")
        elif new_state == MissionState.DESCEND_TO_SMALL_ALT:
            self.get_logger().info("DESCEND_TO_SMALL_ALT started")
        elif new_state == MissionState.SEARCH_SMALL:
            self.search_small_start_time_s = self.now_s()
            self.small_search_timeout_warned = False
            self.get_logger().info("SEARCH_SMALL started")
        elif new_state == MissionState.ALIGN_SMALL:
            self.get_logger().info("ALIGN_SMALL started")
        elif new_state == MissionState.LOCK_SMALL:
            self.get_logger().info("LOCK_SMALL acquired")
        elif new_state == MissionState.LAND_ON_SMALL_ARUCO:
            self.get_logger().info("LAND_ON_SMALL_ARUCO started")
        elif new_state == MissionState.DESCEND_TO_DROP_ALT:
            self.get_logger().info("DESCEND_TO_DROP_ALT started")
        elif new_state == MissionState.DROP_PAYLOAD:
            self.drop_cmd_sent = False
            self.drop_done_received = False
            self.drop_start_time_s = None
            self.get_logger().info("DROP_PAYLOAD started")
        elif new_state == MissionState.WAIT_DROP_DONE:
            self.drop_start_time_s = self.now_s()
            self.get_logger().info("WAIT_DROP_DONE started")
        elif new_state == MissionState.RTL_RETURN:
            self.rtl_cmd_sent = False
            self.get_logger().info("RTL_RETURN started")

        if new_state in {
            MissionState.SEARCH_SMALL,
            MissionState.ALIGN_SMALL,
            MissionState.LOCK_SMALL,
            MissionState.LAND_ON_SMALL_ARUCO,
            MissionState.DESCEND_TO_DROP_ALT,
            MissionState.DROP_PAYLOAD,
            MissionState.WAIT_DROP_DONE,
            MissionState.RTL_RETURN,
        }:
            self.big_marker = None
            self.big_last_seen_time_s = None

        elif old_state in {
            MissionState.SEARCH_SMALL,
            MissionState.ALIGN_SMALL,
            MissionState.LOCK_SMALL,
            MissionState.LAND_ON_SMALL_ARUCO,
            MissionState.DESCEND_TO_DROP_ALT,
            MissionState.DROP_PAYLOAD,
            MissionState.WAIT_DROP_DONE,
            MissionState.RTL_RETURN,
        } and new_state not in {
            MissionState.SEARCH_SMALL,
            MissionState.ALIGN_SMALL,
            MissionState.LOCK_SMALL,
            MissionState.LAND_ON_SMALL_ARUCO,
            MissionState.DESCEND_TO_DROP_ALT,
            MissionState.DROP_PAYLOAD,
            MissionState.WAIT_DROP_DONE,
            MissionState.RTL_RETURN,
        }:
            self.small_marker = None
            self.small_last_seen_time_s = None

        if old_state in {
            MissionState.ALIGN_BIG,
            MissionState.LOCK_BIG,
            MissionState.DESCEND_TO_SMALL_ALT,
            MissionState.SEARCH_SMALL,
            MissionState.ALIGN_SMALL,
            MissionState.LOCK_SMALL,
            MissionState.LAND_ON_SMALL_ARUCO,
            MissionState.DESCEND_TO_DROP_ALT,
            MissionState.DROP_PAYLOAD,
            MissionState.WAIT_DROP_DONE,
            MissionState.RTL_RETURN,
        } and new_state not in {
            MissionState.ALIGN_BIG,
            MissionState.LOCK_BIG,
            MissionState.DESCEND_TO_SMALL_ALT,
            MissionState.SEARCH_SMALL,
            MissionState.ALIGN_SMALL,
            MissionState.LOCK_SMALL,
            MissionState.LAND_ON_SMALL_ARUCO,
            MissionState.DESCEND_TO_DROP_ALT,
            MissionState.DROP_PAYLOAD,
            MissionState.WAIT_DROP_DONE,
            MissionState.RTL_RETURN,
        }:
            self.align_heading_deg = None
            self.align_yaw_hold_last_sent_s = None

    def on_telemetry(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict):
                self.telemetry = data
        except Exception as e:
            self.get_logger().warn(f"Telemetry parse error: {e}")

    def on_goal(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                return

            self.goal = {
                "lat": float(data["lat"]),
                "lon": float(data["lon"]),
                "search_alt_m": float(data.get("search_alt_m", self.takeoff_alt_m)),
                "small_alt_m": float(data.get("small_alt_m", self.descend_to_small_alt_m)),
            }
            self.get_logger().info(f"Mission goal received from web: {self.goal}")
            self.publish_state()
        except Exception as e:
            self.get_logger().warn(f"Goal parse error: {e}")

    def on_control(self, msg: String) -> None:
        cmd = msg.data.strip().upper()

        if cmd == "RESET":
            self.reset_runtime()
            self.transition_to(MissionState.WAIT_GUIDED if self.auto_start_on_guided else MissionState.IDLE)
            self.publish_state()
            return

        if cmd == "ABORT":
            if self.state not in {MissionState.IDLE, MissionState.DONE, MissionState.MANUAL_OVERRIDE}:
                self.prev_auto_state = self.state
            self.publish_zero_vel()
            self.transition_to(MissionState.MANUAL_OVERRIDE)
            self.publish_state()

    def on_drop_done(self, msg: String) -> None:
        payload = str(msg.data).strip()
        self.drop_done_received = True
        self.emit_event("DROP_DONE_RECEIVED", f"Drop done received: {payload}")
        self.publish_state()

    def safe_float(self, value: Any, default: float = 0.0) -> float:
        try:
            if value is None:
                return default
            return float(value)
        except Exception:
            return default

    def marker_from_detection(self, det: dict[str, Any], marker_id: int) -> dict[str, Any]:
        return {
            "marker_id": int(marker_id),
            "err_x": self.safe_float(det.get("err_x")),
            "err_y": self.safe_float(det.get("err_y")),
            "center_x": det.get("center_x"),
            "center_y": det.get("center_y"),
            "x_m": self.safe_float(det.get("x_m")),
            "y_m": self.safe_float(det.get("y_m")),
            "z_m": self.safe_float(det.get("z_m")),
            "distance_m": self.safe_float(det.get("distance_m")),
        }

    def on_aruco(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            detections = payload.get("detections", [])
            if not isinstance(detections, list):
                return

            self.clear_inactive_marker_cache()

            best_by_id: dict[int, dict[str, Any]] = {}
            best_score_by_id: dict[int, float] = {}
            target_ids = self.target_marker_ids_for_current_state()
            if not target_ids:
                return

            for det in detections:
                if not isinstance(det, dict):
                    continue

                marker_id_raw = det.get("marker_id", det.get("id"))
                if marker_id_raw is None:
                    continue

                try:
                    marker_id = int(marker_id_raw)
                except Exception:
                    continue

                if marker_id not in target_ids:
                    continue

                if "err_x" not in det or "err_y" not in det:
                    continue

                err_x = self.safe_float(det.get("err_x"))
                err_y = self.safe_float(det.get("err_y"))
                score = abs(err_x) + abs(err_y)

                if marker_id not in best_by_id or score < best_score_by_id[marker_id]:
                    best_by_id[marker_id] = self.marker_from_detection(det, marker_id)
                    best_score_by_id[marker_id] = score

            now = self.now_s()
            if self.big_marker_id in best_by_id:
                self.big_marker = best_by_id[self.big_marker_id]
                self.big_last_seen_time_s = now
            if self.small_marker_id in best_by_id:
                self.small_marker = best_by_id[self.small_marker_id]
                self.small_last_seen_time_s = now

        except Exception as e:
            self.get_logger().warn(f"Aruco parse error: {e}")

    def marker_age_s(self, last_seen_time_s: float | None) -> float | None:
        if last_seen_time_s is None:
            return None
        return self.now_s() - last_seen_time_s

    def big_marker_fresh(self) -> bool:
        if self.big_marker is None:
            return False
        age = self.marker_age_s(self.big_last_seen_time_s)
        return age is not None and age <= self.marker_lost_timeout_s

    def small_marker_fresh(self) -> bool:
        if self.small_marker is None:
            return False
        age = self.marker_age_s(self.small_last_seen_time_s)
        return age is not None and age <= self.marker_lost_timeout_s

    def search_big_timed_out(self) -> bool:
        if self.search_big_start_time_s is None:
            return False
        return (self.now_s() - self.search_big_start_time_s) >= self.search_timeout_s

    def small_search_timed_out(self) -> bool:
        if self.search_small_start_time_s is None:
            return False
        return (self.now_s() - self.search_small_start_time_s) >= self.small_search_timeout_s

    def publish_takeoff(self, alt_m: float) -> None:
        if not self.auto_allowed():
            return
        msg = String()
        msg.data = json.dumps({"alt_m": float(alt_m)}, ensure_ascii=False)
        self.takeoff_pub.publish(msg)
        self.get_logger().info(f"Published TAKEOFF: alt={alt_m:.2f}")

    def publish_goto_search_center(self, alt_m: float, speed_mps: float) -> None:
        if not self.auto_allowed():
            return

        target_lat, target_lon = self.search_center_latlon()
        if target_lat is None or target_lon is None:
            return

        msg = String()
        msg.data = json.dumps(
            {
                "lat": float(target_lat),
                "lon": float(target_lon),
                "alt_m": float(alt_m),
                "speed_mps": float(speed_mps),
            },
            ensure_ascii=False,
        )
        self.goto_pub.publish(msg)
        self.get_logger().info(
            f"Published GOTO_SEARCH_CENTER: lat={target_lat:.7f}, lon={target_lon:.7f}, "
            f"alt={alt_m:.2f}, speed={speed_mps:.2f}"
        )

    def publish_goto_point(self, lat: float, lon: float, alt_m: float, speed_mps: float) -> None:
        if not self.auto_allowed():
            return
        msg = String()
        msg.data = json.dumps(
            {
                "lat": float(lat),
                "lon": float(lon),
                "alt_m": float(alt_m),
                "speed_mps": float(speed_mps),
            },
            ensure_ascii=False,
        )
        self.goto_pub.publish(msg)
        self.get_logger().info(
            f"Published SEARCH_POINT: lat={lat:.7f}, lon={lon:.7f}, alt={alt_m:.2f}, speed={speed_mps:.2f}"
        )

    def publish_hold_here(self, alt_m: float) -> None:
        if not self.auto_allowed():
            return
        lat, lon = self.current_latlon()
        if lat is None or lon is None:
            return

        msg = String()
        msg.data = json.dumps(
            {
                "lat": float(lat),
                "lon": float(lon),
                "alt_m": float(alt_m),
                "speed_mps": 0.5,
            },
            ensure_ascii=False,
        )
        self.goto_pub.publish(msg)
        self.get_logger().info(
            f"Published HOLD_HERE: lat={lat:.7f}, lon={lon:.7f}, alt={alt_m:.2f}"
        )

    def publish_vel_body(self, vx: float, vy: float, vz: float = 0.0, yaw_rate: float = 0.0) -> None:
        if not self.auto_allowed():
            return
        msg = String()
        msg.data = json.dumps(
            {"vx": float(vx), "vy": float(vy), "vz": float(vz), "yaw_rate": float(yaw_rate)},
            ensure_ascii=False,
        )
        self.vel_pub.publish(msg)

    def publish_hold_yaw(self, heading_deg: float, yaw_rate_dps: float | None = None) -> None:
        if not self.auto_allowed():
            return
        if yaw_rate_dps is None:
            yaw_rate_dps = self.align_yaw_rate_dps

        msg = String()
        msg.data = json.dumps(
            {
                "heading_deg": float(heading_deg),
                "yaw_rate_dps": float(yaw_rate_dps),
            },
            ensure_ascii=False,
        )
        self.hold_yaw_pub.publish(msg)

    def publish_zero_vel(self) -> None:
        self.publish_vel_body(0.0, 0.0, 0.0, yaw_rate=0.0)

    def publish_land(self) -> None:
        if not self.auto_allowed():
            return
        msg = String()
        msg.data = "LAND"
        self.land_pub.publish(msg)
        self.get_logger().info("Published LAND")

    def emit_event(self, event_type: str, message: str, level: str = "info") -> None:
        self.pending_event = {"type": str(event_type), "message": str(message), "level": str(level)}
        if level == "warn":
            self.get_logger().warn(message)
        elif level == "error":
            self.get_logger().error(message)
        else:
            self.get_logger().info(message)

    def publish_servo_drop(self) -> None:
        if not self.auto_allowed():
            return
        msg = String()
        msg.data = self.servo_cmd_payload
        self.servo_pub.publish(msg)
        self.emit_event("DROP_PAYLOAD_SENT", f"Drop payload command sent: {self.servo_cmd_payload}")

    def publish_rtl(self) -> None:
        if not self.is_armed():
            return
        msg = String()
        msg.data = self.rtl_cmd_payload
        self.rtl_pub.publish(msg)
        self.emit_event("RTL_SENT", f"RTL command sent: {self.rtl_cmd_payload}")

    def clamp(self, value: float, limit: float) -> float:
        limit = abs(float(limit))
        if value > limit:
            return limit
        if value < -limit:
            return -limit
        return value

    def adaptive_speed_limit(
        self,
        error_px: float,
        slow_error_px: float,
        fast_error_px: float,
        min_speed_mps: float,
        max_speed_mps: float,
    ) -> float:
        if error_px <= 0.0 or max_speed_mps <= 0.0:
            return 0.0

        min_speed_mps = max(0.0, min(min_speed_mps, max_speed_mps))
        if fast_error_px <= slow_error_px:
            return max_speed_mps

        if error_px <= slow_error_px:
            return min_speed_mps
        if error_px >= fast_error_px:
            return max_speed_mps

        ratio = (error_px - slow_error_px) / (fast_error_px - slow_error_px)
        return min_speed_mps + ratio * (max_speed_mps - min_speed_mps)

    def compute_marker_align_velocity(
        self,
        marker: dict[str, Any] | None,
        deadband_px: float,
        gain_vx_from_err_y: float,
        gain_vy_from_err_x: float,
        max_body_speed_mps: float,
        min_body_speed_mps: float,
        slow_error_px: float,
        fast_error_px: float,
    ) -> tuple[float, float]:
        if marker is None:
            return 0.0, 0.0

        err_x = float(marker["err_x"])
        err_y = float(marker["err_y"])

        if abs(err_x) <= deadband_px:
            err_x = 0.0
        if abs(err_y) <= deadband_px:
            err_y = 0.0

        error_px = max(abs(err_x), abs(err_y))
        if error_px <= 0.0:
            return 0.0, 0.0

        speed_limit = self.adaptive_speed_limit(
            error_px,
            slow_error_px,
            fast_error_px,
            min_body_speed_mps,
            max_body_speed_mps,
        )

        vx = gain_vx_from_err_y * err_y
        vy = gain_vy_from_err_x * err_x

        vx = self.clamp(vx, speed_limit)
        vy = self.clamp(vy, speed_limit)
        return vx, vy

    def compute_big_align_velocity(self) -> tuple[float, float]:
        return self.compute_marker_align_velocity(
            self.big_marker,
            self.big_align_deadband_px,
            self.vx_from_err_y_gain,
            self.vy_from_err_x_gain,
            self.big_max_body_speed_mps,
            self.big_min_body_speed_mps,
            self.big_slow_error_px,
            self.big_fast_error_px,
        )

    def compute_small_align_velocity(self) -> tuple[float, float]:
        small_fast_error_px = max(self.small_lock_tolerance_px * 2.0, self.small_align_deadband_px + 1.0)
        return self.compute_marker_align_velocity(
            self.small_marker,
            self.small_align_deadband_px,
            self.small_vx_from_err_y_gain,
            self.small_vy_from_err_x_gain,
            self.small_max_body_speed_mps,
            self.small_min_body_speed_mps,
            self.small_align_deadband_px,
            small_fast_error_px,
        )

    def big_marker_inside_lock_window(self) -> bool:
        if not self.big_marker_fresh() or self.big_marker is None:
            return False

        err_x = abs(float(self.big_marker["err_x"]))
        err_y = abs(float(self.big_marker["err_y"]))
        return err_x <= self.big_lock_tolerance_px and err_y <= self.big_lock_tolerance_px

    def small_marker_inside_lock_window(self) -> bool:
        if not self.small_marker_fresh() or self.small_marker is None:
            return False

        err_x = abs(float(self.small_marker["err_x"]))
        err_y = abs(float(self.small_marker["err_y"]))
        return err_x <= self.small_lock_tolerance_px and err_y <= self.small_lock_tolerance_px

    def small_marker_inside_recenter_window(self) -> bool:
        if not self.small_marker_fresh() or self.small_marker is None:
            return False

        err_x = abs(float(self.small_marker["err_x"]))
        err_y = abs(float(self.small_marker["err_y"]))
        return err_x <= self.land_recenter_tolerance_px and err_y <= self.land_recenter_tolerance_px

    def small_marker_inside_final_land_window(self) -> bool:
        if not self.small_marker_fresh() or self.small_marker is None:
            return False

        err_x = abs(float(self.small_marker["err_x"]))
        err_y = abs(float(self.small_marker["err_y"]))
        return err_x <= self.land_final_tolerance_px and err_y <= self.land_final_tolerance_px

    def small_aruco_z_m(self) -> float | None:
        if not self.small_marker_fresh() or self.small_marker is None:
            return None

        val = self.small_marker.get("z_m")
        try:
            z = float(val)
        except Exception:
            return None

        # Reject invalid or near-zero pose values.
        if z <= 0.05:
            return None

        return z

    def small_marker_inside_aruco_land_window(self) -> bool:
        if not self.small_marker_fresh() or self.small_marker is None:
            return False

        err_x = abs(float(self.small_marker["err_x"]))
        err_y = abs(float(self.small_marker["err_y"]))

        return (
            err_x <= self.land_aruco_z_max_err_px
            and err_y <= self.land_aruco_z_max_err_px
        )

    def land_target_reached(self) -> bool:
        # Main condition: relative altitude from flight controller / Mission Planner.
        rel_alt = self.current_rel_alt()
        if rel_alt is not None and rel_alt <= self.land_on_small_alt_m:
            return True

        # Backup condition: ArUco PnP z_m from ID40.
        # Only use it when marker is fresh and still reasonably close to image center.
        if not self.land_use_aruco_z:
            return False

        aruco_z = self.small_aruco_z_m()
        if (
            aruco_z is not None
            and aruco_z <= self.land_on_small_aruco_z_m
            and self.small_marker_inside_aruco_land_window()
        ):
            return True

        return False

    def small_marker_inside_drop_recenter_window(self) -> bool:
        if not self.small_marker_fresh() or self.small_marker is None:
            return False

        err_x = abs(float(self.small_marker["err_x"]))
        err_y = abs(float(self.small_marker["err_y"]))
        return err_x <= self.drop_recenter_tolerance_px and err_y <= self.drop_recenter_tolerance_px

    def small_marker_inside_final_drop_window(self) -> bool:
        if not self.small_marker_fresh() or self.small_marker is None:
            return False

        err_x = abs(float(self.small_marker["err_x"]))
        err_y = abs(float(self.small_marker["err_y"]))
        return err_x <= self.drop_final_tolerance_px and err_y <= self.drop_final_tolerance_px

    def small_marker_inside_aruco_drop_window(self) -> bool:
        if not self.small_marker_fresh() or self.small_marker is None:
            return False

        err_x = abs(float(self.small_marker["err_x"]))
        err_y = abs(float(self.small_marker["err_y"]))
        return err_x <= self.drop_aruco_z_max_err_px and err_y <= self.drop_aruco_z_max_err_px

    def drop_target_reached(self) -> bool:
        # Called only after ID40 is fresh and reasonably centered.
        rel_alt = self.current_rel_alt()
        if rel_alt is not None and rel_alt <= self.drop_alt_m:
            return True

        if not self.drop_use_aruco_z:
            return False

        aruco_z = self.small_aruco_z_m()
        if (
            aruco_z is not None
            and aruco_z <= self.drop_aruco_z_m
            and self.small_marker_inside_aruco_drop_window()
        ):
            return True

        return False

    def maybe_refresh_align_yaw_hold(self) -> None:
        if self.align_heading_deg is None:
            return

        now = self.now_s()
        if self.align_yaw_hold_last_sent_s is None or (now - self.align_yaw_hold_last_sent_s) >= self.align_yaw_hold_resend_s:
            self.publish_hold_yaw(self.align_heading_deg, self.align_yaw_rate_dps)
            self.align_yaw_hold_last_sent_s = now

    def log_align_command(
        self,
        state_label: str,
        marker_id: int,
        marker: dict[str, Any] | None,
        vx: float,
        vy: float,
        vz: float,
    ) -> None:
        if marker is None:
            return

        now = self.now_s()
        if now - self.last_align_log_time_s < 1.0:
            return

        self.last_align_log_time_s = now
        rel_alt = self.current_rel_alt()
        alt_text = "N/A" if rel_alt is None else f"{rel_alt:.1f}"
        err_x = float(marker["err_x"])
        err_y = float(marker["err_y"])
        self.get_logger().info(
            f"{state_label} marker={marker_id} err=({err_x:.1f},{err_y:.1f}) "
            f"v=({vx:.2f},{vy:.2f},{vz:.2f}) alt={alt_text}"
        )

    def publish_state(self) -> None:
        payload = {
            "state": self.state.value,
            "prev_auto_state": self.prev_auto_state.value if self.prev_auto_state else None,
            "mode": self.telemetry.get("mode"),
            "armed": self.telemetry.get("armed"),
            "connected": self.telemetry.get("connected"),
            "rel_alt_m": self.telemetry.get("rel_alt_m"),
            "goal": self.goal,
            "goto_speed_mps": self.goto_speed_mps,
            "marker_ref_lat": self.marker_ref_lat,
            "marker_ref_lon": self.marker_ref_lon,
            "search_enter_radius_m": self.search_enter_radius_m,
            "big_marker_id": self.big_marker_id,
            "small_marker_id": self.small_marker_id,
            "small_marker_enable_alt_m": self.small_marker_enable_alt_m,
            "active_marker_id": self.active_marker_id(),
            "active_marker_seen": self.active_marker_seen(),
            "active_marker": self.active_marker(),
            "big_marker_seen": False if self.in_small_marker_phase() else self.big_marker_fresh(),
            "small_marker_seen": self.small_marker_fresh() if self.in_small_marker_phase() else False,
            "big_marker": None if self.in_small_marker_phase() else self.big_marker,
            "small_marker": self.small_marker if self.in_small_marker_phase() else None,
            "search_loop_index": self.search_loop_index,
            "search_point_index": self.search_point_index,
            "align_heading_deg": self.align_heading_deg,
            "descend_target_alt_m": self.current_descend_target_alt_m(),
            "descend_to_small_alt_m": self.current_descend_target_alt_m(),
            "enable_land_after_lock_small": self.enable_land_after_lock_small,
            "land_on_small_alt_m": self.land_on_small_alt_m,
            "land_use_aruco_z": self.land_use_aruco_z,
            "land_on_small_aruco_z_m": self.land_on_small_aruco_z_m,
            "land_aruco_z_max_err_px": self.land_aruco_z_max_err_px,
            "small_aruco_z_m": self.small_aruco_z_m(),
            "land_descend_speed_mps": self.land_descend_speed_mps,
            "land_recenter_tolerance_px": self.land_recenter_tolerance_px,
            "land_final_tolerance_px": self.land_final_tolerance_px,
            "enable_drop_after_lock_small": self.enable_drop_after_lock_small,
            "drop_alt_m": self.drop_alt_m,
            "drop_use_aruco_z": self.drop_use_aruco_z,
            "drop_aruco_z_m": self.drop_aruco_z_m,
            "drop_aruco_z_max_err_px": self.drop_aruco_z_max_err_px,
            "drop_descend_speed_mps": self.drop_descend_speed_mps,
            "drop_recenter_tolerance_px": self.drop_recenter_tolerance_px,
            "drop_final_tolerance_px": self.drop_final_tolerance_px,
            "drop_done_received": self.drop_done_received,
            "drop_cmd_sent": self.drop_cmd_sent,
            "enable_rtl_after_drop": self.enable_rtl_after_drop,
            "rtl_cmd_sent": self.rtl_cmd_sent,
            "event": self.pending_event,
            "lock_big_descend_started": self.state in {
                MissionState.DESCEND_TO_SMALL_ALT,
                MissionState.SEARCH_SMALL,
                MissionState.ALIGN_SMALL,
                MissionState.LOCK_SMALL,
                MissionState.LAND_ON_SMALL_ARUCO,
            },
            "lock_big_descend_completed": self.state in {
                MissionState.SEARCH_SMALL,
                MissionState.ALIGN_SMALL,
                MissionState.LOCK_SMALL,
                MissionState.LAND_ON_SMALL_ARUCO,
            },
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.state_pub.publish(msg)
        self.pending_event = None

    def handle_search_big_pattern(self) -> None:
        if self.search_big_timed_out():
            self.get_logger().warn("SEARCH_BIG_PATTERN timeout, restart search loop")
            self.restart_search_pattern()
            self.publish_state()
            return

        if self.big_marker_fresh():
            self.publish_zero_vel()
            self.transition_to(MissionState.ALIGN_BIG)
            self.publish_state()
            return

        lat_t, lon_t = self.current_search_point()

        if lat_t is None or lon_t is None:
            if self.search_loop_index + 1 < self.search_pattern_loops_max:
                self.start_search_loop(self.search_loop_index + 1)
            else:
                self.start_search_loop(0)
            self.publish_state()
            return

        if not self.search_nav_sent:
            self.publish_goto_point(lat_t, lon_t, self.takeoff_alt_m, self.search_waypoint_speed_mps)
            self.search_nav_sent = True
            self.search_waypoint_sent_time_s = self.now_s()
            self.search_point_hold_start_s = None
            self.publish_state()
            return

        dist = self.distance_to_point_m(lat_t, lon_t)

        if dist is not None and dist <= self.search_waypoint_reached_m:
            if self.search_point_hold_start_s is None:
                self.search_point_hold_start_s = self.now_s()

            if (self.now_s() - self.search_point_hold_start_s) >= self.search_step_hold_s:
                self.search_point_index += 1
                self.search_nav_sent = False
                self.search_waypoint_sent_time_s = None
                self.search_point_hold_start_s = None

        elif (
            self.search_waypoint_sent_time_s is not None
            and (self.now_s() - self.search_waypoint_sent_time_s) >= self.search_waypoint_timeout_s
        ):
            self.get_logger().warn(
                f"SEARCH_POINT timeout, skip point {self.search_point_index}"
            )
            self.search_point_index += 1
            self.search_nav_sent = False
            self.search_waypoint_sent_time_s = None
            self.search_point_hold_start_s = None

        self.publish_state()

    def handle_align_big(self) -> None:
        if self.search_big_timed_out():
            self.get_logger().warn("ALIGN_BIG timeout, restart search loop")
            self.restart_search_pattern()
            self.publish_state()
            return

        if not self.big_marker_fresh():
            self.get_logger().warn("Big marker lost in ALIGN_BIG; restart search pattern")
            self.restart_search_pattern()
            self.publish_state()
            return

        vx_body, vy_body = self.compute_big_align_velocity()
        self.publish_vel_body(vx_body, vy_body, 0.0, yaw_rate=0.0)
        self.log_align_command(
            "ALIGN_BIG",
            self.big_marker_id,
            self.big_marker,
            vx_body,
            vy_body,
            0.0,
        )

        if self.big_marker_inside_lock_window():
            if self.lock_start_time_s is None:
                self.lock_start_time_s = self.now_s()

            if (self.now_s() - self.lock_start_time_s) >= self.big_lock_hold_s:
                self.publish_zero_vel()
                self.transition_to(MissionState.LOCK_BIG)
        else:
            self.lock_start_time_s = None

        self.publish_state()

    def handle_lock_big(self) -> None:
        if not self.big_marker_fresh():
            self.publish_zero_vel()
            self.transition_to(MissionState.SEARCH_BIG_PATTERN)
            self.publish_state()
            return

        if not self.marker_inside_descend_window():
            self.publish_zero_vel()
            self.transition_to(MissionState.ALIGN_BIG)
            self.publish_state()
            return

        vx_body, vy_body = self.compute_big_align_velocity()
        self.publish_vel_body(vx_body, vy_body, 0.0, yaw_rate=0.0)
        self.log_align_command(
            "LOCK_BIG",
            self.big_marker_id,
            self.big_marker,
            vx_body,
            vy_body,
            0.0,
        )

        if self.enable_descend_after_lock_big:
            self.publish_zero_vel()
            self.transition_to(MissionState.DESCEND_TO_SMALL_ALT)

        self.publish_state()

    def handle_descend_to_small_alt(self) -> None:
        # Ưu tiên cao nhất: chỉ khi đạt độ cao đặt trước thì mới chuyển sang SEARCH_SMALL.
        # Không chuyển sang ALIGN_SMALL sớm chỉ vì thấy ID nhỏ trong lúc đang hạ.
        if self.descend_target_reached():
            self.publish_zero_vel()
            rel_alt = self.current_rel_alt()
            alt_text = "N/A" if rel_alt is None else f"{rel_alt:.2f}"
            self.get_logger().info(f"DESCEND_TO_SMALL_ALT completed at rel_alt={alt_text}m")
            self.transition_to(MissionState.SEARCH_SMALL)
            self.publish_state()
            return

        # Bình thường: nếu còn thấy ID lớn thì dùng ID lớn làm neo chính để căn tâm và hạ.
        if self.big_marker_fresh():
            if not self.marker_inside_descend_window():
                self.publish_zero_vel()
                self.transition_to(MissionState.ALIGN_BIG)
                self.publish_state()
                return

            vx_body, vy_body = self.compute_big_align_velocity()
            vz_body = self.descend_speed_mps
            self.publish_vel_body(vx_body, vy_body, vz_body, yaw_rate=0.0)
            self.log_align_command(
                "DESCEND_TO_SMALL_ALT_BIG",
                self.big_marker_id,
                self.big_marker,
                vx_body,
                vy_body,
                vz_body,
            )
            self.publish_state()
            return

        # Dự phòng: nếu mất ID lớn nhưng đã thấy ID nhỏ thì dùng ID nhỏ để giữ XY và tiếp tục hạ.
        # Vẫn giữ state DESCEND_TO_SMALL_ALT cho tới khi rel_alt đạt descend_to_small_alt_m.
        if self.small_marker_fresh():
            vx_body, vy_body = self.compute_small_align_velocity()
            vz_body = self.descend_speed_mps
            self.publish_vel_body(vx_body, vy_body, vz_body, yaw_rate=0.0)
            self.log_align_command(
                "DESCEND_TO_SMALL_ALT_SMALL_BACKUP",
                self.small_marker_id,
                self.small_marker,
                vx_body,
                vy_body,
                vz_body,
            )
            self.publish_state()
            return

        # Mất cả ID lớn và ID nhỏ: không lao đi search big ngay trong lúc descend.
        # Giữ hover để chờ marker quay lại, tránh chuyển pha gây drone bay vọt đi tìm marker lớn.
        self.get_logger().warn(
            "No marker during DESCEND_TO_SMALL_ALT; holding instead of restarting SEARCH_BIG_PATTERN"
        )
        self.publish_zero_vel()
        self.publish_state()

    def handle_search_small(self) -> None:
        if self.small_marker_fresh():
            self.get_logger().info(f"Small marker detected ID {self.small_marker_id}")
            self.publish_zero_vel()
            self.transition_to(MissionState.ALIGN_SMALL)
            self.publish_state()
            return

        if self.small_search_timed_out() and not self.small_search_timeout_warned:
            self.get_logger().warn(
                f"SEARCH_SMALL timeout: marker ID {self.small_marker_id} not detected, holding near big marker"
            )
            self.small_search_timeout_warned = True

        self.publish_zero_vel()
        self.publish_state()

    def handle_align_small(self) -> None:
        small_age = self.marker_age_s(self.small_last_seen_time_s)
        if small_age is None or small_age > self.marker_lost_timeout_s:
            self.publish_zero_vel()
            self.transition_to(MissionState.SEARCH_SMALL)
            self.publish_state()
            return

        if small_age > min(0.30, self.marker_lost_timeout_s * 0.5):
            self.publish_zero_vel()
            self.publish_state()
            return

        vx_body, vy_body = self.compute_small_align_velocity()
        self.publish_vel_body(vx_body, vy_body, 0.0, yaw_rate=0.0)
        self.log_align_command(
            "ALIGN_SMALL",
            self.small_marker_id,
            self.small_marker,
            vx_body,
            vy_body,
            0.0,
        )

        if self.small_marker_inside_lock_window():
            if self.lock_start_time_s is None:
                self.lock_start_time_s = self.now_s()

            if (self.now_s() - self.lock_start_time_s) >= self.small_lock_hold_s:
                self.publish_zero_vel()
                self.transition_to(MissionState.LOCK_SMALL)
        else:
            self.lock_start_time_s = None

        self.publish_state()

    def handle_lock_small(self) -> None:
        if not self.small_marker_fresh():
            self.publish_zero_vel()
            self.transition_to(MissionState.SEARCH_SMALL)
            self.publish_state()
            return

        if not self.small_marker_inside_drop_recenter_window():
            self.publish_zero_vel()
            self.transition_to(MissionState.ALIGN_SMALL)
            self.publish_state()
            return

        vx_body, vy_body = self.compute_small_align_velocity()
        self.publish_vel_body(vx_body, vy_body, 0.0, yaw_rate=0.0)
        self.log_align_command(
            "LOCK_SMALL",
            self.small_marker_id,
            self.small_marker,
            vx_body,
            vy_body,
            0.0,
        )

        if self.enable_drop_after_lock_small:
            if self.small_marker_inside_final_drop_window():
                if self.lock_start_time_s is None:
                    self.lock_start_time_s = self.now_s()

                if (self.now_s() - self.lock_start_time_s) >= self.drop_hold_s:
                    self.publish_zero_vel()
                    self.transition_to(MissionState.DESCEND_TO_DROP_ALT)
            else:
                self.lock_start_time_s = None

            self.publish_state()
            return

        # Legacy fallback: keep old landing flow only when explicitly enabled.
        if self.enable_land_after_lock_small:
            if self.small_marker_inside_final_land_window():
                if self.lock_start_time_s is None:
                    self.lock_start_time_s = self.now_s()

                if (self.now_s() - self.lock_start_time_s) >= self.land_small_hold_s:
                    self.publish_zero_vel()
                    self.transition_to(MissionState.LAND_ON_SMALL_ARUCO)
            else:
                self.lock_start_time_s = None

        self.publish_state()

    def handle_descend_to_drop_alt(self) -> None:
        if not self.small_marker_fresh():
            self.publish_zero_vel()
            self.transition_to(MissionState.SEARCH_SMALL)
            self.publish_state()
            return

        if not self.small_marker_inside_drop_recenter_window():
            self.publish_zero_vel()
            self.transition_to(MissionState.ALIGN_SMALL)
            self.publish_state()
            return

        if self.drop_target_reached():
            rel_alt = self.current_rel_alt()
            aruco_z = self.small_aruco_z_m()
            rel_text = "N/A" if rel_alt is None else f"{rel_alt:.2f}"
            z_text = "N/A" if aruco_z is None else f"{aruco_z:.2f}"
            self.emit_event(
                "DROP_ALT_REACHED",
                f"Drop altitude reached: rel_alt={rel_text}m, aruco_z={z_text}m",
            )
            self.publish_zero_vel()
            self.transition_to(MissionState.DROP_PAYLOAD)
            self.publish_state()
            return

        vx_body, vy_body = self.compute_small_align_velocity()

        if self.small_marker_inside_final_drop_window():
            vz_body = self.drop_descend_speed_mps
        else:
            vz_body = 0.0

        self.publish_vel_body(vx_body, vy_body, vz_body, yaw_rate=0.0)
        self.log_align_command(
            "DESCEND_TO_DROP_ALT",
            self.small_marker_id,
            self.small_marker,
            vx_body,
            vy_body,
            vz_body,
        )
        self.publish_state()

    def handle_drop_payload(self) -> None:
        if not self.drop_cmd_sent:
            self.publish_zero_vel()
            self.publish_servo_drop()
            self.drop_cmd_sent = True

        self.transition_to(MissionState.WAIT_DROP_DONE)
        self.publish_state()

    def handle_wait_drop_done(self) -> None:
        self.publish_zero_vel()

        elapsed = 0.0
        if self.drop_start_time_s is not None:
            elapsed = self.now_s() - self.drop_start_time_s

        if self.drop_done_received:
            self.emit_event("DROP_DONE", "Drop done confirmed")
            if self.enable_rtl_after_drop:
                self.transition_to(MissionState.RTL_RETURN)
            else:
                self.transition_to(MissionState.DONE)
            self.publish_state()
            return

        if elapsed >= self.drop_timeout_s:
            self.emit_event(
                "DROP_TIMEOUT",
                f"Drop done timeout after {elapsed:.1f}s; continue to RTL",
                level="warn",
            )
            if self.enable_rtl_after_drop:
                self.transition_to(MissionState.RTL_RETURN)
            else:
                self.transition_to(MissionState.DONE)
            self.publish_state()
            return

        self.publish_state()

    def handle_rtl_return(self) -> None:
        if not self.rtl_cmd_sent:
            self.publish_zero_vel()
            self.publish_rtl()
            self.rtl_cmd_sent = True

        # After RTL is sent, ArduPilot owns the return-home behavior.
        # Move mission node to DONE to avoid treating RTL mode as manual override.
        self.transition_to(MissionState.DONE)
        self.publish_state()

    def handle_land_on_small_aruco(self) -> None:
        if self.land_target_reached():
            self.publish_zero_vel()
            self.publish_land()
            self.transition_to(MissionState.LAND)
            self.publish_state()
            return

        if not self.small_marker_fresh():
            self.publish_zero_vel()
            self.transition_to(MissionState.SEARCH_SMALL)
            self.publish_state()
            return

        if not self.small_marker_inside_recenter_window():
            self.publish_zero_vel()
            self.transition_to(MissionState.ALIGN_SMALL)
            self.publish_state()
            return

        vx_body, vy_body = self.compute_small_align_velocity()

        if self.small_marker_inside_final_land_window():
            vz_body = self.land_descend_speed_mps
        else:
            vz_body = 0.0

        self.publish_vel_body(vx_body, vy_body, vz_body, yaw_rate=0.0)
        self.log_align_command(
            "LAND_ON_SMALL_ARUCO",
            self.small_marker_id,
            self.small_marker,
            vx_body,
            vy_body,
            vz_body,
        )
        self.publish_state()

    def on_timer(self) -> None:
        auto_states = {
            MissionState.TAKEOFF_CMD_SENT,
            MissionState.HOLD_BEFORE_GOTO,
            MissionState.GOTO_MARKER_REGION,
            MissionState.SEARCH_BIG_PATTERN,
            MissionState.ALIGN_BIG,
            MissionState.LOCK_BIG,
            MissionState.DESCEND_TO_SMALL_ALT,
            MissionState.SEARCH_SMALL,
            MissionState.ALIGN_SMALL,
            MissionState.LOCK_SMALL,
            MissionState.LAND_ON_SMALL_ARUCO,
            MissionState.DESCEND_TO_DROP_ALT,
            MissionState.DROP_PAYLOAD,
            MissionState.WAIT_DROP_DONE,
            MissionState.RTL_RETURN,
            MissionState.LAND,
        }

        if self.state in auto_states and not self.auto_allowed():
            self.prev_auto_state = self.state
            self.transition_to(MissionState.MANUAL_OVERRIDE)
            self.publish_state()
            return

        if self.state == MissionState.IDLE:
            self.publish_state()
            return

        if self.state == MissionState.WAIT_GUIDED:
            if self.auto_allowed():
                self.reset_runtime()
                self.transition_to(MissionState.TAKEOFF_CMD_SENT)
                self.publish_takeoff(self.takeoff_alt_m)
            self.publish_state()
            return

        if self.state == MissionState.TAKEOFF_CMD_SENT:
            rel_alt = self.current_rel_alt()
            if rel_alt is not None and rel_alt >= (self.takeoff_alt_m - self.alt_reached_tol_m):
                self.transition_to(MissionState.HOLD_BEFORE_GOTO)
                self.hold_start_time_s = self.now_s()
            self.publish_state()
            return

        if self.state == MissionState.HOLD_BEFORE_GOTO:
            if self.hold_start_time_s is None:
                self.hold_start_time_s = self.now_s()

            if (self.now_s() - self.hold_start_time_s) >= self.hold_before_goto_s:
                self.transition_to(MissionState.GOTO_MARKER_REGION)

            self.publish_state()
            return

        if self.state == MissionState.GOTO_MARKER_REGION:
            if not self.goto_sent:
                self.publish_goto_search_center(self.takeoff_alt_m, self.goto_speed_mps)
                self.goto_sent = True

            if self.entered_search_region():
                self.publish_hold_here(self.takeoff_alt_m)
                self.transition_to(MissionState.SEARCH_BIG_PATTERN)
                self.search_big_start_time_s = self.now_s()
                self.start_search_loop(0)

            self.publish_state()
            return

        if self.state == MissionState.SEARCH_BIG_PATTERN:
            self.handle_search_big_pattern()
            return

        if self.state == MissionState.ALIGN_BIG:
            self.handle_align_big()
            return

        if self.state == MissionState.LOCK_BIG:
            self.handle_lock_big()
            return

        if self.state == MissionState.DESCEND_TO_SMALL_ALT:
            self.handle_descend_to_small_alt()
            return

        if self.enable_land_after_lock_small and self.state in {
            MissionState.SEARCH_SMALL,
            MissionState.ALIGN_SMALL,
            MissionState.LOCK_SMALL,
            MissionState.LAND_ON_SMALL_ARUCO,
        }:
            if self.land_target_reached():
                rel_alt = self.current_rel_alt()
                aruco_z = self.small_aruco_z_m()

                rel_text = "N/A" if rel_alt is None else f"{rel_alt:.2f}"
                z_text = "N/A" if aruco_z is None else f"{aruco_z:.2f}"

                self.get_logger().warn(
                    f"Small landing trigger reached: rel_alt={rel_text}m, "
                    f"aruco_z={z_text}m; switching to LAND"
                )

                self.publish_zero_vel()
                self.publish_land()
                self.transition_to(MissionState.LAND)
                self.publish_state()
                return

        if self.state == MissionState.SEARCH_SMALL:
            self.handle_search_small()
            return

        if self.state == MissionState.ALIGN_SMALL:
            self.handle_align_small()
            return

        if self.state == MissionState.LOCK_SMALL:
            self.handle_lock_small()
            return

        if self.state == MissionState.DESCEND_TO_DROP_ALT:
            self.handle_descend_to_drop_alt()
            return

        if self.state == MissionState.DROP_PAYLOAD:
            self.handle_drop_payload()
            return

        if self.state == MissionState.WAIT_DROP_DONE:
            self.handle_wait_drop_done()
            return

        if self.state == MissionState.RTL_RETURN:
            self.handle_rtl_return()
            return

        if self.state == MissionState.LAND_ON_SMALL_ARUCO:
            self.handle_land_on_small_aruco()
            return

        if self.state == MissionState.LAND:
            if not self.land_sent:
                self.publish_land()
                self.land_sent = True

            if not self.is_armed():
                self.transition_to(MissionState.DONE)

            self.publish_state()
            return

        if self.state == MissionState.MANUAL_OVERRIDE:
            self.publish_state()
            return

        if self.state == MissionState.DONE:
            self.publish_state()
            return


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
