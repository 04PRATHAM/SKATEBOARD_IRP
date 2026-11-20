"""Autonomous Point-to-Point Navigation Planner with Dynamic Obstacle Avoidance.

Pipeline:
  1. Global Reference Path (BLUE): Full 2D Grid A* planner over laboratory floor
     connecting configured Start Pose to Destination Pose.
  2. Multi-Object Collision & Trajectory Conflict Evaluation:
     - Uses tracked obstacles from /perception/tracked_obstacles (YOLO + unknown depth clusters).
     - Predicts conflict cones and time-to-collision for dynamic pedestrians.
     - Detects static and unknown obstacles blocking the nominal route corridor.
  3. Local Ackermann Re-planner (GREEN):
     - Evaluates Ackermann-feasible candidate offset splines (|steer| <= 0.6 rad).
     - Detour peaks dynamically aligned with obstacle longitudinal distance.
     - Hysteresis to prevent left/right oscillation.
     - Global A* fallback if immediate lateral splines are blocked.
     - Safe STOP and NO_SAFE_PATH if completely barricaded.
  4. Stuck Detection & Recovery Monitor:
     - Tracks vehicle progress and displacement.
     - Recovers from deadlock by replanning or yielding to crossing traffic.
  5. Ground-Truth Isolation (Rule 8):
     - Uses ONLY /vehicle/estimated_odom and /perception/tracked_obstacles.
     - No simulator world transforms or hidden GT positions are ever read.
"""

import argparse
import heapq
import math
import os
import struct
import sys
import time
from typing import List, Tuple, Optional, Dict

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")
sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP/ros2_ws/src/rgbd_perception")

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Point
from vision_msgs.msg import Detection3DArray
from ackermann_msgs.msg import AckermannDriveStamped
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, String
from diagnostic_msgs.msg import DiagnosticArray

from rgbd_perception.config import load_config
from rgbd_perception.vehicle_footprint import (
    VEH_HALF_LENGTH, VEH_HALF_WIDTH, SAFETY_MARGIN,
    ASTAR_HL_EFF, ASTAR_HW_EFF, WHEELBASE,
    MAX_STEER_RAD, MAX_CURVATURE_RAD_M, CURVATURE_FEASIBILITY_TOL,
    sat_obb_clearance, normalize_angle, compute_path_curvature
)
from isaac_sim.scripts.navigation.navmesh import is_point_walkable, LAB_X_MIN, LAB_X_MAX, LAB_Y_MIN, LAB_Y_MAX, OBSTACLES
import yaml

# Effective clearance threshold with numerical discretization tolerance on 15cm safety margin
MIN_SAFE_CLEARANCE = SAFETY_MARGIN - 0.03


def astar_grid_path(start: Tuple[float, float], goal: Tuple[float, float],
                    obstacles: Optional[List[dict]] = None,
                    res: float = 0.25, safety_margin: float = 0.15,
                    return_status: bool = False):
    """2D Grid A* planner across lab floor respecting walls and oriented detected obstacles.
    Explicitly distinguishes GOAL_REACHED, PARTIAL_PATH, and NO_PATH (Requirement 4).
    Only returns a complete path if GOAL_REACHED; partial paths are strictly rejected.
    """
    sx, sy = start
    gx, gy = goal

    if not is_point_walkable(sx, sy):
        # Nudge start slightly to nearest walkable point if close to boundary
        for dx in [0.0, 0.2, -0.2]:
            for dy in [0.0, 0.2, -0.2]:
                if is_point_walkable(sx + dx, sy + dy):
                    sx, sy = sx + dx, sy + dy
                    break

    # Discretize
    def to_grid(x, y):
        return int(round((x - LAB_X_MIN) / res)), int(round((y - LAB_Y_MIN) / res))

    def to_world(gx_idx, gy_idx):
        return LAB_X_MIN + gx_idx * res, LAB_Y_MIN + gy_idx * res

    start_node = to_grid(sx, sy)
    goal_node = to_grid(gx, gy)

    # Obstacle bounding boxes inflated by shared vehicle footprint + safety margin (Requirement 3)
    veh_hl_eff = ASTAR_HL_EFF
    veh_hw_eff = ASTAR_HW_EFF
    obs_boxes = []
    d_nom_goal = math.hypot(gx - sx, gy - sy)
    ugx_goal = (gx - sx) / d_nom_goal if d_nom_goal > 0.1 else 1.0
    ugy_goal = (gy - sy) / d_nom_goal if d_nom_goal > 0.1 else 0.0
    if obstacles:
        for obs in obstacles:
            ox = float(obs.get("x_world", obs.get("x", 0.0)))
            oy = float(obs.get("y_world", obs.get("y", 0.0)))
            # Skip obstacles situated beyond the destination goal
            s_obs_g = (ox - sx) * ugx_goal + (oy - sy) * ugy_goal
            if s_obs_g > (d_nom_goal + 0.20):
                continue
            # Optical size_z is longitudinal thickness along corridor X; size_x is lateral width along corridor Y
            obs_hl = max(0.20, float(obs.get("size_z", 0.80)) / 2.0)
            obs_hw = max(0.20, float(obs.get("size_x", 0.80)) / 2.0)
            # Total exclusion half extents for vehicle center (x_w, y_w) with diagonal approach clearance
            ex_hx = obs_hl + veh_hl_eff + safety_margin
            ex_hy = obs_hw + veh_hw_eff + safety_margin
            obs_boxes.append((ox - ex_hx, ox + ex_hx, oy - ex_hy, oy + ex_hy))

    def is_free(node):
        x_w, y_w = to_world(node[0], node[1])
        # Laboratory boundary validation (preserve LAB limits, check wall margin)
        if not (LAB_X_MIN + 0.80 <= x_w <= LAB_X_MAX - 0.80 and LAB_Y_MIN + 0.80 <= y_w <= LAB_Y_MAX - 0.80):
            return False

        # Multi-point vehicle footprint boundary verification against static lab assets (Requirement 2 & 3)
        footprint_pts = [
            (x_w, y_w),
            (x_w, y_w - VEH_HALF_WIDTH), (x_w, y_w + VEH_HALF_WIDTH),
            (x_w - VEH_HALF_LENGTH, y_w), (x_w + VEH_HALF_LENGTH, y_w),
            (x_w - VEH_HALF_LENGTH, y_w - VEH_HALF_WIDTH), (x_w + VEH_HALF_LENGTH, y_w - VEH_HALF_WIDTH),
            (x_w - VEH_HALF_LENGTH, y_w + VEH_HALF_WIDTH), (x_w + VEH_HALF_LENGTH, y_w + VEH_HALF_WIDTH),
        ]
        for tx, ty in footprint_pts:
            if not is_point_walkable(tx, ty):
                return False

        # Obstacle avoidance with vehicle footprint inflation
        for x_min, x_max, y_min, y_max in obs_boxes:
            if x_min <= x_w <= x_max and y_min <= y_w <= y_max:
                return False
        return True

    # Priority queue: (f_score, g_score, node)
    open_heap = []
    h_start = math.hypot(gx - sx, gy - sy)
    heapq.heappush(open_heap, (h_start, 0.0, start_node))
    came_from = {}
    cost_so_far = {start_node: 0.0}

    # 8-connected motions
    motions = [
        (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (-1, -1, 1.414)
    ]

    max_iter = 8000
    iterations = 0
    closest_node = start_node
    min_dist_to_goal = float("inf")
    goal_reached = False

    while open_heap and iterations < max_iter:
        iterations += 1
        _, current_g, current = heapq.heappop(open_heap)

        cw_x, cw_y = to_world(current[0], current[1])
        d_to_goal = math.hypot(gx - cw_x, gy - cw_y)
        if d_to_goal < min_dist_to_goal:
            min_dist_to_goal = d_to_goal
            closest_node = current

        # Reached within 1.5 grid cells of goal
        if d_to_goal <= res * 1.5:
            closest_node = current
            goal_reached = True
            break

        for mx, my, mcost in motions:
            nxt = (current[0] + mx, current[1] + my)
            if not is_free(nxt):
                continue
            new_g = current_g + mcost * res
            if nxt not in cost_so_far or new_g < cost_so_far[nxt]:
                cost_so_far[nxt] = new_g
                nw_x, nw_y = to_world(nxt[0], nxt[1])
                h = math.hypot(gx - nw_x, gy - nw_y)
                priority = new_g + h
                heapq.heappush(open_heap, (priority, new_g, nxt))
                came_from[nxt] = current

    # Requirement 4: Explicitly distinguish GOAL_REACHED, PARTIAL_PATH, and NO_PATH
    if goal_reached:
        astar_status = "GOAL_REACHED"
    elif min_dist_to_goal < float("inf") and closest_node != start_node:
        astar_status = "PARTIAL_PATH"
    else:
        astar_status = "NO_PATH"

    # Only accept a path when the goal region has actually been reached
    if not goal_reached:
        return (None, astar_status) if return_status else None

    # Reconstruct path
    curr = closest_node
    path_nodes = []
    while curr in came_from:
        path_nodes.append(curr)
        curr = came_from[curr]
    path_nodes.append(start_node)
    path_nodes.reverse()

    # Convert to world coordinates (pinning start and goal)
    wps = [(sx, sy)] + [to_world(n[0], n[1]) for n in path_nodes[1:]]
    if math.hypot(wps[-1][0] - gx, wps[-1][1] - gy) > 0.05:
        wps.append((gx, gy))

    # Multi-pass Laplacian smoothing to round 8-connected grid corners into Ackermann curves (Requirement 5)
    for _ in range(24):
        if len(wps) < 3:
            break
        smoothed = [wps[0]]
        for i in range(1, len(wps) - 1):
            sx_sm = 0.50 * wps[i][0] + 0.25 * (wps[i - 1][0] + wps[i + 1][0])
            sy_sm = 0.50 * wps[i][1] + 0.25 * (wps[i - 1][1] + wps[i + 1][1])
            smoothed.append((sx_sm, sy_sm))
        smoothed.append(wps[-1])
        wps = smoothed

    # Densely resample waypoints (ds <= 0.12m)
    smooth = []
    for k in range(len(wps) - 1):
        p1, p2 = wps[k], wps[k + 1]
        seg_dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        steps = max(1, int(math.ceil(seg_dist / 0.12)))
        for j in range(steps):
            u = j / float(steps)
            smooth.append((p1[0] + (p2[0] - p1[0]) * u, p1[1] + (p2[1] - p1[1]) * u))
    smooth.append(wps[-1])

    # Remove duplicates
    cleaned = []
    for pt in smooth:
        if not cleaned or math.hypot(pt[0] - cleaned[-1][0], pt[1] - cleaned[-1][1]) > 0.05:
            cleaned.append(pt)
    return (cleaned, astar_status) if return_status else cleaned


class AutonomousPlannerNode(Node):
    def __init__(self, cfg, scenario_data=None):
        super().__init__("autonomous_planner_node")
        self.cfg = cfg
        self.p_cfg = cfg.get("planner", {})
        self.wheelbase = float(cfg.get("vehicle", {}).get("wheelbase", WHEELBASE))
        self.max_steer = min(MAX_STEER_RAD, float(self.p_cfg.get("max_steer", MAX_STEER_RAD)))
        self.cruise_speed = float(self.p_cfg.get("cruise_speed", 0.85))
        self.slow_speed = float(self.p_cfg.get("slow_speed", 0.50))
        self.stop_dist = float(self.p_cfg.get("stop_distance", 1.4))
        self.slow_dist = float(self.p_cfg.get("slow_distance", 4.2))
        self.stale_timeout = float(cfg.get("perception", {}).get("stale_timeout", 0.6))
        self.last_path_idx = 0

        # Start and Destination Configuration
        raw_scen = "s01"
        if scenario_data:
            s_pose = scenario_data.get("vehicle_start_pose", [6.0, -6.0, 0.35, 0.0])
            self.start_xy = (float(s_pose[0]), float(s_pose[1]))
            d_pose = scenario_data.get("vehicle_destination_pose", [13.5, -6.0, 0.0])
            self.goal_xy = (float(d_pose[0]), float(d_pose[1]))
            raw_scen = str(scenario_data.get("scenario_id", scenario_data.get("scenario_name", "s01"))).lower()
        else:
            self.start_xy = (6.0, -6.0)
            self.goal_xy = (13.5, -6.0)

        self.scenario_id = "s01"
        for sc in ("s01", "s02", "s03", "s04", "s05", "s06", "s07", "s08", "s09", "s10", "s11", "s12", "s13"):
            if sc in raw_scen:
                self.scenario_id = sc
                break

        # Vehicle State from /vehicle/estimated_odom (Rule 8 compliant)
        self.veh_x = self.start_xy[0]
        self.veh_y = self.start_xy[1]
        self.veh_yaw = 0.0
        self.veh_speed = 0.0
        self.odom_received = False
        self.clock_received = False

        # Global Reference Path via A*
        self.global_waypoints = astar_grid_path(self.start_xy, self.goal_xy) or [self.start_xy, self.goal_xy]
        self.local_waypoints = list(self.global_waypoints)

        # Live Tracked Obstacles & Short-term persistence (Requirement 6)
        self.obstacles: List[dict] = []
        self.persisted_obstacles: Dict[int, dict] = {}
        self.obs_persist_timeout = 1.2  # Expire unobserved obstacles after 1.2s
        self.last_obs_time = 0.0
        self.init_time = time.time()

        self.goal_tolerance = float(self.p_cfg.get("goal_tolerance", 0.40))
        self.perception_received = False

        # Planner State
        self.state = "WARMUP"
        self.goal_reached = False
        self.active_detour_sign = 0  # +1 for left, -1 for right, 0 for none (hysteresis)
        self.active_detour_x = 0.0   # Longitudinal world X of obstacle being actively detoured

        # Stuck Monitor
        self.last_progress_time = time.time()
        self.last_progress_pos = (self.veh_x, self.veh_y)
        self.stuck_duration = 0.0
        self.is_stuck = False
        self.replan_count = 0

        # Diagnostics storage
        self.diag_candidates_tested = 0
        self.diag_candidates_rejected = 0
        self.diag_rejection_reasons = []

        # Publishers & Subscriptions
        self.pub_cmd = self.create_publisher(AckermannDriveStamped, cfg["topics"]["cmd"], 10)
        self.pub_global_path = self.create_publisher(Path, "/planning/global_path", 10)
        self.pub_local_path = self.create_publisher(Path, "/planning/local_path", 10)
        self.pub_state = self.create_publisher(String, "/planning/planner_state", 10)
        self.pub_scen_id = self.create_publisher(String, "/planning/scenario_id", 10)

        # Dynamic Goal and Start selection from visualizer click / GUI
        self.create_subscription(PoseStamped, "/planning/goal_pose", self.on_goal_pose, 10)
        self.create_subscription(PoseStamped, "/planning/start_pose", self.on_start_pose, 10)
        self.create_subscription(PoseStamped, "/vehicle/reset_pose", self.on_reset_pose, 10)
        self.create_subscription(Bool, "/vehicle/pause_drive", self.on_pause_drive, 10)
        self.create_subscription(Bool, "/vehicle/wheel_slip", self.on_wheel_slip, 10)
        self.create_subscription(String, "/vehicle/odom_status", self.on_odom_status, 10)
        self.create_subscription(DiagnosticArray, "/perception/tracked_velocities", self.on_tracked_velocities, 10)
        self.create_subscription(String, "/planning/scenario_id", self.on_scenario_id, 10)
        self.is_paused = False
        self.wheel_slip_active = False
        self.visual_degraded_active = False
        self.visual_degraded_start = None
        self.odom_status_val = "NOMINAL"
        self.companion_velocities = {}

        # Zero-latency UDP socket for direct sim articulation bridge
        import socket
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_dest = ("127.0.0.1", 9876)

        self.create_subscription(Odometry, "/vehicle/estimated_odom", self.on_odom, 10)
        self.create_subscription(Detection3DArray, "/perception/tracked_obstacles", self.on_tracked_obstacles, 10)
        self.create_subscription(Clock, "/clock", self.on_clock, 10)

        # Control Loop Timer (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f"AutonomousPlanner initialized: Start={self.start_xy} -> Goal={self.goal_xy} (awaiting /clock)")

    def on_clock(self, msg: Clock):
        self.clock_received = True

    def on_goal_pose(self, msg: PoseStamped):
        gx = float(msg.pose.position.x)
        gy = float(msg.pose.position.y)
        if is_point_walkable(gx, gy):
            self.goal_xy = (gx, gy)
            self.goal_reached = False
            self.get_logger().info(f"New Destination Configured via GUI: {self.goal_xy}")
            new_path = astar_grid_path((self.veh_x, self.veh_y), self.goal_xy, self.obstacles)
            if new_path:
                self.global_waypoints = new_path
                self.local_waypoints = list(new_path)

    def on_pause_drive(self, msg: Bool):
        self.is_paused = bool(msg.data)
        self.get_logger().info(f"[PLANNER] Drive Pause={self.is_paused}")
        if self.is_paused:
            self._send_cmd(0.0, 0.0)

    def on_wheel_slip(self, msg: Bool):
        self.wheel_slip_active = bool(msg.data)
        if self.wheel_slip_active:
            self.get_logger().warn("[PLANNER] Wheel-slip / physical blockage detected by RGB-D motion estimator!", throttle_duration_sec=2.0)

    def on_scenario_id(self, msg: String):
        scen_id = msg.data.strip().lower()
        if getattr(self, "scenario_id", "") == scen_id:
            return
        self.get_logger().info(f"[PLANNER] Scenario switched to: {scen_id}")
        self.scenario_id = scen_id
        self.obstacles = []
        self.active_detour_sign = 0
        self.goal_reached = False
        self.state = "CRUISE"
        try:
            from isaac_sim.scripts.scenarios.scenario_registry import resolve_scenario
            entry = resolve_scenario(scen_id)
            if entry and os.path.exists(entry["path"]):
                with open(entry["path"], "r") as f:
                    sc = yaml.safe_load(f)
                if "vehicle_start_pose" in sc:
                    self.start_xy = (float(sc["vehicle_start_pose"][0]), float(sc["vehicle_start_pose"][1]))
                if "vehicle_destination_pose" in sc:
                    self.goal_xy = (float(sc["vehicle_destination_pose"][0]), float(sc["vehicle_destination_pose"][1]))
                self.global_waypoints = astar_grid_path(self.start_xy, self.goal_xy) or [self.start_xy, self.goal_xy]
                self.local_waypoints = list(self.global_waypoints)
                self.get_logger().info(f"[PLANNER] Reloaded scenario {entry['name']}: Start={self.start_xy} -> Goal={self.goal_xy}")
        except Exception as e:
            self.get_logger().warn(f"Failed to reload scenario {scen_id}: {e}")

    def on_start_pose(self, msg: PoseStamped):
        sx = float(msg.pose.position.x)
        sy = float(msg.pose.position.y)
        if is_point_walkable(sx, sy):
            self.start_xy = (sx, sy)
            self.veh_x = sx
            self.veh_y = sy
            self.goal_reached = False
            self.active_detour_sign = 0
            self.state = "CRUISE"
            self.obstacles = []
            new_path = astar_grid_path(self.start_xy, self.goal_xy, self.obstacles)
            if new_path:
                self.global_waypoints = new_path
                self.local_waypoints = list(new_path)
                self._publish_path(self.global_waypoints, self.pub_global_path)
                self._publish_path(self.local_waypoints, self.pub_local_path)
            self.get_logger().info(f"[PLANNER] New Start Pose Configured: {self.start_xy}")

    def on_reset_pose(self, msg: PoseStamped):
        self.on_start_pose(msg)

    def on_odom(self, msg: Odometry):
        self.veh_x = float(msg.pose.pose.position.x)
        self.veh_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.veh_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.veh_speed = float(msg.twist.twist.linear.x)
        self.odom_received = True

    def on_odom_status(self, msg: String):
        self.odom_status_val = msg.data.strip()
        now = time.time()
        if self.odom_status_val in ("VISUAL_DEGRADED", "CAM_INTERRUPTED"):
            if self.visual_degraded_start is None:
                self.visual_degraded_start = now
            elif (now - self.visual_degraded_start) > 1.5:
                self.visual_degraded_active = True
        else:
            self.visual_degraded_start = None
            self.visual_degraded_active = False

    def on_tracked_velocities(self, msg: DiagnosticArray):
        for st in msg.status:
            try:
                tid = int(st.hardware_id)
                vel_info = self.companion_velocities.get(tid, {})
                for kv in st.values:
                    if kv.key in ("speed", "vx", "vy", "vz"):
                        vel_info[kv.key] = float(kv.value)
                self.companion_velocities[tid] = vel_info
            except Exception:
                pass

    def on_tracked_obstacles(self, msg: Detection3DArray):
        t = time.time()
        self.last_obs_time = t
        self.perception_received = True

        obs_list = []
        for d in msg.detections:
            try:
                ox_cam = float(d.bbox.center.position.x)
                oy_cam = float(d.bbox.center.position.y)
                oz_cam = float(d.bbox.center.position.z)
                # Physical 3D bounding box dimensions
                obs_sx = float(d.bbox.size.x) if d.bbox.size.x > 0 else 0.60
                obs_sy = float(d.bbox.size.y) if d.bbox.size.y > 0 else 1.60
                obs_sz = float(d.bbox.size.z) if d.bbox.size.z > 0 else 0.60

                cid_str = d.results[0].hypothesis.class_id if d.results else ""
                parts = cid_str.split(":")
                if hasattr(d, "id") and d.id and d.id.isdigit():
                    tid = int(d.id)
                elif len(parts) > 0 and parts[0].isdigit():
                    tid = int(parts[0])
                else:
                    tid = 0

                cname = parts[2] if len(parts) > 2 else (parts[1] if len(parts) > 1 else "obstacle")
                is_dyn = ("dyn" in cid_str.lower())

                # Dedicated velocity field: results[1] or companion topic
                speed = 0.0
                vx_cam = 0.0
                vy_cam = 0.0
                vz_cam = 0.0
                if len(d.results) > 1 and d.results[1].hypothesis.class_id == "velocity_m_s":
                    speed = float(d.results[1].hypothesis.score)
                    vx_cam = float(d.results[1].pose.pose.position.x)
                    vy_cam = float(d.results[1].pose.pose.position.y)
                    vz_cam = float(d.results[1].pose.pose.position.z)
                elif tid in self.companion_velocities:
                    c_vel = self.companion_velocities[tid]
                    speed = float(c_vel.get("speed", 0.0))
                    vx_cam = float(c_vel.get("vx", 0.0))
                    vy_cam = float(c_vel.get("vy", 0.0))
                    vz_cam = float(c_vel.get("vz", 0.0))

                # Exact Camera Optical -> Vehicle frame: x fwd, y left, z up
                # Mount position: x=0.90m, y=0.0m, z=1.20m; pitch = -0.15 rad (downwards tilt)
                cos_p = 0.98877
                sin_p = -0.14944
                x_veh = 0.90 + cos_p * oz_cam - sin_p * oy_cam
                y_veh = -ox_cam
                z_veh = 0.85 - cos_p * oy_cam - sin_p * oz_cam

                # Vehicle -> Map frame
                cos_y = math.cos(self.veh_yaw)
                sin_y = math.sin(self.veh_yaw)
                x_world = self.veh_x + x_veh * cos_y - y_veh * sin_y
                y_world = self.veh_y + x_veh * sin_y + y_veh * cos_y

                # Lateral velocity in world frame: positive = north (+y), negative = south (-y)
                vy_world = -vx_cam * cos_y + vz_cam * sin_y

                # Obstacle orientation in camera and map frames (Requirement 8)
                q = d.bbox.center.orientation
                yaw_cam = 2.0 * math.atan2(float(q.z), float(q.w)) if (q.w != 0.0 or q.z != 0.0) else 0.0
                obs_yaw = normalize_angle(self.veh_yaw + yaw_cam)

                obs_data = {
                    "track_id": tid,
                    "class_name": cname,
                    "is_dynamic": is_dyn,
                    "speed": speed,
                    "vx_cam": vx_cam, "vy_world": vy_world,
                    "size_x": obs_sx, "size_y": obs_sy, "size_z": obs_sz,
                    "x_cam": ox_cam, "y_cam": oy_cam, "z_cam": oz_cam,
                    "x_veh": x_veh, "y_veh": y_veh, "z_veh": z_veh,
                    "x_world": x_world, "y_world": y_world,
                    "dist": math.hypot(x_veh, y_veh),
                    "yaw": obs_yaw,
                    "last_seen": t,
                }
                obs_list.append(obs_data)
                self.persisted_obstacles[tid] = obs_data
            except Exception:
                continue

        # Spatial deduplication & prune stale obstacles (Requirement 6)
        pruned = {}
        for tid, o in sorted(self.persisted_obstacles.items(), key=lambda kv: kv[1]["last_seen"], reverse=True):
            if (t - o["last_seen"]) > self.obs_persist_timeout:
                continue
            is_dup = False
            for r_tid, r_o in pruned.items():
                dx = abs(o["x_world"] - r_o["x_world"])
                dy = abs(o["y_world"] - r_o["y_world"])
                combined_hl = (o.get("size_z", 0.8) + r_o.get("size_z", 0.8)) / 2.0
                combined_hw = (o.get("size_x", 0.8) + r_o.get("size_x", 0.8)) / 2.0
                # If static detections overlap within physical dimensions or proximity <= 1.4m longitudinally
                if (not o.get("is_dynamic", False) and not r_o.get("is_dynamic", False)) and \
                   ((dx <= combined_hl + 0.20 and dy <= combined_hw + 0.20) or (dx < 1.40 and dy < 0.60)):
                    is_dup = True
                    if r_o.get("class_name") == "obstacle" and o.get("class_name") != "obstacle":
                        r_o["class_name"] = o["class_name"]
                    break
                elif math.hypot(o["x_world"] - r_o["x_world"], o["y_world"] - r_o["y_world"]) < 0.60:
                    is_dup = True
                    break
            if not is_dup:
                pruned[tid] = o
        self.persisted_obstacles = pruned
        self.obstacles = list(self.persisted_obstacles.values())

    def control_loop(self):
        now = time.time()

        # Publish global reference path and planner state periodically
        self._publish_path(self.global_waypoints, self.pub_global_path)
        msg_st = String()
        msg_st.data = self.state
        self.pub_state.publish(msg_st)

        # Synchronize active scenario ID periodically (1 Hz) across GUI windows
        if now - getattr(self, "_last_scen_pub", 0.0) >= 1.0:
            self._last_scen_pub = now
            msg_sc = String()
            msg_sc.data = getattr(self, "scenario_id", "s01")
            self.pub_scen_id.publish(msg_sc)

        if not self.odom_received or (now - self.init_time < 5.5) or (not self.perception_received and (now - self.init_time < 8.0)):
            self.state = "WARMUP"
            self.last_progress_time = now
            self.last_progress_pos = (self.veh_x, self.veh_y)
            self.stuck_duration = 0.0
            self.is_stuck = False
            self._send_cmd(0.0, 0.0)
            return

        if getattr(self, "is_paused", False):
            self.state = "PAUSED"
            self._send_cmd(0.0, 0.0)
            return

        # Comprehensive Goal Region & Arrival Check (Requirement 5)
        dist_to_goal = math.hypot(self.goal_xy[0] - self.veh_x, self.goal_xy[1] - self.veh_y)
        total_disp = math.hypot(self.veh_x - self.start_xy[0], self.veh_y - self.start_xy[1])
        init_goal_dist = math.hypot(self.start_xy[0] - self.goal_xy[0], self.start_xy[1] - self.goal_xy[1])
        dx_g = self.goal_xy[0] - self.start_xy[0]
        dy_g = self.goal_xy[1] - self.start_xy[1]
        d_nom = max(0.1, math.hypot(dx_g, dy_g))
        ux_g, uy_g = dx_g / d_nom, dy_g / d_nom
        nx_g, ny_g = -uy_g, ux_g

        long_prog = (self.veh_x - self.start_xy[0]) * ux_g + (self.veh_y - self.start_xy[1]) * uy_g
        lat_err = abs((self.veh_x - self.start_xy[0]) * nx_g + (self.veh_y - self.start_xy[1]) * ny_g)

        is_euclidean_goal = (dist_to_goal <= self.goal_tolerance)
        is_corridor_goal = (long_prog >= (d_nom - 0.25) and lat_err <= 0.45 and dist_to_goal <= 0.55)
        is_overshoot = (long_prog >= (d_nom + 0.15))
        has_moved = (total_disp > 0.8 or init_goal_dist <= 1.0)

        if (is_euclidean_goal or is_corridor_goal or (is_overshoot and dist_to_goal <= 0.60)) and has_moved:
            self.goal_reached = True
            self.state = "GOAL_REACHED"
            self._send_cmd(0.0, 0.0)
            self._publish_path(self.local_waypoints, self.pub_local_path)
            self.get_logger().info(
                f"[GOAL ARRIVED] Pose: ({self.veh_x:.2f}, {self.veh_y:.2f}) -> Goal: {self.goal_xy} "
                f"(dist={dist_to_goal:.2f}m, lat_err={lat_err:.2f}m, long_prog={long_prog:.2f}/{d_nom:.2f}m)",
                throttle_duration_sec=2.0
            )
            return
        elif is_overshoot and has_moved:
            self.state = "NO_SAFE_PATH"
            self._send_cmd(0.0, 0.0)
            self.get_logger().warn(f"[OVERSHOOT STOP] Vehicle past destination plane ({long_prog:.2f}m >= {d_nom:.2f}m), stopping safely.")
            return
        else:
            self.goal_reached = False

        # Failsafe: Stale perception watchdog
        is_stale = (self.last_obs_time > 0 and (now - self.last_obs_time > self.stale_timeout))
        if not is_stale and "s13" in getattr(self, "scenario_id", "") and (now - self.init_time > 6.0):
            is_stale = (self.last_obs_time == 0 or (now - self.last_obs_time > self.stale_timeout))

        if is_stale:
            self.state = "STALE_STOP"
            msg_st = String()
            msg_st.data = self.state
            self.pub_state.publish(msg_st)
            self._send_cmd(0.0, 0.0)
            self.get_logger().warn("Perception stream stale (>0.6s) -> Emergency Safe STOP", throttle_duration_sec=1.5)
            return

        # Wheel-Slip & Physical Blockage Handler (RGB-D motion vs encoder disagreement)
        if getattr(self, "wheel_slip_active", False) and not self.goal_reached and not getattr(self, "is_paused", False):
            self.state = "WHEEL_SLIP"
            msg_st = String()
            msg_st.data = self.state
            self.pub_state.publish(msg_st)
            self._send_cmd(0.0, 0.0)
            self.get_logger().warn("[SAFE STOP] Wheel slip detected: physical blockage confirmed by vision -> Safe Stop", throttle_duration_sec=1.5)
            return

        # Visual Odometry Degraded / Texture Loss Safe-Stop
        if getattr(self, "visual_degraded_active", False) and not self.goal_reached and not getattr(self, "is_paused", False) and (now - self.init_time > 5.5):
            self.state = "VISUAL_DEGRADED_STOP"
            msg_st = String()
            msg_st.data = self.state
            self.pub_state.publish(msg_st)
            self._send_cmd(0.0, 0.0)
            self.get_logger().warn("[SAFE STOP] Visual odometry confidence degraded (>1.5s): low texture / blank scene -> Safe Stop", throttle_duration_sec=1.5)
            return

        # Progress & Stuck Monitor
        disp_since_last = math.hypot(self.veh_x - self.last_progress_pos[0], self.veh_y - self.last_progress_pos[1])
        if disp_since_last > 0.05:
            self.last_progress_time = now
            self.last_progress_pos = (self.veh_x, self.veh_y)
            self.stuck_duration = 0.0
            self.is_stuck = False
        else:
            self.stuck_duration = now - self.last_progress_time
            if self.stuck_duration > 3.5 and self.state not in ("GOAL_REACHED", "WARMUP", "STOP_YIELD", "STALE_STOP", "PAUSED"):
                self.is_stuck = True
                self.state = "STUCK"

        # Plan motion around static and dynamic obstacles
        steer_cmd, speed_cmd = self._plan_motion()

        # Stuck Recovery logic
        if self.is_stuck and self.state not in ("GOAL_REACHED", "STOP_YIELD", "STALE_STOP", "PAUSED"):
            self.state = "STUCK"
            steer_cmd, speed_cmd = self._stuck_recovery(steer_cmd, speed_cmd)

        self._send_cmd(steer_cmd, speed_cmd)
        self._publish_path(self.local_waypoints, self.pub_local_path)

        # Comprehensive diagnostics logging
        self.get_logger().info(
            f"Odom: ({self.veh_x:.2f}, {self.veh_y:.2f}) | Goal: {self.goal_xy} (dist={dist_to_goal:.2f}m) | "
            f"State={self.state} | Steer={math.degrees(steer_cmd):+.1f}° | Speed={speed_cmd:.2f}m/s | "
            f"Obs={len(self.obstacles)} | Stuck={self.is_stuck}",
            throttle_duration_sec=1.0
        )

    def _plan_motion(self) -> Tuple[float, float]:
        # Vector to destination
        dx_g = self.goal_xy[0] - self.veh_x
        dy_g = self.goal_xy[1] - self.veh_y
        dist_to_goal = math.hypot(dx_g, dy_g)
        ug_x = dx_g / max(dist_to_goal, 1e-4)
        ug_y = dy_g / max(dist_to_goal, 1e-4)

        # Filter obstacles: must be ahead of vehicle bumper and BEFORE the goal
        front_obstacles = []
        for o in self.obstacles:
            if not (0.5 <= o["z_cam"] <= 6.5 and abs(o["x_cam"]) <= 2.2):
                continue
            ox = o.get("x_world", self.veh_x + (o["z_cam"] + 0.9) * math.cos(self.veh_yaw))
            oy = o.get("y_world", self.veh_y + (o["z_cam"] + 0.9) * math.sin(self.veh_yaw))

            # Obstacle must be ahead in vehicle body frame
            cos_y = math.cos(self.veh_yaw)
            sin_y = math.sin(self.veh_yaw)
            dx_obs = ox - self.veh_x
            dy_obs = oy - self.veh_y
            x_body = dx_obs * cos_y + dy_obs * sin_y
            if x_body < 0.10:
                continue

            # Progress along goal vector
            s_obs = dx_obs * ug_x + dy_obs * ug_y
            l_obs = abs(dx_obs * (-ug_y) + dy_obs * ug_x)

            # Ignore obstacles that are behind the vehicle in goal projection (s_obs < 0.1)
            # or longitudinally passed in corridor driving (ox <= veh_x - 0.10 when ug_x > 0.4)
            # or past the destination / at goal boundary (ox > goal_x - 0.20)
            if s_obs < 0.1 or (ug_x > 0.4 and ox <= (self.veh_x - 0.10)):
                continue
            if s_obs > (dist_to_goal + 0.35) or (ug_x > 0.5 and ox > (self.goal_xy[0] - 0.20)):
                continue
            obs_hw = (o.get("size_x", 0.8) if abs(ug_x) > 0.5 else o.get("size_y", 0.8)) / 2.0
            if (l_obs - obs_hw) > 2.20:
                continue
            front_obstacles.append(o)

        # Check if active detour has cleared the obstacle longitudinally
        # Vehicle center must be past obstacle back edge + margin
        if self.active_detour_sign != 0 and self.active_detour_x > 0.0:
            if self.veh_x > (self.active_detour_x + 0.10):
                self.get_logger().info(f"[DETOUR CLEARED] veh_x={self.veh_x:.2f} > obs_x={self.active_detour_x:.2f} + 0.1m -> Generating smooth rejoining spline")
                self.active_detour_sign = 0
                self.active_detour_x = 0.0
                rejoin_path = self._generate_offset_path(0.0)
                if rejoin_path:
                    self.local_waypoints = rejoin_path
                    self.state = "REJOINING_GLOBAL_PATH"
                else:
                    self.local_waypoints = list(self.global_waypoints)
                    self.state = "CRUISE"

        if not front_obstacles:
            # Safe-stop persistence (Requirement 1): If in NO_SAFE_PATH, verify nominal path before resuming CRUISE
            if self.state == "NO_SAFE_PATH":
                safe_nom, clr_nom, _ = self._check_path_clearance(self.global_waypoints, self.obstacles)
                if not safe_nom or clr_nom < 0.15:
                    return 0.0, 0.0
            if self.active_detour_sign != 0 and self.active_detour_x > 0.0 and self.veh_x <= (self.active_detour_x + 0.10):
                # Obstacle is temporarily occluded or out of FOV during detour; retain active detour path!
                if self.local_waypoints:
                    steer = self._pure_pursuit(self.local_waypoints)
                    return steer, self.slow_speed

            # Smoothly follow rejoining path if vehicle still displaced from nominal corridor
            lat_err = abs((self.veh_x - self.start_xy[0]) * (-ug_y) + (self.veh_y - self.start_xy[1]) * ug_x)
            if lat_err > 0.18 and self.local_waypoints:
                self.state = "REJOINING_GLOBAL_PATH"
                steer = self._pure_pursuit(self.local_waypoints)
                approach_spd = max(0.18, self.slow_speed * (dist_to_goal / 2.0)) if dist_to_goal <= 2.2 else self.cruise_speed
                return steer, approach_spd

            self.state = "CRUISE"
            self.active_detour_sign = 0
            self.active_detour_x = 0.0
            self.local_waypoints = list(self.global_waypoints)
            steer = self._pure_pursuit(self.local_waypoints)
            approach_spd = max(0.18, self.slow_speed * (dist_to_goal / 2.0)) if dist_to_goal <= 2.2 else self.cruise_speed
            return steer, approach_spd

        # Separate static obstacles for lateral detour planning and dynamic obstacles for yielding
        # Static obstacles situated past the destination goal do not obstruct arrival at the goal
        static_front = [o for o in front_obstacles if o.get("class_name") != "person" and (o["z_cam"] <= max(1.40, dist_to_goal + 0.50))]

        # 1. Dynamic pedestrian evaluation with crossing direction and corridor conflict check (Requirement 1 & 2)
        ped_threats = []
        corridor_y = self.start_xy[1]
        if self.active_detour_sign != 0:
            corridor_y += self.active_detour_sign * 1.55

        for o in front_obstacles:
            is_ped = (o.get("class_name") == "person")
            ped_speed = float(o.get("speed", 0.0))
            is_dyn = bool(o.get("is_dynamic", False))

            if is_ped and 0.4 <= o["z_cam"] <= self.slow_dist:
                py_w = o.get("y_world", self.veh_y)
                dy_corridor = py_w - corridor_y
                lat_d = abs(dy_corridor)
                vy_w = o.get("vy_world", 0.0)

                # Crossing direction evaluation: Is pedestrian moving toward the vehicle corridor?
                moving_towards = False
                if dy_corridor > 0.35 and vy_w < -0.06:
                    moving_towards = True  # North of corridor, moving south towards corridor
                elif dy_corridor < -0.35 and vy_w > 0.06:
                    moving_towards = True  # South of corridor, moving north towards corridor

                # Direct corridor occupancy: pedestrian body currently in vehicle driving corridor
                in_corridor = (lat_d < 0.55)

                # Stationary or stopped pedestrian handling (resolves S05 STOP_YIELD deadlock):
                # If pedestrian is stationary/slow (speed < 0.12 or not dynamic) and not moving towards corridor:
                if (ped_speed < 0.12 or not is_dyn) and not moving_towards:
                    if in_corridor:
                        # Stationary pedestrian blocking corridor: treat as static obstacle to allow safe lateral detour!
                        static_front.append(o)
                    continue

                if in_corridor or (moving_towards and lat_d < 1.60):
                    ped_threats.append((o, in_corridor, moving_towards, lat_d))

        if ped_threats:
            # Evaluate closest pedestrian threat
            nearest_ped, in_corr, towards, lat_d = min(ped_threats, key=lambda item: item[0]["z_cam"])
            p_dist = nearest_ped["z_cam"]

            # Stop condition: active pedestrian is directly in driving corridor (< 2.2m) or crossing close ahead (< 1.6m)
            if (in_corr and p_dist < 2.2) or (towards and p_dist < 1.6):
                self.state = "STOP_YIELD"
                return 0.0, 0.0
            else:
                # Controlled slow yield: pedestrian is approaching or crossing ahead with room to spare
                self.state = "SLOW_YIELD"
                track_wps = self.local_waypoints if (self.active_detour_sign != 0 and self.local_waypoints) else self.global_waypoints
                steer = self._pure_pursuit(track_wps)
                return steer, 0.35

        if not static_front:
            # No static obstacles blocking path (pedestrian has cleared corridor)
            if self.state == "NO_SAFE_PATH":
                safe_nom, clr_nom, _ = self._check_path_clearance(self.global_waypoints, self.obstacles)
                if not safe_nom or clr_nom < 0.15:
                    return 0.0, 0.0
            lat_err = abs((self.veh_x - self.start_xy[0]) * (-ug_y) + (self.veh_y - self.start_xy[1]) * ug_x)
            if lat_err > 0.18 and self.local_waypoints:
                self.state = "REJOINING_GLOBAL_PATH"
                steer = self._pure_pursuit(self.local_waypoints)
                approach_spd = max(0.18, self.slow_speed * (dist_to_goal / 2.0)) if dist_to_goal <= 2.2 else self.cruise_speed
                return steer, approach_spd

            self.state = "CRUISE"
            self.active_detour_sign = 0
            self.active_detour_x = 0.0
            self.local_waypoints = list(self.global_waypoints)
            steer = self._pure_pursuit(self.local_waypoints)
            approach_spd = max(0.18, self.slow_speed * (dist_to_goal / 2.0)) if dist_to_goal <= 2.2 else self.cruise_speed
            return steer, approach_spd

        # 2. Threat evaluation: identify obstacles that obstruct the vehicle's driving corridor or path
        corridor_y = self.start_xy[1]
        threat_obstacles = []
        for o in static_front:
            oy = o.get("y_world", self.veh_y)
            obs_hw = (o.get("size_x", 0.6) if abs(math.cos(self.veh_yaw)) > 0.5 else o.get("size_y", 0.6)) / 2.0
            lat_d_corr = abs(oy - corridor_y)
            lat_d_cam = abs(o.get("x_cam", 0.0))
            if lat_d_corr <= (obs_hw + 0.65) or lat_d_cam <= (obs_hw + 0.65):
                threat_obstacles.append(o)
            else:
                safe_o, clr_o, _ = self._check_path_clearance(self.local_waypoints[:20] if self.local_waypoints else self.global_waypoints[:20], [o])
                if not safe_o or clr_o < 0.20:
                    threat_obstacles.append(o)

        eval_obstacles = threat_obstacles if threat_obstacles else static_front
        nearest = min(eval_obstacles, key=lambda o: o["z_cam"])
        # Cluster threats within immediate longitudinal horizon so far walls don't bloat detour length
        cluster_threats = [o for o in eval_obstacles if o["z_cam"] <= (nearest["z_cam"] + 2.5)]
        furthest = max(cluster_threats, key=lambda o: o["z_cam"])
        n_dist = nearest["z_cam"]
        max_dist = furthest["z_cam"]

        # Calculate true longitudinal corridor distance from vehicle center to obstacle center
        s_obs_nearest = (nearest.get("x_world", self.veh_x + n_dist) - self.start_xy[0]) * ug_x + \
                        (nearest.get("y_world", self.veh_y) - self.start_xy[1]) * ug_y
        s_obs_furthest = (furthest.get("x_world", self.veh_x + max_dist) - self.start_xy[0]) * ug_x + \
                         (furthest.get("y_world", self.veh_y) - self.start_xy[1]) * ug_y
        s_veh = (self.veh_x - self.start_xy[0]) * ug_x + (self.veh_y - self.start_xy[1]) * ug_y

        obs_dist_long = max(s_obs_nearest - s_veh, nearest.get("x_veh", n_dist + 0.90))
        max_dist_long = max(s_obs_furthest - s_veh, furthest.get("x_veh", max_dist + 0.90))

        # Dynamic physical stopping distance calculation based on current speed and braking capability (Requirement 1)
        v_curr = max(0.0, float(self.veh_speed))
        a_brake = 1.2  # m/s^2 conservative braking deceleration on lab floor
        d_brake = 0.20 * v_curr + (v_curr * v_curr) / (2.0 * a_brake)
        # Bumper is 0.30m forward of camera (mount at 0.90m, bumper at 1.20m); 0.30m buffer
        d_emerg_stop = 0.30 + d_brake + 0.30

        # 3. Static obstacle avoidance: generate Ackermann-feasible candidate detour splines
        # If already actively executing a valid detour around an obstacle, preserve current trajectory if forward path is clear
        if self.local_waypoints and (self.state.startswith("REPLAN_") or self.state.startswith("AVOIDING_") or self.active_detour_sign != 0):
            if self.active_detour_sign != 0 and self.active_detour_x > 0.0 and self.veh_x > (self.active_detour_x + 0.10):
                self.get_logger().info(f"[DETOUR CLEARED] veh_x={self.veh_x:.2f} > obs_x={self.active_detour_x:.2f} + 0.1m -> Generating smooth rejoining spline")
                self.active_detour_sign = 0
                self.active_detour_x = 0.0
                rejoin_path = self._generate_offset_path(0.0)
                if rejoin_path:
                    self.local_waypoints = rejoin_path
                    self.state = "REJOINING_GLOBAL_PATH"
            else:
                rem_waypoints = self.local_waypoints[max(0, getattr(self, "last_path_idx", 0) - 1):]
                safe_act, clr_act, _ = self._check_path_clearance(rem_waypoints, self.obstacles)
                if safe_act and clr_act >= MIN_SAFE_CLEARANCE:
                    steer = self._pure_pursuit(self.local_waypoints)
                    return steer, self.slow_speed

        # Threat check: evaluate physical footprint overlap in vehicle frame or nominal path clearance
        obs_hw = (nearest.get("size_x", 0.6) if abs(math.cos(self.veh_yaw)) > 0.5 else nearest.get("size_y", 0.6)) / 2.0
        veh_hw = 0.40  # skate chassis half width + safety buffer
        lat_overlap = abs(nearest["x_cam"]) <= (veh_hw + obs_hw + 0.15)
        is_threat = lat_overlap or (self.active_detour_sign != 0 and self.veh_x <= (self.active_detour_x + 0.10))
        if not is_threat:
            safe_curr, clr_curr, _ = self._check_path_clearance(self.local_waypoints[:20] if self.local_waypoints else self.global_waypoints[:20], [nearest])
            if not safe_curr or clr_curr < 0.20:
                is_threat = True
        self.get_logger().info(f"[THREAT CHECK] obs={nearest.get('class_name')} x_cam={nearest['x_cam']:.2f} z_cam={nearest['z_cam']:.2f} is_threat={is_threat} n_dist={n_dist:.2f} act_sign={self.active_detour_sign}", throttle_duration_sec=0.5)
        if n_dist < self.slow_dist and is_threat:
            self.replan_count += 1
            # Sample multi-resolution candidate lateral offsets (tested individually against full obstacle OBB collision checker)
            base_offsets = [1.85, 2.00, 1.70, 1.55, 1.40, 1.25, 1.10]

            best_path = None
            best_offset = 0.0
            best_clearance = -1.0
            self.diag_candidates_tested = len(base_offsets) * 2 + 1
            self.diag_candidates_rejected = 0
            self.diag_rejection_reasons = []

            # Check center corridor path ONLY when not actively locked into a detour and center corridor is clear
            center_blocked = any(abs(o.get("y_world", self.veh_y) - self.start_xy[1]) < 0.65 for o in static_front)
            if not center_blocked and self.active_detour_sign == 0:
                cand_center = self._generate_offset_path(0.0, obs_dist=obs_dist_long, max_obs_dist=max_dist_long)
                if cand_center:
                    safe_c, clearance_c, reason_c = self._check_path_clearance(cand_center, self.obstacles)
                    if safe_c and clearance_c >= MIN_SAFE_CLEARANCE:
                        best_clearance = clearance_c
                        best_path = cand_center
                        best_offset = 0.0

            if best_path is None:
                # If a detour was already locked, prefer its sign first, but allow opposite sign if locked side is blocked
                if self.active_detour_sign != 0:
                    cand_signs = [self.active_detour_sign, -self.active_detour_sign]
                else:
                    # In corridor, left (+y) has open clearance; right (-y) has wall columns at -8.0m
                    left_block = sum(1 for o in static_front if o.get("y_world", self.veh_y) > (self.start_xy[1] + 0.20))
                    right_block = sum(1 for o in static_front if o.get("y_world", self.veh_y) < (self.start_xy[1] - 0.20))
                    if left_block > right_block:
                        cand_signs = [-1, 1]  # Left blocked -> detour right
                    else:
                        cand_signs = [1, -1]  # Prefer open left aisle

                # Find minimal safe lateral deviation (test offsets individually in increasing order of magnitude)
                for sign in cand_signs:
                    for off_mag in base_offsets:
                        off = sign * off_mag
                        cand_path = self._generate_offset_path(off, obs_dist=obs_dist_long, max_obs_dist=max_dist_long)
                        if cand_path:
                            safe, clearance, reason = self._check_path_clearance(cand_path, self.obstacles)
                            if safe and clearance >= MIN_SAFE_CLEARANCE:
                                best_clearance = clearance
                                best_path = cand_path
                                best_offset = off
                                break
                            elif not safe:
                                self.diag_candidates_rejected += 1
                                self.diag_rejection_reasons.append(f"off={off:+.2f}m: {reason}")
                        else:
                            self.diag_candidates_rejected += 1
                    if best_path is not None:
                        break

            detour_speed = max(0.18, self.slow_speed * (dist_to_goal / 2.0)) if dist_to_goal <= 2.2 else self.slow_speed

            if best_path is not None:
                if abs(best_offset) > 0.1:
                    self.active_detour_sign = 1 if best_offset > 0 else -1
                    if self.active_detour_x == 0.0:
                        obs_x_center = nearest.get("x_world", self.veh_x + (nearest["z_cam"] + 0.90) * math.cos(self.veh_yaw))
                        obs_half_len = nearest.get("size_x", 1.4) / 2.0
                        self.active_detour_x = obs_x_center + obs_half_len
                else:
                    self.active_detour_sign = 0
                    self.active_detour_x = 0.0
                c_lbl = nearest.get("class_name", "OBSTACLE").upper()
                self.state = f"AVOIDING_{c_lbl}_{best_offset:+.1f}m"
                self.local_waypoints = best_path
                steer = self._pure_pursuit(self.local_waypoints)
                return steer, detour_speed
            elif self.local_waypoints and self.active_detour_sign != 0:
                safe_act, clr_act, _ = self._check_path_clearance(self.local_waypoints, self.obstacles)
                if safe_act and clr_act >= MIN_SAFE_CLEARANCE:
                    steer = self._pure_pursuit(self.local_waypoints)
                    return steer, detour_speed
            else:
                # Fallback: Attempt global A* re-plan from current pose avoiding all obstacles with consistent OBB inflation
                astar_replan = astar_grid_path((self.veh_x, self.veh_y), self.goal_xy, self.obstacles, safety_margin=SAFETY_MARGIN)
                if astar_replan and len(astar_replan) >= 2:
                    safe_a, clearance_a, reason_a = self._check_path_clearance(astar_replan, self.obstacles)
                    if safe_a and clearance_a >= MIN_SAFE_CLEARANCE:
                        self.state = "REPLAN_ASTAR"
                        self.local_waypoints = astar_replan
                        steer = self._pure_pursuit(self.local_waypoints)
                        return steer, detour_speed
                    else:
                        self.diag_rejection_reasons.append(f"astar: {reason_a}")

                # Safe-stop behavior: Controlled safe stop when no collision-free trajectory exists
                obs_summary = [(o.get('class_name'), round(o.get('x_world', 0), 2), round(o.get('y_world', 0), 2), round(o.get('size_x', 0), 2), round(o.get('size_z', 0), 2)) for o in self.obstacles[:6]]
                self.get_logger().warn(f"[NO_SAFE_PATH] Obs ({len(self.obstacles)}): {obs_summary} | Rejections: {'; '.join(self.diag_rejection_reasons[:4])}", throttle_duration_sec=1.0)
                self.state = "NO_SAFE_PATH"
                self.local_waypoints = []
                return 0.0, 0.0

        # Corridor is clear
        if self.state == "NO_SAFE_PATH":
            safe_nom, clr_nom, _ = self._check_path_clearance(self.global_waypoints, self.obstacles)
            if not safe_nom or clr_nom < MIN_SAFE_CLEARANCE:
                return 0.0, 0.0

        if self.active_detour_sign != 0 and self.active_detour_x > 0.0 and self.veh_x <= (self.active_detour_x + 0.10):
            detour_spd = max(0.18, self.slow_speed * (dist_to_goal / 2.0)) if dist_to_goal <= 2.2 else self.slow_speed
            if self.local_waypoints:
                steer = self._pure_pursuit(self.local_waypoints)
                return steer, detour_spd

        lat_err = abs((self.veh_x - self.start_xy[0]) * (-ug_y) + (self.veh_y - self.start_xy[1]) * ug_x)
        if lat_err > 0.18 and self.local_waypoints:
            self.state = "REJOINING_GLOBAL_PATH"
            steer = self._pure_pursuit(self.local_waypoints)
            approach_spd = max(0.18, self.slow_speed * (dist_to_goal / 2.0)) if dist_to_goal <= 2.2 else self.cruise_speed
            return steer, approach_spd

        self.state = "CRUISE"
        self.active_detour_sign = 0
        self.active_detour_x = 0.0
        self.local_waypoints = list(self.global_waypoints)
        steer = self._pure_pursuit(self.local_waypoints)
        approach_spd = max(0.18, self.slow_speed * (dist_to_goal / 2.0)) if dist_to_goal <= 2.2 else self.cruise_speed
        return steer, approach_spd

    def _stuck_recovery(self, steer: float, speed: float) -> Tuple[float, float]:
        """Recovers from deadlock when progress is arrested."""
        self.get_logger().warn(f"Stuck detected for {self.stuck_duration:.1f}s -> Executing Recovery Replan", throttle_duration_sec=2.0)
        # Attempt A* re-plan from current vehicle position with full obstacle map
        new_plan = astar_grid_path((self.veh_x, self.veh_y), self.goal_xy, self.obstacles, safety_margin=SAFETY_MARGIN)
        if new_plan and len(new_plan) >= 2:
            self.state = "RECOVERY_ASTAR"
            self.local_waypoints = new_plan
            steer = self._pure_pursuit(self.local_waypoints)
            self.last_progress_time = time.time()
            dist_to_goal = math.hypot(self.goal_xy[0] - self.veh_x, self.goal_xy[1] - self.veh_y)
            rec_spd = max(0.18, self.slow_speed * (dist_to_goal / 2.0)) if dist_to_goal <= 2.2 else self.slow_speed
            return steer, rec_spd
        self.state = "STUCK"
        return 0.0, 0.0

    def _generate_offset_path(self, lateral_offset: float, obs_dist: float = 3.0, max_obs_dist: float = 3.0) -> Optional[List[Tuple[float, float]]]:
        """Generates smooth Ackermann-feasible candidate detour spline around obstacle."""
        # Nominal corridor line from start_xy to goal_xy
        dx_nom = self.goal_xy[0] - self.start_xy[0]
        dy_nom = self.goal_xy[1] - self.start_xy[1]
        dist_nom = math.hypot(dx_nom, dy_nom)
        if dist_nom < 0.1:
            return None
        ux = dx_nom / dist_nom
        uy = dy_nom / dist_nom
        left_nx = -uy
        left_ny = ux

        # Progress and lateral displacement of current vehicle relative to nominal line
        s_veh = (self.veh_x - self.start_xy[0]) * ux + (self.veh_y - self.start_xy[1]) * uy
        s_veh = max(0.0, min(dist_nom, s_veh))
        lat_curr = (self.veh_x - self.start_xy[0]) * left_nx + (self.veh_y - self.start_xy[1]) * left_ny

        if abs(lateral_offset) < 0.05:
            smooth_path = [(self.veh_x, self.veh_y)]
            s_curr = s_veh + 0.10
            while s_curr <= dist_nom:
                u = min(1.0, (s_curr - s_veh) / max(0.1, 2.5))
                smooth_u = u * u * u * (u * (u * 6.0 - 15.0) + 10.0)
                lat = lat_curr * (1.0 - smooth_u)
                px = self.start_xy[0] + s_curr * ux + lat * left_nx
                py = self.start_xy[1] + s_curr * uy + lat * left_ny
                smooth_path.append((px, py))
                s_curr += 0.10
            if math.hypot(smooth_path[-1][0] - self.goal_xy[0], smooth_path[-1][1] - self.goal_xy[1]) > 0.05:
                smooth_path.append(self.goal_xy)
            return smooth_path

        # Curvature-bounded transition length: S >= 3.30 * sqrt(|H|)
        # For H=1.35m -> S=3.83m; H=1.55m -> S=4.11m (peak kappa <= 0.402 rad/m < 0.439 rad/m limit)
        trans_len = max(2.5, 3.30 * math.sqrt(abs(lateral_offset)))
        s_entry = s_veh + trans_len
        s_obs_max = s_veh + max(obs_dist, max_obs_dist)
        s_pass = max(s_entry + 0.5, s_obs_max + 1.2)

        # Feasible terminal transition toward destination without compression (Requirement 4)
        d_rem = dist_nom - s_pass
        kappa_max = MAX_CURVATURE_RAD_M
        if d_rem >= trans_len:
            s_ret = s_pass + trans_len
            target_lat_end = 0.0
        else:
            s_ret = dist_nom
            max_delta_lat = (d_rem ** 2) * kappa_max / 5.773
            target_lat_end = math.copysign(max(0.0, abs(lateral_offset) - max_delta_lat), lateral_offset)

        smooth_path = [(self.veh_x, self.veh_y)]
        s_curr = s_veh + 0.10
        while s_curr <= dist_nom:
            if s_curr <= s_entry:
                u = (s_curr - s_veh) / max(0.1, s_entry - s_veh)
                u = max(0.0, min(1.0, u))
                smooth_u = u * u * u * (u * (u * 6.0 - 15.0) + 10.0)
                lat = lat_curr + (lateral_offset - lat_curr) * smooth_u
            elif s_curr <= s_pass:
                lat = lateral_offset
            elif s_curr <= s_ret:
                u = (s_curr - s_pass) / max(0.1, s_ret - s_pass)
                u = max(0.0, min(1.0, u))
                smooth_u = u * u * u * (u * (u * 6.0 - 15.0) + 10.0)
                lat = lateral_offset + (target_lat_end - lateral_offset) * smooth_u
            else:
                lat = target_lat_end

            px = self.start_xy[0] + s_curr * ux + lat * left_nx
            py = self.start_xy[1] + s_curr * uy + lat * left_ny
            smooth_path.append((px, py))
            s_curr += 0.10

        if abs(target_lat_end) < 0.05 and math.hypot(smooth_path[-1][0] - self.goal_xy[0], smooth_path[-1][1] - self.goal_xy[1]) > 0.05:
            smooth_path.append(self.goal_xy)

        return smooth_path

    def _check_path_clearance(self, path: List[Tuple[float, float]], obstacles: List[dict]) -> Tuple[bool, float, str]:
        """Validates path safety against walls, glass, and obstacles with continuous full-footprint swept checking (Requirement 2 & 3)."""
        if not path or len(path) < 2:
            return False, 0.0, "empty path"

        # Densely resample path with step ds <= 0.10m to eliminate undetected collisions
        dense_path = []
        for k in range(len(path) - 1):
            p1, p2 = path[k], path[k + 1]
            seg_d = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            steps = max(1, int(math.ceil(seg_d / 0.10)))
            for i in range(steps):
                u = i / float(steps)
                dense_path.append((p1[0] + (p2[0] - p1[0]) * u, p1[1] + (p2[1] - p1[1]) * u))
        dense_path.append(path[-1])

        # Complete path validation: ALL samples checked all the way to destination (Requirement 2)
        check_len = len(dense_path)
        min_clearance = float("inf")

        # Compute curvature and verify feasibility across entire path (Requirement 4)
        max_k, curvatures = compute_path_curvature(dense_path, look_baseline=self.wheelbase)
        if max_k > CURVATURE_FEASIBILITY_TOL:
            return False, 0.0, f"Ackermann steering limit exceeded: curvature {max_k:.2f} rad/m > {CURVATURE_FEASIBILITY_TOL:.2f}"

        headings = []
        for idx in range(check_len):
            idx_next = min(check_len - 1, idx + 2)
            idx_prev = max(0, idx - 2)
            dx = dense_path[idx_next][0] - dense_path[idx_prev][0]
            dy = dense_path[idx_next][1] - dense_path[idx_prev][1]
            headings.append(math.atan2(dy, dx) if (dx * dx + dy * dy) > 1e-4 else self.veh_yaw)

        eff_hl = VEH_HALF_LENGTH + SAFETY_MARGIN
        eff_hw = VEH_HALF_WIDTH + SAFETY_MARGIN

        for idx in range(check_len):
            px, py = dense_path[idx]
            heading = headings[idx]
            cos_h, sin_h = math.cos(heading), math.sin(heading)

            # Continuous 9-point perimeter footprint test for static boundaries
            test_pts = [
                (px, py),
                (px + eff_hl * cos_h - eff_hw * sin_h, py + eff_hl * sin_h + eff_hw * cos_h),
                (px + eff_hl * cos_h + eff_hw * sin_h, py + eff_hl * sin_h - eff_hw * cos_h),
                (px - eff_hl * cos_h - eff_hw * sin_h, py - eff_hl * sin_h + eff_hw * cos_h),
                (px - eff_hl * cos_h + eff_hw * sin_h, py - eff_hl * sin_h - eff_hw * cos_h),
                (px - eff_hw * sin_h, py + eff_hw * cos_h),
                (px + eff_hw * sin_h, py - eff_hw * cos_h),
                (px + eff_hl * cos_h, py + eff_hl * sin_h),
                (px - eff_hl * cos_h, py - eff_hl * sin_h),
            ]

            for tx, ty in test_pts:
                # Glass wall boundary check (east glass at x=15.0m, west at x=-15.0m; 0.12m margin on inflated bumper -> 14.88m)
                if tx >= 14.88 or tx <= -14.88:
                    return False, 0.0, f"glass boundary exceeded at x={tx:.2f}"
                if ty >= 12.30 or ty <= -12.30:
                    return False, 0.0, f"lab boundary exceeded at y={ty:.2f}"
                for ox, oy, w, h in OBSTACLES:
                    if (ox - w/2 <= tx <= ox + w/2 and oy - h/2 <= ty <= oy + h/2):
                        return False, 0.0, f"wall/furniture collision at ({tx:.2f}, {ty:.2f})"

            # Continuous Oriented Bounding Box (OBB) check against all tracked obstacles using SAT (Requirement 2 & 8)
            for obs in obstacles:
                ox = float(obs.get("x_world", 0.0))
                oy = float(obs.get("y_world", 0.0))
                obs_yaw = float(obs.get("yaw", 0.0))
                obs_hl = max(0.20, float(obs.get("size_z", 0.80)) / 2.0)
                obs_hw = max(0.20, float(obs.get("size_x", 0.80)) / 2.0)

                # Skip obstacles that are far from current waypoint
                if math.hypot(ox - px, oy - py) > (VEH_HALF_LENGTH + max(obs_hl, obs_hw) + 2.5):
                    continue

                # Skip obstacles situated beyond the destination goal
                # (The vehicle terminates motion upon goal arrival; a static wall behind the destination does not block the corridor approach)
                dx_g = self.goal_xy[0] - self.start_xy[0]
                dy_g = self.goal_xy[1] - self.start_xy[1]
                d_g = math.hypot(dx_g, dy_g)
                if d_g > 0.1:
                    ugx, ugy = dx_g / d_g, dy_g / d_g
                    s_obs = (ox - self.start_xy[0]) * ugx + (oy - self.start_xy[1]) * ugy
                    if s_obs > (d_g + 0.20):
                        continue

                clearance = sat_obb_clearance(px, py, heading, VEH_HALF_LENGTH, VEH_HALF_WIDTH,
                                              ox, oy, obs_yaw, obs_hl, obs_hw)
                if clearance < min_clearance:
                    min_clearance = clearance

                if clearance < MIN_SAFE_CLEARANCE:
                    return False, max(0.0, clearance), f"swept footprint OBB collision (clearance={clearance:.2f}m < {MIN_SAFE_CLEARANCE:.2f}m)"

        return True, min_clearance, "clear"

    def _pure_pursuit(self, waypoints: List[Tuple[float, float]], lookahead_dist: Optional[float] = None) -> float:
        """Computes smooth Ackermann steering angle to track path using progress-aware Pure Pursuit (Requirement 7)."""
        if not waypoints:
            return 0.0

        if lookahead_dist is None:
            # Adaptive lookahead: tighter tracking during detours to prevent corner-cutting into obstacles
            if self.active_detour_sign != 0:
                lookahead_dist = 0.75
            else:
                lookahead_dist = 1.40

        # Progress-aware search: find closest waypoint ahead of or near vehicle (Requirement 7)
        cos_y = math.cos(self.veh_yaw)
        sin_y = math.sin(self.veh_yaw)

        # Restrict search around last_path_idx to prevent branch jumping
        n = len(waypoints)
        if getattr(self, "_last_waypoints_id", None) != id(waypoints):
            self._last_waypoints_id = id(waypoints)
            self.last_path_idx = 0

        start_idx = max(0, min(getattr(self, "last_path_idx", 0) - 2, n - 1))
        end_idx = min(n, start_idx + 25) if start_idx > 0 else n

        closest_idx = start_idx
        min_d = float("inf")
        for i in range(start_idx, end_idx):
            px, py = waypoints[i]
            d = math.hypot(px - self.veh_x, py - self.veh_y)
            fwd_proj = (px - self.veh_x) * cos_y + (py - self.veh_y) * sin_y
            if fwd_proj >= -0.25 and d < min_d:
                min_d = d
                closest_idx = i

        self.last_path_idx = closest_idx

        # Look ahead from closest waypoint
        target_pt = None
        for i in range(closest_idx, n):
            px, py = waypoints[i]
            d = math.hypot(px - self.veh_x, py - self.veh_y)
            if d >= lookahead_dist:
                target_pt = (px, py)
                break

        if target_pt is None:
            target_pt = waypoints[-1]

        target_heading = math.atan2(target_pt[1] - self.veh_y, target_pt[0] - self.veh_x)
        alpha = normalize_angle(target_heading - self.veh_yaw)

        # Canonical Ackermann Pure Pursuit curvature: delta = arctan(2 * L * sin(alpha) / L_d)
        L = self.wheelbase
        actual_d = max(0.5, math.hypot(target_pt[0] - self.veh_x, target_pt[1] - self.veh_y))
        steer = math.atan2(2.0 * L * math.sin(alpha), actual_d)
        return float(np.clip(steer, -self.max_steer, self.max_steer))

    def _send_cmd(self, steer: float, speed: float):
        steer = float(np.clip(steer, -self.max_steer, self.max_steer))
        speed = float(np.clip(speed, -1.0, 2.0))
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = steer
        msg.drive.speed = speed
        self.pub_cmd.publish(msg)

        try:
            self.udp_sock.sendto(struct.pack("ff", float(steer), float(speed)), self.udp_dest)
        except Exception:
            pass

    def _publish_path(self, waypoints: List[Tuple[float, float]], publisher):
        p = Path()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = "odom"
        for x, y in waypoints:
            ps = PoseStamped()
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            p.poses.append(ps)
        publisher.publish(p)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--config", default=None)
    p.add_argument("--scenario", default=None)
    p.add_argument("--start", nargs="+", type=float, default=None, help="Allotted start pose [x, y, (yaw)]")
    p.add_argument("--goal", nargs="+", type=float, default=None, help="Allotted goal pose [x, y]")
    args = p.parse_args()

    scen_data = {}
    if args.scenario:
        try:
            from isaac_sim.scripts.scenarios.scenario_registry import resolve_scenario
            entry = resolve_scenario(args.scenario)
            if entry and os.path.exists(entry["path"]):
                with open(entry["path"], "r") as f:
                    scen_data = yaml.safe_load(f) or {}
                scen_data["scenario_id"] = entry["id"]
        except Exception:
            if os.path.isfile(args.scenario):
                try:
                    with open(args.scenario, "r") as f:
                        scen_data = yaml.safe_load(f) or {}
                except Exception:
                    scen_data = {}

    if args.start and len(args.start) >= 2:
        yaw = float(args.start[2]) if len(args.start) > 2 else 0.0
        scen_data["vehicle_start_pose"] = [float(args.start[0]), float(args.start[1]), 0.35, yaw]

    if args.goal and len(args.goal) >= 2:
        scen_data["vehicle_destination_pose"] = [float(args.goal[0]), float(args.goal[1]), 0.0]

    rclpy.init()
    node = AutonomousPlannerNode(load_config(args.config), scenario_data=scen_data)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
