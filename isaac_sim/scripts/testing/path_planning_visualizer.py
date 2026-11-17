"""Top-Down Path Planning & Trajectory Visualizer (WINDOW 3).

Interactive Autonomous Navigation GUI:
  - Real-time 2D top-down map of the laboratory
  - Scenario Selector: Cycle between all 13 scenarios (S01 - S13)
  - Interactive Start & Goal Selection: Click buttons or press 'S' / 'G'
  - Physical Vehicle Repositioning & Odometry Reset via UDP bridge & ROS 2
  - Ground-truth scenario static obstacles and pedestrian trajectories
  - Real-time perception overlays (YOLO boxes, depth clusters, dynamic tracks)
  - Global reference path (A*), local detour spline, and actual vehicle trajectory
  - Full HUD with speed, steering, physical vs estimated poses, delta error, and safety clearance
  - Keyboard shortcuts: 'S'=Start, 'G'=Goal, 'R'=Reset, 'P'/Space=Pause, Esc=E-Stop, '['/']'=Scenario
  - Automated categorized evidence logging into results/scenario_sXX/
"""

import argparse
import json
import math
import os
import socket
import struct
import sys
import time
from collections import deque
from pathlib import Path
from typing import List, Tuple, Optional, Dict, Any

for p in [
    "/home/ubuntu/SKATEBOARD_IRP",
    "/home/ubuntu/SKATEBOARD_IRP/ros2_ws/src/rgbd_perception",
    "/home/ubuntu/ros_overlay/opt/ros/humble/local/lib/python3.10/dist-packages",
    "/opt/ros/humble/local/lib/python3.10/dist-packages",
    "/opt/ros/humble/lib/python3.10/site-packages",
]:
    if p not in sys.path:
        sys.path.insert(0, p)

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path as RosPath, Odometry
from geometry_msgs.msg import PoseStamped, Quaternion
from vision_msgs.msg import Detection3DArray
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Bool, String, Float32

from isaac_sim.scripts.navigation.navmesh import (
    OBSTACLES, LAB_X_MIN, LAB_X_MAX, LAB_Y_MIN, LAB_Y_MAX, is_point_walkable
)
import yaml

ROOT = "/home/ubuntu/SKATEBOARD_IRP"
SCEN_DIR = os.path.join(ROOT, "configs/scenarios")

from isaac_sim.scripts.scenarios.scenario_registry import (
    SCENARIOS_DATA as SCENARIOS, resolve_scenario, get_all_scenarios, get_scenario_by_index
)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class TopDownVisualizerNode(Node):
    def __init__(self, args):
        super().__init__("path_planning_visualizer")
        self.args = args
        self.out_base_dir = Path(args.out_dir)
        self.out_base_dir.mkdir(parents=True, exist_ok=True)
        self.save_every = args.save_every
        self.no_show = args.no_show
        self.last_save_time = time.time()
        self.frame_idx = 0

        # Scenario State
        self.current_scen_idx = 0
        if args.scenario:
            req_stem = Path(args.scenario).stem.lower()
            for idx, sc in enumerate(SCENARIOS):
                if sc["id"] in req_stem or Path(sc["file"]).stem in req_stem:
                    self.current_scen_idx = idx
                    break

        # Map display configuration (pixels)
        self.img_w = 1100
        self.img_h = 860
        self.map_x_min = 20
        self.map_x_max = 1080
        self.map_y_min = 70
        self.map_y_max = 770

        # Coordinate conversion bounds (meters)
        self.world_min_x = -16.0
        self.world_max_x = 16.0
        self.world_min_y = -14.0
        self.world_max_y = 14.0

        # UI & Interactive Mode
        # Modes: "NORMAL", "SET_START", "SET_GOAL"
        self.interact_mode = "NORMAL"
        self.is_paused = False
        self.mouse_world_pos: Tuple[float, float] = (0.0, 0.0)
        self.status_msg = "Ready. Use buttons or keys (S=Start, G=Goal, R=Reset, Space=Pause, Esc=Stop)."
        self.status_time = time.time()

        # UDP Socket for fast direct simulation commands
        self.sim_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sim_addr = ("127.0.0.1", 9876)

        # Navigation & Trajectory state
        self.global_path: List[Tuple[float, float]] = []
        self.local_path: List[Tuple[float, float]] = []
        self.actual_trail: deque = deque(maxlen=1000)
        self.est_trail: deque = deque(maxlen=1000)
        self.obstacles: List[dict] = []
        self.predicted_trajectories: List[List[Tuple[float, float]]] = []

        # Vehicle state
        self.veh_x = 6.0
        self.veh_y = -6.0
        self.veh_yaw = 0.0
        self.actual_x = 6.0
        self.actual_y = -6.0
        self.actual_yaw = 0.0
        self.cmd_speed = 0.0
        self.cmd_steer = 0.0
        self.planner_state = "CRUISE"

        # Scenario items
        self.static_obstacles: List[dict] = []
        self.scenario_pedestrians: List[dict] = []
        self.start_pt = (6.0, -6.0)
        self.goal_pt = (13.5, -6.0)

        # Publishers
        self.pub_goal = self.create_publisher(PoseStamped, "/planning/goal_pose", 10)
        self.pub_start = self.create_publisher(PoseStamped, "/planning/start_pose", 10)
        self.pub_reset = self.create_publisher(PoseStamped, "/vehicle/reset_pose", 10)
        self.pub_pause = self.create_publisher(Bool, "/vehicle/pause_drive", 10)
        self.pub_scen_id = self.create_publisher(String, "/planning/scenario_id", 10)

        # Subscriptions
        self.create_subscription(RosPath, "/planning/global_path", self.on_global_path, 10)
        self.create_subscription(RosPath, "/planning/local_path", self.on_local_path, 10)
        self.create_subscription(Odometry, "/vehicle/estimated_odom", self.on_est_odom, 10)
        self.create_subscription(Detection3DArray, "/perception/tracked_obstacles", self.on_obstacles, 10)
        self.create_subscription(AckermannDriveStamped, "/cmd_ackermann", self.on_cmd, 10)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 10)
        self.create_subscription(String, "/planning/planner_state", self.on_planner_state, 10)
        self.create_subscription(String, "/planning/switch_scenario", self.on_switch_scenario, 10)
        self.create_subscription(Bool, "/vehicle/wheel_slip", self.on_wheel_slip, 10)
        self.create_subscription(String, "/vehicle/odom_status", self.on_odom_status, 10)
        self.create_subscription(Float32, "/vehicle/visual_odom_confidence", self.on_vis_conf, 10)
        self.wheel_slip_active = False
        self.odom_status_val = "NOMINAL"
        self.vis_conf_val = 1.0

        # Ground-truth UDP socket from simulator (for offline evaluation)
        try:
            self.gt_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.gt_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.gt_sock.bind(("127.0.0.1", 9879))
            self.gt_sock.setblocking(False)
        except Exception:
            self.gt_sock = None

        # Load initial scenario
        self.load_scenario(self.current_scen_idx, notify_sim=False)

        # Setup window and mouse callback
        self.win_name = "Window 3 - Top-Down Path Planning & Trajectory Monitor"
        if not self.no_show:
            cv2.namedWindow(self.win_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.win_name, 960, 520)
            cv2.moveWindow(self.win_name, 960, 520)
            cv2.setMouseCallback(self.win_name, self.on_mouse_event)

        # Render timer (15 Hz)
        self.timer = self.create_timer(1.0 / 15.0, self.render_frame)
        self.get_logger().info("TopDownVisualizerNode running (Window 3 interactive GUI ready).")

    def on_planner_state(self, msg: String):
        self.planner_state = msg.data

    def load_scenario(self, scen_idx: int, notify_sim: bool = True):
        scen_idx = scen_idx % len(SCENARIOS)
        self.current_scen_idx = scen_idx
        scen_info = SCENARIOS[scen_idx]
        scen_file = os.path.join(SCEN_DIR, scen_info["file"])
        self.scenario_id = scen_info["id"]
        scen_folder = Path(scen_info["file"]).stem

        # Setup categorized evidence directories for this scenario
        self.cat_dirs = {
            "progress": self.out_base_dir / scen_folder / "progress",
            "obstacle": self.out_base_dir / scen_folder / "obstacle",
            "planned":  self.out_base_dir / scen_folder / "planned",
            "goal":     self.out_base_dir / scen_folder / "goal",
            "blocked":  self.out_base_dir / scen_folder / "blocked",
            "failed":   self.out_base_dir / scen_folder / "failed",
        }
        for d in self.cat_dirs.values():
            d.mkdir(parents=True, exist_ok=True)

        try:
            with open(scen_file, "r") as f:
                sc = yaml.safe_load(f)
            if getattr(self.args, "start", None) and len(self.args.start) >= 2:
                self.start_pt = (float(self.args.start[0]), float(self.args.start[1]))
            elif "vehicle_start_pose" in sc:
                self.start_pt = (float(sc["vehicle_start_pose"][0]), float(sc["vehicle_start_pose"][1]))

            if getattr(self.args, "goal", None) and len(self.args.goal) >= 2:
                self.goal_pt = (float(self.args.goal[0]), float(self.args.goal[1]))
            elif "vehicle_destination_pose" in sc:
                self.goal_pt = (float(sc["vehicle_destination_pose"][0]), float(sc["vehicle_destination_pose"][1]))
            self.static_obstacles = sc.get("static_obstacles", []) or []
            self.scenario_pedestrians = sc.get("pedestrians", []) or []
        except Exception as e:
            self.get_logger().warn(f"Failed to load scenario YAML {scen_file}: {e}")

        # Clear trajectory history
        self.actual_trail.clear()
        self.est_trail.clear()
        self.obstacles.clear()
        self.predicted_trajectories.clear()

        # Reposition vehicle to scenario start pose
        self.veh_x = self.start_pt[0]
        self.veh_y = self.start_pt[1]
        self.veh_yaw = 0.0
        self.actual_x = self.start_pt[0]
        self.actual_y = self.start_pt[1]
        self.actual_yaw = 0.0

        # Broadcast scenario ID to all nodes
        msg_scen = String()
        msg_scen.data = self.scenario_id
        if hasattr(self, "pub_scen_id"):
            self.pub_scen_id.publish(msg_scen)

        if notify_sim:
            # Send reload command (-888.0) to Isaac Sim
            try:
                pkt = struct.pack("ffff", -888.0, float(scen_idx), 0.0, 0.0)
                self.sim_sock.sendto(pkt, self.sim_addr)
            except Exception:
                pass

            # Notify ROS components
            self.publish_reset(self.start_pt[0], self.start_pt[1], 0.0)
            self.publish_goal(self.goal_pt[0], self.goal_pt[1])

        self.set_status(f"Loaded {scen_info['name']}")

    def on_switch_scenario(self, msg: String):
        raw = msg.data.strip()
        target_info = resolve_scenario(raw)
        if target_info:
            target_id = target_info["id"].lower()
            for idx, sc in enumerate(SCENARIOS):
                if sc["id"].lower() == target_id:
                    self.load_scenario(idx, notify_sim=True)
                    return
        target = raw.lower()
        for idx, sc in enumerate(SCENARIOS):
            if sc["id"].lower() == target or str(idx) == target or sc["name"].lower() == target:
                self.load_scenario(idx, notify_sim=True)
                break

    def on_wheel_slip(self, msg: Bool):
        self.wheel_slip_active = bool(msg.data)

    def on_odom_status(self, msg: String):
        self.odom_status_val = msg.data.strip()

    def on_vis_conf(self, msg: Float32):
        self.vis_conf_val = float(msg.data)

    def set_status(self, msg: str):
        self.status_msg = msg
        self.status_time = time.time()

    def world_to_pixel(self, wx: float, wy: float) -> Tuple[int, int]:
        px = int(self.map_x_min + (wx - self.world_min_x) / (self.world_max_x - self.world_min_x) * (self.map_x_max - self.map_x_min))
        py = int(self.map_y_min + (self.world_max_y - wy) / (self.world_max_y - self.world_min_y) * (self.map_y_max - self.map_y_min))
        return max(0, min(self.img_w - 1, px)), max(0, min(self.img_h - 1, py))

    def pixel_to_world(self, px: int, py: int) -> Tuple[float, float]:
        wx = self.world_min_x + (px - self.map_x_min) / float(self.map_x_max - self.map_x_min) * (self.world_max_x - self.world_min_x)
        wy = self.world_max_y - (py - self.map_y_min) / float(self.map_y_max - self.map_y_min) * (self.world_max_y - self.world_min_y)
        return round(wx, 2), round(wy, 2)

    def publish_goal(self, gx: float, gy: float):
        self.goal_pt = (gx, gy)
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = "odom"
        ps.pose.position.x = float(gx)
        ps.pose.position.y = float(gy)
        self.pub_goal.publish(ps)
        self.set_status(f"New Destination Set: ({gx:.2f}, {gy:.2f})")

    def publish_reset(self, rx: float, ry: float, ryaw: float):
        self.start_pt = (rx, ry)
        # Send physical reset to Isaac Sim (-999.0)
        try:
            pkt = struct.pack("ffff", -999.0, float(rx), float(ry), float(ryaw))
            self.sim_sock.sendto(pkt, self.sim_addr)
        except Exception:
            pass

        # Publish to ROS
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = "odom"
        ps.pose.position.x = float(rx)
        ps.pose.position.y = float(ry)
        ps.pose.orientation = yaw_to_quaternion(ryaw)
        self.pub_reset.publish(ps)
        self.pub_start.publish(ps)

        # Clear trails
        self.actual_trail.clear()
        self.est_trail.clear()
        self.set_status(f"Vehicle Repositioned to Start: ({rx:.2f}, {ry:.2f})")

    def swap_start_and_goal(self):
        new_s, new_g = self.goal_pt, self.start_pt
        yaw = math.atan2(new_g[1] - new_s[1], new_g[0] - new_s[0])
        self.publish_reset(new_s[0], new_s[1], yaw)
        self.publish_goal(new_g[0], new_g[1])
        self.set_status(f"Swapped Start: ({new_s[0]:.2f}, {new_s[1]:.2f}) <-> Goal: ({new_g[0]:.2f}, {new_g[1]:.2f})")

    def clear_points(self):
        scen_info = SCENARIOS[self.current_scen_idx]
        s_def = scen_info.get("default_start", (6.0, -6.0, 0.35, 0.0))
        g_def = scen_info.get("default_goal", (13.5, -6.0, 0.0))
        self.publish_reset(s_def[0], s_def[1], s_def[3])
        self.publish_goal(g_def[0], g_def[1])
        self.set_status(f"Points cleared to defaults: Start=({s_def[0]:.1f}, {s_def[1]:.1f}), Goal=({g_def[0]:.1f}, {g_def[1]:.1f})")

    def reset_vehicle(self):
        yaw = math.atan2(self.goal_pt[1] - self.start_pt[1], self.goal_pt[0] - self.start_pt[0])
        self.publish_reset(self.start_pt[0], self.start_pt[1], yaw)
        self.set_status(f"Vehicle reset to Start ({self.start_pt[0]:.2f}, {self.start_pt[1]:.2f})")

    def plan_route(self):
        self.publish_goal(self.goal_pt[0], self.goal_pt[1])
        self.set_status(f"Route planned: ({self.start_pt[0]:.2f}, {self.start_pt[1]:.2f}) -> ({self.goal_pt[0]:.2f}, {self.goal_pt[1]:.2f})")

    def start_autonomous(self):
        self.is_paused = False
        msg = Bool()
        msg.data = False
        self.pub_pause.publish(msg)
        self.set_status("START AUTONOMOUS: Vehicle drive engaged!")

    def toggle_pause(self):
        self.is_paused = not self.is_paused
        msg = Bool()
        msg.data = self.is_paused
        self.pub_pause.publish(msg)
        self.set_status("VEHICLE DRIVE PAUSED" if self.is_paused else "VEHICLE DRIVE RESUMED")

    def emergency_stop(self):
        self.is_paused = True
        msg = Bool()
        msg.data = True
        self.pub_pause.publish(msg)
        self.set_status("EMERGENCY STOP ACTIVATED (Zero Velocity Command)")

    def reset_scenario(self):
        self.load_scenario(self.current_scen_idx, notify_sim=True)
        self.set_status(f"Scenario Reset: {SCENARIOS[self.current_scen_idx]['name']}")

    def on_mouse_event(self, event, x, y, flags, param):
        wx, wy = self.pixel_to_world(x, y)
        self.mouse_world_pos = (wx, wy)

        # Check button clicks (Left Click)
        if event == cv2.EVENT_LBUTTONDOWN:
            # 1. Header Toolbar Buttons (Scenario Selector)
            if 15 <= y <= 51:
                # [ < Prev ]
                if 20 <= x <= 100:
                    self.load_scenario(self.current_scen_idx - 1, notify_sim=True)
                    return
                # Scenario Dropdown box
                elif 110 <= x <= 560:
                    self.load_scenario(self.current_scen_idx + 1, notify_sim=True)
                    return
                # [ Next > ]
                elif 570 <= x <= 650:
                    self.load_scenario(self.current_scen_idx + 1, notify_sim=True)
                    return

            # 2. Bottom Toolbar Buttons (y in [800, 850])
            if 800 <= y <= 850:
                bw = 98
                gap = 8
                for idx in range(10):
                    bx1 = 20 + idx * (bw + gap)
                    bx2 = bx1 + bw
                    if bx1 <= x <= bx2:
                        if idx == 0:  # Set Start
                            self.interact_mode = "NORMAL" if self.interact_mode == "SET_START" else "SET_START"
                            self.set_status("MODE: CLICK MAP TO SET START" if self.interact_mode == "SET_START" else "MODE: NORMAL")
                        elif idx == 1:  # Set Goal
                            self.interact_mode = "NORMAL" if self.interact_mode == "SET_GOAL" else "SET_GOAL"
                            self.set_status("MODE: CLICK MAP TO SET GOAL" if self.interact_mode == "SET_GOAL" else "MODE: NORMAL")
                        elif idx == 2:  # Swap S/G
                            self.swap_start_and_goal()
                        elif idx == 3:  # Clear Pts
                            self.clear_points()
                        elif idx == 4:  # Reset Veh
                            self.reset_vehicle()
                        elif idx == 5:  # Plan Route
                            self.plan_route()
                        elif idx == 6:  # Start Auto
                            self.start_autonomous()
                        elif idx == 7:  # Pause / Resume
                            self.toggle_pause()
                        elif idx == 8:  # E-Stop
                            self.emergency_stop()
                        elif idx == 9:  # Reset Scen
                            self.reset_scenario()
                        return

            # 3. Map Viewport Clicks
            if self.map_x_min <= x <= self.map_x_max and self.map_y_min <= y <= self.map_y_max:
                if self.interact_mode == "SET_START":
                    if is_point_walkable(wx, wy):
                        yaw = math.atan2(self.goal_pt[1] - wy, self.goal_pt[0] - wx)
                        self.publish_reset(wx, wy, yaw)
                        self.interact_mode = "NORMAL"
                        self.set_status(f"Start Point Confirmed: ({wx:.2f}, {wy:.2f})")
                    else:
                        self.set_status(f"BLOCKED: Point ({wx:.2f}, {wy:.2f}) is inside wall/obstacle!")
                elif self.interact_mode == "SET_GOAL":
                    if is_point_walkable(wx, wy):
                        self.publish_goal(wx, wy)
                        self.interact_mode = "NORMAL"
                        self.set_status(f"Destination Confirmed: ({wx:.2f}, {wy:.2f})")
                    else:
                        self.set_status(f"BLOCKED: Point ({wx:.2f}, {wy:.2f}) is inside wall/obstacle!")
                else:
                    # Normal Left Click sets Destination
                    if is_point_walkable(wx, wy):
                        self.publish_goal(wx, wy)
                    else:
                        self.set_status(f"BLOCKED: Point ({wx:.2f}, {wy:.2f}) is inside wall/obstacle!")

        elif event == cv2.EVENT_RBUTTONDOWN:
            # Right Click directly sets Start Pose
            if self.map_x_min <= x <= self.map_x_max and self.map_y_min <= y <= self.map_y_max:
                if is_point_walkable(wx, wy):
                    yaw = math.atan2(self.goal_pt[1] - wy, self.goal_pt[0] - wx)
                    self.publish_reset(wx, wy, yaw)
                    self.set_status(f"Start Point Set (Right Click): ({wx:.2f}, {wy:.2f})")
                else:
                    self.set_status(f"BLOCKED: Point ({wx:.2f}, {wy:.2f}) is inside wall/obstacle!")
                    self.set_status(f"Point ({wx}, {wy}) inside wall/obstacle! Cannot place start.")

    def on_global_path(self, msg: RosPath):
        self.global_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def on_local_path(self, msg: RosPath):
        self.local_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def on_est_odom(self, msg: Odometry):
        self.veh_x = float(msg.pose.pose.position.x)
        self.veh_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.veh_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.est_trail.append((self.veh_x, self.veh_y))

    def on_cmd(self, msg: AckermannDriveStamped):
        self.cmd_speed = float(msg.drive.speed)
        self.cmd_steer = float(msg.drive.steering_angle)

    def on_tf(self, msg: TFMessage):
        for t in msg.transforms:
            if "vehicle" in t.child_frame_id.lower() or "chassis" in t.child_frame_id.lower():
                tx = float(t.transform.translation.x)
                ty = float(t.transform.translation.y)
                q = t.transform.rotation
                siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                self.actual_yaw = math.atan2(siny_cosp, cosy_cosp)
                self.actual_x = tx
                self.actual_y = ty
                self.actual_trail.append((tx, ty))
                break

    def on_obstacles(self, msg: Detection3DArray):
        obs_list = []
        pred_list = []
        cos_y = math.cos(self.veh_yaw)
        sin_y = math.sin(self.veh_yaw)

        for d in msg.detections:
            try:
                ox = float(d.bbox.center.position.x)
                oy = float(d.bbox.center.position.y)
                oz = float(d.bbox.center.position.z)

                cid_str = d.results[0].hypothesis.class_id if d.results else ""
                parts = cid_str.split(":")
                tid = d.id if (hasattr(d, "id") and d.id) else (parts[0] if len(parts) > 0 else "?")
                cname = parts[2] if len(parts) > 2 else (parts[1] if len(parts) > 1 else "obstacle")
                is_dyn = ("dyn" in cid_str.lower())

                # Dedicated velocity hypothesis
                spd = 0.0
                if len(d.results) > 1 and d.results[1].hypothesis.class_id == "velocity_m_s":
                    spd = float(d.results[1].hypothesis.score)

                # Camera optical -> Vehicle frame
                cos_p = 0.98877
                sin_p = -0.14944
                x_veh = 0.90 + cos_p * oz - sin_p * oy
                y_veh = -ox
                x_w = self.veh_x + x_veh * cos_y - y_veh * sin_y
                y_w = self.veh_y + x_veh * sin_y + y_veh * cos_y

                obs_list.append({
                    "id": tid, "cname": cname, "is_dyn": is_dyn,
                    "x_w": x_w, "y_w": y_w, "dist": math.hypot(x_veh, y_veh),
                    "speed": spd,
                })

                if is_dyn and spd > 0.15:
                    preds = []
                    for h in [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]:
                        fut_oz = oz - spd * h
                        fut_x_veh = fut_oz + 0.9
                        fut_y_veh = -ox
                        fut_xw = self.veh_x + fut_x_veh * cos_y - fut_y_veh * sin_y
                        fut_yw = self.veh_y + fut_x_veh * sin_y + fut_y_veh * cos_y
                        preds.append((fut_xw, fut_yw))
                    pred_list.append(preds)
            except Exception:
                continue

        self.obstacles = obs_list
        self.predicted_trajectories = pred_list

    def render_frame(self):
        # Update ground-truth pose from simulator socket
        if self.gt_sock:
            try:
                while True:
                    data, _ = self.gt_sock.recvfrom(32)
                    if len(data) == 12:
                        gx, gy, gyaw = struct.unpack("fff", data)
                        self.actual_x = float(gx)
                        self.actual_y = float(gy)
                        self.actual_yaw = float(gyaw)
                        self.actual_trail.append((self.actual_x, self.actual_y))
            except Exception:
                pass
        if not self.actual_trail:
            self.actual_x = float(self.start_pt[0])
            self.actual_y = float(self.start_pt[1])
            self.actual_yaw = 0.0

        canvas = np.zeros((self.img_h, self.img_w, 3), dtype=np.uint8)
        canvas[:] = (20, 24, 30)

        # 1. Render Top Header Toolbar
        self._render_header(canvas)

        # 2. Draw Lab perimeter walls and Glass Wall
        p_top_l = self.world_to_pixel(LAB_X_MIN, LAB_Y_MAX)
        p_bot_r = self.world_to_pixel(LAB_X_MAX, LAB_Y_MIN)
        cv2.rectangle(canvas, p_top_l, p_bot_r, (75, 80, 95), 2)
        # Highlight glass wall boundary at X=15.0
        p_glass_t = self.world_to_pixel(15.0, LAB_Y_MAX)
        p_glass_b = self.world_to_pixel(15.0, LAB_Y_MIN)
        cv2.line(canvas, p_glass_t, p_glass_b, (220, 200, 120), 2)
        cv2.putText(canvas, "GLASS WALL (COLLISION PROTECTED)", (p_glass_t[0] - 220, p_glass_t[1] + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (220, 200, 120), 1, cv2.LINE_AA)

        # 3. Draw static background Lab obstacles (desks & columns)
        for ox, oy, w, h in OBSTACLES:
            tl = self.world_to_pixel(ox - w / 2, oy + h / 2)
            br = self.world_to_pixel(ox + w / 2, oy - h / 2)
            cv2.rectangle(canvas, tl, br, (38, 42, 50), -1)
            cv2.rectangle(canvas, tl, br, (65, 70, 80), 1)

        # 4. Render Scenario Ground-Truth Static Obstacles
        self._render_scenario_statics(canvas)

        # 5. Render Scenario Pedestrian Trajectories (Ground Truth)
        self._render_scenario_pedestrians(canvas)

        # 6. Draw Start and Goal Points
        p_start = self.world_to_pixel(self.start_pt[0], self.start_pt[1])
        cv2.circle(canvas, p_start, 10, (220, 200, 50), -1)
        cv2.circle(canvas, p_start, 13, (255, 235, 100), 1)
        cv2.putText(canvas, f"START ({self.start_pt[0]:.1f}, {self.start_pt[1]:.1f})",
                    (p_start[0] - 25, p_start[1] - 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (220, 200, 50), 1, cv2.LINE_AA)

        p_goal = self.world_to_pixel(self.goal_pt[0], self.goal_pt[1])
        cv2.circle(canvas, p_goal, 12, (50, 220, 60), 2)
        cv2.circle(canvas, p_goal, 6, (50, 220, 60), -1)
        cv2.putText(canvas, f"GOAL ({self.goal_pt[0]:.1f}, {self.goal_pt[1]:.1f})",
                    (p_goal[0] - 35, p_goal[1] - 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (50, 220, 60), 1, cv2.LINE_AA)

        # 7. Draw Global Reference Path (BLUE)
        if len(self.global_path) > 1:
            pts = np.array([self.world_to_pixel(x, y) for x, y in self.global_path], dtype=np.int32)
            cv2.polylines(canvas, [pts], isClosed=False, color=(230, 140, 40), thickness=2, lineType=cv2.LINE_AA)

        # 8. Draw Active Local Planned Path (GREEN)
        if len(self.local_path) > 1:
            pts = np.array([self.world_to_pixel(x, y) for x, y in self.local_path], dtype=np.int32)
            cv2.polylines(canvas, [pts], isClosed=False, color=(60, 230, 80), thickness=3, lineType=cv2.LINE_AA)

        # 9. Draw Actual Vehicle Trajectory (ORANGE) & Odometry Trail (CYAN)
        if len(self.actual_trail) > 1:
            pts = np.array([self.world_to_pixel(x, y) for x, y in self.actual_trail], dtype=np.int32)
            cv2.polylines(canvas, [pts], isClosed=False, color=(40, 140, 250), thickness=2, lineType=cv2.LINE_AA)

        # 10. Draw Detected Obstacles & Safety Margins (RED + YELLOW)
        for obs in self.obstacles:
            op = self.world_to_pixel(obs["x_w"], obs["y_w"])
            rad_px = int(0.65 / (self.world_max_x - self.world_min_x) * (self.map_x_max - self.map_x_min))
            cv2.circle(canvas, op, rad_px, (40, 210, 240), 1)  # Yellow 0.65m safety margin
            cv2.circle(canvas, op, 6, (40, 40, 240), -1)       # Red obstacle center
            lbl = f"{obs['cname']}:{obs['id']}"
            cv2.putText(canvas, lbl, (op[0] + 8, op[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 240), 1)

        # 11. Draw Predicted Dynamic Trajectories (CYAN/PURPLE)
        for traj in self.predicted_trajectories:
            if len(traj) > 1:
                t_pts = np.array([self.world_to_pixel(x, y) for x, y in traj], dtype=np.int32)
                cv2.polylines(canvas, [t_pts], isClosed=False, color=(240, 180, 40), thickness=2, lineType=cv2.LINE_AA)
                for pt in t_pts:
                    cv2.circle(canvas, tuple(pt), 3, (240, 100, 200), -1)

        # 12. Draw Vehicle Platform with Knuckle Steer Angles
        self._render_vehicle(canvas)

        # 13. HUD Information Panel
        self._render_hud(canvas)

        # 14. Render Bottom Toolbar Controls
        self._render_toolbar(canvas)

        # Show in OpenCV window & handle keyboard input
        if not self.no_show:
            cv2.imshow(self.win_name, canvas)
            key = cv2.waitKey(1) & 0xFF
            self._handle_key(key)

        # Categorized Evidence Logging
        now = time.time()
        if now - self.last_save_time >= self.save_every:
            self.last_save_time = now
            self._save_categorized_evidence(canvas)
            self.frame_idx += 1

    def _render_header(self, canvas):
        cv2.rectangle(canvas, (0, 0), (self.img_w, 65), (12, 15, 20), -1)
        cv2.line(canvas, (0, 65), (self.img_w, 65), (55, 60, 75), 1)

        # [< Prev] button
        cv2.rectangle(canvas, (20, 15), (100, 51), (35, 40, 50), -1)
        cv2.rectangle(canvas, (20, 15), (100, 51), (70, 75, 90), 1)
        cv2.putText(canvas, "< Prev", (29, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1, cv2.LINE_AA)

        # Scenario Title Box
        scen_info = SCENARIOS[self.current_scen_idx]
        cv2.rectangle(canvas, (110, 15), (560, 51), (25, 30, 40), -1)
        cv2.rectangle(canvas, (110, 15), (560, 51), (80, 120, 180), 1)
        title_txt = f"{scen_info['name']} ({self.current_scen_idx + 1}/13)"
        cv2.putText(canvas, title_txt, (120, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2, cv2.LINE_AA)

        # [Next >] button
        cv2.rectangle(canvas, (570, 15), (650, 51), (35, 40, 50), -1)
        cv2.rectangle(canvas, (570, 15), (650, 51), (70, 75, 90), 1)
        cv2.putText(canvas, "Next >", (579, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1, cv2.LINE_AA)

        # Interactive Mode Indicator Box
        mode_bg = (35, 40, 50)
        mode_col = (200, 200, 200)
        mode_str = f"MODE: {self.interact_mode} | Click: L=Goal, R=Start"
        if self.interact_mode == "SET_START":
            mode_bg = (20, 60, 40)
            mode_col = (100, 255, 120)
            mode_str = "MODE: SET START -> Click anywhere walkable on map"
        elif self.interact_mode == "SET_GOAL":
            mode_bg = (50, 45, 15)
            mode_col = (255, 220, 100)
            mode_str = "MODE: SET GOAL -> Click anywhere walkable on map"

        cv2.rectangle(canvas, (665, 15), (1080, 51), mode_bg, -1)
        cv2.rectangle(canvas, (665, 15), (1080, 51), (70, 75, 90), 1)
        cv2.putText(canvas, mode_str, (675, 37), cv2.FONT_HERSHEY_SIMPLEX, 0.42, mode_col, 1, cv2.LINE_AA)

    def _render_scenario_statics(self, canvas):
        for obj in self.static_obstacles:
            otype = str(obj.get("object_type", "")).lower()
            oid = str(obj.get("object_id", otype))
            pos = obj.get("position", [0.0, 0.0, 0.0])
            dims = obj.get("dimensions", [1.4, 0.8, 0.75])
            ox, oy = pos[0], pos[1]
            w = dims[0] if len(dims) >= 1 else 1.4
            l = dims[1] if len(dims) >= 2 else 0.8

            tl = self.world_to_pixel(ox - w / 2.0, oy + l / 2.0)
            br = self.world_to_pixel(ox + w / 2.0, oy - l / 2.0)

            if otype == "chair":
                # Slate/brown chair
                cv2.rectangle(canvas, tl, br, (70, 95, 130), -1)
                cv2.rectangle(canvas, tl, br, (110, 140, 180), 1)
                cv2.putText(canvas, f"chair:{oid[-2:]}", (tl[0] - 5, tl[1] - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (160, 180, 210), 1)
            elif otype in ("box", "crate", "unknown", "cube"):
                # Bright orange unclassified crate
                cv2.rectangle(canvas, tl, br, (30, 120, 240), -1)
                cv2.rectangle(canvas, tl, br, (40, 180, 255), 2)
                cv2.line(canvas, tl, br, (20, 80, 180), 1)
                cv2.putText(canvas, "CRATE (unclassified)", (tl[0] - 15, tl[1] - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (40, 220, 255), 1, cv2.LINE_AA)
            elif "barrier" in oid:
                # Red/hazard blockade
                cv2.rectangle(canvas, tl, br, (35, 35, 180), -1)
                cv2.rectangle(canvas, tl, br, (60, 60, 255), 1)
                cv2.putText(canvas, "BARRIER", (tl[0] + 5, (tl[1] + br[1]) // 2 + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (220, 220, 255), 1)
            else:
                # Office/Lab Table
                cv2.rectangle(canvas, tl, br, (65, 80, 110), -1)
                cv2.rectangle(canvas, tl, br, (120, 140, 175), 1)
                cv2.putText(canvas, f"table:{oid[-2:]}", (tl[0] + 4, tl[1] - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 200, 230), 1)

    def _render_scenario_pedestrians(self, canvas):
        for ped in self.scenario_pedestrians:
            pid = ped.get("pedestrian_id", "ped")
            ipos = ped.get("initial_position", [0.0, 0.0, 0.0])
            dest = ped.get("destination", [0.0, 0.0, 0.0])
            vel = float(ped.get("velocity", 0.45))
            delay = float(ped.get("start_delay", 0.0))

            p_init = self.world_to_pixel(ipos[0], ipos[1])
            p_dest = self.world_to_pixel(dest[0], dest[1])

            # Trajectory arrow
            cv2.arrowedLine(canvas, p_init, p_dest, (210, 80, 210), 2, tipLength=0.20)

            # Spawn circle (person)
            cv2.circle(canvas, p_init, 8, (210, 60, 210), -1)
            cv2.circle(canvas, p_init, 11, (240, 120, 240), 1)
            lbl = f"PED ({vel:.2f}m/s dly={delay:.1f}s)"
            cv2.putText(canvas, lbl, (p_init[0] - 40, p_init[1] - 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (230, 140, 230), 1, cv2.LINE_AA)

            # Destination target
            cv2.circle(canvas, p_dest, 6, (180, 40, 180), 1)
            cv2.drawMarker(canvas, p_dest, (180, 40, 180), cv2.MARKER_TILTED_CROSS, 8, 1)

    def _render_vehicle(self, canvas):
        vp = self.world_to_pixel(self.veh_x, self.veh_y)
        veh_len_px = int(1.1 / (self.world_max_x - self.world_min_x) * (self.map_x_max - self.map_x_min))
        veh_wid_px = int(0.6 / (self.world_max_y - self.world_min_y) * (self.map_y_max - self.map_y_min))

        # Vehicle bounding box rotated by yaw
        cos_y = math.cos(self.veh_yaw)
        sin_y = math.sin(self.veh_yaw)
        hl = veh_len_px / 2.0
        hw = veh_wid_px / 2.0

        corners = [
            (-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw)
        ]
        poly_pts = []
        for cx, cy in corners:
            rx = vp[0] + cx * cos_y + cy * sin_y
            ry = vp[1] - cx * sin_y + cy * cos_y
            poly_pts.append((int(rx), int(ry)))
        cv2.polylines(canvas, [np.array(poly_pts, dtype=np.int32)], isClosed=True, color=(180, 220, 255), thickness=2)
        cv2.fillPoly(canvas, [np.array(poly_pts, dtype=np.int32)], color=(40, 60, 80))

        # Heading vector arrow
        arrow_len = int(veh_len_px * 1.5)
        end_x = int(vp[0] + arrow_len * cos_y)
        end_y = int(vp[1] - arrow_len * sin_y)
        cv2.arrowedLine(canvas, vp, (end_x, end_y), (0, 255, 255), 2, tipLength=0.25)
        cv2.circle(canvas, vp, 4, (255, 255, 255), -1)

    def _render_hud(self, canvas):
        hud_w, hud_h = 410, 270
        overlay = canvas.copy()
        cv2.rectangle(overlay, (20, 75), (20 + hud_w, 75 + hud_h), (10, 12, 16), -1)
        cv2.addWeighted(overlay, 0.88, canvas, 0.12, 0, canvas)
        cv2.rectangle(canvas, (20, 75), (20 + hud_w, 75 + hud_h), (80, 85, 100), 1)

        cv2.putText(canvas, "AUTONOMOUS NAVIGATION HUD", (32, 98),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2, cv2.LINE_AA)

        dist_to_dest = math.hypot(self.goal_pt[0] - self.veh_x, self.goal_pt[1] - self.veh_y)
        pos_err = math.hypot(self.veh_x - self.actual_x, self.veh_y - self.actual_y)

        nearest_str = "None"
        nearest_col = (180, 180, 180)
        if self.obstacles:
            n_obs = min(self.obstacles, key=lambda o: o["dist"])
            nearest_str = f"{n_obs['cname']} ({n_obs['dist']:.2f}m)"
            if n_obs["dist"] < 1.5:
                nearest_col = (50, 50, 255)
            elif n_obs["dist"] < 3.0:
                nearest_col = (0, 220, 255)

        odom_st = getattr(self, "odom_status_val", "NOMINAL")
        odom_conf = getattr(self, "vis_conf_val", 0.0)
        if odom_st == "NOMINAL":
            odom_col = (80, 220, 100)
        elif odom_st == "VISUAL_DEGRADED":
            odom_col = (0, 140, 255)
        elif odom_st in ("WHEEL_SLIP", "CAM_INTERRUPTED", "NO_ENCODER"):
            odom_col = (0, 0, 255)
        else:
            odom_col = (180, 180, 180)

        state_str = "CRUISE"
        state_color = (60, 220, 60)
        if "WHEEL_SLIP" in self.planner_state or getattr(self, "wheel_slip_active", False) or odom_st == "WHEEL_SLIP":
            state_str = "WHEEL SLIP / PHYSICAL BLOCKAGE (SAFE STOP)"
            state_color = (0, 0, 255)
        elif "CAM_INTERRUPTED" in odom_st or "CAM_INTERRUPTED" in self.planner_state:
            state_str = "CAMERA INTERRUPTED (HOLD)"
            state_color = (0, 0, 255)
        elif "VISUAL_DEGRADED" in self.planner_state or odom_st == "VISUAL_DEGRADED":
            state_str = "VISUAL ODOM DEGRADED (LOW TEXTURE SAFE STOP)"
            state_color = (0, 140, 255)
        elif "STUCK" in self.planner_state:
            state_str = "STUCK (PROGRESS BLOCKED)"
            state_color = (0, 0, 255)
        elif dist_to_dest < 0.45 and abs(self.cmd_speed) < 0.05:
            state_str = "GOAL REACHED"
            state_color = (50, 220, 100)
        elif self.is_paused:
            state_str = "PAUSED / HOLD"
            state_color = (140, 140, 255)
        elif "NO_SAFE_PATH" in self.planner_state or (self.cmd_speed == 0.0 and self.obstacles and min(o["dist"] for o in self.obstacles) < 1.5):
            state_str = "STOPPED (SAFE HOLD)"
            state_color = (40, 40, 240)
        elif abs(self.cmd_steer) > 0.20:
            state_str = "REPLAN / DETOUR"
            state_color = (40, 220, 240)
        elif self.cmd_speed < 0.5:
            state_str = "SLOW / YIELD"
            state_color = (0, 180, 240)

        lines = [
            (f"Scenario: {SCENARIOS[self.current_scen_idx]['id'].upper()} - {SCENARIOS[self.current_scen_idx]['name'].split(': ')[-1]}", (220, 220, 220)),
            (f"State: {state_str}", state_color),
            (f"Cmd Speed: {self.cmd_speed:.2f} m/s", (220, 220, 220)),
            (f"Cmd Steer: {math.degrees(self.cmd_steer):+.1f} deg", (220, 220, 220)),
            (f"Dist to Goal: {dist_to_dest:.2f} m", (220, 220, 220)),
            (f"Estimated Pose: ({self.veh_x:.2f}, {self.veh_y:.2f})", (200, 200, 200)),
            (f"Ground Truth: ({self.actual_x:.2f}, {self.actual_y:.2f})", (180, 180, 180)),
            (f"Odometry Error: {pos_err:.2f} m", (80, 220, 100) if pos_err < 0.15 else (0, 180, 255)),
            (f"Odom Status: {odom_st} (Conf: {odom_conf:.2f})", odom_col),
            (f"Nearest Obs: {nearest_str}", nearest_col),
            (f"Mouse World Pos: ({self.mouse_world_pos[0]:.2f}, {self.mouse_world_pos[1]:.2f})", (150, 190, 220)),
        ]

        y_offset = 120
        for text, color in lines:
            cv2.putText(canvas, text, (32, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1, cv2.LINE_AA)
            y_offset += 19

        # Prominent warning indicator badge when odometry is degraded or wheel slip occurs
        if odom_st in ("VISUAL_DEGRADED", "WHEEL_SLIP", "CAM_INTERRUPTED", "NO_ENCODER"):
            if odom_st == "WHEEL_SLIP":
                badge_text = "WARNING: WHEEL SLIP DETECTED (PHYSICAL BLOCKAGE)"
                badge_col = (0, 0, 255)
            elif odom_st == "VISUAL_DEGRADED":
                badge_text = "WARNING: VISUAL ODOM DEGRADED (LOW TEXTURE / UNVERIFIED)"
                badge_col = (0, 140, 255)
            elif odom_st == "CAM_INTERRUPTED":
                badge_text = "WARNING: CAMERA STREAM INTERRUPTED (MOTION HALTED)"
                badge_col = (0, 0, 255)
            else:
                badge_text = "WARNING: NO ENCODER FEEDBACK (MOTION HALTED)"
                badge_col = (0, 100, 255)

            bw, bh = 560, 32
            bx = (self.img_w - bw) // 2
            by = 22
            cv2.rectangle(canvas, (bx, by), (bx + bw, by + bh), (15, 15, 25), -1)
            cv2.rectangle(canvas, (bx, by), (bx + bw, by + bh), badge_col, 2)
            cv2.putText(canvas, badge_text, (bx + 15, by + 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.44, badge_col, 2, cv2.LINE_AA)

        # Legend at bottom right of map
        leg_x = self.img_w - 245
        leg_y = self.map_y_max - 140
        cv2.rectangle(canvas, (leg_x - 10, leg_y - 20), (self.map_x_max - 10, self.map_y_max - 10), (12, 15, 20), -1)
        cv2.rectangle(canvas, (leg_x - 10, leg_y - 20), (self.map_x_max - 10, self.map_y_max - 10), (70, 75, 85), 1)
        cv2.putText(canvas, "LEGEND", (leg_x, leg_y - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        legend_items = [
            ("Global Route (A*)", (230, 140, 40)),
            ("Planned Local Path", (60, 220, 60)),
            ("Actual Trajectory", (40, 140, 250)),
            ("Detected Obstacle", (40, 40, 240)),
            ("Dynamic Ped Prediction", (240, 200, 30)),
        ]
        ly = leg_y + 16
        for name, col in legend_items:
            cv2.line(canvas, (leg_x, ly - 4), (leg_x + 20, ly - 4), col, 2)
            cv2.putText(canvas, name, (leg_x + 26, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (200, 200, 200), 1)
            ly += 18

    def _render_toolbar(self, canvas):
        cv2.rectangle(canvas, (0, 775), (self.img_w, self.img_h), (12, 15, 20), -1)
        cv2.line(canvas, (0, 775), (self.img_w, 775), (55, 60, 75), 1)

        # Status text line
        st_col = (100, 240, 120) if time.time() - self.status_time < 3.0 else (180, 180, 180)
        cv2.putText(canvas, f"STATUS: {self.status_msg}", (22, 793),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, st_col, 1, cv2.LINE_AA)

        # Buttons definition: 10 buttons across 1060px (x=20 to 1080)
        btn_y1, btn_y2 = 802, 848
        btn_defs = [
            ("Set Start (S)", self.interact_mode == "SET_START", (25, 65, 35), (100, 255, 120)),
            ("Set Goal (G)",  self.interact_mode == "SET_GOAL",  (65, 55, 20), (255, 220, 100)),
            ("Swap S/G (W)",  False, (35, 45, 55), (200, 220, 255)),
            ("Clear Pts (C)", False, (40, 40, 45), (210, 210, 210)),
            ("Reset Veh (R)", False, (35, 45, 40), (220, 220, 220)),
            ("Plan Route (L)",False, (30, 45, 60), (140, 210, 255)),
            ("Start Auto (Ent)", not self.is_paused, (20, 60, 30), (120, 255, 140)),
            ("Resume" if self.is_paused else "Pause (Spc)", self.is_paused, (60, 30, 50), (255, 180, 220)),
            ("E-Stop (Esc)",  self.is_paused, (60, 20, 25), (255, 120, 120)),
            ("Reset Scen",    False, (50, 35, 25), (255, 190, 140)),
        ]

        bw = 98
        gap = 8
        for i, (lbl, active, act_bg, act_fg) in enumerate(btn_defs):
            bx1 = 20 + i * (bw + gap)
            bx2 = bx1 + bw
            bg = act_bg if active else (28, 32, 40)
            fg = act_fg if active else (210, 215, 225)
            cv2.rectangle(canvas, (bx1, btn_y1), (bx2, btn_y2), bg, -1)
            cv2.rectangle(canvas, (bx1, btn_y1), (bx2, btn_y2), (75, 80, 95) if not active else act_fg, 1)
            (tw, th), _ = cv2.getTextSize(lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.35, 1)
            tx = bx1 + max(2, (bw - tw) // 2)
            ty = btn_y1 + (btn_y2 - btn_y1 + th) // 2
            cv2.putText(canvas, lbl, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.35, fg, 1, cv2.LINE_AA)

    def _handle_key(self, key: int):
        if key in (ord('s'), ord('S')):
            self.interact_mode = "NORMAL" if self.interact_mode == "SET_START" else "SET_START"
            self.set_status("MODE: CLICK MAP TO SET START" if self.interact_mode == "SET_START" else "MODE: NORMAL")
        elif key in (ord('g'), ord('G')):
            self.interact_mode = "NORMAL" if self.interact_mode == "SET_GOAL" else "SET_GOAL"
            self.set_status("MODE: CLICK MAP TO SET GOAL" if self.interact_mode == "SET_GOAL" else "MODE: NORMAL")
        elif key in (ord('w'), ord('W')):
            self.swap_start_and_goal()
        elif key in (ord('c'), ord('C')):
            self.clear_points()
        elif key in (ord('r'), ord('R')):
            self.reset_vehicle()
        elif key in (ord('l'), ord('L')):
            self.plan_route()
        elif key in (10, 13):  # Enter
            self.start_autonomous()
        elif key in (ord('p'), ord('P'), ord(' ')):
            self.toggle_pause()
        elif key == 27:  # ESC
            self.emergency_stop()
        elif key == 8:  # Backspace
            self.reset_scenario()
        elif key == ord('['):
            self.load_scenario(self.current_scen_idx - 1, notify_sim=True)
        elif key == ord(']'):
            self.load_scenario(self.current_scen_idx + 1, notify_sim=True)

    def _save_categorized_evidence(self, canvas):
        dist_to_goal = math.hypot(self.goal_pt[0] - self.veh_x, self.goal_pt[1] - self.veh_y)
        goal_reached = ("GOAL_REACHED" in self.planner_state) or (dist_to_goal < 0.65 and abs(self.cmd_speed) < 0.05)
        is_blocked = ("NO_SAFE_PATH" in self.planner_state or (self.cmd_speed == 0.0 and len(self.obstacles) > 0 and min(o["dist"] for o in self.obstacles) < 1.5))
        is_detour = abs(self.cmd_steer) > 0.20 or ("REPLAN" in self.planner_state)

        category = "progress"
        if goal_reached:
            category = "goal"
        elif is_blocked:
            category = "blocked"
        elif is_detour:
            category = "planned"
        elif len(self.obstacles) > 0:
            category = "obstacle"

        cat_path = self.cat_dirs.get(category, self.cat_dirs["progress"])
        img_name = f"snapshot_{self.scenario_id}_{self.frame_idx:04d}_{category}.png"
        meta_name = f"snapshot_{self.scenario_id}_{self.frame_idx:04d}_{category}.json"

        cv2.imwrite(str(cat_path / img_name), canvas)

        metadata = {
            "timestamp": time.time(),
            "frame_idx": self.frame_idx,
            "scenario_id": self.scenario_id,
            "vehicle_pose_estimated": [self.veh_x, self.veh_y, self.veh_yaw],
            "vehicle_pose_ground_truth": [self.actual_x, self.actual_y, self.actual_yaw],
            "odometry_error_m": math.hypot(self.veh_x - self.actual_x, self.veh_y - self.actual_y),
            "goal_pose": [self.goal_pt[0], self.goal_pt[1]],
            "distance_to_goal": dist_to_goal,
            "planner_state": self.planner_state,
            "commanded_speed": self.cmd_speed,
            "commanded_steering": self.cmd_steer,
            "num_obstacles_detected": len(self.obstacles),
            "goal_reached": goal_reached,
            "is_blocked": is_blocked,
            "is_detouring": is_detour,
            "category": category,
        }
        try:
            with open(cat_path / meta_name, "w") as f:
                json.dump(metadata, f, indent=2)
        except Exception:
            pass


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--scenario", default=None)
    p.add_argument("--start", nargs="+", type=float, default=None, help="Allotted start pose [x, y, (yaw)]")
    p.add_argument("--goal", nargs="+", type=float, default=None, help="Allotted destination pose [x, y]")
    p.add_argument("--out-dir", default="results")
    p.add_argument("--save-every", type=float, default=2.0)
    p.add_argument("--no-show", action="store_true", default=False)
    args = p.parse_args()

    rclpy.init()
    node = TopDownVisualizerNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    try:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
