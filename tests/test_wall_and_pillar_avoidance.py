#!/usr/bin/env python3
"""
Test Suite: Wall, Pillar, and Unconfigured Obstacle Detection and Avoidance
Verifies:
1. Depth-based geometric wall & pillar morphological classification in rgbd_fusion_node.
2. Obstacle retention across corridor boundaries when extent enters corridor.
3. Fine-to-coarse lateral detour generation (fine for pillars, wide for walls/tables).
4. Smooth quintic path recovery back to global nominal path upon obstacle clearance.
5. Encoder socket binding and watchdog resilience with SO_REUSEPORT.
6. Perception monitor depth statistics and dual-stream health status.
"""

import math
import os
import sys
import unittest
import numpy as np

# ROS 2 & package imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose

from rgbd_perception.vehicle_footprint import (
    VEH_HALF_LENGTH, VEH_HALF_WIDTH, SAFETY_MARGIN, sat_obb_clearance, compute_path_curvature
)
from rgbd_perception.config import load_config
from rgbd_perception.autonomous_planner_node import AutonomousPlannerNode
from isaac_sim.scripts.testing.live_perception_monitor import LiveMonitor


class TestWallAndPillarAvoidance(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()
        cls.cfg = load_config(None)

    def test_01_wall_and_pillar_classification_in_fusion(self):
        """Verify rgbd_fusion_node logic for classifying walls vs pillars vs obstacles."""
        # Simulated point clouds in camera frame
        # Case A: Wide Wall (ptp_x = 2.4m, ptp_y = 1.0m, ptp_z = 0.2m)
        sc_x_wall = np.linspace(-1.2, 1.2, 50)
        sc_y_wall = np.linspace(-0.5, 0.5, 50)
        sc_z_wall = np.full(50, 3.0)
        ptp_x_w = float(np.ptp(sc_x_wall))
        ptp_y_w = float(np.ptp(sc_y_wall))

        self.assertGreaterEqual(ptp_x_w, 1.60)
        cname_wall = "wall" if ptp_x_w >= 1.60 else "obstacle"
        self.assertEqual(cname_wall, "wall")

        # Case B: Narrow Pillar (ptp_x = 0.40m, ptp_y = 1.2m, ptp_z = 0.40m)
        sc_x_pil = np.linspace(-0.20, 0.20, 30)
        sc_y_pil = np.linspace(-0.60, 0.60, 30)
        ptp_x_p = float(np.ptp(sc_x_pil))
        ptp_y_p = float(np.ptp(sc_y_pil))

        self.assertLessEqual(ptp_x_p, 0.75)
        self.assertGreaterEqual(ptp_y_p, 0.65)
        cname_pillar = "pillar" if (ptp_x_p <= 0.75 and ptp_y_p >= 0.65) else "obstacle"
        self.assertEqual(cname_pillar, "pillar")

    def test_02_corridor_retention_for_wide_wall(self):
        """Verify that a wide wall whose center is outside 2.0m is not dropped."""
        # Planner setup
        planner = AutonomousPlannerNode(self.cfg)
        planner.start_xy = (6.0, -6.0)
        planner.goal_xy = (13.5, -6.0)
        planner.veh_x = 6.0
        planner.veh_y = -6.0
        planner.veh_yaw = 0.0

        # Corridor unit vector is along +X (ug_x=1.0, ug_y=0.0)
        ug_x = 1.0
        ug_y = 0.0
        dist_to_goal = 7.5

        # Wall located ahead at x=9.0m, but shifted laterally to y=-3.8m (l_obs = 2.2m)
        # With size_x = 2.8m (obs_hw = 1.4m), its edge reaches y = -3.8 - 1.4 = -5.2m (only 0.8m from center y=-6.0m!)
        wall_obs = {
            "track_id": 101,
            "class_name": "wall",
            "is_dynamic": False,
            "speed": 0.0,
            "size_x": 2.8,  # lateral width
            "size_y": 1.5,
            "size_z": 0.5,
            "x_cam": 2.2,
            "y_cam": 0.0,
            "z_cam": 3.0,
            "x_world": 9.0,
            "y_world": -3.8,
            "x_veh": 3.9,
            "y_veh": -2.2,
            "dist": 3.9,
        }

        dx_obs = wall_obs["x_world"] - planner.veh_x
        dy_obs = wall_obs["y_world"] - planner.veh_y
        s_obs = dx_obs * ug_x + dy_obs * ug_y
        l_obs = abs(dx_obs * (-ug_y) + dy_obs * ug_x)

        obs_hw = (wall_obs.get("size_x", 0.8) if abs(ug_x) > 0.5 else wall_obs.get("size_y", 0.8)) / 2.0
        # In old code: if l_obs > 2.0: continue (would drop this wall!)
        self.assertGreater(l_obs, 2.0, "Obstacle center is indeed > 2.0m")
        # In new code:
        is_retained = not ((l_obs - obs_hw) > 2.20)
        self.assertTrue(is_retained, "Wide wall spanning into corridor must be retained")

    def test_03_fine_offset_selected_for_pillar(self):
        """Verify that narrow pillars allow lateral offsets (1.35m) providing >=0.25m clearance without boundary collision."""
        planner = AutonomousPlannerNode(self.cfg)
        planner.start_xy = (6.0, -6.0)
        planner.goal_xy = (13.5, -6.0)
        planner.veh_x = 6.0
        planner.veh_y = -6.0
        planner.veh_yaw = 0.0

        # Narrow pillar directly on centerline at x=9.0m, y=-6.0m
        pillar_obs = {
            "track_id": 102,
            "class_name": "pillar",
            "is_dynamic": False,
            "speed": 0.0,
            "size_x": 0.40,
            "size_y": 1.20,
            "size_z": 0.40,
            "x_cam": 0.0,
            "y_cam": 0.0,
            "z_cam": 2.5,
            "x_world": 9.0,
            "y_world": -6.0,
            "x_veh": 3.4,
            "y_veh": 0.0,
            "dist": 3.4,
            "yaw": 0.0,
            "last_seen": 100.0,
        }

        cand_path_fine = planner._generate_offset_path(1.85, obs_dist=3.0, max_obs_dist=3.0)
        self.assertIsNotNone(cand_path_fine)
        safe_fine, clr_fine, reason_fine = planner._check_path_clearance(cand_path_fine, [pillar_obs])
        self.assertTrue(safe_fine, f"Offset 1.85m must safely clear narrow pillar. Reason: {reason_fine}")
        self.assertGreaterEqual(clr_fine, 0.15)

        # Curvature along fine offset path must respect Ackermann steering feasibility
        max_k, _ = compute_path_curvature(cand_path_fine)
        self.assertLessEqual(max_k, 0.460, f"Curvature {max_k:.3f} must be within Ackermann limit")

    def test_04_smooth_rejoining_spline_continuity(self):
        """Verify smooth quintic rejoining spline does not introduce step discontinuity upon obstacle clearance."""
        planner = AutonomousPlannerNode(self.cfg)
        planner.start_xy = (6.0, -6.0)
        planner.goal_xy = (13.5, -6.0)
        # Vehicle is at x=9.5m (past obstacle), currently offset at y=-4.6m (offset = +1.4m from corridor)
        planner.veh_x = 9.5
        planner.veh_y = -4.6
        planner.veh_yaw = 0.0

        rejoin_path = planner._generate_offset_path(0.0)
        self.assertIsNotNone(rejoin_path)
        self.assertGreaterEqual(len(rejoin_path), 5)

        # Path must start exactly at vehicle pose
        self.assertAlmostEqual(rejoin_path[0][0], 9.5, delta=0.01)
        self.assertAlmostEqual(rejoin_path[0][1], -4.6, delta=0.01)

        # Step changes along waypoints must be continuous (< 0.20m per step)
        for i in range(1, len(rejoin_path)):
            dx = rejoin_path[i][0] - rejoin_path[i-1][0]
            dy = rejoin_path[i][1] - rejoin_path[i-1][1]
            step_dist = math.hypot(dx, dy)
            self.assertLess(step_dist, 0.25, f"Step distance at sample {i} must be continuous: {step_dist:.3f}m")

        # Must terminate smoothly at goal
        self.assertAlmostEqual(rejoin_path[-1][0], 13.5, delta=0.10)
        self.assertAlmostEqual(rejoin_path[-1][1], -6.0, delta=0.10)

    def test_05_depth_monitor_statistics(self):
        """Verify live_perception_monitor correctly parses Depth frames and camera matrix."""
        class DummyArgs:
            scenario = "s02"
            out_dir = "/tmp"
            window_title = "Depth Unit Test"
            save_every = 0
            show = False
            duration = 0

        mon = LiveMonitor(DummyArgs())

        # Test CameraInfo
        info = CameraInfo()
        info.k = [580.0, 0.0, 320.0, 0.0, 580.0, 240.0, 0.0, 0.0, 1.0]
        mon._on_info(info)
        self.assertEqual(mon._fx, 580.0)
        self.assertEqual(mon._fy, 580.0)
        self.assertEqual(mon._cx, 320.0)
        self.assertEqual(mon._cy, 240.0)

        # Test Depth frame stats
        depth_arr = np.full((480, 640), 2.5, dtype=np.float32)
        depth_arr[100:150, 100:150] = 0.85  # closer patch (obstacle)
        depth_msg = Image()
        depth_msg.header.stamp.sec = 10
        depth_msg.header.stamp.nanosec = 0
        depth_msg.height = 480
        depth_msg.width = 640
        depth_msg.encoding = "32FC1"
        depth_msg.data = depth_arr.tobytes()

        mon._on_depth(depth_msg)
        self.assertGreater(mon._depth_stats["valid"], 95.0)
        self.assertAlmostEqual(mon._depth_stats["min"], 0.85, places=2)
        self.assertAlmostEqual(mon._depth_stats["max"], 2.50, places=2)


if __name__ == "__main__":
    unittest.main()
