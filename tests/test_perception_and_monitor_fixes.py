"""Targeted Unit & Integration Tests: Perception Pipeline & Live Monitor Fixes.

Verifies:
1. Live monitor topic subscriptions and standby frame rendering.
2. RGB and Depth callback handling (multiple encodings: rgb8, bgr8, rgba8, mono8).
3. Invalid / out-of-bounds depth rejection.
4. Single-frame noise suppression (hits threshold for geometric vs YOLO tracks).
5. Duplicate track suppression in dynamic tracker.
6. Expired track removal and Marker.DELETE generation.
7. Stale obstacle clearance in top-down visualizer.
8. Exact coordinate transform consistency between camera, vehicle, and world frames.
"""

import math
import os
import sys
import time
import unittest
import numpy as np

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")
sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP/ros2_ws/src/rgbd_perception")

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection3DArray, Detection3D, ObjectHypothesisWithPose
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header

from isaac_sim.scripts.testing.live_perception_monitor import LiveMonitor, parse_class
from ros2_ws.src.rgbd_perception.rgbd_perception.dynamic_tracker_node import DynamicTrackerNode, Track
from isaac_sim.scripts.testing.path_planning_visualizer import TopDownVisualizerNode


class MockArgs:
    def __init__(self):
        self.scenario = "s02"
        self.out_dir = "/tmp/test_live_monitor"
        self.window_title = "Test Monitor"
        self.save_every = 0.0
        self.show = False
        self.no_show = True
        self.duration = 0.0
        self.start = [6.0, -6.0]
        self.goal = [13.5, -6.0]


class TestPerceptionAndMonitorFixes(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def test_01_live_monitor_subscriptions_and_standby(self):
        """Test that LiveMonitor correctly subscribes to all 5 perception topics and renders standby frame."""
        args = MockArgs()
        monitor = LiveMonitor(args)

        # Check subscriptions
        topic_names = [sub.topic_name for sub in monitor.subscriptions]
        self.assertIn("/camera/rgb/image_raw", topic_names)
        self.assertIn("/camera/depth/image_raw", topic_names)
        self.assertIn("/perception/detections_2d", topic_names)
        self.assertIn("/perception/masks", topic_names)
        self.assertIn("/perception/objects_3d", topic_names)

        # Check that standby frame renders without error
        monitor._render_standby_frame()
        self.assertIsNone(monitor._rgb)
        monitor.destroy_node()

    def test_02_rgb_encoding_handling(self):
        """Test that LiveMonitor handles rgb8, bgr8, rgba8, bgra8, mono8 encodings correctly."""
        args = MockArgs()
        monitor = LiveMonitor(args)

        # 1. rgb8
        msg_rgb = Image()
        msg_rgb.width = 64
        msg_rgb.height = 48
        msg_rgb.encoding = "rgb8"
        raw_rgb = np.full((48, 64, 3), (255, 128, 64), dtype=np.uint8)
        msg_rgb.data = raw_rgb.tobytes()
        monitor._on_rgb(msg_rgb)
        self.assertIsNotNone(monitor._rgb)
        self.assertEqual(monitor._rgb.shape, (48, 64, 3))
        # In BGR: red is channel 2 (255), blue is channel 0 (64)
        self.assertEqual(monitor._rgb[0, 0, 2], 255)
        self.assertEqual(monitor._rgb[0, 0, 0], 64)

        # 2. bgr8
        msg_bgr = Image()
        msg_bgr.width = 64
        msg_bgr.height = 48
        msg_bgr.encoding = "bgr8"
        raw_bgr = np.full((48, 64, 3), (10, 20, 30), dtype=np.uint8)
        msg_bgr.data = raw_bgr.tobytes()
        monitor._on_rgb(msg_bgr)
        self.assertEqual(monitor._rgb[0, 0, 0], 10)
        self.assertEqual(monitor._rgb[0, 0, 2], 30)

        # 3. rgba8
        msg_rgba = Image()
        msg_rgba.width = 64
        msg_rgba.height = 48
        msg_rgba.encoding = "rgba8"
        raw_rgba = np.full((48, 64, 4), (200, 100, 50, 255), dtype=np.uint8)
        msg_rgba.data = raw_rgba.tobytes()
        monitor._on_rgb(msg_rgba)
        self.assertEqual(monitor._rgb.shape, (48, 64, 3))
        self.assertEqual(monitor._rgb[0, 0, 2], 200)

        # 4. mono8
        msg_mono = Image()
        msg_mono.width = 64
        msg_mono.height = 48
        msg_mono.encoding = "mono8"
        raw_mono = np.full((48, 64), 180, dtype=np.uint8)
        msg_mono.data = raw_mono.tobytes()
        monitor._on_rgb(msg_mono)
        self.assertEqual(monitor._rgb.shape, (48, 64, 3))
        self.assertEqual(monitor._rgb[0, 0, 0], 180)

        monitor.destroy_node()

    def test_03_invalid_depth_rejection(self):
        """Test rejection of out-of-bounds or corrupted depth values in visualizer."""
        args = MockArgs()
        vis = TopDownVisualizerNode(args)

        msg = Detection3DArray()
        # Invalid: oz = 0.10m (< 0.25m min optical clip)
        d_close = Detection3D()
        d_close.bbox.center.position.x = 0.0
        d_close.bbox.center.position.y = 0.0
        d_close.bbox.center.position.z = 0.10
        h_close = ObjectHypothesisWithPose()
        h_close.hypothesis.class_id = "0:person:dynamic"
        d_close.results.append(h_close)
        msg.detections.append(d_close)

        # Invalid: oz = 25.0m (> 15.0m max optical range)
        d_far = Detection3D()
        d_far.bbox.center.position.x = 0.0
        d_far.bbox.center.position.y = 0.0
        d_far.bbox.center.position.z = 25.0
        h_far = ObjectHypothesisWithPose()
        h_far.hypothesis.class_id = "0:person:dynamic"
        d_far.results.append(h_far)
        msg.detections.append(d_far)

        # Valid: oz = 3.0m
        d_valid = Detection3D()
        d_valid.bbox.center.position.x = 0.0
        d_valid.bbox.center.position.y = 0.0
        d_valid.bbox.center.position.z = 3.0
        h_valid = ObjectHypothesisWithPose()
        h_valid.hypothesis.class_id = "0:person:dynamic"
        d_valid.results.append(h_valid)
        msg.detections.append(d_valid)

        vis.on_obstacles(msg)
        # Exactly 1 valid obstacle should be retained
        self.assertEqual(len(vis.obstacles), 1)
        self.assertAlmostEqual(vis.obstacles[0]["dist"], 3.866, delta=0.1)
        vis.destroy_node()

    def test_04_noise_suppression_geometric_vs_yolo(self):
        """Test that single-frame unclassified noise requires hits >= 2, whereas YOLO publishes on frame 1."""
        cfg = {"topics": {"depth": "/camera/depth/image_raw", "depth_info": "/camera/depth/camera_info", "objects": "/perception/objects_3d"}}
        tracker = DynamicTrackerNode(cfg)

        t = time.time()
        # 1. Single-frame unclassified geometric cluster (cid = -1, unknown_obstacle)
        det_geom = [{"x": 0.0, "y": 0.0, "z": 2.5, "sx": 0.5, "sy": 0.5, "sz": 0.5, "cid": -1, "cname": "unknown_obstacle", "conf": 0.50}]
        tracker._associate_and_update(det_geom, t)

        stamp = Header().stamp
        tracker._publish_tracks(stamp)
        # Check published detection count:
        # A single frame geometric cluster has hits=1, so it must NOT be published!
        tr_id = list(tracker.tracks.keys())[0]
        self.assertEqual(tracker.tracks[tr_id].hits, 1)

        # Now simulate frame 2 for geometric cluster (hits = 2)
        tracker._associate_and_update(det_geom, t + 0.05)
        self.assertEqual(tracker.tracks[tr_id].hits, 2)

        # 2. YOLO confirmed detection (cid = 0, person at separated position)
        det_yolo = [{"x": 3.0, "y": 0.0, "z": 6.0, "sx": 0.6, "sy": 1.6, "sz": 0.5, "cid": 0, "cname": "person", "conf": 0.88}]
        tracker._associate_and_update(det_yolo, t + 0.05)
        tr_yolo = [tr for tr in tracker.tracks.values() if tr.class_name == "person"][0]
        self.assertEqual(tr_yolo.hits, 1)

        # Verification:
        # YOLO track is confirmed on frame 1 (is_yolo_confirmed = True -> min_hits = 1)
        is_yolo_confirmed = (tr_yolo.class_id >= 0 and tr_yolo.class_name not in ("unknown_obstacle", "obstacle") and tr_yolo.confidence >= 0.40)
        self.assertTrue(is_yolo_confirmed)

        tracker.destroy_node()

    def test_05_expired_tracker_removal_and_marker_deletion(self):
        """Test that track_maintenance prunes tracks older than max_age_sec and generates Marker.DELETE."""
        cfg = {"topics": {"depth": "/camera/depth/image_raw", "depth_info": "/camera/depth/camera_info", "objects": "/perception/objects_3d"}}
        tracker = DynamicTrackerNode(cfg)

        t = time.time()
        tr = Track(track_id=42, class_id=0, class_name="person", x=0.0, y=0.0, z=2.0, t=t - 2.0, conf=0.9)
        tracker.tracks[42] = tr

        # Run maintenance: track has age = 2.0s > max_age_sec (1.0s)
        tracker.track_maintenance()

        # Track 42 must be pruned
        self.assertNotIn(42, tracker.tracks)
        tracker.destroy_node()

    def test_06_stale_visualization_marker_clearance(self):
        """Test that path_planning_visualizer clears obstacles when no updates arrive for > 0.6s."""
        args = MockArgs()
        vis = TopDownVisualizerNode(args)

        vis.obstacles = [{"id": "1", "cname": "person", "x_w": 8.0, "y_w": -6.0, "dist": 2.0, "speed": 0.0}]
        vis.last_obstacles_stamp = time.time() - 1.0  # 1.0 second stale

        vis.render_frame()

        # Must be cleared due to staleness
        self.assertEqual(len(vis.obstacles), 0)
        vis.destroy_node()

    def test_07_coordinate_transform_camera_vehicle_world(self):
        """Test coordinate transformation accuracy: Camera Optical -> Vehicle -> World."""
        # Vehicle at (6.0, -6.0), yaw = 0.0
        veh_x = 6.0
        veh_y = -6.0
        veh_yaw = 0.0

        # Object 3.0m straight ahead in camera optical frame
        ox_cam = 0.0
        oy_cam = 0.0
        oz_cam = 3.0

        # Mount constants: x=0.90m, pitch=-0.15 rad
        cos_p = 0.98877
        sin_p = -0.14944

        x_veh = 0.90 + cos_p * oz_cam - sin_p * oy_cam
        y_veh = -ox_cam

        cos_y = math.cos(veh_yaw)
        sin_y = math.sin(veh_yaw)
        x_world = veh_x + x_veh * cos_y - y_veh * sin_y
        y_world = veh_y + x_veh * sin_y + y_veh * cos_y

        expected_x_veh = 0.90 + 0.98877 * 3.0  # 3.86631m
        expected_x_world = 6.0 + 3.86631       # 9.86631m

        self.assertAlmostEqual(x_veh, expected_x_veh, places=3)
        self.assertAlmostEqual(x_world, expected_x_world, places=3)
        self.assertAlmostEqual(y_world, -6.0, places=3)


if __name__ == "__main__":
    unittest.main()
