"""Test Suite for ROS Detection3D Message Semantics, Tracking IDs and Geometric Frames.

Verifies:
  1. results[0].hypothesis.score represents classification confidence in [0.0, 1.0] (not velocity).
  2. d.id field explicitly contains the persistent Track ID string.
  3. results[1] ObjectHypothesis represents dedicated obstacle velocity (velocity_m_s).
  4. Companion topic /perception/tracked_velocities (DiagnosticArray) matches track IDs and speeds.
  5. Camera-to-vehicle frame transformation:
     - Exact mount translation (x=0.90, y=0.0, z=1.20) and pitch (-0.15 rad).
     - Optical Z maps to Vehicle X, Optical -X maps to Vehicle Y.
     - Physical bounding box dimensions (dx, dy, dz) are consistently mapped.
"""

import math
import sys
import numpy as np
import rclpy
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")
sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP/ros2_ws/src/rgbd_perception")

from rgbd_perception.dynamic_tracker_node import DynamicTrackerNode, Track
from rgbd_perception.autonomous_planner_node import AutonomousPlannerNode


def test_detection3d_semantics_and_companion_velocities():
    print("\n--- TEST CASE 1: Detection3D Message Semantics & Companion Tracking ---")
    rclpy.init()
    try:
        cfg = {
            "topics": {
                "objects": "/perception/objects_3d",
                "depth": "/camera/depth/image_raw",
                "depth_info": "/camera/depth/camera_info"
            }
        }
        tracker = DynamicTrackerNode(cfg)

        # Create Track 1: Moving Person (speed = 1.15 m/s, classification confidence = 0.94)
        t1 = Track(track_id=42, class_id=0, class_name="person", x=0.5, y=-0.2, z=3.0, t=10.0, conf=0.94)
        t1.vx = 0.05
        t1.vy = 0.00
        t1.vz = -1.15
        t1.speed = 1.15
        t1.is_dynamic = True
        t1.size_x = 0.45 # Width
        t1.size_y = 1.75 # Height
        t1.size_z = 0.42 # Depth extent
        t1.hits = 5
        tracker.tracks[42] = t1

        # Publish tracks and capture messages
        stamp = rclpy.time.Time(seconds=10, nanoseconds=0).to_msg()

        published_d3d = []
        published_diag = []

        class DummyPub:
            def publish(self, msg):
                published_d3d.append(msg)
        class DummyDiagPub:
            def publish(self, msg):
                published_diag.append(msg)

        tracker.pub_tracked = DummyPub()
        tracker.pub_velocities = DummyDiagPub()
        tracker._publish_tracks(stamp)

        assert len(published_d3d) == 1, "Failed to publish Detection3DArray"
        d3d_arr = published_d3d[0]
        assert len(d3d_arr.detections) == 1, "Expected 1 detection in array"
        d = d3d_arr.detections[0]

        # 1. Verify Track ID
        print(f"Detection ID: '{d.id}' (Expected: '42')")
        assert d.id == "42", f"d.id failed to preserve Track ID: {d.id}"

        # 2. Verify Classification Confidence in results[0]
        assert len(d.results) >= 2, f"Expected at least 2 hypotheses (class + vel), got {len(d.results)}"
        class_hyp = d.results[0]
        print(f"Classification Score: {class_hyp.hypothesis.score:.2f} (Expected: 0.94 confidence)")
        assert 0.0 <= class_hyp.hypothesis.score <= 1.0, f"Score out of probability bounds [0, 1]: {class_hyp.hypothesis.score}"
        assert abs(class_hyp.hypothesis.score - 0.94) < 1e-3, f"Score mismatch: {class_hyp.hypothesis.score}"
        assert "person" in class_hyp.hypothesis.class_id

        # 3. Verify Dedicated Velocity in results[1]
        vel_hyp = d.results[1]
        print(f"Velocity Hypothesis ID: '{vel_hyp.hypothesis.class_id}' | Speed: {vel_hyp.hypothesis.score:.2f} m/s")
        assert vel_hyp.hypothesis.class_id == "velocity_m_s", f"Expected velocity_m_s, got {vel_hyp.hypothesis.class_id}"
        assert abs(vel_hyp.hypothesis.score - 1.15) < 1e-3, f"Velocity mismatch: {vel_hyp.hypothesis.score}"
        assert abs(vel_hyp.pose.pose.position.z - (-1.15)) < 1e-3, f"vz mismatch: {vel_hyp.pose.pose.position.z}"

        # 4. Verify Physical Dimensions in bbox.size
        print(f"Bounding Size: x={d.bbox.size.x:.2f}m (width), y={d.bbox.size.y:.2f}m (height), z={d.bbox.size.z:.2f}m (depth extent)")
        assert abs(d.bbox.size.x - 0.45) < 1e-3
        assert abs(d.bbox.size.y - 1.75) < 1e-3
        assert abs(d.bbox.size.z - 0.42) < 1e-3

        # 5. Verify Companion Diagnostic Tracking Topic
        assert len(published_diag) == 1, "Failed to publish /perception/tracked_velocities"
        diag_arr = published_diag[0]
        assert len(diag_arr.status) == 1
        st = diag_arr.status[0]
        assert st.hardware_id == "42"
        val_dict = {kv.key: kv.value for kv in st.values}
        print(f"Companion Diagnostic Status: {val_dict}")
        assert float(val_dict["speed"]) == 1.15
        assert val_dict["class_name"] == "person"
        assert val_dict["is_dynamic"] == "True"

        print("[PASS] Detection3D message semantics and companion velocities verified!")
    finally:
        rclpy.shutdown()


def test_camera_to_vehicle_frame_transformation():
    print("\n--- TEST CASE 2: Camera Optical to Vehicle Base Frame Transformation ---")
    rclpy.init()
    try:
        p_cfg = {
            "planner": {"max_steer": 0.45, "cruise_speed": 0.8},
            "topics": {"cmd": "/cmd_ackermann", "objects": "/perception/objects_3d", "odom": "/vehicle/estimated_odom"}
        }
        planner = AutonomousPlannerNode(p_cfg)
        planner.veh_x, planner.veh_y, planner.veh_yaw = 6.0, -6.0, 0.0 # Facing +X East

        # Obstacle in optical frame: ox = +0.5m (right), oy = -0.1m (up), oz = 3.0m (forward along bore)
        # Mount parameters: x_mount = 0.90m, pitch = -0.15 rad (tilt down)
        # cos(-0.15) = 0.98877, sin(-0.15) = -0.14944
        # Expected Vehicle Coordinates:
        # x_veh = 0.90 + 0.98877 * 3.0 - (-0.14944) * (-0.1) = 0.90 + 2.9663 - 0.0149 = 3.851m
        # y_veh = -ox = -0.50m (right of camera -> right of vehicle, which is -y)
        # Expected World Coordinates (at veh_x=6.0, veh_y=-6.0, yaw=0.0):
        # x_world = 6.0 + 3.851 = 9.851m
        # y_world = -6.0 - 0.50 = -6.500m

        msg = Detection3DArray()
        d = Detection3D()
        d.id = "42"
        hyp_class = ObjectHypothesisWithPose()
        hyp_class.hypothesis.class_id = "0:person:dynamic"
        hyp_class.hypothesis.score = 0.94
        d.results.append(hyp_class)

        hyp_vel = ObjectHypothesisWithPose()
        hyp_vel.hypothesis.class_id = "velocity_m_s"
        hyp_vel.hypothesis.score = 1.15
        d.results.append(hyp_vel)

        d.bbox.center.position.x = 0.5
        d.bbox.center.position.y = -0.1
        d.bbox.center.position.z = 3.0
        d.bbox.size.x = 0.45
        d.bbox.size.y = 1.75
        d.bbox.size.z = 0.42
        msg.detections.append(d)

        planner.on_tracked_obstacles(msg)
        assert len(planner.obstacles) == 1
        obs = planner.obstacles[0]

        print(f"Vehicle Frame: x_veh = {obs['x_veh']:.3f}m, y_veh = {obs['y_veh']:.3f}m")
        print(f"World Frame:   x_world = {obs['x_world']:.3f}m, y_world = {obs['y_world']:.3f}m")

        assert abs(obs["x_veh"] - 3.851) < 0.02, f"x_veh transform error: {obs['x_veh']}"
        assert abs(obs["y_veh"] - (-0.50)) < 0.01, f"y_veh transform error: {obs['y_veh']}"
        assert abs(obs["x_world"] - 9.851) < 0.02, f"x_world transform error: {obs['x_world']}"
        assert abs(obs["y_world"] - (-6.50)) < 0.01, f"y_world transform error: {obs['y_world']}"
        assert abs(obs["speed"] - 1.15) < 1e-3, f"Speed extraction error: {obs['speed']}"

        print("[PASS] Camera to vehicle frame transformation verified!")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    test_detection3d_semantics_and_companion_velocities()
    test_camera_to_vehicle_frame_transformation()
    print("\n" + "="*80)
    print("ALL DETECTION3D SEMANTICS & GEOMETRY TESTS PASSED!")
    print("="*80)
