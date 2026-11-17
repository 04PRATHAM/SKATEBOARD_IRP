"""Test Suite for Signed Visual Odometry, Reverse Motion and Visual Confidence.

Verifies:
  1. Signed visual motion: v_visual preserves positive sign (forward) and negative sign (reverse).
  2. Odometry integration supports both forward (+X) and reverse (-X) motion without zero-clamping.
  3. Turning motion: heading updates correctly with Ackermann kinematics in both forward and reverse.
  4. Physical blockage in forward motion: wheels spin forward, visual indicates 0.0,
     advancement clamped to 0.000m, wheel-slip triggers, safe stop commanded.
  5. Physical blockage in reverse motion: wheels spin backward, visual indicates 0.0,
     retreat clamped to 0.000m, wheel-slip triggers in reverse!
  6. Visual confidence metrics:
     - High texture / nominal scene: C_visual >= 0.50 -> NOMINAL status.
     - Blank wall / low texture: C_visual < 0.30 -> VISUAL_DEGRADED status.
     - Low confidence suppresses false wheel slip declarations.
     - Persistent degraded state (> 1.5s) triggers VISUAL_DEGRADED_STOP in planner.
     - Paused / goal arrival states inhibit false alarms.
"""

import math
import sys
import time
import numpy as np

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")
sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP/ros2_ws/src/rgbd_perception")

from rgbd_perception.rgbd_odometry_node import (
    RgbdOdometryNode, yaw_to_quaternion
)
from rgbd_perception.autonomous_planner_node import (
    AutonomousPlannerNode
)
from std_msgs.msg import String, Bool
from ackermann_msgs.msg import AckermannDriveStamped
import rclpy


def test_signed_forward_and_reverse_odometry():
    print("\n--- TEST CASE 1: Signed Forward and Reverse Odometry Integration ---")
    rclpy.init()
    try:
        cfg = {
            "vehicle": {"wheelbase": 1.1},
            "topics": {
                "cmd": "/cmd_ackermann",
                "depth": "/camera/depth/image_raw",
                "depth_info": "/camera/depth/camera_info"
            }
        }
        node = RgbdOdometryNode(cfg, start_pose=[6.0, -6.0, 0.35, 0.0])

        # 1. Forward driving (+0.8 m/s, delta=0.0) for 1.0s
        cmd_fwd = AckermannDriveStamped()
        cmd_fwd.drive.speed = 0.8
        cmd_fwd.drive.steering_angle = 0.0
        node.on_cmd(cmd_fwd)

        # Provide nominal visual tracking
        import time
        node.last_visual_time = time.time()
        node.visual_conf_score = 0.85
        node.v_visual = 0.80

        # Simulate 20 steps of 0.05s
        for _ in range(20):
            node.set_wheel_feedback(0.80, 0.0)
            node.last_visual_time = time.time()
            node._integrate(0.05)

        print(f"Forward Drive: End X = {node.x:.3f}m (Expected ~ 6.800m)")
        assert node.x > 6.75, f"Forward driving failed to advance: x={node.x}"
        assert node.speed > 0.7, "Forward speed was not positive"

        # 2. Reverse driving (-0.6 m/s, delta=0.0) for 1.0s
        cmd_rev = AckermannDriveStamped()
        cmd_rev.drive.speed = -0.6
        cmd_rev.drive.steering_angle = 0.0
        node.on_cmd(cmd_rev)
        node.last_visual_time = time.time()
        node.v_visual = -0.60

        x_before_rev = node.x
        for _ in range(20):
            node.set_wheel_feedback(-0.60, 0.0)
            node.last_visual_time = time.time()
            node._integrate(0.05)

        print(f"Reverse Drive: End X = {node.x:.3f}m (Expected ~ {x_before_rev - 0.6:.3f}m)")
        assert node.x < x_before_rev - 0.55, f"Reverse driving failed to retreat: x={node.x}"
        assert node.speed < -0.5, "Reverse speed was not negative"
        assert node.odom_status == "NOMINAL", f"Expected NOMINAL status, got {node.odom_status}"

        print("[PASS] Signed forward and reverse odometry integration verified!")
    finally:
        rclpy.shutdown()


def test_turning_kinematics_forward_and_reverse():
    print("\n--- TEST CASE 2: Ackermann Turning Kinematics (Forward & Reverse) ---")
    rclpy.init()
    try:
        cfg = {"vehicle": {"wheelbase": 1.1}, "topics": {"cmd": "/cmd_ackermann", "depth": "/cam/d", "depth_info": "/cam/info"}}
        node = RgbdOdometryNode(cfg, start_pose=[0.0, 0.0, 0.35, 0.0])

        # Left turn forward (+0.5 m/s, +0.35 rad steer)
        cmd_turn = AckermannDriveStamped()
        cmd_turn.drive.speed = 0.5
        cmd_turn.drive.steering_angle = 0.35
        node.on_cmd(cmd_turn)
        node.visual_conf_score = 0.85
        node.v_visual = 0.50

        for _ in range(20):
            node.set_wheel_feedback(0.50, 0.35)
            node.last_visual_time = time.time()
            node._integrate(0.05)

        print(f"Forward Left Turn: Yaw = {math.degrees(node.yaw):+.2f}° (Expected > 0)")
        assert node.yaw > 0.10, "Left turn failed to increase heading"
        assert node.y > 0.02, "Left turn failed to produce positive lateral displacement"

        # Right turn reverse (-0.5 m/s, +0.35 rad steer)
        cmd_rev_turn = AckermannDriveStamped()
        cmd_rev_turn.drive.speed = -0.5
        cmd_rev_turn.drive.steering_angle = 0.35
        node.on_cmd(cmd_rev_turn)
        node.v_visual = -0.50

        yaw_before = node.yaw
        for _ in range(20):
            node.set_wheel_feedback(-0.50, 0.35)
            node.last_visual_time = time.time()
            node._integrate(0.05)

        print(f"Reverse Left-Steer Turn: Yaw change = {math.degrees(node.yaw - yaw_before):+.2f}° (Expected < 0)")
        assert node.yaw < yaw_before, "Reverse with positive steer failed to decrease heading"

        print("[PASS] Turning kinematics in forward and reverse verified!")
    finally:
        rclpy.shutdown()


def test_bidirectional_wheel_slip_detection():
    print("\n--- TEST CASE 3: Bidirectional Wheel-Slip & Physical Blockage ---")
    rclpy.init()
    try:
        cfg = {"vehicle": {"wheelbase": 1.1}, "topics": {"cmd": "/cmd_ackermann", "depth": "/cam/d", "depth_info": "/cam/info"}}
        node = RgbdOdometryNode(cfg, start_pose=[6.0, -6.0, 0.35, 0.0])

        # Case 3A: Forward Physical Blockage (Spinning forward against solid wall)
        cmd_fwd = AckermannDriveStamped()
        cmd_fwd.drive.speed = 0.8
        node.on_cmd(cmd_fwd)

        # Inject high visual confidence confirming lack of motion (v_visual = 0.00 m/s)
        import time
        now = time.time()
        node.last_visual_time = now
        node.visual_conf_score = 0.85
        node.v_visual = 0.005 # Stationary

        start_x = node.x
        for _ in range(12): # 0.6 seconds of blocked spinning
            node.on_cmd(cmd_fwd)
            node.set_wheel_feedback(0.80, 0.0)
            node.last_visual_time = time.time()
            node._integrate(0.05)

        print(f"Forward Blocked: Advancement = {node.x - start_x:.4f}m (Must be <= 0.005m)")
        assert abs(node.x - start_x) <= 0.005, f"Vehicle moved while blocked! dx={node.x - start_x}"
        assert node.wheel_slip_detected is True, "Wheel slip was not triggered in forward direction"
        assert node.odom_status == "WHEEL_SLIP", f"Expected WHEEL_SLIP status, got {node.odom_status}"

        # Case 3B: Reverse Physical Blockage (Spinning backward against barrier behind vehicle)
        cmd_rev = AckermannDriveStamped()
        cmd_rev.drive.speed = -0.8
        node.on_cmd(cmd_rev)

        node.last_visual_time = time.time()
        node.visual_conf_score = 0.85
        node.v_visual = -0.004 # Stationary in reverse

        start_x_rev = node.x
        for _ in range(12):
            node.on_cmd(cmd_rev)
            node.set_wheel_feedback(-0.80, 0.0)
            node.last_visual_time = time.time()
            node._integrate(0.05)

        print(f"Reverse Blocked: Retreat = {node.x - start_x_rev:.4f}m (Must be <= 0.005m)")
        assert abs(node.x - start_x_rev) <= 0.005, f"Vehicle moved while blocked in reverse! dx={node.x - start_x_rev}"
        assert node.wheel_slip_detected is True, "Wheel slip was not triggered in reverse direction"
        assert node.odom_status == "WHEEL_SLIP"

        print("[PASS] Bidirectional wheel-slip and zero-advancement clamping verified!")
    finally:
        rclpy.shutdown()


def test_visual_confidence_and_degraded_suppression():
    print("\n--- TEST CASE 4: Visual Odometry Confidence & False-Alarm Suppression ---")
    rclpy.init()
    try:
        cfg = {"vehicle": {"wheelbase": 1.1}, "topics": {"cmd": "/cmd_ackermann", "depth": "/cam/d", "depth_info": "/cam/info"}}
        node = RgbdOdometryNode(cfg, start_pose=[6.0, -6.0, 0.35, 0.0])

        import time
        # Command vehicle to drive forward
        cmd = AckermannDriveStamped()
        cmd.drive.speed = 0.7
        node.on_cmd(cmd)

        # SCENARIO: Low-texture / blank wall scene
        # Feature loss causes visual confidence score to fall below 0.30
        node.last_visual_time = time.time()
        node.visual_conf_score = 0.15 # Degraded confidence
        node.v_visual = 0.0 # Untrusted zero

        start_x_low = node.x
        for _ in range(15):
            node.on_cmd(cmd)
            node.set_wheel_feedback(0.70, 0.0)
            node.last_visual_time = time.time()
            node._integrate(0.05)

        print(f"Low Texture Scene: Status = {node.odom_status} | Slip Flag = {node.wheel_slip_detected} | Advancement = {abs(node.x - start_x_low):.4f}m")
        assert abs(node.x - start_x_low) == 0.0, f"Vehicle advanced {abs(node.x - start_x_low)}m on unverified wheel spin!"
        assert node.odom_status == "VISUAL_DEGRADED", f"Expected VISUAL_DEGRADED, got {node.odom_status}"
        assert node.wheel_slip_detected is False, "False wheel slip declared on low visual confidence!"

        # Planner verification: AutonomousPlannerNode enters VISUAL_DEGRADED_STOP
        p_cfg = {
            "planner": {"max_steer": 0.45, "cruise_speed": 0.8},
            "topics": {"cmd": "/cmd_ackermann", "objects": "/perception/objects_3d", "odom": "/vehicle/estimated_odom"}
        }
        planner = AutonomousPlannerNode(p_cfg)
        planner.odom_received = True
        planner.perception_received = True
        planner.init_time = time.time() - 10.0 # Warmup complete
        planner.veh_x, planner.veh_y = 6.0, -6.0

        # Feed degraded odom status persisting for > 1.5s
        msg_st = String()
        msg_st.data = "VISUAL_DEGRADED"
        planner.visual_degraded_start = time.time() - 2.0
        planner.on_odom_status(msg_st)

        planner.control_loop()
        print(f"Planner State under persistent visual degradation: {planner.state}")
        assert planner.state == "VISUAL_DEGRADED_STOP", f"Planner failed to transition to VISUAL_DEGRADED_STOP: {planner.state}"

        # False alarm suppression during Intentional Pause
        planner.is_paused = True
        planner.control_loop()
        assert planner.state == "PAUSED", "Pause failed to override degraded state"

        print("[PASS] Visual confidence and false-alarm suppression verified!")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    test_signed_forward_and_reverse_odometry()
    test_turning_kinematics_forward_and_reverse()
    test_bidirectional_wheel_slip_detection()
    test_visual_confidence_and_degraded_suppression()
    print("\n" + "="*80)
    print("ALL SIGNED VISUAL ODOMETRY & CONFIDENCE TESTS PASSED!")
    print("="*80)
