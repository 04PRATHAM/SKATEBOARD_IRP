"""Phase 3 Verification: Comprehensive Odometry Integrity & Degradation Test Suite.

Verifies:
  1. Forward motion: positive speed, positive visual odometry -> forward progression.
  2. Reverse motion: negative speed, negative visual odometry -> reverse retreat (signed velocity preserved).
  3. Steering kinematics: non-zero steer angle -> yaw updates correctly per Ackermann model.
  4. Physical blockage: wheels spin forward, visual indicates 0.0 -> displacement clamped, WHEEL_SLIP declared.
  5. Spinning wheels with stationary chassis: commanded speed & spinning wheels do not advance odometry when chassis stationary.
  6. Low visual texture: confidence degraded (< 0.30) -> VISUAL_DEGRADED status reported, position does NOT advance on unverified spin.
  7. Interrupted camera stream: stream timeout -> CAM_INTERRUPTED status reported, position does NOT advance.
  8. Commanded speed fallback elimination: cmd_speed alone NEVER produces motion when encoder is missing.
"""

import math
import os
import sys
import time
import struct
import numpy as np

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")
sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP/ros2_ws/src/rgbd_perception")

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rgbd_perception.rgbd_odometry_node import RgbdOdometryNode


def run_all_odometry_integrity_tests():
    rclpy.init()
    results = {}

    cfg = {
        "vehicle": {"wheelbase": 1.1},
        "topics": {
            "cmd": "/cmd_ackermann",
            "depth": "/camera/depth/image_raw",
            "depth_info": "/camera/depth/camera_info"
        }
    }

    print("=" * 80)
    print("PHASE 3: COMPREHENSIVE ODOMETRY INTEGRITY ACCEPTANCE TESTS")
    print("=" * 80)

    # --------------------------------------------------------------------------
    # Case 1: Forward Motion
    # --------------------------------------------------------------------------
    print("\n[Case 1/8] Forward Motion with Confident Visual Odometry...")
    node1 = RgbdOdometryNode(cfg, start_pose=[6.0, -6.0, 0.35, 0.0])
    cmd = AckermannDriveStamped()
    cmd.drive.speed = 0.8
    cmd.drive.steering_angle = 0.0
    node1.on_cmd(cmd)
    node1.set_wheel_feedback(0.80, 0.0)
    node1.last_visual_time = time.time()
    node1.visual_conf_score = 0.85
    node1.v_visual = 0.80

    for _ in range(20):
        node1.set_wheel_feedback(0.80, 0.0)
        node1.last_visual_time = time.time()
        node1._integrate(0.05)

    dx_fwd = node1.x - 6.0
    print(f"  Result: Final X = {node1.x:.3f}m, dx = {dx_fwd:+.3f}m, Status = {node1.odom_status}")
    assert dx_fwd > 0.70, f"Case 1 Failed: dx={dx_fwd} <= 0.70"
    assert node1.odom_status == "NOMINAL", f"Case 1 Failed: Status={node1.odom_status}"
    results["Case 1: Forward Motion"] = "PASS"

    # --------------------------------------------------------------------------
    # Case 2: Reverse Motion (Signed Velocity Preservation)
    # --------------------------------------------------------------------------
    print("\n[Case 2/8] Reverse Motion (Signed Velocity Preservation)...")
    node2 = RgbdOdometryNode(cfg, start_pose=[6.0, -6.0, 0.35, 0.0])
    cmd_rev = AckermannDriveStamped()
    cmd_rev.drive.speed = -0.6
    cmd_rev.drive.steering_angle = 0.0
    node2.on_cmd(cmd_rev)
    node2.set_wheel_feedback(-0.60, 0.0)
    node2.last_visual_time = time.time()
    node2.visual_conf_score = 0.85
    node2.v_visual = -0.60

    for _ in range(20):
        node2.set_wheel_feedback(-0.60, 0.0)
        node2.last_visual_time = time.time()
        node2._integrate(0.05)

    dx_rev = node2.x - 6.0
    print(f"  Result: Final X = {node2.x:.3f}m, dx = {dx_rev:+.3f}m, Status = {node2.odom_status}")
    assert dx_rev < -0.50, f"Case 2 Failed: dx={dx_rev} >= -0.50 (clamped to zero!)"
    assert node2.odom_status == "NOMINAL", f"Case 2 Failed: Status={node2.odom_status}"
    results["Case 2: Reverse Motion"] = "PASS"

    # --------------------------------------------------------------------------
    # Case 3: Steering Kinematics (Ackermann Model)
    # --------------------------------------------------------------------------
    print("\n[Case 3/8] Steering Kinematics (Ackermann Model)...")
    node3 = RgbdOdometryNode(cfg, start_pose=[0.0, 0.0, 0.35, 0.0])
    cmd_steer = AckermannDriveStamped()
    cmd_steer.drive.speed = 0.5
    cmd_steer.drive.steering_angle = 0.35  # ~20 degrees left
    node3.on_cmd(cmd_steer)
    node3.set_wheel_feedback(0.50, 0.35)
    node3.last_visual_time = time.time()
    node3.visual_conf_score = 0.85
    node3.v_visual = 0.50

    for _ in range(20):
        node3.set_wheel_feedback(0.50, 0.35)
        node3.last_visual_time = time.time()
        node3._integrate(0.05)

    yaw_deg = math.degrees(node3.yaw)
    print(f"  Result: Yaw = {yaw_deg:+.2f} deg, Y = {node3.y:+.3f}m, Status = {node3.odom_status}")
    assert yaw_deg > 5.0, f"Case 3 Failed: Yaw={yaw_deg} <= 5.0 deg"
    assert node3.y > 0.02, f"Case 3 Failed: Lateral Y={node3.y} <= 0.02m"
    results["Case 3: Steering Kinematics"] = "PASS"

    # --------------------------------------------------------------------------
    # Case 4: Physical Blockage (Wheels Spin, Visual Confirms Stationary)
    # --------------------------------------------------------------------------
    print("\n[Case 4/8] Physical Blockage (Wheels Spin Against Obstacle)...")
    node4 = RgbdOdometryNode(cfg, start_pose=[6.0, -6.0, 0.35, 0.0])
    cmd_block = AckermannDriveStamped()
    cmd_block.drive.speed = 0.8
    node4.on_cmd(cmd_block)
    node4.set_wheel_feedback(0.80, 0.0)
    node4.last_visual_time = time.time()
    node4.visual_conf_score = 0.85
    node4.v_visual = 0.002  # Stationary

    start_x = node4.x
    for _ in range(15):  # 0.75 seconds of blocked spinning
        node4.set_wheel_feedback(0.80, 0.0)
        node4.last_visual_time = time.time()
        node4._integrate(0.05)

    advancement = abs(node4.x - start_x)
    print(f"  Result: Advancement = {advancement:.4f}m, SlipFlag = {node4.wheel_slip_detected}, Status = {node4.odom_status}")
    assert advancement <= 0.005, f"Case 4 Failed: Advanced {advancement}m while blocked!"
    assert node4.wheel_slip_detected is True, "Case 4 Failed: Wheel slip flag not set"
    assert node4.odom_status == "WHEEL_SLIP", f"Case 4 Failed: Status={node4.odom_status}"
    results["Case 4: Physical Blockage"] = "PASS"

    # --------------------------------------------------------------------------
    # Case 5: Spinning Wheels with Stationary Chassis
    # --------------------------------------------------------------------------
    print("\n[Case 5/8] Spinning Wheels with Stationary Chassis (Motor Spin Rejection)...")
    node5 = RgbdOdometryNode(cfg, start_pose=[6.0, -6.0, 0.35, 0.0])
    cmd_spin = AckermannDriveStamped()
    cmd_spin.drive.speed = 0.85
    node5.on_cmd(cmd_spin)
    node5.set_wheel_feedback(0.85, 0.0)
    node5.last_visual_time = time.time()
    node5.visual_conf_score = 0.90
    node5.v_visual = 0.000

    start_x5 = node5.x
    for _ in range(20):
        node5.set_wheel_feedback(0.85, 0.0)
        node5.last_visual_time = time.time()
        node5._integrate(0.05)

    adv5 = abs(node5.x - start_x5)
    print(f"  Result: Advancement = {adv5:.4f}m, SlipFlag = {node5.wheel_slip_detected}, Status = {node5.odom_status}")
    assert adv5 <= 0.001, f"Case 5 Failed: Advanced {adv5}m on stationary chassis!"
    assert node5.wheel_slip_detected is True, "Case 5 Failed: Slip flag should be True"
    assert node5.odom_status == "WHEEL_SLIP", f"Case 5 Failed: Status={node5.odom_status}"
    results["Case 5: Spinning Wheels Stationary Chassis"] = "PASS"

    # --------------------------------------------------------------------------
    # Case 6: Low Visual Texture (Unverified Wheel Spin)
    # --------------------------------------------------------------------------
    print("\n[Case 6/8] Low Visual Texture / Blank Wall (Unverified Wheel Spin)...")
    node6 = RgbdOdometryNode(cfg, start_pose=[6.0, -6.0, 0.35, 0.0])
    cmd_lowtex = AckermannDriveStamped()
    cmd_lowtex.drive.speed = 0.70
    node6.on_cmd(cmd_lowtex)
    node6.set_wheel_feedback(0.70, 0.0)
    node6.last_visual_time = time.time()
    node6.visual_conf_score = 0.12  # Degraded confidence
    node6.v_visual = 0.0

    start_x6 = node6.x
    for _ in range(15):
        node6.set_wheel_feedback(0.70, 0.0)
        node6.last_visual_time = time.time()
        node6._integrate(0.05)

    adv6 = abs(node6.x - start_x6)
    print(f"  Result: Advancement = {adv6:.4f}m, Status = {node6.odom_status}, SlipFlag = {node6.wheel_slip_detected}")
    assert adv6 == 0.0, f"Case 6 Failed: Advanced {adv6}m on unverified wheel spin!"
    assert node6.odom_status == "VISUAL_DEGRADED", f"Case 6 Failed: Status={node6.odom_status}"
    assert node6.wheel_slip_detected is False, "Case 6 Failed: False wheel slip declared on low confidence!"
    results["Case 6: Low Visual Texture"] = "PASS"

    # --------------------------------------------------------------------------
    # Case 7: Interrupted Camera Stream
    # --------------------------------------------------------------------------
    print("\n[Case 7/8] Interrupted Camera Stream...")
    node7 = RgbdOdometryNode(cfg, start_pose=[6.0, -6.0, 0.35, 0.0])
    cmd_cam_loss = AckermannDriveStamped()
    cmd_cam_loss.drive.speed = 0.70
    node7.on_cmd(cmd_cam_loss)
    node7.set_wheel_feedback(0.70, 0.0)
    # Camera stream halted 1.5s ago
    node7.last_visual_time = time.time() - 1.5

    start_x7 = node7.x
    for _ in range(15):
        node7.set_wheel_feedback(0.70, 0.0)
        node7._integrate(0.05)

    adv7 = abs(node7.x - start_x7)
    print(f"  Result: Advancement = {adv7:.4f}m, Status = {node7.odom_status}")
    assert adv7 == 0.0, f"Case 7 Failed: Advanced {adv7}m with interrupted camera!"
    assert node7.odom_status == "CAM_INTERRUPTED", f"Case 7 Failed: Status={node7.odom_status}"
    results["Case 7: Interrupted Camera Stream"] = "PASS"

    # --------------------------------------------------------------------------
    # Case 8: Commanded Speed Fallback Elimination (No Encoder Feedback)
    # --------------------------------------------------------------------------
    print("\n[Case 8/8] Commanded Speed Fallback Elimination (No Encoder Feedback)...")
    node8 = RgbdOdometryNode(cfg, start_pose=[6.0, -6.0, 0.35, 0.0])
    # Send commands but DO NOT provide encoder feedback
    cmd_phantom = AckermannDriveStamped()
    cmd_phantom.drive.speed = 0.85
    node8.on_cmd(cmd_phantom)
    node8.last_visual_time = time.time()
    node8.visual_conf_score = 0.85
    node8.v_visual = 0.0

    start_x8 = node8.x
    for _ in range(15):
        node8.last_visual_time = time.time()
        node8._integrate(0.05)

    adv8 = abs(node8.x - start_x8)
    print(f"  Result: Advancement = {adv8:.4f}m, Speed = {node8.speed:.2f} m/s, Status = {node8.odom_status}")
    assert adv8 == 0.0, f"Case 8 Failed: Advanced {adv8}m from phantom cmd_speed!"
    assert node8.speed == 0.0, f"Case 8 Failed: node.speed={node8.speed} set from cmd_speed!"
    assert node8.odom_status == "NO_ENCODER", f"Case 8 Failed: Status={node8.odom_status}"
    results["Case 8: No Encoder Fallback"] = "PASS"

    # Summary
    print("\n" + "=" * 80)
    print("PHASE 3 ODOMETRY INTEGRITY TEST RESULTS SUMMARY")
    print("=" * 80)
    all_pass = True
    for test_name, status in results.items():
        print(f"  {test_name:<50} [{status}]")
        if status != "PASS":
            all_pass = False

    print("=" * 80)
    if all_pass:
        print(">>> ALL 8 PHASE 3 ODOMETRY INTEGRITY TESTS PASSED! <<<")
    else:
        print(">>> SOME TESTS FAILED! <<<")
    print("=" * 80)

    rclpy.shutdown()
    return all_pass


if __name__ == "__main__":
    success = run_all_odometry_integrity_tests()
    sys.exit(0 if success else 1)
