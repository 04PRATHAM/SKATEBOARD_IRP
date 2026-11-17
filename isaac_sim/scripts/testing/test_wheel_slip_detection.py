"""Test script verifying Wheel-Slip and Physical Stuck Detection with RGB-D Visual Odometry."""

import os
import sys
import time
import socket
import struct
import numpy as np

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")
sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP/ros2_ws/src/rgbd_perception")

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from rgbd_perception.rgbd_odometry_node import RgbdOdometryNode
from rgbd_perception.autonomous_planner_node import AutonomousPlannerNode
from rgbd_perception.config import load_config

def run_wheel_slip_verification():
    print("=" * 80)
    print("VERIFICATION: WHEEL-SLIP DETECTION & INDEPENDENT VISUAL MOTION ESTIMATION")
    print("=" * 80)

    rclpy.init()
    cfg = load_config()

    # 1. Initialize Odometry and Planner nodes
    odom_node = RgbdOdometryNode(cfg, start_pose=[6.0, -6.0, 0.35, 0.0])
    planner_node = AutonomousPlannerNode(cfg, scenario_data={
        "vehicle_start_pose": [6.0, -6.0, 0.35, 0.0],
        "vehicle_destination_pose": [13.5, -6.0, 0.0]
    })

    # Prepare sockets and publishers
    enc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # Track states
    recorded_odom_x = []
    slip_states = []

    # Simulation Test Case 1: Normal Motion (Visual movement agrees with encoders)
    print("\n--- TEST CASE 1: Normal Coordinated Motion (No Wheel Slip) ---")
    # Feed encoder velocity = 0.8 m/s
    cmd_msg = AckermannDriveStamped()
    cmd_msg.drive.speed = 0.8
    odom_node.on_cmd(cmd_msg)

    # Set matching visual velocity
    odom_node.v_visual = 0.78
    odom_node.visual_conf = 20
    odom_node.last_visual_time = time.time()

    for step in range(10):
        # Send encoder feedback
        enc_sock.sendto(struct.pack("ff", 0.80, 0.0), ("127.0.0.1", 9878))
        odom_node._integrate(0.1)
        recorded_odom_x.append(odom_node.x)

    print(f"  Start X: 6.00m -> End X: {odom_node.x:.2f}m (Displacement = {odom_node.x - 6.0:.2f}m)")
    print(f"  Wheel Slip Flag: {odom_node.wheel_slip_detected}")
    assert not odom_node.wheel_slip_detected, "False alarm: wheel slip reported during normal driving!"
    assert odom_node.x > 6.5, "Odometry failed to advance during normal motion!"

    # Simulation Test Case 2: Physically Blocked with Spinning Wheels
    print("\n--- TEST CASE 2: Physically Blocked with Wheels Spinning (Wheel Slip Failure Mode) ---")
    start_blocked_x = odom_node.x
    # Motors are commanded to drive, wheels continue spinning at 0.85 m/s
    cmd_msg.drive.speed = 0.85
    odom_node.on_cmd(cmd_msg)

    # But camera is stationary against obstacle (v_visual = 0.0 m/s)
    odom_node.v_visual = 0.00
    odom_node.visual_conf = 25

    blocked_steps = 15
    for step in range(blocked_steps):
        odom_node.on_cmd(cmd_msg)
        odom_node.last_visual_time = time.time()
        # Wheels spin at 0.85 m/s
        enc_sock.sendto(struct.pack("ff", 0.85, 0.0), ("127.0.0.1", 9878))
        odom_node._integrate(0.1)
        slip_states.append(odom_node.wheel_slip_detected)
        time.sleep(0.08)

    end_blocked_x = odom_node.x
    advancement = end_blocked_x - start_blocked_x
    print(f"  Blocked Start X: {start_blocked_x:.2f}m -> End X: {end_blocked_x:.2f}m (Advancement = {advancement:.3f}m)")
    print(f"  Wheel Slip Triggered: {odom_node.wheel_slip_detected}")
    
    assert odom_node.wheel_slip_detected, "Failed to detect wheel slip when wheels spin while vehicle is blocked!"
    assert advancement < 0.15, f"Odometry advanced by {advancement:.2f}m while vehicle was blocked!"

    # Test Case 3: Planner Reacts to Wheel Slip with Safe Stop
    print("\n--- TEST CASE 3: Planner Safe Stop & State Transition ---")
    # Notify planner of wheel slip
    slip_msg = Bool()
    slip_msg.data = True
    planner_node.on_wheel_slip(slip_msg)
    # Simulate planner control tick
    planner_node.odom_received = True
    planner_node.perception_received = True
    planner_node.init_time = time.time() - 10.0
    planner_node.control_loop()

    print(f"  Planner State: {planner_node.state}")
    assert planner_node.state == "WHEEL_SLIP", f"Planner failed to transition to WHEEL_SLIP! (state={planner_node.state})"

    # Test Case 4: False Alarm Suppression on Goal Arrival
    print("\n--- TEST CASE 4: False Alarm Suppression on Pause & Goal Arrival ---")
    planner_node.state = "GOAL_REACHED"
    planner_node.veh_x = 13.5
    planner_node.veh_y = -6.0
    planner_node.control_loop()
    print(f"  State at Goal: {planner_node.state} (Inhibited slip trigger: True)")
    assert planner_node.state == "GOAL_REACHED"

    # Test Pause Suppression
    pause_msg = Bool()
    pause_msg.data = True
    odom_node.on_pause_drive(pause_msg)
    odom_node.speed = 0.8
    odom_node.cmd_speed = 0.8
    odom_node.v_visual = 0.0
    odom_node.last_visual_time = time.time()
    odom_node._integrate(0.1)
    print(f"  Wheel Slip under Pause: {odom_node.wheel_slip_detected} (Expected: False)")
    assert not odom_node.wheel_slip_detected, "Wheel slip triggered while paused!"

    print("\n" + "=" * 80)
    print("ALL WHEEL-SLIP DETECTION & INDEPENDENT MOTION TESTS PASSED!")
    print("=" * 80)

    odom_node.destroy_node()
    planner_node.destroy_node()
    rclpy.shutdown()
    return 0

if __name__ == "__main__":
    sys.exit(run_wheel_slip_verification())
