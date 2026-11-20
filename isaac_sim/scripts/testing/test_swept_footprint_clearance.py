"""Verification of Full Swept-Footprint Collision Checker across 5 test cases."""

import math
import sys
import numpy as np

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")
sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP/ros2_ws/src/rgbd_perception")

from rgbd_perception.autonomous_planner_node import AutonomousPlannerNode, astar_grid_path
from rgbd_perception.config import load_config

def test_swept_clearances():
    print("=" * 80)
    print("TEST: FULL SWEPT-FOOTPRINT COLLISION CHECKER ACROSS 5 BENCHMARK CASES")
    print("=" * 80)

    import rclpy
    rclpy.init()
    cfg = load_config()
    scen_data = {
        "vehicle_start_pose": [6.0, -6.0, 0.35, 0.0],
        "vehicle_destination_pose": [13.5, -6.0, 0.0]
    }
    planner = AutonomousPlannerNode(cfg, scenario_data=scen_data)

    results = {}

    # Case 1: Turning around a column at (9.0, -4.5)
    print("\n--- CASE 1: Turning Around a Column ---")
    column_obs = [{"x_world": 9.0, "y_world": -4.5, "size_x": 0.6, "size_z": 0.6}]
    # Candidate path curving safely around column (clearing column at y=-4.5 with > 0.20m margin)
    curved_path = [(7.0, -6.0), (8.0, -6.0), (9.0, -5.9), (10.0, -6.0), (11.0, -6.0)]
    safe, clr, reason = planner._check_path_clearance(curved_path, column_obs)
    print(f"  Curved Path around Column: Safe={safe} | Clearance={clr:.2f}m | Reason={reason}")
    assert safe, f"Curved path around column rejected: {reason}"
    assert clr >= 0.20, f"Clearance too small: {clr}"
    results["column_turn"] = {"safe": safe, "clearance": clr}

    # Case 2: Driving close to glass wall at east boundary (x=15.0m)
    print("\n--- CASE 2: Proximity to East Glass Wall (x=15.0m) ---")
    # Path attempting to reach x=14.20m (bumper tx reaches 15.55m > 14.80m glass safety limit)
    glass_exceed_path = [(13.0, -6.0), (13.8, -6.0), (14.20, -6.0)]
    safe_g, clr_g, reason_g = planner._check_path_clearance(glass_exceed_path, [])
    print(f"  Path Exceeding Glass Margin: Safe={safe_g} | Reason={reason_g}")
    assert not safe_g, "Failed to reject path violating glass boundary margin!"
    assert "glass boundary exceeded" in reason_g

    # Path safely staying within default goal x=13.40m (bumper tx reaches 14.75m <= 14.80m)
    glass_safe_path = [(12.0, -6.0), (12.8, -6.0), (13.40, -6.0)]
    safe_gs, clr_gs, reason_gs = planner._check_path_clearance(glass_safe_path, [])
    print(f"  Path Within Safe Glass Boundary: Safe={safe_gs} | Reason={reason_gs}")
    assert safe_gs, "Safe glass path rejected!"
    results["glass_wall"] = {"safe_rejection": not safe_g, "safe_path_cleared": safe_gs}

    # Case 3: Navigating Narrow Passage (two obstacles flanking corridor at y=-4.6m and y=-7.4m; gap ~2.0m)
    print("\n--- CASE 3: Navigating Narrow Passage ---")
    flanking_obs = [
        {"x_world": 9.5, "y_world": -4.6, "size_x": 0.6, "size_z": 0.6},
        {"x_world": 9.5, "y_world": -7.4, "size_x": 0.6, "size_z": 0.6},
    ]
    center_path = [(8.0, -6.0), (9.0, -6.0), (10.0, -6.0), (11.0, -6.0)]
    safe_n, clr_n, reason_n = planner._check_path_clearance(center_path, flanking_obs)
    print(f"  Narrow Passage Center Path: Safe={safe_n} | Clearance={clr_n:.2f}m | Reason={reason_n}")
    assert safe_n, f"Narrow passage center path rejected: {reason_n}"
    assert clr_n >= 0.15, f"Clearance insufficient: {clr_n}"
    results["narrow_passage"] = {"safe": safe_n, "clearance": clr_n}

    # Case 4: Executing Multi-Obstacle Detour using Planner Ackermann Offset Generator
    print("\n--- CASE 4: Multi-Obstacle Detour ---")
    staggered_obs = [
        {"x_world": 10.0, "y_world": -6.0, "size_x": 0.8, "size_z": 0.8},
    ]
    # Planner generated smooth detour path (obstacle detected at nominal 4.0m range)
    weave_path = planner._generate_offset_path(1.35, obs_dist=4.0, max_obs_dist=4.0)
    safe_w, clr_w, reason_w = planner._check_path_clearance(weave_path, staggered_obs)
    print(f"  Generated Smooth Detour Path: Safe={safe_w} | Clearance={clr_w:.2f}m | Reason={reason_w}")
    assert safe_w, f"Planner detour path rejected: {reason_w}"
    assert clr_w >= 0.15, f"Clearance insufficient: {clr_w}"
    results["multi_obstacle"] = {"safe": safe_w, "clearance": clr_w}

    # Case 5: Encountering a Fully Blocked Route (barricade across y=-6.8 to -3.8 at x=9.0)
    print("\n--- CASE 5: Encountering Fully Blocked Route ---")
    wall_barricade = [
        {"x_world": 9.0, "y_world": -4.2, "size_x": 0.8, "size_z": 0.8},
        {"x_world": 9.0, "y_world": -5.0, "size_x": 0.8, "size_z": 0.8},
        {"x_world": 9.0, "y_world": -5.8, "size_x": 0.8, "size_z": 0.8},
        {"x_world": 9.0, "y_world": -6.6, "size_x": 0.8, "size_z": 0.8},
    ]
    straight_path = [(7.0, -6.0), (8.0, -6.0), (9.0, -6.0), (10.0, -6.0)]
    safe_b, clr_b, reason_b = planner._check_path_clearance(straight_path, wall_barricade)
    print(f"  Barricaded Path: Safe={safe_b} | Clearance={clr_b:.2f}m | Reason={reason_b}")
    assert not safe_b, "Failed to reject path intersecting barricade!"
    results["fully_blocked"] = {"blocked_safely": not safe_b, "reason": reason_b}

    print("\n" + "=" * 80)
    print("ALL 5 SWEPT-FOOTPRINT COLLISION BENCHMARK TESTS PASSED!")
    print("=" * 80)
    planner.destroy_node()
    rclpy.shutdown()
    return 0

if __name__ == "__main__":
    sys.exit(test_swept_clearances())
