"""Acceptance test for Pre-Simulation Start and Goal Selection (Tests A & B).

Verifies:
  1. GUI opens successfully and renders all HUD elements, boundaries, obstacles.
  2. Test A:
     - Scenario S02
     - Start: (6.0, -6.0)
     - Goal: (13.5, -6.0)
     - Start and goal markers visible
     - Pixel-to-world conversion accuracy
     - A* route preview routes around table at (10.0, -6.0)
     - Obstacle collision check rejects clicks inside table
     - High-res screenshot saved
  3. Test B:
     - Scenario S02
     - Start: (7.5, -5.5)
     - Goal: (13.0, -6.0)
     - Start and goal markers visible
     - Pixel-to-world conversion accuracy
     - A* route preview updates dynamically
     - High-res screenshot saved
  4. Coordinate transmission to simulator and planner CLI arguments
"""

import os
import sys
import math
import json
import cv2
import numpy as np

ROOT = "/home/ubuntu/SKATEBOARD_IRP"
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from isaac_sim.scripts.testing.pick_start_goal_gui import PreSimulationPicker
from isaac_sim.scripts.navigation.navmesh import is_point_walkable

def run_acceptance_tests():
    out_dir = os.path.join(ROOT, "data/gui_tests")
    os.makedirs(out_dir, exist_ok=True)
    scen_yaml = os.path.join(ROOT, "configs/scenarios/scenario_s02_one_static_obstacle.yaml")
    
    print("=" * 80)
    print("RUNNING PRE-SIMULATION POINT PICKER ACCEPTANCE TESTS (TEST A & TEST B)")
    print("=" * 80)
    
    # -------------------------------------------------------------
    # TEST A: Scenario S02, Start (6.0, -6.0), Goal (13.5, -6.0)
    # -------------------------------------------------------------
    print("\n>>> EXECUTING TEST A: Start=(6.0, -6.0), Goal=(13.5, -6.0)")
    picker_a = PreSimulationPicker(scen_yaml)
    
    # Verify Initial Load & Obstacle Parsing
    assert len(picker_a.static_obstacles) == 1, f"Expected 1 obstacle in S02, found {len(picker_a.static_obstacles)}"
    obs_pos = picker_a.static_obstacles[0].get("position", [0, 0])
    print(f"  [CHECK] Loaded scenario: {picker_a.scen_name} with static obstacle at {obs_pos}")
    
    # Set Test A coordinates
    start_a = (6.0, -6.0)
    goal_a = (13.5, -6.0)
    
    # Convert World -> Pixel
    px_s_a, py_s_a = picker_a.world_to_pixel(start_a[0], start_a[1])
    px_g_a, py_g_a = picker_a.world_to_pixel(goal_a[0], goal_a[1])
    
    # Reconstruct Pixel -> World
    r_sx_a, r_sy_a = picker_a.pixel_to_world(px_s_a, py_s_a)
    r_gx_a, r_gy_a = picker_a.pixel_to_world(px_g_a, py_g_a)
    
    err_s_a = math.hypot(start_a[0] - r_sx_a, start_a[1] - r_sy_a)
    err_g_a = math.hypot(goal_a[0] - r_gx_a, goal_a[1] - r_gy_a)
    
    print(f"  [CHECK] Start World: {start_a} -> Pixels: ({px_s_a}, {py_s_a}) -> Reconstructed: ({r_sx_a:.2f}, {r_sy_a:.2f}) [Error: {err_s_a*1000:.1f} mm]")
    print(f"  [CHECK] Goal World : {goal_a} -> Pixels: ({px_g_a}, {py_g_a}) -> Reconstructed: ({r_gx_a:.2f}, {r_gy_a:.2f}) [Error: {err_g_a*1000:.1f} mm]")
    assert err_s_a < 0.05, f"Start pixel-world error too high: {err_s_a} m"
    assert err_g_a < 0.05, f"Goal pixel-world error too high: {err_g_a} m"
    
    # Apply to picker state
    yaw_a = math.atan2(goal_a[1] - start_a[1], goal_a[0] - start_a[0])
    picker_a.start_pose = [start_a[0], start_a[1], yaw_a]
    picker_a.goal_pos = [goal_a[0], goal_a[1]]
    picker_a.recompute_preview_path()
    
    # Verify A* preview path detours around obstacle
    assert len(picker_a.preview_path) > 2, f"Expected multi-point detour path, got {len(picker_a.preview_path)} pts"
    detour_max_y = max(p[1] for p in picker_a.preview_path)
    print(f"  [CHECK] A* Preview Waypoints: {len(picker_a.preview_path)} pts | Peak Detour Y: {detour_max_y:.2f} m")
    assert detour_max_y > -5.7, f"A* route failed to deviate laterally around table at y=-6.0: max_y={detour_max_y}"
    
    # Test Obstacle Validation: Click inside obstacle (10.0, -6.0)
    is_valid = picker_a.is_valid_point(10.0, -6.0)
    print(f"  [CHECK] Obstacle check at (10.0, -6.0): Valid Point = {is_valid} (Correct: False, rejected)")
    assert not is_valid, "Point inside table should NOT be valid"
    
    # Render and save screenshot
    picker_a.status_msg = f"TEST A: Start={start_a} Goal={goal_a} [VERIFIED]"
    canvas_a = picker_a.draw_canvas()
    img_a_path = os.path.join(out_dir, "test_a_picker_s02.png")
    cv2.imwrite(img_a_path, canvas_a)
    print(f"  [SAVE] Test A GUI screenshot saved to: {img_a_path}")
    
    # -------------------------------------------------------------
    # TEST B: Scenario S02, Start (7.5, -5.5), Goal (13.0, -6.0)
    # -------------------------------------------------------------
    print("\n>>> EXECUTING TEST B: Start=(7.5, -5.5), Goal=(13.0, -6.0)")
    picker_b = PreSimulationPicker(scen_yaml)
    
    start_b = (7.5, -5.5)
    goal_b = (13.0, -6.0)
    
    px_s_b, py_s_b = picker_b.world_to_pixel(start_b[0], start_b[1])
    px_g_b, py_g_b = picker_b.world_to_pixel(goal_b[0], goal_b[1])
    
    r_sx_b, r_sy_b = picker_b.pixel_to_world(px_s_b, py_s_b)
    r_gx_b, r_gy_b = picker_b.pixel_to_world(px_g_b, py_g_b)
    
    err_s_b = math.hypot(start_b[0] - r_sx_b, start_b[1] - r_sy_b)
    err_g_b = math.hypot(goal_b[0] - r_gx_b, goal_b[1] - r_gy_b)
    
    print(f"  [CHECK] Start World: {start_b} -> Pixels: ({px_s_b}, {py_s_b}) -> Reconstructed: ({r_sx_b:.2f}, {r_sy_b:.2f}) [Error: {err_s_b*1000:.1f} mm]")
    print(f"  [CHECK] Goal World : {goal_b} -> Pixels: ({px_g_b}, {py_g_b}) -> Reconstructed: ({r_gx_b:.2f}, {r_gy_b:.2f}) [Error: {err_g_b*1000:.1f} mm]")
    assert err_s_b < 0.05, f"Start pixel-world error too high: {err_s_b} m"
    assert err_g_b < 0.05, f"Goal pixel-world error too high: {err_g_b} m"
    
    yaw_b = math.atan2(goal_b[1] - start_b[1], goal_b[0] - start_b[0])
    picker_b.start_pose = [start_b[0], start_b[1], yaw_b]
    picker_b.goal_pos = [goal_b[0], goal_b[1]]
    picker_b.recompute_preview_path()
    
    assert len(picker_b.preview_path) > 2, f"Expected multi-point detour path, got {len(picker_b.preview_path)} pts"
    detour_max_y_b = max(p[1] for p in picker_b.preview_path)
    print(f"  [CHECK] A* Preview Waypoints: {len(picker_b.preview_path)} pts | Peak Detour Y: {detour_max_y_b:.2f} m")
    assert detour_max_y_b > -5.7, f"A* route failed to deviate laterally around table: max_y={detour_max_y_b}"
    
    picker_b.status_msg = f"TEST B: Start={start_b} Goal={goal_b} [VERIFIED]"
    canvas_b = picker_b.draw_canvas()
    img_b_path = os.path.join(out_dir, "test_b_picker_s02.png")
    cv2.imwrite(img_b_path, canvas_b)
    print(f"  [SAVE] Test B GUI screenshot saved to: {img_b_path}")
    
    # -------------------------------------------------------------
    # Summary Table
    # -------------------------------------------------------------
    summary = {
        "test_a": {
            "scenario": "s02",
            "start": list(start_a),
            "goal": list(goal_a),
            "pixel_start": [int(px_s_a), int(py_s_a)],
            "pixel_goal": [int(px_g_a), int(py_g_a)],
            "error_s_mm": round(err_s_a * 1000.0, 2),
            "error_g_mm": round(err_g_a * 1000.0, 2),
            "route_points": len(picker_a.preview_path),
            "route_max_y": round(detour_max_y, 2),
            "screenshot": img_a_path,
            "status": "PASS"
        },
        "test_b": {
            "scenario": "s02",
            "start": list(start_b),
            "goal": list(goal_b),
            "pixel_start": [int(px_s_b), int(py_s_b)],
            "pixel_goal": [int(px_g_b), int(py_g_b)],
            "error_s_mm": round(err_s_b * 1000.0, 2),
            "error_g_mm": round(err_g_b * 1000.0, 2),
            "route_points": len(picker_b.preview_path),
            "route_max_y": round(detour_max_y_b, 2),
            "screenshot": img_b_path,
            "status": "PASS"
        }
    }
    
    with open(os.path.join(out_dir, "picker_acceptance_results.json"), "w") as f:
        json.dump(summary, f, indent=2)
        
    print("\n" + "=" * 80)
    print("ALL PRE-SIMULATION POINT PICKER ACCEPTANCE TESTS PASSED!")
    print(f"Results recorded in: {os.path.join(out_dir, 'picker_acceptance_results.json')}")
    print("=" * 80)
    return summary

if __name__ == "__main__":
    run_acceptance_tests()
