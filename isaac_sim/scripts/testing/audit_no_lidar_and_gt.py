"""Audit script verifying zero LiDAR dependencies and zero runtime GT leakage."""

import os
import re
import sys

FILES_TO_AUDIT = [
    "isaac_sim/scripts/testing/run_integrated_perception_gui.py",
    "isaac_sim/scripts/scenarios/scenario_runner.py",
    "isaac_sim/scripts/bridge/ros2_bridge.py",
    "ros2_ws/src/rgbd_perception/rgbd_perception/yolo_seg_node.py",
    "ros2_ws/src/rgbd_perception/rgbd_perception/rgbd_fusion_node.py",
    "ros2_ws/src/rgbd_perception/rgbd_perception/dynamic_tracker_node.py",
    "ros2_ws/src/rgbd_perception/rgbd_perception/rgbd_odometry_node.py",
    "ros2_ws/src/rgbd_perception/rgbd_perception/autonomous_planner_node.py",
    "isaac_sim/scripts/testing/path_planning_visualizer.py",
    "isaac_sim/scripts/testing/live_perception_monitor.py",
]

ROOT = "/home/ubuntu/SKATEBOARD_IRP"

def audit_files():
    print("=" * 80)
    print("AUDITING RUNTIME STACK: LIDAR & GROUND TRUTH ISOLATION")
    print("=" * 80)

    lidar_keywords = ["laserscan", "pointcloud2", "pointcloud", "/scan", "rtx_lidar", "lidar"]
    gt_leak_keywords = ["/tf_static", "get_world_poses", "get_linear_velocity"]

    findings = []
    
    for rel_path in FILES_TO_AUDIT:
        full_path = os.path.join(ROOT, rel_path)
        if not os.path.isfile(full_path):
            findings.append((rel_path, "FILE_NOT_FOUND", "File does not exist"))
            continue

        with open(full_path, "r", encoding="utf-8") as f:
            lines = f.readlines()

        # Check for LiDAR usage
        for i, line in enumerate(lines, 1):
            line_clean = line.strip()
            if line_clean.startswith("#") or line_clean.startswith('"""') or line_clean.startswith("'''"):
                continue  # Skip comments/docstrings for functional check
            
            for kw in lidar_keywords:
                if re.search(r'\b' + re.escape(kw) + r'\b', line, re.IGNORECASE):
                    findings.append((rel_path, f"Line {i}", f"LiDAR keyword found: {line_clean}"))

            # Check for GT leak in runtime nodes (excluding offline evaluator or simulator spawner)
            if "node.py" in rel_path:
                for kw in gt_leak_keywords:
                    if kw in line:
                        findings.append((rel_path, f"Line {i}", f"Potential GT leak: {line_clean}"))

    print(f"Total files audited: {len(FILES_TO_AUDIT)}")
    if not findings:
        print("[SUCCESS] Zero LiDAR dependencies detected! Architecture is 100% RGB-D based.")
        print("[SUCCESS] Zero runtime Ground Truth leakage detected in navigation nodes!")
        return 0
    else:
        print(f"[WARNING] Found {len(findings)} potential flags:")
        for f in findings:
            print(f"  - {f[0]}:{f[1]} -> {f[2]}")
        return 1

if __name__ == "__main__":
    sys.exit(audit_files())
