#!/usr/bin/env python3
"""Master test runner for the Final Precision Audit suite."""

import os
import subprocess
import sys
import time

TEST_SCRIPTS = [
    ("test_signed_visual_odometry_and_confidence.py", "1. Signed Visual Odometry & Confidence Metric"),
    ("test_ros_detection3d_semantics_and_frames.py", "2. ROS Detection3D Semantics & Frame Transforms"),
    ("test_physical_steering_limits.py", "3. Physical Ackermann Steering Limits & Knuckle Margin"),
    ("test_offline_ground_truth_validation.py", "4. Offline Ground Truth Validation Across 6 Regimes"),
    ("test_live_camera_synchronization.py", "5. RGB-D Ring-Buffer Camera Synchronization"),
    ("test_swept_footprint_clearance.py", "6. Swept Footprint Clearance Verification"),
    ("audit_no_lidar_and_gt.py", "7. Architecture Isolation (No LiDAR & Zero Runtime GT)"),
]

ROOT_DIR = "/home/ubuntu/SKATEBOARD_IRP"
TEST_DIR = os.path.join(ROOT_DIR, "isaac_sim/scripts/testing")
LOG_FILE = os.path.join(ROOT_DIR, "results/gui_runtime_precision_audit_nov15/master_precision_audit.log")
PYTHON_EXEC = "/home/ubuntu/perception_venv/bin/python"

PERCEPTION_ROSLIBS = "/home/ubuntu/ros_overlay/opt/ros/humble/lib:/opt/ros/humble/lib"
ROSPYTHON = "/home/ubuntu/ros_overlay/opt/ros/humble/local/lib/python3.10/dist-packages"

def main():
    os.makedirs(os.path.dirname(LOG_FILE), exist_ok=True)
    all_passed = True
    results = []

    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = f"{PERCEPTION_ROSLIBS}:{env.get('LD_LIBRARY_PATH', '')}"
    env["PYTHONPATH"] = f"{ROSPYTHON}:{env.get('PYTHONPATH', '')}"

    header = "=" * 80 + "\n"
    header += "SKATEBOARD_IRP: FINAL PRECISION AUDIT MASTER REGRESSION SUITE\n"
    header += f"Date: {time.strftime('%Y-%m-%d %H:%M:%S')}\n"
    header += f"Interpreter: {PYTHON_EXEC}\n"
    header += "=" * 80 + "\n"
    print(header)

    with open(LOG_FILE, "w", encoding="utf-8") as log:
        log.write(header)

        for script_name, description in TEST_SCRIPTS:
            script_path = os.path.join(TEST_DIR, script_name)
            banner = f"\n>>> RUNNING TEST: {description} ({script_name})\n"
            print(banner, end="")
            log.write(banner)

            t0 = time.time()
            res = subprocess.run(
                [PYTHON_EXEC, script_path],
                cwd=ROOT_DIR,
                capture_output=True,
                text=True,
                env=env,
            )
            elapsed = time.time() - t0

            stdout_str = res.stdout.strip()
            stderr_str = res.stderr.strip()

            log.write(stdout_str + "\n")
            if stderr_str:
                log.write("STDERR:\n" + stderr_str + "\n")

            if res.returncode == 0:
                status_str = f"[PASS] {description} ({elapsed:.2f}s)"
                print(status_str)
                log.write(f"\n{status_str}\n")
                results.append((script_name, description, True, elapsed))
            else:
                status_str = f"[FAIL] {description} (exit code {res.returncode}, {elapsed:.2f}s)"
                print(status_str)
                if stderr_str:
                    print(stderr_str)
                log.write(f"\n{status_str}\n")
                results.append((script_name, description, False, elapsed))
                all_passed = False

        summary = "\n" + "=" * 80 + "\n"
        summary += "MASTER REGRESSION AUDIT SUMMARY:\n"
        summary += "=" * 80 + "\n"
        for script_name, description, passed, elapsed in results:
            mark = "[PASS]" if passed else "[FAIL]"
            summary += f"  {mark:6s} {description:<55} ({elapsed:.2f}s)\n"
        summary += "=" * 80 + "\n"
        summary += f"OVERALL STATUS: {'ALL TESTS PASSED (100%)' if all_passed else 'SOME TESTS FAILED'}\n"
        summary += "=" * 80 + "\n"

        print(summary)
        log.write(summary)

    return 0 if all_passed else 1

if __name__ == "__main__":
    sys.exit(main())
