"""Master Regression and Verification Runner for Final Runtime Reliability Audit (Nov 15).

Executes:
  1. No-LiDAR & Ground-Truth Isolation Audit (Rule 8)
  2. 3D Bounding-Box Geometry, Physical Extents & Semantic Priors
  3. Continuous Full-Footprint Swept Collision Checking (5 Cases)
  4. RGB-D Sensor Synchronization & Ring Buffer Matching
  5. Wheel-Slip Detection & Visual Odometry Clamping
  6. 2D-to-3D Association and Drop Resilience
Logs all evidence and metrics to results/gui_runtime_final_reliability_nov15/.
"""

import os
import subprocess
import sys
import time

ROOT = "/home/ubuntu/SKATEBOARD_IRP"
OUT_DIR = os.path.join(ROOT, "results/gui_runtime_final_reliability_nov15")
os.makedirs(OUT_DIR, exist_ok=True)

PYTHON_BIN = "/home/ubuntu/perception_venv/bin/python"
ENV_CMD = (
    "source /opt/ros/humble/setup.bash && "
    "export PYTHONPATH=/home/ubuntu/ros_overlay/opt/ros/humble/local/lib/python3.10/dist-packages:$PYTHONPATH && "
    "export LD_LIBRARY_PATH=/home/ubuntu/ros_overlay/opt/ros/humble/lib:/opt/ros/humble/lib:$LD_LIBRARY_PATH && "
)

TESTS = [
    ("LiDAR & Ground-Truth Isolation Audit", "isaac_sim/scripts/testing/audit_no_lidar_and_gt.py"),
    ("3D Bounding-Box Geometry & Semantic Priors", "isaac_sim/scripts/testing/test_3d_bounding_box_geometry.py"),
    ("Swept-Footprint Collision Checker (5 Cases)", "isaac_sim/scripts/testing/test_swept_footprint_clearance.py"),
    ("RGB-D Synchronization Benchmark", "isaac_sim/scripts/testing/test_rgbd_synchronization.py"),
    ("Wheel-Slip & Visual Odometry Clamping", "isaac_sim/scripts/testing/test_wheel_slip_detection.py"),
    ("2D-to-3D Association Resilience", "isaac_sim/scripts/testing/test_2d_3d_association.py"),
]

def main():
    print("=" * 80)
    print("STARTING FINAL RUNTIME RELIABILITY MASTER AUDIT")
    print(f"Output directory: {OUT_DIR}")
    print("=" * 80)

    summary_log = os.path.join(OUT_DIR, "master_audit_summary.log")
    test_results = {}

    with open(summary_log, "w") as s_file:
        s_file.write(f"FINAL RUNTIME RELIABILITY AUDIT EXECUTION LOG - {time.ctime()}\n")
        s_file.write("=" * 80 + "\n\n")

        for test_name, script_rel in TESTS:
            script_full = os.path.join(ROOT, script_rel)
            print(f"\n[RUNNING] {test_name} ({script_rel})...")
            cmd = f"{ENV_CMD} {PYTHON_BIN} {script_full}"
            t0 = time.time()
            res = subprocess.run(["bash", "-c", cmd], cwd=ROOT, capture_output=True, text=True)
            elapsed = time.time() - t0

            passed = (res.returncode == 0)
            status_str = "PASS" if passed else "FAIL"
            test_results[test_name] = status_str

            print(f"[{status_str}] {test_name} ({elapsed:.2f}s, exit code {res.returncode})")
            if res.stdout:
                print(res.stdout.strip())
            if res.stderr and not passed:
                print(f"[STDERR]\n{res.stderr.strip()}")

            # Write to summary log
            s_file.write(f"--- TEST: {test_name} [{status_str}] (Elapsed: {elapsed:.2f}s) ---\n")
            s_file.write(res.stdout + "\n")
            if res.stderr:
                s_file.write(f"[STDERR]\n{res.stderr}\n")
            s_file.write("\n" + "-" * 80 + "\n\n")

        # Summary Table
        print("\n" + "=" * 80)
        print("MASTER AUDIT COMPLIANCE MATRIX")
        print("=" * 80)
        s_file.write("=" * 80 + "\nMASTER AUDIT COMPLIANCE MATRIX\n" + "=" * 80 + "\n")
        all_passed = True
        for name, status in test_results.items():
            line = f"  {name:<50} -> [{status}]"
            print(line)
            s_file.write(line + "\n")
            if status != "PASS":
                all_passed = False

        print("=" * 80)
        final_str = "ALL RELIABILITY AUDIT TESTS PASSED!" if all_passed else "SOME AUDIT TESTS FAILED!"
        print(final_str)
        s_file.write("\n" + final_str + "\n")

    return 0 if all_passed else 1

if __name__ == "__main__":
    sys.exit(main())
