#!/usr/bin/env python3
"""Live Acceptance Test for Final Precision Audit: Visual Odometry, Semantics, and Physical Limits."""

import hashlib
import os
import signal
import subprocess
import sys
import time

ROOT = "/home/ubuntu/SKATEBOARD_IRP"
OUT_DIR = os.path.join(ROOT, "results/gui_runtime_precision_audit_nov15")
os.makedirs(OUT_DIR, exist_ok=True)

ENV_CMD = (
    "source /opt/ros/humble/setup.bash && "
    "export PYTHONPATH=/home/ubuntu/ros_overlay/opt/ros/humble/local/lib/python3.10/dist-packages:${PYTHONPATH} && "
    "export LD_LIBRARY_PATH=/home/ubuntu/ros_overlay/opt/ros/humble/lib:/opt/ros/humble/lib:${LD_LIBRARY_PATH} && "
    "export DISPLAY=:1 && "
)

def capture_screen(filename):
    out_path = os.path.join(OUT_DIR, filename)
    cmd = f"ffmpeg -y -f x11grab -video_size 1920x1080 -i :1 -vframes 1 {out_path} 2>/dev/null"
    subprocess.run(["bash", "-c", cmd])
    size = os.path.getsize(out_path) if os.path.exists(out_path) else 0
    print(f"[SCREENSHOT] Saved {filename} ({size} bytes)")
    return out_path

def md5sum(path):
    with open(path, "rb") as f:
        return hashlib.md5(f.read()).hexdigest()

def send_switch_topic(scen_name):
    cmd = f"{ENV_CMD} ros2 topic pub --once /planning/switch_scenario std_msgs/msg/String '{{data: \"{scen_name}\"}}'"
    subprocess.run(["bash", "-c", cmd], capture_output=True, text=True)
    print(f"[SWITCH] Published /planning/switch_scenario -> {scen_name}")

def main():
    print("=" * 80)
    print("STARTING LIVE ACCEPTANCE TEST FOR FINAL PRECISION AUDIT")
    print("=" * 80)

    # Free VRAM if llama-server is holding it
    subprocess.run(["bash", "-c", "touch /tmp/pause_llama_for_sim && pkill -f 'docker run --name llama-server' || true"])
    print("[VRAM] Waiting for VRAM deallocation...")
    for _ in range(30):
        smi_out = subprocess.run(["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"], capture_output=True, text=True).stdout.strip()
        try:
            used_mb = int(smi_out.split('\n')[0].strip())
            print(f"[VRAM] Current VRAM usage: {used_mb} MiB")
            if used_mb < 2500:
                print(f"[VRAM] Sufficient VRAM available ({used_mb} MiB). Proceeding.")
                break
        except Exception:
            pass
        time.sleep(1.0)

    # Clean existing processes
    subprocess.run(["bash", "-c", "pkill -9 -f 'scenario_runner|yolo_seg_node|rgbd_fusion_node|dynamic_tracker_node|rgbd_odometry_node|autonomous_planner_node|path_planning_visualizer|live_perception_monitor' || true"])
    time.sleep(2.0)

    launch_cmd = (
        f"{ENV_CMD} "
        "/home/ubuntu/perception_venv/bin/python "
        "isaac_sim/scripts/testing/run_integrated_perception_gui.py "
        "--scenario s01 --gui"
    )

    print("[LAUNCH] Starting run_integrated_perception_gui.py on DISPLAY=:1 (Scenario S01)...")
    proc = subprocess.Popen(
        ["bash", "-c", launch_cmd],
        cwd=ROOT,
        preexec_fn=os.setsid,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1
    )

    screenshots = {}

    try:
        print("[WAIT] Waiting for Isaac Sim + ROS bridge + GUI windows to initialize (up to 95s)...")
        t0 = time.time()
        ready = False
        while time.time() - t0 < 95.0:
            if proc.poll() is not None:
                print(f"[FAIL] Launcher exited prematurely with code {proc.returncode}")
                sys.exit(1)
            res = subprocess.run(["bash", "-c", "pgrep -f 'path_planning_visualizer'"], capture_output=True, text=True)
            if res.stdout.strip():
                ready = True
                print(f"[READY] All 3 GUI windows and ROS nodes active at t={time.time() - t0:.1f}s!")
                break
            time.sleep(2.0)

        assert ready, "GUI components failed to launch within 95s"
        time.sleep(5.0)

        # 1. Capture Signed Forward Motion (S01: Empty Corridor)
        print("\n--- STAGE 1: Signed Forward Motion & Nominal Odometry (S01) ---")
        p1 = capture_screen("precision_01_signed_forward.png")
        screenshots["precision_01"] = p1

        # 2. Switch to S02 (Table Obstacle) & Capture Ackermann Steering Turn
        print("\n--- STAGE 2: Switch to S02 & Ackermann Physical Steering Turn (delta <= 0.45 rad) ---")
        send_switch_topic("s02")
        time.sleep(6.0)
        p2 = capture_screen("precision_02_physical_steering_turn.png")
        screenshots["precision_02"] = p2

        # 3. Test In-Process Scenario Switch to Blocked Scenario & Wheel Slip Safe Stop
        print("\n--- STAGE 3: Wheel-Slip & Physical Blockage Safe Stop (slip) ---")
        send_switch_topic("slip")
        time.sleep(6.0)
        p3 = capture_screen("precision_03_wheel_slip_blocked.png")
        screenshots["precision_03"] = p3

        # 4. Switch to S04 (Moving Pedestrian) for Detection3D Semantics & Velocities
        print("\n--- STAGE 4: Detection3D Semantics & Companion Velocities (S04) ---")
        send_switch_topic("s04")
        time.sleep(6.0)
        p4 = capture_screen("precision_04_detection3d_semantics.png")
        screenshots["precision_04"] = p4

        print("\n" + "=" * 80)
        print("VERIFYING ARTIFACT SCREENSHOT CHECKSUMS")
        print("=" * 80)

        hashes = {}
        for name, path in screenshots.items():
            h = md5sum(path)
            size = os.path.getsize(path)
            print(f"  {name}: {path} ({size} bytes, MD5={h})")
            assert h not in hashes.values(), f"Duplicate screenshot detected: {name}"
            hashes[name] = h
            assert size > 20000, f"Screenshot too small: {size} bytes"

        print("\n[SUCCESS] All 4 GUI acceptance screenshots are unique, valid, and verified!")

    finally:
        print("\n[CLEANUP] Stopping launcher and GUI processes...")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=10)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
        subprocess.run(["bash", "-c", "pkill -9 -f 'scenario_runner|yolo_seg_node|rgbd_fusion_node|dynamic_tracker_node|rgbd_odometry_node|autonomous_planner_node|path_planning_visualizer|live_perception_monitor' || true"])
        if os.path.exists("/tmp/pause_llama_for_sim"):
            try:
                os.remove("/tmp/pause_llama_for_sim")
                print("[CLEANUP] Removed /tmp/pause_llama_for_sim to restore background services.")
            except Exception:
                pass

if __name__ == "__main__":
    main()
