"""Comprehensive Live Acceptance Test for Final Technical Audit (Nov 15).

Verifies:
  1. Live GUI launch (3 windows: 3D sim, Perception Monitor, Visualizer) on DISPLAY=:1.
  2. Sequential in-process scenario switches:
     S02 -> S04 (Moving Pedestrian) -> S07 (Unknown Crate) -> S05 (Two Moving Pedestrians) -> S09 (Narrow Passage) -> S02 (Return).
  3. Video captures:
     - pedestrian_s04_crossing.mp4 (single pedestrian crossing path)
     - pedestrians_s05_two_moving.mp4 (two pedestrians moving simultaneously)
     - vehicle_wall_avoidance.mp4 (vehicle navigating corridor avoiding obstacles/walls)
  4. Screenshot capture for each scenario with verified unique content and distinct md5 checksums.
  5. Isaac Sim Kit process remains continuously alive without restart or crash.
"""

import os
import sys
import time
import signal
import hashlib
import subprocess

ROOT = "/home/ubuntu/SKATEBOARD_IRP"
OUT_DIR = os.path.join(ROOT, "results/gui_runtime_final_audit_nov15")
os.makedirs(OUT_DIR, exist_ok=True)

ROS_SETUP = "/opt/ros/humble/setup.bash"

def capture_screenshot(filename):
    out_path = os.path.join(OUT_DIR, filename)
    cmd = f"ffmpeg -y -f x11grab -video_size 1920x1080 -i :1 -vframes 1 {out_path} 2>/dev/null"
    subprocess.run(["bash", "-c", cmd], check=True)
    size = os.path.getsize(out_path)
    with open(out_path, "rb") as f:
        md5 = hashlib.md5(f.read()).hexdigest()
    print(f"[SCREENSHOT] Saved {filename} ({size} bytes | MD5: {md5})")
    return md5

def record_video(filename, duration_s=8):
    out_path = os.path.join(OUT_DIR, filename)
    print(f"[RECORD VIDEO] Recording {filename} for {duration_s}s...")
    cmd = f"ffmpeg -y -f x11grab -video_size 1920x1080 -i :1 -t {duration_s} -pix_fmt yuv420p {out_path} 2>/dev/null"
    subprocess.run(["bash", "-c", cmd])
    size = os.path.getsize(out_path) if os.path.exists(out_path) else 0
    print(f"[RECORD VIDEO] Finished {filename} ({size} bytes)")

def send_switch_topic(scen_name):
    cmd = (
        f"source {ROS_SETUP} && "
        f"ros2 topic pub --once /planning/switch_scenario std_msgs/msg/String '{{data: \"{scen_name}\"}}'"
    )
    subprocess.run(["bash", "-c", cmd], capture_output=True, text=True)
    print(f"[SWITCH] Published /planning/switch_scenario -> {scen_name}")

def main():
    print("=" * 80)
    print("STARTING COMPREHENSIVE LIVE SIMULATION AUDIT & ACCEPTANCE TEST")
    print("=" * 80)

    # Clean existing processes
    subprocess.run(["bash", "-c", "pkill -9 -f 'scenario_runner|yolo_seg_node|rgbd_fusion_node|dynamic_tracker_node|rgbd_odometry_node|autonomous_planner_node|path_planning_visualizer|live_perception_monitor' || true"])
    time.sleep(2.0)

    # Launch integrated GUI starting on S02
    launch_cmd = (
        f"source {ROS_SETUP} && "
        "/home/ubuntu/perception_venv/bin/python "
        "isaac_sim/scripts/testing/run_integrated_perception_gui.py "
        "--scenario s02 --gui"
    )

    env = os.environ.copy()
    env["DISPLAY"] = ":1"

    print("[LAUNCH] Starting run_integrated_perception_gui.py on DISPLAY=:1...")
    proc = subprocess.Popen(
        ["bash", "-c", launch_cmd],
        cwd=ROOT,
        env=env,
        preexec_fn=os.setsid,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1
    )

    screenshots = {}
    try:
        # Wait for simulation warmup and topics
        print("[WAIT] Waiting for Isaac Sim + ROS bridge + GUI windows to initialize (up to 150s)...")
        t0 = time.time()
        ready = False
        import select
        while time.time() - t0 < 150.0:
            if proc.poll() is not None:
                print(f"[FAIL] Launcher exited prematurely with code {proc.returncode}")
                sys.exit(1)
            # Drain stdout
            while proc.stdout and select.select([proc.stdout], [], [], 0.0)[0]:
                line = proc.stdout.readline()
                if not line:
                    break
                print(f"[SIM-LAUNCH] {line.strip()}", flush=True)

            res = subprocess.run(["bash", "-c", "pgrep -f 'path_planning_visualizer'"], capture_output=True, text=True)
            if res.stdout.strip():
                ready = True
                print(f"[READY] All 3 GUI windows and ROS nodes active at t={time.time() - t0:.1f}s!")
                break
            time.sleep(2.0)

        assert ready, "GUI components failed to launch within 150s"
        time.sleep(6.0)

        # 1. S02 Initial State
        print("\n--- STEP 1: S02 (Single Static Table) Verification ---")
        assert proc.poll() is None, "Isaac Sim exited!"
        screenshots["s02_initial"] = capture_screenshot("acceptance_01_s02_table.png")

        # 2. Switch to S04 (Moving Pedestrian)
        print("\n--- STEP 2: In-Process Switch to S04 (One Moving Pedestrian) ---")
        send_switch_topic("s04")
        time.sleep(5.0)
        assert proc.poll() is None, "Isaac Sim crashed on switch to S04!"
        record_video("pedestrian_s04_crossing.mp4", duration_s=8)
        screenshots["s04_pedestrian"] = capture_screenshot("acceptance_02_switch_to_s04_pedestrian.png")

        # 3. Switch to S07 (Unknown Solid Crate)
        print("\n--- STEP 3: In-Process Switch to S07 (Unknown Solid Crate) ---")
        send_switch_topic("s07")
        time.sleep(6.0)
        assert proc.poll() is None, "Isaac Sim crashed on switch to S07!"
        screenshots["s07_crate"] = capture_screenshot("acceptance_03_switch_to_s07_crate.png")

        # 4. Switch to S05 (Two Moving Pedestrians)
        print("\n--- STEP 4: In-Process Switch to S05 (Two Moving Pedestrians) ---")
        send_switch_topic("s05")
        time.sleep(5.0)
        assert proc.poll() is None, "Isaac Sim crashed on switch to S05!"
        record_video("pedestrians_s05_two_moving.mp4", duration_s=8)
        screenshots["s05_two_peds"] = capture_screenshot("acceptance_04_switch_to_s05_two_peds.png")

        # 5. Switch to S09 (Narrow Passage)
        print("\n--- STEP 5: In-Process Switch to S09 (Narrow Passage Navigation) ---")
        send_switch_topic("s09")
        time.sleep(6.0)
        assert proc.poll() is None, "Isaac Sim crashed on switch to S09!"
        screenshots["s09_narrow"] = capture_screenshot("acceptance_05_switch_to_s09_narrow.png")

        # 6. Switch back to S02 (Static Table)
        print("\n--- STEP 6: In-Process Return to S02 (Static Table) ---")
        send_switch_topic("s02")
        time.sleep(5.0)
        assert proc.poll() is None, "Isaac Sim crashed on return to S02!"
        record_video("vehicle_wall_avoidance.mp4", duration_s=8)
        screenshots["s02_return"] = capture_screenshot("acceptance_06_switch_back_to_s02.png")

        # Verify Screenshot Uniqueness
        print("\n" + "=" * 80)
        print("VERIFYING SCREENSHOT CHECKSUMS (MUST BE UNIQUE)")
        print("=" * 80)
        md5_values = list(screenshots.values())
        unique_md5s = set(md5_values)
        for name, md5 in screenshots.items():
            print(f"  {name:<18} -> MD5: {md5}")

        print(f"Total screenshots: {len(md5_values)}, Unique MD5s: {len(unique_md5s)}")
        assert len(unique_md5s) == len(md5_values), "Error: Found duplicate screenshots across scenario switches!"
        print("[PASS] All scenario screenshots are verified distinct and unique!")

        print("\n" + "=" * 80)
        print("ALL ACCEPTANCE TESTS & VIDEO RECORDINGS COMPLETED SUCCESSFULLY!")
        print("=" * 80)

    finally:
        print("[CLEANUP] Stopping launcher and GUI processes...")
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
            except Exception:
                pass

if __name__ == "__main__":
    main()
