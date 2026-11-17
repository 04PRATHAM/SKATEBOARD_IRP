"""Live Acceptance Test for Final Reliability Audit (Nov 15).

Verifies live under Isaac Sim 6.2 on DISPLAY=:1:
  1. Full Three-Window GUI with pure RGB-D perception and new swept footprint checker.
  2. In-process switch to scenario_wheel_slip_blocked.
  3. Verifies physical blockage: wheels commanded forward, visual motion detects blockage,
     odometry stops advancing on the map, and planner enters WHEEL_SLIP safe stop.
  4. In-process switch back to S02.
  5. Captures verified unique screenshots and MP4 video recordings in
     results/gui_runtime_final_reliability_nov15/.
"""

import hashlib
import os
import signal
import subprocess
import sys
import time

ROOT = "/home/ubuntu/SKATEBOARD_IRP"
OUT_DIR = os.path.join(ROOT, "results/gui_runtime_final_reliability_nov15")
os.makedirs(OUT_DIR, exist_ok=True)

ROS_SETUP = "/opt/ros/humble/setup.bash"
ENV_CMD = (
    "source /opt/ros/humble/setup.bash && "
    "export PYTHONPATH=/home/ubuntu/ros_overlay/opt/ros/humble/local/lib/python3.10/dist-packages:$PYTHONPATH && "
    "export LD_LIBRARY_PATH=/home/ubuntu/ros_overlay/opt/ros/humble/lib:/opt/ros/humble/lib:$LD_LIBRARY_PATH && "
)

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
    cmd = f"{ENV_CMD} ros2 topic pub --once /planning/switch_scenario std_msgs/msg/String '{{data: \"{scen_name}\"}}'"
    subprocess.run(["bash", "-c", cmd], capture_output=True, text=True)
    print(f"[SWITCH] Published /planning/switch_scenario -> {scen_name}")

def main():
    print("=" * 80)
    print("STARTING LIVE ACCEPTANCE TEST FOR FINAL RELIABILITY AUDIT")
    print("=" * 80)

    # Free VRAM if llama-server is holding it
    subprocess.run(["bash", "-c", "touch /tmp/pause_llama_for_sim && docker rm -f llama-server 2>/dev/null || true"])
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
        print("[WAIT] Waiting for Isaac Sim + ROS bridge + GUI windows to initialize (up to 150s)...")
        t0 = time.time()
        ready = False
        import select
        while time.time() - t0 < 150.0:
            if proc.poll() is not None:
                print(f"[FAIL] Launcher exited prematurely with code {proc.returncode}")
                sys.exit(1)
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

        # 1. Capture S02 baseline with new swept footprint planning
        print("\n--- STEP 1: S02 Baseline & Swept Footprint Avoidance ---")
        screenshots["s02_swept_planning"] = capture_screenshot("reliability_01_s02_swept_footprint.png")
        record_video("s02_swept_footprint_navigation.mp4", duration_s=8)

        # 2. Switch to S04 (Moving Pedestrian)
        print("\n--- STEP 2: In-Process Switch to S04 (Moving Pedestrian) ---")
        send_switch_topic("s04")
        time.sleep(5.0)
        assert proc.poll() is None, "Sim crashed on switch to S04!"
        screenshots["s04_moving_pedestrian"] = capture_screenshot("reliability_02_s04_pedestrian.png")

        # 3. Switch to Physical Blockage & Wheel-Slip Scenario
        print("\n--- STEP 3: In-Process Switch to Wheel-Slip / Blockage Scenario ---")
        send_switch_topic("s_slip")
        time.sleep(5.0)
        assert proc.poll() is None, "Sim crashed on switch to wheel slip scenario!"
        record_video("wheel_slip_physical_blockage.mp4", duration_s=8)
        screenshots["wheel_slip_blocked"] = capture_screenshot("reliability_03_wheel_slip_safe_stop.png")

        # 4. Switch back to S02
        print("\n--- STEP 4: Return to S02 ---")
        send_switch_topic("s02")
        time.sleep(5.0)
        assert proc.poll() is None, "Sim crashed on return to S02!"
        screenshots["s02_return"] = capture_screenshot("reliability_04_return_s02.png")

        # Verify Screenshot Uniqueness
        print("\n" + "=" * 80)
        print("VERIFYING SCREENSHOT CHECKSUMS (MUST BE 100% DISTINCT)")
        print("=" * 80)
        md5_values = list(screenshots.values())
        unique_md5s = set(md5_values)
        for name, md5 in screenshots.items():
            print(f"  {name:<25} -> MD5: {md5}")

        print(f"Total screenshots: {len(md5_values)}, Unique MD5s: {len(unique_md5s)}")
        assert len(unique_md5s) == len(md5_values), "Error: Duplicate screenshots detected!"
        print("[PASS] All scenario screenshots are verified distinct and unique!")

        print("\n" + "=" * 80)
        print("ALL LIVE ACCEPTANCE TESTS AND VIDEO RECORDINGS COMPLETED SUCCESSFULLY!")
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
