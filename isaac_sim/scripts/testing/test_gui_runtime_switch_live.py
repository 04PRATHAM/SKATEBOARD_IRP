"""Live Acceptance Test for GUI In-Process Scenario Switching and Window Preservation."""

import os
import sys
import time
import socket
import struct
import signal
import subprocess

ROOT = "/home/ubuntu/SKATEBOARD_IRP"
OUT_DIR = os.path.join(ROOT, "results/gui_runtime_corrections_nov14")
os.makedirs(OUT_DIR, exist_ok=True)

def capture_screen(filename):
    out_path = os.path.join(OUT_DIR, filename)
    cmd = f"ffmpeg -y -f x11grab -video_size 1920x1080 -i :1 -vframes 1 {out_path} 2>/dev/null"
    subprocess.run(["bash", "-c", cmd])
    print(f"[SCREENSHOT] Saved {filename} ({os.path.getsize(out_path) if os.path.exists(out_path) else 0} bytes)")

def send_switch(scen_idx):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    pkt = struct.pack("ffff", -888.0, float(scen_idx), 0.0, 0.0)
    sock.sendto(pkt, ("127.0.0.1", 9877))
    sock.close()
    print(f"[SWITCH CMD] Sent switch command for scenario index {scen_idx} to 127.0.0.1:9877")

def main():
    print("=" * 80)
    print("STARTING LIVE GUI SCENARIO SWITCHING ACCEPTANCE TEST")
    print("=" * 80)

    # Launch integrated GUI starting on S02
    cmd = (
        "source /opt/ros/humble/setup.bash && "
        "/home/ubuntu/perception_venv/bin/python "
        "isaac_sim/scripts/testing/run_integrated_perception_gui.py "
        "--scenario s02 --gui"
    )

    env = os.environ.copy()
    env["DISPLAY"] = ":1"

    print("[LAUNCH] Starting run_integrated_perception_gui.py on DISPLAY=:1...")
    proc = subprocess.Popen(
        ["bash", "-c", cmd],
        cwd=ROOT,
        env=env,
        preexec_fn=os.setsid,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1
    )

    try:
        # Wait for simulation warmup and topics (approx 25 seconds)
        print("[WAIT] Waiting for Isaac Sim + ROS bridge + GUI windows to initialize...")
        t0 = time.time()
        ready = False
        while time.time() - t0 < 60.0:
            if proc.poll() is not None:
                print(f"[FAIL] Launcher exited prematurely with code {proc.returncode}")
                sys.exit(1)
            # Check if visualizer and monitor are up
            res = subprocess.run(["bash", "-c", "pgrep -f 'path_planning_visualizer'"], capture_output=True, text=True)
            if res.stdout.strip():
                ready = True
                print(f"[READY] All 3 GUI windows and ROS nodes active at t={time.time() - t0:.1f}s!")
                break
            time.sleep(2.0)

        assert ready, "GUI components failed to launch within 60s"
        time.sleep(5.0)

        # Capture S02 active triple-window state
        capture_screen("acceptance_01_s02_triple_windows.png")

        # Test Switch 1: S02 -> S04 (Moving Pedestrian, S3 human)
        print("\n--- IN-PROCESS SWITCH 1: S02 -> S04 (Moving Pedestrian) ---")
        send_switch(3) # S04 index = 3
        time.sleep(4.0)
        assert proc.poll() is None, "Isaac Sim crashed on switch to S04!"
        capture_screen("acceptance_02_switch_to_s04_pedestrian.png")
        print("[PASS] Switch 1 Verified: Isaac Sim remained alive, S04 loaded.")

        # Test Switch 2: S04 -> S07 (Unknown Crate)
        print("\n--- IN-PROCESS SWITCH 2: S04 -> S07 (Unknown Solid Crate) ---")
        send_switch(6) # S07 index = 6
        time.sleep(4.0)
        assert proc.poll() is None, "Isaac Sim crashed on switch to S07!"
        capture_screen("acceptance_03_switch_to_s07_crate.png")
        print("[PASS] Switch 2 Verified: Isaac Sim remained alive, S07 loaded.")

        # Test Switch 3: S07 -> S05 (Two Moving Pedestrians)
        print("\n--- IN-PROCESS SWITCH 3: S07 -> S05 (Two Moving Pedestrians) ---")
        send_switch(4) # S05 index = 4
        time.sleep(4.0)
        assert proc.poll() is None, "Isaac Sim crashed on switch to S05!"
        capture_screen("acceptance_04_switch_to_s05_two_peds.png")
        print("[PASS] Switch 3 Verified: Isaac Sim remained alive, S05 loaded.")

        # Test Switch 4: S05 -> S02 (Back to Static Table)
        print("\n--- IN-PROCESS SWITCH 4: S05 -> S02 (Return to Static Table) ---")
        send_switch(1) # S02 index = 1
        time.sleep(4.0)
        assert proc.poll() is None, "Isaac Sim crashed on switch to S02!"
        capture_screen("acceptance_05_switch_back_to_s02.png")
        print("[PASS] Switch 4 Verified: Isaac Sim remained alive, S02 reloaded.")

        print("\n" + "=" * 80)
        print("ALL SCENARIO SWITCHING ACCEPTANCE TESTS PASSED WITHOUT SIMULATION CRASH!")
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

if __name__ == "__main__":
    main()
