"""Stationary Vehicle + Moving Pedestrian Live Perception Test.

Runs:
  1. Isaac Sim 6.2 with scenario_07_person_approaching (stationary vehicle, approaching pedestrian)
  2. yolo_seg_node (Ultralytics YOLO26n-seg in perception_venv)
  3. rgbd_fusion_node (mask-median depth backprojection in perception_venv)
  4. Live monitoring node (records rates, detections, 3D objects, depth progression, saves composite images)

Produces:
  - data/stationary_moving_test/metrics.json
  - data/stationary_moving_test/detection_*.png
  - Console report with topic Hz, 3D trajectory tracking, and verification verdict.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
import numpy as np

ROOT = "/home/ubuntu/SKATEBOARD_IRP"
OUT_DIR = os.path.join(ROOT, "data/stationary_moving_test")
ISAAC_PYTHON = os.environ.get("ISAAC_PYTHON", "/home/ubuntu/isaacsim-6.2.0/python.sh")
PERCEPTION_PYTHON = "/home/ubuntu/perception_venv/bin/python"
ROS_SETUP = "/opt/ros/humble/setup.bash"
ROS_DOMAIN = os.environ.get("ROS_DOMAIN_ID", "85")

os.makedirs(OUT_DIR, exist_ok=True)

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--scenario", default="configs/scenarios/scenario_07_person_approaching.yaml")
    p.add_argument("--duration", type=float, default=25.0)
    p.add_argument("--gui", action="store_true", default=False)
    p.add_argument("--headless", action="store_true", default=False)
    return p.parse_args()

def sh(cmd, env_extra=None):
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(ROS_DOMAIN)
    env["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    if env_extra:
        env.update(env_extra)
    return subprocess.Popen(
        cmd, cwd=ROOT, preexec_fn=os.setsid,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1, env=env)

def stream_prefix(proc, tag, max_lines=50):
    try:
        import select
        for _ in range(max_lines):
            if proc.stdout and select.select([proc.stdout], [], [], 0.0)[0]:
                line = proc.stdout.readline()
                if not line:
                    break
                if tag in ("collector", "yolo") or "STAGE" in line or "Error" in line:
                    print(f"[{tag}] {line.rstrip()}", flush=True)
            else:
                break
    except Exception:
        pass

def main():
    args = parse_args()
    print(f"=== TEST 1: Stationary Vehicle + Moving Pedestrian Live Perception ===")
    print(f"Scenario: {args.scenario}")
    print(f"Duration: {args.duration}s | ROS_DOMAIN_ID: {ROS_DOMAIN}")

    gui_flag = "--gui" if args.gui and not args.headless else "--headless"
    sim_cmd = [
        ISAAC_PYTHON,
        "isaac_sim/scripts/scenarios/scenario_runner.py",
        args.scenario,
        "--ros",
        gui_flag,
        "--duration", str(args.duration)
    ]

    print(f"[LAUNCH 1/3] Starting Isaac Sim 6.2...")
    sim_proc = sh(sim_cmd)

    # Monitor warmup until ROS topics appear
    print("[WAIT] Waiting for ROS 2 bridge topics to appear...", flush=True)
    topics_ready = False
    deadline = time.time() + 120.0
    while time.time() < deadline:
        stream_prefix(sim_proc, "sim")
        if sim_proc.poll() is not None:
            print("[FAIL] Simulator exited prematurely during warmup!", flush=True)
            sys.exit(1)
        # Check topic list
        tl = subprocess.run(
            ["bash", "-c", f"source {ROS_SETUP} && export ROS_DOMAIN_ID={ROS_DOMAIN} && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && ros2 topic list"],
            capture_output=True, text=True, timeout=10)
        out = tl.stdout.split()
        if "/camera/rgb/image_raw" in out and "/camera/depth/image_raw" in out:
            print("[READY] /camera/rgb/image_raw and /camera/depth/image_raw are LIVE!", flush=True)
            topics_ready = True
            break
        time.sleep(2.0)

    if not topics_ready:
        print("[FAIL] Camera topics failed to publish within deadline.")
        try:
            os.killpg(os.getpgid(sim_proc.pid), signal.SIGTERM)
        except Exception:
            pass
        sys.exit(1)

    # Launch yolo_seg_node in perception_venv
    env_sh = f"source {ROS_SETUP} && export ROS_DOMAIN_ID={ROS_DOMAIN} && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && exec "
    yolo_cmd = ["bash", "-c", env_sh + f"{PERCEPTION_PYTHON} ros2_ws/src/rgbd_perception/rgbd_perception/yolo_seg_node.py"]
    print("[LAUNCH 2/3] Starting yolo_seg_node...")
    yolo_proc = sh(yolo_cmd)

    # Launch rgbd_fusion_node in perception_venv
    fusion_cmd = ["bash", "-c", env_sh + f"{PERCEPTION_PYTHON} ros2_ws/src/rgbd_perception/rgbd_perception/rgbd_fusion_node.py"]
    print("[LAUNCH 3/3] Starting rgbd_fusion_node...")
    fusion_proc = sh(fusion_cmd)

    time.sleep(2.0)

    # Launch listener / recorder node in perception_venv
    collector_cmd = [
        "bash", "-c",
        env_sh + f"{PERCEPTION_PYTHON} isaac_sim/scripts/testing/collect_perception_metrics.py --duration {args.duration} --out {OUT_DIR}"
    ]
    print("[MONITOR] Running perception metrics collector...")
    collector_proc = sh(collector_cmd)

    children = [("sim", sim_proc), ("yolo", yolo_proc), ("fusion", fusion_proc), ("collector", collector_proc)]

    try:
        while True:
            for tag, p in children:
                stream_prefix(p, tag)
            if collector_proc.poll() is not None:
                print(f"[INFO] Collector finished with code {collector_proc.returncode}")
                break
            if sim_proc.poll() is not None:
                print(f"[INFO] Sim finished with code {sim_proc.returncode}")
                time.sleep(3.0)
                break
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[INFO] Test interrupted by user.")
    finally:
        for tag, p in children:
            if p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                    p.wait(timeout=5)
                except Exception:
                    try:
                        os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                    except Exception:
                        pass
        print("[CLEANUP] All processes terminated.")

if __name__ == "__main__":
    main()
