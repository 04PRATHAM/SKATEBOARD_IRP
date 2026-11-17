"""Manual Driving + Live Perception Test Suite (Tests 2, 3, 4, 5).

Runs:
  1. Isaac Sim 6.2 with scenario + PhysX vehicle articulation + camera attached + ROS 2 bridge
  2. Keyboard / WASD driving control (viewport carb.input + stdin)
  3. YOLO26n-seg in perception_venv
  4. Mask-median depth fusion in perception_venv
  5. Perception metrics collector (rates, detections, 3D optical coords, vehicle TF poses, composite snapshots)

Supports:
  - Stage 2: Manual straight movement (W)
  - Stage 3: Manual steering (A / D)
  - Stage 4: Moving pedestrian interaction
  - Stage 5: Two pedestrians
  - Interactive mode (user drives via WASD) or Automated sequence verification

Usage:
  python3 isaac_sim/scripts/testing/run_manual_drive_perception_test.py --stage 2 --gui
  python3 isaac_sim/scripts/testing/run_manual_drive_perception_test.py --stage all --gui
  python3 isaac_sim/scripts/testing/run_manual_drive_perception_test.py --stage 2 --gui --interactive
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time

ROOT = "/home/ubuntu/SKATEBOARD_IRP"
OUT_BASE = os.path.join(ROOT, "data/manual_drive_test")
ISAAC_PYTHON = os.environ.get("ISAAC_PYTHON", "/home/ubuntu/isaacsim-6.2.0/python.sh")
PERCEPTION_PYTHON = "/home/ubuntu/perception_venv/bin/python"
ROS_SETUP = "/opt/ros/humble/setup.bash"
ROS_DOMAIN = os.environ.get("ROS_DOMAIN_ID", "85")

STAGE_CONFIGS = {
    "2": {
        "name": "stage_02_straight",
        "description": "Test 2: Manual straight movement (W) + approaching pedestrian",
        "scenario": "configs/scenarios/scenario_07_person_approaching.yaml",
        "duration": 14.0,
        "actions": [
            (1.5, "w"),   # forward +0.4 m/s
            (2.5, "w"),   # forward +0.8 m/s
            (7.0, " "),   # brake
        ]
    },
    "3": {
        "name": "stage_03_steering",
        "description": "Test 3: Manual steering (A/D) + moving pedestrian",
        "scenario": "configs/scenarios/scenario_07_person_approaching.yaml",
        "duration": 15.0,
        "actions": [
            (1.5, "w"),   # forward +0.4 m/s
            (2.5, "a"),   # steer left +0.15 rad
            (3.5, "a"),   # steer left +0.30 rad
            (5.5, "d"),   # counter-steer right
            (6.5, "d"),   # steer right
            (7.5, "r"),   # center steer
            (9.0, " "),   # brake
        ]
    },
    "4": {
        "name": "stage_04_interaction",
        "description": "Test 4: Moving pedestrian interaction (approach + vehicle forward)",
        "scenario": "configs/scenarios/scenario_07_person_approaching.yaml",
        "duration": 14.0,
        "actions": [
            (1.0, "w"),   # drive forward towards pedestrian
            (2.0, "w"),   # 0.8 m/s
            (6.0, " "),   # stop safely ahead of pedestrian
        ]
    },
    "5": {
        "name": "stage_05_two_pedestrians",
        "description": "Test 5: Two pedestrians in scene + manual driving",
        "scenario": "configs/scenarios/scenario_08_two_people.yaml",
        "duration": 15.0,
        "actions": [
            (1.5, "w"),   # forward
            (3.0, "a"),   # slight steer towards Person A
            (5.5, "d"),   # steer towards Person B
            (7.5, "r"),   # center
            (9.0, " "),   # brake
        ]
    }
}


def sh(cmd, env_extra=None, pipe_in=False):
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(ROS_DOMAIN)
    env["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    if env_extra:
        env.update(env_extra)
    return subprocess.Popen(
        cmd, cwd=ROOT, preexec_fn=os.setsid,
        stdin=subprocess.PIPE if pipe_in else None,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1, env=env)


def stream_prefix(proc, tag, max_lines=50):
    try:
        import select
        for _ in range(max_lines):
            if proc.stdout and select.select([proc.stdout], [], [], 0.0)[0]:
                line = proc.stdout.readline()
                if not line:
                    break
                if tag in ("collector", "yolo") or "WASD" in line or "vehicle x=" in line:
                    print(f"[{tag}] {line.rstrip()}", flush=True)
            else:
                break
    except Exception:
        pass


def run_stage(stage_key: str, gui: bool = True, interactive: bool = False):
    cfg = STAGE_CONFIGS[stage_key]
    out_dir = os.path.join(OUT_BASE, cfg["name"])
    os.makedirs(out_dir, exist_ok=True)

    print("\n" + "=" * 70)
    print(f" STARTING {cfg['description']}")
    print(f" Scenario: {cfg['scenario']} | Duration: {cfg['duration']}s | Output: {out_dir}")
    print("=" * 70, flush=True)

    # 1. Reset ROS 2 daemon
    subprocess.run(["bash", "-c", f"source {ROS_SETUP} && ros2 daemon stop >/dev/null 2>&1 || true"], timeout=5)

    # 2. Launch Isaac Sim with --wasd
    gui_flag = "--gui" if gui else "--headless"
    sim_cmd = [
        ISAAC_PYTHON,
        "isaac_sim/scripts/scenarios/scenario_runner.py",
        cfg["scenario"],
        "--ros",
        "--wasd",
        gui_flag,
        "--duration", str(cfg["duration"])
    ]
    print(f"[LAUNCH 1/4] Starting Isaac Sim 6.2 with WASD control: {' '.join(sim_cmd)}")
    sim_proc = sh(sim_cmd, pipe_in=True)

    # 3. Wait for camera topics to be live
    print("[WAIT] Waiting for ROS 2 camera topics...")
    deadline = time.time() + 120.0
    topics_live = False
    while time.time() < deadline:
        if sim_proc.poll() is not None:
            print("[FAIL] Simulator exited early during warmup!")
            return False
        tl = subprocess.run(
            ["bash", "-c", f"source {ROS_SETUP} && export ROS_DOMAIN_ID={ROS_DOMAIN} && ros2 topic list"],
            capture_output=True, text=True, timeout=10)
        t_list = tl.stdout.split()
        if "/camera/rgb/image_raw" in t_list and "/camera/depth/image_raw" in t_list:
            print("[READY] /camera/rgb/image_raw and /camera/depth/image_raw are LIVE!")
            topics_live = True
            break
        time.sleep(2.0)

    if not topics_live:
        print("[FAIL] Camera topics failed to publish.")
        try: os.killpg(os.getpgid(sim_proc.pid), signal.SIGTERM)
        except Exception: pass
        return False

    # 4. Launch YOLO and Fusion nodes
    env_sh = f"source {ROS_SETUP} && export ROS_DOMAIN_ID={ROS_DOMAIN} && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && exec "
    print("[LAUNCH 2/4] Starting YOLO26n-seg node...")
    yolo_proc = sh(["bash", "-c", env_sh + f"{PERCEPTION_PYTHON} ros2_ws/src/rgbd_perception/rgbd_perception/yolo_seg_node.py"])

    print("[LAUNCH 3/4] Starting depth fusion node...")
    fusion_proc = sh(["bash", "-c", env_sh + f"{PERCEPTION_PYTHON} ros2_ws/src/rgbd_perception/rgbd_perception/rgbd_fusion_node.py"])

    time.sleep(2.0)

    # 5. Launch Collector
    print(f"[LAUNCH 4/4] Starting metrics collector -> {out_dir}...")
    collector_cmd = [
        "bash", "-c",
        env_sh + f"{PERCEPTION_PYTHON} isaac_sim/scripts/testing/collect_perception_metrics.py --duration {cfg['duration']} --out {out_dir}"
    ]
    collector_proc = sh(collector_cmd)

    children = [("sim", sim_proc), ("yolo", yolo_proc), ("fusion", fusion_proc), ("collector", collector_proc)]

    # 6. Driving execution
    action_idx = 0
    actions = cfg["actions"]
    t0 = time.time()

    print("\n[ACTIVE] Driving experiment running...")
    if interactive:
        print("[INTERACTIVE] Focus the Isaac Sim viewport or type W/A/S/D into this terminal!")
    else:
        print(f"[AUTOMATED] Executing scripted WASD sequence: {actions}")

    try:
        while True:
            for tag, p in children:
                stream_prefix(p, tag)

            now = time.time() - t0

            # Execute scripted actions if not interactive
            if not interactive and action_idx < len(actions):
                act_time, act_key = actions[action_idx]
                if now >= act_time:
                    print(f"  >>> [DRIVE ACTION] t={now:.1f}s: Key '{act_key}' applied to vehicle", flush=True)
                    if sim_proc.stdin:
                        try:
                            sim_proc.stdin.write(act_key + "\n")
                            sim_proc.stdin.flush()
                        except Exception as e:
                            print(f"[WARN] stdin write failed: {e}")
                    action_idx += 1

            if collector_proc.poll() is not None:
                print(f"[INFO] Collector finished with code {collector_proc.returncode}")
                break
            if sim_proc.poll() is not None:
                print(f"[INFO] Simulator finished with code {sim_proc.returncode}")
                break

            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[INFO] Stage interrupted by user.")
    finally:
        for tag, p in children:
            if p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                    p.wait(timeout=5)
                except Exception:
                    try: os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                    except Exception: pass
        print(f"[CLEANUP] Stage {stage_key} completed.")

    # Load and print results
    res_path = os.path.join(out_dir, "metrics.json")
    if os.path.isfile(res_path):
        with open(res_path) as f:
            data = json.load(f)
        summary = data.get("summary", {})
        print("\n" + "=" * 60)
        print(f" STAGE {stage_key} RESULTS SUMMARY: {cfg['name']}")
        print("=" * 60)
        print("Counts:   ", summary.get("counts", {}))
        print("Rates (Hz):", summary.get("rates_hz", {}))
        pa = summary.get("pedestrian_analysis")
        if pa:
            print(f"Pedestrian 3D observations: {pa['count']}")
            print(f"Depth range: initial={pa['initial_depth_m']}m -> min={pa['min_depth_m']}m -> final={pa['final_depth_m']}m")
            print(f"Distance range: initial={pa['initial_distance_m']}m -> min={pa['min_distance_m']}m -> final={pa['final_distance_m']}m")
        else:
            print("[WARN] No pedestrian observations recorded.")
        print("=" * 60 + "\n")
        return True
    return False


def main():
    ap = argparse.ArgumentParser(description="Manual Driving Perception Test Runner")
    ap.add_argument("--stage", default="2", choices=["2", "3", "4", "5", "all"],
                    help="Stage to run: 2 (straight), 3 (steer), 4 (interaction), 5 (two peds), all")
    ap.add_argument("--gui", action="store_true", default=True, help="Run in GUI mode")
    ap.add_argument("--headless", action="store_true", default=False, help="Run headless")
    ap.add_argument("--interactive", action="store_true", default=False,
                    help="Manual interactive driving via keyboard")
    args = ap.parse_args()

    gui = args.gui and not args.headless

    if args.stage == "all":
        stages_to_run = ["2", "3", "4", "5"]
    else:
        stages_to_run = [args.stage]

    results = {}
    for s in stages_to_run:
        ok = run_stage(s, gui=gui, interactive=args.interactive)
        results[s] = ok
        time.sleep(3.0)

    print("\n" + "=" * 60)
    print(" ALL TEST STAGES COMPLETED")
    print("=" * 60)
    for s, ok in results.items():
        name = STAGE_CONFIGS[s]["name"]
        print(f"  Stage {s} ({name}): {'PASS' if ok else 'FAIL'}")
    print("=" * 60)


if __name__ == "__main__":
    main()
