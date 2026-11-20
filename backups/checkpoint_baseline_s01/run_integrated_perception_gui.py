"""Single integrated perception test launcher (SKATEBOARD_IRP Phase 7).

ONE command starts the full RGB-D perception stack in the correct order:

  sim (isaacsim py3.11: scenario_runner --ros [+ S3 reopen])
    -> yolo_seg_node (perception_venv py3.10: RGB-only YOLO26n-seg)
    -> rgbd_fusion_node (perception_venv: mask-median depth -> objects_3d)
    -> validator (perception_venv: LIVE cv2 detection window + PNG + metrics)

Why separate processes: ROS Humble rclpy is py3.10-only and cannot import
inside isaacsim py3.11; YOLO/ultralytics lives in perception_venv. This file
imports NEITHER (stdlib + subprocess only) so it runs with system python3.

Windows (GUI mode): Isaac Sim viewport (WINDOW 1) + "YOLO RGB - scenario NN"
cv2 viewer (WINDOW 2) visible SIMULTANEOUSLY. Both stay open: sim holds via
scenario_runner --gui hold-open; viewer holds until its idle-timeout elapses.

Usage:
  python3 isaac_sim/scripts/testing/run_integrated_perception_gui.py --scenario 02 --gui
  python3 isaac_sim/scripts/testing/run_integrated_perception_gui.py --scenario 05 --gui --duration 60
  python3 isaac_sim/scripts/testing/run_integrated_perception_gui.py --scenario s3gui --gui
    (s3gui = scenarios/scenario_multi_person_vehicle_gui.yaml, 3 REAL S3 humans)
  python3 isaac_sim/scripts/testing/run_integrated_perception_gui.py --scenario 02 --headless --duration 20 --no-show

Ground-truth isolation: this launcher never subscribes /odom or GT poses;
GT json written by the sim side is eval-only (see docs/RGBD_MODEL_INTERFACE.md).
"""

import argparse
import atexit
import os
import signal
import subprocess
import sys
import time

ROOT = "/home/ubuntu/SKATEBOARD_IRP"
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

def restore_background_services():
    if os.path.exists("/tmp/pause_llama_for_sim"):
        try:
            os.remove("/tmp/pause_llama_for_sim")
            print("[CLEANUP] Resumed background LLM service.", flush=True)
        except Exception:
            pass

atexit.register(restore_background_services)

def get_gpu_processes():
    """Returns a list of (pid, name, used_mb) of active GPU processes."""
    procs = []
    try:
        out = subprocess.run(
            ["nvidia-smi", "--query-compute-apps=pid,process_name,used_memory", "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=3
        )
        for line in out.stdout.strip().split("\n"):
            line = line.strip()
            if not line:
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) >= 3:
                procs.append((parts[0], parts[1], int(parts[2])))
    except Exception:
        pass
    return procs


def ensure_gpu_memory(min_free_mb=8000, allow_pause_background=True):
    """Checks GPU memory before Isaac Sim and YOLO launch, reporting high-memory processes."""
    try:
        smi = subprocess.run(
            ["nvidia-smi", "--query-gpu=memory.free,memory.total", "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=3
        )
        parts = smi.stdout.strip().split("\n")[0].split(",")
        free_mb = int(parts[0].strip())
        total_mb = int(parts[1].strip())
        print(f"[VRAM CHECK] GPU Free: {free_mb} MiB / {total_mb} MiB (Required: >= {min_free_mb} MiB)", flush=True)

        if free_mb < min_free_mb:
            procs = get_gpu_processes()
            print(f"[VRAM WARNING] Available GPU memory ({free_mb} MiB) is below required ({min_free_mb} MiB).", flush=True)
            if procs:
                print("  Active GPU Compute Processes:", flush=True)
                for pid, name, used in procs:
                    print(f"    - PID {pid}: {name} ({used} MiB)", flush=True)

            llama_procs = [p for p in procs if "llama" in p[1].lower()]
            if llama_procs and allow_pause_background:
                print("[VRAM] Identified background LLM service holding VRAM. Pausing background service via /tmp/pause_llama_for_sim...", flush=True)
                subprocess.run(["bash", "-c", "touch /tmp/pause_llama_for_sim && pkill -15 -f 'docker run --name llama-server' || true"], timeout=5)
                for _ in range(15):
                    time.sleep(1.0)
                    smi_retry = subprocess.run(
                        ["nvidia-smi", "--query-gpu=memory.free", "--format=csv,noheader,nounits"],
                        capture_output=True, text=True, timeout=3
                    )
                    free_mb = int(smi_retry.stdout.strip().split("\n")[0])
                    if free_mb >= min_free_mb:
                        print(f"[VRAM SUCCESS] Successfully freed VRAM. Available: {free_mb} MiB. Proceeding.", flush=True)
                        break

            if free_mb < min_free_mb:
                print(f"[VRAM ERROR] GPU memory remains insufficient ({free_mb} MiB < {min_free_mb} MiB).", flush=True)
                print("  Please close competing GPU applications or pause background services, then retry.", flush=True)
                raise RuntimeError(f"Insufficient GPU memory: {free_mb} MiB available, {min_free_mb} MiB required.")
    except Exception as e:
        if isinstance(e, RuntimeError):
            raise
        print(f"[VRAM WARNING] Could not verify GPU memory: {e}", flush=True)
# Isaac Sim launcher: defaults to 6.2 (migration target); override for rollback:
#   ISAAC_PYTHON=/home/ubuntu/isaacsim/python.sh python3 run_integrated_perception_gui.py ...
ISAAC_PYTHON = os.environ.get("ISAAC_PYTHON", "/home/ubuntu/isaacsim-6.2.0/python.sh")
PERCEPTION_PYTHON = "/home/ubuntu/perception_venv/bin/python"
ROS_SETUP = "/opt/ros/humble/setup.bash"

SCEN_DIR = os.path.join(ROOT, "configs/scenarios")
SCEN_FILES = {
    # Autonomous 13-Scenario Matrix (S01 - S13)
    "s01": "scenario_s01_empty_lab.yaml",
    "s02": "scenario_s02_one_static_obstacle.yaml",
    "s03": "scenario_s03_multiple_static_obstacles.yaml",
    "s04": "scenario_s04_one_moving_pedestrian.yaml",
    "s05": "scenario_s05_two_moving_pedestrians.yaml",
    "s06": "scenario_s06_moving_ped_plus_furniture.yaml",
    "s07": "scenario_s07_unknown_unclassified_object.yaml",
    "s08": "scenario_s08_unexpected_obstacle.yaml",
    "s09": "scenario_s09_narrow_passage.yaml",
    "s10": "scenario_s10_multiple_obstacles_crossing.yaml",
    "s11": "scenario_s11_obstacle_leaves_route.yaml",
    "s12": "scenario_s12_no_safe_route.yaml",
    "s13": "scenario_s13_perception_failure.yaml",
    # Aliases
    "01": "scenario_s01_empty_lab.yaml",
    "02": "scenario_s02_one_static_obstacle.yaml",
    "03": "scenario_s03_multiple_static_obstacles.yaml",
    "04": "scenario_s04_one_moving_pedestrian.yaml",
    "05": "scenario_s05_two_moving_pedestrians.yaml",
    "06": "scenario_s06_moving_ped_plus_furniture.yaml",
    "07": "scenario_s07_unknown_unclassified_object.yaml",
    "08": "scenario_s08_unexpected_obstacle.yaml",
    "09": "scenario_s09_narrow_passage.yaml",
    "10": "scenario_s10_multiple_obstacles_crossing.yaml",
    "11": "scenario_s11_obstacle_leaves_route.yaml",
    "12": "scenario_s12_no_safe_route.yaml",
    "13": "scenario_s13_perception_failure.yaml",
    # Legacy perception validation mappings
    "p01": "scenario_01_stationary_person.yaml",
    "p02": "scenario_02_person_center.yaml",
    "s3gui": "../../scenarios/scenario_multi_person_vehicle_gui.yaml",
    # WASD manual-drive scenarios
    "wasd":  "scenario_wasd_manual_perception.yaml",
    "wasd2": "scenario_wasd_two_pedestrians.yaml",
    # Autonomous point-to-point general default
    "p2p":   "scenario_autonomous_point_to_point.yaml",
    "auto":  "scenario_autonomous_point_to_point.yaml",
}


try:
    from isaac_sim.scripts.scenarios.scenario_registry import resolve_scenario as registry_resolve
except Exception:
    registry_resolve = None


def resolve_scenario(spec):
    spec = str(spec).strip()
    if os.path.isfile(spec):
        return spec
    if registry_resolve:
        try:
            entry = registry_resolve(spec)
            if entry and os.path.isfile(entry.yaml_path):
                return entry.yaml_path
        except Exception:
            pass
    lower = spec.lower()
    if lower in SCEN_FILES:
        return os.path.normpath(os.path.join(SCEN_DIR, SCEN_FILES[lower]))
    key = spec.zfill(2) if spec.isdigit() else spec
    if key in SCEN_FILES:
        return os.path.normpath(os.path.join(SCEN_DIR, SCEN_FILES[key]))
    raise SystemExit(f"unknown scenario {spec!r}; use S01..S13, 01..13, p2p, wasd, or a YAML path")


SIM_ROSLIBS = "/home/ubuntu/isaacsim-6.2.0/exts/isaacsim.ros2.core/humble/lib"
PERCEPTION_ROSLIBS = "/home/ubuntu/ros_overlay/opt/ros/humble/lib:/opt/ros/humble/lib"
ROSPYTHON = "/home/ubuntu/ros_overlay/opt/ros/humble/local/lib/python3.10/dist-packages"


def sh_sim(cmd):
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = f"{SIM_ROSLIBS}:{env.get('LD_LIBRARY_PATH', '')}"
    return subprocess.Popen(
        cmd, cwd=ROOT, preexec_fn=os.setsid, env=env,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)


def sh(cmd):
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = f"{PERCEPTION_ROSLIBS}:{env.get('LD_LIBRARY_PATH', '')}"
    env["PYTHONPATH"] = f"{ROSPYTHON}:{env.get('PYTHONPATH', '')}"
    return subprocess.Popen(
        cmd, cwd=ROOT, preexec_fn=os.setsid, env=env,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)


def stream_prefix(proc, tag, max_lines=100):
    try:
        import select
        for _ in range(max_lines):
            if proc.stdout and select.select([proc.stdout], [], [], 0.0)[0]:
                line = proc.stdout.readline()
                if not line:
                    break
                print(f"[{tag}] {line.rstrip()}", flush=True)
            else:
                break
    except Exception:
        pass


def main():
    ap = argparse.ArgumentParser(description="Integrated perception launcher (Phase 7)")
    ap.add_argument("--scenario", default="02")
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--gui", action="store_true", default=False)
    ap.add_argument("--headless", action="store_true", default=False)
    ap.add_argument("--out", default=os.path.join(ROOT, "data/model_tests"))
    ap.add_argument("--no-show", dest="show", action="store_false", default=None,
                    help="do not open cv2 window (headless CI)")
    ap.add_argument("--sim-wait", type=float, default=90.0,
                    help="seconds to wait for sim+bridge before starting perception")
    ap.add_argument("--idle-timeout", type=float, default=3600.0,
                    help="validator window hold-open after stream idles (GUI stays open)")
    ap.add_argument("--wasd", action="store_true", default=False,
                    help="enable manual WASD vehicle driving (mutually exclusive with autonomous)")
    ap.add_argument("--auto", "--autonomous", dest="autonomous", action="store_true", default=None,
                    help="enable full autonomous point-to-point navigation (3-window architecture)")
    ap.add_argument("--monitor", action="store_true", default=False,
                    help="also launch live_perception_monitor.py (annotated RGB window)")
    ap.add_argument("--start", nargs="+", type=float, default=None,
                    help="allotted vehicle start pose [x, y, (yaw)]")
    ap.add_argument("--goal", nargs="+", type=float, default=None,
                    help="allotted destination goal pose [x, y]")
    ap.add_argument("--pick", "--pick-points", dest="pick", action="store_true", default=None,
                    help="open interactive 2D map to pick start and endpoint BEFORE simulation begins")
    ap.add_argument("--no-pick", dest="no_pick", action="store_true", default=False,
                    help="skip pre-simulation point picker")
    args = ap.parse_args()

    # Ensure DISPLAY is properly configured in environment before any GUI interaction
    if not os.environ.get("DISPLAY"):
        if os.path.exists("/tmp/.X11-unix/X1"):
            os.environ["DISPLAY"] = ":1"
        elif os.path.exists("/tmp/.X11-unix/X0"):
            os.environ["DISPLAY"] = ":0"

    has_display = bool(os.environ.get("DISPLAY"))
    if args.headless:
        gui = False
    elif args.gui:
        gui = True
    else:
        # Default behavior: GUI mode if DISPLAY environment variable is present, else headless
        gui = has_display

    # Determine mode: default to autonomous unless --wasd is explicitly given
    if args.autonomous is None:
        args.autonomous = not args.wasd

    scen_path = resolve_scenario(args.scenario)
    scen_id = os.path.splitext(os.path.basename(scen_path))[0]

    # Pre-simulation picker is enabled ONLY if --pick is explicitly passed
    should_pick = bool(args.pick and not args.no_pick and gui and not args.headless)

    # Pre-simulation interactive point allotment if requested or running interactive GUI
    if should_pick:
        try:
            try:
                from pick_start_goal_gui import pick_points_interactive
            except ImportError:
                from isaac_sim.scripts.testing.pick_start_goal_gui import pick_points_interactive
            allot_s, allot_g = pick_points_interactive(scen_path)
            if allot_s is None or allot_g is None:
                print("[LAUNCHER] Pre-simulation point picking was cancelled by user. Exiting.")
                sys.exit(0)
            args.start = allot_s
            args.goal = allot_g
            print(f"[LAUNCHER] Applying allotted Start={args.start} and Goal={args.goal} to simulation & planner.", flush=True)
        except Exception as e:
            print(f"[LAUNCHER] Failed to open pre-simulation picker: {e}", flush=True)

    if gui and "--duration" not in sys.argv:
        args.duration = 3600.0  # Interactive mode stays active until closed

    if args.show is not None:
        show = args.show
    else:
        show = gui and has_display

    os.makedirs(args.out, exist_ok=True)
    scen_out = os.path.join(args.out, scen_id)
    os.makedirs(scen_out, exist_ok=True)

    sim_cmd = [ISAAC_PYTHON, "isaac_sim/scripts/scenarios/scenario_runner.py",
               scen_path, "--ros", "--duration", str(args.duration)]
    sim_cmd.append("--gui" if gui else "--headless")
    if args.wasd:
        sim_cmd.append("--wasd")
    if args.start and len(args.start) >= 2:
        sim_cmd.extend(["--start"] + [str(x) for x in args.start])
    if args.goal and len(args.goal) >= 2:
        sim_cmd.extend(["--goal"] + [str(x) for x in args.goal])
    print(f"[LAUNCH 1/4] sim: {' '.join(sim_cmd)}", flush=True)
    try:
        subprocess.run(["bash", "-c", f"source {ROS_SETUP} >/dev/null 2>&1 && ros2 daemon stop"], timeout=5)
        subprocess.run(["bash", "-c", "pkill -9 -f 'rgbd_odometry_node|autonomous_planner_node|dynamic_tracker_node|yolo_seg_node|rgbd_fusion_node|live_perception_monitor|path_planning_visualizer' || true"], timeout=5)
    except Exception:
        pass

    ensure_gpu_memory()
    sim = sh_sim(sim_cmd)

    # Wait for the bridge: fixed warmup (app ready ~11s + stage build) then
    # HARD topic verification (Phase 7): poll ros2 topic list until the RGB +
    # depth topics exist. Never report success from process startup alone;
    # name the exact failed stage (SIM-BOOT / ROS-TOPICS / YOLO / FUSION / VIEWER).
    print("[WAIT] Waiting for SimulationApp + ROS bridge topics...", flush=True)
    deadline = time.time() + args.sim_wait + 60.0
    rgb_live = depth_live = False
    time.sleep(10.0)
    while time.time() < deadline:
        stream_prefix(sim, "sim")
        if sim.poll() is not None:
            print("[FAIL] STAGE=SIM-BOOT sim exited early during warmup; log tail above", flush=True)
            sys.exit(1)
        try:
            tl = subprocess.run(
                ["bash", "-c", f"source {ROS_SETUP} >/dev/null 2>&1 && ros2 topic list"],
                capture_output=True, text=True, timeout=10)
            topics = tl.stdout.split()
            rgb_live = "/camera/rgb/image_raw" in topics
            depth_live = "/camera/depth/image_raw" in topics
            if rgb_live and depth_live:
                print("[CHECK] STAGE=ROS-TOPICS topics discovered!", flush=True)
                break
        except Exception as e:
            pass
        time.sleep(2.0)
    print(f"[CHECK] STAGE=ROS-TOPICS rgb_live={rgb_live} depth_live={depth_live}", flush=True)
    if not (rgb_live and depth_live):
        print("[FAIL] STAGE=ROS-TOPICS camera topics absent (see STEP 6 diagnosis: "
              "syntheticdata writer-attach). YOLO/fusion/viewer would have no data; "
              "aborting instead of faking success. Stopping sim...", flush=True)
        stream_prefix(sim, "sim")
        try:
            os.killpg(os.getpgid(sim.pid), signal.SIGTERM)
            sim.wait(timeout=10)
        except Exception:
            try:
                os.killpg(os.getpgid(sim.pid), signal.SIGKILL)
            except Exception:
                pass
        sys.exit(2)

    env_sh = (f"source {ROS_SETUP} && "
              f"export PYTHONPATH={ROSPYTHON}:$PYTHONPATH && "
              f"export LD_LIBRARY_PATH={PERCEPTION_ROSLIBS}:$LD_LIBRARY_PATH && "
              f"export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && exec ")

    yolo_cmd = ["bash", "-c", env_sh + f"{PERCEPTION_PYTHON} "
                "ros2_ws/src/rgbd_perception/rgbd_perception/yolo_seg_node.py"]
    fusion_cmd = ["bash", "-c", env_sh + f"{PERCEPTION_PYTHON} "
                  "ros2_ws/src/rgbd_perception/rgbd_perception/rgbd_fusion_node.py"]
    print("[LAUNCH 2/5] yolo_seg_node (RGB-only, classes from configs/perception.yaml)", flush=True)
    yolo = sh(yolo_cmd)
    print("[LAUNCH 3/5] rgbd_fusion_node (mask-median depth, live K)", flush=True)
    fusion = sh(fusion_cmd)
    time.sleep(3.0)

    tracker = None
    odom_node = None
    planner = None
    visualizer = None

    if args.autonomous:
        # Launch Multi-Object Tracking & Dynamic Trajectory Prediction
        tracker_cmd = ["bash", "-c", env_sh + f"{PERCEPTION_PYTHON} "
                       "ros2_ws/src/rgbd_perception/rgbd_perception/dynamic_tracker_node.py"]
        print("[LAUNCH 4/5] dynamic_tracker_node (multi-object tracking & dynamic trajectory prediction)", flush=True)
        tracker = sh(tracker_cmd)

        # Launch Legitimate Dead-Reckoning Localisation (No GT leaks)
        odom_str = f"ros2_ws/src/rgbd_perception/rgbd_perception/rgbd_odometry_node.py --scenario {scen_path}"
        if args.start and len(args.start) >= 2:
            odom_str += f" --start {' '.join(str(x) for x in args.start)}"
        odom_cmd = ["bash", "-c", env_sh + f"{PERCEPTION_PYTHON} {odom_str}"]
        print("[LAUNCH 5/5] rgbd_odometry_node (dead-reckoning odometry, rule 8 GT isolation)", flush=True)
        odom_node = sh(odom_cmd)

        # Launch Autonomous Point-to-Point Planner (Global A* + Local Ackermann detour)
        planner_str = f"ros2_ws/src/rgbd_perception/rgbd_perception/autonomous_planner_node.py --scenario {scen_path}"
        if args.start and len(args.start) >= 2:
            planner_str += f" --start {' '.join(str(x) for x in args.start)}"
        if args.goal and len(args.goal) >= 2:
            planner_str += f" --goal {' '.join(str(x) for x in args.goal)}"
        planner_cmd = ["bash", "-c", env_sh + f"{PERCEPTION_PYTHON} {planner_str}"]
        print("[LAUNCH AUTONOMOUS PLANNER] autonomous_planner_node (point-to-point Ackermann navigation)", flush=True)
        planner = sh(planner_cmd)

        # Launch Window 3: Top-Down Path Planning & Trajectory Visualizer (Interactive GUI)
        vis_out = os.path.join(ROOT, "results")
        os.makedirs(vis_out, exist_ok=True)
        vis_str = f"isaac_sim/scripts/testing/path_planning_visualizer.py --scenario {scen_path} --out-dir {vis_out} --save-every 2.0"
        if not show:
            vis_str += " --no-show"
        if args.start and len(args.start) >= 2:
            vis_str += f" --start {' '.join(str(x) for x in args.start)}"
        if args.goal and len(args.goal) >= 2:
            vis_str += f" --goal {' '.join(str(x) for x in args.goal)}"
        vis_cmd = ["bash", "-c", env_sh + f"{PERCEPTION_PYTHON} {vis_str}"]
        print("[LAUNCH Window 3] path_planning_visualizer (interactive top-down map & GUI)", flush=True)
        visualizer = sh(vis_cmd)

    # In standalone evaluation mode (no wasd, no autonomous), validator logs metrics in background
    val = None
    if not args.wasd and not args.autonomous:
        val_show = show
        val_cmd = ["bash", "-c", env_sh + f"{PERCEPTION_PYTHON} "
                   "isaac_sim/scripts/testing/validate_yolo_scenarios_ext.py "
                   f"--scenario {scen_id} --out {scen_out} "
                   f"{'--show' if val_show else '--no-show'} --idle-timeout {args.idle_timeout}"]
        val = sh(val_cmd)

    # Dedicated Window 2: Live perception monitor
    monitor = None
    if show or args.monitor or args.autonomous or args.wasd:
        mon_out = os.path.join(args.out, "live_monitor")
        os.makedirs(mon_out, exist_ok=True)
        monitor_cmd = ["bash", "-c", env_sh + f"{PERCEPTION_PYTHON} "
                       f"isaac_sim/scripts/testing/live_perception_monitor.py "
                       f"--scenario {scen_id} --out-dir {mon_out} --save-every 2 "
                       f"{'--no-show' if not show else ''}"]
        print("[LAUNCH Window 2] live_perception_monitor (interactive YOLO26n-seg + 3D depth fusion)", flush=True)
        monitor = sh(monitor_cmd)

    if args.autonomous:
        print("[RUNNING] TRIPLE-WINDOW AUTONOMOUS NAVIGATION ACTIVE:", flush=True)
        print("  WINDOW 1: Isaac Sim 3D simulation (autonomous Ackermann vehicle navigating in lab)", flush=True)
        print("  WINDOW 2: Live YOLO26n-seg perception monitor (masks, boxes, depth, 3D coordinates)", flush=True)
        print("  WINDOW 3: Top-Down Path Planning & Trajectory Visualizer (global, planned, actual, obstacles, conflict cone)", flush=True)
    else:
        print("[RUNNING] DUAL-WINDOW MANUAL WASD SETUP ACTIVE:", flush=True)
        print("  WINDOW 1: Isaac Sim 3D simulation (click viewport for WASD controls)", flush=True)
        print("  WINDOW 2: Live YOLO26n-seg + 3D depth fusion perception window", flush=True)
        print("  Controls: W=Forward, S=Reverse, A=Steer Left, D=Steer Right, Space=Brake, R=Reset Steer", flush=True)
    print("  Press Ctrl+C in terminal to stop all processes.", flush=True)

    children = [("sim", sim), ("yolo", yolo), ("fusion", fusion)]
    if val is not None:
        children.append(("val", val))
    if tracker is not None:
        children.append(("tracker", tracker))
    if odom_node is not None:
        children.append(("odom", odom_node))
    if planner is not None:
        children.append(("planner", planner))
    if visualizer is not None:
        children.append(("visualizer", visualizer))
    if monitor is not None:
        children.append(("monitor", monitor))
    injected_failure = False
    fault_inject_time = time.time() + 8.0 if ("s13" in scen_id or "13" in scen_id) else None

    try:
        while True:
            if fault_inject_time and time.time() > fault_inject_time and not injected_failure:
                print("[FAULT-INJECTION] Scenario S13: Injecting perception failure (killing yolo + fusion)...", flush=True)
                try:
                    os.killpg(os.getpgid(yolo.pid), signal.SIGKILL)
                    os.killpg(os.getpgid(fusion.pid), signal.SIGKILL)
                except Exception:
                    pass
                injected_failure = True

            for tag, p in children:
                stream_prefix(p, tag)
            if sim.poll() is not None:
                print(f"[INFO] sim exited code={sim.returncode}; stopping perception tail...", flush=True)
                time.sleep(10.0)
                break
            if val is not None and val.poll() is not None:
                print(f"[INFO] validator exited code={val.returncode} (PNG+metrics saved to {scen_out})", flush=True)
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("[INFO] Ctrl+C — terminating all children...", flush=True)
    finally:
        for tag, p in children:
            if p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                except Exception:
                    try:
                        p.terminate()
                    except Exception:
                        pass
        try:
            subprocess.run(["bash", "-c", "pkill -9 -f 'rgbd_odometry_node|autonomous_planner_node|dynamic_tracker_node|yolo_seg_node|rgbd_fusion_node|live_perception_monitor|path_planning_visualizer' || true"], timeout=5)
        except Exception:
            pass
        print(f"[DONE] artifacts in {scen_out} (scenario PNG + ext metrics, GT eval-only)", flush=True)
        restore_background_services()


if __name__ == "__main__":
    main()
