#!/usr/bin/env python3
"""
SKATEBOARD_IRP — 3D Digital Twin World & Interactive Inspection Launcher.

Builds the full lab + kinematic vehicle + 24 inspection cameras through the
SHARED builder (isaac_sim/scripts/utils/world_setup.py) — the same world every
other runner and test uses. Optional --ros2 streams RGB/depth/CameraInfo/clock/TF.

Usage:
  /home/ubuntu/isaacsim/python.sh isaac_sim/scripts/run_fidelity_world.py --inspect
  /home/ubuntu/isaacsim/python.sh isaac_sim/scripts/run_fidelity_world.py --headless --duration 3.0
  /home/ubuntu/isaacsim/python.sh isaac_sim/scripts/run_fidelity_world.py --headless --ros2 --duration 300
"""

import sys
import argparse

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")


def parse_args():
    parser = argparse.ArgumentParser(description="SKATEBOARD_IRP 3D World & Vehicle Inspection Launcher")
    parser.add_argument("--inspect", action="store_true", default=True, help="Continuous interactive inspection mode (no auto shutdown)")
    parser.add_argument("--headless", action="store_true", help="Run in headless mode without GUI viewport")
    parser.add_argument("--duration", type=float, default=None, help="Optional duration in seconds (if specified, stops after N seconds)")
    parser.add_argument("--ros2", action="store_true", help="Enable Milestone 6 ROS 2 bridge (RGB/depth/CameraInfo/clock/TF streaming)")
    return parser.parse_args()


def main():
    args = parse_args()

    from isaacsim import SimulationApp
    simulation_app = SimulationApp({
        "headless": args.headless,
        "renderer": "RayTracedLighting",
    })

    print("==================================================================")
    print("  SKATEBOARD_IRP — 3D DIGITAL TWIN & LABORATORY WORLD LAUNCHER    ")
    print("==================================================================")

    from isaac_sim.scripts.utils.world_setup import build_world
    built = build_world(dynamic=False, with_cameras=True, with_ros=args.ros2)
    world, sim_cfg = built["world"], built["sim_cfg"]

    # Headless render products still need render steps to produce images.
    need_render = (not args.headless) or args.ros2

    if args.duration is not None:
        print(f"[INFO] Running in DURATION MODE for {args.duration} seconds...")
        max_steps = int(args.duration / sim_cfg["physics"]["dt"])
        step_count = 0
        while simulation_app.is_running() and step_count < max_steps:
            world.step(render=need_render)
            step_count += 1
        print("[INFO] Duration mode execution finished.")
    else:
        print("\n==================================================================")
        print(" [INSPECTION / DEVELOPMENT MODE ACTIVE]                           ")
        print(" Inspect vehicle, RGB-D camera post, and lab via the Camera menu  ")
        print(" (24 views). Close the window or press Ctrl+C when finished.      ")
        print(" Viewport → Camera → /World/Vehicle/Chassis/Sensors/RGBD_Camera/RGBD_Camera  (driver, moves)")
        print(" or → /World/Vehicle/Chassis/Driver_Seat_Camera/DriverSeat_Camera  (seat eye, moves)")
        print("==================================================================\n")

        # Continuous inspection mode — runs indefinitely until user closes window or Ctrl+C
        try:
            while simulation_app.is_running():
                world.step(render=need_render)
        except KeyboardInterrupt:
            print("[INFO] Interrupted by user", flush=True)
        finally:
            print("[INFO] Closing SimulationApp...")
            simulation_app.close()
            print("[INFO] Closed cleanly.", flush=True)
            return

    print("[INFO] Closing SimulationApp...")
    try:
        simulation_app.close()
    except Exception:
        pass
    print("[INFO] Closed cleanly.", flush=True)


if __name__ == "__main__":
    main()
