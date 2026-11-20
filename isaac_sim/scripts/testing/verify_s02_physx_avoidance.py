#!/usr/bin/env python3
"""Live PhysX closed-loop obstacle avoidance verification for Scenario S02 in Isaac Sim 6.2.

Executes:
1. Launches Isaac Sim 6.2 with Scenario S02 (table obstacle at x=10.0, y=-6.0) and ROS 2 UDP bridge.
2. Initializes AutonomousPlannerNode with exact 2D SAT OBB collision checking.
3. Streams PhysX vehicle ground-truth poses at 60 Hz via port 9879.
4. Computes Ackermann-compliant detour commands (steering and speed) at 20 Hz and sends them via port 9876.
5. Continuously records true physical OBB clearance against the table obstacle using SAT.
6. Asserts:
   - Vehicle avoids table with minimum physical clearance >= 0.15 m margin.
   - Zero physical collisions throughout maneuver.
   - Vehicle successfully reaches destination goal (13.5, -6.0).
"""

import os
import sys
import math
import time
import socket
import struct
import argparse
import subprocess
import signal

ROOT = "/home/ubuntu/SKATEBOARD_IRP"
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
if f"{ROOT}/ros2_ws/src/rgbd_perception" not in sys.path:
    sys.path.insert(0, f"{ROOT}/ros2_ws/src/rgbd_perception")

ISAAC_PYTHON = "/home/ubuntu/isaacsim-6.2.0/python.sh"
PERCEPTION_PYTHON = "/home/ubuntu/perception_venv/bin/python"

from rgbd_perception.vehicle_footprint import sat_obb_clearance, VEH_HALF_LENGTH, VEH_HALF_WIDTH


def run_live_test(gui: bool = False, duration: float = 45.0):
    print("=" * 85)
    print("LIVE PHYSX VERIFICATION: SCENARIO S02 PHYSICAL AVOIDANCE IN ISAAC SIM 6.2")
    print(f"Mode: {'GUI (Viewport visible)' if gui else 'Headless'}")
    print("=" * 85)

    # Clean up lingering port 9876 / 9878 / 9879 processes
    os.system("pkill -9 -f scenario_runner.py 2>/dev/null; fuser -k 9876/udp 9878/udp 9879/udp 2>/dev/null")
    time.sleep(0.5)

    # Listen on port 9879 for ground truth pose from Isaac Sim
    pose_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    pose_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    pose_sock.bind(("127.0.0.1", 9879))
    pose_sock.settimeout(20.0)

    # UDP socket to send commands to Isaac Sim on port 9876
    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd_dest = ("127.0.0.1", 9876)

    # Launch Isaac Sim scenario_runner with S02 and ROS bridge enabled
    log_file = open("/tmp/live_physx_sim.log", "w")
    sim_mode_flag = "--gui" if gui else "--headless"
    sim_cmd = f"{ISAAC_PYTHON} {ROOT}/isaac_sim/scripts/scenarios/scenario_runner.py --scenario s02 {sim_mode_flag} --ros --duration {duration}"
    print(f"Launching Isaac Sim: {sim_cmd}")
    sim_proc = subprocess.Popen(
        ["bash", "-c", sim_cmd],
        cwd=ROOT,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid
    )

    # Wait for first pose from Isaac Sim
    print("Waiting for Isaac Sim to initialize and publish pose on UDP 9879...")
    t0 = time.time()
    init_pose = None
    try:
        while time.time() - t0 < 50.0:
            try:
                data, _ = pose_sock.recvfrom(32)
                if len(data) == 12:
                    gx, gy, gyaw = struct.unpack("fff", data)
                    init_pose = (float(gx), float(gy), float(gyaw))
                    print(f"Isaac Sim connected! Initial vehicle pose: x={gx:.3f}, y={gy:.3f}, yaw={math.degrees(gyaw):.1f}°")
                    break
            except socket.timeout:
                continue
    except Exception as e:
        print(f"Error waiting for Isaac Sim: {e}")

    if init_pose is None:
        print("FAILED: Did not receive pose from Isaac Sim within 50s!")
        try:
            os.killpg(os.getpgid(sim_proc.pid), signal.SIGKILL)
        except Exception:
            pass
        return False

    pose_sock.setblocking(False)

    # Initialize AutonomousPlannerNode in this process
    import rclpy
    from rgbd_perception.autonomous_planner_node import AutonomousPlannerNode
    from rgbd_perception.config import load_config
    rclpy.init()
    cfg = load_config()
    scen_data = {
        "vehicle_start_pose": [6.0, -6.0, 0.35, 0.0],
        "vehicle_destination_pose": [13.5, -6.0, 0.0]
    }
    planner = AutonomousPlannerNode(cfg, scenario_data=scen_data)
    planner.odom_received = True
    planner.perception_received = True

    # Obstacle table at (10.0, -6.0), length=1.4m (X), width=0.8m (Y)
    table_x, table_y = 10.0, -6.0
    table_hl, table_hw = 0.70, 0.40

    min_clearance = float("inf")
    collision_detected = False
    goal_reached = False

    print("\nExecuting live PhysX closed-loop avoidance navigation...", flush=True)
    start_time = time.time()
    last_print = 0.0

    while time.time() - start_time < (duration * 2.0):
        # Read latest pose from Isaac Sim (draining queue to get freshest telemetry)
        vx, vy, vyaw = None, None, None
        try:
            while True:
                data, _ = pose_sock.recvfrom(32)
                if len(data) == 12:
                    gx, gy, gyaw = struct.unpack("fff", data)
                    vx, vy, vyaw = float(gx), float(gy), float(gyaw)
        except (socket.timeout, BlockingIOError, OSError):
            pass

        if vx is None:
            time.sleep(0.02)
            continue

        # Check true physical OBB clearance against table using unified footprint
        clr = sat_obb_clearance(vx, vy, vyaw, VEH_HALF_LENGTH, VEH_HALF_WIDTH, table_x, table_y, 0.0, table_hl, table_hw)
        if clr < min_clearance:
            min_clearance = clr

        if clr <= 0.0:
            print(f"!!! COLLISION DETECTED: veh=({vx:.2f}, {vy:.2f}) yaw={math.degrees(vyaw):.1f}° clr={clr:.3f}m !!!", flush=True)
            collision_detected = True
            break

        # Check goal arrival (Euclidean or post-detour corridor region)
        dist_to_goal = math.hypot(vx - 13.5, vy - (-6.0))
        if dist_to_goal <= 0.45 or (vx >= 13.20 and abs(vy - (-6.0)) <= 0.45):
            print(f"\n>>> PHYSICAL GOAL REACHED! Final Pose: ({vx:.3f}, {vy:.3f}), Goal Dist: {dist_to_goal:.3f}m <<<", flush=True)
            goal_reached = True
            # Send stop command
            cmd_sock.sendto(struct.pack("ff", 0.0, 0.0), cmd_dest)
            break

        # Perception & planning update
        planner.veh_x = vx
        planner.veh_y = vy
        planner.veh_yaw = vyaw

        # Compute table relative coordinates in camera optical frame
        dx_w = table_x - vx
        dy_w = table_y - vy
        cos_y = math.cos(vyaw)
        sin_y = math.sin(vyaw)
        x_veh = cos_y * dx_w + sin_y * dy_w
        y_veh = -sin_y * dx_w + cos_y * dy_w

        cos_p = 0.98877
        z_cam = (x_veh - 0.90) / cos_p
        x_cam = -y_veh

        if z_cam >= 0.5 and abs(x_cam) / max(0.1, z_cam) < 1.0:
            planner.obstacles = [{
                "track_id": 1,
                "class_name": "table",
                "is_dynamic": False,
                "speed": 0.0,
                "size_x": table_hw * 2.0,
                "size_y": 0.75,
                "size_z": table_hl * 2.0,
                "x_cam": x_cam,
                "y_cam": 0.0,
                "z_cam": z_cam,
                "x_veh": x_veh,
                "y_veh": y_veh,
                "z_veh": 0.0,
                "x_world": table_x,
                "y_world": table_y,
                "dist": math.hypot(x_veh, y_veh)
            }]
        else:
            planner.obstacles = []

        steer, speed = planner._plan_motion()
        planner.veh_speed = speed

        # Send command to Isaac Sim PhysX articulation
        cmd_sock.sendto(struct.pack("ff", float(steer), float(speed)), cmd_dest)

        now = time.time()
        if now - last_print >= 1.0:
            last_print = now
            print(f"t={now-start_time:.1f}s | pos=({vx:.2f}, {vy:.2f}) | yaw={math.degrees(vyaw):+.1f}° | "
                  f"steer={math.degrees(steer):+.1f}° | spd={speed:.2f}m/s | clr={clr:.2f}m | state={planner.state}", flush=True)

        time.sleep(0.05)

    # Terminate sim cleanly
    try:
        os.killpg(os.getpgid(sim_proc.pid), signal.SIGINT)
        time.sleep(1.0)
        os.killpg(os.getpgid(sim_proc.pid), signal.SIGKILL)
    except Exception:
        pass

    log_file.close()
    pose_sock.close()
    cmd_sock.close()
    rclpy.shutdown()

    print("\n" + "=" * 85)
    print("LIVE PHYSX VERIFICATION REPORT:")
    print(f"  Spawn Position:           ({init_pose[0]:.3f}, {init_pose[1]:.3f})")
    print(f"  Goal Reached:             {goal_reached}")
    print(f"  Collision Detected:       {collision_detected}")
    print(f"  Minimum Recorded OBB Clr: {min_clearance:.3f} m (Required >= 0.15 m)")
    print("=" * 85)

    assert not collision_detected, "Physical collision detected during avoidance!"
    assert goal_reached, "Vehicle failed to reach goal in PhysX simulation!"
    assert min_clearance >= 0.15, f"Clearance {min_clearance:.3f}m < 0.15m margin!"
    print("\n>>> LIVE PHYSX AVOIDANCE PROVEN! <<<\n")
    return True


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Live PhysX S02 Obstacle Avoidance Verification")
    parser.add_argument("--gui", action="store_true", help="Launch Isaac Sim with visible GUI viewport")
    parser.add_argument("--duration", type=float, default=45.0, help="Simulation duration in seconds")
    args = parser.parse_args()

    success = run_live_test(gui=args.gui, duration=args.duration)
    sys.exit(0 if success else 1)
