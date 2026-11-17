#!/usr/bin/env python3
"""
GUI Ackermann motion demo — stays open until closed.

Same shared world as every other runner (lab + dynamic vehicle + 24
inspection cameras), plus the 6-DOF Ackermann articulation driven through
isaacsim's AckermannController. Auto sequence on launch:
  settle 2 s -> straight 1 m/s 3 s -> left 0.4 rad 3 s -> stop 1 s
  -> right -0.4 rad 3 s -> stop, then HOLD OPEN for manual inspection
  and live /cmd_ackermann commands.

Usage (GUI, stays open):
  cd /home/ubuntu/SKATEBOARD_IRP
  source /opt/ros/humble/setup.bash            # enables live /cmd_ackermann
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  /home/ubuntu/isaacsim/python.sh isaac_sim/scripts/run_ackermann_demo.py

Manual ROS 2 (second terminal, while demo runs):
  ros2 topic pub -r 10 /cmd_ackermann ackermann_msgs/msg/AckermannDriveStamped \
    '{drive: {steering_angle: 0.0, speed: 1.0}}'    # straight
  ... steering_angle: 0.4 ...                       # left
  ... steering_angle: -0.4 ...                      # right
  ... speed: 0.0 ...                                # stop
"""

import sys

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")


def main():
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({"headless": False, "renderer": "RayTracedLighting"})

    import numpy as np
    # pyrefly: ignore [missing-import]
    from pxr import UsdGeom, Gf, Usd
    # pyrefly: ignore [missing-import]
    import omni.usd
    from isaac_sim.scripts.utils.config_loader import load_config
    from isaac_sim.scripts.utils.world_setup import build_world
    # pyrefly: ignore [missing-import]
    from isaacsim.robot.wheeled_robots.controllers.ackermann_controller import AckermannController

    built = build_world(dynamic=True, with_cameras=True, with_ros=False)
    world, stage, art = built["world"], built["stage"], built["articulation"]
    vehicle_cfg, sim_cfg = built["vehicle_cfg"], built["sim_cfg"]
    dt = float(sim_cfg["physics"]["dt"])
    art.initialize()
    print("\n" + "="*70)
    print(" DRIVER VIEW: Select Viewport → Camera →")
    print("   /World/Vehicle/Chassis/Sensors/RGBD_Camera/RGBD_Camera")
    print(" for the moving RGB-D driver camera (mount 0.9,0,1.2, pitch -8.5°).")
    print(" Inspection 'Driver_Perspective' at (-4.3,0,1.15) is STATIC world-")
    print(" anchored and does NOT move with the vehicle at (6,-6).")
    print("="*70 + "\n", flush=True)

    ctrl = AckermannController(
        "demo_ack", wheel_base=float(vehicle_cfg["wheelbase"]),
        track_width=float(vehicle_cfg["front_track"]),
        front_wheel_radius=float(vehicle_cfg["wheel_radius"]),
        back_wheel_radius=float(vehicle_cfg["wheel_radius"]),
        max_wheel_velocity=30.0,
        max_wheel_rotation_angle=float(vehicle_cfg["max_steering_angle"]))

    # Optional live /cmd_ackermann (needs ROS env sourced before launch)
    ros_cb = {"steer": None, "speed": None}
    try:
        import rclpy
        # pyrefly: ignore [missing-import]
        from ackermann_msgs.msg import AckermannDriveStamped

        rclpy.init()
        node = rclpy.create_node("ackermann_demo_bridge")

        def cb(msg):
            ros_cb["steer"] = float(np.clip(msg.drive.steering_angle,
                                            -vehicle_cfg["max_steering_angle"],
                                            vehicle_cfg["max_steering_angle"]))
            ros_cb["speed"] = float(np.clip(msg.drive.speed,
                                            -vehicle_cfg["max_speed"],
                                            vehicle_cfg["max_speed"]))

        node.create_subscription(AckermannDriveStamped, "/cmd_ackermann", cb, 10)
        print("[DEMO] Live /cmd_ackermann READY — publish to override the auto sequence.", flush=True)
    except Exception as e:
        print(f"[DEMO] ROS 2 unavailable ({e}); auto sequence only. Source humble to enable.", flush=True)
        node = None

    from isaac_sim.scripts.vehicle.ackermann_controller_ros import drive_step
    names = list(art.dof_names)
    I = {n: names.index(n) for n in names}
    drive_pos = {}

    def drive(steer, speed):
        """One control tick: pure articulation drive (no chassis override)."""
        omega = float(speed) / float(vehicle_cfg["wheel_radius"])
        drive_step(art, I, steer, omega, dt, drive_pos)

    def phase(seconds, steer, speed, label):
        print(f"[DEMO] {label}", flush=True)
        for _ in range(int(seconds / dt)):
            if node is not None:
                try:
                    rclpy.spin_once(node, timeout_sec=0.0)
                except Exception:
                    pass
                if ros_cb["steer"] is not None:  # manual override wins
                    drive(ros_cb["steer"], ros_cb["speed"])
                    world.step(render=True)
                    continue
            drive(steer, speed)
            world.step(render=True)

    # Open-lane waypoints from (0,-4) +X: straight stays on corridor, turns stay clear of walls/columns
    phase(2.0, 0.0, 0.0, "settle 2 s (stationary)")
    phase(2.5, 0.0, 0.8, "straight 0.8 m/s 2.5 s (open lane +X)")
    phase(2.5, 0.35, 0.6, "LEFT 0.35 rad 2.5 s (wide turn, stays clear)")
    phase(1.0, 0.0, 0.0, "stop 1 s")
    phase(2.5, -0.35, 0.6, "RIGHT -0.35 rad 2.5 s (wide turn, stays clear)")
    phase(1.0, 0.0, 0.0, "stop — GUI stays open, close window or Ctrl+C to exit")

    # Hold-open: runs indefinitely until user closes window or Ctrl+C
    try:
        while simulation_app.is_running():
            if node is not None:
                try:
                    rclpy.spin_once(node, timeout_sec=0.0)
                except Exception:
                    pass
                drive(ros_cb["steer"] or 0.0, ros_cb["speed"] or 0.0)
            else:
                drive(0.0, 0.0)
            world.step(render=True)
    except KeyboardInterrupt:
        print("[DEMO] Interrupted", flush=True)
    finally:
        simulation_app.close()
        print("[DEMO] Closed cleanly.", flush=True)


if __name__ == "__main__":
    main()
