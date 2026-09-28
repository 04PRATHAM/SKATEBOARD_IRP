#!/usr/bin/env python3
"""
Milestone 4.5 — Reference Fidelity Pass Test
Tests the vehicle with improved visuals and lab environment.
Preserves the working physics test.
"""

import sys
import argparse

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")


def parse_args():
    parser = argparse.ArgumentParser(description="Run fidelity test with improved visuals")
    parser.add_argument("--headless", action="store_true", help="Run in headless mode")
    parser.add_argument("--duration", type=float, default=20.0, help="Simulation duration in seconds")
    return parser.parse_args()


def main():
    args = parse_args()
    
    # Launch Isaac Sim FIRST - this initializes all extensions including pxr
    from isaacsim import SimulationApp
    
    simulation_app = SimulationApp({
        "headless": args.headless,
        "renderer": "RayTracedLighting",
    })
    
    # NOW import pxr and other modules that depend on extensions being loaded
    from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf, Sdf, UsdLux
    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.utils.prims import create_prim, get_prim_at_path
    
    # Import our modules
    from isaac_sim.scripts.utils.config_loader import load_config
    from isaac_sim.scripts.vehicle.simple_vehicle import create_simple_vehicle
    from isaac_sim.scripts.environments.lab_environment import create_lab_environment
    
    print("[INFO] Isaac Sim launched successfully")
    print(f"[INFO] Running fidelity test for {args.duration} seconds")
    
    # Load configurations
    vehicle_cfg = load_config("configs/vehicle.yaml")["vehicle"]
    sim_cfg = load_config("configs/simulation.yaml")["simulation"]
    steering_cfg = load_config("configs/steering.yaml")["steering"]
    
    # Create World
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=sim_cfg["physics"]["dt"],
        rendering_dt=sim_cfg["physics"]["dt"],
    )
    print("[INFO] World created")
    
    # Create lab environment
    create_lab_environment(world, vehicle_cfg)
    print("[INFO] Lab environment created")
    
    # Create vehicle with visual improvements
    vehicle = create_simple_vehicle(world, vehicle_cfg)
    print("[INFO] Vehicle with visual improvements created")
    
    # Reset world to initialize physics
    world.reset()
    print("[INFO] World reset - physics initialized")
    
    # Get joint references for control
    stage = omni.usd.get_context().get_stage()
    
    # Rear drive joints
    joint_rl = stage.GetPrimAtPath("/World/Vehicle/Joint_RL")
    joint_rr = stage.GetPrimAtPath("/World/Vehicle/Joint_RR")
    
    # Front steering joints
    joint_fl = stage.GetPrimAtPath("/World/Vehicle/Joint_FL")
    joint_fr = stage.GetPrimAtPath("/World/Vehicle/Joint_FR")
    
    # Ensure steering joints have targetPosition attribute
    for j in [joint_fl, joint_fr]:
        if j.IsValid() and not j.GetAttribute("drive:angular:physics:targetPosition"):
            j.CreateAttribute("drive:angular:physics:targetPosition", Sdf.ValueTypeNames.Float).Set(0.0)
    
    print("[INFO] Starting fidelity test...")
    print("  Phase 1: Forward (0-5s)")
    print("  Phase 2: Stop (5-7s)")
    print("  Phase 3: Left turn (7-12s)")
    print("  Phase 4: Stop (12-14s)")
    print("  Phase 5: Right turn (14-19s)")
    print("  Phase 6: Stop (19-20s)")
    
    step_count = 0
    max_steps = int(args.duration / sim_cfg["physics"]["dt"])
    dt = sim_cfg["physics"]["dt"]
    
    # Test parameters
    drive_velocity = 10.0  # rad/s for rear wheels
    max_steering = vehicle_cfg["max_steering_angle"]
    wheelbase = vehicle_cfg["wheelbase"]
    front_track = vehicle_cfg["front_track"]
    
    import math
    
    def set_steering(angle_rad):
        """Apply Ackermann steering geometry"""
        angle_rad = max(-max_steering, min(max_steering, angle_rad))
        if abs(angle_rad) < 1e-6:
            delta_fl = delta_fr = 0.0
        else:
            tan_delta = math.tan(angle_rad)
            delta_fl = math.atan(wheelbase * tan_delta / (wheelbase - front_track / 2.0 * tan_delta))
            delta_fr = math.atan(wheelbase * tan_delta / (wheelbase + front_track / 2.0 * tan_delta))
        
        if joint_fl.IsValid():
            joint_fl.GetAttribute("drive:angular:physics:targetPosition").Set(float(delta_fl))
        if joint_fr.IsValid():
            joint_fr.GetAttribute("drive:angular:physics:targetPosition").Set(float(delta_fr))
    
    def set_drive_velocity(vel):
        if joint_rl.IsValid():
            joint_rl.GetAttribute("drive:angular:physics:targetVelocity").Set(vel)
        if joint_rr.IsValid():
            joint_rr.GetAttribute("drive:angular:physics:targetVelocity").Set(vel)
    
    # Initial setup
    set_steering(0.0)
    set_drive_velocity(0.0)
    
    while simulation_app.is_running() and step_count < max_steps:
        time = step_count * dt
        
        # Phase 1: Forward (0-5s)
        if time < 5.0:
            set_drive_velocity(10.0)
            set_steering(0.0)
            phase = "FORWARD"
        
        # Phase 2: Stop (5-7s)
        elif time < 7.0:
            set_drive_velocity(0.0)
            set_steering(0.0)
            phase = "STOP"
        
        # Phase 3: Left turn (7-12s)
        elif time < 12.0:
            set_drive_velocity(8.0)
            set_steering(0.4)
            phase = "LEFT TURN"
        
        # Phase 4: Stop (12-14s)
        elif time < 14.0:
            set_drive_velocity(0.0)
            set_steering(0.0)
            phase = "STOP"
        
        # Phase 5: Right turn (14-19s)
        elif time < 19.0:
            set_drive_velocity(8.0)
            set_steering(-0.4)
            phase = "RIGHT TURN"
        
        # Phase 6: Final stop (19-20s)
        else:
            set_drive_velocity(0.0)
            set_steering(0.0)
            phase = "STOP"
        
        world.step(render=not args.headless)
        step_count += 1
        
        if step_count % 100 == 0:
            pos, rot = vehicle.get_world_poses()
            if len(pos) > 0:
                fl_steer = joint_fl.GetAttribute("drive:angular:physics:targetPosition").Get() if joint_fl.IsValid() else 0
                fr_steer = joint_fr.GetAttribute("drive:angular:physics:targetPosition").Get() if joint_fr.IsValid() else 0
                rl_vel = joint_rl.GetAttribute("drive:angular:physics:targetVelocity").Get() if joint_rl.IsValid() else 0
                print(f"  t={time:.1f}s [{phase}]: pos=({pos[0][0]:.2f}, {pos[0][1]:.2f}, {pos[0][2]:.2f}) FL={fl_steer:.3f} FR={fr_steer:.3f} drive={rl_vel:.1f}")
    
    print("[INFO] Fidelity test complete")
    simulation_app.close()


if __name__ == "__main__":
    main()