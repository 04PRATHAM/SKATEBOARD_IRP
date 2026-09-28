#!/usr/bin/env python3
"""
Minimal Isaac Sim World - Milestone 2
Creates a simple test scene with:
- Ground plane
- Lighting
- Simple 4-wheeled vehicle (primitives)
- One wall
- One obstacle (box)
- One person (capsule)

Run with:
    /home/ubuntu/isaacsim/python.sh isaac_sim/scripts/run_minimal_world.py --headless --duration 5.0
"""

import sys
import argparse

# Add project configs to path
sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")


def parse_args():
    parser = argparse.ArgumentParser(description="Run minimal Isaac Sim world")
    parser.add_argument("--headless", action="store_true", help="Run in headless mode")
    parser.add_argument("--duration", type=float, default=5.0, help="Simulation duration in seconds")
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
    from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf
    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.utils.prims import create_prim, get_prim_at_path
    
    # Import our modules
    from isaac_sim.scripts.utils.config_loader import load_config
    from isaac_sim.scripts.vehicle.simple_vehicle import create_simple_vehicle
    from isaac_sim.scripts.environment.minimal_env import create_minimal_environment
    
    print("[INFO] Isaac Sim launched successfully")
    print(f"[INFO] Running minimal world for {args.duration} seconds")
    
    # Load configurations
    vehicle_cfg = load_config("configs/vehicle.yaml")["vehicle"]
    sim_cfg = load_config("configs/simulation.yaml")["simulation"]
    
    # Create World
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=sim_cfg["physics"]["dt"],
        rendering_dt=sim_cfg["physics"]["dt"],
    )
    print("[INFO] World created")
    
    # Create minimal environment (floor, lighting, wall, obstacle, person)
    create_minimal_environment(world, vehicle_cfg)
    print("[INFO] Minimal environment created")
    
    # Create simple vehicle
    vehicle = create_simple_vehicle(world, vehicle_cfg)
    print("[INFO] Simple vehicle created")
    
    # Reset world to initialize physics
    world.reset()
    print("[INFO] World reset - physics initialized")
    
    # Run simulation for specified duration
    print(f"[INFO] Starting simulation loop for {args.duration}s...")
    step_count = 0
    max_steps = int(args.duration / sim_cfg["physics"]["dt"])
    
    # Drive command: apply velocity to rear wheels after 2 seconds (100 steps)
    drive_start_step = 100  # 2 seconds at 50Hz
    rear_wheel_velocity = 10.0  # rad/s
    
    # Get rear wheel joint paths
    joint_rl_path = "/World/Vehicle/Joint_RL"
    joint_rr_path = "/World/Vehicle/Joint_RR"
    
    from pxr import Usd
    stage = omni.usd.get_context().get_stage()
    joint_rl = stage.GetPrimAtPath(joint_rl_path)
    joint_rr = stage.GetPrimAtPath(joint_rr_path)
    
    while simulation_app.is_running() and step_count < max_steps:
        # Apply drive command after initial settling
        if step_count == drive_start_step:
            print("[INFO] Applying drive command to rear wheels...")
            if joint_rl.IsValid():
                joint_rl.GetAttribute("drive:angular:physics:targetVelocity").Set(rear_wheel_velocity)
            if joint_rr.IsValid():
                joint_rr.GetAttribute("drive:angular:physics:targetVelocity").Set(rear_wheel_velocity)
        
        world.step(render=not args.headless)
        step_count += 1
        
        if step_count % 100 == 0:
            # Print vehicle position every 2 seconds (at 50Hz)
            if vehicle:
                pos, _ = vehicle.get_world_poses()
                # get_world_poses returns arrays, get first vehicle's position
                if len(pos) > 0:
                    print(f"  Step {step_count}: Vehicle position = ({pos[0][0]:.2f}, {pos[0][1]:.2f}, {pos[0][2]:.2f})")
    
    print("[INFO] Simulation complete")
    simulation_app.close()


if __name__ == "__main__":
    main()