#!/usr/bin/env python3
"""
Development Mode - Visual Inspection Script
Creates the full SKATEBOARD_IRP world and keeps Isaac Sim GUI OPEN for inspection.
No automatic time limit - simulation runs until user closes it manually.

Usage:
    /home/ubuntu/isaacsim/python.sh isaac_sim/scripts/run_development_mode.py

This is MODE 1: DEVELOPMENT / INSPECTION MODE
- World loads and stays open
- No episode timer
- No automatic reset/shutdown
- User can orbit, zoom, inspect hierarchy, check camera placement, review lighting
"""

import sys
import math

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")


def main():
    # Launch Isaac Sim FIRST - this initializes all extensions including pxr
    from isaacsim import SimulationApp

    # DEVELOPMENT MODE: headless=False, NO duration limit
    simulation_app = SimulationApp({
        "headless": False,  # GUI visible for inspection
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

    print("=" * 70)
    print("SKATEBOARD_IRP - DEVELOPMENT / INSPECTION MODE")
    print("=" * 70)
    print("[INFO] Isaac Sim launched successfully")
    print("[INFO] Mode: DEVELOPMENT / INSPECTION (no time limit)")
    print("[INFO] GUI will remain open for visual inspection")
    print("[INFO] Press Ctrl+C or close window to exit")
    print("=" * 70)

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

    # Create lab environment
    create_lab_environment(world, vehicle_cfg)
    print("[INFO] Lab environment created")

    # Create vehicle with visual improvements
    vehicle = create_simple_vehicle(world, vehicle_cfg)
    print("[INFO] Vehicle with visual improvements created")

    # Reset world to initialize physics
    world.reset()
    print("[INFO] World reset - physics initialized")

    # ============================================================
    # INSPECTION CAMERAS - Multiple viewpoints for visual review
    # ============================================================
    stage = omni.usd.get_context().get_stage()

    def create_inspection_camera(name, position, target=(0, 0, 0)):
        """Create a camera prim for inspection viewpoints."""
        cam_path = f"/World/InspectionCameras/{name}"
        cam_prim = UsdGeom.Camera.Define(stage, cam_path)
        xform = UsdGeom.Xformable(cam_prim.GetPrim())
        xform.AddTranslateOp().Set(Gf.Vec3d(*position))
        # Point camera at target using LookAt
        # USD camera looks along -Z, so we need to orient it
        if target != (0, 0, 0):
            direction = Gf.Vec3d(*target) - Gf.Vec3d(*position)
            # Calculate yaw and pitch to look at target
            yaw = math.degrees(math.atan2(direction[1], direction[0])) - 90.0
            pitch = math.degrees(math.atan2(-direction[2], math.sqrt(direction[0]**2 + direction[1]**2)))
            xform.AddRotateXYZOp().Set(Gf.Vec3d(pitch, 0.0, yaw))
        return cam_prim

    # Create inspection cameras group
    inspect_xform = create_prim("/World/InspectionCameras", "Xform")

    # Vehicle position for targeting
    veh_x, veh_y, veh_z = -8.0, 0.0, 0.65

# 1. Three-Quarter Front View (Default Inspection) - looking at vehicle
    create_inspection_camera(
        "ThreeQuarter_Front",
        (-5.0, -3.0, 2.0),
        (veh_x, veh_y, 0.5)
    )

    # 2. Side Profile View - looking at vehicle side
    create_inspection_camera(
        "Side_Profile",
        (veh_x, -4.0, 1.5),
        (veh_x, veh_y, 0.5)
    )

    # 3. Rear View - looking at vehicle rear
    create_inspection_camera(
        "Rear_View",
        (veh_x - 4.0, 0.0, 1.5),
        (veh_x, veh_y, 0.5)
    )

    # 4. Top-Down View - full lab overview
    create_inspection_camera(
        "TopDown_Lab_Overview",
        (0.0, 0.0, 20.0),
        (0.0, 0.0, 0.0)
    )

    # 5. Driver's Perspective - from driver seat
    create_inspection_camera(
        "Driver_Perspective",
        (veh_x - 0.3, 0.0, 1.15),
        (veh_x + 2.0, 0.0, 1.15)
    )

    # 6. Front View - looking down the lab from front wall
    create_inspection_camera(
        "FrontWall_View",
        (14.0, 0.0, 2.5),
        (-5.0, 0.0, 0.5)
    )

    # 7. Elevated Corner View - full lab diagonal
    create_inspection_camera(
        "Corner_Overview",
        (-15.0, -12.0, 8.0),
        (0.0, 0.0, 1.0)
    )

    print("[INFO] Inspection cameras created:")
    print("  1. ThreeQuarter_Front - 3/4 front view of vehicle")
    print("  2. Side_Profile - Vehicle side silhouette")
    print("  3. Rear_View - Vehicle rear")
    print("  4. TopDown_Lab_Overview - Full lab from above")
    print("  5. Driver_Perspective - From driver seat")
    print("  6. FrontWall_View - Looking down lab from front")
    print("  7. Corner_Overview - Diagonal lab overview")

    # ============================================================
    # MAIN LOOP - Runs until user closes Isaac Sim
    # ============================================================
    print("\n" + "=" * 70)
    print("SIMULATION RUNNING - GUI OPEN FOR INSPECTION")
    print("=" * 70)
    print("You can now:")
    print("  - Orbit camera to inspect vehicle from all angles")
    print("  - Switch to inspection cameras in viewport")
    print("  - Check USD hierarchy in Stage window")
    print("  - Verify materials, lighting, proportions")
    print("  - Check RGB-D camera placement on vehicle")
    print("  - Inspect lab environment (walls, furniture, lighting)")
    print("")
    print("Close Isaac Sim window or press Ctrl+C to exit.")
    print("=" * 70 + "\n")

    step_count = 0
    try:
        while simulation_app.is_running():
            world.step(render=True)
            step_count += 1
            
            # Print status every 5 seconds (250 steps at 50Hz)
            if step_count % 250 == 0:
                pos, _ = vehicle.get_world_poses()
                if len(pos) > 0:
                    print(f"  [Step {step_count}] Vehicle pos: ({pos[0][0]:.2f}, {pos[0][1]:.2f}, {pos[0][2]:.2f})")
                    
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user")
    finally:
        print("[INFO] Closing simulation...")
        simulation_app.close()


if __name__ == "__main__":
    main()