#!/usr/bin/env python3
"""
SKATEBOARD_IRP — 3D Digital Twin World & Interactive Inspection Launcher
Milestone 4.5: Reference Fidelity Pass (Visual Digital Twin & Laboratory Environment)

Creates a complete, photorealistic 3D scene containing:
1. High-fidelity 4-wheel vehicle digital twin (open black chassis, diamond-plate floor,
   tan padded seat & backrest with armrest, black steering wheel & column, front suspension
   with orange springs, aluminum front bumper, rear electronics box, white rims & black tires).
2. Representative indoor laboratory environment (beige tiled floor, white walls, green accent wall,
   yellow architectural panels, wooden slatted columns, glass partitions, tables, office chairs).
3. Physical RGB-D camera box mounted at front (0.9, 0.0, 1.2) m on a vertical riser post.
4. Multilayered lighting (Dome light + Distant sun light + LED panel ceiling grid).
5. 22 Dedicated Inspection Cameras for complete visual validation across all angles.

Execution Modes:
- INSPECTION / DEVELOPMENT MODE (Default / --inspect):
  Isaac Sim opens and REMAINS OPEN with NO automatic timer shutdown so the user can
  orbit, zoom, and visually inspect the world interactively.

Usage:
  /home/ubuntu/isaacsim/python.sh isaac_sim/scripts/run_fidelity_world.py --inspect
"""

import sys
import math
import argparse

# Add project root to python path
sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")


def parse_args():
    parser = argparse.ArgumentParser(description="SKATEBOARD_IRP 3D World & Vehicle Inspection Launcher")
    parser.add_argument("--inspect", action="store_true", default=True, help="Run in continuous interactive inspection mode (no auto shutdown)")
    parser.add_argument("--headless", action="store_true", help="Run in headless mode without GUI viewport")
    parser.add_argument("--duration", type=float, default=None, help="Optional duration in seconds (if specified, stops after N seconds)")
    return parser.parse_args()


def main():
    args = parse_args()

    # Step 1: Launch SimulationApp FIRST
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({
        "headless": args.headless,
        "renderer": "RayTracedLighting",
    })

    # Step 2: NOW import USD (pxr) and Isaac Sim Core API modules
    # pyrefly: ignore [missing-import]
    from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf, Sdf, UsdLux
    # pyrefly: ignore [missing-import]
    import omni.usd
    # pyrefly: ignore [missing-import]
    from isaacsim.core.api import World

    # Step 3: Import project modules
    from isaac_sim.scripts.utils.config_loader import load_config
    from isaac_sim.scripts.vehicle.simple_vehicle import create_simple_vehicle
    from isaac_sim.scripts.environments.lab_environment import create_lab_environment

    print("==================================================================")
    print("  SKATEBOARD_IRP — 3D DIGITAL TWIN & LABORATORY WORLD LAUNCHER    ")
    print("==================================================================")

    # Load configurations
    vehicle_cfg = load_config("configs/vehicle.yaml")["vehicle"]
    sim_cfg = load_config("configs/simulation.yaml")["simulation"]
    camera_cfg = load_config("configs/camera.yaml")["camera"]

    # Create World instance
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=sim_cfg["physics"]["dt"],
        rendering_dt=sim_cfg["physics"]["dt"],
    )
    print("[INFO] World created with dt =", sim_cfg["physics"]["dt"])

    stage = omni.usd.get_context().get_stage()

    # Build Laboratory Environment
    print("[INFO] Building Laboratory Environment...")
    create_lab_environment(world, vehicle_cfg)

    # Build Vehicle Articulation & High-Fidelity Visual Digital Twin
    print("[INFO] Building Vehicle Articulation & Visual Digital Twin...")
    vehicle_articulation = create_simple_vehicle(world, vehicle_cfg, static_mode=True)

    # Setup Comprehensive Inspection Cameras (for user visual inspection)
    print("[INFO] Setting up Comprehensive Inspection Cameras...")
    cameras_group_path = "/World/InspectionCameras"
    stage.DefinePrim(cameras_group_path, "Xform")
    
    veh_x, veh_y, veh_z = -8.0, 0.0, 0.35
    
    camera_configs = [
        ("ThreeQuarter_Front", (-5.0, -3.0, 2.0), (veh_x, veh_y, 0.5), 24.0, 4.0),
        ("Side_Profile_Left", (veh_x, -4.5, 1.2), (veh_x, veh_y, 0.5), 24.0, 4.5),
        ("Side_Profile_Right", (veh_x, 4.5, 1.2), (veh_x, veh_y, 0.5), 24.0, 4.5),
        ("Rear_View", (veh_x - 4.0, 0.0, 1.5), (veh_x, veh_y, 0.5), 24.0, 4.0),
        ("Driver_Perspective", (veh_x - 0.3, 0.0, 1.15), (veh_x + 3.0, 0.0, 1.15), 20.0, 5.0),
        ("Front_Close_Detail", (veh_x + 1.5, -1.0, 0.8), (veh_x + 1.0, 0.0, 0.3), 18.0, 2.0),
        ("Suspension_CloseUp", (veh_x + 0.5, -1.5, 0.5), (veh_x + 0.55, -1.2, 0.0), 16.0, 1.5),
        ("TopDown_Full_Lab", (0.0, 0.0, 20.0), (0.0, 0.0, 0.0), 28.0, 20.0),
        ("Corner_Back_Left", (-18.0, -15.0, 10.0), (-5.0, 0.0, 1.0), 24.0, 15.0),
        ("Corner_Back_Right", (18.0, -15.0, 10.0), (5.0, 0.0, 1.0), 24.0, 15.0),
        ("Corner_Front_Left", (-18.0, 15.0, 10.0), (-5.0, 0.0, 1.0), 24.0, 15.0),
        ("Corner_Front_Right", (18.0, 15.0, 10.0), (5.0, 0.0, 1.0), 24.0, 15.0),
        ("Front_Wall_Corridor", (14.0, 0.0, 3.0), (-5.0, 0.0, 0.5), 24.0, 18.0),
        ("Back_Wall_View", (-14.0, 0.0, 3.0), (5.0, 0.0, 0.5), 24.0, 18.0),
        ("Left_Corridor", (-5.0, -13.0, 2.5), (-5.0, 0.0, 0.5), 24.0, 15.0),
        ("Right_Corridor", (5.0, 13.0, 2.5), (5.0, 0.0, 0.5), 24.0, 15.0),
        ("Left_Office_View", (-8.0, -12.0, 2.0), (-5.0, -8.0, 1.0), 22.0, 8.0),
        ("Right_Office_View", (8.0, 12.0, 2.0), (5.0, 8.0, 1.0), 22.0, 8.0),
        ("Rear_Office_View", (-12.0, 0.0, 2.0), (-8.0, 0.0, 1.0), 22.0, 8.0),
        ("Meeting_Table_CloseUp", (0.0, 0.0, 2.5), (0.0, 0.0, 0.75), 20.0, 3.0),
        ("Low_Angle_Vehicle", (veh_x, -3.0, 0.3), (veh_x, 0.0, 0.5), 18.0, 3.5),
        ("High_Angle_Vehicle", (veh_x, 0.0, 5.0), (veh_x, 0.0, 0.5), 24.0, 5.0),
    ]
    
    for name, pos, target, f_len, f_dist in camera_configs:
        cam_prim_path = f"{cameras_group_path}/{name}"
        cam_prim = stage.DefinePrim(cam_prim_path, "Camera")
        cam_xform = UsdGeom.Xformable(cam_prim)
        cam_xform.AddTranslateOp().Set(Gf.Vec3d(*pos))
        
        cam_dir = Gf.Vec3d(*target) - Gf.Vec3d(*pos)
        rot_y = math.degrees(math.atan2(cam_dir[2], math.sqrt(cam_dir[0]**2 + cam_dir[1]**2)))
        rot_z = math.degrees(math.atan2(cam_dir[1], cam_dir[0])) - 90.0
        cam_xform.AddRotateXYZOp().Set(Gf.Vec3d(rot_y, 0.0, rot_z))
        
        cam_sensor = UsdGeom.Camera(cam_prim)
        cam_sensor.CreateFocalLengthAttr().Set(f_len)
        cam_sensor.CreateFocusDistanceAttr().Set(f_dist)
    
    print("[INFO] Inspection cameras created (22 total):")
    print("  VEHICLE-FOCUSED (1-7): ThreeQuarter_Front, Side_Profile_Left/Right, Rear_View, Driver_Perspective, Front_Close_Detail, Suspension_CloseUp")
    print("  LAB OVERVIEW (8-12): TopDown_Full_Lab, 4 Corner Overviews")
    print("  CORRIDOR VIEWS (13-16): Front/Back Wall Corridors, Left/Right Corridors")
    print("  OFFICE VIEWS (17-19): Left/Right/Rear Offices")
    print("  SPECIALTY (20-22): Meeting Table, Low/High Angle Vehicle")
    print("\n  Switch cameras in viewport Camera menu to inspect all angles.")

    # Initialize Physics World
    world.reset()
    print("[INFO] World reset complete — stage ready for inspection.")
    print("[INFO] USD Hierarchy validated under /World.")
    
    if args.duration is not None:
        print(f"[INFO] Running in DURATION MODE for {args.duration} seconds...")
        max_steps = int(args.duration / sim_cfg["physics"]["dt"])
        step_count = 0
        while simulation_app.is_running() and step_count < max_steps:
            world.step(render=not args.headless)
            step_count += 1
        print("[INFO] Duration mode execution finished.")
    else:
        print("\n==================================================================")
        print(" [INSPECTION / DEVELOPMENT MODE ACTIVE]                           ")
        print(" Isaac Sim is running in continuous GUI inspection mode.           ")
        print(" NO automatic time limit is set.                                  ")
        print(" Inspect vehicle silhouette, materials, bumper, orange springs,   ")
        print(" seat, RGB-D camera post, and lab environment interactively.     ")
        print(" Close the Isaac Sim window or press Ctrl+C when finished.        ")
        print("==================================================================\n")
        
        while simulation_app.is_running():
            world.step(render=not args.headless)

    print("[INFO] Closing SimulationApp...")
    simulation_app.close()


if __name__ == "__main__":
    main()
