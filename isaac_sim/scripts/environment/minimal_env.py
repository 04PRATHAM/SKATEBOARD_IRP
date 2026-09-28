"""
Minimal environment for Milestone 2 testing.
Creates:
- Ground plane (floor)
- Lighting (dome + distant light)
- One wall
- One obstacle (box)
- One person (capsule approximation)
"""

import numpy as np


def create_minimal_environment(world, vehicle_cfg: dict):
    """
    Create minimal test environment.
    
    Args:
        world: Isaac Sim World instance
        vehicle_cfg: Vehicle configuration (for sizing reference)
    """
    # Import pxr modules here (after SimulationApp is initialized)
    from pxr import Usd, UsdGeom, UsdPhysics, UsdLux, Gf, Sdf
    import omni.usd
    from isaacsim.core.utils.prims import define_prim
    from isaacsim.core.api import World
    
    stage = omni.usd.get_context().get_stage()
    
    # Environment root
    env_path = "/World/Environment"
    define_prim(env_path, "Xform")
    
    # Helper to create a cube with transform
    def create_cube(path, translation, scale, color=None):
        prim = define_prim(path, "Cube")
        xformable = UsdGeom.Xformable(prim)
        xformable.AddTranslateOp().Set(Gf.Vec3d(*translation))
        xformable.AddScaleOp().Set(Gf.Vec3d(*scale))
        if color:
            # Apply material color (simplified)
            pass
        return prim
    
    def create_capsule(path, translation, radius, height):
        prim = define_prim(path, "Capsule")
        xformable = UsdGeom.Xformable(prim)
        xformable.AddTranslateOp().Set(Gf.Vec3d(*translation))
        # Capsule has radius and height attributes
        prim.GetAttribute("radius").Set(radius)
        prim.GetAttribute("height").Set(height)
        return prim
    
    def create_sphere(path, translation, radius):
        prim = define_prim(path, "Sphere")
        xformable = UsdGeom.Xformable(prim)
        xformable.AddTranslateOp().Set(Gf.Vec3d(*translation))
        prim.GetAttribute("radius").Set(radius)
        return prim
    
    def create_cylinder(path, translation, radius, height, orientation=None):
        prim = define_prim(path, "Cylinder")
        xformable = UsdGeom.Xformable(prim)
        xformable.AddTranslateOp().Set(Gf.Vec3d(*translation))
        if orientation:
            xformable.AddRotateXYZOp().Set(Gf.Vec3d(*orientation))
        prim.GetAttribute("radius").Set(radius)
        prim.GetAttribute("height").Set(height)
        return prim
    
    # --- 1. GROUND PLANE ---
    ground_path = f"{env_path}/GroundPlane"
    ground_prim = create_cube(ground_path, (0.0, 0.0, -0.05), (50.0, 50.0, 0.1))
    UsdPhysics.CollisionAPI.Apply(ground_prim)
    rigid_api = UsdPhysics.RigidBodyAPI.Apply(ground_prim)
    rigid_api.CreateKinematicEnabledAttr().Set(True)
    print("[INFO] Ground plane created (50m x 50m)")
    
    # --- 2. LIGHTING ---
    lighting_path = "/World/Lighting"
    define_prim(lighting_path, "Xform")
    
    # Dome light
    dome_light_path = f"{lighting_path}/DomeLight"
    dome_light = UsdLux.DomeLight.Define(stage, dome_light_path)
    dome_light.CreateIntensityAttr().Set(500.0)
    dome_light.CreateColorAttr().Set(Gf.Vec3f(1.0, 1.0, 1.0))
    
    # Distant light
    distant_light_path = f"{lighting_path}/DistantLight"
    distant_light = UsdLux.DistantLight.Define(stage, distant_light_path)
    distant_light.CreateIntensityAttr().Set(3000.0)
    distant_light.CreateColorAttr().Set(Gf.Vec3f(1.0, 0.95, 0.9))
    distant_xform = UsdGeom.Xformable(distant_light.GetPrim())
    distant_xform.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 30.0, 0.0))
    print("[INFO] Lighting created (Dome + Distant)")
    
    # --- 3. WALL ---
    wall_path = f"{env_path}/Wall"
    wall_prim = create_cube(wall_path, (5.0, 0.0, 1.5), (0.2, 6.0, 3.0))
    UsdPhysics.CollisionAPI.Apply(wall_prim)
    rigid_api = UsdPhysics.RigidBodyAPI.Apply(wall_prim)
    rigid_api.CreateKinematicEnabledAttr().Set(True)
    print("[INFO] Wall created at x=5m")
    
    # --- 4. OBSTACLE (Box) ---
    obstacle_path = f"{env_path}/Obstacle_Box"
    obstacle_prim = create_cube(obstacle_path, (3.0, 1.0, 0.5), (1.0, 1.0, 1.0))
    UsdPhysics.CollisionAPI.Apply(obstacle_prim)
    rigid_api = UsdPhysics.RigidBodyAPI.Apply(obstacle_prim)
    rigid_api.CreateKinematicEnabledAttr().Set(True)
    print("[INFO] Obstacle box created at (3, 1)")
    
    # --- 5. PERSON (Capsule approximation) ---
    person_path = f"{env_path}/Person_01"
    person_xform = define_prim(person_path, "Xform")
    xformable = UsdGeom.Xformable(person_xform)
    xformable.AddTranslateOp().Set(Gf.Vec3d(4.0, -1.0, 0.0))
    
    # Body capsule (torso)
    body_path = f"{person_path}/Body"
    body_prim = create_capsule(body_path, (0.0, 0.0, 1.2), 0.25, 1.0)
    UsdPhysics.CollisionAPI.Apply(body_prim)
    
    # Head sphere
    head_path = f"{person_path}/Head"
    head_prim = create_sphere(head_path, (0.0, 0.0, 1.85), 0.15)
    UsdPhysics.CollisionAPI.Apply(head_prim)
    
    # Make person static (kinematic)
    for part in [body_prim, head_prim]:
        rigid_api = UsdPhysics.RigidBodyAPI.Apply(part)
        rigid_api.CreateKinematicEnabledAttr().Set(True)
    print("[INFO] Person (capsule) created at (4, -1)")
    
    # --- Optional: Second person ---
    person2_path = f"{env_path}/Person_02"
    person2_xform = define_prim(person2_path, "Xform")
    xformable2 = UsdGeom.Xformable(person2_xform)
    xformable2.AddTranslateOp().Set(Gf.Vec3d(6.0, 0.0, 0.0))
    
    body2_path = f"{person2_path}/Body"
    body2_prim = create_capsule(body2_path, (0.0, 0.0, 1.2), 0.25, 1.0)
    UsdPhysics.CollisionAPI.Apply(body2_prim)
    rigid_api = UsdPhysics.RigidBodyAPI.Apply(body2_prim)
    rigid_api.CreateKinematicEnabledAttr().Set(True)
    print("[INFO] Second person created at (6, 0)")
    
    print("[INFO] Minimal environment creation complete")


def create_lab_environment(world, vehicle_cfg: dict):
    """Placeholder for full laboratory environment (Milestone 7)."""
    pass