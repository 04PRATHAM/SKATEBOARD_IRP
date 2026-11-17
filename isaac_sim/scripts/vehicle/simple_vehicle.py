"""
Simple 4-wheeled vehicle physics articulation using Isaac Sim primitives.
Creates the physics collision and joint structure for the vehicle while keeping
physics primitives INVISIBLE in the renderer so the high-fidelity visual digital twin
(open tubular frame, tan seat, steering assembly, orange springs, aluminum bumper, white rims)
is rendered clearly without being obscured.

Vehicle Root Path: /World/Vehicle
"""

import numpy as np
from typing import Optional


def create_simple_vehicle(world, vehicle_cfg: dict, static_mode: bool = True,
                        start_xy: tuple = (-4.0, 0.0)):
    """
    Create a 4-wheeled vehicle physics articulation using USD primitives.

    Args:
        world: Isaac Sim World instance
        vehicle_cfg: Vehicle configuration dictionary
        static_mode: If True, vehicle physics articulation is initialized cleanly
        start_xy: (x, y) spawn on the lab floor. Static inspect uses (-4, 0);
            dynamic motion tests use the open spot (6, -6) — see
            utils/world_setup.py. Passed through to the RGB-D mount.

    Returns:
        Articulation object for the vehicle
    """
    # pyrefly: ignore [missing-import]
    from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf, Sdf
    # pyrefly: ignore [missing-import]
    import omni.usd
    # pyrefly: ignore [missing-import]
    from isaacsim.core.prims import Articulation
    # pyrefly: ignore [missing-import]
    from isaacsim.core.utils.prims import define_prim, get_prim_at_path
    
    stage = omni.usd.get_context().get_stage()
    
    def safe_define_prim(path: str, prim_type: str = "Xform"):
        prim = get_prim_at_path(path)
        if prim and prim.IsValid():
            return prim
        return define_prim(path, prim_type)
    
    # Vehicle dimensions from config
    length = vehicle_cfg["length"]              # 1.8m
    width = vehicle_cfg["width"]                # 1.2m
    height = vehicle_cfg["height"]              # 0.4m
    wheelbase = vehicle_cfg["wheelbase"]        # 1.1m
    front_track = vehicle_cfg["front_track"]    # 1.0m
    rear_track = vehicle_cfg["rear_track"]      # 1.0m
    wheel_radius = vehicle_cfg["wheel_radius"]  # 0.25m
    wheel_width = vehicle_cfg["wheel_width"]    # 0.18m
    ground_clearance = vehicle_cfg["ground_clearance"] # 0.15m
    chassis_mass = vehicle_cfg["chassis_mass"]  # 150kg
    max_steering_angle = vehicle_cfg["max_steering_angle"] # 0.6 rad
    
    vehicle_path = "/World/Vehicle"
    
    vehicle_start_x, vehicle_start_y = float(start_xy[0]), float(start_xy[1])
    vehicle_root_z = ground_clearance + height / 2.0  # 0.35m
    
    # Vehicle Xform: pure STATIC grouping (no RigidBody, no ArticulationRoot).
    # All dynamics live on the unscaled rigid-body Xforms below so joint
    # anchors are never corrupted by non-uniform scale.
    # NOTE: This keeps Vehicle as spawn group; Chassis is the floating
    # base that moves via physics.  Visuals/Sensors are parented to Chassis
    # so they follow.  See probe_hierarchy.py for world-transform audit.
    vehicle_xform = safe_define_prim(vehicle_path, "Xform")
    xformable = UsdGeom.Xformable(vehicle_xform)
    xformable.AddTranslateOp().Set(Gf.Vec3d(vehicle_start_x, vehicle_start_y, vehicle_root_z))
    chassis_path = f"{vehicle_path}/Chassis"
    
    # Helper to create exact 1:1 metric cube primitive
    def create_cube(path, translation, scale):
        prim = safe_define_prim(path, "Cube")
        cube_geom = UsdGeom.Cube(prim)
        cube_geom.CreateSizeAttr().Set(1.0)  # Size 1.0 so scale matches exact meter dimensions
        xformable = UsdGeom.Xformable(prim)
        xformable.AddTranslateOp().Set(Gf.Vec3d(*translation))
        xformable.AddScaleOp().Set(Gf.Vec3d(*scale))
        return prim
    
    def create_cylinder(path, translation, radius, height, orientation=None):
        prim = safe_define_prim(path, "Cylinder")
        xformable = UsdGeom.Xformable(prim)
        xformable.AddTranslateOp().Set(Gf.Vec3d(*translation))
        if orientation:
            xformable.AddRotateXYZOp().Set(Gf.Vec3d(*orientation))
        prim.GetAttribute("radius").Set(radius)
        prim.GetAttribute("height").Set(height)
        return prim
    
    # --- CHASSIS: unscaled rigid Xform (floating base) + child collision box ---
    chassis_prim = safe_define_prim(chassis_path, "Xform")
    UsdGeom.Xformable(chassis_prim).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
    rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(chassis_prim.GetPrim())
    mass_api = UsdPhysics.MassAPI.Apply(chassis_prim.GetPrim())
    mass_api.CreateMassAttr().Set(chassis_mass)
    mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(
        float(vehicle_cfg.get("chassis_inertia_x", 20.0)),
        float(vehicle_cfg.get("chassis_inertia_y", 50.0)),
        float(vehicle_cfg.get("chassis_inertia_z", 60.0))
    ))
    try:
        physx_chassis = PhysxSchema.PhysxRigidBodyAPI.Apply(chassis_prim.GetPrim())
        physx_chassis.CreateSolverPositionIterationCountAttr().Set(16)
        physx_chassis.CreateSolverVelocityIterationCountAttr().Set(8)
        physx_chassis.CreateEnableCcdAttr().Set(True)
    except Exception:
        pass

    if not static_mode:
        UsdPhysics.ArticulationRootAPI.Apply(chassis_prim.GetPrim())
    chassis_collider = create_cube(f"{chassis_path}/CollisionBox", (0.0, 0.0, 0.0),
                                   (length, width, height))
    UsdPhysics.CollisionAPI.Apply(chassis_collider.GetPrim())
    try:
        physx_col = PhysxSchema.PhysxCollisionAPI.Apply(chassis_collider.GetPrim())
        physx_col.CreateContactOffsetAttr().Set(0.04)
        physx_col.CreateRestOffsetAttr().Set(0.0)
    except Exception:
        pass
    UsdGeom.Imageable(chassis_collider.GetPrim()).MakeInvisible()
    
    if static_mode:
        rigid_body_api.CreateKinematicEnabledAttr().Set(True)
    else:
        pass
    
    # --- WHEEL POSITIONS (Local to vehicle root at z = 0.35m) ---
    front_axle_x = wheelbase / 2.0
    rear_axle_x = -wheelbase / 2.0
    # Wheel axle center is at world z = 0.25m -> local z = 0.25 - 0.35 = -0.10m
    wheel_local_z = wheel_radius - vehicle_root_z  # 0.25 - 0.35 = -0.10m
    
    wheel_positions = {
        "FL": (front_axle_x, front_track / 2.0, wheel_local_z),
        "FR": (front_axle_x, -front_track / 2.0, wheel_local_z),
        "RL": (rear_axle_x, rear_track / 2.0, wheel_local_z),
        "RR": (rear_axle_x, -rear_track / 2.0, wheel_local_z),
    }
    
    # --- CREATE WHEEL PHYSICS BODIES & JOINTS ---
    # True rolling wheels: Cylinder (axis Y) with CCD + proper inertia, plus
    # steer knuckles for front wheels so they can both steer (Z) and roll (Y).
    # Box wheels are stable but slide; cylinder is the correct rolling collider.
    for name, (wx, wy, wz) in wheel_positions.items():
        is_front = name in ["FL", "FR"]
        if is_front:
            knuckle_path = f"{vehicle_path}/Knuckle_{name}"
            knuckle_prim = safe_define_prim(knuckle_path, "Xform")
            knuckle_xf = UsdGeom.Xformable(knuckle_prim)
            knuckle_xf.AddTranslateOp().Set(Gf.Vec3d(wx, wy, wz))
            knuckle_rb = UsdPhysics.RigidBodyAPI.Apply(knuckle_prim.GetPrim())
            UsdPhysics.MassAPI.Apply(knuckle_prim.GetPrim()).CreateMassAttr().Set(1.0)
            if static_mode:
                knuckle_rb.CreateKinematicEnabledAttr().Set(True)
            # Wheel is SIBLING of knuckle (not child) so articulation finds both joints
            wheel_path = f"{vehicle_path}/Wheel_{name}"
            wheel_rel_pos = (wx, wy, wz)
        else:
            wheel_path = f"{vehicle_path}/Wheel_{name}"
            wheel_rel_pos = (wx, wy, wz)
            knuckle_path = None

        # Wheel rigid BODY: unscaled Xform (joint anchors stay exact).
        # Collider is a CHILD cylinder WITH (90,0,0) (correct axle Y): rotation
        # on the child shape is safe for joints AND contact (the old failure
        # combined rotated collider + scaled joint frames).
        wheel_prim = safe_define_prim(wheel_path, "Xform")
        UsdGeom.Xformable(wheel_prim).AddTranslateOp().Set(
            Gf.Vec3d(*wheel_rel_pos) if is_front else Gf.Vec3d(wx, wy, wz))
        rb_api = UsdPhysics.RigidBodyAPI.Apply(wheel_prim.GetPrim())
        UsdPhysics.MassAPI.Apply(wheel_prim.GetPrim()).CreateMassAttr().Set(vehicle_cfg.get("wheel_mass", 7.5))
        # Cylinder collider child, axle along Y (CollisionAPI explicit —
        # create_cylinder does not apply it). Make collider invisible so only
        # the realistic tire/rim visuals are seen.
        wheel_collider = create_cylinder(f"{wheel_path}/Collider", (0.0, 0.0, 0.0),
                                         wheel_radius, wheel_width, (90, 0, 0))
        UsdPhysics.CollisionAPI.Apply(wheel_collider.GetPrim())
        UsdGeom.Imageable(wheel_collider.GetPrim()).MakeInvisible()
        # Enable CCD and solver iterations to prevent contact chatter and tunneling
        try:
            physx_rb = PhysxSchema.PhysxRigidBodyAPI.Apply(wheel_prim.GetPrim())
            physx_rb.CreateCcdEnabledAttr().Set(True)
            physx_rb.CreateSolverPositionIterationCountAttr().Set(16)
            physx_rb.CreateSolverVelocityIterationCountAttr().Set(8)
        except Exception:
            pass

        if static_mode:
            # STATIC MODE: Make wheels kinematic too
            rigid_api = UsdPhysics.RigidBodyAPI.Apply(wheel_prim.GetPrim())
            rigid_api.CreateKinematicEnabledAttr().Set(True)

        # Keep wheel Xform VISIBLE so its visual children are visible.
        # Only the collision shape is hidden; the tire/rim/hub are separate
        # visible prims under the same Xform and will be culled if parent is invisible.
        # Do NOT make the wheel Xform invisible.
        # UsdGeom.Imageable(wheel_prim.GetPrim()).MakeInvisible()  # REMOVED per user report: wheels missing
        
        # --- CREATE JOINTS (only in dynamic mode) ---
        if not static_mode:
            if is_front:
                # Front steer: Chassis -> Knuckle (Z, position-controlled)
                steer_path = f"{vehicle_path}/Joint_Steer_{name}"
                steer = UsdPhysics.RevoluteJoint.Define(stage, steer_path)
                steer.CreateAxisAttr("Z")
                # CRITICAL: UsdPhysics revolute joint limits are in DEGREES, not
                # radians. max_steering_angle is stored in radians (0.6 rad), so
                # convert to degrees (~34.4 deg). Previously these were set to the
                # raw radian value 0.6 -> PhysX clamped the steer joint to ±0.6°
                # (±0.0105 rad), so the wheels could never actually steer.
                steer_limit_deg = float(np.degrees(max_steering_angle * 1.25))
                steer.CreateLowerLimitAttr().Set(-steer_limit_deg)
                steer.CreateUpperLimitAttr().Set(steer_limit_deg)
                steer.CreateBody0Rel().AddTarget(chassis_path)
                steer.CreateBody1Rel().AddTarget(knuckle_path)
                steer.CreateLocalPos0Attr().Set(Gf.Vec3f(wx, wy, wz))
                steer.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
                steer.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
                steer.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
                # Proper DriveAPI (raw CreateAttribute drive:* attrs are ignored
                # by the PhysX tensor pipeline — steer never tracked with them).
                # Steer = position servo only (no velocity target: it would
                # fight position tracking).
                sp = steer.GetPrim()
                sp_drive = UsdPhysics.DriveAPI.Apply(sp, "angular")
                sp_drive.CreateStiffnessAttr().Set(5e4)
                sp_drive.CreateDampingAttr().Set(500.0)
                sp_drive.CreateTargetPositionAttr().Set(0.0)
                sp_drive.CreateMaxForceAttr().Set(5000.0)
                # Front drive: Knuckle -> Wheel (Y, velocity-controlled, rolls)
                drive_path = f"{vehicle_path}/Joint_Drive_{name}"
                drive = UsdPhysics.RevoluteJoint.Define(stage, drive_path)
                drive.CreateAxisAttr("Y")
                # Continuous rotation: DO NOT set limits (PhysX will clamp to ±180 and limit rotation)
                drive.CreateBody0Rel().AddTarget(knuckle_path)
                drive.CreateBody1Rel().AddTarget(wheel_path)
                drive.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
                drive.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
                drive.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
                # Identity: bodies are unscaled, axle mapping lives on the
                # collider child, so joint frames already agree.
                drive.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
                dp = drive.GetPrim()
                dp_drive = UsdPhysics.DriveAPI.Apply(dp, "angular")
                dp_drive.CreateStiffnessAttr().Set(0.0)
                dp_drive.CreateDampingAttr().Set(10.0)
                dp_drive.CreateMaxForceAttr().Set(5000.0)
                dp_drive.CreateTargetVelocityAttr().Set(0.0)
            else:
                joint_path = f"{vehicle_path}/Joint_{name}"
                joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)
                joint.CreateAxisAttr("Y")
                # Continuous rotation: no limits
                joint.CreateBody0Rel().AddTarget(chassis_path)
                joint.CreateBody1Rel().AddTarget(wheel_path)
                joint.CreateLocalPos0Attr().Set(Gf.Vec3f(wx, wy, wz))
                joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
                joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
                joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
                jp = joint.GetPrim()
                jp_drive = UsdPhysics.DriveAPI.Apply(jp, "angular")
                jp_drive.CreateStiffnessAttr().Set(0.0)
                jp_drive.CreateDampingAttr().Set(10.0)
                jp_drive.CreateMaxForceAttr().Set(5000.0)
                jp_drive.CreateTargetVelocityAttr().Set(0.0)
    
    # --- CHASSIS<->WHEEL/KNUCKLE FILTER (dynamic mode only) ---
    if not static_mode:
        for wn in ["FL", "FR", "RL", "RR"]:
            fp_api = UsdPhysics.FilteredPairsAPI.Apply(get_prim_at_path(chassis_path))
            if wn in ["FL", "FR"]:
                fp_api.CreateFilteredPairsRel().AddTarget(Sdf.Path(f"{vehicle_path}/Knuckle_{wn}"))
                fp_api.CreateFilteredPairsRel().AddTarget(Sdf.Path(f"{vehicle_path}/Wheel_{wn}"))
            else:
                fp_api.CreateFilteredPairsRel().AddTarget(Sdf.Path(f"{vehicle_path}/Wheel_{wn}"))

    # --- TIRE/FLOOR FRICTION (dynamic mode only) ---
    # Values from configs/vehicle.yaml (0.9 longitudinal / 0.8 lateral in the
    # plan). PhysX material is isotropic: static 0.9 grips at launch, dynamic
    # 0.8 lets the rear-driven slide settle. Bound to the 4 physics wheels and
    # the lab ground plane so drive torque actually moves the vehicle.
    if not static_mode:
        # pyrefly: ignore [missing-import]
        from isaacsim.core.api.materials import PhysicsMaterial
        # pyrefly: ignore [missing-import]
        from pxr import UsdShade as _UsdShade
        tire_mat = PhysicsMaterial(
            f"{vehicle_path}/TirePhysicsMaterial",
            static_friction=float(vehicle_cfg.get("wheel_longitudinal_friction", 0.9)),
            dynamic_friction=float(vehicle_cfg.get("wheel_friction", 0.8)),
            restitution=0.0,
        )
        # Bind the COLLIDER children (bodies carry no collision themselves)
        wheel_paths = [f"{vehicle_path}/Wheel_{n}/Collider" for n in ["FL", "FR", "RL", "RR"]]
        bind_targets = wheel_paths[:]
        # Ground contact is now the dedicated infinite PhysX ground plane
        # (/World/Environment/Floor/PhysicsGroundPlane), not the visual-only
        # tiled floor cube. Bind the tire material to the ground plane's
        # collision geometry so wheel/ground friction (0.9/0.8) is preserved.
        for ground_candidate in [
            "/World/Environment/Floor/PhysicsGroundPlane/CollisionPlane",
            "/World/Environment/Floor/PhysicsGroundPlane/collisionPlane",
            "/World/Environment/Floor/PhysicsGroundPlane/geom",
            "/World/Environment/Floor/PhysicsGroundPlane",
            "/World/Environment/Floor/Tile_Ground_Plane",  # fallback (box floor)
        ]:
            gprim = stage.GetPrimAtPath(ground_candidate)
            if gprim and gprim.IsValid() and gprim.HasAPI(UsdPhysics.CollisionAPI):
                bind_targets.append(ground_candidate)
                break
        for bp in bind_targets:
            bprim = stage.GetPrimAtPath(bp)
            if bprim and bprim.IsValid():
                _UsdShade.MaterialBindingAPI.Apply(bprim).Bind(
                    tire_mat.material,
                    bindingStrength=_UsdShade.Tokens.weakerThanDescendants,
                    materialPurpose="physics",
                )
        print(f"[INFO] Tire physics material bound to {len(bind_targets)} colliders.")

    # --- SEAT VOLUME MARKER (Invisible, NEVER a collider) ---
    # This box overlaps the dynamic chassis volume. Giving it CollisionAPI would
    # make it a STATIC collider interpenetrating the chassis rigid body ->
    # permanent contact resolution -> jitter/explosion in dynamic mode.
    # The real seat collision is the chassis box itself; the visual seat mesh
    # carries no physics. Prim is kept so the hierarchy stays stable.
    seat_path = f"{vehicle_path}/Seat_Collision"
    seat_prim = create_cube(
        seat_path,
        (-length / 4.0, 0.0, height / 2.0),
        (length / 2.0, width * 0.6, height * 0.4),
    )
    UsdGeom.Imageable(seat_prim.GetPrim()).MakeInvisible()
    
    # --- CREATE RGB-D CAMERA SENSOR ---
    try:
        from isaac_sim.scripts.utils.config_loader import load_config
        camera_cfg = load_config("configs/camera.yaml")["camera"]
        from isaac_sim.scripts.sensors.rgbd_camera import create_rgbd_camera, create_camera_visual, setup_camera_render_product
        
        camera_prim_path = create_rgbd_camera(
            vehicle_path, camera_cfg, vehicle_cfg,
            vehicle_start=(vehicle_start_x, vehicle_start_y, vehicle_root_z))
        create_camera_visual(camera_prim_path.rsplit('/', 1)[0], camera_cfg)
        setup_camera_render_product(camera_prim_path, camera_cfg)
    except Exception as e:
        pass

    # --- DEDICATED DRIVER SEAT CAMERA (for true 1:1 driver POV) ---
    # Positioned at seat level (-0.25,0,0.75) local to Chassis, pitch 0°, facing +X.
    # Parented to Chassis so it moves 1:1 with vehicle and appears in Viewport Camera menu.
    try:
        # pyrefly: ignore [missing-import]
        from pxr import UsdGeom
        from isaac_sim.scripts.utils.aim import aim_prim
        driver_cam_path = f"{chassis_path}/Driver_Seat_Camera"
        driver_prim = safe_define_prim(driver_cam_path, "Xform")
        # Local position relative to Chassis
        driver_local = (-0.25, 0.0, 0.75)
        # World eye at spawn
        sx, sy = float(vehicle_start_x), float(vehicle_start_y)
        world_eye = (sx + driver_local[0], sy + driver_local[1], vehicle_root_z + driver_local[2])
        world_target = (world_eye[0] + 10.0, world_eye[1], world_eye[2])  # pitch 0, yaw 0, forward +X
        aim_prim(stage, driver_cam_path, world_eye, world_target)
        # Create Camera prim as child — distinct name for dropdown (was "Camera", now "DriverSeat_Camera")
        cam_sensor = UsdGeom.Camera.Define(stage, f"{driver_cam_path}/DriverSeat_Camera")
        cam_sensor.CreateClippingRangeAttr().Set(Gf.Vec2f(0.1, 1000.0))
        cam_sensor.CreateFocalLengthAttr().Set(18.0)
        cam_sensor.CreateHorizontalApertureAttr().Set(20.955)
        cam_sensor.CreateVerticalApertureAttr().Set(15.290)
        print(f"[INFO] Driver_Seat_Camera created at {driver_cam_path}/DriverSeat_Camera local {driver_local} pitch 0°")
    except Exception as e:
        print(f"[WARN] Driver seat camera failed: {e}")
    
    if not static_mode:
        # --- CREATE ARTICULATION API OBJECT (only for dynamic mode) ---
        articulation = Articulation(chassis_path)
        # world may be None in the stage-only (pre-reopen) build: the caller
        # adds the articulation to the scene after reopening.
        if world is not None:
            world.scene.add(articulation)
    else:
        articulation = None
    
    # --- ADD HIGH-FIDELITY VISUAL DIGITAL TWIN ---
    from isaac_sim.scripts.vehicle.visual_vehicle import create_vehicle_visuals
    create_vehicle_visuals(vehicle_path, vehicle_cfg)
    
    if static_mode:
        print(f"[INFO] Static visual vehicle created at {vehicle_path}")
        print(f"  Physics: KINEMATIC (STAGE A - visual inspection only)")
        print(f"  Visual digital twin active; physics articulation disabled.")
    else:
        print(f"[INFO] Physics vehicle articulation initialized at {vehicle_path}")
        print(f"  Physics collision bodies hidden in renderer; visual digital twin active.")
    
    return articulation


def apply_ackermann_steering(articulation, steering_angle: float, vehicle_cfg: dict):
    """Apply Ackermann steering geometry to front wheels."""
    wheelbase = vehicle_cfg["wheelbase"]
    front_track = vehicle_cfg["front_track"]
    max_steering = vehicle_cfg["max_steering_angle"]
    
    steering_angle = np.clip(steering_angle, -max_steering, max_steering)
    
    if abs(steering_angle) < 1e-6:
        delta_fl = 0.0
        delta_fr = 0.0
    else:
        tan_delta = np.tan(steering_angle)
        delta_fl = np.arctan(wheelbase * tan_delta / (wheelbase - front_track / 2.0 * tan_delta))
        delta_fr = np.arctan(wheelbase * tan_delta / (wheelbase + front_track / 2.0 * tan_delta))
    
    # Clip knuckle joint targets to physically allowed revolute limit with margin
    limit = max_steering * 1.20
    delta_fl = float(np.clip(delta_fl, -limit, limit))
    delta_fr = float(np.clip(delta_fr, -limit, limit))
    return delta_fl, delta_fr