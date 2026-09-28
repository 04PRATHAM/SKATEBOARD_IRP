"""
Single unified world builder (SKATEBOARD_IRP).

EVERY runner and test builds the SAME lab + vehicle + cameras through this
module — no more per-script settings drift. SimulationApp must already exist
(import this after creating it); pass dynamic=True for physics articulation.

Returns dict(world, stage, articulation, vehicle_cfg, camera_cfg, sim_cfg).
"""

VEHICLE_START_STATIC = (-4.0, 0.0)  # approved inspect pose, clear of glass at x=-5
VEHICLE_START_DYNAMIC = (6.0, -6.0)  # open lane for motion tests: 9 m to front wall +15, 6.5 m to left wall
# WASD manual test uses (0,-4) open corridor; verifiers keep (6,-6) for regression.
RGBD_CAMERA_PRIM = "/World/Vehicle/Chassis/Sensors/RGBD_Camera/RGBD_Camera"
DRIVER_CAMERA_PRIM = "/World/Vehicle/Chassis/Driver_Seat_Camera/DriverSeat_Camera"


def build_world(dynamic: bool = False, with_cameras: bool = True,
                with_ros: bool = False, start_xy: tuple = None,
                glass_has_collision: bool = True, create_world: bool = True):
    """Build lab + vehicle (+cameras, +ROS bridge) on the current stage.

    Args:
        create_world: when False, NO World object is created and no reset
            happens — builders run stage-only (lab/vehicle/cameras do not need
            a World; the articulation is returned un-added). This is for the
            real-human flow, which exports + reopens the stage BEFORE the one
            and only World is created (a pre-reopen World would tick against
            an invalidated stage after open_stage and hang).
    """
    from isaacsim.core.api import World
    import omni.usd
    from isaac_sim.scripts.utils.config_loader import load_config
    from isaac_sim.scripts.vehicle.simple_vehicle import create_simple_vehicle
    from isaac_sim.scripts.environments.lab_environment import create_lab_environment

    vehicle_cfg = load_config("configs/vehicle.yaml")["vehicle"]
    sim_cfg = load_config("configs/simulation.yaml")["simulation"]
    camera_cfg = load_config("configs/camera.yaml")["camera"]

    world = None
    if create_world:
        world = World(
            stage_units_in_meters=1.0,
            physics_dt=sim_cfg["physics"]["dt"],
            rendering_dt=sim_cfg["physics"]["dt"],
        )
    stage = omni.usd.get_context().get_stage()

    if start_xy is None:
        start_xy = VEHICLE_START_DYNAMIC if dynamic else VEHICLE_START_STATIC
    print("[INFO] Building Laboratory Environment...")
    create_lab_environment(world, vehicle_cfg, glass_has_collision=glass_has_collision)
    print("[INFO] Building Vehicle dynamic=%s start=%s..." % (dynamic, start_xy))
    articulation = create_simple_vehicle(world, vehicle_cfg, static_mode=not dynamic,
                                           start_xy=start_xy)

    if with_cameras:
        from isaac_sim.scripts.utils.inspection_cameras import create_inspection_cameras
        # Lab viewpoints stay anchored at the static pose; the vehicle may
        # drive away from them in dynamic runs (corner/top views still cover it).
        create_inspection_cameras(stage, *VEHICLE_START_STATIC)

    if world is not None:
        world.reset()
        print("[INFO] World reset complete — stage ready.")

    if with_ros:
        from isaac_sim.scripts.bridge.ros2_bridge import setup_ros2_bridge
        setup_ros2_bridge(RGBD_CAMERA_PRIM, camera_cfg, sim_cfg,
                           vehicle_path="/World/Vehicle")

    return {"world": world, "stage": stage, "articulation": articulation,
            "vehicle_cfg": vehicle_cfg, "camera_cfg": camera_cfg, "sim_cfg": sim_cfg}
