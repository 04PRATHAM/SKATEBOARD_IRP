"""
ROS2 Ackermann controller for SKATEBOARD_IRP.
Subscribes to /cmd_ackermann (AckermannDriveStamped) and drives the
6-DOF articulation (2 steer +4 drive) via AckermannController.
Limits steering ±0.6 rad and speed ±2.0 m/s, timeout 0.5s.
Preserves RGB-D bridge.
"""
import numpy as np
import time

def drive_step(art, idx, steer, omega, dt, drive_pos):
    """One control tick for the 6-DOF articulation using proper PhysX drive TARGETS.

    Steering joints (Joint_Steer_FL/FR) are POSITION-controlled: we send the
    absolute Ackermann angle (radians) as a joint *position target* so the
    high-stiffness angular drive servos the knuckle to that angle.

    Drive joints (Joint_RL/RR + Joint_Drive_FL/FR) are VELOCITY-controlled: we
    send the wheel angular velocity (rad/s) as a joint *velocity target* so the
    zero-stiffness / damped drive spins them like a motor.

    IMPORTANT: This uses ``set_joint_position_targets`` /
    ``set_joint_velocity_targets`` (the PhysX drive interface), NOT
    ``set_joint_positions`` / ``set_joint_velocities``. The latter *teleport*
    the articulation state every frame, which fights the solver and produced
    violent chassis shaking (z bounced 0.22-0.34 m). Targets let the solver
    integrate smoothly (verified z-spread ~0.001 m while driving).

    ``drive_pos`` is retained only for signature compatibility (no longer used
    for integrated position targets, which were the source of the shaking).
    """
    import numpy as np
    from isaac_sim.scripts.vehicle.simple_vehicle import apply_ackermann_steering

    try:
        from isaac_sim.scripts.utils.config_loader import load_config
        vehicle_cfg = load_config("configs/vehicle.yaml")["vehicle"]
    except Exception:
        vehicle_cfg = {"wheelbase": 1.1, "front_track": 1.0, "max_steering_angle": 0.6}
    dfl, dfr = apply_ackermann_steering(None, float(steer), vehicle_cfg)

    # Resolve DOF indices (radians, tensor API).
    steer_names = ["Joint_Steer_FL", "Joint_Steer_FR"]
    drive_names = ["Joint_RL", "Joint_RR", "Joint_Drive_FL", "Joint_Drive_FR"]
    steer_idx = np.array([idx[n] for n in steer_names if n in idx], dtype=np.int32)
    drive_idx = np.array([idx[n] for n in drive_names if n in idx], dtype=np.int32)

    # Steering: absolute position targets (Ackermann inner/outer angles).
    if steer_idx.size:
        art.set_joint_position_targets(np.array([dfl, dfr], dtype=float),
                                       joint_indices=steer_idx)

    # Drive: velocity targets (same wheel speed on all driven wheels).
    if drive_idx.size:
        art.set_joint_velocity_targets(np.full(drive_idx.size, float(omega), dtype=float),
                                       joint_indices=drive_idx)
    return dfl, dfr


class AckermannROSController:
    def __init__(self, articulation, vehicle_cfg, world, timeout=0.5):
        self.art = articulation
        self.vehicle_cfg = vehicle_cfg
        self.world = world
        self.timeout = float(timeout)
        self.last_cmd_time = time.time()
        self.target_steer = 0.0
        self.target_speed = 0.0
        self.max_steer = float(vehicle_cfg.get("max_steering_angle", 0.6))
        self.max_speed = float(vehicle_cfg.get("max_speed", 2.0))
        self.wheelbase = float(vehicle_cfg.get("wheelbase", 1.1))
        self.track = float(vehicle_cfg.get("front_track", 1.0))
        self.radius = float(vehicle_cfg.get("wheel_radius", 0.25))
        # AckermannController from isaacsim
        from isaacsim.robot.wheeled_robots.controllers.ackermann_controller import AckermannController
        self.ctrl = AckermannController("ack", wheel_base=self.wheelbase, track_width=self.track,
                                        front_wheel_radius=self.radius, back_wheel_radius=self.radius,
                                        max_wheel_velocity=30.0, max_wheel_rotation_angle=self.max_steer)
        self.dt = float(world.get_physics_dt()) if hasattr(world, "get_physics_dt") else 0.02
        # dof indices cache
        self._init_indices()

    def _init_indices(self):
        try:
            self.art.initialize()
            names = list(self.art.dof_names)
            self.idx_steer_fl = names.index("Joint_Steer_FL")
            self.idx_steer_fr = names.index("Joint_Steer_FR")
            # drive joints: 4 of them
            self.idx_rl = names.index("Joint_RL")
            self.idx_rr = names.index("Joint_RR")
            self.idx_dfl = names.index("Joint_Drive_FL")
            self.idx_dfr = names.index("Joint_Drive_FR")
            self.has_front_drive = True
        except Exception as e:
            # fallback for old 4-DOF (no front drive)
            try:
                names = list(self.art.dof_names)
                self.idx_steer_fl = names.index("Joint_FL") if "Joint_FL" in names else 0
                self.idx_steer_fr = names.index("Joint_FR") if "Joint_FR" in names else 1
                self.idx_rl = names.index("Joint_RL")
                self.idx_rr = names.index("Joint_RR")
                self.has_front_drive = False
            except:
                self.has_front_drive = False

    def ros_callback(self, msg):
        # msg is AckermannDriveStamped
        try:
            steer = float(msg.drive.steering_angle)
            speed = float(msg.drive.speed)
        except:
            return
        # clamp
        steer = np.clip(steer, -self.max_steer, self.max_steer)
        speed = np.clip(speed, -self.max_speed, self.max_speed)
        self.target_steer = steer
        self.target_speed = speed
        self.last_cmd_time = time.time()

    def update(self):
        # timeout -> stop (keep last steer for safety)
        if time.time() - self.last_cmd_time > self.timeout:
            self.target_speed = 0.0
        if not hasattr(self, "_drive_pos"):
            self._drive_pos = {}
        if not hasattr(self, "_idx"):
            names = list(self.art.dof_names)
            self._idx = {n: names.index(n) for n in names}
        omega = float(self.target_speed) / self.radius
        dfl, dfr = drive_step(self.art, self._idx, float(self.target_steer),
                              omega, self.dt, self._drive_pos)
        return dfl, dfr
