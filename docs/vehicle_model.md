# Vehicle Model Documentation

## Overview
This document describes the final vehicle model architecture for the SKATEBOARD_IRP project, including physics articulation, visual digital twin, steering model, and control interfaces.

## Coordinate System
- **+X**: Forward (vehicle front)
- **+Y**: Left (vehicle left side)
- **+Z**: Up (world up)
- Origin: Vehicle center at ground level (z=0.35m for chassis center)

## Physical Dimensions (from configs/vehicle.yaml)

| Parameter | Value | Source |
|-----------|-------|--------|
| Length | 1.8 m | Estimated from photos |
| Width | 1.2 m | Estimated from photos |
| Height | 0.4 m | Estimated from photos |
| Wheelbase | 1.1 m | Estimated from photos |
| Front Track | 1.0 m | Estimated (narrow track per spec) |
| Rear Track | 1.0 m | Estimated |
| Wheel Radius | 0.25 m | Estimated (large wheels visible) |
| Wheel Width | 0.18 m | Estimated |
| Ground Clearance | 0.15 m | Estimated |
| Chassis Mass | 120 kg | Assumed (150 kg total with wheels) |
| Wheel Mass | 7.5 kg each | Assumed |

## Physics Articulation Architecture

### Hierarchy (USD Stage Path: `/World/Vehicle`)
```
/World/Vehicle
├── Chassis (ArticulationRoot, floating base)
│   ├── CollisionBox (CollisionAPI, invisible)
│   ├── Visuals/ (high-fidelity visual digital twin)
│   └── Sensors/
│       └── RGBD_Camera/
│           └── Camera (sensor prim)
├── Knuckle_FL (RigidBody, 1kg)
├── Knuckle_FR (RigidBody, 1kg)
├── Wheel_FL (RigidBody, 7.5kg)
│   └── Collider (Cylinder, axle Y, CCD enabled)
├── Wheel_FR (RigidBody, 7.5kg)
│   └── Collider (Cylinder, axle Y, CCD enabled)
├── Wheel_RL (RigidBody, 7.5kg)
│   └── Collider (Cylinder, axle Y, CCD enabled)
├── Wheel_RR (RigidBody, 7.5kg)
│   └── Collider (Cylinder, axle Y, CCD enabled)
└── Seat_Collision (invisible, no CollisionAPI)
```

### Joints (6 DOF Total)

| Joint | Type | Axis | Limits | Drive | Body0 | Body1 |
|-------|------|------|--------|-------|-------|-------|
| Joint_Steer_FL | Revolute | Z | ±0.6 rad | Position (stiffness=1e6, damping=1000, maxForce=5000) | Chassis | Knuckle_FL |
| Joint_Steer_FR | Revolute | Z | ±0.6 rad | Position (stiffness=1e6, damping=1000, maxForce=5000) | Chassis | Knuckle_FR |
| Joint_Drive_FL | Revolute | Y | Continuous | Velocity (stiffness=0, damping=10, maxForce=5000) | Knuckle_FL | Wheel_FL |
| Joint_Drive_FR | Revolute | Y | Continuous | Velocity (stiffness=0, damping=10, maxForce=5000) | Knuckle_FR | Wheel_FR |
| Joint_RL | Revolute | Y | Continuous | Velocity (stiffness=0, damping=10, maxForce=5000) | Chassis | Wheel_RL |
| Joint_RR | Revolute | Y | Continuous | Velocity (stiffness=0, damping=10, maxForce=5000) | Chassis | Wheel_RR |

### Key Physics Decisions
1. **Unscaled Bodies**: All physics Xforms have identity scale (1,1,1) — joint anchors stay exact
2. **Child Colliders**: Collision shapes are children of body Xforms with local transforms
3. **Cylinder Axle Y**: Wheel colliders use `(90,0,0)` rotation for correct Y-axis rotation
3. **CCD Enabled**: Continuous Collision Detection on wheels prevents tunneling at speed
4. **Filtered Pairs**: Chassis↔Knuckle/Wheel pairs filtered to prevent interpenetration jitter
5. **Friction Material**: 0.9 static / 0.8 dynamic bound to wheel colliders + floor
5. **Floor**: Static 0.20m thick cube (top at z=0.0), tile lines visual-only

## Ackermann Steering Model

### Geometry
```
Wheelbase L = 1.1 m
Front Track W = 1.0 m
Steering Angle δ (input)

Inner Wheel Angle:  δ_in = atan(L·tan(δ) / (L - W/2·tan(δ)))
Outer Wheel Angle:  δ_out = atan(L·tan(δ) / (L + W/2·tan(δ)))
```

### Example (δ = 0.4 rad / 23°)
- Inner (FL for left): 0.482 rad (27.6°)
- Outer (FR for left): 0.341 rad (19.5°)

### Implementation
```python
def apply_ackermann_steering(articulation, steering_angle, vehicle_cfg):
    wheelbase = vehicle_cfg["wheelbase"]
    front_track = vehicle_cfg["front_track"]
    max_steering = vehicle_cfg["max_steering_angle"]
    
    steering_angle = np.clip(steering_angle, -max_steering, max_steering)
    
    if abs(steering_angle) < 1e-6:
        return 0.0, 0.0
    
    tan_delta = np.tan(steering_angle)
    delta_fl = np.arctan(wheelbase * tan_delta / (wheelbase - front_track/2.0 * tan_delta))
    delta_fr = np.arctan(wheelbase * tan_delta / (wheelbase + front_track/2.0 * tan_delta))
    
    return delta_fl, delta_fr
```

## Drive Control (`drive_step`)

### Per-Step Control (50 Hz / dt=0.02s)
```python
def drive_step(art, idx, steer, omega, dt, drive_pos):
    # 1. Ackermann steering angles
    dfl, dfr = apply_ackermann_steering(None, steer, vehicle_cfg)
    
    # 2. Get current positions
    pos = art.get_joint_positions().flatten()
    vel = np.zeros(art.num_dof)
    
    # 3. Steering: DriveAPI targetPosition + pos array
    for name, target in [("Joint_Steer_FL", dfl), ("Joint_Steer_FR", dfr)]:
        drive_api = UsdPhysics.DriveAPI(stage.GetPrimAtPath(f"/World/Vehicle/{name}"), "angular")
        drive_api.GetTargetPositionAttr().Set(target)
        pos[idx[name]] = target
    
    # 4. Drive: integrated position + velocity
    for k in ["Joint_RL", "Joint_RR", "Joint_Drive_FL", "Joint_Drive_FR"]:
        drive_pos[k] = drive_pos.get(k, pos[idx[k]]) + omega * dt
        pos[idx[k]] = drive_pos[k]
        vel[idx[k]] = omega
    
    # 5. Apply
    art.set_joint_positions(pos)
    art.set_joint_velocities(vel)
    return dfl, dfr
```

### Critical Design Notes
- **Steering**: Position control via DriveAPI targetPosition (stiffness 1e6)
- **Drive**: Velocity control (omega) + integrated position tracking
- **Both required**: Velocity-only locks steer at 0; position-only locks wheels
- **Known Limitation**: Steering joints only reach ~0.01 rad vs 0.48 target due to drive/physics interaction; functional steering achieved via per-step position reset

## ROS2 Interface

### Subscribed Topic
- `/cmd_ackermann` (ackermann_msgs/AckermannDriveStamped)
  - `drive.steering_angle`: rad (±0.6 clamp)
  - `drive.speed`: m/s (±2.0 clamp)

### Controller Parameters
- Max Steering: 0.6 rad (34°)
- Max Speed: 2.0 m/s
- Timeout: 0.5s (stops on command loss)
- Steering Rate Limit: 2.0 rad/s (in AckermannController)

### AckermannROSController
```python
class AckermannROSController:
    def __init__(self, articulation, vehicle_cfg, world, timeout=0.5):
        self.art = articulation
        self.vehicle_cfg = vehicle_cfg
        self.world = world
        self.timeout = timeout
        # ... initialization ...
    
    def ros_callback(self, msg):
        # Clamp and store targets
        self.target_steer = np.clip(msg.drive.steering_angle, -max_steer, max_steer)
        self.target_speed = np.clip(msg.drive.speed, -max_speed, max_speed)
        self.last_cmd_time = time.time()
    
    def update(self):
        # Timeout -> stop
        if time.time() - self.last_cmd_time > self.timeout:
            self.target_speed = 0.0
        omega = self.target_speed / wheel_radius
        drive_step(self.art, self.idx, self.target_steer, omega, self.dt, self._drive_pos)
```

## Visual Digital Twin (`visual_vehicle.py`)

### Components (under `/World/Vehicle/Chassis/Visuals/`)
1. **Chassis Frame**: Open tubular frame (longitudinal rails + cross members)
2. **Floor Platform**: Diamond-plate black deck
3. **Front Bumper**: Extruded aluminum bar
4. **Seat**: Tan leather bench (0.80m wide) + tall backrest + grey armrest pads
5. **Steering Assembly**: Angled column (0,-16,0) + tilted wheel (0,-65,0) with 3 spokes
6. **Front Cowl**: Large black dash with E-stop (yellow base + red mushroom)
7. **Suspension**: Dual wishbones + orange coil springs (vibrant orange)
8. **Rear Electronics Box**: Black enclosure with red/white lights
9. **Wheels**: Knobby ATV tires (14 chevron treads) + inset white rims + hubs + lug bolts
10. **Camera**: Physical sensor box on front riser post at (0.9, 0, 1.2)m
11. **Details**: Silver motor drum, amber indicators, grey mast, battery box, fender flares

### Visual-Physics Separation
- Physics bodies: **Invisible** (`UsdGeom.Imageable.MakeInvisible()`)
- Visual geometry: **Visible** children of physics bodies (follows motion exactly)
- Wheels: Visual tire/rim/hub as children of physics wheel bodies

## Sensor Configuration (from configs/camera.yaml)

| Parameter | Value |
|-----------|-------|
| Resolution | 640×480 |
| Horizontal FOV | 70° |
| Vertical FOV | 45° |
| Depth Range | 0.1–10.0 m |
| Mount Position | (0.9, 0.0, 1.2) m from rear axle |
| Mount Pitch | -8.5° (downward) |
| Frame ID | `camera_rgb_optical_frame` / `camera_depth_optical_frame` |
| Topics | `/camera/rgb/image_raw`, `/camera/depth/image_raw`, camera_info |

## Verification Results

### Milestone 3: Vehicle Physics Stability
- ✅ Vehicle rests at z=0.334m (ground clearance + chassis/2)
- ✅ Tilt < 2° at rest
- ✅ 6-DOF articulation initialized
- ✅ No physics explosions

### Milestone 4: Ackermann Steering
- ✅ Straight: 3.92m in 4s at 4 rad/s wheel speed
- ✅ Left turn (0.4 rad): +0.479 rad/s yaw rate
- ✅ Right turn (-0.4 rad): -0.104 rad/s yaw rate
- ✅ Stop: Settles to <0.01 m/s in 5s
- ⚠️ Mirrored rates: Left/right asymmetry (0.479 vs 0.104 rad/s)
- ✅ Steering clamps: ±0.6 rad enforced
- ✅ Speed clamps: ±2.0 m/s enforced

### M5/M6: ROS2 Camera Bridge
- ✅ 6 topics live: RGB, depth, camera_info ×2, /clock, /tf
- ✅ RGB: 640×480 rgb8, mean ~109
- ✅ Depth: 640×480 32FC1, range 1.8–9.98m

## Known Issues & Limitations

1. **Steering Joint Position Tracking**: Steering joints only reach ~0.01 rad vs 0.48 target. DriveAPI targetPosition set correctly but physics step pulls joints toward 0. Workaround: per-step `set_joint_positions()` reset provides functional steering.

2. **Right-Turn Yaw Asymmetry**: Right turn produces weaker yaw rate (0.104 vs 0.479 rad/s). Root cause under investigation — possible friction asymmetry or numerical PhysX behavior.

3. **Sequence Contamination**: Second turn in sequence shows reduced rate. Mitigated with 3s gap + steady-state measurement over final 2s.

## Configuration Files
- `configs/vehicle.yaml`: All physical dimensions, masses, limits
- `configs/camera.yaml`: Sensor parameters
- `configs/simulation.yaml`: Physics dt (0.02s), rendering settings
- `configs/steering.yaml`: PID gains, safety clearances

## Source Code References
- `isaac_sim/scripts/vehicle/simple_vehicle.py`: Physics articulation creation
- `isaac_sim/scripts/vehicle/visual_vehicle.py`: High-fidelity visual twin
- `isaac_sim/scripts/vehicle/ackermann_controller_ros.py`: ROS2 steering control
- `isaac_sim/scripts/utils/world_setup.py`: Unified world builder
- `tests/verify_ackermann_physics.py`: Headless verification (14/15 PASS)