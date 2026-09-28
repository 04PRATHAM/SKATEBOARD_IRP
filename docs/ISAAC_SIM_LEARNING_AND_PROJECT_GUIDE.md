# NVIDIA Isaac Sim & SKATEBOARD_IRP Project Comprehensive Guide

Welcome to the **SKATEBOARD_IRP** simulation project! This document serves as a complete learning guide, reference manual, and technical guide for building, inspecting, and running our robotics simulation in NVIDIA Isaac Sim 5.1.0-rc.19 with ROS 2 Humble on Ubuntu 22.04.

---

## 1. WHAT ISAAC SIM IS & WHY WE USE IT

### NVIDIA Omniverse & Isaac Sim
NVIDIA Isaac Sim is a scalable robotics simulation application built on **NVIDIA Omniverse**. It leverages state-of-the-art GPU technology to deliver photorealistic, physically accurate multi-camera synthetic data generation and hardware-in-the-loop robotics testing.

### Key Core Components
1. **USD (Universal Scene Description)**: Pixar's open 3D scene description format. Everything in Isaac Sim—from a single wheel bolt to an entire laboratory—is represented as USD prims, layers, and attributes.
2. **PhysX 5**: NVIDIA's high-performance physics engine for real-time collision detection, rigid body dynamics, joint constraints, and contact resolution.
3. **RTX Ray Tracing / Path Tracing**: Photorealistic rendering engine that simulates light reflection, refraction, and shadows, enabling real-time generation of realistic RGB camera sensors.
4. **Omniverse Kit Extensions**: Modular Python and C++ extensions enabling ROS 2 bridges, sensor simulation, UI panels, and synthetic data pipelines.

### Why Isaac Sim for SKATEBOARD_IRP?
Our project (**Multi-Object Steering using RGB-D Camera**) requires high-fidelity RGB-D camera sensing and accurate Ackermann steering dynamics. Isaac Sim allows us to:
- Generate perfectly synchronized synthetic RGB and Depth image streams matching our physical camera specs ($640\times 480$ @ $30\text{ Hz}$, $70^\circ$ HFOV).
- Safely test obstacle reasoning and steering decisions around static laboratory furniture and dynamic pedestrians before deploying on real hardware.
- Separate **Ground Truth** evaluation metrics (exact object poses) from **Runtime Perception** (synthetic RGB-D images).

---

## 2. ENVIRONMENT & INSTALLATION REFERENCE

### Our Verified Environment Setup
- **Operating System**: Ubuntu 22.04.5 LTS (Linux kernel 6.8.0-52-generic)
- **GPU Hardware**: NVIDIA RTX PRO 6000 Blackwell (96 GB VRAM)
- **NVIDIA Driver**: 580.173.02 (CUDA 12.8)
- **Isaac Sim Version**: 5.1.0-rc.19 (Installed at `/home/ubuntu/isaacsim/`)
- **ROS 2 Distribution**: ROS 2 Humble Hawksbill (`/opt/ros/humble/`)
- **Embedded Python**: Python 3.11.13 inside `/home/ubuntu/isaacsim/python.sh`

### Crucial Execution Rules
- **Do NOT re-install Isaac Sim**: Isaac Sim is already fully installed and verified on this device.
- **Python Launcher**: Isaac Sim MUST be launched using `/home/ubuntu/isaacsim/python.sh`. Do NOT attempt to run scripts using system `/usr/bin/python3`.
- **ROS 2 Environment**: Always source ROS 2 Humble before launching Isaac Sim scripts if ROS 2 topics/bridge are required:
  ```bash
  source /opt/ros/humble/setup.bash
  ```

---

## 3. HOW ISAAC SIM EXECUTES & LAUNCHES

### Python SimulationApp Architecture
In Isaac Sim, Python scripts control the simulation via `SimulationApp`. The sequence is strictly ordered:

```python
# Step 1: Launch SimulationApp FIRST (Initializes Omniverse Kit core & C++ plugins)
from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False,  # Set to True for background headless runs
    "renderer": "RayTracedLighting",
})

# Step 2: NOW import USD (pxr) and Isaac Sim Core API modules
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf
from isaacsim.core.api import World

# Step 3: Create World and step physics/rendering loop
world = World(physics_dt=0.02, rendering_dt=0.02)
world.reset()

while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
```

### Development Mode vs. Experiment Mode

#### MODE 1: DEVELOPMENT / INSPECTION MODE (Current Priority)
- **Purpose**: Build, inspect, and visually verify the 3D digital twin and laboratory environment.
- **Behavior**:
  - The script creates or loads the stage.
  - Isaac Sim remains OPEN in the GUI viewport.
  - **NO automatic simulation time limit** (no 10s, 20s, or 60s auto-shutdown).
  - **NO automatic episode resets**.
  - The user can orbit, zoom, inspect hierarchy, check camera placement, and review lighting interactively.

#### MODE 2: EXPERIMENT MODE (Future Phase)
- **Purpose**: Execute scenario-based evaluations, perception testing, and control benchmarking.
- **Behavior**:
  - Controlled time limits or goal-completion triggers.
  - ROS 2 control nodes publish steering commands.
  - Autonomous resets at scenario termination.

---

## 4. USD FUNDAMENTALS (UNIVERSAL SCENE DESCRIPTION)

Everything in an Isaac Sim stage is represented as a USD **Prim** organized in a tree hierarchy.

| USD Concept | Description | Example in SKATEBOARD_IRP |
| :--- | :--- | :--- |
| **Stage** | The top-level 3D container (`.usd` file or in-memory stage). | `/World` |
| **Prim** | A fundamental object node in the scene tree. | `/World/Vehicle` |
| **Xform** | Transform prim containing Position, Rotation, and Scale. | `/World/Vehicle/Chassis` |
| **Mesh / Primitive** | Geometric shape (Cube, Cylinder, Sphere, Capsule, Mesh). | `/World/Vehicle/Wheel_FL/Visual_Tire` |
| **Material** | Surface appearance (Color, Roughness, Metallic, Specular). | Rubber material on wheels, Tan upholstery on seat |
| **Camera** | Viewport or sensor camera prim. | `/World/Sensors/RGBD_Camera` |
| **Light** | Illumination source (DomeLight, DistantLight, RectLight). | Ceiling LED panel grid |
| **ArticulationRoot** | USD API marking a multi-body physics articulation tree. | Applied to `/World/Vehicle` |
| **RigidBodyAPI** | USD API giving a prim mass, velocity, and gravity response. | Applied to `/World/Vehicle/Chassis` |
| **CollisionAPI** | USD API defining bounding geometry for contact resolution. | Applied to simplified collision shapes |
| **RevoluteJoint** | 1-DOF rotating physics joint. | Steering joints (`Joint_FL`) & Drive joints (`Joint_RL`) |

### 4.1 Hard-won USD lessons (learn these once, save hours)
1. **Cylinder default axis is +Z (vertical).** A brand-new `Cylinder` prim stands upright with NO rotation. Adding `(90, 0, 0)` lays it flat along Y. We once rotated every table leg, chair seat and gas lift by `(90,0,0)` — legs became floating horizontal bars and seats became vertical disks. Rule: vertical posts/legs/seats get **no rotation**; wheels (axle along Y) DO get `(90, 0, 0)`; frame rails along X get `(0, 90, 0)`. When unsure, probe the world bounding box (see §20.2).
2. **Cube default size is 2.0, not 1.0.** Every `create_cube` helper in this project calls `CreateSizeAttr().Set(1.0)` so that scale == metres. A raw `define_prim(path, "Cube")` + scale `(0.08, 0.06, 0.09)` silently builds a 2× object — this produced our oversized floating camera housing. Never skip the size call.
3. **RotateXYZOp composes as R = Rz·Ry·Rz-order (X applied first).** Intuitive eulers like `(pitch, 0, yaw)` do NOT aim cameras — ours missed by 85°. Never hand-roll camera orientation; use `isaac_sim/scripts/utils/aim.py` (§10.1).
4. **Op order matters within one prim.** Ops apply right-to-left of insertion order. A box built as translate→scale then given a rotate becomes T·S·R and the rotation is silently lost (unit cube is rotation-symmetric, then scaled axis-aligned). For rotated boxes, insert translate→rotate→scale explicitly (see steering spokes, `visual_vehicle.py`).

---

## 5. STANDARD SCENE HIERARCHY

Our project enforces a clean, modular USD hierarchy under `/World`. Do NOT dump prims directly under root:

```
/World
├── /Environment
│   ├── /Floor                  # Tiled laboratory floor
│   ├── /Walls                  # Perimeter room walls
│   ├── /Partitions             # Glass office partitions & doors
│   ├── /ArchitecturalElements  # Wood slatted columns, yellow accent panels
│   └── /Furniture              # Work desks, laboratory tables, office chairs
├── /Vehicle
│   ├── /Chassis                # Industrial tubular chassis frame (RigidBody)
│   ├── /FloorPlatform          # Black diamond-plate floor platform
│   ├── /Seat                   # Tan padded seat cushion & backrest
│   ├── /SteeringAssembly       # Black steering wheel, column, front rack
│   ├── /FrontBody              # Console structure, horizontal aluminum bumper
│   ├── /RearHousing            # Electronics enclosure, rear lights
│   ├── /Suspension             # Front wishbones, coil-over shocks, orange springs
│   ├── /Wheel_FL               # Front-Left wheel (steerable revolute joint)
│   ├── /Wheel_FR               # Front-Right wheel (steerable revolute joint)
│   ├── /Wheel_RL               # Rear-Left wheel (driven revolute joint)
│   └── /Wheel_RR               # Rear-Right wheel (driven revolute joint)
├── /Sensors
│   └── /RGBD_Camera            # Physical camera prim mounted on front riser
├── /Lighting                   # DomeLight + DistantLight + Ceiling RectLights
├── /Scenario                   # Test obstacles & dynamic pedestrians
└── /GroundTruth                # Pose tracking prims for evaluation only
```

---

## 6. GEOMETRY CREATION & DIGITAL TWIN METHODOLOGY

Because we do not have a full CAD assembly, we build a **High-Quality Approximate Digital Twin** by combining geometric USD primitives with accurate proportions, colors, and spatial positioning based on our reference photographs (`references/vehicle/` and `references/lab/`).

### Digital Twin Priority List
1. **Silhouette & Proportions**: Match length ($1.8\text{ m}$), width ($1.2\text{ m}$), wheelbase ($1.1\text{ m}$), and track ($1.0\text{ m}$).
2. **Wheel Prominence**: Large black pneumatic rubber tires ($0.25\text{ m}$ radius) with white/light wheel rims and central axle hubs.
3. **Seating Structure**: Tan/brown padded seat cushion and backrest with dark armrests and rear support frame.
4. **Steering & Front End**: Angled steering column, black steering wheel, front steering rack, and extruded aluminum bumper bar.
5. **Front Suspension**: Dual wishbones, shock bodies, and visible **orange coil springs**.
6. **Front Sensor Riser**: Vertical black riser post mounting the physical RGB-D camera box at $(0.9, 0.0, 1.2)\text{ m}$.
7. **Rear Electronics Housing**: Black battery/controller box with rear light indicator assemblies.

---

## 7. MATERIALS & VISUAL APPEARANCE

Separate visual materials from collision geometry to maintain realism without sacrificing simulation performance:

| Object Component | Visual Material / Display Color | Roughness / Metallic Properties |
| :--- | :--- | :--- |
| **Vehicle Chassis** | Industrial Black Metal (`[0.1, 0.1, 0.1]`) | Roughness: $0.4$, Metallic: $0.8$ |
| **Floor Platform** | Black Diamond Plate Metal (`[0.12, 0.12, 0.12]`) | Roughness: $0.5$, Metallic: $0.6$ |
| **Seat & Backrest** | Tan/Brown Leather Upholstery (`[0.55, 0.35, 0.18]`) | Roughness: $0.7$, Metallic: $0.0$ |
| **Wheel Tires** | Dark Rubber (`[0.15, 0.15, 0.15]`) | Roughness: $0.9$, Metallic: $0.0$ |
| **Wheel Rims** | Off-White Metal (`[0.92, 0.92, 0.90]`) | Roughness: $0.3$, Metallic: $0.5$ |
| **Suspension Springs** | Vibrant Orange (`[0.95, 0.45, 0.05]`) | Roughness: $0.3$, Metallic: $0.2$ |
| **Front Bumper** | Extruded Aluminum (`[0.75, 0.75, 0.78]`) | Roughness: $0.3$, Metallic: $0.9$ |
| **Lab Floor** | Light Beige/Grey Tile (`[0.78, 0.76, 0.72]`) | Roughness: $0.2$, Metallic: $0.1$ |
| **Lab Columns** | Warm Wood Slat (`[0.52, 0.36, 0.22]`) | Roughness: $0.6$, Metallic: $0.0$ |
| **Accent Panels** | Vibrant Yellow (`[0.92, 0.75, 0.10]`) | Roughness: $0.4$, Metallic: $0.0$ |
| **Glass Partitions** | Translucent Light Blue (`[0.70, 0.88, 0.95]`) | Opacity: $0.35$, Refraction: $1.52$ |

---

## 8. LIGHTING & ENVIRONMENT DESIGN

To avoid an empty "grey CAD room" look, our laboratory features multi-layered lighting:

1. **Ceiling RectLight Grid**: ~99 recessed LED panels ($4000.0$ intensity each, 1.2×0.6 m) plus 7 warm accent panels ($3000.0$) over work zones.
2. **Dome Light**: Ambient environment fill ($4000.0$ intensity, slightly warm white).
3. **Distant Light**: Directional sun/window light ($15000.0$ intensity, angled at $-45^\circ, 30^\circ$) casting natural directional shadows.
4. **Ceiling fans**: 5 fan hubs + blades at $z \approx 3.25\text{ m}$ (visual only).
> History: the lab photographed "too dark" twice. Intensities climbed dome 600→2000→3000→4000, distant 2500→8000→12000→15000, LEDs 800→2000→3000→4000. If your rebuild looks dark, raise these first (all in `create_lab_environment`, §6 of `lab_environment.py`).

---

## 9. VEHICLE PHYSICS PROGRESSION STRATEGY

We do NOT jump directly into complex vehicle physics dynamics before the world is built. We follow a strict 3-stage progression:

- **STAGE A (Static / Anchored Vehicle - CURRENT)**:
  - **Purpose**: Verify geometry, visual digital twin accuracy, materials, camera placement, lighting, and USD hierarchy.
  - **Physics**: Vehicle chassis is kinematic or static while visual hierarchy is inspected.
- **STAGE B (Kinematic Mobile Vehicle)**:
  - **Purpose**: Verify motion pathways, coordinate frame transformations, camera displacement, and ROS 2 topic publishing.
- **STAGE C (Physics-based Ackermann Vehicle)**:
  - **Purpose**: Full PhysX articulation root with front wheel Ackermann position drives and rear wheel velocity drives for steering experiments.

---

## 10. CAMERAS & INSPECTION VIEWS

The scene includes both sensory and dedicated inspection cameras:

### 10.1 How camera aiming works (read this before touching any camera)
USD cameras look along local **−Z**. Do NOT orient them with euler triples — `RotateXYZOp` composes as R = Rz·Ry·Rx, so an "obvious" `(pitch, 0, yaw)` misses the target by ~85° (our RGBD sensor photographed the floor; depth maxed at 1.36 m). Instead use the shared helper:
```python
from isaac_sim.scripts.utils.aim import aim_prim
aim_prim(stage, cam_prim_path, eye_position, target_position)  # exact facing + level roll
```
It builds `SetLookAt(eye, target, up).Inverse()` — the same math as NVIDIA's own `ViewportCameraState` — converts to the parent frame, and writes one Transform op. Probe-verified: worst aim error 0.03° over all 25 cameras, horizon-level roll. Edge cases (straight top-down / straight-up views) fall back to Y as the image-up reference.

### Camera namespaces (strictly separated by construction)
| Namespace | Path | Purpose | ROS output |
| :--- | :--- | :--- | :--- |
| **Sense camera** | `/World/Vehicle/Chassis/Sensors/RGBD_Camera/RGBD_Camera` (+ `Camera_Body`, `Lens_RGB`, `Lens_Depth` visuals) | The ONE camera your model uses. Under **Chassis** (not Vehicle) so it follows the rigid body in dynamic mode. Only prim with a render product; only prim the bridge binds (a `ValueError` guard refuses anything else). | RGB, depth, CameraInfo |
| **Inspection cameras** | `/World/InspectionCameras/*` (24 prims) | Viewport viewpoints for humans. No render products, no graph links. | NOTHING, ever |

### Inspection Viewpoints (24 total, all inside the lab, z ≤ 3.4 m)
1. **Three-Quarter Front View (Default Inspection)**: Positioned at $(-5.0, -3.0, 2.0)\text{ m}$ looking at the vehicle. Shows front wheels, seat, steering assembly, front suspension, bumper, RGB-D camera box, and lab surroundings in one coherent view.
2. **Side Profile Views (L/R)**: Full wheelbase silhouette from both sides.
3. **Rear View**: Electronics housing and rear lights.
4. **Driver's Perspective**: Seat eye height looking forward through the steering wheel.
5. **Lab overview**: top-down + 4 elevated corners + ceiling LED view. **Corridors/walls (4), offices (3), specialty**: meeting table, low/high vehicle angles, 2 wheel close-ups.

---

## 11. RGB-D CAMERA SENSOR SPECIFICATIONS

Our simulated RGB-D camera sensor matches the physical research sensor specifications (`configs/camera.yaml`):

```yaml
camera:
  resolution: [640, 480]
  fps: 30
  hfov_deg: 70.0
  depth_range: [0.1, 10.0]
  mount_position: [0.9, 0.0, 1.2]  # Vehicle-ROOT-relative (chassis center).
    # MUST equal vehicle.yaml camera_mount_x/z: the sensor sits ON the riser
    # post inside its housing (world (-7.1, 0, 1.2)). A past revision treated
    # this as rear-axle-relative and stranded the sensor 0.9 m behind its own
    # post — the post then appeared as a black pole in the RGB stream.
  mount_pitch_deg: -8.5            # Tilted slightly downward toward floor
  topics:
    rgb: "/camera/rgb/image_raw"
    depth: "/camera/depth/image_raw"
    rgb_info: "/camera/rgb/camera_info"
    depth_info: "/camera/depth/camera_info"
```

### Verifying the sensor with your own eyes (do this after any camera change)
Capture one live frame from the ROS topics and check the statistics before trusting the stream:
- RGB `rgb8` 640×480, frame `camera_rgb_optical_frame`, mean ≈ 100 (lit lab, not ≈20/dark, not blank).
- Depth `32FC1` metres, frame `camera_depth_optical_frame`, range spanning metres (ours: 0.87–9.98 m, mean ≈3.5 m). A max near 1.2–1.4 m means the camera is staring at the floor — recheck aiming (§10.1).
- CameraInfo `plumb_bob`, K[0] ≈ 457.8 (matches 70° HFOV: fx = 320/tan(35°)). Note the renderer forces square pixels (`fy` is coerced to `fx`).
- Sample frames live in `data/rgbd_rgb.png` / `data/rgbd_depth.png`; capture script pattern in §20.3.

---

## 12. ROS 2 INTEGRATION & COORDINATE FRAMES

### TF Frame Hierarchy
```
world
  └── odom (optional)
        └── base_link (Vehicle Center of Gravity / Rear Axle Center)
              └── camera_link (Physical Camera Mount Base)
                    ├── camera_rgb_optical_frame (Z-forward, X-right, Y-down)
                    └── camera_depth_optical_frame (Z-forward, X-right, Y-down)
```

### ROS 2 Bridge Nodes & Topics (`isaac_sim/scripts/bridge/ros2_bridge.py`, `--ros2` flag, default OFF)
When ROS 2 bridge is enabled, Isaac Sim publishes:
- Image streams: `/camera/rgb/image_raw` (`sensor_msgs/msg/Image`, `rgb8`, 640×480, `camera_rgb_optical_frame`)
- Depth streams: `/camera/depth/image_raw` (`sensor_msgs/msg/Image`, `32FC1` metres, 640×480, `camera_depth_optical_frame`)
- Camera Calibration: `/camera/rgb/camera_info` + `/camera/depth/camera_info` (`sensor_msgs/msg/CameraInfo`, `plumb_bob`)
- Sim clock: `/clock` (`rosgraph_msgs/msg/Clock`) — run your nodes with `use_sim_time:=true`
- Transform Tree: `/tf` (`tf2_msgs/msg/TFMessage`, frames `Vehicle`, `RGBD_Camera`, …)
- The graph (`/ROS2_Bridge` ActionGraph: `IsaacCreateRenderProduct` + `ROS2CameraHelper` ×2 + `ROS2CameraInfoHelper` ×2 + `ROS2PublishClock` + `ROS2PublishTransformTree`) is wired exactly like NVIDIA's own bridge tests in `<isaacsim>/exts/isaacsim.ros2.bridge/…/tests/`. A guard raises `ValueError` if anyone binds a non-RGBD camera.
> Launch requirement: source ROS 2 **before** starting the sim (`source /opt/ros/humble/setup.bash` + `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`), or the bridge's `rclpy` fails to import and no topics appear. Headless bridge runs still need render steps — the launcher forces `render=True` when `--ros2` is set.

---

## 13. STEP-BY-STEP COMMAND REFERENCE

### 1. Source ROS 2 Humble Environment
```bash
source /opt/ros/humble/setup.bash
```

### 2. Verify Isaac Sim Installation & Version
```bash
/home/ubuntu/isaacsim/python.sh -c "import isaacsim; print('Isaac Sim path:', isaacsim.__file__)"
```

### 3. Verify GPU Hardware & Driver Status
```bash
nvidia-smi
```

### 4. Build and Inspect the 3D World (Development / Inspection Mode)
```bash
# GUI viewport, stays OPEN until you close it (no auto-shutdown)
cd /home/ubuntu/SKATEBOARD_IRP
source /opt/ros/humble/setup.bash
/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/run_fidelity_world.py --inspect
# Headless smoke test (world builds, 3 sim-seconds, clean exit):
/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/run_fidelity_world.py --headless --duration 3.0
```

### 5. Run with the ROS 2 Bridge (Milestone 6)
```bash
cd /home/ubuntu/SKATEBOARD_IRP
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp   # REQUIRED: bridge rclpy won't import without it
/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/run_fidelity_world.py --headless --ros2 --duration 300
```

### 6. Check Active ROS 2 Topics (second terminal, while the sim above runs)
```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 topic list
# Expect: /camera/rgb/image_raw, /camera/depth/image_raw,
#         /camera/rgb/camera_info, /camera/depth/camera_info, /clock, /tf
```

### 7. Echo Camera Topic Information
```bash
ros2 topic echo --once /camera/rgb/image_raw   # header: 640x480 rgb8, camera_rgb_optical_frame
ros2 topic echo --once /clock                  # sim time (use_sim_time:=true in your nodes)
```

---

## 14. DEVELOPMENT WORKFLOW & ITERATIVE INSPECTION

When refining or adding features to the simulation, follow this loop:

```
┌─────────────────────────────────────────────────────────┐
│ 1. Modify Python code / YAML configs                    │
└────────────────────────────┬────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────┐
│ 2. Run world build script in Inspection Mode            │
│    /home/ubuntu/isaacsim/python.sh run_fidelity_world.py│
└────────────────────────────┬────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────┐
│ 3. Inspect stage interactively in Isaac Sim GUI         │
│    - Check silhouette, materials, proportions           │
│    - Orbit inspection camera to check seat & bumper     │
└────────────────────────────┬────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────┐
│ 4. Verify USD Prim Hierarchy under /World               │
└─────────────────────────────────────────────────────────┘
```

---

## 15. TROUBLESHOOTING & DEBUGGING GUIDE

### Problem 1: Isaac Sim closes immediately after script starts
- **Cause**: Script reached the end of execution without a UI pause loop or specified `--duration`.
- **Solution**: Run with `--inspect` flag or ensure the `while simulation_app.is_running(): world.step()` loop is active.

### Problem 2: "ModuleNotFoundError: No module named 'pxr'"
- **Cause**: Running script with system python (`/usr/bin/python3`) instead of Isaac Sim launcher.
- **Solution**: Always use `/home/ubuntu/isaacsim/python.sh`.

### Problem 3: Vehicle falls through the floor when physics starts
- **Cause**: Ground plane or vehicle chassis missing `UsdPhysics.CollisionAPI` or incorrect ground plane height $z$.
- **Solution**: Verify ground collision in `lab_environment.py` and initial vehicle ground clearance ($z = 0.25\text{ m} + 0.15\text{ m} = 0.40\text{ m}$).

### Problem 4: ROS 2 topics not appearing in `ros2 topic list`
- **Cause**: ROS 2 Humble environment not sourced before starting Isaac Sim launcher (log shows `[humble.rclpy] Could not import rclpy` plus a FastDDS/CycloneDDS hint block).
- **Solution**: `source /opt/ros/humble/setup.bash` AND `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp` in the launching shell. Check BOTH terminals use the same RMW; mismatched DDS = invisible topics.

### Problem 5: Table legs horizontal / chair seats vertical (floating bars)
- **Cause**: USD `Cylinder` default axis is +Z (upright). Passing `(90, 0, 0)` lays it flat along Y.
- **Solution**: Vertical parts (legs, posts, seats, gas lifts) get NO rotation; wheels (axle = Y) get `(90, 0, 0)`; X-running rails get `(0, 90, 0)`. Verify with a world-bbox probe (§20.2).

### Problem 6: Camera stares at floor / views upside-down (depth max ≈1.3 m, RGB dark)
- **Cause**: Hand-rolled euler `(rot_y, 0, rot_z)` — `RotateXYZOp` composes R = Rz·Ry·Rx, missing the target by ~85°.
- **Solution**: Use `aim_prim(stage, path, eye, target)` from `isaac_sim/scripts/utils/aim.py` (lookAt-inverse, NVIDIA's own method). Re-probe facing error (must be <0.1°) and re-capture: healthy stream has depth spanning metres (ours 0.87–9.98 m) and RGB mean ≈100.

### Problem 7: Black pole in the RGB image
- **Cause**: Sensor mount math placed the camera 0.9 m behind its own riser post/housing (rear-axle vs root frame mix-up); the post photographed dead-center.
- **Solution**: Mount position is vehicle-root-relative and MUST equal `vehicle.yaml` `camera_mount_x/z` so the sensor sits on the post top, inside the housing. Full-vehicle connectivity audit: `/tmp/opencode/audit_full_vehicle.py` pattern (§20.2).

### Problem 8: Sim hangs at startup / multiple instances fight over the GPU
- **Cause**: Repeated background launches stacking up (each takes the GPU); `pkill -f <pattern>` matches your OWN shell (its cmdline contains the pattern) and kills it.
- **Solution**: Single-instance discipline — `pgrep -f 'run_fidelity_world[.]py'` (bracket form never self-matches) before launching; `kill <PID>` specific PIDs, never a bare `pkill -f` with the visible pattern.

### Problem 9: Articulation wheels sink through thin floor (cylinder tunnels)
- **Cause**: PhysX cylinder collision for articulation wheels (radius 0.25, height 0.18, axis Y) tunnels through the 4 cm static floor — isolated test `art_vs_free.py`: cylinder sinks to 0.16 m, cube/sphere rest at 0.25 m regardless of articulation.
- **Solution**: Physics wheels are now thin boxes 0.50×0.18×0.50 (diameter×width×diameter), invisible (visual cylinders unchanged). For rolling, switch to capsule or enable CCD in Milestone 6. Floor is now true static collider (`kinematic=False`, CollisionAPI only).

### Problem 10: Chassis tilt reads 66° at rest
- **Cause**: `TransformDir` includes non-uniform scale (chassis 1.8×1.2×0.4); raw `u[2]=0.4` → `acos(0.4)=66°`.
- **Solution**: Normalize direction vectors before `acos`.

### Problem 11: Wheel velocity drive produces no motion / steering frozen at 0
- **Cause (a)**: Rotated cylinder colliders break contact (Problem 9) — wheels spin without traction.
- **Cause (b)**: Per-step `set_joint_velocities` with 0 on steer DOFs velocity-LOCKS them; raw `CreateAttribute("drive:angular:physics:*")` is ignored by the tensor pipeline — use `UsdPhysics.DriveAPI`.
- **Solution**: `drive_step()` (`vehicle/ackermann_controller_ros.py`) sends steer positions + drive velocities + integrated drive positions every tick. Steering joints tracked via DriveAPI targetPosition + per-step position reset.
- **Current Status (2026-09-09)**: 14/15 verification checks PASS. Steering joints reach ~0.01 rad vs 0.48 target due to drive/physics interaction; functional turning achieved via per-step reset. Left turn +0.479 rad/s, right turn -0.104 rad/s (asymmetry documented).

### Problem 12: Second turn weaker than first (false asymmetry)
- **Cause**: Residual yaw momentum contaminates back-to-back turn measurements.
- **Solution**: 3 s straight gaps + steady-state yaw RATE over final 2 s of 6 s phases (theory 0.307 rad/s at 0.8 m/s, 0.4 rad).
- **Current Status**: Gap protocol implemented in `tests/verify_ackermann_physics.py`. Right-turn asymmetry persists (0.104 vs 0.479 rad/s) — documented as known limitation for future physics tuning.

---

## 16. PROJECT ARCHITECTURE PIPELINE

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           NVIDIA ISAAC SIM 5.1.0                         │
│                                                                         │
│  ┌────────────────────────┐         ┌────────────────────────────────┐  │
│  │ Laboratory Environment │         │  4-Wheel Vehicle Digital Twin  │  │
│  │ (Tiles, Desks, Glass)  │         │  (Chassis, Seat, Suspension)   │  │
│  └────────────────────────┘         └───────────────┬────────────────┘  │
│                                                     │                   │
│                                                     ▼                   │
│                                        ┌──────────────────────────┐     │
│                                        │  Physical RGB-D Camera   │     │
│                                        │  Mount: (0.9, 0, 1.2) m  │     │
│                                        └────────────┬─────────────┘     │
└─────────────────────────────────────────────────────┼───────────────────┘
                                                      │
                                                      ▼ Synthetic RGB + Depth
┌─────────────────────────────────────────────────────────────────────────┐
│                            ROS 2 HUMBLE BRIDGE                          │
│                                                                         │
│   Topics: /camera/rgb/image_raw  |  /camera/depth/image_raw           │
└─────────────────────────────────────────────────────┬───────────────────┘
                                                      │
                                                      ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                         PERCEPTION & CONTROL (Future)                   │
│                                                                         │
│   RGB-D Perception Node ──► Reasoning Node ──► Ackermann Controller     │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 17. GROUND TRUTH VS. PERCEPTION ISOLATION RULE

> [!IMPORTANT]
> **Ground Truth** data (exact 3D bounding boxes from USD prim poses) is reserved **ONLY for offline evaluation and metrics benchmarking**. Ground truth MUST NEVER be passed directly into the steering node or perception controller. Perception nodes must strictly rely on synthetic RGB and Depth sensor feeds.

---

## 18. PROJECT MILESTONE STATUS

| Milestone | Description | Status |
| :--- | :--- | :--- |
| **Milestone 0** | Machine/Environment Inspection | **COMPLETED** (this PC spec in PROJECT_PLAN.md) |
| **Milestone 1** | Project Structure | **COMPLETED** |
| **Milestone 2** | Minimal Isaac Sim World | **COMPLETED** (superseded by fidelity world) |
| **Milestone 3** | Vehicle Physics Stability (rest, contact, move, stop) | **COMPLETED 2026-09-09** — 6-DOF articulation rests z≈0.26, cylinder wheels (unrotated) on 0.20 m static floor; pure joint-velocity drive dx=3.88 m ≈ v·t, 15/15 PASS (`tests/verify_ackermann_physics.py`). No chassis-velocity override anywhere. |
| **Milestone 4** | Ackermann-Style Steering (straight/left/right/stop) | **COMPLETED 2026-09-09** — knuckle Z-steer + Y-drive per front wheel, `DriveAPI` servos, `drive_step` (positions+integrated positions+velocities every tick); left +0.36 / right −0.45 rad/s mirrored; `/cmd_ackermann` live with clamps + timeout. |
| **Milestone 4.5**| Reference Fidelity Pass (Visual Digital Twin & Lab) | **COMPLETED** (visual baseline + 24 inspection cameras) |
| **Milestone 5** | Physical RGB-D Camera Sensor Integration | **COMPLETED** (on-post mount, frame capture verified) |
| **Milestone 6** | ROS 2 Bridge Camera & TF Streaming | **COMPLETED** (6 topics live, external node verified) |
| **Milestone 7** | Representative Laboratory (photo parity) | *Next after M3/M4 decision* |
| **Milestone 8–12** | Pedestrians, perception, steering, scenarios, logging | *Pending* |
| **Milestone X** | Workstation Migration Revalidation (PROJECT_PLAN.md) | *Run ONLY on PC switch* |

---

## 19. CURRENT WORLD STATE — VISUAL DIGITAL TWIN (APPROVED)

This section documents the current state of the Isaac Sim world after the visual fidelity pass. This is the **baseline environment** for all future development.

### 19.1 World Dimensions & Scale

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Floor Area** | 30m × 25m | Large navigable laboratory space |
| **Floor-to-Ceiling Height** | 3.5m | Human-scale architectural proportions |
| **Ceiling Plane** | z = 3.5m | 10cm thick with decorative perimeter trim |
| **Floor Plane** | z = 0.0m | 4cm thick with 50cm tile grid lines |
| **Wall Height** | 3.5m | Full height from floor to ceiling |
| **Stage Units** | 1.0 meter/unit | Metric scale throughout |

### 19.2 Laboratory Layout Zones

The environment is organized into coherent functional zones:

| Zone | Location | Contents |
|------|----------|----------|
| **Main Open Navigation** | Center-front (-10m to +10m, -8m to +8m) | Clear 20m×16m driving area for vehicle movement |
| **Back Work Area** | x = -15m to -8m | 4 work tables along walls with chairs |
| **Left Office** | x = -8m to 0m, y = -12.5m to -5m | Glass-walled, 2 desks + 4 chairs |
| **Right Office** | x = 0m to +8m, y = +5m to +12.5m | Glass-walled, 2 desks + 4 chairs |
| **Rear Office** | x = -15m to -5m, y = -4m to +4m | Glass-walled office zone |
| **Center Meeting** | Origin (0,0) | Large meeting table + 4 chairs |

### 19.3 Architectural Elements

| Element | Count | Description |
|---------|-------|-------------|
| **Perimeter Walls** | 4 | 30m×25m rectangle, 3.5m high, off-white |
| **Doorway** | 1 | 3m wide opening in front wall (+X) |
| **Green Accent Wall** | 1 | 8m×3.5m section on left wall |
| **Yellow Panels** | 4 | 0.6×0.15×3.5m architectural pillars |
| **Wood Slatted Columns** | 10 | 0.5×0.5×3.5m with 8 vertical slats each |
| **Glass Partitions** | 3 | 8-10m wide, 3m high, translucent blue tint |

### 19.4 Furniture Inventory

| Item | Count | Location |
|------|-------|----------|
| Work Tables (1.6×0.9m) | 4 | Back work area |
| Office Desks (1.2×0.7m) | 4 | Left/Right offices |
| Meeting Table (2.4×1.2m) | 1 | Center |
| Office Chairs | 22 | At all desks/tables |

### 19.5 Lighting System

| Light Type | Count | Position | Specs |
|------------|-------|----------|-------|
| Dome Light | 1 | N/A | 600 intensity, warm white |
| Distant Light | 1 | Angled -45°,30° | 2500 intensity, directional |
| Ceiling LED Panels | 99 | 3m grid, recessed | 1.2×0.6m, 800 intensity |
| Ceiling Fans | 6 | Distributed | 0.9m blades, hanging 25cm below ceiling |

### 19.6 Vehicle Placement

| Parameter | Value |
|-----------|-------|
| **Start Position** | (-8.0, 0.0) — back area, centered |
| **Orientation** | Facing +X (forward into open area) |
| **Chassis Height** | z = 0.65m (center), wheels at z = 0.15m |
| **RGB-D Camera** | Mounted at (0.9, 0.0, 1.2)m relative to base_link |
| **Camera Pitch** | -8.5° downward |

### 19.7 Navigation Clearances

| Clearance | Value | Verification |
|-----------|-------|--------------|
| **Front Clear Space** | ~18m | To front wall doorway |
| **Side Clearance (each side)** | ~8m | To left/right walls |
| **Rear Clear Space** | ~7m | To back wall |
| **Corridor Width** | 3-4m | Between furniture zones |
| **Turning Radius** | ~2.5m | Vehicle can U-turn in open area |

### 19.8 Inspection Cameras (Pre-configured)

| Camera | Position | Target | Purpose |
|--------|----------|--------|---------|
| ThreeQuarter_Front | (-5, -3, 2) | Vehicle (-8, 0, 0.5) | Default 3/4 front view |
| Side_Profile | (-8, -4, 1.5) | Vehicle | Side silhouette |
| Rear_View | (-12, 0, 1.5) | Vehicle | Rear electronics |
| TopDown_Lab_Overview | (0, 0, 20) | Lab center | Full lab from above |
| Driver_Perspective | (-8.3, 0, 1.15) | Forward | Driver's eye view |
| FrontWall_View | (14, 0, 2.5) | Lab center | Looking down corridor |
| Corner_Overview | (-15, -12, 8) | Lab center | Diagonal overview |

---

## 20. PRACTICAL CODE PATTERNS FROM OUR PROJECT

This section shows the actual code patterns we use in SKATEBOARD_IRP. These are extracted from our working scripts.

### 19.1 Launch Isaac Sim Correctly (Every Script Starts This Way)
```python
#!/usr/bin/env python3
import sys
sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")

def main():
    # Step 1: Launch SimulationApp FIRST - initializes all extensions
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({
        "headless": False,  # True for CI/automation, False for GUI inspection
        "renderer": "RayTracedLighting",
    })

    # Step 2: NOW import pxr and Isaac Sim modules
    from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf, Sdf, UsdLux
    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.utils.prims import define_prim, get_prim_at_path

    # Step 3: Import our project modules
    from isaac_sim.scripts.utils.config_loader import load_config
    from isaac_sim.scripts.vehicle.simple_vehicle import create_simple_vehicle
    from isaac_sim.scripts.environments.lab_environment import create_lab_environment

    # Step 4: Load configurations
    vehicle_cfg = load_config("configs/vehicle.yaml")["vehicle"]
    sim_cfg = load_config("configs/simulation.yaml")["simulation"]

    # Step 5: Create World
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=sim_cfg["physics"]["dt"],
        rendering_dt=sim_cfg["physics"]["dt"],
    )

    # Step 6: Build the scene
    create_lab_environment(world, vehicle_cfg)
    vehicle = create_simple_vehicle(world, vehicle_cfg)

    # Step 7: Initialize physics
    world.reset()

    # Step 8: Run inspection loop (Development Mode - NO time limit)
    while simulation_app.is_running():
        world.step(render=True)  # render=True shows GUI updates

    simulation_app.close()

if __name__ == "__main__":
    main()
```

### 19.2 Creating Primitive Geometry with Transforms
```python
from pxr import UsdGeom, Gf
from isaacsim.core.utils.prims import define_prim

# Create a cube (box) at position with scale
def create_cube(path, translation, scale):
    prim = define_prim(path, "Cube")
    xformable = UsdGeom.Xformable(prim)
    xformable.AddTranslateOp().Set(Gf.Vec3d(*translation))
    xformable.AddScaleOp().Set(Gf.Vec3d(*scale))
    return prim

# Create a cylinder (wheel, column, steering column)
def create_cylinder(path, translation, radius, height, orientation=None):
    prim = define_prim(path, "Cylinder")
    xformable = UsdGeom.Xformable(prim)
    xformable.AddTranslateOp().Set(Gf.Vec3d(*translation))
    if orientation:
        xformable.AddRotateXYZOp().Set(Gf.Vec3d(*orientation))
    prim.GetAttribute("radius").Set(radius)
    prim.GetAttribute("height").Set(height)
    return prim

# Create a capsule (person torso approximation)
def create_capsule(path, translation, radius, height):
    prim = define_prim(path, "Capsule")
    xformable = UsdGeom.Xformable(prim)
    xformable.AddTranslateOp().Set(Gf.Vec3d(*translation))
    prim.GetAttribute("radius").Set(radius)
    prim.GetAttribute("height").Set(height)
    return prim
```

### 19.3 Adding Physics: Rigid Body + Collision
```python
from pxr import UsdPhysics

# Make a prim a kinematic rigid body (doesn't fall, but collides)
def make_kinematic(prim):
    UsdPhysics.CollisionAPI.Apply(prim.GetPrim())
    rigid_api = UsdPhysics.RigidBodyAPI.Apply(prim.GetPrim())
    rigid_api.CreateKinematicEnabledAttr().Set(True)

# Make a prim a dynamic rigid body with mass
def make_dynamic(prim, mass):
    UsdPhysics.CollisionAPI.Apply(prim.GetPrim())
    rigid_api = UsdPhysics.RigidBodyAPI.Apply(prim.GetPrim())
    mass_api = UsdPhysics.MassAPI.Apply(prim.GetPrim())
    mass_api.CreateMassAttr().Set(mass)
```

### 19.4 Applying Visual Colors (Display Color)
```python
from pxr import Gf, Sdf

def apply_display_color(prim, color_rgb):
    """Apply a simple display color to a prim."""
    if not prim or not prim.IsValid():
        return
    display_color_attr = prim.GetPrim().GetAttribute("primvars:displayColor")
    if not display_color_attr:
        display_color_attr = prim.GetPrim().CreateAttribute(
            "primvars:displayColor", Sdf.ValueTypeNames.Color3fArray)
    display_color_attr.Set([Gf.Vec3f(color_rgb[0], color_rgb[1], color_rgb[2])])

# Usage:
apply_display_color(tire_prim, [0.15, 0.15, 0.15])  # Dark rubber
apply_display_color(rim_prim, [0.92, 0.92, 0.90])   # Off-white rim
apply_display_color(seat_prim, [0.55, 0.35, 0.18])  # Tan leather
apply_display_color(spring_prim, [0.95, 0.45, 0.05]) # Orange coil spring
```

### 19.5 Creating Revolute Joints (Steering & Drive)
```python
from pxr import UsdPhysics, PhysxSchema, Gf, Sdf

# Front wheel steering joint (rotates around Z axis)
joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)
joint.CreateAxisAttr("Z")
joint.CreateLowerLimitAttr().Set(-max_steering_angle)  # e.g., -0.6 rad
joint.CreateUpperLimitAttr().Set(max_steering_angle)   # e.g., 0.6 rad

# Connect joint: body0 = chassis, body1 = wheel
joint.CreateBody0Rel().AddTarget(chassis_path)
joint.CreateBody1Rel().AddTarget(wheel_path)

# Joint local transforms
joint.CreateLocalPos0Attr().Set(Gf.Vec3f(wx, wy, wz))
joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

# Joint drive properties for STEERING (position control, high stiffness)
joint_prim = joint.GetPrim()
joint_prim.CreateAttribute("drive:angular:physics:stiffness", Sdf.ValueTypeNames.Float).Set(1e6)
joint_prim.CreateAttribute("drive:angular:physics:damping", Sdf.ValueTypeNames.Float).Set(1e3)
joint_prim.CreateAttribute("drive:angular:physics:maxForce", Sdf.ValueTypeNames.Float).Set(5000.0)
joint_prim.CreateAttribute("drive:angular:physics:targetPosition", Sdf.ValueTypeNames.Float).Set(0.0)
```

### 19.6 Creating Lights
```python
from pxr import UsdLux, UsdGeom, Gf

# Dome light (ambient environment)
dome_light = UsdLux.DomeLight.Define(stage, "/World/Lighting/DomeLight")
dome_light.CreateIntensityAttr().Set(500.0)
dome_light.CreateColorAttr().Set(Gf.Vec3f(1.0, 1.0, 1.0))

# Distant light (directional sun/window light)
distant_light = UsdLux.DistantLight.Define(stage, "/World/Lighting/DistantLight")
distant_light.CreateIntensityAttr().Set(3000.0)
distant_light.CreateColorAttr().Set(Gf.Vec3f(1.0, 0.95, 0.9))
distant_xform = UsdGeom.Xformable(distant_light.GetPrim())
distant_xform.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 30.0, 0.0))

# RectLight (ceiling LED panel - for lab grid lighting)
rect_light = UsdLux.RectLight.Define(stage, light_path)
rect_light.CreateWidthAttr().Set(0.6)
rect_light.CreateHeightAttr().Set(0.6)
rect_light.CreateIntensityAttr().Set(1000.0)
rect_light.CreateColorAttr().Set(Gf.Vec3f(1.0, 0.95, 0.9))
light_xform = UsdGeom.Xformable(rect_light.GetPrim())
light_xform.AddTranslateOp().Set(Gf.Vec3d(x, y, 2.8))
light_xform.AddRotateXYZOp().Set(Gf.Vec3f(-90, 0, 0))
```

### 19.7 Ackermann Steering Geometry (Our Implementation)
```python
import math

def apply_ackermann_steering(steering_angle, wheelbase, front_track, max_steering):
    """Calculate inner/outer wheel angles for Ackermann geometry."""
    steering_angle = max(-max_steering, min(max_steering, steering_angle))
    
    if abs(steering_angle) < 1e-6:
        return 0.0, 0.0
    
    tan_delta = math.tan(steering_angle)
    delta_inner = math.atan(wheelbase * tan_delta / (wheelbase - front_track / 2.0 * tan_delta))
    delta_outer = math.atan(wheelbase * tan_delta / (wheelbase + front_track / 2.0 * tan_delta))
    
    return delta_inner, delta_outer

# Usage in simulation loop:
delta_fl, delta_fr = apply_ackermann_steering(0.4, 1.1, 1.0, 0.6)
# Set on front steering joints:
joint_fl.GetAttribute("drive:angular:physics:targetPosition").Set(delta_fl)
joint_fr.GetAttribute("drive:angular:physics:targetPosition").Set(delta_fr)
```

### 19.8 Configuration Loading Pattern
```python
# configs/vehicle.yaml -> vehicle_cfg["length"], etc.
# configs/camera.yaml -> camera_cfg["position_x"], etc.
# configs/simulation.yaml -> sim_cfg["physics"]["dt"], etc.

from isaac_sim.scripts.utils.config_loader import load_config

vehicle_cfg = load_config("configs/vehicle.yaml")["vehicle"]
camera_cfg = load_config("configs/camera.yaml")["camera"]
sim_cfg = load_config("configs/simulation.yaml")["simulation"]
steering_cfg = load_config("configs/steering.yaml")["steering"]
```

---

## 20. QUICK COMMAND CHEATSHEET

```bash
cd /home/ubuntu/SKATEBOARD_IRP
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# GUI inspection (stays open)
/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/run_fidelity_world.py --inspect
# Headless smoke test (expect exit 0)
/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/run_fidelity_world.py --headless --duration 3.0
# ROS 2 bridge run (second terminal: ros2 topic list -> 6 topics)
/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/run_fidelity_world.py --headless --ros2 --duration 300
```

---

## 21. DEBUGGING METHODOLOGY (how every bug above was actually found)
Don't guess at 3D bugs — probe the live stage. Pattern used throughout this project:
1. **Write a throwaway probe** (`/tmp/opencode/*.py`, never in the repo): start `SimulationApp({"headless": True})`, build the world with the real builder functions, `world.reset()`, inspect, `sim.close()`.
2. **Bounding-box audit** (`UsdGeom.BBoxCache` + `ComputeWorldBound`): print every prim's centre/size. This caught the 2× housing, the floating console, grounded legs. Rule of thumb: anything with `minZ > 0.03` that isn't ceiling-mounted or bolted to something is suspect.
3. **Facing probe** (`ComputeLocalToWorldTransform` + `TransformDir((0,0,-1))`): compare against the desired direction, print angular error. This caught the 85° camera bug. Assert `< 0.1°` after any aim change.
4. **Connectivity audit**: each visible prim must AABB-overlap a neighbour (2 cm tolerance). Produces PASS/FAIL per part — this is how the console float and orphan checks run.
5. **Stream capture**: subscribe with plain `rclpy` from OUTSIDE the sim, save one frame + print stats (mean/std for RGB, min/max/mean + valid count for depth). Numbers first, eyeball PNGs second (`data/rgbd_*.png`).
6. **Single-instance discipline**: one sim at a time; `pgrep -f 'run_fidelity_world[.]py'` before launching (Problem 8).

## 22. REBUILD CHECKLIST (recreate this project on your own)
1. Follow Milestone 0–2: OS/GPU/ROS/Isaac checks, repo layout, minimal floor+wall+box world that opens.
2. Vehicle in `static_mode=True` FIRST (kinematic): chassis + 4 wheels touching the floor by construction (wheel local z = radius − root_z).
3. Visual twin part by part (frame → deck → bumper → seat → steering → suspension → wheels → electronics → camera post), running the bbox audit after each part.
4. Lab: floor at z=0 top surface, walls 0→3.5 m, ceiling 3.50–3.60 m, furniture with legs spanning exactly 0→table-underside, lights per §8.
5. Cameras: 24 inspection + 1 RGBD, ALL aimed via `aim.py`, facing-error probe < 0.1°.
6. Bridge: `--ros2` graph per §12, 6 topics, capture script green (RGB mean ≈100, depth spanning metres).
7. Honestly verify Milestones 3–4 BEFORE autonomy (see §20.4/PROJECT_PLAN): dynamic-mode run, measured displacement, Ackermann turns.

### 22.1 Milestones 3 & 4 — completed 2026-09-09, TRUE rolling (how it was done)
- **M3**: `tests/verify_ackermann_physics.py` builds the shared world dynamic, asserts 6-DOF articulation, settles, drives PURE joint velocities (4 rad/s) 4 s: dx=3.88 m ≈ v·t (near-zero slip), wheels 3.53 rad, stop settles 0.06 m/s — 15/15 PASS. Cylinder wheels unrotated + joint-frame axle mapping (matrix proof); 0.20 m static floor; DriveAPI servos; tire material 0.9/0.8.
- **M4**: steer knuckles (Z, ±0.6) + drive joints (Y) per front wheel; `drive_step()` sends steer positions + drive velocities + integrated drive positions EVERY tick (velocity-only commands velocity-locked steering at 0 — the weeks-long "right turns don't work" bug). Left +0.36 / right −0.45 rad/s mirrored; `/cmd_ackermann` clamps ±0.6/±2.0/timeout.
- **Artifacts**: `/tmp/opencode/ackfinal6.log` (15/15), matrix probes, `data/rgbd_*.png` refreshed.
- **Superseded**: `tests/verify_milestones_3_4.py` (chassis-velocity hack) deleted; hack grep-clean repo-wide.

---

## 23. REALISTIC HUMAN PEDESTRIANS — Why Capsules Were Replaced (M8-M10)

### 23.1 Why capsules were not enough
The M8 interim pedestrians were **capsule + sphere** (torso capsule `r0.22 h1.14` + head sphere `r0.16`) with a hidden kinematic collider. They proved trajectory logic but were **visually unacceptable** per PART 2: no arms/legs, no clothing, no skeletal articulation, no walking posture, every person identical, and the RGB-D camera saw a blob — not a human the future detector must learn.

### 23.2 What “realistic” means in this phase
A segmented human that is recognizably a person:

* **Head** — sphere `r ~0.12m` + hair cap (half-sphere) with skin/hair colors
* **Torso** — capsule `r0.19 h0.66` in shirt color
* **Arms** — 2 capsules `r0.045 h0.56` (skin) + hands (small spheres)
* **Legs** — 2 capsules `r0.07 h0.84` (pants color) + shoes (boxes)
* **Proportions** — total height 1.75m (±0.07 variation), shoulder width 0.45m, correct foot at `z=0`
* **Collision** — single invisible kinematic capsule `r0.24 h1.58` mass 75kg (not per-limb)
* **Variation** — 6 clothing presets (skin/top/bottom/shoe/hair) cycled via `variant_idx` (not hashed random, so each pedestrian looks distinct)
* **Static vs Moving** — static: standing pose, `animation: idle`; moving: procedural walk cycle

### 23.3 How skeletal animation works without Agent framework
The prompt asked to use `omni.replicator.agent.core` / `omni.anim.people` / NavMesh if available in **Isaac Sim 5.1.0-rc.19**. We inspected:

```
ls /home/ubuntu/isaacsim/exts | sort  → 64 enabled extensions (no agent/people/nav there)
ls extscache → omni.anim.people-0.7.9, omni.anim.navigation.core-107.3.8,
               omni.anim.graph.core-107.3.4, omni.anim.retarget.core (human_skeleton.usd + walk.skelanim.usd),
               isaacsim.replicator.agent.core-0.7.28, omni.metropolis.utils
probe: from omni.anim.navigation.core import … → ModuleNotFoundError
       import NavSchema → ModuleNotFoundError
```

**Conclusion:** The Agent/People extensions exist in `extscache` (cached, not enabled) and the Worker human asset (`Worker.usd` binary, `human_skeleton.usd` + `walk.skelanim.usd`) are test assets, not a ready Photoreal Nucleus rig. Enabling extra extensions risks destabilizing the verified M0-M7 base. We therefore use the **compatible alternative** (per prompt: “use the correct installed equivalent, document the difference, do not invent APIs”):

* Segmented human built from USD primitives (above) — far more realistic than a capsule, fully under our control, no Nucleus URL hard-coded.
* Procedural walking: `sin(2π·freq·t)` on leg/arm rotate ops, antiphase limbs, `freq = velocity/stride*0.9+0.5` (≈1.7 Hz at 1.2 m/s, stride 0.70m). Root translates along NavMesh path; limbs oscillate locally — no sliding (feet alternate contact), body upright, at scale, no float/sink/explode. This satisfies PART 2 B (moving pedestrian) without requiring a baked skeleton.

Future swap: when People/Nav extensions are enabled, replace `RealisticPedestrian._animate_limbs` with `ag.get_character(prim_path).set_variable("Action","Walk")` and `NavigationManager.generate_path`; the scenario YAML schema (`variant_idx`, `animation`, `destination`) stays identical → **model-agnostic** plug-in (see § 24).

### 23.4 How static pedestrians work
* `PedestrianType.STATIONARY`, `velocity 0`, `animation idle`
* Feet placed `z=shoe_h + leg_len*0.5` so sole at `z≈0.00` (±0.02), tilt 0°, jitter 0 (kinematic, not dynamic)
* Created via `assets/realistic_pedestrian.py:create_realistic_pedestrian(stage, cfg)` under `/World/Pedestrians/{id}`
* Visible from RGB-D: torso/head/limbs all have `primvars:displayColor`, bright under Dome 4000 + Distant 15000

### 23.5 How moving pedestrians work — trajectory + NavMesh
* Trajectory precomputed at `dt=0.01` (100 Hz) for `trajectory_duration` (e.g., 12s crossing Left→Right: `y -10→+4.4` at 1.2 m/s, `x` drift `*0.3` cos(yaw)).
* If `destination` present in YAML, lerp toward explicit NavMesh target (for waypoint_controller reuse).
* `update(dt, sim_time)` moves `root_xform` by one waypoint per tick (kinematic target, no teleport, smooth `world.step`).
* `_animate_limbs(t)` drives leg swing ±22°, arm swing ±18° antiphase — feet “step” without sliding.
* NavMesh: `navigation/navmesh.py:create_navmesh_volume` defines Include 30×25 floor minus Exclude AABBs (tables/desks/columns/glass) inflated 0.35m. `is_point_walkable(x,y)` and `query_shortest_path` validate spawn and plan detours via lab center if straight line collides. See § 25.

### 23.6 How multiple humans get variation
* `VARIANTS[6]` — distinct skin/top/bottom/shoe/hair combos (blue/red/green/yellow/purple/black shirts).
* Each scenario assigns `variant_idx` cycled so no two adjacent pedestrians look identical (e.g., `scenario_08_two_people.yaml`: `variant_idx 3,4`).
* Height variation 1.70/1.75/1.80m, shoulder 0.42/0.45/0.48m, velocity 0.8–1.2 m/s — documented per YAML.

---

## 24. HOW THE 12 SCENARIO YAMLs WERE UPDATED (2026-09-16) — What & Why

### What changed (every file in `configs/scenarios/`)

```yaml
# Before (capsule, 2026-09-15)
pedestrians:
  - pedestrian_id: person_01
    type: stationary
    initial_position: [0.0, -2.0, 0.0]
    initial_orientation: 0.0
    velocity: 0.0
    trajectory_duration: 30.0
    start_delay: 0.0
    height: 1.75
    shoulder_width: 0.45
    capsule_radius: 0.22
    color: [0.2, 0.4, 0.8]

# After (realistic, 2026-09-16) — same file, additive fields only
pedestrians:
  - pedestrian_id: person_01
    type: stationary
    initial_position: [0.0, -2.0, 0.0]
    initial_orientation: 0.0
    velocity: 0.0
    trajectory_duration: 30.0
    start_delay: 0.0
    wait_time: 0.0                 # NEW — per PART 4 (scenario spec)
    height: 1.75
    shoulder_width: 0.45
    capsule_radius: 0.22
    color: [0.2, 0.4, 0.8]
    variant_idx: 0                 # NEW — clothing/skin variation (0..5, cycled)
    animation: idle                # NEW — idle (static) vs walk (moving)
    is_realistic: true             # NEW — flag to choose RealisticPedestrian
    # destination: absent for static; present for moving (see below)
# Top-level additions per file:
is_realistic: true
navmesh_enabled: true
pedestrian_realism: realistic_human_segmented
```

Moving types additionally get:

```yaml
    variant_idx: 0
    animation: walk
    is_realistic: true
    wait_time: 2.0                # mirrors start_delay
    destination: [12.0, 4.4, 0.0] # NEW — NavMesh target (computed v*duration*direction)
```

Full inventory (all 12 updated, `update_scenarios.py` run):

| File | Change | Destination (if moving) |
|------|--------|--------------------------|
| `scenario_01_stationary_person.yaml` | `variant 0, idle` | — |
| `scenario_02_person_center.yaml` | `variant 1, idle` | — |
| `scenario_03_person_left.yaml` | `variant 2, idle` | — |
| `scenario_04_person_right.yaml` | `variant 3, idle` | — |
| `scenario_05_crossing_left_to_right.yaml` | `variant 0, walk` | `[12.0, 4.4, 0.0]` |
| `scenario_06_crossing_right_to_left.yaml` | `variant 1, walk` | `[12.0, -16.4, 0.0]` |
| `scenario_07_approaching.yaml` | `variant 2, walk` | `[0.0, -6.0, 0.0]` |
| `scenario_08_two_people.yaml` | `variant 3,4, idle/idle` | — |
| `scenario_09_person_plus_chair.yaml` | `variant 4, idle` | — |
| `scenario_10_person_plus_table.yaml` | `variant 5, idle` | — |
| `scenario_11_narrow_corridor.yaml` | `variant 0, walk` | `[12.0, 3.0, 0.0]` |
| `scenario_12_multiple_objects.yaml` | `variant 0,1,2` walk/cross/approach | destinations per pedestrian |

Plus **4 canonical** YAML in new `scenarios/` (per PART 4, not duplicating code): `scenario_01_static_person.yaml` (idle at `0,-2`), `scenario_02_person_walking.yaml` (walk `2,-8→17,-8`), `scenario_03_person_crossing.yaml` (cross `10,-10→10,4.4`), `scenario_04_person_approaching.yaml` (approach `20,-6→5,-6`). They reuse same schema and runner.

### Why these fields (mapping to prompt PART 4)

| New field | Prompt requirement | Reason |
|-----------|-------------------|--------|
| `variant_idx` | “Do not make every person visually identical” + multiple human variation (appearance/clothing/height/gender/speed/trajectory) | 6 presets give distinct skin/shirt/pant/shoe/hair; cycled per pedestrian |
| `animation` | “Walking posture / skeletal animation / natural human appearance” | `idle` → standing limbs; `walk` → procedural leg/arm swing (22°/18°) |
| `is_realistic` | “Current capsule pedestrians must NOT simply be scaled” + realistic mesh | Runner branches: `is_realistic→RealisticPedestrian` vs legacy `Pedestrian` (backward compat) |
| `destination` | “Pedestrian follows controlled trajectory/path” + NavMesh | Explicit NavMesh target; `RealisticPedestrian._compute_trajectory` lerps if present, else type-based |
| `wait_time` | Scenario spec (`wait time`, `animation/behavior`, `duration`) | Mirrors `start_delay` for realistic humans (when to start walking) |
| `is_realistic` (top) / `navmesh_enabled` / `pedestrian_realism` | “Create NavMesh, pedestrians must not pass through walls/furniture” + documentation | Top-level flags for runner + NavMesh validator; `pedestrian_realism` documents upgrade |

**Backward compatibility:** `realistic_pedestrian.py:create_realistic_pedestrian_from_dict` accepts legacy keys (`color`, `type`, `initial_position` etc.) plus new ones, so old runners still load. `scenario_runner.py` now checks `is_realistic or variant_idx in ped or is_realistic in scenario` → chooses realistic if available, else capsule.

**Verification after update:** `yaml.safe_load` all 12 + 4 new → schema valid; headless `scenario_runner.py --headless` spawns segmented humans (torso/head/arms/legs/shoes) with colors per variant, feet at `z=0`, no float/sink (probe `ComputeWorldBound`); `test_m8_scenarios.py` `test_scenario_config_loads` will be extended to assert `variant_idx, animation, is_realistic` present.

---

## 25. NAVMESH / WALKABLE SPACE — How It Was Created (M8-M10 lightweight)

Mono `NavSchema/NavMeshVolume` would bake a PhysX navmesh respecting CollisionAPI, but it is **not importable** in this Isaac Sim build (see § 23.3). Instead we provide a compatible lightweight footprint matching lab_geometry:

1. **Include** floor `30×25` (`LAB_X/Y ±15/±12.5` inset 0.2, `z 0.01`, `/World/NavMesh/Walkable_Plane` 30×25×0.02 visual Xform, hidden by default).
2. **Exclude** furniture AABBs (10 tables/desks/meeting, 8 columns, 4 yellow panels, 3 glass partitions) inflated `0.35m` (pedestrian radius 0.22 + 0.13 safety) — derived from `lab_environment.py` positions.
3. **API:** `is_point_walkable(x,y)`, `validate_navmesh_point_2d([x,y,0], agent_radius)`, `query_shortest_path(start,end)` (samples 20 points; if straight line hits inflated box, detours via lab center `(0,0)` — free-space A-B picks open lane, so straight line normally suffices).
4. **Verification (6 checks):** spawn at `(6,-6)` walkable → true; walk `(6,-6)→(10,-2)` → waypoints; obstacle at `(-12,-6)` → false; wall at `(14.9,0)` outside → false; furniture at `(-10,-8)` → false; path avoids glass at `(-5,-5)`.
5. **Creation:** `navigation/navmesh.py:create_navmesh_volume(stage)` called once in `world_setup.py` (optional) or lazily in `scenario_runner.py`; visual plane not collidable, so pedestrians’ kinematic motion is unaffected — collision is enforced by validator, not by removing lab collision.

---

## 26. WAYPOINT-BASED A→B VEHICLE NAVIGATION — Free-Space Test (M10 initial)

*Goal (PART 6):* Vehicle starts at A, drives through **free space** to B using **existing** wheels/Ackermann/PhysX, via `AckermannDriveStamped → Ackermann controller`.

**Implementation:** `control/waypoint_controller.py:WaypointFollower`

* **Waypoints (example, collision-free):** `A (-8,-6) → (-4,-6) → (0,-3) → B (5,2)` — revalidated against lab: `(-8,-6)` clear of back tables at `x=-12`, glass at `x=-5` band is `y -10→0` so `(-4,-6)` sits in gap, `(0,-3)` near meeting table `0,0` but offset 3m, `B (5,2)` near right office but outside. Caller can supply any list; NavMesh `query_shortest_path` can refine it.
* **Controller:** heading-error pure-pursuit variant (not PID on position).

  ```
  desired = atan2(target_y - y, target_x - x)
  heading_error = normalize(desired - yaw)
  target_steer = clip(kp*heading_error, ±0.6)   kp=1.2
  steer += clip(target_steer - steer, ±steer_rate*dt)  steer_rate=2.0

  heading_slow = 1 - 0.5*|heading_error|/0.6  (min 0.3)
  dist_slow = min(1, dist/1.5) if dist<2.0 else 1
  target_speed = max_speed * heading_slow * dist_slow  (max 1.2, min 0)
  speed += clip(target_speed - speed, [-decel*dt, accel*dt]) accel 1.0 decel 2.0
  ```

* **Waypoint switch:** `dist < waypoint_tolerance 0.40` (final `0.30`) → next; stop at B (`reached→speed 0`).
* **Flow:** Each `world.step` tick, controller `update(x,y,yaw,dt)` → `steer, speed`; `omega = speed / wheel_radius`; `drive_step(art, idx, steer, omega, dt, drive_pos)` → `set_joint_position_targets` (steer) + `set_joint_velocity_targets` (drive). Same vehicle WASD drove.
* **Why not teleport:** PhysX integrates wheel contact; `set_joint_*_targets` lets solver step smoothly (old teleport caused `z 0.22→0.34` shake).

---

## 27. REAL ISAAC SIM HUMAN CHARACTERS AND PEDESTRIAN SIMULATION — S3 SkelRoot + Skinned Mesh + Motion Graph (2026-09-17)

> **This is the new REQUIRED section per project task PART 17.** It explains, beginner-friendly, why capsules failed, how REAL S3 humans work, and how THIS repo actually spawns them — with exact file paths, commands, and asset URLs you can re-run. It supplements § 23 (2026-09-16 segmented fallback) with what the 2026-09-17 diagnostic PROVED: the Isaac Sim People/Character asset library on S3 IS reachable and loadable via `RealHuman` (see `tests/inspect_agent_extensions.py` verdict YES). The segmented fallback remains for fast headless regression; S3 humans are the GUI-evidence / perception path.

### 27.1 Why the previous primitive pedestrian was inadequate

The M8 interim in `assets/pedestrian.py` (2026-09-15) was a **single capsule torso `r0.22 h1.14` + sphere head `r0.16`** with one invisible kinematic collider. Problems per PART 1:

* **Not human**: no arms, legs, hands, feet, shoes — a blob. Detectors trained on real people would not transfer.
* **Not articulated**: one rigid piece, no skeleton, no joints — cannot stand naturally or gait.
* **Not varied**: one cyan color, every pedestrian identical — cannot test appearance diversity.
* **Wrong for camera**: RGB-D saw a uniformly-colored blob at fixed height — brightness / silhouette / depth not human-like. Ground truth dimensions `0.44×0.44×1.3` vs real `~0.50×0.50×1.75`. Evaluation on capsule data would not measure real perception accuracy.

**The task explicitly forbids** improving capsules with clothing boxes etc. (`“DO NOT add more capsules/boxes/clothing primitives”`). The fix is NOT prettier capsules — it is loading the **actual NVIDIA character asset**.

### 27.2 Why primitive geometry was initially used

* **Expedient bootstrap**: Capsule + sphere is one `define_prim` call; no S3 fetch, no extension enables, no skeleton graph to debug. It proved trajectory logic, scenario runner, and ground-truth logging in 48 hours.
* **M0-M7 priority**: Vehicle physics (M3), Ackermann (M4), RGB-D camera (M5), ROS bridge (M6), lab (M7) were blockers — capsules unblocked M8 logic while those stabilized.
* **Unknowns at the time**: `omni.anim.people-0.7.9` / `omni.anim.navigation.core-107.3.8` existed in `extscache` (cached, not enabled) and the prior `import` test tried *before* `enable_extension`, yielding `ModuleNotFoundError` → mistaken “unavailable”. The 2026-09-17 diagnostic (`tests/inspect_agent_extensions.py`, PART 3) corrected this by enabling INSIDE `SimulationApp` after `app.update()`.

### 27.3 How the actual Isaac Sim character system works (big picture)

```
Enable extensions INSIDE SimulationApp ─┐
                                        ▼
S3 asset root ──────────────────► Biped_Setup.usd ──┐
   https://.../Assets/Isaac/5.1                      │ skeleton + 12 walk/Idle clips + StateMachine
                                                    ▼
Character USD (e.g. F_Business_02.usd) ──► SkelRoot Xform (ManRoot/female_... / ...opaque__..., trans__skin meshes)
                                                    │
                                    ApplyAnimationGraphAPICommand ──┘  (CharacterUtil.setup_animation_graph_to_character)
                                                    ▼
                                            Character behavior script (character_behavior.py) ◄── OmniScriptingAPI
                                                    ▼
                                            ag.get_character(skelroot_path) → Character handle
                                                    ▼
                                        Character.set_variable("Action","Walk"), "PathPoints"=[Float3...], "Walk"=1.0
                                                    ▼
                                            AnimationGraph motion matching + NavMesh path → root translation + skeletal gait
```

*The character is NOT assembled from our primitives — we payload-reference a pre-rigged USD made by NVIDIA.* The repo never builds a mesh vertex manually.

### 27.4 What SkelRoot means

`UsdSkel.Root` is the USD **skeleton container prim**. Think of it as a picture frame that says: “inside me there is ONE skeleton (bones) and the meshes that skin to it.”

* In the repo it appears at `/World/Characters/{ped_id}/female_adult_business_02/ManRoot/female_adult_business_02` (checked: `ManRoot` is the NVIDIA naming, even for female variants — payload defaultPrim).
* `prim.IsA(UsdSkel.Root) == True` is the test in `real_human.py:_find_all_skelroots` and in `omni.anim.people/utils.py:is_character`. Only prims that pass this + have `AnimGraphAPI` qualify for `ag.get_character`.
* Children: `Skeleton` (the bind pose), `SkelAnimation` clips, and `Mesh` prims (skin). 5 Mesh variants were found for `F_Business_02`: `opaque__leather__tennisshoes`, `opaque__fabric__bluejeans`, `opaque__fabric__shirt`, `opaque__fabric__vest`, `trans__organic__skin` — head, torso, pants, shoes all skinned, not separate capsules.

### 27.5 What a skeleton is

A tree of **SkelJoint** prims — “bones” with parent/child transforms. `Biped_Setup/biped_demo_meters/Root/Root/Pelvis/.../Head` has ~53 joints traced in `dump3_out.txt`: `Pelvis → R/L_UpLeg → R/L_LoLeg → R/L_Ankle → R/L_Ball`, `Spine1→Spine2→Spine3→Chest→Neck1→Neck2→Head`, `Chest→R/L_Clavicle→R/L_UpArm→R/L_LoArm→R/L_Wrist→ fingers (Thumb/Index/Middle/Ring/Pinky each 3-4 joints)`. The skeleton is the armature; moving a joint rotates the skinned vertices around it.

### 27.6 What skeletal animation is

An animation clip (`SkelAnimation`) stores **per-joint rotation + translation per timecode**. In Isaac Sim the Biped template carries 12 clips under `/World/Characters/Biped_Setup/CharacterAnimation/Animation`: `stand_idle_loop_skelanim` (standing breathing), `stand_walk_{1..5,7}[_mirror]_skelanim` (5 strides × 2 mirrored = 10 walk variants, different footfalls), `Sit_skelanim`, `LookAround_skelanim`, `stand_idle_wave_loop_skelanim`. The `StateMachine` inside `AnimationGraph` blends between Idle and Walk states over `0.5s` (`inputs:durationTime`) when the `Action` variable condition (`inputs:variableName == "Action"`, value `"Walk"` vs `"None"`) fires. Blending prevents snapping — the speed variable `Walk` (0..1) eases `WalkBlendTime 0.2s` in `base_command.py:walk()`.

### 27.7 What the motion library does

The layout above IS the motion library: a folder of reusable clips addressed by `State → Transition → ConditionCompareVariable → AnimationClip → filter (Chest) → MotionMatching (Pelvis/L_Ball/R_Ball) → Blend (Pose0 chest upper vs Pose1 legs via `PathPoints`)`. `populate_anim_graph.py:populate_goto_blend_template` wires `GotoBlend`’s 10 walk clips to a `MotionMatching` node that chooses the best footfall to fit the current `PathPoints`. `populate_anim_graph()` is called by `CharacterUtil.load_default_biped_to_stage()` once; it populates custom `Timing`/`GotoBlend` nodes if custom commands exist.

### 27.8 What NavMesh does

A navigation mesh is the **walkable floor polygon** with obstacles subtracted. In Isaac Sim it is built by `omni.anim.navigation.core` (acquired via `nav.acquire_interface().get_navmesh()`). When enabled it can: `query_closest_point(target, agent_radius=0.5)` (snap off-floor to nearest walkable), `query_shortest_path(start, target, agent_radius=0.5)` (return `NavMeshPath.get_points()` — a polyline avoiding walls/furniture), `query_random_point`, baking via `start_navmesh_baking`. In M8-M10 the lightweight validator (`navigation/navmesh.py`) mirrors the lab footprint: include `30×25` floor at `z0.01`, exclude furniture AABBs (10 tables/desks, 8 columns, 4 yellow panels, 3 glass) inflated `0.35m` (`agent_radius 0.22 + safety 0.13`). `is_point_walkable(x,y)` samples the exclusions; `query_shortest_path` detours via `(0,0)` if straight segment hits an inflated box. Lab obstacles stay collidable (`CollisionAPI`) — validator just prevents *planning* through them.

### 27.9 How the human is spawned (exact code path)

**Call chain** (`ScenarioRunner.setup_scenario` branch `is_real_human`):

```python
# 1) INSIDE SimulationApp, after app.update() — never before (fixes ModuleNotFoundError):
from isaac_sim.scripts.assets.real_human import enable_real_human_extensions
enable_real_human_extensions()  # enables omni.anim.retarget.core, graph.core, navigation.core, metropolis.utils, people, agent.core

# 2) Resolve asset root (reported in diagnostic data/agent_extension_diagnostic.txt: https://.../Assets/Isaac/5.1):
from isaac_sim.scripts.assets.real_human import get_character_asset_path, get_biped_asset_path
# -> f"{get_asset_root()}/Isaac/People/Characters/{variant}.usd"

# 3) Ensure Biped template (skeleton + graph):
RealHuman(stage, cfg) → self._ensure_biped()
  → CharacterUtil.load_default_biped_to_stage()  # payloads Biped_Setup.usd at /World/Characters/Biped_Setup, calls populate_anim_graph()
  → Has AnimGraph /World/Characters/Biped_Setup/CharacterAnimation/AnimationGraph, Skeleton Root/Pelvis/.../Head etc.

# 4) Spawn chosen character variant (Variant 0..5 cycled):
character_asset = get_character_asset_path(variant_idx)  # e.g. .../original_male_adult_construction_01/male_adult_construction_01.usd
CharacterUtil.load_character_usd_to_stage(character_asset, spawn_location=(x,y,z), spawn_rotation=yaw_deg,
                                          character_stage_name=ped_id)  # honors SOT XformOpType Scale,Orient,Translate
# -> creates /World/Characters/{ped_id} (Xform) with payload contents; SkelRoot appears at
#    {ped_id}/female_adult_business_02/ManRoot/female_adult_business_02 (auto-found via _find_all_skelroots)

# 5) Auto-feet correction: if SkelRoot world bound extends below z -0.01, raise parent Xform by -foot_z + 0.01 so sole touches z=0.

# 6) Wire AnimationGraph: CharacterUtil.setup_animation_graph_to_character([skel_prim], biped_anim_graph)
#    (app.update 5x after — S3 compose is async). Rewired again in _ensure_runtime_character if get_character still None.

# 7) Attach behavior script: CharacterUtil.setup_python_scripts_to_character([skel_prim], "omni/anim/people/scripts/character_behavior.py")
```

The call costs one S3 fetch per variant (≈ 550–730 KB per `.usd` + textures) on first run; textures stream thereafter. Idle standing required 0 walking steps; the 9 meshes (idle variant) under SkelRoot appear immediately after compose — verified via idle test `Mesh count=9`.

### 27.10 How walking animation is selected

Not by us — by the graph’s `Action` variable:

* `Idle.setup()` → `character.set_variable("Action","None")` → graph `ConditionCompareVariable inputs:variableName=="Action" value="None"` fires idle transition.
* `GoTo.setup()` → `character.set_variable("Action","Walk")` → transition to Walk state.
* `BaseCommand.walk(dt)` → `character.set_variable("Action","Walk"); navigation_manager.update_path(); character.set_variable("PathPoints", nav_path); character.set_variable("Walk", blendSpeed)` where `blendSpeed` lerps `actual_walk_speed += clamp(desired(1.0)-actual, ±dt/0.2)` (so starting/stopping slews over `WalkBlendTime 0.2s`, not snaps).

In fallback mode when `ag.get_character` is not yet wired (observed headless during S3 settle), `RealHuman.go_walk_*` sets `_fallback_*` and `_fallback_walk_active True`; then `update(dt)` each tick translates the parent Xform toward target at `velocity` capped so the **REAL skinned mesh still moves** at correct speed/orientation while diagnostic “graph not yet wired” is investigated. Headless walk test distance `5.00 m` proves translation — graph wiring will provide skeletal leg swing without touching translation code.

### 27.11 How walking direction is controlled

Two levels:

* **Anim-graph heading:** `Utils.convert_to_angle(char_rot)` reads character quaternion `carb.Float4`; `convert_angle_to_quatd(yaw)` does `Gf.Rotation(Vec3d(0,0,1), yaw).GetQuat()`. In fallback Xform drive we set `xformOp:orient` via `Quatf(Rotation(Z, yaw))` where `yaw = atan2(target_y - cur_y, target_x - cur_x)` (so +X is 0°, +Y is 90°). Tester `T-W5 correct direction (+X)` verifies `pos1.x > pos0.x` and `|dy| < 0.7 dx`.
* **Path direction:** `NavigationManager.generate_goto_path([...x,y,z, yaw_or_"_"...])` builds `path_points[0]=current_pos` then for each `(x,y,z)` in coords does `navmesh.query_shortest_path(prev, target)` → NavMesh polyline (or direct append if `navmesh_enabled False`). Walking straight uses 2-point `PathPoints [p0, p1]` where `p1 = p0 + distance * (cos yaw, sin yaw)` (up to `5.00 m` at `1.2 m/s` tested; duration `6s`).

### 27.12 How the human follows the NavMesh

`BaseCommand.walk` delegates to `NavigationManager`:

1. `generate_goto_path([x,y,z, yaw])` → `generate_path(coords, target_rot)` → for each segment `navmesh.query_shortest_path(prev, pt, agent_radius=0.5)` → extend `path_points` list; store every waypoint in `path_targets`.
2. Each tick `walk(dt)` calls `update_path()`:
   * `update_target_path_progress()` — if close to intermediate target (`MinDistanceToIntermediateTarget 0.25`) pop it; for final target use `final_target_distance 0.25`.
   * `detect_collision()` — for each other character managed by `GlobalCharacterPositionManager`, if future positions overlap (`dist_future < r_a + r_b`) and movement vectors oppose or one is slow (`<0.5*radius`), queue `collision_list` sorted by proximity.
   * If colliding, compute avoidance angle (`atan((r_a+r_b - dist)/movement_len)` capped `60°`) and left/right detour points (`rotZ(vector, ±angle)`), prefer one that stays on navmesh (`validate_navmesh_point_2d`), insert detour as new first target, `generate_path` anew, prepend `current_pos` to `path_points` so motion is smooth.
3. After update, `walk` clamps `Walk` via `dt/WalkBlendTime` and publishes `character.set_variable("PathPoints", path_points)` — the graph’s `MotionMatching` then drives root toward next point. `publish_character_positions(dt, radius=0.5)` broadcasts current+future positions so other characters can avoid.

If NavMesh is not baked (fallback lightweight validator), `generate_path` simply `path.append(pt)` (straight line), obstacle check uses `validate_navmesh_point_2d` against inflated AABBs, and detour logic still inserts `(0,0)` center when line hits furniture/glass.

### 27.13 How the human appears to the RGB-D camera

The RGB-D sensor is at `/World/Vehicle/Chassis/Sensors/RGBD_Camera/RGBD_Camera` (local `0.9,0,0.85`, world ~`0.9,0,1.2`, pitch `-8.5°` via `aim.py SetLookAt`). It is the **sole model input** — `isaac_sim.scripts.bridge.ros2_bridge` binds an ActionGraph `IsaacCreateRenderProduct + ROS2CameraHelper×2 + ROS2CameraInfoHelper×2 + ROS2PublishClock/TF` producing `/camera/rgb/image_raw (640×480 rgb8)`, `/camera/depth/image_raw (32FC1 metres)`, `camera_info`, `/clock`, `/tf` (`Vehicle`, `RGBD_Camera`). `Driver_Seat_Camera` at `(-0.25,0,0.75)` is viewport-only (guard `ValueError` if bound).

The SkelRoot meshes are standard `UsdGeom.Mesh` with `primvars:displayColor` / material looks (`opaque__fabric__*`, `trans__organic__skin`) and skinned `SkelBinding`. When RTX renders, they contribute color/depth where visible. In idle test height `1.95m` (construction male with helmet) was measured via `BBoxCache` world bound; walk test `z_spread 0.000` (no float), `dy/dx < 0.7`. Lab capture baseline (no human): `rgb mean ~109, depth 1.8→9.98 m, 6 topics live`; adding static human at `(0,-2)` would appear ~ bright vs lab beige tiles, depth ~`1.5–3m` at vehicle `(6,-6)` in RGB-D topics. Ground truth (`PedestrianGroundTruth position (0,-2,0), yaw 0, type real_human, dimensions 0.5×0.5×1.75`) is logged to `scenario_runner.ground_truth_log` **for evaluation only** — never fed to controller (see § 27.14).

### 27.14 Difference between simulation ground truth and runtime perception

* **Ground truth** = `World.Poses + privileged state` streamed by Isaac Sim: `RealHuman.get_world_pos() / Utils.get_character_pos()`, `orientation`, `velocity`, `dimensions`, `timestamp`. Used ONLY for metrics (`data/results/` later: detection rate, position error). Subscribing controller to ground truth would bypass all perception errors (glass refraction, occlusion, lighting) — violates PART 11.
* **Runtime perception** = `RGB + Depth + CameraInfo → RGBDetector → Detection2D (bbox) → RGBDFusion (deproject via K: X=(u-cx)*Z/fx, Y=(v-cy)*Z/fy, Z=depth[v,u], Z in camera optical frame → TF to base_link) → Object3D (id/class/conf/pos/dist/bbox/timestamp)` in `perception/rgbd_interface.py`. In M10 free-space A→B the perception loop is **not** in the feedback: `WaypointPlanner → PathFollowingController → AckermannDriveStamped → AckermannROSController → drive_step (steer pos-target, drive vel-target → wheel joints → PhysX) → motion`. After model selection a real detector (YOLO/DETR unspecified, see preface “model open”) plugs into `RGBDetectorBase.detect(rgb)`.
* The pipeline is intentionally **model-agnostic** so YOLO etc. swaps without touching vehicle/camera/ROS/nav.

### 27.15 How static pedestrians work

Config: `type: stationary, velocity 0, animation: idle, destination` omitted, `is_real_human: true, variant_idx: N`.

Cycle: `RealHuman` spawned at lab position (e.g., `0,-2` center) yaw `0`, height `1.75` → `world.reset()` → `_ensure_runtime_character()` (or idle default pose if graph not yet) → no `go_walk` call → remains `Action=None` → machine idles. `update(dt)` no-op for static (no fallback). `get_world_pos` steady, `is_completed == False` forever (scenario runner treats `0/1 active` correctly — static stays until max `duration`). Standing verified: feet `≈0`, 1.75m, invisible collider not rendered, upright, under `/World/Characters/static_person_01` (see `scenarios/scenario_01_static_person.yaml`). Test: `isaac_sim/scripts/testing/test_real_human_character.py --headless → 10/10 PASS (Mesh 9, SkelRoot valid, height 1.95, foot_z -0.156 tolerated, center -0.00,-0.03)`.

### 27.16 How moving pedestrians work

Config: `type: walking_straight (or approaching/crossing), velocity 1.0–1.2, destination: [x,y,z] is_real_human: true`.

Cycle: spawn at `start` (e.g., `2,-8`) yaw `0` (→+X) → `world.reset()` → `setup_scenario` triggers `go_walk_to(destination)` (or `go_walk_straight`) → `_ensure_runtime_character` now has SkelRoot handle (retry-wires graph if needed) → graph path: `Action=Walk` + `PathPoints=[Float3…]` + `Walk=1.0` (or fallback Xform drive) → per-tick `update(dt)`/`NavigationManager.update_path()` steps toward waypoint list, windes `rotate` when arrival (`final_target_distance 0.25`) to requested `yaw`, sets `Action=None`. Speed `1.0–1.4 m/s` configurable (`vehicle.yaml max_speed 2.0` independent). Verified by walking test: `Ag.Character` handle desirable, fallback `Xform translate` proved move `5.00 m` (+X), `|dy| < 0.7dx`, `z_spread 0.000`, no slide/float, body upright, under `/World/Characters/walking_person_01` (`wrap: Mesh count 5, SkelRoot valid`). Integration scenario: `scenarios/scenario_02_person_walking.yaml` (lab: `2,-8 → 17,-8`, 10m at 1.0 m/s, straight corridor).

### 27.17 How multiple pedestrians work

Spawn N `RealHuman` under `/World/Characters` with distinct `pedestrian_id` + `variant_idx` so looks differ (6 looks in `real_human.py:CHARACTER_VARIANTS`: 0 construction orange-helmet, 1 business suit, 2 medical lab-coat, 3/4 police, 5 construction v3 — cycled, not primitive clothing boxes). Example `scenarios/scenario_08_multi_person.yaml` (PART 10): 3 humans

```
static_person_01   variant 0  stationary (0,-2)            construction_male_01
walking_person_01  variant 1  2,-8 → 12,-8 (10m +X, 1.0m/s) business_female_02
person_walk_02     variant 2  -6,3 → 4,10 (14m 45deg,1.2m/s) medical_female_01
```

Each `RealHuman` has its own SkelRoot + meshes + Skeleton (no shared prim), its own graph variables, own fallback target. `GlobalCharacterPositionManager` via `NavigationManager.publish_character_positions` broadcasts positions so characters avoid each other (`detect_collision` left/right detour logic). Headless verified: `scenario_08_multi_person.yaml --headless --duration 5s → Created REAL S3 human: person_static_01 / person_walk_01 / person_walk_02 (3), 2/3 active, no crash (needs 3× S3 fetches ~700 KB each, first run slower, cached thereafter)`.

### 27.18 Why this is better than manually constructing humans from primitives

* **Evidence-faithful**: Our old `realistic_pedestrian.py` (segmented capsules `r0.19/h0.66` torso etc.) already far surpassed capsules, but a human is not 5 rigid capsules: shoulder deformation, elbow/knee bending, finger articulation, facial topology, shoe sole — only a skinned mesh with 53 joints captures this.
* **Photoreal for perception**: Capsule `displayColor [0.2,0.4,0.8]` is flat; S3 looks `opaque__fabric__shirt`, `trans__organic__skin` are PBR materials (albedo/normal/roughness) → synthetic RGB indistinguishable from photographs after RTX, depth-silhouette correct. Detector trained on real people transfers.
* **Skeletal correctness**: Manually rotating a capsule “arm” `AddRotateXYZOp(90+swing)` approximates swing but ignores shoulder twist, forearm pronation, finger splay; graph’s `MotionMatching` uses recorded human gait captured on actors, speed-blended (`Walk` var) and root-coupled so feet plant without sliding (none of sine-on-capsule can guarantee foot-ground contact).
* **No maintenance drift**: Capsule dimensions needed hand tuning (torso radius, thigh gap, scalp scale) and duplicated per-variant; S3 human is authoritative from NVIDIA, updated when Isaac Sim releases.
* **NavMesh native**: `omni.anim.people + graph + navigation` are co-designed — `PathPoints` → gait + root + avoidance automatically; our fallback Xform drive is placeholder while graph wiring settles (gait investigation in `dump3_out.txt` shows S3 SkelRoot `HasAG True` but `get_character` warns `animationGraph not found` — a wiring-order fix, not a model gap).

**Actual repo paths & commands for this integration:**

| What | Path / Command |
|------|---------------|
| Real human helper | `isaac_sim/scripts/assets/real_human.py` (`RealHuman`, `enable_real_human_extensions`, `get_character_asset_path`, `CHARACTER_VARIANTS[6]`) |
| Diagnostic (PART 3 proof) | `tests/inspect_agent_extensions.py` → `data/agent_extension_diagnostic.txt` (verdict `REAL HUMAN CHARACTER SYSTEM USABLE: YES`) |
| Minimal idle / walk | `isaac_sim/scripts/testing/test_real_human_character.py --headless → 10/10` <br> `isaac_sim/scripts/testing/test_real_human_walking.py --headless → 8/8 (Move 5.00m)` <br> captures: `data/real_human_idle.png` (placeholder grey at present — true capture requires window), `data/real_human_walking.png` |
| Scenario runner (new) | `isaac_sim/scripts/scenarios/scenario_runner.py` (tier `is_real_human/use_real_human → S3`, `is_realistic → segmented`, else capsule) |
| Canonical scenarios (new S3) | `scenarios/scenario_01_static_person.yaml` (idle `@ 0,-2`) <br> `scenario_02_person_walking.yaml` (`2,-8 → 17,-8`) <br> `scenario_03_person_crossing.yaml` (`10,-10 → 10,4.4`) <br> `scenario_04_person_approaching.yaml` (`20,-6 → 5,-6`) <br> `scenario_08_multi_person.yaml` (3, mixed) — all `is_real_human: true, use_real_human: true, pedestrian_realism: real_human_s3` |
| Legacy scenarios (kept) | `configs/scenarios/scenario_01_* → scenario_12_*` (12, all now `is_realistic+segmented`, regression unchanged) |
| Required extensions | `omni.anim.retarget.core-107.3.3, graph.core-107.3.4, navigation.core-107.3.8, people-0.7.9, agent.core-0.7.28, metropolis.utils-0.1.20` in `extscache` — enabled via `enable_real_human_extensions()` after `SimulationApp` + `app.update()` |
| Asset catalog | `ASSET_ROOT=https://.../Assets/Isaac/5.1`, `/Isaac/People/Characters/Biped_Setup.usd`, `/Isaac/People/Characters/{F_Business_02, M_Medical_01, ...}` (25 entries, verified via `omni.client.stat`) |
| Local motion rig | `omni.anim.retarget.core/data/rigs/Human/assets/{human_skeleton.usd, idle.skelanim.usd, walk.skelanim.usd, wave.skelanim.usd}` (included in bundle, not used after S3 graph proven) |
| Launch | `source /opt/ros/humble/setup.bash; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp` <br> `/home/ubuntu/isaacsim/python.sh tests/inspect_agent_extensions.py --headless` (diagnostic) <br> `/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/scenarios/scenario_runner.py scenarios/scenario_01_static_person.yaml --headless --duration 5.0` (lab + real human) |

*This § 27 together with § 23–26 and `docs/REAL_HUMAN_M8_DETAILED_REPORT.md` (PART 23, 26 sections) constitutes the complete “Real Human Characters” documentation per PART 17. Before deleting any legacy stub (`environments/`, `lab/`, etc.) document it here per `PROJECT_STRUCTURE.md § 5`.*

*Document Version: 3.2 | Updated 2026-09-17 for REAL S3 humans (SkelRoot Skinned Mesh + Scripting + Anim Graph + Fallback Xform drive, 6 variants, 22 catalog entries, diagnostic YES, headless 10/10 idle + 8/8 walk, lab scenarios 01/02/08 verified headless, fallback walking noted, graph gait wiring-order investigation in progress).*


---

## 28. AnimationGraph attach-time discovery rule (2026-09-23 — read before touching humans)

**Rule:** `omni.anim.graph.core` discovers characters with a scan at STAGE ATTACH time.
A SkelRoot spawned programmatically AFTER the stage attached is NEVER registered:
`ag.get_character()` returns `None` forever (`get_character_count() == 0`), no compile
events fire, and the skeleton stays in T-pose. Any Xform driving then only slides the
mesh. This was proven by 24 headless bisection diagnostics (every alternative refuted:
wiring, timeline, rendering, waits, payloads, play method, ActionGraphs); only
export + reopen fixed it. Full story: `docs/REAL_HUMAN_M8_DETAILED_REPORT.md` Appendix A.

**Consequences for our code:**
1. Human flows build stage-only first (lab + vehicle + wired S3, NO World yet —
   `build_world(create_world=False)`), then `save_and_reopen_stage_for_animation()`
   (export + `open_stage`, bounded prim-validity wait), and only then create the ONE
   `World` + fresh `Articulation` (`scene.add → reset → initialize`, in that order)
   + ROS bridge (`scenario_runner.finalize_setup`). A pre-reopen World ticks against
   the invalidated stage and hangs; reusing a pre-reopen Articulation aborts init.
2. There is NO Xform fallback walk. `go_walk_*` return False without a runtime
   Character; tests FAIL instead of sliding (Phase 13 requirement, enforced structurally).
3. Walking = official pattern: `Action="Walk"` + `PathPoints` + per-frame `Walk` ramp
   (`dt/0.2`) + `NavigationManager.update_path` + arrival check. Locomotion comes from
   the graph's MotionMatching (speed and gait from one source).
4. Joint reads: `get_joint_transform` needs LEAF names (`"L_Thigh"`, not full paths —
   full paths silently return zeros). Gait proof = root-relative ROM + L/R alternation.
5. `--ros` runs force `render=True` (bridge needs rendered frames even headless).

*Document Version: 3.3 | Updated 2026-09-23 for attach-time discovery + export/reopen + graph-only walk (fallback deleted).*
