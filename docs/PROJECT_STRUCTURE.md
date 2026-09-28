# SKATEBOARD_IRP — Project Structure & File Reference

**Version:** 2026-09-17 (M8–M10 Real Human & Navigation Phase)  
**Audience:** Robotics engineers, system architects, and core contributors — technically exact, comprehensive, and clear.  
**Project Root:** `~/SKATEBOARD_IRP` (`/home/ubuntu/SKATEBOARD_IRP`)  
**Isaac Sim Environment:** 5.1.0-rc.19 at `/home/ubuntu/isaacsim/` — Python launcher is `/home/ubuntu/isaacsim/python.sh` (do **NOT** `source` it; execute it directly).  
**ROS 2 Environment:** Humble Hawksbill at `/opt/ros/humble/` — always `source /opt/ros/humble/setup.bash` and `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp` in ROS terminals.

---

## 1. Architectural Philosophy & Directory Design

Robotics simulation platforms require strict separation of concerns. In NVIDIA Isaac Sim 5.1 and ROS 2 Humble projects, mixing geometry, physics solver parameters, sensor optics, and controller logic into single monolithic scripts causes severe fragility: changing a visual material could corrupt rigid body collision frames, or modifying a control loop rate could invalidate camera render product timestamps.

`SKATEBOARD_IRP` enforces a **modular, data-driven architecture**:

1. **Centralized Configuration (`configs/`)**: Single source of truth for physical dimensions, solver settings, topic strings, and safety thresholds. Code never hardcodes physical constants.
2. **Procedural World & Visual Builders (`isaac_sim/scripts/`)**: Reusable Python scripts that construct USD (Universal Scene Description) stages programmatically—including lab architecture, vehicle articulations, visual digital twins, camera optics, and ROS 2 ActionGraphs.
3. **Model-Agnostic Interfaces (`isaac_sim/scripts/perception/`, `control/`, `navigation/`)**: Perception feeds directly from physical camera sensors without synthetic ground-truth shortcuts. Controllers output standardized `AckermannDriveStamped` commands.
4. **Realistic Characters & Walkable Spaces (`isaac_sim/scripts/assets/`, `navigation/`)**: Supports photorealistic animated humans (`SkelRoot` + `Skeleton` + skinned mesh + animation graphs) navigating collision-aware lab spaces.
5. **Deterministic Verification Suites (`tests/`)**: Automated CI-style test suites ensuring that physics stability, Ackermann steering limits, sensor streams, and scenario execution remain green.
6. **Documentation & Evaluation (`docs/`, `data/`)**: Comprehensive technical guides, visual evidence, regression logs, and design reference media.

---

## 2. Complete Workspace Directory Tree

Below is the verified, file-by-file directory tree for the entire `SKATEBOARD_IRP` repository:

```
SKATEBOARD_IRP/
├── configs/                                 # Centralized YAML configuration files
│   ├── camera.yaml                          # Camera resolution, FOV, clipping, optical frames, topics
│   ├── simulation.yaml                      # Timestep (100Hz), TGS solver iterations, ROS bridge, environment USD
│   ├── steering.yaml                        # Ackermann limits, velocity/acceleration constraints, safety margins
│   ├── vehicle.yaml                         # Chassis mass, wheelbase, track width, wheel radius, friction, camera mount
│   └── scenarios/                           # Legacy scenario YAML definitions (Scenarios 01 to 12)
│       ├── scenario_01_stationary_person.yaml
│       ├── scenario_02_person_center.yaml
│       ├── scenario_03_person_left.yaml
│       ├── scenario_04_person_right.yaml
│       ├── scenario_05_person_crossing_left_to_right.yaml
│       ├── scenario_06_person_crossing_right_to_left.yaml
│       ├── scenario_07_person_approaching.yaml
│       ├── scenario_08_two_people.yaml
│       ├── scenario_09_person_plus_chair.yaml
│       ├── scenario_10_person_plus_table.yaml
│       ├── scenario_11_narrow_corridor.yaml
│       └── scenario_12_multiple_objects.yaml
├── scenarios/                               # Canonical per-specification scenario YAML files
│   ├── scenario_01_static_person.yaml       # Stationary human obstacle at center
│   ├── scenario_02_person_walking.yaml      # Pedestrian walking along open lane
│   ├── scenario_03_person_crossing.yaml     # Perpendicular crossing pedestrian
│   ├── scenario_04_person_approaching.yaml  # Head-on approaching pedestrian
│   └── scenario_08_multi_person.yaml        # Multi-pedestrian dynamic environment
├── isaac_sim/                               # Isaac Sim extension & python module root
│   └── scripts/                             # Active Python source code for simulation
│       ├── __init__.py                      # Package initialization for isaac_sim.scripts
│       ├── run_ackermann_demo.py            # Interactive Ackermann motion demo with live ROS 2 control
│       ├── run_development_mode.py          # Legacy interactive development entry point
│       ├── run_fidelity_test.py             # Legacy visual & physics fidelity test runner
│       ├── run_fidelity_world.py            # Primary thin launcher for stage inspection & headless smoke runs
│       ├── run_minimal_world.py             # Legacy minimal stage launcher
│       ├── run_waypoint_navigation.py       # Standalone runner for autonomous A-to-B waypoint navigation
│       ├── verify_vehicle_complete.py       # Authoritative 56-check vehicle acceptance test suite
│       ├── assets/                          # Character & obstacle asset generation modules
│       │   ├── __init__.py                  # Subpackage initialization
│       │   ├── pedestrian.py                # Interim capsule pedestrian model
│       │   ├── real_human.py                # Photorealistic S3 SkelRoot human character builder with Biped anim graph
│       │   └── realistic_pedestrian.py      # Fallback segmented-mesh realistic pedestrian
│       ├── bridge/                          # ROS 2 Omniverse ActionGraph bridges
│       │   ├── __init__.py                  # Subpackage initialization
│       │   └── ros2_bridge.py               # ROS 2 Humble bridge ActionGraph builder (RGB, Depth, CameraInfo, Clock, TF)
│       ├── control/                         # Vehicle motion controllers
│       │   ├── __init__.py                  # Subpackage initialization
│       │   └── waypoint_controller.py       # Pure-pursuit / heading-error waypoint follower generating Ackermann commands
│       ├── environment/                     # Legacy environment stubs
│       │   ├── __init__.py                  # Subpackage initialization
│       │   └── minimal_env.py               # Minimal room setup script
│       ├── environments/                    # Production digital twin environment modules
│       │   ├── __init__.py                  # Subpackage initialization
│       │   ├── lab_assets.py                # Procedural furniture & architectural asset builders
│       │   └── lab_environment.py           # 30x25m high-fidelity laboratory digital twin builder
│       ├── navigation/                      # Walkable space & path planning modules
│       │   ├── __init__.py                  # Subpackage initialization
│       │   └── navmesh.py                   # 2D walkable space validator, obstacle AABB inflator, and path planner
│       ├── perception/                      # Sensor processing & computer vision interfaces
│       │   ├── __init__.py                  # Subpackage initialization
│       │   └── rgbd_interface.py            # Model-agnostic RGB-D frame extraction & depth point cloud interface
│       ├── scenarios/                       # Scenario execution engine
│       │   ├── __init__.py                  # Subpackage initialization
│       │   └── scenario_runner.py           # Reusable scenario loader, pedestrian spawner, & evaluator
│       ├── sensors/                         # Physical sensor simulation
│       │   ├── __init__.py                  # Subpackage initialization
│       │   └── rgbd_camera.py               # Physical Intel RealSense RGB-D camera sensor prim builder
│       ├── testing/                         # Manual interactive test utilities
│       │   ├── __init__.py                  # Subpackage initialization
│       │   ├── run_wasd_test.py             # Indefinite interactive WASD manual vehicle locomotion test runner
│       │   ├── test_real_human_character.py # Test script for validating SkelRoot character spawning & idle poses
│       │   ├── test_real_human_walking.py   # Test script for validating real human walking animations & trajectories
│       │   └── wasd_controller.py           # Event-driven keyboard input handler for manual vehicle driving
│       ├── utils/                           # Core utilities & builder abstractions
│       │   ├── __init__.py                  # Subpackage initialization
│       │   ├── aim.py                       # Precision camera lookAt orientation math utility
│       │   ├── config_loader.py             # YAML configuration loader with caching
│       │   ├── inspection_cameras.py        # 24-viewpoint inspection camera rig builder
│       │   └── world_setup.py               # Single unified world builder (`build_world`) used across all scripts
│       └── vehicle/                         # Vehicle articulation, kinematics, & visual models
│           ├── __init__.py                  # Subpackage initialization
│           ├── ackermann_controller_ros.py  # Ackermann drive step execution & ROS 2 subscriber controller
│           ├── simple_vehicle.py            # 6-DOF vehicle physics articulation, joints, & collision colliders
│           └── visual_vehicle.py            # High-fidelity visual digital twin (tubular frame, seat, springs, rims)
├── tests/                                   # Automated test suites
│   ├── inspect_agent_extensions.py          # Extension availability diagnostic script for Omniverse agent extensions
│   ├── test_m8_scenarios.py                 # Test suite for scenario execution, ground truth, & regression checks
│   ├── test_vehicle_physics.py              # Physics stability regression test
│   └── verify_ackermann_physics.py          # 15-check Ackermann vehicle physics verification suite
├── data/                                    # Captured simulation outputs, diagnostics, & visual evidence
│   ├── agent_extension_diagnostic.txt       # Extension discovery report for `omni.anim.*` extensions
│   ├── real_human_idle.png                  # Render capture of real human character in idle stance
│   ├── real_human_walking.png               # Render capture of real human character walking
│   ├── rgbd_depth.png                       # Sample depth image captured from vehicle RGB-D camera
│   └── rgbd_rgb.png                         # Sample RGB image captured from vehicle RGB-D camera
├── docs/                                    # Technical documentation & project specifications
│   ├── architecture.md                      # System node diagram & data flow architecture
│   ├── COMMANDS.md                          # Quick-reference command cheatsheet
│   ├── ISAAC_SIM_LEARNING_AND_PROJECT_GUIDE.md # 850+ line comprehensive Isaac Sim & ROS 2 learning guide
│   ├── PROJECT_STRUCTURE.md                 # ← This comprehensive file structure & file reference document
│   ├── REAL_HUMAN_M8_DETAILED_REPORT.md     # Engineering report detailing S3 SkelRoot human integration
│   ├── reference_fidelity_plan.md           # Visual upgrade implementation plan
│   ├── reference_observations.md            # Gap analysis comparing simulation to physical reference photos
│   ├── simulation_workflow.md               # Step-by-step developer simulation workflow
│   ├── simulation_world_plan.md             # Initial USD stage structure & vehicle geometry design plan
│   ├── vehicle_model.md                     # Vehicle kinematics, joint limits, & coordinate transform math
│   └── VEHICLE_RECONSTRUCTION_REPORT.md     # Detailed 27-point reconstruction report
├── references/                              # Physical reference media
│   ├── lab/                                 # 37 reference photographs of physical lab environment
│   │   └── WhatsApp Image *.jpeg
│   └── vehicle/                             # 4 reference photographs of physical skateboard vehicle
│       └── WhatsApp Image *.jpeg
├── ros2_ws/                                 # ROS 2 workspace (reserved for ROS packages)
│   └── src/                                 # ROS 2 package source tree
│       └── rgbd_perception/                 # Stub for ROS 2 perception node package
├── FINAL_REPORT_33_POINT.md                 # 33-point completion report for earlier project phases
├── PROJECT_PLAN.md                          # Master project milestone roadmap (M0 through M10)
├── README.md                                # Repository overview & quickstart instructions
├── TODO.md                                  # Active task tracking & session logs
└── work.md                                  # Chronological development log (Sessions 0 to 24)
```

---

## 3. Exhaustive File-by-File Technical Reference

Below is a detailed breakdown of **every single file** in the `SKATEBOARD_IRP` repository, grouped by directory.

---

### 3.1. Root Configuration Directory (`configs/`)

* **`configs/camera.yaml`**
  * **Purpose:** Centralized source of truth for the physical Intel RealSense RGB-D camera sensor mounted on the vehicle.
  * **Key Elements:** Image resolution (`640x480`), horizontal FOV (`1.2217 rad` / `70 deg`), vertical FOV (`0.7907 rad` / `45.3 deg`), clipping range (`0.1m` to `10.0m`), mount offset (`position_x: 0.9`, `position_y: 0.0`, `position_z: 1.2`), optical frame names (`camera_rgb_optical_frame`), and ROS 2 topic names (`/camera/rgb/image_raw`, `/camera/depth/image_raw`).
  * **Inputs/Dependencies:** Read by `utils/config_loader.py`.
  * **Outputs/Effects:** Configures camera optics in `sensors/rgbd_camera.py` and topic names in `bridge/ros2_bridge.py`.
  * **Used By:** `rgbd_camera.py`, `ros2_bridge.py`, `world_setup.py`.
  * **Safety:** Mount coordinates must match `vehicle.yaml` camera mount values; altering topic strings requires updating external ROS 2 subscribers.

* **`configs/simulation.yaml`**
  * **Purpose:** Defines global Isaac Sim environment settings, physics solver parameters, rendering quality, and ROS 2 bridge configurations.
  * **Key Elements:** Physics timestep `dt: 0.01` (100 Hz), `substeps: 2`, `solver_type: "TGS"`, `solver_iterations: 16` (to eliminate contact chatter), gravity `[0.0, 0.0, -9.81]`, rendering resolution `1280x720`, ROS 2 domain ID `0`, clock topic `/clock`, TF topic `/tf`.
  * **Inputs/Dependencies:** Loaded via `config_loader.py`.
  * **Outputs/Effects:** Configures the `isaacsim.core.api.World` instance and PhysX scene parameters.
  * **Used By:** `world_setup.py`, `ros2_bridge.py`, `run_fidelity_world.py`, all runners.
  * **Safety:** Keeping `dt: 0.01` and `solver_iterations: 16` is critical for chassis stability and avoiding wheel contact chatter.

* **`configs/steering.yaml`**
  * **Purpose:** Configures Ackermann steering limits, vehicle dynamic speed constraints, controller gains, and safety emergency distances.
  * **Key Elements:** Max steering angle `0.6 rad`, max speed `2.0 m/s`, max acceleration `1.5 m/s²`, max deceleration `3.0 m/s²`, Ackermann geometry factor `1.0`, PID gains (`kp: 1.5`, `ki: 0.0`, `kd: 0.1`), safety slow distance `2.0m`, emergency stop distance `0.5m`, control loop rate `50 Hz`.
  * **Inputs/Dependencies:** Loaded via `config_loader.py`.
  * **Outputs/Effects:** Provides parameter limits to `waypoint_controller.py` and `ackermann_controller_ros.py`.
  * **Used By:** `waypoint_controller.py`, `ackermann_controller_ros.py`.
  * **Safety:** Must stay synchronized with `vehicle.yaml` steering limits.

* **`configs/vehicle.yaml`**
  * **Purpose:** Physical specification for the 4-wheeled skateboard vehicle platform.
  * **Key Elements:** Vehicle length `1.8m`, width `1.2m`, height `0.4m`, wheelbase `1.1m`, track width `1.0m`, wheel radius `0.25m`, wheel width `0.18m`, ground clearance `0.15m`, chassis mass `120.0 kg` (total vehicle `150.0 kg` with wheels), chassis diagonal inertia tensor `(20.0, 50.0, 60.0)`, wheel friction (`static: 0.9`, `dynamic: 0.8`), camera mount offset (`0.9, 0.0, 1.2`), camera pitch `-0.15 rad` (`-8.5 deg`).
  * **Inputs/Dependencies:** Loaded via `config_loader.py`.
  * **Outputs/Effects:** Parameterizes USD geometry construction, rigid body mass APIs, and physics material binding in `simple_vehicle.py`.
  * **Used By:** `simple_vehicle.py`, `visual_vehicle.py`, `rgbd_camera.py`, `world_setup.py`.
  * **Safety:** High impact. Changing wheelbase/track alters joint anchor points and joint limit enforcement.

---

### 3.2. Scenario Configuration Directories (`configs/scenarios/` and `scenarios/`)

* **`configs/scenarios/scenario_01_stationary_person.yaml` through `scenario_12_multiple_objects.yaml` (12 files)**
  * **Purpose:** Legacy scenario specification files defining initial vehicle spawn poses, pedestrian spawn locations, pedestrian types (stationary, walking, crossing, approaching), walking velocities, trajectory durations, and obstacle objects.
  * **Key Elements:** `scenario_name`, `vehicle_start_pose` (`[x, y, z, yaw]`), `pedestrians` array containing `pedestrian_id`, `type`, `position`, `yaw_deg`, `velocity`, `trajectory_duration`, and `destination`.
  * **Inputs/Dependencies:** Parsed by `scenarios/scenario_runner.py`.
  * **Outputs/Effects:** Parameterizes automated scenario execution runs and test suites.
  * **Used By:** `scenarios/scenario_runner.py`, `tests/test_m8_scenarios.py`.
  * **Safety:** Schema must be preserved for backward compatibility.

* **`scenarios/scenario_01_static_person.yaml`**
  * **Purpose:** Canonical M8 scenario: Vehicle at `(-8.0, -6.0)` drives toward destination while a stationary real human stands at `(-2.0, -6.0)`.
* **`scenarios/scenario_02_person_walking.yaml`**
  * **Purpose:** Canonical M8 scenario: Pedestrian walks along the vehicle's driving corridor.
* **`scenarios/scenario_03_person_crossing.yaml`**
  * **Purpose:** Canonical M8 scenario: Pedestrian crosses perpendicular to the vehicle's trajectory.
* **`scenarios/scenario_04_person_approaching.yaml`**
  * **Purpose:** Canonical M8 scenario: Pedestrian walks head-on toward the approaching vehicle.
* **`scenarios/scenario_08_multi_person.yaml`**
  * **Purpose:** Multi-agent scenario featuring multiple interactive pedestrians in the lab environment.

---

### 3.3. Simulation Core Scripts (`isaac_sim/scripts/`)

#### 3.3.1. Top-Level Entry Points (`isaac_sim/scripts/`)

* **`isaac_sim/scripts/__init__.py`**
  * **Purpose:** Package marker allowing Python to import `isaac_sim.scripts` submodules cleanly.

* **`isaac_sim/scripts/run_ackermann_demo.py`**
  * **Purpose:** Interactive demonstration script running an automated 6-phase maneuver sequence (settle -> straight -> left turn -> hold -> right turn -> hold) while subscribing to `/cmd_ackermann` for live ROS 2 control.
  * **Inputs/Dependencies:** `world_setup.py`, `ackermann_controller_ros.py`, `SimulationApp`.
  * **Outputs/Effects:** Opens GUI viewport, executes vehicle motions, logs vehicle poses.
  * **Used By:** Manual demonstration and ROS 2 control validation.

* **`isaac_sim/scripts/run_development_mode.py`**
  * **Purpose:** Legacy developer launcher providing an interactive stage setup with debugging inspection helpers.
  * **Inputs/Dependencies:** `world_setup.py`.
  * **Outputs/Effects:** Boots Isaac Sim with visual inspection cameras.

* **`isaac_sim/scripts/run_fidelity_test.py`**
  * **Purpose:** Legacy visual & physics fidelity test runner used during early stage reconstruction.

* **`isaac_sim/scripts/run_fidelity_world.py`**
  * **Purpose:** Primary thin launcher for inspecting the lab stage (`--inspect`), running headless smoke checks (`--headless`), or launching ROS 2 bridge mode (`--ros2`).
  * **Inputs/Dependencies:** `utils/world_setup.py`, `SimulationApp`.
  * **Outputs/Effects:** Builds unified world via `build_world()`, keeps GUI window open or runs headless for a specified `--duration`.
  * **Used By:** Primary entry point for visual inspection and CI smoke testing.

* **`isaac_sim/scripts/run_minimal_world.py`**
  * **Purpose:** Legacy lightweight stage runner from initial development phases.

* **`isaac_sim/scripts/run_waypoint_navigation.py`**
  * **Purpose:** Standalone runner for autonomous free-space A-to-B navigation (M8–M10).
  * **Inputs/Dependencies:** `world_setup.py`, `waypoint_controller.py`, `navmesh.py`, `ackermann_controller_ros.py`, `real_human.py`.
  * **Outputs/Effects:** Spawns vehicle at A `(-8,-6)`, navigates to B `(5,2)`, avoids obstacles, executes smooth Ackermann maneuvers, and stops cleanly.
  * **Used By:** Primary entry point for testing autonomous navigation.

* **`isaac_sim/scripts/verify_vehicle_complete.py`**
  * **Purpose:** Authoritative 56-check acceptance test suite validating USD hierarchy, local transforms, wheel joint axes, collision API bindings, friction materials, camera alignments, and physics stability.
  * **Inputs/Dependencies:** `world_setup.py`, `pxr.Usd`, `pxr.UsdPhysics`.
  * **Outputs/Effects:** Runs headless checks and outputs detailed `[PASS]/[FAIL]` diagnostics (exit code 0 on success).
  * **Used By:** Regression testing prior to commits.

---

#### 3.3.2. Assets Subpackage (`isaac_sim/scripts/assets/`)

* **`isaac_sim/scripts/assets/__init__.py`**
  * **Purpose:** Subpackage initialization for asset generators.

* **`isaac_sim/scripts/assets/pedestrian.py`**
  * **Purpose:** Interim capsule pedestrian model providing primitive collider geometry and ground-truth logging structures.
  * **Key Elements:** `PedestrianConfig`, `PedestrianType`, `PedestrianState`, `Pedestrian` class creating a torso capsule (`r:0.22m, h:1.14m`), head sphere (`r:0.16m`), and invisible kinematic collider (`r:0.24m, h:1.58m`).
  * **Inputs/Dependencies:** `pxr.Usd`, `pxr.UsdGeom`, `pxr.UsdPhysics`.
  * **Outputs/Effects:** Creates USD prim `/World/Pedestrians/{id}`.
  * **Used By:** Legacy scenario tests (`test_m8_scenarios.py`).

* **`isaac_sim/scripts/assets/real_human.py`**
  * **Purpose:** Production builder for real photorealistic Isaac Sim human characters loaded from NVIDIA's S3 asset library (`https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1`).
  * **Key Elements:** `RealHuman` class, `RealHumanConfig`, `CHARACTER_VARIANTS` (construction male, business female, medical female/male, police female). Loads `Biped_Setup.usd`, instantiates character USD with `SkelRoot` + `Skeleton` + skinned mesh (~1.75m high), wires `AnimationGraph`, attaches `navigation_manager`, and drives `Action="Walk"` / `PathPoints` for realistic walking animations.
  * **Inputs/Dependencies:** `omni.anim.graph.core`, `omni.anim.people`, `isaacsim.replicator.agent.core`, `pxr.UsdSkel`.
  * **Outputs/Effects:** Creates `/World/Characters/{id}` containing true rigged human mesh.
  * **Used By:** `scenarios/scenario_runner.py`, `run_waypoint_navigation.py`, `test_real_human_walking.py`.
  * **Safety:** Must enable required extensions (`enable_real_human_extensions()`) after `SimulationApp` startup.

* **`isaac_sim/scripts/assets/realistic_pedestrian.py`**
  * **Purpose:** Fallback segmented-mesh realistic pedestrian builder used when S3 character assets or animation graph extensions are unavailable.
  * **Key Elements:** Multi-part mesh (head, torso, upper/lower arms, legs, shoes) with programmatic sine-wave walking limb articulation.

---

#### 3.3.3. ROS 2 Bridge Subpackage (`isaac_sim/scripts/bridge/`)

* **`isaac_sim/scripts/bridge/__init__.py`**
  * **Purpose:** Subpackage initialization for ROS 2 bridges.

* **`isaac_sim/scripts/bridge/ros2_bridge.py`**
  * **Purpose:** Constructs the Omniverse ActionGraph (`/ROS2_Bridge`) that connects Isaac Sim render products and physics stages to ROS 2 Humble topics.
  * **Key Elements:** `setup_ros2_bridge()`. Creates nodes: `IsaacReadSimulationTime`, `ROS2PublishClock`, `ROS2Context`, `IsaacCreateRenderProduct`, `ROS2CameraHelper` (for RGB and Depth), `ROS2PublishTransformTree` (for `/tf`). Publishes `/camera/rgb/image_raw`, `/camera/depth/image_raw`, `/camera/rgb/camera_info`, `/camera/depth/camera_info`, `/clock`, `/tf`.
  * **Inputs/Dependencies:** `omni.graph.core`, `isaacsim.ros2.bridge` extension.
  * **Outputs/Effects:** Populates ActionGraph under `/ROS2_Bridge` on the stage.
  * **Used By:** `utils/world_setup.py` when `with_ros=True`.
  * **Safety:** Contains an explicit guard preventing binding to inspection cameras or viewport-only driver seat cameras.

---

#### 3.3.4. Vehicle Control Subpackage (`isaac_sim/scripts/control/`)

* **`isaac_sim/scripts/control/__init__.py`**
  * **Purpose:** Subpackage initialization for control modules.

* **`isaac_sim/scripts/control/waypoint_controller.py`**
  * **Purpose:** Implements a pure-pursuit / heading-error waypoint-following controller for free-space vehicle navigation (M8–M10).
  * **Key Elements:** `WaypointFollower` class, `WaypointControllerConfig`. Computes target heading `atan2(dy, dx)`, heading error `normalize_angle(desired - yaw)`, proportional steering command `clip(kp * heading_error, ±0.6)`, rate-limited steering, and distance-based acceleration/deceleration speed profiles. Switches waypoints within `waypoint_tolerance` (0.4m) and comes to a full stop at destination B (`dist < 0.3m`).
  * **Inputs/Dependencies:** Vehicle pose `(x, y, yaw)` and physics `dt`.
  * **Outputs/Effects:** Returns `(steer_rad, speed_mps, reached_flag)`.
  * **Used By:** `run_waypoint_navigation.py`, autonomous navigation scripts.

---

#### 3.3.5. Environment Subpackages (`isaac_sim/scripts/environment/` and `environments/`)

* **`isaac_sim/scripts/environment/__init__.py`** & **`minimal_env.py`**
  * **Purpose:** Legacy environment stubs preserved for backwards compatibility.

* **`isaac_sim/scripts/environments/__init__.py`**
  * **Purpose:** Subpackage initialization for production environment builders.

* **`isaac_sim/scripts/environments/lab_assets.py`**
  * **Purpose:** Procedural asset creation functions for laboratory interior elements.
  * **Key Elements:** Functions creating wooden slatted columns (`create_slatted_column`), yellow architectural pillar panels (`create_yellow_pillar_panel`), glass partition walls with aluminum frames (`create_glass_partition`), meeting tables, office desks, and chairs.
  * **Inputs/Dependencies:** `pxr.Usd`, `pxr.UsdGeom`, `pxr.UsdPhysics`.
  * **Outputs/Effects:** Populates USD geometry under `/World/Environment/`.
  * **Used By:** `lab_environment.py`.

* **`isaac_sim/scripts/environments/lab_environment.py`**
  * **Purpose:** Builds the complete 30x25m high-fidelity representative laboratory environment.
  * **Key Elements:** `create_lab_environment()`. Constructs tiled floor plane (`Tile_Ground_Plane`), infinite physics ground plane (`GroundPlane` at `z=0`), 3.5m off-white walls with green accent sections, ceiling at `z=3.50m` with 5 rotating ceiling fans, 8 wooden columns, yellow panels, 3 glass partition walls (collidable), 10 tables/desks, 12 chairs, and multi-source RTX lighting (Dome light 4000K, Distant sun light 15000 lx, 6 interior LED panel lights).
  * **Inputs/Dependencies:** `lab_assets.py`, `pxr.Usd`, `pxr.UsdGeom`, `pxr.UsdPhysics`, `pxr.UsdLux`.
  * **Outputs/Effects:** Populates stage under `/World/Environment` and `/World/Lighting`.
  * **Used By:** `utils/world_setup.py`.

---

#### 3.3.6. Navigation Subpackage (`isaac_sim/scripts/navigation/`)

* **`isaac_sim/scripts/navigation/__init__.py`**
  * **Purpose:** Subpackage initialization for navigation algorithms.

* **`isaac_sim/scripts/navigation/navmesh.py`**
  * **Purpose:** Lightweight 2D walkable space validator, obstacle AABB inflator, and path planner for the 30x25m lab.
  * **Key Elements:** `LAB_X_MIN/MAX` (`±14.8m`), `LAB_Y_MIN/MAX` (`±12.3m`), `OBSTACLES` list (derived from lab asset coordinates for tables, desks, columns, glass walls), `INFLATE` margin (`0.35m`). `is_point_walkable(x, y)`, `validate_navmesh_point_2d(point)`, `query_shortest_path(start, end)`, and `create_navmesh_volume(stage)` (creates visual representation under `/World/NavMesh`).
  * **Inputs/Dependencies:** `pxr.UsdGeom`.
  * **Outputs/Effects:** Validates spawn points and generates collision-free A-to-B waypoint lists.
  * **Used By:** `run_waypoint_navigation.py`, `real_human.py`, `scenario_runner.py`.

---

#### 3.3.7. Perception Subpackage (`isaac_sim/scripts/perception/`)

* **`isaac_sim/scripts/perception/__init__.py`**
  * **Purpose:** Subpackage initialization for perception interfaces.

* **`isaac_sim/scripts/perception/rgbd_interface.py`**
  * **Purpose:** Model-agnostic perception interface for reading camera sensor data from Isaac Sim render products.
  * **Key Elements:** `RGBDInterface` class, `CameraFrame` dataclass (storing timestamp, RGB numpy array `uint8`, Depth numpy array `float32` in meters, CameraInfo matrix `K`). `get_current_frame()`, `depth_to_point_cloud()`.
  * **Inputs/Dependencies:** `omni.replicator.core`, `usdrt.Sdf`, `numpy`.
  * **Outputs/Effects:** Returns structured camera frames and deprojected 3D point clouds without using ground-truth position cheating.
  * **Used By:** Autonomous perception pipelines and evaluation scripts.

---

#### 3.3.8. Scenarios Subpackage (`isaac_sim/scripts/scenarios/`)

* **`isaac_sim/scripts/scenarios/__init__.py`**
  * **Purpose:** Subpackage initialization for scenario runners.

* **`isaac_sim/scripts/scenarios/scenario_runner.py`**
  * **Purpose:** Unified engine for executing data-driven scenario YAML files.
  * **Key Elements:** `ScenarioRunner` class, `ScenarioResult`. Loads scenario YAML, resets vehicle spawn pose, instantiates real human characters or capsule pedestrians, steps simulation at `dt=0.01`, logs ground-truth trajectories and vehicle poses, and evaluates safety metrics (collisions, minimum clearance).
  * **Inputs/Dependencies:** `utils/world_setup.py`, `assets/real_human.py`, `assets/pedestrian.py`, `utils/config_loader.py`.
  * **Outputs/Effects:** Runs scenarios in GUI or headless mode, writes evaluation logs.
  * **Used By:** `tests/test_m8_scenarios.py`, standalone scenario command-line invocations.

---

#### 3.3.9. Sensors Subpackage (`isaac_sim/scripts/sensors/`)

* **`isaac_sim/scripts/sensors/__init__.py`**
  * **Purpose:** Subpackage initialization for physical sensors.

* **`isaac_sim/scripts/sensors/rgbd_camera.py`**
  * **Purpose:** Programmatically builds the physical Intel RealSense RGB-D camera sensor prim on the vehicle chassis.
  * **Key Elements:** `create_rgbd_camera()`, `create_camera_visual()`. Defines `UsdGeom.Xform` at `/World/Vehicle/Chassis/Sensors/RGBD_Camera` and `UsdGeom.Camera` at `.../RGBD_Camera`. Sets focal length (`24.0mm`), aperture, clipping range (`0.1m`–`10.0m`), and aims optics downward by `-8.5 deg` using `aim_prim()`.
  * **Inputs/Dependencies:** `camera.yaml`, `vehicle.yaml`, `utils/aim.py`, `pxr.UsdGeom`.
  * **Outputs/Effects:** Creates USD camera prim parented under Chassis so optics move 1:1 with vehicle rigid body dynamics.
  * **Used By:** `vehicle/simple_vehicle.py`, `utils/world_setup.py`.

---

#### 3.3.10. Manual Testing Subpackage (`isaac_sim/scripts/testing/`)

* **`isaac_sim/scripts/testing/__init__.py`**
  * **Purpose:** Subpackage initialization for testing modules.

* **`isaac_sim/scripts/testing/run_wasd_test.py`**
  * **Purpose:** Standalone interactive runner allowing indefinte manual driving of the vehicle using keyboard WASD controls.
  * **Key Elements:** `main()`. Launches `SimulationApp`, calls `build_world(dynamic=True)`, attaches `WASDState`, executes main simulation loop (`while sim.is_running(): world.step(render=True)`), logs vehicle pose every 2 seconds.
  * **Inputs/Dependencies:** `world_setup.py`, `wasd_controller.py`, `ackermann_controller_ros.py`.
  * **Outputs/Effects:** Keeps interactive GUI window open indefinitely until explicit user exit.
  * **Used By:** Manual driving, steering feel verification, driver POV alignment checks.

* **`isaac_sim/scripts/testing/test_real_human_character.py`**
  * **Purpose:** Minimal verification script validating that real S3 `SkelRoot` human characters load correctly, establish proper standing height (~1.75m–1.95m), and maintain idle poses.

* **`isaac_sim/scripts/testing/test_real_human_walking.py`**
  * **Purpose:** Verification script testing real human walking trajectories and animation graph execution over a 5.0m path.

* **`isaac_sim/scripts/testing/wasd_controller.py`**
  * **Purpose:** Event-driven keyboard input handler for WASD manual driving.
  * **Key Elements:** `WASDState` class (tracks speed, steer angle, max clamps `±1.5 m/s` / `±0.6 rad`), `handle_key()` mapping `W` (forward), `S` (reverse), `A` (steer left), `D` (steer right), `Space` (brake), `R` (reset steering).
  * **Inputs/Dependencies:** `carb.input` keyboard events.
  * **Outputs/Effects:** Updates velocity and steering targets consumed by `drive_step()`.
  * **Used By:** `run_wasd_test.py`.

---

#### 3.3.11. Utilities Subpackage (`isaac_sim/scripts/utils/`)

* **`isaac_sim/scripts/utils/__init__.py`**
  * **Purpose:** Subpackage initialization for utility modules.

* **`isaac_sim/scripts/utils/aim.py`**
  * **Purpose:** Precision matrix transformation helper for camera orientation.
  * **Key Elements:** `aim_prim(stage, prim_path, eye, target, up)`. Computes world lookAt matrix via `Gf.Matrix4d().SetLookAt(eye, target, up)`, inverts it, multiplies by parent world inverse, and sets exact local `xformOp:transform`.
  * **Inputs/Dependencies:** `pxr.Gf`, `pxr.UsdGeom`.
  * **Outputs/Effects:** Guarantees camera optical axis aims at exact target point without gimbal lock or euler orientation errors (<0.03 deg error).
  * **Used By:** `sensors/rgbd_camera.py`, `vehicle/simple_vehicle.py` (driver seat camera).

* **`isaac_sim/scripts/utils/config_loader.py`**
  * **Purpose:** Centralized loader for YAML configuration files with in-memory caching.
  * **Key Elements:** `load_config(config_path)`. Resolves path relative to project root, reads YAML, caches dictionary.
  * **Inputs/Dependencies:** `yaml`, `os.path`.
  * **Outputs/Effects:** Returns parsed configuration dictionary.
  * **Used By:** All modules requiring config parameters.

* **`isaac_sim/scripts/utils/inspection_cameras.py`**
  * **Purpose:** Builds a 24-camera diagnostic inspection rig positioned around the lab and vehicle spawn locations.
  * **Key Elements:** `create_inspection_cameras()`. Defines `UsdGeom.Camera` prims under `/World/InspectionCameras/` (e.g., `ThreeQuarter_Front`, `Top_Down_Overview`, `Chassis_CloseUp`).
  * **Inputs/Dependencies:** `aim.py`, `pxr.UsdGeom`.
  * **Outputs/Effects:** Populates Viewport camera dropdown menu without creating render products or ROS 2 topics.
  * **Used By:** `world_setup.py`.

* **`isaac_sim/scripts/utils/world_setup.py`**
  * **Purpose:** Single unified world builder (`build_world()`) enforcing zero-drift stage construction across all runners and tests.
  * **Key Elements:** `build_world(dynamic=False, with_cameras=True, with_ros=False, start_xy=None)`. Instantiates `isaacsim.core.api.World`, loads configs, builds lab environment via `create_lab_environment()`, creates vehicle via `create_simple_vehicle()`, initializes inspection cameras, resets stage, and optionally sets up ROS 2 bridge.
  * **Inputs/Dependencies:** `config_loader.py`, `lab_environment.py`, `simple_vehicle.py`, `inspection_cameras.py`, `ros2_bridge.py`.
  * **Outputs/Effects:** Returns dictionary containing `{"world", "stage", "articulation", "vehicle_cfg", "camera_cfg", "sim_cfg"}`.
  * **Used By:** Every runner and test suite in the project (`run_fidelity_world.py`, `run_ackermann_demo.py`, `run_wasd_test.py`, `run_waypoint_navigation.py`, `verify_vehicle_complete.py`, `verify_ackermann_physics.py`, `test_m8_scenarios.py`).
  * **Safety:** Critical single source of truth. Constants `VEHICLE_START_STATIC (-4.0, 0.0)` and `VEHICLE_START_DYNAMIC (6.0, -6.0)` must not be modified arbitrarily.

---

#### 3.3.12. Vehicle Kinematics & Physics Subpackage (`isaac_sim/scripts/vehicle/`)

* **`isaac_sim/scripts/vehicle/__init__.py`**
  * **Purpose:** Subpackage initialization for vehicle modules.

* **`isaac_sim/scripts/vehicle/ackermann_controller_ros.py`**
  * **Purpose:** Low-level vehicle execution engine and ROS 2 subscriber controller.
  * **Key Elements:** `drive_step(art, idx, steer, omega, dt, drive_pos)` function. Computes exact Ackermann inner/outer steer angles (`apply_ackermann_steering`), sets steering joint position targets on `Joint_Steer_FL` and `Joint_Steer_FR`, sets drive joint velocity targets (`omega = speed / radius`) on `Joint_RL`, `Joint_RR`, `Joint_Drive_FL`, and `Joint_Drive_FR`. `AckermannROSController` class subscribes to `/cmd_ackermann` (`AckermannDriveStamped`) with a 0.5s timeout.
  * **Inputs/Dependencies:** `simple_vehicle.py`, `pxr.UsdPhysics`, `rclpy`.
  * **Outputs/Effects:** Directs PhysX joint drive targets on the vehicle articulation.
  * **Used By:** `run_ackermann_demo.py`, `run_wasd_test.py`, `run_waypoint_navigation.py`, scenario runners.
  * **Safety:** Must use position/velocity drive targets rather than hard coordinate teleportation to prevent physics instability.

* **`isaac_sim/scripts/vehicle/simple_vehicle.py`**
  * **Purpose:** Programmatically constructs the 6-DOF vehicle physics articulation structure, collision geometries, joints, mass properties, and physics materials.
  * **Key Elements:** `create_simple_vehicle()`, `apply_ackermann_steering()`. Creates static group `/World/Vehicle`, dynamic floating base `/World/Vehicle/Chassis` (`chassis_mass: 120kg`, diagonal inertia `(20,50,60)`), invisible chassis collision box (`1.8x1.2x0.4m`), 2 front steer knuckles (`Knuckle_FL`, `Knuckle_FR`), 4 cylinder wheel colliders (`r:0.25m, h:0.18m`, axle Y), 2 revolute steer joints (axis Z, stiffness `5e4`, damping `500.0`, limits `±0.6 rad`), 4 revolute drive joints (axis Y, stiffness `0`, damping `10.0`), PhysX solver position iterations (`16`) & velocity iterations (`8`), `UsdPhysics.FilteredPairsAPI` (preventing chassis-wheel self-collision), tire physics material (`static: 0.9, dynamic: 0.8`) bound to wheels and ground plane, and instantiates driver POV camera at `/World/Vehicle/Chassis/Driver_Seat_Camera/DriverSeat_Camera`.
  * **Inputs/Dependencies:** `vehicle.yaml`, `sensors/rgbd_camera.py`, `visual_vehicle.py`, `pxr.UsdPhysics`, `pxr.PhysxSchema`.
  * **Outputs/Effects:** Spawns complete physics vehicle articulation under `/World/Vehicle`.
  * **Used By:** `utils/world_setup.py`.

* **`isaac_sim/scripts/vehicle/visual_vehicle.py`**
  * **Purpose:** Constructs the high-fidelity visual digital twin of the vehicle platform mounted under `/World/Vehicle/Chassis/Visuals` (separated from collision shapes).
  * **Key Elements:** `create_vehicle_visuals()`. Programmatically models the dark open tubular steel frame, aluminum diamond-plate floorboard, tan leather driver bench seat with backrest and armrests, front steering column assembly with orange coilover springs, silver aluminum front/rear crash bumpers, dark grey camera riser post, dual-lens camera housing, red emergency stop button, and 4 white custom wheel rims with grey rubber tires.
  * **Inputs/Dependencies:** `vehicle.yaml`, `pxr.UsdGeom`, `pxr.UsdShade`.
  * **Outputs/Effects:** Creates visual USD mesh tree.
  * **Used By:** `simple_vehicle.py`.

---

### 3.4. Test Suites Directory (`tests/`)

* **`tests/inspect_agent_extensions.py`**
  * **Purpose:** Diagnostic tool probing the availability and import health of Omniverse agent and animation extensions (`omni.anim.*`, `isaacsim.replicator.agent.*`) within the current Isaac Sim Kit installation.
  * **Outputs/Effects:** Generates diagnostic log at `data/agent_extension_diagnostic.txt`.

* **`tests/test_m8_scenarios.py`**
  * **Purpose:** Automated test suite running M8 scenarios, verifying ground-truth logging, scenario completion, and regression metrics (11/11 passing).
  * **Inputs/Dependencies:** `scenarios/scenario_runner.py`, `configs/scenarios/`.

* **`tests/test_vehicle_physics.py`**
  * **Purpose:** Legacy unit test verifying basic vehicle instantiation and joint creation.

* **`tests/verify_ackermann_physics.py`**
  * **Purpose:** Automated 15-check physics test suite validating vehicle settling height, roll/pitch levelness, 6-DOF joint counting, straight driving distance, left turn positive yaw rate, right turn negative yaw rate, stop braking distance, and joint limit enforcement (15/15 passing).
  * **Inputs/Dependencies:** `world_setup.py`, `ackermann_controller_ros.py`, `SimulationApp`.
  * **Outputs/Effects:** Runs headless simulation ticks and reports pass/fail statistics.

---

### 3.5. Data & Captured Evidence Directory (`data/`)

* **`data/agent_extension_diagnostic.txt`**
  * **Purpose:** Diagnostic text file recording extension state and import status for character animation extensions.
* **`data/real_human_idle.png`**
  * **Purpose:** Image capture validating real S3 `SkelRoot` human character standing in idle pose inside the lab.
* **`data/real_human_walking.png`**
  * **Purpose:** Image capture validating real human character walking animation along a trajectory.
* **`data/rgbd_depth.png`**
  * **Purpose:** Captured depth image from vehicle RGB-D camera verifying valid distance measurements (`1.8m` to `9.98m`).
* **`data/rgbd_rgb.png`**
  * **Purpose:** Captured RGB image from vehicle RGB-D camera verifying proper forward-facing view of the lab.

---

### 3.6. Documentation Directory (`docs/`)

* **`docs/architecture.md`**: High-level diagram and textual overview of system nodes, USD hierarchy, and ROS 2 bridges.
* **`docs/COMMANDS.md`**: Quick-reference command cheatsheet for running stage inspections, manual WASD tests, ROS 2 bridges, scenario runners, and verification suites.
* **`docs/ISAAC_SIM_LEARNING_AND_PROJECT_GUIDE.md`**: Comprehensive 850+ line developer and onboarding guide covering Isaac Sim fundamentals, USD architecture, physics tuning, ROS 2 bridge setup, real human integration, and troubleshooting patterns.
* **`docs/PROJECT_STRUCTURE.md`**: ← **This document.** Exhaustive repository structure, directory organization, and file reference manual.
* **`docs/REAL_HUMAN_M8_DETAILED_REPORT.md`**: Technical report documenting S3 `SkelRoot` photorealistic human character integration, Biped animation graphs, and NavMesh navigation.
* **`docs/reference_fidelity_plan.md`**: Architectural plan for visual digital twin reconstruction.
* **`docs/reference_observations.md`**: Gap analysis comparing simulation environment to physical reference photographs.
* **`docs/simulation_workflow.md`**: Step-by-step development and verification workflow guide.
* **`docs/simulation_world_plan.md`**: Design blueprint for USD stage hierarchy, lab layout, and vehicle geometry.
* **`docs/vehicle_model.md`**: Detailed mathematical reference for vehicle dimensions, mass properties, Ackermann kinematics, joint limits, and coordinate frames.
* **`docs/VEHICLE_RECONSTRUCTION_REPORT.md`**: 27-point engineering reconstruction report detailing vehicle physics stabilization, camera aiming fixes, and USD hierarchy cleanup.

---

### 3.7. Reference Media Directory (`references/`)

* **`references/lab/` (37 JPEG files)**: High-resolution photographs of the physical laboratory environment showing tiled flooring, glass partitions, wood slat columns, ceiling beams, furniture layouts, and lighting. Used as visual design ground truth.
* **`references/vehicle/` (4 JPEG files)**: Physical reference photographs of the custom 4-wheeled skateboard vehicle platform showing frame rails, seating position, steering knuckle linkage, large wheels, and camera mounting post.

---

### 3.8. ROS 2 Workspace Directory (`ros2_ws/`)

* **`ros2_ws/src/rgbd_perception/`**: Stub directory reserved for ROS 2 perception node packages processing camera streams (`/camera/rgb/image_raw`, `/camera/depth/image_raw`).

---

### 3.9. Root Reports & Task Files

* **`FINAL_REPORT_33_POINT.md`**: Milestone completion report detailing 33 verification points from initial project phases.
* **`PROJECT_PLAN.md`**: Living project roadmap tracking milestone objectives from M0 (Environment Setup) through M10 (Final Integration).
* **`README.md`**: Main repository README providing project overview, quickstart instructions, and system requirements.
* **`TODO.md`**: Active task tracking list organized by phase and session goals.
* **`work.md`**: Comprehensive chronological engineering log documenting all work sessions (Sessions 0 through 24).

---

## 4. Domain Categorization Matrix

| Category | Associated Files | Key Responsibility |
| :--- | :--- | :--- |
| **Simulation Core** | `world_setup.py`, `lab_environment.py`, `lab_assets.py`, `run_fidelity_world.py` | Stage construction, environment digital twin, lighting, unified world builder |
| **Vehicle Architecture** | `simple_vehicle.py`, `visual_vehicle.py`, `ackermann_controller_ros.py`, `configs/vehicle.yaml` | 6-DOF physics articulation, visual digital twin, Ackermann drive execution |
| **Sensors & Perception** | `rgbd_camera.py`, `rgbd_interface.py`, `configs/camera.yaml`, `bridge/ros2_bridge.py` | Physical RGB-D camera sensor prims, render products, frame extraction |
| **ROS 2 Integration** | `ros2_bridge.py`, `ackermann_controller_ros.py`, `configs/simulation.yaml` | ActionGraph ROS 2 bridge, `/clock`, `/tf`, sensor publishers, `/cmd_ackermann` subscriber |
| **Navigation & Control** | `waypoint_controller.py`, `navmesh.py`, `run_waypoint_navigation.py`, `configs/steering.yaml` | Pure-pursuit waypoint follower, 2D walkable space validator, A-to-B planner |
| **Human Assets** | `real_human.py`, `pedestrian.py`, `realistic_pedestrian.py` | S3 photorealistic SkelRoot human characters, Biped anim graph, capsule fallback |
| **Scenarios & Testing** | `scenario_runner.py`, `run_wasd_test.py`, `verify_vehicle_complete.py`, `verify_ackermann_physics.py`, `test_m8_scenarios.py` | Automated scenario execution, WASD manual driving, 56-check & 15-check test suites |
| **Configuration** | `vehicle.yaml`, `camera.yaml`, `simulation.yaml`, `steering.yaml`, `scenarios/*.yaml` | Centralized parameters, solver settings, topic strings, scenario definitions |
| **Documentation & Logs** | `ISAAC_SIM_LEARNING_AND_PROJECT_GUIDE.md`, `PROJECT_STRUCTURE.md`, `REAL_HUMAN_M8_DETAILED_REPORT.md`, `work.md`, `PROJECT_PLAN.md` | Developer onboarding, architecture specifications, session history |

---

## 5. Active vs. Legacy / Stub Status Matrix

| File / Directory | Operational Status | Rule for Developers |
| :--- | :--- | :--- |
| `isaac_sim/scripts/utils/world_setup.py` | **ACTIVE (CRITICAL)** | Single source of truth for world creation. Do not replicate world setup in new scripts. |
| `isaac_sim/scripts/vehicle/simple_vehicle.py` | **ACTIVE (CRITICAL)** | Core 6-DOF vehicle articulation. Do not alter joint axes or remove collision filtering. |
| `isaac_sim/scripts/vehicle/ackermann_controller_ros.py` | **ACTIVE (CRITICAL)** | Core drive step execution. Always set joint drive position/velocity targets, never teleport. |
| `isaac_sim/scripts/sensors/rgbd_camera.py` | **ACTIVE (CRITICAL)** | Physical camera prim builder. Camera path must remain under Chassis. |
| `isaac_sim/scripts/bridge/ros2_bridge.py` | **ACTIVE** | ActionGraph ROS 2 bridge. Guard enforces binding only to RGBD camera. |
| `isaac_sim/scripts/environments/lab_environment.py` | **ACTIVE** | 30x25m lab environment. Tile ground plane visual-only; infinite physics plane at z=0. |
| `isaac_sim/scripts/assets/real_human.py` | **ACTIVE** | Photorealistic S3 human character builder. Primary human path for M8–M10. |
| `isaac_sim/scripts/control/waypoint_controller.py` | **ACTIVE** | Autonomous A-to-B waypoint controller. Outputs Ackermann commands. |
| `isaac_sim/scripts/navigation/navmesh.py` | **ACTIVE** | Walkable space validator and AABB obstacle inflator. |
| `isaac_sim/scripts/perception/rgbd_interface.py` | **ACTIVE** | Model-agnostic RGB-D frame & point cloud extraction interface. |
| `isaac_sim/scripts/run_fidelity_world.py` | **ACTIVE** | Thin stage launcher for GUI inspect and headless smoke checks. |
| `isaac_sim/scripts/testing/run_wasd_test.py` | **ACTIVE** | Manual WASD locomotion test runner. Runs indefinitely in GUI. |
| `isaac_sim/scripts/run_waypoint_navigation.py` | **ACTIVE** | Standalone A-to-B waypoint navigation runner. |
| `isaac_sim/scripts/assets/pedestrian.py` | **LEGACY (ACTIVE FOR TESTS)** | Interim capsule pedestrian. Kept for backwards compatibility in test suite. |
| `isaac_sim/scripts/run_minimal_world.py` | **LEGACY** | Early minimal stage runner. Kept for historical reference. |
| `isaac_sim/scripts/run_development_mode.py` | **LEGACY** | Early interactive dev launcher. Kept for historical reference. |
| `isaac_sim/scripts/run_fidelity_test.py` | **LEGACY** | Early visual fidelity check script. Kept for historical reference. |
| `isaac_sim/scripts/environment/minimal_env.py` | **LEGACY STUB** | Legacy environment stub. Do not delete without dependency check. |
| `ros2_ws/src/rgbd_perception/` | **RESERVED STUB** | Reserved for future ROS 2 package compilation. |

---

## 6. System Launch Command Reference

Below are the exact command line invocations for running every entry point in the project:

```bash
# 1. Static Stage & Lab Architecture Inspection (GUI stays open)
source /opt/ros/humble/setup.bash
/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/run_fidelity_world.py --inspect

# 2. Headless Simulation Smoke Check (3 second automated run)
/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/run_fidelity_world.py --headless --duration 3.0

# 3. Interactive WASD Manual Locomotion Test (GUI stays open indefinitely)
/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/testing/run_wasd_test.py

# 4. Standalone Autonomous Waypoint Navigation (A to B)
/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/run_waypoint_navigation.py

# 5. Interactive Ackermann Motion Demo (Sequence + Live ROS 2 control)
/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/run_ackermann_demo.py

# 6. Authoritative Vehicle Acceptance Test Suite (56 checks, Headless)
/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/verify_vehicle_complete.py --headless

# 7. Automated Ackermann Physics Verification Suite (15 checks, Headless)
/home/ubuntu/isaacsim/python.sh tests/verify_ackermann_physics.py --headless

# 8. Scenario Test Suite (11 checks, Headless)
/home/ubuntu/isaacsim/python.sh tests/test_m8_scenarios.py --headless

# 9. Real Human Character Spawning & Idle Verification
/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/testing/test_real_human_character.py --headless

# 10. Real Human Walking Trajectory Verification
/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/testing/test_real_human_walking.py --headless

# 11. ROS 2 Bridge Live Streaming Run
# Terminal 1 (Simulation Launcher):
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/run_fidelity_world.py --headless --ros2 --duration 300

# Terminal 2 (ROS 2 Observer):
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 topic list
ros2 topic echo --once /camera/rgb/image_raw
```

---

## 7. Data Flow & Execution Pipelines

### 7.1. Pipeline 1: Interactive Manual Locomotion (WASD)

```
Keyboard Event (W/A/S/D / Space / R)
  ↓
WASDState.handle_key() (Updates speed target ±1.5 m/s, steer target ±0.6 rad)
  ↓
drive_step(articulation, indices, steer, omega, dt, drive_pos)
  ↓
apply_ackermann_steering() → Calculates exact inner/outer wheel angles (FL/FR)
  ↓
PhysX DriveAPI target assignment:
  - Joint_Steer_FL / FR → Position targets (Stiffness 5e4, Damping 500)
  - Joint_RL / RR / Drive_FL / FR → Velocity targets (omega = speed / radius)
  ↓
PhysX Rigid Body Simulation Step (100 Hz, TGS 16 iterations)
  ↓
Vehicle Chassis moves → Cameras under /World/Vehicle/Chassis move 1:1
  ↓
Viewport Camera rendering (DriverSeat_Camera or RGBD_Camera)
```

### 7.2. Pipeline 2: Autonomous Free-Space Waypoint Navigation (A to B)

```
Navigation Goal Specification (Start A (-8,-6) → Goal B (5,2))
  ↓
navmesh.py: query_shortest_path(A, B) → Collision-free 2D Waypoints
  ↓
waypoint_controller.py: WaypointFollower.update(x, y, yaw, dt)
  - Calculates heading_error = normalize_angle(atan2(dy, dx) - yaw)
  - Calculates steer_cmd = clip(kp * heading_error, ±0.6)
  - Calculates speed_cmd (slows for turns and arrival)
  ↓
drive_step(articulation, indices, steer_cmd, omega, dt, drive_pos)
  ↓
PhysX Vehicle Articulation Step (100 Hz)
  ↓
Pose update from USD Chassis transformation matrix
  ↓
Arrival check (dist < 0.3m to B) → Full stop (speed 0, steer 0)
```

### 7.3. Pipeline 3: ROS 2 Perception & Control Bridge

```
Isaac Sim Stage Step
  ↓
Render Product at /World/Vehicle/Chassis/Sensors/RGBD_Camera/RGBD_Camera
  ↓
Omniverse ActionGraph (/ROS2_Bridge)
  ├── ROS2PublishClock → /clock (rosgraph_msgs/Clock)
  ├── ROS2PublishTransformTree → /tf (tf2_msgs/TFMessage)
  ├── ROS2CameraHelper (RGB) → /camera/rgb/image_raw & /camera/rgb/camera_info
  └── ROS2CameraHelper (Depth) → /camera/depth/image_raw & /camera/depth/camera_info
  ↓
External ROS 2 Perception Node (e.g. rgbd_interface.py)
  ↓
AckermannDriveStamped command published on /cmd_ackermann
  ↓
ackermann_controller_ros.py: ROS subscriber callback
  ↓
drive_step() → PhysX Joint Targets
```

---

## 8. Developer Modification Rules & Checklist

When making modifications to the repository, follow these mandatory safety rules:

1. **Parameters in Configs**: Never hardcode physical dimensions, solver iterations, topic strings, or controller gains inside Python scripts. Always place them in `configs/*.yaml` and load them via `config_loader.py`.
2. **Unified Builder Usage**: Always use `build_world()` from `utils/world_setup.py` when instantiating the simulation stage in new scripts. Do not write ad-hoc stage setup code.
3. **No Chassis Teleportation**: Never modify the `xformOp:translate` of `/World/Vehicle/Chassis` while simulation physics is active (`dynamic=True`). Motion must be driven exclusively via joint drives in `drive_step()`.
4. **Camera Aiming**: Always use `aim_prim()` from `utils/aim.py` when orienting cameras to prevent gimbal lock and rotation matrix errors.
5. **Camera Mounting**: Ensure camera mount offsets in `camera.yaml` match `vehicle.yaml` (`position_x: 0.9`, `position_z: 1.2`), or the camera housing will clip through the visual post.
6. **Child Colliders Only**: In USD physics setup, apply `UsdPhysics.CollisionAPI` to child shape prims (`Cube`, `Cylinder`), never to parent `Xform` prims holding `RigidBodyAPI`.
7. **Regression Testing**: Before committing changes, run the full headless verification suite:
   ```bash
   /home/ubuntu/isaacsim/python.sh isaac_sim/scripts/verify_vehicle_complete.py --headless
   /home/ubuntu/isaacsim/python.sh tests/verify_ackermann_physics.py --headless
   /home/ubuntu/isaacsim/python.sh tests/test_m8_scenarios.py --headless
   ```
