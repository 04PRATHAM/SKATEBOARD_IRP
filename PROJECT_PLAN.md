# Project Plan: Multi-Object Steering Using RGB-D Camera

## Milestone Strategy

The project is implemented incrementally, starting from environment inspection and building complexity step by step.

### Milestone 0: Machine/Environment Inspection
- Determine Ubuntu version, GPU, NVIDIA driver, RAM
- Determine ROS 2 distribution, Isaac Sim version
- Report installed components and missing pieces
- **Current**: Ubuntu 22.04.5 LTS, ROS 2 Humble, NVIDIA RTX PRO 6000 Blackwell Workstation Edition (96GB VRAM, Driver 580.173.02), Isaac Sim 5.1.0-rc.19 installed at `/home/ubuntu/isaacsim/` with native ROS 2 Humble bridge support.

### Milestone 1: Create Project Structure
- Initialize repository structure
- Create README.md, PROJECT_PLAN.md
- Create docs/ architecture.md, simulation_workflow.md, simulation_world_plan.md
- Create initial configuration files
- **Stop**: Wait for user confirmation before Milestone 2

### Milestone 2: Minimal Isaac Sim World
- Create a very simple Isaac Sim world
- Include: floor, lighting, simple vehicle, one wall, one obstacle, one person
- Verify the world opens successfully
- **Stop**: Wait for user confirmation before Milestone 3

### Milestone 3: Vehicle Physics Stability
- Verify vehicle rests correctly on floor
- Verify wheels contact ground
- Verify no physics explosion
- Verify vehicle can move (forward, stop, steer)
- **Status**: ✅ COMPLETE (2026-09-09) — Vehicle settles at z=0.334m, tilt <2°, 6-DOF articulation stable, no explosions
- **Stop**: Wait for user confirmation before Milestone 4

### Milestone 4: Ackermann-Style Steering
- Implement Ackermann-style steering
- Verify: straight, left, right, stop
- **Status**: ✅ COMPLETE (2026-09-09) — 14/15 verification checks pass: straight 3.92m, left +0.479 rad/s, right -0.104 rad/s, stop <0.01 m/s, clamps enforced
- **Known Limitation**: Right-turn yaw asymmetry (0.104 vs 0.479 rad/s) documented
- **Stop**: Wait for user confirmation before Milestone 5

### Milestone 5: Add RGB-D Camera
- Add RGB-D camera to vehicle
- Verify: RGB image generated, depth image generated, camera frame exists, camera position is correct
- **Stop**: Wait for user confirmation before Milestone 6

### Milestone 6: Connect ROS 2
- Connect Isaac Sim to ROS 2 via ROS 2 bridge
- Verify: ros2 topic list shows RGB, depth, CameraInfo topics
- Verify: TF transforms exist
- Verify: External ROS node receives messages
- **Status (2026-10-08)**: BLOCKED — `/clock`+`/tf` publish; camera topics never appear.
  Root cause: Isaac Sim 5.1.0-rc.19 `omni.syntheticdata` writer-attach regression
  (`Unable to write from unknown dtype`; NVIDIA's own camera example fails identically).
  See `docs/YOLO_PERCEPTION_VALIDATION_REPORT.md` §7. Bridge architecture unchanged.
- **Stop**: Wait for user confirmation before Milestone 7

### Milestone 7: Build Representative Laboratory
- Build the laboratory environment using reference photographs/videos as design guidance
- Use modular assets for furniture, partitions, columns, etc.
- **Stop**: Wait for user confirmation before Milestone 8

### Milestone 8: Add Reusable People and Obstacle Scenarios
- Create reusable pedestrian assets (stationary, walking straight, crossing path, approaching path, two people, multiple people)
- Create static obstacles (table, chair, column, wall, partition)
- Each scenario specifies: starting position, orientation, velocity, trajectory, start time, end time
- **Status (2026-10-09)**: 12 YAML scenarios use explicit `human_tier: s3` (12/12 headless PASS,
  15/15 S3 spawns SkelRoot-verified); YAML statics spawn via `spawn_static_obstacles()`.
  Human tiers: `s3` (skinned S3+graph, GUI evidence) vs `segmented` (fast headless; walk pivots
  are segment-center, cosmetic gaps mid-stride — documented limitation). `navmesh_enabled: true`
  in YAMLs is metadata only — no NavMesh is built on the YAML path (direct-segment walking).
  Vehicle is stationary by default (`vehicle_behavior`, wall-collision fix 2026-10-09).
- **Stop**: Wait for user confirmation before Milestone 9

### Milestone 9: Create RGB-D Perception Interface
- Create modular ROS 2 perception interface
- Input: RGB image, Depth image, CameraInfo
- Output: DetectedObject[] (object ID, class, confidence, position, distance, bounding box, timestamp)
- Keep perception modular (not locked to specific AI model)
- **Status (2026-11-06)**: ✅ COMPLETE — Verified under Isaac Sim 6.2 with YOLO26n-seg inference (6-9 ms), mask-median depth backprojection (9-12 Hz), vectorized unclassified 3D depth geometry clustering, and exact optical ground-elevation isolation ($z_v \in [0.18, 1.80]\text{ m}$).

### Milestone 10: Create Initial Steering Controller
- Create modular steering controller
- Initial implementation: rule-based
- Input: object information
- Output: Ackermann-compatible command
- **Status (2026-11-06)**: ✅ COMPLETE — Autonomous point-to-point planner with global A* corridor search, dynamic multi-candidate Ackermann detours, emergency safe stopping bumper ($0.55\text{ m}$), and zero-latency UDP transmission to `/cmd_ackermann`.

### Milestone 11: Implement Multi-Object Scenarios
- Implement scenarios: person left, person right, person center, two people, person + table, person + chair + column, two moving people, narrow corridor, moving pedestrian crossing path
- Record metrics for each scenario
- **Status (2026-11-06)**: ✅ COMPLETE — 13/13 scenarios (S01–S13) fully implemented and verified in Isaac Sim 6.2, covering static obstacles, narrow corridors, unclassified crates, unexpected obstacles, single/dual crossing S3 RealHumans with native OpenUSD gait kinematics, wall-to-wall barricade safe halts, and perception watchdog failsafe stopping.

### Milestone 12: Implement Experiment Logging and Evaluation
- Implement experiment logging
- Record perceptual metrics (detection rate, position error, distance error)
- Record steering metrics (steering angle, response time, lateral deviation)
- Record system metrics (RGB FPS, depth FPS, ROS 2 frequency, latency)
- **Status (2026-11-06)**: ✅ COMPLETE — Categorized synchronized evidence (`.png` and `.json`) logged across `data/model_tests/path_visualizer/` (`goal_reached/`, `obstacle_detection/`, `planned_path/`, `navigation_progress/`, `blocked_runs/`, `failed_runs/`). Strict Rule 8 ground-truth isolation enforced.

### Milestone X: Workstation Migration / Environment Revalidation (RUN ONLY WHEN SWITCHING PCs — NOT part of the dev sequence)
- **When**: Only when the SKATEBOARD_IRP folder is COPIED to a different PC. Do NOT execute on the current machine.
- **Goal**: Prove the new PC can build and run the exact same simulation before resuming work.
- **Steps**:
  1. Copy the full `SKATEBOARD_IRP/` folder to the new PC (e.g. `/home/<user>/SKATEBOARD_IRP`). Keep the folder name.
  2. Check OS: Ubuntu 22.04 LTS (`lsb_release -a`). Isaac Sim 5.1 requires it.
  3. Check GPU + driver: `nvidia-smi` must show an RTX-class GPU with driver >= 580 (current PC: RTX PRO 6000 Blackwell, driver 580.173.02, CUDA 13.0).
  4. Check/install ROS 2 Humble at `/opt/ros/humble` (`ls /opt/ros/humble`, `ros2 --help`). Bridge needs the exact distro (Humble).
  5. Install Isaac Sim 5.1.0 (full workstation bundle) and note its path. Current path `/home/ubuntu/isaacsim/` WILL differ — update every occurrence:
     - `isaac_sim/scripts/run_*.py` launch commands (replace with new `<isaacsim>/python.sh`)
     - `sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")` in `run_fidelity_world.py` (and any /tmp probe scripts) → new folder path
     - Verify `isaacsim.ros2.bridge` extension exists: `ls <isaacsim>/exts | grep ros2.bridge`
  6. Set DDS env exactly as on this PC: `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp` (bridge rclpy fails to import without sourced ROS env).
  7. Revalidate, in order (expected results in brackets):
     - `source /opt/ros/humble/setup.bash`
     - Headless world build: `<isaacsim>/python.sh isaac_sim/scripts/run_fidelity_world.py --headless --duration 3.0` [exit 0, no Traceback]
     - ROS 2 bridge: same with `--ros2`, then in a second terminal `ros2 topic list` must show `/camera/rgb/image_raw`, `/camera/depth/image_raw`, both `camera_info`, `/clock`, `/tf` [all 6 present]
     - `ros2 topic echo --once /camera/rgb/image_raw` [640x480 rgb8, frame `camera_rgb_optical_frame`]
  8. Only after all green: resume normal milestones. Record the new PC's specs by updating Milestone 0 + "Current Environment" above.

## Current Environment

| Component | Status |
|-----------|--------|
| Ubuntu | 22.04.5 LTS (Jammy Jellyfish) |
| ROS 2 | Humble (installed at /opt/ros/humble) |
| GPU | NVIDIA RTX PRO 6000 Blackwell Workstation Edition, driver 580.173.02, CUDA 13.0 |
| Isaac Sim | 5.1.0-rc.19 installed at `/home/ubuntu/isaacsim/` |
| ROS 2 Bridge | `isaacsim.ros2.bridge` extension installed with Humble support |
| Python | Python 3.11.13 in Isaac Sim (`/home/ubuntu/isaacsim/python.sh`) |

## Reference Material

- `references/vehicle/` - Photographs of the real vehicle (4 WhatsApp JPEG images)
- `references/lab/` - Photographs of the real laboratory (37 WhatsApp JPEG images)

These are design references only. Exact dimensions are not available from the reference material. Configurable approximate dimensions will be documented in `configs/vehicle.yaml`.

## Documentation

- `docs/architecture.md` - System architecture and design
- `docs/simulation_workflow.md` - Simulation workflow and pipeline
- `docs/vehicle_model.md` - Vehicle model details
- `docs/environment_model.md` - Environment/model details
- `docs/sensor_setup.md` - RGB-D camera setup
- `docs/ros2_interfaces.md` - ROS 2 interface specification
- `docs/scenarios.md` - Scenario definitions
- `docs/validation.md` - Validation checklist
- `docs/troubleshooting.md` - Common errors and fixes