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
- **Stop**: Wait for user confirmation before Milestone 7

### Milestone 7: Build Representative Laboratory
- Build the laboratory environment using reference photographs/videos as design guidance
- Use modular assets for furniture, partitions, columns, etc.
- **Stop**: Wait for user confirmation before Milestone 8

### Milestone 8: Add Reusable People and Obstacle Scenarios (split 2026-09-17 — do NOT mark M8 complete until all 5 pass GUI)

- Create reusable pedestrian assets (stationary, walking_straight, crossing path, approaching path, two people, multiple people)
- Create static obstacles (table, chair, column, wall, partition)
- Each scenario specifies: starting position, orientation, velocity, trajectory, start time, end time

#### Historical M8 (capsule, 2026-09-15 — baseline, kept for regression):
- **Status** ✅ COMPLETE capsule — 11/11 `test_m8_scenarios.py` + 15/15 `verify_ackermann` + 56/56 `verify_vehicle`:
  capsule collider `r0.22`, kinematic motion, 6 types, 12 YAML `configs/scenarios/scenario_01..12`, runner headless/GUI, GT (eval only), lab 309 collidable, RGB-D still publishes. **This is NOT the final realistic human** (see § 27) and must not be called "realistic" in final docs — see PART 1.

#### M8.1 Realistic static human (REAL S3; root cause fixed 2026-09-23, GUI still pending):
- Load `S3 /Isaac/People/Characters/{F_Business_02, ...}` via `RealHuman` under `/World/Characters` (SkelRoot + 101-joint skeleton + Skinned Mesh, not capsules)
- Stand naturally, feet on floor `z≈0`, ~1.9m, idle `Action=None` from the real graph
- **Status** ⚠️ PARTIAL headless-STRICT — 2026-09-23 `test_real_human_character.py --headless` **12/12 PASS**: runtime Character STRICT valid, 9 skinned meshes, 101 joints, height 1.96m, foot_z −0.156, arms hang (drop 0.49m, lateral <0.06 — NOT T-pose), idle micro-motion 3.8cm. Root cause of old T-pose found: AnimationGraph discovery is an attach-time scan — stage is now exported+reopened before play (`save_and_reopen_stage_for_animation`), Xform fallback deleted. **GUI pending** — no screenshots captured this session (no DISPLAY); tests hold the window open under `--gui`. Do NOT mark complete until GUI shows standing pose + feet/shoes proportions.

#### M8.2 Realistic moving human (REAL S3 gait; fixed 2026-09-23, GUI still pending):
- Walk via `Action="Walk" + PathPoints + per-frame Walk ramp` on the valid runtime Character (official `base_command.walk` pattern); MotionMatching drives root + gait from one source. Xform fallback DELETED — triggers return False without a Character, tests FAIL instead of sliding.
- **Status** ⚠️ PARTIAL headless-STRICT — 2026-09-23 `test_real_human_walking.py --headless` **14/14 PASS**: Character STRICT, graph walk 4.65m +X, z spread 0.000, thigh ROM 0.128m, foot ROM 0.450m, arm ROM 0.240m, L/R thigh correlation **−0.998** (alternating gait), no T-pose at walk end. Lab `scenario_02_person_walking.yaml --headless` → GRAPH trigger `ok=True` (2,-8→17,-8 corridors). **GUI pending** — need walk capture with visible arm/leg swing.

#### M8.3 Real human NavMesh navigation (2026-09-23: straight-line graph paths; bake pending):
- `NavigationManager` is now created with navmesh DISABLED (no baked volume in this flow) so `generate_goto_path` yields direct segments via the official non-navmesh branch; per-frame `update_path` + arrival tracking mirror the official GoTo command. Destinations propagate from YAML (e.g., `2,-8 → 17,-8`).
- **Status** ⚠️ PARTIAL — graph walk to destination proven (`ok=True`, arrival via root proximity); real NavMesh bake + obstacle-avoidance detour NOT done (straight lines may visually clip furniture; humans have no physics). Mark complete only after baked-volume detour + GUI proof.

#### M8.4 Multiple real humans (visually distinct, 2026-09-17):
- At least 3 simultaneous S3 humans with distinct looks: `CHARACTER_VARIANTS[6]` (`construction_male_01` orange, `business_female_02` suit, `medical_female_01` lab-coat, + 3 police/construction) cycled via `variant_idx` (not hashed identical, not clothing-box fake). Scenario `scenarios/scenario_08_multi_person.yaml` spawns 3 under `/World/Characters`: `person_static_01 variant 0 idle (0,-2)`, `person_walk_01 variant 1 2,-8→12,-8`, `person_walk_02 variant 2 -6,3→4,10` (headless 2/3 active, 3 SkelRoots, 3 meshes). `scenarios/scenario_02_person_walking.yaml` etc. already use distinct variant per YAML. Headless: `scenario_08_multi_person.yaml --headless --duration 5s → 3 REAL S3 humans, 2/3 active, ok=True` each (S3 fetch ~700KB ×3). RGB-D must see all three at once at different depths.
- **Status** ⚠️ PARTIAL headless PASS 2026-09-23 (3 distinct S3 spawned post-reopen, 2 GRAPH walk triggers `ok=True`, 2/3 active, no crash — joint gait proven per-human in M8.2 tests); **GUI pending** — need viewport shot `data/gui_multiple_real_humans.png` showing 3 varied S3 humans animating, not T-posing, not overlapping.

#### M8.5 RGB-D visibility validation (2026-09-17):
- RGB-D is `/World/Vehicle/Chassis/Sensors/RGBD_Camera/RGBD_Camera → /ROS2_Bridge → /camera/rgb/image_raw (rgb8 640×480), /camera/depth/image_raw (32FC1 m)`, verified last `rgb mean ~109, depth 1.8-9.98m, 6 topics`. Static `person at (0,-2)` should appear depth `~2-3m`, walking person `2,-8` moves across frames, multi-person `depth follows position`. Ground truth (`PedestrianGroundTruth position (0,-2,0)... dimensions 0.5×0.5×1.75`) eval-only, never input. Headless `capture_rgb` currently yields placeholder (`22,22,30 grey, /tmp/rep_idle_capture empty`) because headless has no window/viewport — RGB-D validation requires GUI render or external `ros2 topic echo /camera/rgb/image_raw` capture via `--ros2` flag.
- **Status** ⚠️ PARTIAL — 2026-09-23 post-reopen bridge verified LIVE with a real human in scene: 6/6 topics (`rgb/depth/image_raw`, both `camera_info`, `/clock`, `/tf`) + RGB header (`camera_rgb_optical_frame`) + depth header (`camera_depth_optical_frame`) with sim-time stamps (runner now forces `render=True` under `--ros`). Image CONTENT (human pixels/depth) still needs GUI eyes/capture → `data/gui_rgbd_human_view.png` pending.

**M8 overall**: Interim capsule baseline ✅; S3 skeletal animation proven headless-STRICT ✅ (idle 12/12 with no-T-pose + micro-motion; walk 19/19 with 0.47m foot swing + alternating gait; multi 3-human graph triggers; regressions 15/15 + 56/56; ROS topics+frames live). **2026-09-25 integrated validation**: multi-person+vehicle+RGB-D scenario 22/22 (peds 2.25/6.46/6.25 m @0.95-1.03 m/s, vehicle C8R 23.89 m err 0.29 m), ROS checker 10/10, real-GUI RGB evidence in `data/integrated_rgb_*.png` + `data/integrated_overview_gui.png` (X :1, no scripting popup). GUI mid-stride gait capture still pending. Vehicle route C8R validated: START (12,7) → (9.5,5) → (12.5,3) → (12.5,-1) → (11,-4.5) → (7,-6) → GOAL (2,-6), 23.89 m, error 0.29m, 13/13 PASS.

#### M8.6 Integrated multi-person + vehicle + RGB-D (2026-09-25):
- `scenarios/scenario_multi_person_vehicle_gui.yaml` + `run_multi_person_vehicle_gui.py --gui [--ros]`: 3 S3 variants on diverse audited paths + C8R drive + RGB-D + bridge, TEST A-G,K,L 22/22, TEST H-J 10/10 via `check_rgbd_topics.py`, GT eval-only (TEST-K), no perception→steering (Phase 9 gate).
- **Status** ✅ HEADLESS COMPLETE + ⚠️ GUI PARTIAL (real frames captured; mid-stride gait + live inspection pending).

- **Stop**: Wait for user confirmation before Milestone 9

### Milestone 9: Create RGB-D Perception Interface
- Create modular ROS 2 perception interface
- Input: RGB image, Depth image, CameraInfo
- Output: DetectedObject[] (object ID, class, confidence, position, distance, bounding box, timestamp)
- Keep perception modular (not locked to specific AI model)
- **Stop**: Wait for user confirmation before Milestone 10

### Milestone 10: Create Initial Steering Controller
- Create modular steering controller
- Initial implementation: rule-based
- Input: object information
- Output: Ackermann-compatible command
- **Stop**: Wait for user confirmation before Milestone 11

### Milestone 11: Implement Multi-Object Scenarios
- Implement scenarios: person left, person right, person center, two people, person + table, person + chair + column, two moving people, narrow corridor, moving pedestrian crossing path
- Record metrics for each scenario
- **Stop**: Wait for user confirmation before Milestone 12

### Milestone 12: Implement Experiment Logging and Evaluation
- Implement experiment logging
- Record perceptual metrics (detection rate, position error, distance error)
- Record steering metrics (steering angle, response time, lateral deviation)
- Record system metrics (RGB FPS, depth FPS, ROS 2 frequency, latency)
- **Stop**: Final milestone

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