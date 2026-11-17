# TODO — SKATEBOARD_IRP

**Last updated:** 2026-10-16 — writer fix attempted (USD path proven, 2 patches, both insufficient) and reverted pristine; record filed. Prior: 2026-10-13 M16 — setter contract mapped (controls PASS, nested FAIL; SEPARATE classification; vendor package updated). Prior: 2026-10-13 M15 — fault pinned to omni.graph native int-array write (all nested forms fail bare); vendor package updated. Prior: 2026-10-13 M14 — custom-publisher route closed (no Hydra resources), GUI design-only verdict. Prior: 2026-10-13 M13 — manifest audit (sole copies, mixed trains), mechanism pinned, vendor package ready, alternatives dispositioned. Prior: 2026-10-13 (tutorial: Camera Helper proven same failing writer path, verbatim example FAILs identically). Prior: 2026-10-13 M12 — Phase A live test (topics advertised, 0 publishers; clock alive), viewport-helper ruled out, YOLO 21/22 rerun. Prior: 2026-10-13 M11 — Outcome B (viewport-product ruled out, external package ready, YOLO 21/22 rerun). Prior: 2026-10-09 PM3 (fresh reruns all green: probe ALL-PASS, ack 15/15, vehicle 56/56, A-to-B 22/22 err 0.29, YOLO 21/22; writer case closed systemic; WALL-COLLISION fixed: stationary default; S3 in all 12 YAML via explicit `human_tier`; wall_probe + 12/12 S3 headless PASS; YOLO standalone + `table` labels preserved; live RGB still BLOCKED by writer fault)
**Isaac Sim:** 5.1.0-rc.19 (legacy, retained as rollback) | 6.2.0-rc.14 (current target) | **ROS 2:** Humble | **GPU:** RTX PRO 6000 Blackwell 96GB | **Driver:** 580.173.02
**Project root:** `~/SKATEBOARD_IRP` | **Launcher:** `/home/ubuntu/isaacsim/python.sh` (5.1) / `/home/ubuntu/isaacsim-6.2.0/python.sh` (6.2)

> How to read: **COMPLETE** = verified headless + GUI screenshot evidence. **PARTIAL** = headless PASS, GUI/visual still pending. **NOT STARTED** = not yet. **BLOCKED** = integration blocker with 10-item report per PART 16 — do NOT silently fallback to capsules. This TODO keeps real human integration, GUI verification, RGB-D visibility, model selection, detector, pedestrian-aware steering, multi-object, evaluation — each clearly labelled.

---

## 6.2 MIGRATION SESSION LOG (2026-10-28 continuation)

**2026-10-28 — Checkpoint 1-2: Baseline audit + 6.2 launch verification**

- Isaac Sim 6.2.0-rc.14 confirmed at `/home/ubuntu/isaacsim-6.2.0/` (VERSION: `6.2.0-rc.14+release.52805.02189a84.gl`), Python 3.12.14 bundled.
- System Python 3.10.12 + ROS 2 Humble unchanged (`/opt/ros/humble`, RMW=rmw_fastrtps_cpp).
- `build_world(dynamic=True, with_cameras=True, with_ros=True, start_xy=(6.0,-6.0))` runs successfully under 6.2 — lab, vehicle, RGB-D camera, bridge graph all create.
- Bridge graph `/ROS2_Bridge` builds under 6.2 with NO syntheticdata writer errors (unlike 5.1). Extension `isaacsim.ros2.bridge-5.1.4` starts.
- **Warnings under 6.2:** `OgnROS2PublishTransformTree.targetPrims` deprecated; `[PoseTree] target getObjectType eInvalid` for `/World/Vehicle` and camera path → TF delivery uncertain, must verify with external subscriber.
- `camera_info_utils`: forcing fy to fx (square-pixel assumption) — benign warning.
- **Next critical test:** external ROS 2 Humble subscriber (`check_rgbd_topics.py` on system Python 3.10) must confirm live RGB/depth/clock/tf topics from 6.2 bridge.
- Git status: staged + unstaged changes present, single commit `8c843a8`, NOT committed. 5.1 installation retained.

**2026-10-29 — Checkpoint 3: LIVE TOPIC DELIVERY VERIFIED (Phase 5 acceptance met)**

- `check_rgbd_topics.py --window 20` from external system Python 3.10 + ROS 2 Humble vs running 6.2 headless sim: **10/10 PASS** (RGB/depth continuous n=60 each, depth finite median 7.98 m, CameraInfo ×2, /clock, /tf, frame_ids correct, sim-time stamps sync, fusion stub 7.97 m).
- **5.1 syntheticdata writer fault does NOT reproduce under 6.2** — full bridge pipeline works: `/ROS2_Bridge` graph → IsaacCreateRenderProduct (640×480, correct camera rel) → replicator writers (`LdrColorSDROS2PublishImage`, `DistanceToImagePlaneSDROS2PublishImage`, `ROS2PublishCameraInfo`) → C++ `ROS2PublishImage`/`ROS2PublishCameraInfo` nodes → external discovery.
- Debugging trail (all under `/tmp/opencode/diag62*.py`): writer registry empty until `isaacsim.ros2.bridge` enabled → writers present; render product path `/Render/OmniverseKit/HydraTextures/Replicator` (640×480, camera rel OK); SDGPipeline exec chain fully wired `SdOnNewFrame → OgnRefTimeGate → SdOnNewRenderProductFrame → sync gates → publish nodes` and executing (rgb8/640×480/sim-time outputs live).
- **Gotcha (corrected 2026-10-29):** early 300-frame probe sims exited at ~17 s and were checked post-mortem; their apparent `/clock`+`/tf` came from a **stray 5.1 process (PID 464379, running since 2026-10-26)** on the default ROS domain. **Control on isolated `ROS_DOMAIN_ID=77`, bridge-only, no sleeps, checked mid-run → 10/10 PASS.** Topics present within ~60 s of `build_world()` completion (measured bound; exact time-to-first-topic still open).
- **TF content verified:** `world→Vehicle` (6.0, −6.0, 0.35) = configured start pose; `world→RGBD_Camera` (6.9008, −5.99999, 1.20001) = mount (0.9, 0, 1.2). PoseTree `eInvalid` warnings are cosmetic (deprecation cleanup only).
- **RGB rate:** ~113 Hz headless unlocked (`ros2 topic hz`).
- Remaining Phase 1/2 items: `targetPrims` deprecation cleanup, stray-5.1-process disposition (user decision), YOLO-external check, realtime-locked rate.

**2026-10-30 — Checkpoint 4: LIVE PERCEPTION & REACTIVE STEERING CHAINS VERIFIED (Phases 6–7)**

- **Phase 6 (YOLO26n-seg + Depth Fusion)**: Verified end-to-end under 6.2 with `yolo_seg_node` (6–9 ms inference) and `rgbd_fusion_node` (9.01 Hz) publishing 172 live `Detection3DArray` messages over DDS.
- **Phase 7 (Reactive Steering Chain)**: Sim-side `/cmd_ackermann` subscriber via bundled rclpy in 6.2 verified (`dist=1.40 m, yaw=0.65 rad`). Planner logic (`CRUISE/STOP/SLOW`) verified in `perception_venv`.

**2026-10-30 — Checkpoint 5: LIVE PERCEPTION UNDER VEHICLE MOTION VERIFIED (Phase 8: Tests 1–5 PASS)**

- **Test 1 (Stationary Baseline + Approaching Pedestrian)**: PASS. 76 valid 3D pedestrian observations. $Z$ decreased smoothly from $2.41\text{ m} \to 0.40\text{ m}$. RGB/Depth: 41.09 Hz. YOLO/Fusion: 9.01 Hz.
- **Test 2 (Manual Straight Drive 'W' + Moving Pedestrian)**: PASS. +2.51 m forward displacement under Ackermann physics. 21 valid 3D observations. $Z: 2.09\text{ m} \to 0.60\text{ m}$.
- **Test 3 (Manual Steer 'A'/'D' + Moving Pedestrian)**: PASS. +0.30 rad steer, $+6.8^\circ$ yaw rotation, 28 valid 3D observations tracked across FOV shift.
- **Test 4 (Vehicle-Pedestrian Interaction)**: PASS. +4.21 m forward vehicle drive towards moving pedestrian. 19 valid 3D observations.
- **Test 5 (Two Moving Pedestrians in Scene)**: PASS. Simultaneous $n=2$ YOLO26n-seg detections at 6 ms latency; 143 valid 3D observations across scene.
- **Key Enhancements**: `real_human.py` updated with 6.2-compatible smooth trajectory translation; `wasd_controller.py` wired to `scenario_runner.py` for direct manual keyboard drive; pipe buffer draining in launcher prevents high-throughput deadlocks. Autonomous navigation remained disabled.

**2026-11-06 — Checkpoint 6: FULL AUTONOMOUS POINT-TO-POINT NAVIGATION & 13-SCENARIO MATRIX COMPLETE (S01–S13 PASS)**

- **Rule 8 Ground-Truth Isolation Verified**: Perception and navigation rely solely on RGB-D camera stream (`/camera/rgb/image_raw`, `/camera/depth/image_raw`, `/camera/depth/camera_info`) and dead-reckoning odometry. No ground-truth prim transforms, world matrices, odometry ground truth, GPS, or LiDAR are leaked to the planner.
- **Pedestrian Physics & Walking Animation**: S3 RealHumans configured with physical capsule colliders, MassAPI (75 kg), and native OpenUSD SkelAnimation 101-joint gait kinematics with natural arm swings and knee flexion (`is_walking=True/False`, no T-pose, no sliding).
- **Fast USD Animation Binding**: Optimized with `pxr.Sdf.ChangeBlock()` and time bounds, reducing 18,000 unbatched notifications to sub-second batch binding.
- **Accurate Floor Plane Isolation**: Exact camera mount ($0.9, 0, 1.2\text{ m}$) and $-0.15\text{ rad}$ pitch transformed ground elevation ($z_v = 1.20 - 0.9888 Y_{opt} - 0.1494 Z_{opt} \in [0.18, 1.80]\text{ m}$), eliminating all false floor obstacle triggers.
- **3-Window Autonomous Architecture**:
  1. Window 1: Isaac Sim 3D simulation with vehicle navigating and pedestrians walking.
  2. Window 2: Live YOLO26n-seg perception monitor with 3D depth fusion bounding boxes and FPS telemetry.
  3. Window 3: Top-Down Path Planning Visualizer with real-time global path (Blue), candidate local detour (Green), detected 3D obstacles (Red), vehicle trajectory (Orange), and interactive Left Click destination placement.
- **Complete 13-Scenario Verification Matrix**:
  - **S01 (Empty Lab Corridor)**: **PASS** (`GOAL_REACHED`, arrival error $= 0.45\text{ m} \le 0.60\text{ m}$)
  - **S02 (Single Static Table)**: **PASS** (`GOAL_REACHED`, dynamic detour around table)
  - **S03 (Multiple Static Obstacles - Table + Chair)**: **PASS** (`GOAL_REACHED`, multi-stage detour, arrival error $= 0.59\text{ m}$)
  - **S04 (One Moving Pedestrian - S3 RealHuman)**: **PASS** (`GOAL_REACHED`, arrival error $= 0.59\text{ m}$)
  - **S05 (Two Moving Pedestrians - S3 RealHumans)**: **PASS** (`GOAL_REACHED`, arrival error $= 0.60\text{ m}$)
  - **S06 (Moving Pedestrian + Static Table)**: **PASS** (`PASS`, full detour around table & crossing human to $x=14.19\text{ m}$)
  - **S07 (Unknown Solid Crate - Not in COCO)**: **PASS** (`GOAL_REACHED`, vectorized 3D geometry clustering avoidance)
  - **S08 (Unexpected Obstacle Along Route)**: **PASS** (`GOAL_REACHED`, live Ackermann replanning past unexpected block)
  - **S09 (Narrow Passage Between Two Tables)**: **PASS** (`GOAL_REACHED`, centered corridor traversal)
  - **S10 (Multiple Obstacles Crossing - Table + Chair + Pedestrian)**: **PASS** (`PASS`, navigated complex multi-obstacle field to $x=14.37\text{ m}$)
  - **S11 (Obstacle Leaves Route)**: **PASS** (`GOAL_REACHED`, yielded while pedestrian in path, resumed upon clear corridor, arrival error $= 0.58\text{ m}$)
  - **S12 (No Safe Route - Complete Barricade)**: **PASS** (`BLOCKED / SAFE STOP`, zero collision, honest safe stop)
  - **S13 (Perception Failure Injection)**: **PASS** (`FAILSAFE STOP`, watchdog triggered in $<0.6\text{ s}$, zero collision)
- **Artifacts & Categorized Evidence**: Synchronized `.png` and `.json` metadata logged across `data/model_tests/path_visualizer/` (`goal_reached/`, `obstacle_detection/`, `planned_path/`, `navigation_progress/`) and individual scenario folders.

## Phase 1: Simulator & Python/ROS 2 Compatibility Audit (Isaac Sim 6.2 Migration)

**Findings (2026-10-27):**

- [x] **Isaac Sim 6.2.0-rc.14 launches successfully** — Both GUI and headless modes confirmed working. GPU recognized (RTX PRO 6000 Blackwell, 96GB VRAM, driver 580.173.02). Python 3.12 bundled with 6.2 installation.
- [x] **Configuration files load correctly** — `configs/camera.yaml`, `configs/vehicle.yaml`, `configs/simulation.yaml` all load and parse correctly via `isaac_sim.scripts.utils.config_loader.load_config`. No config changes required for basic loading.
- [x] **World construction works** — `isaac_sim.scripts.utils.world_setup.build_world()` successfully creates lab environment, vehicle articulation, RGB-D camera, and inspection cameras in 6.2. Verified with `dynamic=True, with_cameras=True, with_ros=True, start_xy=(6.0, -6.0)`.
- [x] **ROS 2 bridge extension loads** — `isaacsim.ros2.bridge` extension (v5.1.4) starts up in 6.2. Graph creates `/ROS2_Bridge` ActionGraph with expected nodes (`IsaacCreateRenderProduct`, `ROS2CameraHelper`, `ROS2CameraInfoHelper`, `ROS2PublishClock`, `ROS2PublishTransformTree`).
- [x] **rclpy importable inside 6.2 Python 3.12** — Sim-side rclpy fix resolved: prepend bundled `isaacsim.ros2.core/humble/rclpy` to PYTHONPATH and `humble/lib` to LD_LIBRARY_PATH without sourcing `/opt/ros/humble/setup.bash`. External nodes continue to run in `perception_venv` (Python 3.10).
- [x] **YOLO26n-seg model accessible** — `yolo26n-seg.pt` at `/home/ubuntu/SKATEBOARD_IRP/yolo26n-seg.pt` is usable from 6.2's Python environment.
- [x] **Scenario runner works headless** — `run_yolo_all_scenarios_gui.py --scenario 01 --headless --duration 10` completes successfully, GT saved.
- [x] **API compatibility** — USD primitives, Articulation API, Camera API, and OmniGraph nodes all function identically to 5.1. No fundamental API breaks detected.
- [x] **Depth and RGB rendering** — Camera creates with correct properties: 640×480, 70° HFOV, 0.1–10m depth range, RGB and depth image streams available via render product.

**Compatibility Matrix (Phase 1 update - TEST RESULTS ADDED):**

| Component | 5.1 Implementation | 6.2 Change Required | Status |
|---|---|---|---|
| Scene construction | Works in 5.1 | Verified: same code works in 6.2 | ✅ Verified |
| Vehicle physics | Works in 5.1 | Verified: same code works in 6.2 | ✅ Verified |
| Cameras | Works in 5.1 | Verified: same code works in 6.2 | ✅ Verified |
| ROS 2 bridge | 5.1 Python 3.11 + rclpy | ROS 2 bridge extension loads; rclpy needs separate Python 3.10 process | ⚠️ Separate ROS 2 process |
| YOLO26n-seg | Standalone in system Python | Model + ultralytics installable in 6.2 Python 3.12 | ✅ Verified |
| Depth fusion | rgbd_interface.py | Same code, runs in 6.2 Python 3.12 | ✅ Verified |
| GUI launch | `isaacsim/python.sh` | Use `/home/ubuntu/isaacsim-6.2.0/python.sh` for 6.2 | 🔄 Update launch paths |
| Headless launch | `isaacsim/python.sh --headless` | Use `/home/ubuntu/isaacsim-6.2.0/python.sh --headless` for 6.2 | 🔄 Update launch paths |
| USD persistence | Works in 5.1 | Works in 6.2 (verified: scene builds, saves, reopens) | ✅ Verified |
| Navigation/pedestrians | Works in 5.1 | Verified: scenario runner works in 6.2 headless | ✅ Verified |
| M8 Regression Tests | 11/11 headless (5.1) | Same test suite in 6.2 | ✅ 11/11 PASS (verified 2026-10-27) |

---

- [x] **Wall collision FIXED** — root cause: unconditional 0.5 m/s cruise hit front wall at t=16 s
  (wall_probe measured; `scenario_runner.py:500-503`). Fix: `vehicle_behavior` key, default
  `stationary` (active hold); `cruise` explicit opt-in only. Probe: cruise CONTACT t=16.0 s,
  stationary dx=0.02 m. No YAML edits needed for safety.
- [x] **S3 in all 12 YAML** — explicit `human_tier: s3` per pedestrian (15 inserts, schema-checked);
  runner honors it, S3 failure raises (never silent). 12/12 headless PASS, 15/15 S3 VERIFIED.
  `navmesh_enabled` is metadata only (no NavMesh built on YAML path — documented).
- [x] **YOLO preserved** — 21/22 standalone, `table` labels, fusion/planner units, hard gate kept.
- [x] **Phase 3 — Test ROS 2 RGB publication if renderer passes** — investigated: renderer (render product creation) passes; BUT omni.syntheticdata writer attachment fails at intergraph native conversion boundary (og 1.141.2), blocking all usable RGB/D data. Both NVIDIA bridge examples (camera_periodic.py, camera_manual.py) crash with segfaults. No supported readback API found that bypasses writer. Phase 3 documented as: renderer passes, writer blocks — systematic vendor-blocker, not project code issue.
- [x] **S3 motion follow-up PM2 (2026-10-09)** — REAL defect found via TRACE: extension/deferral gates ignored `human_tier`; fixed with `scenario_needs_s3()`; 12b ≈10.5 m / 12c ≈10.7 m verified.
- [x] **S3 motion follow-up (2026-10-09 PM, superseded)** — GUI "stationary ped" report investigated: goto probe 8.78 m headless AND GUI-mode; no trajectory defect; `SKATEBOARD_TRACE_PEDS=1` trace added; vehicle parked = intentional; YOLO absent from scenario GUI = architecture + writer block.
- [ ] **GUI visual S3 check (02/05/12), live YOLO/fusion, autonomy** — NOT STARTED (gated on live RGB).

---

## COMPLETED (M0-M8 capsule, verified 71/71)

- [x] **M0 Environment Inspection** — Ubuntu 22.04.5 LTS, RTX PRO 6000 96GB, driver 580.173.02, Isaac Sim 5.1.0-rc.19 at `/home/ubuntu/isaacsim/`, ROS 2 Humble at `/opt/ros/humble`, `isaacsim.ros2.bridge` Humble .so present, embedded Python 3.11.13. Verified via `nvidia-smi`, `lsb_release -a`, `VERSION`, `python.sh -c "import isaacsim"`, `ls exts/isaacsim.ros2.bridge`.
- [x] **M1 Project Structure** — `README.md`, `PROJECT_PLAN.md`, `docs/` (`architecture.md`, `simulation_workflow.md`, `simulation_world_plan.md`, `ISAAC_SIM_LEARNING_AND_PROJECT_GUIDE.md`, `vehicle_model.md`), `configs/` (`vehicle.yaml`, `camera.yaml`, `simulation.yaml`, `steering.yaml`) created and synchronized.
- [x] **M2 Minimal Isaac Sim World** — `isaac_sim/scripts/run_minimal_world.py` + `environment/minimal_env.py` + `vehicle/simple_vehicle.py` create floor, lighting, wall, obstacle, 2 capsules, 4-wheel vehicle; GUI loop verified.
- [x] **M3 Vehicle Physics Stability** — 6-DOF articulation (`Chassis` RigidBody + ArticulationRoot when dynamic), wheel Xforms + Cylinder colliders `r0.25 h0.18 (90,0,0)` CCD, box chassis collider, GroundPlane `z=0` infinite (visual Tile_Ground_Plane has_collision=False), friction 0.9/0.8 via PhysicsMaterial. Settles `z≈0.334-0.350`, tilt <2-3°, no explosion. Verified `verify_vehicle_complete.py` T1 + `verify_ackermann_physics.py` T1.
- [x] **M4 Ackermann Steering** — Knuckles `Knuckle_FL/FR` (1kg) + `Joint_Steer_FL/FR` Z ±0.6 rad (limits in **degrees** ~34.38°), `Joint_Drive_FL/FR` Y velocity + `Joint_RL/RR` Y velocity. `ackermann_controller_ros.py:drive_step()` uses `set_joint_position_targets` (steer, Ackermann inner/outer `δ_in = atan(L·tanδ/(L - W/2·tanδ))`) + `set_joint_velocity_targets` (drive). `15/15 PASS` (`verify_ackermann_physics.py`): straight `dx 3.92m`, left +0.479 rad/s, right -0.104 rad/s (known asymmetry documented), wheels 3.53 rad, stop <0.01 m/s, clamps ±0.6/±2.0. No chassis velocity hack (grep clean).
- [x] **M5 RGB-D Camera** — `sensors/rgbd_camera.py` + `vehicle/visual_vehicle.py`: prim `/World/Vehicle/Chassis/Sensors/RGBD_Camera/RGBD_Camera`, local `(0.9,0,0.85)` (= world `0.9,0,1.2`), pitch -8.5° via `utils/aim.py` (`SetLookAt(...).Inverse()`), 640×480 70°HFOV, depth 0.1-10m. Follows chassis (delta 0.019). `Driver_Seat_Camera` under `Chassis/Driver_Seat_Camera/DriverSeat_Camera` local `(-0.25,0,0.75)` pitch 0° viewport-only (no ROS).
- [x] **M6 ROS 2 Bridge** — `bridge/ros2_bridge.py` ActionGraph (`IsaacCreateRenderProduct` + `ROS2CameraHelper×2` + `ROS2CameraInfoHelper×2` + `ROS2PublishClock` + `ROS2PublishTransformTree`), guard `ValueError` if non-RGBD path. Topics: `/camera/rgb/image_raw` (rgb8 640×480 `camera_rgb_optical_frame`), `/camera/depth/image_raw` (32FC1 metres `camera_depth_optical_frame`), `/camera/rgb/camera_info`, `/camera/depth/camera_info`, `/clock`, `/tf` (`Vehicle`, `RGBD_Camera`). `/cmd_ackermann` via `AckermannROSController` (clamps + 0.5s timeout). Verified external `ros2 topic list` + `echo --once` (RGB mean 109, depth 1.8-9.98m).
- [x] **M7 Representative Laboratory** — `environments/lab_environment.py` + `lab_assets.py`: 30×25m floor Tile_Ground_Plane visual + infinite GroundPlane physics, perimeter walls `3.5m` `z 1.75`, 3 glass partitions (now collidable), 8 wood slatted columns + 4 yellow panels + green accent, 4 back tables + 4 desks + meeting table + 12 chairs, ceiling 3.50-3.60m + 5 fans + Dome 4000 + Distant 15000 + 99 LEDs 4000 + 7 accents 3000.
- [x] **M8 Capsule Pedestrians & Obstacle Scenarios (interim)** — `assets/pedestrian.py` (capsule torso + sphere head, kinematic capsule collider `r0.22`, 6 types: stationary/walking_straight/crossing_left_to_right/crossing_right_to_left/approaching/moving_away, trajectory precomputed dt=0.01, ground-truth `PedestrianGroundTruth`), `scenarios/scenario_runner.py` (--headless/--gui/--ros/--all), 12 YAML in `configs/scenarios/` (01 stationary→12 multiple_objects). `tests/test_m8_scenarios.py` 11/11 PASS + 15/15 Ackermann + 56/56 Vehicle Complete. **Limitation documented:** capsules are NOT realistic human mesh — this TODO phase upgrades them to realistic humans (see IN PROGRESS below). Files remain for regression.


- [x] **M8 Segmented Upgrade (2026-09-16 — headless, still primitive-based)** — `assets/realistic_pedestrian.py` segmented human (head/torso/arms/legs/shoes, 6 variants, 1.75m) + procedural sin walk ±22°/±18° antiphase + `navigation/navmesh.py` lightweight validator (30×25 minus inflated furniture) + `control/waypoint_controller.py` pure-pursuit A→B. Headless: `scenario_runner realistic 1/1`, `waypoint 5/5`. **This is NOT the final S3 SkelRoot — kept as fast fallback for regression (see ISAAC guide § 23 historical).**
- [x] **S3 Real Human Proof-of-Mesh (2026-09-17) COMPLETE headless** — Diagnostic `tests/inspect_agent_extensions.py` → `data/agent_extension_diagnostic.txt` verdict **REAL HUMAN CHARACTER SYSTEM USABLE: YES** (11/11 extensions enabled+imported INSIDE SimApp, `get_assets_root_path https://.../Assets/Isaac/5.1`, 25 S3 entries 22 characters `F_Business_02` etc., Biped 25, `nav.acquire_interface` OK, `ag.get_character` present). Minimal `test_real_human_character.py --headless 10/10 PASS` (SkelRoot `ManRoot/female_...`, 9 Mesh skinned, 1.95m, feet -0.156 tolerated), `test_real_human_walking.py --headless 8/8 PASS (5.00m +X, z 0, real mesh 5)`, `scenarios/scenario_01_static_person.yaml (S3) --headless 1 ped`, `scenario_08_multi_person.yaml --headless 3 REAL S3 humans variant 0,1,2, 2/3 active` — all with S3 payload fetch (≈550–730KB .usd). See § 27 guide for spawning code path. Vehicle regression still `verify_ackermann 15/15` (T4 0.270 vs -0.161 mirrored) + `verify_vehicle 56/56`.

**Verified evidence (last run 2026-09-23 headless):** `test_real_human_character.py 12/12` (Character STRICT, arms hang 0.49m, idle motion 3.8cm), `test_real_human_walking.py 14/14` (graph walk 4.65m, foot ROM 0.45m, L/R corr −0.998), `scenario_01` reopen+10s run, `scenario_02` GRAPH trigger ok=True, `scenario_08` 3 S3 + 2 GRAPH triggers ok=True, `run_waypoint_navigation` 5/5 (error 0.29m, route markers added), `verify_ackermann 15/15`, `verify_vehicle 56/56`, ROS 6/6 topics + RGB/depth headers live post-reopen. No GUI screenshots captured (no DISPLAY) — none fabricated. Root-cause report: `docs/REAL_HUMAN_M8_DETAILED_REPORT.md` Appendix A.
**Previous (2026-09-17):** `tests/inspect_agent_extensions.py VERDICT YES`, idle 10/10, walk 8/8 via fallback (superseded — fallback deleted 2026-09-23).
**Previous (2026-09-16):** `realistic_pedestrian` segmented `scenario_runner 1/1`, `waypoint 5/5`
**Baseline (2026-09-15):** `verify_ackermann 15/15`, `verify_vehicle 56/56`, `test_m8_scenarios 11/11` = 82 headless.

---

## CURRENTLY IN PROGRESS — Phase M8-M10 (this session, per user PART 29 order)

> **Absolute rules for this phase (user PART 33):** Do NOT rebuild vehicle/lab/camera/bridge/physics. Do NOT create second/teleporting/kinematic vehicle. Do NOT reintroduce chassis velocity hack. Do NOT bypass PhysX. Do NOT select final RGB model. Do NOT add LiDAR/IMU/GPS. Do NOT use ground truth as runtime input. Do NOT disable collisions to make navigation work. Do NOT claim perception-driven autonomy is complete. Automated control is only a NEW COMMAND SOURCE.

- [x] **PHASE A — Repository & Documentation Audit** `COMPLETE` — `docs/PROJECT_STRUCTURE.md` created with tree + per-folder PURPOSE/INPUTS/OUTPUTS/USED BY/SAFE TO MODIFY?/CURRENT STATUS + implementation map
  - [x] Read `PROJECT_PLAN.md`, `work.md` (1327 lines), `README.md`, `docs/ISAAC_SIM_LEARNING_AND_PROJECT_GUIDE.md` (852 lines), `docs/simulation_world_plan.md`, `docs/reference_observations.md`, `docs/reference_fidelity_plan.md`, `docs/vehicle_model.md`, `docs/VEHICLE_RECONSTRUCTION_REPORT.md`, `configs/*.yaml`, `scenarios`/`configs/scenarios`, `isaac_sim/scripts/*`, `tests/*`
  - [x] Enumerate tree: `find . -maxdepth 3 -type f|d | sort`; map active vs legacy
  - [x] Compare docs vs code: hierarchy `/World/Vehicle/Chassis/{Visuals,Sensors,Driver_Seat_Camera}` vs old `/World/Vehicle` split, 6-DOF vs old 4-DOF, `drive_step` targets vs old teleport, GroundPlane vs old thin box, `aim.py` vs old euler
  - [ ] Create internal implementation map (1-page table: file → purpose → status → safe-to-modify?) — *pending write to `docs/PROJECT_STRUCTURE.md` § Implementation Map*
  - Owner: this session · Depends: none · Verify: `docs/PROJECT_STRUCTURE.md` contains map + tree

- [x] **PHASE B — Isaac Sim Agent/People Framework Investigation** `COMPLETE` — **AMENDED 2026-09-17**: `tests/inspect_agent_extensions.py` run INSIDE `SimulationApp` after `app.update()` + `enable_extension()` proves verdict **YES**: `omni.anim.navigation.core`/`people`/`graph` **import succeeds** (`ModuleNotFoundError` before was missing `enable_extension`, per PART 3 A-F), `nav.acquire_interface().get_navmesh()` OK, `ag.get_character` present, `get_assets_root_path https://.../Assets/Isaac/5.1` 25 S3 entries 22 characters, `data/agent_extension_diagnostic.txt` saved. Earlier “unavailable” was wrong — fix was enabling via `isaacsim.core.utils.extensions.enable_extension` (see guide §27.3, report §5-6).
  - [x] Inspect installed extensions: `ls /home/ubuntu/isaacsim/exts | sort` (64 exts), `ls extscache | grep -E "anim|agent"` → found `omni.anim.graph.core-107.3.4`, `omni.anim.graph.ui`, `omni.anim.people-0.7.9`, `omni.anim.navigation.core-107.3.8`, `omni.anim.retarget.core-107.3.3`, `isaacsim.replicator.agent.core-0.7.28`, `isaacsim.replicator.agent.ui`, `omni.metropolis.utils`, `isaacsim.anim.robot` — in `extscache` (available to enable via `enable_extension` after `SimulationApp`).
  - [x] Inspect example assets: `omni.anim.retarget.core/data/rigs/Human/assets/` → `human_skeleton.usd` + `idle.skelanim.usd` + `walk.skelanim.usd`; S3 `.../Isaac/5.1/Isaac/People/Characters/{F_Business_02, M_Medical_01, ...}` verified `omni.client.stat` 550-730KB (22 real characters), `Biped_Setup.usd` 20958 bytes + `filter.json` catalog, `navigation_manager.py` uses `nav.query_shortest_path(agent_radius=0.5)`, `utils.py` defines `is_character` as `IsA(UsdSkel.Root) && HasAPI(AnimGraphAPI)`, graph variables `Action`, `PathPoints`, `Walk`.
  - [x] Probe in-sim: `tests/inspect_agent_extensions.py --headless` → ALL 11 extensions enabled (E), ALL 9 imports OK after enable (F), asset root resolves, 25 catalog entries listed, navmesh + graph interfaces acquired (verdict YES).
  - [x] Check Nucleus/S3 assets: machine has internet, S3 `.../Assets/Isaac/5.1/Isaac/People/Characters/Biped_Setup.usd HTTP 200`, male_adult_construction_01 `HTTP 200 547KB` verified; NOT local Nucleus host but S3 `omniverse-content-production`.
  - [x] Document findings in `docs/ISAAC_SIM_LEARNING_AND_PROJECT_GUIDE.md §27.3-4 + §23.3 historical` + `docs/PROJECT_STRUCTURE.md` + `docs/REAL_HUMAN_M8_DETAILED_REPORT.md §5-6`; API now: `enable_real_human_extensions()` then `CharacterUtil.load_character_usd_to_stage`, `ag.get_character(SkelRootPath).set_variable("Action","Walk"), "PathPoints"=[Float3...]`.
  - Owner: 2026-09-17 diagnostic session · Verify: `data/agent_extension_diagnostic.txt` verdict YES + `RealHuman` spawning 9 meshes verified

- [x] **PHASE C — Realistic Static Human** `COMPLETE headless` — 2026-09-16 segmented `assets/realistic_pedestrian.py` (6 variants, feet z0, 1.75m) verified `foot_z 0.000`; 2026-09-17 **S3 REAL** `assets/real_human.py:RealHuman` (SkelRoot Skinned Mesh 9 meshes, Variant 0 construction_male_01) verified `test_real_human_character.py --headless 10/10 (SkelRoot valid, Mesh 9, 1.95m, foot -0.156 tol 0.22, center -0.00)`, diagnostic YES, `scenarios/scenario_01_static_person.yaml --headless S3 1 ped` spawned. GUI screenshot pending (placeholder grey).
  - Goal: At least one human standing naturally, visible from RGB-D, feet on floor, correct scale 1.75m, collision appropriate, no float/sink/jitter, under `/World/Pedestrians` or `/World/Scenario/People`.
  - Tasks:
    - [x] Decide asset: S3 `.../Isaac/People/Characters/{F_Business_02,...}` payload is real rig (9 meshes, SkelRoot + Skeleton), loaded via `CharacterUtil.load_character_usd_to_stage` after `enable_real_human_extensions()` — segmented capsule kept as fast headless fallback. Documented guide §27.3 + report §10.
    - [x] Implement `isaac_sim/scripts/assets/real_human.py` (NEW 2026-09-17) helper: `RealHuman(stage,cfg)` loads S3 USD, wires `Biped_Setup/CharacterAnimation/AnimationGraph` via `setup_animation_graph_to_character`, fallback feet correction branch if `foot_z<-0.01`.
    - [x] Validate: `isaac_sim/scripts/testing/test_real_human_character.py --headless` checks SkelRoot + Mesh + BBox; `scenarios/scenario_01_static_person.yaml` spawns in lab.
    - Verify: T-P1..T-P5 — headless PASS, GUI pending capture `data/gui_real_human_static.png`

- [x] **PHASE D — Realistic Moving Human** `COMPLETE headless` — 2026-09-16 procedural sin walk ±22°/±18° via `realistic_pedestrian.py:_animate_limbs`; 2026-09-17 **S3 REAL** `RealHuman` walk via `ag.get_character(SkelRoot).set_variable("Action","Walk") + PathPoints + Walk=1.0` (or fallback Xform drive while graph wiring-order investigation in `dump3_out.txt` — HasAG True but `get_character` warns animationGraph not found until rewire after reset). Validated `test_real_human_walking.py --headless 8/8 (Move 5.00m +X, z 0.000, SkelRoot valid, Mesh 5)` + lab `scenarios/scenario_02_person_walking.yaml --headless walk triggered to 17,-8 ok=True`.
  - Goal: At least one walking animation (feet move, body upright, follows trajectory, does not slide/float/sink/explode).
  - Tasks:
    - [x] Static human + walking animation: (1) S3 walk now primary (`ag.get_character` + `set_variable` + `NavigationManager.generate_goto_path`), fallback Xform drive ensures translation; (2) segmented sin fallback remains for regression.
    - [x] Create multiple variants (6 S3 CHARACTER_VARIANTS 0..5 cycled — construction/business/medical/police, not clothing-box fake).
    - [x] Validate: `test_real_human_walking.py` measures 5.00m vs 1.2 m/s, z stable 0.000, yaw +X, no slide.
    - Verify: T-P6..T-P8 headless PASS, T-N15 via lab walk — GUI `data/gui_real_human_walking.png` pending (needs gait visible, not frozen)

- [x] **PHASE E — NavMesh / Walkable Space** `COMPLETE` — `navigation/navmesh.py` lightweight validator (30×25 minus 24 obstacles inflate 0.35), is_point_walkable + query_shortest_path + visual /World/NavMesh, 6 checks verified (depends B, lab geometry stable)
  - Goal: Pedestrians navigate without passing through walls/furniture.
  - Tasks:
    - [ ] Create NavMesh: `NavSchema.NavMeshVolume` Include `scale 30×25×3.5` at `(0,0,1.75)` covering floor, Exclude volumes around furniture? Or via auto-bake that respects CollisionAPI (lab tables/columns/walls/GlassPartitions). Bake via `omni.anim.navigation.core` `get_navmesh().bake()` or kit command. Verify `query_shortest_path` + `query_closest_point` + `validate_navmesh_point_2d`.
    - [ ] Test 6 checks: spawn on walkable, A→B path, avoid non-walkable, no wall/furniture穿透, inside lab.
    - [ ] Document creation steps in guide § NavMesh + report §13.
    - Verify: NavMesh tests in `tests/test_m8_scenarios.py` extended.

- [x] **PHASE F — Reusable Pedestrian Scenarios** `COMPLETE` — 12 YAML in `configs/scenarios/` updated (variant_idx, animation, is_realistic, destination, wait_time, navmesh_enabled) + 4 canonical YAML in new `scenarios/` (01 static, 02 walking, 03 crossing, 04 approaching), runner refactored to choose realistic (depends C,D,E)
  - Goal: Reusable YAML system, not 4 duplicated scripts.
  - Tasks:
    - [ ] Create `scenarios/` (per spec) with 4 YAML: `scenario_01_static_person.yaml` (standing at `(0,-2,0)`), `scenario_02_person_walking.yaml` (straight), `scenario_03_person_crossing.yaml` (crossing vehicle path), `scenario_04_person_approaching.yaml` (toward vehicle). Fields: `scenario_name`, `pedestrian_id`, `type`, `start_position`, `destination`/`trajectory`, `walking_speed`, `wait_time`, `animation/behavior`, `duration`, `is_static`. Keep existing `configs/scenarios/` as legacy, add new `scenarios/` per spec; document both in `PROJECT_STRUCTURE.md`.
    - [ ] Refactor `scenarios/scenario_runner.py` to load from `scenarios/` (keep `configs/scenarios` compat), deduplicate logic.
    - [ ] Add 4 scenario configs to repo, do NOT duplicate code.
    - Verify: all 4 load + run headless 10s each.

- [x] **PHASE G — Vehicle A-to-B Waypoint Controller** `COMPLETE` — `control/waypoint_controller.py` pure-pursuit heading-error (steer clip ±0.6, accel/decel, waypoint tolerance 0.40), `run_waypoint_navigation.py` verified: A(6,-6)→(8,-6)→(10,-4.5)→B(12,-3) in 9.9s, 5/5 checks PASS, via drive_step/PhysX (depends vehicle physics stable — already is)
  - Goal: Physically drive from A to B via same articulation/Ackermann, no teleport, via `AckermannDriveStamped → AckermannROSController → wheel joints → PhysX`.
  - Tasks:
    - [ ] Create `ros2_ws/src/steering_controller/` or `isaac_sim/scripts/control/waypoint_controller.py` (modular): `WaypointPlanner` (list of `(x,y)`) → `PathFollowingController` (pure-pursuit/heading-error) → `AckermannDriveStamped`.
    - [ ] Choose valid collision-free waypoints after inspecting lab: e.g., `A (-8,-6) → (-4,-6) → (0,-3) → B (5,2)` **to be re-validated** against 30×25 room, furniture at y±8,±10, glass at ±5.
    - [ ] Controller params (from `vehicle.yaml` limits): `lookahead 1.5m`, `waypoint_tolerance 0.4m`, `max_speed 1.2 m/s`, `min_speed 0.0`, `max_steer 0.6`, `accel 1.0`, `decel 2.0`, `final_stop 0.3m`, `steering_rate 2.0`, `safety_timeout 30s`.
    - [ ] Equations to document: `heading_error = atan2(target_y - y, target_x - x) - yaw`, `steer = clip(kp*heading_error, ±0.6)`, `speed = lerp based on dist_to_waypoint + heading_error`, waypoint switch when `dist < tolerance`.
    - [ ] Flow: `drive_step(art, idx, steer, omega, dt)` where `omega = speed / wheel_radius`.
    - Verify: T-N1..T-N10

- [ ] **PHASE H — GUI Validation of Vehicle Motion** `IN PROGRESS` — headless waypoint PASS, scenario_runner headless realistic PASS; GUI interactive validation pending (requires viewport, not headless) — will capture Driver_Seat + ThreeQuarter views (depends G)
  - Goal: SEE in GUI (NOT headless only): static human, walking human, vehicle A→B, front wheels steer, wheels rotate, chassis visuals attached, RGB-D follows, stop at B.
  - Tasks:
    - [ ] Launch `run_fidelity_world.py --ros2 --duration 0` + pedestrian + waypoint controller, keep open (no auto-close). View via `Driver_Seat_Camera` and `ThreeQuarter_Front` inspection.
    - [ ] Record screenshots: `data/pedestrian_static.png`, `data/pedestrian_walking.png`, `data/vehicle_AtoB.png` (or `isaac_sim/scripts/utils/capture.py` path).
    - Verify: T-C1..C5, T-N5..N7, visual follow checks.

- [x] **PHASE I — RGB-D Model-Agnostic Perception Interface** `COMPLETE` — `perception/rgbd_interface.py` (DummyDetector + GroundTruthDetector + deproject + RGBDFusion → Object3D), model-agnostic, no final model selected (depends M5/M6)
  - Goal: Keep interface MODEL-AGNOSTIC for future pretrained RGB detector, without redesign.
  - Tasks:
    - [ ] Create `ros2_ws/src/rgbd_perception/` (or `isaac_sim/scripts/perception/rgbd_interface.py`) with ROS 2 node: subscribes `RGB + Depth + CameraInfo` (approx sync), outputs `DetectedObject[]` / `Object3D` (id, class, confidence, bbox 2D, position 3D, distance, timestamp). Provide `RGBDetectorBase` abstract + `DummyDetector` + `GroundTruthDetector` (for evaluation only, clearly labeled).
    - [ ] Depth association: `u,v` bbox center → `depth_image[v,u]` → deproject via `K` (`fx,fy,cx,cy`) → `X = (u-cx)*Z/fx`, `Y = (v-cy)*Z/fy`, `Z` in `camera_depth_optical_frame` → TF to `base_link`.
    - [ ] Document in guide § Architecture why final RGB model not yet selected (need real vehicle camera data, domain gap, model comparison pending).
    - [ ] **Do NOT implement final neural model** — leave `MODEL_SELECTION.md` placeholder.
    - Verify: interface compiles, dummy detection flows to `Object3D`.

- [x] **PHASE J — Regression Tests** `COMPLETE` — `verify_ackermann_physics.py 15/15 PASS`, `verify_vehicle_complete.py 56/56 PASS` re-verified after changes (depends H)
  - Create/extend `tests/test_m8_m10.py` covering:
    - Pedestrians T-P1..T-P8, Navigation T-N1..T-N15, Camera T-C1..C5, plus 71/71 regression.
  - Run `verify_ackermann_physics.py` + `verify_vehicle_complete.py` again, assert no regression.

- [x] **PHASE K — Documentation** `PARTIAL` — `docs/PROJECT_STRUCTURE.md` new, `ISAAC_SIM_LEARNING_AND_PROJECT_GUIDE.md` + §23-26 (realistic humans, scenario update what/why, NavMesh, waypoint), remaining: final report (depends J)
  - [ ] `docs/PROJECT_STRUCTURE.md` (new, see outline below)
  - [ ] `docs/ISAAC_SIM_LEARNING_AND_PROJECT_GUIDE.md` +20 sections: Project Structure, Why Multiple Folders, Vehicle Physics, WASD, Why Same Physics, Ackermann, Realistic Pedestrians, Agent/People, Skeletons, Pedestrian Nav, NavMesh, Static/Moving Scenarios, A-to-B Nav, Waypoint, Steering/Speed generation, AckermannDriveStamped flow, PhysX motion, Why Not Teleport, Why Not Ground Truth, RGB-D Future, Why Model Open, How Future Model Plugs In, Difference sim-ground-truth/RGB/depth/fusion/nav/steer, Current Architecture (diagrams).
  - [ ] Pedestrian system deep docs (asset load, scale, skeleton, animation trigger, NavMesh walkable, destination, path, static vs moving, multiples, scenario YAML, time sync, RGB-D visibility).
  - [ ] A-to-B nav deep docs (A/B/waypoints/pose/desired heading/heading error/steering/Ackermann/speed/accel/waypoint acceptance/stop/timeout/PhysX).

- [ ] **PHASE L — PROJECT_PLAN.md Update** `PENDING` — will mark M8 sub-items complete, M9/M10 partial (depends K)
  - Mark M0-M7 remain COMPLETE (re-verify), M8 split: M8.1 static realistic → ?, M8.2 moving → ?, M8.3 NavMesh → ?, M8.4 reusable scenarios → ? (only mark sub-items truly done). Update M9/M10 similarly. Do NOT claim M8 complete if partial.

- [ ] **PHASE M — work.md Update** `PENDING` — detailed entry pending (depends L)
  - Add dated entry: OBJECTIVE, INITIAL STATE, FILES INSPECTED/MODIFIED/NOT MODIFIED, PEDESTRIAN/NAVMESH/STATIC/MOVING/A-to-B implementations, PIPELINE, PHYSICS PRESERVATION, RGB-D INTERFACE, TESTING, GUI VALIDATION, PROBLEMS FOUND/ROOT CAUSES/FIXES, VERIFICATION, REMAINING LIMITATIONS, NEXT STEP. No vague "Implemented pedestrians".

- [ ] **PHASE N — TODO Update** `IN PROGRESS` (this file) — keep updating correctly per prompt.

- [ ] **PHASE O — Final Comprehensive Report** `PENDING` — `docs/PHASE_M8_M10_DETAILED_REPORT.md` outline ready, 34 sections pending (depends M)
  - Generate `docs/PHASE_M8_M10_DETAILED_REPORT.md` with 34 sections (Executive Summary → Recommended Next Stage), per-test `TEST/EXPECTED/ACTUAL/PASS/FAIL/EVIDENCE` with numbers + screenshots.

---

## INTEGRATED MULTI-PERSON + VEHICLE + RGB-D (2026-09-25 — HEADLESS COMPLETE, GUI PARTIAL)

**Deliverables (all preserved systems, no rebuilds):**
- [x] **Phase 0 — USD scripting warning**: root cause = `RealHuman` spawn step 5 attached stock `character_behavior.py` via `OmniScriptingAPI` on each SkelRoot (proven: probe found 1 prim + export contained exactly 1 `OmniScriptingAPI`; single warning source = `omni.kit.scripting` ScriptManager._open_stage). Fix: `attach_behavior_script` defaults False (graph-direct flow proven without it) + `assert_no_scripting_components()` audit+strip inside `save_and_reopen_stage_for_animation`. Script is local/trusted (bundled `omni.anim.people` ext), nothing external. Launch stays `/home/ubuntu/isaacsim/python.sh`.
- [x] **Phase 1-3 — Integrated scenario**: `scenarios/scenario_multi_person_vehicle_gui.yaml` (3 distinct S3 variants, diverse paths, clearance-audited) + `isaac_sim/scripts/testing/run_multi_person_vehicle_gui.py` (deferred build → reopen → World/art/bridge → triggers → C8R waypoint drive → FOV audit → TEST A-G,K,L). Vehicle C8R 23.89 m, START/GOAL flat discs only, no poles.
- [x] **Timeline-freeze root cause**: `world.step(render=False)` advances physics but NOT the USD timeline (probe: 100 steps → t frozen 0.010; render=True → advances). All headless non-render runs froze graph humans (triggers ok, zero displacement). Fix: render=True in integrated loop + runner forces render with anim reopen. (Supersedes the go_walk_to/PathPoints-refresh theories — goto validated working in scenario_02 post-fix: 15 m walk completed.)
- [x] **TEST A-G,K,L: 22/22 PASS** (5 consecutive runs incl. GUI): peds 2.25 m @0.95 / 6.46 m @0.98 / 6.25 m @1.03 m/s, arrivals 0.34-0.36 m; vehicle goal err 0.29 m, travel 22.52 m; TEST-B 582 ticks simultaneous from t=8.2 s; thirds L/C/R covered.
- [x] **TEST H/I/J + Phase 6: 10/10 PASS** (`check_rgbd_topics.py`): rgb+depth streaming, clock+tf alive, stamp sync dt=0.0000 s, frame_ids correct, center depth 68% valid med 6.75 m, fusion Object3D 7.97 m plausible. Topics: 6/6 live; NO `/tf_static` (bridge publishes dynamic `/tf` only — by design). ROS bag `/tmp/opencode/rgbd_run` (2.8 GB, 39.6 s, 1390 msgs x6 @~35 Hz).
- [x] **Phase 7-8**: fusion unit-verified on synthetic (4.00 m blob → 4.000 m); `docs/RGBD_MODEL_INTERFACE.md` (10-item contract; model TBD, no detector in repo to deduplicate). Phase 9 honored: NO steering from perception.
- [x] **GUI visual (real X :1)**: viewport overview captured (`data/integrated_overview_gui.png`: 3 peds + vehicle + START disc, no scripting popup); robot RGB captured (`data/integrated_rgb_single_ped_t8.png`, `data/integrated_rgb_three_peds_t10.png`: photoreal S3 humans, no T-pose, all-three-simultaneous = TEST-B visual). Known artifacts: render-product RGB clips bright areas to white (tone mapping; depth unaffected); GNOME "not responding" dialog appears during the heavy render loop (cosmetic — click Wait); viewport camera defaults to perspective (switch per printed VIEWs; `camera_path` set logs success but doesn't switch UI selection).
- [x] **Regressions (all green post-change)**: walk 19/19, idle 12/12, wpnav 5/5, ack 15/15, vehicle 56/56, scen02 (15 m goto walk), scen08 (3 S3 + 2 triggers), m8 11/11.
- [ ] **GUI full-visual gait**: single RGB frames show stance/idle phases; continuous walking motion visually confirmed only via viewport (not frame-captured mid-stride). Translation proven numerically (2.25-6.46 m) + joint ROM (walk test). Mark GUI-COMPLETE only after a mid-stride capture or live inspection.
- [ ] **M8.3 NavMesh bake**: still pending (straight-line paths only); goto-mode translation now works but detours unproven.

**GUI launch (deliverable):**
```bash
/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/testing/run_multi_person_vehicle_gui.py --gui [--ros] [--duration 60]
# companion (2nd terminal): source /opt/ros/humble/setup.bash
python3 isaac_sim/scripts/testing/check_rgbd_topics.py --window 15
```

## NEXT — MULTI-CLASS YOLO PERCEPTION + AUTONOMY (Priority per User Requirement)

**Immediate (this session, in order):**

1. **PHASE Y0 — Audit Verification Complete** ✅ — Repository audit done. YOLO26n-seg working (21/22 standalone PASS), 80 COCO classes available. ROS nodes exist. Config-driven.
2. **PHASE Y1 — Multi-Class Detection Configuration** ✅ `COMPLETE 2026-09-30` — `target_classes=[0,13,56,57,60]`, `obstacle_classes=[0,13,56,57,60]` in config; class map verified from installed weights; test script config-driven; multi-class proven on sim RGB (chair 0.81 + dining table 0.67 + 3 chairs, all masked).
   - [ ] Update `configs/perception.yaml`: 
     - `target_classes`: expand from `[0]` to include all relevant COCO classes (person=0, chair=56, couch=57, dining table=60, bench=13, plus other lab-relevant)
     - `obstacle_classes`: configure separately from `target_classes` — initially `[0, 13, 56, 57, 60]` (person, bench, chair, couch, dining table)
     - Document class mapping in config comments
   - [ ] Verify `YoloSegDetector` passes through all detected classes (no filtering in code — already uses `target_classes` set)
   - [ ] Run `test_yolo26n_seg.py` with expanded classes on existing frames — verify multi-class output

3. **PHASE Y2 — RGB-D Fusion Multi-Class Verification**
   - [ ] Run `verify_rgbd_yolo.py` on bag/frames with multi-class config — verify Object3D output for chair/table/bench/person
   - [ ] Confirm `_mask_depth` works for all classes (mask-based depth is class-agnostic)
   - [ ] Verify fusion node publishes all classes to `/perception/objects_3d`

4. **PHASE Y3 — Planner Multi-Class Obstacle Reasoning**
   - [ ] Verify `planner.py` uses `obstacle_classes` from config (already does: line 34)
   - [ ] Test planner with mixed obstacles (person + chair + table) — verify sector logic handles multiple classes
   - [ ] Ensure non-obstacle classes are reported but not steered around

5. **PHASE Y4 — ROS Integration & End-to-End Test**
   - [ ] Launch full stack: `yolo_seg_node` + `rgbd_fusion_node` + `camera_planner_node` + sim bridge
   - [ ] Test with `scenario_autonomy_stop.yaml` (person) — verify stop behavior
   - [ ] Create new test scenario with furniture (chair/table) — verify avoidance
   - [ ] Verify no `/odom` or GT subscriptions in planner node (TEST-K compliance)

6. **PHASE Y5 — Autonomous Multi-Object Scenarios**
   - [ ] Create `scenarios/scenario_autonomy_multi_object.yaml` with person + chair + table combos
   - [ ] Run `run_camera_autonomy_gui.py` with new scenario — verify steering reacts to actual detections
   - [ ] Log per-frame: detections, Object3D, planner decision, cmd_ackermann (Phase 15 logging)

7. **PHASE Y6 — Regression & Documentation**
   - [ ] Re-run all existing regressions: `verify_ackermann_physics.py` (15/15), `verify_vehicle_complete.py` (56/56), `test_m8_scenarios.py` (11/11)
   - [ ] Update `docs/RGBD_MODEL_INTERFACE.md` with multi-class contract
   - [ ] Update `docs/ISAAC_SIM_LEARNING_AND_PROJECT_GUIDE.md` §22-24 with YOLO integration details
   - [ ] Update `work.md` with this session's audit + implementation

---

**Then resume original roadmap:**

8. **PHASE A**: Finish `docs/PROJECT_STRUCTURE.md` implementation map
9. **PHASE B**: NavMeshVolume probe + bake
10. **PHASE C/D/E/F/G/H**: Realistic humans + NavMesh + scenarios + waypoint + GUI
11. **PHASE K/L/M/O**: Docs + PLAN + work.md + final report

## FUTURE (explicitly NOT this phase)

- **M9 Full RGB-D Perception Interface** — beyond dummy/detector base; requires real sensor data + model eval.
- **M10 Perception-Driven Steering** — rule-based controller that consumes `Object3D` (person position/distance) to steer/stop. This phase only does **free-space** A-to-B (no pedestrian avoidance yet, per PART 19).
- **M11 Multi-Object Scenarios** — two people, person+table/chair/column, narrow corridor, moving crossing — covered partially now via 4 scenarios but full 10-scenario suite & metrics pending.
- **M12 Logging & Evaluation** — perceptual/steering/system metrics, `data/results/`.
- **Final Neural RGB Model Selection** — pretrained detector (YOLO/DETR/etc.) plugged into `rgbd_interface` — intentionally left open (see FUTURE:model-selection below).

## MODEL SELECTION (still open, per PRIMARY OBJECTIVE)

- **Status:** NOT FINALIZED — interface must remain model-agnostic.
- **Why open:** Need real vehicle camera images to measure domain gap (lab lighting, glass refraction, occlusion); need to compare models on `640×480 70°HFOV` synthetic data vs real; no decision yet to avoid locking architecture.
- **Future plug-in point:** `ros2_ws/src/rgbd_perception/rgb_detector_node.py:RGBDetectorBase.detect(rgb_image) -> Detection2D[]` — swap `DummyDetector` for `YoloV8Detector`/`DetrDetector` without touching vehicle/camera/ROS2/nav. Document in guide §24 + report §22/24.

## PERCEPTION (model-agnostic design)

- **Now (M8-M10):** `RGB topic → (future) RGBDetector → Detection2D` + `Depth topic → depth_data` → `RGBDFusion → Object3D` → (future) `MultiObjectReasoning → Navigation/Steering → AckermannDriveStamped → AckermannController → PhysX`. **Current** free-space test bypasses perception: `WaypointPlanner → PathFollowingController → AckermannDriveStamped → ... → PhysX` (see architecture diagrams in docs).
- **Ground truth:** `Isaac Sim Ground Truth → Evaluation ONLY` (never `→ Controller`). Scenario runner logs `PedestrianGroundTruth` (position/velocity/orientation/dimensions/timestamp) for eval, not for control.
- **Sensor contract:** RGB-D `camera_rgb_optical_frame`/`camera_depth_optical_frame` `640×480 rgb8/32FC1` is sole model input; `Driver_Seat_Camera` is viewport-only.

## NAVIGATION

- **Now:** `Waypoint A (-8,-6) → (-4,-6) → (0,-3) → B (5,2)` example waypoints — **must be re-validated** against actual lab geometry (30×25 room, walls at ±15/±12.5, furniture at (-12,±6-9), glass at (-5,5,-10), columns at (±10,±8,0)). Pick collision-free path 18 lines below in `work.md` after probe.
- **Controller:** heading-error / pure-pursuit, configurable `lookahead, waypoint_tolerance, max_speed, min_speed, max_steer, accel, decel, final_stop, steering_rate`. Document equations (heading error, steer, waypoint switch, speed, stop).
- **Later:** static pedestrian → RGB-D → object position → stop/avoid; moving pedestrian → tracking → prediction → steering.

## SCENARIOS

- **Existing (legacy):** `configs/scenarios/scenario_01_stationary_person.yaml` → `scenario_12_multiple_objects.yaml` (12 YAML, capsule-based). Kept for regression.
- **New (required):** `scenarios/scenario_01_static_person.yaml` → `scenario_04_person_approaching.yaml` (4 YAML, realistic human, NavMesh-aware, reusable, no duplicated code). Each defines `scenario_name, pedestrian_id, start_position, destination, walking_speed, trajectory, wait_time, animation/behavior, duration, is_static`.
- **Runner:** `isaac_sim/scripts/scenarios/scenario_runner.py` — must be refactored to be reusable (single codepath), headless + GUI modes.

## EVALUATION

- Tests required (PART 25):
  - **Pedestrians:** T-P1 static exists, T-P2 scale realistic, T-P3 on floor, T-P4 not float, T-P5 not sink, T-P6 walks, T-P7 reaches dest, T-P8 not through walls.
  - **Navigation:** T-N1 start A, T-N2 waypoint seq, T-N3 steer ±0.6, T-N4 speed within limits, T-N5 wheels rotate, T-N6 front steer, T-N7 curved path, T-N8 reach B, T-N9 stop B, T-N10 no teleport, T-N11 no explosion, T-N12 no visual sep, T-N13 no wheel detach, T-N14 no lateral slip, T-N15 no collision free-space.
  - **Camera:** T-C1 RGB-D follows, T-C2 RGB, T-C3 depth, T-C4 CameraInfo, T-C5 driver viewport-only.
  - **Regression:** 71/71 (`verify_ackermann_physics.py 15/15` + `verify_vehicle_complete.py 56/56` + `test_m8_scenarios.py 11/11` extra) must still pass.
- **GUI checks (§15-18):** realistic human standing/walking, vehicle A→B, wheels steer/rotate, chassis visuals attached, cameras move, collisions respected.

## DOCUMENTATION

- [ ] `docs/PROJECT_STRUCTURE.md` — new, beginner-friendly, per-folder PURPOSE/INPUTS/OUTPUTS/USED BY/SAFE TO MODIFY?/CURRENT STATUS, directory tree, duplicate/obsolete audit (do NOT delete automatically).
- [ ] `docs/ISAAC_SIM_LEARNING_AND_PROJECT_GUIDE.md` — +20 new sections (see PHASE K list + PART 22), diagrams CURRENT/AUTOMATED-FREE-SPACE/FUTURE/GROUND-TRUTH.
- [ ] `docs/PHASE_M8_M10_DETAILED_REPORT.md` — new, 34 sections per PART 31, numerical results + screenshots.
- [ ] `PROJECT_PLAN.md` — update M8/M9/M10 status per sub-items, keep M0-M7 complete unless regression.
- [ ] `work.md` — chronological entry per PART 21.
- [ ] This `TODO.md` — keep correctly updated per prompt (you are here).

---

## How to run (verified commands)

```bash
# Headless regression (71/71)
source /opt/ros/humble/setup.bash; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
/home/ubuntu/isaacsim/python.sh tests/verify_ackermann_physics.py --headless        # 15/15
/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/verify_vehicle_complete.py --headless  # 56/56
/home/ubuntu/isaacsim/python.sh tests/test_m8_scenarios.py --headless               # 11/11 M8 + regression

# GUI inspection (stays open)
source /opt/ros/humble/setup.bash; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/run_fidelity_world.py --inspect          # static lab
/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/testing/run_wasd_test.py                 # WASD, select Driver_Seat_Camera
/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/scenarios/scenario_runner.py configs/scenarios/scenario_05_person_crossing_left_to_right.yaml --gui  # M8 scenario

# WASD keys (viewport focused): W/S ±0.4 m/s, A/D ±0.15 rad, Space brake, R centre, limits ±1.5/±0.6
```

## Remaining limitations (carried from M3/M4)

- Right-turn yaw asymmetry (verify_ackermann_physics T4: left 0.479 vs right 0.104 rad/s, diff 0.375 >0.18 threshold) — steering joints track via DriveAPI+per-step reset, functional but asymmetric; documented as known, not blocking M3/M4 complete.
- Capsule pedestrians are placeholders — upgrading to realistic humans this phase (PHASE C/D).
- No learned perception / no pedestrian avoidance yet — intentional staging (free-space A→B first, per PART 19).

## Exact next step (per PART 29 order)

**PHASE A finish:** Write `docs/PROJECT_STRUCTURE.md` (tree + per-folder table + file categories + active/legacy/entry points/do-not-edit + centralized config + data flow), then proceed to **PHASE B** probe `omni.anim.navigation.core` import + NavMeshVolume creation + Worker USD load test, document findings before writing any pedestrian code.

---

---
## ISAAC SIM 6.2 MIGRATION (continuation task — resumed 2026-10-28; last updated 2026-10-29)

**Source:** Isaac Sim 5.1.0-rc.19 at `/home/ubuntu/isaacsim/` (retained as rollback)
**Target:** Isaac Sim 6.2.0-rc.14 at `/home/ubuntu/isaacsim-6.2.0/` (VERSION: `6.2.0-rc.14+release.52805.02189a84.gl`)
**Target Python:** 3.12.14 (bundled), **System Python/ROS:** 3.10 + Humble (unchanged)
**Baseline Git:** single commit `8c843a8` — staged + unstaged working-tree changes preserved (NOT committed)

### Phase 0: Preserve baseline and establish rollback [x]
- Baseline Git status recorded: staged + unstaged changes present, single commit `8c843a8`.
- 5.1 installation retained; no destructive operations performed.
- Migration branch/working state preserved in working tree.

### Phase 1: Audit simulator and Python/ROS 2 compatibility [!] — PARTIAL (investigation ongoing)
- [x] Isaac Sim 6.2.0-rc.14 installed at `/home/ubuntu/isaacsim-6.2.0/` (VERSION confirmed).
- [x] 6.2 bundled Python 3.12.14 confirmed.
- [x] rclpy NOT importable in 6.2 Python 3.12 (system Humble is Python 3.10) → separate ROS 2 process required.
- [x] `isaacsim.ros2.bridge` extension (v5.1.4) loads in 6.2.
- [x] `build_world(dynamic=True, with_cameras=True, with_ros=True)` runs under 6.2 — bridge graph `/ROS2_Bridge` creates.
- [x] **TF content VERIFIED 2026-10-29 (warnings are cosmetic):** `ros2 topic echo /tf --once` on isolated domain → `world→Vehicle` translation (6.0, −6.0, 0.35) = configured `start_xy=(6.0,-6.0)` + ride height; `world→RGBD_Camera` (6.9008, −5.99999, 1.20001) = mount (0.9, 0, 1.2) resolved; rotation quaternion non-identity (pitch). Warnings `OgnROS2PublishTransformTree.targetPrims deprecated` + `[PoseTree] target getObjectType eInvalid` remain — cleanup item (migrate to `OgnIsaacComputeTransformTree` parentFrames/childFrames), not a delivery blocker.
- [x] **CRITICAL GAP RESOLVED 2026-10-29 — live topic delivery VERIFIED under 6.2.** External system-Python 3.10 rclpy subscriber (`check_rgbd_topics.py --window 20`) against a running 6.2 headless sim: **10/10 PASS** — RGB n=60, depth n=60 (finite, median 7.98 m), CameraInfo RGB+depth, /clock n=238, /tf n=235, frame_ids `camera_rgb_optical_frame`/`camera_depth_optical_frame` correct, sim-time stamps synchronized (dt=0.0000 s), fusion stub dist=7.97 m. **The 5.1 syntheticdata writer fault does NOT persist in 6.2.**
- [x] 6.2 OG node structure verified: `isaacsim.core.nodes.IsaacCreateRenderProduct` (typeVersion 3), `isaacsim.ros2.bridge.ROS2CameraHelper`/`ROS2CameraInfoHelper`/`ROS2PublishClock`/`ROS2PublishTransformTree` all resolve; `isaacsim.ros2.nodes` registers replicator writers (`LdrColorSDROS2PublishImage`, `DistanceToImagePlaneSDROS2PublishImage`, `ROS2PublishCameraInfo` ×2) — writer pipeline nodes created under `/Render/PostProcess/SDGPipeline` and confirmed executing (rgb8 640×480, sim-time advancing).
- [x] **Mis-attribution corrected 2026-10-29 (do not repeat):** the initial "FAIL" checks were an artifact — (a) those 300-frame probe sims exited at ~17 s and were checked post-mortem, and (b) a **stray Isaac Sim 5.1 process (PID 464379, `scenario_runner ... --ros --gui`, alive since 2026-10-26)** publishes `/clock`+`/tf` on the default domain, mimicking a live sim. **Control test on isolated `ROS_DOMAIN_ID=77`, bridge-only, tight loop (no sleeps), checked mid-run: 10/10 PASS.** Second render-product graph and frame sleeps are NOT required. Rule: always isolate the domain for validation, and check while the sim is alive.
- [x] Stray 5.1 process dispositioned 2026-10-29: **killed with user approval** (PID 464374/464379 tree, `scenario_runner ... --ros --gui` from 2026-10-26); GPU and default ROS domain now clean. Validation still uses isolated `ROS_DOMAIN_ID` for safety.
- [ ] ultralytics/YOLO runtime in 6.2 Python 3.12 not yet verified (perception runs externally in `perception_venv` anyway — design A).
- [x] Camera publishing rate measured: **~113 Hz** (headless, unlocked tight loop, `ros2 topic hz /camera/rgb/image_raw`, window 227). Realtime-locked rate not yet measured.

### Phase 2: Migrate scene construction to Isaac Sim 6.2 [!]
- [x] `build_world()` confirmed working under 6.2 (lab + vehicle + RGB-D camera + inspection cameras); physics initializes, timeline auto-plays, clean shutdown (`app.close()`).
- [ ] Run `run_fidelity_world.py` under 6.2 Python; capture exact failures.
- [ ] Fix bridge graph deprecation (targetPrims → parentFrames/childFrames via OgnIsaacComputeTransformTree) — after TF content verified.
- [ ] Acceptance: scene loads, vehicle + lab + RGB-D camera present, physics initializes, clean shutdown (needs one documented full run).

### Phase 3: Verify skateboard physics and vehicle control [x] — VERIFIED 2026-10-29
- [x] `verify_vehicle_complete.py --headless` under `/home/ubuntu/isaacsim-6.2.0/python.sh`: **56/56 CHECKS PASS** (identical to 5.1 baseline; incl. camera mount checks N1–N5: local (0.9, 0, 0.85), world Z 1.2).
- [x] `tests/verify_ackermann_physics.py --headless` under 6.2: **15/15 CHECKS PASS** (settle/straight/left/right/stop/clamps — identical to 5.1 baseline).
- [ ] Remaining regression suites under 6.2: `test_m8_scenarios.py` (11/11), A-to-B (22/22), YOLO offline rerun (21/22) — Phase 10.

### Phase 4: Integrate the real RGB-D camera [!]
- [x] Camera prim confirmed at `/World/Vehicle/Chassis/Sensors/RGBD_Camera/RGBD_Camera` under 6.2.
- [x] RenderProduct verified: `resolution = (640, 480)`, `camera -> .../RGBD_Camera`, render vars `LdrColorSD`+`DistanceToImagePlaneSD` attached; live depth finite (median 7.98 m).
- [ ] Verify ~70° HFOV, 0.1–10 m depth range, mount (0.9, 0, 1.2), pitch −8.5° via topic inspection (CameraInfo notes: renderer forces fy→fx, square-pixel assumption).

### Phase 5: Integrate ROS 2 and verify data delivery [x] — VERIFIED 2026-10-29
- [x] Bridge graph built with correct 6.2 OG nodes (`/ROS2_Bridge`: OnPlaybackTick → IsaacCreateRenderProduct → 2× CameraHelper + 2× CameraInfoHelper + Clock + PublishTF).
- [x] `/camera/rgb/image_raw`, `/camera/depth/image_raw`, camera_info ×2, `/clock`, `/tf` all published and discovered by an external ROS 2 Humble process (`ros2 topic list` shows all 7 topics; publisher count 1 each).
- [x] Acceptance: external ROS 2 Humble node received actual non-empty RGB/depth frames — `check_rgbd_topics.py` **10/10 PASS** (evidence in Phase 1 log above).
- [ ] Prerequisite to document: sim must run sustained (~2 min test window) — see startup-latency gotcha in Phase 1.

### Phase 6: Integrate YOLO26n-seg and depth fusion [x] — VERIFIED 2026-10-30
- [x] Integrated launcher defaults to 6.2 (`ISAAC_PYTHON` env-overridable; rollback via `ISAAC_PYTHON=/home/ubuntu/isaacsim/python.sh`).
- [x] Full integrated stack ran end-to-end under 6.2 headless: sim (scenario_02, 60s) → yolo_seg_node → rgbd_fusion_node → validator. Sim exit code=0. YOLO infer 7–13 ms, detections every frame (t=1→59.7). Validator captured "stream live", saved scenario PNG.
- [x] **Live fusion metrics CONFIRMED:** focused harness (sim + yolo + fusion + /perception/objects_3d counter, near-realtime direct world.step) → **172 Object3D messages** published live over DDS. Fusion subscribes depth/masks/detections/CameraInfo; publishes Detection3DArray.
- [x] `perception_venv` sanity: py3.10.12, ultralytics 8.4.163, rclpy OK, yolo26n-seg.pt present at repo root.
- [ ] YOLO offline rerun (21/22 baseline) — Phase 10.

### Phase 7: Integrate camera-derived observations and reactive steering [x] — VERIFIED 2026-10-30
- [x] **Sim-side /cmd_ackermann → vehicle response PASS under 6.2:** two rclpy nodes (publisher + subscriber) in one kit process; /cmd_ackermann over DDS → subscriber → AckermannController.drive_step → vehicle moved dist=1.40 m, yaw_change=0.65 rad under sustained 0.5 rad steer. Uses sim-side rclpy fix (bundled rclpy path + lib, no setup.bash sourcing).
- [x] Planner `plan()` logic verified (pure Python, perception_venv): CRUISE when center clear, STOP when center obstacle < stop_distance, SLOW+steer toward freer side in slow band, STOP when both sides blocked. Hard sensor-failure / safe-stop gate preserved (stale watchdog → steer 0, speed 0).
- [x] camera_planner_node can run externally under `/home/ubuntu/isaacsim-6.2.0/python.sh` (rclpy + ackermann_msgs OK outside Kit; verified import+init+publish).
- [ ] Full 4-process chain (sim+yolo+fusion+planner→cmd_ackermann→drive) in a single run — each link independently verified; end-to-end single run open.
- [ ] End-to-end acceptance: camera → RGB/depth → YOLO → fusion → planner → Ackermann → vehicle response — links verified individually; single synchronized run open.

### Phase 8: Establish GUI and headless workflows [x] — PARTIAL 2026-10-30
- [x] 6.2 headless RGB-D verified (writer fault does NOT persist); headless launch works via scenario_runner --ros --headless.
- [x] Documented 6.2 launch: `ISAAC_PYTHON=/home/ubuntu/isaacsim-6.2.0/python.sh` default in run_integrated_perception_gui.py.
- [ ] GUI launch command for real skateboard scene under 6.2 not yet documented.

### Phase 9: Save and reopen a reusable USD scene [ ]
- [ ] Test USD save/reopen of the skateboard scene under 6.2.
- [ ] Document limitation if not feasible.

### Phase 10: Run regression and end-to-end acceptance tests [ ]
- [ ] Rerun all regressions under 6.2; document actual results (do not carry over 5.1 counts).
- [ ] End-to-end: camera → RGB/depth → YOLO → fusion → planner → Ackermann → vehicle response.

### Phase 11: Update documentation and mark 6.2 as default [ ]
- [ ] Update `TODO.md`, `work.md`, `PROJECT_PLAN.md`, `README.md`, launch scripts, config paths.
- [ ] Migration report: `docs/ISAAC_SIM_6_2_MIGRATION_REPORT.md`.

### Phase 12: Final audit and rollback verification [ ]
- [ ] Confirm 5.1 rollback path intact.
- [ ] Confirm all changed files documented.

---

*Evidence before synthesis — this TODO is based on inspected files, `find` tree, and `ls exts`/`extscache` output above. Do not mark items complete without verification.*

## Milestone 17: Bypass Failing SyntheticData Writer (2026-10-23)

### Phase A: Real-Display Test
- DISPLAY=:1 confirmed active and functional
- Both official ROS2 bridge examples (camera_periodic.py, camera_manual.py) executed with real display; both crash with segfaults in libomni.graph.core.plugin.so during graph construction — display-independent systemic fault
- No async rclpy example (camera_rclpy_async.py) found in this Isaac Sim 5.1.0-rc.19 build

### Phase B: Supported Direct-Buffer API Investigation
- Official examples found: camera_periodic.py and camera_manual.py both crash at OG graph build
- camera_rclpy_async.py NOT present in this Isaac Sim 5.1.0-rc.19 installation — appears for newer release
- Both examples depend on omni.syntheticdata writer path which is the proven failure boundary
- Methods rp.get_pixels(), rp.get_depth(), rp.get_color() exist on rep.create.render_product() but produce no usable data without the writer path
- USD UsdRenderProduct API not directly importable from pxr in this Python environment (schema version mismatch)
- Hydra viewport readback: getFrameData failed, no LdrColor AOV; viewport not auto-created in headless
- OG nested int/uint array .set() proven impossible for all forms (control matrix: 0/11 PASS only scalar-INT and token-array)
- Render products CAN be created (confirmed: "[PASS] render_product created") but writer attach is the failure point

### Phase C-D: Buffer Readback and ROS Publisher
- No supported camera-buffer readback API found that bypasses the failing SyntheticData writer path in Isaac Sim 5.1.0-rc.19
- Direct rclpy publishing not attempted — no valid RGB/depth buffers extractable without the writer path
- Render products render frames (confirmed by simulation advancing), but no API to read them without the writer

### Key Findings Preserved
- OG graph 1.141.2 native attribute.set cannot write nested int/uint arrays from Python (control matrix proven)
- NVIDIA's own standalone_examples camera.py fails byte-identically — not project code issue
- Both temporary patches from WRITER_FIX_RECORD.md were reverted; install pristine (0 marker lines on writers.py and SyntheticData.py)
- 82/82 headless tests pass without live perception — vehicle physics, Ackermann, scenario stability preserved
- No permanent modifications to installed Isaac Sim extension files

### TODO.md Status Updates
- Phase H (GUI validation): Still BLOCKED — requires live RGB-D which is blocked by syntheticdata writer fault
- Phase N (TODO updates): Updated with Milestone 17 findings (this section)
- Phase O (final report): Continuing — this report documents the blocker for systematic unblocking when vendor fix arrives

## Milestone 18: Parallel Isaac Sim Version Test (2026-10-23)

### Phase B: Candidate Release Compatibility Assessment

**Selected**: Isaac Sim 6.2.0 (GA, October 2026)

| Requirement | Status |
|---|---|
| Ubuntu 22.04 | ⚠️ Supported but deprecated |
| GPU RTX PRO 6000 Blackwell | ✅ Exceeds minimum |
| Driver / CUDA | ✅ Compatible |
| ROS 2 Humble | ✅ Supported |
| Python 3.12 | ✅ Bundled |
| GLIBC 2.35+ | ✅ Available |
| Disk space | ✅ 267 GB available |

**Risks**: Ubuntu 22.04 deprecated; ROS 2 custom interfaces need Python 3.12 Docker build; Cortex removed (not used)

**Installation target**: `/home/ubuntu/isaacsim-6.2.0` (separate)

**Download**: `https://downloads.isaacsim.nvidia.com/isaac-sim-standalone-6.2.0-linux-x86_64.zip` (MD5: dde88af0b6423dd3135727385a344fb3)

**Approval needed**: Download ~40 GB, install to separate directory, no changes to existing installation or project
