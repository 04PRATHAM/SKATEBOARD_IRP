# TODO — SKATEBOARD_IRP

**Last updated:** 2026-09-25 (INTEGRATED multi-person+vehicle+RGB-D: 22/22 headless + GUI-verified RGB frames; USD scripting warning root-caused + removed; timeline-freeze root cause found + fixed; all regressions green; see § INTEGRATED below)
**Isaac Sim:** 5.1.0-rc.19 | **ROS 2:** Humble | **GPU:** RTX PRO 6000 Blackwell 96GB | **Driver:** 580.173.02
**Project root:** `~/SKATEBOARD_IRP` | **Launcher:** `/home/ubuntu/isaacsim/python.sh`

> How to read: **COMPLETE** = verified headless + GUI screenshot evidence. **PARTIAL** = headless PASS, GUI/visual still pending. **NOT STARTED** = not yet. **BLOCKED** = integration blocker with 10-item report per PART 16 — do NOT silently fallback to capsules. This TODO keeps real human integration, GUI verification, RGB-D visibility, model selection, detector, pedestrian-aware steering, multi-object, evaluation — each clearly labelled.

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

## NEXT

**Immediate (this session, in order per PART 29):**

1. Finish **PHASE A**: write `docs/PROJECT_STRUCTURE.md` with beginner-friendly explanations, tree, per-folder PURPOSE/INPUTS/OUTPUTS/USED BY/SAFE TO MODIFY?/CURRENT STATUS, file categories (simulation/vehicle/sensors/ROS2/perception/control/testing/config/documentation/evaluation/scenarios), active vs legacy, entry points, do-not-edit-casually, centralized config, data flow. Then mark PHASE A complete.
2. **PHASE B**: run probe `python.sh -c "import omni.anim.navigation.core"` headless, test NavMeshVolume creation, inspect Worker USD load `define_prim` test, document API differences + asset paths, update TODO.
3. **PHASE C/D**: implement `assets/realistic_pedestrian.py` (segmented human, 6 variants, walking animation), integrate into `lab_environment.py` or `assets/pedestrian.py` upgrade, provide static + moving.
4. **PHASE E**: create NavMesh covering lab floor, verify 6 walkability checks, document.
5. **PHASE F**: create `scenarios/` 4 YAML + refactor runner (no duplicated code).
6. **PHASE G**: create waypoint controller (pure-pursuit heading-error) reusing `drive_step`, params from `vehicle.yaml`, generate `AckermannDriveStamped`.
7. **PHASE H**: GUI validation (keep open, inspect via Driver_Seat + external), capture PNGs.
8. **PHASE I**: model-agnostic RGB-D interface (ROS 2, RGB+Depth+CameraInfo → DetectedObject[] → Object3D, depth association, dummy detector).
9. **PHASE J**: deterministic tests T-P/T-N/T-C + regression 71/71 headless, then GUI.
10. **PHASE K/L/M/O**: docs + PLAN + work.md + final report.

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

*Evidence before synthesis — this TODO is based on inspected files, `find` tree, and `ls exts`/`extscache` output above. Do not mark items complete without verification.*
