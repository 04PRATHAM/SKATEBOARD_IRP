# Latest Session Audit — SKATEBOARD_IRP perception validation (2026-10-08)

Audit-first document. No source modifications made to produce it (only this file + the new
diagnostic script `isaac_sim/scripts/testing/diagnose_rgb_writer.py` were created).
Prior forensic work: `data/audit/latest_session_audit.md` (Phases 0–5) + `scenario_12_matrix.md` +
`data/model_tests/perception_test_report.md` (Phases 6–14). This file is the newly requested audit path.

## 1. Existing working architecture

Lab (`lab_environment.py` + `lab_assets.py`: floor/walls/glass/tables/desks/chairs/columns) → physical
Ackermann vehicle (`simple_vehicle.py` + `visual_vehicle.py`, `drive_step` in `ackermann_controller_ros.py`)
→ vehicle-mounted RGB-D camera (`sensors/rgbd_camera.py`, prim
`/World/Vehicle/Chassis/Sensors/RGBD_Camera/RGBD_Camera`, 640×480, HFOV 70°, pitch −8.5°) → OG ROS bridge
(`bridge/ros2_bridge.py`, `/ROS2_Bridge`) → `/camera/rgb|depth/image_raw` + CameraInfos + `/clock` + `/tf`
→ `yolo_seg_node` (RGB-only) → `detections_2d` + `masks` → `rgbd_fusion_node` (+depth+K) → `objects_3d`
→ `validate_yolo_scenarios_ext.py` cv2 viewer. Single builder `utils/world_setup.py::build_world` for all runners.

## 2. Existing YOLO implementation (`ros2_ws/src/rgbd_perception/rgbd_perception/`)

- `yolo_seg.py:YoloSegDetector`: RGB `uint8 HxWx3` in; `Detection2D{bbox,class_id,class_name,confidence,bool mask,timestamp}` out;
  masks resized to frame via interpolate; `task != "segment"` raises; CUDA→CPU fallback; warmup excluded from latency.
- `yolo_seg_node.py`: subscribes rgb8 `/camera/rgb/image_raw` only; publishes `detections_2d` (`"<id>:<label>"`+score+pixel bbox)
  and mono8 `/perception/masks` (idx+1); rate-limited, drops counted. No depth/GT/odom subscription.
- Verified 2026-10-08 from installed weights: task `segment`, 80 classes,
  0=person 13=bench 56=chair 57=couch 60=dining table (all match expected).
- Standalone `test_yolo26n_seg.py`: 21/22 (sole FAIL = never-recorded env image). Multi-class proven on sim RGB:
  chair 0.81 + dining table 0.67 + chairs 0.59/0.57/0.50 (all masked); 3 persons 0.87/0.84/0.68.

## 3. Existing scenario architecture

12 YAMLs in `configs/scenarios/` (segmented tier: `is_realistic:true` + `variant_idx`, NO `is_real_human` flag) +
`scenarios/scenario_multi_person_vehicle_gui.yaml` (S3 tier: `is_real_human:true` + `use_real_human:true` ×3).
`scenario_runner.py`: Tier1 S3 → Tier2 SEGMENTED → Tier3 capsule (branching unchanged); now with unambiguous
tier logs, loud S3 fallback, `verify_spawned_tier()` SkelRoot check, `spawn_static_obstacles()` (chair/table/column
at YAML poses). `run_yolo_all_scenarios_gui.py`: sim-side batch (segmented/capsule only, no S3 import).
`run_integrated_perception_gui.py` (NEW, prior session): one launcher, sim `--ros` → yolo → fusion → viewer.

## 4. Existing S3 human implementation (`assets/real_human.py`, intact, unmodified)

S3 root `Assets/Isaac/5.1` (HTTP 200 verified 2026-10-08 for `F_Business_02.usd` AND `Biped_Setup.usd`);
6 `CHARACTER_VARIANTS`; Biped anim graph; `attach_behavior_script=False` default (no OmniScriptingAPI);
scripting-component audit/strip; export + `save_and_reopen_stage_for_animation` → CharacterManager refresh →
`refresh_after_reopen` → graph `go_walk_*`; requires `timeline.play()` + rendered steps.

## 5. Existing segmented/basic implementation (intentional fast tier)

`realistic_pedestrian.py` (head sphere + capsule torso/arms/legs + box shoes + procedural walk) and legacy
`pedestrian.py` capsules. Used by all 12 YAMLs deliberately (fast headless perception testing). Logs now read
`SEGMENTED (primitive-assembly, NOT S3)`. Not a regression — a documented tier.

## 6. Existing RGB-D implementation

RGB + depth 640×480 via one UsdGeom.Camera + OG `IsaacCreateRenderProduct` → `ROS2CameraHelper(type=rgb|depth)` +
CameraInfo helpers. Fusion: `rgbd_fusion_node.py` + `rgbd_interface._mask_depth` (mask-median/trimmed, bbox-center
fallback, min/max-depth + min-valid-fraction gates, live-K deprojection). Unit-tested (4.001 m on synthetic plane).

## 7. Existing GUI implementation

WINDOW 1: Isaac Sim viewport (any runner `--gui`). WINDOW 2: `validate_yolo_scenarios_ext.py`
`cv2.imshow("YOLO RGB - scenario …")` rendering ACTUAL detections+masks+objects_3d on ACTUAL RGB frames
(raw LEFT + annotated RIGHT composite PNG + `ext_<NN>.json`). Two-process split is mandatory
(rclpy py3.10-only vs isaacsim py3.11). X server confirmed live on `:1` (Xorg + `xdpyinfo` OK), so GUI runs are feasible.

## 8. Existing ROS bridge implementation (`bridge/ros2_bridge.py`, untouched)

ActionGraph `/ROS2_Bridge`: OnPlaybackTick → IsaacCreateRenderProduct → RGB/Depth/Infos/TF; ReadSimTime → Clock/TF.
Binds ONLY the RGBD camera (refuses inspection/DriverSeat cameras). Matches NVIDIA's own bridge test wiring.

## 9. Exact files modified in the previous session (`git diff --stat`)

- `ros2_ws/.../config.py`: defaults `[0]`→`[0,13,56,57,60]` (verified via `load_config`).
- `isaac_sim/scripts/scenarios/scenario_runner.py` (+115): tier log disambiguation, loud S3 fallback,
  `verify_spawned_tier()`, `spawn_static_obstacles()` (+ builder-race verification).
- NEW: `run_integrated_perception_gui.py`, `diagnose_rgb_writer.py` (this session),
  `data/audit/`, `data/model_tests/perception_test_report.md`.
- Pre-existing uncommitted (kept): multi-class `perception.yaml`, config-driven `test_yolo26n_seg.py`,
  `run_yolo_all_scenarios_gui.py`, `validate_yolo_scenarios_ext.py`, TODO/work notes.
- Single-commit history (`8c843a8`); nothing reset/checked-out/discarded.

## 10. Exact commands/scripts used or documented for the previous run

- `.../python.sh .../scenario_runner.py configs/scenarios/scenario_09...yaml --headless --duration 8` → EXIT 0
  (`SEGMENTED person_09`, `STATIC chair_09` verified). Same for scenario_12 (3 peds + 3 statics) → EXIT 0.
- `run_integrated_perception_gui.py --scenario 02 --headless ...` → all 4 processes start, yolo logs
  `classes=[0,13,56,57,60]`, but validator `no RGB within wait_timeout` (see §11).
- `/home/ubuntu/perception_venv/bin/python .../test_yolo26n_seg.py --device cuda:0` → 21/22.
- Planner/fusion unit checks → PASS. GT-isolation grep → clean (only docstrings + eval-only stub).

## 11. Current known failures

PRIMARY BLOCKER: live ROS RGB (and hence depth) never publish in headless `--ros` runs. Sim log:
`[isaacsim.core.nodes.impl.base_writer_node] Could not process writer attach request (...), Unable to write
from unknown dtype, kind=i/f, size=0`. Reproduces with GPU FREE (46 °C/4%), refuting contention.
Camera prim path re-verified (real `UsdGeom.Camera`); bridge graph untouched. Fault localized to the
Isaac 5.1 replicator/OG writer-attach path — under diagnosis via `diagnose_rgb_writer.py` (L1 Camera-API vs L2 bridge).

## 12. Current known successes

Segmented 12-scenario sims run headless; statics spawn; S3 assets reachable; YOLO multi-class standalone proven;
planner multi-class + fusion unit PASS; GT isolation holds; launcher ordering proven; X available for GUI.

## 13. Current blockers

(a) writer-attach dtype failure → no live topics → no live YOLO/GUI validation yet;
(b) YAML statics (09/10/12) spawn but sit outside RGB-D frustum → expect 0 YAML chair/table detections;
(c) `column` has no COCO class (geometry-only by nature).

## 14. Recommended minimal fixes (in order)

1. Run `diagnose_rgb_writer.py` headless L1 (camera-only) then L2 (bridge) to isolate camera vs writer.
2. Apply the smallest fix the diagnostic indicates (warmup/viewport/render-product handling); do NOT rewrite the bridge graph unless proven necessary; never drop depth.
3. Verify `ros2 topic hz /camera/rgb|depth/image_raw` + encodings, then YOLO live, then fusion.
4. Dual GUI on `:1` (sim `--gui` + cv2 viewer), scenario_12 with S3 first, then 01–12 with measured table.
5. Reports: `docs/YOLO_PERCEPTION_VALIDATION_REPORT.md` + `data/perception_12_scenario_summary.md`. No planning.
