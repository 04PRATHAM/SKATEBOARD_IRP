# Perception Test Report — SKATEBOARD_IRP (2026-10-06, Phases 6–14)

Companion docs: `data/audit/latest_session_audit.md` (forensic audit, Phases 0–5),
`data/audit/scenario_12_matrix.md` (12-scenario matrix). No path planning implemented (out of scope by design).

## A. What was working before

S3 humans via `run_multi_person_vehicle_gui.py` (3 walking S3 + `data/integrated_rgb_*` evidence);
YOLO26n-seg 3-person overlay; ROS bridge topics; Ackermann physics 15/15 + 56/56; M8 11/11; standalone YOLO 21/22;
multi-class proof on sim RGB (chair 0.81 + dining table 0.67 + 3 chairs, all masked) — see `data/yolo_scenario_validation/`.

## B. What changed (this task only; full diff: `git diff --stat`)

1. `ros2_ws/.../config.py`: `_DEFAULTS` `[0]` → `[0,13,56,57,60]` (align fallback with YAML).
2. `isaac_sim/scripts/scenarios/scenario_runner.py` (+115): tier logs disambiguated
   (`SEGMENTED (primitive-assembly, NOT S3)`; S3 failures are `[ERROR]` loud fallbacks),
   `verify_spawned_tier()` stage check (SkelRoot validation, never trusts logs),
   `spawn_static_obstacles()` (chair/table/column at YAML-exact poses; unknown types loud-skipped).
3. NEW `isaac_sim/scripts/testing/run_integrated_perception_gui.py`: one-command launcher
   (sim `--ros` → yolo → fusion → cv2 viewer; stdlib-only, correct two-Python ordering, cleanup).
4. NEW `data/audit/latest_session_audit.md`, `data/audit/scenario_12_matrix.md`, this file.
5. Pre-existing uncommitted (kept, not authored here): multi-class `perception.yaml`, config-driven
   `test_yolo26n_seg.py`, `run_yolo_all_scenarios_gui.py`, `validate_yolo_scenarios_ext.py`, TODO/work notes.

## C. What was run in the last session (recovered from `~/.bash_history`)

12× `scenario_runner.py <scenario_XX.yaml> --gui --duration 60.0` — all WITHOUT `--ros`,
without YOLO/fusion/validator. Earlier: one scenario-01 sim+validator pair and standalone YOLO runs.

## D. Exact regression cause

No working code was lost. Three compounding expectation gaps: (a) Tier-2 segmented log
`Created REALISTIC (segmented)` misread as S3 success — all 12 YAMLs lack `is_real_human`, so the S3
branch is never taken; (b) detection window absent because its 3 processes were never launched
(`scenario_runner.py` contains zero YOLO/viewer code; two-process design is mandatory since
rclpy/py3.10 cannot import in isaacsim py3.11); (c) YAML `static_obstacles` silently dropped
(zero handlers repo-wide) AND placed outside the RGB-D frustum.

## E. Why the realistic humans "disappeared"

They didn't — the 12-scenario suite never requested them. S3 path (`real_human.py`, intact) triggers only on
`is_real_human/use_real_human` (cf. `scenarios/scenario_multi_person_vehicle_gui.yaml`, which has them).
Fix applied: unambiguous tier labels + loud S3 fallback + `verify_spawned_tier()` SkelRoot check.
12 YAMLs intentionally left on the segmented tier (fast, headless-friendly); S3 remains via the S3 GUI scenario.

## F. Why the detection GUI disappeared

Omitted process, not a crash: validator `cv2.imshow("YOLO RGB - scenario …")` (RIGHT pane) + yolo/fusion nodes
were never started alongside the 12 sim runs. New single launcher (Phase 7) starts all four in order.
ADDITIONAL live-streaming blocker found (see §N): ROS camera writers fail at replicator writer-attach
(`unknown dtype`) even with free GPU — corrected diagnosis below.

## G/H/I. Files causing / modified / unchanged

Causing: 12 scenario YAMLs (no S3 opt-in) + runner tier logging/fallback silence + missing statics support + manual multi-terminal ops.
Modified: `config.py`, `scenario_runner.py`; NEW launcher + 3 report files.
Unchanged: vehicle/physics (`simple_vehicle.py`, `ackermann_controller_ros.py`), bridge graph (`ros2_bridge.py`),
camera (`rgbd_camera.py`), lab (`lab_environment.py`, `lab_assets.py`), `real_human.py`, `realistic_pedestrian.py`,
`yolo_seg.py`, `yolo_seg_node.py`, `rgbd_fusion_node.py`, `planner.py`, `rgbd_interface.py`, all 12 scenario YAMLs,
`perception.yaml` values.

## J. YOLO model status

`yolo26n-seg.pt` (official, task `segment`), ultralytics 8.4.163, CUDA FP16. Standalone 21/22
(sole FAIL = never-recorded env image, pre-existing). RGB-only input; masks resized to frame; latency ~11–45 ms.

## K. Object classes tested (standalone, live sim RGB frames)

`target_classes=[0,13,56,57,60]`, `obstacle_classes` identical, YAML-configurable, detector has no hard-code.
`data/rgbd_rgb.png` → chair 0.81 + dining table 0.67 + chairs 0.59/0.57/0.50 (all masked) — MULTICLASS PASS.
`three_peds` → persons 0.87/0.84/0.68 masked. Planner verified: person/chair/table at 2–3 m → SLOW;
1 m → STOP; 6 m → CRUISE (chair/table treated as obstacles, not discarded).

## L. RGB-D fusion status

`mask_median` + bbox-center fallback + `min/max_depth` + `min_valid_fraction` + live-K deprojection
`X=(u-cx)*Z/fx, Y=(v-cy)*Z/fy` — unit-tested on synthetic 4.0 m plane (4.001 m, PASS). Class-agnostic.
Runtime consumes only RGB+depth+CameraInfo; GT/odom isolation grep-clean (only docstring mentions + clearly-labeled eval-only `GroundTruthDetector`, unused by nodes).

## M. All 12 scenario results (honest)

- Headless sim smokes (modified runner): scenario 09 (segmented ped + STATIC chair_09 spawn) EXIT 0;
  scenario 12 (3 segmented peds + chair/table/column statics) EXIT 0. Statics now spawn at YAML poses.
- Standalone YOLO on archived sim frames: multi-class PASS (see §K).
- Live 12-scenario GUI runs (sim+YOLO+viewer simultaneously): NOT completed — blocked by §N bridge fault.
  Per-scenario expectations/visibility: see `scenario_12_matrix.md`. No numbers faked; YAML chair/table/column
  statics are out-of-frustum (report honestly; do not retune thresholds to force hits). Previous artifacts in
  `data/yolo_scenario_validation/` untouched; new artifacts in `data/model_tests/`.

## N. GUI verification + corrected streaming diagnosis

- Launcher smoke (scenario 02): all 4 processes start; `yolo_seg_node` logs `classes=[0,13,56,57,60]` live. PASS.
- Live RGB over ROS: FAIL — validator `no RGB within wait_timeout`; sim log shows replicator
  `Could not process writer attach request ... Unable to write from unknown dtype (kind=i/f, size=0)`.
- CORRECTION to Sept-30 hypothesis: GPU is now FREE (46 °C, 4 %, no procs) yet the identical dtype error reproduces,
  so GPU contention was NOT the cause. Camera prim path verified correct (`.../RGBD_Camera/RGBD_Camera` IS a real
  `UsdGeom.Camera`, `rgbd_camera.py:91`); bridge graph matches NVIDIA's own test wiring and was left untouched.
  Fault lies in the Isaac Sim 5.1 replicator/OG writer-attach path for headless `--ros` runs — requires a
  bridge/render-product investigation (GUI-attached run or NVIDIA 5.1 writer API check), explicitly NOT attempted
  here to avoid rebuilding the verified bridge blind. Non-ROS sim (physics/peds/statics) is fully green.

## O. Remaining limitations

1. Live camera streaming over ROS bridge unresolved (§N) → live 12-scenario detection pending.
2. YAML statics spawn but sit outside the camera frustum (09/10/12) → chair/table YAML tests still expect 0 detections.
3. `column` has no COCO class (never YOLO-detectable) — keep as geometry-only.
4. Deprecated `half` flag warnings from ultralytics (cosmetic).
5. No path planning (next phase, only after §N resolved and 12-scenario perception proven).

## P. Exact command for the full integrated test (Phase 7)

```bash
cd /home/ubuntu/SKATEBOARD_IRP
python3 isaac_sim/scripts/testing/run_integrated_perception_gui.py --scenario 02 --gui --duration 60
# S3 humans: --scenario s3gui   | headless CI: --headless --duration 20 --no-show
# Requires §N bridge fix for live RGB; until then validate standalone:
# /home/ubuntu/perception_venv/bin/python isaac_sim/scripts/testing/test_yolo26n_seg.py --device cuda:0
```
