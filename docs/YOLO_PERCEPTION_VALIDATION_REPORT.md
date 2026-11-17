# YOLO Perception Validation Report — SKATEBOARD_IRP (2026-10-08)

Perception-only milestone. Path planning NOT started (no planner launched, no `/cmd_ackermann`
wiring in any perception path — verified by grep; only docstrings mention its absence).
Companion audits: `data/latest_session_audit.md`, `data/audit/latest_session_audit.md`,
`data/audit/scenario_12_matrix.md`, `data/model_tests/perception_test_report.md`.

## 1. Initial state

Verified lab + Ackermann physics + RGB-D camera prim + OG ROS bridge graph + YOLO26n-seg
multi-class (standalone) + S3 human module (intact) + segmented 12-scenario suite. Single-commit
history (`8c843a8`); prior uncommitted work: multi-class config, two validation runners, launcher.

## 2. Previous-session audit

`git status`: 6 modified (`TODO.md`, `perception.yaml`, `scenario_runner.py`, `test_yolo26n_seg.py`,
`config.py`, `work.md`) + 5 untracked. Last runs: 12× `scenario_runner --gui --duration 60`
WITHOUT `--ros`/YOLO/viewer; one scenario-01 sim+validator pair; standalone YOLO. Full tree, docs
(`README`, `PROJECT_PLAN`, `work`, all 11 existing `docs/` — `camera_model.md`, `ros2_interfaces.md`,
`evaluation.md` do NOT exist), and keyword sweeps recorded in `data/latest_session_audit.md`.

## 3. Files inspected

`scenario_runner.py`, `real_human.py`, `realistic_pedestrian.py`, `pedestrian.py`, `world_setup.py`,
`rgbd_camera.py`, `visual_vehicle.py`, `simple_vehicle.py`, `ros2_bridge.py`, `lab_environment.py`,
`lab_assets.py`, `yolo_seg.py`, `yolo_seg_node.py`, `rgbd_fusion_node.py`, `planner.py`, `config.py`,
`rgbd_interface.py`, `validate_yolo_scenarios_ext.py`, `run_yolo_all_scenarios_gui.py`,
`run_integrated_perception_gui.py`, `check_rgbd_topics.py`, all 12 `configs/scenarios/*.yaml`,
`camera.yaml`, `simulation.yaml`, `perception.yaml`, NVIDIA `test_camera.py` + `camera_annotator_device.py`.

## 4. Files modified (this task)

- `isaac_sim/scripts/testing/diagnose_rgb_writer.py` (NEW): L1 Camera-API / L2 bridge-graph /
  L3 rep+BasicWriter / L3b viewport-bound / L4 viewport-capture isolation probes.
- `isaac_sim/scripts/testing/run_integrated_perception_gui.py`: hardened with `ROS-TOPICS`
  hard gate (polls `ros2 topic list` for RGB+depth; aborts with named stage instead of faking success).
- `data/latest_session_audit.md` (NEW, required path), this report (NEW),
  `data/perception_12_scenario_summary.md` (NEW).
- Earlier (kept): tier-log disambiguation + loud S3 fallback + `verify_spawned_tier()` +
  `spawn_static_obstacles()` in `scenario_runner.py`; multi-class `config.py` defaults.
- No YAML, vehicle, bridge, camera, YOLO, fusion, or planner logic touched.

## 5. Why S3 humans reverted

They never did — no code was lost. All 12 YAMLs request the segmented tier (`is_realistic` only, no
`is_real_human` flag), so `scenario_runner.py` correctly takes Tier 2 and the old `REALISTIC (segmented)`
log was misread as S3 success. S3 module intact; S3 assets HTTP-200 reachable (`F_Business_02.usd`,
`Biped_Setup.usd`); S3-only scenario file intact. 12 YAMLs deliberately left on the fast tier
(documented architecture: fast headless suite vs S3 GUI demo); no OmniScriptingAPI reintroduced.

## 6. Exact fix (human side)

Unambiguous tier labels (`SEGMENTED (primitive-assembly, NOT S3)`), S3 failures escalated to loud
`[ERROR]` fallbacks, `verify_spawned_tier()` SkelRoot stage check, statics spawner. Verified by 12/12
headless sims (§14) with 15 Tier-2 spawn logs and zero errors.

## 7. RGB writer failure root cause (definitive)

`TypeError: Unable to write from unknown dtype` from
`omni/syntheticdata/.../SyntheticData.py:1070 _add_node_downstream_intergraph_dependency`
(`dep_attrib_data.set(dep_data)`, empty arrays; writer node `inputs:render_products:resolution`
unwritable). Eliminated one by one with evidence: GPU contention (fails at 4% load), stale code
(bridge matches NVIDIA's own `test_camera.py`), wrong prim (real `UsdGeom.Camera` verified),
cold attach (fails warm too), headless-only (fails under Xvfb GUI), viewport absence (viewport
exists; binding camera + dedicated window change nothing), data starvation (480 s / ~14k rendered
steps yield zero Hydra frames; viewport AOV list empty; buffer capture times out). **NVIDIA's own
`standalone_examples/.../camera.py` fails byte-identically here**, and the NVIDIA forum confirms the
same 5.x regression (`syntheticdata 0.6.13`, Sept 2025, unresolved). Verdict: environment-level
Isaac Sim 5.1.0-rc.19 syntheticdata fault; every render-product consumer (Camera API, BasicWriter,
OG ROS helpers) is affected. No fix exists within the constraints (no upgrades/rebuilds).

## 8. Exact RGB fix

None applied — every candidate (warmup, timeline, viewport binding, dedicated window, rep product
`force_new`, `annotator_device` cpu/cuda) was tested and failed; NVIDIA's reference code fails too.
The bridge graph was deliberately NOT rewritten. Launcher now fails loudly at `ROS-TOPICS` instead of
pretending. Options for a future session: NVIDIA extension fix, or GUI run on the real `:1` display
(not attempted — intrusive without permission).

## 9. YOLO model details

`yolo26n-seg.pt` (official Ultralytics, `ros2_ws/src/rgbd_perception/model/`), task `segment`,
ultralytics 8.4.163, CUDA FP16, RGB-`uint8` in only, masks resized to frame, CPU fallback, warmup
excluded from latency. Untouched, untrained, unreplaced.

## 10. Configured classes

`target_classes = obstacle_classes = [0, 13, 56, 57, 60]`, YAML-driven (`load_config` verified),
passed into `YoloSegDetector` by node and test script; no hard-coded class anywhere.

## 11. Actual detected classes (standalone, real sim RGB — NOT faked, NOT live)

`data/rgbd_rgb.png` → chair 0.81 + dining table 0.67 + chair 0.59/0.58/0.50, all masked;
`integrated_rgb_three_peds_t10.png` → person 0.87/0.84/0.68, masked. Chair/table/person proven
together with masks; bench/couch configured but absent from test frames (honestly reported).

## 12. RGB-D fusion status

`mask_median` + bbox-center fallback + validity gates + live-K deprojection; unit PASS (4.001 m on
synthetic plane); planner multi-class PASS (chair/table slow+steer like persons). Live fusion
BLOCKED on §7 (no topics). Runtime inputs remain RGB+depth+CameraInfo only.

## 13. GUI architecture

WINDOW 1 = Isaac Sim viewport (any `--gui` runner; X live on `:1`, Xvfb available).
WINDOW 2 = `validate_yolo_scenarios_ext.py` cv2 viewer (actual detections on actual RGB; raw+annotated
composite + `ext_<NN>.json`). Two-process split mandatory (rclpy/py3.10 vs isaacsim/py3.11).
Dual-GUI live demo BLOCKED on §7; launcher verifies stages and aborts loudly.

## 14. Scenario results 1–12 (measured 2026-10-08, `--headless --duration 8`)

All 12: sim PASS (setup complete, EXIT 0, 0 errors/tracebacks). Tier logs: 15× SEGMENTED (08×2, 12×3),
0× S3 (correct per YAML). Statics: chair_09, table_10, chair/table/column_12a all spawn-verified.
Live RGB/Depth/YOLO/Fusion/Viewer per scenario: BLOCKED (§7) — see summary table. No PASS fabricated.

## 15. Detection examples

person 0.87/0.84/0.68 +masks (three-peds frame); chair 0.81 + dining table 0.67 + chairs
0.59/0.58/0.50 +masks (lab frame). Boxes/masks spatially aligned in saved composites
(`data/yolo_scenario_validation/`).

## 16–18. Inference latency / RGB FPS / Depth FPS (Phase 15)

YOLO (RTX PRO 6000, 640×640 imgsz, FP32): median 21.7 ms, p90 35.7 ms (~46 FPS end-to-end);
FP16: 11–45 ms total, ~6–8 ms infer-only. No contention failure. RGB/depth topic FPS: UNMEASURABLE
(blocked, §7) — recorded as such, not hidden.

## 19. Known limitations

§7 writer fault (primary); YAML statics spawn outside RGB-D frustum (09/10/12 expect 0 YAML
detections); `column` has no COCO class; ultralytics `half` deprecation warnings (cosmetic);
`config.py` vs YAML duplication (YAML wins); no path planning (next milestone, gated on §7).

## 20. GT/odom confirmation

Runtime perception consumes only RGB+depth+CameraInfo. `check_rgbd_topics.py`, both nodes, planner,
and launcher verified GT/odom-free by grep (only docstrings + clearly-labeled eval-only stub).

## 21. Path-planning confirmation

NOT started: no planner node launched, no `/cmd_ackermann` published by any perception component,
vehicle stationary in all perception runs (scenario default drive is scenario progression, unchanged).
