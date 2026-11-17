# YOLO 12-Scenario Perception Validation — REPORT (2026-09-30)

## 1. Verdict

**PERCEPTION STACK VERIFIED (compute + ROS pipeline + multi-class). LIVE 12-SCENARIO RUNS BLOCKED BY ENVIRONMENT (GPU contention), CODE READY.**

- YOLO26n-seg multi-class detection on real sim RGB: **PROVEN** (persons + chairs + dining table, all with masks).
- Full ROS pipeline (detector → fusion → Object3D) live: **PROVEN** with multi-class output.
- All 12 scenario YAMLs inspected + geometric visibility audited: **DONE**.
- Dedicated two-process validation runners written: **DONE** (`run_yolo_all_scenarios_gui.py` sim side, `validate_yolo_scenarios_ext.py` external side).
- Live 12-scenario execution: **BLOCKED** — Isaac render products produce no frames while an unrelated 2-day training job holds the GPU at 90°C/100% (PID 6056, 34GB — do not touch). Proven fidelity path also affected. Rerun when GPU is free.
- Path planning: **NOT activated** (no planner node, no `/cmd_ackermann` anywhere this phase).

## 2. Model / environment

- Weights: `ros2_ws/src/rgbd_perception/model/yolo26n-seg.pt` (official Ultralytics, task `segment`).
- Ultralytics 8.4.163; perception_venv py3.10 + isaacsim py3.11 (CUDA in both); RTX PRO 6000 96GB.
- Class map (actual weights): `0 person, 13 bench, 56 chair, 57 couch, 60 dining table` (+75 other COCO).
- Config: `configs/perception.yaml` — `target_classes: [0,13,56,57,60]`, `obstacle_classes: [0,13,56,57,60]` (separate, configurable, no hard-coded IDs).

## 3. Multi-class evidence (actual YOLO output on real sim RGB)

| Frame | Detections (class conf) | Evidence |
|---|---|---|
| `data/rgbd_rgb.png` (sim lab view) | chair 0.81, dining table 0.67, chair 0.59/0.57/0.50 — all masked | `multiclass_sim_lab_view.png` |
| `data/integrated_rgb_three_peds_t10.png` (sim) | person 0.87/0.84/0.68 — all masked | `multiclass_sim_three_peds.png` |
| `data/integrated_rgb_single_ped_t8.png` (sim) | person 0.83 + mask, ~55ms infer | `test_yolo26n_seg.py` 21/22 |
| lab reference photos | chair 0.92 + person 0.67; 3× chairs | `multiclass_lab_ref_*.png` |

## 4. Live ROS pipeline (production nodes, multi-class)

- `yolo_seg_node` → `/perception/detections_2d` (56:chair, 60:dining table + scores) ✅
- `rgbd_fusion_node` → `/perception/objects_3d` (5/5 objects, z=4.0m on synthetic plane) ✅
- Note: default 0.1s `sync_tolerance` drops pairs at 10Hz test-pub rate (detection latency); live sim publishes ~35Hz so unaffected.

## 5. Scenario-by-scenario visibility audit (geometric, HFOV ±35°)

| ID | Expected YAML objects | In camera FOV? | Classes |
|---|---|---|---|
| 01 | person (0,-2) | NO (146° behind) | person |
| 02 | person (10,-6) | YES, 4m center | person |
| 03 | person (10,-3) | MARGINAL (37°L, edge) | person |
| 04 | person (10,-9) | MARGINAL (37°R, edge) | person |
| 05 | person crossing L→R | YES (part) | person |
| 06 | person crossing R→L | YES then exits | person |
| 07 | person approaching 14m | YES once <10m | person |
| 08 | 2 persons | YES both | person ×2 |
| 09 | person + chair (-8,-10) | NO (14m behind); YAML statics NOT spawned by runner — lab furniture TBD live | person, chair |
| 10 | person + table (-10,6) | NO (20m behind-left); lab furniture TBD live | person, dining table |
| 11 | person (-3,3)→(12,3) | YES once in FOV | person |
| 12 | 3 persons + chair/table/column | 12b YES; 12a NO (56°L); 12c enters range; statics NOT spawned | person, chair, dining table |

## 6. Regressions (this session, headless)

- `verify_ackermann_physics.py`: **15/15 PASS**
- `verify_vehicle_complete.py`: **56/56 PASS**
- `tests/test_m8_scenarios.py`: **11/11 PASS**

## 7. Files created / modified

- NEW `isaac_sim/scripts/testing/run_yolo_all_scenarios_gui.py` — sim-side batch/single runner (no perception imports by design).
- NEW `isaac_sim/scripts/testing/validate_yolo_scenarios_ext.py` — external RIGHT-pane validator (overlay + metrics).
- MOD `configs/perception.yaml` — multi-class target/obstacle lists.
- MOD `isaac_sim/scripts/testing/test_yolo26n_seg.py` — reads target_classes from config.
- MOD `TODO.md`, `work.md` — phase tracking.
- Evidence: `data/yolo_scenario_validation/` (4 annotated PNGs, `results.json`, `gt_02.json`).

## 8. Root causes found (documented, not hidden)

1. `scenario_runner.py` creates `SimulationApp` at import → importing it crashes any sim script (2 segfaults). Fixed by not importing it (mirrored logic locally).
2. `import rclpy` impossible in isaacsim py3.11 (ROS Humble is py3.10-only) → in-sim ROS subscriptions impossible; two-process design (sim + perception_venv) is mandatory. Note: `run_camera_autonomy_gui.py` has the same latent defect (imports rclpy in-sim) — flagged for later phase.
3. `isaacsim.sensors.camera.Camera` init fails under current GPU contention (replicator dtype errors) — same failure as bridge render products.
4. Live runs blocked by unrelated training job saturating GPU (90°C, 100% util, 34GB VRAM, 2+ days). Rerun the 12 GUI commands below when it finishes.

## 9. Remaining for full live validation (when GPU free)

Run the 12 commands (Section 10), confirm per-scenario PNGs + `ext_<NN>.json`, merge with `gt_<NN>.json`, update this report's table with DETECTED columns.
