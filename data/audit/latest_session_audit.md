# Latest Session Forensic Audit — SKATEBOARD_IRP (2026-10-06)

> PHASE 0 + 1 + 2 + 3 + 4 forensic audit. NO source files modified to produce this report.
> Only new files created: `data/audit/latest_session_audit.md`, `data/audit/scenario_12_matrix.md`.

## 0. Repository inventory (Phase 0)

- Root: `/home/ubuntu/SKATEBOARD_IRP`, branch `master`, single commit `8c843a8` tag `v0.1-september-2026`.
- `git status --short` (recorded 2026-10-06):
  - `M TODO.md`, `M configs/perception.yaml`, `M isaac_sim/scripts/testing/test_yolo26n_seg.py`, `M work.md`
  - `?? data/yolo_scenario_validation/` (REPORT.md, results.json, gt_01/02.json, 4 PNGs)
  - `?? isaac_sim/scripts/testing/run_yolo_all_scenarios_gui.py`
  - `?? isaac_sim/scripts/testing/validate_yolo_scenarios_ext.py`
- `git log --oneline --decorate -20`: exactly 1 commit. **No prior working version exists in git** — comparison against "previously working" must use `work.md` session notes + `data/` evidence PNGs, not git diff.
- `git diff` content (only functional changes):
  - `configs/perception.yaml`: `target_classes [0]` → `[0,13,56,57,60]`, same for `obstacle_classes`.
  - `test_yolo26n_seg.py`: hardcoded `target_classes=(0,)` → reads `target_classes` from `configs/perception.yaml`.
  - `TODO.md`/`work.md`: appended 2026-09-29 audit + 2026-09-30 validation notes. No logic change.
- Key source files (all present, unmodified since commit): `isaac_sim/scripts/scenarios/scenario_runner.py`,
  `isaac_sim/scripts/assets/real_human.py`, `isaac_sim/scripts/assets/realistic_pedestrian.py`,
  `isaac_sim/scripts/assets/pedestrian.py`, `isaac_sim/scripts/utils/world_setup.py`,
  `isaac_sim/scripts/sensors/rgbd_camera.py`, `isaac_sim/scripts/bridge/ros2_bridge.py`,
  `isaac_sim/scripts/vehicle/simple_vehicle.py`, `isaac_sim/scripts/vehicle/ackermann_controller_ros.py`,
  `ros2_ws/src/rgbd_perception/rgbd_perception/{yolo_seg.py,yolo_seg_node.py,rgbd_fusion_node.py,planner.py,config.py,camera_planner_node.py}`,
  `isaac_sim/scripts/perception/rgbd_interface.py`, all 12 `configs/scenarios/scenario_*.yaml`.
- `data/audit/` and `data/model_tests/` did NOT exist before this audit (created now for required reports).
- Weights: `ros2_ws/src/rgbd_perception/model/yolo26n-seg.pt` + duplicate `./yolo26n-seg.pt` (6.7 MB official Ultralytics).
- NOT inspected line-by-line (binary/large): `*.pt`, `references/lab/*.jpeg`, `__pycache__/`, `.git/objects/`.

## 1. What was run in the last session (Phase 1 — from `~/.bash_history`, recovered verbatim)

Order (tail of history, oldest → newest):

1. `run_multi_person_vehicle_gui.py --gui` (×2), `--gui --ros` — the S3-human integrated test.
2. `/home/ubuntu/perception_venv/bin/python ros2_ws/src/rgbd_perception/rgbd_perception/yolo_seg_node.py`
3. `/home/ubuntu/perception_venv/bin/python ros2_ws/src/rgbd_perception/rgbd_perception/rgbd_fusion_node.py`
4. `/home/ubuntu/perception_venv/bin/python .../validate_yolo_scenarios_ext.py --scenario 01 --out data/yolo_scenario_validation --show`
5. `/home/ubuntu/isaacsim/python.sh .../run_yolo_all_scenarios_gui.py --scenario 01 --gui --duration 12 --out data/yolo_scenario_validation`
6. Repeated `yolo_seg_node.py` + `rgbd_fusion_node.py` launches (sourced ROS Humble).
7. Then the 12 sequential GUI runs the task description references — **all via `scenario_runner.py`, all `--gui --duration 60.0`, NO `--ros`, NO YOLO, NO validator**:
   `scenario_01_stationary_person.yaml`, `scenario_02_person_center.yaml`, `scenario_03_person_left.yaml`,
   `scenario_04_person_right.yaml`, `scenario_05_person_crossing_left_to_right.yaml` (×2),
   `scenario_06_person_crossing_right_to_left.yaml`, `scenario_07_person_approaching.yaml`,
   `scenario_08_two_people.yaml`, `scenario_09_person_plus_chair.yaml`, `scenario_10_person_plus_table.yaml`,
   `scenario_11_narrow_corridor.yaml`, `scenario_12_multiple_objects.yaml`, then `scenario_05` again.
8. No `--headless --duration 30.0` run found in history — the task's example headless command was never executed
   (all recovered runs are `--gui --duration 60.0`). No `run_multi_person_vehicle_gui.py` interleaved with the 12.

Answers to the 10 required questions:

1. Scenarios executed: all 12 `configs/scenarios/scenario_*.yaml` (05 twice) via `scenario_runner.py`.
2. Runner used: `isaac_sim/scripts/scenarios/scenario_runner.py` (`run_scenario_file` → `initialize_sim` → `setup_scenario` → `finalize_setup` → `run_scenario`). NOT `run_yolo_all_scenarios_gui.py`, NOT `run_multi_person_vehicle_gui.py`.
3. Human creation function used: `_create_realistic_from_dict` from `realistic_pedestrian.py` (Tier 2 segmented) — see §2.
4. Config loaded: each scenario YAML listed above; vehicle/camera/sim from `configs/{vehicle,camera,simulation}.yaml` via `world_setup.build_world`. Perception YAML never read by this runner.
5. YOLO node started? NO — history shows yolo nodes were started only for the earlier scenario-01 validation pair, never alongside the 12 `scenario_runner` runs; `scenario_runner.py` contains zero YOLO imports.
6. ROS 2 started? NO — all 12 commands lack `--ros`, so `build_world(with_ros=False)` and `finalize_setup` skips `setup_ros2_bridge`. No `/camera/*` topics in those runs.
7. RGB-D bridge started? NO (same reason). Camera prim exists on vehicle (`simple_vehicle.py:333` → `rgbd_camera.py`) but no render-product → ROS writers without bridge.
8. Model visualization process started? NO — `validate_yolo_scenarios_ext.py` (the `cv2.imshow("YOLO RGB - scenario …")` RIGHT pane) was run once for scenario 01 earlier, never for the 12.
9. GUI crash vs never-started vs wrong camera: **never started**. The two-pane design is mandatory (rclpy is py3.10-only, cannot import in isaacsim py3.11 — documented in REPORT.md §8.2), so a single `scenario_runner` process can never show the detection window by itself.
10. Old vs new human implementation: **old Tier-2 segmented path** (`realistic_pedestrian.py`), not the S3 path (`real_human.py`). See §2.

## 2. Human regression audit (Phase 2)

### 2.1 Verified S3 architecture (from `real_human.py` + `run_multi_person_vehicle_gui.py` + `work.md`)

- S3 asset root `Assets/Isaac/5.1`, `CHARACTER_VARIANTS`, `Biped_Setup.usd` template, `SkelRoot` + ~101-joint skeleton + skinned meshes, `ApplyAnimationGraphAPICommand` wiring, `attach_behavior_script=False` default (no OmniScriptingAPI), `audit/strip` scripting components, export + `save_and_reopen_stage_for_animation` → `CharacterManager` refresh → `refresh_after_reopen` → `go_walk_*` graph triggers, `timeline.play()` + `world.step(render=True)` required.

### 2.2 Actual runtime call chain in the last session

```
scenario YAML (is_realistic=true, variant_idx=N, NO is_real_human flag)
  → scenario_runner.setup_scenario (lines 212-214):
      is_real_human = ped.get("is_real_human") or ped.get("use_real_human")  → False for ALL 12 YAMLs
      use_realistic = ped.get("is_realistic") or "variant_idx" in ped        → True for ALL 12 YAMLs
  → Tier 2 branch (line 224): _create_realistic_from_dict(stage, ped_config)
  → realistic_pedestrian.py: head sphere + capsule torso/arms/legs + box shoes + procedural walk
  → log line 228: "Created REALISTIC (segmented) human: person_XX …"
  → _needs_anim_reopen=False (line 243 checks class name == "RealHuman") → no reopen, no graph
```

Key evidence: **zero of the 12 YAMLs contain `is_real_human`/`use_real_human`** (verified by reading all 12 files).
The S3 Tier-1 branch (lines 216-223) is dead code for these configs. `HAS_REAL_HUMAN` import succeeds but is never taken.
`run_yolo_all_scenarios_gui.py` (lines 88-156) has the same limitation by design — segmented/capsule only, no S3 import at all.

### 2.3 Therefore both hypotheses in the task resolve as follows

- (A) "Log says REALISTIC but asset is wrong": **YES — terminology collision, not a code change.**
  `REALISTIC (segmented)` in the log means `realistic_pedestrian.py` primitives, while the user's "realistic S3 humans"
  means `real_human.py` SkelRoot assets. The screenshot log `Created REALISTIC (segmented) human: person_05` is the
  Tier-2 path working as coded — it was misread as S3 success.
- (B) "Correct asset replaced/stripped later": **NO evidence.** No fallback creation after S3 (S3 never created);
  no delete/recreate; no stale USD (`ANIM_REOPEN_PATH=/tmp/skateboard_anim_reopen.usd` only written when
  `_needs_anim_reopen=True`, which was False); no CharacterManager involvement at all.
- `pedestrian.py` capsule Tier 3 also never taken (every ped has `is_realistic`/`variant_idx`).
- No `OmniScriptingAPI`/`attach_behavior_script`/`anim_reopen` regression: those code paths were simply not entered.
- Previous S3 success came from a **different runner**: `run_multi_person_vehicle_gui.py` lines 120-127 call
  `create_real_human(stage, pcfg)` unconditionally (its YAML `scenarios/scenario_multi_person_vehicle_gui.yaml`
  is a different schema with direct S3 configs), plus `test_real_human_character.py` / `test_real_human_walking.py`.
  Those files are untouched and intact — nothing was lost, the 12-scenario suite just never pointed at them.

### 2.4 Smallest correct regression (no blind revert)

- The "regression" is a **configuration/runner mismatch**, not a broken S3 module. `real_human.py` is intact.
- Fix direction (Phase 6, not done here): decide per-scenario whether the 12 YAMLs should opt into
  `is_real_human: true` (+ required S3 fields) OR the segmented tier should be renamed so logs cannot be confused
  with S3, AND make any fallback loud (raise when S3 requested but unavailable instead of silent segmented downgrade).
- Do NOT revert the repo: single-commit history has no earlier version; the S3 code is already the good version.

## 3. YOLO model audit (Phase 3)

- Loader/adapter: `yolo_seg.py:YoloSegDetector` — RGB-only `np.uint8 HxWx3` in, `Detection2D{bbox,class_id,class_name,confidence,bool mask,timestamp}` out; masks resized via `interpolate` to frame; `task != segment` raises.
- Inference node: `yolo_seg_node.py` — subscribes `/camera/rgb/image_raw` (rgb8 only), publishes `/perception/detections_2d` (`"<id>:<label>"` + score + pixel bbox) and `/perception/masks` (mono8 idx+1). No depth/GT/odom subscription. Rate-limited (`perception_rate`), drops counted.
- Config: `configs/perception.yaml` (working tree) already multi-class — `target_classes: [0,13,56,57,60]`,
  `obstacle_classes: [0,13,56,57,60]` (= person, bench, chair, couch, dining table). Filtering is purely
  `if cid not in target_classes: continue` (yolo_seg.py:79) — **no person-only hard-code in the detector**.
  `test_yolo26n_seg.py` now reads classes from YAML (uncommitted fix, correct).
- Residual person-only default: `rgbd_perception/config.py:_DEFAULTS` still `[0]/[0]` — only matters if YAML missing; harmless when YAML present but should be aligned in Phase 6.
- Fusion: `rgbd_fusion_node.py` + `rgbd_interface._mask_depth` — mask-median/trimmed + bbox-center fallback, `min/max_depth`, `min_valid_fraction`, `sync_tolerance`, live `K` deprojection `X=(u-cx)*Z/fx, Y=(v-cy)*Z/fy`. Class-agnostic (passes `class_id` through). Formula matches Phase 10 requirement.
- Planner separation: `planner.py:34` uses `obstacle_classes` — detection vs control classes already separated as required.
  No retraining needed; class list is YAML-configurable.
- Provenance: `results.json` + REPORT.md show chair 0.81 + dining-table 0.67 + 3 chairs (masked) on `data/rgbd_rgb.png`,
  3 persons on three-peds frame, 21/22 standalone, live `detections_2d` (5 multi-class) → `objects_3d` (5/5 z=4.0 m synthetic). Model receives RGB only; depth used post-YOLO. Compliant.

## 4. Detection GUI audit (Phase 4)

- Previously successful screenshot came from `validate_yolo_scenarios_ext.py`: `annotate()` renders ACTUAL `detections_2d` + `masks` + `objects_3d` (boxes + mask overlays + `label conf dist`) onto the ACTUAL `/camera/rgb/image_raw` frame; `cv2.imshow("YOLO RGB - scenario %s")` is WINDOW 2 (RIGHT pane); composite PNG `scenario_<NN>.png` = raw LEFT + annotated RIGHT. Subscribes to live ROS topics, runs in `perception_venv` (rclpy), separatefed process by necessity.
- Full chain: `RGBD_Camera prim (simple_vehicle+rgbd_camera) → setup_ros2_bridge writers → /camera/rgb/image_raw + /camera/depth/* + CameraInfo → yolo_seg_node → detections_2d+masks → rgbd_fusion_node (+depth+K) → objects_3d → validator overlay → cv2 window`. Every link exists and was proven live once (scenario 01 pair).
- Why absent in latest run: `scenario_runner.py` has no YOLO imports, no overlay code, no `imshow`, no validator launch;
  with `--gui` and without `--ros` it opens only the Isaac Sim viewport (WINDOW 1). The 12 commands started no
  `yolo_seg_node`, no `rgbd_fusion_node`, no validator — so WINDOW 2 could not exist. Not a crash, not a wrong camera,
  not a silent failure: **omitted second process**. Additionally the Sept-30 REPORT documents live runs were blocked
  by GPU contention (bridge published `/clock`+`/tf` but zero camera topics, replicator `unknown dtype`) — same
  symptom would appear if anyone re-ran headless under load.
- Headless confusion: any `--headless` run cannot show either window; the recovered 12 runs were all `--gui` so
  WINDOW 1 was visible, WINDOW 2 was simply never launched.

## 5. Scenario + static-obstacle finding (feeds `scenario_12_matrix.md`)

- `scenario_runner.py` **never reads `static_obstacles`**: grep finds zero handlers (only `run_yolo_all_scenarios_gui.py:234`
  records them into `gt.json` for eval). Consequence: `chair_09`, `table_10`, `chair_12a/table_12a/column_12a` in
  scenarios 09/10/12 are **never spawned** by either runner. Chair/table detection in those scenarios can currently only
  come from built-in lab furniture (`lab_environment.py` + `lab_assets.py`: work tables, desks, office chairs), not
  from the YAML statics — and YAML statics at (-8,-10), (-10,8), (-6.5,-10.5) sit ~14–20 m behind/left of the
  vehicle start (6,-6, yaw 0), outside the 70° HFOV RGB-D frustum anyway (see matrix).
- All 12 YAMLs: `is_realistic: true`, `pedestrian_realism: realistic_human_segmented`, per-ped `variant_idx` +
  `animation` + `is_realistic: true`, but **no `is_real_human` flag** → segmented humans by design (see §2).
- Vehicle pose constant `(6,-6,0.35,0)` except scenario 11 `(-2,0,0.35,0)`; camera is the front-riser RGB-D
  (`camera.yaml`, pitch −8.5°) facing +X. Visibility column in the matrix uses REPORT.md §5 geometric audit (verified
  against YAML coordinates, not re-guessed).

## 6. What was working before vs what changed (summary for Phase 14)

- WORKING BEFORE: S3 humans via `run_multi_person_vehicle_gui.py` / `test_real_human_*.py` (3 walking S3 + evidence PNGs
  `integrated_rgb_*`); YOLO26n-seg 3-person overlay; RGB-D bridge topics; Ackermann physics (15/15, 56/56); M8 11/11.
- CHANGED since commit: only multi-class config expansion + config-driven test + two new validation runners + docs.
  No S3 file was edited; no vehicle/physics/bridge file was edited.
- LAST SESSION RAN: 12× `scenario_runner.py --gui --duration 60` (segmented humans, no ROS, no YOLO, no GUI viewer).
- REGRESSION CAUSE: wrong expectation, not lost code — (a) segmented Tier-2 humans mistaken for S3 Tier-1 due to
  `REALISTIC` log wording; (b) detection window absent because its three processes were never launched;
  (c) YAML statics never spawned by the runner AND placed outside the camera frustum.
- FILES CAUSING IT: `configs/scenarios/scenario_*.yaml` (missing `is_real_human` opt-in) + `scenario_runner.py`
  tier-selection/logging (ambiguous `REALISTIC` label, silent segmented fallback, no `static_obstacles` support) +
  operation (no integrated launcher → user ran sim alone). S3/YOLO/fusion/planner modules themselves are healthy.

## 8. ADDENDUM 2026-10-06 — streaming diagnosis corrected (supersedes §4 GPU-contention note)

Reproduced the replicator writer-attach failure (`Could not process writer attach request ...
Unable to write from unknown dtype, kind=i/f, size=0`) with GPU FREE (46 °C, 4 % util, no procs):
validator `no RGB within wait_timeout`, zero camera topics. The Sept-30 "unrelated training job" hypothesis
is therefore REFUTED as the cause. Camera prim path re-verified correct (real `UsdGeom.Camera` at
`.../Sensors/RGBD_Camera/RGBD_Camera`, `rgbd_camera.py:91`); bridge graph matches NVIDIA's own test wiring
and was intentionally left untouched. Fault is in the Isaac Sim 5.1 replicator/OG writer-attach path for
headless `--ros` runs — open investigation item, see `data/model_tests/perception_test_report.md` §N.
Non-ROS simulation (physics/pedestrians/new statics spawning) is green: scenarios 09 + 12 headless EXIT 0.

## 7. Next steps (Phase 6+, NOT executed here)

1. Disambiguate human tiers (rename log `REALISTIC (segmented)` → `SEGMENTED`, fail loudly when S3 requested but unavailable).
2. Decide S3 opt-in for the 12 YAMLs (add `is_real_human` + S3 fields) or keep segmented for speed — explicit per-scenario choice.
3. Implement `static_obstacles` spawning (or reposition YAML statics into the RGB-D frustum) so scenarios 09/10/12 can actually test chair/table.
4. Align `config.py` defaults with YAML multi-class lists.
5. Provide single integrated launcher (sim + ROS bridge + yolo + fusion + cv2 viewer) with correct ordering and cleanup; keep both windows open.
6. Then run perception-only validation of all 12 into `data/model_tests/scenario_NN/` without overwriting `data/yolo_scenario_validation/`.
