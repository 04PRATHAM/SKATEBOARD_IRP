# S3 Human + Class-Label Fix Report — SKATEBOARD_IRP (2026-10-08)

Scope: restore verified S3 appearance path + expose COCO 60 as `table`. No path planning.
No YAML changes. No vehicle/camera/bridge/YOLO-weight changes. Live RGB writer remains the known
external blocker (`docs/YOLO_PERCEPTION_VALIDATION_REPORT.md` §7); nothing below claims live data.

## 1. ROOT CAUSE OF BROKEN S3 HUMAN APPEARANCE

No S3 code regression exists. `real_human.py` is byte-identical to the verified version; S3 assets
are reachable (HTTP 200); the spawn chain (CharacterUtil reference → SkelRoot discovery → Biped graph
wiring, `attach_behavior_script=False`) is intact. Two findings explain the observation:

(a) **Tier confusion (primary).** All 12 `configs/scenarios/*.yaml` request the segmented tier
(`is_realistic` only). The S3 tier triggers solely on `is_real_human`/`use_real_human`
(`scenarios/scenario_multi_person_vehicle_gui.yaml`). A 12-scenario GUI therefore shows the
*intentional* primitive assembly (capsule torso/limbs, sphere head/hands, box shoes). Its walk
animator additionally rotates limbs about segment centers rather than shoulder/hip joints
(`realistic_pedestrian.py::_animate_limbs`), so mid-stride shoulders/hips visibly gap — matching
the "detached/floating" description. The segmented tier is preserved as designed (not rebuilt);
its pivot imperfection is documented here as a known cosmetic limitation for a follow-up.

(b) **S3 pipeline verified healthy.** Fresh headless evidence this session (below) proves SkelRoot +
skinned meshes + 101-joint skeletons + graph walk + reopen preservation. If a viewport still shows
broken S3 skins, that is a render-side residual not reproducible headless — needs a GUI screenshot
against the reference image to pursue further.

## 2. EXACT FILES CHANGED

- `configs/perception.yaml`: added `application_class_names` (60 → `table`).
- `ros2_ws/.../config.py`: same map in `_DEFAULTS` + canonical `application_label()` (single layer).
- `ros2_ws/.../yolo_seg.py`: `YoloSegDetector(..., app_names=...)`; `class_name` from map, IDs untouched.
- `ros2_ws/.../yolo_seg_node.py`, `isaac_sim/.../verify_rgbd_yolo.py`,
  `isaac_sim/.../test_yolo26n_seg.py`: pass the map through (no logic change).
- `isaac_sim/.../run_camera_autonomy_gui.py`, `isaac_sim/.../run_testA_perception.py` (legacy):
  name derivation routed through `application_label()` (single layer, no scattered replaces).
- `isaac_sim/scripts/scenarios/scenario_runner.py`: log-only — `[S3]` / `[SEGMENTED]` / `[CAPSULE]`
  prefixes (prior disambiguation/verification/statics work kept).
- NEW diagnostics (not architecture): `probe_s3_hierarchy.py` (hierarchy pre/post reopen),
  both kept out of runtime paths.

## 3. EXACT FIX APPLIED

S3: no source fix required or made — verified healthy; tier selection made unambiguous in logs.
Labels: one centralized map (`config.application_label`), applied at the single point where
`Detection2D.class_name` is assigned; every downstream consumer (node `"id:label"` string, fusion
pass-through, viewer parsing, PNG overlay, JSON) inherits it with zero further edits.

## 4. HOW THE S3 HUMAN ASSET IS NOW LOADED

Unchanged verified path: `CharacterUtil.load_character_usd_to_stage(S3_URL + variant_usd_rel)` →
`/World/Characters/{id}` → warmup compose → deepest-under-`ManRoot` SkelRoot selected →
`setup_animation_graph_to_character(all SkelRoots, Biped AnimationGraph)` → export + reopen →
`World`/`Articulation` recreated → `refresh_after_reopen` → `timeline.play()` → `go_idle`/`go_walk_*`.

## 5. HOW THE SKINNED MESH IS CONNECTED TO THE SKELETON

S3 USD payload reference (not rebuilt geometry): probe shows per variant one SkelRoot containing a
101-joint `Skeleton` + 5–9 `Mesh` prims (20 meshes / 3 variants + Biped template), preserved
identically across save/reopen (20 → 20). Skinning rides the asset's own SkelBinding; animation
targets the same SkelRoot the meshes are bound under (graph Character acquired per-SkelRoot).

## 6. HOW ANIMATION IS INITIALIZED

Graph-direct only: `enable_real_human_extensions()` once → spawn → reopen (attach-time CharacterManager
discovery) → reset → `refresh_after_reopen` → `timeline.play()` → `go_idle` (`Action=None`) or
`go_walk_to`/`go_walk_straight` (`Action=Walk` + `PathPoints`, Walk ramp pump in `update()`).
Proven: idle micro-motion 3.8 cm; walk 7.64 m @ 0.78 m/s, L/R thigh correlation −0.998, no T-pose.

## 7. CONFIRMATION THAT OmniScriptingAPI WAS NOT REINTRODUCED

`attach_behavior_script` default `False`; zero `=True` sites, zero `setup_python_scripts_to_character`
calls, zero `OmniScriptingAPI`/`character_behavior` references in runners, scenarios, or configs
(grep-verified). No security popup mechanism exists in the flow.

## 8. CONFIRMATION THAT THE THREE S3 HUMAN VARIANTS WORK

`test_real_human_character.py` 12/12 (construction SkelRoot, 9 meshes, 101 joints, idle alive);
`probe_s3_hierarchy.py` PASS (construction/business/medical SkelRoots + 20 meshes pre/post reopen,
`go_idle` True ×3); `scenario_runner` + S3 YAML: 3× `[S3]`, 3× `[VERIFY] VERIFIED`, 3× GRAPH walk
`ok=True`, clean exit; `test_real_human_walking.py` 19/19 (business variant gait).

## 9. CLASS MAPPING BEFORE/AFTER

Before: model name `dining table` leaked into `Detection2D.class_name`, overlays, JSON.
After: `60 → table` via `application_class_names`; `0 → person`, `13 → bench`, `56 → chair`,
`57 → couch` unchanged in meaning; IDs `0/13/56/57/60` byte-identical; weights untouched.

## 10. TEST RESULTS

- TEST 1 S3 asset (idle/attach/no-popup): PASS 12/12.
- TEST 2 S3 animation (walk/gait/mesh-attached): PASS 19/19.
- TEST 3 three variants simultaneous: PASS (probe + S3-yaml run).
- TEST 4 YAML tier 09 + 12 (+ full 12-scenario loop): PASS, segmented preserved, EXIT 0, statics spawn.
- TEST 5 labels (table 0.67 / chair 0.81 / person 0.83; IDs unchanged; no `dining table` leak): PASS.
- TEST 6 fusion label (`Object3D.label == "table"`, dist 3.14 m): PASS.
- TEST 7 regressions (compile all, planner multi-class, subscription audit, scripting audit,
  13+ vehicle/world loads via scenario runs): PASS.
- Live RGB/depth/YOLO/fusion/GUI: BLOCKED (external writer fault, unchanged, honestly reported).

## 11. ANY REMAINING ISSUES

1. Live RGB-D writer (external, §7 of YOLO report) — gates all live states.
2. Segmented mid-stride joint pivots (cosmetic; documented, tier preserved; fix = reparent limbs
   under joint Xforms — deferred as broader change).
3. GUI skin rendering of S3 not headless-verifiable — needs screenshot comparison if still reported.
4. `column` YAML statics are geometry-only (no COCO class); 09/10/12 YAML statics spawn outside the
   camera frustum (expect no YAML detections there).

## STATE TABLE (honest)

| Component | Status |
|---|---|
| S3 human mesh | PASS (12/12 + probe 20/20 meshes) |
| S3 animation | PASS (19/19, gait −0.998) |
| Three S3 variants | PASS |
| Segmented YAML tier | PASS (12/12 sims, preserved) |
| YOLO26n-seg standalone | PASS (21/22, pre-existing missing env frame) |
| YOLO segmentation | PASS (masks on all detections) |
| Multi-class detection | PASS (person/chair/table proven) |
| Class 60 application label | PASS — `table`, IDs unchanged |
| Standalone model GUI (annotated PNGs) | PASS (labels show `table`) |
| RGB-D fusion unit test | PASS (`table`, 3.14 m) |
| Planner unit test | PASS (chair/table slow like persons) |
| OmniScriptingAPI | PASS — not used |
| Vehicle / camera / bridge arch | UNCHANGED (13+ loads green) |
| Live RGB ROS topic | BLOCKED (external writer fault) |
| Live depth ROS topic | BLOCKED (same fault) |
| Live YOLO / fusion / Object3D | BLOCKED by RGB stream |
| Autonomous camera-only control | NOT STARTED |

## ADDENDUM 2026-10-09 PM — GUI "stationary pedestrian" follow-up

**Finding: no trajectory defect.** Goto-mode root motion measured 8.78 m / 8 sim-s headless AND
8.78 m with a GUI window (`probe_goto_displacement.py`, scenario_05 ped config, trigger ok=True,
state WALKING). Animation + trajectory advance in both modes. No motion-logic change made.

**Why the GUI looked static (assessed):** sim-time advances slowly under S3 GUI load (walks finish
while observer orients); scenario 09 + 12a peds are stationary BY DESIGN; post-arrival idle looks
static; wrong viewport camera hides motion. `SKATEBOARD_TRACE_PEDS=1` now prints root positions
every ~2 sim-s for independent verification. **Vehicle parked = intentional** (`vehicle_behavior`
default). **No YOLO in scenario GUI = architecture** (two-process + writer blocker); standalone
viewer unaffected. GUI visual sign-off still requires human eyes with the trace on.

## ADDENDUM 2026-10-09 PM2 — ACTUAL motion defect found and fixed (supersedes AM2 addendum)

AM2's "no defect" verdict was based on configs that enable S3 extensions; the 12-YAML path did NOT:
extension enabling + World deferral were gated on legacy `is_real_human` flags only, so with
`human_tier: s3` every `go_walk_to` failed (`No module named 'omni.anim.graph.core'`) and walking
peds stood idle (spawn + SkelRoot VERIFY were fine — only triggers failed). Fixed with
`scenario_needs_s3()` used at both gates. Verified: 12b ≈10.5 m, 12c ≈10.7 m displacement,
12a correctly stationary, triggers ok=True. No motion-logic, asset, or animation change needed.
