# Perception 12-Scenario Summary — measured 2026-10-08 (headless sim + standalone YOLO)

Sim column = `scenario_runner.py --headless --duration 8` (all EXIT 0, 0 errors).
RGB/Depth/YOLO/Fusion/GUI columns = LIVE over ROS → BLOCKED by the syntheticdata writer fault
(see `docs/YOLO_PERCEPTION_VALIDATION_REPORT.md` §7). YOLO-standalone column = archived sim frames.
S3 column: all 12 YAMLs request the segmented tier (correct, intentional); S3 lives in the `s3gui` scenario.

| Scenario | Sim | RGB live | Depth live | YOLO live | Person | Chair | Table | Fusion live | GUI live | S3 | Result |
|----------|-----|----------|------------|-----------|--------|-------|-------|-------------|----------|----|--------|
| 01 stationary | PASS | BLOCKED | BLOCKED | BLOCKED | standalone-proven* | n/a | n/a | BLOCKED | BLOCKED | seg (by design) | SIM-PASS |
| 02 center | PASS | BLOCKED | BLOCKED | BLOCKED | standalone-proven* | n/a | n/a | BLOCKED | BLOCKED | seg (by design) | SIM-PASS |
| 03 left | PASS | BLOCKED | BLOCKED | BLOCKED | edge/FOV-marginal | n/a | n/a | BLOCKED | BLOCKED | seg (by design) | SIM-PASS |
| 04 right | PASS | BLOCKED | BLOCKED | BLOCKED | edge/FOV-marginal | n/a | n/a | BLOCKED | BLOCKED | seg (by design) | SIM-PASS |
| 05 cross L→R | PASS | BLOCKED | BLOCKED | BLOCKED | standalone-proven* | n/a | n/a | BLOCKED | BLOCKED | seg (by design) | SIM-PASS |
| 06 cross R→L | PASS | BLOCKED | BLOCKED | BLOCKED | standalone-proven* | n/a | n/a | BLOCKED | BLOCKED | seg (by design) | SIM-PASS |
| 07 approaching | PASS | BLOCKED | BLOCKED | BLOCKED | standalone-proven* | n/a | n/a | BLOCKED | BLOCKED | seg (by design) | SIM-PASS |
| 08 two people | PASS | BLOCKED | BLOCKED | BLOCKED | 2× standalone-proven* | n/a | n/a | BLOCKED | BLOCKED | seg ×2 (by design) | SIM-PASS |
| 09 person+chair | PASS (chair_09 spawned) | BLOCKED | BLOCKED | BLOCKED | YAML ped out-of-frustum | YAML chair out-of-frustum | n/a | BLOCKED | BLOCKED | seg (by design) | SIM-PASS |
| 10 person+table | PASS (table_10 spawned) | BLOCKED | BLOCKED | BLOCKED | YAML ped out-of-frustum | n/a | YAML table out-of-frustum | BLOCKED | BLOCKED | seg (by design) | SIM-PASS |
| 11 corridor | PASS | BLOCKED | BLOCKED | BLOCKED | standalone-proven* | n/a | n/a | BLOCKED | BLOCKED | seg (by design) | SIM-PASS |
| 12 multi-object | PASS (chair/table/column spawned) | BLOCKED | BLOCKED | BLOCKED | standalone-proven*; 12b visible live-pending | YAML statics out-of-frustum | YAML statics out-of-frustum | BLOCKED | BLOCKED | seg ×3 (by design) | SIM-PASS |

`*standalone-proven`: class detected with mask on archived sim RGB (person 0.87/0.84/0.68;
chair 0.81/0.59/0.58/0.50; dining table 0.67) — model capability proven, live per-scenario pending §7 fix.

Overall: perception validation INCOMPLETE — sim + standalone green, live pipeline red (environment fault).
No cell fabricated. Next: fix §7 → rerun this table live → then path planning.
