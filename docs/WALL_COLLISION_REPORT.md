# Wall-Collision Root-Cause Report — SKATEBOARD_IRP (2026-10-09)

## Symptom

YAML scenarios drive the vehicle into the lab front wall when run with default durations.

## Root cause (measured, not assumed)

The scenario runner commanded **unconditional open-loop cruise**: `scenario_runner.py`
`run_scenario` called `drive_step(articulation, idx, steer=0.0, omega=0.5/wheel_radius)` on
*every physics step* unless WASD manual mode (`manual_control`) was on. No perception input, no
obstacle check, no stop condition except pedestrian-completion (>10 s) or end of duration.

Geometry: default start `(6, -6, yaw 0)` faces +X; lab front-wall inner face is `x = 14.9`
(30 m room, 0.20 m wall — `lab_environment.py`). Clearance ≈ 8.9 m → contact at ≈16–18 s.

Reproduction (`isaac_sim/scripts/testing/wall_probe.py`, exact runner drive call, headless):
- cruise 0.5 m/s: `CONTACT at t=16.0s x=13.98`, dx=8.00 m over 30 s (gap −0.13 m).
- stationary hold (omega=0): dx=0.02 m, min gap 7.83 m, no contact.

Ruled out with evidence: stale ROS/`/cmd_ackermann` (runner never subscribes; uses `drive_step`
directly), waypoint controllers (separate runners only), chassis velocity hacks (none exist —
`verify_vehicle_complete.py` audits for them), joint-init kick (stationary case holds),
collision-geometry error (walls/floor/vehicle colliders correct; vehicle spawns clear).

Contributing factor: the 0.5 s command-timeout deadman lives in `AckermannROSController`, which
scenario runs never instantiate — so no failsafe applied to the cruise.

## Fix (minimal, in `scenario_runner.py` only)

- New scenario key `vehicle_behavior`: `stationary` (default — active zero-velocity hold every
  step) or `cruise` (legacy 0.5 m/s, explicit opt-in, logs an open-loop WARNING).
- None of the 12 YAMLs opts into cruise → all stationary; no YAML edits needed for safety.
- WASD/manual path, waypoint/route runners, vehicle physics, bridge: untouched.

## Regression coverage

- `wall_probe.py`: legacy-cruise-contacts (characterization) + stationary-hold PASS/FAIL + exit code.
- Schema: all 12 YAMLs default to stationary (no `cruise` request).
- Planner stop/slow unit tests (existing) still pass; live obstacle-stop tests remain NOT STARTED
  (require live perception, still blocked by the syntheticdata writer fault).

## Appendix A (2026-10-09): A-to-B success vs YAML collision — reconciled

**Historical A-to-B test:** `isaac_sim/scripts/testing/run_multi_person_vehicle_gui.py --headless/--gui`
with `scenarios/scenario_multi_person_vehicle_gui.yaml` (C8R route, 7 waypoints, 23.89 m,
START (12,7) → GOAL (2,-6)). Controller: `WaypointFollower` (heading-error P, waypoint-tolerance
switching with hairpin anticipation, accel/decel limits, final-stop at GOAL) fed by chassis USD
pose (`ComputeLocalToWorldTransform` — simulator pose, test-only per TEST-K note) → `drive_step`
directly (no `/cmd_ackermann`). Stops via `reached` + settle drift check. Historical figures:
travelled ≈22.52 m, goal error ≈0.29 m, max deviation ≈1.18–1.20 m, no wall contact.

**Why it passed while YAML scenarios crashed:** different active controller (E).
The route follower steers around walls toward waypoints and stops at the goal; the YAML runner
drove blind straight with no stop. Same vehicle/physics/colliders/lab in both. The A-to-B
controller is a *test-only* regression (pose-fed, not camera-only) and must never be wired into
perception scenarios or cited as autonomy evidence.

**Modes preserved:** (A) YAML perception scenarios — stationary, no route code runs there
(`run_scenario` contains no waypoint logic); (B) dedicated A-to-B regression — the historical
script/config above, launched explicitly only.
