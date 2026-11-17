# Isaac Sim 6.2 Migration Report — SKATEBOARD_IRP

**Date:** 2026-10-30
**From:** Isaac Sim 5.1.0-rc.19 (`/home/ubuntu/isaacsim/`, retained as rollback)
**To:** Isaac Sim 6.2.0-rc.14 (`/home/ubuntu/isaacsim-6.2.0/`, VERSION `6.2.0-rc.14+release.52805.02189a84.gl`)
**Bundled Python:** 3.12.14 | **System Python/ROS 2:** 3.10.12 + Humble (unchanged)
**Status:** Phases 0–7 verified; Phase 8 (Live Perception under Vehicle Motion) VERIFIED; Phases 9–12 pending.

---

## 1. Executive summary

The migration preserves the architecture decision **Design A (process boundary)**: the simulator
publishes ROS 2 topics from its bundled bridge (Python 3.12 / bundled rclpy), while perception,
fusion and planning run externally on system Python 3.10 with `perception_venv` (ROS 2 Humble
rclpy). No vendor files were modified; no commits were pushed; the 5.1 install is intact.

**Headline result:** the 5.1 hard blocker — the `omni.syntheticdata` writer fault
(`Unable to write from unknown dtype, kind=i/f, size=0`) that made live camera topics
impossible — **does not reproduce under 6.2**. A full external-subscriber acceptance test
passes **10/10**, and the physics regression suites reproduce the 5.1 baselines exactly
(**56/56 vehicle, 15/15 Ackermann**).

---

## 2. Verified results (evidence)

| # | Item | Result | Evidence |
|---|------|--------|----------|
| Phase 0 | Baseline preserved, rollback intact | PASS | Git: single commit `8c843a8`, staged+unstaged changes preserved (not committed); `/home/ubuntu/isaacsim/` untouched |
| Phase 1 | 6.2 install + Python 3.12.14 | PASS | `VERSION` file; `python.sh` reports 3.12.14 |
| Phase 1 | External ROS 2 delivery of RGB-D | **PASS 10/10** | `check_rgbd_topics.py --window 20` on system Python 3.10 vs live 6.2 headless sim |
| Phase 1 | TF frame content | PASS | `ros2 topic echo /tf --once`: `world→Vehicle` (6.0, −6.0, 0.35) = configured `start_xy`+ride height; `world→RGBD_Camera` (6.9008, −5.99999, 1.20001) = mount (0.9, 0, 1.2) |
| Phase 1 | Camera publish rate | ~113 Hz | `ros2 topic hz /camera/rgb/image_raw` (headless, unlocked loop, window 227) |
| Phase 2 | `build_world()` under 6.2 | PASS | lab + vehicle + RGB-D camera + inspection cameras; physics init; timeline play; clean `app.close()` |
| Phase 3 | `verify_vehicle_complete.py --headless` | **PASS 56/56** | identical to 5.1 baseline (incl. camera mount N1–N5) |
| Phase 3 | `verify_ackermann_physics.py --headless` | **PASS 15/15** | identical to 5.1 baseline |
| Phase 4 | Render product config | PASS | `resolution=(640,480)`, `camera → /World/Vehicle/Chassis/Sensors/RGBD_Camera/RGBD_Camera`, vars `LdrColorSD`+`DistanceToImagePlaneSD` |
| Phase 5 | All 7 ROS topics live + external discovery | PASS | `ros2 topic list` on isolated domain: rgb/depth image ×2, camera_info ×2, `/clock`, `/tf`, publisher count 1 each |
| Phase 6 | YOLO26n-seg + depth fusion | **PASS** | Integrated stack ran end-to-end headless (sim→yolo→fusion→validator, sim exit 0); **172 live Object3D messages** over DDS via focused harness |
| Phase 7 | Planner → /cmd_ackermann → vehicle response | **PASS (sim half)** | /cmd_ackermann over DDS → drive_step → vehicle moved 1.40 m, yaw 0.65 rad; planner logic (CRUISE/STOP/SLOW) verified; planner runs externally under 6.2 python.sh |
| Phase 8 | Live Perception under Vehicle Motion (Stationary & Manual WASD) | **PASS 5/5** | Tests 1–5: Stationary moving ped (76 obs), manual straight (21 obs, +2.51m), manual steer (28 obs, +6.8° yaw), vehicle-ped interaction (19 obs, +4.21m), two peds (143 obs). RGB/Depth: ~39–41 Hz, YOLO: 6–9 ms, Fusion: ~9 Hz. |

### 2.1 The 10/10 acceptance run (Phase 5)

External system-Python 3.10 + ROS 2 Humble subscriber vs 6.2 headless sim:

```
[PASS] RGB publishing continuously n=60
[PASS] Depth publishing continuously n=60
[PASS] /clock alive n=238
[PASS] /tf alive n=235
[PASS] CameraInfo RGB+depth received
[PASS] TEST-H stamps synchronized (<0.1 s) dt=0.0000 s
[PASS] TEST-H frame_ids correct camera_rgb_optical_frame / camera_depth_optical_frame
[PASS] TEST-H timestamps valid sim-time
[PASS] TEST-I depth valid+finite (center region) frac=1.00 med=7.98 m
[PASS] TEST-J fusion Object3D plausible (center-stub) dist=7.97 m
[RESULT] 10/10 ROS checks PASS
```

Reproduced twice: once with an extra diagnostic render-product graph, and once in the
**control run (bridge-only, tight loop, no sleeps, isolated `ROS_DOMAIN_ID=77`)** — proving
neither extra graphs nor frame sleeps are required.

---

## 3. How the 6.2 bridge works (verified pipeline)

```
/ROS2_Bridge (execution OmniGraph)
  OnPlaybackTick ─▶ IsaacCreateRenderProduct (typeVersion 3, 640×480)
                      ├▶ ROS2CameraHelper "rgb"   → /camera/rgb/image_raw   (rgb8)
                      ├▶ ROS2CameraHelper "depth" → /camera/depth/image_raw (32FC1)
                      ├▶ ROS2CameraInfoHelper ×2  → /camera/*/camera_info
                      └▶ ROS2PublishTransformTree → /tf   (+ IsaacReadSimulationTime → ROS2PublishClock → /clock)
replicator writers (registered by isaacsim.ros2.nodes once isaacsim.ros2.bridge is enabled):
  LdrColorSDROS2PublishImage, DistanceToImagePlaneSDROS2PublishImage, ROS2PublishCameraInfo
  → /Render/PostProcess/SDGPipeline
     SdOnNewFrame → OgnRefTimeGate → SdOnNewRenderProductFrame → sync gates
       → C++ ROS2PublishImage / ROS2PublishCameraInfo nodes → DDS
```

No changes to `isaac_sim/scripts/bridge/ros2_bridge.py` were required for delivery.

---

## 4. Pitfalls discovered (do not re-fall into these)

1. **Stray processes impersonate a live sim.** A 5.1 `scenario_runner` (started 2026-10-26,
   `--duration 20 --gui`, never exited) kept publishing `/clock`+`/tf` on the default domain.
   Early "no camera topics" checks were (a) run against sims that had already exited at ~17 s
   and (b) fed `/clock`/`/tf` by the stray. **Rule: validate on an isolated
   `ROS_DOMAIN_ID` and confirm the sim process is alive while checking.**
   Disposition: killed with user approval on 2026-10-29.
2. **Writer registry looks empty too early.** ROS2 writers are registered during
   `isaacsim.ros2.nodes` startup / `isaacsim.ros2.bridge` enable — query before that and you
   get `[]`.
3. **USD attribute listing ≠ runtime value.** `CreateRenderProduct.inputs:cameraPrim` did not
   appear in a `GetAttributes()` dump yet was correctly authored; always read runtime values
   via `og.Controller.attribute(path).get()`.
4. **Render product path changed vs 5.1:** 6.2 creates `/Render/OmniverseKit/HydraTextures/Replicator`
   (a `RenderProduct` prim) instead of a `/World/RenderProducts/...`-style path.

---

## 5. Known issues / follow-ups

| Issue | Severity | Status |
|-------|----------|--------|
| `OgnROS2PublishTransformTree.targetPrims` deprecated (use `OgnIsaacComputeTransformTree` parentFrames/childFrames); `[PoseTree] target getObjectType eInvalid` warnings | Cosmetic — TF content verified correct | Open (cleanup) |
| `camera_info_utils`: "Forcing fy to fx (575.66 != 457.85)" — renderer assumes square pixels | Informational | Documented; camera.yaml keeps fx=fy placeholders |
| **Sim-side `rclpy` fails under 6.2 when `/opt/ros/humble/setup.bash` is sourced first:** system Humble's py3.10 `rclpy` shadows the bundled py3.12 one (extension does `sys.path.append`, so system wins), error `No module named 'rclpy._rclpy_pybind11'` | Blocks sim-side Python ROS nodes (e.g. `run_ackermann_demo.py` subscriptions) | **FIXED & validated 2026-10-30:** launch sim **without** sourcing `setup.bash`; `ROS_DISTRO=humble`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, prepend bundled `humble/rclpy` to PYTHONPATH and `humble/lib` to LD_LIBRARY_PATH. Validated `import rclpy` + pub/sub inside Kit and outside Kit; `/cmd_ackermann` → vehicle response PASS. |
| Camera rate only measured in unlocked headless loop (~113 Hz) | Minor | Realtime-locked rate still open |
| Exact time-to-first-topic not bounded | Minor | Measured ≤ ~60 s after `build_world()` completion |

---

## 6. Files changed during migration (working tree, not committed)

- `TODO.md` — deduplicated into one authoritative Phase 0–12 migration section; statuses carry evidence.
- `work.md` — Session 26 entry (Phase 5 verification, pitfalls, control run).
- `isaac_sim/scripts/testing/run_integrated_perception_gui.py` — `ISAAC_PYTHON` now
  env-overridable, default `/home/ubuntu/isaacsim-6.2.0/python.sh`
  (rollback: `ISAAC_PYTHON=/home/ubuntu/isaacsim/python.sh …`).
- `docs/ISAAC_SIM_6_2_MIGRATION_REPORT.md` — this file.

- `docs/ISAAC_SIM_6_2_MIGRATION_REPORT.md` — this file (Phases 6–7 results appended below).

## 6. Phase 6 — YOLO + depth fusion (verified 2026-10-30)

Integrated stack ran end-to-end headless under 6.2: sim (scenario_02_person_center, 60 s sim-time, exit code 0) → yolo_seg_node (infer 7–13 ms, detection every frame, t=1→59.7) → rgbd_fusion_node → validator ("stream live", scenario PNG saved).

**Live fusion metrics:** focused harness (sim + yolo + fusion + `/perception/objects_3d` counter, near-realtime via direct `world.step`, isolated `ROS_DOMAIN_ID=81`) → **172 Object3D (Detection3DArray) messages** published live over DDS. Confirms the live depth → fusion → Object3D chain.

`perception_venv`: py3.10.12, ultralytics 8.4.163, rclpy OK, `yolo26n-seg.pt` at repo root.

## 7. Phase 7 — Reactive steering (verified 2026-10-30)

**Sim-side `/cmd_ackermann` → vehicle response: PASS.** A kit process ran two rclpy nodes (command-sequence publisher + subscriber→`drive_step`). `/cmd_ackermann` over DDS → `AckermannController.drive_step` → vehicle moved **dist=1.40 m**, **yaw_change=0.65 rad** under sustained 0.5 rad steer. Uses the sim-side rclpy fix (bundled rclpy path prepended to PYTHONPATH, bundled `humble/lib` prepended to LD_LIBRARY_PATH, no `setup.bash` sourcing, isolated `ROS_DOMAIN_ID=82`).

**Planner logic** (`rgbd_perception.planner.plan`, pure Python, verified in perception_venv): CRUISE when center clear, STOP when center obstacle < `stop_distance`, SLOW+steer toward freer side in slow band, STOP when both sides blocked. Hard safe-stop gate preserved (stale watchdog → steer 0, speed 0). No GT/odom/LiDAR/GPS inputs.

`camera_planner_node` runs externally under `/home/ubuntu/isaacsim-6.2.0/python.sh` (rclpy + `ackermann_msgs` import+init+publish verified outside Kit).

**Full 4-process single run (sim+yolo+fusion+planner→cmd_ackermann→drive) still open** — each link independently verified.

## 8. Phase 8 — Live Perception under Vehicle Motion & Manual WASD Driving (verified 2026-10-30)

Live perception under moving conditions was validated across 5 distinct experimental stages using PhysX vehicle dynamics and RealHuman pedestrian motion:

1. **Test 1: Stationary Vehicle Baseline with Moving Pedestrian (Scenario 07)**
   - Pedestrian moving towards vehicle from 11.0 m to 7.5 m (Z relative: 2.41 m → 0.40 m).
   - RGB/Depth rate: 41.09 Hz (492/493 frames). YOLO inference: 7–9 ms. Fusion rate: 9.01 Hz.
   - 76 valid 3D pedestrian observations published over DDS with smooth monotonic distance decay (2.414 m → 0.405 m).
2. **Test 2: Manual Straight Movement ('W') + Moving Pedestrian**
   - Vehicle driven forward by +2.51 m (x: 6.00 m → 8.51 m).
   - RGB/Depth rate: 38.89 Hz (544 frames). YOLO rate: 9.03 Hz.
   - 21 valid 3D pedestrian observations. Optical Z: 2.088 m → 0.601 m.
3. **Test 3: Manual Steering ('A'/'D') + Moving Pedestrian**
   - Steer command applied (+0.30 rad left), chassis rotated +6.8° yaw, vehicle displaced laterally y: -6.00 m → -5.74 m.
   - RGB/Depth rate: 39.23 Hz (588 frames). YOLO rate: 9.01 Hz.
   - 28 valid 3D pedestrian observations tracked across camera field-of-view change. Optical Z: 1.787 m → 0.336 m → 3.274 m.
4. **Test 4: Moving Pedestrian Interaction**
   - Vehicle drove forward by +4.21 m (x: 6.00 m → 10.21 m) while pedestrian approached.
   - RGB/Depth rate: 39.61 Hz (554 frames). YOLO rate: 9.15 Hz.
   - 19 valid 3D pedestrian observations. Optical Z: 2.089 m → 0.801 m.
5. **Test 5: Two Moving Pedestrians in Scene (Scenario 08)**
   - Vehicle driven under manual control with multiple pedestrian assets active.
   - RGB/Depth rate: 38.68 Hz (580 frames). YOLO rate: 9.13 Hz (6 ms GPU inference).
   - 143 valid 3D observations with simultaneous multi-target detections (n=2). Optical Z: 4.445 m → 2.454 m → 2.754 m.

**Architectural Fixes Implemented:**
- `real_human.py`: Implemented 6.2-compatible smooth trajectory updates via `xformOp:translate` to bypass missing 5.1 `NavigationManager` / `omni.anim.people`.
- `scenario_runner.py`: Integrated `wasd_controller.py` with `carb.input` and `drive_step`, supporting both interactive GUI keyboard control and programmatic input.
- `run_integrated_perception_gui.py`: Resolved pipe buffer saturation by draining non-blocking stdout lines in loops.

---

## 9. Rollback procedure

```bash
# 5.1 remains installed and unmodified at /home/ubuntu/isaacsim/
export ISAAC_PYTHON=/home/ubuntu/isaacsim/python.sh   # launchers honoring the env var
/home/ubuntu/isaacsim/python.sh <script>              # or direct
```
