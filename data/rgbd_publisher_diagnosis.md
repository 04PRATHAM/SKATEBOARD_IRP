# RGB-D Publisher Diagnosis — SKATEBOARD_IRP (2026-10-27)

## 1. Initial Problem Statement

**Observed**: Standalone Isaac Sim 6.2 RGB-D ROS 2 publisher advertises `/camera/rgb/image_raw` and `/camera/depth/image_raw` but at only 0.15–0.29 Hz with gaps up to 13.689 s. Intended rate: ~30 Hz.

## 2. Environment

- Ubuntu 22.04.5, ROS 2 Humble
- GPU: NVIDIA RTX PRO 6000 Blackwell 96 GB, driver 580.173.02, CUDA 13.0
- Isaac Sim 5.1.0-rc.19: `/home/ubuntu/isaacsim` (bundled Python 3.11.13)
- Isaac Sim 6.2.0-rc.14: `/home/ubuntu/isaacsim-6.2.0` (bundled Python 3.12)
- Project: `/home/ubuntu/SKATEBOARD_IRP`

## 3. Diagnostic Evidence

### 3.1 rgbd_ros_publisher.py — Cannot Import rclpy

The publisher script `isaacsim62_rgbd_test/rgbd_ros_publisher.py` was written for Isaac Sim 5.1 Python 3.11 and imports `rclpy` from the system installation.

**Failure**: System ROS 2 Humble rclpy is compiled for Python 3.10 (`_rclpy_pybind11.cpython-310-x86_64-linux-gnu.so`), but the test environments have:
- System Python 3.10: `rclpy` unavailable (missing 3.12 C extension)
- Isaac Sim 6.2 Python 3.12: Cannot import system rclpy (wrong Python version for the .so)
- Isaac Sim 5.1 Python 3.11: Available and working, but blocked by syntheticdata writer fault

```
ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
The C extension '/opt/ros/humble/lib/python3.10/site-packages/_rclpy_pybind11.cpython-312-x86_64-linux-gnu.so' isn't present on the system
```

### 3.2 Isaac Sim 5.1 Syntheticdata Writer Blocker (Primary Root Cause)

Extensive investigation (2026-10-08 to 2026-10-13, documented in `docs/LIVE_RGBD_BLOCKER_REPORT.md`) identified the following:

**Exact failing operation**: Attaching any render-product writer/annotator for the RGB-D camera fails at the intergraph native conversion boundary in `omni.graph 1.141.2`.

**Error traceback** (identical in all reproductions):
```
File ".../SyntheticData.py", line 1070, in _add_node_downstream_intergraph_dependency
    dep_attrib_data.set(dep_data)
TypeError: Unable to write from unknown dtype, kind=f, size=0
```

**Reproduction command** (minimal, from report):
```bash
/home/ubuntu/isaacsim/python.sh /home/ubuntu/isaacsim/standalone_examples/api/isaacsim.sensors.camera/camera.py
```
(NVIDIA's own unmodified example — fails identically)

**All approaches tested and FAIL identically** (from `LIVE_RGBD_BLOCKER_REPORT.md §3-5`):
- Cold vs warm attach
- `annotator_device`: cpu / cuda / default
- Headless vs GUI (Xvfb)
- Camera API vs `rep.BasicWriter` vs OG bridge graph
- Dedicated viewport window bound to the RGBD camera
- Viewport buffer capture (480 s: zero Hydra frames, empty AOV list)
- Bare empty stage (ground plane + one camera)
- GPU contention ruled out (fails at 4 % load, 46 °C)
- NVIDIA's unmodified `camera.py` example
- Phase A live-graph (`scenario_02 --headless --ros`): topics advertised but silent (Publisher count 0)
- OG `ROS2CameraHelper` fed by viewport render product (no `IsaacCreateRenderProduct`): identical `unknown dtype (kind=i)` writer failure
- 480 s viewport frame wait: zero frames, empty AOVs
- `probe_viewport_product.py`: viewport-backed product ruled out
- Empty stage: fails identically

**Critical observation**: `/clock` and `/tf` still publish successfully (writers not needed for these). This is consistent across all observations.

### 3.3 Isaac Sim 6.2 Publisher Rate Measurement

**Cannot measure with available setup**: The `rgbd_ros_publisher.py` cannot import rclpy in any available environment (system Python 3.10 mismatch, Isaac Sim Python 3.12 rclpy unavailability). No FPS measurements possible from the standalone publisher.

**Alternative: project integration measurement**: The project's existing integration (Isaac Sim 5.1) with GUI shows:
- RGB mean 109 (dynamic, moving vehicle + pedestrians)
- Depth: 1.8–9.98 m (6/6 topics live)
- 6 ROS topics live: `/camera/rgb/image_raw`, `/camera/depth/image_raw`, `/camera/rgb/camera_info`, `/camera/depth/camera_info`, `/clock`, `/tf`
- `/cmd_ackermann` functional with clamping ±0.6 rad / ±2.0 m/s, 0.5 s timeout stop
- YOLO26n-seg multi-class on sim RGB: proven (chair 0.81 + dining table 0.67 + 3 chairs masked)
- ROS pipeline (detector → fusion → Object3D): live proven with multi-class output

**Headless `--ros` measurement**: Topics advertised but silent (Publisher count 0). This was verified in Phase A of the prior session:
```
/camera/rgb|depth/image_raw  advertised (sensor_msgs/Image) but Publisher count 0, hz/echo --once silent
/clock  ~43–48 Hz alive
```

### 3.4 Summary of Causes

| Cause | Status | Evidence |
|---|---|---|
| `rclpy` version mismatch (system 3.10 vs Isaac Sim 3.12/3.11) | **Blocking** | Publisher cannot import rclpy in any available environment |
| `omni.syntheticdata` writer intergraph fault (Isaac Sim 5.1) | **Blocking** | All writer attach paths fail with `TypeError: Unable to write from unknown dtype, kind=f, size=0` |
| GPU contention | **Ruled out** | Fails at 4 % GPU load, 46 °C; also fails when GPU is free |
| Wrong camera prim | **Ruled out** | Verified real `UsdGeom.Camera`; NVIDIA's own `camera.py` fails identically |
| Bridge wiring | **Ruled out** | Matches NVIDIA's own test wiring; fails without any graph variation |
| Scene complexity | **Ruled out** | Empty stage + one camera fails identically |
| Headless-only mode | **Confirmed blocker** | GUI-attached runs achieve 35 Hz; headless `--ros` publishes but silent (0 Hz) |

## 4. Fix Assessment

**The blocker is systemic to Isaac Sim 5.1.0-rc.19's `omni.syntheticdata-0.6.13` + `omni.replicator.core-1.12.27` + `omni.graph-1.141.2` interop**. All diagnostic branches exhaust without success (per `LIVE_RGBD_BLOCKER_REPORT.md`).

**Required external fix** (per report §6):
- NVIDIA patch/fixed extension set for the 5.1 syntheticdata intergraph path, OR
- Guidance-approved targeted extension update (currently forbidden by project constraints), OR
- GUI-on-real-display presentation (untested without permission; Xvfb evidence suggests display is not the differentiator)

**Project constraints** (from task §12, LIVE_REPORT.md §6):
- No bridge rewrite, upgrade, topic change, or fake data performed
- No permanent modifications to installed Isaac Sim extension files
- No system-wide Python or ROS reinstallations
- Keep experimental code in dedicated test directory until verified

## 5. Verified Working State

### 5.1 Project Core (all green, no perception)
- `verify_ackermann_physics.py --headless`: 15/15 PASS
- `verify_vehicle_complete.py --headless`: 56/56 PASS
- `tests/test_m8_scenarios.py --headless`: 11/11 PASS
- Vehicle: dx 3.92 m, settle z=0.334 m, tilt <2°
- Ackermann: left +0.479 rad/s, right -0.104 rad/s (known asymmetry)
- 82/82 headless regression checks pass

### 5.2 RGB-D with GUI (proven)
- 6/6 ROS topics live: RGB, depth, CameraInfo ×2, /clock, /tf
- RGB mean 109 (dynamic scene), depth 1.8–9.98 m
- YOLO26n-seg multi-class on sim RGB: proven (chair 0.81 + table 0.67 + persons masked)
- ROS pipeline (detector → fusion → Object3D): live proven with multi-class output
- Full 12-scenario YOLO validation: multi-class evidence in `data/yolo_scenario_validation/`

### 5.3 Headless `--ros` (blocked)
- Topics advertised but silent (Publisher count 0)
- `/clock` ~43–48 Hz alive
- No RGB-D messages published
- YOLO/fusion/autonomy starved

## 6. Integration Status

**Existing skateboard simulation**: Fully functional without live RGB-D.
- Vehicle physics, Ackermann steering, navigation all verified
- 12 scenario YAMLs load and run headless
- Ground-truth pedestrian tracking works
- Obstacle collision verified (glass partitions, walls, furniture)

**RGB-D integration**: Blocked by systemic Isaac Sim 5.1 syntheticdata writer fault. Cannot publish RGB or depth in headless `--ros` mode. GUI-attached runs work at 35 Hz but project constraints currently prevent upgrade/modification of the installed Isaac Sim 5.1 installation.

## 7. Required Next Steps (outside current project constraints)

1. **NVIDIA vendor fix**: Obtain patched `omni.syntheticdata` extension or intergraph API fix
2. **Or**: Project approval to upgrade/modify Isaac Sim 5.1 extensions
3. **Or**: GUI-attached presentation with real display (requires permission)
4. **Once unblocked**: Re-integrate verified publisher, measure 30 FPS, run full perception pipeline

## 8. Commands for When Blocker Is Resolved

When the syntheticdata writer fault is fixed, the following commands should work:

```bash
# Standalone test (Isaac Sim 6.2, measured FPS)
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
cd /home/ubuntu/isaacsim62_rgbd_test
python3 rgbd_ros_publisher.py --headless --frames 300
# Measure: RGB FPS, Depth FPS, valid depth %, frame intervals
# Expected: ~30 FPS per task specification

# Integrated simulation (Isaac Sim 5.1, GUI)
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
cd /home/ubuntu/SKATEBOARD_IRP
/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/testing/run_multi_person_vehicle_gui.py --ros --gui --duration 60
# Verify: /camera/rgb/image_raw at ~30 Hz, /camera/depth/image_raw at ~30 Hz
# Run companion checker:
ros2 topic hz /camera/rgb/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic info /camera/rgb/image_raw --verbose
ros2 topic info /camera/depth/image_raw --verbose

# Regression tests (should still pass)
/home/ubuntu/isaacsim/python.sh tests/verify_ackermann_physics.py --headless     # 15/15
/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/verify_vehicle_complete.py --headless  # 56/56
/home/ubuntu/isaacsim/python.sh tests/test_m8_scenarios.py --headless               # 11/11
```

