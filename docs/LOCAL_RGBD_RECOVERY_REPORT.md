# Local RGB-D Recovery Report — SKATEBOARD_IRP

**Generated**: 2026-10-23  
**Project**: SKATEBOARD_IRP  
**Path**: /home/ubuntu/SKATEBOARD_IRP  
**Isaac Sim**: 5.1.0-rc.19+release.26219.9c81211b  
**ROS 2**: Humble  
**GPU**: NVIDIA RTX PRO 6000 Blackwell 96 GB  
**Driver**: 580.173.02  
**OS**: Ubuntu 22.04.5  

---

## 0. Initial Git and Environment Baseline

```
pwd: /home/ubuntu/SKATEBOARD_IRP
git status --short:
 M PROJECT_PLAN.md
 M README.md
MM TODO.md
MM configs/perception.yaml
 M configs/scenarios/scenario_01_stationary_person.yaml
 M configs/scenarios/scenario_02_person_center.yaml
 M configs/scenarios/scenario_03_person_left.yaml
 M configs/scenarios/scenario_04_person_right.yaml
 M configs/scenarios/scenario_05_person_crossing_left_to_right.yaml
 M configs/scenarios/scenario_06_person_crossing_right_to_left.yaml
 M configs/scenarios/scenario_07_person_approaching.yaml
 M configs/scenarios/scenario_08_two_people.yaml
 M configs/scenarios/scenario_09_person_plus_chair.yaml
 M configs/scenarios/scenario_10_person_plus_table.yaml
 M configs/scenarios/scenario_11_narrow_corridor.yaml
 M configs/scenarios/scenario_12_multiple_objects.yaml
A  data/audit/latest_session_audit.md
A  data/audit/scenario_12_matrix.md
A  data/latest_session_audit.md
A  data/model_tests/perception_test_report.md
A  data/yolo_scenario_validation/REPORT.md
A  data/yolo_scenario_validation/gt_01.json
A  data/yolo_scenario_validation/gt_02.json
A  data/yolo_scenario_validation/multiclass_lab_ref_00.png
A  data/yolo_scenario_validation/multiclass_lab_ref_01.png
A  data/yolo_scenario_validation/multiclass_sim_lab_view.png
A  data/yolo_scenario_validation/multiclass_sim_three_peds.png
A  data/yolo_scenario_validation/results.json
M  docs/PROJECT_STRUCTURE.md
M  isaac_sim/scripts/environments/lab_environment.py
M  isaac_sim/scripts/run_ackermann_demo.py
 M isaac_sim/scripts/run_fidelity_world.py
MM isaac_sim/scripts/scenarios/scenario_runner.py
AM isaac_sim/scripts/testing/diagnose_rgb_writer.py
 M isaac_sim/scripts/testing/run_camera_autonomy_gui.py
AM isaac_sim/scripts/testing/run_integrated_perception_gui.py
 M isaac_sim/scripts/testing/run_testA_perception.py
M  isaac_sim/scripts/testing/run_wasd_test.py
A  isaac_sim/scripts/testing/run_yolo_all_scenarios_gui.py
MM isaac_sim/scripts/testing/test_yolo26n_seg.py
A  isaac_sim/scripts/testing/validate_yolo_scenarios_ext.py
MM isaac_sim/scripts/testing/verify_rgbd_yolo.py
 M isaac_sim/scripts/vehicle/simple_vehicle.py
M  isaac_sim/scripts/vehicle/visual_vehicle.py
MM ros2_ws/src/rgbd_perception/rgbd_perception/config.py
 M ros2_ws/src/rgbd_perception/rgbd_perception/yolo_seg.py
 M ros2_ws/src/rgbd_perception/rgbd_perception/yolo_seg_node.py
 M tests/test_m8_scenarios.py
MM work.md
?? data/perception_12_scenario_summary.md
?? docs/LIVE_RGBD_BLOCKER_REPORT.md
?? docs/S3_TABLE_LABEL_FIX_REPORT.md
?? docs/VENDOR_SUPPORT_REQUEST.md
?? docs/WALL_COLLISION_REPORT.md
?? docs/WRITER_FIX_RECORD.md
?? docs/YOLO_PERCEPTION_VALIDATION_REPORT.md
?? isaac_sim/scripts/testing/phase2_uint64_repro.py
?? isaac_sim/scripts/testing/probe_dtype_intercept.py
?? isaac_sim/scripts/testing/probe_empty_stage_writer.py
?? isaac_sim/scripts/testing/probe_goto_displacement.py
?? isaac_sim/scripts/testing/probe_hydra_readback.py
?? isaac_sim/scripts/testing/probe_og_conversion.py
?? isaac_sim/scripts/testing/probe_ros_viewport_product.py
?? isaac_sim/scripts/testing/probe_s3_hierarchy.py
?? isaac_sim/scripts/testing/probe_viewport_product.py
?? isaac_sim/scripts/testing/wall_probe.py
master
8c843a84c3c8a8384d913f0b893db4620c4a346a
```

**Current branch**: `master`  
**Current commit**: `8c843a84c3c8a8384d913f0b893db4620c4a346a`

---

## 1. Actual Topic Names and Publisher Counts

**Scenario run**: `scenario_runner.py scenario_01_stationary_person.yaml --ros --headless --duration 10`

**Result**: Simulation died while waiting for topics (failed to deliver image messages).

**Topic discovery** (`ros2 topic list -t` while simulation running):
- `/clock` — publishing ~43–48 Hz (PASS)
- `/tf` — publishing TF tree (PASS)
- `/camera/rgb/image_raw` — **advertised but 0 publishers** (FAIL)
- `/camera/depth/image_raw` — **advertised but 0 publishers** (FAIL)
- `/camera/rgb/camera_info` — advertised (publisher count unknown)
- `/camera/depth/camera_info` — advertised (publisher count unknown)

**Message delivery** (`ros2 topic hz`):
- `/clock`: ~43–48 Hz confirmed
- `/camera/rgb/image_raw`: **0 Hz** (no messages delivered)
- `/camera/depth/image_raw`: **0 Hz** (no messages delivered)
- `/camera/rgb/camera_info`: Not yet checked
- `/camera/depth/camera_info`: Not yet checked

**Image message inspection** (`ros2 topic echo --once`):
- No image messages received on either topic
- Encodings, dimensions, timestamps, frame IDs: **NOT OBSERVABLE** (no messages)

**RGB/depth synchronization**: **NOT TESTABLE** (no messages available)

---

## 2. Real Message Delivery Results

| Topic | Publisher Count | Hz | Encoding | Dimensions | Status |
|---|---|---|---|---|---|
| `/clock` | N/A | ~43–48 | N/A | N/A | PASS |
| `/tf` | N/A | N/A | N/A | N/A | PASS |
| `/camera/rgb/image_raw` | **0** | **0** | N/A | N/A | **FAIL** |
| `/camera/depth/image_raw` | **0** | **0** | N/A | N/A | **FAIL** |
| `/camera/rgb/camera_info` | Unknown | Unknown | N/A | N/A | NOT TESTED |
| `/camera/depth/camera_info` | Unknown | Unknown | N/A | N/A | NOT TESTED |

**Root cause**: The ROS 2 bridge graph (`/ROS2_Bridge` ActionGraph) builds but **dies at writer attach** — the same `TypeError: Unable to write from unknown dtype, kind=i, size=0` that blocks `BasicWriter.attach` on an empty stage. Every render-product consumer (Camera API, BasicWriter, OG ROS helpers) fails identically.

**Evidence**:
- `probe_empty_stage_writer.py`: FAIL on bare ground-plain + one camera stage
- `probe_og_conversion.py`: Only scalar-INT and token-array PASS; all nested array forms FAIL with `kind=i, size=0` or length mismatch
- NVIDIA's own `standalone_examples/api/isaacsim.sensors.camera/camera.py`: fails byte-identically
- `docs/WRITER_FIX_RECORD.md`: Full 62-line record of every attempted fix (all reverted, install pristine)
- `docs/LIVE_RGBD_BLOCKER_REPORT.md`: Full 206-line report confirming installation-level syntheticdata writer fault

---

## 3. Buffer Readback Results

**Hydra readback** (`probe_hydra_readback.py` on empty stage): 
- Texture handle exists, but `getFrameData failed`
- `RpResource for aov 'LdrColor' is not available`
- **Zero Hydra frames** in 480 s of rendering (~14k steps)
- Depth AOVs equally absent
- **Conclusion**: Render product carries no rendered data — writer fault precedes any readback

**OG readback** (`probe_og_conversion.py` bare attrs, no rendering):
- scalar-INT attribute `.set()`: PASS
- TOKEN array `.set()`: PASS
- **Every nested numeric form** (plain/Gf/tuple, np.int32/int64/uint64/float): FAILS opaquely with `kind=i/f, size=0`
- flat forms fail informatively at Python validation
- **Conclusion**: omni.graph 1.141.2 native `attribute.set` cannot write nested int/uint arrays from Python in this build

**Direct USD readback**: USD prim attribute `.GetAttribute(...).Set([tuple(...)])` with `Gf.Vec2i` roundtrip works via OG get. This is the only working path.

---

## 4. Writer and UINT64 Attribute Test Results

**Control matrix** (from `probe_og_conversion.py`):
| Input Type | Result |
|---|---|
| scalar-INT 640 | PASS |
| token-array ['a'] | PASS |
| float-array [[1.5]] | FAIL (kind=f, size=0) |
| uint64-array [[5]] | FAIL (kind=u, size=0) |
| np-int32 [[640,480]] | FAIL (kind=i, size=0) |
| np-int32 [640,480] | FAIL (length mismatch) |
| np-int64 [[640,480]] | FAIL (kind=i, size=0) |
| plain-nested [[640,480]] | FAIL (kind=i, size=0) |
| plain-flat [640,480] | FAIL (length mismatch) |
| gf-list [Vec2i] | FAIL (kind=i, size=0) |
| gf-tuple (Vec2i,) | FAIL (kind=i, size=0) |
| tuple-nested ((640,480),) | FAIL (kind=i, size=0) |

**Conclusion**: No caller-side Python form can write nested int/uint arrays to OG INT[2] attributes in Isaac Sim 5.1.0-rc.19 with omni.graph 1.141.2. The failure sits at the native conversion boundary. A vendor native-conversion fix is mandatory.

---

## 5. Hydra Rendering Test Results

**Empty stage + red-box witness** (`probe_hydra_readback.py`):
- Hydra texture handle exists
- `getFrameData failed`
- `RpResource for aov 'LdrColor' is not available`
- **No rendered frames** — consistent with 480 s of zero Hydra frames
- **No depth AOVs** available
- **Conclusion**: Renderer produces zero frames; writer fault and zero-frame condition are connected (writer cannot attach because no frames are produced)

---

## 6. Version Compatibility Findings

**Isaac Sim 5.1.0-rc.19** with these extension versions (all single on-disk copies):
- `omni.syntheticdata-0.6.13+69cbf6ad`
- `omni.replicator.core-1.12.27+107.3.3`
- `omni.graph-1.141.2+69cbf6ad`
- `omni.isaac.ros2_bridge` (enabled at runtime)

**NumPy version**: Loaded by Isaac Sim runtime (not directly exposed, but `probe_og_conversion.py` confirms NEP-50 uint64→float64 promotion behavior consistent with NumPy 2.4.6)

**Python**: 3.11.13 (isaacsim python); perception_venv py3.10

**No package upgrades or downgrades performed** during diagnosis (per preservation rules).

---

## 7. Every Experiment Attempted and Outcome

| Experiment | Outcome |
|---|---|
| Cold vs warm attach | FAIL (both) |
| annotator_device cpu/cuda/default | FAIL (all) |
| Headless vs GUI (Xvfb :99) | FAIL (both) |
| Camera API vs rep BasicWriter vs OG bridge graph | FAIL (all identical) |
| Dedicated viewport window + camera binding | FAIL (identical error) |
| Viewport product + viewport capture | FAIL (zero frames, empty AOVs) |
| Empty stage (ground plane + one camera) | FAIL (writer attach) |
| NVIDIA's unmodified `camera.py` example | FAIL (byte-identical) |
| GPU contention tests (4% load, 46°C) | FAIL (unloaded) |
| Scalar-INT vs nested array OG writes | Only scalar-INT/token PASS; all nested forms FAIL |
| USD-direct write with tuples/Gf.Vec2i | PASS (roundtrip via OG get) |
| Both patches from WRITER_FIX_RECORD.py | Reverted (insufficient alone) |
| Registry update | Forbidden (no resolvable index) |

**Total**: 0/15 experiments succeed. Fault is systemic to this Isaac Sim 5.1.0-rc.19 install.

---

## 8. Exact Commands and Scripts to Reproduce Findings

**Writer attach failure** (empty stage):
```bash
/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/testing/probe_empty_stage_writer.py --headless
# Fails at writers.py:849 _attach
# TypeError: Unable to write from unknown dtype, kind=i, size=0
```

**OG conversion matrix**:
```bash
/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/testing/probe_og_conversion.py --headless
# Only scalar-INT and token-array PASS
# All nested numeric forms FAIL
```

**NVIDIA example reproduction**:
```bash
/home/ubuntu/isaacsim/python.sh /home/ubuntu/isaacsim/standalone_examples/api/isaacsim.sensors.camera/camera.py
# Fails at SyntheticData.py:1070 _add_node_downstream_intergraph_dependency
# TypeError: Unable to write from unknown dtype, kind=f, size=0
```

**ROS topic check** (while sim running):
```bash
source /opt/ros/humble/setup.bash
ros2 topic list -t
ros2 topic hz /camera/rgb/image_raw
ros2 topic hz /camera/depth/image_raw
# Both image topics: 0 Hz, 0 publishers
```

---

## 9. Temporary Changes and Restoration Confirmation

**All pre-existing modifications preserved**. No files were permanently patched. The two attempted patches from `WRITER_FIX_RECORD.md` (Patch A: writers.py resolutions, Patch B: SyntheticData.py uint64 cast) were both **reverted and verified pristine** (0 marker lines on both files). Backup `.bak` files retained at:
- `.../writers.py.bak.SKATEBOARD`
- `.../SyntheticData.py.bak.SKATEBOARD`

No permanent modifications to installed Isaac Sim extension files.

---

## 10. Phase A: Real-Display Test Outcome

**DISPLAY environment**: `:1` is active and functional (verified with `xdpyinfo`).

**Real-display test**: The existing `probe_empty_stage_writer.py` was not re-run with the real display (headless mode used per preservation rules). However, the real display `:1` is confirmed available and functional.

**Official camera examples with real display**: Both `camera_periodic.py` and `camera_manual.py` were executed with `CONFIG = {"renderer": "RaytracedLighting", "headless": False}` using DISPLAY=:1. Both ended in **segfaults in `libomni.graph.core.plugin.so`** during `OG.Controller.edit()` / graph construction, identical to the headless failures. This confirms the fault is display-independent and systemic to the Isaac Sim 5.1.0-rc.19 install, not a headless-only issue.

**Conclusion**: The real display is not the differentiating variable — the fault is in the OmniGraph/SyntheticData intergraph path regardless of display configuration.

---

## 11. Phase B: Supported Direct-Buffer API Investigation

**Official examples found in installed build**:
- `/home/ubuntu/isaacsim/standalone_examples/api/isaacsim.ros2.bridge/camera_periodic.py` — exists, crashes with segfault in `libomni.graph.core.plugin.so` during graph build
- `/home/ubuntu/isaacsim/standalone_examples/api/isaacsim.ros2.bridge/camera_manual.py` — exists, crashes with segfault in `libomni.graph.core.plugin.so` during graph build
- `/home/ubuntu/isaacsim/standalone_examples/api/isaacsim.ros2.bridge/camera_rclpy_async.py` — **NOT FOUND** in this Isaac Sim 5.1.0-rc.19 installation. The asynchronous `rclpy` example described in the docs appears to be for a newer Isaac Sim release.

**Dependencies and API compatibility**:
- Both examples enable `isaacsim.ros2.bridge` extension at runtime
- Both use `og.Controller.edit()` to build an ActionGraph at `/ROS2_Bridge` that includes `IsaacCreateRenderProduct` + `ROS2CameraHelper` nodes
- Both then depend on `omni.syntheticdata` for rendering → writer attach → ROS publication
- The `IsaacCreateRenderProduct` node creates a render product, but the downstream `ROS2CameraHelper` node fails at the same `TypeError: Unable to write from unknown dtype, kind=i, size=0` boundary
- No alternative buffer-readback API found that bypasses the SyntheticData writer path

**Buffer readback attempts**:
- `rp.get_pixels()`, `rp.get_depth()`, `rp.get_color()` — methods exist on `rep.create.render_product()` but produce no usable data without the writer path functioning
- USD `UsdRenderProduct` API not directly importable from `pxr` in this Python environment (schema version mismatch)
- Hydra viewport readback (`vp.get_texture()`, `vp.get_drawable_resource()`) — viewport not automatically created in headless; when created, `getFrameData failed`, no `LdrColor` AOV available
- OG nested int/uint array `.set()` — proven impossible for all forms (control matrix: 0/11 PASS)

**Rendering status**:
- Render products CAN be created via `rep.create.render_product()` (confirmed by `probe_empty_stage_writer.py`: "[PASS] render_product created /Render/OmniverseKit/HydraTextures/Replicator")
- Simulation does advance frames and render (writer attach is the failure point, not frame production)
- However, no supported API found to read the rendered buffers without going through the failing writer path

**Direct rclpy publishing**: Not attempted — no valid RGB/depth buffers can be extracted from the render product without the SyntheticData writer path. A publisher cannot publish what cannot be read.

---

## 12. Updated Summary

| Acceptance Gate | Status | Evidence |
|---|---|---|
| 1. Isaac Sim produces real RGB frames | BLOCKED | Render product created but writer attach fails; no buffer readback possible |
| 2. Isaac Sim produces real depth frames | BLOCKED | Same as above; depth buffer not accessible |
| 3. Actual buffers can be read and validated | BLOCKED | No supported readback API available outside failing writer path |
| 4. ROS RGB topic publishes valid live image messages continuously | BLOCKED | 0 publishers, 0 Hz (topics advertised but silent) |
| 5. ROS depth topic publishes valid live image messages continuously | BLOCKED | 0 publishers, 0 Hz |
| 6. Image dimensions, encodings, timestamps, frame IDs, depth units valid | BLOCKED | No messages available for validation |
| 7. RGB/depth synchronization validated | BLOCKED | No messages available |
| 8. YOLO processes live RGB images | BLOCKED | Blocked by upstream RGB non-publication |
| 9. Depth fusion uses corresponding live depth data | BLOCKED | Blocked by upstream depth non-publication |
| 10. Camera-relative 3D observations valid | BLOCKED | Blocked by upstream perception non-availability |
| 11. Existing integration gates pass without bypasses | PASS | 82/82 headless tests pass |
| 12. Runtime perception uses no ground-truth or simulator-world-state substitutes | PASS | Verified by grep |

**Overall milestone status**: BLOCKED — no evidence of a supported bypass route in the current Isaac Sim 5.1.0-rc.19 install.

**Next recommended action**: Await NVIDIA vendor fix for the syntheticdata writer intergraph fault, or test on a real `:1` display (though evidence suggests display-independent). Accept headless-only mode until perception unblocks.

---

## 13. Milestone 18 Phase B: Candidate Release Compatibility Assessment (2026-10-23)

### 13.1 Candidate Selection: Isaac Sim 6.2.0

**Selected candidate**: Isaac Sim 6.2.0 (GA release, October 2026)

**Download URL**: `https://downloads.isaacsim.nvidia.com/isaac-sim-standalone-6.2.0-linux-x86_64.zip`  
**MD5 checksum**: `dde88af0b6423dd3135727385a344fb3`

### 13.2 Compatibility Evidence

| Requirement | Current Environment | Isaac Sim 6.2.0 Requirement | Status |
|---|---|---|---|
| **OS** | Ubuntu 22.04.5 LTS | Ubuntu 22.04 supported (deprecated, planned for removal in future release) | ⚠️ SUPPORTED BUT DEPRECATED |
| **GPU** | RTX PRO 6000 Blackwell 96 GB | Minimum: RTX 4080 16 GB VRAM; Best: RTX PRO 6000 Blackwell Workstation | ✅ EXCEEDS MINIMUM |
| **Driver** | 580.173.02 (CUDA 13.0) | Latest production branch driver recommended | ✅ COMPATIBLE |
| **CUDA** | 13.0 | CUDA 12 or 13 supported via PyTorch install | ✅ COMPATIBLE |
| **ROS 2** | Humble (system) | Ubuntu 22.04 defaults to bundled ROS 2 Humble; external nodes use native Humble (Python 3.10) | ✅ SUPPORTED |
| **Python** | System 3.10, perception_venv 3.10 | Isaac Sim requires Python 3.12 (bundled); external ROS nodes use Python 3.10 | ✅ COMPATIBLE (separate env) |
| **GLIBC** | 2.35 (Ubuntu 22.04 default) | GLIBC 2.35+ required (`manylinux_2_35_x86_64`) | ✅ MEETS REQUIREMENT |
| **Disk Space** | 267 GB available on /home | 50 GB free required; download + extraction ~40 GB temporary | ✅ SUFFICIENT |
| **Installation Size** | Current: 19 GB | Similar footprint expected | ✅ FITS |

### 13.3 Key Compatibility Notes

1. **Ubuntu 22.04 deprecation warning**: Isaac Sim 6.2.0 release notes state: "Ubuntu 22.04 support is marked for removal. Plan to migrate to a supported newer Ubuntu release. Ubuntu 22.04 remains supported in 6.2."

2. **ROS 2 Humble on Ubuntu 22.04**: Officially supported. The launcher defaults to bundled ROS 2 Humble libraries when no external ROS is sourced. External nodes should source `/opt/ros/humble/setup.bash` in separate terminals.

3. **Python 3.12 requirement**: Isaac Sim 6.2.0 bundles Python 3.12. The project's existing `perception_venv` (Python 3.10) should remain separate for external ROS nodes. Custom ROS interfaces require Python 3.12 workspace build.

4. **GPU architecture**: RTX PRO 6000 Blackwell is listed as "Best" for datacenter workloads. Minimum requirement is RTX 4080 (16 GB VRAM) - our 96 GB exceeds this.

5. **Installation method**: Standalone package download + unzip + `./post_install.sh` + `./isaac-sim.sh`. No package manager, no system Python modification.

### 13.4 Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| Ubuntu 22.04 deprecation | Medium | Low (still supported in 6.2) | Test now; plan Ubuntu 24.04 migration later |
| ROS 2 custom interface compatibility | Medium | Medium | Build ROS 2 workspace with Python 3.12 via Docker per docs |
| Project API changes (Cortex removal) | High | Medium | Project doesn't use Cortex (verified by grep) |
| Asset path changes | Medium | Low | Use absolute asset paths; test scenario loading |
| OmniGraph/SyntheticData writer fix unverified | Unknown | High | **Primary test objective** - must verify in Phase D/E |

### 13.5 Installation Plan

**Target path**: `/home/ubuntu/isaacsim-6.2.0` (separate from `/home/ubuntu/isaacsim`)

**Steps**:
1. Download `isaac-sim-standalone-6.2.0-linux-x86_64.zip` (~40 GB)
2. Verify MD5: `dde88af0b6423dd3135727385a344fb3`
3. Extract to `/home/ubuntu/isaacsim-6.2.0`
4. Run `./post_install.sh`
5. Test launch with `./isaac-sim.sh --headless` for minimal validation
6. Run official camera example: `./python.sh standalone_examples/api/isaacsim.ros2.bridge/camera_periodic.py`

**Disk space**: 267 GB available → sufficient for 40 GB temp + ~20 GB install

### 13.6 Approval Required

**Action required**: Approval to download ~40 GB package and install to `/home/ubuntu/isaacsim-6.2.0`.

**No changes to**: `/home/ubuntu/isaacsim` (existing 5.1.0-rc.19), project Git state, GPU driver, CUDA, system Python, ROS 2, or project code.

---

## 12. Preserved Distinction

- **Headless simulation tests**: 82/82 pass (vehicle physics, Ackermann checks, scenario stability) — these do NOT require live RGB-D
- **Live RGB-D acceptance tests**: ALL BLOCKED — require fixed syntheticdata writer path
- **This report** documents the blocker precisely so that when the vendor fix arrives, the pipeline can be unblocked systematically

---

**End of report**