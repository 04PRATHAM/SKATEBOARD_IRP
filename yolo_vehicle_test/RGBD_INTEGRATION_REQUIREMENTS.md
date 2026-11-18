# RGB-D Perception & Obstacle-Avoidance Integration Requirements

**Document ID:** RIR-VIRYA-2026-004  
**Project:** RGB-D Multi-Object Perception and Obstacle-Avoidance System  
**Target Vehicle:** VIRYA DBW Electric Vehicle  
**Workspace:** `/home/vat/yolo_vehicle_test/`  
**Status:** Requirements & System Integration Specification  

---

## 1. Executive Intent & Architecture Overview

Our project objective is to upgrade the perception and decision stack of the VIRYA DBW vehicle from a primitive 2D monocular webcam with heuristic column-based steering to a robust, metric 3D perception and intelligent obstacle-avoidance system.

### Target Architecture

```
           +---------------------------------------+
           |       RGB-D SENSOR SUBSYSTEM          |
           |      (e.g., Intel RealSense D435)     |
           +---------------------------------------+
                |                             |
                | Color Stream                | Aligned Depth Stream
                v                             v
       sensor_msgs/Image             sensor_msgs/Image
       (/camera/color/image_raw)     (/camera/aligned_depth_to_color/image_raw)
                |                             |
                v                             |
       +------------------+                   |
       |   YOLO26n-seg    |                   |
       |  (Segmentation)  |                   |
       +------------------+                   |
                |                             |
     2D Masks & Bounding Boxes                |
                |                             |
                +--------------+--------------+
                               |
                               v
               +-------------------------------+
               |    RGB-D FUSION & 3D NODE     |
               | - Depth sampling on seg masks |
               | - Outlier rejection / median  |
               | - Pin-hole 3D reprojection    |
               +-------------------------------+
                               |
                   Metric (X, Y, Z) per Object
                               |
                               v
               +-------------------------------+
               |  PATH & OBSTACLE EVALUATOR    |
               | - Collision risk assessment   |
               | - Safe driving corridor eval  |
               | - Target heading / clearance  |
               +-------------------------------+
                               |
                               v
               +-------------------------------+
               |   DECISION & MOTION PLANNER   |
               | - Smooth steering PID/ramp    |
               | - Speed regulation / braking  |
               | - Autonomy deadman publisher  |
               +-------------------------------+
                |              |              |
                v              v              v
        /dbw/auto/cmd_steer  /dbw/auto/cmd_vel  /dbw/auto/brake
                \              |              /
                 \             |             /
                  v            v            v
               +-------------------------------+
               |      EXISTING DBW STACK       |
               | (dbw_node / can / mod_writer) |
               +-------------------------------+
```

---

## 2. Component Categorization Matrix

| Component | Status | Location / Package | Architectural Role in New System |
|---|---|---|---|
| Linux SocketCAN Setup (`can0`) | **REUSE (PRESERVE)** | System `/bin/ip` via `start_buggy.sh` | Must run prior to vehicle operations at 500 kbps bitrate. |
| DBW Core Launcher (`dbw_launch.bash`) | **REUSE (PRESERVE)** | `/home/vat/dbw_ws_2/dbw_launch.bash` | Launches `dbw_module_cpp dbw_launch.py`. Must remain completely untouched. |
| DBW C++ Supervisor (`dbw_node`) | **REUSE (PRESERVE)** | `dbw_module_cpp` | Manages state machine, speed clamping, and deadman timer. Untouched. |
| CAN Writer (`can_writer_node`) | **REUSE (PRESERVE)** | `dbw_module_cpp` | Translates velocity and brake to CAN ID `0x12280797`. Untouched. |
| Modbus Writer (`mod_writer_node`) | **REUSE (PRESERVE)** | `dbw_module_cpp` | Translates steering angle to Modbus TCP Register 3. Untouched. |
| CAN Reader (`can_reader.py`) | **REUSE (PRESERVE)** | `dbw_module_cpp` | Reads vehicle speed and telemetry from CAN ID `0x12910109`. Untouched. |
| Diagnostics Node (`diagnostics_node`) | **REUSE (PRESERVE)** | `dbw_module_cpp` | Monitors heartbeat timeouts. Untouched. |
| USB Webcam Driver (`v4l2_camera`) | **REPLACE** | `/opt/ros/jazzy` (`v4l2_camera_node`) | Monocular 2D webcam cannot provide depth. Replace with RGB-D camera driver. |
| Legacy YOLO Avoidance Node | **REPLACE** | `my_package/yolo_obstacle_avoidance_node` | Inadequate 3-zone heuristic with no distance awareness. To be completely replaced. |
| Deadman Terminal Publisher | **REPLACE** | `start_buggy.sh` line 50 (`ros2 topic pub`) | Replace CLI publisher with supervised deadman publisher tied to node health. |

---

## 3. Identification of Missing Functionality

The existing vehicle codebase lacks several critical capabilities required for reliable autonomous obstacle avoidance:

1. **Absence of Metric Depth Stream:** The current vehicle setup utilizes a monocular Logitech C270 USB webcam on `/image_raw`. There is no hardware depth sensor actively publishing in the launch workflow.
2. **Absence of Camera Intrinsics & Extrinsics (TF Tree):**
   - No camera-to-baselink transform (`tf2`) exists.
   - `/camera_info` is unpopulated/uncalibrated, preventing metric pinhole reprojection $(u,v,d) \to (X,Y,Z)$.
3. **Absence of 3D Object Localization:** The current YOLO node operates purely in pixel space $(x_{\text{pixel}}, y_{\text{pixel}})$, with zero awareness of real-world distance in meters.
4. **Absence of Instance Segmentation:** Current YOLOv8n only generates coarse axis-aligned 2D bounding boxes. Bounding boxes include substantial background noise, which corrupts depth averaging. Segmentation masks are required to isolate object pixels cleanly.
5. **Absence of Multi-Object Tracking & Classification:** No persistent tracking across frames exists; sudden detection dropouts cause control chatter.
6. **Absence of Slew-Rate Limited Steering Control:** The legacy node publishes step changes (`10.0°` or `-10.0°`), which causes actuator stalling or jerky responses given the physical actuator's 30°/s slew limit.
7. **Absence of Sensor Timeout Safety Interlock:** In the legacy node, if camera frames freeze, `image_callback` stops firing, but the vehicle will keep moving forward at 1.0 m/s until the 150 ms command staleness watchdog triggers.

---

## 4. Hardware & Sensor Requirements (RGB-D)

To realize metric distance estimation, an RGB-D sensor (e.g. Intel RealSense D435) must be utilized:
- **Driver Package:** `realsense2_camera` (verified installed in system ROS 2 Jazzy).
- **Required Streams:**
  - Color stream: `sensor_msgs/msg/Image` (e.g. `/camera/camera/color/image_raw`), format `RGB8` or `BGR8`, $640 \times 480$ at $\ge 15$–$30$ FPS.
  - Aligned Depth stream: `sensor_msgs/msg/Image` (e.g. `/camera/camera/aligned_depth_to_color/image_raw`), format `16UC1` (depth in millimeters), aligned pixel-to-pixel with the color image.
  - Camera Intrinsics: `sensor_msgs/msg/CameraInfo` (e.g. `/camera/camera/color/camera_info`), supplying focal lengths $(f_x, f_y)$ and principal point $(c_x, c_y)$.
- **Mounting & Extrinsic Calibration:**
  - A static transform (`tf2`) from `base_link` (rear axle center / vehicle reference) to `camera_link` is mandatory to translate camera coordinates $(X_c, Y_c, Z_c)$ into vehicle body coordinates $(X_v, Y_v, Z_v)$.

---

## 5. Perception Pipeline Requirements (YOLO26n-seg)

- **Model Selection:** `yolo26n-seg.pt` (weights already located in `/home/vat/yolo_vehicle_test/`).
  - *Benchmarking Data:* On the vehicle's CPU-only host environment, the Nano segmentation model (`yolo26n-seg`) provides optimal latency vs accuracy balance compared to larger variants.
- **Inference Requirements:**
  - Input: $640 \times 480$ RGB image.
  - Device: `"cpu"` with multi-threading optimization.
  - Output: Detection classes, confidence scores, 2D bounding boxes, and **binary polygon masks** for segmented object instances.
- **Relevant Target Classes:**
  - Primary obstacles: `person`, `car`, `truck`, `bus`, `motorcycle`, `bicycle`.
  - Secondary path obstacles: cones, barriers, boxes (custom trained or mapped via general object classes).

---

## 6. RGB-D Fusion & Spatial Position Estimation

For each segmented object $k$:
1. **Mask Extraction:** Extract binary mask $M_k(u,v) \in \{0, 1\}$ from YOLO26n-seg.
2. **Depth Sampling:** Crop the aligned depth image $D(u,v)$ across the mask pixels where $M_k(u,v) = 1$.
3. **Outlier Filtering:** Reject zero-depth pixels (sensor noise/occlusion) and filter values using statistical median or trimmed mean:
   $$d_k = \text{median}\Big(\big\{ D(u, v) \;\big|\; M_k(u, v) = 1 \text{ and } D_{\min} \le D(u, v) \le D_{\max} \big\}\Big)$$
4. **3D Metric Reprojection:** Using camera intrinsics $(f_x, f_y, c_x, c_y)$ and mask centroid $(u_c, v_c)$:
   $$Z_c = d_k$$
   $$X_c = \frac{(u_c - c_x) \times Z_c}{f_x}$$
   $$Y_c = \frac{(v_c - c_y) \times Z_c}{f_y}$$
5. **Vehicle Frame Transform:** Transform $[X_c, Y_c, Z_c]^T$ into vehicle coordinates $[X_v, Y_v, Z_v]^T$ via mounting extrinsics.

---

## 7. Intelligent Decision-Making & Motion Planning

Rather than naive 3-zone screen splitting, the decision logic must evaluate obstacles against the vehicle's geometric footprint and projected dynamic swept path:

### 7.1 Zone Assessment
- **Vehicle Clearance Corridor:** Vehicle width $W_v \approx 1.2\text{ m}$ plus lateral safety margin $\delta_y \approx 0.4\text{ m}$ ($|Y_v| \le 1.0\text{ m}$).
- **Stopping Zone ($Z_v < D_{\text{stop}}$):** If any obstacle is within the clearance corridor and distance $Z_v \le D_{\text{stop}}$ (e.g. $2.5\text{ m}$):
  - Command immediate braking: `/dbw/auto/brake` $\to \text{True}$.
  - Command zero velocity: `/dbw/auto/cmd_vel` $\to 0.0\text{ m/s}$.
  - Maintain centered steering: `/dbw/auto/cmd_steer` $\to 0.0^\circ$.
- **Avoidance Zone ($D_{\text{stop}} < Z_v \le D_{\text{avoid}}$):** If an obstacle is detected in the path at intermediate distance (e.g. $2.5\text{ m} < Z_v \le 6.0\text{ m}$):
  - Compute lateral offset required to clear obstacle.
  - Determine optimal steering angle $\theta_{\text{steer}}$ using bicycle model kinematics or potential field repulsion.
  - Modulate speed: reduce velocity smoothly (e.g. to $0.5$–$0.8\text{ m/s}$).
  - Slew-rate limit steering angle:
    $$\Delta \theta \le \theta_{\text{max\_rate}} \times \Delta t \quad (\le 30^\circ/\text{s})$$
- **Clear Zone ($Z_v > D_{\text{avoid}}$ or corridor unobstructed):**
  - Release brake: `/dbw/auto/brake` $\to \text{False}$.
  - Walk steering back to $0.0^\circ$.
  - Cruise at nominal forward speed (e.g. $1.0$–$1.5\text{ m/s}$).

---

## 8. Safety & Fail-Safe Requirements

1. **Synchronized Command Generation:**
   Both `/dbw/auto/cmd_vel` and `/dbw/auto/cmd_steer` must be published simultaneously on a steady periodic timer ($20$–$50\text{ Hz}$) to prevent the 120 ms steering staleness timeout in `DBWNode`.
2. **Perception Watchdog & Sensor Loss:**
   If RGB or Depth camera frames drop for $> 200$ ms, the decision node must automatically enter emergency stop mode (command $v = 0.0$, brake = True).
3. **Heartbeat & Deadman Integration:**
   The autonomy stack must maintain `/dbw/deadman_switch` with `"{data: Auto}"` at $20\text{ Hz}$ only while all self-checks pass. On any internal failure or unhandled exception, deadman publishing must cease immediately, allowing the vehicle's 200 ms hardware watchdog to halt the buggy.

---

## 9. Recommended Implementation & Staging Plan

```
Stage 1: Offline Sensor & Pipeline Validation (Dry Testing)
  - Launch RealSense D435 node; verify RGB + aligned depth sync.
  - Develop new ROS 2 perception node in /home/vat/yolo_vehicle_test/.
  - Verify YOLO26n-seg inference and metric 3D point estimation on recorded / live video.
  - ZERO vehicle commands published during this stage.

Stage 2: Software-in-the-Loop & Visualization
  - Publish RViz2 visualization markers for 3D detected obstacles and projected vehicle corridor.
  - Verify obstacle distance accuracy against ground-truth tape measurements.
  - Log planner decisions (cmd_vel, cmd_steer, brake) to file/console without connecting to DBW.

Stage 3: Static Vehicle Actuator Verification
  - Vehicle safely jacked up on stands / wheels off the ground.
  - Bring up DBW stack via dbw_launch.bash.
  - Enable deadman and verify steering response and rear wheel rotation under controlled obstacle stimuli.

Stage 4: Low-Speed Proving Ground Track Test
  - Enclosed test track with safety operator seated at physical emergency brake.
  - Test stationary obstacle avoidance at 0.5 m/s.
  - Test pedestrian crossing scenario and measure stopping margin.
```
