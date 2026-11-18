# Complete Technical Master Report: Perception to Vehicle Control Integration
## VIRYA Drive-By-Wire (DBW) Electric Buggy Autonomy Platform

**Document Reference:** VIRYA-DBW-CTRL-INT-2026-001  
**Project:** RGB-D Multi-Object Obstacle Detection and Avoidance for VIRYA Buggy  
**Classification:** Authoritative Technical Specification & Integration Verification  
**Author:** Senior Robotics Software Engineer (Autonomy & DBW Systems)  
**Workspace:** `/home/vat/yolo_vehicle_test/`  
**Date of Verification:** November 2026  
**Status:** VALIDATED — 100% OFFLINE PASS (18/18 Automated Integration Scenarios)

---

## Executive Summary

This report establishes the complete, authoritative bridge between our new **YOLO26n-seg Multi-Object Perception & Decision Pipeline** and the **VIRYA Drive-By-Wire (DBW) vehicle architecture**. 

Prior to this phase, the perception dashboard (`perception_gui.py`) successfully identified, segmented, and tracked four target classes (**Person**, **Bench**, **Chair**, **Dining Table**), accurately evaluated driving corridor intrusions, and computed avoidance decisions (`MOVE_RIGHT`, `MOVE_LEFT`, `SLOW_DOWN`, `STOP`, `PATH_CLEAR`). However, the physical vehicle did not actuate steering, traction, or braking.

This investigation conducted a line-by-line audit of the existing vehicle control architecture, identified the root causes of non-actuation, engineered a high-reliability rate-decoupling vehicle control adapter (`vehicle_control_adapter.py`), updated the live perception GUI with end-to-end status telemetry, implemented an 18-scenario automated integration test suite (`test_control_integration.py`), and created a project-local operational launcher (`start_multi_object_buggy.sh`).

All newly developed software and documentation reside strictly inside `/home/vat/yolo_vehicle_test/`. In strict compliance with safety directives, zero physical actuation commands were transmitted to the real vehicle hardware during this development stage; all verification was completed in offline simulation and mock mode.

---

## 1. Original Vehicle Control Architecture Overview

The VIRYA Drive-By-Wire platform is an industrial electric buggy retrofitted for autonomous control. It features an asymmetric, dual-bus control architecture:

```
                            +-----------------------------------+
                            |       Vehicle On-Board PC         |
                            |   ROS 2 Jazzy Autonomy Stack      |
                            +-----------------+-----------------+
                                              |
                   +--------------------------+--------------------------+
                   |                                                     |
        Modbus TCP (Ethernet)                                SocketCAN (can0)
        IP: 192.168.0.5:502                                   Bitrate: 500 kbps
        Target: Industrial PLC                                Target: CCU
                   |                                                     |
                   v                                                     v
      +------------------------+                             +------------------------+
      | Front-Wheel Steering   |                             | Rear Traction & Brakes |
      | Electric Servo / Motor |                             | Dual Hub Motors / CCU  |
      +------------------------+                             +------------------------+
```

### 1.1 Architectural Separation
1. **Front-Wheel Steering Subsystem:**
   - Managed by an **Industrial PLC** accessed via **Modbus TCP** over Ethernet (`192.168.0.5:502`, Slave ID `0x0001`).
   - Steering does **not** communicate over CAN bus.
2. **Rear-Wheel Traction and Acceleration Subsystem:**
   - Managed by the vehicle **Central Control Unit (CCU)** accessed via **SocketCAN** (`can0`, 500 kbps, Classic CAN 2.0B Extended 29-bit frames).
   - CCU directly regulates motor controllers driving the rear axle.
3. **Braking Subsystem:**
   - Operates a **dual-path safety architecture**:
     - Primary: CAN frame Byte 3 transmitted to the CCU (`0x64` = 100% full service brake).
     - Secondary: Modbus Coil 4 asserted to the PLC (`Utils::CoilId::BRAKE`).
4. **Supervisory State Machine & Safety Watchdogs:**
   - Centralized in `dbw_node.cpp` (DBW Supervisor Node).
   - Enforces deadman heartbeats (`/dbw/deadman_switch` at 20 Hz), speed freshness (<150 ms), and steer-speed coupling (<120 ms).

---

## 2. Original Vehicle Startup Sequence Audit (`start_buggy.sh`)

The authoritative vehicle startup procedure is recorded in `/home/vat/yolo_vehicle_test/start_buggy.sh`. A systematic step-by-step breakdown:

```bash
#!/bin/bash
# [0/5] Setting up CAN Interface...
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
sleep 2

# [1/5] Starting CAN Monitor...
gnome-terminal -- bash -c "candump can0; exec bash"
sleep 2

# [2/5] Launching DBW Stack...
gnome-terminal -- bash -c "
source /opt/ros/jazzy/setup.bash
cd ~/dbw_ws_2
source install/setup.bash
bash dbw_launch.bash
exec bash"
sleep 10

# [3/5] Starting Camera Node...
gnome-terminal -- bash -c "
source /opt/ros/jazzy/setup.bash
ros2 run v4l2_camera v4l2_camera_node
exec bash"
sleep 5

# [4/5] Publishing Deadman Switch...
gnome-terminal -- bash -c "
source /opt/ros/jazzy/setup.bash
ros2 topic pub -r 20 /dbw/deadman_switch std_msgs/msg/String '{data: Auto}'
exec bash"
sleep 3

# [5/5] Starting YOLO Obstacle Avoidance...
gnome-terminal -- bash -c "
source /opt/ros/jazzy/setup.bash
cd ~/dbw_ws_2
source install/setup.bash
ros2 run my_package yolo_obstacle_avoidance_node
exec bash"
```

### Analysis of the Startup Pipeline:
- **Step 0 & 1:** SocketCAN interface `can0` is brought down, configured to **500,000 bit/s**, brought up, and monitored via `candump can0`.
- **Step 2:** Sourced ROS 2 Jazzy and launched `dbw_launch.bash`, which starts:
  - `dbw_node`: Supervisory FSM and control multiplexer.
  - `can_writer_node`: SocketCAN serializer (publishes to `can0`).
  - `mod_writer_node`: Modbus TCP serializer (communicates with `192.168.0.5:502`).
  - `can_reader.py`: CAN frame deserializer (parses `0x12910109`).
  - `diagnostics_node`: System health aggregator.
- **Step 3:** Hardware video capture launched using ROS 2 driver `v4l2_camera_node`, broadcasting on `/image_raw`.
- **Step 4:** A standalone terminal continuously publishes a deadman heartbeat at **20 Hz** (`ros2 topic pub -r 20 /dbw/deadman_switch std_msgs/msg/String '{data: Auto}'`).
- **Step 5:** Legacy obstacle avoidance node `yolo_obstacle_avoidance_node` launched. This legacy node subscribed to `/image_raw`, performed single-class person detection, and published to `/dbw/auto/cmd_steer` and `/dbw/auto/cmd_vel`.

---

## 3. Front-Wheel Steering Architecture & Protocols

### 3.1 Actuator & Hardware Interface
The steering actuator is an electric servo controlled by an industrial PLC.
- **Network Interface:** Modbus TCP over Ethernet
- **PLC IP Address:** `192.168.0.5`
- **TCP Port:** `502`
- **Slave ID:** `0x0001` (Holding Registers)

### 3.2 Steering Angle Representation & Sign Convention
Authoritative source code evidence (`yolo_obstacle_avoidance_node.py` lines 87–101; `dbw_teleop.py` lines 133–137; `mod_writer.cpp` lines 117–126):
- **Units:** Physical degrees of steering angle ($^\circ$).
- **Neutral Steering:** `0.0°` (wheels centered).
- **Left Turn Convention:** Positive values ($+$, $> 0.0^\circ$) command front wheels to turn **LEFT**.
  $$\text{Target} = +10.0^\circ \implies \text{Turn LEFT}$$
- **Right Turn Convention:** Negative values ($-$, $< 0.0^\circ$) command front wheels to turn **RIGHT**.
  $$\text{Target} = -10.0^\circ \implies \text{Turn RIGHT}$$
- **Software Clamp:** Defined in `dbw.cpp` line 3:
  $$\text{steer\_limit\_} = \pm 30.0^\circ$$

### 3.3 Modbus Register Encoding
In `/home/vat/DBW/src/dbw_module/dbw_module_cpp/src/mod_writer/mod_writer.cpp`:
1. Commanded angle is scaled by a factor of 10 to obtain integer tenths of a degree:
   $$\text{steer\_angle\_int} = \text{static\_cast<int16\_t>}(\text{angle} \times 10)$$
2. If angle $\ge 0$:
   Written directly to **Register 3** (`Utils::RegisterId::STEER_WRITE_REGISTER`).
   $$\text{Example: } +10.0^\circ \implies 100 \implies \text{Register 3} = \text{0x0064}$$
3. If angle $< 0$:
   Encoded into 16-bit **Two's Complement** representation:
   $$\text{twosComplement} = (1 \ll 16) + \text{steer\_angle\_int}$$
   $$\text{Example: } -10.0^\circ \implies -100 \implies 65536 - 100 = 65436 \implies \text{Register 3} = \text{0xFFA0}$$

### 3.4 Steering Feedback
- `ModWriter` reads **Register 0** (`STEER_READ_REGISTER`) every 50 ms.
- Scaled back to floating-point degrees:
  $$\text{current\_steer\_degrees} = \frac{\text{static\_cast<int16\_t>}(\text{register0})}{10.0}$$
- Published to `/dbw/feedback/current_steer` (`std_msgs/msg/Float64`).

### 3.5 Actuator Slew Rate Limitation
As documented in `teleop_remap.cpp` (lines 157–160):
> *"The published steering angle is slew-limited to steer_rate deg/s rather than stepped to the target, because the steering actuator will not accept a jump - it only tracks a command that walks there."*
- **Slew Rate Parameter:** `steer_rate = 30.0` deg/s (`config/dbw_config.yaml`).
- Slew rate filtering must be enforced before transmitting commands to prevent actuator fault or mechanical rejection.

---

## 4. Rear-Wheel Traction & Acceleration Architecture

### 4.1 Actuator & Hardware Interface
The rear axle is driven by dual electric hub/traction motors commanded by the vehicle CCU over CAN bus:
- **Interface:** SocketCAN `can0`
- **Bitrate:** 500,000 bps (500 kbps)
- **Frame Format:** Classic CAN 2.0B Extended (29-bit identifier)
- **Transmit CAN ID:** `0x12280797`
- **Telemetry CAN ID:** `0x12910109`

### 4.2 Velocity Representation & Scaling
- **ROS 2 Topic:** `/dbw/auto/cmd_vel` (`std_msgs/msg/Float64`).
- **Units:** Linear velocity in meters per second ($\text{m/s}$).
- **Range & Clamping:** `velocity_limit_ = 2.5` m/s (`dbw.cpp` line 4). Clamped to `[-2.5, +2.5]` m/s.
- **Speed Magnitude Encoding:** In `dbw.cpp` line 377 and `can_writer.cpp` lines 71–78:
  $$\text{speed\_c\_value} = \text{round}(|\text{velocity}| \times 100) \quad (\text{cm/s})$$

### 4.3 Transmit CAN Frame Layout (CAN ID: `0x12280797`)

| Byte Index | Field Name | Data Type | Encoding / Values | Description |
|---|---|---|---|---|
| **Byte 0** | Drive Mode | `uint8_t` | `0x01`: Forward ($v > 0$)<br>`0x03`: Reverse ($v < 0$)<br>`0x00`: Neutral ($v = 0$) | Commands CCU drive direction |
| **Byte 1** | Velocity LSB | `uint8_t` | `speed_c_value & 0xFF` | Commanded speed low byte (cm/s) |
| **Byte 2** | Velocity MSB | `uint8_t` | `(speed_c_value >> 8) & 0xFF` | Commanded speed high byte (cm/s) |
| **Byte 3** | Service Brake | `uint8_t` | `0x64`: 100% Brake<br>`0x00`: Brake Released | Physical service brake actuator |
| **Byte 4** | Park Brake | `uint8_t` | `0x01` on E-Stop; `0x00` normal | Electric parking brake |
| **Byte 5** | Reserved | `uint8_t` | `0x00` | Unused |
| **Byte 6** | Reserved | `uint8_t` | `0x00` | Unused |
| **Byte 7** | CCU Heartbeat | `uint8_t` | `0x01` | Active heartbeat marker |

### 4.4 Ground Speed Telemetry
The CCU broadcasts vehicle telemetry on CAN ID `0x12910109`.
- **Bytes 6 & 7:** Signed 16-bit integer speed in cm/s.
  $$\text{ground\_speed} = \frac{(\text{Byte 7} \ll 8) \mid \text{Byte 6}}{100.0} \quad (\text{m/s})$$
- Published to `/dbw/feedback/vehicle_speed` (`std_msgs/msg/Float64`).

---

## 5. Braking System Architecture & Automatic Interlocks

### 5.1 Braking Command Interface
- **ROS 2 Topic:** `/dbw/auto/brake` (`std_msgs/msg/Bool`).
- `data: true`: Command service brake at 100% pressure.
- `data: false`: Release service brake.

### 5.2 Automatic Brake Interlocks in `dbw_node.cpp`
Authoritative logic in `DBWNode::send_can_message()` (`dbw.cpp` lines 373–376):
```cpp
int brake = coils_.brake;
if (velocity_ == 0 || emergency_stop_){
    brake = 1; // Set brake to 1 if velocity is 0
}
```
**Mandatory Automatic Braking Triggers:**
1. **Zero Velocity Commanded:** Whenever `velocity_ == 0.0`, the service brake is automatically asserted (`brake = 1`) to prevent roll-away.
2. **Speed Command Staleness:** If `/dbw/auto/cmd_vel` is not published for $> 150$ ms, `velocity_` drops to `0.0`, immediately engaging full braking.
3. **Deadman Release:** If `/dbw/deadman_switch` drops for $> 200$ ms, FSM drops to `IDLE`, velocity is zeroed, and brake engages.
4. **Shutdown Signal:** Receiving `true` on `/dbw/shutdown` forces `brake = 1` and `velocity_ = 0.0`.
5. **Hardware E-Stop:** CAN ID `0x12910109` Byte 4 Bit 4 asserted sets `emergency_stop_ = true`.

---

## 6. Deadman Switch, Watchdogs, and DBW FSM

### 6.1 Deadman Switch Protocol
- **ROS 2 Topic:** `/dbw/deadman_switch` (`std_msgs/msg/String`).
- **Required Payload:** String literal `"{data: Auto}"`.
- **Publishing Rate:** Recommended **20 Hz** (50 ms interval).
- **Timeout Threshold:** Defined in `utils.hpp` line 11:
  $$\text{Utils::DEADMAN\_TIMEOUT} = 200\text{ ms}$$

### 6.2 Critical Steer-Speed Coupling Watchdog
In `dbw.cpp` lines 395–426:
```cpp
auto topic_staleness = std::chrono::duration_cast<std::chrono::milliseconds>(curr_time - speed_receive_time_).count();
if (topic_staleness <= 120){
    mod_message_->steer_angle = is_shutdown_ ? 0 : steer_angle_;
} else {
    mod_message_->steer_angle = 0; // Forced to center if speed is stale!
}
```
> [!WARNING]
> **CRITICAL CONTROL DEPENDENCY:**  
> The front-wheel steering actuator relies on a live velocity heartbeat. If `/dbw/auto/cmd_vel` is not published for $> 120$ ms, the steering angle will **snap to 0 degrees** even if steering commands are arriving fresh on `/dbw/auto/cmd_steer`. Both must be published in tandem at $\ge 20$ Hz.

---

## 7. ROS 2 Interface Matrix & Topic Definitions

| Topic Name | Message Type | Rate (Hz) | Purpose / Semantics | DBW Target |
|---|---|---|---|---|
| `/dbw/deadman_switch` | `std_msgs/msg/String` | 20 Hz | Autonomous enable heartbeat (`"Auto"`) | `dbw_node` (FSM guard) |
| `/dbw/auto/cmd_steer` | `std_msgs/msg/Float64` | 20 Hz | Front wheel steering angle (deg) | `mod_writer_node` (PLC Reg 3) |
| `/dbw/auto/cmd_vel` | `std_msgs/msg/Float64` | 20 Hz | Vehicle linear velocity (m/s) | `can_writer_node` (CAN 0x12280797) |
| `/dbw/auto/brake` | `std_msgs/msg/Bool` | 20 Hz | Service brake actuation request | `can_writer_node` & `mod_writer_node` |
| `/dbw/shutdown` | `std_msgs/msg/Bool` | Event | System emergency shutdown | `dbw_node` (Forces FAULT & Brake 1) |
| `/dbw/feedback/current_steer` | `std_msgs/msg/Float64` | 20 Hz | Steering encoder actual angle (deg) | Feedback from PLC Reg 0 |
| `/dbw/feedback/vehicle_speed` | `std_msgs/msg/Float64` | 20 Hz | CCU measured ground speed (m/s) | Feedback from CAN 0x12910109 |

---

## 8. Root Cause Analysis: Why the Vehicle Did Not Actuate

An exhaustive trace identified five distinct reasons why the previous perception GUI decisions did not actuate the physical vehicle:

1. **Omission of ROS 2 Topic Publishers:**
   - Both `perception_gui.py` and `run_perception_pipeline.py` were implemented as standalone Python computer-vision processes.
   - They computed `DecisionResult(decision=NavigationDecision.MOVE_RIGHT, suggested_steer_dir='RIGHT', ...)` and logged decisions to JSONL, but **contained zero ROS 2 node initializations, zero topic publishers, and zero SocketCAN or Modbus sockets**.
2. **Intentional Offline Safety Gate:**
   - In accordance with early-phase safety directives, `perception_gui.py` explicitly displayed:
     - `OFFLINE SAFETY MODE: ACTUATION DISCONNECTED (NO PHYSICAL COMMANDS)`
     - `ACTUAL BRAKE STATUS: NOT CONNECTED (OFFLINE DEMO MODE)`
   - The application was intentionally isolated from vehicle actuators to prevent unexpected motion during vision tuning.
3. **Absence of Deadman Heartbeat (`/dbw/deadman_switch`):**
   - The vehicle DBW node (`dbw_node.cpp`) strictly gates all actuation commands with `fsm_.get_state() == fsm_state::AUTOMODE`.
   - Without an active deadman publisher streaming `"{data: Auto}"` at $\ge 10$ Hz, the vehicle remained trapped in `IDLE` state. In `IDLE`, all actuator setpoints are forced to neutral (`vel = 0`, `steer = 0`, `brake = 1`).
4. **Rate Mismatch and Watchdog Expiration:**
   - On CPU inference, `YOLO26n-seg` operates at approximately **10.5–11.0 FPS** (90–110 ms latency with occasional frame jitter up to 160 ms).
   - The vehicle DBW node enforces three strict watchdogs:
     - Deadman watchdog: 200 ms timeout.
     - Velocity watchdog: 150 ms timeout.
     - Steer-speed coupling watchdog: 120 ms timeout.
   - Even if the vision pipeline published on each processed frame (~100 ms), occasional jitter (>120 ms) would immediately trigger watchdog bite, snapping steering to $0^\circ$ and clamping brakes.
5. **Actuator Slew-Rate Rejection:**
   - The vision pipeline makes discrete state decisions (e.g., transitioning instantly from `PATH_CLEAR` ($0^\circ$) to `MOVE_RIGHT` ($-10.0^\circ$)).
   - As documented in `teleop_remap.cpp`, the physical steering servo cannot accept instantaneous step commands; it requires continuous smooth ramping at $\le 30.0^\circ/\text{s}$.

---

## 9. Perception-to-Control Data Flow Architecture

The integrated perception-to-control architecture resolves all five root causes through a decoupled, multi-threaded design:

```
[ Logitech C270 Webcam / Video Feed ]
                  |
                  v
[ YOLO26n-seg Neural Inference (10.9 FPS) ]  ---> Filtered: Person (0), Bench (13), Chair (56), Table (60)
                  |
                  v
[ ByteTrack Multi-Object Association ]      ---> Persistent Track IDs, Velocity Vectors
                  |
                  v
[ Spatial Driving Corridor Analysis ]       ---> Overlap Ratio in [0.25, 0.75] Width & Clearance
                  |
                  v
[ Obstacle Decision Engine ]                 ---> Decisions: STOP, MOVE_LEFT, MOVE_RIGHT, SLOW_DOWN, CLEAR
                  |
                  v  (Thread-Safe Queue / Event Dispatch)
+=================================================================================================+
|                              VehicleControlAdapter (20 Hz Periodic Loop)                        |
|                                                                                                 |
|   1. Perception Staleness Watchdog (< 300 ms)                                                   |
|   2. Setpoint Translator:                                                                       |
|      - MOVE_LEFT  -> Steer Target: +10.0 deg | Speed: 0.5 m/s | Brake: 0.0                       |
|      - MOVE_RIGHT -> Steer Target: -10.0 deg | Speed: 0.5 m/s | Brake: 0.0                       |
|      - STOP       -> Steer Target:   0.0 deg | Speed: 0.0 m/s | Brake: 1.0                       |
|   3. Steering Slew Rate Limiter (<= 30.0 deg/s)                                                 |
|   4. Acceleration Ramp Limiter (<= 1.5 m/s^2)                                                   |
|   5. Automatic Brake Interlock (Brake = 1 if velocity == 0)                                     |
|   6. Deadman Heartbeat Generator (20 Hz, {data: Auto})                                          |
|   7. Hardware Serializer / Offline Mock Engine:                                                 |
|      - Modbus TCP: Register 3 (Scaled x10, 16-bit Two's Comp for negative)                      |
|      - SocketCAN can0: Transmit ID 0x12280797 (Mode, Speed cm/s, Brake 0x64)                    |
+=================================================================================================+
                  |
                  +-----------------------------------+-----------------------------------+
                  |                                                                       |
                  v                                                                       v
   [ Live Perception GUI Dashboard ]                                    [ DBW Actuation Interface ]
   - Commanded Slew-Limited Steering Angle                              - Mode: MODE_OFFLINE_MOCK (Simulated)
   - Real-time Subsystem Status Tags:                                   - Mode: MODE_ACTUATION_GATE (Locked)
     [REQUESTED], [ACCEPTED], [EXECUTED]
   - Modbus & CAN Bus Encodings
```

---

## 10. Vehicle Control Adapter Implementation (`vehicle_control_adapter.py`)

The control adapter is implemented in `/home/vat/yolo_vehicle_test/vehicle_control_adapter.py`. Key architectural components:

### 10.1 Operational Modes
- `MODE_OFFLINE_MOCK` (Default): 100% simulated DBW supervisor, simulated Modbus registers, simulated SocketCAN frames, and realistic actuator response lag ($\tau_{\text{steer}} = 0.08\text{ s}$, $\tau_{\text{vel}} = 0.15\text{ s}$). Zero physical ROS 2/CAN transmission.
- `MODE_MONITOR`: Passive listener to vehicle feedback topics without publishing setpoints.
- `MODE_ACTUATION_GATE`: Staging mode for physical hardware actuation, strictly locked behind a passkey check (`ACTUATE_VIRYA_CONFIRM`).

### 10.2 Periodic Rate Decoupling Loop (20 Hz)
The adapter runs an internal background thread at **20 Hz** ($50\text{ ms}$ tick). This decouples the ~10 FPS vision pipeline from the DBW watchdogs:
- Even if vision frames jitter or arrive at 100 ms intervals, the adapter continuously publishes heartbeats and smoothly steps the actuators.
- In each 50 ms tick ($\Delta t = 0.05\text{ s}$), steering steps by at most:
  $$\Delta \delta = 30.0^\circ/\text{s} \times 0.05\text{ s} = 1.5^\circ$$
- An instantaneous step from $0^\circ$ to $+10.0^\circ$ ramps smoothly over 7 ticks ($\sim 0.33\text{ s}$), preventing servo faults.

### 10.3 Automatic Watchdogs & Fail-Safes
- **Perception Staleness Watchdog:** If no fresh decision is received within **300 ms**, the adapter immediately forces velocity to $0.0\text{ m/s}$, asserts full service brake, and centers steering.
- **Camera Fault Handling:** If camera frame acquisition fails, the adapter immediately drops into safe neutral stop.
- **Deadman Inactive Handling:** If deadman heartbeat is suspended, FSM transitions from `AUTOMODE` to `IDLE`, locking mechanical brakes.
- **Emergency Stop:** Triggering e-stop transitions FSM to `FAULT`, clamps service brake (`0x64`), and engages electric parking brake (`0x01` in CAN Byte 4).

---

## 11. Authoritative Control Mapping Rules & Encodings

| Perception Decision | Target Steer ($^\circ$) | Filtered Slew Rate | Target Speed ($\text{m/s}$) | Brake Request | Modbus Reg 3 (Hex) | CAN Tx ID `0x12280797` (Bytes 0..3) | Operational Behavior |
|---|---|---|---|---|---|---|---|
| **STOP** | $0.0^\circ$ | Centered | $0.0\text{ m/s}$ | `True` (Assert) | `0x0000` | `[0x00, 0x00, 0x00, 0x64]` | Immediate full service brake; drive neutral |
| **MOVE_LEFT** | $+10.0^\circ$ | $+30^\circ/\text{s}$ ramp | $0.5\text{ m/s}$ | `False` (Release) | `0x0064` ($+100$) | `[0x01, 0x32, 0x00, 0x00]` | Steer wheels Left; caution pace (50 cm/s) |
| **MOVE_RIGHT** | $-10.0^\circ$ | $-30^\circ/\text{s}$ ramp | $0.5\text{ m/s}$ | `False` (Release) | `0xFFA0` ($65436$) | `[0x01, 0x32, 0x00, 0x00]` | Steer wheels Right; caution pace (50 cm/s) |
| **SLOW_DOWN** | $0.0^\circ$ | Centered | $0.5\text{ m/s}$ | `False` (Release) | `0x0000` | `[0x01, 0x32, 0x00, 0x00]` | Maintain straight path; caution pace (50 cm/s) |
| **PATH_CLEAR** | $0.0^\circ$ | Centered | $1.0\text{ m/s}$ | `False` (Release) | `0x0000` | `[0x01, 0x64, 0x00, 0x00]` | Maintain straight path; nominal speed (100 cm/s) |
| **UNKNOWN** | $0.0^\circ$ | Centered | $0.0\text{ m/s}$ | `True` (Assert) | `0x0000` | `[0x00, 0x00, 0x00, 0x64]` | Safe neutral stop; full service brake |

---

## 12. Live Perception GUI Integration (`perception_gui.py`)

The live perception dashboard was updated to incorporate real-time vehicle control telemetry directly alongside computer-vision overlays:

1. **Integrated Worker Pipeline:** `PerceptionWorker` instantiates and manages `VehicleControlAdapter`, continuously piping navigation decisions into the control adapter and extracting atomic telemetry packets (`control_telemetry`).
2. **Visual Steering Slew Representation (Section 3B):**
   - The steering panel displays both the discrete target angle and the real-time slew-limited angle:
     `STEER RIGHT (-10.0 deg | Slew: -8.5 deg)`
3. **Dedicated DBW Control & Hardware Feedback Monitor (Section 3C):**
   - Replaced static placeholder text with a comprehensive 6-row real-time control status display:
     - **Steering Channel:** `Req: -10.0 deg | Cmd: -10.0 deg | FB: -9.8 deg [ACCEPTED]`
     - **Speed Channel:** `Req: 0.5 m/s | Cmd: 0.5 m/s | FB: 0.48 m/s [ACCEPTED]`
     - **Brake Channel:** `Req: RELEASED | Cmd: RELEASED | FB: RELEASED [ACCEPTED]`
     - **FSM & Deadman:** `FSM: AUTOMODE (SIM) | DEADMAN: 20 Hz ACTIVE`
     - **Modbus TCP:** `Reg3=0xFFA0 (Cmd) | Reg0=0xFFA2 (FB)`
     - **Safety Banner:** `STATUS: OFFLINE MOCK MODE - ZERO PHYSICAL ACTUATION TRANSMITTED`
4. **Visual Verification Snapshot:**
   - Captured and archived at: `/home/vat/yolo_vehicle_test/results/evaluation/integrated_control_gui_snapshot.jpg` (246 KB).

---

## 13. Automated Integration Test Suite Results (`test_control_integration.py`)

A comprehensive 18-scenario automated integration test suite was developed and executed. The results are permanently logged in `/home/vat/yolo_vehicle_test/results/evaluation/control_integration_test_report.json`.

**Overall Success Rate: 100.0% (18/18 Scenarios Passed)**

```
================================================================================
🧪 RUNNING VIRYA DBW PERCEPTION-TO-CONTROL INTEGRATION TEST SUITE (18 SCENARIOS)
   Mode: Strictly Offline & Non-Actuating (MODE_OFFLINE_MOCK)
================================================================================
[01/18] ✅ PASS - STOP Decision Setpoint Mapping
[02/18] ✅ PASS - MOVE_LEFT Setpoint Mapping (+10.0 deg Left, 0.5 m/s)
[03/18] ✅ PASS - MOVE_RIGHT Setpoint Mapping (-10.0 deg Right, 0.5 m/s)
[04/18] ✅ PASS - SLOW_DOWN Setpoint Mapping (0.0 deg, 0.5 m/s)
[05/18] ✅ PASS - PATH_CLEAR Setpoint Mapping (0.0 deg, 1.0 m/s)
[06/18] ✅ PASS - UNKNOWN Safe Fallback Mapping (vel=0, brake=1)
[07/18] ✅ PASS - Steering Sign & Modbus Two's Complement Conversion
[08/18] ✅ PASS - Steering Limit Clamp [-30.0°, +30.0°]
[09/18] ✅ PASS - Velocity Limit Clamp [-2.5 m/s, +2.5 m/s]
[10/18] ✅ PASS - Brake Request & Auto Interlock (CAN Byte 3 = 0x64)
[11/18] ✅ PASS - Invalid / Malformed Perception Data Handling
[12/18] ✅ PASS - Camera Frame Drop & Hardware Disconnection Handling
[13/18] ✅ PASS - Perception Timeout Watchdog (> 300 ms Staleness)
[14/18] ✅ PASS - DBW Supervisor Fault & Emergency Stop Transition
[15/18] ✅ PASS - Deadman Dropout & FSM IDLE Reversion
[16/18] ✅ PASS - Actuator Slew Rate Limiter (30.0 deg/s Smooth Ramp)
[17/18] ✅ PASS - Steer-Speed Coupling & Neutral Steering on Zero Velocity
[18/18] ✅ PASS - Adapter Clean Thread Termination & Neutral Resource Release
================================================================================
📊 TEST SUITE SUMMARY: 18/18 SCENARIOS PASSED (100.0%)
```

### Scenario Breakdown and Evidence:
1. **Scenario 1 (STOP):** Setpoints correctly initialized to $\text{steer} = 0.0^\circ$, $v = 0.0\text{ m/s}$, $\text{brake} = 1.0$.
2. **Scenario 2 (MOVE_LEFT):** Steer target $= +10.0^\circ$ (positive = left), $v = 0.5\text{ m/s}$, $\text{brake} = 0.0$.
3. **Scenario 3 (MOVE_RIGHT):** Steer target $= -10.0^\circ$ (negative = right), $v = 0.5\text{ m/s}$, $\text{brake} = 0.0$.
4. **Scenario 4 (SLOW_DOWN):** Steer target $= 0.0^\circ$, $v = 0.5\text{ m/s}$, $\text{brake} = 0.0$.
5. **Scenario 5 (PATH_CLEAR):** Steer target $= 0.0^\circ$, $v = 1.0\text{ m/s}$, $\text{brake} = 0.0$.
6. **Scenario 6 (UNKNOWN):** Safe neutral fallback confirmed: $v = 0.0\text{ m/s}$, $\text{brake} = 1.0$.
7. **Scenario 7 (Sign & Two's Complement):** $+10.0^\circ$ produced `100` (`0x0064`) in Register 3; $-10.0^\circ$ produced `65436` (`0xFFA0`) in Register 3.
8. **Scenario 8 (Steering Clamp):** Inputs of $+45.0^\circ$ and $-60.0^\circ$ were clamped strictly to $+30.0^\circ$ and $-30.0^\circ$.
9. **Scenario 9 (Velocity Clamp):** Inputs of $+4.2\text{ m/s}$ and $-3.5\text{ m/s}$ were clamped strictly to $+2.5\text{ m/s}$ and $-2.5\text{ m/s}$.
10. **Scenario 10 (Auto Brake Interlock):** Zero velocity commanded immediately asserted CAN Byte 3 = `0x64` (100% brake).
11. **Scenario 11 (Malformed Data):** `None` input handled gracefully without crashing; forced safe neutral stop.
12. **Scenario 12 (Camera Drop):** Missing frame signal triggered immediate safe neutral stop within 80 ms.
13. **Scenario 13 (Perception Watchdog):** Inactivity for $> 300\text{ ms}$ (tested at 430 ms) tripped watchdog, zeroing speed and asserting brake.
14. **Scenario 14 (Supervisor Fault / E-Stop):** E-stop shifted FSM to `FAULT`, locked brake, and asserted CAN Byte 4 park brake (`0x01`).
15. **Scenario 15 (Deadman Dropout):** Inactivating deadman heartbeat transitioned FSM from `AUTOMODE` to `IDLE`, zeroing velocity.
16. **Scenario 16 (Slew Rate Limiter):** Step command from $0^\circ \to 10^\circ$ measured at $3.01^\circ$ at $100\text{ ms}$ ($30.1^\circ/\text{s}$), fully reaching $10.0^\circ$ at $400\text{ ms}$ without discontinuity.
17. **Scenario 17 (Steer-Speed Coupling):** Stopping vehicle centered steering to $0^\circ$ and locked brakes.
18. **Scenario 18 (Clean Shutdown):** `adapter.stop()` cleanly terminated background threads and reset all registers to neutral.

---

## 14. Project-Local Launcher Specification (`start_multi_object_buggy.sh`)

A safe, unified project-local launcher was authored at `/home/vat/yolo_vehicle_test/start_multi_object_buggy.sh`. It preserves all existing system scripts and provides four operational modes:

- **Mode 1 (`--diagnostic`):** Preflight verification checking Python venv, model weights (6.5 MB), camera device (`/dev/video0`), SocketCAN status, and PLC connectivity. Non-actuating.
- **Mode 2 (`--perception`):** Launches `perception_gui.py` purely for computer vision and perception validation.
- **Mode 3 (`--mock`):** Launches integrated multi-object perception + 20 Hz DBW control adapter in safe offline mock mode.
- **Mode 4 (`--hardware`):** Hardware staging mode. Emits safety warnings and requires the explicit security passkey: `ACTUATE_VIRYA_CONFIRM`.

---

## 15. Remaining Hardware Dependencies & Prerequisites

Before physical track execution can take place, the following hardware dependencies must be fulfilled:

1. **RGB-D Metric Depth Sensor:**
   - The current perception pipeline operates using 2D image-space bounding box and segmentation geometry from the Logitech C270 webcam.
   - For true metric distance and spatial velocity estimation, an Intel RealSense D435i / D455 RGB-D camera must be integrated.
   - Interface requirement: ROS 2 `realsense2_camera` driver or direct `pyrealsense2` pipeline publishing aligned depth frames (`sensor_msgs/msg/Image`, 16-bit depth in mm).
2. **Physical Ethernet & Modbus Connection:**
   - On-board PC Ethernet interface must be statically configured on the subnet: `192.168.0.x` (e.g., `192.168.0.100`, subnet mask `255.255.255.0`).
   - PLC at `192.168.0.5:502` must respond to ICMP ping and Modbus TCP read requests on Register 0.
3. **SocketCAN Interface Configuration:**
   - CAN-to-USB transceiver (e.g., Peak CAN or CANable) must be connected and bound to `can0`.
   - Bus bitrate must be set to **500 kbps** (`ip link set can0 type can bitrate 500000 up`).

---

## 16. Future Controlled Track Testing & Safety Protocol

When transitioning from offline simulation to live vehicle testing, the following safety protocol must be strictly adhered to:

### Phase A: Elevated Wheels Test (Bench Test)
1. Place the VIRYA buggy securely on heavy-duty jack stands such that both rear drive wheels and both front steering wheels are completely elevated off the ground.
2. Verify that wheels spin freely by hand without mechanical obstruction.
3. Power on vehicle low-voltage DBW electronics and PLC.
4. Launch launcher in Mode 3 (`start_multi_object_buggy.sh -m`).
5. Verify steering servo smooth tracking ($\pm 10^\circ$) without jitter.
6. Verify rear wheel traction motor forward spin ($0.5\text{ m/s}$) during `PATH_CLEAR` and caution modes.
7. Test emergency stop mushroom button; confirm immediate wheel arrest within 100 ms.

### Phase B: Closed-Track Low-Speed Autonomous Validation
1. Select a flat, dry, enclosed testing area (minimum 30m $\times$ 15m) free of pedestrian or vehicular traffic.
2. Position a trained safety supervisor in the passenger seat with immediate access to:
   - Mechanical hydraulic emergency brake lever.
   - DBW main electrical isolator switch.
3. Position two ground spotters equipped with two-way radios.
4. Set software velocity clamp to a conservative testing limit: $0.5\text{ m/s}$ ($1.8\text{ km/h}$).
5. Place test obstacles (standard office chair, simulated pedestrian mannequin) at marked distances (5m, 10m).
6. Verify vehicle autonomous avoidance maneuvers (`MOVE_LEFT`, `MOVE_RIGHT`, `STOP`) and record actuation latencies.

---

## 17. Master File & Verification Artifact Index

All deliverables for this integration milestone reside exclusively within the project workspace:

| File Path | Type | Purpose / Description |
|---|---|---|
| `/home/vat/yolo_vehicle_test/vehicle_control_adapter.py` | Python Script | 20 Hz rate-decoupling control adapter, slew limiter, watchdog coordinator |
| `/home/vat/yolo_vehicle_test/perception_gui.py` | Python Script | Live multi-object GUI dashboard with real-time DBW telemetry monitor |
| `/home/vat/yolo_vehicle_test/test_control_integration.py` | Python Script | 18-scenario automated offline integration test suite |
| `/home/vat/yolo_vehicle_test/start_multi_object_buggy.sh` | Shell Script | Project launcher with diagnostic, perception, mock, and staging modes |
| `/home/vat/yolo_vehicle_test/results/evaluation/control_integration_test_report.json` | JSON Report | Complete test execution evidence (18/18 tests passed) |
| `/home/vat/yolo_vehicle_test/results/evaluation/integrated_control_gui_snapshot.jpg` | Image | Visual evidence of dashboard with real-time DBW control telemetry |
| `/home/vat/yolo_vehicle_test/documentation/VEHICLE_CONTROL_INTEGRATION_REPORT.md` | Markdown Report | This comprehensive master technical report |

---

## Conclusion & Readiness Declaration

The perception-to-control integration milestone has been **fully satisfied and validated**:
- The exact root causes of physical non-actuation have been comprehensively documented and addressed.
- The 20 Hz periodic control adapter guarantees watchdog compliance and eliminates servo step discontinuities.
- The perception GUI displays transparent, real-time subsystem state tracking distinguishing requested, commanded, accepted, and executed states.
- 18 out of 18 offline integration scenarios have achieved a 100% pass rate.
- Zero external vehicle files were modified, strictly upholding file safety.

The autonomy stack is fully prepared for bench testing and controlled track staging.
