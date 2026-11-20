# VIRYA Drive-By-Wire (DBW) Vehicle Architecture Report

**Document ID:** VAR-VIRYA-2026-001  
**Project:** RGB-D Multi-Object Perception and Obstacle-Avoidance System  
**Investigation Workspace:** `/home/vat/yolo_vehicle_test/`  
**Primary Vehicle Stack:** `/home/vat/dbw_ws/` | `/home/vat/dbw_ws_2/` | `/home/vat/DBW/`  
**Target Environment:** ROS 2 Jazzy Jalisco on Ubuntu 24.04 LTS (x86_64, CPU inference, No Discrete GPU)  
**Status:** Read-Only Technical Architecture Baseline  

---

## 1. Executive Summary

This report establishes the baseline technical architecture of the operational VIRYA Drive-By-Wire (DBW) electric vehicle. The existing vehicle platform features full electronic drive-by-wire capability encompassing front-wheel steering, rear-wheel motor traction and velocity control, pneumatic/electromechanical braking, a hardware deadman safety watchdog, and basic camera-based perception.

The objective of our upcoming project is to integrate an advanced perception and intelligent decision system utilizing:
- **YOLO26n-seg** for multi-object detection and instance segmentation.
- **RGB-D camera data** (replacing monocular RGB) for accurate 3D metric distance and spatial position estimation.
- **Dynamic vehicle path assessment** and intelligent steering/braking decision logic.
- Standardized ROS 2 Jazzy interfaces compatible with the existing vehicle DBW stack without modifying any operational vehicle components.

---

## 2. Overall System Architecture

The VIRYA DBW system is architected across three distinct functional domains:
1. **Perception and Autonomy Layer (ROS 2 Python / C++):** Interfaced via standard ROS 2 topics.
2. **Drive-By-Wire Core and Gateway Layer (`dbw_module_cpp`):** Handles state machine management (FSM), command validation, multi-node heartbeat tracking, slew-rate conditioning, and protocol translation.
3. **Vehicle Actuator and Bus Layer:**
   - **SocketCAN (`can0`, 500 kbps):** Communicates with the Central Control Unit (CCU) for propulsion (rear wheels), service braking, drive modes, and speed feedback.
   - **Modbus TCP (`192.168.0.5:502`, Slave ID `0x0001`):** Communicates with the front-wheel steering PLC/servo actuator for steering angle commands and real-time encoder feedback.

```
       +-----------------------------------------------------------+
       |             Perception & Autonomy Layer                   |
       |  (v4l2_camera / Intel RealSense D435 + YOLO Perception)   |
       +-----------------------------------------------------------+
               |                       |                     |
  /dbw/auto/cmd_steer         /dbw/auto/cmd_vel      /dbw/auto/brake
  (std_msgs/Float64)          (std_msgs/Float64)     (std_msgs/Bool)
               |                       |                     |
               +-----------------------+---------------------+
                                       |
                                       v
       +-----------------------------------------------------------+
       |             DBW Core Supervisor: dbw_node                 |
       |           (FSM: IDLE -> AUTOMODE / FAULT)                 |
       |  Watches /dbw/deadman_switch (200ms) & Staleness (150ms)  |
       +-----------------------------------------------------------+
                     |                               |
       /dbw/modbus_publish                   /dbw/canbus_publish
      (mod_write_msgs/ModRequest)           (can_write_msgs/CanRequest)
                     |                               |
                     v                               v
       +---------------------------+   +---------------------------+
       |      mod_writer_node      |   |      can_writer_node      |
       |       (Modbus TCP)        |   |        (SocketCAN)        |
       +---------------------------+   +---------------------------+
                     |                               |
         Modbus TCP: 192.168.0.5:502             SocketCAN: can0 (500 kbps)
                     |                               |
                     v                               v
       +---------------------------+   +---------------------------+
       |  Front Steering Actuator  |   |  Rear Traction Motor/Brake|
       |   (PLC / Servo Drive)     |   |   (Vehicle CCU Controller)|
       +---------------------------+   +---------------------------+
```

---

## 3. Startup Sequence Analysis (`start_buggy.sh`)

The authoritative vehicle initialization routine is codified in `/home/vat/yolo_vehicle_test/start_buggy.sh`. The table below provides an exhaustive command-by-command audit of this sequence:

| Step | Command Executed | Purpose & Action | Underlying Executable & Source File | Workspace & Package | Topics Used | Communicates with Hardware? | Relevance to New Architecture |
|---|---|---|---|---|---|---|---|
| **[0/5]** | `sudo ip link set can0 down`<br>`sudo ip link set can0 type can bitrate 500000`<br>`sudo ip link set can0 up` | Configures Linux SocketCAN interface `can0` to 500 kbit/s bitrate. Resets interface to clear bus-off errors. | System kernel SocketCAN / iproute2 (`/bin/ip`) | System / OS | N/A (CAN link layer) | **YES** (CAN hardware transceiver) | **REQUIRED (PRESERVE)**: Must be configured before any CAN communication occurs. |
| **[1/5]** | `gnome-terminal -- bash -c "candump can0; exec bash"` | Launches terminal monitoring live raw CAN frames on `can0`. | `candump` (can-utils binary) | System / OS | N/A (Direct SocketCAN sniffer) | **YES** (Passively listens to CAN bus) | **DIAGNOSTIC ONLY**: Invaluable for monitoring CAN communication during validation. |
| **[2/5]** | `gnome-terminal -- bash -c "source /opt/ros/jazzy/setup.bash; cd ~/dbw_ws_2; source install/setup.bash; bash dbw_launch.bash; exec bash"` | Executes `dbw_launch.bash`, which brings up `can0` if down and launches `dbw_module_cpp dbw_launch.py`. | Python launcher `ros2 launch dbw_module_cpp dbw_launch.py` launching 5 ROS 2 nodes. | Built from `/home/vat/DBW/src/dbw_module/dbw_module_cpp/` installed in `/home/vat/dbw_ws/install/` and `/home/vat/dbw_ws_2/install/` | Multiple topics (see Section 4) | **YES** (Full CAN and Modbus vehicle control) | **REQUIRED CORE (UNTOUCHED)**: The central vehicle hardware gateway. Must remain completely intact. |
| **[3/5]** | `gnome-terminal -- bash -c "source /opt/ros/jazzy/setup.bash; ros2 run v4l2_camera v4l2_camera_node; exec bash"` | Starts standard V4L2 USB camera driver capturing from `/dev/video0` (Logitech C270). | `v4l2_camera_node` | `/opt/ros/jazzy` (`v4l2_camera`) | Publishes `/image_raw`, `/camera_info` | **YES** (USB 2.0 Web Camera) | **TO BE REPLACED**: Monocular 2D webcam without depth information. Will be replaced by an RGB-D camera driver (e.g. `realsense2_camera_node`). |
| **[4/5]** | `gnome-terminal -- bash -c "source /opt/ros/jazzy/setup.bash; ros2 topic pub -r 20 /dbw/deadman_switch std_msgs/msg/String '{data: Auto}'; exec bash"` | Publishes heartbeat on deadman topic at 20 Hz (50 ms interval). Keeps DBW node in `AUTOMODE`. | `ros2 topic pub` CLI tool | ROS 2 Core (`std_msgs`) | Publishes `/dbw/deadman_switch` | **NO** (ROS 2 internal IPC topic) | **TEMPORARY / REPLACED**: Must be active during autonomy, but should be managed by our system supervisor node rather than an unmonitored terminal. |
| **[5/5]** | `gnome-terminal -- bash -c "source /opt/ros/jazzy/setup.bash; cd ~/dbw_ws_2; source install/setup.bash; ros2 run my_package yolo_obstacle_avoidance_node; exec bash"` | Launches legacy YOLOv8n obstacle avoidance node. | Python script `/home/vat/dbw_ws_2/src/my_package/my_package/yolo_obstacle_avoidance_node.py` | `/home/vat/dbw_ws_2/src/my_package/` | Subscribes `/image_raw`; Publishes `/dbw/auto/cmd_vel`, `/dbw/auto/cmd_steer`, `/dbw/auto/brake` | **NO** (Commands sent to DBW stack) | **TO BE REPLACED**: This is the legacy baseline node to be replaced by the new RGB-D YOLO26n-seg pipeline. |

---

## 4. Drive-By-Wire (DBW) Core Stack Architecture

The DBW core stack is implemented in C++ in package `dbw_module_cpp`. It is initiated via `/home/vat/DBW/src/dbw_module/dbw_module_cpp/launch/dbw_launch.py`.

### 4.1 Node Breakdown

#### 1. `dbw_node` (`src/dbw/dbw_node.cpp`, `dbw.cpp`, `fsm.cpp`)
- **Role:** Central vehicle coordinator and Finite State Machine (FSM).
- **FSM States:** `IDLE`, `JOYMODE`, `AUTOMODE`, `TELEOPMODE`, `FAULT`.
- **Transitions:**
  - Transitions to `AUTOMODE` upon receiving string `"Auto"` on `/dbw/deadman_switch`.
  - Reverts to `IDLE` if `/dbw/deadman_switch` is not received for **200 ms** (`Utils::DEADMAN_TIMEOUT`).
  - Transitions to `FAULT` if a shutdown message (`data: true`) is received on `/dbw/shutdown`.
- **Gating Logic:**
  - All autonomous control callbacks (`auto_speed_callback`, `auto_steer_callback`, `auto_brake_callback`) are strictly gated by `fsm_.get_state() == fsm_state::AUTOMODE`.
  - When in `IDLE` or `FAULT`, commands are zeroed and brakes are applied.
- **Periodic Publishing Loops:**
  - **`can_timer_` (20 ms / 50 Hz):** Evaluates `topic_staleness = curr_time - speed_receive_time_`. If stale (>150 ms), zeroes velocity. Formulates `can_write_msgs/msg/CanRequest` with `velocity = velocity_ * 100` and `brake = (velocity_ == 0 || emergency_stop_ || coils_.brake) ? 1 : 0`, published to `/dbw/canbus_publish`.
  - **`mod_timer_` (20 ms / 50 Hz):** Formulates `mod_write_msgs/msg/ModRequest`. If `topic_staleness <= 120 ms`, transmits `steer_angle_`. If `>120 ms`, resets `steer_angle` to `0`. Publishes to `/dbw/modbus_publish`.
  - **`heartbeat_timer_` (100 ms / 10 Hz):** Publishes `heartbeat_msgs/msg/Heartbeat` with `node_name = "DBW"` to `/dbw/heartbeat`.
  - **`deadman_timer_` (50 ms / 20 Hz):** Executes `check_deadman_level()`. Evaluates duration since last deadman message. If elapsed $\ge 200\text{ ms}$, transitions state to `IDLE` and logs warning.

#### 2. `can_writer_node` (`src/can_writer/can_writer_node.cpp`, `can_writer.cpp`)
- **Role:** Low-level bridge between ROS 2 and SocketCAN.
- **Parameters:**
  - `interface`: `"can0"`
  - `sender_id`: `0x12280797` (Extended CAN ID)
  - `receiver_id`: `0x12910109` (Extended CAN ID)
  - `receiver_filter`: `"12910109:FFFFFFFF"`
  - `enable_fd`: `false` (Classic CAN 2.0B, 8-byte payload)
- **Functions:**
  - Subscribes to `/dbw/canbus_publish` (`can_write_msgs/msg/CanRequest`).
  - Serializes data into 8-byte CAN frame:
    - Byte 0: `0x01` (Forward, $v > 0$), `0x03` (Reverse, $v < 0$), `0x00` (Neutral/Stopped, $v = 0$).
    - Bytes 1 & 2: Speed value in cm/s (16-bit integer, LSB=Byte 1, MSB=Byte 2).
    - Byte 3: Brake level (`0x64` = 100% brake if applied; `0x00` = released).
    - Byte 7: Heartbeat marker (`0x01`).
  - Receives vehicle status from CAN ID `0x12910109` every 20 ms and publishes to `/dbw/canbus_receive` (`can_msgs/msg/Frame`).
  - Safety Watchdog: If no command is received on `/dbw/canbus_publish` for `2000 ms`, enters `shutdown_ = true`, broadcasts safe stop CAN packet (Brake `0x64`, Drive Mode `0x00`, Auto Park Brake `1`), and stops sending heartbeat.

#### 3. `mod_writer_node` (`src/mod_writer/mod_writer_node.cpp`, `mod_writer.cpp`)
- **Role:** Industrial Modbus TCP client communicating with front-wheel steering controller.
- **Parameters:**
  - `ipaddress`: `"192.168.0.5"`
  - `port`: `"502"`
  - `slave_id`: `0x0001`
- **Functions:**
  - Subscribes to `/dbw/modbus_publish` (`mod_write_msgs/msg/ModRequest`).
  - Scales steering angle by $10$ (`steer_angle_int = static_cast<int16_t>(steer_angle * 10)`).
  - Handles two's complement for negative angles: `(1 << 16) + steer_angle_int`.
  - Writes to Modbus Register 3 (`Utils::RegisterId::STEER_WRITE_REGISTER`).
  - Reads registers 0, 2, 4 every 50 ms. Register 0 is actual steering feedback, published on `/dbw/modread/registers`.
  - Publishes heartbeat on `/dbw/heartbeat` with `node_name = "ModWriter"`.

#### 4. `diagnostics_node` (`src/diagnostics/diagnostics_node.cpp`, `diagnostics.cpp`)
- **Role:** System-wide heartbeat and watchdog monitor.
- **Monitored Nodes & Tolerances:**
  - `"DBW"`: 4000 ms
  - `"CanWriter"`: 4000 ms
  - `"ModWriter"`: 4000 ms
- **Action:** If any node fails to transmit `/dbw/heartbeat` within its tolerance, publishes `data: true` to `/dbw/shutdown`. This immediately transitions `dbw_node` to `FAULT` and locks brakes.

#### 5. `can_reader.py` (`scripts/can_reader.py`)
- **Role:** Python-based CAN monitor running `candump can0` via a dedicated thread.
- **Functions:**
  - Parses CAN ID `12910109` (0x12910109):
    - Byte 4 bit 0: DBW Mode (`Auto` vs `Manual`), published on `/dbw/feedback/mode` (`std_msgs/Int32`).
    - Byte 4 bit 4: Emergency Stop status.
    - Bytes 6 & 7: Vehicle speed (`raw_speed / 100.0` m/s), published on `/dbw/feedback/vehicle_speed` (`std_msgs/Float64`).
    - Brake actuator pose, Park brake, LLC stop, Battery SOC published on `/dbw/feedback/brake_status` and `/dbw/feedback/battery_soc`.
  - Parses CAN ID `12890812` (0x12890812) for manual brake pedal percentage.

---

## 5. Steering and Braking Control Flow

### 5.1 Front-Wheel Steering Pipeline

```
  Perception / Navigation Node
               |
               v  Publishes target angle [-30.0°, +30.0°] in degrees
       /dbw/auto/cmd_steer (std_msgs/Float64)
               |
               v  Gated by FSM (AUTOMODE) & Clamped to steer_limit_ (30.0°)
            DBWNode
               |
               v  Packaged into ModRequest message
       /dbw/modbus_publish (mod_write_msgs/ModRequest)
               |
               v  Scaled x10, two's complement if negative
         ModWriter Node
               |
               v  Modbus TCP Function 0x06 (Write Single Register)
     Ethernet TCP/IP (192.168.0.5:502) -> Holding Register 3
               |
               v
     Steering Actuator / Servo Drive turns front wheels
               |
               v  Encoder feedback read from Register 0 via Modbus TCP
         ModWriter Node
               |
               v
     /dbw/modread/registers (register0 / 10.0)
               |
               v
     /dbw/feedback/current_steer (std_msgs/Float64)
```

### 5.2 Acceleration and Braking Pipeline

```
  Perception / Navigation Node
               |
       +-------+-------+
       |               |
       v               v
/dbw/auto/cmd_vel  /dbw/auto/brake
(std_msgs/Float64) (std_msgs/Bool)
       |               |
       +-------+-------+
               |
               v  Gated by AUTOMODE; Speed clamped [-2.5, +2.5] m/s
            DBWNode
               |  If velocity == 0 or speed stale (>150ms), brake forced to 1
               v
       /dbw/canbus_publish (can_write_msgs/CanRequest)
               |
               v  Byte 0: Mode (0x01 Fwd / 0x03 Rev / 0x00 Stop)
         CanWriter Node    Bytes 1-2: Speed in cm/s
               |           Byte 3: Brake (0x64 if active, 0x00 if released)
               v
         SocketCAN can0 (500 kbps) -> CAN ID: 0x12280797
               |
               v
     Vehicle Central Control Unit (CCU) / Rear Traction Motor & Brake
               |
               v  Feedback via CAN ID: 0x12910109
     can_writer_node & can_reader.py
               |
               v
     /dbw/feedback/vehicle_speed & /dbw/feedback/brake_status
```

---

## 6. ROS 2 Workspace & Package Structure

The system filesystem contains several related workspaces resulting from vehicle development history:

```
/home/vat/
├── yolo_vehicle_test/            <-- Our designated active workspace (isolated)
│   ├── start_buggy.sh            <-- Startup script copy
│   ├── benchmark_yolo.py         <-- YOLO benchmarking tool
│   ├── test_yolo_webcam.py       <-- Webcam testing script
│   ├── yolo26n-seg.pt            <-- Candidate segmentation model weights
│   ├── yolo26s-seg.pt            <-- Candidate segmentation model weights
│   └── venv/                     <-- Python virtual environment
├── dbw_ws_2/                     <-- Primary operational workspace running on vehicle
│   ├── dbw_launch.bash           <-- DBW launcher script
│   ├── yolo_obstacle_avoidance_node.py <-- Standalone test script (incomplete)
│   └── src/
│       ├── my_package/           <-- Active vehicle application package
│       │   └── my_package/
│       │       └── yolo_obstacle_avoidance_node.py <-- ACTUAL executed node
│       ├── camera_pkg/           <-- Custom camera node (V4L2 OpenCV backend)
│       ├── adas_detection/       <-- Standalone FCW/pedestrian node
│       ├── master_hmi/           <-- HMI monitoring tools
│       └── tlsids_can/           <-- CAN intrusion detection research package
├── dbw_ws/                       <-- Base DBW installation referenced by setup scripts
│   ├── install/
│   │   ├── dbw_module_cpp/       <-- Core DBW binaries & launch files
│   │   ├── can_write_msgs/       <-- Custom ROS 2 message definitions
│   │   ├── mod_write_msgs/       <-- Custom ROS 2 message definitions
│   │   ├── mod_read_msgs/        <-- Custom ROS 2 message definitions
│   │   ├── heartbeat_msgs/       <-- Custom ROS 2 message definitions
│   │   └── stamped_msgs/         <-- Custom ROS 2 message definitions
└── DBW/                          <-- Full C++ source tree for dbw_module_cpp
    └── src/
        ├── dbw_module/
        │   ├── dbw_module_cpp/   <-- Source for dbw_node, can_writer, mod_writer
        │   └── messages/         <-- Source for all DBW message packages
        └── thirdparty/           <-- ros2_socketcan, libros2qt, serial
```

---

## 7. Safety Architecture & Safeguards

The vehicle incorporates a multi-tiered safety watchdog architecture:

```
[ Tier 1: Application Watchdog ]
  - Deadman Switch: /dbw/deadman_switch must publish "Auto" at >= 5 Hz (timeout: 200 ms).
  - Command Staleness: Speed commands stale after 150 ms trigger immediate zero-velocity and brake lock.
  - Steer Staleness: Speed commands stale after 120 ms force steer angle to centre (0.0°).

[ Tier 2: Middleware & Heartbeat Watchdog ]
  - dbw/heartbeat tracked by diagnostics_node.
  - If DBW, CanWriter, or ModWriter drops for >4000 ms, /dbw/shutdown is published.
  - /dbw/shutdown forces FSM into FAULT state.

[ Tier 3: Driver & Actuator Watchdog ]
  - can_writer_node monitors incoming CanRequest messages. Timeout: 2000 ms.
  - On timeout, can_writer_node autonomously injects emergency stop CAN frame:
    Drive Mode = 0, Brake = 0x64 (100%), Auto Park Brake = 1.
  - ModWriter monitors TCP socket connectivity to 192.168.0.5:502. On failure, triggers auto-reconnect.

[ Tier 4: Hardware Emergency Stop ]
  - Hardwired emergency stop button on vehicle asserted over CAN ID 0x12910109 (Byte 4 Bit 4).
  - dbw_node immediately aborts AUTOMODE upon detecting emergency bit.
```

---

## 8. Architectural Conclusions for RGB-D Integration

1. **Hardware Bridge Integrity:** The C++ core stack (`dbw_node`, `can_writer_node`, `mod_writer_node`, `diagnostics_node`) is stable, robust, and must remain completely untouched.
2. **Clear Decoupling Boundary:** The entire vehicle actuation layer is controllable strictly through three well-defined ROS 2 topics:
   - `/dbw/auto/cmd_steer` (`std_msgs/msg/Float64`)
   - `/dbw/auto/cmd_vel` (`std_msgs/msg/Float64`)
   - `/dbw/auto/brake` (`std_msgs/msg/Bool`)
   - Accompanied by the continuous deadman signal on `/dbw/deadman_switch` (`std_msgs/msg/String`, data `"Auto"`).
3. **Perception Replacement:** The legacy `v4l2_camera` node and `my_package/yolo_obstacle_avoidance_node` must be retired in favor of an RGB-D depth camera pipeline and our new YOLO26n-seg perception and spatial path planning system.
