# Comprehensive File & Dependency Reference Index

**Document ID:** FRI-VIRYA-2026-005  
**Target Platform:** VIRYA Drive-By-Wire Vehicle  
**Workspace:** `/home/vat/yolo_vehicle_test/`  
**Status:** Read-Only Inventory and Dependency Map  

---

## 1. Master File Reference Inventory

The table below catalogs every relevant file, script, node, configuration, and launch asset across the vehicle system. Each asset is categorized by origin, execution capability, runtime dependency status, and modification constraints.

| # | Absolute File Path | Subsystem / Package | Purpose & Functionality | Original vs Copied | Executable? | Runtime Dependency? | Modification Policy |
|---|---|---|---|---|---|---|---|
| **1** | `/home/vat/yolo_vehicle_test/start_buggy.sh` | Workspace Launcher | Copied startup script orchestrating CAN setup, candump, DBW stack, camera, deadman switch, and YOLO node. | **Copied** | **Yes** (`rwxrwxr-x`) | No (Reference script) | **DO NOT MODIFY** (Reference baseline) |
| **2** | `/home/vat/yolo_vehicle_test/benchmark_yolo.py` | Benchmark / Test | Headless benchmark script evaluating YOLO model latency, FPS, and class detections on CPU using recorded video. | Original in workspace | No (`rw-rw-r--`) | No (Benchmarking tool) | Can be used/adapted in active workspace |
| **3** | `/home/vat/yolo_vehicle_test/test_yolo_webcam.py` | Benchmark / Test | Interactive webcam testing script for real-time YOLO CPU inference, bounding boxes, and FPS display. | Original in workspace | No (`rw-rw-r--`) | No (Testing tool) | Can be used/adapted in active workspace |
| **4** | `/home/vat/yolo_vehicle_test/yolo26n-seg.pt` | Perception Weights | PyTorch model weights for YOLO26 Nano instance segmentation. | Original in workspace | No (`rw-rw-r--`) | **Yes** (Model asset for new perception) | **READ-ONLY ASSET** |
| **5** | `/home/vat/yolo_vehicle_test/yolo26s-seg.pt` | Perception Weights | PyTorch model weights for YOLO26 Small instance segmentation. | Original in workspace | No (`rw-rw-r--`) | No (Alternative heavier model) | **READ-ONLY ASSET** |
| **6** | `/home/vat/dbw_ws_2/dbw_launch.bash` | Vehicle DBW Stack | Primary bash launcher bringing up `can0` at 500k bitrate, sourcing ROS Jazzy and `dbw_ws`, and launching `dbw_launch.py`. | **Original Vehicle File** | **Yes** (`rwxrwxr-x`) | **YES (Core Vehicle Launcher)** | **STRICTLY UNTOUCHED (READ-ONLY)** |
| **7** | `/home/vat/dbw_ws_2/yolo_obstacle_avoidance_node.py` | Legacy Test Script | Standalone prototype YOLOv8n script with subscribers/publishers created but no publish calls. | **Original Vehicle File** | No (`rw-rw-r--`) | No (Incomplete prototype) | **STRICTLY UNTOUCHED (READ-ONLY)** |
| **8** | `/home/vat/dbw_ws_2/src/my_package/setup.py` | ROS 2 Package Config | Package configuration registering console scripts including `yolo_obstacle_avoidance_node`, `steer`, `brake_code`, etc. | **Original Vehicle File** | No (`rw-rw-r--`) | **YES (Build/Entry Config)** | **STRICTLY UNTOUCHED (READ-ONLY)** |
| **9** | `/home/vat/dbw_ws_2/src/my_package/my_package/yolo_obstacle_avoidance_node.py` | Legacy Perception | **The actual operational node** executed by `ros2 run my_package yolo_obstacle_avoidance_node`. Contains 3-zone heuristic control. | **Original Vehicle File** | **Yes** (`rwxrwxr-x`) | **YES (Current Active Node)** | **STRICTLY UNTOUCHED (READ-ONLY)** |
| **10** | `/home/vat/dbw_ws_2/src/my_package/my_package/steer.py` | Test Utility | Test publisher commanding static `10.0°` to `/dbw/auto/cmd_steer` at 20 Hz. | **Original Vehicle File** | No (`rw-rw-r--`) | No (Test node) | **STRICTLY UNTOUCHED (READ-ONLY)** |
| **11** | `/home/vat/dbw_ws_2/src/my_package/my_package/steer2.py` | Test Utility | Test publisher sweeping steering between `-0.20` and `+0.20` at 20 Hz. | **Original Vehicle File** | No (`rw-rw-r--`) | No (Test node) | **STRICTLY UNTOUCHED (READ-ONLY)** |
| **12** | `/home/vat/dbw_ws_2/src/my_package/my_package/brake_code.py` | Test Utility | Test publisher commanding brake on `/dbw/auto/brake` at 20 Hz. | **Original Vehicle File** | No (`rw-rw-r--`) | No (Test node) | **STRICTLY UNTOUCHED (READ-ONLY)** |
| **13** | `/home/vat/dbw_ws_2/src/my_package/my_package/yolo_brake_node.py` | ADAS Utility | Vision node detecting objects and commanding full brake if any object detected, else 1.0 m/s cruise. | **Original Vehicle File** | No (`rw-rw-r--`) | No (Alternative ADAS node) | **STRICTLY UNTOUCHED (READ-ONLY)** |
| **14** | `/home/vat/dbw_ws_2/src/my_package/my_package/object_brake_node.py` | ADAS Utility | Haar-cascade full-body detection node commanding brake upon pedestrian detection. | **Original Vehicle File** | **Yes** (`rwxrwxr-x`) | No (Legacy OpenCV node) | **STRICTLY UNTOUCHED (READ-ONLY)** |
| **15** | `/home/vat/dbw_ws_2/brake_keyboard.py` | Teleop / Testing | Interactive console tool allowing operator to type `1` to apply brake and `0` to release brake. | **Original Vehicle File** | **Yes** (`rwxrwxr-x`) | No (Manual test tool) | **STRICTLY UNTOUCHED (READ-ONLY)** |
| **16** | `/home/vat/dbw_ws/dbw_teleop.py` | Teleop Controller | Full terminal keyboard teleop node for WASD driving, spacebar emergency brake, and feedback monitoring. | **Original Vehicle File** | **Yes** (`rwxr-x--x`) | No (Operator teleop utility) | **STRICTLY UNTOUCHED (READ-ONLY)** |
| **17** | `/home/vat/dbw_ws_2/control_gui.py` | GUI Utility | Tkinter-based control GUI providing arming switches, velocity/steering sliders, and status display. | **Original Vehicle File** | No (`rw-rw-r--`) | No (Operator GUI utility) | **STRICTLY UNTOUCHED (READ-ONLY)** |
| **18** | `/home/vat/DBW/src/dbw_module/dbw_module_cpp/launch/dbw_launch.py` | DBW Core Launch | Authoritative ROS 2 launch description launching `can_writer_node`, `dbw_node`, `mod_writer_node`, `diagnostics_node`, and `can_reader.py`. | **Original Vehicle File** | No (`rw-rw-r--`) | **YES (Core Vehicle Launcher)** | **STRICTLY UNTOUCHED (READ-ONLY)** |
| **19** | `/home/vat/DBW/src/dbw_module/dbw_module_cpp/config/dbw_config.yaml` | DBW Configuration | Central parameter configuration setting velocity limits (2.5), steer limits (30), slew rates (30 deg/s), and timeouts. | **Original Vehicle File** | No (`rw-rw-r--`) | **YES (Core Configuration)** | **STRICTLY UNTOUCHED (READ-ONLY)** |
| **20** | `/home/vat/DBW/src/dbw_module/dbw_module_cpp/src/dbw/dbw_node.cpp` | DBW Supervisor | C++ executable entry point initializing `DBWNode` instance and spinning ROS 2 executor. | **Original Vehicle File** | No (`rw-rw-r--`) | **YES (Core Binary Source)** | **STRICTLY UNTOUCHED (READ-ONLY)** |
| **21** | `/home/vat/DBW/src/dbw_module/dbw_module_cpp/src/dbw/dbw.cpp` | DBW Supervisor | Central implementation of FSM transitions, command subscriptions, topic staleness watchdogs, and actuation loops. | **Original Vehicle File** | No (`rw-rw-r--`) | **YES (Core Binary Source)** | **STRICTLY UNTOUCHED (READ-ONLY)** |
| **22** | `/home/vat/DBW/src/dbw_module/dbw_module_cpp/include/dbw_module_cpp/dbw.hpp` | DBW Header | C++ class declaration for `DBWNode`. | **Original Vehicle File** | No (`rw-rw-r--`) | **YES (Core Binary Header)** | **STRICTLY UNTOUCHED (READ-ONLY)** |
| **23** | `/home/vat/DBW/src/dbw_module/dbw_module_cpp/src/dbw/fsm.cpp` | State Machine | C++ finite state machine implementation managing transitions between `IDLE`, `AUTOMODE`, `JOYMODE`, and `FAULT`. | **Original Vehicle File** | No (`rw-rw-r--`) | **YES (Core State Logic)** | **STRICTLY UNTOUCHED (READ-ONLY)** |
| **24** | `/home/vat/DBW/src/dbw_module/dbw_module_cpp/include/dbw_module_cpp/utils.hpp` | Constants / Enums | Header defining critical vehicle constants (`DEADMAN_TIMEOUT = 200`, `CAN_CMD_TIME_OUT = 2000`, register IDs, coil IDs). | **Original Vehicle File** | No (`rw-rw-r--`) | **YES (Core Constants)** | **STRICTLY UNTOUCHED (READ-ONLY)** |
| **25** | `/home/vat/DBW/src/dbw_module/dbw_module_cpp/src/can_writer/can_writer_node.cpp` | CAN Writer Node | C++ main entry point initializing `CanWriter` with interface `can0`, sender ID `0x12280797`, and receiver ID `0x12910109`. | **Original Vehicle File** | No (`rw-rw-r--`) | **YES (Core Binary Source)** | **STRICTLY UNTOUCHED (READ-ONLY)** |
| **26** | `/home/vat/DBW/src/dbw_module/dbw_module_cpp/src/can_writer/can_writer.cpp` | CAN Writer | C++ SocketCAN gateway implementation packing speed, direction, and brake commands into 8-byte CAN frames. | **Original Vehicle File** | No (`rw-rw-r--`) | **YES (Core Binary Source)** | **STRICTLY UNTOUCHED (READ-ONLY)** |
| **27** | `/home/vat/DBW/src/dbw_module/dbw_module_cpp/src/mod_writer/mod_writer_node.cpp` | Modbus Writer Node | C++ main entry point initializing `ModWriter` with IP `192.168.0.5` and port `502`. | **Original Vehicle File** | No (`rw-rw-r--`) | **YES (Core Binary Source)** | **STRICTLY UNTOUCHED (READ-ONLY)** |
| **28** | `/home/vat/DBW/src/dbw_module/dbw_module_cpp/src/mod_writer/mod_writer.cpp` | Modbus Writer | C++ Modbus TCP gateway writing steering angle (Register 3) and reading steering feedback (Register 0). | **Original Vehicle File** | No (`rw-rw-r--`) | **YES (Core Binary Source)** | **STRICTLY UNTOUCHED (READ-ONLY)** |
| **29** | `/home/vat/DBW/src/dbw_module/dbw_module_cpp/src/diagnostics/diagnostics_node.cpp` | Watchdog Monitor | C++ main entry point initializing `Diagnostics` node. | **Original Vehicle File** | No (`rw-rw-r--`) | **YES (Core Binary Source)** | **STRICTLY UNTOUCHED (READ-ONLY)** |
| **30** | `/home/vat/DBW/src/dbw_module/dbw_module_cpp/src/diagnostics/diagnostics.cpp` | Watchdog Monitor | Monitors node heartbeats (4000 ms tolerance). Publishes `/dbw/shutdown` on failure. | **Original Vehicle File** | No (`rw-rw-r--`) | **YES (Core Binary Source)** | **STRICTLY UNTOUCHED (READ-ONLY)** |
| **31** | `/home/vat/DBW/src/dbw_module/dbw_module_cpp/scripts/can_reader.py` | Telemetry Reader | Python script reading CAN telemetry on `can0` via `candump` and publishing speed and brake status. | **Original Vehicle File** | **Yes** (`rwxrwxr-x`) | **YES (Core Telemetry Node)** | **STRICTLY UNTOUCHED (READ-ONLY)** |
| **32** | `/home/vat/DBW/src/dbw_module/dbw_module_cpp/src/teleop_remap/teleop_remap.cpp` | Teleop Converter | C++ node converting `geometry_msgs/Twist` into slew-limited steering angles, velocities, and deadman signals. | **Original Vehicle File** | No (`rw-rw-r--`) | No (Teleop converter) | **STRICTLY UNTOUCHED (READ-ONLY)** |
| **33** | `/home/vat/DBW/src/dbw_module/dbw_module_cpp/src/joy_remap/joy_remap.cpp` | Joystick Converter | C++ node converting `sensor_msgs/Joy` into DBW auto commands and deadman signals. | **Original Vehicle File** | No (`rw-rw-r--`) | No (Joystick converter) | **STRICTLY UNTOUCHED (READ-ONLY)** |
| **34** | `/home/vat/DBW/src/dbw_module/messages/can_write_msgs/msg/CanRequest.msg` | Message Definition | ROS 2 message definition for internal CAN write requests (`header`, `id`, `velocity`, `brake`). | **Original Vehicle File** | No (`rw-rw-r--`) | **YES (Message Dependency)** | **STRICTLY UNTOUCHED (READ-ONLY)** |
| **35** | `/home/vat/DBW/src/dbw_module/messages/mod_write_msgs/msg/ModRequest.msg` | Message Definition | ROS 2 message definition for internal Modbus requests (`steer_angle`, `indicator`, `speed`, `brake`, etc.). | **Original Vehicle File** | No (`rw-rw-r--`) | **YES (Message Dependency)** | **STRICTLY UNTOUCHED (READ-ONLY)** |
| **36** | `/home/vat/DBW/src/dbw_module/messages/mod_read_msgs/msg/Registers.msg` | Message Definition | ROS 2 message definition for Modbus register feedback (`register0`, `register2`, `register4`). | **Original Vehicle File** | No (`rw-rw-r--`) | **YES (Message Dependency)** | **STRICTLY UNTOUCHED (READ-ONLY)** |
| **37** | `/home/vat/DBW/src/dbw_module/messages/heartbeat_msgs/msg/Heartbeat.msg` | Message Definition | ROS 2 message definition for node heartbeats (`node_name`, `heartbeat_count`, `timestamp`). | **Original Vehicle File** | No (`rw-rw-r--`) | **YES (Message Dependency)** | **STRICTLY UNTOUCHED (READ-ONLY)** |
| **38** | `/home/vat/DBW/src/dbw_module/messages/stamped_msgs/msg/Float64.msg` | Message Definition | Custom stamped float message definition (`timestamp`, `priority_level`, `data`). | **Original Vehicle File** | No (`rw-rw-r--`) | **YES (Message Dependency)** | **STRICTLY UNTOUCHED (READ-ONLY)** |

---

## 2. Classification of Reference Relevance for the New Project

### Category A: Required for Integration (Must Interface With)
- `/dbw/auto/cmd_steer` (`std_msgs/msg/Float64`) -> Front-wheel steering command input.
- `/dbw/auto/cmd_vel` (`std_msgs/msg/Float64`) -> Rear-wheel velocity command input.
- `/dbw/auto/brake` (`std_msgs/msg/Bool`) -> Service brake actuation command input.
- `/dbw/deadman_switch` (`std_msgs/msg/String`, `"{data: Auto}"`) -> Autonomy watchdog keep-alive input.
- `/dbw/shutdown` (`std_msgs/msg/Bool`) -> External emergency stop trigger.
- `/dbw/feedback/vehicle_speed` (`std_msgs/msg/Float64`) -> Ground speed feedback for odometry/stopping distance.
- `/dbw/feedback/current_steer` (`std_msgs/msg/Float64`) -> Steering angle feedback for closed-loop control.
- `/home/vat/yolo_vehicle_test/yolo26n-seg.pt` -> Neural network weights for instance segmentation.

### Category B: Useful for Understanding the Architecture
- `/home/vat/DBW/src/dbw_module/dbw_module_cpp/src/teleop_remap/teleop_remap.cpp` -> Provides exact reference for steer slew-rate limiting, centering timeout, and deadman gating.
- `/home/vat/dbw_ws/dbw_teleop.py` -> Confirms keyboard mapping and key-to-actuation conventions.
- `/home/vat/DBW/src/dbw_module/dbw_module_cpp/config/dbw_config.yaml` -> Contains vehicle parameter specifications.
- `/home/vat/yolo_vehicle_test/benchmark_yolo.py` -> Baseline benchmarking methodology for CPU inference.

### Category C: Existing Implementation That Must Remain Untouched
- All files in `/home/vat/DBW/` (C++ source, headers, message definitions, CMakeLists).
- All files in `/home/vat/dbw_ws/` and `/home/vat/dbw_ws_2/` (install, launch, configuration, and Python nodes).
- SocketCAN link configuration (`can0` at 500 kbps).
- Modbus TCP connection parameters (`192.168.0.5:502`).

### Category D: Not Relevant to This Project
- Cyberattack experiment scripts and dataset generation logs in `/home/vat/dbw_ws_2/` (`dos*.log`, `fuzzy*.log`, `speed_attack_dataset.sh`, `cantransfer_model.h5`, `tlsids_*`).
- Pallet position and lifting controls (`auto_lift_subscriber_`, `pallet_position_publisher_`).
- Trailer hitch status and beacon indicators (`/dbw/auto/beacon`, `/dbw/auto/lift`).
