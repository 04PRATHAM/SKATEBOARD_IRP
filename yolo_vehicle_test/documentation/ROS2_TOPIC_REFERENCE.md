# ROS 2 Topic & Interface Reference Guide

**Document ID:** RTR-VIRYA-2026-002  
**Target Platform:** VIRYA Drive-By-Wire Vehicle  
**ROS Distribution:** ROS 2 Jazzy Jalisco  
**Workspace:** `/home/vat/yolo_vehicle_test/`  
**Status:** Read-Only Technical Reference  

---

## 1. Overview

This document provides a comprehensive reference of all ROS 2 topics, message types, publisher nodes, subscriber nodes, QoS configurations, command meanings, and safety implications across the VIRYA DBW vehicle architecture.

Topics are categorized by operational criticality:
- **Control Topics (High Risk / Actuation):** Directly command physical actuators (steering, traction, braking).
- **Safety & Mode Topics (Critical):** Control the state machine, deadman watchdog, and emergency shutdown.
- **Feedback & Telemetry Topics (Safe / Observable):** Report physical vehicle state, sensor data, and diagnostics.
- **Internal Gateway Topics (Middleware):** Used internally between DBW C++ nodes and protocol drivers.

---

## 2. Master Topic Reference Table

| ROS 2 Topic Name | Message Type | Primary Publisher(s) | Primary Subscriber(s) | QoS Depth | Command Meaning & Value Range | Hardware Control Relevance | Safety & Risk Implication |
|---|---|---|---|---|---|---|---|
| `/dbw/auto/cmd_steer` | `std_msgs/msg/Float64` | `yolo_obstacle_avoidance_node` / Autonomous Planner | `dbw_node` (`auto_steering_subscriber_`) | 10 (Reliable) | Steering angle in **Degrees (°)**.<br>Range: `[-30.0, +30.0]`.<br>`> 0`: Steer **LEFT**.<br>`< 0`: Steer **RIGHT**.<br>`0.0`: Neutral Center. | **HIGH (Front Wheels)**: Directly drives Modbus steering actuator. | High risk. Commands must be smoothly ramped (max 30°/s) to avoid actuator stall or vehicle instability. |
| `/dbw/auto/cmd_vel` | `std_msgs/msg/Float64` | `yolo_obstacle_avoidance_node` / Autonomous Planner | `dbw_node` (`auto_speed_subscriber_`) | 10 (Reliable) | Target vehicle speed in **m/s**.<br>Range: `[-2.5, +2.5]`.<br>`> 0`: Forward drive.<br>`< 0`: Reverse drive.<br>`0.0`: Stop / Idle. | **HIGH (Rear Traction)**: Directly commands traction motor via CAN ID `0x12280797`. | High risk. Positive velocity moves vehicle. If velocity drops to 0, service brake is automatically asserted. Must publish continuously (<150 ms). |
| `/dbw/auto/brake` | `std_msgs/msg/Bool` | `yolo_obstacle_avoidance_node` / Safety Supervisor | `dbw_node` (`auto_brake_subscriber_`) | 10 (Reliable) | Service brake engagement.<br>`true`: Engage 100% brake (`0x64`).<br>`false`: Release service brake. | **HIGH (Brake Actuator)**: Directly commands vehicle braking system. | Critical safety. Setting `true` halts vehicle. Releasing requires explicit `false` and non-zero velocity command. |
| `/dbw/deadman_switch` | `std_msgs/msg/String` | Periodic Heartbeat Publisher / Autonomy Watchdog | `dbw_node` (`deadman_subscriber_`) | 10 (Reliable) | Mode enable string:<br>`"Auto"`: Transitions/maintains `AUTOMODE`.<br>`"Joy"`: Transitions to `JOYMODE`.<br>`"Teleop"`: Transitions to `TELEOPMODE`. | **CRITICAL (Master Enable)**: Without this signal, DBW reverts to `IDLE` and rejects all motion. | Critical safety watchdog. Must publish at $\ge 5\text{ Hz}$ (timeout is 200 ms). If it stops, vehicle immediately stops and locks brakes. |
| `/dbw/shutdown` | `std_msgs/msg/Bool` | `diagnostics_node` (`shutdown_publisher_`) / Safety Node | `dbw_node` (`safety_subscriber_`) | 10 (Reliable) | Emergency system shutdown.<br>`true`: Immediately triggers `FAULT`, sets speed=0, steer=0, brake=1. | **CRITICAL (Emergency Stop)**: Overrides all active control inputs. | Highest safety tier. Can be published by external safety nodes to force vehicle emergency stop. |
| `/dbw/feedback/current_steer` | `std_msgs/msg/Float64` | `dbw_node` (`current_steer_publisher_`) | Teleop / HMI / Planners | 10 (Reliable) | Actual measured front-wheel steering angle in **Degrees (°)** (from Modbus Register 0 / 10.0). | **Feedback Only (Safe)**: Non-actuating telemetry. | Essential for closed-loop steering control and path tracking. |
| `/dbw/feedback/vehicle_speed` | `std_msgs/msg/Float64` | `can_reader.py` / `dbw_node` | Teleop / HMI / Planners | 10 (Reliable) | Actual vehicle ground speed in **m/s** (derived from CAN ID `0x12910109` Bytes 6-7). | **Feedback Only (Safe)**: Non-actuating telemetry. | Essential for velocity feedback, odometry, and stopping distance calculations. |
| `/dbw/feedback/brake_status` | `std_msgs/msg/Int32MultiArray` | `can_reader.py` (`brake_status_publisher_`) | HMI / Diagnostics | 10 (Reliable) | Array: `[Park_Brake, Emergency_Stop, brake_actuator_pose, LLC_Stop]`. | **Feedback Only (Safe)**: Non-actuating telemetry. | Verifies whether physical brake calipers/actuator are engaged. |
| `/dbw/feedback/mode` | `std_msgs/msg/Int32` | `can_reader.py` (`mode_publsisher_`) | HMI / Diagnostics | 10 (Reliable) | Vehicle hardware mode:<br>`1`: Autonomous (DBW Auto).<br>`0`: Manual override / disengaged. | **Feedback Only (Safe)**: Non-actuating telemetry. | Critical. If mode is `0`, physical manual driver has overridden DBW or key switch is manual. |
| `/dbw/feedback/status` | `std_msgs/msg/Int32` | `dbw_node` (`dbw_status_publisher_`) | HMI / Diagnostics | 10 (Reliable) | FSM State integer code:<br>`0`: IDLE<br>`1`: JOYMODE<br>`2`: AUTOMODE<br>`3`: TELEOPMODE<br>`4`: FAULT | **Feedback Only (Safe)**: Non-actuating telemetry. | Confirms FSM state. Autonomy logic must verify state is `2` (`AUTOMODE`). |
| `/dbw/feedback/battery_soc` | `std_msgs/msg/Int32` | `can_reader.py` (`battery_soc_publisher_`) | HMI / Diagnostics | 10 (Reliable) | Traction battery State of Charge in percent (`0 - 100%`). | **Feedback Only (Safe)**: Non-actuating telemetry. | Battery monitoring. |
| `/dbw/feedback/manual_brake` | `std_msgs/msg/Int32` | `can_reader.py` (`manual_brake_publisher_`) | HMI / Diagnostics | 10 (Reliable) | Driver physical brake pedal displacement percentage (`0 - 100%`). | **Feedback Only (Safe)**: Non-actuating telemetry. | Used to detect human manual intervention. |
| `/dbw/diagnostics` | `std_msgs/msg/String` | `dbw_node`, `can_writer_node`, `mod_writer_node`, `diagnostics_node` | System Logger / HMI | 10 (Reliable) | Plaintext diagnostic error and warning strings. | **Feedback Only (Safe)**: System status monitoring. | Invaluable for real-time debugging of bus faults or timeout events. |
| `/dbw/heartbeat` | `heartbeat_msgs/msg/Heartbeat` | `dbw_node`, `can_writer_node`, `mod_writer_node` | `diagnostics_node` (`heartbeat_subscriber_`) | 100 (Reliable) | Heartbeat message containing `node_name`, `heartbeat_count`, `timestamp`. | **Internal Middleware (Safe)** | Tracks node liveness. Failure triggers `/dbw/shutdown`. |
| `/dbw/canbus_publish` | `can_write_msgs/msg/CanRequest` | `dbw_node` (`can_publisher_`) | `can_writer_node` (`can_subscriber_`) | 10 (Reliable) | Internal request: `header`, `id`, `velocity` (cm/s), `brake` (0 or 1). | **Internal (Actuation Bridge)** | Intermediate representation prior to SocketCAN transmission. |
| `/dbw/modbus_publish` | `mod_write_msgs/msg/ModRequest` | `dbw_node` (`mod_publisher_`) | `mod_writer_node` (`modbus_subscriber_`) | 10 (Reliable) | Internal request: `steer_angle`, `speed`, `brake`, `indicator`, `horn`, `emergency_status`. | **Internal (Actuation Bridge)** | Intermediate representation prior to Modbus register writes. |
| `/dbw/canbus_receive` | `can_msgs/msg/Frame` | `can_writer_node` (`can_receive_publisher_`) | `dbw_node` (`canbus_receive_subscriber_`) | 10 (Reliable) | Raw 8-byte CAN frame received from CAN ID `0x12910109`. | **Internal (Sensor Bridge)** | Used by DBWNode to parse CCU auto mode bit and emergency stop bit. |
| `/dbw/modread/registers` | `mod_read_msgs/msg/Registers` | `mod_writer_node` (`registers_publisher_`) | `dbw_node` (`registers_receive_subscriber_`) | 10 (Reliable) | Modbus registers: `register0` (steer), `register2` (PWM), `register4`. | **Internal (Sensor Bridge)** | Used by DBWNode to formulate `/dbw/feedback/current_steer`. |
| `/image_raw` | `sensor_msgs/msg/Image` | `v4l2_camera_node` | `yolo_obstacle_avoidance_node` | 10 (SensorData / Volatile) | Monocular 640x480 RGB image stream from Logitech C270 webcam. | **Sensor Stream (Safe)** | Current visual input. To be replaced by RGB-D streams. |
| `/camera_info` | `sensor_msgs/msg/CameraInfo` | `v4l2_camera_node` | Visual Odometry / Perception | 10 (SensorData / Volatile) | Camera intrinsic calibration matrix ($K, D, R, P$). Currently uncalibrated defaults. | **Sensor Stream (Safe)** | Required for 3D projection and spatial coordinate estimation. |

---

## 3. Topic Safety Categorization

```
                  =============================================
                  CLASSIFICATION OF ROS 2 TOPICS BY SAFETY RISK
                  =============================================

  [ DANGER: DIRECT ACTUATION - PUBLISHING CAUSES VEHICLE MOTION ]
  * /dbw/auto/cmd_vel        -> Controls traction motor speed (CAN ID 0x12280797)
  * /dbw/auto/cmd_steer      -> Controls front steering motor (Modbus Register 3)
  * /dbw/auto/brake          -> Actuates pneumatic/mechanical service brake
  * /dbw/canbus_publish      -> Low-level CAN write request
  * /dbw/modbus_publish      -> Low-level Modbus write request

  [ CRITICAL SAFETY INTERLOCKS - DIRECTLY GOVERN VEHICLE STATE ]
  * /dbw/deadman_switch      -> Required 20 Hz heartbeat ("Auto") to enable DBW
  * /dbw/shutdown            -> Emergency stop command (True triggers FAULT)

  [ SAFE TELEMETRY - PASSIVE MONITORING ONLY ]
  * /dbw/feedback/current_steer  -> Real-time steering feedback (degrees)
  * /dbw/feedback/vehicle_speed  -> Real-time ground speed feedback (m/s)
  * /dbw/feedback/brake_status   -> Physical brake state flags
  * /dbw/feedback/mode           -> DBW Auto vs Manual switch status
  * /dbw/feedback/status         -> FSM state (0:IDLE, 2:AUTOMODE, 4:FAULT)
  * /dbw/feedback/battery_soc    -> Battery percentage
  * /dbw/feedback/manual_brake   -> Driver pedal percentage
  * /dbw/diagnostics             -> Textual fault strings
  * /image_raw                   -> Visual camera stream
  * /camera_info                 -> Camera calibration metadata
```

---

## 4. QoS (Quality of Service) Configurations

Based on static inspection of `/home/vat/DBW/src/dbw_module/dbw_module_cpp/src/`:
- **Command & Telemetry Topics:** All DBW command subscriptions and feedback publishers utilize standard ROS 2 `create_subscription` and `create_publisher` calls with an explicit queue depth of `10`:
  - Reliability: `RMW_QOS_POLICY_RELIABILITY_RELIABLE`
  - Durability: `RMW_QOS_POLICY_DURABILITY_VOLATILE`
  - History: `KEEP_LAST` with depth `10`.
- **Heartbeat Topics:** `/dbw/heartbeat` utilizes queue depth `100` to prevent dropouts during multi-node bursts.
- **Camera Topic (`/image_raw`):** Standard `v4l2_camera_node` publishes using ROS 2 SensorData profile (`BEST_EFFORT` reliability, depth `10`) or `RELIABLE` depth `10` depending on driver parameters. Subscriptions in Python (`CvBridge`) default to queue depth `10`.
