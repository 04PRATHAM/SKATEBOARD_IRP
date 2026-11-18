# Vehicle Control Reference Manual: VIRYA DBW Platform

**Document ID:** VCR-VIRYA-2026-003  
**Target Vehicle:** VIRYA Electric Drive-By-Wire Platform  
**Target Subsystems:** Steering, Traction, Braking, Deadman, FSM State Machine  
**Workspace:** `/home/vat/yolo_vehicle_test/`  
**Status:** Read-Only Technical Control Specification  

---

## 1. Front-Wheel Steering Control System

### 1.1 Physical Actuator & Communication Mechanism
The vehicle's front wheels are steered by an electric steering servo/actuator managed by an industrial controller (PLC). Rather than CAN bus, the steering subsystem communicates exclusively via **Modbus TCP** over Ethernet:
- **IP Address:** `192.168.0.5`
- **Port:** `502`
- **Modbus Protocol:** Modbus TCP (libmodbus)
- **Slave ID:** `0x0001` (Holding Registers)

### 1.2 Steering Angle Representation & Sign Convention
Authoritative source code evidence confirms the following mathematical and physical steering conventions:
- **Control Units:** Physical degrees of steering angle ($^\circ$).
- **Neutral Steering:** `0.0` degrees indicates the wheels are perfectly straight.
- **Left Turn Convention:** Positive values (`+`, $> 0.0^\circ$) command front wheels to turn **LEFT**.
  - *Evidence 1:* `/home/vat/dbw_ws_2/src/my_package/my_package/yolo_obstacle_avoidance_node.py` lines 97-101:
    ```python
    elif obstacle_position == "right":
        self.get_logger().info("STEER LEFT")
        self.steer_pub.publish(Float64(data=10.0))
    ```
  - *Evidence 2:* `/home/vat/dbw_ws/dbw_teleop.py` lines 133-134:
    ```python
    elif key == 'a':
        node.current_steer += STEER_STEP   # LEFT
    ```
- **Right Turn Convention:** Negative values (`-`, $< 0.0^\circ$) command front wheels to turn **RIGHT**.
  - *Evidence 1:* `/home/vat/dbw_ws_2/src/my_package/my_package/yolo_obstacle_avoidance_node.py` lines 87-94:
    ```python
    elif obstacle_position == "left":
        self.get_logger().info("STEER RIGHT")
        self.steer_pub.publish(Float64(data=-10.0))
    ```
  - *Evidence 2:* `/home/vat/dbw_ws/dbw_teleop.py` lines 136-137:
    ```python
    elif key == 'd':
        node.current_steer -= STEER_STEP   # RIGHT
    ```
- **Maximum Limits:**
  - Absolute software clamp in `dbw.cpp` (line 3): `steer_limit_ = 30.0` degrees.
  - Symmetrical range: `[-30.0°, +30.0°]`.

### 1.3 Modbus Register Encoding & Conversion
In `/home/vat/DBW/src/dbw_module/dbw_module_cpp/src/mod_writer/mod_writer.cpp` (lines 117-126):
1. The floating-point steering angle is scaled by a factor of $10$ to yield integer tenths of a degree:
   $$\text{steer\_angle\_int} = \text{static\_cast<int16\_t>}(\text{angle} \times 10)$$
2. If the commanded angle is non-negative ($\ge 0$), it is written directly to **Register 3** (`Utils::RegisterId::STEER_WRITE_REGISTER`):
   ```cpp
   steer_flag = modbus_write_register(ctx_, Utils::RegisterId::STEER_WRITE_REGISTER, steer_angle_int);
   ```
3. If the commanded angle is negative ($< 0$), it is written as a 16-bit Two's Complement representation:
   ```cpp
   int twosComplement = (1 << 16) + steer_angle_int;
   steer_flag = modbus_write_register(ctx_, Utils::RegisterId::STEER_WRITE_REGISTER, twosComplement);
   ```

### 1.4 Real-Time Steering Feedback
In `/home/vat/DBW/src/dbw_module/dbw_module_cpp/src/mod_writer/mod_writer.cpp` (lines 214-234) and `dbw.cpp` (lines 268-283):
1. `ModWriter` polls Modbus Holding **Register 0** (`STEER_READ_REGISTER`) every 50 ms.
2. Raw 16-bit register value is published to `/dbw/modread/registers`.
3. `DBWNode` converts raw integer back to floating-point degrees:
   $$\text{current\_steer\_degrees} = \frac{\text{static\_cast<int16\_t>}(\text{register0})}{10.0}$$
4. Published on topic: `/dbw/feedback/current_steer` (`std_msgs/msg/Float64`).

### 1.5 Actuator Slew Rate Limitation & Dynamics
As documented in `/home/vat/DBW/src/dbw_module/dbw_module_cpp/src/teleop_remap/teleop_remap.cpp` (lines 157-160):
> *"The published steering angle is slew-limited to steer_rate deg/s rather than stepped to the target, because the steering actuator will not accept a jump - it only tracks a command that walks there."*
- **Slew Rate Parameter:** `steer_rate = 30.0` deg/s (`config/dbw_config.yaml`).
- **Autonomy Requirement:** Autonomous path-tracking algorithms must publish continuous, smooth trajectory profiles (e.g. at 20–50 Hz) rather than discrete step changes.

---

## 2. Rear-Wheel Traction & Acceleration Control System

### 2.1 Physical Actuator & Communication Mechanism
The vehicle's traction drive system consists of an electric motor driving the rear axle, controlled by the vehicle Central Control Unit (CCU) via **SocketCAN**:
- **Interface:** `can0`
- **Bitrate:** 500,000 bit/s (500 kbps)
- **CAN Frame Type:** Classic CAN 2.0B Extended Frame (29-bit identifier)
- **Sender CAN ID:** `0x12280797`
- **Feedback CAN ID:** `0x12910109`

### 2.2 Velocity Command Mapping & Units
- **ROS 2 Topic:** `/dbw/auto/cmd_vel` (`std_msgs/msg/Float64`).
- **Units:** Linear velocity in meters per second ($\text{m/s}$).
- **Range & Clamping:**
  In `dbw.cpp` (line 4 & lines 112-114):
  $$\text{velocity\_limit\_} = 2.5\text{ m/s}$$
  Commands outside `[-2.5, +2.5]` m/s are hard-clamped.
- **Directional Modes:**
  - $v > 0$: Forward motion. CAN Byte 0 = `0x01`.
  - $v < 0$: Reverse motion. CAN Byte 0 = `0x03`.
  - $v = 0$: Stopped / Neutral. CAN Byte 0 = `0x00`.
- **Speed Magnitude Encoding:**
  In `dbw.cpp` line 377:
  $$\text{can\_message->velocity} = \text{velocity\_} \times 100$$
  In `can_writer.cpp` lines 71-78:
  $$\text{speed\_c\_value} = \text{static\_cast<int16\_t>}(\text{round}(\text{abs}(\text{velocity})))$$
  The speed in integer centimeters per second ($\text{cm/s}$) is mapped into:
  - CAN Byte 1: Least Significant Byte (`speed_c_value & 0xFF`)
  - CAN Byte 2: Most Significant Byte (`(speed_c_value >> 8) & 0xFF`)

### 2.3 CAN Transmit Frame Specification (CAN ID: `0x12280797`)

| Byte Index | Field Name | Type | Value / Bit Mask | Description |
|---|---|---|---|---|
| **Byte 0** | Drive Mode Command | `uint8_t` | `0x01`: Forward<br>`0x03`: Reverse<br>`0x00`: Neutral/Stop | Commands CCU drive direction. |
| **Byte 1** | Velocity LSB | `uint8_t` | `speed_c_value & 0xFF` | Low byte of commanded speed in cm/s. |
| **Byte 2** | Velocity MSB | `uint8_t` | `(speed_c_value >> 8) & 0xFF` | High byte of commanded speed in cm/s. |
| **Byte 3** | Service Brake Command | `uint8_t` | `0x64`: Brake 100%<br>`0x00`: Brake Released | Commands physical brake actuator. |
| **Byte 4** | Auto Park Brake | `uint8_t` | `0x01` during emergency stop; otherwise `0x00` | Controls electric parking brake. |
| **Byte 5** | Reserved | `uint8_t` | `0x00` | Unused. |
| **Byte 6** | Reserved | `uint8_t` | `0x00` | Unused. |
| **Byte 7** | CCU Heartbeat | `uint8_t` | `0x01` | Active heartbeat marker sent to CCU. |

### 2.4 Ground Speed Feedback
In `scripts/can_reader.py` (lines 88-95) and `dbw.cpp` (lines 260-264):
- The CCU broadcasts vehicle telemetry on CAN ID `0x12910109`.
- Bytes 6 & 7 encode real-time ground wheel speed:
  $$\text{raw\_speed} = (\text{Byte 7} \ll 8) \mid \text{Byte 6} \quad (\text{signed int16})$$
  $$\text{ground\_speed} = \frac{\text{raw\_speed}}{100.0}\text{ m/s}$$
- Published to: `/dbw/feedback/vehicle_speed` (`std_msgs/msg/Float64`).

---

## 3. Braking System & Actuation

### 3.1 Braking Command Interface
- **ROS 2 Topic:** `/dbw/auto/brake` (`std_msgs/msg/Bool`).
- **Boolean Meaning:**
  - `data: true`: Actuates service brake at full pressure ($100\%$ / `0x64`).
  - `data: false`: Releases service brake.

### 3.2 Dual Braking Paths (CAN and Modbus)
When a brake command is processed by `DBWNode`:
1. **CAN Brake:** Transmitted via CAN ID `0x12280797` Byte 3 (`0x64` = 100% brake).
2. **Modbus Brake:** Written to Modbus Coil 4 (`Utils::CoilId::BRAKE`).

### 3.3 Automatic Brake Triggers & Interlocks
The vehicle incorporates mandatory automatic braking logic embedded in `DBWNode::send_can_message()` (`dbw.cpp` lines 373-376):
```cpp
int brake = coils_.brake;
if (velocity_ == 0 || emergency_stop_){
    brake = 1; // Set brake to 1 if velocity is 0
}
```
Automatic braking is triggered when:
1. **Zero Velocity Commanded:** Whenever `velocity_ == 0`, the service brake is automatically locked to prevent roll-away.
2. **Speed Command Staleness:** If no fresh message is received on `/dbw/auto/cmd_vel` for $> 150$ ms, `velocity_` is forced to `0.0`, immediately triggering `brake = 1`.
3. **Deadman Release:** If `/dbw/deadman_switch` drops for $> 200$ ms, state reverts to `IDLE`, velocity is forced to `0.0`, and brake is locked.
4. **Shutdown Signal:** Receiving `data: true` on `/dbw/shutdown` explicitly forces `coils_.brake = 1` and `velocity_ = 0.0`.
5. **Hardware Emergency Stop:** If CAN ID `0x12910109` Byte 4 Bit 4 is asserted, `emergency_stop_ = true`, triggering immediate full braking.

---

## 4. Deadman Switch & FSM State Machine

### 4.1 Deadman Switch Protocol
- **ROS 2 Topic:** `/dbw/deadman_switch` (`std_msgs/msg/String`).
- **Required Payload:** String literal `"{data: Auto}"`.
- **Publishing Frequency:** Minimum $5$ Hz ($\le 200$ ms interval); recommended **$20$ Hz** (every $50$ ms).
- **Timeout Threshold:** Defined in `utils.hpp` line 11:
  $$\text{Utils::DEADMAN\_TIMEOUT} = 200\text{ ms}$$
- **Timeout Consequence:** If no valid deadman message arrives within 200 ms:
  - FSM transitions from `AUTOMODE` to `IDLE`.
  - `velocity_` drops to `0.0`.
  - Service brake engages (`brake = 1`).
  - Modbus steer command defaults to straight (`0.0°`).

### 4.2 Critical Control Coupling Warning
In `dbw.cpp` lines 395-426, the Modbus steering loop checks the timestamp of the **speed command** (`speed_receive_time_`):
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
> The front-wheel steering actuator relies on a live velocity heartbeat! If `/dbw/auto/cmd_vel` is not published for $> 120$ ms, the steering angle will **snap to 0 degrees** even if steering commands are arriving fresh on `/dbw/auto/cmd_steer`.  
> Both `/dbw/auto/cmd_vel` and `/dbw/auto/cmd_steer` MUST be published continuously in tandem at $\ge 20$ Hz!

---

## 5. Summary of Authoritative Source Code References

| Control Functionality | Authoritative File Path | Key Class / Method | Line Numbers | Hardware Comm? |
|---|---|---|---|---|
| Steering Limits & Clamp | `/home/vat/DBW/src/dbw_module/dbw_module_cpp/src/dbw/dbw.cpp` | `DBWNode::auto_steer_callback` | Lines 121–131 | No (ROS 2 state) |
| Steering Modbus Serialization | `/home/vat/DBW/src/dbw_module/dbw_module_cpp/src/mod_writer/mod_writer.cpp` | `ModWriter::write_registers` | Lines 104–137 | **Yes** (Modbus TCP 192.168.0.5) |
| Steering Encoder Feedback | `/home/vat/DBW/src/dbw_module/dbw_module_cpp/src/dbw/dbw.cpp` | `DBWNode::modbus_receive_callback` | Lines 268–283 | **Yes** (Read Register 0) |
| Speed Limits & Clamp | `/home/vat/DBW/src/dbw_module/dbw_module_cpp/src/dbw/dbw.cpp` | `DBWNode::auto_speed_callback` | Lines 107–119 | No (ROS 2 state) |
| Traction CAN Serialization | `/home/vat/DBW/src/dbw_module/dbw_module_cpp/src/can_writer/can_writer.cpp` | `CanWriter::can_callback` | Lines 65–90 | **Yes** (SocketCAN can0) |
| Speed Encoder Feedback | `/home/vat/DBW/src/dbw_module/dbw_module_cpp/scripts/can_reader.py` | `SteeringAnglePublisher::candump_reader_thread` | Lines 87–95 | **Yes** (SocketCAN can0) |
| Brake Automatic Actuation | `/home/vat/DBW/src/dbw_module/dbw_module_cpp/src/dbw/dbw.cpp` | `DBWNode::send_can_message` | Lines 363–380 | **Yes** (CAN ID 0x12280797) |
| Deadman Switch FSM Transition | `/home/vat/DBW/src/dbw_module/dbw_module_cpp/src/dbw/dbw.cpp` | `DBWNode::deadman_callback` | Lines 296–317 | No (ROS 2 state) |
| Deadman Watchdog Timeout | `/home/vat/DBW/src/dbw_module/dbw_module_cpp/src/dbw/dbw.cpp` | `DBWNode::check_deadman_level` | Lines 328–351 | No (FSM state guard) |
| Emergency System Shutdown | `/home/vat/DBW/src/dbw_module/dbw_module_cpp/src/dbw/dbw.cpp` | `DBWNode::shutdown_callback` | Lines 178–189 | **Yes** (FSM FAULT & Brake 1) |
