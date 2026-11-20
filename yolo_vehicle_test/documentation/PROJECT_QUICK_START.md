# VIRYA DBW Buggy - Project Quick Start Guide

**Workspace:** `/home/vat/yolo_vehicle_test/`  
**Operational Modes:** Diagnostic, Perception-Only, Offline Mock, Passive Monitor, Hardware Staging  
**Target Hardware:** VIRYA Electric Buggy (Intel CPU x86_64, Logitech C270 Webcam, PEAK-System CAN, Ethernet PLC)  

---

## 1. Quick Environment Setup

Always navigate to the workspace and activate the dedicated virtual environment:

```bash
cd /home/vat/yolo_vehicle_test
source venv/bin/activate
```

Verify that the model weights and target classes are ready:
```bash
python3 -c "
from ultralytics import YOLO
m = YOLO('yolo26n-seg.pt')
print('Model loaded successfully.')
for cid in [0, 13, 56, 60]:
    print(f'Class {cid:02d}: {m.names[cid]}')
"
```

---

## 2. Integrated Orchestrator Commands (`start_multi_object_buggy.sh`)

The primary entry point for all operations is the unified launcher script:

### A. Non-Actuating Diagnostic Preflight
Checks Python environment, YOLO model, camera access, SocketCAN `can0`, PLC reachability, and ROS 2 Jazzy:
```bash
./start_multi_object_buggy.sh --diagnostic
# Shortcut:
./start_multi_object_buggy.sh -d
```

### B. Integrated Offline Simulation / Mock Mode (Default Safe Mode)
Runs the complete multi-object perception pipeline, decision engine, GUI dashboard, and 20 Hz vehicle control adapter in internal simulation (zero physical actuation commands sent to hardware):
```bash
./start_multi_object_buggy.sh --mock
# Or with webcam:
./start_multi_object_buggy.sh --mock --source 0
```

### C. Passive Hardware Monitoring Mode
Subscribes to live ROS 2 DBW feedback topics (`/dbw/feedback/current_steer`, `/dbw/feedback/vehicle_speed`, `/dbw/feedback/status`) and visualizes actual physical vehicle feedback without publishing any actuation commands:
```bash
./start_multi_object_buggy.sh --monitor --source 0
# Shortcut:
./start_multi_object_buggy.sh -o
```

### D. Supervised Hardware Staging Mode (Elevated Wheels Only!)
Requires the safety passkey `ACTUATE_VIRYA_CONFIRM`. Starts the DBW hardware stack, 20 Hz deadman heartbeat, and active ROS 2 bridge publishing to `/dbw/auto/*`:
```bash
./start_multi_object_buggy.sh --hardware --passkey ACTUATE_VIRYA_CONFIRM
# Shortcut:
./start_multi_object_buggy.sh -h -k ACTUATE_VIRYA_CONFIRM
```
> [!CAUTION]
> The vehicle must be placed on jack stands with all four wheels elevated before running hardware staging mode!

---

## 3. Direct Python Execution Commands

### A. Run Live Perception & Decision GUI
```bash
# Offline Mock Mode (default, safe):
python3 perception_gui.py --source 0 --mode mock

# Passive Monitor Mode (listens to real DBW feedback, zero actuation):
python3 perception_gui.py --source 0 --mode monitor

# Supervised Hardware Actuation Mode:
python3 perception_gui.py --source 0 --mode hardware --passkey ACTUATE_VIRYA_CONFIRM

# Recorded Benchmark Video:
python3 perception_gui.py --source benchmark_input.mp4 --mode mock

# Headless Test Mode:
python3 perception_gui.py --source benchmark_input.mp4 --max-frames 50 --headless --mode mock
```

### B. Run Automated Test Suites
```bash
# Automated Control Integration Test Suite (18 Scenarios):
python3 test_control_integration.py

# Complete 29-Item Automated Perception Verification Suite:
python3 test_comprehensive_suite.py

# 18-Scenario Navigation Evaluation:
python3 evaluate_scenarios.py

# Decision Engine Invariant Validation:
python3 validate_decision_engine.py

# Quantitative Class Validation (Person, Bench, Chair, Table):
python3 evaluate_classes.py
```

---

## 4. Key Troubleshooting Points

1. **Physical Vehicle Not Responding Under `--mock`:**
   - `--mock` intentionally simulates vehicle kinematics offline for safety.
   - To connect to physical hardware, ensure the vehicle is elevated on jack stands, run `./start_multi_object_buggy.sh -d` to verify CAN and PLC, then run with `--hardware --passkey ACTUATE_VIRYA_CONFIRM`.
2. **Competing Perception Node Warning:**
   - If `start_buggy.sh` was previously run, kill the competing legacy node:
     ```bash
     killall -9 yolo_obstacle_avoidance_node 2>/dev/null || true
     ```
3. **Camera Not Opening / Resource Busy:**
   - Ensure `v4l2_camera_node` is not holding `/dev/video0`.
   - Check with `fuser /dev/video0`.
4. **Steering Wheel Detected as "Chair" in Cockpit:**
   - If the webcam FOV captures the vehicle's own steering wheel, YOLO26n-seg may detect it as a chair (COCO Class 56).
   - Tilt the webcam upward or mount it ahead of the steering column to exclude the interior cockpit from the camera frame.
