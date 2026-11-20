# VIRYA Buggy RGB-D Perception & Multi-Object Avoidance Project

**Project Location:** `/home/vat/yolo_vehicle_test/`  
**Current Phase:** Phase 1–4 (Offline Multi-Object Perception, Tracking, Positioning & Decision Engine)  
**Host Architecture:** Ubuntu Linux x86_64, ROS 2 Jazzy, CPU Inference (No GPU)  
**Safety Status:** OFFLINE / VEHICLE DISCONNECTED (Zero Actuation / Zero CAN / Zero Modbus)  

---

## 1. Project Overview

This project develops an advanced multi-object perception and obstacle-avoidance system for the VIRYA Drive-By-Wire (DBW) buggy. In the current development stage, development is conducted strictly offline using the onboard USB webcam and the `yolo26n-seg.pt` instance segmentation model.

Physical vehicle controls, CAN communication, and Modbus steering actuation remain completely disconnected and untouched during this phase.

---

## 2. Directory Layout

```
/home/vat/yolo_vehicle_test/
├── config/
│   ├── object_classes.yaml       <-- Target class IDs (0, 13, 56, 60), colors, priorities
│   └── detection_config.yaml     <-- Camera parameters, model settings, corridor boundaries
├── datasets/
│   └── laboratory_images/        <-- Captured test frames for offline validation
│       ├── lab_sample_01.jpg
│       └── lab_sample_02.jpg
├── results/
│   ├── detection/                <-- Output detection videos and snapshot frames
│   │   ├── webcam_test.mp4
│   │   └── snapshot_0030.jpg
│   ├── tracking/                 <-- Output multi-object tracking videos and snapshots
│   │   ├── webcam_tracking_test.mp4
│   │   └── track_snap_0030.jpg
│   └── evaluation/               <-- Output decision engine HUD videos
│       └── decision_engine_test.mp4
├── logs/
│   └── decision_log.jsonl        <-- Real-time decision and telemetry logs
├── documentation/                <-- Architectural baseline reports from previous phase
├── venv/                         <-- Python 3.12 virtual environment (PyTorch + Ultralytics)
├── yolo26n-seg.pt                <-- Active YOLO26 Nano segmentation weights
├── yolo26s-seg.pt                <-- Alternative Small model weights
├── webcam_detection.py           <-- Phase 1: Standalone webcam segmentation app
├── object_tracking.py            <-- Phase 2: Multi-object ByteTrack tracking engine
├── obstacle_positioning.py       <-- Phase 3: Spatial positioning & corridor analysis
├── obstacle_decision_engine.py   <-- Phase 4: Offline safety reasoning & decision engine
└── README_PROJECT_STATUS.md      <-- Project status and execution documentation
```

---

## 3. Verified Target Object Categories

Verified directly against `yolo26n-seg.pt` (`model.names`):
1. **Person:** Class ID `0` (Safety Priority 1 - Highest)
2. **Bench:** Class ID `13` (Safety Priority 2)
3. **Chair:** Class ID `56` (Safety Priority 2)
4. **Dining Table:** Class ID `60` (Safety Priority 2)

All other object categories are filtered out to conserve CPU computation and avoid extraneous detections.

---

## 4. Execution Commands

All scripts must be executed using the project's local virtual environment:

### Phase 1: Standalone Detection & Segmentation
```bash
/home/vat/yolo_vehicle_test/venv/bin/python3 /home/vat/yolo_vehicle_test/webcam_detection.py
```
*Options:*
- `--camera 0`: Device index (`/dev/video0`)
- `--conf 0.25`: Confidence threshold
- `--headless`: Run without GUI window (auto-detected if `$DISPLAY` is absent)
- `--save-video results/detection/run.mp4`: Record output stream
- `--save-snapshots`: Periodically write JPEG snapshots

### Phase 2: Multi-Object Tracking (ByteTrack)
```bash
/home/vat/yolo_vehicle_test/venv/bin/python3 /home/vat/yolo_vehicle_test/object_tracking.py
```

### Phase 3: Obstacle Positioning & Driving Corridor
```bash
/home/vat/yolo_vehicle_test/venv/bin/python3 /home/vat/yolo_vehicle_test/obstacle_positioning.py \
    --corridor-left 0.25 --corridor-right 0.75
```

### Phase 4: Decision Engine HUD (Offline)
```bash
/home/vat/yolo_vehicle_test/venv/bin/python3 /home/vat/yolo_vehicle_test/obstacle_decision_engine.py
```

---

## 5. Performance Benchmarks on Vehicle Hardware (CPU Inference)

| Metric | Phase 1 (Detection) | Phase 2 (Tracking) | Phase 3 (Positioning) | Phase 4 (Decision Engine) |
|---|---|---|---|---|
| **Median Inference Latency** | 92.16 ms | 114.04 ms | 111.40 ms | 99.60 ms |
| **Mean Inference Latency** | 146.07 ms | 170.78 ms | 158.20 ms | 134.50 ms |
| **P95 Latency** | 179.47 ms | 274.78 ms | 246.40 ms | 202.00 ms |
| **Pipeline Throughput** | ~8–11 FPS | ~7–9 FPS | ~7–9 FPS | ~7–10 FPS |
| **Objects Tracked** | Multiple | Up to 5 concurrent | Up to 5 concurrent | Up to 5 concurrent |

---

## 6. Safety Compliance Audit

- **Files outside `/home/vat/yolo_vehicle_test/` modified:** **NONE** (Zero modifications).
- **Existing vehicle DBW code modified:** **NONE** (All reference code strictly read-only).
- **Vehicle controls published:** **NONE** (Zero ROS 2 publish commands).
- **CAN / Modbus transmissions:** **NONE** (No packets sent).
- **Physical vehicle movement:** **NONE** (Vehicle completely stationary).
