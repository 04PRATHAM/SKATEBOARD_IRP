# VIRYA DBW Buggy - Project Development Changelog

All modifications and developments are strictly recorded inside `/home/vat/yolo_vehicle_test/`.

---

## [Phase 6] - Live Webcam GUI Verification & Master Technical Documentation
**Date:** 2026-11-06  
**Status:** Completed & Verified  

### Added:
- **`perception_gui.py`:** Live multi-threaded GUI dashboard featuring live camera feed, detected objects table, decision banner, 2D corridor map, steering indicator, braking panel, performance metrics bar, and scrolling event log.
- **`test_comprehensive_suite.py`:** Comprehensive 29-item automated test runner validating all classes, tracking, spatial layouts, decision states, GUI components, and camera lifecycle.
- **`documentation/COMPLETE_MASTER_PROJECT_REPORT.md`:** 22-chapter master technical report serving as the permanent engineering reference for the entire vehicle perception project.
- **`documentation/PROJECT_QUICK_START.md`:** Operator quick-start manual with verified run commands.
- **`documentation/PROJECT_CURRENT_STATUS.md`:** Verified subsystem status matrix and limitations breakdown.
- **`documentation/PROJECT_CHANGELOG.md`:** Chronological engineering history.
- **`results/evaluation/live_webcam_gui_snapshot.jpg`:** Verified snapshot of the live webcam GUI dashboard running on `/dev/video0`.
- **`results/evaluation/live_webcam_gui_test.mp4`:** Verified video recording of the live webcam GUI dashboard.
- **`results/evaluation/comprehensive_test_report.json`:** Structured results of the 29-item automated test suite (100% pass rate).

### Fixed / Corrected:
- **V4L2 Camera Double-Open Contention Bug in GUI:**
  - *Root Cause:* In earlier iterations, `PerceptionWorker` initialized `IntegratedPerceptionPipeline` (which opened `/dev/video0` with `CAP_V4L2`) and then called `self.cap = cv2.VideoCapture(0)` on the next line. This second open failed due to Linux V4L2 exclusive device locking, preventing live frames from being acquired.
  - *Fix:* `PerceptionWorker` now directly reuses `self.cap = self.pipeline.cap`. Redundant device handles eliminated. Live frames stream continuously into the GUI queue.
- **OpenCV Hershey ASCII Font Cleanup:**
  - Replaced non-ASCII UTF-8 warning characters (`⚠️`) in badges and event logs with clean ASCII indicators (`[!]` and `[WARN]`), resolving `??` rendering artifacts.
- **OpenCV Point Tuple Casting:**
  - Explicitly cast all floating-point coordinates in `cv2.arrowedLine` within the 2D corridor map to integer pairs `(int(x), int(y))`, preventing overload resolution failures.

---

## [Phase 5] - Integrated Perception Pipeline & Scenario Validation
**Date:** 2026-11-06  
**Status:** Completed & Verified  

### Added:
- **`run_perception_pipeline.py`:** Unified single-pass perception and decision engine combining camera capture, `yolo26n-seg`, ByteTrack, corridor analysis, and HUD rendering.
- **`evaluate_classes.py`:** Quantitative evaluation script across 279 empirical frames (Person: 229, Chair: 1066, Dining Table: 248, Bench: 0).
- **`evaluate_scenarios.py`:** Comprehensive 18-scenario testing harness (Scenarios A through R).
- **`validate_decision_engine.py`:** Decision invariant validation script checking all 6 decision states and 7 safety rules.
- **`documentation/PHASE_5_VALIDATION_REPORT.md`:** Detailed technical report on Phase 5 execution.

### Fixed / Corrected:
- **Ultralytics Configuration Permission Bug:**
  - Fixed `[Errno 30] Read-only file system: /home/vat/.config/Ultralytics/settings.json` by exporting `os.environ["YOLO_CONFIG_DIR"] = "/home/vat/yolo_vehicle_test/.config/Ultralytics"`.
- **Empty-Frame Clear Path Hysteresis:**
  - Fixed safety flaw in empty-frame handling: single dropped frames now return `SLOW_DOWN (Verifying path)`; 3 consecutive clear frames required before `PATH_CLEAR`.
- **Dual-Side Corridor Pinch Protection:**
  - Prevented hazardous steering when obstacles occupy both left and right corridor halves by commanding `STOP`.

---

## [Phase 1 to Phase 4] - Architecture Audit & Milestone Prototypes
**Date:** 2026-11-02  
**Status:** Completed  

### Added:
- **`webcam_detection.py`:** Standalone camera capture and YOLO26n-seg instance segmentation.
- **`object_tracking.py`:** Standalone ByteTrack multi-object tracking.
- **`obstacle_positioning.py`:** 2D normalized corridor occupancy calculation prototype.
- **`obstacle_decision_engine.py`:** Rule-based decision engine prototype.
- **`config/object_classes.yaml`:** Target object configurations and color codes.
- **`config/detection_config.yaml`:** Camera and corridor parameters.
- **`documentation/VEHICLE_ARCHITECTURE_REPORT.md`:** Exhaustive read-only investigation of VIRYA DBW vehicle architecture.
- **`documentation/VEHICLE_CONTROL_REFERENCE.md`:** Modbus TCP and SocketCAN register specifications.
