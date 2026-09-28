# RGB-D Model Interface (SKATEBOARD_IRP — Phase 8, 2026-09-25)

Status: interface VERIFIED on live data; final RGB model NOT selected (intentionally open).
Rule (Phase 9): perception output is NOT connected to steering yet.

## 1. Model type
TBD — candidates are YOLOv8/DETR-class 2D detectors. Selection requires real
vehicle-camera images (domain gap) + benchmark on 640x480 synthetic data.
No YOLO/DETR/torch/onnx dependency exists in the repo today (verified by grep).

## 2. Model input format
- `sensor_msgs/Image`, encoding `rgb8`, 640x480 — topic `/camera/rgb/image_raw`
- frame_id `camera_rgb_optical_frame`, sim-time stamps (sync to depth: measured dt=0.0000 s)
- plus `sensor_msgs/CameraInfo` `/camera/rgb/camera_info` (plumb_bob; renderer forces square pixels, fx=fy≈575.66)

## 3. Model output format (`Detection2D`, in `isaac_sim/scripts/perception/rgbd_interface.py`)
- `bbox`: (x_min, y_min, x_max, y_max) in pixels
- `class_name`: str, current contract `"person"`
- `confidence`: float 0..1
- `object_id`: int

## 4. Detected classes
`person` (contract today; `class_name` free-form for extension).

## 5. Bounding-box representation
Pixel corners, origin top-left, inclusive range within 640x480.

## 6. Confidence representation
Float 0..1 per detection (`Detection2D.confidence`).

## 7. Expected image size
640x480 RGB. Detector MUST accept exactly this (no resize contract yet).

## 8. Inference framework
TBD with model choice. The interface itself is framework-free (pure Python +
numpy). A ROS wrapper node will be added at integration time; it must implement:
`RGBDetectorBase.detect(rgb_image, camera_info) -> List[Detection2D]`
without touching vehicle/camera/ROS/nav code (plug-in point, verified importable standalone).

## 9. CPU/GPU requirements
TBD with model choice. Machine: RTX PRO 6000 96 GB. CPU-only fallback is a
selection criterion (not yet benchmarked).

## 10. ROS interface (exists as topic contract, `ros2_bridge.py` MODEL DEPLOYMENT CONTRACT)
Subscribe (sim-time, default QoS — all verified live 2026-09-25, 10/10 checker):
- `/camera/rgb/image_raw` (rgb8 640x480, `camera_rgb_optical_frame`)
- `/camera/depth/image_raw` (32FC1 metres 640x480, `camera_depth_optical_frame`)
- `/camera/rgb/camera_info`, `/camera/depth/camera_info`
- `/clock` (use with `use_sim_time:=true`), `/tf` (dynamic only; NO `/tf_static` published)
No detector output topic exists yet (Phase 9 gate). Future: publish detections
(e.g. `vision_msgs/Detection2DArray`) — interface function stays `detect()`.

## RGB-D fusion output (`Object3D`, verified on synthetic + live data)
- `object_id`, `class_name`, `confidence` (pass-through from 2D)
- `position`: (x, y, z) approx base_link (x fwd, y left, z up; optical→base approx in `rgbd_fusion`)
- `distance`: Euclidean, metres; `bbox2d`: source box; `timestamp`; `source="rgbd_fusion"`
- `dimensions`: NOT provided (None-equivalent: fusion outputs position only)
- `velocity`: NOT provided (single-frame; tracking is future work)
- Verified: synthetic blob at 4.00 m -> Object3D dist 4.000 m; live center-stub -> 7.97 m plausible.

## Recorded test data (for the future detector)
- ROS bag `/tmp/opencode/rgbd_run` (2.8 GB, 39.6 s, 1390 msgs x 6 topics @~35 Hz):
  rgb + depth + both infos + tf + clock, covering the 3-pedestrian visibility window.
- Sample RGB frames: `data/integrated_rgb_single_ped_t8.png`,
  `data/integrated_rgb_three_peds_t10.png` (real S3 humans in robot view).

## Teammate detector reuse
No teammate detector exists in the repo (only `DummyDetector` + eval-only
`GroundTruthDetector`). Nothing to deduplicate; the plug-in point above is the contract.
