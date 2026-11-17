"""Load configs/perception.yaml (single source of runtime parameters)."""
import os
import yaml

_DEFAULTS = {
    "perception": {
        "model_path": "yolo26n-seg.pt",
        "task": "segment",
        "confidence_threshold": 0.35,
        "iou_threshold": 0.7,
        "image_size": 640,
        "device": "cuda:0",
        "half_precision": True,
        "target_classes": [0, 13, 56, 57, 60],
        "obstacle_classes": [0, 13, 56, 57, 60],
        "class_names_override": {},
        # Application-facing labels (single canonical rename layer).
        # Key = COCO class id (UNCHANGED, model weights untouched);
        # value = label exposed to Detection2D/Object3D/viewer/logs/JSON.
        # 60 is COCO "dining table" in the weights but "table" everywhere downstream.
        "application_class_names": {0: "person", 13: "bench", 56: "chair",
                                    57: "couch", 60: "table"},
        "depth_method": "mask_median",
        "trim_fraction": 0.1,
        "min_depth": 0.3,
        "max_depth": 10.0,
        "min_valid_fraction": 0.05,
        "perception_rate": 10.0,
        "stale_timeout": 0.5,
        "sync_tolerance": 0.1,
        "debug_visualization": False,
        "debug_out_dir": "/tmp/opencode/yolo_debug",
    },
    "planner": {
        "corridor_width": 1.2,
        "lookahead_min": 1.0,
        "lookahead_max": 8.0,
        "slow_distance": 4.0,
        "stop_distance": 1.5,
        "cruise_speed": 0.8,
        "slow_speed": 0.35,
        "max_steer": 0.6,
        "steer_gain": 0.5,
    },
    "topics": {
        "rgb": "/camera/rgb/image_raw",
        "depth": "/camera/depth/image_raw",
        "rgb_info": "/camera/rgb/camera_info",
        "depth_info": "/camera/depth/camera_info",
        "detections": "/perception/detections_2d",
        "objects": "/perception/objects_3d",
        "cmd": "/cmd_ackermann",
        "rgb_optical_frame": "camera_rgb_optical_frame",
        "depth_optical_frame": "camera_depth_optical_frame",
    },
}

_CANDIDATES = [
    "/home/ubuntu/SKATEBOARD_IRP/configs/perception.yaml",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..",
                 "configs", "perception.yaml"),
]


def application_label(perception_cfg, class_id, model_name):
    """CANONICAL application label for a COCO class id (single mapping layer).

    Resolution order: application_class_names[id] -> class_names_override[id]
    -> model_name. Class IDs are never altered here; only the display label.
    All downstream output (Detection2D, Object3D, viewer, logs, JSON) must call
    this instead of scattering str.replace() calls.
    """
    try:
        app_names = perception_cfg.get("application_class_names") or {}
        for key in (int(class_id), str(class_id)):
            if key in app_names:
                return str(app_names[key])
        over = perception_cfg.get("class_names_override") or {}
        for key in (int(class_id), str(class_id)):
            if key in over:
                return str(over[key])
    except Exception:
        pass
    return str(model_name)


def load_config(path=None):
    """Load perception config; missing keys fall back to _DEFAULTS (never crash)."""
    cfg = {k: dict(v) for k, v in _DEFAULTS.items()}
    paths = [path] if path else _CANDIDATES
    for p in paths:
        try:
            if p and os.path.exists(p):
                with open(p) as f:
                    raw = yaml.safe_load(f) or {}
                for section in ("perception", "planner", "topics"):
                    if section in raw and isinstance(raw[section], dict):
                        cfg[section].update(raw[section])
                cfg["_source"] = p
                break
        except Exception:
            continue
    return cfg
