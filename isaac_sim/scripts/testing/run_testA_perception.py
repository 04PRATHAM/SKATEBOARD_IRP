#!/usr/bin/env /home/ubuntu/isaacsim/python.sh
"""Perception worker: runs YOLO + fusion + planner in sim's python (same DDS context).

Launched as subprocess from main autonomy script. Communicates via JSON files.
"""
import sys, os, json, time, math
sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")

import numpy as np
import cv2
import ultralytics

# Config
with open("/home/ubuntu/SKATEBOARD_IRP/configs/perception.yaml") as f:
    _cfg = yaml.safe_load(f)
_pc = _cfg["perception"]
_planner = _cfg["planner"]

# YOLO
_yolo = ultralytics.YOLO(_pc["model_path"])
if getattr(_yolo.model, "task", "") != "segment":
    raise ValueError("model task is not 'segment'")
_names = dict(_yolo.model.names if hasattr(_yolo.model, "names") else _yolo.names)
_conf = float(_pc["confidence_threshold"])
_iou = float(_pc["iou_threshold"])
_imgsz = int(_pc["image_size"])
_half = bool(_pc["half_precision"])
_target = set(int(c) for c in _pc["target_classes"])

# Warmup
try:
    _yolo.predict(np.zeros((480, 640, 3), dtype=np.uint8), verbose=False, device="cuda:0", half=True, imgsz=640)
except Exception:
    pass

# Planner config
_w = float(_planner.get("corridor_width", 1.2)) / 2.0
_zmin = float(_planner.get("lookahead_min", 1.0))
_zmax = float(_planner.get("lookahead_max", 8.0))
_slow_d = float(_planner.get("slow_distance", 4.0))
_stop_d = float(_planner.get("stop_distance", 1.5))
_cruise = float(_planner.get("cruise_speed", 0.8))
_slow_v = float(_planner.get("slow_speed", 0.35))
_max_steer = float(_planner.get("max_steer", 0.6))
_gain = float(_planner.get("steer_gain", 0.5))
_obstacles = set(int(c) for c in _cfg["perception"].get("obstacle_classes", [0]))

# Config
IN_RGB = "/tmp/opencode/perception_rgb.json"
OUT_CMD = "/tmp/opencode/autonomy_cmd.json"
OUT_DET = "/tmp/opencode/perception_det.json"
OUT_OBJ = "/tmp/opencode/perception_obj.json"

print("[PERCEPTION] Starting perception worker", flush=True)

# Warmup
try:
    _yolo = ultralytics.YOLO("/home/ubuntu/SKATEBOARD_IRP/ros2_ws/src/rgbd_perception/model/yolo26n-seg.pt")
    _yolo.predict(np.zeros((480, 640, 3), dtype=np.uint8), verbose=False, device="cuda:0", half=True, imgsz=640)
except Exception as e:
    print("[PERCEPTION] Warmup failed:", e, flush=True)

_last_yolo = 0
_yolo_period = 0.1

def _predict(rgb):
    res = _yolo.predict(rgb, verbose=False, device="cuda:0", half=True, imgsz=640,
                        conf=0.35, iou=0.7)[0]
    H, W = rgb.shape[:2]
    dets = []
    boxes = getattr(res, "boxes", None)
    masks = getattr(res, "masks", None)
    mdata = None
    if masks is not None and getattr(masks, "data", None) is not None:
        import torch
        mdata = masks.data.detach().to("cpu").float()
        mdata = torch.nn.functional.interpolate(
            mdata.unsqueeze(0), size=(rgb.shape[0], rgb.shape[1]), mode="nearest").squeeze(0) > 0.5
        mdata = mdata.numpy()
    if boxes is not None and len(boxes) > 0:
        xyxy = boxes.xyxy.detach().to("cpu").numpy()
        cls = boxes.cls.detach().to("cpu").numpy().astype(int)
        cnf = boxes.conf.detach().to("cpu").numpy()
        for i in range(len(xyxy)):
            cid = int(cls[i])
            if _target and cid not in _target:
                continue
            x1, y1, x2, y2 = [float(v) for v in xyxy[i]]
            mk = None
            if mdata is not None and i < len(mdata):
                try: mk = np.asarray(mdata[i], dtype=bool)
                except Exception: mk = None
            yield {
                "bbox": (x1, y1, x2, y2),
                "class_id": cid,
                "class_name": str(_names.get(cid, f"class_{cid}")),
                "confidence": float(cnf[i]),
                "mask": mk,
            }

def _plan(objs):
    _w = 0.6; _zmin=1.0; _zmax=8.0; _slow_d=4.0; _stop_d=1.5
    _cruise=0.8; _slow_v=0.35; _max_steer=0.6; _gain=0.5
    _obstacles = {0}
    nearest = {"left": float("inf"), "center": float("inf"), "right": float("inf")}
    for o in objs:
        x, y, z = float(o["x"]), float(o["y"]), float(o["z"])
        cid = int(o.get("class_id", -1))
        if 0 not in {0} and cid not in {0}:  # _obstacles = {0}
            continue
        if not (math.isfinite(x) and math.isfinite(z)) or not (1.0 <= z <= 8.0):
            continue
        sector = "center" if abs(x) <= 0.6 else ("left" if x < 0 else "right")
        if z < nearest[sector]:
            nearest[sector] = z
    if nearest["center"] < 1.5 or (nearest["left"] < 1.5 and nearest["right"] < 1.5 and nearest["center"] < 4.0):
        return 0.0, 0.0, "STOP_blocked"
    if nearest["center"] >= 4.0:
        return 0.0, 0.8, "CRUISE_clear"
    if nearest["right"] >= nearest["left"]:
        return -0.3, 0.35, "SLOW_right"
    else:
        return 0.3, 0.35, "SLOW_left"

# Main loop
print("[PERCEPTION] Starting main loop", flush=True)
_last_yolo = 0
_yolo_period = 0.1

while True:
    try:
        # Read RGB frame
        if os.path.exists("/tmp/opencode/perception_rgb.json"):
            with open("/tmp/opencode/perception_rgb.json") as f:
                frame = json.load(f)
            rgb = np.array(frame["rgb"], dtype=np.uint8).reshape(frame["h"], frame["w"], 3)
            stamp = frame["t"]
            # Run YOLO (rate limited)
            now = time.time()
            if now - _last_yolo >= 0.1:
                _last_yolo = time.time()
                dets = list(_predict(frame["rgb"]))
                # Write detections
                with open("/tmp/opencode/perception_det.json", "w") as f:
                    json.dump({"t": stamp, "dets": dets}, f)
                # Fusion + planning (simplified: bbox-center depth)
                # For now, just output dummy - will integrate depth later
                with open("/tmp/opencode/perception_obj.json", "w") as f:
                    json.dump({"t": time.time(), "objs": []}, f)
    except Exception as e:
        pass
    time.sleep(0.01)
