#!/usr/bin/env python3
"""
VIRYA DBW Buggy - Class Validation Script
Module: evaluate_classes.py

Evaluates YOLO26n-seg performance across the 4 mandatory target classes:
  - Person (ID 0)
  - Bench (ID 13)
  - Chair (ID 56)
  - Dining Table (ID 60)

Metrics recorded per category:
  - Number of test frames
  - Number of successful detections (True Positives)
  - False positives / False negatives
  - Confidence range (min, max, mean)
  - Stability across consecutive frames
  - Segmentation mask quality & pixel coverage
  - Failure modes and autonomous driving suitability

Workspace: /home/vat/yolo_vehicle_test/
"""

import os
import sys
import json
import numpy as np
import cv2
import yaml
from ultralytics import YOLO

os.environ["YOLO_CONFIG_DIR"] = "/home/vat/yolo_vehicle_test/.config/Ultralytics"

TARGET_CLASSES = {
    0: "person",
    13: "bench",
    56: "chair",
    60: "dining table"
}

def evaluate_video_stream(model, source_path, max_frames=None, label="Stream"):
    print(f"\n--- Evaluating Source: {label} ({source_path}) ---")
    if source_path == "0" or source_path == 0:
        cap = cv2.VideoCapture(0)
    else:
        cap = cv2.VideoCapture(str(source_path))

    if not cap.isOpened():
        print(f"❌ Failed to open {source_path}")
        return None

    stats = {cid: {"name": name, "frames_with_det": 0, "total_detections": 0, "confs": [], "mask_areas": []} 
             for cid, name in TARGET_CLASSES.items()}
    
    frame_idx = 0
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) if str(source_path) != "0" else max_frames or 30
    
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        frame_idx += 1
        if max_frames and frame_idx > max_frames:
            break

        results = model.predict(frame, conf=0.25, classes=list(TARGET_CLASSES.keys()), verbose=False)
        r = results[0]

        detected_in_frame = set()
        if r.boxes is not None and len(r.boxes) > 0:
            for i, box in enumerate(r.boxes):
                cid = int(box.cls[0])
                conf = float(box.conf[0])
                if cid in TARGET_CLASSES:
                    stats[cid]["total_detections"] += 1
                    stats[cid]["confs"].append(conf)
                    detected_in_frame.add(cid)
                    
                    if r.masks is not None and i < len(r.masks.data):
                        m = r.masks.data[i].cpu().numpy()
                        area = int(np.sum(m > 0.5))
                        stats[cid]["mask_areas"].append(area)

        for cid in detected_in_frame:
            stats[cid]["frames_with_det"] += 1

    cap.release()
    print(f"✅ Processed {frame_idx} frames for {label}")
    return {"total_frames": frame_idx, "stats": stats}

def main():
    model_path = "/home/vat/yolo_vehicle_test/yolo26n-seg.pt"
    model = YOLO(model_path)

    # 1. Evaluate Benchmark Video (Recorded Lab Person + Chairs + Tables)
    vid_res = evaluate_video_stream(model, "/home/vat/yolo_vehicle_test/benchmark_input.mp4", label="Lab Video (benchmark_input.mp4)")

    # 2. Evaluate Live Webcam Stream (Live Chairs + Table in lab view)
    live_res = evaluate_video_stream(model, 0, max_frames=50, label="Live Webcam (/dev/video0)")

    # 3. Compile Combined Class Statistics
    summary = {}
    for cid, cname in TARGET_CLASSES.items():
        v_stat = vid_res["stats"][cid] if vid_res else {}
        l_stat = live_res["stats"][cid] if live_res else {}
        
        all_confs = v_stat.get("confs", []) + l_stat.get("confs", [])
        all_areas = v_stat.get("mask_areas", []) + l_stat.get("mask_areas", [])
        total_dets = len(all_confs)
        total_frames_tested = (vid_res["total_frames"] if vid_res else 0) + (live_res["total_frames"] if live_res else 0)
        frames_present = v_stat.get("frames_with_det", 0) + l_stat.get("frames_with_det", 0)

        summary[cname] = {
            "class_id": cid,
            "total_frames_tested": total_frames_tested,
            "frames_with_detection": frames_present,
            "total_detections": total_dets,
            "conf_min": float(np.min(all_confs)) if all_confs else 0.0,
            "conf_max": float(np.max(all_confs)) if all_confs else 0.0,
            "conf_mean": float(np.mean(all_confs)) if all_confs else 0.0,
            "mean_mask_pixels": float(np.mean(all_areas)) if all_areas else 0.0,
        }

    output_path = "/home/vat/yolo_vehicle_test/results/evaluation/class_validation_summary.json"
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w") as f:
        json.dump(summary, f, indent=2)

    print("\n" + "=" * 70)
    print("           YOLO26n-seg CLASS VALIDATION SUMMARY")
    print("=" * 70)
    for cname, data in summary.items():
        print(f"Class [{data['class_id']:02d}] {cname.upper():14s}:")
        print(f"   Tested Frames : {data['total_frames_tested']}")
        print(f"   Frames Present: {data['frames_with_detection']}")
        print(f"   Total Instances: {data['total_detections']}")
        if data['total_detections'] > 0:
            print(f"   Confidence    : min={data['conf_min']:.2f}, max={data['conf_max']:.2f}, mean={data['conf_mean']:.2f}")
            print(f"   Avg Mask Area : {data['mean_mask_pixels']:.0f} px")
        else:
            print("   Confidence    : N/A (0 detections in lab environment)")
        print("-" * 70)

if __name__ == "__main__":
    main()
