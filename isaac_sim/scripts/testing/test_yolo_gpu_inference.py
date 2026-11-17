"""Phase 2 Verification: YOLO Model Load & GPU Inference with VRAM Recording.

Verifies:
  1. Pre-inference GPU memory check.
  2. YOLO26n-seg model loading onto configured CUDA device.
  3. Real RGB frame inference on actual simulation frames (person, bench, table).
  4. Post-inference GPU memory measurement and latency profiling.
  5. Accurate detection output validation.
"""

import sys
import os
import time
import subprocess
import cv2
import numpy as np

ROOT = "/home/ubuntu/SKATEBOARD_IRP"
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
if f"{ROOT}/ros2_ws/src/rgbd_perception" not in sys.path:
    sys.path.insert(0, f"{ROOT}/ros2_ws/src/rgbd_perception")

from rgbd_perception.config import load_config
from rgbd_perception.yolo_seg import YoloSegDetector


def get_gpu_vram():
    """Returns free and used VRAM in MiB via nvidia-smi."""
    try:
        smi = subprocess.run(
            ["nvidia-smi", "--query-gpu=memory.free,memory.used,memory.total", "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=3
        )
        parts = [int(p.strip()) for p in smi.stdout.strip().split("\n")[0].split(",")]
        return {"free_mb": parts[0], "used_mb": parts[1], "total_mb": parts[2]}
    except Exception as e:
        return {"error": str(e)}


def test_yolo_gpu_inference():
    print("=" * 80)
    print("PHASE 2: YOLO MODEL LOAD & GPU INFERENCE TEST")
    print("=" * 80)

    # 1. Ensure GPU memory is available
    from isaac_sim.scripts.testing.run_integrated_perception_gui import ensure_gpu_memory
    ensure_gpu_memory(min_free_mb=8000)

    # Pre-inference VRAM check
    vram_before = get_gpu_vram()
    print(f"\n[GPU VRAM BEFORE INFERENCE]")
    print(f"  Used VRAM:  {vram_before.get('used_mb')} MiB")
    print(f"  Free VRAM:  {vram_before.get('free_mb')} MiB")
    print(f"  Total VRAM: {vram_before.get('total_mb')} MiB")
    assert vram_before.get("free_mb", 0) > 4000, f"Insufficient VRAM before test: {vram_before}"

    # 2. Load configuration & model
    cfg_path = os.path.join(ROOT, "configs/perception.yaml")
    cfg = load_config(cfg_path)
    pc = cfg["perception"]

    t0 = time.time()
    det = YoloSegDetector(
        model_path=pc["model_path"],
        device=pc["device"],
        conf=pc["confidence_threshold"],
        iou=pc["iou_threshold"],
        imgsz=pc["image_size"],
        half=pc["half_precision"],
        target_classes=tuple(pc["target_classes"]),
        class_names_override=pc.get("class_names_override") or {},
        app_names=pc.get("application_class_names") or {}
    )
    load_time_sec = time.time() - t0
    print(f"\n[MODEL LOADED]")
    print(f"  Model Path:     {pc['model_path']}")
    print(f"  Target Device:  {det.device}")
    print(f"  Target Classes: {sorted(det.target_classes)}")
    print(f"  Load Time:      {load_time_sec:.3f} s")

    # 3. Test on actual simulation images
    test_images = [
        ("data/yolo_scenario_validation/scenario_02.png", "person"),
        ("data/yolo_scenario_validation/multiclass_sim_lab_view.png", "bench"),
    ]

    total_dets = 0
    for rel_path, expected_class in test_images:
        full_path = os.path.join(ROOT, rel_path)
        assert os.path.exists(full_path), f"Test image missing: {full_path}"
        bgr = cv2.imread(full_path)
        assert bgr is not None, f"Failed to decode image: {full_path}"
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

        dets, lat = det.predict(rgb, stamp=time.time(), frame_id="camera_rgb_optical_frame")
        print(f"\n[INFERENCE RESULT: {rel_path}]")
        print(f"  Image Shape: {rgb.shape} (H x W x C)")
        print(f"  Inference Latency: {lat.get('yolo_inf_ms', 0):.2f} ms (Total: {lat.get('total_ms', 0):.2f} ms)")
        print(f"  Detections Count: {len(dets)}")
        for idx, d in enumerate(dets):
            print(f"    - Detection {idx + 1}: class='{d.class_name}' ({d.class_id}) conf={d.confidence:.3f} bbox={d.bbox}")
        assert len(dets) > 0, f"Expected detections for {rel_path}, got 0!"
        found_expected = any(d.class_name == expected_class for d in dets)
        assert found_expected, f"Expected '{expected_class}' in detections for {rel_path}, found {[d.class_name for d in dets]}"
        total_dets += len(dets)

    # 4. Post-inference VRAM check
    vram_after = get_gpu_vram()
    print(f"\n[GPU VRAM AFTER INFERENCE]")
    print(f"  Used VRAM:  {vram_after.get('used_mb')} MiB (delta: +{vram_after.get('used_mb', 0) - vram_before.get('used_mb', 0)} MiB)")
    print(f"  Free VRAM:  {vram_after.get('free_mb')} MiB")
    print(f"  Total VRAM: {vram_after.get('total_mb')} MiB")

    print("\n" + "=" * 80)
    print(f"PHASE 2 PASS: Model loaded and executed {total_dets} detections successfully on real frames.")
    print("=" * 80)
    return True


if __name__ == "__main__":
    success = test_yolo_gpu_inference()
    sys.exit(0 if success else 1)
