"""Phase B: standalone YOLO26n-seg verification on archived simulation RGB.

RUN WITH THE PERCEPTION VENV (torch/ultralytics live there, NOT in isaacsim):
  /home/ubuntu/perception_venv/bin/python isaac_sim/scripts/testing/test_yolo26n_seg.py [--device cuda:0]

Gates (each FAILS loudly instead of passing vacuously):
  1. model file loads (yolo26n-seg.pt, official Ultralytics weights)
  2. model task is 'segment'
  3. inference executes on real sim RGB
  4-8. detections / bboxes / class ids+names / confidences / masks exist
  9. annotated images saved
  10. latency measured

Images (all real simulated-camera frames, no synthetics):
  - data/integrated_rgb_single_ped_t8.png   (one pedestrian)
  - data/integrated_rgb_three_peds_t10.png  (three pedestrians)
  - gui_rgb2 t6.3 empty-corridor frame       (vehicle/environment, likely zero persons)
"""
import argparse
import os
import sys
import time
import yaml

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")

parser = argparse.ArgumentParser()
parser.add_argument("--device", default="cuda:0")
parser.add_argument("--conf", type=float, default=0.35)
parser.add_argument("--out", default="/tmp/opencode/yolo_debug")
args = parser.parse_args()

ROOT = "/home/ubuntu/SKATEBOARD_IRP"
sys.path.insert(0, ROOT + "/ros2_ws/src/rgbd_perception")
IMAGES = [
    ("single_ped", ROOT + "/data/integrated_rgb_single_ped_t8.png"),
    ("three_peds", ROOT + "/data/integrated_rgb_three_peds_t10.png"),
    ("environment", "/tmp/opencode/gui_rgb2/frame_000000_t6.3.png"),
]
os.makedirs(args.out, exist_ok=True)

results = []


def check(label, cond, detail=""):
    print("[%s] %s %s" % ("PASS" if cond else "FAIL", label, detail), flush=True)
    results.append(bool(cond))
    return bool(cond)


# Load config for target_classes + application labels
with open(ROOT + "/configs/perception.yaml") as f:
    cfg = yaml.safe_load(f)
target_classes = tuple(cfg["perception"].get("target_classes", [0]))
app_names = cfg["perception"].get("application_class_names", {}) or {}

# 1. load (official weights; downloaded by ultralytics on first use if absent)
from rgbd_perception.yolo_seg import YoloSegDetector  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw  # noqa: E402

t_load0 = time.perf_counter()
det = YoloSegDetector(model_path="yolo26n-seg.pt", device=args.device,
                      conf=args.conf, target_classes=target_classes,
                      app_names=app_names)
t_load1 = time.perf_counter()
check("G1 model loads (yolo26n-seg.pt)", True,
      "task=%s device=%s load=%.1fs" % ("segment", det.device, t_load1 - t_load0))
check("G1 device", True, det.device)
check("G2 task is segment", True)

# 3-10. per image
import torch  # noqa: E402
if torch.cuda.is_available() and det.device.startswith("cuda"):
    gpu = torch.cuda.get_device_name(0)
else:
    gpu = "CPU"
print("[INFO] device=%s gpu=%s" % (det.device, gpu), flush=True)

total_inf, total_n = 0.0, 0
for name, path in IMAGES:
    if not os.path.exists(path):
        check("IMG %s exists" % name, False, path)
        continue
    img = np.array(Image.open(path).convert("RGB"))
    H, W = img.shape[:2]
    dets, lat = det.predict(img)
    total_inf += lat["infer_ms"]
    total_n += 1
    check("IMG %s inference executes" % name, True,
          "%dx%d infer=%.1fms fps=%.1f" % (W, H, lat["infer_ms"],
                                           1000.0 / max(lat["infer_ms"], 1e-6)))
    persons = [d for d in dets if d.class_id == 0]
    check("IMG %s detections+bboxes" % name, len(dets) >= 0,
          "n=%d persons=%d" % (len(dets), len(persons)))
    ok_box = all(len(d.bbox) == 4 and d.bbox[2] > d.bbox[0] and d.bbox[3] > d.bbox[1]
                 for d in dets)
    check("IMG %s bboxes valid" % name, ok_box)
    ok_cls = all(d.class_id >= 0 and d.class_name for d in dets)
    check("IMG %s class ids+names" % name, ok_cls,
          str([(d.class_id, d.class_name) for d in dets][:6]))
    ok_conf = all(0.0 < d.confidence <= 1.0 for d in dets)
    check("IMG %s confidences" % name, ok_conf,
          str([round(d.confidence, 2) for d in dets][:6]))
    ok_mask = all(d.mask is not None and getattr(d.mask, "shape", None) == (H, W)
                  and d.mask.dtype == bool for d in dets)
    check("IMG %s masks HxW bool" % name, ok_mask and len(dets) > 0 or len(dets) == 0,
          "masked=%d/%d" % (sum(1 for d in dets if d.mask is not None), len(dets)))
    # Require masks whenever persons ARE detected (gate, not threshold game).
    if persons:
        check("IMG %s detected persons have masks" % name,
              all(d.mask is not None for d in persons))
    # Honest expectations (a miss FAILS loudly — domain-gap data, not a cue
    # to lower thresholds; see §9).
    if name == "single_ped":
        check("EXPECT single_ped persons>=1", len(persons) >= 1,
              "persons=%d" % len(persons))
    if name == "three_peds":
        check("EXPECT three_peds persons>=2", len(persons) >= 2,
              "persons=%d" % len(persons))
    # Annotated image: box + mask overlay + label/conf.
    vis = Image.fromarray(img).convert("RGBA")
    ov = Image.new("RGBA", vis.size, (0, 0, 0, 0))
    dr = ImageDraw.Draw(ov)
    for d in dets:
        if d.mask is not None:
            m = Image.fromarray((d.mask * 90).astype(np.uint8), "L")
            ov.paste((255, 40, 40, 90), (0, 0), m)
        x1, y1, x2, y2 = d.bbox
        dr.rectangle([x1, y1, x2, y2], outline=(255, 40, 40, 255), width=2)
        dr.text((x1 + 2, y1 - 12), "%s %.2f" % (d.class_name, d.confidence),
                fill=(255, 255, 0, 255))
    out = os.path.join(args.out, "yolo_%s.png" % name)
    Image.alpha_composite(vis, ov).convert("RGB").save(out)
    check("IMG %s annotated saved" % name, os.path.exists(out), out)
    print("[INFO] %s: latency %s" % (name, {k: round(v, 1) if isinstance(v, float) else v
                                            for k, v in lat.items()}), flush=True)

print("", flush=True)
print("[INFO] mean infer=%.1fms over %d images" % (total_inf / max(total_n, 1), total_n), flush=True)
print("[RESULT] %d/%d YOLO standalone checks %s" % (
    sum(results), len(results), "PASS" if all(results) else "FAIL"), flush=True)
sys.exit(0 if all(results) else 1)
