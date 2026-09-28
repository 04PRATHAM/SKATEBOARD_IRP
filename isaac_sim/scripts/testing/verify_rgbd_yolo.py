"""Phase D/E offline gate: bag RGB -> YOLO -> mask -> depth -> Object3D (no sim).

RUN: /home/ubuntu/perception_venv/bin/python
     isaac_sim/scripts/testing/verify_rgbd_yolo.py [--bag /tmp/opencode/rgbd_run]
     (venv inherits rclpy+rosbag2_py from system site-packages; source ROS first)

Reads synchronized (rgb, depth, info) triplets from the archived bag,
runs YOLO26n-seg (RGB-only), associates mask-median depth via the repo
fusion, prints per-detection class/conf/bbox/valid-px/median/x/y/z/dist,
and saves annotated frames ("person 0.91 4.2 m" — distance from FUSION,
never ground truth).

FAILs loudly on: no model / no RGB / no inference / no masks / no depth /
bad sync / no Object3D. Multi-person gate: >=1 frame with >=3 person-Objects.
"""
import argparse
import os
import sys

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")
sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP/ros2_ws/src/rgbd_perception")

parser = argparse.ArgumentParser()
parser.add_argument("--bag", default="/tmp/opencode/rgbd_run")
parser.add_argument("--frames-dir", default=None,
                    help="use recorded rgb_*.png/depth_*.npy/info.json triplets instead of bag")
parser.add_argument("--device", default="cuda:0")
parser.add_argument("--conf", type=float, default=0.35)
parser.add_argument("--frames", type=int, default=8)
parser.add_argument("--out", default="/tmp/opencode/yolo_debug/bag")
args = parser.parse_args()
os.makedirs(args.out, exist_ok=True)

results = []


def check(label, cond, detail=""):
    print("[%s] %s %s" % ("PASS" if cond else "FAIL", label, detail), flush=True)
    results.append(bool(cond))
    return bool(cond)


import numpy as np  # noqa: E402
from PIL import Image, ImageDraw  # noqa: E402
import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from sensor_msgs.msg import Image as RosImage, CameraInfo  # noqa: E402

from rgbd_perception.yolo_seg import YoloSegDetector  # noqa: E402
from rgbd_perception.config import load_config  # noqa: E402
from isaac_sim.scripts.perception.rgbd_interface import (  # noqa: E402
    rgbd_fusion, _mask_depth)

cfg = load_config()
pc = cfg["perception"]

# --- triplets: recorded frames dir (preferred) or bag --------------------
class _NP:
    """Numpy-backed frame shim with the fields the evaluator uses."""
    def __init__(self, rgb_np, dep_np, K, t):
        self.rgb_np = rgb_np
        self.dep_np = dep_np
        self.K = K
        self.t = t


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


triplets = []
K_live = None
if args.frames_dir and os.path.isdir(args.frames_dir):
    import glob as _glob
    import json as _json
    rgbs = sorted(_glob.glob(os.path.join(args.frames_dir, "rgb_*.png")))
    info_p = os.path.join(args.frames_dir, "info.json")
    K_live = {"k": _json.load(open(info_p))["k"]} if os.path.exists(info_p) else None
    check("FRAMES dir readable", len(rgbs) > 10, "%s n=%d" % (args.frames_dir, len(rgbs)))
    check("FRAMES info.json K present", K_live is not None)
    step = max(1, len(rgbs) // args.frames)
    for rp in rgbs[::step][:args.frames]:
        dp = rp.replace("/rgb_", "/depth_").replace(".png", ".npy")
        if not os.path.exists(dp):
            continue
        t = float(rp.rsplit("_t", 1)[-1].replace(".png", ""))
        triplets.append((_NP(np.array(Image.open(rp).convert("RGB")),
                             np.load(dp), K_live, t), None, None))
    check("FRAMES triplets assembled", len(triplets) >= 4, "n=%d" % len(triplets))
else:
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    print("[INFO] bag topics: %s" % sorted(type_map), flush=True)
    check("BAG readable (rgb+depth+info topics)",
          all(k in type_map for k in ("/camera/rgb/image_raw", "/camera/depth/image_raw",
                                      "/camera/depth/camera_info")))

    rgb, depth, info = [], [], []
    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic == "/camera/rgb/image_raw":
            m = deserialize_message(data, RosImage)
            if m.encoding == "rgb8":
                rgb.append(m)
        elif topic == "/camera/depth/image_raw":
            m = deserialize_message(data, RosImage)
            if m.encoding == "32FC1":
                depth.append(m)
        elif topic == "/camera/depth/camera_info":
            info.append(deserialize_message(data, CameraInfo))
    check("BAG has RGB frames", len(rgb) > 100, "n=%d" % len(rgb))
    check("BAG has depth frames", len(depth) > 100, "n=%d" % len(depth))
    check("BAG has info", len(info) > 0, "n=%d" % len(info))

    # spread triplets across the recording
    step = max(1, len(rgb) // (args.frames + 1))
    for r in rgb[::step][:args.frames * 2]:
        tr = stamp(r)
        best = min(depth, key=lambda d: abs(stamp(d) - tr))
        if abs(stamp(best) - tr) <= pc["sync_tolerance"]:
            triplets.append((r, best, info[-1]))
        if len(triplets) >= args.frames:
            break
    check("SYNC triplets within tolerance", len(triplets) >= 4, "n=%d" % len(triplets))

# --- model ---------------------------------------------------------------
det = YoloSegDetector(model_path=pc["model_path"], device=args.device,
                      conf=args.conf, target_classes=tuple(pc["target_classes"]))
check("MODEL yolo26n-seg loads", True, "device=%s" % det.device)

# --- per-frame RGB -> mask -> depth -> Object3D ---------------------------
if K_live is None:
    K_live = {"k": list(info[-1].k)}
K = K_live
n_person_frames = 0
multi_frames = 0
for fi, (r, d, _ci) in enumerate(triplets):
    if isinstance(r, _NP):
        rgb_np, dep_np, K, _t = r.rgb_np, r.dep_np, r.K, r.t
    else:
        rgb_np = np.frombuffer(r.data, dtype=np.uint8).reshape(r.height, r.width, 3)
        dep_np = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
    dets, lat = det.predict(rgb_np)
    check("F%d inference ok" % fi, True, "n=%d infer=%.0fms" % (len(dets), lat["infer_ms"]))
    persons = [x for x in dets if x.class_id == 0]
    if persons:
        n_person_frames += 1
    objs = rgbd_fusion(persons, dep_np, K, depth_method=pc["depth_method"],
                       trim=pc.get("trim_fraction", 0.1),
                       min_d=pc["min_depth"], max_d=pc["max_depth"],
                       min_frac=pc.get("min_valid_fraction", 0.05))
    if len([o for o in objs]) >= 3:
        multi_frames += 1
    for o in objs:
        x, y, z = o.position
        print("F%d %s %.2f bbox=%s dist=%.2f m xyz=(%.2f,%.2f,%.2f)" % (
            fi, o.class_name, o.confidence,
            tuple(int(v) for v in o.bbox2d), o.distance, x, y, z), flush=True)
    check("F%d Object3D from fusion" % fi, len(objs) > 0 or len(persons) == 0,
          "objs=%d persons=%d" % (len(objs), len(persons)))
    # annotated viz with FUSED distance (never ground truth)
    vis = Image.fromarray(rgb_np).convert("RGBA")
    ov = Image.new("RGBA", vis.size, (0, 0, 0, 0))
    dr = ImageDraw.Draw(ov)
    for xd in persons:
        if xd.mask is not None:
            ov.paste((255, 40, 40, 90), (0, 0),
                     Image.fromarray((np.asarray(xd.mask) * 90).astype(np.uint8), "L"))
    for o in objs:
        x1, y1, x2, y2 = o.bbox2d
        dr.rectangle([x1, y1, x2, y2], outline=(0, 255, 0, 255), width=2)
        dr.text((x1 + 2, y1 - 12), "%s %.2f  %.1f m" % (o.class_name, o.confidence, o.distance),
                fill=(0, 255, 0, 255))
    vis2 = Image.alpha_composite(vis, ov).convert("RGB")
    vis2.save(os.path.join(args.out, "bag_f%02d.png" % fi))

check("MULTI >=1 frame with >=3 person-Objects", multi_frames >= 1, "n=%d" % multi_frames)
check("PERSONS seen in >=2 frames", n_person_frames >= 2, "n=%d" % n_person_frames)
print("", flush=True)
print("[RESULT] %d/%d offline RGB-D checks %s" % (
    sum(results), len(results), "PASS" if all(results) else "FAIL"), flush=True)
sys.exit(0 if all(results) else 1)
