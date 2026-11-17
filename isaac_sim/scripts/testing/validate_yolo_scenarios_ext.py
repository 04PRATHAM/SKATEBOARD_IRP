"""External YOLO perception validator — RIGHT pane (perception_venv + rclpy).

Subscribes to the LIVE sim bridge + repo perception nodes:
  /camera/rgb/image_raw      (sensor_msgs/Image rgb8 640x480)
  /camera/depth/camera_info  (for K to match 2D<->3D)
  /perception/detections_2d  (from repo yolo_seg_node)
  /perception/masks          (from repo yolo_seg_node, mono8 idx+1)
  /perception/objects_3d     (from repo rgbd_fusion_node, optical XYZ)

Renders the ACTUAL detection output (masks + boxes + class/conf/distance)
onto the ACTUAL RGB frame in a cv2 window and saves a composite PNG
(LEFT: raw RGB sim view, RIGHT: annotated) + ext_<NN>.json metrics.

Launch order per scenario:
  1. repo nodes: yolo_seg_node + rgbd_fusion_node (persistent across scenarios)
  2. this validator: --scenario NN (captures until RGB stream idles out)
  3. sim side: run_yolo_all_scenarios_gui.py --scenario NN [--gui]
  (NO planner node anywhere in this phase: no /cmd_ackermann is produced.)

Run: source /opt/ros/humble/setup.bash
  /home/ubuntu/perception_venv/bin/python \\
    isaac_sim/scripts/testing/validate_yolo_scenarios_ext.py \\
    --scenario 02 --out data/yolo_scenario_validation [--show]
"""
import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")

parser = argparse.ArgumentParser(description="External YOLO validation RIGHT pane")
parser.add_argument("--scenario", required=True)
parser.add_argument("--out", default="/home/ubuntu/SKATEBOARD_IRP/data/yolo_scenario_validation")
parser.add_argument("--show", dest="show", action="store_true", default=None)
parser.add_argument("--no-show", dest="show", action="store_false")
parser.add_argument("--wait-timeout", type=float, default=180.0)
parser.add_argument("--idle-timeout", type=float, default=8.0)
args = parser.parse_args()
show = args.show if args.show is not None else bool(os.environ.get("DISPLAY"))
os.makedirs(args.out, exist_ok=True)

import numpy as np  # noqa: E402
from PIL import Image, ImageDraw  # noqa: E402
import rclpy  # noqa: E402
from sensor_msgs.msg import Image as RosImage, CameraInfo  # noqa: E402
from vision_msgs.msg import Detection2DArray, Detection3DArray  # noqa: E402

try:
    import cv2  # noqa: E402
    HAS_CV2 = True
except Exception:
    HAS_CV2 = False
    show = False

CLASS_COLORS = {0: (0, 255, 0), 13: (255, 165, 0), 56: (0, 180, 255),
                57: (255, 0, 255), 60: (255, 255, 0)}

latest = {"rgb": None, "rgb_t": 0.0, "dets": None, "masks": None,
          "objs": None, "K": None}


def parse_cid(s):
    try:
        return int(str(s).split(":")[0])
    except Exception:
        return -1


def parse_label(s):
    try:
        return str(s).split(":", 1)[1]
    except Exception:
        return str(s)


def on_rgb(m):
    if m.encoding != "rgb8":
        return
    try:
        latest["rgb"] = (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                         np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, 3))
        latest["rgb_t"] = time.time()
    except Exception:
        pass


def on_info(m):
    try:
        latest["K"] = list(m.k)
    except Exception:
        pass


def on_det(m):
    try:
        latest["dets"] = m
    except Exception:
        pass


def on_mask(m):
    try:
        latest["masks"] = (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                           np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width))
    except Exception:
        pass


def on_obj(m):
    try:
        latest["objs"] = m
    except Exception:
        pass


def project_uv(X, Y, Z, K):
    fx, fy, cx, cy = float(K[0]), float(K[4]), float(K[2]), float(K[5])
    if not Z or Z <= 0:
        return None
    return (fx * X / Z + cx, fy * Y / Z + cy)


def annotate(rgb_np, dets_msg, masks_np, objs_msg, K):
    """Render ACTUAL pipeline output. Dets<->objs matched by projected pixels."""
    vis = Image.fromarray(rgb_np).convert("RGBA")
    ov = Image.new("RGBA", vis.size, (0, 0, 0, 0))
    dr = ImageDraw.Draw(ov)
    det_list = list(dets_msg.detections) if dets_msg is not None else []
    obj_list = list(objs_msg.detections) if objs_msg is not None else []
    for i, r in enumerate(det_list):
        try:
            cid = parse_cid(r.results[0].hypothesis.class_id)
        except Exception:
            cid = -1
        color = CLASS_COLORS.get(cid, (255, 40, 40))
        if masks_np is not None:
            try:
                m = Image.fromarray(((masks_np == (i + 1)) * 110).astype(np.uint8), "L")
                ov.paste(color + (110,), (0, 0), m)
            except Exception:
                pass
    vis = Image.alpha_composite(vis, ov)
    dr2 = ImageDraw.Draw(vis)
    used_objs = set()
    dist_txt = {}
    for i, r in enumerate(det_list):
        try:
            cid = parse_cid(r.results[0].hypothesis.class_id)
            label = parse_label(r.results[0].hypothesis.class_id)
            conf = float(r.results[0].hypothesis.score)
        except Exception:
            continue
        color = CLASS_COLORS.get(cid, (255, 40, 40))
        u = float(r.bbox.center.position.x)
        v = float(r.bbox.center.position.y)
        bw, bh = float(r.bbox.size_x), float(r.bbox.size_y)
        x1, y1, x2, y2 = int(u - bw / 2), int(v - bh / 2), int(u + bw / 2), int(v + bh / 2)
        best, best_d = None, 1e9
        if K is not None:
            for j, o in enumerate(obj_list):
                if j in used_objs:
                    continue
                try:
                    ocid = parse_cid(o.results[0].hypothesis.class_id)
                except Exception:
                    continue
                if ocid != cid:
                    continue
                puv = project_uv(float(o.bbox.center.position.x),
                                 float(o.bbox.center.position.y),
                                 float(o.bbox.center.position.z), K)
                if puv is None:
                    continue
                d = math.hypot(puv[0] - u, puv[1] - v)
                if d < best_d:
                    best, best_d = j, d
        dist = None
        if best is not None and best_d <= 60.0:
            used_objs.add(best)
            o = obj_list[best]
            dist = math.sqrt(float(o.bbox.center.position.x) ** 2
                             + float(o.bbox.center.position.y) ** 2
                             + float(o.bbox.center.position.z) ** 2)
            dist_txt[i] = dist
        dr2.rectangle([x1, y1, x2, y2], outline=color + (255,), width=2)
        tag = "%s %.2f%s" % (label, conf, (" %.1fm" % dist) if dist is not None else " no-depth")
        dr2.text((x1 + 2, max(0, y1 - 12)), tag, fill=(255, 255, 255, 255))
    return vis.convert("RGB"), dist_txt


def composite(raw_rgb, annotated_rgb, tl, tr):
    W, H = raw_rgb.size
    canvas = Image.new("RGB", (W * 2 + 12, H + 28), (20, 20, 20))
    canvas.paste(raw_rgb, (0, 28))
    canvas.paste(annotated_rgb, (W + 12, 28))
    dr = ImageDraw.Draw(canvas)
    dr.text((8, 8), tl, fill=(200, 200, 200))
    dr.text((W + 20, 8), tr, fill=(200, 200, 200))
    return canvas


def main():
    rclpy.init()
    node = rclpy.create_node("yolo_validation_ext")
    node.create_subscription(RosImage, "/camera/rgb/image_raw", on_rgb, 10)
    node.create_subscription(CameraInfo, "/camera/depth/camera_info", on_info, 10)
    node.create_subscription(Detection2DArray, "/perception/detections_2d", on_det, 10)
    node.create_subscription(RosImage, "/perception/masks", on_mask, 10)
    node.create_subscription(Detection3DArray, "/perception/objects_3d", on_obj, 10)
    t0 = time.time()
    while latest["rgb"] is None and time.time() - t0 < args.wait_timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    if latest["rgb"] is None:
        print("[FAIL] no RGB within wait_timeout", flush=True)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    print("[CAPTURE] scenario %s stream live" % args.scenario, flush=True)
    frames = det_msgs = obj_msgs = 0
    det_counts, det_confs, obj_dists = {}, {}, {}
    last_rgb_wall, saved = time.time(), False
    last_ann = None
    while True:
        rclpy.spin_once(node, timeout_sec=0.1)
        if latest["rgb"] is None:
            continue
        frames += 1
        last_rgb_wall = time.time()
        rgb_t, rgb = latest["rgb"]
        ann, dmap = annotate(rgb, latest["dets"], latest["masks"][1] if latest["masks"] else None,
                             latest["objs"], latest["K"])
        last_ann = (rgb, ann)
        if latest["dets"] is not None:
            det_msgs += 1
            for r in latest["dets"].detections:
                try:
                    cid = parse_cid(r.results[0].hypothesis.class_id)
                    label = parse_label(r.results[0].hypothesis.class_id)
                    conf = float(r.results[0].hypothesis.score)
                except Exception:
                    continue
                det_counts[label] = det_counts.get(label, 0) + 1
                det_confs.setdefault(label, []).append(conf)
        if latest["objs"] is not None:
            for o in latest["objs"].detections:
                try:
                    label = parse_label(o.results[0].hypothesis.class_id)
                    d = math.sqrt(float(o.bbox.center.position.x) ** 2
                                  + float(o.bbox.center.position.y) ** 2
                                  + float(o.bbox.center.position.z) ** 2)
                except Exception:
                    continue
                obj_msgs += 1
                obj_dists.setdefault(label, []).append(d)
        if show and HAS_CV2:
            try:
                cv2.imshow("YOLO RGB - scenario %s" % args.scenario,
                           cv2.cvtColor(np.array(ann), cv2.COLOR_RGB2BGR))
                cv2.waitKey(1)
            except Exception:
                pass
        if not saved and det_msgs >= 5:
            try:
                composite(Image.fromarray(rgb), ann, "SIM RGB (raw camera view)",
                          "YOLO26n-seg + mask-median depth").save(
                              os.path.join(args.out, "scenario_%s.png" % args.scenario))
                saved = True
            except Exception as e:
                print("[WARN] save failed: %s" % e, flush=True)
        if time.time() - last_rgb_wall > args.idle_timeout and frames > 30:
            break
    if not saved and last_ann is not None:
        try:
            composite(Image.fromarray(last_ann[0]), last_ann[1],
                      "SIM RGB (raw camera view)",
                      "YOLO26n-seg + mask-median depth").save(
                          os.path.join(args.out, "scenario_%s.png" % args.scenario))
        except Exception as e:
            print("[WARN] fallback save failed: %s" % e, flush=True)
    res = {"scenario": args.scenario, "rgb_frames": frames, "det_msgs": det_msgs,
           "det_counts": det_counts,
           "det_conf_mean": {k: round(sum(v) / len(v), 3) for k, v in det_confs.items()},
           "obj_samples": obj_msgs,
           "obj_dist_median": {k: round(sorted(v)[len(v) // 2], 2) for k, v in obj_dists.items()},
           "obj_dist_min": {k: round(min(v), 2) for k, v in obj_dists.items()},
           "obj_dist_max": {k: round(max(v), 2) for k, v in obj_dists.items()},
           "evidence": "scenario_%s.png" % args.scenario}
    with open(os.path.join(args.out, "ext_%s.json" % args.scenario), "w") as f:
        json.dump(res, f, indent=2)
    print("[RESULT] %s" % json.dumps(res), flush=True)
    if show and HAS_CV2:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
