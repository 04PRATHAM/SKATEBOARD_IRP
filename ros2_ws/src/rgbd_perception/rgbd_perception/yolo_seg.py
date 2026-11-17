"""YOLO26n-seg wrapper — RGB-ONLY inference with segmentation masks.

Contract:
- Input: RGB numpy HxWx3 uint8 ONLY. Depth is NEVER fed to the model.
- Output: List[Detection2D] (repo contract) with bbox, class_id, label,
  confidence, bool HxW mask, timestamp. Masks are resized to the input frame.
- Device: cuda:0 when available else cpu (logged; FP16 on CUDA only).
- Latency: per-call preprocess/inference/postprocess ms recorded.

Portability: pure function of (image, config); no simulator/ROS imports here.
The ROS node lives in yolo_seg_node.py. Same file runs on the real vehicle.
"""
import time
import warnings
warnings.filterwarnings("ignore", message=".*'half' is deprecated.*")

from isaac_sim.scripts.perception.rgbd_interface import Detection2D


class YoloSegDetector:
    def __init__(self, model_path="yolo26n-seg.pt", device="cuda:0",
                 conf=0.35, iou=0.7, imgsz=640, half=True,
                 target_classes=(0,), class_names_override=None,
                 app_names=None):
        from ultralytics import YOLO
        import torch
        if device.startswith("cuda") and not torch.cuda.is_available():
            device = "cpu"
        self.device = device
        self.model = YOLO(model_path)
        if getattr(self.model, "task", "") != "segment":
            raise ValueError("model task is %r, need 'segment' for %s"
                             % (getattr(self.model, "task", None), model_path))
        self.names = dict(getattr(self.model, "model", {}).names
                          if hasattr(getattr(self.model, "model", None), "names")
                          else getattr(self.model, "names", {}))
        if class_names_override:
            self.names.update({int(k): str(v) for k, v in class_names_override.items()})
        # Application-facing labels (e.g. 60 -> "table"): canonical layer only.
        # Class IDs are never changed; model weights untouched.
        self.app_names = {}
        try:
            for k, v in dict(app_names or {}).items():
                self.app_names[int(k)] = str(v)
        except Exception:
            pass
        self.conf = float(conf)
        self.iou = float(iou)
        self.imgsz = int(imgsz)
        self.half = bool(half) and device.startswith("cuda")
        self.target_classes = set(int(c) for c in (target_classes or []))
        self.last_latency_ms = {}
        # Warm up (allocates CUDA context; excluded from measured latency).
        try:
            import numpy as np
            self.model.predict(_warmup_img(), verbose=False, device=self.device,
                               half=self.half, imgsz=self.imgsz)
        except Exception:
            pass

    def predict(self, rgb, stamp=0.0, frame_id=""):
        """Run RGB-only inference. Returns (detections, latency_ms dict)."""
        import numpy as np
        import torch
        t0 = time.perf_counter()
        res = self.model.predict(rgb, verbose=False, device=self.device,
                                 half=self.half, imgsz=self.imgsz,
                                 conf=self.conf, iou=self.iou)[0]
        t1 = time.perf_counter()
        H, W = rgb.shape[:2]
        dets = []
        boxes = getattr(res, "boxes", None)
        masks = getattr(res, "masks", None)
        mdata = None
        if masks is not None and getattr(masks, "data", None) is not None:
            try:
                mdata = masks.data.detach().to("cpu").float()
                # masks.data: N x h x w (letterboxed imgsz) -> resize to HxW
                mdata = torch.nn.functional.interpolate(
                    mdata.unsqueeze(0), size=(H, W), mode="nearest").squeeze(0) > 0.5
                mdata = mdata.numpy()
            except Exception:
                mdata = None
        if boxes is not None and len(boxes) > 0:
            xyxy = boxes.xyxy.detach().to("cpu").numpy()
            cls = boxes.cls.detach().to("cpu").numpy().astype(int)
            cnf = boxes.conf.detach().to("cpu").numpy()
            for i in range(len(xyxy)):
                cid = int(cls[i])
                if self.target_classes and cid not in self.target_classes:
                    continue
                x1, y1, x2, y2 = [float(v) for v in xyxy[i]]
                mk = None
                if mdata is not None and i < len(mdata):
                    try:
                        mk = np.asarray(mdata[i], dtype=bool)
                    except Exception:
                        mk = None
                dets.append(Detection2D(
                    bbox=(x1, y1, x2, y2),
                    class_name=str(self.app_names.get(
                        cid, self.names.get(cid, "class_%d" % cid))),
                    confidence=float(cnf[i]),
                    object_id=i,
                    class_id=cid,
                    mask=mk,
                    timestamp=float(stamp),
                    frame_id=str(frame_id),
                ))
        t2 = time.perf_counter()
        lat = {"total_ms": (t2 - t0) * 1000.0, "infer_ms": (t1 - t0) * 1000.0,
               "post_ms": (t2 - t1) * 1000.0, "n": len(dets)}
        try:
            spd = getattr(res, "speed", None) or {}
            lat["yolo_pre_ms"] = float(spd.get("preprocess", 0.0))
            lat["yolo_inf_ms"] = float(spd.get("inference", 0.0))
            lat["yolo_post_ms"] = float(spd.get("postprocess", 0.0))
        except Exception:
            pass
        self.last_latency_ms = lat
        return dets, lat


def _warmup_img():
    import numpy as np
    return np.zeros((480, 640, 3), dtype=np.uint8)
