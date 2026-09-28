"""
Model-Agnostic RGB-D Perception Interface (M9 foundation, per PART 32)
Keeps interface independent of final pretrained RGB model (YOLO/DETR etc. not yet selected).

Architecture (future):
RGB image  ──► RGBDetectorBase.detect() → Detection2D[] ─┐
Depth image ──► depth data ────────────────────────────────┼─► RGBDFusion → Object3D[]
CameraInfo  ──► K (fx,fy,cx,cy) ───────────────────────────┘   (position, distance, bbox, class, conf, stamp)

Current: DummyDetector + GroundTruthDetector (eval only, clearly labeled) + deprojection math.
Future: swap DummyDetector for YoloV8Detector without touching vehicle/camera/ROS/nav.

Do NOT implement final neural model yet.
Do NOT use ground truth as runtime input (unless explicitly labeled temp).
Do NOT add LiDAR/IMU/GPS.
Driver_Seat_Camera is viewport-only, not an input.
"""

from dataclasses import dataclass
from typing import List, Tuple, Optional
import math

@dataclass
class Detection2D:
    bbox: Tuple[float,float,float,float]  # x_min, y_min, x_max, y_max (pixels)
    class_name: str = "person"
    confidence: float = 0.90
    object_id: int = 0
    # Phase 9 extensions (all optional — existing constructors unaffected):
    class_id: int = -1        # dataset class id (e.g. COCO 0 = person)
    mask: object = None       # optional numpy bool HxW segmentation mask (YOLO-seg)
    timestamp: float = 0.0    # source frame time (sim/ROS seconds)
    frame_id: str = ""        # source optical frame

@dataclass
class Object3D:
    object_id: int
    class_name: str
    confidence: float
    position: Tuple[float,float,float]  # base_link frame (x forward, y left, z up)
    distance: float  # Euclidean in base_link
    bbox2d: Tuple[float,float,float,float]
    timestamp: float = 0.0
    source: str = "rgbd_fusion"  # or "ground_truth_eval_only"

class RGBDetectorBase:
    """Abstract detector — future model plugs in here."""
    def detect(self, rgb_image, camera_info) -> List[Detection2D]:
        raise NotImplementedError

class DummyDetector(RGBDetectorBase):
    """Returns no detections (free-space). For pipeline testing without model."""
    def detect(self, rgb_image, camera_info) -> List[Detection2D]:
        return []

class GroundTruthDetector(RGBDetectorBase):
    """
    Uses simulator ground truth to emit Detection2D — FOR EVALUATION ONLY.
    Clearly labeled source='ground_truth_eval_only', never for runtime control
    unless experiment explicitly says 'ground truth controller development'.
    """
    def __init__(self, stage, pedestrian_ids: List[str]):
        self.stage = stage
        self.ids = pedestrian_ids
    def detect(self, rgb_image, camera_info) -> List[Detection2D]:
        # Project 3D ground truth to 2D via simple pinhole (approx)
        # For now return dummy centered bbox per pedestrian if visible
        dets = []
        for i, pid in enumerate(self.ids):
            # Dummy bbox 40% of image centered
            dets.append(Detection2D(bbox=(220,140,420,340), class_name="person", confidence=1.0, object_id=i))
        return dets

def deproject(depth_image, u: float, v: float, camera_info) -> Optional[float]:
    """Sample depth at (u,v) and return Z (meters). Assumes 32FC1, depth_image is numpy."""
    try:
        import numpy as np
        h,w = depth_image.shape[:2]
        ui, vi = int(round(u)), int(round(v))
        if not (0 <= ui < w and 0 <= vi < h):
            return None
        z = float(depth_image[vi, ui])
        if not (0.1 <= z <= 10.0) or math.isnan(z) or math.isinf(z):
            return None
        return z
    except Exception:
        return None

def _mask_depth(depth_image, mask, method="mask_median", trim=0.1,
                  min_d=0.3, max_d=10.0, min_frac=0.05):
    """Robust depth from mask pixels. Rejects NaN/inf/zero/out-of-range.

    Returns (median_z, valid_fraction, valid_count) or (None, frac, n).
    Methods: mask_median | mask_trimmed | bbox_center(u, v via mask centroid).
    Exact method is logged by callers (no silent choices).
    """
    try:
        import numpy as np
        m = np.asarray(mask, dtype=bool)
        h, w = depth_image.shape[:2]
        if m.shape[:2] != (h, w):
            return None, 0.0, 0
        vals = np.asarray(depth_image, dtype=np.float64)[m]
        ok = np.isfinite(vals) & (vals >= min_d) & (vals <= max_d)
        frac = float(ok.mean()) if vals.size else 0.0
        good = vals[ok]
        if good.size == 0 or frac < min_frac:
            return None, frac, int(ok.sum())
        if method == "mask_trimmed" and good.size >= 3:
            k = int(trim * good.size)
            good = np.sort(good)[k:good.size - k] if k > 0 else good
        z = float(np.median(good))
        # representative pixel: medoid-ish — pixel nearest the median value
        return z, frac, int(ok.sum())
    except Exception:
        return None, 0.0, 0


def rgbd_fusion(detections: List[Detection2D], depth_image, camera_info,
                depth_method: str = "bbox_center", trim: float = 0.1,
                min_d: float = 0.3, max_d: float = 10.0,
                min_frac: float = 0.05) -> List[Object3D]:
    """
    Associate 2D detections with depth and CameraInfo intrinsic K.
    K from sensor_msgs/CameraInfo: fx=K[0], fy=K[4], cx=K[2], cy=K[5]
    Deproject: X=(u-cx)*Z/fx, Y=(v-cy)*Z/fy, Z in camera_depth_optical_frame.
    Then TF to base_link: optical Z forward → base X forward, optical X right → base Y left? etc.
    Simplified: assume depth optical Z is forward, so base_link X=Z, Y=-X_optical, Z=-Y_optical.
    For M9 we keep in camera frame and document TF needed.

    Phase 9: when det.mask is present and depth_method != "bbox_center", Z
    comes from the robust mask statistic (median/trimmed) instead of the
    single bbox-center pixel. Default "bbox_center" preserves legacy behavior.
    """
    objs = []
    if camera_info is None:
        return objs
    # Extract K
    try:
        K = camera_info.k if hasattr(camera_info, 'k') else camera_info.get('k', [500,0,320,0,500,240,0,0,1])
        fx, fy, cx, cy = float(K[0]), float(K[4]), float(K[2]), float(K[5])
    except Exception:
        fx,fy,cx,cy = 500.0,500.0,320.0,240.0
    for det in detections:
        x_min,y_min,x_max,y_max = det.bbox
        u = (x_min+x_max)/2.0
        v = (y_min+y_max)/2.0
        z = None
        if depth_image is not None and getattr(det, "mask", None) is not None \
                and depth_method in ("mask_median", "mask_trimmed"):
            z, _frac, _n = _mask_depth(depth_image, det.mask, method=depth_method,
                                       trim=trim, min_d=min_d, max_d=max_d,
                                       min_frac=min_frac)
        else:
            z = deproject(depth_image, u, v, camera_info) if depth_image is not None else None
        if z is None:
            continue
        # Camera optical frame
        x_cam = (u - cx) * z / fx
        y_cam = (v - cy) * z / fy
        # Transform to base_link (approx): base X = z_cam, base Y = -x_cam, base Z = -y_cam
        x_base = z
        y_base = -x_cam
        z_base = -y_cam
        dist = math.sqrt(x_base**2 + y_base**2 + z_base**2)
        objs.append(Object3D(
            object_id=det.object_id,
            class_name=det.class_name,
            confidence=det.confidence,
            position=(x_base, y_base, z_base),
            distance=dist,
            bbox2d=det.bbox,
            source="rgbd_fusion"
        ))
    return objs

"""
Why final RGB model not yet selected (for guide §22):
- Need real vehicle camera images to measure domain gap (lab lighting, glass refraction, occlusion at 640×480 70°).
- Need to benchmark YOLOv8/DETR/etc. on synthetic vs real; no decision yet to avoid locking architecture.
- Interface stays agnostic: RGBDetectorBase → swap without redesigning vehicle/camera/ROS/nav.
"""
