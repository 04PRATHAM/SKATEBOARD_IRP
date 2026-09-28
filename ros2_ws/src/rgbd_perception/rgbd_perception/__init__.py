"""rgbd_perception: camera-only autonomy (YOLO26n-seg RGB + mask depth fusion + planner).

Runtime contract: ONLY /camera/rgb/image_raw, /camera/depth/image_raw and
camera info may enter. Ground truth is evaluation-only (see TEST-K).
Portable: same topics/frames run on the real vehicle.
"""
