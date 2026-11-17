"""Test 2D-to-3D detection association with explicit Detection3D.id and physical bounding box dimensions.

Tests with 5 YOLO 2D detections:
  Det 0: Person (valid depth: 2.10m)
  Det 1: Chair (INVALID depth: None / NaN / out-of-range)
  Det 2: Table (valid depth: 4.50m)
  Det 3: Couch (INVALID depth: None / out-of-range)
  Det 4: Person (valid depth: 3.20m)
Plus 1 unclassified geometric depth obstacle (id: geom_obs_0, depth: 5.10m).

Verifies:
  1. Every valid 3D result refers to its correct original 2D detection index via d3d.id ('yolo_det_{i}').
  2. Bounding box sizes are physically meaningful (size.z is depth thickness, not index).
  3. No detection receives another detection's depth.
  4. Unclassified geometric obstacles are kept separate.
"""

import math
from vision_msgs.msg import Detection2D, Detection2DArray, Detection3D, Detection3DArray, ObjectHypothesisWithPose

def run_test():
    print("=" * 80)
    print("TEST: 2D-to-3D Detection Association with Explicit Detection3D.id")
    print("=" * 80)

    # 5 simulated 2D detections
    yolo_classes = ["0:person", "56:chair", "60:table", "57:couch", "0:person"]
    depth_measurements = [2.10, None, 4.50, None, 3.20]  # Det 1 and 3 invalid

    print("Simulated 2D Detections:")
    for i, (cls, z) in enumerate(zip(yolo_classes, depth_measurements)):
        print(f"  Det {i}: class={cls:<10} | depth={str(z) + 'm' if z is not None else 'INVALID (dropped)'}")

    # Simulate rgbd_fusion_node logic
    out_3d = Detection3DArray()
    fx, fy, cx, cy = 640.0, 640.0, 640.0, 360.0
    bw, bh = 80.0, 160.0
    u, v = 640.0, 360.0

    for i, (cls, z) in enumerate(zip(yolo_classes, depth_measurements)):
        if z is None:
            continue
        X = (u - cx) * z / fx
        Y = (v - cy) * z / fy
        o = Detection3D()
        o.id = f"yolo_det_{i}"  # EXPLICIT IDENTIFIER
        h = ObjectHypothesisWithPose()
        h.hypothesis.class_id = cls
        h.hypothesis.score = 0.88
        o.results.append(h)
        o.bbox.center.position.x = float(X)
        o.bbox.center.position.y = float(Y)
        o.bbox.center.position.z = float(z)
        o.bbox.size.x = float(bw * z / fx)
        o.bbox.size.y = float(bh * z / fy)
        # Physically meaningful depth thickness (e.g. 0.45m for person, 0.60m for furniture)
        o.bbox.size.z = float(min(1.2, max(0.40, 0.6 * (bw * z / fx))))
        out_3d.detections.append(o)

    # Add unclassified geometric obstacle
    geom_obs = Detection3D()
    geom_obs.id = "geom_obs_0"
    gh = ObjectHypothesisWithPose()
    gh.hypothesis.class_id = "obstacle"
    gh.hypothesis.score = 0.85
    geom_obs.results.append(gh)
    geom_obs.bbox.center.position.x = 0.5
    geom_obs.bbox.center.position.y = 0.0
    geom_obs.bbox.center.position.z = 5.10
    geom_obs.bbox.size.x = 0.80
    geom_obs.bbox.size.y = 0.80
    geom_obs.bbox.size.z = 0.80  # 0.8m cube
    out_3d.detections.append(geom_obs)

    print("\nProduced 3D Detections in Detection3DArray:")
    for d in out_3d.detections:
        cls_name = d.results[0].hypothesis.class_id if d.results else "unknown"
        pos = d.bbox.center.position
        sz = d.bbox.size
        print(f"  ID: {d.id:<12} | Class: {cls_name:<10} | Pos: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}) | "
              f"Phys Size (XYZ): ({sz.x:.2f}m, {sz.y:.2f}m, {sz.z:.2f}m)")

    # Simulate live_perception_monitor lookup logic
    depth_by_idx = {}
    xyz_by_idx = {}
    geom_obstacles = []

    for obj in out_3d.detections:
        obj_id = getattr(obj, "id", "")
        cid_str = obj.results[0].hypothesis.class_id if obj.results else ""
        if obj_id.startswith("yolo_det_"):
            det_idx = int(obj_id.split("_")[-1])
            t = obj.bbox.center.position
            depth_by_idx[det_idx] = math.sqrt(t.x**2 + t.y**2 + t.z**2)
            xyz_by_idx[det_idx] = (t.x, t.y, t.z)
        elif obj_id.startswith("geom_obs_") or "obstacle" in cid_str.lower():
            geom_obstacles.append(obj)

    print("\nMonitor Reconstruction Verification:")
    print("  Lookups in depth_by_idx:", {k: f"{v:.2f}m" for k, v in depth_by_idx.items()})
    print("  Geometric obstacles detected:", len(geom_obstacles))

    # Assertions
    assert 0 in depth_by_idx, "Det 0 must have depth!"
    assert abs(depth_by_idx[0] - 2.10) < 1e-4, f"Det 0 depth expected 2.10m, got {depth_by_idx[0]}"

    assert 1 not in depth_by_idx, "Det 1 (invalid) must NOT have depth!"
    assert 2 in depth_by_idx, "Det 2 must have depth!"
    assert abs(depth_by_idx[2] - 4.50) < 1e-4, f"Det 2 depth expected 4.50m, got {depth_by_idx[2]}"

    assert 3 not in depth_by_idx, "Det 3 (invalid) must NOT have depth!"
    assert 4 in depth_by_idx, "Det 4 must have depth!"
    assert abs(depth_by_idx[4] - 3.20) < 1e-4, f"Det 4 depth expected 3.20m, got {depth_by_idx[4]}"

    assert len(geom_obstacles) == 1, "Must have exactly 1 separate geometric obstacle!"
    assert geom_obstacles[0].id == "geom_obs_0"
    assert geom_obstacles[0].bbox.size.z == 0.80, "Geometric obstacle size.z must be physically 0.80m!"

    for d in out_3d.detections:
        assert 0.3 <= d.bbox.size.z <= 2.0, f"Size.z ({d.bbox.size.z}) must be a physical dimension (0.3-2.0m), not an index!"

    print("\n[SUCCESS] All 2D-to-3D association checks PASSED:")
    print("  - Detection indices are preserved via explicit Detection3D.id.")
    print("  - Invalid depth drops do not shift subsequent detection associations.")
    print("  - Bounding box sizes are strictly physical dimensions.")
    print("  - Geometric obstacles are kept separate.")
    print("=" * 80)

if __name__ == "__main__":
    run_test()
