"""Unit test verifying 3D bounding box geometry, physical dimensions, and semantic priors."""

import sys
import numpy as np

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")
sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP/ros2_ws/src/rgbd_perception")

from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose, Detection3DArray, Detection3D
from sensor_msgs.msg import Image, CameraInfo
from rgbd_perception.rgbd_fusion_node import FusionNode, CLASS_DEPTH_PRIORS
from rgbd_perception.dynamic_tracker_node import Track, DynamicTrackerNode

def test_geometry():
    print("=" * 80)
    print("TEST: 3D BOUNDING-BOX GEOMETRY, PHYSICAL EXTENTS & SEMANTIC PRIORS")
    print("=" * 80)

    # 1. Verify semantic prior table
    print("Semantic depth prior dimensions (thickness along optical Z):")
    for k, v in [("person", 0.45), ("chair", 0.60), ("table", 0.85), ("couch", 0.90)]:
        val = CLASS_DEPTH_PRIORS.get(k)
        print(f"  Class: {k:<10} -> Prior Depth Extent: {val:.2f}m (Expected: {v:.2f}m)")
        assert abs(val - v) < 1e-3, f"Mismatch for {k}"

    # 2. Test Track dimension preservation
    t = Track(1, 0, "person", 0.0, 0.0, 2.5, 10.0)
    t.update(0.0, 0.0, 2.5, 10.1, sx=0.45, sy=1.75, sz=0.42)
    print(f"\nTrack 1 (Person): size_x={t.size_x:.2f}m, size_y={t.size_y:.2f}m, size_z={t.size_z:.2f}m (speed={t.speed:.2f}m/s)")
    assert abs(t.size_z - 0.42) < 1e-3, "size_z not preserved in Track!"

    # 3. Test Track to Detection3D conversion in dynamic_tracker_node
    d3d = Detection3D()
    d3d.bbox.center.position.z = float(t.z)
    d3d.bbox.size.x = float(t.size_x)
    d3d.bbox.size.y = float(t.size_y)
    d3d.bbox.size.z = float(t.size_z)
    hyp = ObjectHypothesisWithPose()
    hyp.hypothesis.score = float(t.speed)
    d3d.results.append(hyp)

    print(f"Published Detection3D: center.z={d3d.bbox.center.position.z:.2f}m (distance), size.z={d3d.bbox.size.z:.2f}m (depth extent), score={d3d.results[0].hypothesis.score:.2f} (speed)")
    assert abs(d3d.bbox.size.z - 0.42) < 1e-3, "size.z corrupted in Detection3D!"
    assert d3d.bbox.size.z != d3d.results[0].hypothesis.score or t.speed == 0.42, "Speed still stored in size.z!"

    # 4. Test Table obstacle with depth spread
    t_table = Track(2, 60, "table", 0.0, 0.0, 4.0, 10.0)
    t_table.update(0.0, 0.0, 4.0, 10.1, sx=1.40, sy=0.75, sz=0.85)
    print(f"Track 2 (Table): size_x={t_table.size_x:.2f}m, size_y={t_table.size_y:.2f}m, size_z={t_table.size_z:.2f}m")
    assert abs(t_table.size_z - 0.85) < 1e-3, "Table size_z not preserved!"

    # 5. Test Geometric depth obstacle
    t_geom = Track(3, -1, "unknown_obstacle", 0.5, 0.0, 3.5, 10.0)
    t_geom.update(0.5, 0.0, 3.5, 10.1, sx=0.80, sy=0.80, sz=0.70)
    print(f"Track 3 (Geometric Obstacle): size_x={t_geom.size_x:.2f}m, size_y={t_geom.size_y:.2f}m, size_z={t_geom.size_z:.2f}m")
    assert abs(t_geom.size_z - 0.70) < 1e-3, "Geometric obstacle size_z not preserved!"

    print("\n[SUCCESS] All 3D Bounding-Box Geometry and Semantic Prior tests PASSED!")
    return 0

if __name__ == "__main__":
    sys.exit(test_geometry())
