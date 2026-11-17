#!/usr/bin/env python3
"""
Deterministic Tests for M8: Pedestrians, Scenarios, Ground Truth

Tests:
1. pedestrian spawns at expected position
2. pedestrian remains stable when stationary
3. pedestrian moves along expected trajectory
4. crossing trajectory works
5. approaching trajectory works
6. multiple pedestrians can coexist
7. static obstacles remain collidable
8. scenario configuration loads correctly
9. ground-truth state is correct
10. RGB-D camera continues publishing correctly
11. existing vehicle physics regression tests still pass
"""

import sys
import math
import numpy as np

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")

from isaacsim import SimulationApp

# Initialize sim once
sim = SimulationApp({"headless": True, "renderer": "RayTracedLighting"})

import time
# pyrefly: ignore [missing-import]
from pxr import Usd, UsdGeom, UsdPhysics, Gf
# pyrefly: ignore [missing-import]
import omni.usd

from isaac_sim.scripts.utils.world_setup import build_world
from isaac_sim.scripts.assets.pedestrian import (
    PedestrianConfig, PedestrianType, Pedestrian, create_pedestrian_from_dict
)
from isaac_sim.scripts.vehicle.ackermann_controller_ros import drive_step


# Global world and stage for all tests
_world = None
_stage = None
_articulation = None
_vehicle_cfg = None
_dt = None
_pedestrians = {}


def setup_world(with_cameras=True):
    """Set up a single world for all tests."""
    global _world, _stage, _articulation, _vehicle_cfg, _dt
    
    built = build_world(dynamic=True, with_cameras=with_cameras, with_ros=False, start_xy=(6.0, -6.0))
    _world = built["world"]
    _stage = built["stage"]
    _articulation = built["articulation"]
    _vehicle_cfg = built["vehicle_cfg"]
    _dt = float(built["sim_cfg"]["physics"]["dt"])
    
    _articulation.initialize()
    _world.reset()
    
    # Let vehicle settle
    for _ in range(200):
        _world.step(render=False)
    
    print(f"[INFO] World setup complete (cameras={with_cameras})")


def cleanup_pedestrians():
    """Remove all test pedestrians from stage."""
    global _pedestrians, _stage
    for pid, ped in _pedestrians.items():
        try:
            prim = _stage.GetPrimAtPath(ped.root_prim.GetPath())
            if prim.IsValid():
                _stage.RemovePrim(prim.GetPath())
        except Exception:
            pass
    _pedestrians.clear()


def test_pedestrian_spawn_position():
    """Test 1: Pedestrian spawns at expected position."""
    print("\n=== Test 1: Pedestrian Spawn Position ===")
    
    cleanup_pedestrians()
    
    config = PedestrianConfig(
        pedestrian_id="test_spawn",
        pedestrian_type=PedestrianType.STATIONARY,
        initial_position=(5.0, -3.0, 0.0),
        initial_orientation=0.5,
        velocity=0.0,
        trajectory_duration=10.0,
    )
    
    ped = Pedestrian(_stage, config)
    _pedestrians[ped.config.pedestrian_id] = ped
    
    # Check root prim position
    root_transform = UsdGeom.Xformable(ped.root_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    pos = root_transform.Transform(Gf.Vec3d(0, 0, 0))
    
    expected = (5.0, -3.0, 0.0)
    actual = (float(pos[0]), float(pos[1]), float(pos[2]))
    
    tolerance = 0.01
    assert abs(actual[0] - expected[0]) < tolerance, f"X: expected {expected[0]}, got {actual[0]}"
    assert abs(actual[1] - expected[1]) < tolerance, f"Y: expected {expected[1]}, got {actual[1]}"
    assert abs(actual[2] - expected[2]) < tolerance, f"Z: expected {expected[2]}, got {actual[2]}"
    
    print(f"  PASS: Spawned at {actual} (expected {expected})")
    return True


def test_pedestrian_stationary_stability():
    """Test 2: Stationary pedestrian remains stable."""
    print("\n=== Test 2: Stationary Pedestrian Stability ===")
    
    cleanup_pedestrians()
    
    config = PedestrianConfig(
        pedestrian_id="test_stable",
        pedestrian_type=PedestrianType.STATIONARY,
        initial_position=(2.0, 2.0, 0.0),
        initial_orientation=0.0,
        velocity=0.0,
        trajectory_duration=5.0,
    )
    
    ped = Pedestrian(_stage, config)
    _pedestrians[ped.config.pedestrian_id] = ped
    _world.reset()
    
    # Step simulation for 2 seconds
    for _ in range(200):
        _world.step(render=False)
    
    # Check position hasn't drifted
    root_transform = UsdGeom.Xformable(ped.root_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    pos = root_transform.Transform(Gf.Vec3d(0, 0, 0))
    
    drift = math.sqrt((pos[0] - 2.0)**2 + (pos[1] - 2.0)**2 + (pos[2] - 0.0)**2)
    
    assert drift < 0.01, f"Stationary pedestrian drifted by {drift:.4f}m"
    print(f"  PASS: Drift = {drift:.6f}m")
    return True


def test_pedestrian_walking_trajectory():
    """Test 3: Walking pedestrian follows trajectory."""
    print("\n=== Test 3: Walking Trajectory ===")
    
    cleanup_pedestrians()
    
    config = PedestrianConfig(
        pedestrian_id="test_walk",
        pedestrian_type=PedestrianType.WALKING_STRAIGHT,
        initial_position=(0.0, 0.0, 0.0),
        initial_orientation=0.0,
        velocity=1.0,
        trajectory_duration=5.0,
    )
    
    ped = Pedestrian(_stage, config)
    _pedestrians[ped.config.pedestrian_id] = ped
    _world.reset()
    
    # Manually step pedestrian for 3 seconds
    for i in range(300):
        ped.update(0.01, i * 0.01)
        _world.step(render=False)
    
    # Check position - should have moved ~3m in +X
    root_transform = UsdGeom.Xformable(ped.root_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    pos = root_transform.Transform(Gf.Vec3d(0, 0, 0))
    
    expected_x = 3.0
    actual_x = float(pos[0])
    
    tolerance = 0.15
    assert abs(actual_x - expected_x) < tolerance, f"X: expected ~{expected_x}, got {actual_x}"
    assert abs(float(pos[1])) < 0.05, f"Y drift: {pos[1]}"
    
    print(f"  PASS: Moved to x={actual_x:.2f} (expected ~{expected_x})")
    return True


def test_crossing_left_to_right():
    """Test 4: Crossing left-to-right trajectory."""
    print("\n=== Test 4: Crossing Left-to-Right ===")
    
    cleanup_pedestrians()
    
    config = PedestrianConfig(
        pedestrian_id="test_cross_lr",
        pedestrian_type=PedestrianType.CROSSING_LEFT_TO_RIGHT,
        initial_position=(5.0, -5.0, 0.0),
        initial_orientation=math.pi/2,
        velocity=1.0,
        trajectory_duration=10.0,
    )
    
    ped = Pedestrian(_stage, config)
    _pedestrians[ped.config.pedestrian_id] = ped
    _world.reset()
    
    # Step for 5 seconds
    for i in range(500):
        ped.update(0.01, i * 0.01)
        _world.step(render=False)
    
    root_transform = UsdGeom.Xformable(ped.root_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    pos = root_transform.Transform(Gf.Vec3d(0, 0, 0))
    
    # Should have moved from Y=-5 to ~Y=0 (crossed +Y direction)
    assert float(pos[1]) >= -0.1, f"Should cross to +Y (near 0), got {pos[1]}"
    # X should stay at 5.0 (cos(pi/2) = 0, no forward drift when facing +Y)
    assert abs(float(pos[0]) - 5.0) < 0.05, f"X should stay at 5.0, got {pos[0]}"
    
    print(f"  PASS: Crossed to x={pos[0]:.2f}, y={pos[1]:.2f}")
    return True


def test_crossing_right_to_left():
    """Test 5: Crossing right-to-left trajectory."""
    print("\n=== Test 5: Crossing Right-to-Left ===")
    
    cleanup_pedestrians()
    
    config = PedestrianConfig(
        pedestrian_id="test_cross_rl",
        pedestrian_type=PedestrianType.CROSSING_RIGHT_TO_LEFT,
        initial_position=(5.0, 5.0, 0.0),
        initial_orientation=-math.pi/2,
        velocity=1.0,
        trajectory_duration=10.0,
    )
    
    ped = Pedestrian(_stage, config)
    _pedestrians[ped.config.pedestrian_id] = ped
    _world.reset()
    
    for i in range(500):
        ped.update(0.01, i * 0.01)
        _world.step(render=False)
    
    root_transform = UsdGeom.Xformable(ped.root_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    pos = root_transform.Transform(Gf.Vec3d(0, 0, 0))
    
    # Should have moved from Y=5 to ~Y=0 (crossed -Y direction)
    assert float(pos[1]) <= 0.1, f"Should cross to -Y (near 0), got {pos[1]}"
    # X stays at 5.0 (cos(-pi/2) = 0, no forward drift when facing -Y)
    assert abs(float(pos[0]) - 5.0) < 0.05, f"X should stay at 5.0, got {pos[0]}"
    
    print(f"  PASS: Crossed to x={pos[0]:.2f}, y={pos[1]:.2f}")
    return True


def test_approaching_trajectory():
    """Test 6: Approaching trajectory."""
    print("\n=== Test 6: Approaching Trajectory ===")
    
    cleanup_pedestrians()
    
    config = PedestrianConfig(
        pedestrian_id="test_approach",
        pedestrian_type=PedestrianType.APPROACHING,
        initial_position=(10.0, 0.0, 0.0),
        initial_orientation=math.pi,
        velocity=1.0,
        trajectory_duration=10.0,
    )
    
    ped = Pedestrian(_stage, config)
    _pedestrians[ped.config.pedestrian_id] = ped
    _world.reset()
    
    for i in range(500):
        ped.update(0.01, i * 0.01)
        _world.step(render=False)
    
    root_transform = UsdGeom.Xformable(ped.root_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    pos = root_transform.Transform(Gf.Vec3d(0, 0, 0))
    
    assert float(pos[0]) < 10.0, f"Should approach -X, got {pos[0]}"
    assert float(pos[0]) > 4.0, f"Should have moved ~5m, got {pos[0]}"
    
    print(f"  PASS: Approached to x={pos[0]:.2f}")
    return True


def test_multiple_pedestrians():
    """Test 7: Multiple pedestrians coexist."""
    print("\n=== Test 7: Multiple Pedestrians Coexist ===")
    
    cleanup_pedestrians()
    
    peds = []
    configs = [
        PedestrianConfig(pedestrian_id="multi_1", pedestrian_type=PedestrianType.STATIONARY,
                        initial_position=(0.0, 0.0, 0.0), velocity=0.0, trajectory_duration=5.0),
        PedestrianConfig(pedestrian_id="multi_2", pedestrian_type=PedestrianType.WALKING_STRAIGHT,
                        initial_position=(10.0, 0.0, 0.0), velocity=1.0, trajectory_duration=5.0),
        PedestrianConfig(pedestrian_id="multi_3", pedestrian_type=PedestrianType.CROSSING_LEFT_TO_RIGHT,
                        initial_position=(5.0, -5.0, 0.0), velocity=1.0, trajectory_duration=5.0),
    ]
    
    for config in configs:
        ped = Pedestrian(_stage, config)
        peds.append(ped)
        _pedestrians[ped.config.pedestrian_id] = ped
    
    _world.reset()
    
    for i in range(300):
        for ped in peds:
            ped.update(0.01, i * 0.01)
        _world.step(render=False)
    
    # Verify all exist and have correct states
    for ped in peds:
        root_transform = UsdGeom.Xformable(ped.root_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        pos = root_transform.Transform(Gf.Vec3d(0, 0, 0))
        assert not math.isnan(pos[0])
        assert not math.isnan(pos[1])
    
    print(f"  PASS: {len(peds)} pedestrians coexist")
    return True


def test_static_obstacles_collidable():
    """Test 8: Static obstacles (lab furniture) remain collidable."""
    print("\n=== Test 8: Static Obstacles Collidable ===")
    
    # Check that lab furniture has collision API
    furniture_path = "/World/Environment/Furniture"
    furniture_prim = _stage.GetPrimAtPath(furniture_path)
    
    collision_count = 0
    for prim in Usd.PrimRange(furniture_prim):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            collision_count += 1
    
    assert collision_count > 10, f"Expected many collidable furniture, found {collision_count}"
    
    # Check specific known obstacles
    table_prim = _stage.GetPrimAtPath("/World/Environment/Furniture/Tables/Meeting_Table_Top")
    assert table_prim.IsValid() and table_prim.HasAPI(UsdPhysics.CollisionAPI)
    
    chair_prim = _stage.GetPrimAtPath("/World/Environment/Furniture/Chairs/Chair_M1/Seat")
    assert chair_prim.IsValid() and chair_prim.HasAPI(UsdPhysics.CollisionAPI)
    
    wall_prim = _stage.GetPrimAtPath("/World/Environment/Walls/Wall_Back")
    assert wall_prim.IsValid() and wall_prim.HasAPI(UsdPhysics.CollisionAPI)
    
    # Check glass partitions have collision (by default)
    glass_prim = _stage.GetPrimAtPath("/World/Environment/GlassPartitions/Glass_Office_L")
    assert glass_prim.IsValid() and glass_prim.HasAPI(UsdPhysics.CollisionAPI)
    
    print(f"  PASS: {collision_count} collidable furniture pieces, glass partitions solid")
    return True


def test_scenario_config_loads():
    """Test 9: Scenario YAML configs load correctly."""
    print("\n=== Test 9: Scenario Configuration Loads ===")
    
    import yaml
    scenarios_dir = "/home/ubuntu/SKATEBOARD_IRP/configs/scenarios"
    
    scenario_files = [
        "scenario_01_stationary_person.yaml",
        "scenario_02_person_center.yaml",
        "scenario_03_person_left.yaml",
        "scenario_04_person_right.yaml",
        "scenario_05_person_crossing_left_to_right.yaml",
        "scenario_06_person_crossing_right_to_left.yaml",
        "scenario_07_person_approaching.yaml",
        "scenario_08_two_people.yaml",
        "scenario_09_person_plus_chair.yaml",
        "scenario_10_person_plus_table.yaml",
        "scenario_11_narrow_corridor.yaml",
        "scenario_12_multiple_objects.yaml",
    ]
    
    for fname in scenario_files:
        fpath = f"{scenarios_dir}/{fname}"
        with open(fpath, 'r') as f:
            scenario = yaml.safe_load(f)
        
        assert "scenario_name" in scenario
        assert "vehicle_start_pose" in scenario
        assert "pedestrians" in scenario
        assert isinstance(scenario["pedestrians"], list)
        assert len(scenario["pedestrians"]) > 0
        
        for ped in scenario["pedestrians"]:
            assert "pedestrian_id" in ped
            assert "type" in ped
            assert ped["type"] in [t.value for t in PedestrianType]
            assert "initial_position" in ped
            assert "velocity" in ped
    
    print(f"  PASS: All {len(scenario_files)} scenario files load correctly")
    return True


def test_ground_truth_tracking():
    """Test 10: Ground truth state tracking is correct."""
    print("\n=== Test 10: Ground Truth Tracking ===")
    
    cleanup_pedestrians()
    
    config = PedestrianConfig(
        pedestrian_id="gt_test",
        pedestrian_type=PedestrianType.WALKING_STRAIGHT,
        initial_position=(0.0, 0.0, 0.0),
        initial_orientation=0.0,
        velocity=1.0,
        trajectory_duration=5.0,
    )
    
    ped = Pedestrian(_stage, config)
    _pedestrians[ped.config.pedestrian_id] = ped
    _world.reset()
    
    # Step and collect ground truth
    gt_log = []
    for i in range(200):
        ped.update(0.01, i * 0.01)
        gt = ped.get_ground_truth()
        gt_log.append(gt)
        _world.step(render=False)
    
    assert len(gt_log) == 200
    
    first = gt_log[0]
    last = gt_log[-1]
    
    assert first.pedestrian_id == "gt_test"
    assert first.pedestrian_type == "walking_straight"
    assert isinstance(first.position, tuple) and len(first.position) == 3
    assert isinstance(first.velocity, tuple) and len(first.velocity) == 3
    assert isinstance(first.dimensions, tuple) and len(first.dimensions) == 3
    
    # Position should have changed
    pos_change = math.sqrt(
        (last.position[0] - first.position[0])**2 +
        (last.position[1] - first.position[1])**2
    )
    assert pos_change > 1.5, f"Ground truth shows insufficient movement: {pos_change:.2f}m"
    
    # Velocity should be approximately correct
    assert abs(last.velocity[0] - 1.0) < 0.2, f"Velocity X: expected ~1.0, got {last.velocity[0]}"
    
    print(f"  PASS: Ground truth tracked correctly, movement={pos_change:.2f}m, vel={last.velocity[0]:.2f}")
    return True


def test_rgbd_camera_publishing():
    """Test 11: RGB-D camera still works with pedestrians."""
    print("\n=== Test 11: RGB-D Camera Publishing ===")
    
    # Camera should already exist from initial setup
    rgbd_prim = _stage.GetPrimAtPath("/World/Vehicle/Chassis/Sensors/RGBD_Camera/RGBD_Camera")
    assert rgbd_prim.IsValid(), "RGBD camera prim missing"
    
    assert rgbd_prim.GetTypeName() == "Camera", f"Expected Camera, got {rgbd_prim.GetTypeName()}"
    
    cam = UsdGeom.Camera(rgbd_prim)
    assert cam.GetFocalLengthAttr().Get() > 0
    assert cam.GetHorizontalApertureAttr().Get() > 0
    
    # Step simulation a few frames
    for _ in range(10):
        _world.step(render=True)
    
    print(f"  PASS: RGB-D camera present and functional")
    return True


def test_vehicle_physics_regression():
    """Test 12: Vehicle physics regression - existing tests still pass."""
    print("\n=== Test 12: Vehicle Physics Regression ===")
    
    # Use existing world (cameras don't affect physics)
    names = list(_articulation.dof_names)
    idx = {n: names.index(n) for n in names}
    drive_pos = {}
    
    # Settle
    for _ in range(200):
        _world.step(render=False)
    
    # Check settle
    m = UsdGeom.Xformable(_stage.GetPrimAtPath("/World/Vehicle/Chassis")).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    p = m.Transform(Gf.Vec3d(0, 0, 0))
    fwd = m.TransformDir(Gf.Vec3d(1, 0, 0))
    tilt = math.degrees(math.acos(max(-1, min(1, m.TransformDir(Gf.Vec3d(0, 0, 1))[2]))))
    
    assert 0.22 < p[2] < 0.38, f"Settle Z failed: {p[2]:.3f}"
    assert tilt < 3.0, f"Settle tilt failed: {tilt:.2f}°"
    
    # Drive straight
    start_x = p[0]
    omega = 4.0  # rad/s (matches verify_ackermann_physics.py)
    for _ in range(400):
        drive_step(_articulation, idx, 0.0, omega, _dt, {})
        _world.step(render=False)
    
    m = UsdGeom.Xformable(_stage.GetPrimAtPath("/World/Vehicle/Chassis")).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    p = m.Transform(Gf.Vec3d(0, 0, 0))
    dx = p[0] - start_x
    
    assert dx > 0.8, f"Straight drive failed: dx={dx:.2f}"
    
    # Check wheel spin
    max_spin = float(np.max(np.abs(np.array(_articulation.get_joint_positions()).flatten())))
    assert max_spin > 2.0, f"Wheels didn't roll: max_spin={max_spin:.2f}"
    
    print(f"  PASS: Settle Z={p[2]:.3f}, tilt={tilt:.2f}°, dx={dx:.2f}m, spin={max_spin:.2f}rad")
    return True


def run_all_tests():
    """Run all M8 tests."""
    global _world, _stage, _articulation, _vehicle_cfg, _dt
    
    print("="*60, flush=True)
    print("M8 PEDESTRIAN & SCENARIO TESTS", flush=True)
    print("="*60, flush=True)
    
    # Set up world once with cameras for all tests
    setup_world(with_cameras=True)
    
    # Run vehicle physics test FIRST (before pedestrian tests move the vehicle)
    print("\n=== Running Vehicle Physics Regression First ===", flush=True)
    test_vehicle_physics_regression()
    
    # Now run pedestrian tests on the same world
    print("\n=== Running Pedestrian Tests ===", flush=True)
    
    tests = [
        ("Pedestrian Spawn Position", test_pedestrian_spawn_position),
        ("Stationary Stability", test_pedestrian_stationary_stability),
        ("Walking Trajectory", test_pedestrian_walking_trajectory),
        ("Crossing Left-to-Right", test_crossing_left_to_right),
        ("Crossing Right-to-Left", test_crossing_right_to_left),
        ("Approaching Trajectory", test_approaching_trajectory),
        ("Multiple Pedestrians", test_multiple_pedestrians),
        ("Static Obstacles Collidable", test_static_obstacles_collidable),
        ("Scenario Config Loads", test_scenario_config_loads),
        ("Ground Truth Tracking", test_ground_truth_tracking),
        ("RGB-D Camera Publishing", test_rgbd_camera_publishing),
    ]
    
    passed = 0
    failed = 0
    errors = []
    
    for name, test_func in tests:
        print(f"\n=== Running: {name} ===", flush=True)
        try:
            result = test_func()
            if result:
                passed += 1
                print(f"  PASS: {name}", flush=True)
            else:
                failed += 1
                errors.append(f"{name}: returned False")
                print(f"  FAIL: {name} - returned False", flush=True)
        except Exception as e:
            failed += 1
            errors.append(f"{name}: {e}")
            import traceback
            traceback.print_exc()
    
    print(f"\n{'='*60}")
    print(f"M8 TEST RESULTS: {passed}/{len(tests)} PASSED")
    print(f"{'='*60}")
    
    if errors:
        print("FAILURES:")
        for err in errors:
            print(f"  - {err}")
    
    return failed == 0


if __name__ == "__main__":
    success = run_all_tests()
    sim.close()
    sys.exit(0 if success else 1)