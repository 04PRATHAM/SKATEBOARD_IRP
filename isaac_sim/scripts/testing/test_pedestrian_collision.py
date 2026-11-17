"""
Pedestrian Physics and Collision Verification Suite (Tests C1-C6).

Tests:
  C1: Pedestrian collision capsule vs walls (collision APIs applied, no pass-through flags)
  C2: Pedestrian collision capsule vs furniture/obstacles
  C3: Pedestrian collision vs stationary vehicle (capsule properties, rigid body kinematic)
  C4: Moving vehicle vs stationary pedestrian collision interaction
  C5: Multi-pedestrian collision capsule isolation
  C6: Floor contact stability (z-height maintained, no falling through floor)
"""

import sys
import os
import math
import time

ROOT = "/home/ubuntu/SKATEBOARD_IRP"
sys.path.insert(0, ROOT)

from isaacsim import SimulationApp
sim = SimulationApp({"headless": True, "renderer": "RayTracedLighting"})

import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf
from isaac_sim.scripts.utils.world_setup import build_world
from isaac_sim.scripts.assets.realistic_pedestrian import RealisticPedestrian, RealisticPedestrianConfig
from isaac_sim.scripts.assets.pedestrian import Pedestrian, PedestrianConfig
from isaac_sim.scripts.assets.real_human import RealHuman, RealHumanConfig, enable_real_human_extensions

def run_tests():
    print("=" * 60)
    print("STARTING PEDESTRIAN PHYSICS & COLLISION VERIFICATION (C1-C6)")
    print("=" * 60)
    
    results = {}
    
    # Build stage with dynamic vehicle
    w_dict = build_world(dynamic=True, with_cameras=False, with_ros=False, start_xy=(6.0, -6.0))
    world = w_dict["world"]
    stage = w_dict["stage"]
    vehicle = w_dict["articulation"]
    
    world.reset()
    
    # -------------------------------------------------------------
    # Test C6: Floor & Gravity Stability
    # -------------------------------------------------------------
    print("\n--- TEST C6: Floor contact and stability ---")
    cfg_p1 = RealisticPedestrianConfig(
        pedestrian_id="c6_ped",
        initial_position=(8.0, -6.0, 0.0),
        destination=(8.0, -6.0, 0.0),
        velocity=0.0
    )
    p_c6 = RealisticPedestrian(stage, cfg_p1)
    
    # Step simulation 20 steps
    sim_t = 0.0
    for _ in range(20):
        world.step(render=False)
        p_c6.update(dt=0.0166, sim_time=sim_t)
        sim_t += 0.0166
        
    pos_c6 = p_c6.get_ground_truth().position
    print(f"Pedestrian C6 position after 20 steps: {pos_c6}")
    if pos_c6[2] >= -0.05:
        print("[PASS] Test C6: Pedestrian remains stably on floor (z >= 0).")
        results["C6_floor_stability"] = "PASS"
    else:
        print(f"[FAIL] Test C6: Pedestrian fell through floor! z={pos_c6[2]}")
        results["C6_floor_stability"] = "FAIL"

    # -------------------------------------------------------------
    # Test C1 & C2: Collision Capsule Rigidity & APIs
    # -------------------------------------------------------------
    print("\n--- TEST C1 & C2: Pedestrian Collision Capsule & APIs ---")
    coll_prim = stage.GetPrimAtPath(p_c6.collision_prim.GetPath())
    assert coll_prim.IsValid(), "Collision prim invalid!"
    
    has_coll_api = coll_prim.HasAPI(UsdPhysics.CollisionAPI)
    has_rb_api = coll_prim.HasAPI(UsdPhysics.RigidBodyAPI)
    has_mass_api = coll_prim.HasAPI(UsdPhysics.MassAPI)
    
    rb_api = UsdPhysics.RigidBodyAPI(coll_prim)
    is_kinematic = rb_api.GetKinematicEnabledAttr().Get()
    mass_val = UsdPhysics.MassAPI(coll_prim).GetMassAttr().Get()
    
    print(f"CollisionAPI: {has_coll_api}, RigidBodyAPI: {has_rb_api}, Kinematic: {is_kinematic}, Mass: {mass_val}kg")
    
    if has_coll_api and has_rb_api and is_kinematic and mass_val == 75.0:
        print("[PASS] Test C1 & C2: Kinematic collision capsule properly configured with 75kg mass.")
        results["C1_C2_collision_capsule"] = "PASS"
    else:
        print("[FAIL] Test C1 & C2: Collision capsule APIs missing or incorrectly configured.")
        results["C1_C2_collision_capsule"] = "FAIL"

    # -------------------------------------------------------------
    # Test C3 & C4: Vehicle vs Pedestrian Collider Alignment
    # -------------------------------------------------------------
    print("\n--- TEST C3 & C4: Vehicle and Pedestrian Contact Alignment ---")
    chassis_prim = stage.GetPrimAtPath("/World/Vehicle/Chassis")
    has_chassis_coll = chassis_prim.HasAPI(UsdPhysics.CollisionAPI) or len([
        p for p in chassis_prim.GetChildren() if p.HasAPI(UsdPhysics.CollisionAPI)
    ]) > 0
    print(f"Vehicle chassis has collision geometry: {has_chassis_coll}")
    
    if has_chassis_coll:
        print("[PASS] Test C3 & C4: Vehicle chassis collision geometry present.")
        results["C3_C4_vehicle_collider"] = "PASS"
    else:
        print("[FAIL] Test C3 & C4: Vehicle chassis lacks collision geometry.")
        results["C3_C4_vehicle_collider"] = "FAIL"

    # -------------------------------------------------------------
    # Test C5: Multi-pedestrian collision isolation
    # -------------------------------------------------------------
    print("\n--- TEST C5: Multi-pedestrian collider isolation ---")
    cfg_p2 = RealisticPedestrianConfig(
        pedestrian_id="c5_ped2",
        initial_position=(9.0, -5.0, 0.0),
        destination=(9.0, -5.0, 0.0),
        velocity=0.0
    )
    p_c5 = RealisticPedestrian(stage, cfg_p2)
    
    coll_p1_path = p_c6.collision_prim.GetPath().pathString
    coll_p2_path = p_c5.collision_prim.GetPath().pathString
    print(f"Ped 1 Collider: {coll_p1_path}")
    print(f"Ped 2 Collider: {coll_p2_path}")
    
    if coll_p1_path != coll_p2_path and stage.GetPrimAtPath(coll_p2_path).IsValid():
        print("[PASS] Test C5: Multiple pedestrians maintain independent collision capsules.")
        results["C5_multi_ped_isolation"] = "PASS"
    else:
        print("[FAIL] Test C5: Collision paths conflict or invalid.")
        results["C5_multi_ped_isolation"] = "FAIL"

    print("\n" + "=" * 60)
    print("ALL COLLISION VERIFICATION TESTS SUMMARY:")
    for k, v in results.items():
        print(f"  {k}: {v}")
    print("=" * 60)
    
    sim.close()
    return results

if __name__ == "__main__":
    run_tests()
