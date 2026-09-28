print("[SCRIPT START] Module loading...", flush=True)

import sys
import os
import yaml
import time
import argparse
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional, Any
from pathlib import Path

# Add project root to path
sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")

# Parse args FIRST, then create SimulationApp, then import other Isaac Sim modules
parser = argparse.ArgumentParser(description="SKATEBOARD_IRP M8 Scenario Runner")
parser.add_argument("scenario", nargs="?", help="Path to scenario YAML file")
parser.add_argument("--all", action="store_true", help="Run all scenarios in configs/scenarios/")
parser.add_argument("--headless", action="store_true", help="Run headless")
parser.add_argument("--ros", action="store_true", help="Enable ROS 2 bridge")
parser.add_argument("--gui", action="store_true", help="GUI mode (stays open for inspection)")
parser.add_argument("--duration", type=float, default=60.0, help="Max duration per scenario (seconds)")
parser.add_argument("--wasd", action="store_true", help="Enable WASD manual control")
args, unknown = parser.parse_known_args()

print("[BEFORE SIM] About to create SimulationApp", flush=True)

# Now create SimulationApp
from isaacsim import SimulationApp
sim = SimulationApp({"headless": args.headless, "renderer": "RayTracedLighting"})

print("[SIM CREATED] SimulationApp created", flush=True)

# Now import other Isaac Sim modules AFTER SimulationApp is created
import sys
import os
import yaml
import time
import math
import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Gf

from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaac_sim.scripts.utils.world_setup import build_world, RGBD_CAMERA_PRIM
from isaac_sim.scripts.assets.pedestrian import (
    PedestrianConfig, PedestrianType, Pedestrian, 
    create_pedestrian_from_dict, PedestrianGroundTruth
)
# Realistic human upgrades
# 2026-09-16: segmented human (capsule head/torso/arms/legs + procedural walk) via realistic_pedestrian.py
# 2026-09-17: REAL S3 human (SkelRoot + Skinned Mesh + Skeleton + Biped anim graph) via real_human.py
# Runner chooses based on per-pedestrian flags:
#   is_real_human / use_real_human -> S3 RealHuman (photoreal, preferred for GUI evidence)
#   is_realistic / variant_idx      -> segmented realistic_pedestrian (fast headless fallback)
#   otherwise                       -> capsule pedestrian (legacy)
try:
    from isaac_sim.scripts.assets.realistic_pedestrian import (
        create_realistic_pedestrian_from_dict as _create_realistic_from_dict
    )
    HAS_REALISTIC = True
except Exception:
    HAS_REALISTIC = False
    _create_realistic_from_dict = None

try:
    from isaac_sim.scripts.assets.real_human import (
        RealHuman, RealHumanConfig, HumanType,
        enable_real_human_extensions, create_real_human,
        CHARACTER_VARIANTS,
    )
    HAS_REAL_HUMAN = True
except Exception as _e:
    HAS_REAL_HUMAN = False
    RealHuman = None
    enable_real_human_extensions = None
    create_real_human = None
    CHARACTER_VARIANTS = []

from isaac_sim.scripts.vehicle.ackermann_controller_ros import drive_step
from isaac_sim.scripts.testing.wasd_controller import WASDState


@dataclass
class ScenarioResult:
    """Results from a scenario run."""
    scenario_name: str
    success: bool
    duration: float
    pedestrian_ground_truths: Dict[str, List[PedestrianGroundTruth]]
    vehicle_poses: List[Dict]
    errors: List[str]


class ScenarioRunner:
    """Main scenario runner class."""
    
    def __init__(self, headless: bool = True, enable_ros: bool = False, 
                 gui_mode: bool = False, max_duration: float = 60.0):
        self.headless = headless
        self.enable_ros = enable_ros
        self.gui_mode = gui_mode  # if True, stays open for manual inspection
        self.max_duration = max_duration
        self.world = None
        self.stage = None
        self.articulation = None
        self.vehicle_cfg = None
        self.sim_cfg = None
        self.dt = 0.01
        self.pedestrians: List[Pedestrian] = []
        self.ground_truth_log: Dict[str, List[PedestrianGroundTruth]] = {}
        self.vehicle_pose_log: List[Dict] = []
        self.scenario_name = ""
        self.start_time = 0.0
        self.wasd = WASDState(max_speed=1.5, max_steer=0.6)
        self.manual_control = False
    
    def initialize_sim(self, defer_for_anim: bool = False):
        """Initialize world using existing global sim.

        Args:
            defer_for_anim: when True (real-human scenarios), build stage-only
                (no World, no ROS bridge). The animation save/reopen happens
                in finalize_setup(), and the single World + bridge are created
                AFTER the reopen (a pre-reopen World would tick against the
                invalidated stage and hang).
        """
        # Build world with scenario-appropriate settings
        built = build_world(
            dynamic=True,
            with_cameras=True,
            with_ros=self.enable_ros and not defer_for_anim,
            start_xy=None,  # will be overridden per scenario
            glass_has_collision=True,
            create_world=not defer_for_anim,
        )
        self.world = built["world"]
        self.stage = built["stage"]
        self.articulation = built["articulation"]
        self.vehicle_cfg = built["vehicle_cfg"]
        self.camera_cfg = built["camera_cfg"]
        self.sim_cfg = built["sim_cfg"]
        self.dt = float(self.sim_cfg["physics"]["dt"])
        self._needs_anim_reopen = False

        if self.world is not None:
            self.articulation.initialize()
            self.world.reset()
        print(f"[INFO] Scenario runner initialized, dt={self.dt} "
              f"(world={'live' if self.world is not None else 'deferred'})")
    
    def load_scenario(self, scenario_path: str) -> dict:
        """Load scenario from YAML file."""
        with open(scenario_path, 'r') as f:
            scenario = yaml.safe_load(f)
        return scenario
    
    def setup_scenario(self, scenario: dict):
        """Set up scenario: spawn pedestrians, configure vehicle start pose."""
        self.scenario_name = scenario.get("scenario_name", "unknown")
        print(f"[INFO] Setting up scenario: {self.scenario_name}")
        
        # Set vehicle start pose
        vehicle_start = scenario.get("vehicle_start_pose", [6.0, -6.0, 0.35, 0.0])
        vx, vy, vz, vyaw = vehicle_start
        
        # Reposition vehicle: pose goes on /World/Vehicle (the spawn group),
        # Chassis local stays identity. (Writing the scenario pose into the
        # Chassis translate used to DOUBLE the build offset — world (12,-12,
        # 0.70): wall intersection + floating wheels. Fixed 2026-09-24.)
        vehicle_prim = self.stage.GetPrimAtPath("/World/Vehicle")
        if vehicle_prim.IsValid():
            xform = UsdGeom.Xformable(vehicle_prim)
            for op in xform.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    op.Set(Gf.Vec3d(vx, vy, vz))
                elif op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                    op.Set(Gf.Vec3d(0, 0, math.degrees(vyaw)))
            has_rotate = any(op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ
                           for op in xform.GetOrderedXformOps())
            if not has_rotate:
                xform.AddRotateXYZOp().Set(Gf.Vec3d(0, 0, math.degrees(vyaw)))
        chassis_prim = self.stage.GetPrimAtPath("/World/Vehicle/Chassis")
        if chassis_prim.IsValid():
            # Chassis is the physics floating base: local identity so the
            # Vehicle-group pose is applied exactly once.
            cxform = UsdGeom.Xformable(chassis_prim)
            for op in cxform.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    op.Set(Gf.Vec3d(0.0, 0.0, 0.0))
                elif op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                    op.Set(Gf.Vec3d(0, 0, 0))
        
        # Create pedestrians — 3 tiers:
        #   Tier 1 (best): is_real_human/use_real_human -> REAL S3 human (SkelRoot Skinned Mesh, Biped graph)
        #   Tier 2 (good): is_realistic/variant_idx    -> segmented human (capsules+walk cycle, headless-fast fallback)
        #   Tier 3 (legacy): otherwise                  -> capsule placeholder (kept for regression only)
        self.pedestrians = []
        self.ground_truth_log = {}

        # Enable real human stack if any ped requests it (must be inside SimulationApp)
        needs_real = any(bool(p.get("is_real_human", False) or p.get("use_real_human", False))
                         for p in scenario.get("pedestrians", [])) or bool(scenario.get("is_real_human", False))
        if needs_real and HAS_REAL_HUMAN and enable_real_human_extensions is not None:
            try:
                enable_real_human_extensions()
                print("[INFO] Real human extensions enabled for scenario", self.scenario_name, flush=True)
            except Exception as e:
                print(f"[WARN] enable_real_human_extensions failed: {e}", flush=True)

        for ped_config in scenario.get("pedestrians", []):
            is_real_human = bool(ped_config.get("is_real_human", False) or ped_config.get("use_real_human", False))
            use_realistic = bool(ped_config.get("is_realistic", False) or "variant_idx" in ped_config or scenario.get("is_realistic", False))
            ped = None
            if is_real_human and HAS_REAL_HUMAN and create_real_human is not None:
                try:
                    ped = create_real_human(self.stage, ped_config)
                    print(f"  Created REAL S3 human: {ped.config.pedestrian_id} "
                          f"(variant={ped.variant_idx} {ped.label}) at {ped.config.position} yaw={ped.config.yaw_deg}deg")
                except Exception as e:
                    print(f"  [WARN] Real S3 human spawn failed for {ped_config.get('pedestrian_id')}: {e} — falling back to segmented", flush=True)
                    ped = None
            if ped is None and use_realistic and HAS_REALISTIC and _create_realistic_from_dict is not None:
                ped = _create_realistic_from_dict(self.stage, ped_config)
                ped_type = ped.config.pedestrian_type.value
                anim = ped_config.get("animation", "walk" if ped_type != "stationary" else "idle")
                print(f"  Created REALISTIC (segmented) human: {ped.config.pedestrian_id} "
                      f"({ped_type}, anim={anim}, variant={ped_config.get('variant_idx', 'auto')}) at {ped.config.initial_position}")
            if ped is None:
                ped = create_pedestrian_from_dict(self.stage, ped_config)
                print(f"  Created pedestrian: {ped.config.pedestrian_id} "
                      f"({ped.config.pedestrian_type.value}) at {ped.config.initial_position}")
            self.pedestrians.append(ped)
            # RealHuman uses .config.pedestrian_id, others use .config.pedestrian_id too — unify
            _pid = getattr(ped.config, "pedestrian_id", str(id(ped)))
            self.ground_truth_log[_pid] = []
        
        # NOTE: no world.reset() and no walk triggers here anymore — see
        # finalize_setup(): real-human scenarios are exported + reopened first
        # (AnimationGraph discovery requires characters present at stage
        # attach), then reset, then triggers.
        self._needs_anim_reopen = any(
            ped.__class__.__name__ == "RealHuman" for ped in self.pedestrians
        )
        print(f"[INFO] Scenario setup complete: {len(self.pedestrians)} pedestrians "
              f"(anim_reopen={self._needs_anim_reopen})")

    def finalize_setup(self):
        """Reset physics and trigger walks, reopening the stage first if needed.

        Real-human path: export the built stage (lab + vehicle + wired S3
        humans, nothing rebuilt) and reopen it so the AnimationGraph runtime
        discovers the characters; then re-create World/Articulation wrappers
        on the reopened stage, reset, (re)build the ROS bridge, refresh human
        handles, and trigger graph walks. Non-human scenarios keep the exact
        legacy flow (reset only).
        """
        if self._needs_anim_reopen:
            from isaac_sim.scripts.assets.real_human import (
                save_and_reopen_stage_for_animation,
            )
            print("[INFO] Exporting + reopening stage for AnimationGraph discovery...",
                  flush=True)
            want = []
            for ped in self.pedestrians:
                if ped.__class__.__name__ == "RealHuman":
                    want.append(getattr(ped, "character_prim_path", ""))
                    want.append(getattr(ped, "_skelroot_path", ""))
            new_stage = save_and_reopen_stage_for_animation(self.stage, expected_paths=want)
            self.stage = new_stage
            # Create the single World on the reopened stage. The pre-reopen
            # articulation object is bound to the detached stage and must NOT
            # be reused (it poisons init); construct fresh on identical paths.
            # Order matches build_world: scene.add -> reset (creates the
            # physics simulation view) -> articulation.initialize().
            self.world = World(
                stage_units_in_meters=1.0,
                physics_dt=float(self.sim_cfg["physics"]["dt"]),
                rendering_dt=float(self.sim_cfg["physics"]["dt"]),
            )
            self.articulation = Articulation("/World/Vehicle/Chassis")
            self.world.scene.add(self.articulation)
            print("[ANIM-REOPEN] world created, resetting physics...", flush=True)
            self.world.reset()
            print("[ANIM-REOPEN] reset done, initializing articulation...", flush=True)
            self.articulation.initialize()
            print("[INFO] Reopened stage reset complete.", flush=True)
            if self.enable_ros:
                from isaac_sim.scripts.bridge.ros2_bridge import setup_ros2_bridge
                setup_ros2_bridge(RGBD_CAMERA_PRIM, self.camera_cfg,
                                  self.sim_cfg, vehicle_path="/World/Vehicle")
            for ped in self.pedestrians:
                if ped.__class__.__name__ == "RealHuman":
                    ped.refresh_after_reopen(new_stage)
            # Timeline MUST be playing or AnimationGraph MotionMatching never
            # advances (world.step moves physics but graph characters freeze:
            # triggers ok, zero displacement — found 2026-09-25).
            try:
                import omni.timeline as _otl
                _otl.get_timeline_interface().play()
            except Exception:
                pass
        else:
            self.world.reset()

        # Trigger walking for RealHuman S3 humans (requires runtime after reset for graph handle)
        for ped in self.pedestrians:
            if ped.__class__.__name__ == "RealHuman":
                try:
                    from isaac_sim.scripts.assets.real_human import HumanType as _HT
                    is_walking = ped.config.human_type == _HT.WALKING
                    if is_walking:
                        if ped.config.destination:
                            ok = ped.go_walk_to(ped.config.destination)
                            print(f"  -> RealHuman GRAPH walk triggered: {ped.config.pedestrian_id} to {ped.config.destination} ok={ok}", flush=True)
                        else:
                            ok = ped.go_walk_straight(velocity=ped.config.velocity, distance=8.0)
                            print(f"  -> RealHuman GRAPH walk straight triggered: {ped.config.pedestrian_id} v={ped.config.velocity} ok={ok}", flush=True)
                        if not ok:
                            print(f"  [FAIL] RealHuman walk trigger FAILED (no runtime Character, no fallback): {ped.config.pedestrian_id}", flush=True)
                        # prime graph walk blend for first step
                        ped.update(0.01)
                except Exception as e:
                    print(f"  [WARN] RealHuman walk trigger failed {ped.config.pedestrian_id}: {e}", flush=True)
    
    def step_pedestrians(self, dt: float, sim_time: float):
        """Update all pedestrians (capsule, segmented, and S3 real humans)."""
        for ped in self.pedestrians:
            # RealHuman has update(dt) only; capsule/segmented have update(dt, sim_time)
            is_real_human = ped.__class__.__name__ == "RealHuman"
            try:
                if is_real_human:
                    # S3 real human — graph walk pump (Walk ramp + nav update).
                    # No fallback exists: without a runtime Character this is
                    # a no-op and triggers report ok=False (honest failure).
                    ped.update(dt)
                    # Log GT for evaluation (real humans also logged, but flagged real_human=true)
                    # RealHuman has no PedestrianGroundTruth; synthesize from world pos
                    pos = ped.get_world_pos()
                    from isaac_sim.scripts.assets.pedestrian import PedestrianGroundTruth as _PGT
                    _pid = getattr(ped.config, "pedestrian_id", str(id(ped)))
                    gt = _PGT(pedestrian_id=_pid, timestamp=sim_time, position=pos,
                               orientation=math.radians(ped.config.yaw_deg), velocity=(0,0,0),
                               pedestrian_type="real_human", dimensions=(0.5,0.5,1.75))
                else:
                    if ped.is_active() or getattr(ped.state, "value", "") == "idle":
                        ped.update(dt, sim_time)
                    else:
                        # stationary that started idle but already marked idle — still tick once
                        try: ped.update(dt, sim_time)
                        except Exception: pass
                    gt = ped.get_ground_truth()
                    _pid = ped.config.pedestrian_id
            except Exception as e:
                # Don't crash the scenario if one ped update fails
                print(f"[WARN] ped update failed {ped}: {e}", flush=True)
                continue
            self.ground_truth_log[_pid].append(gt)
    
    def log_vehicle_pose(self, sim_time: float):
        """Log vehicle pose for analysis."""
        chassis_prim = self.stage.GetPrimAtPath("/World/Vehicle/Chassis")
        if chassis_prim.IsValid():
            m = UsdGeom.Xformable(chassis_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            p = m.Transform(Gf.Vec3d(0, 0, 0))
            fwd = m.TransformDir(Gf.Vec3d(1, 0, 0))
            yaw = math.atan2(fwd[1], fwd[0])
            
            self.vehicle_pose_log.append({
                "timestamp": sim_time,
                "x": float(p[0]),
                "y": float(p[1]),
                "z": float(p[2]),
                "yaw": float(yaw)
            })
    
    def run_scenario(self, scenario: dict, use_wasd: bool = False) -> ScenarioResult:
        """Run a single scenario."""
        self.setup_scenario(scenario)
        self.finalize_setup()
        
        # Get DOF indices for drive_step
        names = list(self.articulation.dof_names)
        idx = {n: names.index(n) for n in names}
        drive_pos = {}
        
        max_steps = int(self.max_duration / self.dt)
        sim_time = 0.0
        frame = 0
        
        print(f"[INFO] Running scenario for up to {self.max_duration}s ({max_steps} steps)", flush=True)
        
        try:
            while sim.is_running() and frame < max_steps:
                # Handle manual WASD input if enabled
                if use_wasd:
                    # Process keyboard events (handled by callback in run_wasd_test)
                    pass
                
                # Default: drive straight at 0.5 m/s for scenario progression
                if not self.manual_control:
                    omega = 0.5 / float(self.vehicle_cfg["wheel_radius"])
                    drive_step(self.articulation, idx, 0.0, omega, self.dt, drive_pos)
                
                # Step pedestrians
                self.step_pedestrians(self.dt, sim_time)
                
                # Step simulation (ROS bridge needs rendered frames even
                # headless — same rule as run_fidelity_world --ros2 — AND
                # real-human AnimationGraphs advance ONLY on rendered frames:
                # render=False moves physics but freezes graph characters
                # (probe 2026-09-25), so force render with anim reopen too.
                self.world.step(render=(not self.headless) or self.enable_ros
                                or self._needs_anim_reopen)
                
                # Log data
                self.log_vehicle_pose(sim_time)
                
                sim_time += self.dt
                frame += 1
                
                # Check if all pedestrians completed
                if all(p.is_completed() for p in self.pedestrians):
                    print(f"[INFO] All pedestrians completed at t={sim_time:.1f}s", flush=True)
                    # Continue for a bit to let vehicle pass
                    if sim_time > 10.0:
                        break
                
                # Periodic logging
                if frame % int(2.0 / self.dt) == 0:
                    active_count = sum(1 for p in self.pedestrians if p.is_active())
                    print(f"  t={sim_time:.1f}s: {active_count}/{len(self.pedestrians)} pedestrians active", flush=True)
        
        except KeyboardInterrupt:
            print("[INFO] Scenario interrupted by user", flush=True)
        except Exception as e:
            print(f"[ERROR] Scenario failed: {e}", flush=True)
            import traceback
            traceback.print_exc()
            return ScenarioResult(
                scenario_name=self.scenario_name,
                success=False,
                duration=sim_time,
                pedestrian_ground_truths=self.ground_truth_log,
                vehicle_poses=self.vehicle_pose_log,
                errors=[str(e)]
            )
        
        # GUI mode: keep open for inspection
        if self.gui_mode and not self.headless:
            print("[INFO] GUI mode: keeping simulation open for inspection...", flush=True)
            print("       Close the viewport window to exit.", flush=True)
            try:
                while sim.is_running():
                    self.world.step(render=True)
            except KeyboardInterrupt:
                pass
        
        return ScenarioResult(
            scenario_name=self.scenario_name,
            success=True,
            duration=sim_time,
            pedestrian_ground_truths=self.ground_truth_log,
            vehicle_poses=self.vehicle_pose_log,
            errors=[]
        )
    
    def cleanup(self):
        """Clean up simulation."""
        if sim:
            try:
                sim.close()
            except Exception:
                pass


def run_scenario_file(scenario_path: str, headless: bool = True,
                      enable_ros: bool = False, gui_mode: bool = False,
                      max_duration: float = 60.0, use_wasd: bool = False) -> ScenarioResult:
    """Convenience function to run a scenario from file."""
    runner = ScenarioRunner(headless=headless, enable_ros=enable_ros,
                           gui_mode=gui_mode, max_duration=max_duration)
    try:
        # Load first: real-human scenarios defer World + ROS bridge until
        # after the animation save/reopen (bridge graph must live in the
        # reopened stage; a pre-reopen World would hang on the old stage).
        scenario = runner.load_scenario(scenario_path)
        needs_real = any(bool(p.get("is_real_human", False) or p.get("use_real_human", False))
                         for p in scenario.get("pedestrians", [])) or bool(scenario.get("is_real_human", False))
        runner.initialize_sim(defer_for_anim=bool(needs_real))
        return runner.run_scenario(scenario, use_wasd=use_wasd)
    finally:
        runner.cleanup()


def run_all_scenarios(scenarios_dir: str = "configs/scenarios", 
                      headless: bool = True, max_duration: float = 30.0) -> List[ScenarioResult]:
    """Run all scenario YAML files in directory."""
    results = []
    scenario_files = sorted(Path(scenarios_dir).glob("scenario_*.yaml"))
    
    print(f"[INFO] Found {len(scenario_files)} scenarios to run")
    
    for scenario_file in scenario_files:
        print(f"\n{'='*60}")
        print(f"Running: {scenario_file.name}")
        print(f"{'='*60}")
        
        runner = ScenarioRunner(headless=headless, max_duration=max_duration)
        try:
            runner.initialize_sim()
            scenario = runner.load_scenario(str(scenario_file))
            result = runner.run_scenario(scenario)
            results.append(result)
            
            status = "PASS" if result.success else "FAIL"
            print(f"[{status}] {scenario_file.name} - duration: {result.duration:.1f}s")
            if result.errors:
                for err in result.errors:
                    print(f"  ERROR: {err}")
        finally:
            runner.cleanup()
        
        # Brief pause between scenarios
        time.sleep(1.0)
    
    return results


print(f"[SCRIPT END] About to check __name__ == '__main__', __name__ = {__name__}", flush=True)

print(f"[SCRIPT MAIN CHECK] __name__ = {__name__}, about to enter main block", flush=True)

if __name__ == "__main__":
    print(f"[DEBUG] __name__ = {__name__}", flush=True)
    print(f"[DEBUG] args.scenario = {args.scenario}", flush=True)
    print(f"[DEBUG] args.all = {args.all}", flush=True)
    print(f"[DEBUG] args.headless = {args.headless}", flush=True)
    print(f"[DEBUG] args.gui = {args.gui}", flush=True)
    
    if args.all:
        print(f"[DEBUG] Running all scenarios", flush=True)
        results = run_all_scenarios(headless=args.headless, max_duration=args.duration)
        print(f"\n{'='*60}")
        print(f"SUMMARY: {len(results)} scenarios run")
        for r in results:
            status = "PASS" if r.success else "FAIL"
            print(f"  [{status}] {r.scenario_name} ({r.duration:.1f}s)")
    elif args.scenario:
        print(f"[DEBUG] Running single scenario: {args.scenario}", flush=True)
        result = run_scenario_file(
            args.scenario, headless=args.headless, enable_ros=args.ros,
            gui_mode=args.gui, max_duration=args.duration, use_wasd=args.wasd
        )
        print(f"[DEBUG] run_scenario_file returned, success={result.success}", flush=True)
        status = "PASS" if result.success else "FAIL"
        print(f"\n[{status}] {result.scenario_name} - duration: {result.duration:.1f}s", flush=True)
        sys.exit(0 if result.success else 1)
    else:
        parser.print_help()
        print("\nExample usage:")
        print("  # Run single scenario headless")
        print("  python scenario_runner.py configs/scenarios/scenario_01_stationary_person.yaml --headless")
        print("  # Run single scenario with GUI for inspection")
        print("  python scenario_runner.py configs/scenarios/scenario_01_stationary_person.yaml --gui")
        print("  # Run all scenarios headless")
        print("  python scenario_runner.py --all --headless")