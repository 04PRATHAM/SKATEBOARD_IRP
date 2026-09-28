"""
Pedestrian Asset Module for SKATEBOARD_IRP M8
Creates reusable pedestrian representations with proper physics and visuals.

Supports:
- stationary pedestrian
- walking pedestrian (straight)
- crossing pedestrian (left-to-right, right-to-left)
- approaching pedestrian (towards/away from vehicle)

Each pedestrian has:
- Clear visual body (capsule torso + head)
- Physically sensible dimensions (~1.7m height, ~0.45m width)
- Stable ground contact (kinematic rigid body with capsule collider)
- Visible from RGB-D camera
- Configurable initial position, orientation, velocity, trajectory
- Unique scenario/object ID
- Ground-truth state tracking
"""

import math
import uuid
from dataclasses import dataclass, field
from typing import Optional, List, Tuple
from enum import Enum

from pxr import Usd, UsdGeom, Gf


class PedestrianType(Enum):
    STATIONARY = "stationary"
    WALKING_STRAIGHT = "walking_straight"
    CROSSING_LEFT_TO_RIGHT = "crossing_left_to_right"
    CROSSING_RIGHT_TO_LEFT = "crossing_right_to_left"
    APPROACHING = "approaching"
    MOVING_AWAY = "moving_away"


class PedestrianState(Enum):
    IDLE = "idle"
    ACTIVE = "active"
    COMPLETED = "completed"


@dataclass
class PedestrianConfig:
    """Configuration for a pedestrian instance."""
    pedestrian_id: str = field(default_factory=lambda: f"ped_{uuid.uuid4().hex[:8]}")
    pedestrian_type: PedestrianType = PedestrianType.STATIONARY
    initial_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # x, y, z
    initial_orientation: float = 0.0  # yaw in radians
    velocity: float = 1.2  # m/s typical walking speed
    trajectory_duration: float = 10.0  # seconds
    start_delay: float = 0.0  # seconds before starting
    height: float = 1.75  # meters
    shoulder_width: float = 0.45  # meters
    capsule_radius: float = 0.22  # meters (for collision)
    color: Tuple[float, float, float] = (0.2, 0.4, 0.8)  # blue-ish default


@dataclass
class PedestrianGroundTruth:
    """Ground truth state for evaluation (never published to perception pipeline)."""
    pedestrian_id: str
    timestamp: float = 0.0
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    orientation: float = 0.0
    velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    pedestrian_type: str = "stationary"
    dimensions: Tuple[float, float, float] = (0.45, 0.45, 1.75)  # width, depth, height


class Pedestrian:
    """
    Pedestrian entity with visual representation, physics, and ground truth tracking.
    
    Uses kinematic rigid body with capsule collider for stable ground contact.
    Motion is driven by setting kinematic target positions each frame (no teleport).
    """
    
    def __init__(self, stage, config: PedestrianConfig, world_path: str = "/World/Pedestrians"):
        self.stage = stage
        self.config = config
        self.world_path = world_path
        self.state = PedestrianState.IDLE
        self.elapsed_time = 0.0
        self.ground_truth = PedestrianGroundTruth(
            pedestrian_id=config.pedestrian_id,
            pedestrian_type=config.pedestrian_type.value,
            dimensions=(config.shoulder_width, config.shoulder_width, config.height)
        )
        self._trajectory_points: List[Tuple[float, float, float]] = []
        self._current_target_idx = 0
        
        # Create the pedestrian USD prims
        self._create_pedestrian()
        # Compute trajectory at simulation timestep (default 100Hz = 0.01s)
        self._compute_trajectory(sim_dt=0.01)
    
    def _create_pedestrian(self):
        """Create the pedestrian USD hierarchy with visual and collision geometry."""
        from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf
        from isaacsim.core.utils.prims import define_prim
        
        pid = self.config.pedestrian_id
        x, y, z = self.config.initial_position
        base_path = f"{self.world_path}/{pid}"
        
        # Root Xform for the pedestrian (moves with pedestrian)
        self.root_prim = define_prim(base_path, "Xform")
        root_xform = UsdGeom.Xformable(self.root_prim)
        root_xform.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
        if self.config.initial_orientation != 0.0:
            root_xform.AddRotateXYZOp().Set(Gf.Vec3d(0, 0, math.degrees(self.config.initial_orientation)))
        
        # Torso capsule (main body) - visual
        torso_height = self.config.height * 0.65  # ~1.14m
        torso_radius = self.config.capsule_radius
        torso_z = torso_height / 2.0
        
        torso_path = f"{base_path}/Torso"
        torso_prim = define_prim(torso_path, "Capsule")
        torso_xform = UsdGeom.Xformable(torso_prim)
        torso_xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, torso_z))
        torso_xform.AddRotateXYZOp().Set(Gf.Vec3d(90, 0, 0))  # upright
        torso_prim.GetAttribute("radius").Set(torso_radius)
        torso_prim.GetAttribute("height").Set(torso_height)
        
        # Head sphere - visual
        head_radius = self.config.shoulder_width * 0.35  # ~0.16m
        head_z = torso_height + head_radius * 0.8
        head_path = f"{base_path}/Head"
        head_prim = define_prim(head_path, "Sphere")
        head_xform = UsdGeom.Xformable(head_prim)
        head_xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, head_z))
        head_prim.GetAttribute("radius").Set(head_radius)
        
        # Collision capsule (kinematic rigid body) - for ground contact and obstacle interaction
        # Slightly larger than visual for safety
        coll_radius = self.config.capsule_radius * 1.1
        coll_height = self.config.height * 0.9
        coll_z = coll_height / 2.0
        
        coll_path = f"{base_path}/CollisionCapsule"
        coll_prim = define_prim(coll_path, "Capsule")
        coll_xform = UsdGeom.Xformable(coll_prim)
        coll_xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, coll_z))
        coll_xform.AddRotateXYZOp().Set(Gf.Vec3d(90, 0, 0))
        coll_prim.GetAttribute("radius").Set(coll_radius)
        coll_prim.GetAttribute("height").Set(coll_height)
        
        # Apply collision API
        UsdPhysics.CollisionAPI.Apply(coll_prim.GetPrim())
        
        # Make it kinematic rigid body for controlled motion
        rigid_api = UsdPhysics.RigidBodyAPI.Apply(coll_prim.GetPrim())
        rigid_api.CreateKinematicEnabledAttr().Set(True)
        
        # Mass API for physics interactions
        mass_api = UsdPhysics.MassAPI.Apply(coll_prim.GetPrim())
        mass_api.CreateMassAttr().Set(75.0)  # typical adult mass in kg
        
        # Hide collision capsule from renderer (only visual torso/head visible)
        UsdGeom.Imageable(coll_prim.GetPrim()).MakeInvisible()
        
        # Apply colors to visual parts
        color = Gf.Vec3f(*self.config.color)
        for vis_path in [torso_path, head_path]:
            vis_prim = self.stage.GetPrimAtPath(vis_path)
            if vis_prim.IsValid():
                disp_attr = vis_prim.GetAttribute("primvars:displayColor")
                if not disp_attr:
                    disp_attr = vis_prim.CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray)
                disp_attr.Set([color])
        
        # Store references
        self.torso_prim = torso_prim
        self.head_prim = head_prim
        self.collision_prim = coll_prim
        self.root_xform = root_xform
        
        # Initialize ground truth
        self._update_ground_truth(0.0)
    
    def _compute_trajectory(self, sim_dt: float = 0.01):
        """Pre-compute trajectory waypoints based on pedestrian type.
        
        Args:
            sim_dt: Simulation timestep in seconds (default 0.01s / 100Hz).
                    Trajectory points are computed at this resolution so that
                    _current_target_idx advances correctly at each simulation step.
        """
        x, y, z = self.config.initial_position
        yaw = self.config.initial_orientation
        v = self.config.velocity
        duration = self.config.trajectory_duration
        dt = sim_dt  # waypoint resolution matches simulation timestep
        
        self._trajectory_points = []
        num_steps = int(duration / dt)
        
        for i in range(num_steps + 1):
            t = i * dt
            if self.config.pedestrian_type == PedestrianType.STATIONARY:
                px, py = x, y
            elif self.config.pedestrian_type == PedestrianType.WALKING_STRAIGHT:
                px = x + v * t * math.cos(yaw)
                py = y + v * t * math.sin(yaw)
            elif self.config.pedestrian_type == PedestrianType.CROSSING_LEFT_TO_RIGHT:
                # Cross from left to right (negative Y to positive Y) while moving forward in X
                px = x + v * t * math.cos(yaw) * 0.3  # slight forward drift
                py = y + v * t  # cross in +Y direction
            elif self.config.pedestrian_type == PedestrianType.CROSSING_RIGHT_TO_LEFT:
                px = x + v * t * math.cos(yaw) * 0.3
                py = y - v * t  # cross in -Y direction
            elif self.config.pedestrian_type == PedestrianType.APPROACHING:
                # Move towards vehicle (negative X direction if vehicle is at origin)
                px = x - v * t
                py = y
            elif self.config.pedestrian_type == PedestrianType.MOVING_AWAY:
                px = x + v * t
                py = y
            else:
                px, py = x, y
            
            self._trajectory_points.append((px, py, z))
        
        self._current_target_idx = 0
    
    def _update_ground_truth(self, sim_time: float):
        """Update ground truth state from current USD transform."""
        # Get world transform of root
        world_transform = UsdGeom.Xformable(self.root_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        pos = world_transform.Transform(Gf.Vec3d(0, 0, 0))
        
        # Extract yaw from transform
        fwd = world_transform.TransformDir(Gf.Vec3d(1, 0, 0))
        yaw = math.atan2(fwd[1], fwd[0])
        
        # Compute velocity from trajectory
        vel = (0.0, 0.0, 0.0)
        if self.state == PedestrianState.ACTIVE and self._trajectory_points:
            if self._current_target_idx > 0 and self._current_target_idx < len(self._trajectory_points):
                prev = self._trajectory_points[self._current_target_idx - 1]
                curr = self._trajectory_points[min(self._current_target_idx, len(self._trajectory_points) - 1)]
                # Use the same dt as trajectory computation (sim_dt = 0.01)
                dt = 0.01
                vel = ((curr[0] - prev[0]) / dt, (curr[1] - prev[1]) / dt, 0.0)
        
        self.ground_truth = PedestrianGroundTruth(
            pedestrian_id=self.config.pedestrian_id,
            timestamp=sim_time,
            position=(float(pos[0]), float(pos[1]), float(pos[2])),
            orientation=yaw,
            velocity=vel,
            pedestrian_type=self.config.pedestrian_type.value,
            dimensions=(self.config.shoulder_width, self.config.shoulder_width, self.config.height)
        )
    
    def update(self, dt: float, sim_time: float):
        """Update pedestrian position based on trajectory."""
        if self.state == PedestrianState.IDLE:
            if self.elapsed_time >= self.config.start_delay:
                self.state = PedestrianState.ACTIVE
            self.elapsed_time += dt
            return
        
        if self.state == PedestrianState.ACTIVE:
            self.elapsed_time += dt
            
            # Check if trajectory completed
            if self._current_target_idx >= len(self._trajectory_points) - 1:
                self.state = PedestrianState.COMPLETED
                return
            
            # Move to next trajectory point
            target = self._trajectory_points[self._current_target_idx]
            self._current_target_idx += 1
            
            # Update root transform (kinematic motion - no teleport, smooth interpolation)
            self.root_xform.GetTranslateOp().Set(Gf.Vec3d(*target))
            
            # Update ground truth
            self._update_ground_truth(sim_time)
    
    def get_ground_truth(self) -> PedestrianGroundTruth:
        """Get current ground truth (for evaluation only, never for perception)."""
        return self.ground_truth
    
    def is_active(self) -> bool:
        return self.state == PedestrianState.ACTIVE
    
    def is_completed(self) -> bool:
        return self.state == PedestrianState.COMPLETED


def create_pedestrian(stage, config: PedestrianConfig, world_path: str = "/World/Pedestrians") -> Pedestrian:
    """Factory function to create a pedestrian."""
    return Pedestrian(stage, config, world_path)


def create_pedestrian_from_dict(stage, config_dict: dict, world_path: str = "/World/Pedestrians") -> Pedestrian:
    """Create pedestrian from dictionary (e.g., loaded from YAML)."""
    # Convert dict to PedestrianConfig
    config = PedestrianConfig(
        pedestrian_id=config_dict.get("pedestrian_id", f"ped_{uuid.uuid4().hex[:8]}"),
        pedestrian_type=PedestrianType(config_dict.get("type", "stationary")),
        initial_position=tuple(config_dict.get("initial_position", [0.0, 0.0, 0.0])),
        initial_orientation=config_dict.get("initial_orientation", 0.0),
        velocity=config_dict.get("velocity", 1.2),
        trajectory_duration=config_dict.get("trajectory_duration", 10.0),
        start_delay=config_dict.get("start_delay", 0.0),
        height=config_dict.get("height", 1.75),
        shoulder_width=config_dict.get("shoulder_width", 0.45),
        capsule_radius=config_dict.get("capsule_radius", 0.22),
        color=tuple(config_dict.get("color", [0.2, 0.4, 0.8])),
    )
    return create_pedestrian(stage, config, world_path)


if __name__ == "__main__":
    print("Pedestrian asset module loaded successfully")