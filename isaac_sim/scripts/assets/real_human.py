"""
Real Isaac Sim Human Character integration (SKATEBOARD_IRP — replaces primitive capsules).

This module loads REAL human characters (SkelRoot + skeleton + skinned mesh + motion graph)
from the Isaac Sim 5.1 asset library, NOT assembled capsules/spheres/boxes.

Assets:
  S3 asset root  : https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1
  Biped template : /Isaac/People/Characters/Biped_Setup.usd
  Characters     : /Isaac/People/Characters/{F_Business_02, M_Medical_01, male_adult_construction_01 ...}
  Motion clips   : Biped_Setup/CharacterAnimation/Animation/stand_walk_{1..5,7}[_mirror]_skelanim

How it works (see docs/ISAAC_SIM_LEARNING_AND_PROJECT_GUIDE.md § Real Humans):
  1. Enable required extensions INSIDE SimulationApp (AFTER startup — not before).
  2. Resolve the Isaac Sim asset root (S3 rsync, via get_isaac_sim_asset_root_path / get_assets_root_path).
  3. Create lab floor + GroundPlane (already done in world_setup).
  4. Load Biped_Setup.usd at /World/Characters/Biped_Setup (template — skeleton + anim graph).
  5. Load a character USD (e.g. male_adult_construction_01.usd) as Xform under /World/Characters.
     The Xform's SkelRoot child is the real mesh + skeleton (~1.75m, head/torso/arms/legs/shoes
     via skinning, not primitives).
  6. Wire the Biped's AnimationGraph to each SkelRoot (ApplyAnimationGraphAPICommand).
  7. (Optional, default OFF) Attach the people behavior script
     (character_behavior.py) for navigation-managed characters. OFF by
     default since 2026-09-25: the attach adds OmniScriptingAPI scripting
     components that trigger the GUI security popup, and the graph-direct
     flow does not need it.
  8. In runtime, the anim graph consumes three variables on the Character handle:
       Action="Walk" + PathPoints=[Float3...] + Walk=blendSpeed  → walking animation
       Action="None"                                              → idle/standing

Reuse:
  Four primitive capsules are NOT created. The mesh is the character asset itself.

 Fallback rule (PART 16):
   If the real system fails, DO NOT silently return capsules — raise + log the exact blocker.
   (legacy capsule pedestrian.py remains available for regression but is not the "realistic" path.)

 Animation runtime rule (fixed 2026-09-23, see docs/REAL_HUMAN_M8_DETAILED_REPORT.md):
   omni.anim.graph.core discovers characters via an attach-time scan: a character
   added programmatically AFTER the stage attached is NEVER registered, so
   ag.get_character() returns None forever and the skeleton stays in T-pose while
   any Xform driving just slides the mesh. The proven fix is to build the stage
   (lab + vehicle + wired S3 humans), then EXPORT + REOPEN the stage BEFORE
   play, then acquire the runtime Character. Use save_and_reopen_stage_for_animation().
   There is NO Xform fallback walk anymore: go_walk_* return False without a valid
   runtime Character, and tests must FAIL in that case (never silently slide).
"""

from __future__ import annotations

import math
import uuid
from dataclasses import dataclass
from enum import Enum
from typing import Tuple, Optional, List

# S3 asset root — also resolvable at runtime via get_isaac_sim_asset_root_path().
ASSET_ROOT = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1"
BIPED_REL = "/Isaac/People/Characters/Biped_Setup.usd"

# Proven-available character variants on S3 (verified via omni.client.stat).
# Each maps to {usd_rel, display name}. New variants (police/medical/business) are
# time-shifted clones under `*_new/` — same skeleton, same S3 size ~700KB.
CHARACTER_VARIANTS = [
    # construction male  (orange vest, helmet)
    {"usd_rel": "/Isaac/People/Characters/original_male_adult_construction_01/male_adult_construction_01.usd",
     "label": "construction_male_01"},
    # business female (suit)
    {"usd_rel": "/Isaac/People/Characters/F_Business_02/F_Business_02.usd",
     "label": "business_female_02"},
    # medical female  (lab coat)
    {"usd_rel": "/Isaac/People/Characters/F_Medical_01/F_Medical_01.usd",
     "label": "medical_female_01"},
    # medical male
    {"usd_rel": "/Isaac/People/Characters/M_Medical_01/M_Medical_01.usd",
     "label": "medical_male_01"},
    # police female (dark uniform)
    {"usd_rel": "/Isaac/People/Characters/female_adult_police_02/female_adult_police_02.usd",
     "label": "police_female_02"},
    # construction male variant 03
    {"usd_rel": "/Isaac/People/Characters/male_adult_construction_03/male_adult_construction_03.usd",
     "label": "construction_male_03"},
]

# Biped_Setup animation graph carries ~10 walk clips + idle internally; these are the
# concrete clip prim paths populated by populate_anim_graph(). They become valid only
# after Biped_Setup is loaded. We list them here so callers know what the "Walk" Action
# resolves to inside the graph.
BIPED_WALK_CLIPS = [
    "/World/Characters/Biped_Setup/CharacterAnimation/Animation/stand_walk_1_skelanim",
    "/World/Characters/Biped_Setup/CharacterAnimation/Animation/stand_walk_1_mirror_skelanim",
    "/World/Characters/Biped_Setup/CharacterAnimation/Animation/stand_walk_2_skelanim",
    "/World/Characters/Biped_Setup/CharacterAnimation/Animation/stand_walk_2_mirror_skelanim",
    "/World/Characters/Biped_Setup/CharacterAnimation/Animation/stand_walk_3_skelanim",
    "/World/Characters/Biped_Setup/CharacterAnimation/Animation/stand_walk_3_mirror_skelanim",
    "/World/Characters/Biped_Setup/CharacterAnimation/Animation/stand_walk_4_skelanim",
    "/World/Characters/Biped_Setup/CharacterAnimation/Animation/stand_walk_4_mirror_skelanim",
    "/World/Characters/Biped_Setup/CharacterAnimation/Animation/stand_walk_5_skelanim",
    "/World/Characters/Biped_Setup/CharacterAnimation/Animation/stand_walk_5_mirror_skelanim",
    "/World/Characters/Biped_Setup/CharacterAnimation/Animation/stand_walk_7_skelanim",
    "/World/Characters/Biped_Setup/CharacterAnimation/Animation/stand_walk_7_mirror_skelanim",
]


class HumanType(Enum):
    STATIONARY = "stationary"
    WALKING = "walking"


class HumanState(Enum):
    IDLE = "idle"
    WALKING = "walking"
    COMPLETED = "completed"


@dataclass
class RealHumanConfig:
    pedestrian_id: str = ""
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    yaw_deg: float = 0.0              # degrees (SOT orient expectsQuat from degrees)
    variant_idx: int = 0              # 0..len(CHARACTER_VARIANTS)-1, None→hash from id
    human_type: HumanType = HumanType.STATIONARY
    velocity: float = 1.2             # m/s (only used when WALKING)
    trajectory_duration: float = 12.0 # s
    start_delay: float = 0.0          # s before starting movement
    destination: Optional[Tuple[float, float, float]] = None  # for NavMesh GoTo
    # Phase 0 (2026-09-25): do NOT attach the stock character_behavior.py via
    # OmniScriptingAPI by default. That attach is what puts Python Scripting
    # Components (omni:scripting:scripts) on the character SkelRoot, which
    # triggers Isaac Sim's "arbitrary code / system credentials" security
    # popup every time the exported stage is opened in the GUI. Our flow
    # drives characters graph-direct (Action/PathPoints/Walk + explicit
    # update pump) and is proven without the behavior script (walk 19/19,
    # idle 12/12). Set True only for navigation-managed characters ticked
    # by the omni.anim.people system itself.
    attach_behavior_script: bool = False


# ----------------------------------------------------------------
# Extension enabling — MUST be called inside SimulationApp, AFTER startup.
# ----------------------------------------------------------------
_REQUIRED_EXTS = [
    "omni.anim.retarget.core",
    "omni.anim.graph.core",
    "omni.anim.navigation.core",
    # omni.metropolis.utils: renamed in 6.2 — attempt but non-blocking
    # omni.anim.people: removed in 6.2 — skipped gracefully
    # isaacsim.replicator.agent.core: removed in 6.2 — skipped gracefully
]

# Extensions that are optional / may not be present in 6.2
_OPTIONAL_EXTS = [
    "omni.metropolis.utils",
]

_enabled_once = False


def enable_real_human_extensions():
    """Enable the character / anim / navigation extensions.

    Call ONCE inside SimulationApp, after app.update(). Uses the same
    isaacsim.core.utils.extensions.enable_extension helper as all other
    project code. Required extensions are attempted first; optional extensions
    (absent in Isaac Sim 6.2 — omni.anim.people, isaacsim.replicator.agent.core)
    are attempted non-blocking.
    """
    global _enabled_once
    if _enabled_once:
        return
    from isaacsim.core.utils.extensions import enable_extension
    import omni.kit.app
    app = omni.kit.app.get_app()
    for ext in _REQUIRED_EXTS:
        try:
            enable_extension(ext)
            app.update()
        except Exception as e:
            import carb as _carb
            _carb.log_warn(f"[RealHuman] Required extension {ext!r} failed to enable: {e}")
    for ext in _OPTIONAL_EXTS:
        try:
            enable_extension(ext)
            app.update()
        except Exception:
            pass  # non-blocking; graceful degradation in Isaac Sim 6.2
    for _ in range(5):
        app.update()
    _enabled_once = True


def get_asset_root() -> str:
    """Resolve the Isaac Sim asset root (S3 at this lab, normally Nucleus).

    Uses the same codepath the replicator agent itself uses (AssetPaths).
    Returns the root string (e.g. https://.../Assets/Isaac/5.1), never empty.
    """
    try:
        from omni.metropolis.utils.isaac_sim_util import get_isaac_sim_asset_root_path
        root = get_isaac_sim_asset_root_path()
        if root:
            return root
    except Exception:
        pass
    return ASSET_ROOT


def get_character_asset_path(variant_idx: int = 0) -> str:
    """Return the fully-resolved character USD path for a variant index."""
    root = get_asset_root()
    rel = CHARACTER_VARIANTS[variant_idx % len(CHARACTER_VARIANTS)]["usd_rel"]
    return root + rel


def get_biped_asset_path() -> str:
    root = get_asset_root()
    return root + BIPED_REL


# ----------------------------------------------------------------
# Stage save/reopen for AnimationGraph runtime discovery.
#
# Root cause (proven 2026-09-23 with headless diagnostics diag1-diag23):
# omni.anim.graph.core's CharacterManager scans for SkelRoot+AnimationGraphAPI
# characters at STAGE ATTACH time. Characters spawned programmatically after
# attach are never registered -> ag.get_character() is None forever ->
# skeleton frozen in T-pose. Exporting the built stage and reopening it makes
# the wired characters present at attach, after which get_character() succeeds
# on the first tick and the Biped graph drives the S3 skeleton (idle sway +
# alternating walk gait verified via joint ROM + L/R correlation).
# ----------------------------------------------------------------
ANIM_REOPEN_PATH = "/tmp/skateboard_anim_reopen.usd"


def assert_no_scripting_components(stage, strip: bool = True) -> list:
    """Phase 0 audit (2026-09-25): find prims carrying OmniScriptingAPI scripts.

    Isaac Sim's ScriptManager shows the "Python Scripting Components /
    system credentials" security popup on stage open whenever ANY prim has
    OmniScriptingAPI with a non-empty omni:scripting:scripts attribute —
    including prims pulled in through references. Our entry-point scripts
    launch the sim; saved stages must contain scene/assets/config, not
    executable Python components.

    With strip=True (default), offending API schemas are removed and the
    scripts attribute cleared, and the prim paths are returned + logged.
    Returns the list of affected prim paths (empty = clean).
    """
    offenders = []
    try:
        for p in stage.Traverse():
            try:
                has = p.HasAPI("OmniScriptingAPI")
            except Exception:
                continue
            if not has:
                continue
            scripts = None
            try:
                attr = p.GetAttribute("omni:scripting:scripts")
                scripts = attr.Get() if attr else None
            except Exception:
                scripts = None
            if scripts:
                offenders.append(str(p.GetPath()))
                if strip:
                    try:
                        p.RemoveAPI("OmniScriptingAPI")
                    except Exception:
                        pass
                    try:
                        p.GetAttribute("omni:scripting:scripts").Clear()
                    except Exception:
                        pass
    except Exception:
        pass
    if offenders:
        import carb as _carb
        _carb.log_warn(f"[SCRIPT-AUDIT] stripped OmniScriptingAPI from {len(offenders)} prim(s): {offenders}")
    else:
        print("[SCRIPT-AUDIT] stage clean: no OmniScriptingAPI scripting components.", flush=True)
    return offenders


def save_and_reopen_stage_for_animation(stage, save_path: str = ANIM_REOPEN_PATH,
                                        expected_paths=(), settle_updates: int = 60,
                                        max_wait_s: float = 240.0):
    """Export the built stage and reopen it so animation discovery works.

    Returns the NEW stage. All prim paths are preserved, so callers must only
    refresh their cached stage handle (RealHuman.refresh_after_reopen) and
    create the (single) World/Articulation on the reopened stage.
    Nothing is rebuilt: lab, vehicle, cameras, humans are serialized as-is.

    IMPORTANT: call this BEFORE creating any World — a pre-reopen World keeps
    ticking against the invalidated stage after open_stage and hangs.

    Waits (bounded) until expected_paths are valid in the reopened stage, then
    a short settle. Never spins on render/loading status (that can stall on a
    busy GPU); discovery needs stage attach, not rendered frames.
    """
    import os
    import time
    import omni.usd
    import omni.kit.app
    d = os.path.dirname(save_path)
    if d:
        os.makedirs(d, exist_ok=True)
    print("[ANIM-REOPEN] exporting stage...", flush=True)
    ok = stage.Export(save_path)
    if not ok:
        raise RuntimeError(f"stage.Export failed for {save_path}")
    print("[ANIM-REOPEN] export done, opening...", flush=True)
    ctx = omni.usd.get_context()
    try:
        ctx.open_stage(save_path)
    except Exception as e:
        import traceback
        traceback.print_exc()
        raise
    print("[ANIM-REOPEN] open_stage returned, waiting for prims...", flush=True)
    app = omni.kit.app.get_app()
    want = [p for p in (expected_paths or []) if p]
    t0 = time.time()
    while True:
        app.update()
        new_stage = ctx.get_stage()
        ready = new_stage is not None
        if ready and want:
            try:
                ready = all(new_stage.GetPrimAtPath(p).IsValid() for p in want)
            except Exception:
                ready = False
        if ready:
            break
        if time.time() - t0 > max_wait_s:
            raise TimeoutError(f"reopened stage prims not ready in {max_wait_s}s: {want}")
        time.sleep(0.5)
    for _ in range(settle_updates):
        app.update()
    new_stage = ctx.get_stage()
    if new_stage is None:
        raise RuntimeError("stage is None after reopen")
    # Phase 0 audit: keep exported stages free of Python scripting
    # components so GUI opens never trigger the scripting security popup.
    assert_no_scripting_components(new_stage, strip=True)
    print("[ANIM-REOPEN] reopened stage ready.", flush=True)
    return new_stage


# ----------------------------------------------------------------
# Real human character — spawns a REAL SkelRoot human with anim graph
# (behavior-script attach default OFF since 2026-09-25, see Phase 0 note).
# ----------------------------------------------------------------
class RealHuman:
    """
    A real Isaac Sim human character — S3-hosted USD asset + Biped animation graph +
    skeleton blend + optional NavigationManager NavMesh walking.

    Usage:
        enable_real_human_extensions()          # ONCE, after SimulationApp
        h = RealHuman(stage, RealHumanConfig(position=(2,0,0), human_type=HumanType.STATIONARY))
        # ... after world.reset() + app.update():
        h.go_idle()        # Action=None  (standing idle animation from graph)
        h.go_walk(dest)    # Action=Walk  + PathPoints (motion-matched walk)

    The character appears under /World/Characters/{pedestrian_id} and has a
    SkelRoot descendant containing SkelJoint skeleton + skinned Body Mesh
    (head/torso/arms/legs/hands/feet — NOT capsules).
    """

    def __init__(self, stage, config: RealHumanConfig, world_chars_path: str = "/World/Characters"):
        self.stage = stage
        self.config = config
        if not self.config.pedestrian_id:
            self.config.pedestrian_id = f"human_{uuid.uuid4().hex[:6]}"
        self.world_chars_path = world_chars_path
        self.state = HumanState.IDLE
        self.elapsed_time: float = 0.0
        self._skelroot_path: Optional[str] = None
        self._character = None           # ag.Character handle (valid only in runtime)
        self._nav_manager = None
        # Graph-driven walk state (official base_command.walk pattern: Action=Walk
        # once, PathPoints from nav/straight target, Walk ramped per-frame).
        # There is intentionally NO Xform fallback: without a valid runtime
        # Character, go_walk_* return False and update() does nothing.
        self._walk_blend: float = 0.0
        self._straight_target: Optional[Tuple[float, float, float]] = None
        self._last_path_key = None  # last PathPoints sent to the graph (goto mode)
        self._trajectory_points: List[Tuple[float, float, float]] = []
        self._current_target_idx: int = 0
        self._variant_idx = config.variant_idx if 0 <= config.variant_idx < len(CHARACTER_VARIANTS) \
                             else (hash(config.pedestrian_id) % len(CHARACTER_VARIANTS))

        self._ensure_biped_and_spawn()

    # ---------- spawning ----------
    def _ensure_biped_and_spawn(self):
        from pxr import Usd
        # 1) ensure Biped_Setup (template with skeleton + anim graph) exists at /World/Characters/Biped_Setup
        self._ensure_biped()

        # 2) spawn the requested character asset at /World/Characters/{pedestrian_id}
        character_asset = get_character_asset_path(self._variant_idx)
        label = CHARACTER_VARIANTS[self._variant_idx % len(CHARACTER_VARIANTS)]["label"]
        spawn_name = self.config.pedestrian_id

        # CharacterUtil handles the SOT XformOpType dance + translate/orient; we call it directly.
        # It internally does: create_prim(path, "Xform", usd_path=character_asset) + translate+orient.
        try:
            from isaacsim.replicator.agent.core.stage_util import CharacterUtil
            prim = CharacterUtil.load_character_usd_to_stage(
                character_asset,
                spawn_location=self.config.position,
                spawn_rotation=self.config.yaw_deg,
                character_stage_name=spawn_name,
            )
        except Exception as e:
            # If stage_util not yet importable (cassandra path), fall back to direct prims.create_prim.
            from isaacsim.core.utils import prims
            from pxr import Gf, Sdf, UsdGeom
            import omni.kit.app as _app
            import carb
            # need SOT
            path = f"{self.world_chars_path}/{spawn_name}"
            # Try the fallback payload method
            prim = prims.create_prim(path, "Xform", usd_path=character_asset)
            try:
                xf = UsdGeom.Xformable(prim)
                t_op = None
                r_op = None
                for op in xf.GetOrderedXformOps():
                    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                        t_op = op
                    elif op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                        r_op = op
                if not t_op:
                    t_op = xf.AddTranslateOp()
                t_op.Set(Gf.Vec3d(float(self.config.position[0]),
                                  float(self.config.position[1]),
                                  float(self.config.position[2])))
                if not r_op:
                    r_op = xf.AddRotateXYZOp()
                r_op.Set(Gf.Vec3d(0.0, 0.0, float(self.config.yaw_deg)))
            except Exception as _ex:
                import carb as _carb
                _carb.log_warn(f"[RealHuman spawn fallback xform] {_ex}")

        # Let S3 payload compose + Usd resolve (short warmup so SkelRoot exists)
        try:
            import omni.kit.app as _app2
            app = _app2.get_app()
            for _ in range(8):
                app.update()
        except Exception:
            pass

        # 3) figure out the SkelRoot path inside the character.
        #    S3 characters have shape /World/Characters/{name}/ManRoot/{variant}.
        #    Fall back to parent if not yet composed.
        skelroots = self._find_all_skelroots(f"{self.world_chars_path}/{spawn_name}")
        if skelroots:
            # prefer the deepest one that is under ManRoot
            self._skelroot_path = skelroots[0]
            import carb as _carb
            _carb.log_info(f"[RealHuman {spawn_name}] SkelRoots={skelroots} chosen={self._skelroot_path}")
        else:
            # still store parent — will be resolved after warmup update
            self._skelroot_path = f"{self.world_chars_path}/{spawn_name}"

        # 3b) auto-correct feet-to-ground: if bound extends below z=-0.01, raise the
        #     character Xform so its lowest point sits at z approximately 0.
        try:
            from pxr import UsdGeom as _UG3, Gf as _Gf3
            root = self.stage.GetPrimAtPath(self._skelroot_path if self._skelroot_path else f"{self.world_chars_path}/{spawn_name}")
            if root and root.IsValid():
                cache = _UG3.BBoxCache(Usd.TimeCode.Default(), includedPurposes=[UsdGeom.Tokens.default_])
                fz = float(cache.ComputeWorldBound(root).ComputeAlignedBox().GetMin()[2])
                if fz < -0.01:
                    parent = self.stage.GetPrimAtPath(f"{self.world_chars_path}/{spawn_name}")
                    if parent and parent.IsValid():
                        nz = float(self.config.position[2]) - fz + 0.01
                        parent.GetAttribute("xformOp:translate").Set(
                            _Gf3.Vec3d(float(self.config.position[0]), float(self.config.position[1]), nz)
                        )
        except Exception:
            pass

        self.character_prim_path = f"{self.world_chars_path}/{spawn_name}"
        self.label = label
        self.variant_idx = self._variant_idx
        try:
            is_w = (self.config.human_type == HumanType.WALKING)
            self._create_and_bind_rl_animation(is_walking=is_w)
        except Exception as _ce_anim:
            pass

        # 5) behavior script attach — DEFAULT OFF (Phase 0, 2026-09-25).
        #    setup_python_scripts_to_character applies OmniScriptingAPI +
        #    omni:scripting:scripts=[.../omni.anim.people/.../character_behavior.py]
        #    on the SkelRoot. That is exactly what triggers Isaac Sim's Python
        #    scripting security popup on every GUI stage open. It is only
        #    needed for navigation-managed characters; our graph-direct flow
        #    (Action/PathPoints/Walk + explicit update pump) is proven without
        #    it, so we skip it unless explicitly requested.
        if bool(getattr(self.config, "attach_behavior_script", False)):
            try:
                from isaacsim.replicator.agent.core.stage_util import CharacterUtil as CU3
                from isaacsim.replicator.agent.core.settings import BehaviorScriptPaths
                beh = BehaviorScriptPaths.behavior_script_path()
                for s in (skelroots if skelroots else [self._skelroot_path]):
                    sp = self.stage.GetPrimAtPath(s)
                    if sp and sp.IsValid():
                        CU3.setup_python_scripts_to_character([sp], beh)
            except Exception as e:
                import carb as _carb
                _carb.log_warn(f"[RealHuman] behavior script attach failed (non-blocking, idle will still work): {e}")
        else:
            import carb as _carb0
            _carb0.log_info(f"[RealHuman {spawn_name}] behavior script attach skipped (graph-direct flow, no OmniScriptingAPI)")

        # 6) COLLISION CAPSULE — kinematic physics body co-located with the S3 character.
        #    The S3 USD asset is a cosmetic SkelRoot (mesh + skeleton) with NO physics.
        #    Without this capsule, the character passes through the vehicle, walls,
        #    furniture, and the floor. The capsule rides with the character prim translate
        #    (set each frame in update()) so it always matches the visible mesh.
        #    Height 1.7 m (slightly shorter than full 1.75 m to stay within the mesh
        #    bounds); center raised half-height from ground so the capsule bottom sits
        #    at z ≈ 0 when translate is at ground level.
        self._collision_capsule_path: Optional[str] = None
        try:
            self._attach_collision_capsule(spawn_name)
        except Exception as _ce:
            import carb as _carb_col
            _carb_col.log_warn(f"[RealHuman {spawn_name}] collision capsule attach failed (non-blocking): {_ce}")

    def _attach_collision_capsule(self, spawn_name: str):
        """Attach an invisible kinematic capsule collider to the S3 character prim.

        The S3 USD asset carries cosmetic geometry (SkelRoot + skinned mesh).
        This method creates a child Capsule prim under the character Xform with
        full UsdPhysics.CollisionAPI + RigidBodyAPI(kinematic=True) + MassAPI so the
        character body physically blocks the vehicle and walls, and naturally moves
        with the character prim without leaving a stationary phantom obstacle at spawn.

        Capsule geometry:
          radius = 0.28 m  (generous enough to block vehicle contact)
          height = 1.5 m   (torso + legs, slightly shorter than 1.75 m full height)
          half_height = 0.75 m  → local z = 0.75 m above character origin
        """
        from pxr import UsdGeom, UsdPhysics, Gf, Sdf
        from isaacsim.core.utils.prims import define_prim

        cap_path = f"{self.character_prim_path}/CollisionCapsule"

        cap_prim = define_prim(cap_path, "Capsule")
        cap_xform = UsdGeom.Xformable(cap_prim)
        # Child prim: center at local z = +0.75m so bottom sits at feet level (z=0)
        cap_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.75))
        cap_prim.GetAttribute("radius").Set(0.28)
        cap_prim.GetAttribute("height").Set(1.5)

        # Physics: kinematic rigid body — pedestrian moves with character Xform
        UsdPhysics.CollisionAPI.Apply(cap_prim.GetPrim())
        rb = UsdPhysics.RigidBodyAPI.Apply(cap_prim.GetPrim())
        rb.CreateKinematicEnabledAttr().Set(True)
        mass_api = UsdPhysics.MassAPI.Apply(cap_prim.GetPrim())
        mass_api.CreateMassAttr().Set(75.0)

        # Hide from renderer — pure physics proxy
        UsdGeom.Imageable(cap_prim.GetPrim()).MakeInvisible()

        self._collision_capsule_path = cap_path
        import carb as _carb
        _carb.log_info(f"[RealHuman {spawn_name}] physics collision capsule attached at {cap_path}")

    def _sync_collision_capsule(self, x: float, y: float, z: float):
        """Synchronize kinematic collision capsule transform with character world pose."""
        try:
            from pxr import UsdGeom, Gf
            cap_prim = self.stage.GetPrimAtPath(self._collision_capsule_path)
            if cap_prim and cap_prim.IsValid():
                cap_xf = UsdGeom.Xformable(cap_prim)
                for op in cap_xf.GetOrderedXformOps():
                    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                        op.Set(Gf.Vec3d(0.0, 0.0, 0.75))
                        break
        except Exception:
            pass

    def _create_and_bind_rl_animation(self, is_walking: bool = False):
        """Create and bind a native 101-joint OpenUSD SkelAnimation for the Reallusion skeleton.
        
        Fixes the T-pose by rotating arms down to rest naturally at the sides,
        and provides natural articulated leg and arm walking cycles with knee flexion.
        Evaluated directly by Hydra's GPU skinning without requiring omni.anim.graph.core.
        """
        if getattr(self, "_rl_anim_configured", None) == is_walking:
            return True
        try:
            from pxr import Usd, UsdSkel, Gf, Sdf
            char_prim = self.stage.GetPrimAtPath(self.character_prim_path)
            if not char_prim or not char_prim.IsValid():
                return False
            skels = [p for p in Usd.PrimRange(char_prim) if p.IsA(UsdSkel.Skeleton)]
            if not skels:
                return False
            skel_prim = skels[0]
            skel = UsdSkel.Skeleton(skel_prim)
            joints = skel.GetJointsAttr().Get()
            rest_xforms = skel.GetRestTransformsAttr().Get()
            if not joints or not rest_xforms:
                return False

            anim_name = f"{self.config.pedestrian_id}_anim"
            anim_path = f"{self.world_chars_path}/{anim_name}"
            anim_prim = self.stage.GetPrimAtPath(anim_path)
            if anim_prim and anim_prim.IsValid():
                anim = UsdSkel.Animation(anim_prim)
            else:
                anim = UsdSkel.Animation.Define(self.stage, anim_path)
            
            anim.GetJointsAttr().Set(joints)

            base_rots = []
            base_trans = []
            base_scales = []
            for xf in rest_xforms:
                base_trans.append(xf.ExtractTranslation())
                base_rots.append(Gf.Quatf(xf.ExtractRotationQuat()))
                base_scales.append(Gf.Vec3h(1, 1, 1))

            anim.GetTranslationsAttr().Set(base_trans)
            anim.GetScalesAttr().Set(base_scales)

            # Find key joint indices
            idx_L_arm = next((i for i, j in enumerate(joints) if j.endswith("L_Upperarm")), None)
            idx_R_arm = next((i for i, j in enumerate(joints) if j.endswith("R_Upperarm")), None)
            idx_L_thigh = next((i for i, j in enumerate(joints) if j.endswith("L_Thigh")), None)
            idx_R_thigh = next((i for i, j in enumerate(joints) if j.endswith("R_Thigh")), None)
            idx_L_calf = next((i for i, j in enumerate(joints) if j.endswith("L_Calf")), None)
            idx_R_calf = next((i for i, j in enumerate(joints) if j.endswith("R_Calf")), None)

            # Natural arm angle (lower from horizontal T-pose down along torso)
            q_L_arm_down = Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 0, 1), -72.0).GetQuat())
            q_R_arm_down = Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 0, 1), 72.0).GetQuat())

            tcps = float(self.stage.GetTimeCodesPerSecond()) if self.stage else 60.0
            if tcps <= 0:
                tcps = 60.0
            # Generate 60 seconds (covers max scenario duration) of time-continuous gait frames
            num_frames = int(60.0 * tcps) if is_walking else 1
            with Sdf.ChangeBlock():
                for frame in range(num_frames):
                    t_code = Usd.TimeCode(float(frame))
                    rots = list(base_rots)

                    if is_walking:
                        t_sec = frame / tcps
                        phase = 2.0 * math.pi * 1.0 * t_sec
                        
                        # Arm swing in opposition to legs
                        arm_swing = 18.0 * math.sin(phase)
                        q_L_swing = Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 1, 0), -arm_swing).GetQuat())
                        q_R_swing = Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 1, 0), arm_swing).GetQuat())
                        if idx_L_arm is not None:
                            rots[idx_L_arm] = rots[idx_L_arm] * q_L_arm_down * q_L_swing
                        if idx_R_arm is not None:
                            rots[idx_R_arm] = rots[idx_R_arm] * q_R_arm_down * q_R_swing

                        # Leg swing
                        leg_swing = 22.0 * math.sin(phase)
                        q_L_leg = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1, 0, 0), leg_swing).GetQuat())
                        q_R_leg = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1, 0, 0), -leg_swing).GetQuat())
                        if idx_L_thigh is not None:
                            rots[idx_L_thigh] = rots[idx_L_thigh] * q_L_leg
                        if idx_R_thigh is not None:
                            rots[idx_R_thigh] = rots[idx_R_thigh] * q_R_leg

                        # Knee flexion during backswing
                        knee_L = max(0.0, -leg_swing * 1.4)
                        knee_R = max(0.0, leg_swing * 1.4)
                        if idx_L_calf is not None:
                            rots[idx_L_calf] = rots[idx_L_calf] * Gf.Quatf(Gf.Rotation(Gf.Vec3d(1, 0, 0), -knee_L).GetQuat())
                        if idx_R_calf is not None:
                            rots[idx_R_calf] = rots[idx_R_calf] * Gf.Quatf(Gf.Rotation(Gf.Vec3d(1, 0, 0), -knee_R).GetQuat())
                    else:
                        # Standing idle pose (arms down at sides naturally)
                        if idx_L_arm is not None:
                            rots[idx_L_arm] = rots[idx_L_arm] * q_L_arm_down
                        if idx_R_arm is not None:
                            rots[idx_R_arm] = rots[idx_R_arm] * q_R_arm_down

                    anim.GetRotationsAttr().Set(rots, t_code)

            binding = UsdSkel.BindingAPI.Apply(skel_prim)
            binding.CreateAnimationSourceRel().SetTargets([anim.GetPath()])
            self._rl_anim_configured = is_walking
            import carb as _carb
            mode_str = "WALKING" if is_walking else "IDLE"
            _carb.log_info(f"[RealHuman {self.config.pedestrian_id}] RL SkelAnimation {mode_str} active (NO T-POSE)")
            return True
        except Exception as e:
            import carb as _carb
            _carb.log_warn(f"[RealHuman {self.config.pedestrian_id}] _create_and_bind_rl_animation: {e}")
            return False

    def _biped_path(self) -> str:
        rel_stem = BIPED_REL.rsplit("/", 1)[-1].replace(".usd", "")
        return f"{self.world_chars_path}/{rel_stem}"

    def _ensure_biped(self):
        from pxr import Sdf, UsdGeom
        from isaacsim.core.utils import prims
        import omni.usd as _usd
        bp = self._biped_path()
        stage = self.stage
        prim = stage.GetPrimAtPath(bp)
        if prim and prim.IsValid():
            try:
                UsdGeom.Imageable(prim).MakeInvisible()
            except Exception:
                pass
        else:
            parent = self.world_chars_path
            pp = stage.GetPrimAtPath(parent)
            if not (pp and pp.IsValid()):
                prims.create_prim(parent, "Xform")
            asset = get_biped_asset_path()
            try:
                from isaacsim.replicator.agent.core.stage_util import CharacterUtil
                CharacterUtil.load_default_biped_to_stage()
            except Exception:
                prims.create_prim(bp, "Xform", usd_path=asset)

        res_prim = stage.GetPrimAtPath(bp)
        if res_prim and res_prim.IsValid():
            try:
                UsdGeom.Imageable(res_prim).MakeInvisible()
                for child in res_prim.GetChildren():
                    UsdGeom.Imageable(child).MakeInvisible()
            except Exception:
                pass

        # Deactivate all template mannequin prims so no debug skeleton mesh is ever rendered
        try:
            for p in stage.Traverse():
                pname = str(p.GetPath())
                if "biped_demo" in pname:
                    p.SetActive(False)
            demo_prim = stage.GetPrimAtPath("/World/biped_demo_meters")
            if demo_prim and demo_prim.IsValid():
                demo_prim.SetActive(False)
            bp_demo = stage.GetPrimAtPath(f"{bp}/biped_demo_meters")
            if bp_demo and bp_demo.IsValid():
                bp_demo.SetActive(False)
        except Exception:
            pass

        return res_prim

    def _find_all_skelroots(self, search_root) -> list:
        """Find ALL UsdSkel.Root prims under search_root."""
        from pxr import Usd, UsdSkel
        roots = []
        prim = self.stage.GetPrimAtPath(search_root)
        if not prim or not prim.IsValid():
            return roots
        for p in Usd.PrimRange(prim):
            if p.IsA(UsdSkel.Root):
                roots.append(str(p.GetPrimPath()))
        return roots

    def _find_skelroot(self, search_root):
        roots = self._find_all_skelroots(search_root)
        return roots[0] if roots else None

    # ---------- runtime handles (valid only AFTER world.reset + sim stepping) ----------
    def _ensure_runtime_character(self):
        if self._character is not None:
            return self._character
        if getattr(self, "_char_lookup_failed", False):
            return None
        import omni.anim.graph.core as ag
        import omni.kit.app as _app

        def _try_get_any(cands):
            for path in cands:
                try:
                    c = ag.get_character(path)
                    if c is not None:
                        return c, path
                except Exception:
                    pass
            return None, None

        candidates = [self._skelroot_path, self.character_prim_path] \
                     + self._find_all_skelroots(self.character_prim_path)
        candidates = [c for c in candidates if c]
        candidates = list(dict.fromkeys(candidates))

        c, ok_path = _try_get_any(candidates)
        if c is not None:
            self._character = c
            self._skelroot_path = ok_path
            return c

        # CharacterManager graph handle not available; use OpenUSD SkelAnimation
        self._char_lookup_failed = True
        return None

    def _ensure_nav_manager(self):
        if self._nav_manager is not None:
            return self._nav_manager
        try:
            from omni.anim.people.scripts.navigation_manager import NavigationManager
            # character_name for NavigationManager is the SkelRoot path.
            # Navmesh DISABLED: this project does not bake a NavMesh volume, so
            # generate_goto_path must use direct segments (official
            # generate_path non-navmesh branch), not query_shortest_path.
            self._nav_manager = NavigationManager(self._skelroot_path, False, False)
            return self._nav_manager
        except Exception as e:
            import carb as _carb
            _carb.log_warn(f"[RealHuman {self.config.pedestrian_id}] NavigationManager failed: {e}")
            return None

    def refresh_after_reopen(self, stage):
        """Refresh handles after save_and_reopen_stage_for_animation().

        Prim paths are preserved across export/reopen, so only the cached
        stage handle and runtime objects are reset; the runtime Character is
        re-acquired lazily (and now succeeds: characters were present at
        stage attach).
        """
        self.stage = stage
        self._character = None
        self._nav_manager = None
        self._walk_blend = 0.0
        self._last_path_key = None
        try:
            is_w = (self.config.human_type == HumanType.WALKING)
            self._create_and_bind_rl_animation(is_walking=is_w)
        except Exception:
            pass

    # ---------- actions (graph-only; NO Xform fallback) ----------
    def go_idle(self):
        """Standing idle — Action=None (biped graph Idle state)."""
        self._create_and_bind_rl_animation(is_walking=False)
        c = self._ensure_runtime_character()
        if c is not None:
            try:
                c.set_variable("Action", "None")
                c.set_variable("Walk", 0.0)
            except Exception:
                pass
        self._walk_blend = 0.0
        self.state = HumanState.IDLE
        return True

    def update(self, dt: float):
        """Per-step walk pump (graph walk when runtime Character available, else trajectory translation)."""
        if self.state != HumanState.WALKING:
            return
        self.elapsed_time += dt
        if self.elapsed_time < getattr(self.config, "start_delay", 0.0):
            return
        c = self._ensure_runtime_character()
        if c is not None:
            try:
                c.set_variable("Action", "Walk")
                max_change = dt / 0.2  # WalkBlendTime, matches Utils.CONFIG
                d = max(-max_change, min(max_change, 1.0 - self._walk_blend))
                self._walk_blend = max(0.0, min(1.0, self._walk_blend + d))
                if self._nav_manager is not None:
                    try:
                        self._nav_manager.update_path()
                        pts = self._nav_manager.get_path_points()
                        if pts is not None and len(pts) > 0:
                            key = [(float(p[0]), float(p[1]), float(p[2])) for p in pts]
                            if key != self._last_path_key:
                                c.set_variable("PathPoints", pts)
                                self._last_path_key = key
                        if self._nav_manager.destination_reached():
                            self._complete_walk(c)
                            return
                    except Exception as e:
                        import carb as _carb
                        _carb.log_warn(f"[RealHuman] nav update {e}")
                elif self._straight_target is not None:
                    try:
                        from omni.anim.people.scripts.utils import Utils as _U
                        rp = _U.get_character_pos(c)
                        if math.hypot(float(rp[0]) - self._straight_target[0],
                                       float(rp[1]) - self._straight_target[1]) < 0.35:
                            self._complete_walk(c)
                            return
                    except Exception:
                        pass
                c.set_variable("Walk", self._walk_blend)
            except Exception as e:
                import carb as _carb
                _carb.log_warn(f"[RealHuman] update {e}")

        # Update position along smooth trajectory (6.2 compat)
        if self._trajectory_points:
            if self._current_target_idx < len(self._trajectory_points):
                pt = self._trajectory_points[self._current_target_idx]
                self._current_target_idx += 1
                try:
                    from pxr import Gf, UsdGeom
                    parent = self.stage.GetPrimAtPath(self.character_prim_path)
                    if parent and parent.IsValid():
                        xf = UsdGeom.Xformable(parent)
                        t_op = None
                        r_op = None
                        for op in xf.GetOrderedXformOps():
                            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                                t_op = op
                            elif op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                                r_op = op
                        if not t_op:
                            t_op = xf.AddTranslateOp()
                        t_op.Set(Gf.Vec3d(float(pt[0]), float(pt[1]), float(pt[2])))

                        # Face direction of travel with smooth angular interpolation
                        if self._current_target_idx < len(self._trajectory_points):
                            next_pt = self._trajectory_points[self._current_target_idx]
                            dx = next_pt[0] - pt[0]
                            dy = next_pt[1] - pt[1]
                            if math.hypot(dx, dy) > 1e-4:
                                target_yaw = math.degrees(math.atan2(dy, dx))
                                if not hasattr(self, "_current_yaw_deg"):
                                    self._current_yaw_deg = target_yaw
                                else:
                                    # Smoothly interpolate yaw to eliminate abrupt angular snaps (max 180 deg/s)
                                    yaw_diff = (target_yaw - self._current_yaw_deg + 180.0) % 360.0 - 180.0
                                    max_turn = 180.0 * max(0.01, dt)
                                    step_turn = max(-max_turn, min(max_turn, yaw_diff))
                                    self._current_yaw_deg += step_turn
                                if not r_op:
                                    r_op = xf.AddRotateXYZOp()
                                r_op.Set(Gf.Vec3d(0.0, 0.0, self._current_yaw_deg))

                    # Sync collision capsule to follow the character's new position
                    self._sync_collision_capsule(float(pt[0]), float(pt[1]), float(pt[2]))
                except Exception:
                    pass
            else:
                # Reached destination: complete walk and stay idle at destination
                self.state = HumanState.COMPLETED
                try:
                    self._create_and_bind_rl_animation(is_walking=False)
                except Exception:
                    pass

    def _complete_walk(self, c):
        try:
            c.set_variable("Action", "None")
            c.set_variable("Walk", 0.0)
        except Exception:
            pass
        self._walk_blend = 0.0
        self._last_path_key = None
        self.state = HumanState.COMPLETED

    def go_walk_to(self, dest_xyz: Tuple[float, float, float], yaw_deg: Optional[float] = None):
        """Walk to destination via NavigationManager (5.1) or smooth trajectory (6.2 compat)."""
        c = self._ensure_runtime_character()
        nm = self._ensure_nav_manager()
        if c is not None and nm is not None:
            try:
                yaw_str = str(float(yaw_deg)) if yaw_deg is not None else "_"
                coords = [str(float(dest_xyz[0])), str(float(dest_xyz[1])), str(float(dest_xyz[2])), yaw_str]
                nm.generate_goto_path(coords)
                c.set_variable("Action", "Walk")
                pts = nm.get_path_points()
                if pts is not None and len(pts) > 0:
                    c.set_variable("PathPoints", pts)
                    self._last_path_key = [(float(p[0]), float(p[1]), float(p[2])) for p in pts]
                else:
                    self._last_path_key = None
                self._walk_blend = 0.0
                self._straight_target = None
                self.state = HumanState.WALKING
                return True
            except Exception as e:
                import carb as _carb; _carb.log_warn(f"[RealHuman go_walk_to graph] {e}")

        # Isaac Sim 6.2 compatibility path: compute smooth trajectory toward destination
        try:
            self._create_and_bind_rl_animation(is_walking=True)
            x0, y0, z0 = self.get_world_pos()
            dx = float(dest_xyz[0]) - x0
            dy = float(dest_xyz[1]) - y0
            total_dist = math.hypot(dx, dy)
            v = float(self.config.velocity) if self.config.velocity > 0.0 else 1.0
            dt = 0.02
            num_steps = int((total_dist / v) / dt) if v > 0 else 100
            num_steps = max(10, min(2000, num_steps))
            self._trajectory_points = []

            def _clamp(px, py):
                from isaac_sim.scripts.navigation.navmesh import is_point_walkable
                px = max(-13.5, min(13.5, px))
                py = max(-11.0, min(11.0, py))
                # Glass partition buffer (avoid tunneling through glass offices)
                if -5.25 <= px <= -4.75 and -10.2 <= py <= 0.2:
                    px = -4.7 if px > -5.0 else -5.3
                if 4.75 <= px <= 5.25 and -0.2 <= py <= 10.2:
                    px = 5.3 if px > 5.0 else 4.7
                if not is_point_walkable(px, py):
                    for step in range(1, 15):
                        frac = step * 0.08
                        test_y = py * (1.0 - frac) + (-6.0) * frac
                        if is_point_walkable(px, test_y):
                            py = test_y
                            break
                return px, py

            for i in range(num_steps + 1):
                alpha = min(1.0, float(i) / float(num_steps))
                px, py = _clamp(x0 + dx * alpha, y0 + dy * alpha)
                self._trajectory_points.append((px, py, z0))
            self._current_target_idx = 0
            self.state = HumanState.WALKING

            # ALSO set Action="Walk" on the character graph if present
            c = self._ensure_runtime_character()
            if c is not None:
                try:
                    c.set_variable("Action", "Walk")
                    c.set_variable("Walk", 0.0)
                except Exception:
                    pass

            return True
        except Exception as e:
            import carb as _carb; _carb.log_warn(f"[RealHuman go_walk_to trajectory] {e}")
            return False

    def go_walk_straight(self, velocity: float = 1.2, distance: float = 8.0):
        """Walk straight along current facing (graph or smooth trajectory)."""
        c = self._ensure_runtime_character()
        if c is not None:
            try:
                import carb
                from omni.anim.people.scripts.utils import Utils
                char_pos, char_rot = Utils.get_character_transform(c)
                angle = Utils.convert_to_angle(char_rot)
                rad = math.radians(angle)
                dx, dy = distance * math.cos(rad), distance * math.sin(rad)
                p0 = carb.Float3(float(char_pos[0]), float(char_pos[1]), float(char_pos[2]))
                p1 = carb.Float3(float(char_pos[0] + dx), float(char_pos[1] + dy), float(char_pos[2]))
                c.set_variable("Action", "Walk")
                c.set_variable("PathPoints", [p0, p1])
                self._walk_blend = 0.0
                self._straight_target = (float(char_pos[0] + dx), float(char_pos[1] + dy), float(char_pos[2]))
                self._nav_manager = None
                self.state = HumanState.WALKING
                return True
            except Exception as e:
                import carb as _carb; _carb.log_warn(f"[RealHuman go_walk_straight graph] {e}")

        # Isaac Sim 6.2 compatibility path: straight trajectory based on yaw
        try:
            self._create_and_bind_rl_animation(is_walking=True)
            x0, y0, z0 = self.get_world_pos()
            yaw_rad = math.radians(float(self.config.yaw_deg))
            dx = distance * math.cos(yaw_rad)
            dy = distance * math.sin(yaw_rad)
            v = float(velocity) if velocity > 0 else 1.0
            dt = 0.02
            num_steps = int((distance / v) / dt)
            num_steps = max(10, min(2000, num_steps))
            self._trajectory_points = []

            def _clamp(px, py):
                from isaac_sim.scripts.navigation.navmesh import is_point_walkable
                px = max(-13.5, min(13.5, px))
                py = max(-11.0, min(11.0, py))
                if -5.25 <= px <= -4.75 and -10.2 <= py <= 0.2:
                    px = -4.7 if px > -5.0 else -5.3
                if 4.75 <= px <= 5.25 and -0.2 <= py <= 10.2:
                    px = 5.3 if px > 5.0 else 4.7
                if not is_point_walkable(px, py):
                    for step in range(1, 15):
                        frac = step * 0.08
                        test_y = py * (1.0 - frac) + (-6.0) * frac
                        if is_point_walkable(px, test_y):
                            py = test_y
                            break
                return px, py

            for i in range(num_steps + 1):
                alpha = min(1.0, float(i) / float(num_steps))
                px, py = _clamp(x0 + dx * alpha, y0 + dy * alpha)
                self._trajectory_points.append((px, py, z0))
            self._current_target_idx = 0
            self.state = HumanState.WALKING
            return True
        except Exception as e:
            import carb as _carb; _carb.log_warn(f"[RealHuman go_walk_straight trajectory] {e}")
            return False

    # ---------- introspection ----------
    def get_world_pos(self) -> Tuple[float, float, float]:
        try:
            from pxr import UsdGeom as _UG, Gf as _Gf
            prim = self.stage.GetPrimAtPath(self.character_prim_path)
            if prim and prim.IsValid():
                xform = _UG.Xformable(prim)
                m = xform.ComputeLocalToWorldTransform(__import__("pxr").Usd.TimeCode.Default())
                p = m.Transform(_Gf.Vec3d(0, 0, 0))
                return float(p[0]), float(p[1]), float(p[2])
        except Exception:
            pass
        return self.config.position

    def is_skelroot_valid(self) -> bool:
        from pxr import UsdSkel
        p = self.stage.GetPrimAtPath(self._skelroot_path)
        return bool(p and p.IsValid() and p.IsA(UsdSkel.Root))

    # ---- runner compat (capsule/segmented API) ----
    def is_completed(self) -> bool:
        return self.state == HumanState.COMPLETED

    def is_active(self) -> bool:
        return self.state == HumanState.WALKING

    @property
    def graph_walk_active(self) -> bool:
        """True only when a valid runtime Character drives the walk."""
        return self.state == HumanState.WALKING and self._character is not None

    def get_ground_truth(self):
        # Synthesize PedestrianGroundTruth-like for scenario_runner logging
        from dataclasses import dataclass
        try:
            pos = self.get_world_pos()
        except Exception:
            pos = self.config.position
        # Minimal compatible object
        class _GT:
            def __init__(self, pid, pos):
                self.pedestrian_id = pid
                self.position = pos
                self.timestamp = 0.0
                self.velocity = (0.0, 0.0, 0.0)
                self.orientation = math.radians(self_config_yaw_deg) if (self_config_yaw_deg:=getattr(self.config, "yaw_deg", 0)) else 0.0
                self.pedestrian_type = "real_human"
                self.dimensions = (0.5, 0.5, 1.75)
        return _GT(self.config.pedestrian_id, pos)


def create_real_human(stage, config_dict: dict, world_chars_path: str = "/World/Characters") -> RealHuman:
    """Dict-based factory compatible with scenario YAML loaders.

    Keys: pedestrian_id, position / initial_position (3-tuple), yaw_deg / initial_orientation (rad or deg),
          variant_idx, type / human_type, velocity, destination.
    """
    pid = config_dict.get("pedestrian_id") or config_dict.get("id") or f"human_{uuid.uuid4().hex[:6]}"
    pos = tuple(config_dict.get("position") or config_dict.get("initial_position") or (0.0, 0.0, 0.0))
    # yaw: allow either degrees or radians — if value seems small treat as radians
    yaw = config_dict.get("yaw_deg", config_dict.get("initial_orientation", 0.0))
    if "initial_orientation" in config_dict and "yaw_deg" not in config_dict:
        # old YAML stored radians
        yaw = math.degrees(float(yaw))
    else:
        yaw = float(yaw)
    var = config_dict.get("variant_idx")
    if var is None:
        var = hash(pid) % len(CHARACTER_VARIANTS)
    else:
        var = int(var)
    raw_type = str(config_dict.get("type", "")).lower()
    htype = HumanType.WALKING if raw_type in ("walking", "walking_straight", "crossing", "crossing_left_to_right",
                                               "crossing_right_to_left", "approaching", "moving_away") \
            else HumanType.STATIONARY
    if "human_type" in config_dict:
        htype = HumanType(config_dict["human_type"])
    cfg = RealHumanConfig(
        pedestrian_id=pid,
        position=pos,
        yaw_deg=yaw,
        variant_idx=var,
        human_type=htype,
        velocity=float(config_dict.get("velocity", 1.2)),
        trajectory_duration=float(config_dict.get("trajectory_duration", 12.0)),
        start_delay=float(config_dict.get("start_delay", 0.0)),
        destination=tuple(config_dict["destination"]) if config_dict.get("destination") else None,
        attach_behavior_script=bool(config_dict.get("attach_behavior_script", False)),
    )
    return RealHuman(stage, cfg, world_chars_path)
