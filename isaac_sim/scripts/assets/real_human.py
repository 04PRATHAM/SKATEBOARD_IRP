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
    "omni.metropolis.utils",
    "omni.anim.people",
    "isaacsim.replicator.agent.core",
]

_enabled_once = False


def enable_real_human_extensions():
    """Enable the character / anim / navigation extensions.

    Call ONCE inside SimulationApp, after app.update(). Uses the same
    isaacsim.core.utils.extensions.enable_extension helper as all other
    project code. The previous bug was calling `import omni.anim.people`
    BEFORE this — that gave ModuleNotFoundError and was mistaken for
    "unavailable".
    """
    global _enabled_once
    if _enabled_once:
        return
    from isaacsim.core.utils.extensions import enable_extension
    import omni.kit.app
    app = omni.kit.app.get_app()
    for ext in _REQUIRED_EXTS:
        enable_extension(ext)
        app.update()
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
            from pxr import Gf, Sdf
            import omni.kit.app as _app
            import carb
            # need SOT
            path = f"{self.world_chars_path}/{spawn_name}"
            # Try the fallback payload method
            prim = prims.create_prim(path, "Xform", usd_path=character_asset)
            try:
                prim.GetAttribute("xformOp:translate").Set(
                    __import__("pxr").Gf.Vec3d(float(self.config.position[0]),
                                                float(self.config.position[1]),
                                                float(self.config.position[2])))
            except Exception:
                pass

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

        # 4) wire anim graph from Biped to this character (so Action/Walk/PathPoints work)
        #    Apply to ALL SkelRoots found, not just the first.
        try:
            from isaacsim.replicator.agent.core.stage_util import CharacterUtil as CU2
            biped_prim = self.stage.GetPrimAtPath(self._biped_path())
            anim_graph = None
            if biped_prim and biped_prim.IsValid():
                for p in Usd.PrimRange(biped_prim):
                    if p.GetTypeName() == "AnimationGraph":
                        anim_graph = p
                        break
            # fallback: also scan for StateMachine/AnimationGraph under Biped_Setup payload
            if anim_graph is None and biped_prim and biped_prim.IsValid():
                # ensure payload composed
                import omni.kit.app as _app3
                for _ in range(5):
                    _app3.get_app().update()
                for p in Usd.PrimRange(biped_prim):
                    if p.GetTypeName() == "AnimationGraph":
                        anim_graph = p
                        break
            if anim_graph and skelroots:
                prims_list = [self.stage.GetPrimAtPath(s) for s in skelroots if self.stage.GetPrimAtPath(s).IsValid()]
                if prims_list:
                    CU2.setup_animation_graph_to_character(prims_list, anim_graph)
                    import carb as _carb2
                    _carb2.log_info(f"[RealHuman {spawn_name}] graph wired to {len(prims_list)} SkelRoot(s)")
        except Exception as e:
            import carb as _carb
            _carb.log_warn(f"[RealHuman] anim graph wire failed (non-blocking): {e}")

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

        self.character_prim_path = f"{self.world_chars_path}/{spawn_name}"
        self.label = label
        self.variant_idx = self._variant_idx

    def _biped_path(self) -> str:
        rel_stem = BIPED_REL.rsplit("/", 1)[-1].replace(".usd", "")
        return f"{self.world_chars_path}/{rel_stem}"

    def _ensure_biped(self):
        from pxr import Sdf
        from isaacsim.core.utils import prims
        import omni.usd as _usd
        bp = self._biped_path()
        stage = self.stage
        prim = stage.GetPrimAtPath(bp)
        if prim and prim.IsValid():
            return prim
        # create parent folder if needed
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
        return stage.GetPrimAtPath(bp)

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
        import omni.anim.graph.core as ag
        import omni.kit.app as _app
        # If graph handle missing, attempt to (re)wire AnimationGraph now that
        # Biped is fully composed post-reset. The helper's __init__ wiring may
        # have raced the S3 payload compose.
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

        # Retry wiring: find Biped's AnimationGraph now that it should be composed.
        try:
            from isaacsim.replicator.agent.core.stage_util import CharacterUtil as CU
            biped_prim = self.stage.GetPrimAtPath(self._biped_path())
            anim_graph = None
            if biped_prim and biped_prim.IsValid():
                for _ in range(5):
                    _app.get_app().update()
                for p in __import__("pxr").Usd.PrimRange(biped_prim):
                    if p.GetTypeName() == "AnimationGraph":
                        anim_graph = p
                        break
            if anim_graph is not None:
                # Re-collect skelroots (S3 asset may have composed more since __init__)
                skels = self._find_all_skelroots(self.character_prim_path)
                prims_list = [self.stage.GetPrimAtPath(s) for s in skels if self.stage.GetPrimAtPath(s).IsValid()]
                if prims_list:
                    CU.setup_animation_graph_to_character(prims_list, anim_graph)
                    for _ in range(5):
                        _app.get_app().update()
                    c, ok_path = _try_get_any([str(p.GetPrimPath()) for p in prims_list] + candidates)
                    if c is not None:
                        self._character = c
                        self._skelroot_path = ok_path
                        return c
        except Exception as e:
            import carb as _carb2
            _carb2.log_warn(f"[RealHuman {self.config.pedestrian_id}] graph rewire retry failed: {e}")

        import carb as _carb
        _carb.log_warn(f"[RealHuman {self.config.pedestrian_id}] get_character failed for all candidates {candidates}")
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

    # ---------- actions (graph-only; NO Xform fallback) ----------
    def go_idle(self):
        """Standing idle — Action=None (biped graph Idle state).

        Requires a valid runtime Character (stage must have been reopened
        after wiring). Returns False otherwise — callers/tests must FAIL,
        never silently accept a frozen skeleton.
        """
        c = self._ensure_runtime_character()
        if c is None:
            import carb as _carb
            _carb.log_warn(f"[RealHuman {self.config.pedestrian_id}] go_idle: no runtime Character")
            return False
        try:
            c.set_variable("Action", "None")
            c.set_variable("Walk", 0.0)
            self._walk_blend = 0.0
            self.state = HumanState.IDLE
            return True
        except Exception as e:
            import carb as _carb; _carb.log_warn(f"[RealHuman] go_idle {e}")
            return False

    def update(self, dt: float):
        """Per-step graph walk pump (official base_command.walk pattern).

        Ramps Walk 0->1, refreshes PathPoints from the NavigationManager, and
        completes on arrival. Does NOT touch Xform: without a valid runtime
        Character there is no motion at all (honest failure, not sliding).
        """
        if self.state != HumanState.WALKING:
            return
        c = self._ensure_runtime_character()
        if c is None:
            return
        try:
            max_change = dt / 0.2  # WalkBlendTime, matches Utils.CONFIG
            d = max(-max_change, min(max_change, 1.0 - self._walk_blend))
            self._walk_blend = max(0.0, min(1.0, self._walk_blend + d))
            if self._nav_manager is not None:
                try:
                    self._nav_manager.update_path()
                    pts = self._nav_manager.get_path_points()
                    if pts is not None and len(pts) > 0:
                        # Send PathPoints ONLY when the path actually changed.
                        # Re-sending identical points every tick resets
                        # MotionMatching progress and stalls root translation
                        # (observed 2026-09-25: triggers ok, zero displacement).
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
        """Walk to destination via NavigationManager path + motion-matched walk.

        Uses the straight-line (non-baked-navmesh) path: the lab flow does not
        bake a NavMesh, so the manager is created with navmesh disabled and
        generate_goto_path yields direct segments. Per-frame, update() ramps
        Walk and tracks arrival like the official GoTo command.
        Returns False without a valid runtime Character (no fallback).
        """
        c = self._ensure_runtime_character()
        nm = self._ensure_nav_manager()
        if c is None or nm is None:
            import carb as _carb
            _carb.log_warn(f"[RealHuman {self.config.pedestrian_id}] go_walk_to: no runtime Character")
            return False
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
            return False

    def go_walk_straight(self, velocity: float = 1.2, distance: float = 8.0):
        """Walk straight along current facing using PathPoints (no navmesh).

        Graph-only: requires a valid runtime Character, else returns False.
        Speed consistency (Phase 6): translation is produced by the graph's own
        MotionMatching from PathPoints (not by an external Xform drive), so the
        played gait and the root motion come from the same animation source.
        """
        c = self._ensure_runtime_character()
        if c is None:
            import carb as _carb
            _carb.log_warn(f"[RealHuman {self.config.pedestrian_id}] go_walk_straight: no runtime Character")
            return False
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
            return False

    # ---------- introspection ----------
    def get_world_pos(self) -> Tuple[float, float, float]:
        try:
            import omni.anim.graph.core as ag
            from omni.anim.people.scripts.utils import Utils
            c = self._ensure_runtime_character()
            if c is None:
                # fallback: read stage Xform directly
                from pxr import UsdGeom as _UG, Gf as _Gf
                xform = _UG.Xformable(self.stage.GetPrimAtPath(self.character_prim_path))
                m = xform.ComputeLocalToWorldTransform(__import__("pxr").Usd.TimeCode.Default())
                p = m.Transform(_Gf.Vec3d(0, 0, 0))
                return float(p[0]), float(p[1]), float(p[2])
            pos = Utils.get_character_pos(c)
            return float(pos[0]), float(pos[1]), float(pos[2])
        except Exception:
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
        destination=tuple(config_dict["destination"]) if config_dict.get("destination") else None,
        attach_behavior_script=bool(config_dict.get("attach_behavior_script", False)),
    )
    return RealHuman(stage, cfg, world_chars_path)
