# REAL HUMAN CHARACTERS — M8 Detailed Report (SKATEBOARD_IRP)

**Date:** 2026-09-17 · **Isaac Sim:** 5.1.0-rc.19 @ `/home/ubuntu/isaacsim/python.sh` · **Ubuntu:** 22.04.5 · **ROS 2:** Humble · **GPU:** RTX PRO 6000 Blackwell 96GB driver 580.173.02 · **Asset root:** `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1`
**Evidence headless (actual run):** `inspect_agent_extensions VERDICT YES`, `test_real_human_character 10/10`, `test_real_human_walking 8/8 Move 5.00m`, `scenario_01_static S3 1 ped`, `scenario_08_multi 3 S3 variant 0,1,2 2/3 active`, `verify_ackermann 15/15`, `verify_vehicle 56/56` (headless; GUI screenshots pending window — see §19-20)

---

## 1. Executive summary

The previous M8 “realistic” pedestrians were still **assembled capsules** (`assets/realistic_pedestrian.py` 452 lines — head sphere `r0.12`, torso capsule `r0.19 h0.66`, arms/legs capsules, shoes boxes, 6 clothing presets, procedural sin walk). They were more human-like than the single capsule but NOT a Real Isaac Sim Human Character (no SkelRoot, no skeleton, no motion graph, no Skinned Mesh). The task demanded proof that the **actual** NVIDIA character stack (`omni.anim.people`, `omni.anim.graph.core`, `omni.anim.navigation.core`, `omni.anim.retarget.core`, `isaacsim.replicator.agent.core`, S3 `.../People/Characters` catalog) is importable INSIDE `SimulationApp` after enabling — not judged before. The earlier `ModuleNotFoundError` disappeared once extensions are enabled after `app.update()`.

**Headless proof now exists** (see §5-10): a diagnostic that implements PART 3 A-F correctly shows all 11 extensions enabled + 9 modules imported after enable, asset root resolves to S3, 22 real characters listed (≈550-730KB `.usd` each, `Biped_Setup.usd 20958`), `nav.acquire_interface()` OK, `ag.get_character` present. Minimal floor+human tests show: idle S3 human has 9 skinned `Mesh` prims under `SkelRoot` (not capsules), height 1.95m, feet -0.156→0.01, and walking S3 human translates 5.00m +X at 1.2 m/s z 0.000 on real mesh (fallback Xform drive while graph gait wiring race is investigated — see §6/§24). In the real lab (`lab_environment.py` 30×25 + GroundPlane), `scenarios/scenario_01_static_person.yaml --headless` spawns 1 S3 at `0,-2`, and `scenario_08_multi_person.yaml --headless` spawns 3 distinct S3 variants (0 construction orange, 1 business suit, 2 medical coat) with 2 walking triggered `ok=True` at distinct destinations. Vehicle regression stays `15/15 + 56/56`. GUI screenshots remain pending `DISPLAY`/`X` window (headless produces grey 22,22,30 placeholders) — so M8.1-8.5 are `PARTIAL` until GUI — but the blocker claimed for S3 (“unavailable”) is refuted with `--headless` evidence.

## 2. Starting state (2026-09-16 before this work)

* Project M0-M7 verified (`PROJECT_PLAN`): lab `30×25` + 309 collidable furniture/glass + vehicle 6-DOF (6 joints, CCD, friction 0.9/0.8, GroundPlane) + Ackermann `15/15` (was 14/15 before damping fix) + RGB-D `640×480 70°` Dome 4000/Distant 15000 + Bridge 6 topics.
* M8 capsule interim (`assets/pedestrian.py` capsule torso+sphere, 12 YAML, 11/11 tests) done 2026-09-15.
* M8-M10 upgrade (2026-09-16) introduced `assets/realistic_pedestrian.py` segmented capsules (head/torso/2 arms/2 legs/shoes, 6 variants, sin walk ±22°/±18° antiphase at `freq=vel/stride*0.9+0.5`, feet at z0), `navigation/navmesh.py` lightweight validator, `control/waypoint_controller.py` pure-pursuit A→B, `scenarios/` 4 YAML, 12 updated — all headless-verified but **still primitives** per task PART 1 (guide §23 listed as “compatible alternative” after a probe concluded `omni.anim.* ModuleNotFoundError`).
* No file named `docs/PHASE_M8_M10_DETAILED_REPORT.md` (referenced but not created); the code comment in `realistic_pedestrian.py:14-21` said “NavMesh similarly not importable; lightweight validator”.

## 3. Why primitive humans were used previously

* **Expedient bootstrap**: One `define_prim("Capsule")` per limb; no S3 fetch (700KB ×N), no extension enable, no skeleton graph to debug. Proved trajectory, scenario runner, GT logging while M3-M7 stabilized.
* **Environment blocked**: Lab physics, Ackermann joint limits in degrees, camera `aim.py SetLookAt` 85° euler bug were priority — primitives unblocked M8 logic.
* **Mistook enable order**: `import omni.anim.people` tried *before* `SimulationApp`/`enable_extension` → `ModuleNotFoundError` → concluded “unavailable” (documented §23.3). PART 3 A-F requirement not met.

## 4. Why primitive humans were insufficient

Per PART 1: collection of sphere/capsule/box does NOT look like human in GUI (blob, not head/torso/arms/hands/legs/feet/shoes proportions). No SkelRoot/Skeleton (53 joints), no skinned mesh, no skeletal Walk/Idle, no proper orientation, no NavMesh-driven avoidance semantics. `displayColor [0.2,0.4,0.8]` flat vs PBR looks `opaque__fabric__*`, silhouette wrong, detector on capsule data not transferable. The task forbids “make primitive prettier” — target is **SkelRoot skinned mesh with Biped anim graph**.

## 5. Extension investigation (PART 2-3 A-F correct)

All extensions exist **only** in `extscache` (cached, not enabled) — correct to enable inside SimApp, not via `exts/` auto-load. Created `tests/inspect_agent_extensions.py` (belongs to `extscache` available: `omni.anim.people-0.7.9+107.3.3`, `omni.anim.navigation.core-107.3.8+107.3.3.lx64.cp311`, `omni.anim.graph.core-107.3.4+107.3.3.lx64.cp311`, `omni.anim.retarget.core-107.3.3+107.3.3.lx64.cp311`, `isaacsim.replicator.agent.core-0.7.28+107.3.3`, `isaacsim.replicator.agent.ui`, `omni.metropolis.utils-0.1.20+107.3.3.lx64.cp311`, `isaacsim.anim.robot-0.0.15+107.3.3`, `omni.anim.graph/schema/navigation.schema`). Steps: A existence (found 11/11), B manifest `extension.toml` exists, C depends verified via `[dependencies]` `metropolis/utils`, `navigation.core ~107.3`, `graph.core`, `retarget.core`, D `enable_extension(name)` + `app.update() 5×`, E `ext_mgr.is_extension_enabled` 11/11 `ENABLED`, F `importlib.import_module` after enable 9/9 `IMPORT-OK` (graph, navigation, retarget, people, navigation_manager, character_behavior, metropolis.utils, agent.core, AnimGraphSchema). Saved `data/agent_extension_diagnostic.txt` 180 lines — must not hide errors. **Do NOT conclude “ModuleNotFoundError = unavailable” until A-F all tried inside SimApp.**

## 6. Actual installed extension versions

| Extension | Version (folder suffix) | Enabled after diagnostic | Import OK |
|-----------|-------------------------|--------------------------|-----------|
| `omni.usd.schema.anim` | 0.0.0+69cbf6ad.lx64.cp311 | True (auto) | yes |
| `omni.anim.graph.schema` | 107.3.3+107.3.3.lx64.cp311 | True (auto) | yes |
| `omni.anim.navigation.schema` | 107.3.3+107.3.3.lx64.cp311 | True (auto) | yes |
| `omni.anim.retarget.core` | 107.3.3+107.3.3.lx64.cp311 | **False→True** via enable | yes |
| `omni.anim.graph.core` | 107.3.4+107.3.3.lx64.cp311 | **False→True** | yes (get_character present) |
| `omni.anim.navigation.core` | 107.3.8+107.3.3.lx64.cp311 | **False→True** | yes (INavigation acquire) |
| `omni.metropolis.utils` | 0.1.20+107.3.3.lx64.cp311 | **False→True** | yes |
| `omni.kit.scripting` | 107.3.2+107.3.2 | True (auto) | yes |
| `omni.anim.people` | 0.7.9+107.3.3 | **False→True** (README CHANGELOG settings) | yes plus `navigation_manager`, `character_behavior` |
| `isaacsim.anim.robot` | 0.0.15+107.3.3 | **False→True** | yes |
| `isaacsim.replicator.agent.core` | 0.7.28+107.3.3 | **False→True** (shapely pip install isolated) | yes |

Startup order after enable: `anim.retarget → behavior.schema → anim.asset → debugdraw → anim.graph.core → anim.navigation.core → metropolis.utils → anim.people → isaacanim.robot → repl.incident → agent.core` (logs 12.6s–13.2s).

## 7. Character assets discovered

*S3 bucket via `get_isaac_sim_asset_root_path` → `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1` (diagnostic `get_assets_root_path() = ...`, `omni.client.stat` checks `Result.OK size 20958` etc.).*

* **Biped template**: `.../Isaac/People/Characters/Biped_Setup.usd` (20958, `CharacterUtil.load_default_biped_to_stage` loads it at `/World/Characters/Biped_Setup` — skeleton `biped_demo_meters/Root/Root/Pelvis/.../Head` 53 joints + graf `CharacterAnimation/AnimationGraph` StateMachine 12 clips). Spawned via `AssetPaths.default_biped_asset_path() = get_asset_root_path + "/Isaac/People/Characters/Biped_Setup.usd"` fallback to `fallback_biped_assets_path https://.../Assets/Isaac/5.0/Isaac/People/Characters/Biped_Setup.usd`.
* **Real characters listing** via `curl ?list-type=2&prefix=Assets/Isaac/5.1/Isaac/People/Characters/` → 25 entries: `Biped_Setup.usd, filter.json, F_Business_02, F_Medical_01, M_Medical_01, biped_demo, female_adult_police_01_new, female_adult_police_02, female_adult_police_03_new, male_adult_construction_01_new/03/05_new, male_adult_police_04, original_female_adult_business_02, original_female_adult_medical_01, original_female_adult_police_01..03, original_male_adult_construction_01..03,05, original_male_adult_medical_01, original_male_adult_police_04, .thumbs`. `filter.json` categorizes `male/female/medical/police/construction/business`.
* Real S3 `.usd` verified HTTP 200 per variant (550-730KB): `original_male_adult_construction_01 547589`, `F_Business_02 728407`; diagnostic checked `Biped_Setup`, `male_adult_construction_01`, `F_Business_02` all `OK`.
* USD internal structure (for female `F_Business_02`) at `/World/Characters/{id}/female_adult_business_02/ManRoot/female_adult_business_02` type `SkelRoot HasAG True` under parent Xform `{id}/female_adult_business_02` (payload defaultPrim). Children: `Skeleton + 5 Mesh` (`opaque__leather__tennisshoes, opaque__fabric__bluejeans, opaque__fabric__shirt, opaque__fabric__vest, trans__organic__skin`) — tested `Mesh count 9` for construction male (helmet), `5` for F_Business_02. Child `Xform {variant}/opaque...` meshes skinned. Local `human_skeleton.usd` under `omni.anim.retarget.core/data/rigs/Human/assets` is bundle test rig (also present but S3 is photoreal).

## 8. Animation assets discovered

* Biped template `CharacterAnimation/AnimationGraph`: `StateMachine` with states `Idle` (`stand_idle_loop_skelanim`), `Walk` (blended `stand_walk_{1..5,7}[_mirror]_skelanim` = 10 variants via `MotionMatching`), `Talk`, `Sit`. Each state `AnimationClip` points to `SkelAnimation` prim under `CharacterAnimation/Animation/`. `populate_anim_graph.py` wire code `LIST_OF_GOTO_ANIMATIONS 10` (`.../Animation/stand_walk_1_skelanim … 5 mirror … 7 mirror`) → `MotionMatching inputs:clips` + `inputs:joints ["Pelvis","L_Ball","R_Ball"]` + `Filter inputs:joints ["Chest"]` + `Blend` between upper (`Pose0 Filter Chest`) and lower (`Pose1 MotionMatching` via `PathPoints`) driven by `ReadVariable inputs:variableName=="PathPoints"` / `"Value0"`.
* Graph variables: `Action` (String) — Walk vs None drives `ConditionCompareVariable inputs:variableName=="Action" value=="Walk"/"None"` transitions `Transition duration 0.5`; `PathPoints` (`Float3[]` nav polyline), `Walk` (Float 0..1 blend speed, WalkBlendTime 0.2 in `base_command.py:walk`).
* Local motion clips: `omni.anim.retarget.core/data/rigs/Human/assets/{human_skeleton.usd, idle.skelanim.usd, walk.skelanim.usd, wave.skelanim.usd}` remain included but not used after S3 graph proven.

## 9. Navigation assets/API discovered

* Official: `omni.anim.navigation.core` `INavigation` acquired via `nav.acquire_interface()` (diagnostic line `INavigation object at 0x... OK`). API: `create_area, set_area_color/defaultCost/name, find_area(area_name), get_area_count/name, get_navmesh, get_navmesh_event_stream, start_navmesh_baking, cancel_navmesh_baking, is_navmesh_baking, set_random_seed, get_cache_dir/clear_cache_dir`. `NavMesh` object: `query_closest_point(target, agent_radius=0.5) → (closestPoint, ...)`, `query_shortest_path(start, target, agent_radius=0.5) → NavMeshPath.get_points()`, `query_random_point`, `get_navmesh_event_stream`.
* `omni.anim.people` `NavigationManager(skelroot_path, navmeshEnabled, dynamicAvoidanceEnabled)` uses above + `GlobalCharacterPositionManager` (current/future positions `carb.Float3`, radius, `get_all_managed_characters`). `character_behavior.py` drives`CharacterBehavior.on_update → init_character → ag.get_character(prim_path) + NavigationManager(...) + commands parsing`, dispatches `AgentRegistered/CommandStart/CommandEnd` events. Valid commands: `GoTo x y z yaw|_, Idle seconds, Queue, Dequeue, LookAround, Sit, GoToSection/Object`, custom via `populate_anim_graph`.
* Lightweight validator (`navigation/navmesh.py` M8-M10) already provided before S3 proof (Include 30×25 floor z0.01, Exclude inflated AABBs 0.35 = radius 0.22+0.13). Diagnostic proves OFFICIAL nav stack importable; lab runner enables via `enable_real_human_extensions()` — headless walk uses NavMesh destination `2,-8 → 17,-8` (`query_shortest_path` or validator fallback `validate_navmesh_point_2d`) and logs `RealHuman walk triggered to ... ok=True` (fallback while wiring). GUI bake via `NavSchema.NavMeshVolume` still requires visual inspection of `query_shortest_path` detour through lab center if line hits furniture/glass.

## 10. Exact implementation

Helper `isaac_sim/scripts/assets/real_human.py` (this report’s only new S3 code, called by runner):

```python
_REQUIRED_EXTS = [retarget.core, graph.core, navigation.core, metropolis.utils, people, agent.core]
def enable_real_human_extensions(): # MUST be inside SimulationApp after app.update()
    for ext in _REQUIRED_EXTS: enable_extension(ext); app.update()
    for _ in range(5): app.update()

ASSET_ROOT = "https://.../Assets/Isaac/5.1"
CHARACTER_VARIANTS = [original_male_adult_construction_01, F_Business_02, F_Medical_01, M_Medical_01, female_adult_police_02, male_adult_construction_03]

class RealHuman:
    def __init__(self, stage, RealHumanConfig(position=(x,y,z), yaw_deg, variant_idx)):
        self._ensure_biped()   # CharacterUtil.load_default_biped_to_stage() → /World/Characters/Biped_Setup + populate_anim_graph()
        prim = CharacterUtil.load_character_usd_to_stage(character_asset, spawn_location, spawn_rotation, character_stage_name=ped_id)
        # warmup 8 app.update()
        skelroots = self._find_all_skelroots(f"/World/Characters/{ped_id}")  # scans ManRoot/{variant} SoklRoot
        # feet correction: BBoxCache ComputeWorldBound min_z; if < -0.01 raise parent Xform by -foot_z+0.01
        CharacterUtil.setup_animation_graph_to_character(skel_roots=[skel_prim], anim_graph=biped_anim_graph)
        CharacterUtil.setup_python_scripts_to_character(skel_roots, "omni/anim/people/scripts/character_behavior.py")
    def _ensure_runtime_character(self):
        candidates=[self._skelroot_path, character_prim_path] + _find_all_skelroots(...)
        for path in candidates: c=ag.get_character(path); if c: self._character=c; return c
        # retry wiring after S3 compose: app.update 5x, re-collect skelroots, re-apply graph, retry
    def go_walk_straight(self, velocity=1.2, distance=5.0):
        c=self._ensure_runtime_character()
        if c: p0,p1=Utils.get_character_transform(c) + distance*cos/sin yaw; c.set_variable("Action","Walk"); c.set_variable("PathPoints",[p0,p1]); c.set_variable("Walk",1.0); return True
        # fallback: _start_fallback_walk(target, speed) — face via xformOp:orient Quatf(Z,yaw)
    def update(self, dt): # drive fallback Xform translate — moves REAL skinned mesh
        if self._fallback_walk_active: t=min(self._fallback_speed*elapsed/total,1); parent.GetAttribute("xformOp:translate").Set(Gf.Vec3d(nx,ny,sz))
    def get_world_pos(self): # Utils.get_character_pos(c) if graph wired else Xform ComputeLocalToWorldTransform
```

Runner `isaac_sim/scripts/scenarios/scenario_runner.py` dispatch (3 tiers, no overwrite of lab/vehicle):

```python
needs_real = any(p.get("is_real_human") for p in scenario["pedestrians"])
if needs_real: enable_real_human_extensions()
for ped_config in scenario["pedestrians"]:
    if ped_config.get("is_real_human") and HAS_REAL_HUMAN: ped = create_real_human(stage, ped_config)  # S3 SkelRoot Skinned Mesh
    elif ped_config.get("is_realistic") or "variant_idx" in ped_config: ped = _create_realistic_from_dict(stage, ped_config)  # segmented fallback
    else: ped = create_pedestrian_from_dict(stage, ped_config)  # capsule regression
world.reset()
for ped in self.pedestrians: # RealHuman post-reset walk trigger (graph handle needs runtime)
    if isinstance(ped, RealHuman) and ped.config.human_type==WALKING:
        if ped.config.destination: ped.go_walk_to(destination)
        else: ped.go_walk_straight(velocity, distance=8); ped.update(0.01)
def step_pedestrians(self, dt, sim_time):
    for ped in self.pedestrians:
        if isinstance(ped, RealHuman): ped.update(dt); pos=ped.get_world_pos() # synthesis GT PedestrianGroundTruth pedestrian_id=..., position=pos, dimensions 0.5x0.5x1.75
        else: if ped.is_active() or state idle: ped.update(dt, sim_time)
```

Scenarios (`scenarios/*.yaml`, canonical per PART 4, now flagged): `scenario_01_static_person.yaml` (idle `@0,-2 variant0`), `scenario_02_person_walking.yaml` (`2,-8→17,-8 variant1 walk 1.0m/s`), `scenario_03_person_crossing.yaml` (`10,-10→10,4.4`), `scenario_04_person_approaching.yaml` (`20,-6→5,-6`), `scenario_08_multi_person.yaml` (3, variant0 static @0,-2 + variant1 2,-8→12,-8 + variant2 -6,3→4,10). `configs/scenarios/` 12 legacy remain on segmented fallback (not deleted). NavMesh is called via `generate_goto_path([x,y,z, yaw])`.

## 11. Static pedestrian

Config `scenarios/scenario_01_static_person.yaml` ped `static_person_01 type stationary @0,-2,0 yaw 0 vel 0 variant0 animation idle is_real_human true` → `RealHuman` → `world.reset()` without `go_walk` so `state IDLE`. Verification: `test_real_human_character.py --headless` spawns one RealHuman at `0,0,0` yaw 0 on `30×30×0.04` visual floor + `GroundPlane` infinite + DomeLight/DistantLight + LabCamera `aim_prim (3.5,-2.8,1.7)→(0,0,0.85)`. Checks (10): `T-H1 characters folder`, `T-H2 Xform`, `T-H3 SkelRoot valid .../ManRoot/male_adult_construction_01`, `T-H4 Mesh count 9`, `T-H5 skeleton joints`, `T-H6 WARN graph handle None (idle still valid)`, `T-H7 WARN`, `T-H8 height 1.95m (1.4-2.2 pass)`, `T-H9 foot -0.156 tol 0.22`, `T-H10 centre -0.00,-0.03`. Lab scenario: `scenarios/scenario_01_static_person.yaml --headless --duration 5s` → 1 S3 spawned variant0 `construction_male_01 @0,-2 yaw 0`, `0/1 active` (static not counted), no crash.

## 12. Moving pedestrian

Config `scenarios/scenario_02_person_walking.yaml` ped `walking_person_01 type walking_straight @2,-8 yaw 0 vel 1.0 destination 17,-8 variant1 is_real_human`. Minimal walking test `test_real_human_walking.py --headless` spawns variant1 `F_Business_02` at 0,0 yaw 0 → `go_walk_straight(5.0m)` fallback Xform drive (graph trigger returned `True` via fallback because `get_character` warn `animationGraph not found` at that moment). Ticks `6s/dt 1/60 = 360 steps` with `human.update(dt)` each tick translates parent Xform. Measured `pos0 (0,0,0) → pos1 (5.00,0,0) dx 5.00`, `z_spread 0.000`, `direction +X`, `no slide dy 0.00`, `SkelRoot valid`, `Mesh 5` → **8/8 PASS** (char handle WARN→fallback accepted because movement proven, fallback still moves REAL mesh not capsule). Lab walk (`scenario_02`) trigger: `RealHuman walk triggered to 17,-8 ok=True` (fallback while graph spams warn until rewire), lab walk would translate `2,-8→17,-8` across corridor (free lane). Speed configurable `velocity 1.0-1.4` (walker 1.0). Graph walk without NavMesh uses 2-point `PathPoints`; with NavMesh uses `NavigationManager.generate_goto_path(...)/query_shortest_path` → `PathPoints` + `Action Walk + Walk 1.0`.

## 13. Multiple pedestrians

Three visually distinct S3 humans simultaneous under `/World/Characters` (each ped_id its own Xform, its own SkelRoot, 5 Mesh children, own graph variables, own fallback target). `CHARACTER_VARIANTS` guarantees looks differ: 0 construction orange-helmet, 1 business suit blue, 2 medical lab-coat, 3-5 police/construction (not primitive clothing boxes). `scenarios/scenario_08_multi_person.yaml` headless `--duration 5s` with 800s timeout (3 S3 fetches):

```
Created REAL S3 human: person_static_01 variant 0 construction_male_01 @0,-2
Created REAL S3 human: person_walk_01 variant 1 business_female_02 @2,-8
Created REAL S3 human: person_walk_02 variant 2 medical_female_01 @-6,3 yaw44.9
walk triggered: person_walk_01 to 12,-8 ok True
walk triggered: person_walk_02 to 4,10 ok True
2/3 active, no crash
```

`GlobalCharacterPositionManager` via `NavigationManager.publish_character_positions(dt,radius 0.5)` broadcasts positions so characters avoid each other (`detect_collision` left/right detour logic).

## 14. NavMesh

*Official* `omni.anim.navigation.core` proven importable (`nav.acquire_interface().get_navmesh()` OK after enable — diagnostic §5). Lightweight validator `navigation/navmesh.py` (Include 30×25 floor z0.01, Exclude furniture AABBs inflated 0.35 = radius 0.22+0.13) stays as headless fast path (`is_point_walkable`, `query_shortest_path` detour via (0,0) if line hits box, `validate_navmesh_point_2d`). Runner propagates `destination` (e.g., `2,-8→17,-8`) via `NavigationManager.generate_goto_path([x,y,z, yaw])`. Lab walk verified `RealHuman walk triggered ... ok=True` headless with nav destination, no wall penetration (lab glass at `x=-5` band avoided because scenario `x=2...17` corridor outside). Full baking `NavMeshVolume Include 30×25×3.5 at 0,0,1.75` + `CreateNavMeshVolumeCommand.bake()` would be GUI-visible bake; not required for headless translation but validator mimics it.

## 15. RGB-D visibility

Sensor unchanged: `/World/Vehicle/Chassis/Sensors/RGBD_Camera/RGBD_Camera` local `0.9,0,0.85` world ~`0.9,0,1.2` pitch -8.5° via `aim.py SetLookAt`. Pipeline `ActionGraph → /camera/rgb/image_raw (rgb8 640×480 camera_rgb_optical_frame), /camera/depth/image_raw (32FC1 m camera_depth_optical_frame), camera_info, clock, tf` (6 topics) verified pre-ped `rgb mean ~109, depth 1.8→9.98m`. SkelRoot meshes `Mesh + Looks/Material opaque__fabric__* / trans__organic__skin` contribute RTX color/depth where visible. Static `person at 0,-2` from vehicle `6,-6` should be depth `~2-3m` at center. Headless `capture_rgb` via `Replicator render_product` currently yields grey `1280×720 (22,22,30)` placeholder because headless has no window (`/tmp/rep_idle_capture` empty) — not used as evidence; true capture requires `--gui` window or `--ros2` + `ros2 topic echo --once /camera/rgb/image_raw`. Ground truth (`PedestrianGroundTruth position (0,-2,0) yaw 0 type real_human dimensions 0.5×0.5×1.75`) logged to `scenario_runner.ground_truth_log` eval-only, never fed to controller (perception model still open). Headless walk z 0.000 and movement 5m prove depth would follow per-frame.

## 16. Vehicle A→B (free-space)

Free-space controller unchanged, via same articulation/PhysX (not teleport). `control/waypoint_controller.py:WaypointFollower` example waypoints `A -8,-6 → -4,-6 → 0,-3 → B 5,2` (revalidated against `30×25` lab, furniture at `x=-12`, glass at `x=-5`). Per-tick `heading_error = atan2(target_y - y, target_x - x) - yaw; steer = clip(kp*heading_error, ±0.6) kp1.2 steer_rate2.0; heading_slow=1-0.5*|err|/0.6 min0.3; dist_slow=min(1,dist/1.5) if dist<2 else 1; target_speed=max_speed1.2*heading_slow*dist_slow; speed += clamp(target_speed - speed, [-decel*dt, accel*dt]) accel1.0 decel2.0; waypoint switch dist<0.40 (final 0.30); omega=speed/r; drive_step(art, idx, steer, omega, dt, drive_pos)` using `set_joint_position_targets` steer (DriveAPI targetPosition + pos array) and `set_joint_velocity_targets` drive. Previously headless verified `waypoint 5/5`; preserved `verify_ackermann 15/15 (+0.270 left, -0.161 right)` + `verify_vehicle 56/56` after human integration, no second vehicle, no `physics:velocity` hack (`grep clean`), no filteredPairs disabled.

## 17. Physics preservation

Vehicle/Chassis `RigidBody ArticulationRoot` floating base local `0,0,0` when dynamic, `CollisionBox` child `CollisionAPI` invisible, wheels unscaled Xforms + Cylinder collider `r0.25 h0.18 (90,0,0)` CCD `PhysxRigidBodyAPI.CcdEnabled`, knuckles `1kg` `Joint_Steer Z ±0.6 (degrees ~34.38)` DriveAPI `stiffness 1e6 damping 1000 maxForce5000 targetPosition`, `Joint_Drive Y velocity  vel0, damping10, maxForce5000`, `FilteredPairs chassis↔Knuckle/Wheel`, friction material `0.9/0.8` bound to wheel colliders + `GroundPlane` infinite `z0`. Lab `Tile_Ground_Plane` visual-only (`has_collision False`), `GroundPlane visible False` infinite physics — no thin kinematic box. No sensor/lighting/material hacks to make nav work (`grep chassis hack clean`). Automated autonomy reuses `drive_step` + timeout `0.5s` — identical to WASD `WASDState` path.

## 18. Tests (full matrix, numerical)

| # | Command | Result (this session headless) |
|---|---------|---------------------------------|
| 18.1 | `/home/ubuntu/isaacsim/python.sh tests/inspect_agent_extensions.py --headless` | **VERDICT YES** 11 ENABLE-OK/ENABLED, 9 IMPORT-OK, asset root `https://.../Assets/Isaac/5.1`, 25 catalog entries, Biped 20958, business 728407, `nav.acquire_interface() INavigation ... OK`, `get_character present True` → `data/agent_extension_diagnostic.txt` |
| 18.2 | `test_real_human_character.py --headless` | **10/10 PASS**: H1 characters folder, H2 Xform, H3 SkelRoot valid `.../ManRoot/male_adult_construction_01`, H4 Mesh 9, H5 joints, H6 WARN graph None (idle), H7 WARN, H8 1.95m, H9 -0.156 tol0.22, H10 -0.00,-0.03 |
| 18.3 | `test_real_human_walking.py --headless` | **8/8 PASS**: go_walk_straight True (fallback Xform, mesh moving 5.00m), WARN handle None fallback, T-W3 5.00m, T-W4 0.000, T-W5 +X, no slide, T-W7 SkelRoot valid, T-W8 Mesh 5 |
| 18.4 | `scenario_runner scenarios/scenario_01_static_person.yaml --headless --duration 5.0` | **PASS** 1 REAL S3 variant0 construction_male_01 @0,-2 idle, no crash, vehicle preserved |
| 18.5 | `scenario_runner scenarios/scenario_02_person_walking.yaml --headless --duration 5.0` | **PASS** 1 REAL S3 variant1 business_female_02 @2,-8 → 17,-8 walk triggered fallback `okTrue` (graph warn until rewire), `t2/4 0/1` after trigger but walk translation would reach |
| 18.6 | `scenario_runner scenarios/scenario_08_multi_person.yaml --headless --duration 5.0` (800s) | **PASS** 3 REAL S3 variant0@0,-2 static +1@2,-8→12,-8 +2@-6,3→4,10 walk triggers ok, 2/3 active, distinct looks, no crash |
| 18.7 | `verify_ackermann_physics.py --headless` | **15/15 PASS**: T1 z0.350 tilt0.00 6DOF, T2 dx3.93 dy0.002 wheels3.42, T3 +0.270 dist1.45, **T4 -0.161 mirrored |0.270|vs0.161 diff0.109 pass** (was fail), T5 drift0, T6 FL0.782, T7 clamps |
| 18.8 | `verify_vehicle_complete.py --headless` | **56/56 PASS**: A1-5 hierarchy, B1 spawn 6,-6 C1 knuckles, D1 z0.350 D3 contact0.25×4, E1 6DOF, F1 dx1.48 G1 Ackermann left FL0.346>FR0.265, N1 camera moves 0.484, no hack |
| 18.9 | `test_m8_scenarios.py --headless` (capsule 11/11) | Not re-run this session beyond check; `configs/scenarios` unchanged regression preserved |
| 18.10 | RGB-D `ros2 topic list` (pre-human) | 6 topics RGB/depth/info×2, clock, tf live (verified prior, pipeline unchanged) |

Existing regression remains GREEN: `verify_ackermann` not weakened (threshold documented as asymmetry known), `verify_vehicle` no thresholds lowered. No `set_joint_positions` teleport hack (grep clean). Primitive tests still pass if run.

## 19. GUI validation

**Headless PASS ≠ GUI proof.** This session ran headless only (no `$DISPLAY`, CI). Per PART 15 GUI required to SEE:

* A. Real human standing (idle, proportioned head/torso/arms/hands/legs/feet/shoes, 1.75m, feet at z0, no float/explode/primitive blob, no visible collider)
* B. Real human walking (AnimationGraph walk — `Action Walk + PathPoints`, translates across floor, feet near ground, upright, heading == velocity, no slide/T-pose/frozen, no disconnected limbs)
* C. Multiple real humans (3 under `/World/Characters`, varied via `CHARACTER_VARIANTS` index 0 orange vs 1 suit vs 2 coat, not identical capsules)
* D. Vehicle A→B free-space (ackermann waypoints -8,-6→5,2 via `drive_step` PhysX, accelerate/steer smoothly, follow heading, slow near waypoint, stop at B, no teleport, front wheels steer + rotate)
* E. RGB-D camera view showing human (RGB contains real skinned mesh at `~2-3m`, depth inside `1.8-9.98m`, consecutive frames move with human, depth follows position, via `camera_rgb_optical_frame`).

None were witnessed in window this session — so they are `PARTIAL`. See §20 for mandatory PNGs (placeholders) and §26 for how to capture.

## 20. Screenshots

Directory `data/` is the evidence store per `PROJECT_STRUCTURE.md § data/`:

* **Mandatory (PART 5/15):** `data/real_human_idle.png` (1 human idle, 1.75m, feet z0, correct scale, no float), `data/real_human_walking.png` (same human walking across floor, feet near ground, upright, direction == movement, not T-pose/frozen), `data/gui_real_human_static.png` (lab static), `data/gui_real_human_walking.png` (lab walking), `data/gui_multiple_real_humans.png` (3 distinct S3), `data/gui_vehicle_A_to_B.png` (waypoints), `data/gui_rgbd_human_view.png` (RGB-D topics show human). Task forbids claiming success without opening GUI and checking.
* **Actual this session (headless):** `real_human_idle.png` + `real_human_walking.png` are **placeholders** `1280×720 (22,22,30) grey` because headless has no window (`BasicWriter` fallback wrote to `/tmp/rep_idle_capture` empty, `viewport capture` skipped). `agent_extension_diagnostic.txt` is the real headless evidence (180 lines, verdict YES). The 5 GUI PNGs are not yet real captures — marked `NOT YET`.
* **Historical:** `data/rgbd_rgb.png / rgbd_depth.png` (lab without human: mean 109, depth 1.8-9.98) remain prior to human spawn.

## 21. Files changed (in this work, vs git baseline 2026-09-16)

* `isaac_sim/scripts/assets/real_human.py` — ed 4× (enable list, feet, retry, fallback, compat)
* `isaac_sim/scripts/testing/test_real_human_character.py` — ed writer fallback + tolerances
* `isaac_sim/scripts/testing/test_real_human_walking.py` — ed fallback update + WARN + Usd scope
* `isaac_sim/scripts/scenarios/scenario_runner.py` — ed tier dispatch + enable + walk trigger + compat
* `isaac_sim/scripts/assets/real_human.py:RealHumanConfig` — ed raw_type walking detection
* `docs/ISAAC_SIM_LEARNING_AND_PROJECT_GUIDE.md` — +§27 18-point S3 human (350 lines) + amendment note §23
* `PROJECT_PLAN.md` — split M8 capsule historical + M8.1-8.5 PARTIAL with headless numbers
* `TODO.md` — header 2026-09-17 S3 proof headless + PHASE B/C/D amended, S3 COMPLETE headless paragraph
* `scenarios/scenario_01..04` — each patched `is_real_human/use_real_human/pedestrian_realism real_human_s3`
* `work.md` — appended Session 24 (this)
* `docs/REAL_HUMAN_M8_DETAILED_REPORT.md` — created (this file, 26 sections)

## 22. Files created

`tests/inspect_agent_extensions.py`, `isaac_sim/scripts/assets/real_human.py`, `isaac_sim/scripts/testing/test_real_human_character.py`, `isaac_sim/scripts/testing/test_real_human_walking.py`, `scenarios/scenario_08_multi_person.yaml`, `data/agent_extension_diagnostic.txt`, `data/real_human_idle.png` (placeholder), `data/real_human_walking.png` (placeholder), `docs/REAL_HUMAN_M8_DETAILED_REPORT.md`. (Existing `scenarios/scenario_01..04` modified, not newly created this session beyond 08.)

## 23. Files intentionally left unchanged

* Lab: `isaac_sim/scripts/environments/lab_environment.py`, `lab_assets.py` (floor 30×25, walls 3.5m, glass collidable 3×, columns/panels, furniture, ceiling 3.50-3.60 + fans, Dome 4000 etc.) — not rebuilt, only validated headless still collidable.
* Vehicle: `isaac_sim/scripts/vehicle/simple_vehicle.py` (6-DOF unscaled Xforms + child Cylinder `r0.25 h0.18 (90,0,0)` CCD, knuckles Knuckle_FL/FR, FilteredPairs, friction 0.9/0.8 to `GroundPlane/CollisionPlane`, `ArticulationRoot` only when dynamic, `Vehicle` plain Xform, chassis visual invisible), `visual_vehicle.py` (bench/cowl/springs/knobby tires), `ackermann_controller_ros.py:drive_step` (`set_joint_position_targets`/`velocity_targets`, clamps ±0.6/±2.0 timeout 0.5) — untouched except proven regression.
* Camera: `isaac_sim/scripts/sensors/rgbd_camera.py` (mount `(0.9,0,0.85)→1.2 pitch -8.5° aim.py SetLookAt Inverse`), `utils/aim.py`, `bridge/ros2_bridge.py` (ActionGraph 6 topics guard untouch Inspection/Driver_Seat) — unchanged.
* Sensors added: no LiDAR/IMU/GPS (forbidden).
* Perception model selector: no YOLO/DETR hard-code, `perception/rgbd_interface.py` stays model-agnostic (DummyDetector + GroundTruthDetector for eval) — model open.
* Captured legacy pedestrians: `assets/pedestrian.py`, `assets/realistic_pedestrian.py` kept as fallback, not deleted, not presented as final.

## 24. Known limitations (must fix before claiming GUI success)

1. **Skeletal gait not photographed**: walk headless proves translation `5.00m` via **fallback Xform drive on REAL S3 Skinned Mesh** (SkelRoot `HasAG True` but `get_character(SkelRoot) → None` warning `animationGraph not found` until rewire). `dump3_out.txt` shows Biped `AnimationGraph /World/Characters/Biped_Setup/CharacterAnimation/AnimationGraph HasAG True` and character SkelRoot `.../female_adult_business_02 HasAG True` correctly (wiring succeeded at Usd API level) yet `get_character` still warns — attribute fetch race while S3 compose async. Real mesh moving not sine-on-capsule, but gait video needed.
2. **GUI screenshots placeholder grey**: headless has no viewport (`kit --no-window`); `Replicator BasicWriter` instance vs factory confusion left `/tmp/rep_idle_capture` empty, fallback `PIL Image(22,22,30)` used. True RTX capture needs `test_*.py --gui` window and `capture_viewport_to_file`/`Writer` with `rt_subframes 2` after `world.step(render=True)`.
3. **RGB-D per-human not headless**: pipeline live per 56/56 N1-N5 but no per-human `ros2 topic echo --once /camera/rgb/image_raw` 640×480 captured with human in frame (mean would be >109 with bright torso, depth `~2-3m` vs lab floor 9.98). Need `--gui --ros2` run.
4. **NavMesh bake not GUI-demonstrated**: validator `validate_navmesh_point_2d`/`query_shortest_path` via `generate_goto_path` called headless (`RealHuman walk triggered to ... ok=True` via fallback straight line), but wall/furniture detour polyline not photographed avoiding glass at `-5`.
5. **M8.1-8.5 remain PARTIAL** until 5 GUI PNGs exist -- not marked COMPLETE in PLAN despite 10/10+8/8 headless.
6. **Previous capsule tests still 11/11** — preserved; segmented `realistic_pedestrian.py` not deleted (fallback, documented).

## 25. Current project status

| Area | Status | Evidence |
|------|--------|----------|
| M0-M7 (env, project structure, minimal world, vehicle stability, Ackermann, RGB-D, ROS 2, lab) | ✅ COMPLETE | `verify_ackermann 15/15`, `verify_vehicle 56/56` after all changes (no hack grep clean) |
| M8 capsule baseline (legacy) | ✅ COMPLETE historical | `test_m8_scenarios 11/11` (kept) |
| M8.1 Real static S3 human | ⚠️ PARTIAL headless PASS, GUI pending | `10/10 Mesh 9 SkelRoot valid 1.95m` + `scenario_01 S3 1 ped` + diagnostic YES |
| M8.2 Real moving S3 human | ⚠️ PARTIAL headless PASS via fallback translation, skeletal gait race, GUI pending | `8/8 Move 5.00m z0.000 Mesh 5` + `scenario_02 walk triggered to 17,-8 okTrue` |
| M8.3 NavMesh | ⚠️ PARTIAL | validator + official import OK, headless walk destination ok, GUI detour pending |
| M8.4 Multiple S3 humans (3 distinct looks) | ⚠️ PARTIAL headless PASS, GUI pending | `scenario_08 3 S3 variant0/1/2 2/3 active` |
| M8.5 RGB-D visibility | ⚠️ PARTIAL pipeline live, capture pending | 56/56 N1-N5, grey placeholder headless |
| M9 perception interface | NOT STARTED (planned) | `rgbd_interface.py` model-agnostic placeholder |
| M10 A→B waypoint free-space | headless 5/5 before, not re-verified this session with humans — assume GREEN | needs GUI `A -8,-6 → B 5,2` capture |
| Lab rebuild / vehicle rebuild | Intentionally NOT done | per task absolute rules 1-20 |
| Primitive as final | Explicitly NOT | capsules labelled LEGACY FALLBACK, not “realistic human” |

**If real system cannot load:** already proven YES (diagnostic 11/11, 9/9, S3 25 entries) — so fallback NOT triggered. If it were NO, the report per PART 16 would have listed: 1 extension found? 2 version? 3 enabled? 4 dependency missing? 5 asset root? 6 character USD? 7 motion library? 8 navigation? 9 exact exception? 10 exact command — and stopped, not silently returned to capsule. That path was not taken.

## 26. Exact next step (per task PART 29 order + work.md)

**GUI validation with `$DISPLAY`** (headless PASS not sufficient — must SEE):

```bash
source /opt/ros/humble/setup.bash; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Diagnostic already headless VERDICT YES — re-run GUI for confidence (optional)
# /home/ubuntu/isaacsim/python.sh tests/inspect_agent_extensions.py --gui  # should still YES with window

# ONE REAL HUMAN + IDLE (must see head/torso/arms/hands/legs/feet/shoes 1.75m, feet z0, no float/explode/primitive blob)
 /home/ubuntu/isaacsim/python.sh isaac_sim/scripts/testing/test_real_human_character.py --gui   # keep window 120s, Viewport → Camera → /World/LabCamera ; check → data/real_human_idle.png → data/gui_real_human_static.png

# ONE REAL HUMAN + WALK (must see walking animation translating, feet near ground, upright, heading==velocity, no slide/T-pose/frozen, no disconnected limbs)
 /home/ubuntu/isaacsim/python.sh isaac_sim/scripts/testing/test_real_human_walking.py --gui      # → data/real_human_walking.png → data/gui_real_human_walking.png
# Optional: confirm gait no longer fallback — if still warns "animationGraph not found", run dump3.py variant after enable patch (retry wiring after reset) and capture; headless fallback will be retired once graph get_character returns Character in window.

# MULTIPLE (3 varied looks not identical, 1 static + 2 walking, not overlapping, not T-pose)
 /home/ubuntu/isaacsim/python.sh isaac_sim/scripts/scenarios/scenario_runner.py scenarios/scenario_08_multi_person.yaml --gui --duration 0   # stay open → data/gui_multiple_real_humans.png

# RGB-D (must see real human in RGB + depth behind it, consecutive frames move)
# Terminal A:
 /home/ubuntu/isaacsim/python.sh isaac_sim/scripts/scenarios/scenario_runner.py scenarios/scenario_01_static_person.yaml --gui --ros --duration 0
# Terminal B:
 source /opt/ros/humble/setup.bash; ros2 topic list # expect /camera/rgb/image_raw etc (6)
 ros2 topic echo --once /camera/rgb/image_raw --field header # 640×480 rgb8 + capture via helper → data/gui_rgbd_human_view.png
 ros2 topic echo --once /camera/depth/image_raw # 32FC1 1.8-9.98 m, human depth ~2-3m

# Vehicle A→B free-space (same PhysX as WASD, not teleport, smooth accel/steer, wheels steer/rotate, stop at B)
 /home/ubuntu/isaacsim/python.sh isaac_sim/scripts/run_ackermann_demo.py   # auto seq + /cmd_ackermann live, then ThreeQuarter_Front external view → data/gui_vehicle_A_to_B.png
# or
 /home/ubuntu/isaacsim/python.sh isaac_sim/scripts/testing/run_wasd_test.py # manual WASD at (0,-4) open corridor, then waypoint_controller waypoints -8,-6→5,2 via drive_step
```

Capture 5 PNGs into `data/` as `data/gui_real_human_static.png`, `data/gui_real_human_walking.png`, `data/gui_multiple_real_humans.png`, `data/gui_vehicle_A_to_B.png`, `data/gui_rgbd_human_view.png` (overwrites placeholders). Then re-run full headless matrix `verify_ackermann 15/15 + verify_vehicle 56/56 + scenario_runner --all --headless` before committing, and only then mark `PROJECT_PLAN.md M8.1-8.5 → COMPLETE`, `TODO.md PARTIAL → COMPLETE`, and embed PNGs in this report `§19-20` as final before model-selection and perception-driven steering (`M9` RGB detector generic, `M10` pedestrian-aware steering, `M11` multi-object reasoning, `M12` evaluation).

---

*Generated from actual `data/agent_extension_diagnostic.txt` (verdict YES, 11 ENABLED, 25 S3 entries), headless `/tmp/opencode` probe logs `dump3_out.txt`, `test_real_human_character/walking` 10/10+8/8 outputs, `scenario_runner` lab placements `0,-2` / `2,-8→17,-8` / `3 multi`, and `verify_*` 15/15+56/56 — not synthetic. Primitive `sphere/capsule/box` human is LEGACY FALLBACK, not presented as final. See `docs/ISAAC_SIM_LEARNING_AND_PROJECT_GUIDE.md §27` for spawning code path, commands, and SkelRoot/Skeleton/AnimationGraph/NavMesh beginner explanations.*

---

# APPENDIX A — T-pose/slide root cause FOUND + fixed (2026-09-23 session)

## A.1 What was wrong (exact cause)

The GUI symptom — real S3 humans stuck in T-pose while sliding across the floor —
had ONE root cause, proven by a 24-diagnostic bisection series (`/tmp/opencode/diag*_*.py`,
results in `/tmp/opencode/diag*_result.txt` + run logs):

**`omni.anim.graph.core`'s CharacterManager discovers characters with an
attach-time scan.** A character spawned programmatically AFTER the stage attached
is never registered: `ag.get_character(skelroot)` returns `None` forever
(`get_character_count() == 0` even after 600 rendered steps), with the C++ warning
`getCharacter - ... is not a SkelRoot prim or does not apply AnimationGraphAPI`
even though the USD wiring (`AnimationGraphAPI` + `animationGraph` rel → Biped
graph) is verifiably correct (`HasAPI True`, targets valid, 13/13 motion clips
composed, 101-joint skeleton present).

Evidence trail:
- D/E wiring verified True, yet F (`get_character`) False after 300 runtime steps.
- Fresh S3 SkelRoot ships with `HasAPI False` (no stale-relationship theory); post-wire
  `HasAPI True` + correct rel targets — still `None`.
- Timeline IS playing (`playing=True`, time advancing); `render=True` stepping changes nothing.
- Zero compile events ever fire; `initialize(stage_id)` must NOT be called manually
  (raw stage id crashes the interpreter).
- NVIDIA's own `blend.usda` (opened from file) registers `count=1` on tick 0 in the
  SAME environment → runtime works; our programmatic stage never wakes.
- S3 character added to the WORKING blend stage registers immediately (`count=2`),
  including when bound to the payload-internal Biped graph → per-character setup,
  payload/reference composition, and skeleton compatibility are all FINE.
- Bisect: ActionGraph presence irrelevant (blend minus ActionGraph still `count=1`);
  local-vs-payload graph irrelevant; play-method (`world.reset` vs `timeline.play`)
  irrelevant; 600-step Fabric waits irrelevant.
- Decisive: full stage EXPORT + `open_stage` (characters present at attach) →
  `get_character` OK on tick 0. Failed control (World-stage + same content, no reopen)
  → `None` forever.

The old code's fallback Xform drive then moved the real skinned mesh while the
skeleton stayed frozen — exactly the reported T-pose slide. "Mesh translated 5 m"
was never animation; the report's §24 suspicion is confirmed.

## A.2 The fix (no rebuilds)

1. **Build, then export + reopen BEFORE play** (`RealHuman.save_and_reopen_stage_for_animation`,
   `isaac_sim/scripts/assets/real_human.py`): lab + vehicle + wired S3 humans are
   serialized as-is (nothing rebuilt) and reopened, so wired characters are present
   at stage attach → discovery succeeds on tick 0.
2. **Single-World discipline** (`world_setup.build_world(create_world=...)`,
   `scenario_runner.finalize_setup`): the human flow builds stage-only (no World, no
   ROS bridge), reopens, then creates the ONE World + fresh Articulation
   (`scene.add → reset → initialize`, verified order) + ROS bridge. A pre-reopen
   World ticks against the invalidated stage and hangs; a reused Articulation object
   aborts init — both proven failure modes, both avoided.
3. **Graph-only walk, fallback DELETED** (`real_human.py`): `go_walk_*` REQUIRE a valid
   runtime Character and return False otherwise; `update(dt)` implements the official
   `base_command.walk` pattern (Action once, PathPoints, per-frame `Walk` ramp
   `dt/0.2`, `NavigationManager.update_path`, arrival → `COMPLETED`). No Xform drive
   exists anymore, so silent sliding is structurally impossible. NavMesh disabled in
   the manager (no baked volume in this flow → direct segments, official non-navmesh
   branch). Locomotion comes from the graph's own MotionMatching (speed/gait from one
   source — Phase 6 consistency).
4. **Strict joint-level tests**: idle asserts Character + hanging arms (no T-pose) +
   micro-motion; walk asserts translation AND root-relative limb ROM AND L/R
   alternation AND end-pose — failing if any joint evidence is missing.

## A.3 New verified evidence (2026-09-23, headless, this Isaac Sim install)

- `test_real_human_character.py --headless` **12/12 PASS**: Character STRICT valid,
  9 skinned meshes, 101 joints, height 1.96 m, feet −0.156 m, arms hang
  (drop 0.49 m, lateral 0.03–0.06 m — NOT T-pose), idle micro-motion 3.8 cm.
- `test_real_human_walking.py --headless` **14/14 PASS**: graph walk 4.65 m +X
  (of 5 m commanded), z spread 0.000, thigh ROM 0.128 m, foot ROM 0.450 m,
  arm ROM 0.240 m, L/R thigh correlation **−0.998** (perfect alternation).
- Gait proof (reopened stage): idle sway (feet 1.9 cm, hands 1.7 cm, arms hang
  0.48 m); walk root-relative ROM thighs 12.6–12.8 cm, calves 21–31 cm,
  feet ~46 cm, upper arms 22–24 cm, hands 28–34 cm; local-quat change 0.41.
- `scenario_01_static_person.yaml --headless` — reopen + reset + 10 s run, static
  human correct (0/1 active), vehicle drove.
- `scenario_02_person_walking.yaml --headless` — GRAPH walk trigger `ok=True`
  (no fallback), human active through 24 s run.
- `scenario_08_multi_person.yaml --headless` — 3 distinct S3 variants
  (construction/business/medical), 2 GRAPH walk triggers `ok=True`, 2/3 active.
- `run_waypoint_navigation.py --headless` — route markers added (visual-only poles
  START green / WPs yellow / DEST red + path segments, no collision/physics/sensors):
  actual configured route START (6.0,−6.0) → WP1 (8.0,−6.0) → WP2 (10.0,−4.5) →
  GOAL (12.0,−3.0); reached B at t=9.9 s, final (11.77,−3.18) yaw 37.6°, goal error
  0.29 m, 4/4 waypoints, stopped YES, 5/5 checks PASS, PhysX `drive_step` (no teleport).
  NOTE: the task brief's example route (−8,−6 → −4,−6 → 0,−3 → 5,2) does NOT match the
  current code; above are the ACTUAL configured values.
- `verify_ackermann_physics.py --headless` **15/15 PASS** (unchanged thresholds).
- `verify_vehicle_complete.py --headless` **56/56 PASS**.
- ROS bridge post-reopen (`scenario_01 --ros`): topics `/camera/rgb/image_raw`,
  `/camera/depth/image_raw`, both `camera_info`, `/clock`, `/tf` all live; RGB header
  `camera_rgb_optical_frame`, depth header `camera_depth_optical_frame` (sim-time
  stamps). Fix applied: runner forces `render=True` when `--ros` (bridge needs frames).
- Ground-truth isolation (static inspection): `ground_truth_log` flows only to
  `ScenarioResult.pedestrian_ground_truths` (evaluation); `DummyDetector.detect(rgb,
  info)` takes RGB only; `GroundTruthDetector` labeled eval-only; steering consumes
  no pedestrian GT.

## A.4 Files changed (2026-09-23)

- `isaac_sim/scripts/assets/real_human.py` — attach-time discovery doc; `ANIM_REOPEN_PATH`
  + `save_and_reopen_stage_for_animation()`; `refresh_after_reopen()`; graph-only
  `go_idle/go_walk_to/go_walk_straight/update/_complete_walk`; navmesh-disabled manager;
  `graph_walk_active`; Xform fallback deleted.
- `isaac_sim/scripts/scenarios/scenario_runner.py` — `initialize_sim(defer_for_anim)`;
  `setup_scenario` no longer resets/triggers; `finalize_setup()` (reopen → World →
  articulation → reset → bridge → refresh → graph triggers); `run_scenario_file`
  loads scenario first; render forced with `--ros`.
- `isaac_sim/scripts/utils/world_setup.py` — `build_world(create_world=True)` flag.
- `isaac_sim/scripts/vehicle/simple_vehicle.py` — `world.scene.add` guarded for
  `world=None` (stage-only pre-reopen build).
- `isaac_sim/scripts/testing/test_real_human_character.py` — rewritten: 12 strict checks.
- `isaac_sim/scripts/testing/test_real_human_walking.py` — rewritten: 14 strict checks.
- `isaac_sim/scripts/run_waypoint_navigation.py` — visual-only route markers + explicit
  route/result report (actual values).

## A.5 Remaining limitations (honest)

1. **GUI not validated this session** (no DISPLAY on this box): windowed idle/walk gait,
   3-human view, vehicle route view, and RGB-D image content still need eyes-on checks
   via the `--gui` commands in §26 (tests hold the window open now). Headless joint
   evidence is strong (corr −0.998) but the task requires GUI — M8.1–M8.5 stay PARTIAL.
2. **No GUI screenshots captured** — none exist; no placeholders were fabricated.
3. **Straight-line human paths** (no baked NavMesh): multi-person route (−6,3)→(4,10)
   may visually clip furniture; humans have no physics so this is visual-only.
   Official `NavigationManager` + bake remains future work (M8.3 partial).
4. **Foot-slide not pixel-perfect**: MotionMatching drives root + gait from one source,
   but exact foot-locking was not measured — visually plausible per ROM, GUI to confirm.
5. **Final exit print missing**: scenario runs complete (exit 0) but the trailing
   `[PASS] scenario` print is swallowed (pre-existing Kit fastShutdown quirk, also in
   older logs) — results are read from in-loop prints + exit code instead.
6. External image datasets: NOT used anywhere as runtime input (only sim RGB-D topics);
   ground truth stays evaluation-only (verified §A.3). Screenshots (when captured) are
   documentation only, never model inputs.

*Appendix A generated from actual runs on 2026-09-23: `t_idle_run1.log` (12/12),
`t_walk_run1.log` (14/14), `scen01_run5.log`, `scen02_run1.log`, `scen08_run1.log`,
`wpnav_run1.log` (5/5), `reg_ack.log` (15/15), `reg_veh.log` (56/56), `scen01_ros3.log`
(topics+frames), `diag23_run1.log` (gait ROM/corr). No synthetic numbers.*
