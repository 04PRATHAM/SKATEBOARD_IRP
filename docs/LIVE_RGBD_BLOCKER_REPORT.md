# Live RGB-D Blocker Report — syntheticdata writer fault (2026-10-08/09)

## 1. Exact failing operation

Attaching ANY render-product writer/annotator for the RGB-D camera, e.g.
`annotator.attach([render_product_path])` / `BasicWriter.attach(...)` /
OG `IsaacCreateRenderProduct` + `ROS2CameraHelper` graph build. Fails inside
`omni/syntheticdata/.../SyntheticData.py:1070 _add_node_downstream_intergraph_dependency`
(`dep_attrib_data.set(dep_data)` on empty arrays) and at OG writer nodes
(`inputs:render_products:resolution` unwritable, `kind=i/f, size=0`).

## 2. Reproduction command (minimal)

```bash
/home/ubuntu/isaacsim/python.sh /home/ubuntu/isaacsim/standalone_examples/api/isaacsim.sensors.camera/camera.py
# NVIDIA's own example, unmodified. Fails identically (see log excerpt below).
```

Project-local repro: `.../python.sh isaac_sim/scripts/testing/diagnose_rgb_writer.py --headless --steps 5 --skip-l1`
(L1/L2/L3/L3b/L4 branches), and `probe_empty_stage_writer.py` (bare stage).

Relevant error (identical in all repros):

```text
File ".../camera.py", line 390, in initialize
    self.attach_annotator(annotator_name="rgb")
File ".../annotators.py", line 682, in attach
    activated_result = sdg_iface.activate_node_template(...)
    ...
File ".../SyntheticData.py", line 1070, in _add_node_downstream_intergraph_dependency
    dep_attrib_data.set(dep_data)
TypeError: Unable to write from unknown dtype, kind=f, size=0
```

## 3. Diagnostic branches already tested (all FAIL identically)

- Cold vs warm attach (10 rendered frames + timeline play before attach).
- `annotator_device` cpu / cuda / default.
- Headless vs GUI (Xvfb :99 window).
- Camera API vs rep `BasicWriter` vs OG bridge graph.
- Dedicated viewport window bound to the RGBD camera.
- Viewport buffer capture (480 s / ~14k rendered steps: zero Hydra frames, empty AOV list).
- Bare empty stage (ground plane + one camera) — rules out scene complexity.
- NVIDIA's unmodified `camera.py` example — rules out project code.
- GPU contention ruled out (fails at 4 % load, 46 °C).

## 4. Approaches ruled out by evidence

- GPU contention (fails unloaded).
- Wrong camera prim (verified real `UsdGeom.Camera`).
- Bridge wiring (matches NVIDIA's own `test_camera.py`; Camera-API path fails without any graph).
- Missing warmup/viewport (tested, no effect).
- Scene complexity (empty stage fails).
- Headless-only (Xvfb GUI fails identically).

## 5. Assessment

Systemic to this Isaac Sim 5.1.0-rc.19 install (`omni.syntheticdata-0.6.13` +
`omni.replicator.core-1.12.27` intergraph wiring). Matches NVIDIA forum "Camera Initialization
Issue" (Sept 2025, same stack, 5.0) — no posted fix. `/clock` + `/tf` still publish (no writers
needed), which is consistent with all observations including earlier partial claims.

## 6. External fix / approved environment change required (any ONE)

- NVIDIA patch/fixed extension set for the 5.1 syntheticdata intergraph path, OR
- Guidance-approved targeted extension update (currently forbidden by project constraints), OR
- Confirmation that GUI-on-real-display presentation unblocks Hydra frames (untested: intrusive
  without permission; Xvfb evidence suggests display is not the differentiator).

Until then: pipeline stays BLOCKED, hard gate stays, no fake data. No bridge rewrite, no topic
changes, no upgrades performed.

## Addendum 2026-10-13 — Milestone 11: viewport-backed product ruled out (Outcome B)

New single-variable experiment (`probe_viewport_product.py`): dedicated viewport window bound
to the camera exposes `/Render/OmniverseKit/HydraTextures/omni_kit_widget_viewport_ViewportTexture_1`;
`BasicWriter.attach` on it fails with the identical dtype error. Product-source variable is
eliminated — the fault is product-source-independent.

Full experiment matrix (each one variable, none repeated without new information):
cold/warm attach; annotator_device cpu/cuda/default; headless/Xvfb-GUI; Camera API / rep
BasicWriter / OG bridge graph; dedicated viewport window + camera binding; viewport-backed
product path; 480 s viewport frame wait (zero frames, empty AOVs); empty stage; NVIDIA's
unmodified `camera.py` example. Environment: Isaac Sim 5.1.0-rc.19+release.26219.9c81211b,
omni.syntheticdata-0.6.13+69cbf6ad, omni.replicator.core-1.12.27+107.3.3, python 3.11.13,
torch 2.7.0+cu128, RTX PRO 6000 Blackwell 96 GB, driver 580.173.02, Ubuntu 22.04.5.

Conclusion: installation-level fault, vendor fix required (matched final-5.1.0 extension set or
NVIDIA guidance). No bridge rewrite, upgrade, topic change, or fake data performed.

### External-resolution request package
- Build: 5.1.0-rc.19+release.26219.9c81211b (ext/extscache only; no other syntheticdata on disk).
- Repro: `/home/ubuntu/isaacsim/python.sh standalone_examples/api/isaacsim.sensors.camera/camera.py`
  (unmodified) → `camera.initialize()` → traceback ending in
  `SyntheticData.py:1070 _add_node_downstream_intergraph_dependency: dep_attrib_data.set(dep_data)`
  → `TypeError: Unable to write from unknown dtype, kind=f, size=0`.
- Expected: annotator attach succeeds; `get_rgb()` returns uint8 frames (as in 4.5).
- Observed: attach raises in all paths above; no Hydra frames in 480 s; OG ROS helpers fail at
  graph build with the same error (`kind=i/f`).
- Reference: NVIDIA forum "Camera Initialization Issue" (Sept 2025, same stack, unresolved).

## Addendum 2026-10-13 M12 — Phase A live-graph verdict + viewport-fed helper ruled out

Phase A (live system, scenario_02 `--headless --ros`): `/camera/rgb|depth/image_raw` advertised
(sensor_msgs/Image) but **Publisher count 0**, `hz`/`echo --once` silent; `/clock` ~43–48 Hz alive;
a third-party `yolo_seg_node` subscribed and starved. Sim log: 4× `unknown dtype` writer errors.
Verdict: RGB FAIL (advertised, silent), depth FAIL, clock/tf PASS. The active image path IS the
implicated OG writer path.

Phase A2 (new, isolated): OG `ROS2CameraHelper` fed by a viewport render product (no
`IsaacCreateRenderProduct`) — graph builds, then the IDENTICAL `unknown dtype (kind=i)` writer
failure (`probe_ros_viewport_product.py`). All supported in-install combinations now exhausted
without touching production code. Outcome B stands; external fix required.

## Addendum 2026-10-13 (tutorial workflow) — reference Camera Helper uses the SAME failing path

Investigated installed `OgnROS2CameraHelper.py`: it IS a `BaseWriterNode` subclass acquiring
writers via `rep.writers.get(...)` + `omni.syntheticdata` activation — i.e. the tutorial's Camera
Helper is the same underlying writer path, not an alternative. The installed documented example
(`standalone_examples/api/isaacsim.ros2.bridge/camera_periodic.py`: viewport product +
`IsaacSetCameraOnRenderProduct` + rgb/depth/info helpers, push-graph) was run verbatim under
Xvfb: 3× identical `unknown dtype` writer errors on `.../ViewportTexture_0`; topics `rgb`/`depth`
advertised-but-silent (`hz`: not published). Verdict: tutorial workflow FAILs identically —
common failure boundary confirmed at syntheticdata writer attach; no cosmetic graph variation
left to try. YOLO/fusion/autonomy untouched. Production bridge unchanged.

## Addendum 2026-10-13 M13 — manifest audit + failure mechanism + vendor package

- Single copies on disk (no duplicates/shadows): syntheticdata 0.6.13+69cbf6ad, replicator
  1.12.27+107.3.3, omni.graph 1.141.2+69cbf6ad. Mixed build trains inside one rc.19 bundle
  (69cbf6ad vs 107.3.3) — noted as hypothesis context, not proof.
- Exact mechanism (installed source): `_add_node_downstream_intergraph_dependency` creates a
  UINT64 state attr, then `.set(numpy.append(readback, [handle]))` with an EMPTY array —
  failure sits at the syntheticdata↔omni.graph attribute-data interop boundary.
- Supported-alternative dispositions (all evidenced): OG helpers, Camera API, BasicWriter,
  viewport products, viewport capture, empty stage — FAIL; RTX-lidar path N/A (wrong modality +
  no-new-sensors); Isaac ROS stack deferred (needs compatibility assessment); registry update
  not performed (forbidden without approval).
- Vendor package: `docs/VENDOR_SUPPORT_REQUEST.md` (build, repro, traceback, mechanism,
  experiment table, reference thread). No credentials. No vendor response exists.
- Post-fix acceptance order: writer repro clean → live RGB → live depth → message validation →
  `topic info` publishers → sustained `hz` → YOLO-on-live → fusion → Object3D (gates G1–G8
  enforced by launcher + `check_rgbd_topics.py`; autonomy never before all pass).

## Addendum 2026-10-13 M14 — custom-publisher route closed + GUI design-only verdict

New probe (`probe_hydra_readback.py`, empty stage + red-box witness): Hydra texture handle
exists, but `getFrameData failed` and `RpResource for aov 'LdrColor' is not available` — the
render product carries NO rendered resources (consistent with 480 s of zero Hydra frames and
the writers' empty arrays). Any CPU readback (`get_drawable_resource`, same buffer) or custom
ROS publisher built on it has no data source; depth AOVs equally absent. A custom publisher
that doesn't invoke the failing path is therefore impossible, not just unimplemented.

GUI extension verdict (design only, per milestone): technically feasible with installed Kit APIs
(`omni.kit` menu/window extension points, `omni.ui` buttons/toggles, viewport utility for scene
camera, ROS-topic presence subscribers for status lights) for scene-load / play / perception
start-stop / overlay toggle / RGB-depth-YOLO-fusion-Object3D status. Full extension NOT built:
status lights for image stages could never turn green until the writer fault is fixed, and no
vehicle motion may be gated on it. Build it only after live RGB validates.

## Addendum 2026-10-13 M15 — fault isolated to omni.graph native int-array write

New reversible instrumentation (installed sources untouched):
- `probe_dtype_intercept.py` (monkeypatched `AttributeValueHelper.set` logging): the failing
  call is `replicator/.../writers.py:849 _attach` →
  `writer_node.get_attribute("inputs:render_products:resolution")).set(...)`, i.e. an OG-level
  writer attach, not the SyntheticData intergraph line from older traces (same error family).
- `writers.py:831-849` reads product `resolution` attr → list of values → `.set()` on an
  `INT[2]`-array attribute created two lines above (`og.Type(og.BaseDataType.INT, 2, 1)`).
- Product prim itself is VALID with `resolution=(640,480)` and a correct camera rel
  (measured) — the source value is present and correct.
- `probe_og_conversion.py` (bare `int2[]` attribute, no rendering/writers/syntheticdata):
  `[[640,480]]`, `[Gf.Vec2i]`, tuples, `np.int32`/`np.int64` 1D+2D — ALL fail with the byte-identical
  `kind=i, size=0` error; only flat `[640,480]` yields a *different*, informative shape error.
  Conclusion: omni.graph 1.141.2's native `attribute.set` cannot write nested int values to
  int-array attributes from Python in this build. Verdict: option D (incompatible internal
  API/data structure between installed extensions), proximate symptom B. A (declared-type
  mismatch) ruled out at declaration level; C (unrendered product) ruled out as the attach-time
  trigger — attach fails on the write itself with valid source data present.
- ROS boundary (M12 evidence stands, code path unchanged since — no repeat per policy):
  topics advertised, 0 publishers; encodings NOT OBSERVABLE (not failures).
- YOLO subscriber: node subscribes `/camera/rgb/image_raw`; callbacks never fire →
  BLOCKED BY UPSTREAM RGB PUBLICATION, not a model-format issue. Fusion live checks BLOCKED.
- Missing evidence for full closure: none critical — mechanism, boundary, and repro are complete.
  Single untested variable: real-`:1`-display presentation (intrusive; Xvfb evidence suggests
  display-independent). Vendor package updated with this mechanism.

## Addendum 2026-10-13 M16 — setter contract mapped; failures classified SEPARATE

- Control matrix (`probe_og_conversion.py`): scalar-INT PASS, TOKEN scalar/array PASS;
  every nested numeric form FAILs opaquely; flat forms fail informatively at Python validation.
  Contract: nested numeric array writes have NO valid input in omni.graph 1.141.2 → no
  supported workaround exists at caller level; fix belongs in native conversion.
- Classification: setter failure vs missing camera buffers = SEPARATE failures (setter dies with
  valid source data, no rendering involved); connection UNPROVEN. Either alone blocks the pipeline.
- Vendor request updated with matrix. No production changes. Gates unchanged.

## Addendum 2026-10-16 — fix attempted at any cost, reverted, install pristine

- USD-direct write PROVEN working (tuples AND Gf.Vec2i roundtrip `[[640 480]]` via OG get).
- Patch A (writers.py resolutions, backup kept, try-native-first): execution advanced past line
  849 to annotator activation — then died at intergraph UINT64 set (NumPy 2 NEP-50 float64).
- Patch B (exact uint64 cast): error moved kind=f → kind=u size=0; control matrix proves even
  proper uint64 input fails → NO caller-side fix can succeed.
- BOTH patches reverted and verified pristine (0 marker lines); .bak files retained for audit.
- Full record: `docs/WRITER_FIX_RECORD.md`. Gates unchanged. Vendor fix remains the only route.
