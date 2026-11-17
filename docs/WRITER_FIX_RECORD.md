# Writer-Fix Record — everything attempted to restore live RGB-D (2026-10-13/16)

## 0. Environment and installation checks (nothing missing)
- Isaac Sim `5.1.0-rc.19+release.26219.9c81211b.gl`, Ubuntu 22.04.5, isaacsim python 3.11.13
  (torch 2.7.0+cu128, **NumPy 2.4.6**, ultralytics 8.4.163), perception_venv py3.10,
  RTX PRO 6000 Blackwell 96 GB, driver 580.173.02, ROS 2 Humble.
- All required extensions load cleanly (`isaacsim.replicator.writers-1.0.17` startup in logs;
  Hydra scene/usdrt delegates register; zero extension errors in Kit logs).
- Single on-disk copies: `omni.syntheticdata-0.6.13+69cbf6ad`,
  `omni.replicator.core-1.12.27+107.3.3`, `omni.graph-1.141.2+69cbf6ad` (mixed build trains
  inside one rc bundle — context, not proof).
- Registry config exists (`kit/default` + `kit/sdk` blob URLs); index layout not resolvable
  read-only (`index.zip` absent at guessed paths) — no extension update performed.
- S3 asset CDN reachable (HTTP 200). GPU contention ruled out (fails at 4 % load).
- Verdict: installation is COMPLETE — no missing component found. Fault is code-level.

## 1. Diagnosis chain (each step evidenced, repo probes cited)
1. `diagnose_rgb_writer.py` L1/L2/L3/L3b/L4: Camera-API, OG bridge, rep BasicWriter, viewport-bound
   writer, viewport buffer capture — all fail; warmup/timeline/viewport/GUI-Xvfb change nothing.
2. NVIDIA's own `standalone_examples/.../camera.py` fails byte-identically → not project code.
3. `probe_empty_stage_writer.py`: bare stage fails → not scene complexity.
4. `probe_viewport_product.py`: viewport-backed product fails identically → not product source.
5. `probe_dtype_intercept.py` (reversible monkeypatch): failing call = `writers.py:849 _attach`
   `inputs:render_products:resolution .set()`; then `SyntheticData.py:1070` intergraph UINT64
   state `.set()` with `numpy.append(uint64-empty, [handle])` → **float64** (NumPy 2 NEP-50
   promotion: uint64 array + Python int → float64; bundle written for NumPy 1).
6. `probe_og_conversion.py` (bare attrs, no rendering): scalar-INT PASS, TOKEN scalar/array
   PASS; EVERY nested numeric form (plain/Gf/tuple, np.int32/int64/uint64/float) FAILs opaquely;
   flat forms fail informatively at Python validation.
7. `probe_ros_viewport_product.py`: viewport product → OG ROS2CameraHelper: graph builds, same
   writer failure. NVIDIA `camera_periodic.py` verbatim: same 3× errors, topics silent.
8. `probe_hydra_readback.py`: `getFrameData failed`, no LdrColor resource, 480 s zero Hydra
   frames, empty AOVs — SEPARATE renderer-output fault (connection to (9) UNPROVEN).
9. Installed `OgnROS2CameraHelper.py` read: it IS a `BaseWriterNode` using `rep.writers` +
   syntheticdata — the tutorial path is the same failing path, not an alternative.

## 2. Fix attempts (both reverted; install is byte-pristine, .bak files kept alongside)
- **Patch A** (`replicator/.../scripts/writers.py`, resolutions `.set()` → try native, on
  `TypeError` write via USD `prim.GetAttribute(...).Set([tuple(...)])`; verified USD path accepts
  tuples AND Gf.Vec2i with OG readback `[[640 480]]`). Effect: execution advanced past line 849
  to the annotator-activation failure. Reverted (insufficient alone; see 3).
- **Patch B** (`syntheticdata/.../scripts/SyntheticData.py`, `_add_node_downstream...`: exact
  `.astype(uint64)` cast after append). Effect: error moved `kind=f` → `kind=u, size=0` — the
  native UINT64 set rejects even proper uint64 input (confirmed by §1 control matrix).
- Revert verified: `grep -c SKATEBOARD-IRP` = 0/0 on both files. Backups retained at
  `.../writers.py.bak.SKATEBOARD` and `.../SyntheticData.py.bak.SKATEBOARD` for audit.
- Registry update NOT performed (no resolvable index, no approval for speculative upgrade).

## 3. Final verdict (proven, minimal)
- writers.py fix works around its ONE call site, but the pipeline dies at the next native
  int-array write, and the control matrix proves NO input form works → **no caller-side fix can
  succeed; a vendor native-conversion fix is mandatory**. Reverting keeps the install safe for
  the GUI work the project depends on.
- Independently, no Hydra frames render here, so writer success alone would still yield silent
  topics until that second fault is also resolved.
- Live gates stay BLOCKED; hard gate intact; zero fake data; autonomy untouched.

## 4. What would actually resolve it (in order)
1. Vendor fixed extension set (native int-array `attribute.set` + intergraph attach) for 5.1.
2. Re-run `probe_og_conversion.py` (expect nested PASS) → `probe_empty_stage_writer.py`
   (expect PNG) → `ros2 topic hz` on both image topics → YOLO-on-live → fusion → Object3D.
3. Investigate zero-frame rendering only after (1) lands (may share a native root cause).
