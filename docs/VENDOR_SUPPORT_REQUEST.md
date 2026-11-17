# Vendor Support Request — Isaac Sim syntheticdata writer-attach fault

(No credentials included. Reproduction uses only NVIDIA-shipped examples plus small
read-only diagnostic scripts; full logs retained under `/tmp/*.log` and Kit log dir.)

## Environment
- Isaac Sim `5.1.0-rc.19+release.26219.9c81211b.gl`, Ubuntu 22.04.5, python 3.11.13,
  torch 2.7.0+cu128, NVIDIA RTX PRO 6000 Blackwell 96 GB, driver 580.173.02.
- `omni.syntheticdata-0.6.13+69cbf6ad.lx64.r.cp311` (sole copy on disk).
- `omni.replicator.core-1.12.27+107.3.3.lx64.r.cp311` (sole copy on disk).
- `omni.graph-1.141.2+69cbf6ad.lx64.r.cp311`.
- Note mixed build trains (`69cbf6ad` vs `107.3.3`) inside one rc bundle.

## Minimal reproduction (unmodified NVIDIA example)
```bash
/home/ubuntu/isaacsim/python.sh \
  /home/ubuntu/isaacsim/standalone_examples/api/isaacsim.sensors.camera/camera.py
```
Fails in `Camera.initialize()` → `attach_annotator("rgb")`.

## Exception (identical in all paths)
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
(Also observed as `kind=i, size=0` from OG writer nodes.)

## Precise failing operation (from installed source)
`SyntheticData._add_node_downstream_intergraph_dependency` creates state attribute
`state:_sdp_intergraph_downstream_node_handles_` of type `UINT64`, then
`dep_data = numpy.append(dep_attrib_data.get(), [downstream_node_handle])` and
`dep_attrib_data.set(dep_data)`. The `.set()` receives an **empty** float/int array,
so either the read-back or the downstream handle is empty at the syntheticdata↔omni.graph
interop boundary in this bundle.

## Experiments already completed (all FAIL identically)
Camera-direct / viewport-backed / empty-stage render products ×
Camera API / rep BasicWriter / OG `ROS2CameraHelper` writers;
cold/warm attach; `annotator_device` cpu/cuda/default; headless/Xvfb-GUI;
`IsaacCreateViewport`+`IsaacGetViewportRenderProduct` documented example;
480 s viewport frame wait (zero Hydra frames, empty AOV list).
Live impact: `/camera/rgb|depth/image_raw` advertised with 0 publishers; `/clock` alive.

## Request
A supported matched-extension fix (or documented alternative) for render-product writer
attachment in the 5.1 line, or confirmation of the exact version set in final 5.1.0 that
resolves it. Related: NVIDIA forum "Camera Initialization Issue" (Sept 2025, same stack,
no posted fix).

## Update 2026-10-13 M16 — native setter contract mapped (control cases included)

Bare-attribute matrix (`og.Type` INT-family arrays, no rendering/writers involved):
- scalar INT, TOKEN scalar/array: PASS (setter machinery works; roundtrip verified).
- ALL nested numeric forms (plain/Gf/tuple, np.int32/int64/uint64, float): FAIL with the
  identical opaque `Unable to write from unknown dtype` error.
- Flat forms: informative Python-side shape error (validation layer works).
Repro: `isaac_sim/scripts/testing/probe_og_conversion.py --headless` (reversible, no
installed-file edits). Conclusion: no valid input exists for nested numeric array writes in
omni.graph 1.141.2 — there is no supported caller-side workaround; the fix must be in the
native conversion (`attribute.set` for INT/UINT64/FLOAT array attrs). Render-buffer emptiness
is a SEPARATE failure (setter dies with valid source data present; connection UNPROVEN).
