"""OG int-array conversion probe (M15: is Gf.Vec2i the conversion trigger?).

Creates INT-array attributes on a scratch OG node and .set()s structural
variants. No rendering involved (~1 min). If [Gf.Vec2i] fails while [[640,480]]
passes, the writer fault is a Gf passthrough conversion gap (replicator passes
USD Gf types; graph native layer rejects them) — not empty data at all.

Run: /home/ubuntu/isaacsim/python.sh isaac_sim/scripts/testing/probe_og_conversion.py --headless
"""
import argparse
import sys

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true", default=False)
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402
sim = SimulationApp({"headless": args.headless, "renderer": "RayTracedLighting"})

import traceback  # noqa: E402
from pxr import Gf  # noqa: E402
import omni.graph.core as og  # noqa: E402

results = {}


def trial(label, value, base=og.BaseDataType.INT, depth=2, count=1):
    trial.i += 1
    my_id = trial.i
    try:
        og.Controller.edit(
            {"graph_path": "/ConvTest", "evaluator_name": "execution"},
            {og.Controller.Keys.CREATE_NODES: [("N%d" % my_id, "omni.graph.action.OnPlaybackTick")]},
        )
    except Exception:
        pass
    try:
        g = og.get_graph_by_path("/ConvTest")
        node = list(g.get_nodes())[-1]
        attr = og.Controller.create_attribute(
            node=node, attr_name="t_arr_%d" % my_id,
            attr_type=og.Type(base, depth, count),
            attr_port=og.AttributePortType.ATTRIBUTE_PORT_TYPE_INPUT)
        og.AttributeValueHelper(attr).set(value, update_usd=False)
        back = og.AttributeValueHelper(attr).get()
        print("[PASS] %-28s roundtrip=%r" % (label, str(back)[:60]), flush=True)
        results[label] = True
    except Exception as e:
        print("[FAIL] %-28s %r" % (label, e), flush=True)
        results[label] = False


trial.i = 0
import numpy as _npv
# Controls: scalar INT (machinery sanity) + TOKEN array (writer flow used these OK)
trial("scalar-INT 640", 640, base=og.BaseDataType.INT, depth=1, count=0)
trial("token-array ['a']", ["a"], base=og.BaseDataType.TOKEN, depth=1, count=1)
trial("float-array [[1.5]]", [[1.5]], base=og.BaseDataType.FLOAT, depth=1, count=1)
trial("uint64-array [[5]]", _npv.array([[5]], dtype=_npv.uint64),
      base=og.BaseDataType.UINT64, depth=1, count=1)
trial("np-int32 [[640,480]]", _npv.array([[640, 480]], dtype=_npv.int32))
trial("np-int32 [640,480]", _npv.array([640, 480], dtype=_npv.int32))
trial("np-int64 [[640,480]]", _npv.array([[640, 480]], dtype=_npv.int64))
trial("plain-nested [[640,480]]", [[640, 480]])
trial("plain-flat [640,480]", [640, 480])
trial("gf-list [Vec2i]", [Gf.Vec2i(640, 480)])
trial("gf-tuple (Vec2i,)", (Gf.Vec2i(640, 480),))
trial("tuple-nested ((640,480),)", ((640, 480),))
print("[USD-PATH] locating USD prim for writer-style attr", flush=True)
try:
    import omni.usd as _uu
    _stage = _uu.get_context().get_stage()
    from pxr import Sdf as _Sdf2
    g2 = og.get_graph_by_path("/ConvTest")
    node2 = list(g2.get_nodes())[-1]
    attr2 = og.Controller.create_attribute(
        node=node2, attr_name="usd_arr",
        attr_type=og.Type(og.BaseDataType.INT, 2, 1),
        attr_port=og.AttributePortType.ATTRIBUTE_PORT_TYPE_INPUT)
    # Find the backing USD prim/attribute for this OG session attribute
    found = []
    for p in _stage.Traverse():
        if "ConvTest" in str(p.GetPrimPath()):
            found.append(str(p.GetPrimPath()))
    print("[USD-PATH] ConvTest prims: %s" % found[:8], flush=True)
    results["usd-prim-visible"] = len(found) > 0
except Exception as e:
    print("[FAIL] usd-prim-visible %r" % e, flush=True)
    results["usd-prim-visible"] = False
print("[RESULT] %s" % ("; ".join("%s=%s" % (k, "PASS" if v else "FAIL") for k, v in results.items())), flush=True)
sim.close()
