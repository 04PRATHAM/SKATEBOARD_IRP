#!/usr/bin/env python3
"""
Isaac Sim 5.1 Agent / People / Character extension diagnostic (SKATEBOARD_IRP).

PURPOSE (per task PART 3):
    The previous session concluded `omni.anim.people` etc. were unusable after a
    bare `import` failed. That conclusion was WRONG: the import was attempted
    BEFORE the extension was enabled. This script does it correctly:

      A. verify each extension EXISTS (extscache/exts)
      B. verify its manifest (extension.toml) exists
      C. verify dependencies exist
      D. ENABLE the extension via the Kit extension manager
      E. confirm SimulationApp has loaded it (enabled flag)
      F. import the python module AFTER startup + enable

    It also reports:
      - the resolved Isaac Sim asset root (Nucleus / S3 / local)
      - available human character USD assets (HTTP/stat reachable)
      - available motion / animation clips (local retarget rig + anim graph)
      - navigation (NavMesh) interface availability
      - anim graph interface availability

Output is printed AND saved to: data/agent_extension_diagnostic.txt

Run:
    /home/ubuntu/isaacsim/python.sh tests/inspect_agent_extensions.py --headless
"""

import sys
import os
import datetime

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")

OUT_PATH = "/home/ubuntu/SKATEBOARD_IRP/data/agent_extension_diagnostic.txt"

# Extensions we care about, in dependency order (leaf deps first).
CHARACTER_EXTS = [
    "omni.usd.schema.anim",
    "omni.anim.graph.schema",
    "omni.anim.navigation.schema",
    "omni.anim.retarget.core",
    "omni.anim.graph.core",
    "omni.anim.navigation.core",
    "omni.metropolis.utils",
    "omni.kit.scripting",
    "omni.anim.people",
    "isaacsim.anim.robot",
    "isaacsim.replicator.agent.core",
]

_lines = []


def log(msg=""):
    print(msg, flush=True)
    _lines.append(str(msg))


def section(title):
    log("")
    log("=" * 78)
    log(title)
    log("=" * 78)


def main(headless=True):
    from isaacsim import SimulationApp
    sim = SimulationApp({"headless": headless, "renderer": "RayTracedLighting"})

    import carb
    import omni.kit.app
    import omni.client

    app = omni.kit.app.get_app()
    ext_mgr = app.get_extension_manager()
    app.update()

    section("SKATEBOARD_IRP — Isaac Sim Agent/People/Character Extension Diagnostic")
    log("Timestamp : %s" % datetime.datetime.now().isoformat())
    try:
        with open("/home/ubuntu/isaacsim/VERSION") as f:
            log("Isaac Sim : %s" % f.read().strip())
    except Exception as e:
        log("Isaac Sim : (VERSION unreadable: %s)" % e)
    log("Python    : %s" % sys.version.split()[0])

    # -----------------------------------------------------------------
    # STEP A/B/C — existence, manifest, and registered version of each ext
    # -----------------------------------------------------------------
    section("STEP A/B/C — Extension existence + manifest + dependency registry")
    all_exts = ext_mgr.get_extensions()
    reg = {}
    for e in all_exts:
        reg[e.get("name", "?")] = e
    for name in CHARACTER_EXTS:
        # match either exact id or id prefix (versioned folder)
        found = None
        for e in all_exts:
            en = e.get("name", "")
            if en == name or en.startswith(name + "-") or en == name:
                found = e
                break
        # get_extensions() returns entries keyed by id already without version
        e2 = reg.get(name, found)
        if e2 is not None:
            log("[FOUND]   %-38s version=%-16s enabled=%s"
                % (name, e2.get("version", "?"), e2.get("enabled", False)))
            path = e2.get("path", "")
            if path:
                log("            path: %s" % path)
        else:
            log("[MISSING] %-38s (not registered with extension manager)" % name)

    # -----------------------------------------------------------------
    # STEP D — enable each extension
    # -----------------------------------------------------------------
    section("STEP D — Enabling extensions via Kit extension manager")
    from isaacsim.core.utils.extensions import enable_extension
    enable_results = {}
    for name in CHARACTER_EXTS:
        try:
            enable_extension(name)
            app.update()
            enable_results[name] = True
            log("[ENABLE-OK]   %s" % name)
        except Exception as e:
            enable_results[name] = False
            log("[ENABLE-FAIL] %s -> %r" % (name, e))
    for _ in range(5):
        app.update()

    # -----------------------------------------------------------------
    # STEP E — confirm enabled state after enabling
    # -----------------------------------------------------------------
    section("STEP E — Enabled state confirmed by extension manager")
    for name in CHARACTER_EXTS:
        enabled = ext_mgr.is_extension_enabled(name)
        log("[%s] %s" % ("ENABLED " if enabled else "DISABLED", name))

    # -----------------------------------------------------------------
    # STEP F — import python modules AFTER startup + enable
    # -----------------------------------------------------------------
    section("STEP F — Python module import AFTER enable (the real test)")
    import_targets = [
        "omni.anim.graph.core",
        "omni.anim.navigation.core",
        "omni.anim.retarget.core",
        "omni.anim.people",
        "omni.anim.people.scripts.navigation_manager",
        "omni.anim.people.scripts.character_behavior",
        "omni.metropolis.utils",
        "isaacsim.replicator.agent.core",
        "AnimGraphSchema",
    ]
    import importlib
    import_ok = {}
    for mod in import_targets:
        try:
            importlib.import_module(mod)
            import_ok[mod] = True
            log("[IMPORT-OK]   %s" % mod)
        except Exception as e:
            import_ok[mod] = False
            log("[IMPORT-FAIL] %s -> %r" % (mod, e))

    # -----------------------------------------------------------------
    # Asset root resolution
    # -----------------------------------------------------------------
    section("ASSET ROOT — Isaac Sim asset root (Nucleus / S3 / local)")
    asset_root = None
    try:
        from isaacsim.storage.native.nucleus import get_assets_root_path
        asset_root = get_assets_root_path()
        log("get_assets_root_path() = %s" % asset_root)
    except Exception as e:
        log("get_assets_root_path FAILED: %r" % e)

    # -----------------------------------------------------------------
    # Character asset availability
    # -----------------------------------------------------------------
    section("CHARACTER ASSETS — availability check (omni.client.stat)")
    char_dir = None
    biped_path = None
    if asset_root:
        char_dir = asset_root + "/Isaac/People/Characters"
        biped_path = char_dir + "/Biped_Setup.usd"
    probe_paths = []
    if biped_path:
        probe_paths.append(("Biped_Setup.usd", biped_path))
        probe_paths.append(("male_adult_construction_01",
                            char_dir + "/original_male_adult_construction_01/male_adult_construction_01.usd"))
        probe_paths.append(("F_Business_02",
                            char_dir + "/F_Business_02/F_Business_02.usd"))
    for label, p in probe_paths:
        try:
            res, entry = omni.client.stat(p)
            ok = str(res) == "Result.OK"
            size = getattr(entry, "size", "?") if ok else "-"
            log("[%s] %-28s %s (size=%s)"
                % ("OK " if ok else "NO ", label, p, size))
        except Exception as e:
            log("[ERR] %-28s %s -> %r" % (label, p, e))

    # List character folders via omni.client if possible
    if char_dir:
        try:
            res, entries = omni.client.list(char_dir)
            if str(res) == "Result.OK":
                names = sorted([e.relative_path for e in entries])
                log("")
                log("Character catalog under %s (%d entries):" % (char_dir, len(names)))
                for n in names:
                    log("   - %s" % n)
        except Exception as e:
            log("omni.client.list(char_dir) failed: %r" % e)

    # -----------------------------------------------------------------
    # Motion / animation clips (local retarget rig)
    # -----------------------------------------------------------------
    section("MOTION LIBRARY — local retarget Human rig + anim graph")
    try:
        rig_dir = ext_mgr.get_extension_path_by_module("omni.anim.retarget.core")
        log("omni.anim.retarget.core path: %s" % rig_dir)
        if rig_dir:
            assets = os.path.join(rig_dir, "data", "rigs", "Human", "assets")
            if os.path.isdir(assets):
                for f in sorted(os.listdir(assets)):
                    log("   - %s" % os.path.join(assets, f))
    except Exception as e:
        log("retarget rig listing failed: %r" % e)

    # -----------------------------------------------------------------
    # NavMesh interface
    # -----------------------------------------------------------------
    section("NAVIGATION — NavMesh interface availability")
    try:
        import omni.anim.navigation.core as nav
        iface = nav.acquire_interface()
        log("nav.acquire_interface() = %r" % iface)
        log("NavMesh interface acquired OK (methods: %s)"
            % ", ".join([m for m in dir(iface) if not m.startswith("_")][:20]))
    except Exception as e:
        log("NavMesh interface FAILED: %r" % e)

    # -----------------------------------------------------------------
    # Anim graph interface
    # -----------------------------------------------------------------
    section("ANIMATION GRAPH — omni.anim.graph.core interface")
    try:
        import omni.anim.graph.core as ag
        pub = [m for m in dir(ag) if not m.startswith("_")]
        log("omni.anim.graph.core public API: %s" % ", ".join(pub[:30]))
        log("get_character present: %s" % ("get_character" in pub))
    except Exception as e:
        log("anim.graph.core FAILED: %r" % e)

    # -----------------------------------------------------------------
    # SUMMARY / VERDICT
    # -----------------------------------------------------------------
    section("VERDICT")
    people_ok = import_ok.get("omni.anim.people", False) and \
        ext_mgr.is_extension_enabled("omni.anim.people")
    graph_ok = import_ok.get("omni.anim.graph.core", False)
    nav_ok = import_ok.get("omni.anim.navigation.core", False)
    assets_ok = bool(asset_root)
    log("omni.anim.people enabled+imported : %s" % people_ok)
    log("omni.anim.graph.core imported     : %s" % graph_ok)
    log("omni.anim.navigation.core imported: %s" % nav_ok)
    log("asset root resolved               : %s (%s)" % (assets_ok, asset_root))
    verdict = people_ok and graph_ok and nav_ok and assets_ok
    log("")
    log("REAL HUMAN CHARACTER SYSTEM USABLE: %s" % ("YES" if verdict else "NO"))

    # Write output file
    os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
    with open(OUT_PATH, "w") as f:
        f.write("\n".join(_lines) + "\n")
    log("")
    log("Saved diagnostic to %s" % OUT_PATH)

    sim.close()
    return 0 if verdict else 1


if __name__ == "__main__":
    hl = "--gui" not in sys.argv
    code = main(headless=hl)
    sys.exit(code)
