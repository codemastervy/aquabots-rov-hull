"""Volumes and centroids of every part from the CAD model -> analysis/massprops.json, plus STL meshes for the sim."""
import json
import sys
from pathlib import Path

import build123d as bd
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "src"))
from rov import hull_tub, hull_cap, nacelle, foam, foam_cap, thruster_pod, placed_pods, ballast_bays  # noqa: E402
from params import NAC_Y  # noqa: E402

def props(shape):
    c = shape.center(bd.CenterOf.MASS)
    return {"vol_cm3": shape.volume / 1000, "c": [c.X, c.Y, c.Z]}

out = {}
out["hull_tub"] = props(hull_tub().shape)
out["hull_cap"] = props(hull_cap().shape)
nac = nacelle().shape
out["nacelle_left"] = props(nac)
out["nacelle_right"] = props(bd.Rot(180, 0, 0) * nac)
out["foam"] = props(foam().shape)
out["foam_cap"] = props(foam_cap().shape)
for b in ballast_bays().shape.children:
    out[b.label] = props(b)
for p in placed_pods():
    for leaf in p.leaves if hasattr(p, "leaves") else []:
        pass
from cad.build123d import leaves
for p in placed_pods():
    for lf in leaves(p):
        out[f"{p.label}/{lf.label}"] = props(lf)
pod = thruster_pod()
pod.export(HERE / "pod.stl")
(HERE / "massprops.json").write_text(json.dumps(out, indent=1))
for k, v in out.items():
    print(f"{k:28s} {v['vol_cm3']:8.1f} cm3  c=({v['c'][0]:7.1f},{v['c'][1]:7.1f},{v['c'][2]:7.1f})")

# meshes and envelope numbers for the simulation
from rov import envelope  # noqa: E402
from params import TUN_ID, POD_L, NAC_RO  # noqa: E402
hull_tub().export(HERE / "hull_tub.stl")
hull_cap().export(HERE / "hull_cap.stl")
nacelle().export(HERE / "nacelle.stl")
import math
extra = {"env_hull_cm3": envelope().volume / 1000,
         "nac_bore_cm3": math.pi * (TUN_ID / 2) ** 2 * POD_L / 1000,
         "nac_outer_cm3": math.pi * NAC_RO ** 2 * POD_L / 1000}
d = json.loads((HERE / "massprops.json").read_text()); d["_extra"] = extra
(HERE / "massprops.json").write_text(json.dumps(d, indent=1)); print(extra)
