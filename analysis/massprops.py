"""Volumes and centroids for the simulator (writes analysis/massprops.json and frame.stl).

Run: cad run analysis/massprops.py
"""
import json
import sys
from pathlib import Path

import build123d as bd

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "src"))
from rov import frame, pod_locations, thruster_pod  # noqa: E402

out = {}
f = frame().shape
c = f.center()
out["frame"] = {"vol_cm3": f.volume / 1000, "c": [c.X, c.Y, c.Z]}
bd.export_stl(f, str(HERE / "frame.stl"), tolerance=0.2)
pod = thruster_pod().shape
for name, loc in pod_locations().items():
    for ch in pod.children:
        s = loc * ch
        cc = s.center()
        out[f"{name}/{ch.label}"] = {"vol_cm3": s.volume / 1000, "c": [cc.X, cc.Y, cc.Z]}
(HERE / "massprops.json").write_text(json.dumps(out, indent=1))
print(json.dumps({k: round(v["vol_cm3"], 1) for k, v in out.items()}))
