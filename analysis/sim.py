# /// script
# dependencies = ["numpy", "trimesh", "scipy", "networkx", "rtree"]
# ///
"""6-DOF simulation of the Aqua Bots ROV (Onshape frame + pool noodle), from the part meshes in design/parts.

Axes: x forward (the end without the tether ring, STL -Y), y left (STL +X), z up.

Run:  cad run analysis/sim.py                      (PETG, parts flooded, 320 mm noodle, trims from the software)
      cad run analysis/sim.py --pla --sealed --noodle-len=200 --noodle-hole=20 --no-trim

  --pla            PLA instead of PETG
  --sealed         air stays trapped in the 15 % gyroid infill (default: parts flooded through drain holes)
  --noodle-len=MM  pool noodle length (default 320)
  --noodle-hole=MM hole down the middle of the noodle (default 0: solid, as modelled)
  --no-trim        motors exactly as wired (v13 original settings)
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import sim_core as sim  # noqa: E402
from sim_core import ASSUME, RHO, Vehicle, simulate  # noqa: E402

PARTS = HERE.parent / "design" / "parts"
ARG = {a.split("=")[0]: (a.split("=") + [""])[1] for a in sys.argv[1:]}
PLA = "--pla" in ARG
SEALED = "--sealed" in ARG
NOODLE_L = float(ARG.get("--noodle-len") or 320)
NOODLE_HOLE = float(ARG.get("--noodle-hole") or 0)
TRIM = "--no-trim" not in ARG
DENSITY = 1.24 if PLA else 1.27
# Bambu Studio, 15 % gyroid, 3 walls, 4 top/bottom layers (print/process_gyroid.json), model only
WEIGHT_G = {"frame": 216.36, "pod": 100.86, "skid": 169.08} if PLA else {"frame": 213.06, "pod": 99.33, "skid": 166.51}
SKID_PLASTIC_CM3 = 380.0      # the skid as modelled (100 % infill weighed 483 g PETG in the slicer)
NOODLE_DENSITY = 0.030        # pool noodle foam, g/cm3
# SETUP > EXTRAS > FRAME > NEW FRAME in the ROV software (rov.py DEFAULT_DESIGNS "new")
DESIGN = {"pitch_mix": 0.25, "heave_front": 0.75, "heave_rear": 1.0, "surge_pitch": 0.3}
if not TRIM:
    DESIGN = {"pitch_mix": 1.0, "heave_front": 1.0, "heave_rear": 1.0, "surge_pitch": 0.0}


def mix(cmd):
    """rov.py mixing with the design trim."""
    s, yw, hv, pt = (cmd.get(k, 0.0) for k in ("surge", "yaw", "heave", "pitch"))
    p = pt * DESIGN["pitch_mix"] + s * DESIGN["surge_pitch"]
    out = {"left": s - yw, "right": s + yw, "front": hv * DESIGN["heave_front"] + p, "rear": hv * DESIGN["heave_rear"] - p}
    k = max(1.0, *(abs(v) for v in out.values()))
    return {r: v / k for r, v in out.items()}


sim.mix = mix


def tf(v):
    v = np.asarray(v, float)
    out = np.empty_like(v)
    out[..., 0], out[..., 1], out[..., 2] = -v[..., 1], v[..., 0], v[..., 2]
    return out


def load(name):
    return trimesh.load(PARTS / f"{name}.stl", force="mesh")


def vol_c(p):
    """Outer volume (cm3) and centre; the Onshape export leaves some skid pieces open, so close them or use the hull."""
    if not p.is_watertight:
        q = p.copy(); q.fill_holes()
        if q.is_watertight:
            p = q
    if p.is_watertight:
        return abs(p.volume) / 1000, tf(p.center_mass)
    h = p.convex_hull
    return h.volume / 1000, tf(h.center_mass)


def printed(kind, outer_cm3, share=1.0):
    """(kg, displaced cm3) of a 15 % gyroid print: sealed = it displaces its outer volume; flooded = only its plastic."""
    g = WEIGHT_G[kind] * share
    return g / 1000, (outer_cm3 if SEALED else g / DENSITY)


def vox_drag(V, mesh, cds, depth_c, pitch=3.0):
    pts = tf(mesh.voxelized(pitch).points)
    for axis in range(3):
        o = [k for k in range(3) if k != axis]
        uniq = np.unique(np.round(pts[:, o] / pitch).astype(int), axis=0) * pitch
        P = np.zeros((len(uniq), 3)); P[:, o[0]] = uniq[:, 0]; P[:, o[1]] = uniq[:, 1]; P[:, axis] = depth_c[axis]
        V.drag.append((axis, P, pitch * pitch, cds[axis]))


def build():
    V = Vehicle("aqua bot (Onshape frame)")
    frame = load("frame")
    fv, fc = vol_c(frame)
    V.add("frame", *printed("frame", fv), fc)
    pods = {r: load("pod_" + r) for r in ("left", "right", "front", "rear")}
    for role, p in pods.items():
        v, c = vol_c(p)
        V.add(role + "/holster", *printed("pod", v), c)
        V.add(role + "/motor", ASSUME["pump_motor_kg"], 184.1, c)        # SEAFLO 1100 motor, strainer cut off
        V.add(role + "/prop", ASSUME["prop_kg"], ASSUME["prop_cm3"], c)
        V.entrained.append((0.15, c))
    noodle = load("noodle")
    nv, nc = vol_c(noodle)
    nv = nv * NOODLE_L / 200 * (1 - (NOODLE_HOLE / 56) ** 2)            # modelled 56 x 200 mm, solid
    V.add("pool noodle", nv * NOODLE_DENSITY / 1000, nv, nc)
    skid = [p for p in load("skid").split(only_watertight=False) if p.extents.max() > 15]   # drop export slivers
    vs = [vol_c(p) for p in skid]
    k = SKID_PLASTIC_CM3 / sum(v for v, _ in vs)
    for (v, c) in vs:
        V.add("skid", *printed("skid", v * k, v * k / SKID_PLASTIC_CM3), c)
    front_x = tf(frame.bounds)[:, 0].max()
    V.add("camera", ASSUME["camera_kg"], ASSUME["camera_cm3"], (front_x - 20, 0, 0))
    V.add("zip ties / screws", ASSUME["fasteners_kg"], ASSUME["fasteners_cm3"], (0, 0, 0))
    V.add("wiring + splices", ASSUME["wiring_kg"], ASSUME["wiring_cm3"], (0, 0, -20))
    allp = tf(np.concatenate([frame.vertices, noodle.vertices, load("skid").vertices] + [p.vertices for p in pods.values()]))
    V.V_outer = np.prod(allp.max(0) - allp.min(0)) * 1e-9 * 0.45
    vox_drag(V, frame, (1.2, 1.2, 1.2), (0, 0, 0))
    for role, p in pods.items():
        vox_drag(V, p, (0.8, 1.0, 1.0) if role in ("left", "right") else (1.0, 1.0, 0.8), tf(p.bounds.mean(0)))
    vox_drag(V, noodle, (1.0, 0.9, 0.9), nc)
    for p in skid:
        vox_drag(V, p, (1.0, 1.0, 1.0), tf(p.bounds.mean(0)))
    c = {r: tf(p.bounds.mean(0)) for r, p in pods.items()}
    V.thrusters = [(c["left"], (1, 0, 0), "left"), (c["right"], (1, 0, 0), "right"),
                   (c["front"], (0, 0, 1), "front"), (c["rear"], (0, 0, 1), "rear")]
    sim.setup(V)
    return V


def main():
    V = build()
    net = (V.V * RHO - V.m) * 1000
    print(f"=== {'PLA' if PLA else 'PETG'}, 15 % gyroid parts {'SEALED (air trapped)' if SEALED else 'FLOODED (drain holes)'}, "
          f"noodle 56 x {NOODLE_L:.0f} mm{f' with {NOODLE_HOLE:.0f} mm hole' if NOODLE_HOLE else ''}, "
          f"{'trim: new frame' if TRIM else 'no trim (v13 settings)'} ===")
    print(f"mass in air {V.m*1000:.0f} g; net {net:+.0f} g ({'floats up' if net > 0 else 'SINKS'})")
    print(f"centre of buoyancy {V.cb[2]-V.cg[2]:.1f} mm above the centre of gravity, {V.cb[0]-V.cg[0]:+.1f} mm ahead")
    per_mm = math.pi / 4 * (56 ** 2 - NOODLE_HOLE ** 2) / 1000 * (1 - NOODLE_DENSITY)
    print(f"noodle: each 1 mm = {per_mm:.2f} g of lift; for +15 g change the length by {(15 - net) / per_mm:+.0f} mm")
    r = simulate(V, lambda t: {}, 30)[-1]
    print(f"at rest: roll {r[10]:+.1f} deg, pitch {r[11]:+.1f} deg, drifting {r[6]*100:+.1f} cm/s (+ = up)")
    res = {}
    f = simulate(V, lambda t: {"surge": 1}, 25)[-1]; res.update(fwd=f[4], fwd_pitch=f[11])
    b = simulate(V, lambda t: {"surge": -1}, 25)[-1]; res.update(rev=-b[4], rev_pitch=b[11])
    u = simulate(V, lambda t: {"heave": 1}, 25)[-1]; res.update(up=u[6], up_pitch=u[11])
    d = simulate(V, lambda t: {"heave": -1}, 25)[-1]; res.update(down=-d[6], down_pitch=d[11])
    y = simulate(V, lambda t: {"yaw": 1}, 25)[-1]; res.update(spin=abs(y[9]))
    t = simulate(V, lambda t: {"surge": 1, "yaw": 0.5}, 25)[-1]; res.update(turn_radius=abs(t[4] / math.radians(t[9])))
    print("forward {fwd:.2f} m/s (tilt {fwd_pitch:+.1f} deg) | reverse {rev:.2f} m/s ({rev_pitch:+.1f}) | up {up:.2f} m/s "
          "({up_pitch:+.1f}) | down {down:.2f} m/s ({down_pitch:+.1f}) | spin {spin:.0f} deg/s | turn radius {turn_radius:.2f} m"
          .format(**res))
    for p in (0.4, 1.0):
        lg = simulate(V, lambda t: {"pitch": p}, 15)
        late = [row[11] for row in lg if row[0] > 10]
        flips = max(abs(row[10]) for row in lg) > 120 or max(abs(x) for x in late) > 85
        print(f"tilt stick {p:.0%}: " + ("FLIPS OVER" if flips else f"holds {abs(np.mean(late)):.0f} deg"))
    for name, ang, ax in (("on its side", 90, 1), ("upside down", 178, 1), ("nose down 90 deg", 90, 2)):
        a = math.radians(ang) / 2
        q0 = (math.cos(a), math.sin(a), 0, 0) if ax == 1 else (math.cos(a), 0, math.sin(a), 0)
        lg = simulate(V, lambda t: {}, 20, q0=q0)
        tu = next((row[0] for row in lg if abs(row[10]) < 10 and abs(row[11]) < 10), None)
        print(f"released {name}: " + (f"upright after {tu:.1f} s" if tu else "DOES NOT RIGHT ITSELF (20 s)"))
    return {"net_g": net, "mass_g": V.m * 1000, "BG_mm": float(V.cb[2] - V.cg[2]), **{k: float(v) for k, v in res.items()}}


if __name__ == "__main__":
    out = main()
    if "--json" in ARG:
        print(json.dumps(out))
