# /// script
# dependencies = ["numpy"]
# ///
"""6-DOF underwater simulation of the Aqua Bots v14 hull (and the old v13 frame for comparison).

Geometry comes from the CAD model (massprops.json + part meshes). Everything that is an assumption is in
ASSUME below: change it when you have measured the real thing (weigh a pod in air and in water, and pull
one thruster against a luggage scale).

Run:  cad run analysis/sim.py
"""
import json
import math
import struct
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
RHO = 1000.0          # pool water, kg/m3
G = 9.81

ASSUME = {
    "pump_motor_kg": 0.300,      # SEAFLO 1100 is ~350 g stock; strainer base cut off
    "casing_cm3": 92.3,          # measured from motor caseing v1.stl
    "casing_density": 1.24,      # PLA
    "print_density": 1.27,       # PETG for the new hull
    "foam_density": 0.035,       # XPS / closed-cell foam
    "prop_kg": 0.006, "prop_cm3": 5.0,
    "camera_kg": 0.035, "camera_cm3": 12.0,
    "fasteners_kg": 0.025, "fasteners_cm3": 3.2,
    "wiring_kg": 0.060, "wiring_cm3": 25.0,
    "thrust_fwd_N": 5.0,         # per thruster, 12 V, 55 mm prop in duct (typical 1100 GPH conversions: 4-7 N)
    "thrust_rev_N": 3.5,
    "motor_tau_s": 0.15,
    "pitch_gain": 0.4,           # config.toml: front motor pitch = 0.4, rear motor pitch = -0.4 (was +-1.0)
    "target_net_g": 15.0,        # slightly positive: it floats up gently if anything fails
    "ballast_density": 7.85,     # steel nuts / washers (lead = 11.3)
    "ballast_packing": 0.55,     # nuts and washers fill about half of a bay
}


# ------------------------------------------------------------------ meshes
def load_stl(p):
    b = Path(p).read_bytes()
    if b[:5] == b"solid" and b"facet" in b[:400]:
        v = [list(map(float, ln.split()[1:])) for ln in b.decode(errors="ignore").splitlines()
             if ln.strip().startswith("vertex")]
        return np.array(v).reshape(-1, 3, 3)
    n = struct.unpack("<I", b[80:84])[0]
    dt = np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")])
    return np.frombuffer(b[84:84 + n * 50], dtype=dt)["v"].reshape(-1, 3, 3).astype(float)


def rot_y90(t):            # build123d Rot(0, 90, 0): x' = z, z' = -x
    out = t.copy(); out[..., 0] = t[..., 2]; out[..., 2] = -t[..., 0]; return out


def rot_x180(t):
    out = t.copy(); out[..., 1] = -t[..., 1]; out[..., 2] = -t[..., 2]; return out


def silhouette(tris, axis, pix=3.0):
    """Pixel centres (mm, in-plane coords) of the mesh's projection along axis."""
    a, b = [i for i in range(3) if i != axis]
    P = tris[..., [a, b]]
    lo = P.reshape(-1, 2).min(0) - pix; hi = P.reshape(-1, 2).max(0) + pix
    nx, ny = (np.ceil((hi - lo) / pix)).astype(int)
    grid = np.zeros((nx, ny), bool)
    gx = lo[0] + (np.arange(nx) + 0.5) * pix
    gy = lo[1] + (np.arange(ny) + 0.5) * pix
    for tri in P:
        x0, y0 = tri.min(0); x1, y1 = tri.max(0)
        i0, i1 = int((x0 - lo[0]) / pix), int((x1 - lo[0]) / pix) + 1
        j0, j1 = int((y0 - lo[1]) / pix), int((y1 - lo[1]) / pix) + 1
        if i1 <= i0 or j1 <= j0:
            continue
        X, Y = np.meshgrid(gx[i0:i1], gy[j0:j1], indexing="ij")
        (ax, ay), (bx, by), (cx, cy) = tri
        d = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
        if abs(d) < 1e-12:
            continue
        l1 = ((by - cy) * (X - cx) + (cx - bx) * (Y - cy)) / d
        l2 = ((cy - ay) * (X - cx) + (ax - cx) * (Y - cy)) / d
        inside = (l1 >= -1e-9) & (l2 >= -1e-9) & (l1 + l2 <= 1 + 1e-9)
        grid[i0:i1, j0:j1] |= inside
    I, J = np.nonzero(grid)
    return np.stack([gx[I], gy[J]], 1), pix * pix


# ------------------------------------------------------------------ vehicle
class Vehicle:
    def __init__(self, name):
        self.name = name
        self.items = []        # (label, mass kg, displaced m3, centre mm)
        self.drag = []         # per axis list of (pts mm (N,3), area mm2, Cd)
        self.thrusters = []    # (pos mm, dir unit, role)
        self.entrained = []    # (mass kg, centre mm): water carried inside the flooded hull

    def add(self, label, kg, cm3, c):
        self.items.append((label, kg, cm3 * 1e-6, np.array(c, float)))

    def add_drag(self, tris, cds, depth_c):
        for axis in range(3):
            if cds[axis] <= 0:
                continue
            pts2, a = silhouette(tris, axis)
            others = [i for i in range(3) if i != axis]
            P = np.zeros((len(pts2), 3)); P[:, others[0]] = pts2[:, 0]; P[:, others[1]] = pts2[:, 1]
            P[:, axis] = depth_c[axis]
            self.drag.append((axis, P, a, cds[axis]))

    # ---- statics
    def statics(self):
        m = sum(i[1] for i in self.items); V = sum(i[2] for i in self.items)
        cg = sum(i[1] * i[3] for i in self.items) / m
        cb = sum(i[2] * i[3] for i in self.items) / V
        return m, V, cg, cb

    def finish(self):
        m, V, cg, cb = self.statics()
        self.m, self.V, self.cg, self.cb = m, V, cg, cb
        # inertia about CG (point masses + entrained water), kg m2
        I = np.zeros((3, 3)); mtot = m
        pts = [(i[1], i[3]) for i in self.items] + list(self.entrained)
        for mk, c in pts:
            r = (np.asarray(c) - cg) * 1e-3
            I += mk * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
        I += np.diag([0.002, 0.002, 0.002])          # parts' own inertia (shells, motors), small floor
        me = sum(e[0] for e in self.entrained)
        self.I = I
        self.m_inertial = m + me
        # added mass (rough: box of the outer envelope)
        Vout = self.V_outer
        self.MA = np.array([0.15, 0.9, 1.2]) * RHO * Vout
        self.IA = np.diag(np.diag(I)) * 0.5
        for k, (axis, P, a, cd) in enumerate(self.drag):
            self.drag[k] = (axis, (P - cg) * 1e-3, a * 1e-6, cd)
        self.thr = [((np.asarray(p) - cg) * 1e-3, np.asarray(d, float), role) for p, d, role in self.thrusters]


def quat_mul(q, r):
    w0, x0, y0, z0 = q; w1, x1, y1, z1 = r
    return np.array([w0*w1 - x0*x1 - y0*y1 - z0*z1, w0*x1 + x0*w1 + y0*z1 - z0*y1,
                     w0*y1 - x0*z1 + y0*w1 + z0*x1, w0*z1 + x0*y1 - y0*x1 + z0*w1])


def quat_to_R(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def euler(q):
    R = quat_to_R(q)
    roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))
    pitch = math.degrees(-math.asin(max(-1, min(1, R[2, 0]))))
    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    return roll, pitch, yaw


def mix(cmd):
    """rov.py default mixing: left/right = surge +- yaw, front/rear up = heave +- pitch."""
    s, yw, hv, pt = cmd.get("surge", 0), cmd.get("yaw", 0), cmd.get("heave", 0), cmd.get("pitch", 0)
    pt = pt * ASSUME["pitch_gain"]
    out = {"left": s - yw, "right": s + yw, "front": hv + pt, "rear": hv - pt}
    k = max(1.0, *(abs(v) for v in out.values()))
    return {r: v / k for r, v in out.items()}


def simulate(veh, cmd_fn, T, q0=(1, 0, 0, 0), dt=0.004, ballast_kg=0.0):
    v = np.zeros(3); w = np.zeros(3); q = np.array(q0, float); q /= np.linalg.norm(q)
    pos = np.zeros(3); thrust = {r: 0.0 for *_, r in veh.thr}
    Mlin = veh.m_inertial + veh.MA
    Irot = veh.I + veh.IA; Iinv = np.linalg.inv(Irot)
    rB = (veh.cb - veh.cg) * 1e-3
    W = veh.m * G; Bf = veh.V * RHO * G
    log = []
    steps = int(T / dt)
    for n in range(steps):
        t = n * dt
        R = quat_to_R(q)
        g_body = R.T @ np.array([0, 0, -1.0])
        F = W * g_body + Bf * (-g_body)
        M = np.cross(rB, Bf * (-g_body))
        cmd = mix(cmd_fn(t))
        for p, d, role in veh.thr:
            u = cmd[role]
            target = (veh.F_fwd if u >= 0 else veh.F_rev) * u
            thrust[role] += (target - thrust[role]) * dt / veh.tau
            f = thrust[role] * d
            F = F + f; M = M + np.cross(p, f)
        for axis, P, a, cd in veh.drag:
            u = v[axis] + (np.cross(w, P))[:, axis]
            f = -0.5 * RHO * cd * a * u * np.abs(u)
            F[axis] += f.sum()
            fv = np.zeros((len(P), 3)); fv[:, axis] = f
            M = M + np.cross(P, fv).sum(0)
        # rigid body (body frame, origin at CG)
        vdot = (F - np.cross(w, veh.m_inertial * v)) / Mlin
        wdot = Iinv @ (M - np.cross(w, Irot @ w))
        v = v + vdot * dt; w = w + wdot * dt
        q = q + 0.5 * dt * quat_mul(q, np.array([0, *w])); q /= np.linalg.norm(q)
        pos = pos + (quat_to_R(q) @ v) * dt
        if n % int(0.05 / dt) == 0:
            log.append((t, *pos, *v, *np.degrees(w), *euler(q)))
    return np.array(log)


# ------------------------------------------------------------------ build the two vehicles
def pod_mesh():
    t = load_stl(HERE / "pod.stl")
    x = t[..., 0]; r = np.hypot(t[..., 1], t[..., 2])
    prop = np.all((x > -14.2) & (x < -7.8) & (r < 27.7), axis=1)     # the prop disc spins: not a drag surface
    return t[~prop]


def new_vehicle():
    mp = json.loads((HERE / "massprops.json").read_text())
    ex = mp.pop("_extra")
    V = Vehicle("v14 (new)")
    pd = ASSUME["print_density"]
    for k in ("hull_tub", "hull_cap", "nacelle_left", "nacelle_right"):
        V.add(k, mp[k]["vol_cm3"] * pd / 1000, mp[k]["vol_cm3"], mp[k]["c"])
    for k in ("foam", "foam_cap"):
        V.add(k, mp[k]["vol_cm3"] * ASSUME["foam_density"] / 1000, mp[k]["vol_cm3"], mp[k]["c"])
    for p in ("pod_front", "pod_rear", "pod_left", "pod_right"):
        V.add(p + "/casing", ASSUME["casing_cm3"] * ASSUME["casing_density"] / 1000, ASSUME["casing_cm3"], mp[p + "/casing"]["c"])
        V.add(p + "/motor", ASSUME["pump_motor_kg"], mp[p + "/pump_motor"]["vol_cm3"], mp[p + "/pump_motor"]["c"])
        V.add(p + "/prop", ASSUME["prop_kg"], ASSUME["prop_cm3"], mp[p + "/prop"]["c"])
    V.add("camera", ASSUME["camera_kg"], ASSUME["camera_cm3"], (112, 0, 20))
    V.add("fasteners", ASSUME["fasteners_kg"], ASSUME["fasteners_cm3"], (0, 0, 0))
    V.add("wiring", ASSUME["wiring_kg"], ASSUME["wiring_cm3"], (0, 0, -40))
    # water carried along inside the flooded hull and nacelles
    solids_in_hull = sum(i[2] for i in V.items if not i[0].startswith(("nacelle", "pod_left", "pod_right"))) * 1e6
    V.entrained.append(((ex["env_hull_cm3"] - solids_in_hull) * 1e-3, (0, 0, 0)))
    for sy in (1, -1):
        V.entrained.append(((ex["nac_bore_cm3"] - 92.3 - 184) * 1e-3, (0, sy * 84.6, 0)))
    V.V_outer = (ex["env_hull_cm3"] + 2 * ex["nac_outer_cm3"]) * 1e-6
    # drag surfaces
    hull = np.concatenate([load_stl(HERE / "hull_tub.stl"), load_stl(HERE / "hull_cap.stl")])
    V.add_drag(hull, (0.30, 1.0, 1.1), (0, 0, 0))
    nac = load_stl(HERE / "nacelle.stl")
    V.add_drag(nac, (0.15, 1.0, 1.0), (0, 82.6, 0))
    V.add_drag(rot_x180(nac), (0.15, 1.0, 1.0), (0, -82.6, 0))
    pod = pod_mesh()
    for sy in (1, -1):
        t = pod.copy(); t[..., 0] *= -1; t[..., 1] *= -1; t[..., 0] += 57.5 - 25; t[..., 1] += sy * 84.6
        V.add_drag(t, (0.8, 0, 0), (0, sy * 84.6, 0))          # side pods: inside nacelles, flow through the duct
    for sx in (1, -1):
        t = rot_y90(pod); t[..., 0] += sx * 56; t[..., 2] += -65 + 115 - 25
        V.add_drag(t, (0, 0, 0.8), (sx * 56, 0, 0))             # vertical pods: inside the tunnels
    V.thrusters = [((0, 84.6, 0), (1, 0, 0), "left"), ((0, -84.6, 0), (1, 0, 0), "right"),
                   ((56, 0, 0), (0, 0, 1), "front"), ((-56, 0, 0), (0, 0, 1), "rear")]
    return V


def old_vehicle(new):
    """v13 frame (aqua bots v13(1).stl), forward = its +Y; turned so forward = +X here."""
    f = load_stl(HERE.parent / "ref" / "aqua_bots_v13.stl")
    t = f.copy(); t[..., 0] = f[..., 1]; t[..., 1] = -f[..., 0]
    V = Vehicle("v13 (old)")
    vol = 273.9
    V.add("frame", vol * 1.24 / 1000 * 0.6, vol * 0.6, (0, 0, 0))   # printed ~60 % solid
    pod = pod_mesh()
    for name, sy in (("left", -91.6), ("right", 91.6)):
        p = pod.copy(); p[..., 0] += -32.5; p[..., 1] += sy
        V.add_drag(p, (0.8, 1.0, 1.0), (0, sy, 0))
    for sx in (100, -100):
        p = rot_y90(pod); p[..., 0] += sx; p[..., 2] += 2.4 + 32.5
        V.add_drag(p, (1.0, 1.0, 0.8), (sx, 0, 2.4))
    for sy in (91.6, -91.6):
        V.add(f"pod{sy}", ASSUME["pump_motor_kg"] + 0.114 + 0.006, 92.3 + 184 + 5, (0, sy, 0))
    for sx in (100, -100):
        V.add(f"podv{sx}", ASSUME["pump_motor_kg"] + 0.114 + 0.006, 92.3 + 184 + 5, (sx, 0, -15))
    V.add_drag(t, (1.2, 1.2, 1.2), (0, 0, 0))
    # assume the team floated it neutral with foam on top (unknown); only its steady speeds are compared
    m, Vd, cg, cb = V.statics()
    V.add("float (assumed)", 0.0, (m - Vd * RHO) / RHO * 1e6 + 25, (0, 0, 40))
    V.V_outer = 0.248 * 0.269 * 0.069
    V.thrusters = [((0, 91.6, 0), (1, 0, 0), "left"), ((0, -91.6, 0), (1, 0, 0), "right"),
                   ((100, 0, 2.4), (0, 0, 1), "front"), ((-100, 0, 2.4), (0, 0, 1), "rear")]
    return V


def setup(V):
    V.F_fwd = ASSUME["thrust_fwd_N"]; V.F_rev = ASSUME["thrust_rev_N"]; V.tau = ASSUME["motor_tau_s"]
    V.finish()


def solve_ballast(V, bays):
    """Steel in the three floor bays so that it is slightly buoyant and sits level.
    bays: [(x, z, capacity_kg)] front, middle, rear. Returns (need_kg, (score, front, middle, rear))."""
    m, Vd, cg, cb = V.statics()
    net = Vd * RHO - m
    k = 1 - RHO / (ASSUME["ballast_density"] * 1000)
    need = (net - ASSUME["target_net_g"] / 1000) / k
    mom = (sum(i[2] * RHO * i[3][0] for i in V.items) - sum(i[1] * i[3][0] for i in V.items))
    (xf, _, cf), (xm, _, cm), (xr, _, cr) = bays
    best = None
    for frac in np.linspace(0, 1, 401):
        bm = need * frac; rest = need - bm
        bf, br = np.linalg.solve(np.array([[1, 1], [xf, xr]]), np.array([rest, mom / k - bm * xm]))
        if min(bf, br, bm) < -1e-9:
            continue
        over = max(bf - cf, bm - cm, br - cr)
        score = over if over > 0 else -bm      # prefer the middle bay (keeps the ends light: less pitch inertia)
        if best is None or score < best[0]:
            best = (score, bf, bm, br)
    return need, best


def main():
    if "--pla" in sys.argv:
        ASSUME["print_density"] = 1.24
    Vn = new_vehicle()
    mp = json.loads((HERE / "massprops.json").read_text())
    pk = ASSUME["ballast_density"] * ASSUME["ballast_packing"] / 1000
    bays = [(mp[n]["c"][0], mp[n]["c"][2], mp[n]["vol_cm3"] * pk) for n in ("bay_front", "bay_middle", "bay_rear")]
    need, best = solve_ballast(Vn, bays)
    m, Vd, cg, cb = Vn.statics()
    print("=== v14 hydrostatics (before ballast) ===")
    print(f"mass {m*1000:.0f} g, displacement {Vd*1e6:.0f} cm3 -> lift {(Vd*RHO-m)*1000:+.0f} g")
    if best is None or best[0] > 0:
        print("!! cannot trim with the bays:", need, best); sys.exit(1)
    _, bf, bm, br = best
    print(f"ballast (steel): front {bf*1000:.0f} g, middle {bm*1000:.0f} g, rear {br*1000:.0f} g  "
          f"(bay capacity {bays[0][2]*1000:.0f}/{bays[1][2]*1000:.0f}/{bays[2][2]*1000:.0f} g)")
    for (x, z, _), b, nm in zip(bays, (bf, bm, br), ("front", "middle", "rear")):
        if b > 0:
            Vn.add("ballast_" + nm, b, b / ASSUME["ballast_density"] * 1000, (x, 0, z))
    setup(Vn)
    BG = (Vn.cb - Vn.cg)
    print(f"mass {Vn.m*1000:.0f} g in air, net {(Vn.V*RHO - Vn.m)*1000:+.0f} g (positive = floats)")
    print(f"CG ({Vn.cg[0]:.1f}, {Vn.cg[1]:.1f}, {Vn.cg[2]:.1f}) mm   CB ({Vn.cb[0]:.1f}, {Vn.cb[1]:.1f}, {Vn.cb[2]:.1f}) mm")
    print(f"CB above CG by {BG[2]:.1f} mm; righting moment {Vn.V*RHO*G*BG[2]*1e-3*1000:.0f} mN*m per radian")
    rest = simulate(Vn, lambda t: {}, 30)
    r = rest[-1]
    print(f"at rest after 30 s: roll {r[10]:.1f} deg, pitch {r[11]:.1f} deg, rising {r[6]*100:.1f} cm/s")

    Vo = old_vehicle(Vn); setup(Vo)

    def steady(V, cmd, T=25, col=None):
        lg = simulate(V, lambda t: cmd, T)
        return lg

    results = {}
    for V in (Vn, Vo):
        res = {}
        lg = steady(V, {"surge": 1}); res["fwd_speed"] = lg[-1][4]; res["fwd_pitch"] = lg[-1][11]
        t90 = next((row[0] for row in lg if row[4] >= 0.9 * lg[-1][4]), None); res["fwd_t90"] = t90
        lg = steady(V, {"surge": -1}); res["rev_speed"] = -lg[-1][4]
        lg = steady(V, {"yaw": 1}); res["yaw_rate"] = abs(lg[-1][9])
        lg = steady(V, {"heave": 1}); res["up_speed"] = lg[-1][6]; res["up_pitch"] = lg[-1][11]
        lg = steady(V, {"heave": -1}); res["down_speed"] = -lg[-1][6]
        lg = steady(V, {"surge": 1, "yaw": 0.5}); res["turn_radius"] = abs(lg[-1][4] / math.radians(lg[-1][9])) if abs(lg[-1][9]) > 0.1 else float("inf")
        results[V.name] = res

    print("\n=== steady performance (full stick) ===")
    keys = [("fwd_speed", "forward speed m/s", "{:.2f}"), ("fwd_t90", "time to 90% speed s", "{:.1f}"),
            ("rev_speed", "reverse speed m/s", "{:.2f}"), ("fwd_pitch", "pitch at full forward deg", "{:+.1f}"),
            ("yaw_rate", "spin rate deg/s", "{:.0f}"), ("turn_radius", "turn radius (fwd + half yaw) m", "{:.2f}"),
            ("up_speed", "up speed m/s", "{:.2f}"), ("down_speed", "down speed m/s", "{:.2f}"),
            ("up_pitch", "pitch while rising deg", "{:+.1f}")]
    print(f"{'':32s}{'v14 new':>10s}{'v13 old':>10s}")
    for k, lab, fmt in keys:
        a = results[Vn.name][k]; b = results[Vo.name][k]
        print(f"{lab:32s}{fmt.format(a):>10s}{fmt.format(b):>10s}")

    print("\n=== v14 control & stability tests ===")
    for p in (0.4, 0.6, 1.0):
        lg = simulate(Vn, lambda t: {"pitch": p}, 15)
        late = [row[11] for row in lg if row[0] > 10]
        flips = max(abs(row[10]) for row in lg) > 120 or max(abs(x) for x in late) > 85
        print(f"tilt stick at {p:.0%} (pitch mix {ASSUME['pitch_gain']}): " + ("FLIPS OVER (loops)" if flips else f"holds {abs(np.mean(late)):.0f} deg nose {'up' if np.mean(late) > 0 else 'down'}"))
    lg = simulate(Vn, lambda t: {}, 12, q0=(math.cos(math.radians(45)), math.sin(math.radians(45)), 0, 0))
    t10 = next((row[0] for row in lg if abs(row[10]) < 10), None); print(f"released on its side (90 deg roll): upright (<10 deg) after {t10:.1f} s")
    a = math.radians(178) / 2
    lg = simulate(Vn, lambda t: {}, 20, q0=(math.cos(a), math.sin(a), 0, 0))
    t10 = next((row[0] for row in lg if abs(row[10]) < 10), None)
    print(f"released upside down (178 deg): upright after {t10:.1f} s" if t10 else "released upside down: DID NOT RIGHT ITSELF")
    a = math.radians(30) / 2
    lg = simulate(Vn, lambda t: {}, 12, q0=(math.cos(a), 0, math.sin(a), 0))
    t5 = next((row[0] for row in lg if abs(row[11]) < 3), None)
    settle = next((row[0] for k, row in enumerate(lg) if all(abs(r2[11]) < 3 for r2 in lg[k:])), None)
    print(f"knocked 30 deg nose-down: settles within 3 deg (for good) after {settle:.1f} s" if settle else "knocked 30 deg: still rocking after 12 s")
    lg = simulate(Vn, lambda t: {"surge": 1} if t < 8 else {}, 30)
    stop = next((row[0] - 8 for row in lg if row[0] > 8 and row[4] < 0.2), None)
    print(f"coasts below 0.2 m/s {stop:.1f} s after letting go of full forward" if stop else "still coasting after 22 s")
    json.dump({"results": results, "ballast_g": [bf*1000, bm*1000, br*1000], "net_g": (Vn.V*RHO - Vn.m)*1000,
               "BG_mm": BG[2], "mass_g": Vn.m * 1000}, open(HERE / "sim_results.json", "w"), indent=1)


if __name__ == "__main__":
    main()
