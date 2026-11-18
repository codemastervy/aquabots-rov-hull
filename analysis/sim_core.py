# /// script
# dependencies = ["numpy"]
# ///
"""6-DOF underwater simulator (rigid body, buoyancy, drag on every part, added mass, thruster lag).
Used by sim.py. ASSUME holds the motor and thrust numbers: change them when you have measured the real thing."""
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


def setup(V):
    V.F_fwd = ASSUME["thrust_fwd_N"]; V.F_rev = ASSUME["thrust_rev_N"]; V.tau = ASSUME["motor_tau_s"]
    V.finish()
