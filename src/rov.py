"""Aqua Bots v17 "v13 shape" ROV: looks like our v13 frame - flat, solid, a ring at each of the four ends,
truss windows, a cross bar out to the side rings - but the middle is a box holding two pool-noodle pieces,
so it floats, sits level and rights itself (motors hang low, foam sits in the middle).

- up/down rings on the build plate, pods hang below (prop duct up); 2 zip-tie lugs per ring
- side rings at the cross bar ends (prop duct forward); a fin with 2 zip-tie holes on top
- noodles slide in from the top past 45 deg lips; a spine between them carries the camera mast,
  the splice rack and the tether post above the foam
- one print, no supports, zip ties only

Build: cad run src/rov.py
"""
import math

import build123d as bd
from cad import model

from params import (CAM_W, CAM_X, CAM_Z, CB, DECK_T, DUCT_L, DUCT_OD, LIP, NOODLE_D, NOODLE_HOLE, NOODLE_L, NY,
                    NZ, PLATE_T, POD_OD, POST_X, PROP_D, R_IN, R_OUT, RACK_TOP, RING_L, SLEEVE_ID, SPINE_T, SY, SZ,
                    BOX_TOP, TOWER_TOP, VX, WALL_IN, WALL_Y, XR, XS, Z_BOT, Z_TOP)

ORANGE = "#f28c28"
WHITE = "#e8e8e8"
RED = "#c0392b"
GREY = "#555555"
YELLOW = "#f4d03f"
S2 = math.sqrt(2.0)
RING_TOP = Z_BOT + RING_L
FILLET = 2.0


# ---------------------------------------------------------------- helpers
def box(x0, x1, y0, y1, z0, z1):
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1)); z0, z1 = sorted((z0, z1))
    return bd.Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * bd.Box(x1 - x0, y1 - y0, z1 - z0)


def z_cyl(r, z0, z1, x=0.0, y=0.0):
    return bd.Pos(x, y, z0) * bd.Cylinder(r, z1 - z0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))


def x_cyl(r, x0, x1, y=0.0, z=0.0):
    return bd.Pos(x0, y, z) * bd.Rot(0, 90, 0) * bd.Cylinder(r, x1 - x0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))


def x_diamond(r, x0, x1, y, z):
    return bd.Pos((x0 + x1) / 2, y, z) * bd.Rot(45, 0, 0) * bd.Box(x1 - x0, 2 * r, 2 * r)


def plate(plane, outline, windows, t, r=FILLET):
    s = bd.Polygon(*outline, align=None)
    for w in windows:
        wp = bd.Polygon(*w, align=None)
        s = s - (bd.fillet(wp.vertices(), r) if r else wp)
    return bd.extrude(plane * s, amount=t / 2, both=True)


def xz_plane(y):
    return bd.Plane(origin=(0, y, 0), x_dir=(1, 0, 0), z_dir=(0, -1, 0))


def yz_plane(x):
    return bd.Plane(origin=(x, 0, 0), x_dir=(0, 1, 0), z_dir=(1, 0, 0))


def mirror_u(windows):
    return [[(-u, v) for u, v in w] for w in windows]


def truss_windows(u0):
    def sh(pts):
        return [(u0 + u, v) for u, v in pts]
    return [sh([(15, 0), (30, -23), (45, 0), (30, 23)]),
            sh([(3, 29), (15, 29), (3, 11)]), sh([(45, 29), (33, 29), (45, 11)]),
            sh([(3, -29), (15, -29), (3, -11)]), sh([(45, -29), (33, -29), (45, -11)])]


def tangent_end(px, py):
    """Point where a line from (px, py) (y > 0) touches the front up/down ring, 1 mm inside its wall."""
    r = R_OUT - 1.0
    qx, qy = px - VX, -py
    d = math.hypot(qx, qy)
    a = math.atan2(qy, qx) + math.acos(r / d)
    return VX + r * math.cos(a), -(r * math.sin(a))


# ---------------------------------------------------------------- the centre box
def side_wall(sy):
    outline = [(-XS - 1, Z_BOT), (XS + 1, Z_BOT), (XS + 1, BOX_TOP), (-XS - 1, BOX_TOP)]
    half = truss_windows(13.0)
    return plate(xz_plane(sy * WALL_Y), outline, half + mirror_u(half), PLATE_T)


def angle_wall(sx, sy):
    """v13's diamond brace: from the box corner in to the up/down ring, tapering down to the ring's height."""
    tx, ty = tangent_end(XS, WALL_Y)
    dx, dy = tx - XS, ty - WALL_Y
    L = math.hypot(dx, dy)
    ux, uy = dx / L, dy / L
    uf = 0.35 * L
    outline = [(-2, Z_BOT), (L, Z_BOT), (L, RING_TOP), (uf, BOX_TOP), (-2, BOX_TOP)]
    wins = [[(5, -29), (uf - 3, -29), (5, 18)], [(uf + 4, -29), (L - 10, -29), (uf + 4, 14)]]
    b = plate(bd.Plane(origin=(XS, WALL_Y, 0), x_dir=(ux, uy, 0), z_dir=(uy, -ux, 0)), outline, wins, PLATE_T)
    if sy < 0:
        b = b.mirror(bd.Plane.XZ)
    if sx < 0:
        b = b.mirror(bd.Plane.YZ)
    return b


def box_outline():
    """Plan outline of the box (and its deck): straight sides, then angled in to the two up/down rings."""
    tx, ty = tangent_end(XS, WALL_Y)
    return [(tx, ty), (XS, WALL_Y), (-XS, WALL_Y), (-tx, ty), (-tx, -ty), (-XS, -WALL_Y), (XS, -WALL_Y), (tx, -ty)]


def bottom_deck():
    d = bd.extrude(bd.Plane.XY.offset(Z_BOT) * bd.Polygon(*box_outline(), align=None), amount=DECK_T)
    for sx in (1, -1):
        for sy in (1, -1):        # 4 rounded windows (v13's top-plate look); a noodle can't drop through
            w = bd.Pos(sx * 36.0, sy * 34.0, Z_BOT) * bd.Box(34, 44, 2 * DECK_T + 2)
            d -= bd.fillet(w.edges().filter_by(bd.Axis.Z), 6.0)
    return d


def spine():
    """Thin wall between the two noodles; above the deck it becomes the splice rack, the camera mast
    and the tether post."""
    x_post0, x_cam1 = POST_X - 20.0, CAM_X[1]
    outline = [(-XS - 1, Z_BOT), (XS + 1, Z_BOT), (XS + 1, BOX_TOP), (x_cam1, BOX_TOP), (x_cam1, CAM_Z),
               (CAM_X[0], CAM_Z), (CAM_X[0] - 10, RACK_TOP), (POST_X + 20, RACK_TOP), (POST_X + 20, TOWER_TOP - 12),
               (x_post0, TOWER_TOP - 12), (x_post0, BOX_TOP), (-XS - 1, BOX_TOP)]
    wins = []
    for xc in (-40.0, 0.0, 40.0):
        wins.append([(xc - 12, 0), (xc, -24), (xc + 12, 0), (xc, 24)])
    for xc in (-8.0, 8.0):                           # zip-tie holes in the splice rack
        wins.append([(xc - 2.5, BOX_TOP + 4), (xc + 2.5, BOX_TOP + 4), (xc + 2.5, BOX_TOP + 7), (xc - 2.5, BOX_TOP + 7)])
    s = plate(xz_plane(0.0), outline, wins, SPINE_T, r=0.8)
    return s


def lips():
    """45 deg lips along the top of the side walls and the spine: the noodle squeezes in past them and stays."""
    out = bd.Part()
    for sy in (1, -1):
        for y_face, inward in ((sy * WALL_IN, -sy), (sy * SPINE_T / 2, sy)):
            tri = bd.Polygon((y_face, BOX_TOP), (y_face + inward * LIP, BOX_TOP), (y_face, BOX_TOP - LIP), align=None)
            out += bd.extrude(yz_plane(0.0) * tri, amount=XS, both=True)
    return out


# ---------------------------------------------------------------- rings and cross bar
def vertical_ring(sx):
    x = sx * VX
    r = z_cyl(R_OUT, Z_BOT, RING_TOP, x)
    for a in ((45, -45) if sx > 0 else (135, 225)):
        lug = box(R_OUT - 2, R_OUT + 10, -5, 5, Z_BOT, Z_BOT + 3)
        for rr in (R_OUT + 3.0, R_OUT + 7.0):
            lug -= box(rr - 0.9, rr + 0.9, -1.6, 1.6, Z_BOT - 1, Z_BOT + 4)
        r += bd.Pos(x, 0, 0) * bd.Rot(0, 0, a) * lug
    return r


def side_ring(sy):
    y = sy * SY
    a, b = XR - CB, XR + CB
    d = x_diamond(R_OUT, a, b, y, SZ)
    outer = (x_cyl(R_OUT, a, b, y, SZ) + (d & box(a, b, y - 60, y + 60, Z_BOT, SZ - R_OUT / S2))
             + (d & box(a, b, y - 60, y + 60, SZ + R_OUT / S2, Z_TOP)))
    fin = box(a, XR - 2.0, y - 1.5, y + 1.5, Z_TOP - 4.0, Z_TOP + 7.0)
    for x in (XR - 10.0, XR - 5.0):
        fin -= box(x - 1.6, x + 1.6, y - 3, y + 3, Z_TOP + 2.2, Z_TOP + 4.0)
    return outer + fin


def side_bore(sy):
    y = sy * SY
    a, c = XR - CB - 1, XR + CB + 1
    b = x_cyl(R_IN, a, c, y, SZ)
    b += x_diamond(R_IN, a, c, y, SZ) & box(a, c, y - 60, y + 60, SZ + R_IN / S2, Z_TOP - 2.0)
    return b


def cross_bar():
    """Outside the box, like v13: two plates (stadium shape out to the side ring), top and bottom straps, mid web."""
    c = bd.Part()
    for sy in (1, -1):
        y0, y1 = sy * (WALL_IN + 0.5), sy * SY
        outline = [(min(y0, y1), Z_BOT), (max(y0, y1), Z_BOT), (max(y0, y1), Z_TOP), (min(y0, y1), Z_TOP)]
        yc = sy * (WALL_IN + PLATE_T + 6.0)
        win = [[(yc - 3, 5), (yc + 3, 5), (yc + 3, 18), (yc, 21), (yc - 3, 18)],
               [(yc - 3, -29), (yc + 3, -29), (yc + 3, -16), (yc, -13), (yc - 3, -16)]]
        for sx in (1, -1):
            c += plate(yz_plane(XR + sx * (CB - PLATE_T / 2)), outline, win, PLATE_T, r=0.8)
        c += box(XR - CB, XR + CB, y0, y1, Z_TOP - DECK_T, Z_TOP)
        c += box(XR - CB, XR + CB, y0, y1, Z_BOT, Z_BOT + DECK_T)
        c += box(XR - CB + PLATE_T, XR + CB - PLATE_T, y0, y1, -1.2, 1.2)
    c += box(XR - CB, XR + CB, -WALL_IN, WALL_IN, Z_BOT, Z_BOT + DECK_T)        # strap under the noodles
    return c


# ---------------------------------------------------------------- camera and tether
def camera_shelf():
    x0, x1 = CAM_X
    s = box(x0, x1, -CAM_W / 2, CAM_W / 2, CAM_Z, CAM_Z + DECK_T)
    for xg in (x0 + 1.2, (x0 + x1) / 2, x1 - 1.2):      # 45 deg gussets from the spine
        g = bd.Polygon((-CAM_W / 2, CAM_Z), (CAM_W / 2, CAM_Z), (0, CAM_Z - CAM_W / 2), align=None)
        s += bd.extrude(yz_plane(xg) * g, amount=1.2, both=True)
    for x in (x0 + 4.0, x1 - 4.0):
        for yy in (-12.0, 12.0):
            s -= box(x - 1.2, x + 1.2, yy - 2.5, yy + 2.5, CAM_Z - 1, CAM_Z + DECK_T + 1)
    return s & box(x0 - 1, x1 + 1, -50, 50, BOX_TOP, CAM_Z + DECK_T + 1)


def tether_post():
    T = TOWER_TOP
    zc = TOWER_TOP - 12.0
    t = box(POST_X - 20, POST_X + 20, -2.0, 2.0, BOX_TOP - 1, zc) + box(POST_X - 1.2, POST_X + 1.2, -8.0, 8.0, BOX_TOP, zc)
    xz = bd.extrude(bd.Plane.XZ * bd.Polygon((POST_X - 4, T - 26), (POST_X + 4, T - 26), (POST_X + 20, T - 10), (POST_X + 20, T),
                                             (POST_X - 20, T), (POST_X - 20, T - 10), align=None), amount=19, both=True)
    yz = bd.extrude(bd.Plane.YZ * bd.Polygon((-8, T - 26), (8, T - 26), (19, T - 15), (19, T), (-19, T), (-19, T - 15), align=None),
                    amount=200, both=True)
    t += xz & yz
    for sy in (1, -1):        # 45 deg gussets from the post foot out along the deck lips
        g = bd.Polygon((POST_X - 20, BOX_TOP), (POST_X + 20, BOX_TOP), (POST_X, BOX_TOP + 14), align=None)
        t += bd.extrude(xz_plane(sy * 1.2) * g, amount=1.2, both=True)
    t -= x_cyl(7.5, POST_X - 30, POST_X + 30, -5.0, TOWER_TOP)
    t -= x_cyl(4.0, POST_X - 30, POST_X + 30, 12.0, TOWER_TOP)
    for dx in (-12.0, 12.0):
        t -= box(POST_X + dx - 2.5, POST_X + dx + 2.5, -25, 25, T - 13.4, T - 11.0)
    return t


@model
def frame():
    f = bottom_deck() + spine() + lips() + cross_bar() + camera_shelf() + tether_post()
    for s in (1, -1):
        f += side_wall(s) + vertical_ring(s) + side_ring(s)
        for t in (1, -1):
            f += angle_wall(s, t)
    for s in (1, -1):
        f -= z_cyl(R_IN, Z_BOT - 1, TOWER_TOP + 40, s * VX)
        f -= side_bore(s)
        f -= x_cyl(POD_OD / 2 + 1.0, XR - 130, XR - CB, s * SY, SZ)
        f -= x_cyl(R_IN, XR + CB, XR + CB + 60, s * SY, SZ)
    sol = f.solids()
    if len(sol) > 1:
        sol = sorted(sol, key=lambda s: s.volume, reverse=True)
        assert sum(s.volume for s in sol[1:]) < 1.0, f"frame is {len(sol)} pieces: {[round(s.volume) for s in sol]}"
        f = sol[0]
    f.label = "frame"
    f.color = ORANGE
    return f


# ---------------------------------------------------------------- pods and floats (not printed)
@model
def thruster_pod():
    duct = x_cyl(DUCT_OD / 2, -DUCT_L, 0) - x_cyl(DUCT_OD / 2 - 2.5, -26, 1)
    sleeve = x_cyl(POD_OD / 2, 60, 90) - x_cyl(SLEEVE_ID / 2, 59, 91)
    rings = x_cyl(POD_OD / 2, 0, 6) - x_cyl(POD_OD / 2 - 3, -1, 7)
    casing = duct + sleeve + rings
    for a in (45, 135, 225, 315):
        casing += bd.Pos(32, 0, 0) * bd.Rot(a, 0, 0) * bd.Pos(0, 0, POD_OD / 2 - 3) * bd.Box(56, 6, 6)
    casing.label = "casing"; casing.color = WHITE
    motor = x_cyl(24, 15, 62) + x_cyl(SLEEVE_ID / 2 - 0.5, 60, 90)
    motor.label = "pump_motor"; motor.color = RED
    prop = x_cyl(PROP_D / 2, -14, -8) + x_cyl(4, -14, 15)
    prop.label = "prop"; prop.color = GREY
    return bd.Compound(children=[casing, motor, prop], label="thruster_pod")


def noodle_piece(length=NOODLE_L):
    """Noodle piece squeezed to the box height (64.6 mm), ends cut at 45 deg to fit the angled walls."""
    r = NOODLE_D / 2 - 0.3
    n = x_cyl(r, -length / 2, length / 2)
    if NOODLE_HOLE > 0:
        n -= x_cyl(NOODLE_HOLE / 2, -length / 2 - 1, length / 2 + 1)
    return n


def pod_locations():
    return {
        "pod_front": bd.Pos(VX, 0, Z_BOT) * bd.Rot(0, 90, 0),
        "pod_rear": bd.Pos(-VX, 0, Z_BOT) * bd.Rot(0, 90, 0),
        "pod_left": bd.Pos(XR - CB, SY, SZ) * bd.Rot(0, 0, 180),
        "pod_right": bd.Pos(XR - CB, -SY, SZ) * bd.Rot(0, 0, 180),
    }


NOODLE_X = -4.0     # noodle centre from analysis/sim.py


@model
def rov_v17():
    parts = [frame().shape]
    pod = thruster_pod().shape
    for name, loc in pod_locations().items():
        p = loc * pod
        p.label = name
        parts.append(p)
    for sy, nm in ((1, "noodle_left"), (-1, "noodle_right")):
        n = bd.Pos(NOODLE_X, sy * NY, NZ) * noodle_piece()
        n.label = nm; n.color = YELLOW
        parts.append(n)
    return bd.Compound(children=parts, label="rov_v17")


if __name__ == "__main__":
    frame()
    rov_v17()
