"""Aqua Bots v16 "classic" ROV: the shape of our v13 frame, redone so it floats, rights itself,
holds the wiring and prints without supports.

What it keeps from v13: a flat frame as tall as a side ring, two rails between the up/down rings,
diamond braces out to a box cross bar that ends in the two side rings, triangle-truss windows, round
cable tubes where the rails cross the cross bar. Like v13 it grips each pod by its 64 mm prop duct.

What is new:
- pool-noodle floats in two saddles on top of the cross bar (buoyancy high: it rights itself)
- the up/down rings sit on the build plate; the pods hang below with their heavy motors low
- tether + camera-cable clamp post (rear), camera shelf (front), splice tray (bottom deck)
- zip-tie lugs on every ring: no screws, no glue

Build: cad run src/rov.py
"""
import math

import build123d as bd
from cad import model

from params import (BRACE_Y, CAM_W, CAM_X, CB, DECK_T, DUCT_L, DUCT_OD, FULL_X, NOODLE_D, NOODLE_HOLE, NOODLE_L,
                    NY, NZ, PLATE_T, POD_OD, POST_X, PROP_D, R_IN, R_OUT, RAIL_Y, RING_L, SADDLE_R, SLEEVE_ID, SY,
                    SZ, TOWER_TOP, VX, Z_BOT, Z_TOP)

ORANGE = "#f28c28"
WHITE = "#e8e8e8"
RED = "#c0392b"
GREY = "#555555"
YELLOW = "#f4d03f"
S2 = math.sqrt(2.0)
RING_TOP = Z_BOT + RING_L          # top of the up/down rings (-9.7)
FILLET = 2.0                       # window corner radius


# ---------------------------------------------------------------- helpers
def box(x0, x1, y0, y1, z0, z1):
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1)); z0, z1 = sorted((z0, z1))
    return bd.Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * bd.Box(x1 - x0, y1 - y0, z1 - z0)


def z_cyl(r, z0, z1, x=0.0, y=0.0):
    return bd.Pos(x, y, z0) * bd.Cylinder(r, z1 - z0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))


def x_cyl(r, x0, x1, y=0.0, z=0.0):
    return bd.Pos(x0, y, z) * bd.Rot(0, 90, 0) * bd.Cylinder(r, x1 - x0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))


def x_diamond(r, x0, x1, y, z):
    """Square prism along X, turned 45 deg: corners r*sqrt2 above, below and to each side of (y, z)."""
    return bd.Pos((x0 + x1) / 2, y, z) * bd.Rot(45, 0, 0) * bd.Box(x1 - x0, 2 * r, 2 * r)


def plate(plane, outline, windows, t):
    """A flat plate: outline and windows as (u, v) points on `plane`, windows with rounded corners, t thick."""
    s = bd.Polygon(*outline, align=None)
    for w in windows:
        wp = bd.Polygon(*w, align=None)
        s = s - bd.fillet(wp.vertices(), FILLET)
    return bd.extrude(plane * s, amount=t / 2, both=True)


def xz_plane(y):           # u = world X, v = world Z
    return bd.Plane(origin=(0, y, 0), x_dir=(1, 0, 0), z_dir=(0, -1, 0))


def yz_plane(x):           # u = world Y, v = world Z
    return bd.Plane(origin=(x, 0, 0), x_dir=(0, 1, 0), z_dir=(1, 0, 0))


def mirror_u(windows):
    return [[(-u, v) for u, v in w] for w in windows]


# truss windows for one half of a full-height plate (u = distance from its inner end), all overhangs >= 45 deg
def truss_windows(u0):
    def sh(pts):
        return [(u0 + u, v) for u, v in pts]
    return [sh([(15, 0), (0 + 30, -23), (45, 0), (30, 23)]),           # centre diamond
            sh([(3, 29), (15, 29), (3, 11)]), sh([(45, 29), (33, 29), (45, 11)]),
            sh([(3, -29), (15, -29), (3, -11)]), sh([(45, -29), (33, -29), (45, -11)])]


# ---------------------------------------------------------------- frame parts
def rail(sy):
    """Side rail between the up/down rings: full height in the middle, tapering down onto the rings."""
    outline = [(-VX, Z_BOT), (VX, Z_BOT), (VX, RING_TOP), (FULL_X, Z_TOP), (-FULL_X, Z_TOP), (-VX, RING_TOP)]
    half = truss_windows(16.0) + [[(60, -29), (76, -29), (60, 10)]]
    return plate(xz_plane(sy * (RAIL_Y - PLATE_T / 2)), outline, half + mirror_u(half), PLATE_T)


def brace_end():
    """Where a diamond brace from the cross bar (x=CB, y=BRACE_Y) touches the front up/down ring (tangent)."""
    px, py = VX - CB, -BRACE_Y           # cross bar end, relative to the ring centre
    d = math.hypot(px, py)
    a = math.atan2(py, px) + math.acos(R_OUT / d)
    return VX + R_OUT * math.cos(a), -(R_OUT * math.sin(a))     # (x, y) with y > 0


def brace(sx, sy):
    tx, ty = brace_end()
    dx, dy = tx - CB, ty - BRACE_Y
    L = math.hypot(dx, dy)
    ux, uy = dx / L, dy / L
    uf = (FULL_X - CB) / ux
    outline = [(-2, Z_BOT), (L, Z_BOT), (L, RING_TOP), (uf, Z_TOP), (-2, Z_TOP)]
    wins = truss_windows(5.0) + [[(uf - 1, -29), (uf + 16, -29), (uf - 1, 8)]]
    pl = bd.Plane(origin=(CB, BRACE_Y, 0), x_dir=(ux, uy, 0), z_dir=(uy, -ux, 0))
    b = plate(pl, outline, wins, PLATE_T)
    if sy < 0:
        b = b.mirror(bd.Plane.XZ)
    if sx < 0:
        b = b.mirror(bd.Plane.YZ)
    return b


def cross_bar():
    """Box beam across the middle (like v13): two plates, top and bottom decks, a web out to the side rings."""
    outline = [(-SY, Z_BOT), (SY, Z_BOT), (SY, Z_TOP), (-SY, Z_TOP)]
    centre = [[(4, 0), (16, -22), (28, 0), (16, 22)]]
    outer = [[(38, 5), (51, 5), (51, 18), (44.5, 24.5), (38, 18)],
             [(38, -29), (51, -29), (51, -16), (44.5, -9.5), (38, -16)]]
    wins = centre + outer
    wins = wins + mirror_u(wins)
    c = bd.Part()
    for sx in (1, -1):
        c += plate(yz_plane(sx * (CB - PLATE_T / 2)), outline, wins, PLATE_T)
    c += box(-CB, CB, -SY, SY, Z_TOP - DECK_T, Z_TOP)                         # top deck (carries the saddles)
    c += box(-CB, CB, -SY, SY, Z_BOT, Z_BOT + DECK_T)                         # bottom deck
    for sy in (1, -1):
        c += box(-CB + PLATE_T, CB - PLATE_T, sy * RAIL_Y, sy * SY, -1.2, 1.2)   # web to the side ring
        c += z_cyl(10.0, Z_BOT, Z_TOP, 0, sy * 30.0)                          # cable tube (v13's round holes)
    return c


def side_ring(sy):
    """Ring round a side pod's prop duct (axis X). Flat foot on the plate; the bore's top is pointed then
    flattened (a 26 mm bridge), so it prints round enough without support. Top fin: 2 zip-tie holes."""
    y = sy * SY
    d = x_diamond(R_OUT, -CB, CB, y, SZ)
    outer = (x_cyl(R_OUT, -CB, CB, y, SZ) + (d & box(-CB, CB, y - 60, y + 60, Z_BOT, SZ - R_OUT / S2))
             + (d & box(-CB, CB, y - 60, y + 60, SZ + R_OUT / S2, Z_TOP)))      # flat top and foot, like v13's
    fin = box(-CB, -2.0, y - 1.5, y + 1.5, Z_TOP - 4.0, Z_TOP + 7.0)
    for x in (-10.0, -5.0):
        fin -= box(x - 1.6, x + 1.6, y - 3, y + 3, Z_TOP + 2.2, Z_TOP + 4.0)
    return outer + fin


def side_bore(sy):
    y = sy * SY
    b = x_cyl(R_IN, -CB - 1, CB + 1, y, SZ)
    b += x_diamond(R_IN, -CB - 1, CB + 1, y, SZ) & box(-CB - 1, CB + 1, y - 60, y + 60, SZ + R_IN / S2, Z_TOP - 2.0)
    return b


def vertical_ring(sx):
    """Ring round an up/down pod's prop duct, standing on the plate. Two lugs with zip-tie slots
    on the outer diagonals line up with the pod's cage bars."""
    x = sx * VX
    r = z_cyl(R_OUT, Z_BOT, RING_TOP, x)
    for a in ((45, -45) if sx > 0 else (135, 225)):
        lug = box(R_OUT - 2, R_OUT + 10, -5, 5, Z_BOT, Z_BOT + 3)
        for rr in (R_OUT + 3.0, R_OUT + 7.0):
            lug -= box(rr - 0.9, rr + 0.9, -1.6, 1.6, Z_BOT - 1, Z_BOT + 4)
        r += bd.Pos(x, 0, 0) * bd.Rot(0, 0, a) * lug
    return r


def splice_tray():
    """Bottom deck between the rails: the 8 motor-to-tether splices lie here, zip-tied, with drain holes."""
    xe = VX - R_IN
    t = box(-xe, xe, -(RAIL_Y - 1), RAIL_Y - 1, Z_BOT, Z_BOT + DECK_T)
    for x in (-50.0, -28.0, 28.0, 50.0):
        for yy in (-14.0, 14.0):
            t -= box(x - 1.2, x + 1.2, yy - 2.5, yy + 2.5, Z_BOT - 1, Z_BOT + 4)     # zip-tie slots
    for x in (-40.0, 40.0):
        for yy in (-20.0, 0.0, 20.0):
            t -= z_cyl(3.0, Z_BOT - 1, Z_BOT + 4, x, yy)                              # drain holes
    return t


def camera_shelf():
    """Top deck plate at the front for the camera: 3 ribs under it keep the bridges short."""
    x0, x1 = CAM_X
    s = box(x0, x1, -(RAIL_Y - 1), RAIL_Y - 1, Z_TOP - DECK_T, Z_TOP)
    rib_win = [[(x0 + 4, 0), (x0 + 15, -24), (x1 - 4, 0), (x0 + 15, 24)]]
    for yy in (-16.0, 0.0, 16.0):
        s += plate(xz_plane(yy), [(x0, Z_BOT), (x1, Z_BOT), (x1, Z_TOP - DECK_T), (x0, Z_TOP - DECK_T)], rib_win, 2.4)
    for x in (x0 + 4.0, x1 - 4.0):
        for yy in (-12.0, 12.0):
            s -= box(x - 1.2, x + 1.2, yy - 2.5, yy + 2.5, Z_TOP - 4, Z_TOP + 1)
    return s


def tether_post():
    """Rear post: the tether (up to 14 mm) and the camera cable (up to 7 mm) lie in grooves along X at the top
    and are zip-tied down; a pull goes into the frame, not into the splices."""
    zc = TOWER_TOP - 12.0
    t = box(POST_X - 20, -CB, -2.0, 2.0, Z_BOT, zc) + box(POST_X - 1.2, POST_X + 1.2, -8.0, 8.0, Z_BOT, zc)
    xz = bd.extrude(bd.Plane.XZ * bd.Polygon((POST_X - 4, 46), (POST_X + 4, 46), (POST_X + 20, 62), (POST_X + 20, 72),
                                             (POST_X - 20, 72), (POST_X - 20, 62), align=None), amount=19, both=True)
    yz = bd.extrude(bd.Plane.YZ * bd.Polygon((-8, 46), (8, 46), (19, 57), (19, 72), (-19, 72), (-19, 57), align=None),
                    amount=200, both=True)
    t += xz & yz
    t -= x_cyl(7.5, POST_X - 30, POST_X + 30, -5.0, TOWER_TOP)                 # tether
    t -= x_cyl(4.0, POST_X - 30, POST_X + 30, 12.0, TOWER_TOP)                 # camera cable
    for dx in (-12.0, 12.0):
        t -= box(POST_X + dx - 2.5, POST_X + dx + 2.5, -25, 25, 58.6, 61.0)   # zip-tie slots under the grooves
    return t


def saddle(sy):
    """Noodle cradle on the cross bar: pointed (45 deg) underside, 2 zip-tie holes in each arm."""
    y = sy * NY
    ro = SADDLE_R + 2.4
    s = (x_cyl(ro, -CB, CB, y, NZ) + (x_diamond(ro, -CB, CB, y, NZ) & box(-CB, CB, y - 60, y + 60, Z_TOP - 9, NZ - ro / S2))) \
        & box(-CB, CB, y - 60, y + 60, Z_TOP - 0.5, NZ)
    s -= x_cyl(SADDLE_R, -CB - 1, CB + 1, y, NZ)
    for side in (1, -1):
        for x in (-7.0, 7.0):
            s -= box(x - 1.6, x + 1.6, y + side * (SADDLE_R - 2), y + side * (SADDLE_R + 6), NZ - 7.0, NZ - 5.2)
    return s


@model
def frame():
    f = cross_bar()
    for s in (1, -1):
        f += rail(s) + vertical_ring(s) + side_ring(s) + saddle(s)
        for t in (1, -1):
            f += brace(s, t)
    f += splice_tray() + camera_shelf() + tether_post()
    # keep the pods, their water flow and the noodles clear
    for s in (1, -1):
        f -= z_cyl(R_IN, Z_BOT - 1, TOWER_TOP + 40, s * VX)                     # up/down duct bores, open above
        f -= side_bore(s)
        f -= x_cyl(POD_OD / 2 + 1.0, -130, -CB, s * SY, SZ)                     # side pod body behind its ring
        f -= x_cyl(R_IN, CB, CB + 60, s * SY, SZ)                               # side pod intake in front
        f -= x_cyl(SADDLE_R, -90, 90, s * NY, NZ)                               # noodle
        f -= z_cyl(7.5, Z_BOT - 1, Z_TOP + 1, 0, s * 30.0)                      # cable tube holes
    sol = f.solids()
    if len(sol) > 1:
        sol = sorted(sol, key=lambda s: s.volume, reverse=True)
        assert sum(s.volume for s in sol[1:]) < 1.0, f"frame is {len(sol)} pieces: {[round(s.volume) for s in sol]}"
        f = sol[0]
    f.label = "frame"
    f.color = ORANGE
    return f


# ---------------------------------------------------------------- thruster pods and floats (not printed here)
@model
def thruster_pod():
    """Stand-in for 'motor caseing v1' + the modified pump. Local axis +X; prop duct at x=-25..0, sleeve 60..90."""
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
    n = x_cyl(NOODLE_D / 2, -length / 2, length / 2)
    if NOODLE_HOLE > 0:
        n -= x_cyl(NOODLE_HOLE / 2, -length / 2 - 1, length / 2 + 1)
    return n


def pod_locations():
    """name -> Location. Up/down pods: prop duct up in the ring on the plate, pod body below.
    Side pods: prop duct forward in the side ring, pod body behind."""
    return {
        "pod_front": bd.Pos(VX, 0, Z_BOT) * bd.Rot(0, 90, 0),
        "pod_rear": bd.Pos(-VX, 0, Z_BOT) * bd.Rot(0, 90, 0),
        "pod_left": bd.Pos(-CB, SY, SZ) * bd.Rot(0, 0, 180),
        "pod_right": bd.Pos(-CB, -SY, SZ) * bd.Rot(0, 0, 180),
    }


NOODLE_X = 0.0      # noodle centre: set from analysis/sim.py


@model
def rov_v16():
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
    return bd.Compound(children=parts, label="rov_v16")


if __name__ == "__main__":
    frame()
    rov_v16()
