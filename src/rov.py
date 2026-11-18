"""Aqua Bots v14: streamlined 4-thruster ROV hull for the team's bilge-pump thruster pods.

Printed parts (FDM, PETG recommended; PLA works in a pool):
  hull_tub     printed floor-down, open top
  hull_cap     printed roof-down (top 10 mm of the hull, screws on with 6 x M3)
  nacelle x2   printed standing on end; bolt to the hull sides with 4 x M3 each

Check: cad model dfm "$ID" --process printing --technology fdm --material petg --build-volume 256x256x256
"""
import math

import build123d as bd
from cad import model

from params import (B, BALLAST_H, BOLTS, BOSS_R, BOSSES, CAM_D, CAM_DEPTH, CAM_Z, CH, DUCT_ID, DUCT_OD, FOAM_Z0, H, NAC_RO, NAC_WALL, NAC_X, NAC_Y, NOSE_A, BOSS_NAC_R, POD_L, POD_OD, PROP_D, TOP_DECK, SKIN, SLEEVE_ID, TETHER_D, TETHER_Z, TUN_ID, TUN_WALL, TX, Z_SPLIT)


ORANGE = "#f07a1a"
GREY = "#3b4048"
RED = "#a3201c"
WHITE = "#e9e6df"
FOAM = "#f2d64bcc"


# ---------------------------------------------------------------- shapes
def plan(off=0.0):
    """Plan-view outline of the hull: straight sides with elliptical nose and tail."""
    with bd.BuildSketch() as s:
        bd.Rectangle(2 * TX, 2 * B)
        for sx in (1, -1):
            with bd.Locations((sx * TX, 0)):
                bd.Ellipse(NOSE_A, B)
    sk = s.sketch
    if off:
        sk = bd.offset(sk, off)
    return sk


def envelope(t=0.0):
    """Hull outer solid, shrunk inward by t (t = SKIN gives the inside of the skin)."""
    s = bd.Pos(0, 0, -H / 2 + t) * bd.extrude(plan(-t), H - 2 * t)
    c = CH - t * (2 - math.sqrt(2))
    top = s.faces().sort_by(bd.Axis.Z)[-1]
    bot = s.faces().sort_by(bd.Axis.Z)[0]
    return bd.chamfer(top.edges() + bot.edges(), c)


def tube_z(x, y, ro, ri, z0, z1):
    h = z1 - z0
    o = bd.Pos(x, y, z0) * bd.Cylinder(ro, h, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    if ri:
        o = o - bd.Pos(x, y, z0 - 1) * bd.Cylinder(ri, h + 2, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    return o


def x_cyl(r, x0, x1, y=0.0, z=0.0):
    return bd.Pos(x0, y, z) * bd.Rot(0, 90, 0) * bd.Cylinder(
        r, x1 - x0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))


def y_cyl(r, y0, y1, x=0.0, z=0.0):
    return bd.Pos(x, y0, z) * bd.Rot(-90, 0, 0) * bd.Cylinder(
        r, y1 - y0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))


def tear_x(r, x0, x1, y=0.0, z=0.0, point=1):
    """Cylinder along X with a 45 degree point on top (point=1) or underneath (point=-1): prints without support."""
    c = x_cyl(r, x0, x1, y=y, z=z)
    d = bd.Pos((x0 + x1) / 2, y, z) * bd.Rot(45, 0, 0) * bd.Box(x1 - x0, 2 * r, 2 * r)
    half = bd.Pos((x0 + x1) / 2, y, z + point * 2 * r) * bd.Box(x1 - x0 + 2, 4 * r, 4 * r)
    return c + (d & half)


def tear_y(r, y0, y1, x=0.0, z=0.0):
    """Cylinder along Y with a 45 degree point towards -X (the nacelle prints standing on its back end)."""
    c = y_cyl(r, y0, y1, x=x, z=z)
    d = bd.Pos(x, (y0 + y1) / 2, z) * bd.Rot(0, 45, 0) * bd.Box(2 * r, y1 - y0, 2 * r)
    half = bd.Pos(x - 2 * r, (y0 + y1) / 2, z) * bd.Box(4 * r, y1 - y0 + 2, 4 * r)
    return c + (d & half)


def slab(z0, z1):
    return bd.Pos(0, 0, z0) * bd.Box(400, 400, z1 - z0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))


def nose_x(y=0.0):
    """x of the nose tip at height 0 for lateral offset y."""
    return TX + NOSE_A * math.sqrt(max(0.0, 1 - (y / B) ** 2))


# ---------------------------------------------------------------- hull
def hull_body():
    """Whole hull (tub + cap, before splitting)."""
    outer = envelope()
    shell = outer - envelope(SKIN)

    # vertical thruster tunnels, trimmed to the outside of the hull
    for sx in (1, -1):
        shell += tube_z(sx * TX, 0, TUN_ID / 2 + TUN_WALL, 0, -H / 2, H / 2) & outer
    # screw bosses for the cap
    for x, y in BOSSES:
        shell += tube_z(x, y, BOSS_R, 0, -H / 2, H / 2) & outer
        shell += tube_z(x, y, BOSS_R + 1.0, 0, Z_SPLIT, H / 2) & outer        # wider in the cap: room for the head
    # camera bay tube in the nose and tether tube in the tail
    shell += (tear_x(CAM_D / 2 + 1.6, nose_x() - CAM_DEPTH - 2, nose_x() + 5, z=CAM_Z, point=-1) & outer)
    shell += (tear_x(TETHER_D / 2 + 2.0, -nose_x() - 5, -nose_x() + 22, z=TETHER_Z, point=-1) & outer)

    # --- cuts
    for sx in (1, -1):
        shell -= tube_z(sx * TX, 0, TUN_ID / 2, 0, -H, H)
        # wire holes (top and bottom) and the pod retaining screw (M3 self-tapper into the pod's sleeve),
        # all through the tunnel wall that faces the middle compartment
        x_in = sx * (TX - TUN_ID / 2)
        for z in (H / 2 - TOP_DECK - 20.0, -H / 2 + 30.0):
            shell -= x_cyl(4.0, x_in - 4, x_in + 4, z=z)
        shell -= x_cyl(1.25, x_in - 4, x_in + 4, z=-H / 2 + 15.0)
    for x, y in BOSSES:
        shell -= tube_z(x, y, 1.25, 0, -H / 2 + SKIN + 2, H / 2 + 1)        # M3 self-tap pilot
        shell -= tube_z(x, y, 2.9, 0, H / 2 - 2.5, H / 2 + 1)               # M3 head counterbore (5.8 mm)
    shell -= tear_x(CAM_D / 2, nose_x() - CAM_DEPTH, nose_x() + 10, z=CAM_Z, point=1)
    shell -= x_cyl(TETHER_D / 2, -nose_x() - 10, -nose_x() + 25, z=TETHER_Z)
    # side wall: nacelle bolt holes + wire hole
    for sy in (1, -1):
        for x, z in BOLTS:
            shell -= y_cyl(1.65, sy * B - 4, sy * B + 4, x=x, z=z)
        shell -= y_cyl(4.0, sy * B - 4, sy * B + 4, x=NAC_X, z=0)
    # flood / drain holes: floor and roof (the hull must fill with water; foam gives the lift)
    for x, y in [(80, 32), (80, -32), (-80, 32), (-80, -32), (0, 20), (0, -20), (112, 12), (-112, 12)]:
        shell -= tube_z(x, y, 2.5, 0, -H, H)
    return shell


@model
def hull_tub():
    tub = hull_body() & slab(-H, Z_SPLIT)
    tub.label = "hull_tub"
    tub.color = ORANGE
    return tub


@model
def hull_cap():
    cap = hull_body() & slab(Z_SPLIT, H)
    for x, y in BOSSES:
        cap -= tube_z(x, y, 1.7, 0, Z_SPLIT - 1, H)                          # M3 clearance through the cap
    cap.label = "hull_cap"
    cap.color = ORANGE
    return cap


# ---------------------------------------------------------------- nacelle (left side; right is mirrored)
@model
def nacelle():
    """Left nacelle: a thin shroud round the side pod, held to the hull by 4 bolt bosses (M3 + nut)."""
    ro, ri = NAC_RO, TUN_ID / 2
    n = x_cyl(ro, NAC_X - POD_L / 2, NAC_X + POD_L / 2, y=NAC_Y)
    # two ribs along the bolt lines spread the load into the tube (45 degree ends: print standing on end)
    for zr in sorted({z for _, z in BOLTS}):
        yt = NAC_Y - math.sqrt(ro ** 2 - zr ** 2)              # tube surface at this height
        L = 46.0
        pts = [(NAC_X - L, NAC_Y), (NAC_X + L, NAC_Y), (NAC_X + L, yt), (NAC_X + L - (yt - B), B),
               (NAC_X - L + (yt - B), B), (NAC_X - L, yt)]
        rib = bd.extrude(bd.Pos(0, 0, zr - 1.5) * bd.Polygon(*pts, align=None), 3.0)
        n += rib
    for x, z in BOLTS:
        n += tear_y(BOSS_NAC_R, B, NAC_Y, x=x, z=z)
    n += tear_y(6.0, B, NAC_Y, x=NAC_X, z=0)                            # wire passage boss
    n -= x_cyl(ri, NAC_X - POD_L, NAC_X + POD_L, y=NAC_Y)
    for x, z in BOLTS:
        n -= y_cyl(1.65, B - 1, NAC_Y, x=x, z=z)
        # nut pocket (M3 nut 5.5 AF x 2.4), slid in from the outside of the boss (top or bottom)
        zs = 1 if z > 0 else -1
        n -= bd.Pos(x, B + 2.0, z) * bd.Box(5.9, 2.7, 12, align=(bd.Align.CENTER, bd.Align.MIN,
                                                                  bd.Align.MIN if zs > 0 else bd.Align.MAX))
    n -= y_cyl(4.0, B - 1, NAC_Y, x=NAC_X, z=0)                          # wire hole
    n -= bd.Pos(NAC_X + 42.0, NAC_Y, 0) * bd.Cylinder(1.25, 2 * ro + 2)  # pod retaining screw (top and bottom)
    n.label = "nacelle"
    n.color = GREY
    return n


# ---------------------------------------------------------------- team's thruster pod (stand-in for the casing + motor)
@model
def thruster_pod():
    """Stand-in for 'motor caseing v1' + the modified pump. Local axis +X; prop duct at x=-25, sleeve at x=60..90."""
    duct = x_cyl(DUCT_OD / 2, -25, 0) - x_cyl(DUCT_ID / 2, -26, 1)
    sleeve = x_cyl(POD_OD / 2, 60, 90) - x_cyl(SLEEVE_ID / 2, 59, 91)
    rings = (x_cyl(POD_OD / 2, 0, 6) - x_cyl(POD_OD / 2 - 3, -1, 7))
    bars = [bd.Pos(32, 0, 0) * bd.Rot(a, 0, 0) * bd.Pos(0, 0, POD_OD / 2 - 3) * bd.Box(56, 6, 6)
            for a in (45, 135, 225, 315)]
    casing = duct + sleeve + rings
    for b in bars:
        casing += b
    casing.label = "casing"
    casing.color = WHITE
    motor = x_cyl(24, 15, 62) + x_cyl(SLEEVE_ID / 2 - 0.5, 60, 90)
    motor.label = "pump_motor"
    motor.color = RED
    prop = x_cyl(PROP_D / 2, -14, -8) + x_cyl(4, -14, 15)
    prop.label = "prop"
    prop.color = GREY
    return bd.Compound(children=[casing, motor, prop], label="thruster_pod")


# ---------------------------------------------------------------- foam (buoyancy) - cut from XPS / closed-cell foam
def cavity(z0, z1):
    c = envelope(SKIN) & slab(z0, z1)
    for sx in (1, -1):
        c -= tube_z(sx * TX, 0, TUN_ID / 2 + TUN_WALL + 0.5, 0, -H, H)
    for x, y in BOSSES:
        c -= tube_z(x, y, BOSS_R + 1.5, 0, -H, H)
    c -= tear_x(CAM_D / 2 + 2.5, nose_x() - CAM_DEPTH - 6, nose_x() + 10, z=CAM_Z, point=-1)
    c -= tear_x(TETHER_D / 2 + 3, -nose_x() - 10, -nose_x() + 30, z=TETHER_Z, point=-1)
    return c


@model
def foam():
    f = cavity(FOAM_Z0, Z_SPLIT - 0.5)
    f.label = "foam"
    f.color = FOAM
    return f


@model
def ballast_bays():
    """The space under the foam, split front / middle / rear (shown only to size the ballast)."""
    z0, z1 = -H / 2 + SKIN, FOAM_Z0
    c = cavity(z0, z1)
    bays = []
    for name, x0, x1 in (("bay_front", TX, 300), ("bay_middle", -TX, TX), ("bay_rear", -300, -TX)):
        b = c & (bd.Pos((x0 + x1) / 2, 0, 0) * bd.Box(x1 - x0, 400, 400))
        b.label = name
        b.color = "#7a7f87"
        bays.append(b)
    return bd.Compound(children=bays, label="ballast_bays")


@model
def foam_cap():
    """Foam sheet glued inside the cap (the highest buoyancy = the most self-righting)."""
    f = cavity(Z_SPLIT + 0.5, H / 2 - SKIN)
    f.label = "foam_cap"
    f.color = FOAM
    return f


# ---------------------------------------------------------------- assembly
def placed_pods():
    pod = thruster_pod().shape
    out = []
    for name, sx in (("pod_front", 1), ("pod_rear", -1)):
        p = bd.Pos(sx * TX, 0, -H / 2 + POD_L - 25) * bd.Rot(0, 90, 0) * pod   # duct up, heavy end flush with the bottom
        p.label = name
        out.append(p)
    for name, sy in (("pod_left", 1), ("pod_right", -1)):
        p = bd.Pos(NAC_X + POD_L / 2 - 25, sy * NAC_Y, 0) * bd.Rot(0, 0, 180) * pod   # prop duct at the front
        p.label = name
        out.append(p)
    return out


@model
def rov_v14():
    tub = hull_tub().shape
    cap = hull_cap().shape
    nac = nacelle().shape
    left = bd.Pos(0, 0, 0) * nac
    left.label = "nacelle_left"
    right = bd.Rot(180, 0, 0) * nac          # same part turned over (the nacelle is symmetric top/bottom)
    right.label = "nacelle_right"
    right.color = GREY
    fm = foam().shape
    fc = foam_cap().shape
    parts = [tub, cap, left, right, fm, fc] + placed_pods()
    return bd.Compound(children=parts, label="rov_v14")


if __name__ == "__main__":
    rov_v14()


# ---------------------------------------------------------------- 2D templates for cutting the foam (print 1:1)
@model
def foam_template_main():
    """Plan-view outline of the main foam block (cut from 20-25 mm sheets, stack and glue)."""
    f = foam().shape
    return bd.section(f, bd.Plane.XY)


@model
def foam_template_cap():
    f = foam_cap().shape
    zc = f.center(bd.CenterOf.BOUNDING_BOX).Z
    return bd.Pos(0, 0, -zc) * bd.section(f, bd.Plane.XY.offset(zc))
