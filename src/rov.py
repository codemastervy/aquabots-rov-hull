"""Aqua Bots v15 ROV: one printed frame, four thruster pods, two pool-noodle floats.

Printed in one piece, no supports. The only hardware is zip ties:
- the pods slide into printed bands and each is locked by a zip tie round one of its cage bars
- the noodles sit in saddles above the side pods, zip-tied over the top
- the tether, camera cable and splices are zip-tied to the tower, the rails and the splice tray

Build: cad run src/rov.py
"""
import math

import build123d as bd
from cad import model

from params import (CONE_H, PB, BAND_ID, BAND_T, BAND_W, CAM_SHELF, CAM_W, CAM_Z, CAMCABLE_D, DUCT_OD, LIP_R, LIP_T, NOODLE_D,
                    NOODLE_HOLE, NOODLE_L, NZ, POD_L, POD_OD, PROP_D, RAIL_T, RAIL_TOP, RAIL_X, RAIL_Y0, SADDLE_R,
                    SLEEVE_ID, SY, SZ, TETHER_D, TOWER_TOP, TRAY_H, VX, Z_BOT)

ORANGE = "#f28c28"
WHITE = "#e8e8e8"
RED = "#c0392b"
GREY = "#555555"
YELLOW = "#f4d03f"
R_IN, R_OUT = BAND_ID / 2, BAND_ID / 2 + BAND_T
S2 = math.sqrt(2.0)


# ---------------------------------------------------------------- helpers
def box(x0, x1, y0, y1, z0, z1):
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1)); z0, z1 = sorted((z0, z1))
    return bd.Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * bd.Box(x1 - x0, y1 - y0, z1 - z0)


def z_cyl(r, z0, z1, x=0.0, y=0.0):
    return bd.Pos(x, y, z0) * bd.Cylinder(r, z1 - z0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))


def x_cyl(r, x0, x1, y=0.0, z=0.0):
    return bd.Pos(x0, y, z) * bd.Rot(0, 90, 0) * bd.Cylinder(r, x1 - x0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))


def tear_x(r, x0, x1, y=0.0, z=0.0, up=True, down=False):
    """Cylinder along X with 45 degree points up and/or down: prints without supports."""
    s = x_cyl(r, x0, x1, y, z)
    diamond = bd.Pos((x0 + x1) / 2, y, z) * bd.Rot(45, 0, 0) * bd.Box(x1 - x0, 2 * r, 2 * r)
    if up:
        s += diamond & box(x0 - 1, x1 + 1, y - 2 * r, y + 2 * r, z + r / S2, z + 2 * r)
    if down:
        s += diamond & box(x0 - 1, x1 + 1, y - 2 * r, y + 2 * r, z - 2 * r, z - r / S2)
    return s


def slot(c, size, axis):
    """A zip-tie slot: box centred on c, size (along slot, across, depth through)."""
    return bd.Pos(*c) * bd.Rot(*axis) * bd.Box(*size)


# ---------------------------------------------------------------- frame
def pod_space(x):
    """What a vertical pod and its wires need clear: the pod above the lip, the wire hole through the lip,
    the skirt under the funnel."""
    return (z_cyl(R_IN, PB, 120, x) + z_cyl(LIP_R, Z_BOT - 1, PB + 1, x)
            + z_cyl(R_IN, Z_BOT - 1, PB - LIP_T - CONE_H, x))


def vertical_band(sx):
    """Tube round the vertical pod from the plate up to z 34, with pointed (45 degree) windows: no overhangs.
    The pod stands on a lip 14 mm up; under it a skirt with a notch lets its wires turn and leave sideways."""
    x = sx * VX
    b = z_cyl(R_OUT, Z_BOT, 20.0 + BAND_W, x)
    lip_ring = z_cyl(R_IN + 0.5, PB - LIP_T, PB, x)
    funnel = z_cyl(R_IN + 0.5, PB - LIP_T - CONE_H, PB - LIP_T, x) - (
        bd.Pos(x, 0, PB - LIP_T - CONE_H) * bd.Cone(R_IN, LIP_R, CONE_H, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)))
    b = b - z_cyl(R_IN, Z_BOT - 1, 60, x) + lip_ring + funnel
    b -= pod_space(x)
    # wire notch in the skirt, facing the splice tray (off-centre, clear of the tether column)
    a = math.radians(180.0 - 25.0 if sx > 0 else 25.0)
    b -= bd.Pos(x + R_IN * math.cos(a), R_IN * math.sin(a), Z_BOT + 3.5) * bd.Rot(0, 0, math.degrees(a)) * bd.Box(10, 12, 9)
    z0, z1, w = PB + 12.4, 20.0 - 2.0, 20.0
    win = bd.Polygon((-w / 2, z0), (w / 2, z0), (w / 2, z1 - w / 2), (0, z1), (-w / 2, z1 - w / 2), align=None)
    for k in range(8):
        a = math.radians(22.5 + 45.0 * k)
        pl = bd.Plane(origin=(x + R_IN * math.cos(a), R_IN * math.sin(a), 0),
                      x_dir=(-math.sin(a), math.cos(a), 0), z_dir=(math.cos(a), math.sin(a), 0))
        b -= bd.extrude(pl * win, amount=6, both=True)
    # zip-tie slots in the top band, either side of the cage bar at 45 degrees (outboard-left)
    for da in (-13.0, 13.0):
        a = math.radians((45.0 if sx > 0 else 135.0) + da)
        c = (x + R_IN * math.cos(a) + 1.2 * math.cos(a), R_IN * math.sin(a) + 1.2 * math.sin(a), 25.5)
        b -= bd.Pos(*c) * bd.Rot(0, 0, math.degrees(a)) * bd.Box(8, 5.0, 2.4)
    return b


def side_band(sy, x0, x1, lip=False):
    y = sy * SY
    outer = tear_x(R_OUT, x0, x1, y, SZ, up=True, down=True)
    b = outer & box(x0 - 1, x1 + 1, y - 60, y + 60, Z_BOT, 200)
    b += box(x0, x1, y - 4.0, y + 4.0, Z_BOT, SZ - R_OUT * S2 + 4.0)          # foot: the point starts on the plate
    b -= tear_x(R_IN, x0 - 1, x1 + 1, y, SZ, up=True)
    if lip:
        l = (tear_x(R_OUT, x0 - LIP_T, x0, y, SZ, up=True, down=True) & box(x0 - 3, x0 + 1, y - 60, y + 60, Z_BOT, 200))
        b += l - tear_x(LIP_R, x0 - LIP_T - 1, x0 + 1, y, SZ, up=True)
    return b


def side_band_slots(sy):
    """Two zip-tie slots in the front band, either side of the cage bar on the outboard-top diagonal."""
    y = sy * SY
    cut = None
    for da in (-13.0, 13.0):
        a = math.radians(45.0 + da)
        cy, cz = y + sy * (R_IN + 1.2) * math.cos(a), SZ + (R_IN + 1.2) * math.sin(a)
        s = bd.Pos(27.0, cy, cz) * bd.Rot(sy * math.degrees(a), 0, 0) * bd.Box(2.4, 8, 5.0)
        cut = s if cut is None else cut + s
    return cut


def noodle_saddle(sy, x0, x1):
    y = sy * SY
    ro = SADDLE_R + BAND_T
    s = tear_x(ro, x0, x1, y, NZ, up=False, down=True) & box(x0 - 1, x1 + 1, y - 60, y + 60, Z_BOT, NZ)
    s -= x_cyl(SADDLE_R, x0 - 1, x1 + 1, y, NZ)
    # zip-tie slots through both walls, just below the noodle's equator
    for side in (1, -1):
        s -= box((x0 + x1) / 2 - 2.5, (x0 + x1) / 2 + 2.5, y + side * (SADDLE_R - 1), y + side * (ro + 1), NZ - 9, NZ - 6.6)
    return s


def rail(sy):
    y0, y1 = sorted((sy * RAIL_Y0, sy * (RAIL_Y0 + RAIL_T)))
    r = box(-RAIL_X, RAIL_X, y0, y1, Z_BOT, RAIL_TOP)
    # lightening windows where nothing is attached (8 mm frame left round them), 45 degree tops
    for xa, xb in ((-99.0, -78.0), (-38.0, 16.0), (37.0, 51.0), (78.0, 99.0)):
        w = xb - xa
        prof = bd.Polygon((xa, Z_BOT + 8), (xb, Z_BOT + 8), (xb, RAIL_TOP - 8 - w / 2), ((xa + xb) / 2, RAIL_TOP - 8),
                          (xa, RAIL_TOP - 8 - w / 2), align=None)
        r -= bd.extrude(bd.Plane.XZ * prof, amount=60, both=True)
    for sx in (1, -1):        # between each vertical pod's two bands
        xa, xb = sx * VX - 12, sx * VX + 12
        prof = bd.Polygon((xa, Z_BOT + BAND_W + 6), (xb, Z_BOT + BAND_W + 6), (xb, 20.0 - 6 - 12), (sx * VX, 20.0 - 6),
                          (xa, 20.0 - 6 - 12), align=None)
        r -= bd.extrude(bd.Plane.XZ * prof, amount=60, both=True)
    # wire clip slots along the top edge (zip-tie round the cables)
    for x in (-88.0, -60.0, -20.0, 0.0, 20.0, 60.0, 88.0):
        r -= box(x - 2.5, x + 2.5, y0 - 1, y1 + 1, RAIL_TOP - 6, RAIL_TOP - 3.6)
    return r


def splice_tray():
    xe = VX - R_OUT + 18.0                            # floor runs under the bands' edges
    t = box(-xe, xe, -RAIL_Y0 - 0.5, RAIL_Y0 + 0.5, Z_BOT, Z_BOT + 2.4)
    for sx in (1, -1):                                # end walls between the vertical bands
        t += box(sx * 16.0 - 1.2, sx * 16.0 + 1.2, -RAIL_Y0 - 0.5, RAIL_Y0 + 0.5, Z_BOT, Z_BOT + TRAY_H)
    for sx in (1, -1):                                # keep the pod bores clear
        t -= pod_space(sx * VX)
    for x in (-10.0, 10.0):                           # zip-tie slots: tie the splice bundle down
        for yy in (-28.0, 28.0):
            t -= box(x - 2.5, x + 2.5, yy - 1.2, yy + 1.2, Z_BOT - 1, Z_BOT + 4)
    for yy in (-15.0, 0.0, 15.0):                     # drain holes
        t -= z_cyl(3.0, Z_BOT - 1, Z_BOT + 4, 0.0, yy)
    return t


def tether_tower():
    """'+' column from the tray to a clamp above the rear pod's duct. The tether pulls near the floats' height,
    so it hardly pitches the robot."""
    zc = TOWER_TOP - 12.0
    # the X plate runs into both vertical pod bands, so a tether yank goes straight into the stiff rings
    t = box(-(VX - R_IN - 0.5), VX - R_IN - 0.5, -2.0, 2.0, Z_BOT, zc) + box(-1.2, 1.2, -8.0, 8.0, Z_BOT, zc)
    # clamp block, widening upward at 45 degrees (no overhang)
    prof = bd.Polygon((-4, zc - 14), (4, zc - 14), (20, zc + 2), (20, TOWER_TOP), (-20, TOWER_TOP), (-20, zc + 2), align=None)
    blk = bd.extrude(bd.Plane.XZ * prof, amount=19, both=True)
    prof_y = bd.Polygon((-8, zc - 14), (8, zc - 14), (19, zc - 3), (19, TOWER_TOP), (-19, TOWER_TOP), (-19, zc - 3), align=None)
    t += blk & bd.extrude(bd.Plane.YZ * prof_y, amount=21, both=True)
    # cable grooves along X in the top: tether and camera cable (half-round, open on top)
    yt, yc = -5.0, 12.0
    t -= x_cyl(TETHER_D / 2 + 0.5, -30, 30, yt, TOWER_TOP)
    t -= x_cyl(CAMCABLE_D / 2 + 0.5, -30, 30, yc, TOWER_TOP)
    # zip-tie slots under the grooves, across Y, two along the clamp (strain relief)
    for x in (-12.0, 12.0):
        t -= box(x - 2.5, x + 2.5, -25, 25, TOWER_TOP - TETHER_D / 2 - 6.4, TOWER_TOP - TETHER_D / 2 - 4.0)
    return t


def camera_shelf():
    x0, x1 = CAM_SHELF
    s = box(x0, x1, -CAM_W / 2, CAM_W / 2, CAM_Z - 3.0, CAM_Z)
    xs = VX + 34.0                                     # fork plates start inside the front band
    for sy in (1, -1):
        s += box(xs, x1, sy * CAM_W / 2 - 3.0 * (sy > 0), sy * CAM_W / 2 + 3.0 * (sy < 0), Z_BOT, CAM_Z)
    s += box(x1 - 3.0, x1, -CAM_W / 2, CAM_W / 2, Z_BOT, CAM_Z)        # front bumper under the camera
    s += box(VX + R_IN + 0.5, x1, -1.2, 1.2, Z_BOT, CAM_Z)               # centre rib: shorter bridges under the shelf
    # windows in the fork plates and bumper, 45 degree tops
    s -= box(x1 - 4, x1 + 1, -CAM_W / 2 + 5, CAM_W / 2 - 5, Z_BOT + 6, CAM_Z - 9 - (CAM_W / 2 - 5))
    s -= bd.Pos(x1 - 1.5, 0, CAM_Z - 9 - (CAM_W / 2 - 5)) * bd.Rot(45, 0, 0) * bd.Box(5, (CAM_W - 10) / S2, (CAM_W - 10) / S2)
    for xa, xb in ((xs + 10, x1 - 8),):  # windows through the fork plates and the rib
        w = xb - xa
        prof = bd.Polygon((xa, Z_BOT + 6), (xb, Z_BOT + 6), (xb, CAM_Z - 9 - w / 2), ((xa + xb) / 2, CAM_Z - 9),
                          (xa, CAM_Z - 9 - w / 2), align=None)
        s -= bd.extrude(bd.Plane.XZ * prof, amount=40, both=True)
    s -= pod_space(VX)
    for x in (x0 + 8, x1 - 10):                                         # zip-tie slots for the camera
        for yy in (-12.0, 12.0):
            s -= box(x - 1.2, x + 1.2, yy - 2.5, yy + 2.5, CAM_Z - 4, CAM_Z + 1)
    return s


@model
def frame():
    f = rail(1) + rail(-1)
    for sx in (1, -1):
        f += vertical_band(sx)
    for sy in (1, -1):
        f += side_band(sy, 20.0, 20.0 + BAND_W)
        f += side_band(sy, -POD_L / 2, -POD_L / 2 + BAND_W, lip=True)
        f += noodle_saddle(sy, 20.0, 20.0 + BAND_W)
        f += noodle_saddle(sy, -POD_L / 2, -POD_L / 2 + BAND_W)
    f += splice_tray() + tether_tower() + camera_shelf()
    for sy in (1, -1):
        f -= side_band_slots(sy)
        f -= tear_x(R_IN, -POD_L / 2, 70.0, sy * SY, SZ, up=True)            # side pod bores clear
        f -= x_cyl(SADDLE_R, -POD_L / 2 - 5, 40.0, sy * SY, NZ)               # noodle seat through the band tops
    for sx in (1, -1):
        f -= pod_space(sx * VX)                                             # vertical pods and their wires clear
    assert len(f.solids()) == 1, f"frame is {len(f.solids())} pieces"
    f.label = "frame"
    f.color = ORANGE
    return f



# ---------------------------------------------------------------- thruster pods and floats (not printed here)
@model
def thruster_pod():
    """Stand-in for 'motor caseing v1' + the modified pump. Local axis +X; prop duct at x=-25..0, sleeve 60..90."""
    duct = x_cyl(DUCT_OD / 2, -25, 0) - x_cyl(DUCT_OD / 2 - 2.5, -26, 1)
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
    """name -> Location. Vertical pods: duct up. Side pods: duct (prop) forward."""
    return {
        "pod_front": bd.Pos(VX, 0, 32.5) * bd.Rot(0, 90, 0),
        "pod_rear": bd.Pos(-VX, 0, 32.5) * bd.Rot(0, 90, 0),
        "pod_left": bd.Pos(32.5, SY, SZ) * bd.Rot(0, 0, 180),
        "pod_right": bd.Pos(32.5, -SY, SZ) * bd.Rot(0, 0, 180),
    }


NOODLE_X = -4.2     # noodle centre from analysis/sim.py: 4 mm behind the frame centre


@model
def rov_v15():
    parts = [frame().shape]
    pod = thruster_pod().shape
    for name, loc in pod_locations().items():
        p = loc * pod
        p.label = name
        parts.append(p)
    for sy, nm in ((1, "noodle_left"), (-1, "noodle_right")):
        n = bd.Pos(NOODLE_X, sy * SY, NZ) * noodle_piece()
        n.label = nm; n.color = YELLOW
        parts.append(n)
    return bd.Compound(children=parts, label="rov_v15")


if __name__ == "__main__":
    frame()
    rov_v15()
