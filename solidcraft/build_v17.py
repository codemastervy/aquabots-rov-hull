"""Writes solidcraft/rov_v17_steps.json: the SolidCraft build of the v17 frame (same geometry as src/rov.py frame(),
small window corners left sharp). Rings, bores, box walls, spine, lips, cross bar, decks, post and shelf follow the
SolidCraft parameters (same names as src/params.py); the angled walls' planes and the window shapes are fixed numbers.

    python3 solidcraft/build_v17.py      then run the steps in SolidCraft (MCP batch with this file as `path`)
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
import params as P  # noqa: E402

R2 = 1 / math.sqrt(2.0)
steps = []


def cmd(c, **p):
    steps.append({"command": c, "params": p})


def f(v):
    return round(float(v), 4)


PARAMS = [("R_IN", f"{P.RING_ID / 2} mm"), ("RING_T", f"{P.RING_T} mm"), ("R_OUT", "R_IN + RING_T"),
          ("RING_L", f"{P.RING_L} mm"), ("VX", f"{P.VX} mm"), ("CB", f"{P.CB} mm"), ("XR", f"{P.XR} mm"),
          ("PLATE_T", f"{P.PLATE_T} mm"), ("DECK_T", f"{P.DECK_T} mm"), ("SPINE_T", f"{P.SPINE_T} mm"),
          ("NOODLE_D", f"{P.NOODLE_D} mm"), ("NY", f"{P.NY} mm"), ("NZ", f"{P.NZ} mm"),
          ("BOX_TOP", "NZ + NOODLE_D/2 + 0.6 mm"), ("XS", f"{P.XS} mm"), ("WALL_IN", "NY + NOODLE_D/2 + 0.5 mm"),
          ("WALL_Y", "WALL_IN + PLATE_T/2"), ("LIP", f"{P.LIP} mm"), ("POD_OD", f"{P.POD_OD} mm"),
          ("SY", "WALL_IN + PLATE_T + POD_OD/2 + 2.5 mm"), ("POST_X", f"{P.POST_X} mm"),
          ("TOWER_TOP", "BOX_TOP + 36 mm"), ("CAM_X0", f"{P.CAM_X[0]} mm"), ("CAM_X1", f"{P.CAM_X[1]} mm"),
          ("CAM_Z", "BOX_TOP + 16 mm"), ("CAM_W", f"{P.CAM_W} mm"), ("RACK_TOP", "BOX_TOP + 10 mm")]
for n, e in PARAMS:
    cmd("parameters.add", name=n, expression=e)

ZB, ZT, RT, BT = "(-R_OUT)", "R_OUT", "(RING_L - R_OUT)", "BOX_TOP"
first = [True]


def add(c, **p):
    if first[0]:
        p.update(operation="new", body_name="Frame")
        first[0] = False
    else:
        p.update(operation="join", targets=["Frame"], touching="one_body")
    cmd(c, **p)


def cut(c, **p):
    p.update(operation="cut", targets=["Frame"])
    cmd(c, **p)


def box(name, x0, x1, y0, y1, z0, z1, op=add):
    op("solid.box", name=name, length=f"({x1}) - ({x0})", width=f"({y1}) - ({y0})", height=f"({z1}) - ({z0})",
       center=[f"(({x0}) + ({x1}))/2", f"(({y0}) + ({y1}))/2", f"(({z0}) + ({z1}))/2"])


def poly(pts):
    cmd("sketch.line", points=[list(p) for p in pts], closed=True)


def sketch(name, plane, offset=None):
    p = {"plane": plane, "name": name}
    if offset is not None:
        p["offset"] = offset
    cmd("sketch.create", **p)


def xz(pts):
    return [(x, f"-({z})" if isinstance(z, str) else -z) for x, z in pts]


def plate_xz(name, y, outline, windows, t="PLATE_T"):
    sketch(name + "Sk", "XZ", y)
    poly(xz(outline))
    for w in windows:
        poly(xz(w))
    cmd("sketch.finish")
    add("solid.extrude", name=name, distance=t, direction="symmetric")


def truss(u0):
    sh = lambda pts: [(u0 + u, v) for u, v in pts]
    return [sh([(15, 0), (30, -23), (45, 0), (30, 23)]), sh([(3, 29), (15, 29), (3, 11)]),
            sh([(45, 29), (33, 29), (45, 11)]), sh([(3, -29), (15, -29), (3, -11)]), sh([(45, -29), (33, -29), (45, -11)])]


def mirror_u(ws):
    return [[(-u, v) for u, v in w][::-1] for w in ws]


def ring_profile(y, r, z_top, z_bot=None):
    """Circle r round (y, 0) with the circumscribed diamond clipped flat at z_top (and z_bot)."""
    w = lambda z: f"{r}*1.41421356 - ({z})"
    pts_top = [(f"{y} + {r}*{R2}", f"{r}*{R2}"), (f"{y} + {w(z_top)}", z_top), (f"{y} - ({w(z_top)})", z_top),
               (f"{y} - {r}*{R2}", f"{r}*{R2}")]
    cmd("sketch.line", points=[list(p) for p in pts_top])
    if z_bot is None:
        cmd("sketch.arc.three_point", start=list(pts_top[3]), end=list(pts_top[0]), through=[y, f"-{r}"])
        return
    wb = f"{r}*1.41421356 + ({z_bot})"
    pts_bot = [(f"{y} - {r}*{R2}", f"-{r}*{R2}"), (f"{y} - ({wb})", z_bot), (f"{y} + {wb}", z_bot),
               (f"{y} + {r}*{R2}", f"-{r}*{R2}")]
    cmd("sketch.arc.three_point", start=list(pts_top[3]), end=list(pts_bot[0]), through=[f"{y} - {r}", 0])
    cmd("sketch.line", points=[list(p) for p in pts_bot])
    cmd("sketch.arc.three_point", start=list(pts_bot[3]), end=list(pts_top[0]), through=[f"{y} + {r}", 0])


def rounded_rect(cx, cy, w, h, r):
    """Closed rounded rectangle (numbers) in the active sketch."""
    x0, x1, y0, y1 = cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2
    k = r * (1 - R2)
    cmd("sketch.line", points=[[x0 + r, y0], [x1 - r, y0]])
    cmd("sketch.arc.three_point", start=[x1 - r, y0], end=[x1, y0 + r], through=[x1 - k, y0 + k])
    cmd("sketch.line", points=[[x1, y0 + r], [x1, y1 - r]])
    cmd("sketch.arc.three_point", start=[x1, y1 - r], end=[x1 - r, y1], through=[x1 - k, y1 - k])
    cmd("sketch.line", points=[[x1 - r, y1], [x0 + r, y1]])
    cmd("sketch.arc.three_point", start=[x0 + r, y1], end=[x0, y1 - r], through=[x0 + k, y1 - k])
    cmd("sketch.line", points=[[x0, y1 - r], [x0, y0 + r]])
    cmd("sketch.arc.three_point", start=[x0, y0 + r], end=[x0 + r, y0], through=[x0 + k, y0 + k])


# angled walls: from the box corner in to the up/down ring (fixed planes)
WALL_Y = P.WALL_Y
r1 = P.R_OUT - 1.0
qx, qy = P.XS - P.VX, -WALL_Y
dd = math.hypot(qx, qy)
aa = math.atan2(qy, qx) + math.acos(r1 / dd)
TX, TY = P.VX + r1 * math.cos(aa), -(r1 * math.sin(aa))

# ---- bottom deck: the box's floor with 4 rounded windows
sketch("DeckSk", "XY", ZB)
poly([(f(TX), f(TY)), ("XS", "WALL_Y"), ("-XS", "WALL_Y"), (f(-TX), f(TY)), (f(-TX), f(-TY)), ("-XS", "-WALL_Y"),
      ("XS", "-WALL_Y"), (f(TX), f(-TY))])
for sx in (1, -1):
    for sy in (1, -1):
        rounded_rect(sx * 36.0, sy * 34.0, 34.0, 44.0, 6.0)
cmd("sketch.finish")
add("solid.extrude", name="BottomDeck", distance="DECK_T")

# ---- box side walls
half = truss(13.0)
for sy, nm in ((1, "L"), (-1, "R")):
    plate_xz("SideWall" + nm, f"{sy}*WALL_Y", [("-XS - 1", ZB), ("XS + 1", ZB), ("XS + 1", BT), ("-XS - 1", BT)],
             half + mirror_u(half))

# ---- angled walls
L = math.hypot(TX - P.XS, TY - WALL_Y)
ux, uy = (TX - P.XS) / L, (TY - WALL_Y) / L
uf = 0.35 * L
a_out = [(-2, P.Z_BOT), (L, P.Z_BOT), (L, P.Z_BOT + P.RING_L), (uf, P.BOX_TOP), (-2, P.BOX_TOP)]
a_win = [[(5, -29), (uf - 3, -29), (5, 18)], [(uf + 4, -29), (L - 10, -29), (uf + 4, 14)]]
for sx in (1, -1):
    for sy in (1, -1):
        sketch(f"AngleSk{sx:+d}{sy:+d}", {"origin": [f(sx * P.XS), f(sy * WALL_Y), 0.0], "x_dir": [f(sx * ux), f(sy * uy), 0.0],
                                          "y_dir": [0, 0, 1]})
        poly([(f(u), f(v)) for u, v in a_out])
        for w in a_win:
            poly([(f(u), f(v)) for u, v in w])
        cmd("sketch.finish")
        add("solid.extrude", name=f"AngleWall{sx:+d}{sy:+d}", distance="PLATE_T", direction="symmetric")

# ---- spine between the noodles: splice rack, camera mast, tether post foot
x_post0 = "POST_X - 20"
spine_out = [("-XS - 1", ZB), ("XS + 1", ZB), ("XS + 1", BT), ("CAM_X1", BT), ("CAM_X1", "CAM_Z"), ("CAM_X0", "CAM_Z"),
             ("CAM_X0 - 10", "RACK_TOP"), ("POST_X + 20", "RACK_TOP"), ("POST_X + 20", "TOWER_TOP - 12"),
             (x_post0, "TOWER_TOP - 12"), (x_post0, BT), ("-XS - 1", BT)]
spine_win = [[(xc - 12, 0), (xc, -24), (xc + 12, 0), (xc, 24)] for xc in (-40.0, 0.0, 40.0)]
spine_win += [[(xc - 2.5, f"{BT} + 4"), (xc + 2.5, f"{BT} + 4"), (xc + 2.5, f"{BT} + 7"), (xc - 2.5, f"{BT} + 7")]
              for xc in (-8.0, 8.0)]
plate_xz("Spine", "0", spine_out, spine_win, t="SPINE_T")

# ---- 45 deg lips along the top edges (the noodles squeeze in past them)
sketch("LipSk", "YZ", "0")
for sy in (1, -1):
    for yf, inward in ((f"{sy}*WALL_IN", -sy), (f"{sy}*SPINE_T/2", sy)):
        poly([(yf, BT), (f"{yf} + {inward}*LIP", BT), (yf, f"{BT} - LIP")])
cmd("sketch.finish")
add("solid.extrude", name="Lips", distance="2*XS", direction="symmetric")

# ---- up/down rings with 2 zip-tie lugs each
for sx, nm in ((1, "F"), (-1, "B")):
    add("solid.cylinder", name="VRing" + nm, radius="R_OUT", height="RING_L", base=[f"{sx}*VX", 0, ZB], axis="Z")
    sketch("Lugs" + nm, "XY", ZB)
    ro = P.R_OUT
    for a in ((45, -45) if sx > 0 else (135, 225)):
        c, s = math.cos(math.radians(a)), math.sin(math.radians(a))
        rot = lambda u, v: (f"{sx}*VX + {f(u * c - v * s)}", f(u * s + v * c))
        poly([rot(u, v) for u, v in ((ro - 2, -5), (ro + 10, -5), (ro + 10, 5), (ro - 2, 5))])
        for rr in (ro + 3.0, ro + 7.0):
            poly([rot(u, v) for u, v in ((rr - 0.9, -1.6), (rr + 0.9, -1.6), (rr + 0.9, 1.6), (rr - 0.9, 1.6))])
    cmd("sketch.finish")
    add("solid.extrude", name="Lug" + nm, distance="3 mm")

# ---- cross bar (outside the box, forward by XR) and the side rings
for sy, nm in ((1, "L"), (-1, "R")):
    y0, y1 = (f"WALL_IN + 0.5", "SY") if sy > 0 else ("-SY", "-(WALL_IN + 0.5)")
    yc = sy * (P.WALL_IN + P.PLATE_T + 6.0)
    win = [[(yc - 3, 5), (yc + 3, 5), (yc + 3, 18), (yc, 21), (yc - 3, 18)],
           [(yc - 3, -29), (yc + 3, -29), (yc + 3, -16), (yc, -13), (yc - 3, -16)]]
    for sx, fb in ((1, "F"), (-1, "B")):
        sketch(f"CrossSk{nm}{fb}", "YZ", f"XR + {sx}*(CB - PLATE_T/2)")
        poly([(y0, ZB), (y1, ZB), (y1, ZT), (y0, ZT)])
        for w in win:
            poly(w)
        cmd("sketch.finish")
        add("solid.extrude", name=f"Cross{nm}{fb}", distance="PLATE_T", direction="symmetric")
    box("CrossTop" + nm, "XR - CB", "XR + CB", y0, y1, f"{ZT} - DECK_T", ZT)
    box("CrossBot" + nm, "XR - CB", "XR + CB", y0, y1, ZB, f"{ZB} + DECK_T")
    box("CrossWeb" + nm, "XR - CB + PLATE_T", "XR + CB - PLATE_T", y0, y1, "-1.2", "1.2")
box("CrossStrap", "XR - CB", "XR + CB", "-WALL_IN", "WALL_IN", ZB, f"{ZB} + DECK_T")
sketch("SideRingSk", "YZ", "XR - CB")
for sy in (1, -1):
    ring_profile(f"{sy}*SY", "R_OUT", ZT, ZB)
cmd("sketch.finish")
add("solid.extrude", name="SideRings", distance="2*CB")
for sy, nm in ((1, "L"), (-1, "R")):
    box("Fin" + nm, "XR - CB", "XR - 2", f"{sy}*SY - 1.5", f"{sy}*SY + 1.5", f"{ZT} - 4", f"{ZT} + 7")
    for dx in (10.0, 5.0):
        box(f"FinHole{nm}{int(dx)}", f"XR - {dx + 1.6}", f"XR - {dx - 1.6}", f"{sy}*SY - 3", f"{sy}*SY + 3",
            f"{ZT} + 2.2", f"{ZT} + 4", op=cut)

# ---- camera shelf on the mast (gussets clipped at the box top)
box("CamShelf", "CAM_X0", "CAM_X1", "-CAM_W/2", "CAM_W/2", "CAM_Z", "CAM_Z + DECK_T")
x0, x1 = P.CAM_X
for i, xg in enumerate((x0 + 1.2, (x0 + x1) / 2, x1 - 1.2)):
    sketch(f"CamGussetSk{i}", "YZ", f"{xg}")
    poly([("-CAM_W/2", "CAM_Z"), ("CAM_W/2", "CAM_Z"), ("CAM_W/2 - (CAM_Z - BOX_TOP)", BT),
          ("-(CAM_W/2 - (CAM_Z - BOX_TOP))", BT)])
    cmd("sketch.finish")
    add("solid.extrude", name=f"CamGusset{i}", distance="2.4 mm", direction="symmetric")
for x in (x0 + 4.0, x1 - 4.0):
    for yy in (-12.0, 12.0):
        box(f"CamSlot{int(x)}{int(yy)}", f"{x - 1.2}", f"{x + 1.2}", f"{yy - 2.5}", f"{yy + 2.5}", "CAM_Z - 1",
            "CAM_Z + DECK_T + 1", op=cut)

# ---- tether post
T = "TOWER_TOP"
zc = "(TOWER_TOP - 12)"
box("PostWebX", "POST_X - 20", "POST_X + 20", "-2", "2", f"{BT} - 1", zc)
box("PostWebY", "POST_X - 1.2", "POST_X + 1.2", "-8", "8", BT, zc)
for sy in (1, -1):
    plate_xz(f"PostGusset{sy:+d}", f"{sy}*1.2", [("POST_X - 20", BT), ("POST_X + 20", BT), ("POST_X", f"{BT} + 14")], [],
             t="2.4 mm")
sketch("ClampSkA", "XZ", "0")
poly(xz([("POST_X - 4", f"{T} - 26"), ("POST_X + 4", f"{T} - 26"), ("POST_X + 20", f"{T} - 10"), ("POST_X + 20", T),
         ("POST_X - 20", T), ("POST_X - 20", f"{T} - 10")]))
cmd("sketch.finish")
cmd("solid.extrude", name="ClampA", distance="38 mm", direction="symmetric", operation="new", body_name="ClampA")
sketch("ClampSkB", "YZ", "POST_X")
poly([(-8, f"{T} - 26"), (8, f"{T} - 26"), (19, f"{T} - 15"), (19, T), (-19, T), (-19, f"{T} - 15")])
cmd("sketch.finish")
cmd("solid.extrude", name="ClampB", distance="60 mm", direction="symmetric", operation="new", body_name="ClampB")
cmd("solid.combine", target="ClampA", tools=["ClampB"], operation="intersect")
cmd("solid.combine", target="Frame", tools=["ClampA"], operation="join")

# ---- clearances
for sx, nm in ((1, "F"), (-1, "B")):
    cut("solid.cylinder", name="VBore" + nm, radius="R_IN", height="TOWER_TOP + 40 + R_OUT + 1",
        base=[f"{sx}*VX", 0, f"{ZB} - 1"], axis="Z")
sketch("SideBoreSk", "YZ", "XR - CB - 1")
for sy in (1, -1):
    ring_profile(f"{sy}*SY", "R_IN", f"{ZT} - 2")
cmd("sketch.finish")
cut("solid.extrude", name="SideBores", distance="2*CB + 2")
for sy, nm in ((1, "L"), (-1, "R")):
    cut("solid.cylinder", name="PodBody" + nm, radius="POD_OD/2 + 1 mm", height="130 - CB", base=["XR - 130", f"{sy}*SY", 0], axis="X")
    cut("solid.cylinder", name="Intake" + nm, radius="R_IN", height="60 mm", base=["XR + CB", f"{sy}*SY", 0], axis="X")
cut("solid.cylinder", name="TetherGroove", radius="7.5 mm", height="60 mm", base=["POST_X - 30", -5, T], axis="X")
cut("solid.cylinder", name="CamCableGroove", radius="4 mm", height="60 mm", base=["POST_X - 30", 12, T], axis="X")
for dx in (-12.0, 12.0):
    box(f"PostSlot{int(dx)}", f"POST_X + {dx - 2.5}", f"POST_X + {dx + 2.5}", "-25", "25", f"{T} - 13.4", f"{T} - 11", op=cut)

out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rov_v17_steps.json")
with open(out, "w") as fh:
    json.dump({"commands": steps}, fh, indent=0)
print(len(steps), "steps ->", out)
