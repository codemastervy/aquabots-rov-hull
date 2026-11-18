"""Writes solidcraft/rov_v16_steps.json: the SolidCraft build of the v16 frame (same geometry as src/rov.py frame(),
window corners left sharp). Rings, bores, rails, cross bar, decks, saddles, tray, post and shelf follow the
SolidCraft parameters (same names as src/params.py); the braces' planes and the window shapes are fixed numbers.

    python3 solidcraft/build_v16.py      then run the steps in SolidCraft (MCP batch with this file as `path`)
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
import params as P  # noqa: E402

S2 = math.sqrt(2.0)
R2 = 1 / S2
steps = []


def cmd(c, **p):
    steps.append({"command": c, "params": p})


def f(v):
    return round(float(v), 4)


PARAMS = [("R_IN", f"{P.RING_ID / 2} mm"), ("RING_T", f"{P.RING_T} mm"), ("R_OUT", "R_IN + RING_T"),
          ("RING_L", f"{P.RING_L} mm"), ("VX", f"{P.VX} mm"), ("SY", f"{P.SY} mm"), ("CB", f"{P.CB} mm"),
          ("PLATE_T", f"{P.PLATE_T} mm"), ("DECK_T", f"{P.DECK_T} mm"), ("FULL_X", f"{P.FULL_X} mm"),
          ("RAIL_Y", "R_OUT - 0.6 mm"), ("SADDLE_R", f"{P.SADDLE_R} mm"), ("SADDLE_T", "2.4 mm"),
          ("NY", f"{P.NY} mm"), ("NZ", "R_OUT + 1 mm + SADDLE_R"), ("POST_X", f"{P.POST_X} mm"),
          ("TOWER_TOP", f"{P.TOWER_TOP} mm"), ("POD_OD", f"{P.POD_OD} mm"),
          ("CAM_X0", f"{P.CAM_X[0]} mm"), ("CAM_X1", f"{P.CAM_X[1]} mm")]
for n, e in PARAMS:
    cmd("parameters.add", name=n, expression=e)

ZB, ZT, RT = "(-R_OUT)", "R_OUT", "(RING_L - R_OUT)"       # bottom, top, top of the up/down rings
first = [True]


def add(c, **p):
    """A solid that joins the frame (the first one makes it)."""
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


def xz(pts):           # (x, z) -> XZ sketch (x, -z)
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


# ---- rails
for sy, nm in ((1, "L"), (-1, "R")):
    half = truss(16.0) + [[(60, -29), (76, -29), (60, 10)]]
    plate_xz("Rail" + nm, f"{sy}*(RAIL_Y - PLATE_T/2)",
             [("-VX", ZB), ("VX", ZB), ("VX", RT), ("FULL_X", ZT), ("-FULL_X", ZT), ("-VX", RT)],
             half + mirror_u(half))

# ---- up/down rings (on the plate), each with 2 zip-tie lugs
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

# ---- braces: diamond plates from the cross bar out to the up/down rings (fixed planes)
px, py = P.VX - P.CB, -P.BRACE_Y
d = math.hypot(px, py)
a = math.atan2(py, px) + math.acos((P.R_OUT - 1.0) / d)
tx, ty = P.VX + (P.R_OUT - 1.0) * math.cos(a), -((P.R_OUT - 1.0) * math.sin(a))
L = math.hypot(tx - P.CB, ty - P.BRACE_Y)
ux, uy = (tx - P.CB) / L, (ty - P.BRACE_Y) / L
uf = (P.FULL_X - P.CB) / ux
b_out = [(-2, P.Z_BOT), (L, P.Z_BOT), (L, P.Z_BOT + P.RING_L), (uf, P.Z_TOP), (-2, P.Z_TOP)]
b_win = truss(5.0) + [[(uf - 1, -29), (uf + 16, -29), (uf - 1, 8)]]
for sx in (1, -1):
    for sy in (1, -1):
        o = [sx * P.CB, sy * P.BRACE_Y, 0.0]
        xd = [f(sx * ux), f(sy * uy), 0.0]
        sketch(f"BraceSk{sx:+d}{sy:+d}", {"origin": [f(v) for v in o], "x_dir": xd, "y_dir": [0, 0, 1]})
        poly([(f(u), f(v)) for u, v in b_out])
        for w in b_win:
            poly([(f(u), f(v)) for u, v in w])
        cmd("sketch.finish")
        add("solid.extrude", name=f"Brace{sx:+d}{sy:+d}", distance="PLATE_T", direction="symmetric")

# ---- cross bar: two plates out to the side rings, decks, mid web, cable tubes
cw = [[(4, 0), (16, -22), (28, 0), (16, 22)],
      [(38, 5), (51, 5), (51, 18), (44.5, 24.5), (38, 18)], [(38, -29), (51, -29), (51, -16), (44.5, -9.5), (38, -16)]]
cw = cw + mirror_u(cw)
for sx, nm in ((1, "F"), (-1, "B")):
    sketch("CrossSk" + nm, "YZ", f"{sx}*(CB - PLATE_T/2)")
    poly([("-SY", ZB), ("SY", ZB), ("SY", ZT), ("-SY", ZT)])
    for w in cw:
        poly(w)
    cmd("sketch.finish")
    add("solid.extrude", name="Cross" + nm, distance="PLATE_T", direction="symmetric")
box("TopDeck", "-CB", "CB", "-SY", "SY", f"{ZT} - DECK_T", ZT)
box("BottomDeck", "-CB", "CB", "-SY", "SY", ZB, f"{ZB} + DECK_T")
for sy, nm in ((1, "L"), (-1, "R")):
    box("Web" + nm, "-CB + PLATE_T", "CB - PLATE_T", "RAIL_Y" if sy > 0 else "-SY", "SY" if sy > 0 else "-RAIL_Y", "-1.2", "1.2")
    add("solid.cylinder", name="Tube" + nm, radius="10.6 mm", height="2*R_OUT", base=[0, f"{sy}*30", ZB], axis="Z")


# ---- side rings: round with a flat top and foot (v13's straight edges), one profile; top fin with 2 zip holes
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


sketch("SideRingSk", "YZ", "-CB")
for sy in (1, -1):
    ring_profile(f"{sy}*SY", "R_OUT", ZT, ZB)
cmd("sketch.finish")
add("solid.extrude", name="SideRings", distance="2*CB")
for sy, nm in ((1, "L"), (-1, "R")):
    box("Fin" + nm, "-CB", "-2", f"{sy}*SY - 1.5", f"{sy}*SY + 1.5", f"{ZT} - 4", f"{ZT} + 7")
    for x in (-10.0, -5.0):
        box(f"FinHole{nm}{int(-x)}", f"{x - 1.6}", f"{x + 1.6}", f"{sy}*SY - 3", f"{sy}*SY + 3", f"{ZT} + 2.2", f"{ZT} + 4", op=cut)

# ---- noodle saddles on the top deck
for sy, nm in ((1, "L"), (-1, "R")):
    y = f"{sy}*NY"
    ro = "(SADDLE_R + SADDLE_T)"
    zf = f"({ZT} - 0.5)"
    wf = f"{ro}*1.41421356 - (NZ - {zf})"
    sketch("SaddleSk" + nm, "YZ", "-CB")
    cmd("sketch.line", points=[[f"{y} - ({wf})", zf], [f"{y} + {wf}", zf], [f"{y} + {ro}*{R2}", f"NZ - {ro}*{R2}"]])
    cmd("sketch.arc.three_point", start=[f"{y} + {ro}*{R2}", f"NZ - {ro}*{R2}"], end=[f"{y} + {ro}", "NZ"],
        through=[f"{y} + {ro}*0.92387953", f"NZ - {ro}*0.38268343"])
    cmd("sketch.line", points=[[f"{y} + {ro}", "NZ"], [f"{y} + SADDLE_R", "NZ"]])
    cmd("sketch.arc.three_point", start=[f"{y} + SADDLE_R", "NZ"], end=[f"{y} - SADDLE_R", "NZ"], through=[y, "NZ - SADDLE_R"])
    cmd("sketch.line", points=[[f"{y} - SADDLE_R", "NZ"], [f"{y} - {ro}", "NZ"]])
    cmd("sketch.arc.three_point", start=[f"{y} - {ro}", "NZ"], end=[f"{y} - {ro}*{R2}", f"NZ - {ro}*{R2}"],
        through=[f"{y} - {ro}*0.92387953", f"NZ - {ro}*0.38268343"])
    cmd("sketch.line", points=[[f"{y} - {ro}*{R2}", f"NZ - {ro}*{R2}"], [f"{y} - ({wf})", zf]])
    cmd("sketch.finish")
    add("solid.extrude", name="Saddle" + nm, distance="2*CB")
    for side in (1, -1):
        for x in (-7.0, 7.0):
            yy0, yy1 = (f"{y} + {side}*(SADDLE_R - 2)", f"{y} + {side}*(SADDLE_R + 6)")
            box(f"SaddleZip{nm}{side:+d}{int(x):+d}", f"{x - 1.6}", f"{x + 1.6}", yy0, yy1, "NZ - 7", "NZ - 5.2", op=cut)

# ---- splice tray, camera shelf, tether post
xe = "(VX - R_IN)"
box("SpliceTray", f"-{xe}", xe, "-(RAIL_Y - 1)", "RAIL_Y - 1", ZB, f"{ZB} + DECK_T")
for x in (-50.0, -28.0, 28.0, 50.0):
    for yy in (-14.0, 14.0):
        box(f"TraySlot{int(x)}{int(yy)}", f"{x - 1.2}", f"{x + 1.2}", f"{yy - 2.5}", f"{yy + 2.5}", f"{ZB} - 1", f"{ZB} + 4", op=cut)
for x in (-40.0, 40.0):
    for yy in (-20.0, 0.0, 20.0):
        cut("solid.cylinder", name=f"Drain{int(x)}{int(yy)}", radius="3 mm", height="5 mm", base=[x, yy, f"{ZB} - 1"], axis="Z")

box("CamShelf", "CAM_X0", "CAM_X1", "-(RAIL_Y - 1)", "RAIL_Y - 1", f"{ZT} - DECK_T", ZT)
x0, x1 = P.CAM_X
for yy in (-16.0, 0.0, 16.0):
    plate_xz(f"CamRib{int(yy)}", f"{yy}", [("CAM_X0", ZB), ("CAM_X1", ZB), ("CAM_X1", f"{ZT} - DECK_T"), ("CAM_X0", f"{ZT} - DECK_T")],
             [[(x0 + 4, 0), (x0 + 15, -24), (x1 - 4, 0), (x0 + 15, 24)]], t="2.4 mm")
for x in (x0 + 4.0, x1 - 4.0):
    for yy in (-12.0, 12.0):
        box(f"CamSlot{int(x)}{int(yy)}", f"{x - 1.2}", f"{x + 1.2}", f"{yy - 2.5}", f"{yy + 2.5}", f"{ZT} - 4", f"{ZT} + 1", op=cut)

zc = "(TOWER_TOP - 12)"
box("PostWebX", "POST_X - 20", "-CB", "-2", "2", ZB, zc)
box("PostWebY", "POST_X - 1.2", "POST_X + 1.2", "-8", "8", ZB, zc)
T = "TOWER_TOP"
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

# ---- clearances for the pods, water flow and cables
for sx, nm in ((1, "F"), (-1, "B")):
    cut("solid.cylinder", name="VBore" + nm, radius="R_IN", height=f"TOWER_TOP + 40 + R_OUT + 1", base=[f"{sx}*VX", 0, f"{ZB} - 1"], axis="Z")
sketch("SideBoreSk", "YZ", "-CB - 1")
for sy in (1, -1):
    ring_profile(f"{sy}*SY", "R_IN", f"{ZT} - 2")
cmd("sketch.finish")
cut("solid.extrude", name="SideBores", distance="2*CB + 2")
for sy, nm in ((1, "L"), (-1, "R")):
    cut("solid.cylinder", name="PodBody" + nm, radius="POD_OD/2 + 1 mm", height="130 - CB", base=[-130, f"{sy}*SY", 0], axis="X")
    cut("solid.cylinder", name="Intake" + nm, radius="R_IN", height="60 mm", base=["CB", f"{sy}*SY", 0], axis="X")
    cut("solid.cylinder", name="TubeHole" + nm, radius="7.5 mm", height="2*R_OUT + 2", base=[0, f"{sy}*30", f"{ZB} - 1"], axis="Z")
cut("solid.cylinder", name="TetherGroove", radius="7.5 mm", height="60 mm", base=["POST_X - 30", -5, T], axis="X")
cut("solid.cylinder", name="CamCableGroove", radius="4 mm", height="60 mm", base=["POST_X - 30", 12, T], axis="X")
for dx in (-12.0, 12.0):
    box(f"PostSlot{int(dx)}", f"POST_X + {dx - 2.5}", f"POST_X + {dx + 2.5}", "-25", "25", f"{T} - 13.4", f"{T} - 11", op=cut)

out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rov_v16_steps.json")
with open(out, "w") as fh:
    json.dump({"commands": steps}, fh, indent=0)
print(len(steps), "steps ->", out)
