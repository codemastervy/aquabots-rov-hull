"""Aqua Bots v17 "v13 shape": our v13 frame's look - flat, solid, rings at the four ends - with the
pool-noodle floats hidden inside a centre box (mm, X forward, Y left, Z up, Z_BOT = the build plate).

Thruster pod = the team's "motor caseing v1" print + modified SEAFLO SFBP1-G1100-01 motor and 55 mm prop.
Pod: 115 long; prop duct OD 64 (25 long) at one end, a 75 OD ring next to it (the shoulder that stops
against the frame), 4 cage bars, a 75 OD sleeve holding the motor. Like v13 the frame grips the prop duct.
"""

# --- thruster pod (team's casing) ---
POD_L = 115.0
POD_OD = 75.0
DUCT_OD = 64.0
DUCT_L = 25.0
SLEEVE_ID = 67.0
PROP_D = 55.0

# --- rings that grip the prop ducts ---
RING_ID = DUCT_OD + 0.6    # 0.3 mm clearance all round
RING_T = 2.4
RING_L = 25.0
R_IN = RING_ID / 2         # 32.3
R_OUT = R_IN + RING_T      # 34.7

# --- frame ---
H = 2 * R_OUT              # frame height = side ring OD (69.4), like v13
Z_BOT = -R_OUT
Z_TOP = R_OUT
VX = 100.0                 # up/down rings at x = +-VX (on the plate, pod hangs below, prop duct up)
CB = 13.0                  # cross bar half width = side ring half length
PLATE_T = 3.0
DECK_T = 2.4

# --- centre box: the floats live here ---
NOODLE_D = 65.0            # standard pool noodle
NOODLE_HOLE = 20.0
NY = 35.0                  # two noodle pieces along X at y = +-35, z = 0 (a 2.4 mm spine between them)
NZ = 15.0                  # raised so the foam sits high: the box stands 13 mm above the deck (a low hump)
BOX_TOP = NZ + 65.0 / 2 + 0.6     # 48.1
XS = 68.0                  # box sides are straight for |x| < XS, then angle in to the up/down rings (v13's diamond)
WALL_IN = NY + NOODLE_D / 2 + 0.5     # 68: inside face of the box side walls
WALL_Y = WALL_IN + PLATE_T / 2        # wall centre line
SPINE_T = 2.4
LIP = 6.0                  # 45 deg lips along the top keep the noodles in (squeeze them in past the lips)
NOODLE_L = 124.0           # from analysis/sim.py (PETG; 123 for PLA)

# --- side rings (outside the box, clear of the side pod bodies) ---
XR = 52.0                  # cross bar / side rings sit this far forward so the trailing side pods balance over the floats
SY = WALL_IN + PLATE_T + POD_OD / 2 + 2.5     # 111
SZ = 0.0

# --- wiring and camera (on the spine, above the noodles) ---
TETHER_D = 14.0
CAMCABLE_D = 7.0
POST_X = -35.0             # tether post (rear, on the spine)
TOWER_TOP = BOX_TOP + 36.0
CAM_X = (36.0, 66.0)       # camera shelf on a mast at the front of the spine
CAM_Z = BOX_TOP + 16.0               # shelf underside
CAM_W = 40.0
RACK_TOP = BOX_TOP + 10.0  # cable rack: the spine stands 10 mm above the box between post and mast
