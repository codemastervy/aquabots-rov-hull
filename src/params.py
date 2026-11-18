"""Aqua Bots v16 "classic": the v13 frame shape, redone around our pods, pool noodles and wiring
(mm, X forward, Y left, Z up, origin = frame centre, Z_BOT = the build plate).

Thruster pod = the team's "motor caseing v1" print + modified SEAFLO SFBP1-G1100-01 motor and 55 mm prop.
Pod (from motor caseing v1.stl): 115 long; prop duct OD 64 at one end (25 long), a 75 OD ring next to it
(the shoulder the frame pushes on), 4 cage bars, then a 75 OD sleeve holding the pump motor.
Like v13, the frame grips each pod by its prop duct.
"""

# --- thruster pod (team's casing) ---
POD_L = 115.0
POD_OD = 75.0
DUCT_OD = 64.0
DUCT_L = 25.0
SLEEVE_ID = 67.0
PROP_D = 55.0

# --- rings that grip the prop ducts ---
RING_ID = DUCT_OD + 0.6    # 0.3 mm clearance all round: a snug slide fit, zip ties stop it backing out
RING_T = 2.4               # 6 lines of a 0.4 nozzle
RING_L = 25.0              # = duct length; the pod's 75 mm shoulder stops against the ring
R_IN = RING_ID / 2         # 32.3
R_OUT = R_IN + RING_T      # 34.7

# --- frame (v13 layout: 2 rails between the up/down rings, diamond braces, box cross bar to the side rings) ---
H = 2 * R_OUT              # frame height = side ring OD (69.4), like v13
Z_BOT = -R_OUT             # the build plate
Z_TOP = R_OUT
VX = 95.0                  # up/down rings at x = +-VX (on the plate; pod hangs below, prop duct up)
SY = 96.0                  # side rings at y = +-SY, z = 0 (prop duct forward, pod body behind)
SZ = 0.0
CB = 13.0                  # cross bar = side ring length: x = -13 .. 13
PLATE_T = 3.0              # rail / brace / cross bar plates
DECK_T = 2.4               # top and bottom plates
RAIL_Y = R_OUT - 0.6       # rails cross into the up/down rings' walls (an exact tangent makes a bad solid)
FULL_X = 62.0              # rails and braces are full height for |x| < FULL_X, then taper down onto the rings
BRACE_Y = 55.0             # diamond braces meet the cross bar at y = +-55

# --- pool noodles: two pieces along X, in saddles on top of the cross bar ---
NOODLE_D = 65.0            # typical 6-7 cm noodle; the saddle takes 55-75
NOODLE_HOLE = 20.0         # hollow core (0 for a solid noodle)
NOODLE_L = 126.0           # from analysis/sim.py (PETG frame; 124 for PLA); about 3 g of lift per mm per piece
SADDLE_R = 33.5            # saddle inside radius
NY = 55.0                  # noodle axis y
NZ = Z_TOP + 1.0 + SADDLE_R   # noodle axis z (69.2)

# --- wiring ---
TETHER_D = 14.0            # tether groove (fits up to 14 mm)
CAMCABLE_D = 7.0           # camera cable groove beside it
POST_X = -35.0             # tether post: rear, between the noodles
TOWER_TOP = 72.0           # clamp height

# --- camera shelf (top, front, between the noodles; looks forward over the front ring) ---
CAM_X = (22.0, 52.0)
CAM_W = 40.0
