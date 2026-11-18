"""Aqua Bots v15: open frame + pool-noodle floats (mm, X forward, Y left, Z up, origin = frame centre).

Thruster pod = the team's "motor caseing v1" print + modified SEAFLO SFBP1-G1100-01 motor and 55 mm prop.
Pod (from motor caseing v1.stl): 115 long; prop duct OD 64 at one end (25 long), a 75 OD ring next to it,
4 cage bars, then a 75 OD sleeve (30 long) holding the pump motor at the other end.
"""

# --- thruster pod (team's casing) ---
POD_L = 115.0
POD_OD = 75.0
DUCT_OD = 64.0
SLEEVE_ID = 67.0
PROP_D = 55.0

# --- pod bands (printed rings the pods slide into, held by one zip tie each) ---
BAND_ID = POD_OD + 1.0     # 0.5 mm clearance all round
BAND_T = 2.4               # 6 lines of a 0.4 nozzle
BAND_W = 14.0
LIP_R = 31.0               # stop lip: catches the sleeve's end rim (sleeve wall is r 33.5..37.5)
LIP_T = 1.6

# --- layout ---
VX = 62.0                  # vertical pods at x = +-VX, y = 0, duct up, z -57.5 .. 57.5
RAIL_Y0 = 40.0             # rails: plates y = +-(40 .. 43), touching the vertical bands
RAIL_T = 3.0
SY = RAIL_Y0 + RAIL_T + 1.0 + BAND_ID / 2      # side pods at y = +-82, prop duct forward
SZ = 0.0                   # side pod axis height = the drag centre height: it stays level when driving
RAIL_X = 105.0
PB = -POD_L / 2            # bottom (motor end) of the vertical pods
WIRE_GAP = 14.0            # skirt under each vertical pod: room for its wires to turn and leave sideways
CONE_H = 7.0               # 45 degree funnel under the stop lip (prints without support)
Z_BOT = PB - LIP_T - WIRE_GAP   # lowest point = the build plate (-73.1)
RAIL_TOP = 36.0

# --- pool noodles: two pieces, laid along X on saddles above the side pods ---
NOODLE_D = 65.0            # typical 6-7 cm noodle; the zip-tied saddle takes 55-75
NOODLE_HOLE = 20.0         # hollow core (0 for a solid noodle)
NOODLE_L = 126.0           # from analysis/sim.py (PETG frame; 125 for PLA). About 3 g of lift per mm per piece
SADDLE_R = 33.5            # saddle inside radius
NZ = SZ + POD_OD / 2 + 3.0 + NOODLE_D / 2        # noodle axis height (sits 3 mm above the side pod)

# --- wiring ---
TETHER_D = 14.0            # tether channel (fits up to 14 mm)
CAMCABLE_D = 7.0           # camera cable channel beside it
TOWER_TOP = 72.0           # tether clamp height: above the rear pod's duct, close to the floats' height
TRAY_H = 12.0              # splice tray: floor on the build plate between the vertical pods (low walls: less drag)

# --- camera shelf (front) ---
CAM_SHELF = (108.0, 144.0)  # x range
CAM_W = 40.0
CAM_Z = -6.0               # shelf top
