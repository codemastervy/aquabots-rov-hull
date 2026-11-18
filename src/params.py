"""Shared dimensions for the Aqua Bots v14 ROV hull (mm, X forward, Y left, Z up).

Thruster pod = the team's "motor caseing v1" print + a SEAFLO SFBP1-G1100-01 pump motor and 55 mm prop.
Measured from motor caseing v1.stl: duct OD 64 / ID 59, cage + sleeve OD 75, sleeve ID 67,
115 mm long (prop duct at one end, pump base in the sleeve at the other).
"""

# --- thruster pod (team's casing) ---
POD_L = 115.0          # casing length
POD_OD = 75.0          # cage / sleeve outside diameter
DUCT_OD = 64.0
DUCT_ID = 59.0
SLEEVE_ID = 67.0
PROP_D = 55.0

# --- hull ---
TOP_DECK = 15.0        # foam deck above the vertical pods (buoyancy as high as possible = strong self-righting)
H = POD_L + TOP_DECK   # hull height; vertical pods sit flush with the bottom
TX = 56.0              # vertical tunnels at x = +-TX (front / rear)
B = 45.0               # hull half width
NOSE_A = 69.0          # elliptical nose / tail, measured from the tunnel centre (length = 2*(TX+NOSE_A) = 250)
CH = 5.0               # 45 degree chamfer on the top and bottom edges
SKIN = 0.9             # hull skin: 2 lines of a 0.4 mm nozzle (backed by foam inside)
CAP_H = 10.0           # the cap is the top 10 mm of the hull (printed roof-down)
Z_SPLIT = H / 2 - CAP_H

TUN_ID = POD_OD + 1.0  # 0.5 mm clearance round the pod
TUN_WALL = 1.2

# --- side thruster nacelles ---
NAC_WALL = 1.6
NAC_RO = TUN_ID / 2 + NAC_WALL
NAC_Y = B + NAC_RO     # nacelle tube touches the hull side
NAC_X = 0.0            # nacelle centre; side pods face prop-forward so their heavy motors balance the camera
BOSS_NAC_R = 5.5       # bolt bosses joining nacelle to hull
BOLTS = [(NAC_X + sx * 28.0, sz * 20.0) for sx in (1, -1) for sz in (1, -1)]   # (x, z) of the M3 bolts (hull frame)

# --- fittings ---
BOSS_R = 3.0            # cap screw bosses in the tub (4 mm radius in the cap, where the screw head sits)
BOSSES = [(100.0, 20.0), (100.0, -20.0), (-100.0, 20.0), (-100.0, -20.0), (0.0, 36.0), (0.0, -36.0)]
CAM_D = 24.0           # camera bay bore (change to fit your camera)
CAM_Z = 20.0
CAM_DEPTH = 26.0
TETHER_D = 10.0
TETHER_Z = 20.0
BALLAST_H = 14.0       # bottom 12 mm of the hull = ballast bays (steel nuts / washers / fishing weights)

# --- foam ---
FOAM_Z0 = -H / 2 + SKIN + BALLAST_H   # foam fills the hull above the ballast bays
