"""Fusion 360 generator: race-car chassis for the line follower.

Run it in Fusion (Utilities > Scripts and Add-Ins) or through the
Fusion360MCP add-in (execute_code). It opens a new design and builds:

  Tray          low wedge tub: windowed floor + short walls (switch, USB and IR
                cable notches); the 12 x 8 cm perfboard sits on brass standoffs
                and the two N20 mounts bolt underneath into nuts trapped in the floor
  N20 mount     printed twice: clamps the motor against the floor underside;
                pocket copied from N20_Gear_Motor_Mount_-_Full_Length (gearbox
                stop + rounded can section), screw heads sunk into the flanges
  Deck          I-shaped plate floating on brass spacers above the perfboard;
                low wedge hood with chevron vents, battery strapped on top
  Rear wing     sits on two 15 mm brass posts on the rear deck studs
  Sensor boom   slides under the tray floor (2 slots, nuts trapped in the floor)
                to set the lookahead; carries the IR array on brass spacers and
                the ball caster
  Caster riser  only for a caster shorter than the boom underside; a taller one
                sits in a pocket in the boom
  References    wheels, motors, perfboard, parts envelope, IR array,
                battery, caster, spacers: do not print

Everything is modelled in place in the robot frame: X forward, Y left, Z up,
origin on the ground under the middle of the rear axle. Dimensions in mm; the
Fusion API uses cm (MM). Print orientation is set in the slicer: tray and deck
as modelled, boom, rear wing and N20 mounts upside down. None of them need
supports.

Hardware: M3 x 6 screws, M3 nuts and the brass standoff kit only (no inserts).
  board + deck  per corner: M3 x 6 up through the floor into an M-F 6, its stud
                through the board into an M-F 20, then an M-F 6 whose stud goes
                through the deck: a nut on the front pair, a F-F 15 wing post on
                the rear pair (a nut if the wing is left off)
  rear wing     M3 x 6 down through each pad into the F-F 15 posts
  N20 mounts    4x M3 x 6 up through the mount flanges (heads sunk 1.5 mm) into
                nuts in the floor
  boom          2x M3 x 6 up through the slots (heads in a recess) into floor nuts;
                every nut joint leaves THREAD_SPARE of screw past the nut
  IR array      2x M-F 10: stud up through the boom into a nut in a hex recess,
                M3 x 6 up through the IR PCB into the standoff
  caster        2x M3 x 6 down through the boom slot and the plate into nuts in
                the caster's own nut slots
  total         16 screws, 12 nuts, 8 M-F 6, 4 M-F 20, 2 M-F 10, 2 F-F 15
"""

import math
import os

import adsk.core
import adsk.fusion

MM = 0.1

# --- Wheels, N20 motors and their mounts ---------------------------------------
WHEEL_D = 33.3
WHEEL_W = 6.2
WHEEL_GAP = 1.5          # tray side wall to the inside face of the tyre
MOTOR_W = 12.0           # gearbox 12 x 10; can 12 across the round sides, 10 across the flats
MOTOR_H = 10.0
GEARBOX_L = 9.0
CAN_L = 15.0             # reference only
SHAFT_L = 10.0           # reference only, from the gearbox face
MOTOR_FIT = 0.2
MOUNT_AXIS_DEPTH = MOTOR_H / 2  # motor axis below the floor: its top flat bears on the floor
MOUNT_WALL = 2.0
MOUNT_DEPTH = MOTOR_H + MOUNT_WALL  # mount hangs this far below the floor
MOUNT_LEN = 26.0         # along the motor axis; outer end flush with the tray side
MOUNT_LIP = 1.0          # outer end wall the gearbox face rests against, shaft slot through it
SHAFT_SLOT_W = 5.0       # takes the 4 mm gearbox bushing
MOUNT_FLANGE_X = 18.0    # flanges reach this far fore and aft of the axle
MOUNT_FLANGE_L = 16.0    # along the motor axis, from the outer end
MOUNT_FLANGE_T = 3.0
MOUNT_HOLE_X = 13.0
MOUNT_HOLE_IN = 9.0      # flange holes, in from the outer end
MOUNT_HEAD_SINK = 1.5    # screw heads sink into the flange so an M3 x 6 has thread to spare

# --- Perfboard and electronics ----------------------------------------------
BOARD_L = 120.0          # along X
BOARD_W = 80.0
BOARD_T = 1.6
BOARD_HOLE_DX = 113.5    # hole centres, long side
BOARD_HOLE_DY = 73.5     # hole centres, short side
BOARD_STANDOFF = 6.0     # M-F 6 on the floor -> board underside: room for solder joints
TALLEST = 20.0           # tallest part above the board
DECK_SPACER = 26.0       # M-F 20 + M-F 6 stack, board top -> deck
USB_FROM_FRONT = (41.0, 64.0)  # ESP32 USB-C ports (left side), from the board's front edge

# --- Tray and deck ----------------------------------------------------------
FLOOR_T = 2.5
WALL_T = 2.0
BOARD_CLEAR = 1.0        # board edge to wall
CORNER_R = 5.0           # inside corner = CORNER_R - WALL_T, must clear the square board corners
DECK_T = 2.5
TUB_H = (11.0, 4.0)      # wall top above the board top at the rear and front: a low wedge,
                         # open between the walls and the deck
SWITCH_W = 13.0          # 12.7 x 6.5 slide switch, press fit in the rear wall
SWITCH_H = 6.8
SWITCH_Z = 1.5           # window bottom above the board top
IR_CABLE_NOTCH = 24.0    # front wall notch for the IR cable, down to the board top + 1

# --- Weight: lightening windows (high-rpm motors have little torque to spare) ----
WINDOW_R = 4.0
# Floor (x0, y0, x1, y1): clear of the bosses, the motor mounts (X +-18) and the
# boom's bearing strips (|Y| 10..15)
FLOOR_WINDOWS = [(24.0, 19.0, 84.0, 37.5), (24.0, -37.5, 84.0, -19.0),
                 (-16.0, -11.0, 26.0, 11.0), (41.0, -10.0, 79.0, 10.0)]
DECK_BEAMS = (-12.0, 86.0)  # deck is full width behind / ahead of these X (spacer screws, wing)
DECK_SPINE_W = 47.0         # between them: battery, strap slots and hood
BOOM_WINDOWS = (4.5, 11.0)  # |Y| range beside the slots, along the narrow part of the boom

# --- Race-car styling -------------------------------------------------------
HOOD_X = (57.0, 94.0)    # wedge hood on the deck ahead of the battery: rear, front
HOOD_W = (36.0, 20.0)    # width at rear, front
HOOD_H = (7.0, 1.0)      # height above the deck at rear, front (solid, so kept low)
LOUVRES = 4              # chevron vents through the hood
LOUVRE_X = 88.0          # front apex
LOUVRE_PITCH = 6.5
LOUVRE_W = 2.6
LOUVRE_SWEEP = 10.0      # apex to arm tips, along X
LOUVRE_SPAN = 22.0

# --- Battery (strapped on the deck) ------------------------------------------
BATT_L = 56.0            # along X
BATT_W = 30.0
BATT_H = 20.0            # reference only
BATT_X = 26.0            # centre, ahead of the axle
RAIL_T = 1.2
RAIL_H = 1.5
STRAP_W = 22.0           # 20 mm velcro strap
STRAP_T = 3.0
CABLE_HOLE = (-16.0, -20.0, -3.0, -6.0)  # x0, y0, x1, y1: battery lead down to the XT30

# --- Rear wing (on F-F 15 brass posts screwed onto the rear deck studs) --------
WING_SPAN = 92.0
WING_X = (-33.0, -12.0)  # trailing and leading edge
WING_T = 3.0             # thickest point of the profile
WING_POST = 15.0
WING_PAD_T = 3.0         # wing thickness at the post screws: M3 x 6 -> 3 mm into the post
WING_PAD_D = 8.0
ENDPLATE_DROP = 12.0

# --- Sensor boom ------------------------------------------------------------
BOOM_T = 4.0
BOOM_W = 30.0
TIP_W = 58.0             # caster plate pocket + 3.6 mm side walls
BOOM_FLOOR_X = (35.0, 85.0)  # fixed screws in the tray floor
BOOM_TRAVEL = 40.0       # lookahead adjustment range
SLOT_MARGIN = 5.0        # boom rear end to the first slot
LOOKAHEAD = 150.0        # axle to the sensor eyes, middle of the travel
SENSOR_L = 64.0          # IR array, across the robot
SENSOR_W = 17.0
SENSOR_T = 1.6
SENSOR_HOLE_DY = 35.5
SENSOR_HOLE_BACK = 4.0   # hole line (and header) in from the rear edge
SENSOR_EYES_BACK = 8.5   # eye line in from the rear edge
SENSOR_EYE_H = 2.5       # LEDs / phototransistors below the PCB: MEASURE
SENSOR_FACE_Z = 3.0      # eyes above the track
SENSOR_SPACER = 10.0     # M-F 10, boom -> sensor PCB, stud up through the boom
HEADER_SLOT = (29.0, 4.5)  # 11-pin header + Dupont plug through the boom tip (Y, X)
TIP_MARGIN = 8.0         # boom tip beyond the sensor holes

# --- Ball caster (25 x 50 mm plate) ------------------------------------------
CASTER_PLATE_L = 25.0    # along X
CASTER_PLATE_W = 50.0
CASTER_H = 19.6          # plate face to the floor (measured)
CASTER_FIT = 0.4         # plate to pocket wall
CASTER_PLATE_T = 2.7     # measured
CASTER_NUT_SLOT = 1.2    # nut slots under the plate (measured): screws come down from the boom top
CASTER_HOLE_DY = 38.0    # riser only (PLACEHOLDER); the boom slot takes any spacing
CASTER_SLOT_DY = 44.0    # slot in the boom takes any hole spacing up to this
CASTER_U = 112.0         # caster centre, from the boom's rear end

# --- Fasteners: M3 x 6 screws only, so every clamped stack is kept thin ----------
# With a nut: stack + NUT_H <= SCREW_L. Into a standoff: SCREW_L - stack >= MIN_THREAD.
M3 = 3.4
HEAD_D = 5.5
SCREW_L = 6.0
STUD_L = 6.0             # M-F standoff stud
NUT_AF = 5.7             # trap, nut is 5.5
NUT_H = 2.4
NUT_WEB = 0.6            # floor left under a boom nut; the boom backs it
MOUNT_NUT_WEB = 1.2      # floor left under a mount nut
NUT_BOSS_D = 10.0
NUT_BOSS_H = 1.5         # above the floor top, so the trap holds the whole nut
MIN_THREAD = 3.0
THREAD_SPARE = 0.6       # screw or stud left past a nut, for print tolerance
SPACER_D = 5.5

# --- Derived ----------------------------------------------------------------
AXLE_Z = WHEEL_D / 2
Z_FB = AXLE_Z + MOUNT_AXIS_DEPTH   # floor underside = mount face
Z_FT = Z_FB + FLOOR_T
Z_BB = Z_FT + BOARD_STANDOFF       # board underside
Z_BT = Z_BB + BOARD_T
Z_DU = Z_BT + DECK_SPACER          # deck underside
Z_DT = Z_DU + DECK_T
TUB_TOP = (Z_BT + TUB_H[0], Z_BT + TUB_H[1])  # rear, front
WING_Z = Z_DT + WING_POST + WING_PAD_T        # top face; prints face down

HALF_W = BOARD_W / 2 + BOARD_CLEAR + WALL_T
MOUNT_Y = HALF_W - MOUNT_LEN / 2   # mount centre; shaft end flush with the side
WHEEL_Y = HALF_W + WHEEL_GAP + WHEEL_W / 2

# Rear board screws go behind the motor flanges so their heads clear them.
BOARD_HOLE_REAR_X = -(MOUNT_FLANGE_X + HEAD_D / 2 + 1.0)
BOARD_X0 = BOARD_HOLE_REAR_X - (BOARD_L - BOARD_HOLE_DX) / 2
BOARD_X1 = BOARD_X0 + BOARD_L
TRAY_X0 = BOARD_X0 - BOARD_CLEAR - WALL_T
TRAY_X1 = BOARD_X1 + BOARD_CLEAR + WALL_T

BOARD_HOLES = [(BOARD_HOLE_REAR_X + dx, sy * BOARD_HOLE_DY / 2)
               for dx in (0.0, BOARD_HOLE_DX) for sy in (-1, 1)]
MOUNT_HOLES = [(sx * MOUNT_HOLE_X, sy * (HALF_W - MOUNT_HOLE_IN))
               for sx in (-1, 1) for sy in (-1, 1)]

Z_BOOM_B = Z_FB - BOOM_T
BOOM_REAR_X = BOOM_FLOOR_X[0] - (SLOT_MARGIN + BOOM_TRAVEL / 2)  # middle of the travel
SLOT_SPACING = BOOM_FLOOR_X[1] - BOOM_FLOOR_X[0]
SENSOR_REAR_X = LOOKAHEAD - SENSOR_EYES_BACK
U_SENSOR = SENSOR_REAR_X + SENSOR_HOLE_BACK - BOOM_REAR_X
BOOM_LEN = U_SENSOR + TIP_MARGIN
U_TIP = CASTER_U - CASTER_PLATE_L / 2 - 2.0   # boom is TIP_W wide from here
SENSOR_PCB_TOP = SENSOR_FACE_Z + SENSOR_EYE_H + SENSOR_T
SENSOR_PAD = Z_BOOM_B - SENSOR_PCB_TOP - SENSOR_SPACER
RISER_H = Z_BOOM_B - CASTER_H        # > 0: printed riser under the boom
CASTER_POCKET = max(0.0, -RISER_H)   # > 0: caster plate sinks this far into the boom
BOOM_RECESS = BOOM_T - (SCREW_L - THREAD_SPARE - NUT_H - NUT_WEB)  # slot screw heads sink this far
SENSOR_NUT_DEPTH = BOOM_T + max(SENSOR_PAD, 0.0) + NUT_H - STUD_L + THREAD_SPARE
CASTER_CHANNEL = max(0.0, BOOM_T - CASTER_POCKET + CASTER_PLATE_T - CASTER_NUT_SLOT + NUT_H
                     - SCREW_L + THREAD_SPARE)  # > 0: caster screw heads sink this far into the boom top


# ===========================================================================
# Helpers
# ===========================================================================

def P(x, y, z=0.0):
    return adsk.core.Point3D.create(x * MM, y * MM, z * MM)


def new_component(root, name, matrix=None):
    occ = root.occurrences.addNewComponent(matrix or adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ.component


def sketch_at(comp, z, name):
    inp = comp.constructionPlanes.createInput()
    inp.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(z * MM))
    sk = comp.sketches.add(comp.constructionPlanes.add(inp))
    sk.name = name
    return sk


def side_sketch(comp, axis, at, name):
    """Sketch on the plane X = at (axis 'x') or Y = at (axis 'y')."""
    base = comp.yZConstructionPlane if axis == "x" else comp.xZConstructionPlane
    for sign in (1, -1):  # the base plane normal may point either way
        inp = comp.constructionPlanes.createInput()
        inp.setByOffset(base, adsk.core.ValueInput.createByReal(sign * at * MM))
        plane = comp.constructionPlanes.add(inp)
        o = plane.geometry.origin
        if abs((o.x if axis == "x" else o.y) - at * MM) < 1e-6:
            sk = comp.sketches.add(plane)
            sk.name = name
            return sk
        plane.deleteMe()
    raise RuntimeError("side plane " + name)


def rect(sk, x0, y0, x1, y1):
    sk.sketchCurves.sketchLines.addTwoPointRectangle(P(x0, y0), P(x1, y1))


def circle(sk, x, y, d):
    sk.sketchCurves.sketchCircles.addByCenterRadius(P(x, y), d / 2 * MM)


def polyline(sk, pts):
    """Closed polygon through sketch-space points."""
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(pts[0], pts[1])
    prev = first
    for p in pts[2:]:
        prev = lines.addByTwoPoints(prev.endSketchPoint, p)
    lines.addByTwoPoints(prev.endSketchPoint, first.startSketchPoint)


def polygon(sk, pts):
    polyline(sk, [P(*p) for p in pts])


def side_polygon(comp, axis, at, pts, name):
    """Polygon on the plane X = at (axis 'x', pts are (y, z)) or Y = at (pts are (x, z))."""
    sk = side_sketch(comp, axis, at, name)
    polyline(sk, [sk.modelToSketchSpace(P(at, a, z) if axis == "x" else P(a, at, z))
                  for a, z in pts])
    return sk


def hexagon(sk, x, y, af):
    r = af / math.sqrt(3)
    polygon(sk, [(x + r * math.cos(a), y + r * math.sin(a))
                 for a in (math.radians(60 * i) for i in range(6))])


def rounded_rect(sk, x0, y0, x1, y1, r):
    lines = sk.sketchCurves.sketchLines
    lines.addByTwoPoints(P(x0 + r, y0), P(x1 - r, y0))
    lines.addByTwoPoints(P(x1, y0 + r), P(x1, y1 - r))
    lines.addByTwoPoints(P(x1 - r, y1), P(x0 + r, y1))
    lines.addByTwoPoints(P(x0, y1 - r), P(x0, y0 + r))
    arcs = sk.sketchCurves.sketchArcs
    for cx, cy, sx, sy in ((x1 - r, y0 + r, x1 - r, y0), (x1 - r, y1 - r, x1, y1 - r),
                           (x0 + r, y1 - r, x0 + r, y1), (x0 + r, y0 + r, x0, y0 + r)):
        arcs.addByCenterStartSweep(P(cx, cy), P(sx, sy), math.pi / 2)


def slot(sk, xa, ya, xb, yb, w):
    """Obround from (xa, ya) to (xb, yb), axis-aligned."""
    if abs(ya - yb) < 1e-9:
        rect(sk, xa, ya - w / 2, xb, ya + w / 2)
    else:
        rect(sk, xa - w / 2, ya, xa + w / 2, yb)
    circle(sk, xa, ya, w)
    circle(sk, xb, yb, w)


def profile(sk):
    profs = [sk.profiles.item(i) for i in range(sk.profiles.count)]
    return max(profs, key=lambda p: p.areaProperties().area)


def all_profiles(sk):
    coll = adsk.core.ObjectCollection.create()
    for i in range(sk.profiles.count):
        coll.add(sk.profiles.item(i))
    return coll


NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation


def own_bodies(inp, comp, op):
    """By default a cut also reaches into other components' bodies; keep it in its own."""
    if op == CUT:
        inp.participantBodies = [comp.bRepBodies.item(i) for i in range(comp.bRepBodies.count)]


def extrude(comp, profs, height, op):
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(profs, op)
    inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal(height * MM))
    own_bodies(inp, comp, op)
    return ext.add(inp)


def extrude_symmetric(comp, profs, total, op):
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(profs, op)
    inp.setSymmetricExtent(adsk.core.ValueInput.createByReal(total * MM), True)
    own_bodies(inp, comp, op)
    return ext.add(inp)


def box(comp, x0, y0, z0, x1, y1, z1, op, name):
    sk = sketch_at(comp, z0, name)
    rect(sk, x0, y0, x1, y1)
    return extrude(comp, profile(sk), z1 - z0, op)


def wall_window(comp, axis, at, a0, a1, z0, z1, name):
    """Window through a wall at X = at (axis 'x', spans Y a0..a1) or Y = at (spans X)."""
    sk = side_sketch(comp, axis, at, name)
    c0 = P(at, a0, z0) if axis == "x" else P(a0, at, z0)
    c1 = P(at, a1, z1) if axis == "x" else P(a1, at, z1)
    sk.sketchCurves.sketchLines.addTwoPointRectangle(sk.modelToSketchSpace(c0),
                                                     sk.modelToSketchSpace(c1))
    return extrude_symmetric(comp, all_profiles(sk), WALL_T + 2.0, CUT)


def side_cylinder(comp, y, width, x, z, d, name):
    sk = side_sketch(comp, "y", y, name)
    sk.sketchCurves.sketchCircles.addByCenterRadius(sk.modelToSketchSpace(P(x, y, z)), d / 2 * MM)
    return extrude_symmetric(comp, profile(sk), width, NEW)


def cylinders(comp, pts, d, z0, z1, op, name):
    sk = sketch_at(comp, z0, name)
    for x, y in pts:
        circle(sk, x, y, d)
    return extrude(comp, all_profiles(sk), z1 - z0, op)


# ===========================================================================
# Printed parts
# ===========================================================================

def build_tray(root):
    comp = new_component(root, "Tray")
    zr, zf = TUB_TOP
    sk = sketch_at(comp, Z_FB, "outline")
    rounded_rect(sk, TRAY_X0, -HALF_W, TRAY_X1, HALF_W, CORNER_R)
    extrude(comp, profile(sk), zr - Z_FB, NEW)
    sk = sketch_at(comp, Z_FT, "inside")
    rounded_rect(sk, TRAY_X0 + WALL_T, -HALF_W + WALL_T, TRAY_X1 - WALL_T, HALF_W - WALL_T,
                 CORNER_R - WALL_T)
    extrude(comp, profile(sk), zr - Z_FT + 1.0, CUT)
    # Wedge: the wall top drops from the rear to the front.
    x0, x1 = TRAY_X0 - 1.0, TRAY_X1 + 1.0
    z_at = lambda x: zr + (zf - zr) * (x - TRAY_X0) / (TRAY_X1 - TRAY_X0)
    sk = side_polygon(comp, "y", 0.0, [(x0, z_at(x0)), (x1, z_at(x1)), (x1, zr + 1.0),
                                       (x0, zr + 1.0)], "wedge")
    extrude_symmetric(comp, profile(sk), 2 * HALF_W + 2.0, CUT)

    # The board stands on M-F 6 standoffs screwed up through the plain floor. Mount
    # and boom screws come up from below into nuts sunk to a thin web, so M3 x 6 reach.
    boom_pts = [(x, 0.0) for x in BOOM_FLOOR_X]
    nut_pts = MOUNT_HOLES + boom_pts
    cylinders(comp, nut_pts, NUT_BOSS_D, Z_FT, Z_FT + NUT_BOSS_H, JOIN, "nut bosses")
    for pts, web, name in ((MOUNT_HOLES, MOUNT_NUT_WEB, "mount nut traps"),
                           (boom_pts, NUT_WEB, "boom nut traps")):
        sk = sketch_at(comp, Z_FB + web, name)
        for x, y in pts:
            hexagon(sk, x, y, NUT_AF)
        extrude(comp, all_profiles(sk), FLOOR_T + NUT_BOSS_H - web + 1.0, CUT)
    cylinders(comp, BOARD_HOLES + nut_pts, M3, Z_FB - 1.0, Z_FT + NUT_BOSS_H + 1.0, CUT,
              "screw holes")

    # Motor wires come up beside the motor's rear end.
    sk = sketch_at(comp, Z_FB - 1.0, "motor wire slots")
    for sy in (-1, 1):
        y_in = MOUNT_Y - MOUNT_LEN / 2
        rect(sk, -4.0, sy * (y_in - 1.5), 4.0, sy * (y_in + 4.0))
    extrude(comp, all_profiles(sk), FLOOR_T + 2.0, CUT)
    sk = sketch_at(comp, Z_FB - 1.0, "floor windows")
    for w in FLOOR_WINDOWS:
        rounded_rect(sk, *w, WINDOW_R)
    extrude(comp, all_profiles(sk), FLOOR_T + 2.0, CUT)

    wall_window(comp, "y", HALF_W - WALL_T / 2,
                BOARD_X1 - USB_FROM_FRONT[1] - 3.0, BOARD_X1 - USB_FROM_FRONT[0] + 3.0,
                Z_BT + 1.0, zr + 1.0, "USB notch")
    wall_window(comp, "x", TRAY_X0 + WALL_T / 2, -SWITCH_W / 2, SWITCH_W / 2,
                Z_BT + SWITCH_Z, Z_BT + SWITCH_Z + SWITCH_H, "switch")
    wall_window(comp, "x", TRAY_X1 - WALL_T / 2, -IR_CABLE_NOTCH / 2, IR_CABLE_NOTCH / 2,
                Z_BT + 1.0, zr + 1.0, "IR cable notch")
    return comp


def intersect(comp, sk, bodies, total):
    """Trim `bodies` to the symmetric extrusion of every profile in `sk`."""
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(all_profiles(sk), adsk.fusion.FeatureOperations.IntersectFeatureOperation)
    inp.setSymmetricExtent(adsk.core.ValueInput.createByReal(total * MM), True)
    inp.participantBodies = bodies
    ext.add(inp)


def join(comp, body, tools):
    coll = adsk.core.ObjectCollection.create()
    for b in tools:
        coll.add(b)
    inp = comp.features.combineFeatures.createInput(body, coll)
    inp.operation = JOIN
    comp.features.combineFeatures.add(inp)


def build_deck(root):
    comp = new_component(root, "Deck")
    # Floats on the spacers, same outline as the tray; flat underside prints the right way up.
    sk = sketch_at(comp, Z_DU, "outline")
    rounded_rect(sk, TRAY_X0, -HALF_W, TRAY_X1, HALF_W, CORNER_R)
    body = extrude(comp, profile(sk), DECK_T, NEW).bodies.item(0)
    # Side cut-outs leave an I shape: full-width beams at the spacers, a spine between.
    sk = sketch_at(comp, Z_DU - 1.0, "side cut-outs")
    xa, xb = DECK_BEAMS
    rounded_rect(sk, xa, DECK_SPINE_W / 2, xb, HALF_W + 5.0, WINDOW_R)
    rounded_rect(sk, xa, -HALF_W - 5.0, xb, -DECK_SPINE_W / 2, WINDOW_R)
    extrude(comp, all_profiles(sk), DECK_T + 2.0, CUT)

    (x0, x1), (w0, w1), (h0, h1) = HOOD_X, HOOD_W, HOOD_H
    sk = sketch_at(comp, Z_DT, "hood plan")
    polygon(sk, [(x0, -w0 / 2), (x1, -w1 / 2), (x1, w1 / 2), (x0, w0 / 2)])
    hood = extrude(comp, profile(sk), h0 + 1.0, NEW).bodies.item(0)
    sk = side_polygon(comp, "y", 0.0, [(x0, Z_DT - 1.0), (x1 + 1.0, Z_DT - 1.0),
                                       (x1 + 1.0, Z_DT + h1), (x0, Z_DT + h0)], "hood side")
    intersect(comp, sk, [hood], w0 + 10.0)
    join(comp, body, [hood])

    sk = sketch_at(comp, Z_DU - 1.0, "holes and vents")
    for x, y in BOARD_HOLES:
        circle(sk, x, y, M3)
    y_s = BATT_W / 2 + 0.3 + RAIL_T + 1.0
    for sy in (-1, 1):
        rect(sk, BATT_X - STRAP_W / 2, sy * y_s, BATT_X + STRAP_W / 2, sy * (y_s + STRAP_T))
    rect(sk, *CABLE_HOLE)
    for i in range(LOUVRES):
        xa, xb = LOUVRE_X - i * LOUVRE_PITCH, LOUVRE_X - i * LOUVRE_PITCH - LOUVRE_SWEEP
        y = LOUVRE_SPAN / 2
        polygon(sk, [(xa, 0.0), (xb, y), (xb - LOUVRE_W, y), (xa - LOUVRE_W, 0.0),
                     (xb - LOUVRE_W, -y), (xb, -y)])
    extrude(comp, all_profiles(sk), DECK_T + HOOD_H[0] + 2.0, CUT)

    # Battery rails; the rear end stays open for the lead.
    sk = sketch_at(comp, Z_DT, "battery rails")
    xa, xb = BATT_X - BATT_L / 2 - 0.3, BATT_X + BATT_L / 2 + 0.3
    yb = BATT_W / 2 + 0.3
    for sy in (-1, 1):
        rect(sk, xa, sy * yb, xb, sy * (yb + RAIL_T))
    rect(sk, xb, -BATT_W / 4, xb + RAIL_T, BATT_W / 4)
    extrude(comp, all_profiles(sk), RAIL_H, JOIN)
    return comp


def build_wing(root):
    comp = new_component(root, "Rear wing")
    xt, xl = WING_X
    z1 = WING_Z
    # Flat top (the print-bed face), thicker toward the leading edge.
    sk = side_polygon(comp, "y", 0.0, [(xl, z1 - 1.0), (xl, z1), (xt, z1), (xt, z1 - 1.2),
                                       (xl - 4.0, z1 - WING_T)], "profile")
    extrude_symmetric(comp, profile(sk), WING_SPAN, NEW)
    for sy in (-1, 1):
        sk = side_polygon(comp, "y", sy * (WING_SPAN / 2 - 1.0),
                          [(xl + 1.0, z1), (xt - 1.0, z1), (xt - 1.0, z1 - ENDPLATE_DROP),
                           (xl - 4.0, z1 - ENDPLATE_DROP + 4.0)], "endplate")
        extrude_symmetric(comp, profile(sk), 2.0, JOIN)

    # Flat pads sit on the brass posts; an M3 x 6 drops through each pad into its post.
    pts = [(BOARD_HOLE_REAR_X, sy * BOARD_HOLE_DY / 2) for sy in (-1, 1)]
    cylinders(comp, pts, WING_PAD_D, z1 - WING_PAD_T, z1 - 0.5, JOIN, "pads")
    cylinders(comp, pts, M3, z1 - WING_PAD_T - 1.0, z1 + 1.0, CUT, "screw holes")
    return comp


def bx(u):
    """Boom coordinate (from its rear end) -> robot X, at the middle of the travel."""
    return BOOM_REAR_X + u


def build_boom(root):
    comp = new_component(root, "Sensor boom")
    sk = sketch_at(comp, Z_BOOM_B, "outline")
    polygon(sk, [(bx(0), -BOOM_W / 2), (bx(U_TIP - 8.0), -BOOM_W / 2), (bx(U_TIP), -TIP_W / 2),
                 (bx(BOOM_LEN), -TIP_W / 2), (bx(BOOM_LEN), TIP_W / 2), (bx(U_TIP), TIP_W / 2),
                 (bx(U_TIP - 8.0), BOOM_W / 2), (bx(0), BOOM_W / 2)])
    extrude(comp, profile(sk), BOOM_T, NEW)

    sensor_pts = [(bx(U_SENSOR), sy * SENSOR_HOLE_DY / 2) for sy in (-1, 1)]
    z_cut = Z_BOOM_B
    if SENSOR_PAD > 0.3:
        cylinders(comp, sensor_pts, SPACER_D + 2.0, Z_BOOM_B - SENSOR_PAD, Z_BOOM_B, JOIN,
                  "sensor pads")
        z_cut = Z_BOOM_B - SENSOR_PAD

    if CASTER_POCKET > 0.0:
        x0 = bx(CASTER_U) - CASTER_PLATE_L / 2 - CASTER_FIT
        y0 = CASTER_PLATE_W / 2 + CASTER_FIT
        box(comp, x0, -y0, Z_BOOM_B, x0 + CASTER_PLATE_L + 2 * CASTER_FIT, y0,
            Z_BOOM_B + CASTER_POCKET, CUT, "caster pocket")

    # Thin spots so M3 x 6 reach: slot screw heads sink into the underside, the
    # sensor standoff nuts (and caster screw heads, if needed) into the top.
    sk = sketch_at(comp, Z_BOOM_B - 1.0, "slot head recesses")
    for i in range(2):
        u0 = SLOT_MARGIN + i * SLOT_SPACING
        slot(sk, bx(u0), 0.0, bx(u0 + BOOM_TRAVEL), 0.0, HEAD_D + 1.0)
    extrude(comp, all_profiles(sk), BOOM_RECESS + 1.0, CUT)
    sk = sketch_at(comp, Z_FB - SENSOR_NUT_DEPTH, "sensor nut recesses")
    for x, y in sensor_pts:
        hexagon(sk, x, y, NUT_AF)
    extrude(comp, all_profiles(sk), SENSOR_NUT_DEPTH + 1.0, CUT)
    if CASTER_CHANNEL > 0.0:
        x, w = bx(CASTER_U), HEAD_D + 1.0
        y = CASTER_SLOT_DY / 2 + w / 2
        box(comp, x - w / 2, -y, Z_FB - CASTER_CHANNEL, x + w / 2, y, Z_FB + 1.0, CUT,
            "caster head channel")

    sk = sketch_at(comp, z_cut - 1.0, "slots and holes")
    for i in range(2):
        u0 = SLOT_MARGIN + i * SLOT_SPACING
        slot(sk, bx(u0), 0.0, bx(u0 + BOOM_TRAVEL), 0.0, M3)
    slot(sk, bx(CASTER_U), -CASTER_SLOT_DY / 2, bx(CASTER_U), CASTER_SLOT_DY / 2, M3)
    for x, y in sensor_pts:
        circle(sk, x, y, M3)
    ya, yb = BOOM_WINDOWS
    w = yb - ya
    for sy in (-1, 1):  # lightening windows beside the slots
        slot(sk, bx(4.0 + w / 2), sy * (ya + yb) / 2, bx(U_TIP - 12.0 - w / 2), sy * (ya + yb) / 2, w)
    hy, hx = HEADER_SLOT
    rect(sk, bx(U_SENSOR) - hx / 2, -hy / 2, bx(U_SENSOR) + hx / 2, hy / 2)
    extrude(comp, all_profiles(sk), Z_FB - z_cut + 2.0, CUT)
    return comp


def build_riser(root):
    if RISER_H < 1.0:
        return None  # caster bolts straight to the boom, or into its pocket
    comp = new_component(root, "Caster riser")
    x = bx(CASTER_U)
    box(comp, x - CASTER_PLATE_L / 2, -CASTER_PLATE_W / 2, CASTER_H,
        x + CASTER_PLATE_L / 2, CASTER_PLATE_W / 2, Z_BOOM_B, NEW, "block")
    cylinders(comp, [(x, -CASTER_HOLE_DY / 2), (x, CASTER_HOLE_DY / 2)], M3,
              CASTER_H - 1.0, Z_BOOM_B + 1.0, CUT, "holes")
    return comp


def build_mounts(root):
    """Left mount, modelled in place; the right one is the same part turned 180 deg."""
    comp = new_component(root, "N20 mount")
    y1 = HALF_W                     # outer end
    y0 = y1 - MOUNT_LEN
    yg = y1 - MOUNT_LIP - GEARBOX_L - 2 * MOTOR_FIT  # gearbox / can step
    hw = MOTOR_W / 2 + MOTOR_FIT
    za = Z_FB - MOUNT_AXIS_DEPTH
    zb = Z_FB - MOTOR_H             # pocket bottom
    box(comp, -hw - MOUNT_WALL, y0, Z_FB - MOUNT_DEPTH, hw + MOUNT_WALL, y1, Z_FB, NEW, "body")
    box(comp, -MOUNT_FLANGE_X, y1 - MOUNT_FLANGE_L, Z_FB - MOUNT_FLANGE_T, MOUNT_FLANGE_X, y1,
        Z_FB, JOIN, "flanges")
    # Full 12 x 10 pocket for the gearbox; the can section is chamfered to clear the
    # can's round sides, so the gearbox corners can't slide inward past the step.
    box(comp, -hw, yg, zb, hw, y1 - MOUNT_LIP, Z_FB + 1.0, CUT, "gearbox pocket")
    sk = side_polygon(comp, "y", (y0 - 1.0 + yg) / 2,
                      [(-hw, Z_FB + 1.0), (-hw, za - 2.0), (-3.9, zb), (3.9, zb),
                       (hw, za - 2.0), (hw, Z_FB + 1.0)], "can pocket")
    extrude_symmetric(comp, profile(sk), yg - y0 + 1.0, CUT)
    # Shaft slot through the lip, open toward the floor so the motor drops in.
    ys = y1 - MOUNT_LIP / 2
    sk = side_sketch(comp, "y", ys, "shaft slot")
    c0 = sk.modelToSketchSpace(P(-SHAFT_SLOT_W / 2, ys, za))
    c1 = sk.modelToSketchSpace(P(SHAFT_SLOT_W / 2, ys, Z_FB + 1.0))
    sk.sketchCurves.sketchLines.addTwoPointRectangle(c0, c1)
    sk.sketchCurves.sketchCircles.addByCenterRadius(sk.modelToSketchSpace(P(0, ys, za)),
                                                    SHAFT_SLOT_W / 2 * MM)
    extrude_symmetric(comp, all_profiles(sk), MOUNT_LIP + 2.0, CUT)

    pts = [(sx * MOUNT_HOLE_X, y1 - MOUNT_HOLE_IN) for sx in (-1, 1)]
    z_fl = Z_FB - MOUNT_FLANGE_T
    cylinders(comp, pts, M3, z_fl - 1.0, Z_FB + 1.0, CUT, "screw holes")
    cylinders(comp, pts, HEAD_D + 1.0, z_fl - 1.0, z_fl + MOUNT_HEAD_SINK, CUT, "head pockets")

    m = adsk.core.Matrix3D.create()
    m.setToRotation(math.pi, adsk.core.Vector3D.create(0, 0, 1), P(0, 0, 0))
    root.occurrences.addExistingComponent(comp, m)
    return comp


# ===========================================================================
# References (do not print)
# ===========================================================================

def build_motor_refs(root):
    comp = new_component(root, "N20 motors (ref)")
    za = Z_FB - MOUNT_AXIS_DEPTH
    for sy in (-1, 1):
        yf = sy * (HALF_W - MOUNT_LIP)   # gearbox face
        yc = yf - sy * GEARBOX_L          # gearbox / can joint
        box(comp, -MOTOR_W / 2, yc, za - MOTOR_H / 2, MOTOR_W / 2, yf, za + MOTOR_H / 2, NEW,
            "gearbox")
        side_cylinder(comp, yc - sy * CAN_L / 2, CAN_L, 0.0, za, MOTOR_W, "can")
        side_cylinder(comp, yf + sy * SHAFT_L / 2, SHAFT_L, 0.0, za, 3.0, "shaft")
    # The can has flats top and bottom.
    for z0, z1 in ((za - MOTOR_W, za - MOTOR_H / 2), (za + MOTOR_H / 2, za + MOTOR_W)):
        box(comp, -MOTOR_W, -HALF_W, z0, MOTOR_W, HALF_W, z1, CUT, "can flats")


def reference(comp):
    for i in range(comp.bRepBodies.count):
        comp.bRepBodies.item(i).opacity = 0.4


def build_references(root):
    comp = new_component(root, "Wheels (ref)")
    for sy in (-1, 1):
        side_cylinder(comp, sy * WHEEL_Y, WHEEL_W, 0.0, AXLE_Z, WHEEL_D, "wheel")

    comp = new_component(root, "Perfboard + parts (ref)")
    box(comp, BOARD_X0, -BOARD_W / 2, Z_BB, BOARD_X1, BOARD_W / 2, Z_BT, NEW, "perfboard")
    box(comp, BOARD_X0 + 2, -BOARD_W / 2 + 2, Z_BT, BOARD_X1 - 2, BOARD_W / 2 - 2,
        Z_BT + TALLEST, NEW, "tallest parts envelope")
    reference(comp)

    comp = new_component(root, "Brass spacers (ref)")
    cylinders(comp, BOARD_HOLES, SPACER_D, Z_FT, Z_BB, NEW, "board standoffs")
    cylinders(comp, BOARD_HOLES, SPACER_D, Z_BT, Z_DU, NEW, "deck spacers")
    cylinders(comp, [p for p in BOARD_HOLES if p[0] < 0], SPACER_D, Z_DT, Z_DT + WING_POST,
              NEW, "wing posts")
    sensor_pts = [(bx(U_SENSOR), sy * SENSOR_HOLE_DY / 2) for sy in (-1, 1)]
    cylinders(comp, sensor_pts, SPACER_D, SENSOR_PCB_TOP, SENSOR_PCB_TOP + SENSOR_SPACER, NEW,
              "sensor spacers")

    comp = new_component(root, "IR array (ref)")
    box(comp, SENSOR_REAR_X, -SENSOR_L / 2, SENSOR_PCB_TOP - SENSOR_T,
        SENSOR_REAR_X + SENSOR_W, SENSOR_L / 2, SENSOR_PCB_TOP, NEW, "pcb")
    box(comp, LOOKAHEAD - 1.5, -SENSOR_L / 2 + 3, SENSOR_FACE_Z,
        LOOKAHEAD + 1.5, SENSOR_L / 2 - 3, SENSOR_PCB_TOP - SENSOR_T, NEW, "eyes")

    comp = new_component(root, "Battery (ref)")
    box(comp, BATT_X - BATT_L / 2, -BATT_W / 2, Z_DT, BATT_X + BATT_L / 2, BATT_W / 2,
        Z_DT + BATT_H, NEW, "battery")
    reference(comp)
    build_caster_ref(root)
    build_motor_refs(root)


def build_caster_ref(root):
    comp = new_component(root, "Ball caster (ref)")
    x = bx(CASTER_U)
    box(comp, x - CASTER_PLATE_L / 2, -CASTER_PLATE_W / 2, CASTER_H - CASTER_PLATE_T,
        x + CASTER_PLATE_L / 2, CASTER_PLATE_W / 2, CASTER_H, NEW, "plate")
    sk = sketch_at(comp, 0.0, "ball housing")
    circle(sk, x, 0.0, 15.0)
    extrude(comp, profile(sk), CASTER_H - CASTER_PLATE_T, NEW)
    reference(comp)


# ===========================================================================
# Entry points
# ===========================================================================

PRINTED = ("Tray", "Deck", "Rear wing", "Sensor boom", "Caster riser", "N20 mount")


def close_previous():
    """Close an unsaved design left by an earlier (failed) run of this script."""
    app = adsk.core.Application.get()
    doc = app.activeDocument
    design = adsk.fusion.Design.cast(app.activeProduct)
    if doc and not doc.isSaved and design and design.allComponents.itemByName("Tray"):
        doc.close(False)


def new_design():
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    return prepare_design(adsk.fusion.Design.cast(app.activeProduct))


def prepare_design(design):
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    if hasattr(adsk.fusion, "DesignIntentTypes"):
        # New Fusion "Part Design" documents allow only one component.
        design.designIntent = adsk.fusion.DesignIntentTypes.HybridDesignIntentType
    design.fusionUnitsManager.distanceDisplayUnits = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
    return design


def root_component():
    app = adsk.core.Application.get()
    return adsk.fusion.Design.cast(app.activeProduct).rootComponent


def check():
    if SENSOR_PAD < -0.01:
        raise ValueError("SENSOR_SPACER too long by %.2f mm" % -SENSOR_PAD)
    if BOOM_T - CASTER_POCKET < 1.6:
        raise ValueError("caster pocket leaves only %.2f mm of boom" % (BOOM_T - CASTER_POCKET))
    if CASTER_POCKET > 0.0 and TIP_W < CASTER_PLATE_W + 2 * CASTER_FIT + 4.0:
        raise ValueError("TIP_W too narrow for the caster pocket walls")
    if TUB_H[0] - SWITCH_Z - SWITCH_H < 2.0:
        raise ValueError("rear wall too low for the switch window")
    # M3 x 6 reach, joint by joint
    if MOUNT_FLANGE_T - MOUNT_HEAD_SINK + MOUNT_NUT_WEB + NUT_H > SCREW_L - THREAD_SPARE:
        raise ValueError("motor mount screws too short")
    if SCREW_L - FLOOR_T < MIN_THREAD or SCREW_L - WING_PAD_T < MIN_THREAD:
        raise ValueError("too little thread in the board standoffs or wing posts")
    if DECK_T + NUT_H > STUD_L or STUD_L - DECK_T < MIN_THREAD:
        raise ValueError("deck too thick for the standoff studs")
    if BOOM_T - CASTER_POCKET - CASTER_CHANNEL < 1.2:
        raise ValueError("boom too thin under the caster screw heads")


def summary():
    r = lambda v: round(v, 2)
    return {
        "length_x": [r(min(TRAY_X0, WING_X[0])), r(SENSOR_REAR_X + SENSOR_W)],
        "height_to_wing_top": r(WING_Z),
        "width_with_wheels": r(2 * (WHEEL_Y + WHEEL_W / 2)),
        "track": r(2 * WHEEL_Y),
        "tray_l_w_rear_front_h": [r(TRAY_X1 - TRAY_X0), r(2 * HALF_W),
                                  r(TUB_TOP[0] - Z_FB), r(TUB_TOP[1] - Z_FB)],
        "deck_top_z": r(Z_DT),
        "lookahead_range": [r(LOOKAHEAD - BOOM_TRAVEL / 2), r(LOOKAHEAD + BOOM_TRAVEL / 2)],
        "wheelbase_to_caster": r(bx(CASTER_U)),
        "floor_underside_z": r(Z_FB),
        "boom_underside_z": r(Z_BOOM_B),
        "sensor_pad": r(SENSOR_PAD),
        "caster_riser_h": r(max(0.0, RISER_H)),
        "caster_pocket": r(CASTER_POCKET),
        "mount_screw_spare": r(SCREW_L - (MOUNT_FLANGE_T - MOUNT_HEAD_SINK + MOUNT_NUT_WEB + NUT_H)),
        "boom_slot_recess_sensor_nut_caster_channel": [r(BOOM_RECESS), r(SENSOR_NUT_DEPTH),
                                                       r(CASTER_CHANNEL)],
        "board_holes": [[r(x), r(y)] for x, y in BOARD_HOLES],
    }


def build_all():
    check()
    new_design()
    root = root_component()
    build_tray(root)
    build_deck(root)
    build_wing(root)
    build_boom(root)
    build_riser(root)
    build_mounts(root)
    build_references(root)
    return summary()


def export_stls(folder):
    root = root_component()
    em = root.parentDesign.exportManager
    os.makedirs(folder, exist_ok=True)
    done = []
    for name in PRINTED:
        comp = root.parentDesign.allComponents.itemByName(name)
        if not comp or comp.bRepBodies.count == 0:
            continue
        path = os.path.join(folder, name.lower().replace(" ", "_") + ".stl")
        opts = em.createSTLExportOptions(comp.bRepBodies.item(0), path)
        opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
        em.execute(opts)
        done.append(path)
    return done


def run(context):
    ui = adsk.core.Application.get().userInterface
    try:
        ui.messageBox(str(build_all()))
    except Exception:
        import traceback
        ui.messageBox(traceback.format_exc())
