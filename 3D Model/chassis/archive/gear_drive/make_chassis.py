"""Fusion 360 generator: race-car chassis for the line follower.

Run it in Fusion (Utilities > Scripts and Add-Ins) or through the
Fusion360MCP add-in (execute_code). It opens a new design and builds:

  Tray          low wedge tub: windowed floor + short walls (switch, USB and IR
                cable notches); the 12 x 8 cm perfboard sits on brass standoffs
                and the two N20 mounts bolt underneath into nuts trapped in the floor
  Drive mount   left and right (mirror images): clamps the motor against the
                floor underside, 16.8 mm ahead of the wheel axle, and carries the
                wheel's standoff axle; pocket copied from
                N20_Gear_Motor_Mount_-_Full_Length (gearbox stop + rounded can
                section), screw heads sunk into the flanges
  Pinion 12T    2.5:1 reduction: pressed flush onto the end of the motor shaft
  Wheel gear    30T, turns on the brass standoff axle, hex socket for the wheel
  Wheel         copy of the original rim (ridge for the tyre's groove), hex key
                into the wheel gear
  Axle washer   holds gear and wheel on the standoff
  Joints        the drive turns in Fusion (drag a wheel, Drive Joints or a Motion
                Study): pinions and wheel gears on revolute joints, each wheel
                fixed to its gear, motion links for the ratio and for both sides
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
as modelled, boom and rear wing upside down. The drive parts (FLIP_FOR_PRINT and
the pinion and washer) are exported ready to print: mounts on their floor face,
wheel gear on its socket face, wheel on its outer face. None of them need
supports.

The extra gear mesh turns each wheel the other way: set MOTOR_L_INVERT and
MOTOR_R_INVERT to true in include/config.h when the gears go on.

Hardware: M3 x 6 screws, M3 nuts and the brass standoff kit only (no inserts).
  board + deck  per corner: M3 x 6 up through the floor into an M-F 6, its stud
                through the board into an M-F 20, then an M-F 6 whose stud goes
                through the deck: a nut on the front pair, a F-F 15 wing post on
                the rear pair (a nut if the wing is left off)
  rear wing     M3 x 6 down through each pad into the F-F 15 posts
  drive mounts  4x M3 x 6 up through the mount flanges (heads sunk 1.5 mm) into
                nuts in the floor
  boom          2x M3 x 6 up through the slots (heads in a recess) into floor nuts;
                every nut joint leaves THREAD_SPARE of screw past the nut
  IR array      2x M-F 10: stud up through the boom into a nut in a hex recess,
                M3 x 6 up through the IR PCB into the standoff
  caster        2x M3 x 6 down through the boom slot and the plate into nuts in
                the caster's own nut slots
  wheel axles   2x M-F 15: stud through the mount's axle boss into a trapped nut;
                M3 x 6 + printed washer into the outer end holds the wheel on
  total         18 screws, 14 nuts, 8 M-F 6, 4 M-F 20, 2 M-F 10, 2 M-F 15, 2 F-F 15
"""

import math
import os
import struct

import adsk.core
import adsk.fusion

MM = 0.1

# --- Wheels, N20 motors and their mounts ---------------------------------------
WHEEL_D = 33.3
WHEEL_W = 6.2            # tyre width = rim seat
MOTOR_W = 12.0           # gearbox 12 x 10; can 12 across the round sides, 10 across the flats
MOTOR_H = 10.0
GEARBOX_L = 9.0
CAN_L = 15.0             # reference only
SHAFT_OUT = 9.0          # motor shaft past the gearbox face, bushing included (measured)
MOTOR_FIT = 0.2
MOUNT_AXIS_DEPTH = MOTOR_H / 2  # motor axis below the floor: its top flat bears on the floor
MOUNT_WALL = 2.0
MOUNT_DEPTH = MOTOR_H + MOUNT_WALL  # mount hangs this far below the floor
MOUNT_LEN = 26.0         # along the motor axis; outer end flush with the tray side
MOUNT_LIP = 1.0          # outer end wall the gearbox face rests against, shaft slot through it
SHAFT_SLOT_W = 5.0       # takes the 4 mm gearbox bushing
MOUNT_FLANGE_X = 18.0    # flanges reach this far fore and aft of the motor axis
MOUNT_FLANGE_L = 16.0    # along the motor axis, from the outer end
MOUNT_FLANGE_T = 3.0
MOUNT_HOLE_X = 13.0
MOUNT_HOLE_IN = 9.0      # flange holes, in from the outer end
MOUNT_HEAD_SINK = 1.5    # screw heads sink into the flange so an M3 x 6 has thread to spare

# --- Drive: 2.5:1 reduction (the 3 V 1000 RPM motors stall on a 20 deg ramp) ------
# 12T pinion on the motor, 30T gear + wheel on a brass M-F 15 standoff axle. The
# motor sits CENTER_DIST ahead of the wheel axle at the same height.
GEAR_MODULE = 0.8        # smallest that prints cleanly with a 0.4 mm nozzle
PRESSURE_ANGLE = 20.0
Z_PINION = 12
Z_GEAR = 30
X_PINION = +0.3          # profile shift: no undercut on the 12T pinion
X_GEAR = -0.3            # equal and opposite, centre distance stays m(z1+z2)/2
BACKLASH = 0.10          # tooth thickness removed per gear at the pitch circle
TIP_MIN = 0.40           # tips thinner than this get truncated
CENTER_DIST = GEAR_MODULE * (Z_PINION + Z_GEAR) / 2
PINION_FACE = 4.5        # pressed on until flush with the end of the shaft
GEAR_FACE = 5.0          # wider than the pinion: tolerates where the pinion ends up
SHAFT_BORE_D = 3.15      # 3 mm D-shaft, snug press fit after FDM shrink
SHAFT_FLAT = 2.55        # width across the flat (shaft is 2.5)
GEAR_COLLAR_D = 12.0     # spacer from the axle boss to the gear teeth
AXLE_STANDOFF = 15.0     # M-F 15 body: the wheel axle
AXLE_BORE_D = 6.2        # hex 5.0 across flats (measured) = 5.77 across corners
AXLE_BOSS_D = 9.0
AXLE_PLATE_T = 4.0
KEY_AF = 12.0            # hex key, wheel into the gear
KEY_FIT = 0.3
KEY_DEPTH = 2.0
GEAR_WHEEL_GAP = 0.5     # wheel's inner face past the gear and the shaft end
AXIAL_PLAY = 0.45        # wheel to washer
RIM_D = 30.8             # original rim over its ridge (measured), WHEEL_W wide
RIB_W = 3.2              # ridge in the middle of the rim, sits in the tyre's groove (measured)
RIB_H = 1.7
WHEEL_HUB_D = 15.0
WHEEL_RIM_T = 1.5
WHEEL_WEB = 2.0          # flat outer face, lightening pocket open on the inner face
WASHER_D = 10.0
WASHER_T = 1.2
HEAD_H = 2.1             # M3 pan head, for the width only

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
# Floor (x0, y0, x1, y1): clear of the bosses, the drive mounts (X -1..35 at |Y| > 17)
# and the boom's bearing strips (|Y| 10..15)
FLOOR_WINDOWS = [(40.0, 19.0, 84.0, 37.5), (40.0, -37.5, 84.0, -19.0),
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
MOUNT_Y = HALF_W - MOUNT_LEN / 2   # mount centre along the motor

# Drive stack along Y on the left (the right mirrors it), outward from the tray side.
MOTOR_X = CENTER_DIST              # motor axis, ahead of the wheel axle at X 0
Y_SHAFT_END = HALF_W - MOUNT_LIP + SHAFT_OUT
Y_PINION = (Y_SHAFT_END - PINION_FACE, Y_SHAFT_END)
Y_GEAR = (sum(Y_PINION) / 2 - GEAR_FACE / 2, sum(Y_PINION) / 2 + GEAR_FACE / 2)
Y_WHEEL0 = max(Y_GEAR[1], Y_SHAFT_END) + GEAR_WHEEL_GAP  # wheel's inner face
Y_AXLE_END = Y_WHEEL0 + WHEEL_W + AXIAL_PLAY             # end of the standoff
Y_SHOULDER = Y_AXLE_END - AXLE_STANDOFF                   # standoff shoulder = boss top
AXLE_BOSS_H = Y_SHOULDER - HALF_W
# Nut fully sunk when the plate allows it; never less than the stud needs.
AXLE_NUT_DEPTH = min(AXLE_PLATE_T - 0.8, max(NUT_H + 0.2, AXLE_BOSS_H + AXLE_PLATE_T + NUT_H
                                             + THREAD_SPARE - STUD_L))
WHEEL_Y = Y_WHEEL0 + WHEEL_W / 2                          # tyre centre

# Rear board screws sit behind the wheel axle plates (axle at X 0).
BOARD_HOLE_REAR_X = -21.75
BOARD_X0 = BOARD_HOLE_REAR_X - (BOARD_L - BOARD_HOLE_DX) / 2
BOARD_X1 = BOARD_X0 + BOARD_L
TRAY_X0 = BOARD_X0 - BOARD_CLEAR - WALL_T
TRAY_X1 = BOARD_X1 + BOARD_CLEAR + WALL_T

BOARD_HOLES = [(BOARD_HOLE_REAR_X + dx, sy * BOARD_HOLE_DY / 2)
               for dx in (0.0, BOARD_HOLE_DX) for sy in (-1, 1)]
MOUNT_HOLES = [(MOTOR_X + sx * MOUNT_HOLE_X, sy * (HALF_W - MOUNT_HOLE_IN))
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


def ring_profile(sk):
    """Largest profile with a hole in it (between two circles)."""
    profs = [sk.profiles.item(i) for i in range(sk.profiles.count)]
    return max((p for p in profs if p.profileLoops.count == 2),
               key=lambda p: p.areaProperties().area)


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


def side_cylinder(comp, y, width, x, z, d, name, op=NEW, d_in=None):
    """Cylinder along Y centred on the plane Y = y; a tube when d_in is given."""
    sk = side_sketch(comp, "y", y, name)
    for dd in (d, d_in) if d_in else (d,):
        sk.sketchCurves.sketchCircles.addByCenterRadius(sk.modelToSketchSpace(P(x, y, z)),
                                                        dd / 2 * MM)
    return extrude_symmetric(comp, ring_profile(sk) if d_in else profile(sk), width, op)


def side_hexagon(comp, y, x, z, af, name):
    """Hexagon on the plane Y = y, a corner pointing up and down (prints without a flat roof)."""
    sk = side_sketch(comp, "y", y, name)
    r = af / math.sqrt(3)
    polyline(sk, [sk.modelToSketchSpace(P(x + r * math.cos(a), y, z + r * math.sin(a)))
                  for a in (math.radians(90 + 60 * i) for i in range(6))])
    return sk


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
        rect(sk, MOTOR_X - 4.0, sy * (y_in - 1.5), MOTOR_X + 4.0, sy * (y_in + 4.0))
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
    for sy, tag in ((1, "L"), (-1, "R")):
        build_mount(root, sy, tag)


def build_mount(root, sy, tag):
    """Motor clamped to the floor underside ahead of the axle; the wheel's standoff axle
    stands on a boss beside it. Left and right are mirror images (sy = +1 / -1)."""
    comp = new_component(root, "N20 drive mount " + tag)
    mx = MOTOR_X
    y1 = sy * HALF_W                # outer end
    y0 = sy * (HALF_W - MOUNT_LEN)
    yl = sy * (HALF_W - MOUNT_LIP)  # gearbox face
    yg = sy * (HALF_W - MOUNT_LIP - GEARBOX_L - 2 * MOTOR_FIT)  # gearbox / can step
    hw = MOTOR_W / 2 + MOTOR_FIT
    za = AXLE_Z                     # motor and wheel axle share this height
    zb = Z_FB - MOTOR_H             # pocket bottom
    box(comp, mx - hw - MOUNT_WALL, y0, Z_FB - MOUNT_DEPTH, mx + hw + MOUNT_WALL, y1, Z_FB, NEW,
        "body")
    box(comp, mx - MOUNT_FLANGE_X, sy * (HALF_W - MOUNT_FLANGE_L), Z_FB - MOUNT_FLANGE_T,
        mx + MOUNT_FLANGE_X, y1, Z_FB, JOIN, "flanges")
    # Axle plate from the flange down past the wheel axle, with the standoff boss.
    rb = AXLE_BOSS_D / 2
    box(comp, -rb - 1.5, sy * (HALF_W - AXLE_PLATE_T), za - rb - 1.0, mx - hw, y1, Z_FB, JOIN,
        "axle plate")
    # The boss runs up to the floor face, so it rests on the bed when the mount prints.
    yb = sy * (HALF_W + AXLE_BOSS_H / 2 - 0.25)
    sk = side_sketch(comp, "y", yb, "axle boss")
    sk.sketchCurves.sketchCircles.addByCenterRadius(sk.modelToSketchSpace(P(0, yb, za)), rb * MM)
    sk.sketchCurves.sketchLines.addTwoPointRectangle(sk.modelToSketchSpace(P(-rb, yb, za)),
                                                     sk.modelToSketchSpace(P(rb, yb, Z_FB)))
    extrude_symmetric(comp, all_profiles(sk), AXLE_BOSS_H + 0.5, JOIN)

    # Full 12 x 10 pocket for the gearbox; the can section is chamfered to clear the
    # can's round sides, so the gearbox corners can't slide inward past the step.
    box(comp, mx - hw, yg, zb, mx + hw, yl, Z_FB + 1.0, CUT, "gearbox pocket")
    sk = side_polygon(comp, "y", (y0 - sy + yg) / 2,
                      [(mx - hw, Z_FB + 1.0), (mx - hw, za - 2.0), (mx - 3.9, zb), (mx + 3.9, zb),
                       (mx + hw, za - 2.0), (mx + hw, Z_FB + 1.0)], "can pocket")
    extrude_symmetric(comp, profile(sk), abs(yg - y0) + 1.0, CUT)
    # Shaft slot through the lip, open toward the floor so the motor drops in.
    ys = sy * (HALF_W - MOUNT_LIP / 2)
    sk = side_sketch(comp, "y", ys, "shaft slot")
    c0 = sk.modelToSketchSpace(P(mx - SHAFT_SLOT_W / 2, ys, za))
    c1 = sk.modelToSketchSpace(P(mx + SHAFT_SLOT_W / 2, ys, Z_FB + 1.0))
    sk.sketchCurves.sketchLines.addTwoPointRectangle(c0, c1)
    sk.sketchCurves.sketchCircles.addByCenterRadius(sk.modelToSketchSpace(P(mx, ys, za)),
                                                    SHAFT_SLOT_W / 2 * MM)
    extrude_symmetric(comp, all_profiles(sk), MOUNT_LIP + 2.0, CUT)

    # Standoff stud through boss and plate into a nut trapped in the plate's inner face.
    side_cylinder(comp, sy * (HALF_W + AXLE_BOSS_H - 3.0), 2 * (AXLE_PLATE_T + 1.0), 0.0, za, M3,
                  "axle hole", CUT)
    yt = sy * (HALF_W - AXLE_PLATE_T + AXLE_NUT_DEPTH / 2 - 0.5)
    extrude_symmetric(comp, profile(side_hexagon(comp, yt, 0.0, za, NUT_AF, "axle nut trap")),
                      AXLE_NUT_DEPTH + 1.0, CUT)

    pts = [(mx + sx * MOUNT_HOLE_X, sy * (HALF_W - MOUNT_HOLE_IN)) for sx in (-1, 1)]
    z_fl = Z_FB - MOUNT_FLANGE_T
    cylinders(comp, pts, M3, z_fl - 1.0, Z_FB + 1.0, CUT, "screw holes")
    cylinders(comp, pts, HEAD_D + 1.0, z_fl - 1.0, z_fl + MOUNT_HEAD_SINK, CUT, "head pockets")
    return comp


# ===========================================================================
# 2.5:1 drive: involute gears (shared by both sides)
# ===========================================================================

def inv(a):
    return math.tan(a) - a


def polar(r, a):
    return (r * math.cos(a), r * math.sin(a))


def gear_geometry(z, x):
    m = GEAR_MODULE
    al = math.radians(PRESSURE_ANGLE)
    r = m * z / 2
    rb = r * math.cos(al)
    ra = r + m * (1 + x)
    rf = r - m * (1.25 - x)
    s = m * (math.pi / 2 + 2 * x * math.tan(al)) - BACKLASH
    psi = s / (2 * r)

    def half(R):  # half tooth angle at radius R
        return psi + inv(al) - inv(math.acos(rb / max(R, rb)))

    while 2 * half(ra) * ra < TIP_MIN:
        ra -= 0.01
    return dict(r=r, rb=rb, ra=ra, rf=rf, half=half)


def tooth_outline(g, phase):
    """Closed polygon of one tooth, centred on angle `phase`, reaching into the root disc."""
    half, ra, rf, rb = g["half"], g["ra"], g["rf"], g["rb"]
    r0 = max(rf, rb)
    r_in = rf - 0.4
    n = 10
    flank = [r0 + (ra - r0) * i / n for i in range(n + 1)]
    th0 = half(r0)
    pts = [polar(r_in, phase - th0)]
    pts += [polar(R, phase - half(R)) for R in flank]
    tip = half(ra)
    pts += [polar(ra, phase - tip + 2 * tip * k / 4) for k in range(1, 4)]
    pts += [polar(R, phase + half(R)) for R in reversed(flank)]
    pts.append(polar(r_in, phase + th0))
    return pts


def gear_teeth(comp, z, x, z0, face, phase, op, name):
    """Root disc + one tooth + circular pattern about local Z."""
    g = gear_geometry(z, x)
    cylinders(comp, [(0.0, 0.0)], 2 * g["rf"], z0, z0 + face, op, name + " root")
    sk = sketch_at(comp, z0, name + " tooth")
    polygon(sk, tooth_outline(g, phase))
    tooth = extrude(comp, profile(sk), face, JOIN)
    pats = comp.features.circularPatternFeatures
    coll = adsk.core.ObjectCollection.create()
    coll.add(tooth)
    inp = pats.createInput(coll, comp.zConstructionAxis)
    inp.quantity = adsk.core.ValueInput.createByReal(z)
    inp.totalAngle = adsk.core.ValueInput.createByString("360 deg")
    inp.isSymmetric = False
    pats.add(inp)
    return g


def contact_ratio():
    g1, g2 = gear_geometry(Z_PINION, X_PINION), gear_geometry(Z_GEAR, X_GEAR)
    al = math.radians(PRESSURE_ANGLE)
    num = (math.sqrt(g1["ra"] ** 2 - g1["rb"] ** 2) + math.sqrt(g2["ra"] ** 2 - g2["rb"] ** 2)
           - CENTER_DIST * math.sin(al))
    return num / (math.pi * GEAR_MODULE * math.cos(al))


def axle_frame(x, y, sy):
    """Part frame on an axle parallel to Y: local Z points outboard, local X = robot X."""
    m = adsk.core.Matrix3D.create()
    m.setWithCoordinateSystem(P(x, y, AXLE_Z), adsk.core.Vector3D.create(1, 0, 0),
                              adsk.core.Vector3D.create(0, 0, -sy),
                              adsk.core.Vector3D.create(0, sy, 0))
    return m


def axle_part(root, name, y, x=0.0):
    """One component, placed on both sides; its geometry is modelled lying on local Z = 0."""
    comp = new_component(root, name, axle_frame(x, y, 1))
    root.occurrences.addExistingComponent(comp, axle_frame(x, -y, -1))
    return comp


def build_pinion(root):
    comp = axle_part(root, "Pinion 12T", Y_PINION[0], MOTOR_X)
    gear_teeth(comp, Z_PINION, X_PINION, 0.0, PINION_FACE, math.pi, NEW, "pinion")  # tooth at the gear
    sk = sketch_at(comp, PINION_FACE / 2, "D bore")
    circle(sk, 0, 0, SHAFT_BORE_D)
    r = SHAFT_BORE_D / 2
    flat = SHAFT_FLAT - r  # flat line distance from centre
    h = math.sqrt(r * r - flat * flat) + 0.3
    sk.sketchCurves.sketchLines.addByTwoPoints(P(flat, -h), P(flat, h))
    extrude_symmetric(comp, profile(sk), PINION_FACE + 2.0, CUT)
    return comp


def build_wheel_gear(root):
    """30T gear on a collar that bears on the axle boss; hex socket for the wheel's key."""
    comp = axle_part(root, "Wheel gear 30T", Y_SHOULDER)
    zt = Y_GEAR[0] - Y_SHOULDER     # collar height
    z1 = zt + GEAR_FACE
    cylinders(comp, [(0.0, 0.0)], GEAR_COLLAR_D, 0.0, zt, NEW, "collar")
    gear_teeth(comp, Z_GEAR, X_GEAR, zt, GEAR_FACE, math.pi / Z_GEAR, JOIN, "gear")  # gap at the pinion
    sk = sketch_at(comp, z1 - KEY_DEPTH, "key socket")
    hexagon(sk, 0.0, 0.0, KEY_AF + KEY_FIT)
    extrude(comp, profile(sk), KEY_DEPTH + 1.0, CUT)
    cylinders(comp, [(0.0, 0.0)], AXLE_BORE_D, -1.0, z1 + 1.0, CUT, "axle bore")
    return comp


def revolve_cut(comp, rz_pts, name):
    """Cut the solid of revolution of a (radius, z) polygon about local Z."""
    sk = comp.sketches.add(comp.xZConstructionPlane)
    sk.name = name
    polyline(sk, [sk.modelToSketchSpace(P(r, 0.0, z)) for r, z in rz_pts])
    revs = comp.features.revolveFeatures
    inp = revs.createInput(profile(sk), comp.zConstructionAxis, CUT)
    inp.setAngleExtent(False, adsk.core.ValueInput.createByString("360 deg"))
    inp.participantBodies = [comp.bRepBodies.item(i) for i in range(comp.bRepBodies.count)]
    return revs.add(inp)


def build_wheel(root):
    """Copy of the original rim; flat outer face, lightening pocket and key on the inner face."""
    comp = axle_part(root, "Wheel", Y_WHEEL0)
    w = WHEEL_W
    cylinders(comp, [(0.0, 0.0)], RIM_D, 0.0, w, NEW, "rim")
    # Rim either side of the ridge. The wheel prints on its outer face, so the ridge's
    # outer side would hang over the rim: that side is a 45 deg slope instead of a step.
    rb, ro = RIM_D / 2 - RIB_H, RIM_D / 2 + 1.0
    za, zb = (w - RIB_W) / 2, (w + RIB_W) / 2
    revolve_cut(comp, [(rb, -1.0), (ro, -1.0), (ro, za), (rb, za)], "inner rim")
    revolve_cut(comp, [(rb, w + 1.0), (ro, w + 1.0), (ro, zb - (ro - rb)), (rb, zb)], "outer rim")
    sk = sketch_at(comp, -1.0, "pocket")
    circle(sk, 0, 0, 2 * rb - 2 * WHEEL_RIM_T)
    circle(sk, 0, 0, WHEEL_HUB_D)
    extrude(comp, ring_profile(sk), 1.0 + w - WHEEL_WEB, CUT)
    key_l = GEAR_WHEEL_GAP + KEY_DEPTH  # bottoms in the gear's socket
    sk = sketch_at(comp, -key_l, "key")
    hexagon(sk, 0.0, 0.0, KEY_AF)
    extrude(comp, profile(sk), key_l, JOIN)
    cylinders(comp, [(0.0, 0.0)], AXLE_BORE_D, -key_l - 1.0, w + 1.0, CUT, "axle bore")
    return comp


def build_washer(root):
    comp = axle_part(root, "Axle washer", Y_AXLE_END)
    sk = sketch_at(comp, 0.0, "washer")
    circle(sk, 0, 0, WASHER_D)
    circle(sk, 0, 0, M3)
    extrude(comp, ring_profile(sk), WASHER_T, NEW)
    return comp


def build_drive(root):
    build_pinion(root)
    build_wheel_gear(root)
    build_wheel(root)
    build_washer(root)


def occurrences(root, name):
    """Occurrences of a component, left (+Y) first."""
    occs = root.occurrencesByComponent(root.parentDesign.allComponents.itemByName(name))
    return sorted((occs.item(i) for i in range(occs.count)), key=lambda o: -o.transform2.translation.y)


def axis_joint(root, occ, base, name):
    """As-built revolute joint: occ turns on its own local Z axis, relative to base."""
    comp = occ.component
    origin = comp.originConstructionPoint.createForAssemblyContext(occ)
    inp = root.asBuiltJoints.createInput(occ, base, adsk.fusion.JointGeometry.createByPoint(origin))
    inp.setAsRevoluteJointMotion(adsk.fusion.JointDirections.CustomJointDirection,
                                 comp.zConstructionAxis.createForAssemblyContext(occ))
    joint = root.asBuiltJoints.add(inp)
    joint.name = name
    return joint


def motion_link(root, joint1, deg1, joint2, deg2, name):
    """deg1 of joint1 goes with deg2 of joint2; a negative deg2 turns joint2 the other way."""
    inp = root.motionLinks.createInput(joint1, joint2)
    inp.valueOne = adsk.core.ValueInput.createByString("%g deg" % deg1)
    inp.valueTwo = adsk.core.ValueInput.createByString("%g deg" % abs(deg2))
    inp.isReversed = deg2 < 0
    link = root.motionLinks.add(inp)
    link.name = name
    return link


def build_joints(root):
    """Lets the drive turn in Fusion: pinions and wheel gears turn on their axles, each
    wheel is fixed to its gear, motion links keep the ratio and turn both sides together.
    Everything else is grounded. Safe to re-run after rebuilding a part."""
    for coll in (root.motionLinks, root.asBuiltJoints):
        while coll.count:
            coll.item(0).deleteMe()
    moving = ("Pinion 12T", "Wheel gear 30T", "Wheel")
    for i in range(root.occurrences.count):
        occ = root.occurrences.item(i)
        occ.isGrounded = occ.component.name not in moving
    pinion_joints = []
    for tag, pinion, gear, wheel in zip("LR", *(occurrences(root, n) for n in moving)):
        mount = occurrences(root, "N20 drive mount " + tag)[0]
        jp = axis_joint(root, pinion, mount, "Pinion " + tag)
        jg = axis_joint(root, gear, mount, "Wheel gear " + tag)
        inp = root.asBuiltJoints.createInput(wheel, gear, None)
        inp.setAsRigidJointMotion()
        root.asBuiltJoints.add(inp).name = "Wheel on gear " + tag
        # Both joint axes point outboard, and meshed gears turn opposite ways.
        ratio = Z_GEAR / Z_PINION
        motion_link(root, jp, 360.0 * ratio, jg, -360.0, "%g:1 %s" % (ratio, tag))
        pinion_joints.append(jp)
    # The axes point outboard, so the sides turn the same way with opposite joint angles.
    motion_link(root, pinion_joints[0], 360.0, pinion_joints[1], -360.0, "Left with right")


DRIVE_COLOURS = {"Pinion 12T": "Plastic - Matte (Yellow)", "Wheel gear 30T": "Plastic - Matte (Blue)"}


def paint_drive(root):
    """Colour the gears so their motion is easy to follow."""
    design = root.parentDesign
    lib = adsk.core.Application.get().materialLibraries.itemByName("Fusion Appearance Library")
    for name, look in DRIVE_COLOURS.items():
        appearance = design.appearances.itemByName(look)
        if not appearance:
            appearance = design.appearances.addByCopy(lib.appearances.itemByName(look), look)
        design.allComponents.itemByName(name).bRepBodies.item(0).appearance = appearance


# ===========================================================================
# References (do not print)
# ===========================================================================

def build_motor_refs(root):
    comp = new_component(root, "N20 motors (ref)")
    za, mx = AXLE_Z, MOTOR_X
    for sy in (-1, 1):
        yf = sy * (HALF_W - MOUNT_LIP)   # gearbox face
        yc = yf - sy * GEARBOX_L          # gearbox / can joint
        box(comp, mx - MOTOR_W / 2, yc, za - MOTOR_H / 2, mx + MOTOR_W / 2, yf, za + MOTOR_H / 2,
            NEW, "gearbox")
        side_cylinder(comp, yc - sy * CAN_L / 2, CAN_L, mx, za, MOTOR_W, "can")
        side_cylinder(comp, yf + sy * SHAFT_OUT / 2, SHAFT_OUT, mx, za, 3.0, "shaft")
    # The can has flats top and bottom.
    for z0, z1 in ((za - MOTOR_W, za - MOTOR_H / 2), (za + MOTOR_H / 2, za + MOTOR_W)):
        box(comp, mx - MOTOR_W, -HALF_W, z0, mx + MOTOR_W, HALF_W, z1, CUT, "can flats")


def reference(comp):
    for i in range(comp.bRepBodies.count):
        comp.bRepBodies.item(i).opacity = 0.4


def build_references(root):
    comp = new_component(root, "Tyres (ref)")
    for sy in (-1, 1):
        side_cylinder(comp, sy * WHEEL_Y, WHEEL_W, 0.0, AXLE_Z, WHEEL_D, "tyre",
                      d_in=RIM_D - 2 * RIB_H)
        side_cylinder(comp, sy * WHEEL_Y, RIB_W, 0.0, AXLE_Z, RIM_D, "tyre groove", CUT)

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
    for sy in (-1, 1):
        side_cylinder(comp, sy * (Y_SHOULDER + AXLE_STANDOFF / 2), AXLE_STANDOFF, 0.0, AXLE_Z,
                      SPACER_D, "axle standoff")

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

PRINTED = ("Tray", "Deck", "Rear wing", "Sensor boom", "Caster riser", "N20 drive mount L",
           "N20 drive mount R", "Pinion 12T", "Wheel gear 30T", "Wheel", "Axle washer")
FLIP_FOR_PRINT = ("N20 drive mount L", "N20 drive mount R", "Wheel gear 30T", "Wheel")


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
    # 2.5:1 drive
    g1, g2 = gear_geometry(Z_PINION, X_PINION), gear_geometry(Z_GEAR, X_GEAR)
    if GEAR_COLLAR_D / 2 > CENTER_DIST - g1["ra"] - 0.3:
        raise ValueError("wheel gear collar reaches the pinion teeth")
    if AXLE_Z - g2["ra"] < 3.0:
        raise ValueError("wheel gear too close to the ground")
    if Y_PINION[0] < HALF_W + 0.5:
        raise ValueError("pinion runs into the mount: shaft too short")
    if AXLE_BOSS_H < 0.3 or Y_GEAR[0] - Y_SHOULDER < 0.5:
        raise ValueError("gear and wheel don't fit on the axle standoff")
    if AXLE_BOSS_H + AXLE_PLATE_T - AXLE_NUT_DEPTH + NUT_H > STUD_L - THREAD_SPARE:
        raise ValueError("axle standoff stud too short for its nut")
    if AXLE_NUT_DEPTH > AXLE_PLATE_T - 0.8:
        raise ValueError("axle nut trap too deep for the plate")


def summary():
    r = lambda v: round(v, 2)
    return {
        "length_x": [r(min(TRAY_X0, WING_X[0])), r(SENSOR_REAR_X + SENSOR_W)],
        "height_to_wing_top": r(WING_Z),
        "width_over_axle_screws": r(2 * (Y_AXLE_END + WASHER_T + HEAD_H)),
        "track": r(2 * WHEEL_Y),
        "drive": {"ratio": Z_GEAR / Z_PINION, "centre_dist": r(CENTER_DIST),
                  "contact_ratio": r(contact_ratio()),
                  "gear_tip_d": r(2 * gear_geometry(Z_GEAR, X_GEAR)["ra"]),
                  "pinion_tip_d": r(2 * gear_geometry(Z_PINION, X_PINION)["ra"]),
                  "gear_ground_clearance": r(AXLE_Z - gear_geometry(Z_GEAR, X_GEAR)["ra"]),
                  "y_pinion_gear_wheel_axle_end": [r(Y_PINION[0]), r(Y_GEAR[0]), r(Y_WHEEL0),
                                                   r(Y_AXLE_END)],
                  "axle_boss_h_nut_depth": [r(AXLE_BOSS_H), r(AXLE_NUT_DEPTH)]},
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
    build_drive(root)
    build_references(root)
    build_joints(root)
    paint_drive(root)
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
        if name in FLIP_FOR_PRINT:
            flip_stl(path)
        done.append(path)
    return done


def flip_stl(path):
    """Turn a binary STL over (180 deg about X, not a mirror) and stand it on z = 0."""
    with open(path, "rb") as f:
        data = bytearray(f.read())
    n = struct.unpack_from("<I", data, 80)[0]
    tris = [list(struct.unpack_from("<12f", data, 84 + i * 50)) for i in range(n)]
    for t in tris:
        for k in range(0, 12, 3):  # normal, then the three vertices
            t[k + 1], t[k + 2] = -t[k + 1], -t[k + 2]
    z0 = min(t[k] for t in tris for k in (5, 8, 11))
    for i, t in enumerate(tris):
        for k in (5, 8, 11):
            t[k] -= z0
        struct.pack_into("<12f", data, 84 + i * 50, *t)
    with open(path, "wb") as f:
        f.write(data)


def run(context):
    ui = adsk.core.Application.get().userInterface
    try:
        ui.messageBox(str(build_all()))
    except Exception:
        import traceback
        ui.messageBox(traceback.format_exc())
