"""Fusion 360 generator: lightweight frame for the line follower.

Run it in Fusion (Utilities > Scripts and Add-Ins) or through the
Fusion360MCP add-in (execute_code). It opens a new design and builds:

  Frame         one flat skeleton plate, as light as the parts allow: the
                perfboard stands on brass standoffs at its four corners, the two
                N20 motors clamp under it (wheels straight on the motor shafts),
                and a fixed arm carries the ball caster and the IR array
  Motor strap   one per motor, clamps the gearbox up against the frame
  Body          optional race-car shell from the tail to the IR array: wedge
                nose with a front wing, cockpit bubble over the battery, side
                pods. Pushes onto four brass posts at the board corners and
                pulls off by hand; four screws can lock it on
  Body wing     rear wing, plugs into the body's tail with two pins
  BODY_STYLE    "hypercar" (default): a smooth Le Mans hypercar body in one piece,
                lofted: low nose with a splitter (the Front wing part), front fenders
                above a lower bonnet, haunches and arches over the rear wheels, a
                teardrop cockpit over the battery and a shark fin up to a raised rear
                wing. Prints upright with tree supports, no lock screws.
                "split": a flat-decked shell with chamfers and a wedge nose, a plug-in
                canopy and a pin-on front wing, all printing without supports.
                "one-piece": the first race-car shell above
  References    wheels, motors, perfboard, parts envelope, battery (strapped on
                the parts), IR array, caster, standoffs: do not print

Everything is modelled in place in the robot frame: X forward, Y left, Z up,
origin on the ground under the middle of the wheel axle. Dimensions in mm; the
Fusion API uses cm (MM). The frame and straps are exported ready to print, upside
down: the frame on its flat top face, the straps on their tabs. No supports. The
body prints upright on its open bottom edge (the wide roof needs tree supports
from the plate, inside the shell only); the wing prints upside down on its plate.

CASTER_IN_FRONT = True builds the other version: the caster ahead of the IR array
instead of behind it, so the caster (not the IR eyes) meets the foot of a steep ramp.
Its frame exports as frame_front_caster.stl; the straps are the same part.

The earlier tub, deck, wing, sliding boom and 2.5:1 gear drive are kept in
archive/gear_drive/.

Hardware: M3 x 6 screws, M3 nuts and the brass standoff kit only (no inserts).
  board         per corner: M3 x 6 up through the frame into an M-F 6, its stud
                through the board, a nut on top
  motors        per strap: 2x M3 x 6 down through the frame into nuts in the
                strap's tabs
  IR array      2x M-F 10: stud up through the arm into a nut in a hex pocket,
                M3 x 6 up through the IR PCB into the standoff
  caster        2x M3 x 6 down through the arm (heads in a channel) and the
                caster plate into nuts in the caster's own nut slots
  total         12 screws, 12 nuts, 4 M-F 6, 2 M-F 10
  body          an F-F 10 in place of each board nut (the board stud threads
                into it); the body's sockets push over them. To lock it, an
                M3 x 6 down each chimney into the post: 4 more screws
Every nut joint keeps THREAD_SPARE of screw past the nut.
"""

import math
import os
import struct

import adsk.core
import adsk.fusion

MM = 0.1

# --- Wheels and N20 motors: the original wheels press straight onto the shafts ---
WHEEL_D = 33.3
WHEEL_W = 6.2
WHEEL_GAP = 1.5          # motor lip to the inside face of the wheel
MOTOR_W = 12.0           # gearbox 12 x 10; can 12 across the round sides, 10 across the flats
MOTOR_H = 10.0
GEARBOX_L = 9.0
CAN_L = 15.0             # reference only
SHAFT_OUT = 9.0          # motor shaft past the gearbox face, bushing included (measured)
MOTOR_FIT = 0.2
LIP_T = 1.2              # wall the gearbox face rests against, shaft slot through it
SHAFT_SLOT_W = 5.0       # takes the 4 mm gearbox bushing
STRAP_W = 8.0            # along the motor axis, under the gearbox
STRAP_T = 1.6
STRAP_TAB_H = 3.0        # tabs bear on the frame underside: nut pocket + web
STRAP_SCREW_X = 11.6     # either side of the motor axis
STRAP_SQUEEZE = 0.1      # strap is this much shorter than the motor, so it clamps

# --- Perfboard and electronics ----------------------------------------------
BOARD_L = 120.0          # along X
BOARD_W = 80.0
BOARD_T = 1.6
BOARD_HOLE_DX = 113.5    # hole centres, long side
BOARD_HOLE_DY = 73.5     # hole centres, short side
BOARD_HOLE_REAR_X = -21.75  # rear holes behind the motors (axle at X 0)
BOARD_STANDOFF = 6.0     # M-F 6 on the frame -> board underside: room for solder joints
TALLEST = 21.4           # tallest part above the board (measured 23 from the board underside)
BATT_L = 56.0            # strapped on top of the board's parts (reference only)
BATT_W = 30.0
BATT_H = 20.0
BATT_X = 26.0            # centre, ahead of the axle

# --- Frame: one flat skeleton plate --------------------------------------------
FRAME_T = 2.0
FRAME_R = 2.0            # strut corner radius
PAD_D = 9.0              # around the board screws
STRIP_W = 8.0            # across the robot, between the motor blocks
SPINE_W = 8.0            # along the middle, from the motors to the arm
CASTER_BAR_L = 12.0      # along X, under the middle of the caster plate
SENSOR_BAR_L = 6.0       # along X, behind the IR header
SENSOR_PAD_W = 9.0       # along X, around each IR standoff

# --- IR array and ball caster on the frame's arm ------------------------------
LOOKAHEAD = 150.0        # axle to the sensor eyes
SENSOR_L = 64.0          # IR array, across the robot
SENSOR_W = 17.0
SENSOR_T = 1.6
SENSOR_HOLE_DY = 35.7
SENSOR_HOLE_BACK = 4.0   # hole line (and header) in from the rear edge
SENSOR_EYES_BACK = 8.5   # eye line in from the rear edge
SENSOR_EYE_H = 2.5       # LEDs / phototransistors below the PCB: MEASURE
SENSOR_FACE_Z = 3.0      # eyes above the track
SENSOR_SPACER = 10.0     # M-F 10, arm -> sensor PCB, stud up through the arm
HEADER_W = 29.0          # 11-pin header + Dupont plug, comes up between the arm's fingers
CASTER_IN_FRONT = False  # True: caster ahead of the IR array (exports frame_front_caster.stl)
CASTER_X_BEHIND = 122.0  # caster centre when it sits behind the IR array
CASTER_PLATE_L = 25.0    # along X
CASTER_PLATE_W = 50.0
CASTER_H = 19.6          # plate face to the floor (measured)
CASTER_PLATE_T = 2.7     # measured
CASTER_NUT_SLOT = 1.2    # nut slots under the plate (measured): screws come down from the top
CASTER_SLOT_DY = 44.0    # screw slot takes any hole spacing up to this

# --- Detachable body ---------------------------------------------------------
BODY_STYLE = "hypercar"  # "hypercar", "split" (no supports) or "one-piece"
BODY_WALL = 0.8          # shell: two 0.4 mm perimeters
BODY_GAP = 2.0           # open bottom edge above the frame top
BODY_ROOF = 56.5         # shoulder line over the board: parts envelope + ~3 mm clearance
BODY_NOSE_Z = 37.5       # nose top over the arm: IR plug and wires run underneath
BATT_GAP = 2.0           # cockpit bubble around the battery
POST_L = 10.0            # F-F 10 on each board stud, in place of the nut
POST_AF = 5.0            # brass standoff hex across flats: MEASURE
POST_FIT = 0.05          # socket bore over the hex corners (prints a little tight)
POST_GRIP = 6.0          # socket length over the post
SOCKET_D = 10.0
SOCKET_TOP = 2.0         # socket roof under the lock screw
CHIMNEY_D = 7.0          # screwdriver hole above each socket
WING_X = -20.0           # rear wing pins, either side of the cockpit
WING_Y = 22.0
WING_PIN_D = 4.0

# --- Fasteners: M3 x 6 screws only, so every clamped stack is kept thin ----------
# With a nut: stack + NUT_H <= SCREW_L. Into a standoff: SCREW_L - stack >= MIN_THREAD.
M3 = 3.4
HEAD_D = 5.5
SCREW_L = 6.0
STUD_L = 6.0             # M-F standoff stud
NUT_AF = 5.7             # trap, nut is 5.5
NUT_H = 2.4
MIN_THREAD = 3.0
THREAD_SPARE = 0.6       # screw or stud left past a nut, for print tolerance
SPACER_D = 5.5

# --- Derived ----------------------------------------------------------------
AXLE_Z = WHEEL_D / 2
Z_FB = AXLE_Z + MOTOR_H / 2      # frame underside = motor top
Z_FT = Z_FB + FRAME_T
Z_BB = Z_FT + BOARD_STANDOFF     # board underside
Z_BT = Z_BB + BOARD_T

HALF_W = BOARD_W / 2 + 3.0       # outer face of the motor lips; wheels clear the board edge
Y_GEARBOX = HALF_W - LIP_T       # gearbox face
WHEEL_Y = HALF_W + WHEEL_GAP + WHEEL_W / 2
STRAP_Y = Y_GEARBOX - 0.2 - STRAP_W / 2    # strap centre, under the gearbox
NUT_R = NUT_AF / math.sqrt(3)    # nut trap, centre to corner
STRAP_TAB_X = STRAP_SCREW_X + NUT_R + 1.2  # tab ends
STRAP_HOLES = [(sx * STRAP_SCREW_X, sy * STRAP_Y) for sx in (-1, 1) for sy in (-1, 1)]

BOARD_X0 = BOARD_HOLE_REAR_X - (BOARD_L - BOARD_HOLE_DX) / 2
BOARD_X1 = BOARD_X0 + BOARD_L
BOARD_HOLES = [(BOARD_HOLE_REAR_X + dx, sy * BOARD_HOLE_DY / 2)
               for dx in (0.0, BOARD_HOLE_DX) for sy in (-1, 1)]

SENSOR_REAR_X = LOOKAHEAD - SENSOR_EYES_BACK
SENSOR_FRONT_X = SENSOR_REAR_X + SENSOR_W
SENSOR_HOLE_X = SENSOR_REAR_X + SENSOR_HOLE_BACK
SENSOR_HOLES = [(SENSOR_HOLE_X, sy * SENSOR_HOLE_DY / 2) for sy in (-1, 1)]
SENSOR_PCB_TOP = SENSOR_FACE_Z + SENSOR_EYE_H + SENSOR_T
SENSOR_PAD = Z_FB - SENSOR_PCB_TOP - SENSOR_SPACER  # under the arm at the IR standoffs
SENSOR_NUT_DEPTH = max(0.0, FRAME_T + SENSOR_PAD + NUT_H - STUD_L + THREAD_SPARE)
FINGER_Y = (HEADER_W / 2 + 0.9, SENSOR_HOLE_DY / 2 + SENSOR_PAD_W / 2)  # |Y| either side of the header
SENSOR_BAR_X = (SENSOR_HOLE_X - SENSOR_PAD_W / 2 - 2.0 - SENSOR_BAR_L,
                SENSOR_HOLE_X - SENSOR_PAD_W / 2 - 2.0)

# Behind, the eyes overhang the caster and meet the foot of a steep ramp first. In front,
# the caster plate clears the IR PCB's front edge (so any ball housing up to the plate's
# length fits) and the caster meets the ramp first.
CASTER_X = SENSOR_FRONT_X + 1.0 + CASTER_PLATE_L / 2 if CASTER_IN_FRONT else CASTER_X_BEHIND
VARIANT = "_front_caster" if CASTER_IN_FRONT else ""
Z_BODY = Z_FT + BODY_GAP         # the body's open bottom edge, flat on the print bed
Z_POST_TOP = Z_BT + POST_L
BODY_HW = WHEEL_Y - WHEEL_W / 2 - 1.5   # half width between the wheels
CASTER_PAD = Z_FB - CASTER_H     # under the arm at the caster plate
CASTER_BAR_HW = CASTER_SLOT_DY / 2 + 4.0
CASTER_CHANNEL = max(0.0, FRAME_T + CASTER_PAD + CASTER_PLATE_T - CASTER_NUT_SLOT + NUT_H
                     - SCREW_L + THREAD_SPARE)  # caster screw heads sink this far into the arm top


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


def rounded_box(comp, x0, y0, z0, x1, y1, z1, r, op, name):
    sk = sketch_at(comp, z0, name)
    rounded_rect(sk, x0, y0, x1, y1, r)
    return extrude(comp, profile(sk), z1 - z0, op)


def join(comp, body, tools):
    coll = adsk.core.ObjectCollection.create()
    for b in tools:
        coll.add(b)
    inp = comp.features.combineFeatures.createInput(body, coll)
    inp.operation = JOIN
    comp.features.combineFeatures.add(inp)


def side_cylinder(comp, y, width, x, z, d, name, op=NEW, d_in=None):
    """Cylinder along Y centred on the plane Y = y; a tube when d_in is given."""
    sk = side_sketch(comp, "y", y, name)
    for dd in (d, d_in) if d_in else (d,):
        sk.sketchCurves.sketchCircles.addByCenterRadius(sk.modelToSketchSpace(P(x, y, z)),
                                                        dd / 2 * MM)
    return extrude_symmetric(comp, ring_profile(sk) if d_in else profile(sk), width, op)


def cylinders(comp, pts, d, z0, z1, op, name):
    sk = sketch_at(comp, z0, name)
    for x, y in pts:
        circle(sk, x, y, d)
    return extrude(comp, all_profiles(sk), z1 - z0, op)


INTERSECT = adsk.fusion.FeatureOperations.IntersectFeatureOperation


def prism(comp, pts, z0, z1, name):
    """Top-view outline extruded from z0 to z1, as a new named body."""
    sk = sketch_at(comp, z0, name)
    polygon(sk, pts)
    body = extrude(comp, profile(sk), z1 - z0, NEW).bodies.item(0)
    body.name = name
    return body


def trim_to_side(comp, body, pts, name):
    """Keep only the part of body inside the side outline pts, (x, z) on the plane Y = 0."""
    sk = comp.sketches.add(comp.xZConstructionPlane)
    sk.name = name
    polyline(sk, [sk.modelToSketchSpace(P(x, 0.0, z)) for x, z in pts])
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(profile(sk), INTERSECT)
    inp.setSymmetricExtent(adsk.core.ValueInput.createByReal(200 * MM), True)
    inp.participantBodies = [body]
    ext.add(inp)
    return comp.bRepBodies.itemByName(body.name)


def round_edges(comp, body, r, z_skip):
    """Fillet every edge of body except those lying in the plane z = z_skip."""
    edges = adsk.core.ObjectCollection.create()
    for i in range(body.edges.count):
        e = body.edges.item(i)
        if any(abs(v.geometry.z - z_skip * MM) > 1e-4 for v in (e.startVertex, e.endVertex)):
            edges.add(e)
    fillets = comp.features.filletFeatures
    inp = fillets.createInput()
    inp.edgeSetInputs.addConstantRadiusEdgeSet(edges, adsk.core.ValueInput.createByReal(r * MM), True)
    fillets.add(inp)


def open_shell(comp, body, t, z_open):
    """Hollow body to wall t, removing its flat faces in the plane z = z_open."""
    faces = adsk.core.ObjectCollection.create()
    for i in range(body.faces.count):
        f = body.faces.item(i)
        p = f.pointOnFace
        normal = f.evaluator.getNormalAtPoint(p)[1]
        if abs(p.z - z_open * MM) < 1e-4 and normal.z < -0.99:
            faces.add(f)
    shells = comp.features.shellFeatures
    inp = shells.createInput(faces, False)
    inp.insideThickness = adsk.core.ValueInput.createByReal(t * MM)
    shells.add(inp)


def round_vertical(comp, body, radius_at):
    """Fillet body's vertical edges; radius_at(x, y) gives each one's radius, or None to
    leave it sharp."""
    groups = {}
    for i in range(body.edges.count):
        e = body.edges.item(i)
        a, b = e.startVertex.geometry, e.endVertex.geometry
        if abs(a.x - b.x) < 1e-5 and abs(a.y - b.y) < 1e-5:
            r = radius_at(a.x / MM, a.y / MM)
            if r:
                groups.setdefault(r, adsk.core.ObjectCollection.create()).add(e)
    fillets = comp.features.filletFeatures
    inp = fillets.createInput()
    for r, edges in groups.items():
        inp.edgeSetInputs.addConstantRadiusEdgeSet(edges, adsk.core.ValueInput.createByReal(r * MM), True)
    fillets.add(inp)
    return comp.bRepBodies.itemByName(body.name)


def intersect_keep(comp, body, tool):
    """body = body inside tool; tool is kept."""
    coll = adsk.core.ObjectCollection.create()
    coll.add(tool)
    inp = comp.features.combineFeatures.createInput(body, coll)
    inp.operation = INTERSECT
    inp.isKeepToolBodies = True
    comp.features.combineFeatures.add(inp)


# ===========================================================================
# Printed parts
# ===========================================================================

def build_frame(root):
    comp = new_component(root, "Frame")
    r = PAD_D / 2
    yh = BOARD_HOLE_DY / 2
    xf = BOARD_HOLE_REAR_X + BOARD_HOLE_DX
    xa, xb = SENSOR_BAR_X
    # With the caster in front, the fingers run on over the IR PCB to the caster bar.
    x_finger = CASTER_X if CASTER_IN_FRONT else SENSOR_HOLE_X + SENSOR_PAD_W / 2
    # Skeleton: motor blocks (rear board pad, strap tabs, lip), a strip between them,
    # a spine to the arm, the front board pads, the caster bar and the IR bar whose
    # fingers reach the standoffs either side of the header.
    sk = sketch_at(comp, Z_FB, "outline")
    for sy in (-1, 1):
        y0, y1 = sorted((sy * (yh - r), sy * HALF_W))
        rounded_rect(sk, BOARD_HOLE_REAR_X - r, y0, STRAP_TAB_X, y1, FRAME_R)
        y0, y1 = sorted((sy * FINGER_Y[0], sy * FINGER_Y[1]))
        rounded_rect(sk, xa + 2.0, y0, x_finger, y1, FRAME_R)
    rounded_rect(sk, -STRIP_W / 2, -HALF_W + 9.0, STRIP_W / 2, HALF_W - 9.0, FRAME_R)
    rounded_rect(sk, 0.0, -SPINE_W / 2, xb - 2.0, SPINE_W / 2, FRAME_R)
    slot(sk, xf, -yh, xf, yh, PAD_D)
    rounded_rect(sk, CASTER_X - CASTER_BAR_L / 2, -CASTER_BAR_HW, CASTER_X + CASTER_BAR_L / 2,
                 CASTER_BAR_HW, FRAME_R)
    rounded_rect(sk, xa, -FINGER_Y[1] + 0.3, xb, FINGER_Y[1] - 0.3, FRAME_R)
    extrude(comp, all_profiles(sk), FRAME_T, NEW)
    bodies = [comp.bRepBodies.item(i) for i in range(comp.bRepBodies.count)]
    if len(bodies) > 1:
        join(comp, bodies[0], bodies[1:])
    if CASTER_IN_FRONT:  # fingers, IR bar and caster bar enclose the header gap: reopen it
        rounded_box(comp, xb, -FINGER_Y[0], Z_FB - 1.0, CASTER_X - CASTER_BAR_L / 2, FINGER_Y[0],
                    Z_FT + 1.0, FRAME_R, CUT, "header window")

    # Under the plate: lips the gearbox faces rest against, pads that bring the caster
    # plate and the IR standoffs to their heights.
    hw = MOTOR_W / 2 + MOTOR_FIT + LIP_T
    for sy in (-1, 1):
        box(comp, -hw, sy * Y_GEARBOX, Z_FB - MOTOR_H, hw, sy * HALF_W, Z_FB, JOIN, "lip")
    rounded_box(comp, CASTER_X - CASTER_BAR_L / 2, -CASTER_BAR_HW, Z_FB - CASTER_PAD,
                CASTER_X + CASTER_BAR_L / 2, CASTER_BAR_HW, Z_FB, FRAME_R, JOIN, "caster pad")
    # The arm carries the board's front corners onto the caster: same depth as the
    # caster pad under the spine, from the front pads to the IR bar.
    rounded_box(comp, xf, -SPINE_W / 2, Z_FB - CASTER_PAD, xa + 2.0, SPINE_W / 2, Z_FB, FRAME_R,
                JOIN, "keel")
    if CASTER_IN_FRONT:  # the fingers carry the caster: ribs under them, as deep as the keel
        for sy in (-1, 1):
            y0, y1 = sorted((sy * FINGER_Y[0], sy * FINGER_Y[1]))
            rounded_box(comp, xa + 2.0, y0, Z_FB - CASTER_PAD, x_finger, y1, Z_FB, FRAME_R,
                        JOIN, "finger rib")
    for x, y in SENSOR_HOLES:
        y0, y1 = sorted((math.copysign(FINGER_Y[0], y), math.copysign(FINGER_Y[1], y)))
        rounded_box(comp, x - SENSOR_PAD_W / 2, y0, Z_FB - SENSOR_PAD, x + SENSOR_PAD_W / 2, y1,
                    Z_FB, FRAME_R, JOIN, "IR pad")

    cylinders(comp, BOARD_HOLES + STRAP_HOLES, M3, Z_FB - 1.0, Z_FT + 1.0, CUT, "screw holes")
    for sy in (-1, 1):  # shaft slot, open at the bottom: the motor goes in from below
        ys = sy * (HALF_W - LIP_T / 2)
        sk = side_sketch(comp, "y", ys, "shaft slot")
        sk.sketchCurves.sketchLines.addTwoPointRectangle(
            sk.modelToSketchSpace(P(-SHAFT_SLOT_W / 2, ys, Z_FB - MOTOR_H - 1.0)),
            sk.modelToSketchSpace(P(SHAFT_SLOT_W / 2, ys, AXLE_Z)))
        sk.sketchCurves.sketchCircles.addByCenterRadius(sk.modelToSketchSpace(P(0.0, ys, AXLE_Z)),
                                                        SHAFT_SLOT_W / 2 * MM)
        extrude_symmetric(comp, all_profiles(sk), LIP_T + 2.0, CUT)

    # IR standoff studs come up through the pads; their nuts sink into the top so the
    # 6 mm studs reach. The pockets open toward the header gap.
    cylinders(comp, SENSOR_HOLES, M3, Z_FB - SENSOR_PAD - 1.0, Z_FT + 1.0, CUT, "IR holes")
    if SENSOR_NUT_DEPTH > 0.0:
        sk = sketch_at(comp, Z_FT - SENSOR_NUT_DEPTH, "IR nut pockets")
        for x, y in SENSOR_HOLES:
            hexagon(sk, x, y, NUT_AF)
        extrude(comp, all_profiles(sk), SENSOR_NUT_DEPTH + 1.0, CUT)

    # Caster screws come down through a slot (any hole spacing); their heads sink into
    # a channel so M3 x 6 reach the caster's nuts.
    sk = sketch_at(comp, Z_FB - CASTER_PAD - 1.0, "caster slot")
    slot(sk, CASTER_X, -CASTER_SLOT_DY / 2, CASTER_X, CASTER_SLOT_DY / 2, M3)
    extrude(comp, all_profiles(sk), CASTER_PAD + FRAME_T + 2.0, CUT)
    if CASTER_CHANNEL > 0.0:
        w = HEAD_D + 1.0
        y = CASTER_SLOT_DY / 2 + w / 2
        box(comp, CASTER_X - w / 2, -y, Z_FT - CASTER_CHANNEL, CASTER_X + w / 2, y, Z_FT + 1.0,
            CUT, "caster head channel")
    return comp


def build_straps(root):
    """U under the gearbox; its tabs hold the nuts and bear on the frame underside, the
    screws come down through the frame. Same part both sides: the second copy is the
    first moved across."""
    comp = new_component(root, "Motor strap")
    hw = MOTOR_W / 2 + MOTOR_FIT
    zt = Z_FB - STRAP_TAB_H                     # tab underside
    zi = Z_FB - MOTOR_H + STRAP_SQUEEZE         # strap floor, under the motor
    zo = zi - STRAP_T
    xl, xt = hw + STRAP_T, STRAP_TAB_X
    sk = side_polygon(comp, "y", STRAP_Y,
                      [(-xt, Z_FB), (-hw, Z_FB), (-hw, zi), (hw, zi), (hw, Z_FB), (xt, Z_FB),
                       (xt, zt), (xl, zt), (xl, zo), (-xl, zo), (-xl, zt), (-xt, zt)], "profile")
    extrude_symmetric(comp, profile(sk), STRAP_W, NEW)
    pts = [(sx * STRAP_SCREW_X, STRAP_Y) for sx in (-1, 1)]
    sk = sketch_at(comp, zt - 1.0, "nut traps")
    for x, y in pts:
        hexagon(sk, x, y, NUT_AF)
    extrude(comp, all_profiles(sk), NUT_H + 1.0, CUT)
    cylinders(comp, pts, M3, zt - 1.0, Z_FB + 1.0, CUT, "screw holes")
    m = adsk.core.Matrix3D.create()
    m.translation = adsk.core.Vector3D.create(0.0, -2.0 * STRAP_Y * MM, 0.0)
    root.occurrences.addExistingComponent(comp, m)
    return comp


def build_body(root):
    """Race-car shell. Each volume is a top-view outline cut by a side outline and
    rounded: the main body (between the wheels, wedge nose over the arm), a bubble over
    the battery and a pod each side ahead of the wheels. Joined, then hollowed with the
    bottom open. A front wing sits over the IR array, and a socket hangs over each
    board-corner post, with a chimney above it for the lock screw."""
    comp = new_component(root, "Body")
    zb, hw, top = Z_BODY, BODY_HW, 90.0   # top: above everything, the side outlines trim it
    main = prism(comp, [(-28.5, -38.0), (-24.0, -hw), (97.0, -hw), (128.0, -16.0), (156.0, -16.0),
                        (162.0, -8.0), (162.0, 8.0), (156.0, 16.0), (128.0, 16.0), (97.0, hw),
                        (-24.0, hw), (-28.5, 38.0)], zb, top, "main")
    main = trim_to_side(comp, main, [(-35.0, zb), (-35.0, BODY_ROOF - 12.2), (-24.0, BODY_ROOF),
                                     (96.0, BODY_ROOF), (114.0, BODY_NOSE_Z + 2.5),
                                     (150.0, BODY_NOSE_Z), (166.0, BODY_NOSE_Z - 7.5), (166.0, zb)],
                        "main side")
    round_edges(comp, main, 5.0, zb)

    xb0, xb1 = BATT_X - BATT_L / 2, BATT_X + BATT_L / 2
    w = BATT_W / 2 + BATT_GAP + BODY_WALL + 0.2
    zt = Z_BT + TALLEST + BATT_H + BATT_GAP + BODY_WALL
    bubble = prism(comp, [(xb0 - 13, -w + 4), (xb0 - 6, -w), (xb1 + 2, -w), (xb1 + 26, -10.0),
                          (xb1 + 26, 10.0), (xb1 + 2, w), (xb0 - 6, w), (xb0 - 13, w - 4)],
                   zb, top, "bubble")
    bubble = trim_to_side(comp, bubble, [(xb0 - 18, zb), (xb0 - 18, BODY_ROOF - 9.5), (xb0 - 3, zt),
                                         (xb1, zt), (xb1 + 31, BODY_ROOF - 3.7), (xb1 + 31, zb)],
                          "bubble side")
    round_edges(comp, bubble, 6.0, zb)

    pods = []
    for sy in (-1, 1):
        y0, y1 = sorted((sy * (hw - 4.0), sy * (WHEEL_Y + WHEEL_W / 2 - 0.7)))
        pod = prism(comp, [(28.0, y0), (90.0, y0), (90.0, y1), (28.0, y1)], zb, top,
                    "pod L" if sy > 0 else "pod R")
        pod = trim_to_side(comp, pod, [(24.0, zb), (24.0, 35.4), (42.0, 47.0), (86.0, 47.0),
                                       (94.0, 39.0), (94.0, zb)], "pod side")
        round_edges(comp, pod, 3.0, zb)
        pods.append(pod)
    join(comp, main, [bubble] + pods)

    hollow_with_sockets(comp, main, top)

    # Front wing over the IR array, behind the IR plug.
    sk = sketch_at(comp, zb, "front wing")
    polygon(sk, [(149.0, -29.0), (155.0, -35.0), (165.0, -35.0), (165.0, 35.0), (155.0, 35.0),
                 (149.0, 29.0)])
    extrude(comp, profile(sk), 1.6, JOIN)
    for sy in (-1, 1):
        y0, y1 = sorted((sy * 33.4, sy * 35.0))
        box(comp, 155.0, y0, zb, 166.0, y1, zb + 9.0, JOIN, "wing end plate")
    cut_sockets(comp, top)
    build_body_posts(root)
    return comp


def hollow_with_sockets(comp, main, top, bosses=(), web_reach=None, extra=()):
    """Hollow main with the bottom open, then hang the post sockets (webbed out to
    |y| = web_reach, trimmed to the side walls), the rear wing bosses, any extra
    bosses (x, y) and any extra bodies from the skin."""
    zb, hw = Z_BODY, web_reach or BODY_HW + 1.0
    # Keep a solid copy of the outside to trim the inner parts to the skin, then hollow.
    outer = comp.features.copyPasteBodies.add(main).bodies.item(0)
    open_shell(comp, main, BODY_WALL, zb)

    zs = Z_POST_TOP - POST_GRIP
    inner = list(extra)
    for x, y in BOARD_HOLES:
        inner.append(cylinders(comp, [(x, y)], SOCKET_D, zs, top, NEW, "socket").bodies.item(0))
        y0, y1 = sorted((y, math.copysign(hw, y)))
        inner.append(box(comp, x - 1.0, y0, zs, x + 1.0, y1, top, NEW, "socket web").bodies.item(0))
    for sy in (-1, 1):
        inner.append(cylinders(comp, [(WING_X, sy * WING_Y)], WING_PIN_D + 4.0, Z_BT + TALLEST + 0.5,
                               top, NEW, "wing boss").bodies.item(0))
    for x, y in bosses:
        inner.append(cylinders(comp, [(x, y)], WING_PIN_D + 4.0, zb, top, NEW, "boss").bodies.item(0))
    for b in inner:
        intersect_keep(comp, b, outer)
    join(comp, main, inner)
    comp.features.removeFeatures.add(outer)


def cut_sockets(comp, top, chimney_top=None):
    """Sockets: the bore grips the post's hex corners in any rotation, a slit lets the
    inner half spring; the lock screw goes down the chimney into the post. A chimney
    that stops at chimney_top (under the skin) only lightens the socket."""
    zs = Z_POST_TOP - POST_GRIP
    for x, y in BOARD_HOLES:
        cylinders(comp, [(x, y)], POST_AF * 2 / math.sqrt(3) + POST_FIT, zs - 1.0, Z_POST_TOP, CUT,
                  "socket bore")
        cylinders(comp, [(x, y)], M3, Z_POST_TOP - 0.1, Z_POST_TOP + SOCKET_TOP + 0.1, CUT,
                  "lock screw hole")
        cylinders(comp, [(x, y)], CHIMNEY_D, Z_POST_TOP + SOCKET_TOP, chimney_top or top, CUT, "chimney")
        r = SOCKET_D / 2 + 0.5
        box(comp, x - r, y - 0.3, zs - 1.0, x + r, y + 0.3, zs + POST_GRIP - 1.5, CUT, "socket slit")
    cylinders(comp, [(WING_X, sy * WING_Y) for sy in (-1, 1)], WING_PIN_D + 0.2, Z_BT + TALLEST,
              top, CUT, "wing pin holes")


def build_body_posts(root):
    posts = new_component(root, "Body posts (ref)")
    sk = sketch_at(posts, Z_BT, "F-F 10")
    for x, y in BOARD_HOLES:
        hexagon(sk, x, y, POST_AF)
    extrude(posts, all_profiles(sk), POST_L, NEW)


def ccw(pts):
    """pts in counter-clockwise order."""
    area = sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]))
    return pts if area > 0 else pts[::-1]


def inset(pts, d):
    """Convex counter-clockwise polygon moved in by d."""
    n = len(pts)
    lines = []
    for i in range(n):
        (x0, y0), (x1, y1) = pts[i], pts[(i + 1) % n]
        l = math.hypot(x1 - x0, y1 - y0)
        nx, ny = -(y1 - y0) / l, (x1 - x0) / l          # inward for counter-clockwise
        lines.append(((x0 + nx * d, y0 + ny * d), (x1 - x0, y1 - y0)))
    out = []
    for i in range(n):
        (p, u), (q, v) = lines[i - 1], lines[i]
        t = ((q[0] - p[0]) * v[1] - (q[1] - p[1]) * v[0]) / (u[0] * v[1] - u[1] * v[0])
        out.append((p[0] + t * u[0], p[1] + t * u[1]))
    return out


def canopy_outline():
    xb0, xb1 = BATT_X - BATT_L / 2, BATT_X + BATT_L / 2
    w = BATT_W / 2 + BATT_GAP + BODY_WALL + 0.2
    return [(xb0 - 13, -w + 4), (xb0 - 6, -w), (xb1 + 2, -w), (xb1 + 26, -10.0),
            (xb1 + 26, 10.0), (xb1 + 2, w), (xb0 - 6, w), (xb0 - 13, w - 4)]


def canopy_ring(d):
    """The canopy's footprint on the deck moved in by d (its ends sit 1.5 mm inside the
    outline's ends)."""
    pts = inset(canopy_outline(), d)
    x0 = canopy_outline()[0][0] + 1.5 + d
    x1 = canopy_outline()[3][0] - 1.5 - d
    return [(min(max(x, x0), x1), y) for x, y in pts]


def chamfer_deck(comp, body, z, d):
    """Chamfer the edges where the flat deck at z meets a vertical wall."""
    edges = adsk.core.ObjectCollection.create()
    for i in range(body.faces.count):
        f = body.faces.item(i)
        p = f.pointOnFace
        if abs(p.z - z * MM) > 1e-4 or f.evaluator.getNormalAtPoint(p)[1].z < 0.99:
            continue
        for j in range(f.edges.count):
            e = f.edges.item(j)
            other = [e.faces.item(k) for k in range(e.faces.count) if e.faces.item(k) != f]
            if other and abs(other[0].evaluator.getNormalAtPoint(other[0].pointOnFace)[1].z) < 1e-3:
                edges.add(e)
    chamfers = comp.features.chamferFeatures
    inp = chamfers.createInput2()
    inp.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, adsk.core.ValueInput.createByReal(d * MM),
                                                       False)
    chamfers.add(inp)


NOSE_BOSS_X = 151.0      # front wing pins, inside the nose in front of the IR plug
NOSE_BOSS_Y = 9.0


def build_body_split(root):
    """Print-friendly body in parts, none needing supports:
    Body shell   one flowing outline: narrow past the wheels, full width between them and
                 the nose, corners rounded from above. Flat deck at BODY_ROOF, 45 deg
                 chamfers and a 45 deg wedge nose instead of rounding; prints upside down
                 on its deck, sockets growing up from it
    Canopy       the bubble over the battery; its skirt plugs into an opening in the
                 deck. Prints upright (the 34 mm roof bridges)
    Front wing   pins up into two bosses inside the nose; prints flat"""
    comp = new_component(root, "Body shell")
    zb, hw, top = Z_BODY, BODY_HW, 90.0
    x_wedge = 128.0                       # deck ends where the nose stops narrowing
    half = [(-28.5, 38.0), (-24.0, hw), (18.0, hw), (34.0, 50.0), (92.0, 50.0), (x_wedge, 16.0),
            (165.0, 16.0)]
    main = prism(comp, [(x, -y) for x, y in half] + [(x, y) for x, y in reversed(half)], zb, top, "main")
    # Round the corners from the wheels forward. The rear ones stay sharp: the rear slope
    # starts mid-corner there, which breaks the deck chamfer.
    main = round_vertical(comp, main, lambda x, y: None if x < 0 or x > 110 else 20.0 if x < 50 else 22.0)
    main = trim_to_side(comp, main, [(-35.0, zb), (-35.0, BODY_ROOF - 11.0), (-24.0, BODY_ROOF),
                                     (x_wedge, BODY_ROOF), (x_wedge + BODY_ROOF - zb, zb)], "main side")
    chamfer_deck(comp, main, BODY_ROOF, 5.0)
    hollow_with_sockets(comp, main, top, [(NOSE_BOSS_X, sy * NOSE_BOSS_Y) for sy in (-1, 1)])
    cut_sockets(comp, top)
    cylinders(comp, [(NOSE_BOSS_X, sy * NOSE_BOSS_Y) for sy in (-1, 1)], WING_PIN_D + 0.2, zb - 1.0,
              zb + 3.0, CUT, "front wing pin holes")
    sk = sketch_at(comp, BODY_ROOF - 5.0, "canopy opening")
    polygon(sk, canopy_ring(1.0))
    extrude(comp, profile(sk), 10.0, CUT)

    # Canopy: bubble shell over the battery, sitting on the deck, with a skirt that
    # plugs into the opening.
    can = new_component(root, "Canopy")
    xb0, xb1 = BATT_X - BATT_L / 2, BATT_X + BATT_L / 2
    zt = Z_BT + TALLEST + BATT_H + BATT_GAP + BODY_WALL
    # The slopes reach the deck 1.5 mm inside the outline's ends, so the bubble ends in
    # clean edges on the deck (no knife edges for the fillets).
    bubble = prism(can, canopy_outline(), BODY_ROOF, top, "bubble")
    bubble = trim_to_side(can, bubble, [(xb0 - 16.5, BODY_ROOF - 9.5), (xb0 - 1.5, zt), (xb1 - 1.5, zt),
                                        (xb1 + 29.5, BODY_ROOF - 3.7)], "bubble side")
    round_edges(can, bubble, 6.0, BODY_ROOF)
    open_shell(can, bubble, BODY_WALL, BODY_ROOF)
    sk = sketch_at(can, BODY_ROOF - 3.0, "skirt")
    polygon(sk, canopy_ring(1.15))
    extrude(can, profile(sk), 3.8, JOIN)
    sk = sketch_at(can, BODY_ROOF, "rim")
    polygon(sk, canopy_ring(0.0))
    extrude(can, profile(sk), 0.8, JOIN)
    sk = sketch_at(can, BODY_ROOF - 4.0, "skirt inside")
    polygon(sk, canopy_ring(1.95))
    extrude(can, profile(sk), 4.8, CUT)

    # Front wing under the nose tip, behind which the IR plug stays clear.
    fw = new_component(root, "Front wing")
    z0 = zb - 1.2                         # 0.8 mm over the arm's fingers
    sk = sketch_at(fw, z0, "front wing")
    polygon(sk, [(149.0, -29.0), (155.0, -35.0), (166.0, -35.0), (166.0, 35.0), (155.0, 35.0),
                 (149.0, 29.0)])
    extrude(fw, profile(sk), 1.2, NEW)
    for sy in (-1, 1):
        y0, y1 = sorted((sy * 33.4, sy * 35.0))
        box(fw, 156.0, y0, z0, 166.0, y1, zb + 6.0, JOIN, "end plate")
    cylinders(fw, [(NOSE_BOSS_X, sy * NOSE_BOSS_Y) for sy in (-1, 1)], WING_PIN_D, zb - 0.5, zb + 2.6,
              JOIN, "pins")
    build_body_posts(root)
    return comp


def build_body_wing(root, seat=BODY_ROOF, z0=None):
    """Rear wing: plate, end plates and two struts standing on the body at Z seat, whose
    pins plug into the body's tail. z0 is the plate underside."""
    comp = new_component(root, "Body wing")
    z0 = z0 or seat + 11.0
    box(comp, WING_X - 12.0, -40.0, z0, WING_X + 6.0, 40.0, z0 + 2.0, NEW, "wing")
    for sy in (-1, 1):
        y0, y1 = sorted((sy * 38.4, sy * 40.0))
        box(comp, WING_X - 14.0, y0, z0 - 8.0, WING_X + 8.0, y1, z0 + 2.0, JOIN, "end plate")
        y0, y1 = sorted((sy * (WING_Y - 1.5), sy * (WING_Y + 1.5)))
        box(comp, WING_X - 3.0, y0, seat, WING_X + 3.0, y1, z0, JOIN, "strut")
    # pins stop short of the hole bottoms (Z_BT + TALLEST)
    cylinders(comp, [(WING_X, sy * WING_Y) for sy in (-1, 1)], WING_PIN_D, max(seat - 3.0, Z_BT + TALLEST + 0.4),
              seat + 0.5, JOIN, "pins")
    return comp


# Le Mans hypercar body. Body sections, rear to nose: (x, half width at the sill, y of
# the fender peak, fender peak z, centre-line z). Over the board the sections clear the
# parts envelope by the wall and ~1.8 mm.
HYPER_BODY = (
    (-31.0, 47.0, 36.0, 55.0, 54.0),
    (-12.0, 52.5, 44.0, 58.5, 55.5),    # rear haunch over the wheel
    (10.0, 52.5, 44.0, 58.5, 55.5),
    (40.0, 50.5, 40.0, 57.5, 55.5),     # waist
    (70.0, 50.0, 40.0, 57.0, 55.5),
    (97.0, 50.0, 40.0, 57.0, 55.5),
    (120.0, 49.0, 37.0, 51.0, 46.0),    # front fenders above a lower bonnet
    (142.0, 47.0, 35.0, 43.0, 37.0),    # IR plug and wires run under here
    (163.0, 42.0, 31.0, 30.5, 28.5),
)
# Cockpit over the battery: (x, half width, top z), lofted up from a base inside the body
HYPER_CANOPY_BASE = 50.0
HYPER_CANOPY = (
    (-16.0, 6.0, 60.0),
    (-9.0, 15.0, 70.0),
    (-3.0, 21.0, 76.0),
    (55.0, 21.0, 76.0),
    (70.0, 17.0, 70.5),
    (84.0, 12.0, 64.0),
    (98.0, 5.0, 57.0),
)
HYPER_WING_Z = 83.0      # rear wing underside, level with the cockpit top
HYPER_ANGULAR = True     # body sections as straight facets (crisp creases); False: smooth
ARCH_R = 19.5            # wheel arch radius (wheel 16.65)
ARCH_Y = 44.0            # arches cut from here outward (wheel inner face 44.5)
LOUVRE_X = (116.0, 122.0, 128.0)   # slots across each front fender
LOUVRE_W = 2.2
LOUVRE_Y = (36.0, 47.0)
SIDE_INTAKE = ((24.0, 36.0), (40.0, 40.0), (40.0, 47.0), (31.0, 45.0))  # (x, z) ahead of the rear wheels
# Headlights: an angled recess in each front fender with a 3 mm LED pushed in from
# inside; its rim stops on the back of a block, the lens just proud of the recess face.
HEADLIGHT_FACE_X = 154.0
HEADLIGHT_EYE = ((25.5, 33.5), (30.0, 30.5), (39.0, 34.0), (39.0, 37.5), (33.0, 36.0))  # (y, z), left side
HEADLIGHT_LED = (32.5, 33.65)       # (y, z) of the LED axis, left side
LED_D = 3.2                          # 3 mm LED, printed a little loose
LED_LENS_L = 5.3                     # rim to lens tip
LENS_GAP = 0.15                      # clear lens to the recess walls (glue it in)
# Cockpit windows, cut out of the body as their own part so the AMS prints them clear
# while the pillars and roof spine stay body colour.
SIDE_WINDOW = ((4.0, 66.0), (16.0, 73.5), (46.0, 73.5), (56.0, 68.0), (50.0, 63.5), (12.0, 63.5))  # (x, z)
SIDE_WINDOW_Y = (8.0, 30.0)
WINDSCREEN = ((62.0, 15.0), (90.0, 9.0), (90.0, -9.0), (62.0, -15.0))  # (x, y)
WINDSCREEN_Z = (59.5, 90.0)

def hyper_section(w, yf, hf, hc, zb):
    """Right half of a section, sill to centre line, as (y, z) points for a spline."""
    return [(w, zb), (w + 0.8, zb + 0.4 * (hf - zb)), (yf + 0.45 * (w - yf), hf - 0.15 * (hf - zb)),
            (yf, hf), (0.5 * yf, hc + 0.15 * (hf - hc)), (0.0, hc)]


def section_profile(comp, x, half, name, smooth=True):
    """Closed section on the plane X = x: a spline (or straight facets) over the top, a
    line along the bottom."""
    pts = half + [(-y, z) for y, z in reversed(half[:-1])]
    if not smooth:
        return profile(side_polygon(comp, "x", x, pts, name))
    sk = side_sketch(comp, "x", x, name)
    coll = adsk.core.ObjectCollection.create()
    for y, z in pts:
        coll.add(sk.modelToSketchSpace(P(x, y, z)))
    sp = sk.sketchCurves.sketchFittedSplines.add(coll)
    sk.sketchCurves.sketchLines.addByTwoPoints(sp.endSketchPoint, sp.startSketchPoint)
    return profile(sk)


def loft(comp, profiles):
    lf = comp.features.loftFeatures
    inp = lf.createInput(NEW)
    for p in profiles:
        inp.loftSections.add(p)
    inp.isSolid = True
    return lf.add(inp).bodies.item(0)


def window_tools(comp, grow):
    """Bodies through the window outlines (2 side windows, then the windscreen), the
    outlines grown by grow mm (negative shrinks)."""
    tools = []
    side = inset(ccw(list(SIDE_WINDOW)), -grow)
    for sy in (-1, 1):
        sk = side_polygon(comp, "y", sy * sum(SIDE_WINDOW_Y) / 2, side, "side window")
        tools.append(extrude_symmetric(comp, profile(sk), SIDE_WINDOW_Y[1] - SIDE_WINDOW_Y[0] + 2 * abs(grow),
                                       NEW).bodies.item(0))
    tools.append(prism(comp, inset(ccw(list(WINDSCREEN)), -grow), WINDSCREEN_Z[0] - grow, WINDSCREEN_Z[1],
                       "windscreen"))
    return tools


def cut_with(comp, target, tools, keep=True):
    coll = adsk.core.ObjectCollection.create()
    for t in tools:
        coll.add(t)
    inp = comp.features.combineFeatures.createInput(target, coll)
    inp.operation = CUT
    inp.isKeepToolBodies = keep
    comp.features.combineFeatures.add(inp)


def build_windows(root):
    """Windscreen and side windows as a colour region of the body print: the body skin
    inside each outline becomes the Windows part and leaves matching holes in the body.
    Exported in register with body.stl, so the two load as one object (multi-colour,
    same filament type)."""
    body_comp = root.parentDesign.allComponents.itemByName("Body")
    body = body_comp.bRepBodies.item(0)
    comp = new_component(root, "Windows")
    skin = comp.features.copyPasteBodies.add(body).bodies.item(0)
    for t in window_tools(comp, 0.0):
        intersect_keep(comp, t, skin)
    comp.features.removeFeatures.add(skin)
    cut_with(body_comp, body, window_tools(body_comp, 0.0), keep=False)
    return comp


def fit_strut_feet(wing, body):
    """Trim the wing's strut feet to the curved body they stand on."""
    tool = wing.features.copyPasteBodies.add(body).bodies.item(0)
    coll = adsk.core.ObjectCollection.create()
    coll.add(tool)
    inp = wing.features.combineFeatures.createInput(wing.bRepBodies.item(0), coll)
    inp.operation = CUT
    wing.features.combineFeatures.add(inp)


def top_z_at(root, body, x, y):
    """Z of body's top surface above (x, y)."""
    hits = adsk.core.ObjectCollection.create()
    faces = root.findBRepUsingRay(P(x, y, 200.0), adsk.core.Vector3D.create(0, 0, -1),
                                  adsk.fusion.BRepEntityTypes.BRepFaceEntityType, -1.0, False, hits)
    name = body.parentComponent.name
    return max(hits.item(i).z for i in range(hits.count)
               if faces.item(i).body.parentComponent.name == name) / MM


def build_body_hypercar(root):
    """Le Mans hypercar body in one piece, lofted through HYPER_BODY with the cockpit
    lofted through HYPER_CANOPY and a shark fin up to the rear wing. Wheel arches over
    the rear wheels. Same post sockets and wing bosses as the other bodies (no lock
    screws: their chimneys stop under the skin). Prints upright on its open bottom with
    tree supports. The splitter under the nose is the Front wing part. Returns the
    lowest height of the body under the rear wing struts (fit_strut_feet trims them to
    the rest)."""
    comp = new_component(root, "Body")
    zb, top = Z_BODY, 100.0
    main = loft(comp, [section_profile(comp, x, hyper_section(w, yf, hf, hc, zb), "body", not HYPER_ANGULAR)
                       for x, w, yf, hf, hc in HYPER_BODY])
    zc = HYPER_CANOPY_BASE
    cockpit = loft(comp, [section_profile(comp, x, hyper_section(w, 0.55 * w, t, t, zc), "cockpit")
                          for x, w, t in HYPER_CANOPY])
    sk = side_polygon(comp, "y", 0.0, [(-6.0, 60.0), (-4.0, 75.5), (-16.0, HYPER_WING_Z - 0.1),
                                       (-31.0, HYPER_WING_Z - 0.1), (-31.0, 54.0), (-6.0, 54.0)], "fin")
    fin = extrude_symmetric(comp, profile(sk), 1.2, NEW).bodies.item(0)
    join(comp, main, [cockpit, fin])
    for sy in (-1, 1):
        side_cylinder(comp, sy * (ARCH_Y + 15.0), 30.0, 0.0, AXLE_Z, 2 * ARCH_R, "arch", CUT)
    main = comp.bRepBodies.item(0)
    seat = min(top_z_at(root, main, WING_X + dx, sy * WING_Y + dy)
               for sy in (-1, 1) for dx in (-3.0, 0.0, 3.0) for dy in (-1.5, 0.0, 1.5))
    # Clear lenses filling the headlight recesses flush with the skin, on the recess
    # floor, with a pocket over the LED tip.
    for sy, side in ((1, "L"), (-1, "R")):
        lc = new_component(root, "Headlight lens " + side)
        eye = inset(ccw([(sy * y, z) for y, z in HEADLIGHT_EYE]), LENS_GAP)
        sk = side_polygon(lc, "x", HEADLIGHT_FACE_X + 10.0, eye, "lens")
        lens = extrude_symmetric(lc, profile(sk), 20.0, NEW).bodies.item(0)
        skin = lc.features.copyPasteBodies.add(main).bodies.item(0)
        intersect_keep(lc, lens, skin)
        lc.features.removeFeatures.add(skin)
        ly, lz = HEADLIGHT_LED
        sk = side_sketch(lc, "x", HEADLIGHT_FACE_X, "LED tip pocket")
        sk.sketchCurves.sketchCircles.addByCenterRadius(sk.modelToSketchSpace(P(HEADLIGHT_FACE_X, sy * ly, lz)),
                                                        (LED_D / 2 + 0.1) * MM)
        extrude_symmetric(lc, profile(sk), 1.2, CUT)

    # blocks behind the headlights, trimmed to the skin with the sockets
    x_rim = HEADLIGHT_FACE_X + 0.3 - LED_LENS_L
    blocks = []
    for sy in (-1, 1):
        y0, y1 = sorted((sy * 25.0, sy * 40.0))
        blocks.append(box(comp, x_rim, y0, 29.0, HEADLIGHT_FACE_X + 3.0, y1, 39.0, NEW,
                          "headlight block").bodies.item(0))
    hollow_with_sockets(comp, main, top, [(NOSE_BOSS_X, sy * NOSE_BOSS_Y) for sy in (-1, 1)], 60.0, blocks)
    cut_sockets(comp, top, Z_BT + TALLEST + 1.0)
    # Open the flat inner wall of each wheel arch so only the curved fender is left.
    for sy in (-1, 1):
        side_cylinder(comp, sy * (ARCH_Y - BODY_WALL / 2), BODY_WALL + 3.0, 0.0, AXLE_Z, 2 * ARCH_R,
                      "arch opening", CUT)
    cylinders(comp, [(NOSE_BOSS_X, sy * NOSE_BOSS_Y) for sy in (-1, 1)], WING_PIN_D + 0.2, zb - 1.0,
              zb + 3.0, CUT, "front wing pin holes")
    for sy in (-1, 1):
        y0, y1 = sorted((sy * LOUVRE_Y[0], sy * LOUVRE_Y[1]))
        for x in LOUVRE_X:
            box(comp, x - LOUVRE_W / 2, y0, 42.0, x + LOUVRE_W / 2, y1, 70.0, CUT, "louvre")
        sk = side_polygon(comp, "y", sy * 53.0, SIDE_INTAKE, "side intake")
        extrude_symmetric(comp, profile(sk), 18.0, CUT)
        # headlight recess, then the LED hole through the block behind it
        sk = side_polygon(comp, "x", HEADLIGHT_FACE_X + 10.0, [(sy * y, z) for y, z in HEADLIGHT_EYE], "headlight")
        extrude_symmetric(comp, profile(sk), 20.0, CUT)
        ly, lz = HEADLIGHT_LED
        sk = side_sketch(comp, "x", HEADLIGHT_FACE_X, "LED hole")
        sk.sketchCurves.sketchCircles.addByCenterRadius(sk.modelToSketchSpace(P(HEADLIGHT_FACE_X, sy * ly, lz)),
                                                        LED_D / 2 * MM)
        extrude_symmetric(comp, profile(sk), 2 * (LED_LENS_L + 2.0), CUT)

    # Splitter under the nose, starting behind the IR plug's reach like the front wing.
    fw = new_component(root, "Front wing")
    z0 = zb - 1.2
    sk = sketch_at(fw, z0, "splitter")
    polygon(sk, [(149.0, -40.0), (153.0, -46.0), (168.0, -46.0), (171.0, -40.0), (171.0, 40.0),
                 (168.0, 46.0), (153.0, 46.0), (149.0, 40.0)])
    extrude(fw, profile(sk), 1.2, NEW)
    cylinders(fw, [(NOSE_BOSS_X, sy * NOSE_BOSS_Y) for sy in (-1, 1)], WING_PIN_D, zb - 0.5, zb + 2.6,
              JOIN, "pins")
    build_body_posts(root)
    return seat


# ===========================================================================
# References (do not print)
# ===========================================================================

def build_motor_refs(root):
    comp = new_component(root, "N20 motors (ref)")
    za = AXLE_Z
    for sy in (-1, 1):
        yf = sy * Y_GEARBOX               # gearbox face
        yc = yf - sy * GEARBOX_L          # gearbox / can joint
        box(comp, -MOTOR_W / 2, yc, za - MOTOR_H / 2, MOTOR_W / 2, yf, za + MOTOR_H / 2, NEW,
            "gearbox")
        side_cylinder(comp, yc - sy * CAN_L / 2, CAN_L, 0.0, za, MOTOR_W, "can")
        side_cylinder(comp, yf + sy * SHAFT_OUT / 2, SHAFT_OUT, 0.0, za, 3.0, "shaft")
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
    cylinders(comp, SENSOR_HOLES, SPACER_D, SENSOR_PCB_TOP, SENSOR_PCB_TOP + SENSOR_SPACER, NEW,
              "IR standoffs")

    comp = new_component(root, "IR array (ref)")
    box(comp, SENSOR_REAR_X, -SENSOR_L / 2, SENSOR_PCB_TOP - SENSOR_T,
        SENSOR_REAR_X + SENSOR_W, SENSOR_L / 2, SENSOR_PCB_TOP, NEW, "pcb")
    box(comp, LOOKAHEAD - 1.5, -SENSOR_L / 2 + 3, SENSOR_FACE_Z,
        LOOKAHEAD + 1.5, SENSOR_L / 2 - 3, SENSOR_PCB_TOP - SENSOR_T, NEW, "eyes")

    comp = new_component(root, "Battery (ref)")
    z = Z_BT + TALLEST
    box(comp, BATT_X - BATT_L / 2, -BATT_W / 2, z, BATT_X + BATT_L / 2, BATT_W / 2, z + BATT_H,
        NEW, "battery")
    reference(comp)
    build_caster_ref(root)
    build_motor_refs(root)


def build_caster_ref(root):
    comp = new_component(root, "Ball caster (ref)")
    box(comp, CASTER_X - CASTER_PLATE_L / 2, -CASTER_PLATE_W / 2, CASTER_H - CASTER_PLATE_T,
        CASTER_X + CASTER_PLATE_L / 2, CASTER_PLATE_W / 2, CASTER_H, NEW, "plate")
    sk = sketch_at(comp, 0.0, "ball housing")
    circle(sk, CASTER_X, 0.0, 15.0)
    extrude(comp, profile(sk), CASTER_H - CASTER_PLATE_T, NEW)
    reference(comp)


# ===========================================================================
# Entry points
# ===========================================================================

PRINTED = ("Frame", "Motor strap", "Body", "Body shell", "Canopy", "Front wing", "Body wing",
           "Headlight lens L", "Headlight lens R",   # the lenses print on their flat backs
           "Windows")                                # in register: load with body.stl as one object
# the one-piece body, canopy and front wing print the way up they sit
FLIP_FOR_PRINT = ("Frame", "Motor strap", "Body shell", "Body wing")


def close_previous():
    """Close an unsaved design left by an earlier (failed) run of this script."""
    app = adsk.core.Application.get()
    doc = app.activeDocument
    design = adsk.fusion.Design.cast(app.activeProduct)
    if doc and not doc.isSaved and design and design.allComponents.itemByName("Frame"):
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
    # M3 x 6 reach, joint by joint
    if SCREW_L - FRAME_T < MIN_THREAD:
        raise ValueError("frame too thick for the board standoff screws")
    if BOARD_T + NUT_H > STUD_L - THREAD_SPARE:
        raise ValueError("board too thick for the standoff studs")
    if FRAME_T + STRAP_TAB_H > SCREW_L - THREAD_SPARE or STRAP_TAB_H - NUT_H < 0.6:
        raise ValueError("motor strap tabs don't suit the screws")
    if SENSOR_PAD < 0.0:
        raise ValueError("IR standoffs too long for the frame height")
    if FRAME_T + SENSOR_PAD - SENSOR_NUT_DEPTH < 1.5:
        raise ValueError("too little arm left under the IR standoff nuts")
    if CASTER_PAD < 0.0:
        raise ValueError("caster taller than the frame underside")
    if FRAME_T + CASTER_PAD - CASTER_CHANNEL < 1.2:
        raise ValueError("arm too thin under the caster screw heads")
    # Body: the board stud must hold the post, and the lock screw must bite without
    # reaching the stud.
    stud_in_post = STUD_L - BOARD_T
    if stud_in_post < MIN_THREAD:
        raise ValueError("board stud too short for the body posts")
    if SCREW_L - SOCKET_TOP < MIN_THREAD or SCREW_L - SOCKET_TOP > POST_L - stud_in_post:
        raise ValueError("body lock screw doesn't suit the post")
    if BODY_ROOF - BODY_WALL - (Z_BT + TALLEST) < 1.5:
        raise ValueError("body roof too low over the parts")
    # Layout
    if WHEEL_Y - WHEEL_W / 2 - BOARD_W / 2 < 2.0:
        raise ValueError("wheels too close to the board edge")
    if CASTER_IN_FRONT:
        if CASTER_X - CASTER_PLATE_L / 2 < SENSOR_FRONT_X:
            raise ValueError("caster runs into the IR array")
    elif SENSOR_REAR_X < CASTER_X + CASTER_PLATE_L / 2:
        raise ValueError("IR array runs into the caster")


def hill_limits():
    """Steepest sharp-edged ramp (deg) before the IR array touches it, at the foot and at
    the top of the ramp; None where it never touches."""
    low = [(LOOKAHEAD, SENSOR_FACE_Z),                          # eyes
           (SENSOR_REAR_X, SENSOR_PCB_TOP - SENSOR_T),          # PCB underside, rear edge
           (SENSOR_FRONT_X, SENSOR_PCB_TOP - SENSOR_T)]         # and front edge
    if CASTER_IN_FRONT:
        # Foot: the caster climbs first and lifts the array. Top: the array rides between
        # the wheels and the caster, closest to a crest right under it.
        top = min(math.asin(min(1.0, z * CASTER_X / (x * (CASTER_X - x)))) for x, z in low)
        return None, round(math.degrees(top), 1)
    # Foot: the array overhangs the caster. Top: it is ahead of both supports.
    foot = min(math.atan(z / (x - CASTER_X)) for x, z in low)
    return round(math.degrees(foot), 1), None


def summary():
    r = lambda v: round(v, 2)
    return {
        "length_x": [r(BOARD_HOLE_REAR_X - PAD_D / 2),
                     r(max(SENSOR_FRONT_X, CASTER_X + CASTER_PLATE_L / 2))],
        "ramp_deg_ir_touches_foot_top": hill_limits(),
        "width_over_wheels": r(2 * (WHEEL_Y + WHEEL_W / 2)),
        "track": r(2 * WHEEL_Y),
        "axle_to_caster_and_eyes": [r(CASTER_X), r(LOOKAHEAD)],
        "frame_underside_board_underside_z": [r(Z_FB), r(Z_BB)],
        "caster_pad_ir_pad": [r(CASTER_PAD), r(SENSOR_PAD)],
        "caster_channel_ir_nut_depth": [r(CASTER_CHANNEL), r(SENSOR_NUT_DEPTH)],
        "strap_screw_spare": r(SCREW_L - FRAME_T - STRAP_TAB_H),
    }


def build_all():
    check()
    new_design()
    root = root_component()
    build_frame(root)
    build_straps(root)
    build_references(root)
    if BODY_STYLE == "hypercar":
        seat = build_body_hypercar(root)
        build_windows(root)
        wing = build_body_wing(root, seat, HYPER_WING_Z)
        fit_strut_feet(wing, root.parentDesign.allComponents.itemByName("Body").bRepBodies.item(0))
    else:
        (build_body_split if BODY_STYLE == "split" else build_body)(root)
        build_body_wing(root)
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
        path = os.path.join(folder, name.lower().replace(" ", "_")
                            + (VARIANT if name == "Frame" else "") + ".stl")
        opts = em.createSTLExportOptions(comp if comp.bRepBodies.count > 1 else comp.bRepBodies.item(0), path)
        opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
        em.execute(opts)
        flip_stl(path, name in FLIP_FOR_PRINT, name.startswith("Headlight lens"),
                 Z_BODY if name == "Windows" else None)   # windows stay in register with the body
        done.append(path)
    return done


def flip_stl(path, flip=True, back_down=False, ground=None):
    """Turn a binary STL over (180 deg about X, not a mirror) if flip, or stand it on its
    rear face (X -> Z, a rotation) if back_down, and stand it on z = 0 (or move it down by
    ground mm, to keep it in register with another part)."""
    with open(path, "rb") as f:
        data = bytearray(f.read())
    n = struct.unpack_from("<I", data, 80)[0]
    tris = [list(struct.unpack_from("<12f", data, 84 + i * 50)) for i in range(n)]
    for t in tris:
        for k in range(0, 12, 3):  # normal, then the three vertices
            if flip:
                t[k + 1], t[k + 2] = -t[k + 1], -t[k + 2]
            elif back_down:
                t[k], t[k + 1], t[k + 2] = t[k + 1], t[k + 2], t[k]
    z0 = ground if ground is not None else min(t[k] for t in tris for k in (5, 8, 11))
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
