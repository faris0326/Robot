"""Fusion 360 generator: enclosed chassis for the line follower.

Run it in Fusion (Utilities > Scripts and Add-Ins) or through the
Fusion360MCP add-in (execute_code). It opens a new design and builds:

  Tray          floor + walls; the 12 x 8 cm perfboard sits on 4 bosses and the
                two N20 mounts (N20_Gear_Motor_Mount_-_Full_Length) bolt underneath
  Deck          lid on brass spacers above the perfboard; battery strapped on top
  Sensor boom   slides under the tray floor (2 slots, nuts trapped in the floor)
                to set the lookahead; carries the IR array on brass spacers and
                the ball caster
  Caster riser  only for a caster shorter than the boom underside; a taller one
                sits in a pocket in the boom
  References    wheels, motor mounts, perfboard, parts envelope, IR array,
                battery, caster, spacers: do not print

Everything is modelled in place in the robot frame: X forward, Y left, Z up,
origin on the ground under the middle of the rear axle. Dimensions in mm; the
Fusion API uses cm (MM). Print orientation is set in the slicer (deck and boom
print upside down).
"""

import math
import os

import adsk.core
import adsk.fusion

MM = 0.1

# --- Wheels and N20 mounts -------------------------------------------------
WHEEL_D = 33.3
WHEEL_W = 6.2
WHEEL_GAP = 1.5          # tray side wall to the inside face of the tyre
MOUNT_AXIS_DEPTH = 5.0   # motor axis below the mount face (open side of the pocket)
MOUNT_DEPTH = 12.0       # mount hangs this far below the floor
MOUNT_LEN = 26.0         # along the motor axis; shaft end flush with the tray side
MOUNT_FLANGE_X = 18.0    # flanges reach this far fore and aft of the axle
MOUNT_HOLE_X = 13.0
MOUNT_HOLE_IN = 9.0      # flange holes, in from the shaft end
MOUNT_STL = "ref/n20_mount_full_length.stl"  # next to this script; box if missing

# --- Perfboard and electronics ----------------------------------------------
BOARD_L = 120.0          # along X
BOARD_W = 80.0
BOARD_T = 1.6
BOARD_HOLE_DX = 113.5    # hole centres, long side
BOARD_HOLE_DY = 73.5     # hole centres, short side
BOSS_H = 5.0             # board underside above the floor: room for solder joints
BOSS_D = 7.0
TALLEST = 20.0           # tallest part above the board
DECK_SPACER = 26.0       # brass spacer stack, board top -> deck (20 + 6 M-F)
USB_FROM_FRONT = (41.0, 64.0)  # ESP32 USB-C ports (left side), from the board's front edge

# --- Tray and deck ----------------------------------------------------------
FLOOR_T = 2.5
WALL_T = 2.0
BOARD_CLEAR = 1.0        # board edge to wall
CORNER_R = 6.0
DECK_T = 2.5
DECK_GAP = 0.3           # wall top to deck, and locating lip to wall
LIP_T = 1.2
LIP_H = 3.0
SWITCH_W = 13.0          # 12.7 x 6.5 slide switch, press fit in the rear wall
SWITCH_H = 6.8
IR_CABLE_SLOT = (16.0, 8.0)  # front wall, IR array cable

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
SENSOR_SPACER = 10.0     # brass spacer, boom -> sensor PCB
HEADER_SLOT = (29.0, 4.5)  # 11-pin header + Dupont plug through the boom tip (Y, X)
TIP_MARGIN = 8.0         # boom tip beyond the sensor holes

# --- Ball caster (25 x 50 mm plate) ------------------------------------------
CASTER_PLATE_L = 25.0    # along X
CASTER_PLATE_W = 50.0
CASTER_H = 19.6          # plate face to the floor (measured)
CASTER_FIT = 0.4         # plate to pocket wall
CASTER_HOLE_DY = 38.0    # riser only (PLACEHOLDER); the boom slot takes any spacing
CASTER_SLOT_DY = 44.0    # slot in the boom takes any hole spacing up to this
CASTER_U = 112.0         # caster centre, from the boom's rear end

# --- Fasteners ----------------------------------------------------------------
M3 = 3.4
HEAD_D = 5.5
NUT_AF = 5.7
NUT_TRAP = 2.6
NUT_BOSS_D = 10.0
NUT_BOSS_H = 3.0
SPACER_D = 5.5

# --- Derived ----------------------------------------------------------------
AXLE_Z = WHEEL_D / 2
Z_FB = AXLE_Z + MOUNT_AXIS_DEPTH   # floor underside = mount face
Z_FT = Z_FB + FLOOR_T
Z_BB = Z_FT + BOSS_H               # board underside
Z_BT = Z_BB + BOARD_T
Z_DU = Z_BT + DECK_SPACER          # deck underside
Z_DT = Z_DU + DECK_T
WALL_TOP = Z_DU - DECK_GAP

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


def polygon(sk, pts):
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(P(*pts[0]), P(*pts[1]))
    prev = first
    for p in pts[2:]:
        prev = lines.addByTwoPoints(prev.endSketchPoint, P(*p))
    lines.addByTwoPoints(prev.endSketchPoint, first.startSketchPoint)


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


def extrude(comp, profs, height, op):
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(profs, op)
    inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal(height * MM))
    return ext.add(inp)


def extrude_symmetric(comp, profs, total, op):
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(profs, op)
    inp.setSymmetricExtent(adsk.core.ValueInput.createByReal(total * MM), True)
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
    sk = sketch_at(comp, Z_FB, "outline")
    rounded_rect(sk, TRAY_X0, -HALF_W, TRAY_X1, HALF_W, CORNER_R)
    extrude(comp, profile(sk), WALL_TOP - Z_FB, NEW)
    sk = sketch_at(comp, Z_FT, "inside")
    rounded_rect(sk, TRAY_X0 + WALL_T, -HALF_W + WALL_T, TRAY_X1 - WALL_T, HALF_W - WALL_T,
                 CORNER_R - WALL_T)
    extrude(comp, profile(sk), WALL_TOP - Z_FT + 1.0, CUT)

    cylinders(comp, BOARD_HOLES, BOSS_D, Z_FT, Z_BB, JOIN, "board bosses")
    boom_pts = [(x, 0.0) for x in BOOM_FLOOR_X]
    cylinders(comp, boom_pts, NUT_BOSS_D, Z_FT, Z_FT + NUT_BOSS_H, JOIN, "boom nut bosses")
    sk = sketch_at(comp, Z_FT + NUT_BOSS_H - NUT_TRAP, "boom nut traps")
    for x, y in boom_pts:
        hexagon(sk, x, y, NUT_AF)
    extrude(comp, all_profiles(sk), NUT_TRAP + 1.0, CUT)
    cylinders(comp, BOARD_HOLES + MOUNT_HOLES + boom_pts, M3, Z_FB - 1.0, Z_BB + 1.0, CUT,
              "screw holes")

    # Motor wires come up beside the motor's rear end.
    sk = sketch_at(comp, Z_FB - 1.0, "motor wire slots")
    for sy in (-1, 1):
        y_in = MOUNT_Y - MOUNT_LEN / 2
        rect(sk, -4.0, sy * (y_in - 1.5), 4.0, sy * (y_in + 4.0))
    extrude(comp, all_profiles(sk), FLOOR_T + 2.0, CUT)

    wall_window(comp, "y", HALF_W - WALL_T / 2,
                BOARD_X1 - USB_FROM_FRONT[1] - 3.0, BOARD_X1 - USB_FROM_FRONT[0] + 3.0,
                Z_BT + 1.0, Z_BT + 19.0, "USB window")
    z_sw = (Z_BT + WALL_TOP) / 2
    wall_window(comp, "x", TRAY_X0 + WALL_T / 2, -SWITCH_W / 2, SWITCH_W / 2,
                z_sw - SWITCH_H / 2, z_sw + SWITCH_H / 2, "switch")
    w, h = IR_CABLE_SLOT
    wall_window(comp, "x", TRAY_X1 - WALL_T / 2, -w / 2, w / 2, Z_BT, Z_BT + h, "IR cable")
    return comp


def build_deck(root):
    comp = new_component(root, "Deck")
    sk = sketch_at(comp, Z_DU, "outline")
    rounded_rect(sk, TRAY_X0, -HALF_W, TRAY_X1, HALF_W, CORNER_R)
    extrude(comp, profile(sk), DECK_T, NEW)

    # Locating lip just inside the walls, stopped short of the spacers.
    sk = sketch_at(comp, Z_DU - LIP_H, "lip")
    y_out = HALF_W - WALL_T - DECK_GAP
    x_a, x_b = BOARD_HOLE_REAR_X + 6.0, BOARD_HOLE_REAR_X + BOARD_HOLE_DX - 6.0
    for sy in (-1, 1):
        rect(sk, x_a, sy * (y_out - LIP_T), x_b, sy * y_out)
    y_e = BOARD_HOLE_DY / 2 - 6.0
    x_r = TRAY_X0 + WALL_T + DECK_GAP
    x_f = TRAY_X1 - WALL_T - DECK_GAP
    rect(sk, x_r, -y_e, x_r + LIP_T, y_e)
    rect(sk, x_f - LIP_T, -y_e, x_f, y_e)
    extrude(comp, all_profiles(sk), LIP_H, JOIN)

    sk = sketch_at(comp, Z_DU - 1.0, "holes")
    for x, y in BOARD_HOLES:
        circle(sk, x, y, M3)
    y_s = BATT_W / 2 + 0.3 + RAIL_T + 1.0
    for sy in (-1, 1):
        rect(sk, BATT_X - STRAP_W / 2, sy * y_s, BATT_X + STRAP_W / 2, sy * (y_s + STRAP_T))
    rect(sk, *CABLE_HOLE)
    extrude(comp, all_profiles(sk), DECK_T + 2.0, CUT)

    # Battery rails; the rear end stays open for the lead.
    sk = sketch_at(comp, Z_DT, "battery rails")
    xa, xb = BATT_X - BATT_L / 2 - 0.3, BATT_X + BATT_L / 2 + 0.3
    yb = BATT_W / 2 + 0.3
    for sy in (-1, 1):
        rect(sk, xa, sy * yb, xb, sy * (yb + RAIL_T))
    rect(sk, xb, -BATT_W / 4, xb + RAIL_T, BATT_W / 4)
    extrude(comp, all_profiles(sk), RAIL_H, JOIN)
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

    sk = sketch_at(comp, z_cut - 1.0, "slots and holes")
    for i in range(2):
        u0 = SLOT_MARGIN + i * SLOT_SPACING
        slot(sk, bx(u0), 0.0, bx(u0 + BOOM_TRAVEL), 0.0, M3)
    slot(sk, bx(CASTER_U), -CASTER_SLOT_DY / 2, bx(CASTER_U), CASTER_SLOT_DY / 2, M3)
    for x, y in sensor_pts:
        circle(sk, x, y, M3)
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


# ===========================================================================
# References (do not print)
# ===========================================================================

def add_mesh(comp, path):
    units = adsk.fusion.MeshUnits.MillimeterMeshUnit
    design = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    if design.designType == adsk.fusion.DesignTypes.ParametricDesignType:
        bf = comp.features.baseFeatures.add()
        bf.startEdit()
        try:
            comp.meshBodies.add(path, units, bf)
        finally:
            bf.finishEdit()
    else:
        comp.meshBodies.add(path, units)


def mount_matrix(side):
    """Mount file frame: pocket open at z = -6, motor axis along +Y to the shaft end."""
    m = adsk.core.Matrix3D.create()
    m.setWithCoordinateSystem(P(0.0, side * MOUNT_Y, Z_FB - 6.0),
                              adsk.core.Vector3D.create(-side, 0, 0),
                              adsk.core.Vector3D.create(0, side, 0),
                              adsk.core.Vector3D.create(0, 0, -1))
    return m


def build_mounts(root):
    occ = root.occurrences.addNewComponent(mount_matrix(1))
    comp = occ.component
    comp.name = "N20 mount (ref, print from its 3mf)"
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), MOUNT_STL) \
        if "__file__" in globals() else ""
    try:
        add_mesh(comp, path)
    except Exception:
        box(comp, -8.0, -MOUNT_LEN / 2, -6.0, 8.0, MOUNT_LEN / 2, 6.0, NEW, "mount box")
    root.occurrences.addExistingComponent(comp, mount_matrix(-1))


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
    cylinders(comp, BOARD_HOLES, SPACER_D, Z_BT, Z_DU, NEW, "deck spacers")
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


def build_caster_ref(root):
    comp = new_component(root, "Ball caster (ref)")
    x = bx(CASTER_U)
    box(comp, x - CASTER_PLATE_L / 2, -CASTER_PLATE_W / 2, CASTER_H - 2.0,
        x + CASTER_PLATE_L / 2, CASTER_PLATE_W / 2, CASTER_H, NEW, "plate")
    sk = sketch_at(comp, 0.0, "ball housing")
    circle(sk, x, 0.0, 15.0)
    extrude(comp, profile(sk), CASTER_H - 2.0, NEW)
    reference(comp)


# ===========================================================================
# Entry points
# ===========================================================================

PRINTED = ("Tray", "Deck", "Sensor boom", "Caster riser")


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


def summary():
    r = lambda v: round(v, 2)
    return {
        "length_x": [r(TRAY_X0), r(SENSOR_REAR_X + SENSOR_W)],
        "width_with_wheels": r(2 * (WHEEL_Y + WHEEL_W / 2)),
        "track": r(2 * WHEEL_Y),
        "tray": [r(TRAY_X1 - TRAY_X0), r(2 * HALF_W), r(WALL_TOP - Z_FB)],
        "deck_top_z": r(Z_DT),
        "lookahead_range": [r(LOOKAHEAD - BOOM_TRAVEL / 2), r(LOOKAHEAD + BOOM_TRAVEL / 2)],
        "wheelbase_to_caster": r(bx(CASTER_U)),
        "floor_underside_z": r(Z_FB),
        "boom_underside_z": r(Z_BOOM_B),
        "sensor_pad": r(SENSOR_PAD),
        "caster_riser_h": r(max(0.0, RISER_H)),
        "caster_pocket": r(CASTER_POCKET),
        "board_holes": [[r(x), r(y)] for x, y in BOARD_HOLES],
    }


def build_all():
    check()
    new_design()
    root = root_component()
    build_tray(root)
    build_deck(root)
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
