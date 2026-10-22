"""Fusion 360 generator: 5:1 step-up wheel drive for one N20 motor.

Run it in Fusion (Utilities > Scripts and Add-Ins > + > this file) or through
the Fusion360MCP add-in (execute_code). It opens a new design and builds:

  Big gear (60T)     press-fits on the N20 output D-shaft
  Pinion wheel       12T pinion + wheel rim in one print, turns on 2x MR63ZZ
  Bracket            plate holding motor face + axle at the gear centre distance
  Spacer A / B       inner-race spacers for the bearings (print, tiny)
  N20 / tyre (ref)   reference bodies only, do not print

World frame: Z up, floor at Z = 0, robot forward = +X, +Y = outboard. The
bracket plate's outer face is Y = 0. Everything is in mm here; the Fusion API
works in cm, see MM. The other side of the robot uses a mirrored bracket
(mirror it in the slicer); gears and wheel are the same on both sides.
"""

import math

import adsk.core
import adsk.fusion

MM = 0.1  # Fusion API length unit is cm

# --- Gear pair ------------------------------------------------------------
MODULE = 0.8           # smallest that prints cleanly with a 0.4 mm nozzle
PRESSURE_ANGLE = 20.0  # degrees
Z_PINION = 12
Z_GEAR = 60            # 60 / 12 = 5:1 step-up
X_PINION = +0.3        # profile shift: no undercut on the 12T pinion
X_GEAR = -0.3          # equal and opposite, centre distance stays m(z1+z2)/2
BACKLASH = 0.10        # tooth thickness removed per gear at the pitch circle
TIP_MIN = 0.40         # tips thinner than this get truncated (FDM can't print them)
CENTER_DISTANCE = MODULE * (Z_PINION + Z_GEAR) / 2  # 28.8

GEAR_FACE = 6.0
PINION_FACE = 7.5      # wider than the gear so small axial offsets still mesh
GAP_PLATE = 0.5        # gear and pinion start this far outboard of the plate
GEAR_WEB = 2.4         # web thickness of the big gear (rim and hub are full face)
GEAR_RIM = 2.0         # radial rim depth under the tooth roots
GEAR_HUB_D = 10.0

# --- N20 motor (measure yours if a part doesn't fit) -----------------------
SHAFT_BORE_D = 3.15    # 3 mm D-shaft, snug press fit after FDM shrink
SHAFT_FLAT = 2.55      # width across the flat (shaft is 2.5)
N20_BOSS_HOLE = 4.6    # clearance for the gearbox output bushing
N20_HOLE_SPACING = 9.0  # 2x M1.6 tapped holes on the gearbox face
N20_SCREW_HOLE = 1.8
N20_W, N20_H, N20_LEN = 12.0, 10.0, 32.0  # incl. encoder, reference only

# --- Wheel -----------------------------------------------------------------
WHEEL_R = 16.75        # radius with tyre = axle height (33.5 mm wheel)
RIM_D = 29.5           # tyre seat: MEASURE the inside diameter of your tyre
TYRE_W = 6.2           # 6 mm tyre (Little Craft 34 mm N20 wheel) + 0.2 clearance
LIP_T = 0.8            # side lips that keep the tyre on
LIP_H = 0.8
WHEEL_W = TYRE_W + 2 * LIP_T
WHEEL_WEB = 2.4

# --- Bearings / axle (M3 x 30 screw, fixed) --------------------------------
BRG_OD = 6.0           # MR63ZZ: 3 x 6 x 2.5
BRG_W = 2.5
BRG_POCKET_D = 6.1
WHEEL_BORE_D = 5.0     # clears spacer B inside the pinion
AXLE_HOLE_D = 3.3
SPACER_OD = 4.2        # touches inner races only
SPACER_ID = 3.3
SPACER_A_LEN = 1.0     # between screw head and outer bearing

# --- Layout ----------------------------------------------------------------
MOTOR_ANGLE = 30.0     # motor shaft above/behind the axle; keeps gear off the floor
PLATE_T = 3.0
FLANGE_DEPTH = 12.0    # inboard top flange for screwing to the chassis
FLANGE_HOLE_D = 3.3

AXLE = (0.0, WHEEL_R)  # world (x, z)
MOTOR = (-CENTER_DISTANCE * math.cos(math.radians(MOTOR_ANGLE)),
         WHEEL_R + CENTER_DISTANCE * math.sin(math.radians(MOTOR_ANGLE)))
Z_TOP = MOTOR[1] + N20_H / 2 + 0.5 + PLATE_T  # top of the chassis flange

# Along the axle (world Y): wheel outer face, bearing pocket bottom.
Y_WHEEL_OUT = GAP_PLATE + PINION_FACE + WHEEL_W
Y_POCKET = Y_WHEEL_OUT - 2 * BRG_W


# ===========================================================================
# Helpers
# ===========================================================================

def P(x, y, z=0.0):
    return adsk.core.Point3D.create(x * MM, y * MM, z * MM)


def polar(r, a):
    return (r * math.cos(a), r * math.sin(a))


def part_frame(origin_xyz):
    """Local Z = world +Y (axle direction), local Y = world -Z."""
    m = adsk.core.Matrix3D.create()
    m.setWithCoordinateSystem(P(*origin_xyz),
                              adsk.core.Vector3D.create(1, 0, 0),
                              adsk.core.Vector3D.create(0, 0, -1),
                              adsk.core.Vector3D.create(0, 1, 0))
    return m


def new_component(root, name, origin_xyz):
    occ = root.occurrences.addNewComponent(part_frame(origin_xyz))
    occ.component.name = name
    return occ.component


def sketch_at(comp, z, name):
    if abs(z) < 1e-9:
        plane = comp.xYConstructionPlane
    else:
        inp = comp.constructionPlanes.createInput()
        inp.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(z * MM))
        plane = comp.constructionPlanes.add(inp)
    sk = comp.sketches.add(plane)
    sk.name = name
    return sk


def circle(sk, x, y, d):
    sk.sketchCurves.sketchCircles.addByCenterRadius(P(x, y), d / 2 * MM)


def polygon(sk, pts):
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(P(*pts[0]), P(*pts[1]))
    prev = first
    for p in pts[2:]:
        prev = lines.addByTwoPoints(prev.endSketchPoint, P(*p))
    lines.addByTwoPoints(prev.endSketchPoint, first.startSketchPoint)


def profile(sk, loops=None):
    """Largest profile, optionally only those with this many loops (rings = 2)."""
    profs = [sk.profiles.item(i) for i in range(sk.profiles.count)]
    if loops:
        profs = [p for p in profs if p.profileLoops.count == loops]
    return max(profs, key=lambda p: p.areaProperties().area)


def all_profiles(sk):
    coll = adsk.core.ObjectCollection.create()
    for i in range(sk.profiles.count):
        coll.add(sk.profiles.item(i))
    return coll


def extrude(comp, prof, height, op):
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(prof, op)
    inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal(height * MM))
    return ext.add(inp)


def cut_through(comp, profs):
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(profs, adsk.fusion.FeatureOperations.CutFeatureOperation)
    inp.setSymmetricExtent(adsk.core.ValueInput.createByReal(200 * MM), True)
    bodies = [comp.bRepBodies.item(i) for i in range(comp.bRepBodies.count)]
    inp.participantBodies = bodies
    return ext.add(inp)


NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation


def cylinder(comp, z0, z1, d, op, name, d_in=None):
    sk = sketch_at(comp, z0, name)
    circle(sk, 0, 0, d)
    if d_in:
        circle(sk, 0, 0, d_in)
    return extrude(comp, profile(sk, 2 if d_in else None), z1 - z0, op)


# ===========================================================================
# Involute gear
# ===========================================================================

def inv(a):
    return math.tan(a) - a


def gear_geometry(z, x):
    m = MODULE
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


def gear_teeth(comp, z, x, z0, face, phase, name):
    """Root disc + one tooth + circular pattern. Returns the geometry dict."""
    g = gear_geometry(z, x)
    cylinder(comp, z0, z0 + face, 2 * g["rf"], NEW if z0 == 0 else JOIN, name + " root")
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


def contact_ratio(g1, g2):
    al = math.radians(PRESSURE_ANGLE)
    num = (math.sqrt(g1["ra"] ** 2 - g1["rb"] ** 2) + math.sqrt(g2["ra"] ** 2 - g2["rb"] ** 2)
           - CENTER_DISTANCE * math.sin(al))
    return num / (math.pi * MODULE * math.cos(al))


# Direction axle -> motor in the part frames (local y = -world z).
_DIR = math.atan2(-(MOTOR[1] - AXLE[1]), MOTOR[0] - AXLE[0])
PINION_PHASE = _DIR                                   # pinion tooth points at the gear
GEAR_PHASE = _DIR + math.pi + math.pi / Z_GEAR        # gear has a gap there


# ===========================================================================
# Parts
# ===========================================================================

def build_big_gear(root):
    comp = new_component(root, "Big gear 60T", (MOTOR[0], GAP_PLATE, MOTOR[1]))
    g = gear_teeth(comp, Z_GEAR, X_GEAR, 0.0, GEAR_FACE, GEAR_PHASE, "gear")

    # Lighten: cut both sides down to a centred web, keep rim and hub full width.
    rim_in = 2 * (g["rf"] - GEAR_RIM)
    side = (GEAR_FACE - GEAR_WEB) / 2
    for z0, nm in ((0.0, "web cut inner"), (GEAR_FACE - side, "web cut outer")):
        sk = sketch_at(comp, z0, nm)
        circle(sk, 0, 0, rim_in)
        circle(sk, 0, 0, GEAR_HUB_D)
        extrude(comp, profile(sk, 2), side, CUT)

    # D-bore for the N20 shaft.
    sk = sketch_at(comp, 0.0, "D bore")
    circle(sk, 0, 0, SHAFT_BORE_D)
    r = SHAFT_BORE_D / 2
    flat = SHAFT_FLAT - r  # flat line distance from centre
    h = math.sqrt(r * r - flat * flat) + 0.3
    sk.sketchCurves.sketchLines.addByTwoPoints(P(flat, -h), P(flat, h))
    cut_through(comp, profile(sk))
    return g


def build_pinion_wheel(root):
    comp = new_component(root, "Pinion wheel 12T", (AXLE[0], GAP_PLATE, AXLE[1]))
    g = gear_teeth(comp, Z_PINION, X_PINION, 0.0, PINION_FACE, PINION_PHASE, "pinion")

    w0, w1 = PINION_FACE, PINION_FACE + WHEEL_W
    cylinder(comp, w0, w1, RIM_D, JOIN, "rim")
    lip_d = RIM_D + 2 * LIP_H
    cylinder(comp, w0, w0 + LIP_T, lip_d, JOIN, "lip inner")
    cylinder(comp, w1 - LIP_T, w1, lip_d, JOIN, "lip outer")

    # Lightening ring pocket from the outer face, leaving a web next to the pinion.
    hub_d = BRG_POCKET_D + 2 * 2.4
    sk = sketch_at(comp, w0 + WHEEL_WEB, "wheel pocket")
    circle(sk, 0, 0, RIM_D - 3.0)
    circle(sk, 0, 0, hub_d)
    extrude(comp, profile(sk, 2), WHEEL_W - WHEEL_WEB, CUT)

    sk = sketch_at(comp, 0.0, "bore")
    circle(sk, 0, 0, WHEEL_BORE_D)
    cut_through(comp, profile(sk))

    sk = sketch_at(comp, w1 - 2 * BRG_W, "bearing pocket")
    circle(sk, 0, 0, BRG_POCKET_D)
    extrude(comp, profile(sk), 2 * BRG_W, CUT)
    return g


def to_local(xz):
    """World (x, z) -> bracket-frame (x, y). Bracket origin is the axle."""
    return (xz[0] - AXLE[0], -(xz[1] - AXLE[1]))


def build_bracket(root):
    comp = new_component(root, "Bracket", (AXLE[0], 0.0, AXLE[1]))
    mx, mz = MOTOR
    outline = [(8.0, WHEEL_R - 6), (8.0, Z_TOP), (mx - 10, Z_TOP),
               (mx - 10, mz - 8), (-6.0, WHEEL_R - 6)]
    sk = sketch_at(comp, -PLATE_T, "plate")
    polygon(sk, [to_local(p) for p in outline])
    extrude(comp, profile(sk), PLATE_T, NEW)

    # Top flange, pointing inboard (-Z local), for screwing to the chassis.
    sk = sketch_at(comp, -PLATE_T - FLANGE_DEPTH, "flange")
    x0, x1 = mx - 10, 8.0
    y_top, y_bot = to_local((0, Z_TOP))[1], to_local((0, Z_TOP - PLATE_T))[1]
    polygon(sk, [(x0, y_top), (x1, y_top), (x1, y_bot), (x0, y_bot)])
    extrude(comp, profile(sk), FLANGE_DEPTH + PLATE_T, JOIN)

    # Holes through the plate: axle, motor bushing, 2x M1.6 for the gearbox face.
    sk = sketch_at(comp, 0.0, "plate holes")
    circle(sk, 0, 0, AXLE_HOLE_D)
    lx, ly = to_local(MOTOR)
    circle(sk, lx, ly, N20_BOSS_HOLE)
    circle(sk, lx - N20_HOLE_SPACING / 2, ly, N20_SCREW_HOLE)
    circle(sk, lx + N20_HOLE_SPACING / 2, ly, N20_SCREW_HOLE)
    cut_through(comp, all_profiles(sk))

    # 2x M3 through the flange (vertical in the world).
    sk = comp.sketches.add(comp.xZConstructionPlane)
    sk.name = "flange holes"
    zc = -PLATE_T - FLANGE_DEPTH / 2
    for x in (3.0, mx - 5):
        p = sk.modelToSketchSpace(P(x, 0, zc))
        sk.sketchCurves.sketchCircles.addByCenterRadius(p, FLANGE_HOLE_D / 2 * MM)
    cut_through(comp, all_profiles(sk))


def build_spacers(root):
    comp = new_component(root, "Spacer B (plate to bearing)", (AXLE[0], 0.0, AXLE[1]))
    cylinder(comp, 0.0, Y_POCKET, SPACER_OD, NEW, "tube", d_in=SPACER_ID)
    comp = new_component(root, "Spacer A (screw head)", (AXLE[0], 0.0, AXLE[1]))
    cylinder(comp, Y_WHEEL_OUT, Y_WHEEL_OUT + SPACER_A_LEN, SPACER_OD, NEW, "ring", d_in=SPACER_ID)


def build_references(root):
    comp = new_component(root, "N20 motor (ref, do not print)", (MOTOR[0], 0.0, MOTOR[1]))
    sk = sketch_at(comp, -PLATE_T - N20_LEN, "body")
    polygon(sk, [(-N20_W / 2, -N20_H / 2), (N20_W / 2, -N20_H / 2),
                 (N20_W / 2, N20_H / 2), (-N20_W / 2, N20_H / 2)])
    extrude(comp, profile(sk), N20_LEN, NEW)
    cylinder(comp, -PLATE_T, -PLATE_T + 10.0, 3.0, NEW, "shaft")

    comp = new_component(root, "Tyre (ref, do not print)", (AXLE[0], 0.0, AXLE[1]))
    z0 = GAP_PLATE + PINION_FACE + LIP_T
    cylinder(comp, z0, z0 + TYRE_W, 2 * WHEEL_R, NEW, "tyre", d_in=RIM_D)


# ===========================================================================
# Entry points
# ===========================================================================

def close_previous():
    """Close an unsaved design left by an earlier (failed) run of this script."""
    app = adsk.core.Application.get()
    doc = app.activeDocument
    design = adsk.fusion.Design.cast(app.activeProduct)
    if doc and not doc.isSaved and design and design.allComponents.itemByName("Big gear 60T"):
        doc.close(False)


def new_design():
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    design.fusionUnitsManager.distanceDisplayUnits = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
    return design


def root_component():
    app = adsk.core.Application.get()
    return adsk.fusion.Design.cast(app.activeProduct).rootComponent


def summary():
    g1 = gear_geometry(Z_PINION, X_PINION)
    g2 = gear_geometry(Z_GEAR, X_GEAR)
    return {
        "ratio": Z_GEAR / Z_PINION,
        "center_distance": CENTER_DISTANCE,
        "pinion_tip_d": round(2 * g1["ra"], 2),
        "gear_tip_d": round(2 * g2["ra"], 2),
        "contact_ratio": round(contact_ratio(g1, g2), 2),
        "motor_xz": [round(v, 2) for v in MOTOR],
        "gear_floor_clearance": round(MOTOR[1] - g2["ra"], 2),
        "flange_top_z": round(Z_TOP, 2),
        "spacer_b_len": round(Y_POCKET, 2),
    }


def build_all():
    new_design()
    root = root_component()
    build_big_gear(root)
    build_pinion_wheel(root)
    build_bracket(root)
    build_spacers(root)
    build_references(root)
    return summary()


def run(context):
    ui = adsk.core.Application.get().userInterface
    try:
        ui.messageBox(str(build_all()))
    except Exception:
        import traceback
        ui.messageBox(traceback.format_exc())
