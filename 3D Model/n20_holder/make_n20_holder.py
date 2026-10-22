"""Fusion 360 generator: standalone N20 motor holder (cradle + screw-on cap).

Run it in Fusion (Utilities > Scripts and Add-Ins) or through the
Fusion360MCP add-in (execute_code). It opens a new design and builds:

  Cradle   screws to the underside of the chassis, holds the top half of the motor
  Cap      clamps the bottom half; the same 2x M3 screws go through cap,
           cradle and chassis (nuts on top of the chassis)
  N20 (ref) reference body only, do not print

Assembled frame: chassis underside at Z = 0, holder hangs below, motor axis
along X with the gearbox face against the front wall at X = FRONT_T and the
shaft pointing to -X. Each part is modelled in its print orientation (flat
face on the bed at local Z = 0) and placed with an occurrence transform.
Dimensions in mm; the Fusion API uses cm (MM).
"""

import adsk.core
import adsk.fusion

MM = 0.1

# --- N20 (measure yours) ---------------------------------------------------
MOTOR_W = 12.0        # across the round sides
MOTOR_H = 10.0        # across the flats
MOTOR_LEN = 32.0      # gearbox + motor + encoder, reference only
FIT = 0.2             # pocket width clearance

# --- Holder ----------------------------------------------------------------
POCKET_W = MOTOR_W + FIT
POCKET_DEPTH = 4.9    # per half: 2 x 4.9 + GAP = MOTOR_H -> screws squeeze 0.2 mm
GAP = 0.2             # between cradle and cap faces when the motor is in
CRADLE_T = 3.0        # material between chassis and motor
CAP_T = 2.5           # material under the motor
WALL = 6.0            # side walls carry the screws
CLAMP_LEN = 16.0      # along the gearbox + front of the motor can
FRONT_T = 2.0         # front wall the gearbox face rests against
NOTCH_W = 5.0         # opening for the 4 mm output bushing and shaft
SCREW_D = 3.3         # M3 clearance

LENGTH = FRONT_T + CLAMP_LEN
WIDTH = POCKET_W + 2 * WALL
SCREW_Y = POCKET_W / 2 + WALL / 2
SCREW_X = FRONT_T + CLAMP_LEN / 2

CRADLE_H = CRADLE_T + POCKET_DEPTH
CAP_H = CAP_T + POCKET_DEPTH
MOTOR_AXIS_Z = -(CRADLE_T + MOTOR_H / 2)  # below the chassis underside


# ===========================================================================
# Helpers
# ===========================================================================

def P(x, y, z=0.0):
    return adsk.core.Point3D.create(x * MM, y * MM, z * MM)


def frame(origin, flip):
    """flip=True: rotate 180 deg about X (print-bed face becomes the top)."""
    m = adsk.core.Matrix3D.create()
    m.setWithCoordinateSystem(P(*origin),
                              adsk.core.Vector3D.create(1, 0, 0),
                              adsk.core.Vector3D.create(0, -1 if flip else 1, 0),
                              adsk.core.Vector3D.create(0, 0, -1 if flip else 1))
    return m


def new_component(root, name, origin, flip=False):
    occ = root.occurrences.addNewComponent(frame(origin, flip))
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


def rect(sk, x0, y0, x1, y1):
    sk.sketchCurves.sketchLines.addTwoPointRectangle(P(x0, y0), P(x1, y1))


NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation


def extrude_all(comp, sk, height, op):
    """Extrude every profile in the sketch by `height` (+Z)."""
    coll = adsk.core.ObjectCollection.create()
    for i in range(sk.profiles.count):
        coll.add(sk.profiles.item(i))
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(coll, op)
    inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal(height * MM))
    return ext.add(inp)


def box(comp, x0, y0, z0, x1, y1, z1, op, name):
    sk = sketch_at(comp, z0, name)
    rect(sk, x0, y0, x1, y1)
    return extrude_all(comp, sk, z1 - z0, op)


def holes(comp, z0, z1, pts, d, name):
    sk = sketch_at(comp, z0, name)
    for x, y in pts:
        sk.sketchCurves.sketchCircles.addByCenterRadius(P(x, y), d / 2 * MM)
    return extrude_all(comp, sk, z1 - z0, CUT)


# ===========================================================================
# Parts (local Z = 0 is the face that goes on the print bed)
# ===========================================================================

def clamp_half(comp, floor_t, height, label):
    """Block with motor pocket, shaft notch in the front wall and 2 screw holes."""
    box(comp, 0, -WIDTH / 2, 0, LENGTH, WIDTH / 2, height, NEW, label + " block")
    box(comp, FRONT_T, -POCKET_W / 2, floor_t, LENGTH, POCKET_W / 2, height, CUT, label + " pocket")
    box(comp, 0, -NOTCH_W / 2, height - NOTCH_W / 2, FRONT_T, NOTCH_W / 2, height, CUT,
        label + " shaft notch")
    holes(comp, 0, height, [(SCREW_X, SCREW_Y), (SCREW_X, -SCREW_Y)], SCREW_D, label + " screw holes")


def build_cradle(root):
    # Printed chassis-face down; assembled upside down under the chassis.
    comp = new_component(root, "N20 holder cradle", (0, 0, 0), flip=True)
    clamp_half(comp, CRADLE_T, CRADLE_H, "cradle")


def build_cap(root):
    z_bottom = -(CRADLE_H + GAP + CAP_H)
    comp = new_component(root, "N20 holder cap", (0, 0, z_bottom))
    clamp_half(comp, CAP_T, CAP_H, "cap")


def build_reference(root):
    comp = new_component(root, "N20 motor (ref, do not print)", (0, 0, 0))
    z0 = MOTOR_AXIS_Z - MOTOR_H / 2
    box(comp, FRONT_T, -MOTOR_W / 2, z0, FRONT_T + MOTOR_LEN, MOTOR_W / 2, z0 + MOTOR_H, NEW, "body")
    box(comp, FRONT_T - 10, -1.5, MOTOR_AXIS_Z - 1.5, FRONT_T, 1.5, MOTOR_AXIS_Z + 1.5, NEW, "shaft")


# ===========================================================================
# Entry points
# ===========================================================================

def new_design():
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    design.fusionUnitsManager.distanceDisplayUnits = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
    return design.rootComponent


def summary():
    return {
        "footprint_x_y": [LENGTH, WIDTH],
        "screw_spacing": round(2 * SCREW_Y, 2),
        "screw_x_from_front": SCREW_X,
        "motor_axis_below_chassis": -MOTOR_AXIS_Z,
        "total_height": round(CRADLE_H + GAP + CAP_H, 2),
        "screw_length_min": round(CRADLE_H + GAP + CAP_H + 1.5 + 2.4, 1),
    }


def build_all():
    root = new_design()
    build_cradle(root)
    build_cap(root)
    build_reference(root)
    return summary()


def run(context):
    ui = adsk.core.Application.get().userInterface
    try:
        ui.messageBox(str(build_all()))
    except Exception:
        import traceback
        ui.messageBox(traceback.format_exc())
