[winch_bodies_fusion_v8_8_22.py](https://github.com/user-attachments/files/34947236/winch_bodies_fusion_v8_8_22.py)
"""winch v8.8 (22/56) rotating bodies -> Autodesk Fusion (Python script)

v8 changes (2026-11-12) against the v5.2 set that locked up in the first run:
  * axial gap between layers 0.5 -> 1.5 mm, gear W2 faces carry a thrust boss D10 (contacts a stationary
    adapter shoulder / spacer / wall at small radius only), first gear CG1 and the motor pinion 1.5 mm off W2
  * tooth thinning 0.10 -> 0.15 mm per gear (backlash ~0.3 mm)
  * widths: S1 helical 6, S2..S7 spur 5, S8 spur 6, S9 herringbone 10
  * stepped bores ON (bearing pockets D15.1 for the 10 x 6700ZZ); running bores D5.5 -> drill D5.1-5.2 through
  * cord hole + knot pocket on OUT and hex socket on CG8a ON
  * motor axis moved to (Y 68.8, Z 15.28) (same centre distance 40.273 to A)
  * v8.7: drum core D13, flat groove X 73.8-82.0 (two turns), 45 deg cone only at X 82-85 next to bearing #2 (85-89)
  * v8.5: S9 herringbone m1.25 16/38 (beta 30.07) -> m1.5 14/35 (beta 19.56), shift +0.3 / -0.3, a = 39
  * v8.4: S1 hub D6 to X 10.5 (grip 9 mm on the motor shaft)
  * v8.2: stage 1 = helical 17T / 56T (was 19/54), profile shift x = +0.4 / -0.4; frame = v8 (unchanged)
Run: put this file in a folder named winch_bodies_fusion_v8 (same name), open an empty design -> Utilities >
Add-Ins > Scripts and Add-Ins > run.  Units mm / deg.  X = shaft axis (0 = inner face of W2).
"""
import math, traceback
try:
    import adsk.core, adsk.fusion
except ImportError:
    adsk = None

# ============================================================================ SETTINGS
MAKE_BORES = True        # centre holes on
SIMPLE_BORE_D = None     # v8: stepped bores (bearing pockets) ON. A number here = one straight hole instead
KEEP_MOTOR_BORE = True   # S1_pinion keeps its D1.9 hole for the D2 motor shaft
MAKE_OTHER_CUTS = True   # v8: hex socket (CG8a) and cord hole / knot pocket (OUT) ON
TWIST_SIGN = +1          # flips the absolute helix hand only (meshing pairs stay opposite)
N_FLANK, N_TIP, N_ROOT = 12, 3, 4
HA, HF, ALPHA_N, THIN = 1.0, 1.25, 20.0, 0.15      # addendum, dedendum, pressure angle, tooth thinning (mm)

# ============================================================================ GEAR TYPES
#            kind           z   mn    beta    width hand
GEAR = {
  "H19":  dict(kind="helical",     z=17, x=0.4, mn=1.0,  beta=25.0,   width=6.0,  hand=+1),   # S1 motor pinion
  "H54":  dict(kind="helical",     z=56, x=-0.4, mn=1.0,  beta=25.0,   width=6.0,  hand=-1),   # S1 gear (CG1)
  "S23":  dict(kind="spur",        z=23, mn=1.0,  beta=0.0,    width=5.0,  hand=+1),   # S2..S7 pinion
  "S55":  dict(kind="spur",        z=55, mn=1.0,  beta=0.0,    width=5.0,  hand=+1),   # S2..S7 gear
  "S23w": dict(kind="spur",        z=23, mn=1.0,  beta=0.0,    width=6.0,  hand=+1),   # S8 pinion (CG7)
  "S55w": dict(kind="spur",        z=55, mn=1.0,  beta=0.0,    width=6.0,  hand=+1),   # S8 gear (CG8a)
  "HB16": dict(kind="herringbone", z=14, x=0.3, mn=1.5, beta=19.557, width=10.0, hand=+1),   # S9 pinion (CG8b)
  "HB38": dict(kind="herringbone", z=35, x=-0.3, mn=1.5, beta=19.557, width=10.0, hand=-1),   # S9 gear (OUT)
}

# shaft centres in the assembly section (Y, Z)
AXIS = {"M": (68.8, 15.28), "A": (32.3, 32.3), "B": (59.9, 59.9)}

# ============================================================================ BODIES (local z = assembly X)
# features (applied in order):
#   ("gear", type, z0)                 toothed section starting at z0
#   ("disc", OD, z0, z1)               solid cylinder (hub / collar / flange)
#   ("lathe", [(r, z), ...])           solid of revolution from an outer (r, z) outline, closed on the axis
#   ("hex", AF, z0, z1)                hexagonal prism (spigot), joined
#   ("hexcut", AF, z0, z1)             hexagonal socket, cut
#   ("hole", r_pos, D, z0, z1)         off-axis hole at radius r_pos (cut)
# bore: [(z0, z1, D), ...]             stepped centre hole (revolve cut)
# rot: mesh phase (deg) about the part's own axis, from the CAD model
def cg(k, gtype, g0, p0, rot):
    gw = GEAR[gtype]["width"]; g1 = g0 + gw
    return dict(name=f"CG{k}", axis="A" if k % 2 else "B", rot=rot,
                feats=[("gear", gtype, g0), ("disc", 16.0, g1 - 0.2, p0 + 0.2), ("gear", "S23", p0)],
                bore=[(g0, g1, 5.40), (g1, p0 + 0.2, 5.80), (p0 + 0.2, p0 + 1.0, 13.60), (p0 + 1.0, p0 + 6.0, 15.10)])

BODIES = [
  dict(name="S1_pinion", axis="M", rot=0.0, feats=[("gear", "H19", 1.5), ("disc", 6.0, 7.3, 10.5)], bore=[(1.5, 10.5, 1.90)]),
  dict(name="CG1", axis="A", rot=205.2727, feats=[("gear", "H54", 1.5), ("disc", 19.0, 7.3, 9.2), ("gear", "S23", 9.0)],
       bore=[(1.5, 9.0, 5.5), (9.0, 10.0, 13.6), (10.0, 14.0, 15.1)]),
  dict(name="CG2", axis="B", rot=345.2545, feats=[("gear", "S55", 9.0), ("disc", 19.0, 13.8, 15.7), ("gear", "S23", 15.5)],
       bore=[(9.0, 15.5, 5.5), (15.5, 16.5, 13.6), (16.5, 20.5, 15.1)]),
  dict(name="CG3", axis="A", rot=180.9759, feats=[("gear", "S55", 15.5), ("disc", 19.0, 20.3, 22.2), ("gear", "S23", 22.0)],
       bore=[(15.5, 22.0, 5.5), (22.0, 23.0, 13.6), (23.0, 27.0, 15.1)]),
  dict(name="CG4", axis="B", rot=136.2282, feats=[("gear", "S55", 22.0), ("disc", 19.0, 26.8, 28.7), ("gear", "S23", 28.5)],
       bore=[(22.0, 28.5, 5.5), (28.5, 29.5, 13.6), (29.5, 33.5, 15.1)]),
  dict(name="CG5", axis="A", rot=121.6648, feats=[("gear", "S55", 28.5), ("disc", 19.0, 33.3, 35.2), ("gear", "S23", 35.0)],
       bore=[(28.5, 35.0, 5.5), (35.0, 36.0, 13.6), (36.0, 40.0, 15.1)]),
  dict(name="CG6", axis="B", rot=159.529, feats=[("gear", "S55", 35.0), ("disc", 19.0, 39.8, 41.7), ("gear", "S23", 41.5)],
       bore=[(35.0, 41.5, 5.5), (41.5, 42.5, 13.6), (42.5, 46.5, 15.1)]),
  dict(name="CG7", axis="A", rot=112.5109, feats=[("gear", "S55", 41.5), ("disc", 19.0, 46.3, 51.1), ("gear", "S23w", 50.9)],
       bore=[(41.5, 50.9, 5.5), (50.9, 51.9, 13.6), (51.9, 56.9, 15.1)]),
  dict(name="CG8a_gear", axis="B", rot=163.1252, feats=[("gear", "S55w", 50.9), ("hexcut", 20.3, 53.7, 56.9)],
       bore=[(50.9, 56.9, 5.5)]),
  dict(name="CG8b_pinion", axis="B", rot=163.1252, feats=[("hex", 20.0, 53.9, 61.0), ("disc", 17.0, 60.9, 61.5), ("gear", "HB16", 61.3)],
       bore=[(53.9, 61.3, 5.5), (61.3, 65.3, 5.9), (65.3, 66.3, 13.6), (66.3, 71.3, 15.1)]),
  dict(name="OUT_G9_spool", axis="A", rot=116.0499,
       feats=[("gear", "HB38", 61.3), ("disc", 34.0, 71.2, 72.8),
              ("lathe", [(0, 72.7), (7.5, 72.7), (7.5, 72.8), (6.5, 73.8), (6.5, 82.0), (9.5, 85.0), (9.5, 89.0), (0, 89.0)]),
              ("hole", 12.5, 6.0, 61.3, 66.3),      # knot pocket
              ("hole", 12.5, 2.8, 61.3, 72.8)],   # cord hole
       bore=[(61.3, 66.3, 15.1), (66.3, 67.3, 13.6), (67.3, 84.5, 5.9), (84.5, 85.0, 13.6), (85.0, 89.0, 15.1)]),
]

# ============================================================================ involute math (same as CAD model)
def inv(a):
    return math.tan(a) - a

class Spec:
    def __init__(s, g):
        s.z, s.mn = g["z"], g["mn"]
        s.beta, s.an = math.radians(g["beta"]), math.radians(ALPHA_N)
        s.mt = s.mn / math.cos(s.beta)
        s.at = math.atan(math.tan(s.an) / math.cos(s.beta))
        s.rp = s.mt * s.z / 2; s.rb = s.rp * math.cos(s.at)
        x = g.get("x", 0.0)
        s.ra = s.rp + (HA + x) * s.mn; s.rf = s.rp - (HF - x) * s.mn
        s.st = s.mt * (math.pi / 2 + 2 * x * math.tan(s.at)) - THIN / math.cos(s.beta)
        s.k = math.tan(s.beta) / s.rp * g["hand"]          # twist rad/mm
    def psi(s, r):
        r = max(r, s.rb); a = math.acos(min(1.0, s.rb / r))
        return s.st / (2 * s.rp) + inv(s.at) - inv(a)

def profile(s, rot):
    r0 = max(s.rf, s.rb)
    rs = [r0 + (s.ra - r0) * i / (N_FLANK - 1) for i in range(N_FLANK)]
    if s.rf < s.rb - 1e-9:
        rs = [s.rf] + rs
    p = 2 * math.pi / s.z; pr, pa = s.psi(s.rf), s.psi(s.ra); pts = []
    for i in range(s.z):
        c = i * p + rot
        for r in rs:
            a = c - s.psi(r); pts.append((r * math.cos(a), r * math.sin(a)))
        for k in range(1, N_TIP + 1):
            t = -pa + 2 * pa * k / (N_TIP + 1); pts.append((s.ra * math.cos(c + t), s.ra * math.sin(c + t)))
        for r in reversed(rs):
            a = c + s.psi(r); pts.append((r * math.cos(a), r * math.sin(a)))
        a0, a1 = c + pr, c + p - pr
        for k in range(1, N_ROOT + 1):
            t = a0 + (a1 - a0) * k / (N_ROOT + 1); pts.append((s.rf * math.cos(t), s.rf * math.sin(t)))
    return pts

def hexagon(af, rot):
    R = af / math.sqrt(3)
    return [(R * math.cos(rot + math.pi / 6 + i * math.pi / 3), R * math.sin(rot + math.pi / 6 + i * math.pi / 3)) for i in range(6)]

# ============================================================================ Fusion helpers
def cm(v):
    return v / 10.0

class Builder:
    def __init__(self, comp):
        self.c = comp; self.f = comp.features; self.body = None
        self.NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
        self.JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
        self.CUT = adsk.fusion.FeatureOperations.CutFeatureOperation

    def plane(self, z):
        if abs(z) < 1e-9:
            return self.c.xYConstructionPlane
        pi = self.c.constructionPlanes.createInput()
        pi.setByOffset(self.c.xYConstructionPlane, adsk.core.ValueInput.createByReal(cm(z)))
        pl = self.c.constructionPlanes.add(pi); pl.isLightBulbOn = False
        return pl

    def poly_sketch(self, z, loops, circles=()):
        sk = self.c.sketches.add(self.plane(z)); sk.isComputeDeferred = True
        L = sk.sketchCurves.sketchLines
        P = lambda q: sk.modelToSketchSpace(adsk.core.Point3D.create(cm(q[0]), cm(q[1]), cm(z)))
        for pts in loops:
            first = L.addByTwoPoints(P(pts[0]), P(pts[1])); prev = first.endSketchPoint
            for q in pts[2:]:
                prev = L.addByTwoPoints(prev, P(q)).endSketchPoint
            L.addByTwoPoints(prev, first.startSketchPoint)
        for (x, y, d) in circles:
            sk.sketchCurves.sketchCircles.addByCenterRadius(P((x, y)), cm(d / 2))
        sk.isComputeDeferred = False
        sk.isVisible = False
        return sk

    def largest(self, sk):
        return max(sk.profiles, key=lambda p: p.areaProperties().area)

    def smallest(self, sk):
        return min(sk.profiles, key=lambda p: p.areaProperties().area)

    def op(self):
        return self.NEW if self.body is None else self.JOIN

    def keep(self, feat, op):
        if op == self.NEW:
            self.body = feat.bodies.item(0)

    def extrude(self, prof, z0, z1, op):
        ei = self.f.extrudeFeatures.createInput(prof, op)
        ei.setDistanceExtent(False, adsk.core.ValueInput.createByReal(cm(z1 - z0)))
        if op != self.NEW and self.body is not None:
            ei.participantBodies = [self.body]
        return self.f.extrudeFeatures.add(ei)

    def sweep(self, prof, z0, z1, twist_deg, op):
        psk = self.c.sketches.add(self.c.xZConstructionPlane)
        a = psk.modelToSketchSpace(adsk.core.Point3D.create(0, 0, cm(z0)))
        b = psk.modelToSketchSpace(adsk.core.Point3D.create(0, 0, cm(z1)))
        ln = psk.sketchCurves.sketchLines.addByTwoPoints(a, b); psk.isVisible = False
        si = self.f.sweepFeatures.createInput(prof, self.f.createPath(ln), op)
        si.twistAngle = adsk.core.ValueInput.createByString(f"{twist_deg:.6f} deg")
        if op != self.NEW and self.body is not None:
            si.participantBodies = [self.body]
        return self.f.sweepFeatures.add(si)

    def combine(self, tool):
        if self.body is None:
            self.body = tool; return
        tools = adsk.core.ObjectCollection.create(); tools.add(tool)
        ci = self.f.combineFeatures.createInput(self.body, tools)
        ci.operation = self.JOIN
        self.f.combineFeatures.add(ci)

    def revolve(self, rz, op):
        """closed (r, z) outline in the local XZ plane, revolved 360 deg about local Z."""
        sk = self.c.sketches.add(self.c.xZConstructionPlane); sk.isComputeDeferred = True
        L = sk.sketchCurves.sketchLines
        P = lambda q: sk.modelToSketchSpace(adsk.core.Point3D.create(cm(q[0]), 0, cm(q[1])))
        first = L.addByTwoPoints(P(rz[0]), P(rz[1])); prev = first.endSketchPoint
        for q in rz[2:]:
            prev = L.addByTwoPoints(prev, P(q)).endSketchPoint
        L.addByTwoPoints(prev, first.startSketchPoint)
        sk.isComputeDeferred = False; sk.isVisible = False
        ri = self.f.revolveFeatures.createInput(sk.profiles.item(0), self.c.zConstructionAxis, op)
        ri.setAngleExtent(False, adsk.core.ValueInput.createByString("360 deg"))
        if op != self.NEW and self.body is not None:
            ri.participantBodies = [self.body]
        return self.f.revolveFeatures.add(ri)

    # ---------------------------------------------------------------- feature types
    def gear(self, gtype, z0, rot):
        g = GEAR[gtype]; s = Spec(g); w = g["width"]
        sk = self.poly_sketch(z0, [profile(s, rot)])
        prof = self.largest(sk)
        if g["kind"] == "spur":
            op = self.op(); self.keep(self.extrude(prof, z0, z0 + w, op), op)
        elif g["kind"] == "helical":
            op = self.op(); tw = math.degrees(s.k * w) * TWIST_SIGN
            self.keep(self.sweep(prof, z0, z0 + w, tw, op), op)
        else:
            half = w / 2; tw = math.degrees(s.k * half) * TWIST_SIGN
            fe = self.sweep(prof, z0, z0 + half, tw, self.NEW)
            hb = fe.bodies.item(0)
            ents = adsk.core.ObjectCollection.create(); ents.add(hb)
            mf = self.f.mirrorFeatures.add(self.f.mirrorFeatures.createInput(ents, self.plane(z0 + half)))
            tools = adsk.core.ObjectCollection.create(); tools.add(mf.bodies.item(0))
            ci = self.f.combineFeatures.createInput(hb, tools); ci.operation = self.JOIN
            self.f.combineFeatures.add(ci)
            self.combine(hb)

    def disc(self, od, z0, z1):
        op = self.op(); self.keep(self.revolve([(0, z0), (od / 2, z0), (od / 2, z1), (0, z1)], op), op)

    def lathe(self, rz):
        op = self.op(); self.keep(self.revolve(rz, op), op)

    def hexa(self, af, z0, z1, rot, cut):
        sk = self.poly_sketch(z0, [hexagon(af, rot)])
        prof = self.largest(sk)
        if cut:
            self.extrude(prof, z0, z1, self.CUT)
        else:
            op = self.op(); self.keep(self.extrude(prof, z0, z1, op), op)

    def hole(self, r_pos, d, z0, z1, rot):
        sk = self.poly_sketch(z0, [], circles=[(r_pos * math.cos(rot), r_pos * math.sin(rot), d)])
        self.extrude(self.smallest(sk), z0, z1, self.CUT)

    def bore(self, steps):
        steps = sorted(steps)
        rz = [(0, steps[0][0])]
        for z0, z1, d in steps:
            rz += [(d / 2, z0), (d / 2, z1)]
        rz.append((0, steps[-1][1]))
        clean = [rz[0]]
        for q in rz[1:]:
            if abs(q[0] - clean[-1][0]) > 1e-9 or abs(q[1] - clean[-1][1]) > 1e-9:
                clean.append(q)
        # extend slightly past both faces so the cut goes fully through
        clean[0] = (0, clean[0][1] - 0.05); clean[1] = (clean[1][0], clean[1][1] - 0.05)
        clean[-1] = (0, clean[-1][1] + 0.05); clean[-2] = (clean[-2][0], clean[-2][1] + 0.05)
        self.revolve(clean, self.CUT)

# ============================================================================ main
def place(root, name, axis):
    yc, zc = AXIS[axis]
    m = adsk.core.Matrix3D.create()
    m.setWithCoordinateSystem(adsk.core.Point3D.create(0, cm(yc), cm(zc)),
                              adsk.core.Vector3D.create(0, 1, 0),     # local x -> assembly Y
                              adsk.core.Vector3D.create(0, 0, 1),     # local y -> assembly Z
                              adsk.core.Vector3D.create(1, 0, 0))     # local z (part axis) -> assembly X
    occ = root.occurrences.addNewComponent(m)
    occ.component.name = name
    return occ.component

def build(root, b):
    comp = place(root, b["name"], b["axis"])
    B = Builder(comp); rot = math.radians(b["rot"])
    for f in b["feats"]:
        t = f[0]
        if t == "gear":
            B.gear(f[1], f[2], rot)
        elif t == "disc":
            B.disc(f[1], f[2], f[3])
        elif t == "lathe":
            B.lathe(f[1])
        elif t == "hex":
            B.hexa(f[1], f[2], f[3], rot, cut=False)
        elif t == "hexcut":
            if MAKE_OTHER_CUTS:
                B.hexa(f[1], f[2], f[3], rot, cut=True)
        elif t == "hole":
            if MAKE_OTHER_CUTS:
                B.hole(f[1], f[2], f[3], f[4], rot)
    if MAKE_BORES and b.get("bore"):
        steps = b["bore"]
        if SIMPLE_BORE_D and not (KEEP_MOTOR_BORE and b["name"] == "S1_pinion"):
            steps = [(min(t[0] for t in steps), max(t[1] for t in steps), SIMPLE_BORE_D)]
        B.bore(steps)
    if B.body is not None:
        B.body.name = b["name"]

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get(); ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("Open a Fusion design first."); return
        done, failed = [], []
        for b in BODIES:
            try:
                build(design.rootComponent, b); done.append(b["name"])
            except Exception:
                failed.append(f'{b["name"]}:\n{traceback.format_exc().splitlines()[-1]}')
            adsk.doEvents()
        msg = "Created: " + ", ".join(done)
        if failed:
            msg += "\n\nFailed:\n" + "\n\n".join(failed)
        msg += "\n\nCheck: Inspect > Interference between meshing parts (should be none)."
        ui.messageBox(msg)
    except Exception:
        if ui:
            ui.messageBox("Failed:\n" + traceback.format_exc())
