"""AeroBlend Example Aircraft Generator.

Blender add-on AND standalone script that generates a complete example aircraft
with NACA airfoil cross-sections and exports it as .glb for aeroblend-native.

Install as add-on:
    Edit > Preferences > Add-ons > dropdown (▼) > Install from Disk > select this file
    Enable "AeroBlend: Example Aircraft Generator" in preferences
    3D Viewport sidebar (N) > AeroBlend tab

Standalone (headless):
    blender --background --python scripts/generate_example_aircraft.py

Parts generated (names match aeroblend-importer parts.rs regex classifier):
    Wing_Left, Wing_Right         -> wing_left, wing_right  (NACA 2412)
    H_Tail_Left, H_Tail_Right    -> h_tail                  (NACA 0009)
    Vertical_Stabilizer           -> v_tail                  (NACA 0009)
    Fuselage                      -> fuselage
    Engine_Left, Engine_Right     -> engine

Coordinate system (Blender Z-up):
    X = right (wing span)
    Y = forward (chord / fuselage length)
    Z = up (airfoil thickness)
glTF export auto-converts to Y-up.
"""

bl_info = {
    "name": "AeroBlend: Example Aircraft Generator",
    "author": "AeroBlend",
    "version": (1, 2, 0),
    "blender": (3, 6, 0),
    "location": "View3D > Sidebar > AeroBlend",
    "description": "Generate, export, build, and run aircraft in AeroBlend simulator",
    "category": "Object",
}

import bpy
import bmesh
import fcntl
import json
import math
import os
import select
import subprocess
import tempfile
from bpy.props import (
    EnumProperty,
    FloatProperty,
    IntProperty,
    StringProperty,
    BoolProperty,
)

# ---------------------------------------------------------------------------
# NACA 4-digit airfoil helpers
# ---------------------------------------------------------------------------
def _cosine_spacing(n):
    """Return *n* values in [0, 1] with cosine spacing (denser near LE)."""
    return [(1.0 - math.cos(math.pi * i / (n - 1))) / 2.0 for i in range(n)]


def _naca_thickness(xc, t):
    """NACA 4-digit half-thickness at chord fraction *xc* (closed TE)."""
    return (t / 0.2) * (
        0.2969 * math.sqrt(max(xc, 0.0))
        - 0.1260 * xc
        - 0.3516 * xc ** 2
        + 0.2843 * xc ** 3
        - 0.1036 * xc ** 4
    )


def _naca_camber(xc, m, p):
    """Mean camber line *yc* and its slope *dyc/dx* for NACA mpxx."""
    if p < 1e-6:
        return 0.0, 0.0
    if xc < p:
        yc = (m / p ** 2) * (2.0 * p * xc - xc ** 2)
        dyc = (2.0 * m / p ** 2) * (p - xc)
    else:
        yc = (m / (1.0 - p) ** 2) * ((1.0 - 2.0 * p) + 2.0 * p * xc - xc ** 2)
        dyc = (2.0 * m / (1.0 - p) ** 2) * (p - xc)
    return yc, dyc


def _naca_profile(naca_str, n_pts):
    """Generate upper/lower (x, z) points for a NACA 4-digit airfoil (chord=1)."""
    m = int(naca_str[0]) / 100.0
    p = int(naca_str[1]) / 10.0
    t = int(naca_str[2:]) / 100.0

    xs = _cosine_spacing(n_pts)
    upper, lower = [], []
    for xc in xs:
        yt = _naca_thickness(xc, t)
        yc, dyc = _naca_camber(xc, m, p)
        theta = math.atan(dyc) if abs(dyc) > 1e-10 else 0.0
        upper.append((xc - yt * math.sin(theta), yc + yt * math.cos(theta)))
        lower.append((xc + yt * math.sin(theta), yc - yt * math.cos(theta)))
    return upper, lower


def _profile_loop(upper, lower):
    """Closed cross-section: upper LE->TE then lower TE->LE (shared endpoints)."""
    return list(upper) + list(reversed(lower[1:-1]))


# ---------------------------------------------------------------------------
# Scene helpers
# ---------------------------------------------------------------------------
def _clear_scene():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    for block in bpy.data.meshes:
        if block.users == 0:
            bpy.data.meshes.remove(block)


def _link(name, mesh_data):
    obj = bpy.data.objects.new(name, mesh_data)
    bpy.context.collection.objects.link(obj)
    return obj


# ---------------------------------------------------------------------------
# Mesh builders
# ---------------------------------------------------------------------------
def _create_wing(name, naca, chord, span, n_af, n_sp,
                 taper=1.0, sweep=0.0, dihedral=0.0,
                 mirror_x=False, pos=(0, 0, 0)):
    upper, lower = _naca_profile(naca, n_af)
    prof = _profile_loop(upper, lower)
    np = len(prof)
    sw = math.tan(math.radians(sweep))
    dh = math.tan(math.radians(dihedral))

    mesh = bpy.data.meshes.new(name)
    bm = bmesh.new()
    secs = []
    for j in range(n_sp + 1):
        t = j / n_sp
        lc = chord * (1.0 - t * (1.0 - taper))
        sp = t * span
        xo = sp * (-1.0 if mirror_x else 1.0)
        yo = sp * sw
        zo = sp * dh
        ring = []
        for ax, az in prof:
            ring.append(bm.verts.new((
                xo + pos[0],
                -ax * lc + yo + pos[1],
                az * lc + zo + pos[2],
            )))
        secs.append(ring)

    bm.verts.ensure_lookup_table()

    for j in range(n_sp):
        s0, s1 = secs[j], secs[j + 1]
        for i in range(np):
            ni = (i + 1) % np
            try:
                if mirror_x:
                    bm.faces.new([s0[i], s0[ni], s1[ni], s1[i]])
                else:
                    bm.faces.new([s0[i], s1[i], s1[ni], s0[ni]])
            except ValueError:
                pass

    try:
        bm.faces.new(list(reversed(secs[0])) if mirror_x else secs[0])
    except ValueError:
        pass
    try:
        bm.faces.new(secs[-1] if mirror_x else list(reversed(secs[-1])))
    except ValueError:
        pass

    bm.to_mesh(mesh)
    bm.free()
    return _link(name, mesh)


def _create_vstab(name, naca, chord, height, n_af, n_sp, pos=(0, 0, 0)):
    upper, lower = _naca_profile(naca, n_af)
    prof = _profile_loop(upper, lower)
    np = len(prof)

    mesh = bpy.data.meshes.new(name)
    bm = bmesh.new()
    secs = []
    for j in range(n_sp + 1):
        t = j / n_sp
        lc = chord * (1.0 - t * 0.3)
        zp = t * height
        ring = []
        for ax, az in prof:
            ring.append(bm.verts.new((
                az * lc + pos[0],
                -ax * lc + pos[1],
                zp + pos[2],
            )))
        secs.append(ring)

    bm.verts.ensure_lookup_table()

    for j in range(n_sp):
        s0, s1 = secs[j], secs[j + 1]
        for i in range(np):
            ni = (i + 1) % np
            try:
                bm.faces.new([s0[i], s1[i], s1[ni], s0[ni]])
            except ValueError:
                pass

    try:
        bm.faces.new(secs[0])
    except ValueError:
        pass
    try:
        bm.faces.new(list(reversed(secs[-1])))
    except ValueError:
        pass

    bm.to_mesh(mesh)
    bm.free()
    return _link(name, mesh)


def _create_fuselage(name, length, max_r, n_circ, n_long, pos=(0, 0, 0)):
    def r_at(t):
        if t < 0.20:
            return max_r * math.sqrt(t / 0.20) * 0.9
        elif t < 0.70:
            return max_r
        else:
            return max_r * (1.0 - (t - 0.70) / 0.30 * 0.7)

    mesh = bpy.data.meshes.new(name)
    bm = bmesh.new()
    rings = []
    for j in range(n_long + 1):
        t = j / n_long
        y = -length * t + length * 0.3 + pos[1]
        r = r_at(t)
        ring = []
        for i in range(n_circ):
            a = 2.0 * math.pi * i / n_circ
            ring.append(bm.verts.new((
                r * math.cos(a) + pos[0], y, r * math.sin(a) + pos[2])))
        rings.append(ring)

    bm.verts.ensure_lookup_table()

    for j in range(n_long):
        for i in range(n_circ):
            ni = (i + 1) % n_circ
            try:
                bm.faces.new([rings[j][i], rings[j+1][i],
                              rings[j+1][ni], rings[j][ni]])
            except ValueError:
                pass

    nose = bm.verts.new((pos[0], length * 0.3 + 0.01 + pos[1], pos[2]))
    bm.verts.ensure_lookup_table()
    for i in range(n_circ):
        ni = (i + 1) % n_circ
        try:
            bm.faces.new([nose, rings[0][ni], rings[0][i]])
        except ValueError:
            pass

    tail = bm.verts.new((pos[0], -length * 0.7 - 0.01 + pos[1], pos[2]))
    bm.verts.ensure_lookup_table()
    for i in range(n_circ):
        ni = (i + 1) % n_circ
        try:
            bm.faces.new([tail, rings[-1][i], rings[-1][ni]])
        except ValueError:
            pass

    bm.to_mesh(mesh)
    bm.free()
    return _link(name, mesh)


def _create_nacelle(name, radius, length, n_circ, pos=(0, 0, 0)):
    def nr(t):
        if t < 0.3:
            return radius * (0.7 + t / 0.3 * 0.3)
        elif t < 0.7:
            return radius
        else:
            return radius * (1.0 - (t - 0.7) / 0.3 * 0.3)

    n_long = 10
    mesh = bpy.data.meshes.new(name)
    bm = bmesh.new()
    rings = []
    for j in range(n_long + 1):
        t = j / n_long
        y = -length * t + length * 0.5 + pos[1]
        r = nr(t)
        ring = []
        for i in range(n_circ):
            a = 2.0 * math.pi * i / n_circ
            ring.append(bm.verts.new((
                r * math.cos(a) + pos[0], y, r * math.sin(a) + pos[2])))
        rings.append(ring)

    bm.verts.ensure_lookup_table()

    for j in range(n_long):
        for i in range(n_circ):
            ni = (i + 1) % n_circ
            try:
                bm.faces.new([rings[j][i], rings[j+1][i],
                              rings[j+1][ni], rings[j][ni]])
            except ValueError:
                pass

    front = bm.verts.new((pos[0], length * 0.5 + 0.01 + pos[1], pos[2]))
    bm.verts.ensure_lookup_table()
    for i in range(n_circ):
        ni = (i + 1) % n_circ
        try:
            bm.faces.new([front, rings[0][ni], rings[0][i]])
        except ValueError:
            pass

    rear = bm.verts.new((pos[0], -length * 0.5 - 0.01 + pos[1], pos[2]))
    bm.verts.ensure_lookup_table()
    for i in range(n_circ):
        ni = (i + 1) % n_circ
        try:
            bm.faces.new([rear, rings[-1][i], rings[-1][ni]])
        except ValueError:
            pass

    bm.to_mesh(mesh)
    bm.free()
    return _link(name, mesh)


def _create_propeller(name, radius, n_circ=24, pos=(0, 0, 0)):
    """Create a propeller disc mesh (flat circle facing forward)."""
    mesh = bpy.data.meshes.new(name)
    bm = bmesh.new()

    center = bm.verts.new((pos[0], pos[1], pos[2]))
    ring = []
    for i in range(n_circ):
        a = 2.0 * math.pi * i / n_circ
        ring.append(bm.verts.new((
            radius * math.cos(a) + pos[0],
            pos[1],
            radius * math.sin(a) + pos[2],
        )))

    bm.verts.ensure_lookup_table()

    for i in range(n_circ):
        ni = (i + 1) % n_circ
        try:
            bm.faces.new([center, ring[i], ring[ni]])
        except ValueError:
            pass

    bm.to_mesh(mesh)
    bm.free()
    return _link(name, mesh)


def _create_strut(name, p0, p1, radius=0.03, n_circ=8):
    """Create a cylinder strut between two 3D points."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length < 1e-6:
        return None

    mesh = bpy.data.meshes.new(name)
    bm = bmesh.new()

    rings = []
    for j in range(2):
        t = j
        cx = p0[0] + dx * t
        cy = p0[1] + dy * t
        cz = p0[2] + dz * t
        # Build a local coordinate frame perpendicular to the strut direction
        d = (dx / length, dy / length, dz / length)
        # Pick an arbitrary vector not parallel to d
        if abs(d[0]) < 0.9:
            up = (1, 0, 0)
        else:
            up = (0, 1, 0)
        # Cross products for local axes
        u = (d[1] * up[2] - d[2] * up[1],
             d[2] * up[0] - d[0] * up[2],
             d[0] * up[1] - d[1] * up[0])
        ul = math.sqrt(u[0] ** 2 + u[1] ** 2 + u[2] ** 2)
        u = (u[0] / ul, u[1] / ul, u[2] / ul)
        v = (d[1] * u[2] - d[2] * u[1],
             d[2] * u[0] - d[0] * u[2],
             d[0] * u[1] - d[1] * u[0])

        ring = []
        for i in range(n_circ):
            a = 2.0 * math.pi * i / n_circ
            ring.append(bm.verts.new((
                cx + radius * (math.cos(a) * u[0] + math.sin(a) * v[0]),
                cy + radius * (math.cos(a) * u[1] + math.sin(a) * v[1]),
                cz + radius * (math.cos(a) * u[2] + math.sin(a) * v[2]),
            )))
        rings.append(ring)

    bm.verts.ensure_lookup_table()

    for i in range(n_circ):
        ni = (i + 1) % n_circ
        try:
            bm.faces.new([rings[0][i], rings[1][i], rings[1][ni], rings[0][ni]])
        except ValueError:
            pass

    # Cap ends
    try:
        bm.faces.new(rings[0])
    except ValueError:
        pass
    try:
        bm.faces.new(list(reversed(rings[1])))
    except ValueError:
        pass

    bm.to_mesh(mesh)
    bm.free()
    return _link(name, mesh)


# ---------------------------------------------------------------------------
# Post-processing
# ---------------------------------------------------------------------------
def _post_process(obj):
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.normals_make_consistent(inside=False)
    bpy.ops.object.mode_set(mode='OBJECT')
    mod = obj.modifiers.new(name="Triangulate", type='TRIANGULATE')
    bpy.ops.object.modifier_apply(modifier=mod.name)
    obj.select_set(False)


# ---------------------------------------------------------------------------
# Core generate function (shared by operator and standalone)
# ---------------------------------------------------------------------------
def generate_aircraft(
    n_airfoil=40, n_span=10, n_circ=24, n_fuse=30,
    wing_chord=2.0, wing_span=5.0, wing_taper=0.5,
    fuse_length=12.0, fuse_radius=0.8,
    clear=True,
):
    """Generate all aircraft parts and return the list of created objects."""
    if clear:
        _clear_scene()

    objs = []

    # Wings — NACA 2412
    objs.append(_create_wing(
        "Wing_Left", "2412", wing_chord, wing_span, n_airfoil, n_span,
        taper=wing_taper, sweep=5.0, dihedral=3.0,
        mirror_x=False, pos=(0, 0, 0)))
    objs.append(_create_wing(
        "Wing_Right", "2412", wing_chord, wing_span, n_airfoil, n_span,
        taper=wing_taper, sweep=5.0, dihedral=3.0,
        mirror_x=True, pos=(0, 0, 0)))

    # Horizontal tail — NACA 0009
    ty = -(fuse_length * 0.45)
    objs.append(_create_wing(
        "H_Tail_Left", "0009", 1.2, 2.5, n_airfoil, n_span,
        taper=0.6, sweep=8.0, dihedral=0.0,
        mirror_x=False, pos=(0, ty, 0.5)))
    objs.append(_create_wing(
        "H_Tail_Right", "0009", 1.2, 2.5, n_airfoil, n_span,
        taper=0.6, sweep=8.0, dihedral=0.0,
        mirror_x=True, pos=(0, ty, 0.5)))

    # Vertical stabilizer — NACA 0009
    objs.append(_create_vstab(
        "Vertical_Stabilizer", "0009", 1.5, 1.8, n_airfoil, n_span,
        pos=(0, ty, 0.5)))

    # Fuselage
    objs.append(_create_fuselage(
        "Fuselage", fuse_length, fuse_radius, n_circ, n_fuse, pos=(0, 0, 0)))

    # Engine nacelles
    ey, ez = 0.5, -0.3
    objs.append(_create_nacelle(
        "Engine_Left", 0.25, 1.5, n_circ, pos=(2.0, ey, ez)))
    objs.append(_create_nacelle(
        "Engine_Right", 0.25, 1.5, n_circ, pos=(-2.0, ey, ez)))

    for obj in objs:
        _post_process(obj)

    return objs


def generate_biplane(
    n_airfoil=40, n_span=10, n_circ=24, n_fuse=30,
    upper_span=9.0, lower_span=7.0, fuse_length=12.7,
    clear=True,
):
    """Generate an An-2 style biplane and return the list of created objects."""
    if clear:
        _clear_scene()

    objs = []

    # Upper wings — NACA 2412, high-mounted
    upper_chord = 2.4
    upper_z = 1.8  # above fuselage centerline
    objs.append(_create_wing(
        "Upper_Wing_Left", "2412", upper_chord, upper_span, n_airfoil, n_span,
        taper=0.9, sweep=3.0, dihedral=2.0,
        mirror_x=False, pos=(0, 0, upper_z)))
    objs.append(_create_wing(
        "Upper_Wing_Right", "2412", upper_chord, upper_span, n_airfoil, n_span,
        taper=0.9, sweep=3.0, dihedral=2.0,
        mirror_x=True, pos=(0, 0, upper_z)))

    # Lower wings — NACA 2412, lower-mounted, shorter
    lower_chord = 2.0
    lower_z = -0.3
    objs.append(_create_wing(
        "Lower_Wing_Left", "2412", lower_chord, lower_span, n_airfoil, n_span,
        taper=0.85, sweep=2.0, dihedral=5.0,
        mirror_x=False, pos=(0, 0, lower_z)))
    objs.append(_create_wing(
        "Lower_Wing_Right", "2412", lower_chord, lower_span, n_airfoil, n_span,
        taper=0.85, sweep=2.0, dihedral=5.0,
        mirror_x=True, pos=(0, 0, lower_z)))

    # Horizontal tail — NACA 0009
    ty = -(fuse_length * 0.45)
    objs.append(_create_wing(
        "H_Tail_Left", "0009", 1.4, 2.8, n_airfoil, n_span,
        taper=0.6, sweep=8.0, dihedral=0.0,
        mirror_x=False, pos=(0, ty, 0.5)))
    objs.append(_create_wing(
        "H_Tail_Right", "0009", 1.4, 2.8, n_airfoil, n_span,
        taper=0.6, sweep=8.0, dihedral=0.0,
        mirror_x=True, pos=(0, ty, 0.5)))

    # Vertical stabilizer — NACA 0009
    objs.append(_create_vstab(
        "Vertical_Stabilizer", "0009", 1.8, 2.2, n_airfoil, n_span,
        pos=(0, ty, 0.5)))

    # Fuselage — thick body
    objs.append(_create_fuselage(
        "Fuselage", fuse_length, 1.0, n_circ, n_fuse, pos=(0, 0, 0)))

    # Propeller disc at nose
    nose_y = fuse_length * 0.3 + 0.1
    objs.append(_create_propeller(
        "Propeller", 1.4, n_circ, pos=(0, nose_y, 0)))

    # Wing struts (interplane, visual only)
    strut_positions = [2.5, 5.0]
    for idx, sx in enumerate(strut_positions):
        for sign, suffix in [(1.0, "Left"), (-1.0, "Right")]:
            x = sx * sign
            dh_upper = math.tan(math.radians(2.0)) * sx
            dh_lower = math.tan(math.radians(5.0)) * sx
            p_upper = (x, 0, upper_z + dh_upper)
            p_lower = (x, 0, lower_z + dh_lower)
            strut = _create_strut(
                f"Strut_{suffix}_{idx}", p_lower, p_upper, radius=0.03)
            if strut:
                objs.append(strut)

    for obj in objs:
        _post_process(obj)

    return objs


# ═══════════════════════════════════════════════════════════════════════════
# Blender Add-on: Operator
# ═══════════════════════════════════════════════════════════════════════════
class AEROBLEND_OT_generate_aircraft(bpy.types.Operator):
    """Generate a NACA-airfoil example aircraft for AeroBlend"""
    bl_idname = "aeroblend.generate_aircraft"
    bl_label = "Generate Aircraft"
    bl_options = {'REGISTER', 'UNDO'}

    clear_scene: BoolProperty(
        name="Clear Scene",
        description="Remove all existing objects before generating",
        default=True,
    )
    n_airfoil: IntProperty(
        name="Airfoil Points",
        description="Points per airfoil surface (higher = smoother)",
        default=40, min=10, max=200,
    )
    n_span: IntProperty(
        name="Span Sections",
        description="Number of spanwise sections for wings",
        default=10, min=3, max=60,
    )
    n_circ: IntProperty(
        name="Circ. Segments",
        description="Circumferential segments for fuselage / nacelle",
        default=24, min=8, max=96,
    )
    wing_chord: FloatProperty(
        name="Wing Chord (m)", default=2.0, min=0.5, max=10.0)
    wing_span: FloatProperty(
        name="Wing Half-Span (m)", default=5.0, min=1.0, max=20.0)
    wing_taper: FloatProperty(
        name="Wing Taper Ratio", default=0.5, min=0.1, max=1.0)
    fuse_length: FloatProperty(
        name="Fuselage Length (m)", default=12.0, min=3.0, max=40.0)
    fuse_radius: FloatProperty(
        name="Fuselage Radius (m)", default=0.8, min=0.2, max=5.0)

    def execute(self, context):
        objs = generate_aircraft(
            n_airfoil=self.n_airfoil,
            n_span=self.n_span,
            n_circ=self.n_circ,
            wing_chord=self.wing_chord,
            wing_span=self.wing_span,
            wing_taper=self.wing_taper,
            fuse_length=self.fuse_length,
            fuse_radius=self.fuse_radius,
            clear=self.clear_scene,
        )
        self.report({'INFO'}, f"AeroBlend: {len(objs)} parts generated")
        return {'FINISHED'}

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=320)

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "clear_scene")
        layout.separator()
        layout.label(text="Resolution")
        layout.prop(self, "n_airfoil")
        layout.prop(self, "n_span")
        layout.prop(self, "n_circ")
        layout.separator()
        layout.label(text="Wing")
        layout.prop(self, "wing_chord")
        layout.prop(self, "wing_span")
        layout.prop(self, "wing_taper")
        layout.separator()
        layout.label(text="Fuselage")
        layout.prop(self, "fuse_length")
        layout.prop(self, "fuse_radius")


class AEROBLEND_OT_generate_biplane(bpy.types.Operator):
    """Generate an An-2 style biplane for AeroBlend"""
    bl_idname = "aeroblend.generate_biplane"
    bl_label = "Biplane (An-2)"
    bl_options = {'REGISTER', 'UNDO'}

    clear_scene: BoolProperty(
        name="Clear Scene", default=True,
    )
    upper_span: FloatProperty(
        name="Upper Wing Half-Span (m)", default=9.0, min=3.0, max=20.0)
    lower_span: FloatProperty(
        name="Lower Wing Half-Span (m)", default=7.0, min=2.0, max=18.0)
    fuse_length: FloatProperty(
        name="Fuselage Length (m)", default=12.7, min=5.0, max=30.0)

    def execute(self, context):
        objs = generate_biplane(
            upper_span=self.upper_span,
            lower_span=self.lower_span,
            fuse_length=self.fuse_length,
            clear=self.clear_scene,
        )
        self.report({'INFO'}, f"AeroBlend: biplane with {len(objs)} parts generated")
        return {'FINISHED'}

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=320)

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "clear_scene")
        layout.separator()
        layout.prop(self, "upper_span")
        layout.prop(self, "lower_span")
        layout.prop(self, "fuse_length")


# ═══════════════════════════════════════════════════════════════════════════
# Blender Add-on: Custom Part Operators
# ═══════════════════════════════════════════════════════════════════════════
class AEROBLEND_OT_add_wing(bpy.types.Operator):
    """Add a parametric wing to the scene"""
    bl_idname = "aeroblend.add_wing"
    bl_label = "Add Wing"
    bl_options = {'REGISTER', 'UNDO'}

    part_name: EnumProperty(
        name="Name",
        items=[
            ('Wing_Left', "Wing Left", ""),
            ('Wing_Right', "Wing Right", ""),
            ('Upper_Wing_Left', "Upper Wing Left", ""),
            ('Upper_Wing_Right', "Upper Wing Right", ""),
            ('Lower_Wing_Left', "Lower Wing Left", ""),
            ('Lower_Wing_Right', "Lower Wing Right", ""),
        ],
        default='Wing_Left',
    )
    naca: StringProperty(name="NACA Profile", default="2412", maxlen=4)
    chord: FloatProperty(name="Chord (m)", default=2.0, min=0.3, max=10.0)
    span: FloatProperty(name="Half-Span (m)", default=5.0, min=0.5, max=25.0)
    taper: FloatProperty(name="Taper Ratio", default=0.5, min=0.1, max=1.0)
    sweep: FloatProperty(name="Sweep (deg)", default=5.0, min=-10.0, max=45.0)
    dihedral: FloatProperty(name="Dihedral (deg)", default=3.0, min=-10.0, max=20.0)

    def execute(self, context):
        mirror = "Right" in self.part_name
        obj = _create_wing(
            self.part_name, self.naca, self.chord, self.span,
            40, 10, taper=self.taper, sweep=self.sweep,
            dihedral=self.dihedral, mirror_x=mirror,
        )
        _post_process(obj)
        self.report({'INFO'}, f"Added {self.part_name}")
        return {'FINISHED'}

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=300)

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "part_name")
        layout.prop(self, "naca")
        layout.prop(self, "chord")
        layout.prop(self, "span")
        layout.prop(self, "taper")
        layout.prop(self, "sweep")
        layout.prop(self, "dihedral")


class AEROBLEND_OT_add_fuselage(bpy.types.Operator):
    """Add a parametric fuselage to the scene"""
    bl_idname = "aeroblend.add_fuselage"
    bl_label = "Add Fuselage"
    bl_options = {'REGISTER', 'UNDO'}

    length: FloatProperty(name="Length (m)", default=12.0, min=2.0, max=50.0)
    radius: FloatProperty(name="Radius (m)", default=0.8, min=0.1, max=5.0)

    def execute(self, context):
        obj = _create_fuselage("Fuselage", self.length, self.radius, 24, 30)
        _post_process(obj)
        self.report({'INFO'}, "Added Fuselage")
        return {'FINISHED'}

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=260)

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "length")
        layout.prop(self, "radius")


class AEROBLEND_OT_add_engine(bpy.types.Operator):
    """Add an engine mesh (propeller disc or jet nacelle) to the scene"""
    bl_idname = "aeroblend.add_engine"
    bl_label = "Add Engine"
    bl_options = {'REGISTER', 'UNDO'}

    engine_type: EnumProperty(
        name="Type",
        items=[
            ('PROPELLER', "Propeller", "Propeller disc mesh"),
            ('JET', "Jet Nacelle", "Jet engine nacelle"),
        ],
        default='PROPELLER',
    )
    part_name: EnumProperty(
        name="Name",
        items=[
            ('Propeller', "Propeller", ""),
            ('Engine_Left', "Engine Left", ""),
            ('Engine_Right', "Engine Right", ""),
        ],
        default='Propeller',
    )
    radius: FloatProperty(name="Radius (m)", default=1.0, min=0.1, max=5.0)
    nacelle_length: FloatProperty(name="Nacelle Length (m)", default=1.5, min=0.3, max=5.0)
    pos_x: FloatProperty(name="Position X", default=0.0)
    pos_y: FloatProperty(name="Position Y", default=0.0)
    pos_z: FloatProperty(name="Position Z", default=0.0)

    def execute(self, context):
        p = (self.pos_x, self.pos_y, self.pos_z)
        if self.engine_type == 'PROPELLER':
            obj = _create_propeller(self.part_name, self.radius, pos=p)
        else:
            obj = _create_nacelle(
                self.part_name, self.radius, self.nacelle_length, 24, pos=p)
        _post_process(obj)
        self.report({'INFO'}, f"Added {self.part_name}")
        return {'FINISHED'}

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=280)

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "engine_type")
        layout.prop(self, "part_name")
        layout.prop(self, "radius")
        if self.engine_type == 'JET':
            layout.prop(self, "nacelle_length")
        layout.separator()
        layout.label(text="Position")
        row = layout.row(align=True)
        row.prop(self, "pos_x", text="X")
        row.prop(self, "pos_y", text="Y")
        row.prop(self, "pos_z", text="Z")


class AEROBLEND_OT_add_tail(bpy.types.Operator):
    """Add a horizontal or vertical tail surface"""
    bl_idname = "aeroblend.add_tail"
    bl_label = "Add Tail"
    bl_options = {'REGISTER', 'UNDO'}

    tail_type: EnumProperty(
        name="Type",
        items=[
            ('H_TAIL', "Horizontal Tail", "H_Tail_Left + H_Tail_Right pair"),
            ('V_TAIL', "Vertical Stabilizer", "Single vertical stabilizer"),
        ],
        default='H_TAIL',
    )
    chord: FloatProperty(name="Chord (m)", default=1.2, min=0.3, max=5.0)
    span: FloatProperty(name="Span / Height (m)", default=2.5, min=0.5, max=10.0)
    pos_y: FloatProperty(name="Position Y (aft)", default=-5.0, min=-20.0, max=0.0)
    pos_z: FloatProperty(name="Position Z", default=0.5, min=-2.0, max=5.0)

    def execute(self, context):
        objs = []
        if self.tail_type == 'H_TAIL':
            objs.append(_create_wing(
                "H_Tail_Left", "0009", self.chord, self.span, 40, 10,
                taper=0.6, sweep=8.0, dihedral=0.0,
                mirror_x=False, pos=(0, self.pos_y, self.pos_z)))
            objs.append(_create_wing(
                "H_Tail_Right", "0009", self.chord, self.span, 40, 10,
                taper=0.6, sweep=8.0, dihedral=0.0,
                mirror_x=True, pos=(0, self.pos_y, self.pos_z)))
        else:
            objs.append(_create_vstab(
                "Vertical_Stabilizer", "0009", self.chord, self.span, 40, 10,
                pos=(0, self.pos_y, self.pos_z)))
        for obj in objs:
            _post_process(obj)
        self.report({'INFO'}, f"Added {self.tail_type.replace('_', ' ').title()}")
        return {'FINISHED'}

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=280)

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "tail_type")
        layout.prop(self, "chord")
        layout.prop(self, "span")
        layout.separator()
        layout.prop(self, "pos_y")
        layout.prop(self, "pos_z")


# ═══════════════════════════════════════════════════════════════════════════
# Blender Add-on: Physics Parameters
# ═══════════════════════════════════════════════════════════════════════════
class AEROBLEND_OT_estimate_physics(bpy.types.Operator):
    """Auto-estimate mass and inertia from scene geometry bounding boxes"""
    bl_idname = "aeroblend.estimate_physics"
    bl_label = "Auto-Estimate from Geometry"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        scene = context.scene
        # Compute aggregate bounding box
        min_c = [1e9, 1e9, 1e9]
        max_c = [-1e9, -1e9, -1e9]
        has_mesh = False
        from mathutils import Vector
        for obj in scene.objects:
            if obj.type != 'MESH':
                continue
            has_mesh = True
            for corner in obj.bound_box:
                wc = obj.matrix_world @ Vector(corner)
                for i in range(3):
                    min_c[i] = min(min_c[i], wc[i])
                    max_c[i] = max(max_c[i], wc[i])

        if not has_mesh:
            self.report({'WARNING'}, "No mesh objects in scene")
            return {'CANCELLED'}

        sx = max_c[0] - min_c[0]
        sy = max_c[1] - min_c[1]
        sz = max_c[2] - min_c[2]

        # Rough estimate: treat as uniform-density box
        # Typical small aircraft ~4500 kg, scale by volume relative to reference
        ref_vol = 12.0 * 10.0 * 2.0  # reference aircraft volume
        vol = sx * sy * sz
        mass = max(500.0, 4500.0 * vol / ref_vol) if ref_vol > 0 else 4500.0

        # Inertia of a uniform box: I_xx = m/12 * (sy^2 + sz^2), etc.
        ixx = mass / 12.0 * (sy * sy + sz * sz)
        iyy = mass / 12.0 * (sx * sx + sz * sz)
        izz = mass / 12.0 * (sx * sx + sy * sy)

        scene["aeroblend_mass_kg"] = round(mass, 1)
        scene["aeroblend_inertia_xx"] = round(ixx, 1)
        scene["aeroblend_inertia_yy"] = round(iyy, 1)
        scene["aeroblend_inertia_zz"] = round(izz, 1)

        self.report({'INFO'},
                    f"Estimated: {mass:.0f} kg, "
                    f"I=[{ixx:.0f}, {iyy:.0f}, {izz:.0f}] kg-m^2")
        return {'FINISHED'}


class AEROBLEND_OT_export_physics(bpy.types.Operator):
    """Export physics parameters to a JSON sidecar file"""
    bl_idname = "aeroblend.export_physics"
    bl_label = "Export Physics JSON"
    bl_options = {'REGISTER'}

    filepath: StringProperty(
        name="File Path",
        description="Output .physics.json path",
        default="//aircraft.physics.json",
        subtype='FILE_PATH',
    )

    def execute(self, context):
        scene = context.scene
        data = {
            "mass_kg": scene.get("aeroblend_mass_kg", 4500.0),
            "inertia": {
                "xx": scene.get("aeroblend_inertia_xx", 5000.0),
                "yy": scene.get("aeroblend_inertia_yy", 30000.0),
                "zz": scene.get("aeroblend_inertia_zz", 32000.0),
            },
            "engines": [],
        }

        for obj in scene.objects:
            if obj.get("aeroblend_engine_thrust_n") is not None:
                data["engines"].append({
                    "name": obj.name,
                    "thrust_n": obj["aeroblend_engine_thrust_n"],
                    "type": obj.get("aeroblend_engine_type", "Propeller"),
                })

        path = bpy.path.abspath(self.filepath)
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(path, 'w') as f:
            json.dump(data, f, indent=2)

        self.report({'INFO'}, f"Physics exported: {path}")
        return {'FINISHED'}

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}


class AEROBLEND_PT_physics(bpy.types.Panel):
    """Physics parameters sub-panel."""
    bl_label = "Physics Parameters"
    bl_idname = "AEROBLEND_PT_physics"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "AeroBlend"
    bl_parent_id = "AEROBLEND_PT_panel"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        scene = context.scene

        # Ensure defaults exist
        if "aeroblend_mass_kg" not in scene:
            scene["aeroblend_mass_kg"] = 4500.0
        if "aeroblend_inertia_xx" not in scene:
            scene["aeroblend_inertia_xx"] = 5000.0
        if "aeroblend_inertia_yy" not in scene:
            scene["aeroblend_inertia_yy"] = 30000.0
        if "aeroblend_inertia_zz" not in scene:
            scene["aeroblend_inertia_zz"] = 32000.0

        # Mass
        box = layout.box()
        box.label(text="Aircraft", icon='OBJECT_DATA')
        row = box.row()
        row.prop(scene, '["aeroblend_mass_kg"]', text="Mass (kg)")
        col = box.column(align=True)
        row = col.row(align=True)
        row.prop(scene, '["aeroblend_inertia_xx"]', text="Ixx")
        row.prop(scene, '["aeroblend_inertia_yy"]', text="Iyy")
        row = col.row(align=True)
        row.prop(scene, '["aeroblend_inertia_zz"]', text="Izz")
        col.label(text="(kg-m\u00b2)")

        layout.operator("aeroblend.estimate_physics", icon='MOD_PHYSICS')

        # Engine properties on selected object
        obj = context.active_object
        if obj and obj.type == 'MESH':
            box = layout.box()
            box.label(text=f"Engine: {obj.name}", icon='FORCE_WIND')

            if "aeroblend_engine_thrust_n" not in obj:
                obj["aeroblend_engine_thrust_n"] = 0.0
            if "aeroblend_engine_type" not in obj:
                obj["aeroblend_engine_type"] = "Propeller"

            box.prop(obj, '["aeroblend_engine_thrust_n"]', text="Thrust (N)")
            box.prop(obj, '["aeroblend_engine_type"]', text="Type")

        layout.separator()
        layout.operator("aeroblend.export_physics", icon='EXPORT')


# ---------------------------------------------------------------------------
# Project path helpers
# ---------------------------------------------------------------------------
def _detect_project_root():
    """Auto-detect the aeroblend project root from this script's location."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidate = os.path.dirname(script_dir)
    if os.path.isfile(os.path.join(candidate, "CMakeLists.txt")) and \
       os.path.isdir(os.path.join(candidate, "rust")):
        return candidate
    return ""


def _get_project_root(context):
    """Return the project root from preferences, falling back to auto-detect."""
    prefs = context.preferences.addons.get(__name__)
    if prefs and prefs.preferences.project_root:
        return prefs.preferences.project_root
    return _detect_project_root()


def _get_executable(project_root):
    """Return the path to the aeroblend executable."""
    return os.path.join(project_root, "build", "cpp", "aeroblend")


def _get_build_script(project_root):
    """Return the path to the build script."""
    return os.path.join(project_root, "build.sh")


# ═══════════════════════════════════════════════════════════════════════════
# Blender Add-on: Preferences
# ═══════════════════════════════════════════════════════════════════════════
class AEROBLEND_AddonPreferences(bpy.types.AddonPreferences):
    bl_idname = __name__

    project_root: StringProperty(
        name="Project Root",
        description=(
            "Path to the aeroblend project folder "
            "(auto-detected from script location if empty)"
        ),
        default="",
        subtype='DIR_PATH',
    )

    auto_export_path: StringProperty(
        name="Quick-Run Export Path",
        description=(
            "Where to save the .glb when using Build & Run "
            "(relative to project root). "
            "Leave empty to use a temporary file"
        ),
        default="assets/models/_blender_quick_run.glb",
    )

    build_type: bpy.props.EnumProperty(
        name="Build Type",
        items=[
            ('Release', "Release", "Optimized build (recommended)"),
            ('Debug', "Debug", "Debug build with symbols"),
        ],
        default='Release',
    )

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "project_root")
        layout.prop(self, "auto_export_path")
        layout.prop(self, "build_type")

        root = _get_project_root(context)
        if root:
            exe = _get_executable(root)
            has_exe = os.path.isfile(exe)
            box = layout.box()
            box.label(text=f"Project: {root}")
            if has_exe:
                box.label(text="Executable: FOUND", icon='CHECKMARK')
            else:
                box.label(text="Executable: NOT BUILT", icon='ERROR')
        else:
            layout.label(text="Project root not found", icon='ERROR')


# ═══════════════════════════════════════════════════════════════════════════
# Blender Add-on: Set Project Root Operator
# ═══════════════════════════════════════════════════════════════════════════
class AEROBLEND_OT_set_project_root(bpy.types.Operator):
    """Select the aeroblend project folder"""
    bl_idname = "aeroblend.set_project_root"
    bl_label = "Select Project Folder"
    bl_options = {'REGISTER'}

    directory: StringProperty(subtype='DIR_PATH')

    def execute(self, context):
        chosen = self.directory.rstrip("/\\")
        # Validate: must contain CMakeLists.txt and rust/
        if not os.path.isfile(os.path.join(chosen, "CMakeLists.txt")) or \
           not os.path.isdir(os.path.join(chosen, "rust")):
            self.report(
                {'ERROR'},
                "Not an AeroBlend project folder "
                "(CMakeLists.txt or rust/ not found)",
            )
            return {'CANCELLED'}

        prefs = context.preferences.addons.get(__name__)
        if prefs:
            prefs.preferences.project_root = chosen
        self.report({'INFO'}, f"Project root set: {chosen}")
        return {'FINISHED'}

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}


# ═══════════════════════════════════════════════════════════════════════════
# Blender Add-on: Export Operator
# ═══════════════════════════════════════════════════════════════════════════
class AEROBLEND_OT_export_aircraft(bpy.types.Operator):
    """Export the current scene as GLB for AeroBlend"""
    bl_idname = "aeroblend.export_aircraft"
    bl_label = "Export Aircraft GLB"
    bl_options = {'REGISTER'}

    filepath: StringProperty(
        name="File Path",
        description="Output .glb path",
        default="//example_aircraft.glb",
        subtype='FILE_PATH',
    )

    def execute(self, context):
        path = bpy.path.abspath(self.filepath)
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.export_scene.gltf(
            filepath=path,
            export_format='GLB',
            use_selection=True,
            export_apply=True,
        )
        self.report({'INFO'}, f"Exported: {path}")
        return {'FINISHED'}

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}


# ═══════════════════════════════════════════════════════════════════════════
# Blender Add-on: Quick Example Operator
# ═══════════════════════════════════════════════════════════════════════════
class AEROBLEND_OT_quick_example(bpy.types.Operator):
    """Generate an example aircraft with default settings and export to assets/models/"""
    bl_idname = "aeroblend.quick_example"
    bl_label = "Example Aircraft"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        # Generate with defaults
        objs = generate_aircraft()

        # Determine output path
        root = _get_project_root(context)
        if root:
            out_dir = os.path.join(root, "assets", "models")
        else:
            out_dir = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                os.pardir, "assets", "models",
            )
        os.makedirs(out_dir, exist_ok=True)
        glb_path = os.path.join(out_dir, "example_aircraft.glb")

        # Export
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.export_scene.gltf(
            filepath=glb_path,
            export_format='GLB',
            use_selection=True,
            export_apply=True,
        )

        self.report(
            {'INFO'},
            f"Example aircraft ready! ({len(objs)} parts → {glb_path})",
        )
        return {'FINISHED'}


# ---------------------------------------------------------------------------
# Status tracker — shared state visible to panel and operators
# ---------------------------------------------------------------------------
class _Status:
    """Singleton to track build/run state for panel display."""
    state = 'IDLE'       # IDLE | EXPORTING | BUILDING | LAUNCHING | RUNNING | DONE | ERROR
    message = ""
    error_detail = ""
    _process = None
    _timer = None
    _lines = []
    _glb_path = ""
    _root = ""
    _run_after_build = False

    @classmethod
    def reset(cls):
        cls.state = 'IDLE'
        cls.message = ""
        cls.error_detail = ""
        cls._lines = []

    @classmethod
    def set_error(cls, msg, detail=""):
        cls.state = 'ERROR'
        cls.message = msg
        cls.error_detail = detail


def _export_glb(context):
    """Export current scene to GLB and return the path."""
    root = _get_project_root(context)
    prefs = context.preferences.addons.get(__name__)
    rel_path = prefs.preferences.auto_export_path if prefs else ""
    if rel_path and root:
        glb_path = os.path.join(root, rel_path)
        os.makedirs(os.path.dirname(glb_path), exist_ok=True)
    else:
        tmp = tempfile.NamedTemporaryFile(
            suffix=".glb", prefix="aeroblend_", delete=False)
        glb_path = tmp.name
        tmp.close()

    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.export_scene.gltf(
        filepath=glb_path,
        export_format='GLB',
        use_selection=True,
        export_apply=True,
    )
    return glb_path


def _force_panel_redraw():
    """Force all 3D viewports to redraw so the panel updates."""
    for area in bpy.context.screen.areas:
        if area.type == 'VIEW_3D':
            area.tag_redraw()


# ═══════════════════════════════════════════════════════════════════════════
# Blender Add-on: Build Operator
# ═══════════════════════════════════════════════════════════════════════════
class AEROBLEND_OT_build(bpy.types.Operator):
    """Build the AeroBlend simulator (runs build.sh)"""
    bl_idname = "aeroblend.build"
    bl_label = "Build Simulator"
    bl_options = {'REGISTER'}

    def execute(self, context):
        root = _get_project_root(context)
        if not root:
            _Status.set_error("Project folder not set")
            _force_panel_redraw()
            return {'CANCELLED'}

        build_script = _get_build_script(root)
        if not os.path.isfile(build_script):
            _Status.set_error("build.sh not found", build_script)
            _force_panel_redraw()
            return {'CANCELLED'}

        _Status.reset()
        _Status.state = 'BUILDING'
        _Status.message = "Compiling..."
        _Status._root = root
        _Status._run_after_build = False
        _force_panel_redraw()

        try:
            _Status._lines = []
            _Status._process = subprocess.Popen(
                ["bash", build_script],
                cwd=root,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            # Set stdout to non-blocking so readline() won't freeze Blender
            fd = _Status._process.stdout.fileno()
            flags = fcntl.fcntl(fd, fcntl.F_GETFL)
            fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
        except OSError as e:
            _Status.set_error("Failed to start build", str(e))
            _force_panel_redraw()
            return {'CANCELLED'}

        _Status._timer = context.window_manager.event_timer_add(
            0.3, window=context.window)
        context.window_manager.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def modal(self, context, event):
        if event.type != 'TIMER':
            return {'PASS_THROUGH'}

        proc = _Status._process
        if proc is None:
            return {'CANCELLED'}

        # Non-blocking read of output lines
        try:
            while select.select([proc.stdout], [], [], 0)[0]:
                line = proc.stdout.readline()
                if not line:
                    break
                stripped = line.rstrip()
                if stripped:
                    _Status._lines.append(stripped)
                    _Status.message = stripped[-60:]
        except (IOError, OSError):
            pass
        _force_panel_redraw()

        ret = proc.poll()
        if ret is not None:
            context.window_manager.event_timer_remove(_Status._timer)
            _Status._timer = None
            _Status._process = None

            if ret != 0:
                last = _Status._lines[-3:] if _Status._lines else ["(no output)"]
                _Status.set_error("Build failed", "\n".join(last))
                _force_panel_redraw()
                return {'FINISHED'}

            # Build succeeded
            if _Status._run_after_build:
                _Status.state = 'LAUNCHING'
                _Status.message = "Starting simulator..."
                _force_panel_redraw()
                root = _Status._root
                exe = _get_executable(root)
                glb = _Status._glb_path
                try:
                    subprocess.Popen(
                        [exe, "--model", glb],
                        cwd=os.path.dirname(exe),
                    )
                    _Status.state = 'RUNNING'
                    _Status.message = "Simulator is running"
                except OSError as e:
                    _Status.set_error("Failed to launch simulator", str(e))
            else:
                _Status.state = 'DONE'
                _Status.message = "Build complete!"

            _force_panel_redraw()
            return {'FINISHED'}

        return {'PASS_THROUGH'}


# ═══════════════════════════════════════════════════════════════════════════
# Blender Add-on: Run Operator
# ═══════════════════════════════════════════════════════════════════════════
class AEROBLEND_OT_run(bpy.types.Operator):
    """Export current scene and launch AeroBlend simulator"""
    bl_idname = "aeroblend.run"
    bl_label = "Run Simulator"
    bl_options = {'REGISTER'}

    def execute(self, context):
        root = _get_project_root(context)
        if not root:
            _Status.set_error("Project folder not set")
            _force_panel_redraw()
            return {'CANCELLED'}

        exe = _get_executable(root)
        if not os.path.isfile(exe):
            _Status.set_error("Not built yet", "Click Build first")
            _force_panel_redraw()
            return {'CANCELLED'}

        _Status.reset()
        _Status.state = 'EXPORTING'
        _Status.message = "Exporting GLB..."
        _force_panel_redraw()

        try:
            glb_path = _export_glb(context)
        except Exception as e:
            _Status.set_error("Export failed", str(e))
            _force_panel_redraw()
            return {'CANCELLED'}

        _Status.state = 'LAUNCHING'
        _Status.message = "Starting simulator..."
        _force_panel_redraw()

        try:
            subprocess.Popen(
                [exe, "--model", glb_path],
                cwd=os.path.dirname(exe),
            )
            _Status.state = 'RUNNING'
            _Status.message = "Simulator is running"
        except OSError as e:
            _Status.set_error("Failed to launch simulator", str(e))

        _force_panel_redraw()
        return {'FINISHED'}


# ═══════════════════════════════════════════════════════════════════════════
# Blender Add-on: Build & Run Operator
# ═══════════════════════════════════════════════════════════════════════════
class AEROBLEND_OT_build_and_run(bpy.types.Operator):
    """Export scene, build the simulator, and launch"""
    bl_idname = "aeroblend.build_and_run"
    bl_label = "Build & Run"
    bl_options = {'REGISTER'}

    def execute(self, context):
        root = _get_project_root(context)
        if not root:
            _Status.set_error("Project folder not set")
            _force_panel_redraw()
            return {'CANCELLED'}

        build_script = _get_build_script(root)
        if not os.path.isfile(build_script):
            _Status.set_error("build.sh not found", build_script)
            _force_panel_redraw()
            return {'CANCELLED'}

        # Export first
        _Status.reset()
        _Status.state = 'EXPORTING'
        _Status.message = "Exporting GLB..."
        _force_panel_redraw()

        try:
            glb_path = _export_glb(context)
        except Exception as e:
            _Status.set_error("Export failed", str(e))
            _force_panel_redraw()
            return {'CANCELLED'}

        # Start build (will auto-launch after)
        _Status.state = 'BUILDING'
        _Status.message = "Compiling..."
        _Status._root = root
        _Status._glb_path = glb_path
        _Status._run_after_build = True
        _force_panel_redraw()

        try:
            _Status._lines = []
            _Status._process = subprocess.Popen(
                ["bash", build_script],
                cwd=root,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            fd = _Status._process.stdout.fileno()
            flags = fcntl.fcntl(fd, fcntl.F_GETFL)
            fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
        except OSError as e:
            _Status.set_error("Failed to start build", str(e))
            _force_panel_redraw()
            return {'CANCELLED'}

        _Status._timer = context.window_manager.event_timer_add(
            0.3, window=context.window)
        context.window_manager.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def modal(self, context, event):
        # Reuse the Build operator's modal logic
        return AEROBLEND_OT_build.modal(self, context, event)


# ═══════════════════════════════════════════════════════════════════════════
# Blender Add-on: Dismiss Status Operator
# ═══════════════════════════════════════════════════════════════════════════
class AEROBLEND_OT_dismiss_status(bpy.types.Operator):
    """Dismiss the status message"""
    bl_idname = "aeroblend.dismiss_status"
    bl_label = "OK"
    bl_options = {'REGISTER'}

    def execute(self, context):
        _Status.reset()
        _force_panel_redraw()
        return {'FINISHED'}


# ═══════════════════════════════════════════════════════════════════════════
# Blender Add-on: Panel
# ═══════════════════════════════════════════════════════════════════════════
class AEROBLEND_PT_panel(bpy.types.Panel):
    """AeroBlend sidebar panel in the 3D Viewport."""
    bl_label = "AeroBlend"
    bl_idname = "AEROBLEND_PT_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "AeroBlend"

    def draw(self, context):
        layout = self.layout

        # --- Quick Start ---
        box = layout.box()
        box.label(text="Quick Start", icon='LIGHTPROBE_VOLUME')
        row = box.row(align=True)
        row.operator("aeroblend.quick_example", icon='MONKEY', text="Example Aircraft")
        row.operator("aeroblend.generate_biplane", icon='OUTLINER_OB_MESH', text="Biplane (An-2)")
        row.scale_y = 1.3

        layout.separator()

        # --- Model ---
        box = layout.box()
        box.label(text="Model", icon='MESH_DATA')
        row = box.row(align=True)
        row.operator("aeroblend.generate_aircraft", icon='ADD')
        row.operator("aeroblend.export_aircraft", icon='EXPORT')

        box.label(text="Add Part:")
        row = box.row(align=True)
        row.operator("aeroblend.add_wing", icon='MOD_ARRAY', text="Wing")
        row.operator("aeroblend.add_fuselage", icon='MESH_CAPSULE', text="Fuselage")
        row = box.row(align=True)
        row.operator("aeroblend.add_engine", icon='FORCE_WIND', text="Engine")
        row.operator("aeroblend.add_tail", icon='FORWARD', text="Tail")

        layout.separator()

        # --- Simulator ---
        box = layout.box()
        box.label(text="Simulator", icon='PLAY')

        root = _get_project_root(context)
        if not root:
            box.label(text="Project folder not set", icon='ERROR')
            box.operator(
                "aeroblend.set_project_root",
                icon='FILE_FOLDER',
                text="Select Project Folder",
            )
            return

        exe = _get_executable(root)
        has_exe = os.path.isfile(exe)
        is_busy = _Status.state in ('EXPORTING', 'BUILDING', 'LAUNCHING')

        # --- Status display ---
        if _Status.state == 'BUILDING':
            sbox = box.box()
            sbox.label(text="Building...", icon='TIME')
            col = sbox.column(align=True)
            col.scale_y = 0.75
            col.label(text=_Status.message)

        elif _Status.state == 'EXPORTING':
            sbox = box.box()
            sbox.label(text="Exporting...", icon='TIME')

        elif _Status.state == 'LAUNCHING':
            sbox = box.box()
            sbox.label(text="Launching...", icon='TIME')

        elif _Status.state == 'RUNNING':
            sbox = box.box()
            sbox.label(text="Simulator is running", icon='CHECKMARK')
            sbox.operator("aeroblend.dismiss_status", icon='X', text="OK")

        elif _Status.state == 'DONE':
            sbox = box.box()
            sbox.label(text=_Status.message, icon='CHECKMARK')
            sbox.operator("aeroblend.dismiss_status", icon='X', text="OK")

        elif _Status.state == 'ERROR':
            sbox = box.box()
            sbox.alert = True
            sbox.label(text=_Status.message, icon='CANCEL')
            if _Status.error_detail:
                col = sbox.column(align=True)
                col.scale_y = 0.7
                for line in _Status.error_detail.split("\n")[-3:]:
                    col.label(text=line[:70])
            sbox.operator("aeroblend.dismiss_status", icon='X', text="Dismiss")

        # --- Action buttons ---
        row = box.row(align=True)
        row.operator("aeroblend.build_and_run", icon='PLAY', text="Build & Run")
        row.scale_y = 1.5
        row.enabled = not is_busy

        row = box.row(align=True)
        sub = row.row(align=True)
        sub.operator("aeroblend.build", icon='PREFERENCES', text="Build")
        sub.enabled = not is_busy
        sub = row.row(align=True)
        sub.operator("aeroblend.run", icon='SCREEN_BACK', text="Run")
        sub.enabled = has_exe and not is_busy

        if not has_exe and _Status.state == 'IDLE':
            box.label(text="Not built yet", icon='INFO')

        # --- Project path ---
        row = box.row(align=True)
        row.scale_y = 0.7
        row.label(text=os.path.basename(root), icon='FILE_FOLDER')
        row.operator(
            "aeroblend.set_project_root",
            icon='FILE_REFRESH',
            text="",
        )


# ═══════════════════════════════════════════════════════════════════════════
# Registration
# ═══════════════════════════════════════════════════════════════════════════
_classes = (
    AEROBLEND_AddonPreferences,
    AEROBLEND_OT_generate_aircraft,
    AEROBLEND_OT_generate_biplane,
    AEROBLEND_OT_add_wing,
    AEROBLEND_OT_add_fuselage,
    AEROBLEND_OT_add_engine,
    AEROBLEND_OT_add_tail,
    AEROBLEND_OT_estimate_physics,
    AEROBLEND_OT_export_physics,
    AEROBLEND_OT_set_project_root,
    AEROBLEND_OT_export_aircraft,
    AEROBLEND_OT_quick_example,
    AEROBLEND_OT_build,
    AEROBLEND_OT_run,
    AEROBLEND_OT_build_and_run,
    AEROBLEND_OT_dismiss_status,
    AEROBLEND_PT_panel,
    AEROBLEND_PT_physics,
)


def register():
    for cls in _classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(_classes):
        bpy.utils.unregister_class(cls)


# ═══════════════════════════════════════════════════════════════════════════
# Standalone execution (blender --background --python ...)
# ═══════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    # Detect headless vs GUI-script-run
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    output = os.path.join(project_root, "assets", "models", "example_aircraft.glb")

    print("=" * 60)
    print("AeroBlend Example Aircraft Generator")
    print("=" * 60)

    objs = generate_aircraft()

    print(f"Exporting to {output} ...")
    os.makedirs(os.path.dirname(output), exist_ok=True)
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.export_scene.gltf(
        filepath=output,
        export_format='GLB',
        use_selection=True,
        export_apply=True,
    )

    print("=" * 60)
    print(f"Done! {len(objs)} parts generated:")
    for obj in objs:
        print(f"  - {obj.name}")
    print(f"Output: {output}")
    print("=" * 60)
