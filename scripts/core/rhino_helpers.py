"""Shared Rhino helper utilities used by all rs_*.py entry-point scripts.

The one home for the small Rhino chores every command needs: document units,
Rhino ``Transform`` <-> numpy, block instance transforms, objects on layers,
object colours, and a pick-one-option prompt.

This module depends on rhinoscriptsyntax and scriptcontext which are only
available inside Rhino 8 ScriptEditor.  Do not import from standalone tests.
"""

import contextlib

import numpy as np
import rhinoscriptsyntax as rs
import scriptcontext as sc


# ---------------------------------------------------------------------------
# Point / curve utilities
# ---------------------------------------------------------------------------

def point_to_array(point):
    """Convert a Rhino Point3d (or any XYZ-like) to a numpy array."""
    if hasattr(point, "X") and hasattr(point, "Y") and hasattr(point, "Z"):
        return np.array([point.X, point.Y, point.Z], dtype=float)
    return np.asarray(point, dtype=float)


def curve_endpoints(curve_id):
    """Return (start, end) as numpy arrays for a Rhino curve."""
    start = point_to_array(rs.CurveStartPoint(curve_id))
    end = point_to_array(rs.CurveEndPoint(curve_id))
    return start, end


# ---------------------------------------------------------------------------
# Document units and transforms
# ---------------------------------------------------------------------------
#
# numpy matrices in this repo are 4x4 homogeneous transforms.  "doc" means the
# translation is in the Rhino document's unit; "mm" means it is in millimetres
# (what the solver, the registry and the robot code work in).  Rotation columns
# are unitless, so only the translation column is ever scaled.


class NotABlockInstanceError(ValueError, RuntimeError):
    """An object id that is not a block instance.

    Both a ``ValueError`` and a ``RuntimeError``: the copies this replaced
    raised one or the other, so callers written against either still catch it.
    """


def doc_unit_scale_to_mm() -> float:
    """Millimetres per document unit (1.0 in a millimetre document)."""
    import Rhino  # noqa: PLC0415

    return float(
        Rhino.RhinoMath.UnitScale(sc.doc.ModelUnitSystem, Rhino.UnitSystem.Millimeters)
    )


def xform_to_numpy(xform, scale_to_mm: float = 1.0) -> np.ndarray:
    """A Rhino ``Transform`` as a 4x4 numpy matrix.

    The translation is multiplied by *scale_to_mm*; leave it at 1.0 to keep
    document units.
    """
    matrix = np.array(
        [[float(xform[r, c]) for c in range(4)] for r in range(4)], dtype=float
    )
    matrix[:3, 3] *= scale_to_mm
    return matrix


def numpy_to_xform(matrix, scale_from_mm: float = 1.0):
    """A 4x4 numpy matrix as a Rhino ``Transform``.

    The translation is multiplied by *scale_from_mm*; leave it at 1.0 when the
    matrix is already in document units.
    """
    import Rhino  # noqa: PLC0415

    m = np.array(matrix, dtype=float, copy=True)
    m[:3, 3] *= scale_from_mm
    xform = Rhino.Geometry.Transform(1.0)
    for r in range(4):
        for c in range(4):
            xform[r, c] = float(m[r, c])
    return xform


def xform_to_np_mm(xform) -> np.ndarray:
    """A document-unit Rhino ``Transform`` as a 4x4 numpy matrix in mm."""
    return xform_to_numpy(xform, doc_unit_scale_to_mm())


def np_mm_to_xform(matrix_mm):
    """A 4x4 numpy matrix in mm as a document-unit Rhino ``Transform``."""
    return numpy_to_xform(matrix_mm, 1.0 / doc_unit_scale_to_mm())


def block_instance_xform(object_id) -> np.ndarray:
    """World transform of a block instance, 4x4 numpy, document units.

    Raises :class:`NotABlockInstanceError` when *object_id* is not one.
    """
    import Rhino  # noqa: PLC0415

    rh_obj = rs.coercerhinoobject(object_id)
    if not isinstance(rh_obj, Rhino.DocObjects.InstanceObject):
        raise NotABlockInstanceError(f"Object {object_id} is not a block instance.")
    return xform_to_numpy(rh_obj.InstanceXform)


def block_instance_xform_mm(object_id) -> np.ndarray:
    """World transform of a block instance, 4x4 numpy, translation in mm."""
    matrix = block_instance_xform(object_id)
    matrix[:3, 3] *= doc_unit_scale_to_mm()
    return matrix


# ---------------------------------------------------------------------------
# Object-ID list normalisation
# ---------------------------------------------------------------------------

def as_object_id_list(object_ids):
    """Normalise *object_ids* (single id, list, or None) to a flat list."""
    if object_ids is None:
        return []
    if isinstance(object_ids, (str, bytes)):
        return [object_ids]
    try:
        return [oid for oid in object_ids if oid is not None]
    except TypeError:
        return [object_ids]


# ---------------------------------------------------------------------------
# Layer helpers
# ---------------------------------------------------------------------------

def ensure_layer(layer_name, color=None):
    """Create *layer_name* (possibly a nested ``Parent::Child`` path) if it
    does not exist, and make every layer along the path visible.  If
    *color* is given (an RGB tuple or System.Drawing.Color) it is always
    applied to the leaf layer.  Returns the full path."""
    parts = layer_name.split("::")
    cur = ""
    for i, name in enumerate(parts):
        cur = name if i == 0 else cur + "::" + name
        if not rs.IsLayer(cur):
            rs.AddLayer(cur)
        if hasattr(rs, "LayerVisible") and not rs.LayerVisible(cur):
            rs.LayerVisible(cur, True)
    if color is not None:
        rs.LayerColor(layer_name, color)
    return layer_name


def objects_on_layers(*layer_names) -> list:
    """Every object id on the named layers, in layer order.

    A layer that does not exist (yet) contributes nothing instead of raising.
    """
    out = []
    for layer in layer_names:
        if rs.IsLayer(layer):
            out.extend(rs.ObjectsByLayer(layer) or [])
    return out


# ---------------------------------------------------------------------------
# Display helpers
# ---------------------------------------------------------------------------

#: ``rs.ObjectColorSource`` values: the object takes its layer's colour, or its own.
COLOR_BY_LAYER = 0
COLOR_BY_OBJECT = 1

def apply_object_display(object_ids, label, color=None, layer_name=None):
    """Set name, color, layer, and reference_label user-text on objects."""
    baked_ids = as_object_id_list(object_ids)
    multiple = len(baked_ids) > 1
    for i, oid in enumerate(baked_ids):
        if layer_name is not None:
            ensure_layer(layer_name)
            rs.ObjectLayer(oid, layer_name)
        if color is not None:
            set_object_color(oid, color)
        obj_label = f"{label}_{i + 1}" if multiple else label
        rs.ObjectName(oid, obj_label)
        rs.SetUserText(oid, "reference_label", label)
    return baked_ids


def set_object_color(object_ids, color):
    """Set by-object color on one or more Rhino objects."""
    for oid in as_object_id_list(object_ids):
        if not rs.IsObject(oid):
            continue
        if hasattr(rs, "ObjectColorSource"):
            rs.ObjectColorSource(oid, COLOR_BY_OBJECT)
        rs.ObjectColor(oid, color)


def reset_object_color(object_ids):
    """Give one or more Rhino objects their layer's colour back."""
    for oid in as_object_id_list(object_ids):
        if rs.IsObject(oid) and hasattr(rs, "ObjectColorSource"):
            rs.ObjectColorSource(oid, COLOR_BY_LAYER)


def delete_objects(object_ids):
    """Delete one or more Rhino objects (silently skips missing ones)."""
    for oid in as_object_id_list(object_ids):
        if rs.IsObject(oid):
            rs.DeleteObject(oid)


# ---------------------------------------------------------------------------
# Layer + group helpers
# ---------------------------------------------------------------------------

def set_objects_layer(object_ids, layer_name):
    """Move objects to *layer_name*, creating it if needed."""
    baked_ids = as_object_id_list(object_ids)
    ensure_layer(layer_name)
    for oid in baked_ids:
        if rs.IsObject(oid):
            rs.ObjectLayer(oid, layer_name)
    return baked_ids


def group_objects(object_ids):
    """Group objects and return the group name, or None."""
    baked_ids = as_object_id_list(object_ids)
    if not baked_ids:
        return None
    group_name = rs.AddGroup()
    if not group_name:
        return None
    rs.AddObjectsToGroup(baked_ids, group_name)
    return group_name


# ---------------------------------------------------------------------------
# Document user text (persistent per-file state)
# ---------------------------------------------------------------------------
#
# Three storage tiers exist for "remember this", and they are easy to mix up:
#
#   * a local / session variable -> dies when the script ends;
#   * ``scriptcontext.sticky``   -> a dict Rhino keeps between script runs, but
#     it dies when Rhino closes and is never written to the .3dm;
#   * ``sc.doc.Strings`` (below) -> document user text, SAVED INSIDE the .3dm,
#     so it survives a Rhino restart and travels with the file.
#
# Anything that must still be true after reopening the document belongs here.
# This is the same store ``rs.Get/SetDocumentUserText`` reads and writes, and it
# shows up under Document Properties -> User Text.


def get_doc_string(key):
    """Return the document-stored string for *key*, or ``None`` if unset.

    Empty strings are normalised to ``None``, so ``set_doc_string(key, "")`` is a
    valid "clear this setting".  Never raises: outside a live document (or on a
    Rhino build without ``doc.Strings``) it simply reports ``None``.
    """
    try:
        value = sc.doc.Strings.GetValue(key)
    except Exception:
        value = None
    return value or None


def set_doc_string(key, value):
    """Store *value* under *key* in the document's user text.

    Pass ``""`` to clear the entry (:func:`get_doc_string` then returns ``None``).
    Never raises, for the same reason as :func:`get_doc_string`.
    """
    try:
        sc.doc.Strings.SetString(key, str(value))
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Command-line prompts
# ---------------------------------------------------------------------------


def ask_option(prompt, options, default=None):
    """Ask the user to click one of *options* on the command line.

    Returns the chosen option's name.  With a *default*, Enter returns it; with
    none, Enter does nothing.  Esc returns ``None``.  Option names follow
    Rhino's rule: letters and digits, no spaces (``JointPairAndTool``).
    """
    import Rhino  # noqa: PLC0415

    go = Rhino.Input.Custom.GetOption()
    go.SetCommandPrompt(prompt)
    by_index = {go.AddOption(name): name for name in options}
    if default is not None:
        go.SetCommandPromptDefault(default)
    go.AcceptNothing(default is not None)
    while True:
        result = go.Get()
        if result == Rhino.Input.GetResult.Nothing:
            return default
        if result == Rhino.Input.GetResult.Option:
            if go.OptionIndex() in by_index:
                return by_index[go.OptionIndex()]
            continue
        return None


# ---------------------------------------------------------------------------
# Redraw context manager
# ---------------------------------------------------------------------------

@contextlib.contextmanager
def suspend_redraw():
    """Context manager that disables viewport redraw for performance."""
    previous_state = None
    redraw_supported = hasattr(rs, "EnableRedraw")
    try:
        if redraw_supported:
            previous_state = rs.EnableRedraw(False)
        yield
    finally:
        redraw_is_enabled = True
        if redraw_supported:
            redraw_is_enabled = True if previous_state is None else bool(previous_state)
            rs.EnableRedraw(redraw_is_enabled)
        if redraw_is_enabled and hasattr(rs, "Redraw"):
            rs.Redraw()


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------


def add_centered_line(midpoint, direction, length_mm):
    """Add a Rhino line of *length_mm* centered at *midpoint* along
    *direction* (unit-normalized internally).  Returns the new line's
    object id.
    """
    direction = np.asarray(direction, dtype=float)
    direction = direction / np.linalg.norm(direction)
    half_length = float(length_mm) / 2.0
    return rs.AddLine(
        midpoint - half_length * direction, midpoint + half_length * direction
    )
