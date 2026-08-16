"""Shared plumbing for the repo's Grasshopper Python 3 components.

A Grasshopper component is not a Rhino command, and three things that are free
inside ``scripts/rs_*.py`` have to be arranged by hand here:

``sc.doc`` points at the wrong document
    Inside a GH script component ``scriptcontext.doc`` is the *Grasshopper*
    document, not the open ``.3dm``.  Every ``rs.ObjectColor`` /
    ``rs.LayerVisible`` / ``sc.doc.Views`` call in ``core.rhino_bar_registry``
    and ``core.ik_viz`` would therefore hit the GH document and silently do
    nothing visible.  :func:`rhino_doc` swaps it for the length of a solve.

Module globals do not survive a solve
    The component shims reload their modules on every solve
    (``importlib.reload``), which wipes module-level state.  Anything that must
    persist between solves goes in ``sc.sticky`` -- see :func:`state`, and
    ``docs/Su_note.md`` section 20 for the four places state can live.

The document cannot be edited during a solution
    Adding the auto-slider means mutating the GH document while GH is in the
    middle of solving it, which is illegal.  :func:`ensure_int_slider` therefore
    defers the edit through ``GH_Document.ScheduleSolution``.

Nothing in this module imports PyBullet or the planner; it is pure Rhino/GH.
"""

import contextlib

import Rhino  # noqa: F401  (Rhino 8 .NET assembly, injected by the host)
import scriptcontext as sc


# All sticky keys this repo writes are namespaced ``bar_joint:`` (Su_note 20);
# GH component state gets its own sub-namespace so it is obvious in a debugger
# which entries belong to a canvas component rather than to a toolbar command.
STICKY_PREFIX = "bar_joint:gh:"


# ---------------------------------------------------------------------------
# Document swap
# ---------------------------------------------------------------------------


@contextlib.contextmanager
def rhino_doc():
    """Point ``sc.doc`` at the active Rhino document for the length of the block.

    Mandatory around *any* call into ``core.rhino_bar_registry`` /
    ``core.ik_viz`` from a GH component.  The previous value is restored in
    ``finally`` so an exception mid-solve cannot leave the GH document pointing
    at the ``.3dm`` (which would corrupt the next GH bake).

    Yields:
        Rhino.RhinoDoc: the active Rhino document.
    """
    previous = sc.doc
    sc.doc = Rhino.RhinoDoc.ActiveDoc
    try:
        yield sc.doc
    finally:
        sc.doc = previous


# ---------------------------------------------------------------------------
# Per-component state
# ---------------------------------------------------------------------------


def component_of(ghenv_or_component):
    """Return the GH component object from whatever the caller had to hand.

    Three things get passed around and any of them should work:

    * ``ghenv`` -- the host-injected object, present in **both** script mode and
      SDK mode, carrying ``.Component``;
    * ``self`` inside a ``GH_ScriptInstance`` -- also carries ``.Component``;
    * the component itself.

    Returns:
        the component, or None if this is not running inside Grasshopper.
    """
    component = getattr(ghenv_or_component, "Component", None)
    if component is not None:
        return component
    # A real component has Params; a bare GUID string does not.
    if hasattr(ghenv_or_component, "Params"):
        return ghenv_or_component
    return None


def _component_guid(ghenv_or_guid):
    """Accept a ``ghenv`` / ``self`` / component, or an already-extracted GUID."""
    component = component_of(ghenv_or_guid)
    if component is not None:
        return str(component.InstanceGuid)
    return str(ghenv_or_guid)


def state(ghenv_or_guid):
    """Return this component's private ``sc.sticky`` dict, creating it if needed.

    Keyed by the component's ``InstanceGuid``, **not** by a fixed string, so two
    copies of the same pasted code keep separate state.  With one shared key
    they would corrupt each other's bookkeeping in ways that are hard to
    diagnose from the canvas -- the worst being the recorded layer visibility:
    copy A records "centerlines were visible" and hides them, copy B finds a
    record already there and records nothing, then disabling A pops the record
    and restores.  B is now hiding layers it has no record of, and disabling B
    restores nothing.  Also affected: button edges (a press swallowed or fired
    twice), the ``ik_viz`` session flag (A's teardown closes the session B
    thinks is open), and the render-skip fingerprint (each overwrites the
    other's, so the skip never matches).

    What this does *not* fix: two simultaneously **enabled** copies still fight
    over one Rhino document -- same colours, same layers, one globally cached
    ``ik_viz`` bundle -- and GH's solve order decides who wins.  Per-component
    state only guarantees each copy can cleanly undo what it did.

    The returned dict is live: mutate it in place and the change persists to the
    next solve.  It dies when Rhino closes and is never written to the ``.3dm``.

    Args:
        ghenv_or_guid: the component's ``ghenv`` / ``self``, or its GUID.

    Returns:
        dict: the component's state slot.
    """
    key = STICKY_PREFIX + _component_guid(ghenv_or_guid)
    slot = sc.sticky.get(key)
    if slot is None:
        slot = {}
        sc.sticky[key] = slot
    return slot


def forget(ghenv_or_guid):
    """Drop this component's state slot (used after a full teardown)."""
    sc.sticky.pop(STICKY_PREFIX + _component_guid(ghenv_or_guid), None)


def rising_edge(store, key, value):
    """True only on the solve where *value* goes False -> True.

    A GH boolean Button stays True for several solves, so a naive ``if reload:``
    would fire the (expensive) action repeatedly.  Deviation from the plan's
    ``rising_edge(key, value)``: the store is passed in explicitly rather than
    looked up from a global, because both components have a button and they must
    not share one edge record.

    Args:
        store (dict): the component's :func:`state` slot.
        key (str): name of the button, unique within the component.
        value: the button's current value.

    Returns:
        bool: True on the rising edge only.
    """
    edges = store.setdefault("_edges", {})
    now = bool(value)
    was = bool(edges.get(key, False))
    edges[key] = now
    return now and not was


# ---------------------------------------------------------------------------
# Auto-created integer slider
# ---------------------------------------------------------------------------


def find_input_param(ghenv, nickname):
    """Return the component's input parameter whose ``NickName`` matches, or None."""
    component = component_of(ghenv)
    if component is None:
        return None
    for param in component.Params.Input:
        if param.NickName == nickname:
            return param
    return None


def ensure_int_slider(ghenv, nickname, minimum, maximum, initial=0):
    """Guarantee an integer slider is wired to the named input, with these bounds.

    Two cases, one deferred edit:

    * nothing wired -> create a ``GH_NumberSlider`` (integer accuracy, bounds as
      ``System.Decimal``) parked to the left of the component, add it to the
      document and wire it to the parameter;
    * a slider already wired -> only update its bounds, and clamp its value if
      the sequence shrank under it.

    Both branches run inside ``GH_Document.ScheduleSolution(1, ...)``: mutating
    the GH document during a solution is illegal, so the edit is queued for a
    fresh solution 1 ms later.  The practical consequence for callers is that
    the slider does **not** exist yet when this function returns -- the caller
    gets its value on the *next* solve, which is why this is bound to a button
    rather than run every pass.

    Args:
        ghenv: the component's ``ghenv``.
        nickname (str): the input parameter's NickName (e.g. ``"step"``).
        minimum (int): slider minimum.
        maximum (int): slider maximum (raised to *minimum* if lower).
        initial (int): value for a freshly created slider.

    Returns:
        str: a one-line report for the component's ``info`` output.
    """
    # Imported lazily: these are .NET assemblies only present inside Grasshopper,
    # so a plain `import` at module scope would make this module unimportable
    # from a Rhino command or from `python -m compileall`.
    import System
    import System.Drawing
    import Grasshopper

    component = component_of(ghenv)
    if component is None:
        return "ensure_int_slider: no ghenv.Component (not running in Grasshopper?)"

    param = find_input_param(ghenv, nickname)
    if param is None:
        return f"ensure_int_slider: no input parameter named {nickname!r}"

    ghdoc = component.OnPingDocument()
    if ghdoc is None:
        return "ensure_int_slider: component is not in a Grasshopper document"

    minimum = int(minimum)
    maximum = max(int(maximum), minimum)
    initial = min(max(int(initial), minimum), maximum)

    slider_type = Grasshopper.Kernel.Special.GH_NumberSlider
    existing = [src for src in param.Sources if isinstance(src, slider_type)]

    def _edit(_doc):
        if existing:
            slider = existing[0]
            slider.Slider.Minimum = System.Decimal(minimum)
            slider.Slider.Maximum = System.Decimal(maximum)
            # A shorter sequence can leave the handle past the new end.
            current = int(slider.Slider.Value)
            if current < minimum or current > maximum:
                slider.SetSliderValue(System.Decimal(min(max(current, minimum), maximum)))
            slider.ExpireSolution(False)
        else:
            slider = slider_type()
            slider.CreateAttributes()
            slider.Slider.Type = Grasshopper.GUI.Base.GH_SliderAccuracy.Integer
            slider.Slider.DecimalPlaces = 0
            slider.Slider.Minimum = System.Decimal(minimum)
            slider.Slider.Maximum = System.Decimal(maximum)
            slider.SetSliderValue(System.Decimal(initial))
            slider.NickName = nickname
            # Park it left of the parameter it feeds, so a component with several
            # auto-sliders would stack them rather than overlap.
            anchor = getattr(param, "Attributes", None) or component.Attributes
            pivot = anchor.Pivot
            slider.Attributes.Pivot = System.Drawing.PointF(
                float(pivot.X) - 260.0, float(pivot.Y) - 10.0
            )
            ghdoc.AddObject(slider, False)
            param.AddSource(slider)
        component.ExpireSolution(False)

    ghdoc.ScheduleSolution(1, Grasshopper.Kernel.GH_Document.GH_ScheduleDelegate(_edit))
    verb = "updated" if existing else "created"
    return f"slider {verb} on '{nickname}': {minimum}..{maximum} (applies next solve)"
