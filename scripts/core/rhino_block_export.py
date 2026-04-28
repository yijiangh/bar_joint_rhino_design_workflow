"""Shared Rhino block-definition export helper.

Used by both ``rs_define_joint_pair`` and ``rs_define_robotic_tool`` to
write a single block definition to a stand-alone .3dm asset file.

Strategy: Rhino's ``_-BlockManager _Export`` command writes the block's
component geometry as loose objects in a new .3dm.  This is what we want:
the asset is openable, visually inspectable, and re-editable by hand.  If
that command fails for any reason we fall back to inserting an instance,
selecting it, and using ``_-Export``.
"""

from __future__ import annotations

import os


def export_block_definition_to_3dm(block_name: str, output_path: str) -> bool:
    """Write a single-block-definition .3dm file.

    Returns True on success.  Verbose progress is printed to the console
    so failures can be diagnosed.
    """
    import rhinoscriptsyntax as rs  # noqa: PLC0415

    output_path = os.path.normpath(output_path)
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    if os.path.exists(output_path):
        try:
            os.remove(output_path)
            print(f"    [export_block] removed existing file: {output_path}")
        except OSError as exc:
            print(f"    [export_block] failed to remove existing file: {exc}")

    cmd_path = output_path.replace("/", "\\")

    # ---- Attempt 1: _-BlockManager _Export ----------------------------
    cmd = '_-BlockManager _Export "{name}" "{path}" _Enter _Enter'.format(
        name=block_name, path=cmd_path
    )
    print(f"    [export_block] running: {cmd}")
    ok_cmd = rs.Command(cmd, echo=True)
    print(
        f"    [export_block] _-BlockManager rs.Command -> {ok_cmd}, "
        f"exists={os.path.exists(output_path)}"
    )
    if ok_cmd and os.path.exists(output_path):
        return True

    # ---- Attempt 2: insert + select + _-Export ------------------------
    temp_id = rs.InsertBlock(block_name, [0.0, 0.0, 0.0])
    print(f"    [export_block] InsertBlock -> {temp_id}")
    if temp_id is None:
        return False
    try:
        rs.UnselectAllObjects()
        rs.SelectObject(temp_id)
        cmd2 = '_-Export "{path}" _Enter _Enter'.format(path=cmd_path)
        print(f"    [export_block] running: {cmd2}")
        ok2 = rs.Command(cmd2, echo=True)
        print(
            f"    [export_block] _-Export rs.Command -> {ok2}, "
            f"exists={os.path.exists(output_path)}"
        )
        return bool(ok2) and os.path.exists(output_path)
    finally:
        if rs.IsObject(temp_id):
            rs.DeleteObject(temp_id)
        rs.UnselectAllObjects()
