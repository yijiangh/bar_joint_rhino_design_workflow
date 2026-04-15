"""Generate a Rhino 8 toolbar (.rui) file for the bar joint design workflow.

Run this script from a terminal (not inside Rhino):
    python scripts/generate_rhino_toolbar.py

It produces `BarJointDesign.rui` in the project root.
Drag and drop that file into Rhino 8 to import the toolbar.

The .rui format was reverse-engineered from RhinoPolyhedra.rui (Rhino 8 package).
"""

from __future__ import annotations

import os
import uuid


def _uuid() -> str:
    return str(uuid.uuid4())


def generate_rui(scripts_dir: str, output_path: str) -> None:
    scripts_dir = os.path.abspath(scripts_dir)

    # Script paths (forward slashes work in Rhino on Windows)
    t1_path = os.path.join(scripts_dir, "t1_bar_axis.py").replace("\\", "/")
    t1r_path = os.path.join(scripts_dir, "t1_bar_axis_rerun.py").replace("\\", "/")
    export_s2_case_path = os.path.join(scripts_dir, "export_t1_s2_case.py").replace("\\", "/")
    t2_path = os.path.join(scripts_dir, "t2_joint_placement.py").replace("\\", "/")
    t2r_path = os.path.join(scripts_dir, "t2_joint_placement_rerun.py").replace("\\", "/")

    # Stable GUIDs for each element
    g_rui = _uuid()
    g_t1_left = _uuid()
    g_t1_right = _uuid()
    g_t2_left = _uuid()
    g_t2_right = _uuid()
    g_export_s2_left = _uuid()
    g_export_s2_right = _uuid()
    g_tbg = _uuid()
    g_tbg_item = _uuid()
    g_tb = _uuid()
    g_tbi_t1 = _uuid()
    g_tbi_t2 = _uuid()
    g_tbi_export_s2 = _uuid()
    g_extend_menu = _uuid()

    xml = f'''<?xml version="1.0" encoding="utf-8"?>
<RhinoUI major_ver="3" minor_ver="0" guid="{g_rui}" localize="False" default_language_id="1033" dpi_scale="100">
  <extend_rhino_menus>
    <menu guid="{g_extend_menu}">
      <text>
        <locale_1033>Extend Rhino Menus</locale_1033>
      </text>
    </menu>
  </extend_rhino_menus>
  <menus />
  <tool_bar_groups>
    <tool_bar_group guid="{g_tbg}" dock_bar_guid32="00000000-0000-0000-0000-000000000000" dock_bar_guid64="00000000-0000-0000-0000-000000000000" active_tool_bar_group="{g_tbg_item}" single_file="False" hide_single_tab="False" point_floating="200,200">
      <text>
        <locale_1033>Bar Joint Design</locale_1033>
      </text>
      <tool_bar_group_item guid="{g_tbg_item}" major_version="1" minor_version="0">
        <text>
          <locale_1033>Bar Joint Design</locale_1033>
        </text>
        <tool_bar_id>{g_tb}</tool_bar_id>
      </tool_bar_group_item>
    </tool_bar_group>
  </tool_bar_groups>
  <tool_bars>
    <tool_bar guid="{g_tb}">
      <text>
        <locale_1033>Bar Joint Design</locale_1033>
      </text>
      <tool_bar_item guid="{g_tbi_t1}">
        <left_macro_id>{g_t1_left}</left_macro_id>
        <right_macro_id>{g_t1_right}</right_macro_id>
      </tool_bar_item>
      <tool_bar_item guid="{g_tbi_t2}">
        <left_macro_id>{g_t2_left}</left_macro_id>
        <right_macro_id>{g_t2_right}</right_macro_id>
      </tool_bar_item>
      <tool_bar_item guid="{g_tbi_export_s2}">
        <left_macro_id>{g_export_s2_left}</left_macro_id>
        <right_macro_id>{g_export_s2_right}</right_macro_id>
      </tool_bar_item>
    </tool_bar>
  </tool_bars>
  <macros>
    <macro_item guid="{g_t1_left}">
      <text>
        <locale_1033>T1 Bar Axis</locale_1033>
      </text>
      <tooltip>
        <locale_1033>T1 Bar Axis Generation (new inputs)</locale_1033>
      </tooltip>
      <button_text>
        <locale_1033>T1 Bar</locale_1033>
      </button_text>
      <script>! _-ScriptEditor _R "{t1_path}"</script>
    </macro_item>
    <macro_item guid="{g_t1_right}">
      <text>
        <locale_1033>T1 Bar Axis Rerun</locale_1033>
      </text>
      <tooltip>
        <locale_1033>T1 Bar Axis Rerun (cached inputs)</locale_1033>
      </tooltip>
      <button_text>
        <locale_1033>T1 Re</locale_1033>
      </button_text>
      <script>! _-ScriptEditor _R "{t1r_path}"</script>
    </macro_item>
    <macro_item guid="{g_t2_left}">
      <text>
        <locale_1033>T2 Joint Placement</locale_1033>
      </text>
      <tooltip>
        <locale_1033>T2 Joint Placement (new inputs)</locale_1033>
      </tooltip>
      <button_text>
        <locale_1033>T2 Joint</locale_1033>
      </button_text>
      <script>! _-ScriptEditor _R "{t2_path}"</script>
    </macro_item>
    <macro_item guid="{g_t2_right}">
      <text>
        <locale_1033>T2 Joint Placement Rerun</locale_1033>
      </text>
      <tooltip>
        <locale_1033>T2 Joint Placement Rerun (cached inputs)</locale_1033>
      </tooltip>
      <button_text>
        <locale_1033>T2 Re</locale_1033>
      </button_text>
      <script>! _-ScriptEditor _R "{t2r_path}"</script>
    </macro_item>
    <macro_item guid="{g_export_s2_left}">
      <text>
        <locale_1033>Export T1-S2 Case</locale_1033>
      </text>
      <tooltip>
        <locale_1033>Export the current T1-S2 selection and solver output as a JSON debug case</locale_1033>
      </tooltip>
      <button_text>
        <locale_1033>S2 Case</locale_1033>
      </button_text>
      <script>! _-ScriptEditor _R "{export_s2_case_path}"</script>
    </macro_item>
    <macro_item guid="{g_export_s2_right}">
      <text>
        <locale_1033>Export T1-S2 Case</locale_1033>
      </text>
      <tooltip>
        <locale_1033>Export the current T1-S2 selection and solver output as a JSON debug case</locale_1033>
      </tooltip>
      <button_text>
        <locale_1033>S2 Case</locale_1033>
      </button_text>
      <script>! _-ScriptEditor _R "{export_s2_case_path}"</script>
    </macro_item>
  </macros>
  <bitmaps>
    <small_bitmap item_width="16" item_height="16" />
    <normal_bitmap item_width="24" item_height="24" />
    <large_bitmap item_width="32" item_height="32" />
  </bitmaps>
</RhinoUI>'''

    with open(output_path, "w", encoding="utf-8") as f:
        f.write(xml)


def main():
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    scripts_dir = os.path.join(project_root, "scripts")
    output_path = os.path.join(project_root, "BarJointDesign.rui")

    generate_rui(scripts_dir, output_path)
    print(f"Toolbar file generated: {output_path}")
    print()
    print("To install in Rhino 8:")
    print("  1. Drag and drop the .rui file into the Rhino window")
    print("  2. Or: Rhino command _ToolbarOpen -> browse to the .rui file")
    print()
    print("The toolbar has 3 buttons:")
    print("  [T1 Bar]   Left-click = new inputs | Right-click = rerun last")
    print("  [T2 Joint]  Left-click = new inputs | Right-click = rerun last")
    print("  [S2 Case]  Left-click / Right-click = export a T1-S2 JSON debug case")


if __name__ == "__main__":
    main()
