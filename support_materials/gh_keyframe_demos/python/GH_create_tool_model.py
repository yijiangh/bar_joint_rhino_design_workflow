from compas_robots import ToolModel
from compas.geometry import Transformation
from compas_rhino.conversions import plane_to_compas_frame, mesh_to_compas

cp_visual_meshes = [mesh_to_compas(vm) for vm in tool_visual_meshes]
cp_collision_meshes = [mesh_to_compas(vm) for vm in tool_collision_meshes]
cp_tool_frame = plane_to_compas_frame(tool_frame)

tool_model = ToolModel(None, cp_tool_frame, None, name)

tool_model.add_link("attached_tool_link", visual_meshes=cp_visual_meshes, collision_meshes=cp_collision_meshes)
tool_model._rebuild_tree()
tool_model._create(tool_model.root, Transformation())