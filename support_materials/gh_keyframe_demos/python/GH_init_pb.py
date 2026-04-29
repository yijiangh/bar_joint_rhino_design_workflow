from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner
import pybullet as p
from scriptcontext import sticky as st
from compas_ghpython import create_id

gh_component = ghenv.Component
sticky_id = create_id(gh_component, '__')
if sticky_id not in st:
    st[sticky_id] = None, None

if trigger:
    client = PyBulletClient(connection_type="gui" if use_gui else "direct", verbose=True)
    client.__enter__()

    # Use PyBullet debug mode by enabling the debug visualizer and optionally adding debug lines/info
    # This will show joint axes, link frames, and other debug info in the GUI
    if use_gui:
        pass
        # p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1, physicsClientId=client.client_id)
        # p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1, physicsClientId=client.client_id)
        # p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 1, physicsClientId=client.client_id)

    planner = PyBulletPlanner(client)
    planner.set_robot_cell(robot_cell)
    st[sticky_id] = client, planner

client, planner = st[sticky_id]
