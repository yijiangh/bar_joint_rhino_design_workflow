#! python 3
# venv: scaffolding_env
# r: numpy
# r: scipy
"""Rhino wrapper for rerunning the last T2 command with cached inputs."""

import importlib
import os
import sys


SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import t2_joint_placement


importlib.reload(t2_joint_placement)
t2_joint_placement.main(rerun=True)
