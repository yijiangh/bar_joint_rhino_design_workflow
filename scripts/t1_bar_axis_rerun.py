"""Rhino wrapper for rerunning the last T1 command with cached inputs."""

import importlib
import os
import sys


SCRIPT_DIR = os.path.dirname(__file__)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import t1_bar_axis


importlib.reload(t1_bar_axis)
t1_bar_axis.main(rerun=True)
