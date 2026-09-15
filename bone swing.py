# SPDX-License-Identifier: GPL-2.0-or-later
"""Compatibility launcher for the historical Text Editor script."""
from pathlib import Path
import sys
import bpy

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import simple_cloth_simulation

if __name__ == '__main__':
    if not hasattr(bpy.types.Scene, 'skeletal_weaver_props'):
        simple_cloth_simulation.register()
