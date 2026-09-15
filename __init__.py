# SPDX-License-Identifier: GPL-2.0-or-later

import bpy
from . import simple_cloth_simulation
from . import skeletal_weaver
from . import bone_swing

bl_info = {
    "name": "Simple Cloth Simulation",
    "author": "Duck Feather",
    "version": (1, 2, 0),
    "blender": (5, 0, 0),
    "location": "View3D > Sidebar > Simple Cloth",
    "description": "Cloth simulation and skeletal weaver mesh generator for armatures",
    "category": "Animation",
}


def register():
    simple_cloth_simulation.register()
    skeletal_weaver.register()
    bone_swing.register()


def unregister():
    bone_swing.unregister()
    skeletal_weaver.unregister()
    simple_cloth_simulation.unregister()


if __name__ == "__main__":
    register()
