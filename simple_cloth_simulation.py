# SPDX-License-Identifier: GPL-2.0-or-later
"""Compatibility entry point. Tree construction lives in skeletal_weaver."""
import bpy
from bpy.props import BoolProperty, FloatProperty, IntProperty, PointerProperty
from bpy.types import Operator, PropertyGroup


class SimpleClothSimulation_Properties(PropertyGroup):
    # Retain old serialized properties for existing blend files. They are never
    # reset by a load handler. New generation uses the shared Weaver controls.
    sub_level: IntProperty(name="Legacy Chain Count", min=0, default=1)
    thickness: FloatProperty(default=1)
    finger_thickness: FloatProperty(default=1)
    parent_bones: FloatProperty()
    connect_mesh: BoolProperty()
    connect_parents: BoolProperty()
    generate_all: BoolProperty()
    head_ornaments: BoolProperty()
    apply_mod: BoolProperty(default=True)
    parent_armature: BoolProperty(default=True)


class BONE_OT_custom_shape(Operator):
    bl_idname = "object.simple_cloth_simulation"
    bl_label = "Simple Cloth Simulation"
    bl_description = "Create Cloth using the shared bone-tree generator and Simple Cloth panel settings"
    bl_options = {'UNDO', 'INTERNAL'}

    @classmethod
    def poll(cls, context):
        return (context.mode == 'POSE' and context.active_object is not None
                and context.active_object.type == 'ARMATURE'
                and bool(context.selected_pose_bones))

    def execute(self, context):
        settings = context.scene.skeletal_weaver_props
        previous = settings.physics_mode, settings.use_physics_modifier
        try:
            settings.physics_mode = 'CLOTH'
            settings.use_physics_modifier = True
            return bpy.ops.armature.skeletal_weave()
        finally:
            settings.physics_mode, settings.use_physics_modifier = previous


classes = (SimpleClothSimulation_Properties, BONE_OT_custom_shape)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.simple_cloth_sim = PointerProperty(type=SimpleClothSimulation_Properties)


def unregister():
    del bpy.types.Scene.simple_cloth_sim
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
