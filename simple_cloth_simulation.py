# SPDX-License-Identifier: GPL-2.0-or-later

import bpy
from bpy.props import FloatProperty, IntProperty, BoolProperty, PointerProperty
from bpy.types import Operator, Panel, PropertyGroup
from mathutils import Vector, Euler

bl_info = {
    "name": "Simple Cloth Simulation",
    "author": "Duck Feather",
    "version": (1, 0, 0),
    "blender": (2, 80, 0),
    "location": "Pose Mode > Sidebar > Simple Cloth",
    "description": "Creates a cloth simulation mesh from selected bones",
    "warning": "",
    "doc_url": "",
    "category": "Object",
}


def create_cloth_mesh(mesh, shape_object, bones, chain_count=6):
    """
    Generate a planar mesh from bone positions.
    """
    verts = []
    edges = []
    faces = []
    bone_chain_edges = []
    idx = 0

    # Generate vertices at bone head and tail positions
    for b in bones:
        vert1 = b.head
        vert2 = b.tail
        verts.append(vert1)
        verts.append(vert2)

        edges.append([idx, idx + 1])
        bone_chain_edges.append([idx, idx + 1])
        idx = idx + 2

    # Create faces connecting bone chains
    for i in range(0, len(bone_chain_edges) - chain_count):
        if len(bone_chain_edges) >= i + chain_count:
            face = (bone_chain_edges[i][0], bone_chain_edges[i][1],
                    bone_chain_edges[i + chain_count][1],
                    bone_chain_edges[i + chain_count][0])
            faces.append(face)

    # Build the mesh
    mesh.from_pydata(verts, edges, faces)
    mesh.update()

    # Remove duplicate vertices
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.remove_doubles()
    bpy.ops.object.mode_set(mode='OBJECT')

    # Create vertex groups for each bone
    pin_group = []
    for i, b in enumerate(bones):
        group_name = f"bone_{i}_{b.name}"
        new_group = shape_object.vertex_groups.new(name=group_name)
        new_group.add([i * 2], 1.0, 'REPLACE')

        # Create constraint to follow mesh vertex
        crc = b.constraints.new('DAMPED_TRACK')
        crc.target = shape_object
        crc.subtarget = group_name
        crc.track_axis = "TRACK_Y"

        # Mark every (chain_count+1)th vertex as pin group
        if i % (chain_count + 1) == 0:
            pin_group.append(i * 2)

    # Create pin group for cloth simulation
    if pin_group:
        pin_vg = shape_object.vertex_groups.new(name="pin_group")
        pin_vg.add(pin_group, 1.0, 'REPLACE')


def setup_cloth_simulation(shape_object):
    """
    Add cloth modifier to the mesh.
    """
    # Add cloth modifier
    cloth_mod = shape_object.modifiers.new(name="Cloth", type='CLOTH')

    # Set pin group
    cloth_mod.settings.vertex_group_mass = "pin_group"

    return cloth_mod


def main(context):
    """
    Main function to create cloth simulation mesh from selected bones.
    """
    # Check if selection is valid
    if len(context.selected_pose_bones) == 0:
        return {'CANCELLED'}, "No bones selected"

    if len(context.selected_objects) == 0:
        return {'CANCELLED'}, "No objects selected"

    if context.selected_objects[0].type != 'ARMATURE':
        return {'CANCELLED'}, "Selected object is not an armature"

    # Get scene properties
    scn = context.scene
    props = scn.simple_cloth_props

    # Get armature and bones
    armature_object = context.object
    bone_selection = list(context.selected_pose_bones)

    # Store original transform
    old_location = Vector(armature_object.location)
    old_rotation = Euler(armature_object.rotation_euler)
    old_scale = Vector(armature_object.scale)

    # Clear transforms for accurate mesh creation
    bpy.ops.object.rotation_clear(clear_delta=False)
    bpy.ops.object.location_clear(clear_delta=False)
    bpy.ops.object.scale_clear(clear_delta=False)

    # Get scale for mesh
    scale = bpy.context.object.scale

    # Switch to object mode and create mesh
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.select_all(action='DESELECT')

    # Create new mesh object at armature location
    bpy.ops.object.add(type='MESH', enter_editmode=False,
                       location=old_location)

    # Get the new mesh object
    ob = context.view_layer.objects.active
    ob.name = armature_object.name + "_cloth"
    me = ob.data
    me.name = armature_object.name + "_cloth_mesh"

    # Create the cloth mesh from bones
    create_cloth_mesh(me, ob, bone_selection, props.chain_count)

    # Setup cloth simulation
    setup_cloth_simulation(ob)

    # Parent mesh to armature
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.select_all(action='DESELECT')
    ob.select_set(True)
    armature_object.select_set(True)
    bpy.context.view_layer.objects.active = armature_object
    bpy.ops.object.parent_set(type='BONE')

    # Restore original transforms
    armature_object.location = old_location
    armature_object.rotation_euler = old_rotation
    armature_object.scale = old_scale
    bpy.ops.object.mode_set(mode='OBJECT')

    return {'FINISHED'}, me


class SIMPLECLOTH_OT_create(Operator):
    bl_idname = "object.simple_cloth_create"
    bl_label = "Create Cloth Mesh"
    bl_description = "Creates a cloth simulation mesh from selected bones"
    bl_options = {'UNDO'}

    @classmethod
    def poll(cls, context):
        return (context.active_object is not None and
                context.active_object.type == 'ARMATURE' and
                context.active_object.mode == 'POSE')

    def execute(self, context):
        result = main(context)
        if result[0] == {'CANCELLED'}:
            self.report({'WARNING'}, result[1])
            return {'CANCELLED'}
        else:
            self.report({'INFO'}, result[1].name + " has been created")
            return {'FINISHED'}


class SIMPLECLOTH_PT_main(Panel):
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Simple Cloth"
    bl_label = "Simple Cloth Simulation"

    @classmethod
    def poll(cls, context):
        ob = context.object
        return ob and ob.type == 'ARMATURE' and ob.mode == 'POSE'

    def draw(self, context):
        layout = self.layout
        props = context.scene.simple_cloth_props

        row = layout.row()
        row.operator("object.simple_cloth_create", text="Create Cloth Mesh",
                     icon='MESH_DATA')

        layout.separator()

        row = layout.row()
        row.prop(props, "chain_count", text="Chain Count")

        row = layout.row()
        row.prop(props, "stiffness", text="Stiffness")
        row.prop(props, "damping", text="Damping")


class SimpleClothProperties(PropertyGroup):
    chain_count: IntProperty(
        name="Chain Count",
        min=1,
        max=20,
        default=6,
        description="Number of bones per chain"
    )

    stiffness: FloatProperty(
        name="Stiffness",
        min=0.0,
        max=1.0,
        default=0.5,
        description="Cloth stiffness"
    )

    damping: FloatProperty(
        name="Damping",
        min=0.0,
        max=1.0,
        default=0.0,
        description="Cloth damping"
    )


def register():
    bpy.utils.register_class(SIMPLECLOTH_OT_create)
    bpy.utils.register_class(SIMPLECLOTH_PT_main)
    bpy.utils.register_class(SimpleClothProperties)

    bpy.types.Scene.simple_cloth_props = PointerProperty(
        type=SimpleClothProperties
    )


def unregister():
    bpy.utils.unregister_class(SIMPLECLOTH_OT_create)
    bpy.utils.unregister_class(SIMPLECLOTH_PT_main)
    bpy.utils.unregister_class(SimpleClothProperties)

    del bpy.types.Scene.simple_cloth_props


if __name__ == "__main__":
    register()
