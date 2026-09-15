# SPDX-License-Identifier: GPL-2.0-or-later
"""Single-bone pyramid spring cages, sharing Weaver ownership and cleanup."""
import bpy
from mathutils import Vector
from uuid import uuid4
from . import skeletal_weaver as sw


def generate_cages(context, top_pin_weight):
    armature = context.active_object
    selected = list(context.selected_pose_bones or [])
    if not selected:
        raise ValueError('Select bones in Pose Mode')
    # Capture every posed bone before changing mode or adding feedback constraints.
    samples = [(b, armature.matrix_world @ b.head,
                armature.matrix_world @ b.tail,
                (armature.matrix_world @ b.matrix).to_3x3() @ Vector((1, 0, 0)))
               for b in selected]
    bpy.ops.object.mode_set(mode='OBJECT')
    cages = []
    for bone, head, tail, right in samples:
        direction = tail - head
        if direction.length < 1e-8:
            raise ValueError('Zero-length bone: ' + bone.name)
        size = direction.length / 4
        direction.normalize()
        right -= direction * right.dot(direction)
        if right.length < 1e-8:
            raise ValueError('Degenerate bone transform: ' + bone.name)
        right.normalize()
        up = right.cross(direction).normalized()
        verts = [tail, head + (right + up)*size, head + (-right + up)*size,
                 head + (-right - up)*size, head + (right - up)*size]
        edges = [(0, 1), (0, 2), (0, 3), (0, 4), (1, 2), (2, 3), (3, 4), (4, 1)]
        mesh = bpy.data.meshes.new('Tetrahedron')
        mesh.from_pydata(verts, edges, [])
        mesh.update()
        cage = bpy.data.objects.new('Tetrahedron', mesh)
        context.collection.objects.link(cage)
        cage[sw.OWNER_KEY] = uuid4().hex
        cage[sw.ROLE_KEY] = 'GUIDE'
        cage[sw.SOURCE_KEY] = armature
        cage[sw.BONES_KEY] = [bone.name]
        cage.show_in_front = True
        pin = cage.vertex_groups.new(name='Pin')
        pin.add([1, 2, 3, 4], 1.0, 'REPLACE')
        pin.add([0], top_pin_weight, 'REPLACE')
        target_group = cage.vertex_groups.new(name=bone.name)
        target_group.add([0], 1.0, 'REPLACE')
        cage[sw.TARGETS_KEY] = {bone.name: target_group.name}
        cloth = sw.add_cloth_modifier(cage)
        cloth.settings.bending_model = 'LINEAR'
        sw.parent_mesh_to_armature(cage, armature, bone.parent)
        cages.append((bone, cage))
    for bone, cage in cages:
        sw.add_bone_constraints(cage, [[bone]], 1, 1)
    context.view_layer.objects.active = armature
    bpy.ops.object.mode_set(mode='POSE')
    return {'FINISHED'}


class OBJECT_OT_create_tetrahedron(bpy.types.Operator):
    bl_idname = 'object.create_tetrahedron'
    bl_label = '生成四面体代理'
    bl_description = '为每根选中骨生成方锥弹簧代理，替换这些骨上插件创建的驱动'
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return (context.mode == 'POSE' and context.active_object is not None
                and context.active_object.type == 'ARMATURE'
                and bool(context.selected_pose_bones))

    def execute(self, context):
        count = len(context.selected_pose_bones)
        result = sw.run_generation(self, context,
            lambda ctx: generate_cages(ctx, ctx.scene.tetra_top_pin_weight))
        if result == {'FINISHED'}:
            self.report({'INFO'}, f'已生成 {count} 个骨骼代理')
        return result


class VIEW3D_PT_tetrahedron_tool(bpy.types.Panel):
    bl_label = '四面体骨骼代理'
    bl_idname = 'VIEW3D_PT_tetrahedron_tool'
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Simple Cloth'
    bl_options = {'DEFAULT_CLOSED'}

    @classmethod
    def poll(cls, context):
        return context.mode == 'POSE' and context.object and context.object.type == 'ARMATURE'

    def draw(self, context):
        self.layout.prop(context.scene, 'tetra_top_pin_weight')
        self.layout.operator('object.create_tetrahedron', text='生成 / 替换选中骨的代理')


classes = (OBJECT_OT_create_tetrahedron, VIEW3D_PT_tetrahedron_tool)


def register():
    bpy.types.Scene.tetra_top_pin_weight = bpy.props.FloatProperty(
        name='尾端 Pin 权重', description='骨尾的回位权重；底部固定跟随父骨',
        default=0.3, min=0, max=1, precision=3)
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
    del bpy.types.Scene.tetra_top_pin_weight
