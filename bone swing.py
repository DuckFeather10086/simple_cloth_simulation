import bpy
import bmesh
import mathutils

bl_info = {
    "name": "骨骼四面体生成器",
    "blender": (3, 0, 0),
    "category": "Object",
    "description": "在选中骨骼的尾部创建四面体，并应用布料模拟",
}


def register_properties():
    bpy.types.Scene.tetra_top_pin_weight = bpy.props.FloatProperty(
        name="顶点权重",
        description="顶点的pin组权重 (0-1)",
        default=0.3,
        min=0.0,
        max=1.0,
        precision=3  # 设置精度为3位小数
    )


def unregister_properties():
    del bpy.types.Scene.tetra_top_pin_weight


class OBJECT_OT_create_tetrahedron(bpy.types.Operator):
    """在选中骨骼的尾部创建四面体"""
    bl_idname = "object.create_tetrahedron"
    bl_label = "生成四面体"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        result = create_tetrahedron_for_bone(
            context.scene.tetra_top_pin_weight)
        if result is not None:
            self.report({'WARNING'}, result)
            return {'CANCELLED'}
        return {'FINISHED'}


class VIEW3D_PT_tetrahedron_tool(bpy.types.Panel):
    """UI 面板：添加到 N 面板的骨骼工具"""
    bl_label = "四面体工具"
    bl_idname = "VIEW3D_PT_tetrahedron_tool"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "骨骼工具"

    def draw(self, context):
        layout = self.layout
        # 使用普通属性字段而不是滑块
        layout.prop(context.scene, "tetra_top_pin_weight")
        layout.operator("object.create_tetrahedron", text="生成四面体")


def create_tetrahedron_for_bone(top_pin_weight):
    obj = bpy.context.object
    if obj is None or obj.type != 'ARMATURE':
        return "请选中一个骨骼对象"

    bone = bpy.context.active_pose_bone
    if bone is None:
        return "请选中一个骨骼"

    armature = obj.data
    if bpy.context.mode != 'EDIT_ARMATURE':
        bpy.ops.object.mode_set(mode='EDIT')

    edit_bone = armature.edit_bones.get(bone.name)
    if edit_bone is None:
        return "无法找到编辑模式中的骨骼"

    head = edit_bone.head
    tail = edit_bone.tail
    direction = (tail - head).normalized()

    up = mathutils.Vector((0, 0, 1)) if abs(
        direction.z) < 0.99 else mathutils.Vector((1, 0, 0))
    right = direction.cross(up).normalized()
    up = right.cross(direction).normalized()

    size = (tail - head).length / 4
    # 调整顶点位置：top 在 tail，底面在 head 附近
    v_top_pos = tail  # top顶点在tail位置
    p1 = head + right * size + up * size
    p2 = head - right * size + up * size
    p3 = head - right * size - up * size
    p4 = head + right * size - up * size

    mesh = bpy.data.meshes.new("Tetrahedron")
    tetra_obj = bpy.data.objects.new("Tetrahedron", mesh)
    bpy.context.collection.objects.link(tetra_obj)

    bm = bmesh.new()
    v_top = bm.verts.new(v_top_pos)  # 在tail位置的顶点
    v1 = bm.verts.new(p1)
    v2 = bm.verts.new(p2)
    v3 = bm.verts.new(p3)
    v4 = bm.verts.new(p4)

    # 创建边
    edges = [
        bm.edges.new([v_top, v1]), bm.edges.new([v_top, v2]),
        bm.edges.new([v_top, v3]), bm.edges.new([v_top, v4]),
        bm.edges.new([v1, v2]), bm.edges.new([v2, v3]),
        bm.edges.new([v3, v4]), bm.edges.new([v4, v1])
    ]

    bm.to_mesh(mesh)
    bm.free()

    pin_group = tetra_obj.vertex_groups.new(name="pin")
    top_group = tetra_obj.vertex_groups.new(name="top")
    pin_group.add([1, 2, 3, 4], 1.0, 'REPLACE')  # 底面顶点
    pin_group.add([0], top_pin_weight, 'REPLACE')  # tail位置的顶点使用传入的权重值
    top_group.add([0], 1.0, 'REPLACE')  # tail位置的顶点

    # 设置显示为In Front
    tetra_obj.show_in_front = True

    cloth_mod = tetra_obj.modifiers.new(name="Cloth", type='CLOTH')
    cloth_mod.settings.vertex_group_mass = "pin"
    cloth_mod.settings.bending_model = 'LINEAR'  # 设置bend mode为linear

    # 获取当前骨骼的父骨骼
    parent_bone = edit_bone.parent
    parent_bone_name = parent_bone.name if parent_bone else None

    if parent_bone_name:
        # 保存原始的世界空间矩阵
        original_matrix_world = tetra_obj.matrix_world.copy()

        # 设置父级关系
        tetra_obj.parent = obj
        tetra_obj.parent_type = 'BONE'
        tetra_obj.parent_bone = parent_bone_name

        # 恢复原始的世界空间位置
        tetra_obj.matrix_world = original_matrix_world

    # 切换回对象模式
    bpy.ops.object.mode_set(mode='OBJECT')

    # 设置四面体为活动对象并选中
    bpy.context.view_layer.objects.active = tetra_obj
    tetra_obj.select_set(True)
    obj.select_set(True)  # 选中骨骼对象

    # 添加约束
    bone = obj.pose.bones[bone.name]  # 重新获取pose bone
    constraint = bone.constraints.new(type='DAMPED_TRACK')
    constraint.subtarget = "top"  # 设置DAMPED_TRACK的顶点组为top
    constraint.target = tetra_obj
    constraint.track_axis = 'TRACK_Y'

    print("四面体创建完成，并应用了骨骼约束！")
    return None


def register():
    register_properties()
    bpy.utils.register_class(OBJECT_OT_create_tetrahedron)
    bpy.utils.register_class(VIEW3D_PT_tetrahedron_tool)


def unregister():
    unregister_properties()
    bpy.utils.unregister_class(OBJECT_OT_create_tetrahedron)
    bpy.utils.unregister_class(VIEW3D_PT_tetrahedron_tool)


if __name__ == "__main__":
    register()
