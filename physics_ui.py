# SPDX-License-Identifier: GPL-2.0-or-later
"""One sidebar for local jiggle, bone cloth and marked hanging chains."""
import bpy
from bpy.props import BoolProperty, EnumProperty, FloatProperty, IntProperty, PointerProperty, StringProperty
from . import physics_core as core, mesh_physics as mesh, chain_layout as chain
from . import skeletal_weaver as sw


class SCSPhysicsSettings(bpy.types.PropertyGroup):
    mode: EnumProperty(name='用途', items=[('JIGGLE', '软体抖动', '身体网格选区或已有软体骨骼'),
        ('CLOTH', '头发 / 衣摆', '沿用骨树织网'), ('CHAIN', '悬吊链条', '代理节点、指定 Hook、链节排列')], default='CHAIN')
    preset: EnumProperty(name='部位', items=[('BREAST', '胸部', '柔软回弹起点'),
        ('THIGH', '大腿', '较紧的回弹起点'), ('BUTT', '臀部', '较重的回弹起点')])
    region: StringProperty(name='抖动权重组', description='留空使用选中顶点；组权重越高越软，不修改原组')
    feather: IntProperty(name='边缘过渡圈数', default=2, min=0, max=10)
    anchor_mode: EnumProperty(name='初始固定点', items=[('FIRST', '首端', '顶点编号较小的端点'),
        ('ENDS', '两端', '固定两个端点'), ('SELECTED', '选中顶点', '在编辑模式选择固定点')])
    target: PointerProperty(name='Hook 目标', type=bpy.types.Object,
        description='留空则新建控制点；选择物体或骨架，绑定时保留当前偏移')
    bone: StringProperty(name='目标骨骼')
    template: PointerProperty(name='链节模板', type=bpy.types.Object, poll=lambda self, obj: obj.type == 'MESH')
    axis: EnumProperty(name='链节长轴', items=[(axis, axis, '') for axis in 'XYZ'], default='Y')
    alternate: BoolProperty(name='相邻链节交替旋转 90°', default=True)
    fit: BoolProperty(name='按代理节距缩放链节', default=True)
    pitch: FloatProperty(name='节距 / 链节长度', default=.7, min=.05, max=2.)
    nodes: IntProperty(name='节点数', default=16, min=3, max=512)
    length: FloatProperty(name='代理长度', default=1., min=.001, unit='LENGTH')
    scene_scope: BoolProperty(name='处理场景全部模拟', default=False,
        description='关闭时仅处理选中骨骼、网格、节点或 Hook 对应的模拟')


def object_mode(context):
    if context.object and context.object.mode != 'OBJECT':
        bpy.ops.object.mode_set(mode='OBJECT')


class SCS_OT_create(bpy.types.Operator):
    bl_idname = 'scs.create_simulation'
    bl_label = '生成模拟'
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        settings = context.scene.scs_physics
        obj = context.object
        if not obj:
            self.report({'ERROR'}, '请选择网格或骨骼')
            return {'CANCELLED'}
        try:
            if context.mode == 'POSE' and obj.type == 'ARMATURE':
                if settings.mode == 'JIGGLE':
                    context.scene.tetra_top_pin_weight = {'BREAST': .65, 'THIGH': .8, 'BUTT': .7}[settings.preset]
                    return bpy.ops.object.create_tetrahedron()
                props = context.scene.skeletal_weaver_props
                props.use_physics_modifier = props.create_vertex_groups = props.add_bone_constraints = True
                props.physics_mode = 'SOFT_BODY' if settings.mode == 'CHAIN' else 'CLOTH'
                if settings.mode == 'CHAIN':
                    props.softbody_hook_preset = 'HEAD_ONLY'
                    props.pin_use_falloff = False
                return bpy.ops.armature.skeletal_weave()
            if obj.type != 'MESH' or settings.mode == 'CLOTH':
                raise ValueError('头发 / 衣摆请在姿态模式选择骨链；软体和链条可选 Mesh')
            if context.mode == 'EDIT_MESH':
                obj.update_from_editmode()
            indices = [v.index for v in obj.data.vertices if v.select]
            object_mode(context)
            if settings.mode == 'CHAIN':
                if settings.anchor_mode != 'SELECTED':
                    indices = mesh.endpoints(obj)[:1 if settings.anchor_mode == 'FIRST' else 2]
                mesh.prepare_chain(obj, indices)
                chain.mark_nodes(obj)
            else:
                if settings.region:
                    group = obj.vertex_groups.get(settings.region)
                    if group is None:
                        raise ValueError('权重组不存在：' + settings.region)
                    weights = {v.index: g.weight for v in obj.data.vertices for g in v.groups if g.group == group.index}
                else:
                    weights = mesh.jiggle_weights(obj, indices, feather=settings.feather)
                goal, stiffness, damping, mass = {
                    'BREAST': (.65, .8, 4., .3), 'THIGH': (.8, .9, 6., .4),
                    'BUTT': (.7, .85, 5., .5)}[settings.preset]
                mesh.prepare_jiggle(obj, weights, goal=goal, stiffness=stiffness, damping=damping, mass=mass)
        except (ValueError, RuntimeError) as exc:
            self.report({'ERROR'}, str(exc))
            return {'CANCELLED'}
        self.report({'INFO'}, '已生成；从场景起始帧连续播放预览')
        return {'FINISHED'}


class SCS_OT_proxy(bpy.types.Operator):
    bl_idname = 'scs.create_chain_proxy'
    bl_label = '在游标处新建链条代理'
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        object_mode(context)
        settings = context.scene.scs_physics
        chain.create_proxy(context, settings.nodes, settings.length)
        return {'FINISHED'}


class SCS_OT_chain(bpy.types.Operator):
    bl_idname = 'scs.chain_action'
    bl_label = '链条操作'
    bl_options = {'REGISTER', 'UNDO'}
    action: EnumProperty(items=[('MARK', '显示节点编号', ''), ('HIDE', '隐藏节点编号', ''),
        ('HOOK', '绑定选中节点的 Hook', ''), ('LINKS', '排列 / 更新链节', ''), ('CLEAR', '移除链节外观', '')])

    def execute(self, context):
        settings = context.scene.scs_physics
        proxy = chain.proxy_from(context)
        if proxy is None:
            self.report({'ERROR'}, '请选择已经生成模拟的代理或节点标记')
            return {'CANCELLED'}
        indices = chain.selected_nodes(context, proxy)
        restore_edit = context.mode == 'EDIT_MESH'
        try:
            object_mode(context)
            if self.action == 'MARK':
                chain.mark_nodes(proxy)
            elif self.action == 'HIDE':
                chain.clear_layout(proxy, 'NODE')
            elif self.action == 'HOOK':
                mesh.add_hook(proxy, indices, target=settings.target,
                              bone=settings.bone if settings.target and settings.target.type == 'ARMATURE' else '')
            elif self.action == 'LINKS':
                count = chain.build_links(proxy, settings.template, axis=settings.axis,
                                          alternate=settings.alternate, fit=settings.fit, pitch=settings.pitch)
                self.report({'INFO'}, f'已沿代理排列 {count} 个链节')
            else:
                chain.clear_layout(proxy, 'LINK')
                chain.clear_layout(proxy, 'LINK_DRIVER')
        except (ValueError, RuntimeError) as exc:
            self.report({'ERROR'}, str(exc))
            return {'CANCELLED'}
        finally:
            if restore_edit:
                context.view_layer.objects.active = proxy
                bpy.ops.object.mode_set(mode='EDIT')
        return {'FINISHED'}


class SCS_OT_manage(bpy.types.Operator):
    bl_idname = 'scs.manage'
    bl_label = '管理模拟'
    bl_options = {'REGISTER', 'UNDO'}
    action: EnumProperty(items=[('BAKE', '烘焙', ''), ('FREE', '清除缓存 / 回起始帧', ''),
        ('ON', '启用', ''), ('OFF', '停用', ''), ('REMOVE', '移除模拟', ''),
        ('COLLIDE', '添加碰撞物', ''), ('UNCOLLIDE', '移除本工具添加的碰撞', '')])

    def execute(self, context):
        objects = core.simulations(context, scene_scope=context.scene.scs_physics.scene_scope)
        try:
            if self.action == 'COLLIDE':
                core.add_colliders(context.selected_objects)
            elif self.action == 'UNCOLLIDE':
                core.remove_colliders(context.selected_objects)
            elif self.action in {'BAKE', 'FREE'}:
                core.cache_operation(context, objects, free=self.action == 'FREE')
            elif self.action in {'ON', 'OFF'}:
                core.set_enabled(objects, self.action == 'ON')
            else:
                core.remove_simulations(objects)
        except (ValueError, RuntimeError) as exc:
            self.report({'ERROR'}, str(exc))
            return {'CANCELLED'}
        return {'FINISHED'}


def action(layout, operator, text, value):
    layout.operator(operator, text=text).action = value


class SCS_PT_physics(bpy.types.Panel):
    bl_idname = 'SCS_PT_physics'
    bl_label = 'Simple Physics · 物理合集'
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Simple Cloth'
    bl_order = 0

    def draw(self, context):
        layout = self.layout
        settings = context.scene.scs_physics
        layout.prop(settings, 'mode')
        if settings.mode == 'JIGGLE':
            layout.prop(settings, 'preset')
            if context.object and context.object.type == 'MESH':
                layout.prop_search(settings, 'region', context.object, 'vertex_groups')
                layout.prop(settings, 'feather')
                layout.label(text='空权重组：使用选中顶点')
            else:
                layout.label(text='选身体局部顶点，或已有软体骨骼')
        elif settings.mode == 'CLOTH':
            layout.label(text='姿态模式选择头发 / 衣摆骨链')
            props = context.scene.skeletal_weaver_props
            layout.prop(props, 'ribbon_width')
            layout.prop(props, 'pin_use_falloff')
        else:
            box = layout.box()
            box.prop(settings, 'nodes')
            box.prop(settings, 'length')
            box.operator('scs.create_chain_proxy')
            layout.label(text='Tab 编辑路径，再生成模拟')
            layout.prop(settings, 'anchor_mode')
        layout.operator('scs.create_simulation', icon='PHYSICS')
        if settings.mode == 'CHAIN':
            box = layout.box()
            box.label(text='节点与 Hook')
            row = box.row(align=True)
            action(row, 'scs.chain_action', '显示编号', 'MARK')
            action(row, 'scs.chain_action', '隐藏编号', 'HIDE')
            box.label(text='选择编号球，或 Tab 选择代理顶点')
            box.prop(settings, 'target')
            if settings.target and settings.target.type == 'ARMATURE':
                box.prop_search(settings, 'bone', settings.target.data, 'bones')
            action(box, 'scs.chain_action', '指定 Hook（空目标则新建）', 'HOOK')
            box = layout.box()
            box.label(text='沿代理排列实际链节')
            box.prop(settings, 'template')
            box.prop(settings, 'axis')
            box.prop(settings, 'alternate')
            box.prop(settings, 'fit')
            if settings.fit:
                box.prop(settings, 'pitch')
            action(box, 'scs.chain_action', '排列 / 更新链节', 'LINKS')
            action(box, 'scs.chain_action', '移除链节外观', 'CLEAR')


class SCS_PT_controls(bpy.types.Panel):
    bl_idname = 'SCS_PT_controls'
    bl_label = '模拟参数 / 碰撞 / 缓存'
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Simple Cloth'
    bl_order = 1

    def draw(self, context):
        layout = self.layout
        settings = context.scene.scs_physics
        objects = core.simulations(context, scene_scope=settings.scene_scope)
        layout.prop(settings, 'scene_scope')
        layout.label(text=f'当前范围：{len(objects)} 个模拟')
        for obj in objects[:8]:
            mod = core.solver_for(obj)
            box = layout.box()
            box.label(text=obj.name, icon='MOD_PHYSICS')
            box.label(text='已烘焙' if mod.point_cache.is_baked else '未烘焙')
            col = box.column()
            col.enabled = not mod.point_cache.is_baked
            native = mod.settings
            col.prop(native, 'mass', text='质量')
            if mod.type == 'SOFT_BODY':
                for name, label in [('pull', '抗拉'), ('push', '抗压'), ('damping', '阻尼'),
                                    ('goal_spring', '回位刚度'), ('goal_friction', '回位阻尼')]:
                    col.prop(native, name, text=label)
            else:
                for name, label in [('tension_stiffness', '抗拉'), ('bending_stiffness', '抗弯'),
                                    ('tension_damping', '阻尼'), ('quality', '求解质量')]:
                    col.prop(native, name, text=label)
                col.prop(mod.collision_settings, 'use_collision', text='碰撞')
            col.prop(native.effector_weights, 'gravity', text='重力影响')
        box = layout.box()
        box.label(text='碰撞：先在物体模式选身体 / 障碍物')
        action(box, 'scs.manage', '添加碰撞物', 'COLLIDE')
        action(box, 'scs.manage', '移除本工具添加的碰撞', 'UNCOLLIDE')
        row = layout.row(align=True)
        row.prop(context.scene, 'frame_start', text='起始')
        row.prop(context.scene, 'frame_end', text='结束')
        row = layout.row(align=True)
        action(row, 'scs.manage', '烘焙', 'BAKE')
        action(row, 'scs.manage', '清除 / 重置', 'FREE')
        row = layout.row(align=True)
        action(row, 'scs.manage', '启用', 'ON')
        action(row, 'scs.manage', '停用', 'OFF')
        action(layout, 'scs.manage', '移除当前范围的模拟', 'REMOVE')


CLASSES = (SCSPhysicsSettings, SCS_OT_create, SCS_OT_proxy, SCS_OT_chain, SCS_OT_manage,
           SCS_PT_physics, SCS_PT_controls)


def register():
    for cls in CLASSES:
        bpy.utils.register_class(cls)
    bpy.types.Scene.scs_physics = PointerProperty(type=SCSPhysicsSettings)


def unregister():
    del bpy.types.Scene.scs_physics
    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)
