# SPDX-License-Identifier: GPL-2.0-or-later
"""Native Mesh springs and local jiggle, appended after existing animation."""
from uuid import uuid4
import bpy
from mathutils import Matrix, Vector
from . import physics_core as core
from . import skeletal_weaver as sw


def endpoints(obj):
    if obj.type != 'MESH' or obj.data.polygons:
        raise ValueError('自动端点需要单条无面的绳中心线；其他网格请手选固定点')
    adjacent = [set() for _ in obj.data.vertices]
    for edge in obj.data.edges:
        a, b = edge.vertices
        adjacent[a].add(b)
        adjacent[b].add(a)
    ends = [i for i, neighbors in enumerate(adjacent) if len(neighbors) == 1]
    if len(ends) != 2 or any(len(a) not in (1, 2) for a in adjacent):
        raise ValueError('需要单条开放边链；闭环、分叉请手选固定点')
    seen, pending = set(), [ends[0]]
    while pending:
        index = pending.pop()
        if index not in seen:
            seen.add(index)
            pending.extend(adjacent[index] - seen)
    if len(seen) != len(adjacent):
        raise ValueError('存在断开的链；请分开处理或手选固定点')
    return ends


def preset_anchors(obj, preset):
    head, tail = endpoints(obj)
    return {'HEAD_ONLY': [head], 'TAIL_ONLY': [tail], 'BOTH_ENDS': [head, tail]}[preset]


def validate_mesh(obj):
    if obj.type != 'MESH' or not obj.data.edges:
        raise ValueError('请选择有边的 Mesh')
    if obj.mode != 'OBJECT':
        raise ValueError('请返回物体模式')
    if any(m.type in {'CLOTH', 'SOFT_BODY'} for m in obj.modifiers):
        raise ValueError('网格已有物理，请先移除或使用原模拟：' + obj.name)
    if any(abs(s - 1) > 1e-5 for s in obj.scale):
        raise ValueError('请先应用对象缩放：' + obj.name)
    # Insert after existing deformation, before topology-changing presentation.
    allowed = {'ARMATURE', 'HOOK', 'LATTICE', 'MESH_DEFORM', 'SURFACE_DEFORM',
               'CORRECTIVE_SMOOTH', 'SMOOTH', 'CAST', 'DISPLACE', 'SIMPLE_DEFORM'}
    presentation = {'SUBSURF', 'SOLIDIFY', 'BEVEL', 'NODES', 'SKIN', 'WEIGHTED_NORMAL', 'NORMAL_EDIT', 'COLLISION'}
    insertion = len(obj.modifiers)
    for i, mod in enumerate(obj.modifiers):
        if mod.type in presentation:
            insertion = min(insertion, i)
        elif mod.type not in allowed or i > insertion:
            raise ValueError('请先整理修改器顺序（动画 → 物理 → 外观）：' + mod.name)
    return insertion


def prepare_chain(obj, anchors, *, mass=.3, stiffness=.9, damping=2., automatic_hooks=True):
    insertion = validate_mesh(obj)
    indices = sorted(set(anchors))
    if not indices or any(i < 0 or i >= len(obj.data.vertices) for i in indices):
        raise ValueError('请选择固定点')
    if len(indices) == len(obj.data.vertices):
        raise ValueError('不能固定全部顶点；至少留一个点参与模拟')
    token = uuid4().hex[:8]
    group = obj.vertex_groups.new(name='SCS Anchors ' + token)
    group.add(list(range(len(obj.data.vertices))), 0., 'REPLACE')
    group.add(indices, 1., 'REPLACE')
    mod = obj.modifiers.new('SCS Chain ' + token, 'SOFT_BODY')
    obj.modifiers.move(list(obj.modifiers).index(mod), insertion)
    settings = mod.settings
    settings.use_goal = settings.use_edges = True
    settings.vertex_group_goal = group.name
    settings.goal_min, settings.goal_max, settings.goal_default = 0., 1., 1.
    settings.mass = mass
    settings.pull = settings.push = stiffness
    settings.damping = damping
    settings.step_min, settings.step_max = 10, 200
    sw.set_cache_range(mod)
    obj[core.KIND], obj[core.SOLVER] = 'CHAIN', mod.name
    obj[core.GROUPS] = [group.name]
    if automatic_hooks:
        for index in indices:
            add_hook(obj, [index])
    return mod


def add_hook(obj, indices, *, target=None, bone=''):
    mod = core.solver_for(obj)
    if not mod:
        raise ValueError('先生成模拟')
    core.require_unbaked([obj])
    indices = sorted(set(indices))
    if not indices or any(i < 0 or i >= len(obj.data.vertices) for i in indices):
        raise ValueError('请选择有效顶点')
    if sw.is_guide_object(obj) and target is None:
        return sw.create_hook(obj, indices, 'SW_Hook_User')[0]
    if target is not None:
        if target == obj or target.get('scs_chain_source') == obj:
            raise ValueError('不能把链条自身、节点标记或链节作为自己的 Hook 目标')
        upstream = target
        while upstream is not None:
            if upstream == obj:
                raise ValueError('Hook 目标不能是链条的子物体')
            upstream = upstream.parent
        if any(getattr(c, 'target', None) == obj for c in target.constraints):
            raise ValueError('Hook 目标正在跟随本代理，会形成循环依赖')
        if bone and (target.type != 'ARMATURE' or bone not in target.pose.bones):
            raise ValueError('Hook 目标骨骼不存在')
        if target.type == 'ARMATURE' and bone:
            chain = [target.pose.bones[bone]] + list(target.pose.bones[bone].parent_recursive)
            if any(c.type == 'DAMPED_TRACK' and c.target == obj for b in chain for c in b.constraints):
                raise ValueError('Hook 目标骨骼由本代理驱动，会形成循环依赖')
    # Sample upstream animation, not raw coordinates, for zero-jump binding.
    suffix = list(obj.modifiers)[list(obj.modifiers).index(mod):]
    states = [m.show_viewport for m in suffix]
    try:
        for m in suffix:
            m.show_viewport = False
        bpy.context.view_layer.update()
        evaluated = obj.evaluated_get(bpy.context.evaluated_depsgraph_get())
        center = sum((evaluated.data.vertices[i].co for i in indices), Vector()) / len(indices)
        positions = {i: evaluated.data.vertices[i].co.copy() for i in indices}
    finally:
        for m, state in zip(suffix, states):
            m.show_viewport = state
    control = target
    if control is None:
        control = bpy.data.objects.new('SCS Anchor', None)
        bpy.context.collection.objects.link(control)
        control.empty_display_type = 'SPHERE'
        control.empty_display_size = .025
        control.hide_render = True
        control['scs_anchor_source'] = obj
        sw.set_parent_keep_world(control, obj.parent, obj.parent_type, obj.parent_bone,
                                 Matrix.Translation(obj.matrix_world @ center))
    bpy.context.view_layer.update()
    target_matrix = (control.matrix_world @ control.pose.bones[bone].matrix
                     if bone else control.matrix_world)
    inverse = target_matrix.inverted() @ obj.matrix_world
    if obj.get('scs_region_source'):
        for index, coordinate in positions.items():
            reference_name = obj['scs_region_reference_hooks'].get(str(index))
            reference_hook = obj.modifiers.get(reference_name) if reference_name else None
            if reference_hook:
                obj.modifiers.remove(reference_hook)
            obj.data.vertices[index].co = coordinate
    # Reassign selected nodes instead of stacking two competing Hook transforms.
    for existing in list(obj.modifiers):
        if existing.type != 'HOOK' or existing.name not in obj.get('scs_physics_hooks', []):
            continue
        existing_group = obj.vertex_groups.get(existing.vertex_group)
        if existing_group:
            existing_group.remove(indices)
            if not any(any(g.group == existing_group.index for g in v.groups) for v in obj.data.vertices):
                old_control = existing.object
                obj.modifiers.remove(existing)
                if old_control and old_control != control and old_control.get('scs_anchor_source') == obj and old_control.users <= 1:
                    bpy.data.objects.remove(old_control, do_unlink=True)
    hook = obj.modifiers.new('SCS Hook ' + uuid4().hex[:8], 'HOOK')
    hook.object = control
    hook.subtarget = bone
    hook.matrix_inverse = inverse
    hook_group = obj.vertex_groups.new(name=hook.name)
    hook_group.add(indices, 1., 'REPLACE')
    hook.vertex_group = hook_group.name
    hook.center = center
    hook_index, solver_index = list(obj.modifiers).index(hook), list(obj.modifiers).index(mod)
    if hook_index > solver_index:
        obj.modifiers.move(hook_index, solver_index)
    group = obj.vertex_groups[mod.settings.vertex_group_goal if mod.type == 'SOFT_BODY'
                              else mod.settings.vertex_group_mass]
    group.add(indices, 1., 'REPLACE')
    obj['scs_physics_hooks'] = list(obj.get('scs_physics_hooks', [])) + [hook.name]
    obj[core.GROUPS] = list(obj.get(core.GROUPS, [])) + [hook_group.name]
    return control


def jiggle_weights(obj, indices, *, feather=2):
    """Taper a selected region inward; all vertices outside it stay pinned."""
    chosen = set(indices)
    if not chosen or len(chosen) == len(obj.data.vertices):
        raise ValueError('请选择局部软体区域，保留区域外的固定顶点')
    adjacent = [set() for _ in obj.data.vertices]
    for edge in obj.data.edges:
        a, b = edge.vertices
        adjacent[a].add(b)
        adjacent[b].add(a)
    boundary = {i for i in chosen if adjacent[i] - chosen}
    weights = {i: 1. for i in chosen}
    visited = set()
    for depth in range(feather):
        for i in boundary:
            weights[i] = (depth + 1) / (feather + 1)
        visited.update(boundary)
        boundary = {n for i in boundary for n in adjacent[i] if n in chosen and n not in visited}
    return weights


def prepare_jiggle(obj, weights, *, goal=.7, stiffness=.8, damping=5., mass=.3):
    insertion = validate_mesh(obj)
    if not obj.data.polygons:
        raise ValueError('局部软体需要有面的身体网格')
    if not weights or not any(w > 0 for w in weights.values()):
        raise ValueError('软体区域没有有效权重')
    if any(i < 0 or i >= len(obj.data.vertices) or not 0 <= w <= 1 for i, w in weights.items()):
        raise ValueError('无效的软体区域权重')
    if sum(w > 0 for w in weights.values()) == len(obj.data.vertices):
        raise ValueError('保留区域外的固定顶点，不要让整个身体参与抖动')
    token = uuid4().hex[:8]
    group = obj.vertex_groups.new(name='SCS Jiggle Goal ' + token)
    group.add(list(range(len(obj.data.vertices))), 1., 'REPLACE')
    for i, weight in weights.items():
        group.add([i], 1. - weight * (1. - goal), 'REPLACE')
    mod = obj.modifiers.new('SCS Jiggle ' + token, 'SOFT_BODY')
    obj.modifiers.move(list(obj.modifiers).index(mod), insertion)
    settings = mod.settings
    settings.use_goal = settings.use_edges = True
    settings.vertex_group_goal = group.name
    settings.goal_min, settings.goal_max, settings.goal_default = 0., 1., 1.
    settings.goal_spring, settings.goal_friction = .8, damping
    settings.pull = settings.push = stiffness
    settings.damping, settings.mass = damping, mass
    settings.use_stiff_quads = True
    settings.step_min, settings.step_max = 10, 200
    sw.set_cache_range(mod)
    obj[core.KIND], obj[core.SOLVER], obj[core.GROUPS] = 'JIGGLE', mod.name, [group.name]
    return mod
