# SPDX-License-Identifier: GPL-2.0-or-later
"""Shared cache, ownership and collider operations for Simple Cloth simulations."""
import bpy
from . import skeletal_weaver as sw

KIND = 'scs_physics_kind'
SOLVER = 'scs_physics_solver'
GROUPS = 'scs_physics_groups'
COLLIDER = 'scs_physics_collider'


def solver_for(obj):
    if obj.type != 'MESH':
        return None
    name = obj.get(SOLVER)
    if name:
        mod = obj.modifiers.get(name)
        return mod if mod and mod.type in {'CLOTH', 'SOFT_BODY'} else None
    if sw.is_guide_object(obj):
        return next((m for m in obj.modifiers if m.type in {'CLOTH', 'SOFT_BODY'}), None)
    return None


def simulations(context, *, scene_scope=False):
    """Resolve selected bones/meshes/controllers, never adopt unrelated physics."""
    if scene_scope:
        return [o for o in context.scene.objects if solver_for(o)]
    selected = set(context.selected_objects)
    found = {o for o in selected if solver_for(o)}
    found.update(o['scs_chain_source'] for o in selected
                 if o.get('scs_chain_source') and solver_for(o['scs_chain_source']))
    for obj in context.scene.objects:
        if not solver_for(obj):
            continue
        if any(m.type == 'HOOK' and m.object in selected for m in obj.modifiers):
            found.add(obj)
        rig = obj.get(sw.SOURCE_KEY)
        if rig not in selected:
            continue
        if context.mode == 'POSE' and rig == context.object:
            names = {b.name for b in context.selected_pose_bones or []}
            if names.intersection(obj.get(sw.BONES_KEY, [])):
                found.add(obj)
        else:
            found.add(obj)
    return sorted(found, key=lambda obj: obj.name)


def require_unbaked(objects):
    baked = [obj.name for obj in objects if solver_for(obj).point_cache.is_baked]
    if baked:
        raise ValueError('先清除缓存：' + ', '.join(baked))


def cache_operation(context, objects, *, free=False):
    if not objects:
        raise ValueError('未选中本插件模拟；可选骨骼、代理网格或 Hook 控制器')
    scene = context.scene
    if not free and scene.frame_start >= scene.frame_end:
        raise ValueError('结束帧必须大于起始帧')
    if not free:
        require_unbaked(objects)
        disabled = [obj.name for obj in objects if not solver_for(obj).show_viewport]
        if disabled:
            raise ValueError('先启用模拟：' + ', '.join(disabled))
    previous = scene.frame_current
    try:
        for obj in objects:
            mod = solver_for(obj)
            cache = mod.point_cache
            with context.temp_override(object=obj, active_object=obj, point_cache=cache):
                if free:
                    if cache.is_baked:
                        bpy.ops.ptcache.free_bake()
                else:
                    cache.frame_start, cache.frame_end = scene.frame_start, scene.frame_end
                    scene.frame_set(scene.frame_start)
                    result = bpy.ops.ptcache.bake(bake=True)
                    if result != {'FINISHED'} or not cache.is_baked:
                        raise RuntimeError('缓存烘焙失败：' + obj.name)
        if free:
            # Reset unbaked sequential playback caches as well.
            scene.frame_set(scene.frame_start - 1)
            for obj in objects:
                obj.update_tag()
    finally:
        scene.frame_set(scene.frame_start if free else previous)


def set_enabled(objects, enabled):
    if not objects:
        raise ValueError('未选中本插件模拟')
    for obj in objects:
        mod = solver_for(obj)
        mod.show_viewport = mod.show_render = enabled
        # Disabling a guide must bypass its feedback as well, otherwise a posed
        # bone still aims at the static guide instead of its original animation.
        for rig in bpy.data.objects:
            if rig.type != 'ARMATURE':
                continue
            for bone in rig.pose.bones:
                for constraint in bone.constraints:
                    if sw.is_weaver_damped_track_constraint(bone, constraint) and constraint.target == obj:
                        constraint.mute = not enabled


def add_colliders(objects):
    meshes = [o for o in objects if o.type == 'MESH']
    if not meshes:
        raise ValueError('在物体模式选择身体或其他碰撞网格')
    if any((solver_for(o) and o.get(KIND) != 'JIGGLE') or sw.is_guide_object(o) for o in meshes):
        raise ValueError('请选择碰撞物，模拟网格不能同时作为本工具的碰撞物')
    for obj in meshes:
        if not any(m.type == 'COLLISION' for m in obj.modifiers):
            mod = obj.modifiers.new('SCS Collision', 'COLLISION')
            obj[COLLIDER] = mod.name
            obj.collision.thickness_outer = .003
            obj.collision.damping = .2
    return len(meshes)


def remove_colliders(objects):
    for obj in objects:
        name = obj.get(COLLIDER)
        if name:
            mod = obj.modifiers.get(name)
            if mod and mod.type == 'COLLISION':
                obj.modifiers.remove(mod)
            del obj[COLLIDER]


def remove_simulations(objects):
    """Remove only owned additions. An input Mesh is never a disposable guide."""
    if not objects:
        raise ValueError('未选中本插件模拟')
    require_unbaked(objects)
    for obj in objects:
        for child in list(bpy.data.objects):
            if child.get('scs_chain_source') == obj:
                bpy.data.objects.remove(child, do_unlink=True)
        if sw.is_guide_object(obj):
            for rig in bpy.data.objects:
                if rig.type != 'ARMATURE':
                    continue
                for bone in rig.pose.bones:
                    removed = False
                    for c in list(bone.constraints):
                        if sw.is_weaver_damped_track_constraint(bone, c) and c.target == obj:
                            bone.constraints.remove(c)
                            removed = True
                    if removed and not any(sw.is_weaver_damped_track_constraint(bone, c) for c in bone.constraints):
                        sw.restore_pre_constraint_pose(bone)
            # Respect manual constraints/children using the guide.
            referenced = obj.children or any(
                getattr(c, 'target', None) == obj
                for source in bpy.data.objects
                for c in list(source.constraints) + ([c for b in source.pose.bones for c in b.constraints]
                                                     if source.type == 'ARMATURE' else []))
            if not referenced:
                sw.delete_guide_objects([obj])
            else:
                solver_for(obj).show_viewport = False
                solver_for(obj).show_render = False
        else:
            hooks = []
            for mod in list(obj.modifiers):
                if mod.type == 'HOOK' and mod.name in obj.get('scs_physics_hooks', []):
                    if mod.object:
                        hooks.append(mod.object)
                    obj.modifiers.remove(mod)
            obj.modifiers.remove(solver_for(obj))
            for name in obj.get(GROUPS, []):
                group = obj.vertex_groups.get(name)
                if group:
                    obj.vertex_groups.remove(group)
            for key in (KIND, SOLVER, GROUPS, 'scs_physics_hooks'):
                if key in obj:
                    del obj[key]
            for hook in hooks:
                if hook.get('scs_anchor_source') == obj and not hook.children and hook.users <= 1:
                    bpy.data.objects.remove(hook, do_unlink=True)
