# SPDX-License-Identifier: GPL-2.0-or-later
"""Numbered proxy nodes and template links driven by evaluated soft-body points."""
import math
import bpy
from mathutils import Matrix, Vector
from . import physics_core as core

SOURCE = 'scs_chain_source'
ROLE = 'scs_chain_role'
NODE = 'scs_chain_node'


def create_proxy(context, count=16, length=1.):
    if count < 3 or length <= 0:
        raise ValueError('至少 3 个节点，长度必须大于零')
    mesh = bpy.data.meshes.new('SCS Chain Path')
    mesh.from_pydata([(i * length / (count - 1), 0, 0) for i in range(count)],
                     [(i, i + 1) for i in range(count - 1)], [])
    obj = bpy.data.objects.new('SCS Chain Proxy', mesh)
    context.collection.objects.link(obj)
    obj.matrix_world = context.scene.cursor.matrix
    obj.show_in_front = True
    obj.hide_render = True
    for other in context.selected_objects:
        other.select_set(False)
    obj.select_set(True)
    context.view_layer.objects.active = obj
    return obj


def proxy_from(context):
    obj = context.object
    if obj is None:
        return None
    if obj and obj.get(SOURCE):
        return obj[SOURCE]
    if obj.get('scs_anchor_source'):
        return obj['scs_anchor_source']
    regions = [o for o in context.scene.objects if o.get('scs_region_source') == obj]
    regions += [o for o in context.scene.objects if o.get('scs_rope_proxy_source') == obj]
    if len(regions) == 1:
        return regions[0]
    return obj if obj and obj.type == 'MESH' and core.solver_for(obj) else None


def selected_nodes(context, proxy):
    if context.mode == 'EDIT_MESH' and context.edit_object == proxy:
        proxy.update_from_editmode()
        return [v.index for v in proxy.data.vertices if v.select]
    markers = [o for o in context.selected_objects if o.get(SOURCE) == proxy and o.get(ROLE) == 'NODE']
    if markers:
        return sorted({int(o[NODE]) for o in markers})
    if context.object == proxy:
        return [v.index for v in proxy.data.vertices if v.select]
    return []


def point_group(proxy, indices, label):
    name = 'SCS ' + label
    # Groups are private and reused by regenerated markers/links.
    owned = list(proxy.get(core.GROUPS, []))
    group = proxy.vertex_groups.get(name) if name in owned else None
    if group is None:
        group = proxy.vertex_groups.new(name=name)
        proxy[core.GROUPS] = owned + [group.name]
    else:
        group.remove(list(range(len(proxy.data.vertices))))
    group.add(indices, 1., 'REPLACE')
    return group.name


def clear_layout(proxy, role):
    for obj in list(bpy.data.objects):
        if obj.get(SOURCE) == proxy and obj.get(ROLE) == role:
            bpy.data.objects.remove(obj, do_unlink=True)


def mark_nodes(proxy):
    if not core.solver_for(proxy):
        raise ValueError('先给代理生成模拟')
    clear_layout(proxy, 'NODE')
    for index in range(len(proxy.data.vertices)):
        marker = bpy.data.objects.new(f'节 {index + 1:03d} · {proxy.name}', None)
        bpy.context.collection.objects.link(marker)
        marker[SOURCE], marker[ROLE], marker[NODE] = proxy, 'NODE', index
        marker.empty_display_type = 'SPHERE'
        marker.empty_display_size = .012
        marker.show_in_front = marker.show_name = True
        marker.hide_render = True
        constraint = marker.constraints.new('COPY_LOCATION')
        constraint.target = proxy
        constraint.subtarget = point_group(proxy, [index], f'Node {index}')
    return len(proxy.data.vertices)


def ordered_edges(proxy):
    """Walk one open path, preserving alternation even if edge indices are shuffled."""
    from .mesh_physics import endpoints
    ends = endpoints(proxy)
    adjacent = [set() for _ in proxy.data.vertices]
    for edge in proxy.data.edges:
        a, b = edge.vertices
        adjacent[a].add(b)
        adjacent[b].add(a)
    result = []
    previous, current = None, ends[0]
    while True:
        following = adjacent[current] - {previous}
        if not following:
            return result
        target = next(iter(following))
        result.append((current, target))
        previous, current = current, target


def build_links(proxy, template, *, axis='Y', alternate=True, fit=True, pitch=.7):
    if not core.solver_for(proxy):
        raise ValueError('先给代理生成模拟')
    if not template or template.type != 'MESH' or template == proxy or template.get(SOURCE) == proxy:
        raise ValueError('请选择独立的链节 Mesh 模板')
    if template.constraints or any(m.type in {'ARMATURE', 'CLOTH', 'SOFT_BODY'} for m in template.modifiers):
        raise ValueError('链节模板应为无骨架/物理/约束的静态 Mesh')
    if axis not in {'X', 'Y', 'Z'} or pitch <= 0:
        raise ValueError('无效的链节轴向或节距比例')
    edges = ordered_edges(proxy)
    # Use rest path, so rebuilding appearance at a later frame cannot resize it.
    coords = [proxy.matrix_world @ v.co for v in proxy.data.vertices]
    axis_index = 'XYZ'.index(axis)
    extent = max(v[axis_index] for v in template.bound_box) - min(v[axis_index] for v in template.bound_box)
    if extent <= 1e-8 or min(abs(v) for v in template.scale) < 1e-8 or any((coords[b] - coords[a]).length < 1e-8 for a, b in edges):
        raise ValueError('链节模板或代理存在零长度')
    # Center template geometry at its bounds center without touching its Mesh.
    center = sum((Vector(v) for v in template.bound_box), Vector()) / 8
    clear_layout(proxy, 'LINK')
    clear_layout(proxy, 'LINK_DRIVER')
    for number, (a, b) in enumerate(edges):
        direction = coords[b] - coords[a]
        driver = bpy.data.objects.new(f'SCS Link Driver {number + 1:03d}', None)
        bpy.context.collection.objects.link(driver)
        driver[SOURCE], driver[ROLE] = proxy, 'LINK_DRIVER'
        driver.empty_display_size = .008
        driver.hide_render = True
        driver.rotation_mode = 'QUATERNION'
        up = 'Y' if axis == 'Z' else 'Z'
        rotation = direction.to_track_quat(axis, up).to_matrix().to_4x4()
        if alternate and number % 2:
            rotation @= Matrix.Rotation(math.pi / 2, 4, axis)
        driver.matrix_world = Matrix.Translation((coords[a] + coords[b]) / 2) @ rotation
        location = driver.constraints.new('COPY_LOCATION')
        location.target = proxy
        location.subtarget = point_group(proxy, [a, b], f'Segment {a} {b}')
        track = driver.constraints.new('DAMPED_TRACK')
        track.target = proxy
        track.subtarget = point_group(proxy, [b], f'Node {b}')
        track.track_axis = 'TRACK_' + axis
        link = template.copy()
        link.animation_data_clear()
        link.name = f'链节 {number + 1:03d} · {proxy.name}'
        bpy.context.collection.objects.link(link)
        link[SOURCE], link[ROLE] = proxy, 'LINK'
        link.parent, link.parent_type, link.parent_bone = driver, 'OBJECT', ''
        link.matrix_parent_inverse = Matrix.Identity(4)
        link.rotation_mode = 'QUATERNION'
        link.rotation_quaternion = (1, 0, 0, 0)
        scale = template.scale.copy()
        if fit:
            scale *= direction.length / (extent * abs(scale[axis_index]) * pitch)
        link.scale = scale
        link.location = -Vector((center[i] * scale[i] for i in range(3)))
        link.hide_viewport = link.hide_render = False
        link.hide_set(False)
    return len(edges)
