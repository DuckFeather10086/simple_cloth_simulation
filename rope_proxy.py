# SPDX-License-Identifier: GPL-2.0-or-later
"""Editable wire cylinders bound to rope geometry by native Surface Deform."""
from uuid import uuid4
import math
import bpy
from mathutils import Vector
from . import region_physics as region, mesh_physics as mesh, physics_core as core

SOURCE = 'scs_rope_proxy_source'


def ordered_path(source, indices):
    adjacent = {i: [] for i in indices}
    for edge in source.data.edges:
        a, b = edge.vertices
        if a in adjacent and b in adjacent:
            adjacent[a].append(b)
            adjacent[b].append(a)
    ends = [i for i, neighbors in adjacent.items() if len(neighbors) == 1]
    if len(ends) != 2 or any(len(v) not in {1, 2} for v in adjacent.values()):
        return None
    result, previous, current = [], None, min(ends)
    while current is not None:
        result.append(current)
        following = [i for i in adjacent[current] if i != previous]
        previous, current = current, following[0] if following else None
    return result


def rope_parts(source, coords, selected):
    groups = region.components(source, selected)
    paths = [ordered_path(source, group) if not source.data.polygons else None for group in groups]
    # Small detached end loops belong to their nearest long strand. This keeps
    # a two-strand rope with two end loops at two cages, not four tiny cages.
    lengths = [sum((coords[b] - coords[a]).length for a, b in zip(path, path[1:])) if path else 0.
               for path in paths]
    attached = set()
    for i, group in enumerate(groups):
        if paths[i]:
            continue
        points = [coords[v] for v in group]
        span = Vector(tuple(max(p[a] for p in points) - min(p[a] for p in points) for a in range(3))).length
        center = sum(points, Vector()) / len(points)
        candidates = [(min((center - coords[path[0]]).length, (center - coords[path[-1]]).length), j)
                      for j, path in enumerate(paths) if path and lengths[j] > 4 * span]
        if candidates:
            distance, target = min(candidates)
            if distance <= 1.5 * span:
                groups[target] = groups[target] + group
                attached.add(i)
    return [(group, paths[i]) for i, group in enumerate(groups) if i not in attached]


def resample(points, count):
    lengths = [0.]
    for a, b in zip(points, points[1:]):
        lengths.append(lengths[-1] + (b - a).length)
    if lengths[-1] < 1e-6:
        raise ValueError('绳子没有可用长度')
    result, segment = [], 0
    for index in range(count):
        distance = index * lengths[-1] / (count - 1)
        while segment < len(points) - 2 and lengths[segment + 1] <= distance:
            segment += 1
        fraction = (distance - lengths[segment]) / max(lengths[segment + 1] - lengths[segment], 1e-12)
        result.append(points[segment].lerp(points[segment + 1], fraction))
    return result


def cylinder(coords, indices, path, count, padding, thickness=None):
    if path:
        centers = resample([coords[i] for i in path], count)
    else:
        # Geometric slices, not equal vertex counts: dense cap rings must not
        # become several overlapping proxy cross-sections at the same height.
        axis = region.principal_axis([coords[i] for i in indices])
        projected = [coords[i].dot(axis) for i in indices]
        low, high = min(projected), max(projected)
        bins = [[] for _ in range(count)]
        for index, position in zip(indices, projected):
            bucket = min(count - 1, int((position - low) * count / (high - low)))
            bins[bucket].append(coords[index])
        centers = resample([sum(section, Vector()) / len(section) for section in bins if section], count)
    points = [coords[i] for i in indices]
    segments, fractions = region.nearest_segments(points, centers)
    radii = [padding] * count
    thickness = [0.] * len(points) if thickness is None else thickness
    for point, segment, fraction, surface_radius in zip(points, segments, fractions, thickness):
        closest = centers[segment].lerp(centers[segment + 1], float(fraction))
        radius = ((point - closest).length + surface_radius + padding) / math.cos(math.pi / 8)
        radii[segment] = max(radii[segment], radius)
        radii[segment + 1] = max(radii[segment + 1], radius)
    # Caps extend beyond the input so end vertices lie inside the target shell.
    centers[0] -= (centers[1] - centers[0]).normalized() * radii[0]
    centers[-1] += (centers[-1] - centers[-2]).normalized() * radii[-1]
    sides = 8
    vertices, faces = [], []
    normal = None
    for index, center in enumerate(centers):
        tangent = (centers[min(index + 1, len(centers) - 1)] - centers[max(0, index - 1)]).normalized()
        if normal is None:
            normal = tangent.cross(Vector((0, 0, 1)))
            if normal.length < 1e-6:
                normal = tangent.cross(Vector((0, 1, 0)))
        normal = (normal - tangent * normal.dot(tangent)).normalized()
        side = tangent.cross(normal).normalized()
        for j in range(sides):
            angle = math.tau * j / sides
            vertices.append(center + radii[index] * (math.cos(angle) * normal + math.sin(angle) * side))
    for row in range(len(centers) - 1):
        for j in range(sides):
            a, b = row * sides + j, row * sides + (j + 1) % sides
            c, d = b + sides, a + sides
            faces.extend(((a, b, c), (a, c, d)))
    for row, reverse in ((0, True), (len(centers) - 1, False)):
        cap = len(vertices)
        vertices.append(centers[row])
        for j in range(sides):
            a, b = row * sides + j, row * sides + (j + 1) % sides
            faces.append((cap, b, a) if reverse else (cap, a, b))
    return vertices, faces


def proxies(context, selected=None):
    selected = list(context.selected_objects if selected is None else selected)
    found = {o for o in selected if o.get(SOURCE)}
    found.update(o['scs_anchor_source'] for o in selected
                 if o.get('scs_anchor_source') and o['scs_anchor_source'].get(SOURCE))
    found.update(o for o in context.scene.objects if o.get(SOURCE) in selected)
    return sorted(found, key=lambda o: o.name)


def create(context, source, selected=None, *, count=16, padding=.0005):
    if source is None or source.type != 'MESH' or source.get(SOURCE):
        raise ValueError('请选择原绳子 Mesh')
    if proxies(context, [source]):
        raise ValueError('这根绳子已有线框代理，直接编辑代理或给代理添加物理')
    insertion = mesh.validate_mesh(source)
    if not 3 <= count <= 128 or padding <= 0:
        raise ValueError('代理分段需在 3–128 之间，包裹余量须大于零')
    indices = list(range(len(source.data.vertices))) if selected is None else sorted(set(selected))
    if len(indices) < 3 or any(i < 0 or i >= len(source.data.vertices) for i in indices):
        raise ValueError('请选择至少 3 个绳子顶点')
    made, modifiers, groups = [], [], []
    suffix = list(source.modifiers)[insertion:]
    states = [m.show_viewport for m in suffix]
    try:
        for modifier in suffix:
            modifier.show_viewport = False
        context.view_layer.update()
        evaluated = source.evaluated_get(context.evaluated_depsgraph_get())
        coords = [v.co.copy() for v in evaluated.data.vertices]
        if len(coords) != len(source.data.vertices):
            raise ValueError('原绳子动画改变了顶点数量，请先准备拓扑稳定的 Mesh')
        parts = rope_parts(source, coords, indices)
        radius_attribute = source.data.attributes.get('rope_radius')
        for number, (indices, path) in enumerate(parts, 1):
            thickness = ([radius_attribute.data[i].value for i in indices] if radius_attribute else
                         [0. if source.data.polygons else .0015] * len(indices))
            vertices, faces = cylinder(coords, indices, path, count, padding, thickness)
            data = bpy.data.meshes.new('绳子代理圆柱')
            data.from_pydata(vertices, [], faces)
            data.update()
            proxy = bpy.data.objects.new(f'{source.name} · 线框代理 {number:02d}', data)
            context.collection.objects.link(proxy)
            made.append(proxy)
            proxy.parent = source
            proxy.matrix_world = source.matrix_world.copy()
            proxy.display_type = 'WIRE'
            proxy.show_in_front = True
            proxy.hide_render = True
            proxy[SOURCE] = source
            proxy['scs_rope_proxy_rows'] = count
            proxy['scs_rope_proxy_sides'] = 8
            group = source.vertex_groups.new(name='SCS Rope Proxy ' + uuid4().hex[:8])
            groups.append(group)
            group.add(indices, 1., 'REPLACE')
            modifier = source.modifiers.new('绳子代理 · Surface Deform ' + str(number), 'SURFACE_DEFORM')
            modifiers.append(modifier)
            modifier.target = proxy
            modifier.vertex_group = group.name
            modifier.use_sparse_bind = True
            source.modifiers.move(list(source.modifiers).index(modifier), insertion + number - 1)
            context.view_layer.update()
            with context.temp_override(object=source, active_object=source):
                bpy.ops.object.surfacedeform_bind(modifier=modifier.name)
            if not modifier.is_bound:
                raise RuntimeError('Surface Deform 绑定失败：' + proxy.name)
            proxy['scs_rope_proxy_modifier'] = modifier.name
            proxy['scs_rope_proxy_group'] = group.name
        for obj in context.selected_objects:
            obj.select_set(False)
        for proxy in made:
            proxy.select_set(True)
        context.view_layer.objects.active = made[0]
        return made
    except Exception:
        for modifier in modifiers:
            source.modifiers.remove(modifier)
        for group in groups:
            source.vertex_groups.remove(group)
        for proxy in made:
            data = proxy.data
            bpy.data.objects.remove(proxy, do_unlink=True)
            bpy.data.meshes.remove(data)
        raise
    finally:
        for modifier, enabled in zip(suffix, states):
            modifier.show_viewport = enabled
        context.view_layer.update()


def add_physics(context, targets, *, preset='HEAD_ONLY'):
    if not targets:
        raise ValueError('先生成线框代理，或选中已有线框代理')
    if any(core.solver_for(proxy) for proxy in targets):
        raise ValueError('选中的代理已有物理，请使用原模拟')
    for proxy in targets:
        mesh.validate_mesh(proxy)
    for proxy in targets:
        sides, rows = proxy['scs_rope_proxy_sides'], proxy['scs_rope_proxy_rows']
        # Two rings hold the end position and its cross-section orientation.
        head = list(range(2 * sides)) + [rows * sides]
        tail = list(range((rows - 2) * sides, rows * sides)) + [rows * sides + 1]
        anchors = {'HEAD_ONLY': head, 'TAIL_ONLY': tail, 'BOTH_ENDS': head + tail}[preset]
        solver = mesh.prepare_chain(proxy, anchors, automatic_hooks=False)
        solver.settings.use_stiff_quads = True
        # Long-range bending springs diverge on millimeter-wide tapered shells;
        # triangulated surface edges already supply the local spring network.
        solver.settings.bend = 0.
        for points in ([head, tail] if preset == 'BOTH_ENDS' else [anchors]):
            mesh.add_hook(proxy, points)
    return targets


def remove(context, targets):
    if not targets:
        raise ValueError('请选择线框代理或对应的原绳子')
    simulated = [o for o in targets if core.solver_for(o)]
    if simulated:
        core.remove_simulations(simulated)
    for proxy in targets:
        source = proxy[SOURCE]
        modifier = source.modifiers.get(proxy['scs_rope_proxy_modifier'])
        if modifier:
            source.modifiers.remove(modifier)
        group = source.vertex_groups.get(proxy['scs_rope_proxy_group'])
        if group:
            source.vertex_groups.remove(group)
        data = proxy.data
        bpy.data.objects.remove(proxy, do_unlink=True)
        if data.users == 0:
            bpy.data.meshes.remove(data)
