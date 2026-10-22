# SPDX-License-Identifier: GPL-2.0-or-later
"""Drive a selected part of an existing Mesh with a separate chain proxy.

The output applies simulation relative to an animated reference, so original
skinning is neither replaced nor applied twice. Rings share one rigid frame;
continuous regions interpolate neighboring proxy frames.
"""
from uuid import uuid4
import bpy
import numpy as np
from mathutils import Matrix, Vector
from . import mesh_physics as mesh
from . import physics_core as core

SOURCE = 'scs_region_source'


def components(obj, selected):
    adjacent = {i: set() for i in selected}
    for edge in obj.data.edges:
        a, b = edge.vertices
        if a in adjacent and b in adjacent:
            adjacent[a].add(b)
            adjacent[b].add(a)
    remaining, groups = set(selected), []
    while remaining:
        pending, group = [min(remaining)], set()
        while pending:
            i = pending.pop()
            if i not in group:
                group.add(i)
                pending.extend(adjacent[i] - group)
        remaining.difference_update(group)
        groups.append(sorted(group))
    return groups


def principal_axis(points):
    coordinates = np.asarray(points, dtype=np.float64)
    centered = coordinates - coordinates.mean(axis=0)
    covariance = centered.T @ centered
    axis = np.zeros(3)
    axis[np.argmax(np.ptp(coordinates, axis=0))] = 1
    for _ in range(24):
        value = covariance @ axis
        length = np.linalg.norm(value)
        if length < 1e-10:
            raise ValueError('选区没有可辨识的长度方向')
        axis = value / length
    return Vector(axis)


def nearest_segments(points, centers):
    """Exact segment projection in batches, without a vertices × nodes array."""
    points = np.asarray(points, dtype=np.float64)
    centers = np.asarray(centers, dtype=np.float64)
    indices = np.zeros(len(points), dtype=np.int32)
    fractions = np.zeros(len(points), dtype=np.float64)
    for start in range(0, len(points), 8192):
        batch = points[start:start + 8192]
        best = np.full(len(batch), np.inf)
        for index, direction in enumerate(np.diff(centers, axis=0)):
            delta = batch - centers[index]
            fraction = np.clip(np.einsum('ij,j->i', delta, direction) / np.dot(direction, direction), 0., 1.)
            offset = delta - fraction[:, None] * direction
            distance = np.einsum('ij,ij->i', offset, offset)
            closer = distance < best
            best[closer] = distance[closer]
            indices[start:start + len(batch)][closer] = index
            fractions[start:start + len(batch)][closer] = fraction[closer]
    return indices, fractions


def station_groups(obj, coords, selected, structure, count):
    if structure == 'RINGS':
        selected = set(selected)
        if any((e.vertices[0] in selected) != (e.vertices[1] in selected) for e in obj.data.edges):
            raise ValueError('独立链环模式请选完整链环；局部连续面请用连续网格模式')
        groups = components(obj, selected)
        if len(groups) < 3:
            raise ValueError('请选择至少 3 个独立链环')
    else:
        axis = principal_axis([coords[i] for i in selected])
        order = sorted(selected, key=lambda i: coords[i].dot(axis))
        count = min(count, len(order))
        if count < 3:
            raise ValueError('选区至少需要 3 个顶点')
        groups = [order[i * len(order) // count:(i + 1) * len(order) // count] for i in range(count)]
    centers = [sum((coords[i] for i in group), Vector()) / len(group) for group in groups]
    axis = principal_axis(centers)
    # Start at one end, then follow nearest unused station for mildly curved paths.
    remaining = set(range(len(groups)))
    index = min(remaining, key=lambda i: centers[i].dot(axis))
    order = []
    while remaining:
        order.append(index)
        remaining.remove(index)
        if remaining:
            index = min(remaining, key=lambda i: (centers[i] - centers[index]).length_squared)
    return [groups[i] for i in order], [centers[i] for i in order]


def attr(data, name, kind, values):
    attribute = data.attributes.new(name, kind, 'POINT')
    array = np.asarray(values, dtype=np.int32 if kind == 'INT' else np.float32)
    attribute.data.foreach_set('vector' if kind == 'FLOAT_VECTOR' else 'value', array.ravel())
    return attribute


class Graph:
    def __init__(self, name):
        self.tree = bpy.data.node_groups.new(name, 'GeometryNodeTree')
        self.tree.interface.new_socket(name='Geometry', in_out='INPUT', socket_type='NodeSocketGeometry')
        self.tree.interface.new_socket(name='Geometry', in_out='OUTPUT', socket_type='NodeSocketGeometry')
        self.input = self.node('NodeGroupInput').outputs['Geometry']
        self.output = self.node('NodeGroupOutput')

    def node(self, kind):
        return self.tree.nodes.new(kind)

    def set(self, socket, value):
        if isinstance(value, bpy.types.NodeSocket):
            self.tree.links.new(value, socket)
        else:
            socket.default_value = value

    def named(self, name, kind):
        node = self.node('GeometryNodeInputNamedAttribute')
        node.data_type = kind
        node.inputs['Name'].default_value = name
        return node.outputs['Attribute']

    def vector(self, operation, a, b=None):
        node = self.node('ShaderNodeVectorMath')
        node.operation = operation
        self.set(node.inputs[0], a)
        if b is not None:
            self.set(node.inputs['Scale'] if operation == 'SCALE' else node.inputs[1], b)
        return node.outputs['Value'] if operation in {'DOT_PRODUCT', 'LENGTH', 'DISTANCE'} else node.outputs['Vector']

    def number(self, operation, a, b=0.):
        node = self.node('ShaderNodeMath')
        node.operation = operation
        self.set(node.inputs[0], a)
        self.set(node.inputs[1], b)
        return node.outputs[0]

    def info(self, obj):
        node = self.node('GeometryNodeObjectInfo')
        node.transform_space = 'RELATIVE'
        node.inputs['Object'].default_value = obj
        return node.outputs['Geometry']

    def sample(self, geometry, value, index):
        node = self.node('GeometryNodeSampleIndex')
        node.data_type = 'FLOAT_VECTOR'
        node.domain = 'POINT'
        node.clamp = True
        self.set(node.inputs['Geometry'], geometry)
        self.set(node.inputs['Value'], value)
        self.set(node.inputs['Index'], index)
        return node.outputs['Value']

    def mix_vector(self, a, b, factor):
        return self.vector('ADD', a, self.vector('SCALE', self.vector('SUBTRACT', b, a), factor))

    def switch_vector(self, condition, false, true):
        node = self.node('GeometryNodeSwitch')
        node.input_type = 'VECTOR'
        self.set(node.inputs['Switch'], condition)
        self.set(node.inputs['False'], false)
        self.set(node.inputs['True'], true)
        return node.outputs['Output']

    def finish(self, *, position=None, offset=None):
        node = self.node('GeometryNodeSetPosition')
        self.set(node.inputs['Geometry'], self.input)
        if position is not None:
            self.set(node.inputs['Position'], position)
        if offset is not None:
            self.set(node.inputs['Offset'], offset)
        self.set(self.output.inputs['Geometry'], node.outputs['Geometry'])
        return self.tree


def reference_graph(reference, names):
    graph = Graph('SCS Animated Reference')
    position = graph.node('GeometryNodeInputPosition').outputs['Position']
    group = graph.named(names['station'], 'INT')
    total = graph.node('GeometryNodeAccumulateField')
    total.data_type, total.domain = 'FLOAT_VECTOR', 'POINT'
    graph.set(total.inputs['Value'], position)
    graph.set(total.inputs['Group Index'], group)
    count = graph.node('GeometryNodeAccumulateField')
    count.data_type, count.domain = 'FLOAT', 'POINT'
    count.inputs['Value'].default_value = 1.
    graph.set(count.inputs['Group Index'], group)
    average = graph.vector('SCALE', total.outputs['Total'], graph.number('DIVIDE', 1., count.outputs['Total']))
    result = graph.sample(graph.info(reference), average, graph.named(names['sample'], 'INT'))
    return graph.finish(position=result)


def deformation_graph(proxy, rest, names, *, smooth=False):
    graph = Graph('SCS Local Chain Deformation')
    position = graph.node('GeometryNodeInputPosition').outputs['Position']
    a, b = graph.named(names['a'], 'INT'), graph.named(names['b'], 'INT')
    fraction = graph.named(names['t'], 'FLOAT')
    simulated, reference = graph.info(proxy), graph.info(rest)
    sa, sb = graph.sample(simulated, position, a), graph.sample(simulated, position, b)
    ra, rb = graph.sample(reference, position, a), graph.sample(reference, position, b)
    old_center, new_center = graph.mix_vector(ra, rb, fraction), graph.mix_vector(sa, sb, fraction)
    if smooth:
        prev, following = graph.number('SUBTRACT', a, 1.), graph.number('ADD', b, 1.)
        rp, rn = graph.sample(reference, position, prev), graph.sample(reference, position, following)
        sp, sn = graph.sample(simulated, position, prev), graph.sample(simulated, position, following)
        old_dir = graph.mix_vector(graph.vector('NORMALIZE', graph.vector('SUBTRACT', rb, rp)),
                                   graph.vector('NORMALIZE', graph.vector('SUBTRACT', rn, ra)), fraction)
        new_dir = graph.mix_vector(graph.vector('NORMALIZE', graph.vector('SUBTRACT', sb, sp)),
                                   graph.vector('NORMALIZE', graph.vector('SUBTRACT', sn, sa)), fraction)
        old_dir, new_dir = graph.vector('NORMALIZE', old_dir), graph.vector('NORMALIZE', new_dir)
    else:
        old_dir = graph.vector('NORMALIZE', graph.vector('SUBTRACT', rb, ra))
        new_dir = graph.vector('NORMALIZE', graph.vector('SUBTRACT', sb, sa))
    cross = graph.vector('CROSS_PRODUCT', old_dir, new_dir)
    fallback = graph.vector('CROSS_PRODUCT', old_dir, (0, 0, 1))
    fallback = graph.switch_vector(graph.number('LESS_THAN', graph.vector('LENGTH', fallback), 1e-6),
                                   fallback, graph.vector('CROSS_PRODUCT', old_dir, (0, 1, 0)))
    axis = graph.switch_vector(graph.number('LESS_THAN', graph.vector('LENGTH', cross), 1e-6), cross, fallback)
    dot = graph.vector('DOT_PRODUCT', old_dir, new_dir)
    angle = graph.number('ARCCOSINE', graph.number('MAXIMUM', -1., graph.number('MINIMUM', 1., dot)))
    rotate = graph.node('ShaderNodeVectorRotate')
    rotate.rotation_type = 'AXIS_ANGLE'
    graph.set(rotate.inputs['Vector'], graph.vector('SUBTRACT', position, old_center))
    graph.set(rotate.inputs['Axis'], axis)
    graph.set(rotate.inputs['Angle'], angle)
    transformed = graph.vector('ADD', new_center, rotate.outputs['Vector'])
    delta = graph.vector('SUBTRACT', transformed, position)
    return graph.finish(offset=graph.vector('SCALE', delta, graph.named(names['mask'], 'FLOAT')))


def create_region(context, source, selected, *, structure='CONTINUOUS', count=16, feather=2,
                  hook_preset=None):
    insertion = mesh.validate_mesh(source)
    if any(m.type == 'COLLISION' for m in source.modifiers):
        raise ValueError('局部模拟网格不能同时作为自身碰撞物；请使用独立碰撞网格')
    if not source.data.polygons:
        raise ValueError('选区模式需要有面的原网格；绳中心线请用独立代理模式')
    selected = sorted(set(selected))
    if len(selected) < 3 or any(i < 0 or i >= len(source.data.vertices) for i in selected):
        raise ValueError('在原 Mesh 的编辑模式选择需要模拟的部分')
    for existing in context.scene.objects:
        if existing.get(SOURCE) == source:
            mask = source.data.attributes[existing['scs_region_attributes'][-1]]
            if any(mask.data[i].value > 0 for i in selected):
                raise ValueError('选区已有链条模拟；请先移除该区域的模拟')
    token = uuid4().hex[:8]
    names = {key: f'scs_{token}_{key}' for key in ('station', 'a', 'b', 't', 'mask', 'sample')}
    original_mesh = source.data
    original_modifiers = set(source.modifiers)
    objects_before, groups_before = set(bpy.data.objects), set(bpy.data.node_groups)
    meshes_before = set(bpy.data.meshes)
    sharing = [o for o in bpy.data.objects if o.type == 'MESH' and o.data == source.data and o != source]
    own_references = [o for o in sharing if o.get('scs_region_reference_for') == source
                      or o.get('scs_body_collider_reference') == source]
    copied = any(o not in own_references for o in sharing)
    try:
        if copied:
            source.data = source.data.copy()
            for reference in own_references:
                reference.data = source.data
        reference = source.copy()
        reference.name = source.name + ' · Animation Reference'
        reference.data = source.data
        reference['scs_region_reference_for'] = source
        context.collection.objects.link(reference)
        for mod in list(reference.modifiers)[insertion:]:
            reference.modifiers.remove(mod)
        for constraint in list(reference.constraints):
            reference.constraints.remove(constraint)
        follow = reference.constraints.new('COPY_TRANSFORMS')
        follow.target = source
        reference.hide_render = True
        reference.hide_set(True)
        context.view_layer.update()
        evaluated = reference.evaluated_get(context.evaluated_depsgraph_get())
        coords = [v.co.copy() for v in evaluated.data.vertices]
        if len(coords) != len(source.data.vertices):
            raise ValueError('原动画改变了顶点数量，无法对应选区')
        sections, centers = station_groups(source, coords, selected, structure, count)
        if any((b - a).length < 1e-6 for a, b in zip(centers, centers[1:])):
            raise ValueError('代理节点重叠，请减少节点数或调整选区')
        size = len(coords)
        station, indices_a, indices_b = [-1] * size, [0] * size, [1] * size
        fractions, masks = [0.] * size, [0.] * size
        for number, section in enumerate(sections):
            for i in section:
                station[i] = number
                if structure == 'RINGS':
                    indices_a[i] = min(number, len(centers) - 2)
                    indices_b[i] = indices_a[i] + 1
                    fractions[i] = float(number == len(centers) - 1)
                    masks[i] = 1.
        if structure == 'CONTINUOUS':
            weights = (mesh.jiggle_weights(source, selected, feather=feather)
                       if len(selected) < size else {i: 1. for i in selected})
            nearest, projected = nearest_segments([coords[i] for i in selected], centers)
            for i, segment, fraction in zip(selected, nearest, projected):
                indices_a[i], fractions[i] = int(segment), float(fraction)
                indices_b[i], masks[i] = indices_a[i] + 1, weights[i]
        for key, kind, values in [('station', 'INT', station), ('a', 'INT', indices_a),
                                  ('b', 'INT', indices_b), ('t', 'FLOAT', fractions), ('mask', 'FLOAT', masks)]:
            attr(source.data, names[key], kind, values)
        data = bpy.data.meshes.new('SCS Region Path')
        data.from_pydata(centers, [(i, i + 1) for i in range(len(centers) - 1)], [])
        attr(data, names['sample'], 'INT', [section[0] for section in sections])
        tree = reference_graph(reference, names)
        rest = bpy.data.objects.new(source.name + ' · Rest Path', data)
        context.collection.objects.link(rest)
        rest.matrix_world = source.matrix_world
        rest.parent = source
        rest.matrix_parent_inverse = source.matrix_world.inverted()
        rest.hide_render = True
        rest.hide_set(True)
        ref_mod = rest.modifiers.new('SCS Reference', 'NODES')
        ref_mod.node_group = tree
        proxy = bpy.data.objects.new(source.name + ' · Chain Region', data.copy())
        context.collection.objects.link(proxy)
        proxy.matrix_world = source.matrix_world
        proxy.parent = source
        proxy.matrix_parent_inverse = source.matrix_world.inverted()
        proxy.show_in_front = True
        proxy.hide_render = True
        # Soft Body requires original mesh data, so reference positions enter
        # through native per-node Hooks rather than Geometry Nodes before it.
        anchors = mesh.preset_anchors(proxy, hook_preset or 'HEAD_ONLY')
        mesh.prepare_chain(proxy, anchors, automatic_hooks=False)
        helpers = {'reference': reference, 'rest': rest}
        reference_hooks = {}
        for i in range(len(centers)):
            group = rest.vertex_groups.new(name=f'Station {i}')
            group.add([i], 1., 'REPLACE')
            control = bpy.data.objects.new(f'SCS Reference Node {i}', None)
            context.collection.objects.link(control)
            follow = control.constraints.new('COPY_LOCATION')
            follow.target, follow.subtarget = rest, group.name
            control.hide_render = True
            control.hide_set(True)
            helpers[f'node_{i}'] = control
        # Resolve all animated reference positions once, then bind all nodes.
        context.view_layer.update()
        for i in range(len(centers)):
            control = helpers[f'node_{i}']
            hook = proxy.modifiers.new(f'SCS Reference Hook {i}', 'HOOK')
            hook.object = control
            hook.vertex_indices_set([i])
            hook.matrix_inverse = control.matrix_world.inverted() @ proxy.matrix_world
            proxy.modifiers.move(list(proxy.modifiers).index(hook), list(proxy.modifiers).index(core.solver_for(proxy)))
            reference_hooks[str(i)] = hook.name
        output_tree = deformation_graph(proxy, rest, names, smooth=structure == 'CONTINUOUS')
        output = source.modifiers.new('SCS Chain Region ' + token, 'NODES')
        output.node_group = output_tree
        source.modifiers.move(list(source.modifiers).index(output), insertion)
        proxy[SOURCE] = source
        proxy['scs_region_modifier'] = output.name
        proxy['scs_region_attributes'] = [names[key] for key in ('station', 'a', 'b', 't', 'mask')]
        proxy['scs_region_helpers'] = helpers
        proxy['scs_region_reference_hooks'] = reference_hooks
        proxy['scs_region_trees'] = [tree.name, output_tree.name]
        proxy['scs_region_structure'] = structure
        if hook_preset:
            for index in anchors:
                control = mesh.add_hook(proxy, [index])
                # The preset handle follows its original animated station;
                # users can offset it or rebind the node to a chosen target.
                from . import skeletal_weaver as sw
                sw.set_parent_keep_world(control, helpers[f'node_{index}'], 'OBJECT', '', control.matrix_world.copy())
        for obj in context.selected_objects:
            obj.select_set(False)
        proxy.select_set(True)
        context.view_layer.objects.active = proxy
        context.view_layer.update()
        return proxy
    except Exception:
        # Restore source data and additions if Blender binding/node creation fails.
        for mod in list(source.modifiers):
            if mod not in original_modifiers:
                source.modifiers.remove(mod)
        for obj in set(bpy.data.objects) - objects_before:
            bpy.data.objects.remove(obj, do_unlink=True)
        for name in names.values():
            attribute = source.data.attributes.get(name)
            if attribute:
                source.data.attributes.remove(attribute)
        if copied:
            source.data = original_mesh
            for reference in own_references:
                reference.data = original_mesh
        for data in set(bpy.data.meshes) - meshes_before:
            if data.users == 0:
                bpy.data.meshes.remove(data)
        for tree in set(bpy.data.node_groups) - groups_before:
            if tree.users == 0:
                bpy.data.node_groups.remove(tree)
        raise
