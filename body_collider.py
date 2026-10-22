# SPDX-License-Identifier: GPL-2.0-or-later
"""Fixed-topology, low-poly body colliders sampled before body physics.

Decimation happens once. Barycentric bindings then follow the body's skinning
and shape keys through an independent upstream reference, without feeding any
cloth/chain/jiggle result back into its own collider.
"""
import bpy
from mathutils import Vector
from mathutils.bvhtree import BVHTree
from .region_physics import Graph, attr
from . import physics_core as core

SOURCE = 'scs_body_collider_source'
REFERENCE = 'scs_body_collider_reference'
DEFORM = {'ARMATURE', 'HOOK', 'LATTICE', 'MESH_DEFORM', 'SURFACE_DEFORM',
          'CORRECTIVE_SMOOTH', 'SMOOTH', 'CAST', 'DISPLACE', 'SIMPLE_DEFORM'}
SUFFIX = {'SUBSURF', 'MULTIRES', 'SOLIDIFY', 'BEVEL', 'NODES', 'SKIN', 'WEIGHTED_NORMAL',
          'NORMAL_EDIT', 'COLLISION', 'CLOTH', 'SOFT_BODY', 'MASK', 'DECIMATE', 'TRIANGULATE'}


def barycentric(point, a, b, c):
    u, v, p = b - a, c - a, point - a
    uu, uv, vv = u.dot(u), u.dot(v), v.dot(v)
    denominator = uu * vv - uv * uv
    if abs(denominator) < 1e-20:
        return (1., 0., 0.)
    y = (vv * p.dot(u) - uv * p.dot(v)) / denominator
    z = (uu * p.dot(v) - uv * p.dot(u)) / denominator
    return 1. - y - z, y, z


def binding_graph(reference):
    graph = Graph('SCS Body Surface Follow')
    geometry = graph.info(reference)
    position = graph.node('GeometryNodeInputPosition').outputs['Position']
    normal = graph.node('GeometryNodeInputNormal').outputs['Normal']
    positions, normals = [], []
    for axis in range(3):
        index = graph.named(f'scs_bind_i{axis}', 'INT')
        weight = graph.named(f'scs_bind_w{axis}', 'FLOAT')
        positions.append(graph.vector('SCALE', graph.sample(geometry, position, index), weight))
        normals.append(graph.vector('SCALE', graph.sample(geometry, normal, index), weight))
    center = graph.vector('ADD', graph.vector('ADD', positions[0], positions[1]), positions[2])
    direction = graph.vector('NORMALIZE', graph.vector('ADD', graph.vector('ADD', normals[0], normals[1]), normals[2]))
    target = graph.vector('ADD', center, graph.vector('SCALE', direction, graph.named('scs_bind_margin', 'FLOAT')))
    delta = graph.vector('SUBTRACT', target, graph.named('scs_bind_rest', 'FLOAT_VECTOR'))
    return graph.finish(offset=delta)


def create_collider(context, source, *, target_faces=2500, margin=.003):
    if source is None or source.type != 'MESH' or not source.data.polygons:
        raise ValueError('请选择已绑定骨架的身体 Mesh')
    if source.mode != 'OBJECT':
        raise ValueError('请返回物体模式')
    if source.get(SOURCE) or source.get(REFERENCE):
        raise ValueError('请选择原身体，不是碰撞体或参考物体')
    if any(o.get(SOURCE) == source for o in context.scene.objects):
        raise ValueError('此身体已有低模碰撞体；修改参数重建前先移除旧碰撞体')
    if target_faces < 4 or margin < 0:
        raise ValueError('面数必须至少为 4，外扩不能为负')
    # Capture only topology-preserving upstream animation. Presentation, masks
    # and physics are never dependencies of the collision geometry.
    prefix = []
    suffix_started = False
    for mod in source.modifiers:
        if mod.type in SUFFIX:
            suffix_started = True
        elif mod.type not in DEFORM or suffix_started:
            raise ValueError('请整理身体修改器：骨架 / 变形应位于遮罩、外观和物理之前：' + mod.name)
        else:
            prefix.append(mod)
    if not any(m.type == 'ARMATURE' and m.object and m.show_viewport for m in prefix):
        raise ValueError('身体需要启用的 Armature 修改器；静态障碍物请用“添加碰撞物”')
    objects_before, meshes_before = set(bpy.data.objects), set(bpy.data.meshes)
    trees_before = set(bpy.data.node_groups)
    try:
        reference = source.copy()
        reference.data = source.data
        reference.name = source.name + ' · Collider Animation'
        for key in list(reference.keys()):
            if key.startswith(('scs_', 'skeletal_weaver_')):
                del reference[key]
        reference[REFERENCE] = source
        context.collection.objects.link(reference)
        for mod in list(reference.modifiers)[len(prefix):]:
            reference.modifiers.remove(mod)
        for constraint in list(reference.constraints):
            reference.constraints.remove(constraint)
        reference.constraints.new('COPY_TRANSFORMS').target = source
        reference.hide_render = True
        reference.hide_set(True)
        context.view_layer.update()
        depsgraph = context.evaluated_depsgraph_get()
        evaluated = reference.evaluated_get(depsgraph)
        reference_mesh = evaluated.data
        reference_mesh.calc_loop_triangles()
        coords = [v.co.copy() for v in reference_mesh.vertices]
        normals = [v.normal.copy() for v in reference_mesh.vertices]
        triangles = [tuple(t.vertices) for t in reference_mesh.loop_triangles]
        if len(coords) != len(source.data.vertices) or not triangles:
            raise ValueError('身体动画前后顶点无法对应，请提供拓扑稳定的身体')
        matrix = source.matrix_world.copy()
        if any(abs(s - 1.) > 1e-4 for s in matrix.to_scale()):
            raise ValueError('请先应用身体及父级缩放，再生成低模碰撞体')
        # Snapshot and simplify once; never run a changing Decimate in simulation.
        data = bpy.data.meshes.new_from_object(evaluated, preserve_all_data_layers=True, depsgraph=depsgraph)
        collider = reference.copy()
        collider.data = data
        collider.name = source.name + ' · Low-poly Collision'
        collider.modifiers.clear()
        collider.constraints.clear()
        collider.animation_data_clear()
        collider.parent = None
        for key in list(collider.keys()):
            if key.startswith(('scs_', 'skeletal_weaver_')):
                del collider[key]
        context.collection.objects.link(collider)
        collider.hide_set(False)
        collider.matrix_world = matrix
        # Snapshot static body masks (for example the unused DAZ head), then
        # bind the remaining low-poly surface to the full, stable reference.
        for original in source.modifiers:
            if original.type == 'MASK' and original.show_viewport:
                if original.mode != 'VERTEX_GROUP':
                    raise ValueError('骨架选择型 Mask 请先转换为顶点组 Mask：' + original.name)
                mask = collider.modifiers.new(original.name, 'MASK')
                mask.vertex_group = original.vertex_group
                mask.threshold = original.threshold
                mask.invert_vertex_group = original.invert_vertex_group
        decimate = collider.modifiers.new('Temporary Decimation', 'DECIMATE')
        decimate.ratio = min(1., target_faces / len(triangles))
        decimate.use_collapse_triangulate = True
        context.view_layer.update()
        simplified = bpy.data.meshes.new_from_object(collider.evaluated_get(depsgraph), depsgraph=depsgraph)
        collider.modifiers.clear()
        collider.data = simplified
        bpy.data.meshes.remove(data)
        simplified.name = source.name + ' · Collision Mesh'
        simplified.materials.clear()
        # Drop copied deform groups / attributes: this mesh is bound explicitly.
        collider.vertex_groups.clear()
        for attribute in list(simplified.attributes):
            if not attribute.is_required:
                simplified.attributes.remove(attribute)
        tree = BVHTree.FromPolygons(coords, triangles, all_triangles=True)
        indices, weights, rest = [], [], []
        for vertex in simplified.vertices:
            nearest, _, triangle, _ = tree.find_nearest(vertex.co)
            if nearest is None:
                raise ValueError('无法将低模顶点绑定到身体表面')
            ids = triangles[triangle]
            weight = barycentric(nearest, *(coords[i] for i in ids))
            normal = sum((normals[i] * w for i, w in zip(ids, weight)), Vector()).normalized()
            point = nearest + normal * margin
            vertex.co = point
            indices.append(ids)
            weights.append(weight)
            rest.append(point)
        for axis in range(3):
            attr(simplified, f'scs_bind_i{axis}', 'INT', [value[axis] for value in indices])
            attr(simplified, f'scs_bind_w{axis}', 'FLOAT', [value[axis] for value in weights])
        attr(simplified, 'scs_bind_rest', 'FLOAT_VECTOR', rest)
        attr(simplified, 'scs_bind_margin', 'FLOAT', [margin] * len(rest))
        simplified.update()
        collider.constraints.new('COPY_TRANSFORMS').target = source
        follow = collider.modifiers.new('SCS Animated Body Surface', 'NODES')
        follow.node_group = binding_graph(reference)
        core.add_colliders([collider])
        collider[SOURCE], collider[REFERENCE] = source, reference
        collider['scs_body_bind_vertices'] = len(coords)
        collider['scs_body_bind_frame'] = context.scene.frame_current
        collider['scs_body_margin'] = margin
        collider['scs_body_tree'] = follow.node_group.name
        collider.display_type = 'WIRE'
        collider.hide_render = True
        collider.color = (.08, .6, .8, 1.)
        for obj in context.selected_objects:
            obj.select_set(False)
        collider.select_set(True)
        context.view_layer.objects.active = collider
        context.view_layer.update()
        return collider
    except Exception:
        for obj in set(bpy.data.objects) - objects_before:
            bpy.data.objects.remove(obj, do_unlink=True)
        for tree in set(bpy.data.node_groups) - trees_before:
            if tree.users == 0:
                bpy.data.node_groups.remove(tree)
        for data in set(bpy.data.meshes) - meshes_before:
            if data.users == 0:
                bpy.data.meshes.remove(data)
        raise


def remove_colliders(objects):
    objects = set(objects)
    owned = [o for o in bpy.data.objects if o.get(SOURCE) and (o in objects or o[SOURCE] in objects)]
    if not owned:
        raise ValueError('请选择本工具生成的低模碰撞体或对应身体')
    for collider in owned:
        reference = collider[REFERENCE]
        tree = bpy.data.node_groups.get(collider['scs_body_tree'])
        for obj in (collider, reference):
            data = obj.data
            bpy.data.objects.remove(obj, do_unlink=True)
            if data.users == 0:
                bpy.data.meshes.remove(data)
        if tree and tree.users == 0:
            bpy.data.node_groups.remove(tree)
