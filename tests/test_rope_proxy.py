"""Actual Surface Deform binding, independent strands, optional cage physics."""
from pathlib import Path
import math
import sys
import unittest
import bpy
from mathutils import Vector

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import simple_cloth_simulation as addon
from simple_cloth_simulation import rope_proxy as rope, physics_core as core
addon.register()
OUT = Path(__file__).resolve().parents[1] / 'out/unified-physics/rope-proxy'
OUT.mkdir(parents=True, exist_ok=True)


def source_rope():
    vertices, edges = [], []
    for y in (0., .08):
        offset = len(vertices)
        vertices.extend((i * .02, y + .03 * math.sin(i * .08), 1.) for i in range(51))
        edges.extend((offset + i, offset + i + 1) for i in range(50))
        offset = len(vertices)
        vertices.extend((.01 * math.cos(i * math.tau / 32) - .01, y, 1. + .01 * math.sin(i * math.tau / 32))
                        for i in range(32))
        edges.extend((offset + i, offset + (i + 1) % 32) for i in range(32))
    data = bpy.data.meshes.new('Two ropes with end loops')
    data.from_pydata(vertices, edges, [])
    obj = bpy.data.objects.new('Two ropes with end loops', data)
    bpy.context.collection.objects.link(obj)
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    return obj


def points(obj):
    bpy.context.view_layer.update()
    return [v.co.copy() for v in obj.evaluated_get(bpy.context.evaluated_depsgraph_get()).data.vertices]


class RopeProxyTests(unittest.TestCase):
    def setUp(self):
        if bpy.context.object and bpy.context.object.mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')
        for obj in list(bpy.data.objects):
            bpy.data.objects.remove(obj, do_unlink=True)
        bpy.context.scene.frame_set(1)
        bpy.context.scene.frame_start, bpy.context.scene.frame_end = 1, 20

    def test_two_wire_cylinders_bind_without_physics_and_deform_independently(self):
        source = source_rope()
        before = points(source)
        targets = rope.create(bpy.context, source, count=12)
        self.assertEqual(len(targets), 2)
        self.assertTrue(all(o.display_type == 'WIRE' and o.hide_render for o in targets))
        self.assertTrue(all(len(o.data.vertices) == 98 for o in targets))
        # Detached end loops may widen the end, never the entire long strand.
        cross_section = [v.co for v in targets[0].data.vertices[5 * 8:6 * 8]]
        center = sum(cross_section, Vector()) / 8
        self.assertLess(max((p-center).length for p in cross_section), .003)
        self.assertTrue(all(not o.modifiers for o in targets))
        self.assertTrue(all(m.type == 'SURFACE_DEFORM' and m.is_bound for m in source.modifiers))
        self.assertLess(max((a - b).length for a, b in zip(points(source), before)), 1e-5)
        for v in targets[0].data.vertices:
            v.co.z += .15 * v.co.x
        targets[0].data.update()
        after = points(source)
        self.assertGreater(max((a - b).length for a, b in zip(after[:83], before[:83])), .1)
        self.assertLess(max((a - b).length for a, b in zip(after[83:], before[83:])), 1e-5)
        self.assertTrue(all((v.co - p).length < 1e-8 for v, p in zip(source.data.vertices, before)))
        rope.remove(bpy.context, targets)
        self.assertFalse(source.modifiers)
        self.assertFalse(source.vertex_groups)
        self.assertLess(max((a - b).length for a, b in zip(points(source), before)), 1e-5)

    def test_physics_only_on_cages_and_source_follows(self):
        source = source_rope()
        targets = rope.create(bpy.context, source, count=10)
        before = points(source)
        rope.add_physics(bpy.context, targets)
        self.assertTrue(all(core.solver_for(o) for o in targets))
        self.assertIsNone(core.solver_for(source))
        self.assertEqual(len([o for o in bpy.data.objects if o.get('scs_anchor_source')]), 2)
        bpy.context.scene.frame_set(0)
        for frame in range(1, 21):
            bpy.context.scene.frame_set(frame)
            for target in targets:
                self.assertTrue(all(math.isfinite(value) for p in points(target) for value in p), f'nonfinite proxy at {frame}')
        actual = points(source)
        self.assertLess(actual[50].z, before[50].z - .01)
        self.assertLess(actual[133].z, before[133].z - .01)
        self.assertLess(abs(actual[0].z - before[0].z), .02)
        bpy.context.scene.frame_set(1)
        bpy.ops.wm.save_as_mainfile(filepath=str(OUT / 'two-ropes.blend'))
        bpy.ops.wm.open_mainfile(filepath=str(OUT / 'two-ropes.blend'))
        source = bpy.data.objects['Two ropes with end loops']
        self.assertTrue(all(m.is_bound for m in source.modifiers))
        targets = rope.proxies(bpy.context, [source])
        self.assertEqual(len(targets), 2)
        rope.remove(bpy.context, targets)
        self.assertFalse(source.modifiers)

    def test_surface_rope_and_original_geometry_nodes_are_preserved(self):
        bpy.ops.mesh.primitive_cylinder_add(vertices=16, radius=.02, depth=1.)
        source = bpy.context.object
        before = points(source)
        graph = bpy.data.node_groups.new('Existing rope appearance', 'GeometryNodeTree')
        graph.interface.new_socket(name='Geometry', in_out='INPUT', socket_type='NodeSocketGeometry')
        graph.interface.new_socket(name='Geometry', in_out='OUTPUT', socket_type='NodeSocketGeometry')
        graph.links.new(graph.nodes.new('NodeGroupInput').outputs['Geometry'], graph.nodes.new('NodeGroupOutput').inputs['Geometry'])
        source.modifiers.new('Appearance', 'NODES').node_group = graph
        targets = rope.create(bpy.context, source, count=8)
        self.assertEqual(source.modifiers[-1].node_group, graph)
        self.assertLess(max((a - b).length for a, b in zip(points(source), before)), 1e-5)
        self.assertEqual(len(targets), 1)

    def test_two_button_workflow(self):
        source = source_rope()
        self.assertEqual(bpy.ops.scs.rope_proxy(action='CREATE'), {'FINISHED'})
        targets = rope.proxies(bpy.context)
        self.assertEqual(len(targets), 2)
        self.assertEqual(bpy.ops.scs.rope_proxy(action='PHYSICS'), {'FINISHED'})
        self.assertEqual(set(core.simulations(bpy.context)), set(targets))
        self.assertEqual(bpy.ops.scs.manage(action='REMOVE'), {'FINISHED'})
        self.assertTrue(all(m.is_bound for m in source.modifiers))
        self.assertEqual(bpy.ops.scs.rope_proxy(action='REMOVE'), {'FINISHED'})
        self.assertFalse(source.modifiers)


result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(RopeProxyTests))
addon.unregister()
if not result.wasSuccessful():
    raise RuntimeError('Rope proxy tests failed')
print('ROPE_PROXY_TESTS_PASSED', result.testsRun, flush=True)
