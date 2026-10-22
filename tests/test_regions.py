"""Real evaluated-Mesh tests for local chain regions, including rigid rings."""
from pathlib import Path
import math
import sys
import unittest
import bpy
import bmesh
from mathutils import Vector

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import simple_cloth_simulation as addon
from simple_cloth_simulation import region_physics as region, physics_core as core, mesh_physics as mesh
addon.register()
OUT = Path(__file__).resolve().parents[1] / 'out/unified-physics/regions'
OUT.mkdir(parents=True, exist_ok=True)


def points(obj):
    bpy.context.view_layer.update()
    return [v.co.copy() for v in obj.evaluated_get(bpy.context.evaluated_depsgraph_get()).data.vertices]


def play():
    bpy.context.scene.frame_set(0)
    for frame in range(1, 21):
        bpy.context.scene.frame_set(frame)
        bpy.context.view_layer.update()


def ribbon():
    vertices = [(i * .06, (j - 1.5) * .025, 1.) for i in range(17) for j in range(4)]
    faces = [(i * 4 + j, (i + 1) * 4 + j, (i + 1) * 4 + j + 1, i * 4 + j + 1)
             for i in range(16) for j in range(3)]
    data = bpy.data.meshes.new('Ribbon')
    data.from_pydata(vertices, [], faces)
    obj = bpy.data.objects.new('Ribbon', data)
    bpy.context.collection.objects.link(obj)
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    return obj, list(range(4 * 4, len(vertices)))


def rings():
    vertices, faces = [], []
    for number in range(11):
        start = len(vertices)
        for i in range(12):
            theta = math.tau * i / 12
            for j in range(6):
                phi = math.tau * j / 6
                x = (.04 + .007 * math.cos(phi)) * math.cos(theta) * 1.35
                y = (.04 + .007 * math.cos(phi)) * math.sin(theta)
                z = .007 * math.sin(phi)
                if number % 2:
                    y, z = -z, y
                vertices.append((number * .07 + x, y, 1 + z))
        for i in range(12):
            for j in range(6):
                faces.append(tuple(start + a * 6 + b for a, b in [
                    (i, j), ((i + 1) % 12, j), ((i + 1) % 12, (j + 1) % 6), (i, (j + 1) % 6)]))
    data = bpy.data.meshes.new('Existing Rings')
    data.from_pydata(vertices, [], faces)
    obj = bpy.data.objects.new('Existing Rings', data)
    bpy.context.collection.objects.link(obj)
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    return obj, list(range(3 * 72, len(vertices)))


class RegionTests(unittest.TestCase):
    def setUp(self):
        if bpy.context.object and bpy.context.object.mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')
        for obj in list(bpy.data.objects):
            bpy.data.objects.remove(obj, do_unlink=True)
        bpy.context.scene.frame_start, bpy.context.scene.frame_end = 1, 20
        bpy.context.scene.frame_set(1)

    def assertClose(self, a, b, epsilon=2e-5):
        self.assertLess((Vector(a) - Vector(b)).length, epsilon, (a, b))

    def test_segment_projection_matches_exhaustive_search_across_batches(self):
        centers = [Vector(p) for p in [(0, 0, 0), (.2, .1, 0), (.3, -.2, .2), (.9, .1, .1)]]
        samples = [Vector((i / 9000, .13 * math.sin(i), .05)) for i in range(9001)]
        indices, fractions = region.nearest_segments(samples, centers)
        for point, actual_index, actual_fraction in zip(samples, indices, fractions):
            candidates = []
            for index, (a, b) in enumerate(zip(centers, centers[1:])):
                direction = b - a
                t = max(0., min(1., (point - a).dot(direction) / direction.length_squared))
                candidates.append(((point - a - t * direction).length_squared, index, t))
            _, expected_index, expected_fraction = min(candidates)
            self.assertEqual(actual_index, expected_index)
            self.assertAlmostEqual(actual_fraction, expected_fraction, places=5)

    def test_continuous_region_keeps_unselected_vertices_and_shape_keys(self):
        source, selected = ribbon()
        source.shape_key_add(name='Basis')
        key = source.shape_key_add(name='Original Shape')
        key.data[40].co.y += .02
        key.value = .2
        original = [v.co.copy() for v in source.data.vertices]
        before = points(source)
        proxy = region.create_region(bpy.context, source, selected, count=9)
        self.assertTrue(all((a - b).length < 2e-5 for a, b in zip(points(source), before)))
        play()
        after = points(source)
        for i in range(16):
            self.assertClose(after[i], before[i])
        movement = max((after[i] - before[i]).length for i in selected)
        print('CONTINUOUS_REGION_MOVEMENT', movement, flush=True)
        self.assertGreater(movement, .01)
        self.assertTrue(all((a.co - b).length < 1e-8 for a, b in zip(source.data.vertices, original)))
        self.assertEqual(list(source.data.shape_keys.key_blocks.keys()), ['Basis', 'Original Shape'])
        core.remove_simulations([proxy])
        self.assertFalse(source.modifiers)
        self.assertFalse(any(a.name.startswith('scs_') for a in source.data.attributes))
        self.assertEqual(len(bpy.data.objects), 1)
        self.assertTrue(all((a - b).length < 2e-5 for a, b in zip(points(source), before)))

    def test_ring_region_keeps_each_ring_rigid_and_other_rings_unchanged(self):
        source, selected = rings()
        before = points(source)
        proxy = region.create_region(bpy.context, source, selected, structure='RINGS')
        with self.assertRaises(ValueError):
            core.add_colliders([source])
        play()
        after = points(source)
        for i in range(3 * 72):
            self.assertClose(after[i], before[i])
        distortion = 0.
        for number in range(3, 11):
            base = number * 72
            for i in range(1, 72):
                distortion = max(distortion, abs((after[base + i] - after[base]).length - (before[base + i] - before[base]).length))
        movement = max((after[i] - before[i]).length for i in selected)
        print('RING_REGION_MOVEMENT_AND_DISTORTION', movement, distortion, flush=True)
        self.assertGreater(movement, .01)
        self.assertLess(distortion, 2e-5)
        self.assertEqual(len(proxy.data.vertices), 8)

    def test_upstream_skinning_not_doubled_and_hook_is_absolute(self):
        bpy.ops.object.armature_add()
        rig = bpy.context.object
        bone = rig.pose.bones[0]
        bone.location = (0, 0, 0)
        bone.keyframe_insert('location', frame=1)
        bone.location = (.2, 0, 0)
        bone.keyframe_insert('location', frame=20)
        bpy.context.scene.frame_set(1)
        source, selected = ribbon()
        weights = source.vertex_groups.new(name=bone.name)
        weights.add(list(range(len(source.data.vertices))), 1., 'REPLACE')
        armature = source.modifiers.new('Original Armature', 'ARMATURE')
        armature.object = rig
        source.location = (.7, -.1, .2)
        before = points(source)
        proxy = region.create_region(bpy.context, source, selected, count=8)
        core.set_enabled([proxy], False)
        bpy.context.scene.frame_set(20)
        for a, b in zip(points(source), before):
            self.assertClose(a, b + Vector((.2, 0, 0)))
        bpy.context.scene.frame_set(1)
        core.set_enabled([proxy], True)
        root_before = (proxy.matrix_world @ points(proxy)[0]).copy()
        mesh.add_hook(proxy, [0], target=rig, bone=bone.name)
        play()
        root_after = proxy.matrix_world @ points(proxy)[0]
        self.assertClose(root_after, root_before + Vector((.2, 0, 0)))
        for i, p in enumerate(points(source)[:16]):
            self.assertClose(p, before[i] + Vector((.2, 0, 0)))

    def test_baked_region_survives_reload(self):
        source, selected = rings()
        proxy = region.create_region(bpy.context, source, selected, structure='RINGS')
        core.cache_operation(bpy.context, [proxy])
        bpy.context.scene.frame_set(20)
        expected = points(source)
        file = OUT / 'local-chain.blend'
        source_name = source.name
        bpy.ops.wm.save_as_mainfile(filepath=str(file))
        bpy.ops.wm.open_mainfile(filepath=str(file))
        source = bpy.data.objects[source_name]
        for a, b in zip(points(source), expected):
            self.assertClose(a, b)

    def test_invalid_partial_ring_rolls_back(self):
        source, selected = rings()
        objects = set(bpy.data.objects)
        with self.assertRaises(ValueError):
            region.create_region(bpy.context, source, selected[1:], structure='RINGS')
        self.assertEqual(set(bpy.data.objects), objects)
        self.assertFalse(source.modifiers)
        self.assertFalse(any(a.name.startswith('scs_') for a in source.data.attributes))

    def test_disjoint_regions_share_source_and_reject_overlap(self):
        source, _ = rings()
        data = source.data
        first = region.create_region(bpy.context, source, list(range(3 * 72, 7 * 72)), structure='RINGS')
        second = region.create_region(bpy.context, source, list(range(7 * 72, 11 * 72)), structure='RINGS')
        self.assertEqual(source.data, data)
        with self.assertRaises(ValueError):
            region.create_region(bpy.context, source, list(range(3 * 72, 7 * 72)), structure='RINGS')
        core.remove_simulations([first])
        play()
        self.assertTrue(core.solver_for(second))
        self.assertGreater(max((a.co - b).length for a, b in zip(source.data.vertices, points(source))), .01)
        core.remove_simulations([second])
        self.assertFalse(source.modifiers)

    def test_actual_edit_selection_operator_preserves_other_vertices(self):
        source, selected = ribbon()
        settings = bpy.context.scene.scs_physics
        settings.mode, settings.chain_input, settings.chain_structure = 'CHAIN', 'REGION', 'CONTINUOUS'
        settings.nodes = 8
        before = points(source)
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.select_all(action='DESELECT')
        edit = bmesh.from_edit_mesh(source.data)
        for vertex in edit.verts:
            vertex.select = vertex.index in selected
        bmesh.update_edit_mesh(source.data)
        self.assertEqual(bpy.ops.scs.create_simulation(), {'FINISHED'})
        proxy = bpy.context.object
        self.assertEqual(proxy[region.SOURCE], source)
        play()
        for a, b in zip(points(source)[:16], before[:16]):
            self.assertClose(a, b)


result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(RegionTests))
addon.unregister()
if not result.wasSuccessful():
    raise RuntimeError('Region physics tests failed')
print('REGION_TESTS_PASSED', result.testsRun, flush=True)
