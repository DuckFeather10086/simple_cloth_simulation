"""Blender integration: real motion, explicit Hooks, link assembly and persistence."""
from pathlib import Path
import sys
import unittest
import bpy
from mathutils import Vector

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import simple_cloth_simulation as addon
from simple_cloth_simulation import physics_core as core, mesh_physics as mesh, chain_layout as chain
addon.register()
OUT = Path(__file__).resolve().parents[1] / 'out/unified-physics/verification'
OUT.mkdir(parents=True, exist_ok=True)


def evaluated_points(obj):
    bpy.context.view_layer.update()
    evaluated = obj.evaluated_get(bpy.context.evaluated_depsgraph_get())
    return [evaluated.matrix_world @ v.co for v in evaluated.data.vertices]


def simulate(end=20):
    scene = bpy.context.scene
    scene.frame_set(0)
    for frame in range(1, end + 1):
        scene.frame_set(frame)
        bpy.context.view_layer.update()


def make_chain(height=1.):
    obj = chain.create_proxy(bpy.context, 12, .55)
    obj.location.z = height
    bpy.context.view_layer.update()
    mesh.prepare_chain(obj, [0])
    return obj


def make_template():
    bpy.ops.mesh.primitive_torus_add(major_radius=.025, minor_radius=.005, major_segments=12, minor_segments=6)
    obj = bpy.context.object
    obj.name = 'Template'
    obj.scale.y = 1.5
    return obj


class PhysicsTests(unittest.TestCase):
    def setUp(self):
        if bpy.context.object and bpy.context.object.mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')
        for obj in list(bpy.data.objects):
            bpy.data.objects.remove(obj, do_unlink=True)
        scene = bpy.context.scene
        scene.frame_start, scene.frame_end = 1, 20
        scene.frame_set(1)
        scene.scs_physics.scene_scope = False

    def assertClose(self, a, b, tolerance=1e-4):
        self.assertLess((Vector(a) - Vector(b)).length, tolerance, (a, b))

    def test_gravity_and_arbitrary_hook_reassignment(self):
        obj = make_chain()
        original = [v.co.copy() for v in obj.data.vertices]
        target = bpy.data.objects.new('User Hook', None)
        bpy.context.collection.objects.link(target)
        target.location = (2, 3, 4)  # preserve offset, no snapping to target origin
        mesh.add_hook(obj, [5], target=target)
        mesh.add_hook(obj, [0], target=target)  # replace automatic first Hook
        target.location.y += .2
        simulate()
        actual = evaluated_points(obj)
        self.assertAlmostEqual(actual[0].y, .2, places=4)
        self.assertAlmostEqual(actual[5].y, .2, places=4)
        self.assertLess(actual[-1].z, .99)
        self.assertTrue(all((v.co - old).length < 1e-8 for v, old in zip(obj.data.vertices, original)))
        core.remove_simulations([obj])
        self.assertIn(target.name, bpy.data.objects)
        self.assertFalse(obj.modifiers)
        self.assertFalse(obj.vertex_groups)

    def test_bone_hook_follows_animated_bone_without_jump(self):
        obj = make_chain()
        bpy.ops.object.armature_add(location=(.4, .2, .7))
        rig = bpy.context.object
        bone = rig.pose.bones[0]
        before = evaluated_points(obj)[4]
        mesh.add_hook(obj, [4], target=rig, bone=bone.name)
        self.assertClose(evaluated_points(obj)[4], before)
        bone.location.x = .2
        simulate()
        self.assertClose(evaluated_points(obj)[4], before + Vector((.2, 0, 0)))

    def test_numbered_nodes_and_links_follow_evaluated_chain(self):
        obj = make_chain()
        self.assertEqual(chain.mark_nodes(obj), 12)
        marker = next(o for o in bpy.data.objects if o.get(chain.NODE) == 5)
        template = make_template()
        data = template.data
        count = chain.build_links(obj, template)
        self.assertEqual(count, 11)
        chain.build_links(obj, template)  # update, no duplicates
        self.assertEqual(len([o for o in bpy.data.objects if o.get(chain.ROLE) == 'LINK']), 11)
        simulate()
        points = evaluated_points(obj)
        self.assertClose(marker.evaluated_get(bpy.context.evaluated_depsgraph_get()).matrix_world.translation, points[5])
        drivers = sorted((o for o in bpy.data.objects if o.get(chain.ROLE) == 'LINK_DRIVER'), key=lambda o: o.name)
        for index, driver in enumerate(drivers):
            matrix = driver.evaluated_get(bpy.context.evaluated_depsgraph_get()).matrix_world
            self.assertClose(matrix.translation, (points[index] + points[index + 1]) / 2)
            self.assertClose(matrix.to_3x3().col[1].normalized(), (points[index + 1] - points[index]).normalized())
        self.assertTrue(all(o.data == data for o in bpy.data.objects if o.get(chain.ROLE) == 'LINK'))
        bpy.ops.object.select_all(action='DESELECT')
        marker.select_set(True)
        bpy.context.view_layer.objects.active = marker
        self.assertEqual(chain.selected_nodes(bpy.context, obj), [5])
        self.assertEqual(core.simulations(bpy.context), [obj])
        with self.assertRaises(ValueError):
            mesh.add_hook(obj, [2], target=marker)
        core.remove_simulations([obj])
        self.assertIn(template.name, bpy.data.objects)
        self.assertFalse(any(o.get(chain.SOURCE) for o in bpy.data.objects))

    def test_bake_reload_free_and_scope(self):
        first = make_chain()
        chain.mark_nodes(first)
        template = make_template()
        chain.build_links(first, template)
        second = make_chain()
        core.cache_operation(bpy.context, [first])
        self.assertTrue(core.solver_for(first).point_cache.is_baked)
        self.assertFalse(core.solver_for(second).point_cache.is_baked)
        with self.assertRaises(ValueError):
            mesh.add_hook(first, [3])
        file = OUT / 'cache.blend'
        name = first.name
        bpy.ops.wm.save_as_mainfile(filepath=str(file))
        bpy.ops.wm.open_mainfile(filepath=str(file))
        first = bpy.data.objects[name]
        self.assertTrue(core.solver_for(first).point_cache.is_baked)
        bpy.context.scene.frame_set(20)
        self.assertLess(evaluated_points(first)[-1].z, .99)
        points = evaluated_points(first)
        driver = bpy.data.objects['SCS Link Driver 001']
        self.assertClose(driver.evaluated_get(bpy.context.evaluated_depsgraph_get()).matrix_world.translation,
                         (points[0] + points[1]) / 2)
        core.cache_operation(bpy.context, [first], free=True)
        self.assertFalse(core.solver_for(first).point_cache.is_baked)

    def test_local_jiggle_pins_outside_region_and_keeps_shape_keys(self):
        bpy.ops.mesh.primitive_grid_add(x_subdivisions=9, y_subdivisions=9, size=.4, location=(0, 0, 1))
        obj = bpy.context.object
        obj.shape_key_add(name='Basis')
        shape = obj.shape_key_add(name='Existing Morph')
        shape.data[0].co.x += .005
        shape.value = .4
        original = [v.co.copy() for v in obj.data.vertices]
        original_shapes = [[v.co.copy() for v in k.data] for k in obj.data.shape_keys.key_blocks]
        selected = [v.index for v in obj.data.vertices if abs(v.co.x) < .13 and abs(v.co.y) < .13]
        weights = mesh.jiggle_weights(obj, selected)
        mod = mesh.prepare_jiggle(obj, weights, goal=.5)
        before = evaluated_points(obj)
        simulate()
        after = evaluated_points(obj)
        for i in range(len(before)):
            if i not in selected:
                self.assertClose(after[i], before[i])
        movement = max((after[i] - before[i]).length for i in selected)
        print('JIGGLE_REGION_MOVEMENT', movement, flush=True)
        self.assertGreater(movement, .0001)
        self.assertTrue(all((v.co - p).length < 1e-8 for v, p in zip(obj.data.vertices, original)))
        self.assertTrue(all((v.co - p).length < 1e-8 for key, previous in zip(obj.data.shape_keys.key_blocks, original_shapes) for v, p in zip(key.data, previous)))
        core.set_enabled([obj], False)
        self.assertFalse(mod.show_viewport)
        core.remove_simulations([obj])
        self.assertEqual(list(obj.data.shape_keys.key_blocks.keys()), ['Basis', 'Existing Morph'])

    def test_jiggle_follows_upstream_animation_and_modifier_order(self):
        bpy.ops.object.armature_add()
        rig = bpy.context.object
        bone = rig.pose.bones[0]
        bone.location = (0, 0, 0)
        bone.keyframe_insert('location', frame=1)
        bone.location = (.1, 0, 0)
        bone.keyframe_insert('location', frame=20)
        bpy.context.scene.frame_set(1)
        bpy.ops.mesh.primitive_grid_add(x_subdivisions=9, y_subdivisions=9, size=.4, location=(0, 0, 1))
        obj = bpy.context.object
        group = obj.vertex_groups.new(name=bone.name)
        group.add(list(range(len(obj.data.vertices))), 1., 'REPLACE')
        armature = obj.modifiers.new('Original Skinning', 'ARMATURE')
        armature.object = rig
        subdivision = obj.modifiers.new('Original Surface', 'SUBSURF')
        subdivision.show_viewport = False
        selected = [v.index for v in obj.data.vertices if abs(v.co.x) < .13 and abs(v.co.y) < .13]
        mesh.prepare_jiggle(obj, mesh.jiggle_weights(obj, selected))
        self.assertEqual([m.type for m in obj.modifiers], ['ARMATURE', 'SOFT_BODY', 'SUBSURF'])
        before = evaluated_points(obj)
        simulate()
        after = evaluated_points(obj)
        for i in range(len(before)):
            if i not in selected:
                self.assertClose(after[i], before[i] + Vector((.1, 0, 0)))
        self.assertEqual(len(obj.vertex_groups[bone.name].name), len(bone.name))
        self.assertTrue(all(any(g.group == group.index and abs(g.weight - 1) < 1e-6 for g in v.groups) for v in obj.data.vertices))

    def test_ui_chain_flow_and_edit_hook(self):
        settings = bpy.context.scene.scs_physics
        settings.mode = 'CHAIN'
        settings.chain_input = 'PATH'
        settings.anchor_mode = 'FIRST'
        self.assertEqual(bpy.ops.scs.create_chain_proxy(), {'FINISHED'})
        obj = bpy.context.object
        self.assertEqual(bpy.ops.scs.create_simulation(), {'FINISHED'})
        for v in obj.data.vertices:
            v.select = v.index == 5
        bpy.ops.object.mode_set(mode='EDIT')
        self.assertEqual(bpy.ops.scs.chain_action(action='HOOK'), {'FINISHED'})
        self.assertEqual(bpy.context.mode, 'EDIT_MESH')
        bpy.ops.object.mode_set(mode='OBJECT')
        self.assertEqual(bpy.ops.scs.manage(action='BAKE'), {'FINISHED'})
        self.assertEqual(bpy.ops.scs.manage(action='FREE'), {'FINISHED'})
        self.assertEqual(bpy.ops.scs.manage(action='REMOVE'), {'FINISHED'})
        self.assertIn(obj.name, bpy.data.objects)

    def test_pose_cloth_uses_shared_cache_and_enable_controls(self):
        bpy.ops.object.armature_add()
        rig = bpy.context.object
        bpy.ops.object.mode_set(mode='POSE')
        rig.pose.bones[0].select = True
        bpy.context.scene.scs_physics.mode = 'CLOTH'
        self.assertEqual(bpy.ops.scs.create_simulation(), {'FINISHED'})
        objects = core.simulations(bpy.context)
        self.assertEqual(len(objects), 1)
        self.assertEqual(bpy.ops.scs.manage(action='BAKE'), {'FINISHED'})
        self.assertTrue(core.solver_for(objects[0]).point_cache.is_baked)
        self.assertEqual(bpy.ops.scs.manage(action='FREE'), {'FINISHED'})
        core.set_enabled(objects, False)
        self.assertTrue(rig.pose.bones[0].constraints[0].mute)
        core.set_enabled(objects, True)
        self.assertFalse(rig.pose.bones[0].constraints[0].mute)
        core.remove_simulations(objects)
        self.assertFalse(rig.pose.bones[0].constraints)

    def test_invalid_inputs_and_collision_ownership(self):
        obj = chain.create_proxy(bpy.context)
        with self.assertRaises(ValueError):
            mesh.prepare_chain(obj, list(range(len(obj.data.vertices))))
        self.assertFalse(obj.modifiers)
        self.assertFalse(obj.vertex_groups)
        bpy.ops.mesh.primitive_cube_add()
        existing = bpy.context.object
        existing.modifiers.new('User Collision', 'COLLISION')
        core.add_colliders([existing])
        core.remove_colliders([existing])
        self.assertEqual(existing.modifiers[0].name, 'User Collision')
        bpy.ops.mesh.primitive_cube_add()
        owned = bpy.context.object
        core.add_colliders([owned])
        core.remove_colliders([owned])
        self.assertFalse(owned.modifiers)

    def test_actual_collision_changes_chain_motion(self):
        def run(collision):
            obj = make_chain(height=.12)
            if collision:
                bpy.ops.mesh.primitive_plane_add(size=4)
                core.add_colliders([bpy.context.object])
            simulate(20)
            result = evaluated_points(obj)[-1].z
            for o in list(bpy.data.objects):
                bpy.data.objects.remove(o, do_unlink=True)
            bpy.context.scene.frame_set(1)
            return result
        free, collided = run(False), run(True)
        print('CHAIN_COLLISION', free, collided, flush=True)
        self.assertLess(free, -.01)
        self.assertGreater(collided, free + .01)


result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(PhysicsTests))
addon.unregister()
addon.register()
addon.unregister()
if not result.wasSuccessful():
    raise RuntimeError('Unified physics tests failed')
print('PHYSICS_TESTS_PASSED', result.testsRun, flush=True)
