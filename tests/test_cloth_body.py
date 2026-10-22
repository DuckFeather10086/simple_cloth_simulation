"""Evaluated animation, collision response and saved-file checks for cloth/body."""
from pathlib import Path
import math
import sys
import unittest
import bpy
from mathutils import Vector

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import simple_cloth_simulation as addon
from simple_cloth_simulation import body_collider as body, cloth_physics as cloth
from simple_cloth_simulation import physics_core as core, chain_layout as chain, mesh_physics as mesh
from simple_cloth_simulation import region_physics as region, skeletal_weaver as sw

OUT = Path(__file__).resolve().parents[1] / 'out/unified-physics/cloth-body'
OUT.mkdir(parents=True, exist_ok=True)


def reset():
    if bpy.context.object and bpy.context.object.mode != 'OBJECT':
        bpy.ops.object.mode_set(mode='OBJECT')
    for obj in list(bpy.data.objects):
        bpy.data.objects.remove(obj, do_unlink=True)
    bpy.context.scene.frame_start, bpy.context.scene.frame_end = 1, 35
    bpy.context.scene.frame_set(1)


def points(obj):
    bpy.context.view_layer.update()
    evaluated = obj.evaluated_get(bpy.context.evaluated_depsgraph_get())
    return [evaluated.matrix_world @ v.co for v in evaluated.data.vertices]


def play(end=35):
    bpy.context.scene.frame_set(0)
    for frame in range(1, end + 1):
        bpy.context.scene.frame_set(frame)
        bpy.context.view_layer.update()


def body_fixture():
    bpy.ops.object.armature_add()
    rig = bpy.context.object
    rig.name = 'Body Rig'
    bpy.ops.object.mode_set(mode='EDIT')
    first = rig.data.edit_bones[0]
    first.name, first.head, first.tail = 'root', (0, 0, .3), (0, 0, .6)
    second = rig.data.edit_bones.new('joint')
    second.head, second.tail, second.parent, second.use_connect = first.tail, (0, 0, .9), first, True
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.mesh.primitive_uv_sphere_add(segments=48, ring_count=24, radius=.3, location=(0, 0, .6))
    source = bpy.context.object
    source.name = 'Skinned Body'
    group_a, group_b = source.vertex_groups.new(name='root'), source.vertex_groups.new(name='joint')
    for v in source.data.vertices:
        weight = max(0, min(1, (v.co.z + .08) / .16))
        group_a.add([v.index], 1-weight, 'REPLACE')
        group_b.add([v.index], weight, 'REPLACE')
    source.shape_key_add(name='Basis')
    key = source.shape_key_add(name='Breath')
    for v in key.data:
        v.co.x *= 1.1
    armature = source.modifiers.new('Original Skinning', 'ARMATURE')
    armature.object = rig
    return source, rig


def sheet_fixture():
    count = 17
    verts = [(-.5 + i/16, -.5 + j/16, 1.08) for i in range(count) for j in range(count)]
    faces = [(i*count+j, (i+1)*count+j, (i+1)*count+j+1, i*count+j+1)
             for i in range(count-1) for j in range(count-1)]
    data = bpy.data.meshes.new('Cloth Sheet')
    data.from_pydata(verts, [], faces)
    obj = bpy.data.objects.new('Cloth Sheet', data)
    bpy.context.collection.objects.link(obj)
    cloth.prepare_cloth(obj, {i: 1. for i in range(count)})
    return obj


def hair_rig():
    bpy.ops.object.armature_add()
    rig = bpy.context.object
    bpy.ops.object.mode_set(mode='EDIT')
    root = rig.data.edit_bones[0]
    root.name, root.head, root.tail = 'head', (0, 0, 1.3), (0, 0, 1.45)
    for col in range(3):
        parent = root
        for row in range(6):
            bone = rig.data.edit_bones.new(f'Hair {col}.{row}')
            bone.head = ((col-1)*.12, 0, 1.3-row*.12)
            bone.tail = ((col-1)*.12, 0, 1.3-(row+1)*.12)
            bone.parent, bone.use_connect = parent, row > 0
            parent = bone
    bpy.ops.object.mode_set(mode='POSE')
    for bone in rig.pose.bones:
        bone.select = bone.name != 'head'
    return rig


def hair_body_fixture(collide=True):
    source, rig = body_fixture()
    source.data.shape_keys.key_blocks['Breath'].value=0
    bpy.ops.object.select_all(action='DESELECT')
    rig.select_set(True)
    bpy.context.view_layer.objects.active=rig
    bpy.ops.object.mode_set(mode='EDIT')
    for col in range(5):
        parent=rig.data.edit_bones['joint']
        for row in range(8):
            bone=rig.data.edit_bones.new(f'Hair {col}.{row}')
            bone.head=((col-2)*.09,-.32,1.05-row*.09)
            bone.tail=((col-2)*.09,-.32,1.05-(row+1)*.09)
            bone.parent,bone.use_connect=parent,row>0
            parent=bone
    bpy.ops.object.mode_set(mode='OBJECT')
    collider=body.create_collider(bpy.context,source,target_faces=480,margin=.006) if collide else None
    bpy.ops.object.select_all(action='DESELECT')
    rig.select_set(True)
    bpy.context.view_layer.objects.active=rig
    bpy.ops.object.mode_set(mode='POSE')
    for bone in rig.pose.bones:
        bone.select=bone.name.startswith('Hair ')
    bpy.context.scene.scs_physics.mode='CLOTH'
    bpy.context.scene.skeletal_weaver_props.cloth_structure='HAIR'
    bpy.context.scene.skeletal_weaver_props.ribbon_width=.04
    assert bpy.ops.scs.create_simulation()=={'FINISHED'}
    guides=core.simulations(bpy.context)
    root=rig.pose.bones['root']
    root.location=(0,0,0)
    root.keyframe_insert('location',frame=1)
    root.location=(.12,0,.15)
    root.keyframe_insert('location',frame=20)
    root.location=(0,0,0)
    root.keyframe_insert('location',frame=35)
    bpy.context.scene.frame_set(1)
    return source, rig, collider, guides


class ClothBodyTests(unittest.TestCase):
    def setUp(self):
        reset()

    def test_body_follows_skin_shape_transform_and_reload(self):
        source, rig = body_fixture()
        original = [v.co.copy() for v in source.data.vertices]
        collider = body.create_collider(bpy.context, source, target_faces=240, margin=.004)
        self.assertLessEqual(len(collider.data.polygons), 250)
        self.assertEqual([m.type for m in collider.modifiers], ['NODES', 'COLLISION'])
        initial = points(collider)
        rig.pose.bones['joint'].rotation_mode = 'XYZ'
        rig.pose.bones['joint'].rotation_euler.x = .7
        source.data.shape_keys.key_blocks['Breath'].value = .8
        rig.location.x = .13
        current = points(collider)
        self.assertGreater(max((a-b).length for a,b in zip(initial,current)), .1)
        bpy.context.view_layer.update()
        ref = collider[body.REFERENCE].evaluated_get(bpy.context.evaluated_depsgraph_get())
        for i, actual in enumerate(current):
            ids = [collider.data.attributes[f'scs_bind_i{axis}'].data[i].value for axis in range(3)]
            weights = [collider.data.attributes[f'scs_bind_w{axis}'].data[i].value for axis in range(3)]
            center = sum((ref.data.vertices[idx].co * w for idx,w in zip(ids,weights)),Vector())
            normal = sum((ref.data.vertices[idx].normal * w for idx,w in zip(ids,weights)),Vector()).normalized()
            self.assertLess((actual - ref.matrix_world @ (center+.004*normal)).length, 1e-5)
        self.assertEqual([m.type for m in source.modifiers], ['ARMATURE'])
        self.assertTrue(all((a-v.co).length == 0 for a,v in zip(original,source.data.vertices)))
        name = collider.name
        bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'body-follow.blend'))
        bpy.ops.wm.open_mainfile(filepath=str(OUT/'body-follow.blend'))
        collider = bpy.data.objects[name]
        self.assertLess(max((a-b).length for a,b in zip(current,points(collider))), 1e-5)
        source = collider[body.SOURCE]
        body.remove_colliders([source])
        self.assertIn(source.name,bpy.data.objects)
        self.assertFalse(any(o.get(body.REFERENCE) for o in bpy.data.objects))

    def test_body_ignores_source_physics_and_rolls_back_failure(self):
        source, rig = body_fixture()
        selected = [v.index for v in source.data.vertices if v.co.x > .1]
        mesh.prepare_jiggle(source, mesh.jiggle_weights(source,selected))
        collider = body.create_collider(bpy.context,source,target_faces=120)
        self.assertEqual([m.type for m in collider[body.REFERENCE].modifiers], ['ARMATURE'])
        body.remove_colliders([collider])
        source.scale.x = 2
        before = set(bpy.data.objects)
        with self.assertRaises(ValueError):
            body.create_collider(bpy.context,source)
        self.assertEqual(set(bpy.data.objects),before)

    def test_cloth_reacts_to_animated_lowpoly_body_and_bakes(self):
        def run(collide):
            source, rig = body_fixture()
            collider = body.create_collider(bpy.context,source,target_faces=480,margin=.006) if collide else None
            root = rig.pose.bones['root']
            root.location.x = 0
            root.keyframe_insert('location',frame=1)
            root.location.x = .07
            root.keyframe_insert('location',frame=35)
            sheet = sheet_fixture()
            core.cache_operation(bpy.context,[sheet])
            bpy.context.scene.frame_set(35)
            result = points(sheet)[8*17+8].z
            if collide:
                expected = points(sheet)
                bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'cloth-body.blend'))
                bpy.ops.wm.open_mainfile(filepath=str(OUT/'cloth-body.blend'))
                sheet = bpy.data.objects['Cloth Sheet']
                self.assertTrue(core.solver_for(sheet).point_cache.is_baked)
                self.assertLess(max((a-b).length for a,b in zip(expected,points(sheet))),1e-5)
                core.cache_operation(bpy.context,[sheet],free=True)
                core.remove_simulations([sheet])
                self.assertFalse(sheet.modifiers)
            reset()
            return result
        free, collided = run(False), run(True)
        print('CLOTH_BODY_COLLISION',free,collided,flush=True)
        self.assertGreater(collided-free,.15)

    def test_hair_separate_strands_and_woven_fabric(self):
        rig = hair_rig()
        bpy.context.scene.scs_physics.mode = 'CLOTH'
        props = bpy.context.scene.skeletal_weaver_props
        props.cloth_structure = 'HAIR'
        self.assertEqual(bpy.ops.scs.create_simulation(),{'FINISHED'})
        guides = core.simulations(bpy.context)
        self.assertEqual(len(guides),3)
        self.assertTrue(all(core.solver_for(o).settings.bending_stiffness == 3 for o in guides))
        root = rig.pose.bones['head']
        root.location.x = 0
        root.keyframe_insert('location',frame=1)
        root.location.x = .2
        root.keyframe_insert('location',frame=16)
        bpy.context.scene.frame_set(1)
        initial = points(guides[0])
        play(16)
        changed = points(guides[0])
        self.assertGreater(max((a-b).length for a,b in zip(initial,changed)),.05)
        core.cache_operation(bpy.context,guides,free=True)
        props.cloth_structure = 'FABRIC'
        self.assertEqual(bpy.ops.scs.create_simulation(),{'FINISHED'})
        guides = core.simulations(bpy.context)
        self.assertEqual(len(guides),1)
        self.assertEqual(core.solver_for(guides[0]).settings.bending_stiffness,.5)

    def test_hair_collision_with_body_on_same_armature(self):
        samples=[]
        for collide in (False,True):
            reset()
            source,rig,collider,guides=hair_body_fixture(collide)
            core.cache_operation(bpy.context,guides)
            values=[]
            for frame in (15,20,25,30,35):
                bpy.context.scene.frame_set(frame)
                values.extend(points(guides[2]))
            samples.append(values)
        difference=max((a-b).length for a,b in zip(*samples))
        print('HAIR_BODY_COLLISION_DIFFERENCE',difference,flush=True)
        self.assertGreater(difference,.005)

    def test_direct_cloth_mesh_ui_and_collider_ui(self):
        source,rig=body_fixture()
        settings=bpy.context.scene.scs_physics
        settings.collider_faces=180
        self.assertEqual(bpy.ops.scs.body_collider(action='CREATE'),{'FINISHED'})
        collider=bpy.context.object
        self.assertEqual(collider[body.SOURCE],source)
        self.assertEqual(bpy.ops.scs.body_collider(action='REMOVE'),{'FINISHED'})
        bpy.ops.mesh.primitive_grid_add(x_subdivisions=10,y_subdivisions=10,size=1)
        sheet=bpy.context.object
        original=sheet.vertex_groups.new(name='User Pins')
        original.add([v.index for v in sheet.data.vertices if v.co.x < -.4],1.,'REPLACE')
        sub=sheet.modifiers.new('User subdivision','SUBSURF')
        settings.mode,settings.cloth_pin='CLOTH','User Pins'
        self.assertEqual(bpy.ops.scs.create_simulation(),{'FINISHED'})
        self.assertEqual([m.type for m in sheet.modifiers],['CLOTH','SUBSURF'])
        self.assertNotEqual(core.solver_for(sheet).settings.vertex_group_mass,original.name)
        core.remove_simulations([sheet])
        self.assertEqual(list(sheet.vertex_groups),[original])
        self.assertEqual(list(sheet.modifiers),[sub])
        settings.cloth_pin=''

    def test_chain_presets_all_entrypoints(self):
        for preset in ('HEAD_ONLY','TAIL_ONLY','BOTH_ENDS'):
            reset()
            settings = bpy.context.scene.scs_physics
            settings.mode,settings.chain_input,settings.hook_preset = 'CHAIN','PATH',preset
            obj = chain.create_proxy(bpy.context,8,.7)
            self.assertEqual(bpy.ops.scs.create_simulation(),{'FINISHED'})
            expected = mesh.preset_anchors(obj,preset)
            self.assertEqual(len(obj.get('scs_physics_hooks',[])),len(expected))
            play(15)
            actual = points(obj)
            for i in expected:
                self.assertLess((actual[i]-obj.data.vertices[i].co).length,1e-4)
            self.assertTrue(any(actual[i].z < -.01 for i in range(8) if i not in expected))
            reset()
            rig = hair_rig()
            settings = bpy.context.scene.scs_physics
            settings.mode,settings.hook_preset = 'CHAIN',preset
            self.assertEqual(bpy.ops.scs.create_simulation(),{'FINISHED'})
            for guide in core.simulations(bpy.context):
                hooks = [m for m in guide.modifiers if m.type == 'HOOK']
                self.assertEqual(len(hooks),2 if preset == 'BOTH_ENDS' else 1)
                group = guide.vertex_groups['Goal']
                fixed = {v.index for v in guide.data.vertices if group.weight(v.index) == 1}
                hooked = {v.index for v in guide.data.vertices for mod in hooks
                          if any(g.group == guide.vertex_groups[mod.vertex_group].index for g in v.groups)}
                self.assertEqual(fixed,hooked)

    def test_region_preset_handle_tracks_animated_source(self):
        for preset in ('HEAD_ONLY','TAIL_ONLY','BOTH_ENDS'):
            reset()
            bpy.ops.mesh.primitive_grid_add(x_subdivisions=16,y_subdivisions=3,size=1)
            source = bpy.context.object
            source.shape_key_add(name='Basis')
            key = source.shape_key_add(name='Original animation')
            for v in key.data:
                v.co.z += .2
            key.value = 0
            proxy = region.create_region(bpy.context,source,list(range(len(source.data.vertices))),count=8,
                                         hook_preset=preset)
            initial = points(proxy)
            key.value = 0
            key.keyframe_insert('value',frame=1)
            key.value = 1
            key.keyframe_insert('value',frame=15)
            play(15)
            for index in mesh.preset_anchors(proxy,preset):
                self.assertAlmostEqual(points(proxy)[index].z-initial[index].z,.2,places=4,msg=preset)


if __name__ == '__main__':
    addon.register()
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(ClothBodyTests))
    if not result.wasSuccessful():
        raise RuntimeError('Cloth/body tests failed')
    print('CLOTH_BODY_TESTS_PASSED',result.testsRun,flush=True)
