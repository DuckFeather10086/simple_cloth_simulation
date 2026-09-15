"""Run with Blender --background --factory-startup --python-exit-code 1 -P this_file.
Optional: -- --output-dir PATH (save/reload persistence probe).
"""
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import bpy
from mathutils import Vector, Quaternion

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import simple_cloth_simulation as addon
from simple_cloth_simulation import skeletal_weaver as sw
addon.register()

BASE = [('anchor',(0,0,-1),(0,0,0),None,False),
        ('a',(0,0,0),(0,1,0),'anchor',False),
        ('b',(0,1,0),(0,2,0),'a',True)]


def make_rig(spec=BASE, selected=('a','b'), name='Rig'):
    if bpy.context.object and bpy.context.object.mode != 'OBJECT':
        bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.select_all(action='DESELECT')
    data = bpy.data.armatures.new(name)
    obj = bpy.data.objects.new(name, data)
    bpy.context.collection.objects.link(obj)
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    bpy.ops.object.mode_set(mode='EDIT')
    for name, head, tail, parent, connect in spec:
        bone = data.edit_bones.new(name)
        bone.head, bone.tail = head, tail
        if parent:
            bone.parent = data.edit_bones[parent]
            bone.use_connect = connect
    bpy.ops.object.mode_set(mode='POSE')
    select(obj, selected)
    return obj


def select(obj, names):
    names = set(names)
    for bone in obj.pose.bones:
        if hasattr(bone, 'select'):
            bone.select = bone.name in names
        else:
            bone.bone.select = bone.name in names
    obj.data.bones.active = obj.data.bones[next(iter(names))] if names else None


def guides():
    return [obj for obj in bpy.data.objects if sw.is_guide_object(obj)]


def tracks(bone):
    return [c for c in bone.constraints if sw.is_weaver_damped_track_constraint(bone,c)]


def group_center(obj, name, evaluated=False):
    group = obj.vertex_groups[name]
    indices = [v.index for v in obj.data.vertices if any(g.group == group.index for g in v.groups)]
    data_obj = obj.evaluated_get(bpy.context.evaluated_depsgraph_get()) if evaluated else obj
    return sum((data_obj.matrix_world @ data_obj.data.vertices[i].co for i in indices),Vector()) / len(indices)


class GenerationTests(unittest.TestCase):
    def setUp(self):
        if bpy.context.object and bpy.context.object.mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')
        for obj in list(bpy.data.objects):
            bpy.data.objects.remove(obj,do_unlink=True)
        for mesh in list(bpy.data.meshes):
            if not mesh.users:
                bpy.data.meshes.remove(mesh)
        props = bpy.context.scene.skeletal_weaver_props
        for key in props.bl_rna.properties.keys():
            if key != 'rna_type':
                props.property_unset(key)
        bpy.context.scene.frame_start = 1
        bpy.context.scene.frame_end = 30
        bpy.context.scene.frame_set(1)

    def assertClose(self, a, b, eps=1e-4):
        self.assertLess((Vector(a)-Vector(b)).length, eps, (a,b))

    def test_repeat_preserves_manual_tracks_and_unselected_bones(self):
        rig = make_rig()
        empty = bpy.data.objects.new('ManualTarget',None)
        bpy.context.collection.objects.link(empty)
        empty.location = (0,2,0)
        manual = rig.pose.bones['a'].constraints.new('DAMPED_TRACK')
        manual.name = 'User Aim'; manual.target = empty
        legacy = rig.pose.bones['b'].constraints.new('DAMPED_TRACK')
        legacy.name = 'SW_DampedTrack_b'
        for _ in range(3):
            self.assertEqual(bpy.ops.armature.skeletal_weave(),{'FINISHED'})
            self.assertEqual(len(guides()),1)
            self.assertEqual(len(tracks(rig.pose.bones['a'])),1)
            self.assertEqual(len(tracks(rig.pose.bones['b'])),1)
            self.assertIsNotNone(rig.pose.bones['a'].constraints.get('User Aim'))
        old = tracks(rig.pose.bones['a'])[0].target
        select(rig, ['b'])
        bpy.ops.armature.skeletal_weave()
        self.assertEqual(tracks(rig.pose.bones['a'])[0].target,old)
        self.assertEqual(len(guides()),2)
        self.assertEqual(len(tracks(rig.pose.bones['b'])),1)

    def test_manual_same_target_is_not_owned(self):
        rig = make_rig(); bpy.ops.armature.skeletal_weave()
        target = guides()[0]
        manual = rig.pose.bones['a'].constraints.new('DAMPED_TRACK')
        manual.name = 'Manual guide aim'; manual.target = target; manual.subtarget = 'a'
        bpy.ops.armature.skeletal_weave()
        self.assertEqual(manual.target,target)
        self.assertEqual(len(guides()),2)  # user still uses the old guide
        self.assertEqual(len(tracks(rig.pose.bones['a'])),1)

    def test_broken_cleanup_only_plugin(self):
        rig=make_rig(); bpy.ops.armature.skeletal_weave()
        bone=rig.pose.bones['a']; tracks(bone)[0].target=None
        user=bone.constraints.new('IK'); user.name='Targetless IK'
        other=bone.constraints.new('DAMPED_TRACK'); other.name='User broken track'
        self.assertEqual(sw.remove_invalid_constraints([bone]),1)
        self.assertEqual(set(bone.constraints.keys()), {'Targetless IK','User broken track'})

    def test_branch_and_different_anchors(self):
        spec=[('root',(0,0,0),(0,1,0),None,False),
              ('left',(0,1,0),(-1,2,0),'root',True),
              ('right',(0,1,0),(1,2,0),'root',True),
              ('outside',(3,0,0),(3,1,0),None,False),
              ('other',(3,1,0),(3,2,0),'outside',False)]
        rig=make_rig(spec,['root','left','right','other'])
        bpy.ops.armature.skeletal_weave()
        self.assertEqual(len(guides()),3)
        self.assertEqual({b.name for b in rig.pose.bones if tracks(b)}, {'root','left','right','other'})
        self.assertEqual(tracks(rig.pose.bones['left'])[0].target.parent_bone,'root')
        self.assertEqual(tracks(rig.pose.bones['other'])[0].target.parent_bone,'outside')
        for name in ['root','left','right','other']:
            self.assertClose(group_center(tracks(rig.pose.bones[name])[0].target,name),rig.matrix_world @ rig.pose.bones[name].tail)

    def test_rotated_posed_scaled_parent_alignment(self):
        rig=make_rig()
        rig.location=(3,4,5); rig.scale=(2,1.5,.7)
        rig.rotation_mode='QUATERNION'; rig.rotation_quaternion=Quaternion((0,0,1),math.pi/4)
        rig.pose.bones['anchor'].rotation_mode='XYZ'; rig.pose.bones['anchor'].rotation_euler.x=.4
        rig.pose.bones['a'].rotation_mode='XYZ'; rig.pose.bones['a'].rotation_euler.z=.3
        bpy.context.view_layer.update()
        expected={b.name:rig.matrix_world @ b.tail for b in rig.pose.bones}
        before=rig.matrix_world.copy()
        bpy.ops.armature.skeletal_weave()
        mesh=guides()[0]
        self.assertEqual(rig.matrix_world,before)
        for name in ['a','b']:
            self.assertClose(group_center(mesh,name),expected[name])
        self.assertTrue(all(p.area > .001 for p in mesh.data.polygons))
        # Root row follows exactly the same upstream bone transform as the rig.
        old_root=group_center(mesh,'Pin')  # only check root vertices explicitly below
        first_indices=[0,1]
        old=[mesh.matrix_world @ mesh.data.vertices[i].co for i in first_indices]
        rig.location.x+=2; bpy.context.view_layer.update()
        for i, prev in zip(first_indices,old):
            self.assertClose(mesh.matrix_world @ mesh.data.vertices[i].co,prev+Vector((2,0,0)))

    def test_legacy_entry_root_no_parent(self):
        rig=make_rig([('a',(0,0,0),(0,1,0),None,False)],['a'])
        rig.location=(3,4,5); bpy.context.view_layer.update()
        before=rig.matrix_world.copy()
        self.assertEqual(bpy.ops.object.simple_cloth_simulation(),{'FINISHED'})
        self.assertEqual(rig.matrix_world,before)
        self.assertEqual(guides()[0].parent_type,'OBJECT')
        self.assertEqual(guides()[0].parent,rig)
        self.assertClose(group_center(guides()[0],'a'),(3,5,5))

    def test_weld_shared_tails_keep_both_groups(self):
        rig=make_rig([('a',(0,0,0),(0,1,0),None,False),
                      ('b',(1,0,0),(0,1,0),None,False)],['a','b'])
        bpy.ops.armature.skeletal_weave()
        mesh=guides()[0]
        self.assertEqual(len(mesh.data.vertices),3)
        self.assertEqual(len(tracks(rig.pose.bones['b'])),1)
        self.assertClose(group_center(mesh,'a'),group_center(mesh,'b'))

    def test_short_chain_weight_by_own_length(self):
        rig=make_rig([('a',(0,0,0),(0,1,0),None,False),
                      ('b',(1,0,0),(1,.2,0),None,False),
                      ('c',(1,.2,0),(1,1,0),'b',True)],['a','b','c'])
        props=bpy.context.scene.skeletal_weaver_props
        props.pin_falloff_pattern='LINEAR'; props.pin_min_weight=.1
        bpy.ops.armature.skeletal_weave()
        mesh=guides()[0]; pin=mesh.vertex_groups['Pin']
        for bone_name,expected in [('a',.1),('b',.82),('c',.1)]:
            target=group_center(mesh,bone_name)
            v=min(mesh.data.vertices,key=lambda v:(v.co-target).length)
            self.assertAlmostEqual(pin.weight(v.index),expected,places=5)

    def test_softbody_repeat_and_clean(self):
        rig=make_rig(); props=bpy.context.scene.skeletal_weaver_props; props.physics_mode='SOFT_BODY'
        for _ in range(3):
            bpy.ops.armature.skeletal_weave()
            self.assertEqual(len(guides()),1)
            mesh=guides()[0]
            self.assertEqual([m.type for m in mesh.modifiers],['HOOK','HOOK','SOFT_BODY'])
            self.assertEqual(len([o for o in bpy.data.objects if o.type=='EMPTY']),2)
            goal=mesh.vertex_groups['Goal']
            for hook in mesh.modifiers[:2]:
                group=mesh.vertex_groups[hook.vertex_group]
                self.assertTrue(all(goal.weight(v.index)==1 for v in mesh.data.vertices
                                    if any(g.group==group.index for g in v.groups)))
            self.assertClose(group_center(mesh,'a'),rig.matrix_world @ rig.pose.bones['a'].tail)
        mesh.name='Renamed guide'
        sw.cleanup_previous_meshes()
        self.assertEqual(len(guides()),0)
        self.assertFalse(any(o.type=='EMPTY' for o in bpy.data.objects))
        self.assertFalse(any(tracks(b) for b in rig.pose.bones))

    def test_multichain_softbody_gets_each_solver(self):
        rig=make_rig([('a',(0,0,0),(0,1,0),None,False),
                      ('b',(1,0,0),(1,1,0),None,False)],['a','b'])
        bpy.context.scene.skeletal_weaver_props.physics_mode='SOFT_BODY'
        bpy.ops.armature.skeletal_weave()
        self.assertEqual(len(guides()),2)
        self.assertTrue(all(any(m.type=='SOFT_BODY' for m in g.modifiers) for g in guides()))

    def test_hook_drives_interior_simulation(self):
        results=[]
        for moved in (False,True):
            self.setUp(); rig=make_rig()
            props=bpy.context.scene.skeletal_weaver_props
            props.physics_mode='SOFT_BODY'; props.add_bone_constraints=False
            bpy.ops.armature.skeletal_weave(); mesh=guides()[0]
            tail=[m for m in mesh.modifiers if m.type=='HOOK'][-1].object
            for frame in range(1,21):
                if moved:
                    tail.location.x=(frame-1)*.05
                bpy.context.scene.frame_set(frame); bpy.context.view_layer.update()
                evaluated=mesh.evaluated_get(bpy.context.evaluated_depsgraph_get())
                points=[v.co.copy() for v in evaluated.data.vertices]
            results.append(points)
        # vertices 2,3,8,9 are the middle cross-section, not hooked endpoints.
        response=max((results[1][i]-results[0][i]).length for i in (2,3,8,9))
        self.assertGreater(response,.001, response)
        print('INTERIOR_HOOK_RESPONSE',response)

    def test_manual_hook_and_parent_follow(self):
        rig=make_rig(); bpy.context.scene.skeletal_weaver_props.physics_mode='SOFT_BODY'
        bpy.context.scene.skeletal_weaver_props.softbody_hook_preset='NONE'
        bpy.ops.armature.skeletal_weave(); mesh=guides()[0]
        bpy.ops.object.mode_set(mode='OBJECT'); bpy.ops.object.select_all(action='DESELECT')
        mesh.select_set(True); bpy.context.view_layer.objects.active=mesh
        for v in mesh.data.vertices: v.select=v.index in (2,3,8,9)
        bpy.ops.object.mode_set(mode='EDIT')
        self.assertEqual(bpy.ops.mesh.skeletal_weaver_add_hook_from_selection(),{'FINISHED'})
        bpy.ops.object.mode_set(mode='OBJECT'); bpy.context.view_layer.update()
        self.assertEqual([m.type for m in mesh.modifiers],['HOOK','SOFT_BODY'])
        empty=mesh.modifiers[0].object; previous=empty.matrix_world.translation.copy()
        rig.location=(1,2,3); bpy.context.view_layer.update()
        self.assertClose(empty.matrix_world.translation,previous+Vector((1,2,3)))

    def test_failure_preserves_previous_result(self):
        rig=make_rig(); bpy.ops.armature.skeletal_weave()
        before_objects=set(bpy.data.objects.keys()); before_meshes=set(bpy.data.meshes.keys())
        before_tracks={b.name:[c.name for c in b.constraints] for b in rig.pose.bones}
        before_matrix=rig.matrix_world.copy()
        with patch.object(sw,'add_cloth_modifier',side_effect=ValueError('Injected build failure')):
            with self.assertRaises(RuntimeError):
                bpy.ops.armature.skeletal_weave()
        self.assertEqual(set(bpy.data.objects.keys()),before_objects)
        self.assertEqual(set(bpy.data.meshes.keys()),before_meshes)
        self.assertEqual(before_tracks,{b.name:[c.name for c in b.constraints] for b in rig.pose.bones})
        self.assertTrue(all(not c.mute for b in rig.pose.bones for c in tracks(b)))
        self.assertEqual(rig.matrix_world,before_matrix)
        self.assertEqual(bpy.context.mode,'POSE')

    def test_invalid_configuration_no_mutation(self):
        rig=make_rig(); bpy.ops.armature.skeletal_weave(); before=set(bpy.data.objects.keys())
        bpy.context.scene.skeletal_weaver_props.create_vertex_groups=False
        with self.assertRaises(RuntimeError):
            bpy.ops.armature.skeletal_weave()
        self.assertEqual(set(bpy.data.objects.keys()),before)
        self.assertEqual(len(tracks(rig.pose.bones['a'])),1)

    def test_tetra_batch_pose_and_replacement(self):
        rig=make_rig()
        rig.location=(3,4,5); rig.scale=(1.5,.7,2)
        rig.rotation_euler=(.1,.2,.3)
        rig.pose.bones['a'].rotation_mode='XYZ'; rig.pose.bones['a'].rotation_euler.z=.4
        bpy.context.view_layer.update()
        expected={b.name:rig.matrix_world @ b.tail for b in rig.pose.bones}
        for _ in range(3):
            self.assertEqual(bpy.ops.object.create_tetrahedron(),{'FINISHED'})
            self.assertEqual(len(guides()),2)
            for name in ('a','b'):
                self.assertEqual(len(tracks(rig.pose.bones[name])),1)
                mesh=tracks(rig.pose.bones[name])[0].target
                self.assertClose(group_center(mesh,name),expected[name])
                self.assertEqual(len(mesh.data.vertices),5)
        sw.cleanup_previous_meshes()
        self.assertFalse(guides())

    def test_tetra_root_follows_armature(self):
        rig=make_rig([('a',(0,0,0),(0,1,0),None,False)],['a'])
        bpy.ops.object.create_tetrahedron(); mesh=guides()[0]
        self.assertEqual(mesh.parent,rig)
        rig.location=(3,4,5); bpy.context.view_layer.update()
        self.assertClose(group_center(mesh,'a'),(3,5,5))

    def test_guide_only_rebuild_replaces_generated_meshes(self):
        make_rig()
        props=bpy.context.scene.skeletal_weaver_props
        props.use_physics_modifier=False; props.add_bone_constraints=False
        for _ in range(3):
            bpy.ops.armature.skeletal_weave()
            self.assertEqual(len(guides()),1)

    def test_reserved_group_names_do_not_capture_pin_group(self):
        rig=make_rig([('Pin',(0,0,0),(0,1,0),None,False)],['Pin'])
        bpy.ops.armature.skeletal_weave()
        constraint=tracks(rig.pose.bones['Pin'])[0]
        self.assertNotEqual(constraint.subtarget,'Pin')
        self.assertClose(group_center(constraint.target,constraint.subtarget),(0,1,0))
        bpy.ops.object.create_tetrahedron()
        constraint=tracks(rig.pose.bones['Pin'])[0]
        self.assertNotEqual(constraint.subtarget,'Pin')
        self.assertClose(group_center(constraint.target,constraint.subtarget),(0,1,0))

    def test_rename_preserves_targets_and_repeat_ownership(self):
        rig=make_rig([('a.001',(0,0,0),(0,1,0),None,False)],['a.001'])
        bpy.ops.armature.skeletal_weave()
        bpy.ops.armature.clean_bone_name_suffixes()
        self.assertIn('a',rig.pose.bones)
        c=tracks(rig.pose.bones['a'])[0]
        self.assertIn(c.subtarget,c.target.vertex_groups)
        self.assertClose(group_center(c.target,c.subtarget),(0,1,0))
        bpy.ops.armature.skeletal_weave()
        self.assertEqual(len(guides()),1)
        self.assertEqual(len(tracks(rig.pose.bones['a'])),1)

    def test_custom_falloff_preserved(self):
        make_rig()
        props=bpy.context.scene.skeletal_weaver_props
        props.pin_falloff_pattern='CUSTOM'
        mapping=sw.get_or_create_pin_falloff_brush().curve_distance_falloff
        mapping.initialize()
        mapping.curves[0].points[0].location=(0,1)
        mapping.curves[0].points[-1].location=(1,.2)
        mapping.update()
        expected=sw.evaluate_pin_falloff(1,'CUSTOM',4,.05,mapping)
        bpy.ops.armature.skeletal_weave()
        mesh=guides()[0]; end=group_center(mesh,'b')
        index=min(mesh.data.vertices,key=lambda v:(v.co-end).length).index
        self.assertAlmostEqual(mesh.vertex_groups['Pin'].weight(index),expected,places=5)
        self.assertAlmostEqual(mapping.curves[0].points[-1].location.y,.2,places=5)

    def test_failure_after_constraints_and_snap_restores_original(self):
        rig=make_rig([('a',(0,0,0),(0,1,0),None,False),
                      ('b',(0,1.0005,0),(0,2,0),'a',False)],['a','b'])
        props=bpy.context.scene.skeletal_weaver_props; props.snap_bones=True
        before=rig.data.bones['b'].head_local.copy()
        real_add=sw.add_bone_constraints
        def fail_after(*args):
            real_add(*args)
            raise ValueError('Injected failure after constraints')
        with patch.object(sw,'add_bone_constraints',side_effect=fail_after):
            with self.assertRaises(RuntimeError):
                bpy.ops.armature.skeletal_weave()
        self.assertFalse(guides())
        self.assertFalse(rig.data.bones['b'].use_connect)
        self.assertClose(rig.data.bones['b'].head_local,before)
        self.assertFalse(any(tracks(b) for b in rig.pose.bones))
        self.assertTrue(all(sw.WEAVER_PRE_CONSTRAINT_MATRIX not in b for b in rig.pose.bones))

    def test_save_reload_retains_settings_and_track_ownership(self):
        if '--output-dir' not in sys.argv:
            self.skipTest('Supply -- --output-dir PATH for persistence test')
        directory=Path(sys.argv[sys.argv.index('--output-dir')+1]); directory.mkdir(parents=True,exist_ok=True)
        rig=make_rig(); bpy.ops.armature.skeletal_weave()
        props=bpy.context.scene.simple_cloth_sim; props.sub_level=7; props.parent_armature=False
        bpy.context.scene.skeletal_weaver_props.pin_min_weight=.17
        bpy.ops.wm.save_as_mainfile(filepath=str(directory/'persistence.blend'))
        bpy.ops.wm.open_mainfile(filepath=str(directory/'persistence.blend'))
        self.assertEqual(bpy.context.scene.simple_cloth_sim.sub_level,7)
        self.assertFalse(bpy.context.scene.simple_cloth_sim.parent_armature)
        self.assertAlmostEqual(bpy.context.scene.skeletal_weaver_props.pin_min_weight,.17,places=6)
        rig=bpy.data.objects['Rig']
        self.assertEqual(len(tracks(rig.pose.bones['a'])),1)
        bpy.ops.armature.skeletal_weave()
        self.assertEqual(len(guides()),1)


suite=unittest.defaultTestLoader.loadTestsFromTestCase(GenerationTests)
result=unittest.TextTestRunner(verbosity=2).run(suite)
addon.unregister(); addon.register(); addon.unregister()
if not result.wasSuccessful():
    raise RuntimeError('Blender regression tests failed')
print('ALL_TESTS_PASSED',result.testsRun)
