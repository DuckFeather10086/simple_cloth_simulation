"""Read-only character verification; all reports/renders go to the addon out/.

Usage: -- character.blend 'Body Mesh' [arm-bone leg-bone]
The input .blend is never saved over or redistributed.
"""
from pathlib import Path
import json
import math
import sys
import bpy
from mathutils import Vector

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT.parent))
import simple_cloth_simulation as addon
from simple_cloth_simulation import body_collider as body

args = sys.argv[sys.argv.index('--')+1:]
if '--daz' in args:
    import addon_utils
    args.remove('--daz')
    assert addon_utils.enable('bl_ext.user_default.import_daz',default_set=True) is not None
bpy.ops.wm.open_mainfile(filepath=args[0])
addon.register()
source = bpy.data.objects[args[1]]
rig = next(m.object for m in source.modifiers if m.type == 'ARMATURE')
OUT = ROOT/'out/unified-physics/cloth-body'
OUT.mkdir(parents=True,exist_ok=True)
source.hide_set(False)
collider = body.create_collider(bpy.context,source,target_faces=2500,margin=.003)


def points(obj):
    bpy.context.view_layer.update()
    evaluated=obj.evaluated_get(bpy.context.evaluated_depsgraph_get())
    return [evaluated.matrix_world @ v.co for v in evaluated.data.vertices]


def check_binding():
    ref=collider[body.REFERENCE].evaluated_get(bpy.context.evaluated_depsgraph_get())
    actual=points(collider)
    errors=[]
    for index,point in enumerate(actual):
        ids=[collider.data.attributes[f'scs_bind_i{axis}'].data[index].value for axis in range(3)]
        ws=[collider.data.attributes[f'scs_bind_w{axis}'].data[index].value for axis in range(3)]
        center=sum((ref.data.vertices[i].co*w for i,w in zip(ids,ws)),Vector())
        normal=sum((ref.data.vertices[i].normal*w for i,w in zip(ids,ws)),Vector()).normalized()
        errors.append((point-ref.matrix_world @ (center+.003*normal)).length)
    assert max(errors)<2e-5,max(errors)
    return max(errors)


initial=points(collider)
report={'input':args[0],'body':source.name,'source_vertices':len(source.data.vertices),
        'collider_vertices':len(collider.data.vertices),'collider_faces':len(collider.data.polygons),
        'source_modifiers':[m.type for m in source.modifiers],'reference_modifiers':[m.type for m in collider[body.REFERENCE].modifiers],
        'binding_error_initial':check_binding(),'poses':[],'status':'已构建未验收'}
bones=args[2:] or ['lForearmBend','lShin']
scene=bpy.context.scene
scene.frame_start,scene.frame_end=1,24
for name in bones:
    bone=rig.pose.bones[name]
    bone.rotation_mode='XYZ'
    bone.keyframe_insert('rotation_euler',frame=1)
    bone.rotation_euler.x += math.radians(55 if 'Shin' in name else -65)
    bone.keyframe_insert('rotation_euler',frame=24)
scene.frame_set(24)
after=points(collider)
report['motion_max']=max((a-b).length for a,b in zip(initial,after))
report['binding_error_posed']=check_binding()
report['poses']=bones
assert report['motion_max']>.1,report
assert len(after)==len(initial)
(OUT/'character-body-verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')

# Render only the simplified body, as a neutral surface with its actual edges.
# The source character, textures and purchased shape assets stay local.
for obj in scene.objects:
    obj.hide_render=True
collider.hide_render=False
mat=bpy.data.materials.new('Collision surface preview')
mat.diffuse_color=(.06,.36,.5,1)
mat.use_nodes=True
mat.node_tree.nodes['Principled BSDF'].inputs['Base Color'].default_value=mat.diffuse_color
mat.node_tree.nodes['Principled BSDF'].inputs['Roughness'].default_value=.65
collider.data.materials.append(mat)
black=bpy.data.materials.new('Actual low-poly edges')
black.diffuse_color=(.005,.025,.04,1)
collider.data.materials.append(black)
wire=collider.modifiers.new('Preview edges only','WIREFRAME')
wire.thickness=.0007
wire.use_replace=False
wire.material_offset=1
scene.world=bpy.data.worlds.new('Collider studio')
scene.world.color=(.12,.12,.12)
center=sum(initial,Vector())/len(initial)
center.x=0
for position,energy,size in [((3,-4,4),800,4),((-3,-1,2),500,3)]:
    bpy.ops.object.light_add(type='AREA',location=position)
    lamp=bpy.context.object
    lamp.data.energy, lamp.data.shape, lamp.data.size=energy,'DISK',size
    lamp.rotation_euler=(center-lamp.location).to_track_quat('-Z','Y').to_euler()
bpy.ops.object.camera_add(location=(2.8,-5,2.2))
camera=bpy.context.object
camera.rotation_euler=(Vector((0,0,.85))-camera.location).to_track_quat('-Z','Y').to_euler()
camera.data.type,camera.data.ortho_scale='ORTHO',1.9
scene.camera=camera
scene.render.engine='CYCLES'
scene.cycles.samples=24
scene.render.resolution_x,scene.render.resolution_y,scene.render.resolution_percentage=700,900,100
scene.render.film_transparent=False
for frame in (1,24):
    scene.frame_set(frame)
    scene.render.filepath=str(OUT/f'body-collider-{frame:02d}.png')
    bpy.ops.render.render(write_still=True)
print('CHARACTER_BODY_VERIFIED',json.dumps(report,ensure_ascii=False),flush=True)
