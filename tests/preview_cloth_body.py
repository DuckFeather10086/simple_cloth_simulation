"""Render the actual baked cloth and independent hair guides, no UI automation."""
from pathlib import Path
import math
import sys
import bpy
from mathutils import Vector

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tests'))
from test_cloth_body import addon, body, core, reset, points, hair_body_fixture
OUT = ROOT/'out/unified-physics/cloth-body'
hair = '--hair' in sys.argv
addon.register()
if hair:
    reset()
    source,rig,collider,guides = hair_body_fixture()
    initial=points(guides[2])
    core.cache_operation(bpy.context,guides)
    bpy.context.scene.frame_set(20)
    assert max((a-b).length for a,b in zip(initial,points(guides[2])))>.05
    bpy.ops.object.mode_set(mode='OBJECT')
    prefix='hair-cloth'
else:
    bpy.ops.wm.open_mainfile(filepath=str(OUT/'cloth-body.blend'))
    source=bpy.data.objects['Skinned Body']
    collider=next(o for o in bpy.data.objects if o.get(body.SOURCE))
    guides=[bpy.data.objects['Cloth Sheet']]
    prefix='cloth-body'

scene=bpy.context.scene


def material(name,color):
    mat=bpy.data.materials.new(name)
    mat.diffuse_color=(*color,1)
    mat.use_nodes=True
    shader=mat.node_tree.nodes['Principled BSDF']
    shader.inputs['Base Color'].default_value=mat.diffuse_color
    shader.inputs['Roughness'].default_value=.45
    return mat


source.hide_render=True
collider.hide_render=False
collider.data.materials.append(material('Animated low-poly body',(.035,.22,.3)))
collider.data.materials.append(material('Collision edges',(.005,.015,.025)))
wire=collider.modifiers.new('Preview edges','WIREFRAME')
wire.thickness=.001
wire.use_replace=False
wire.material_offset=1
for i,guide in enumerate(guides):
    guide.hide_render=False
    guide.data.materials.append(material('Cloth strand',(.5+i*.05,.22+i*.035,.06)))
    solid=guide.modifiers.new('Preview cloth thickness','SOLIDIFY')
    solid.thickness=.003
    for face in guide.data.polygons:
        face.use_smooth=True
bpy.ops.mesh.primitive_plane_add(size=200,location=(0,0,.04))
bpy.context.object.data.materials.append(material('Floor',(.018,.025,.035)))
scene.world=bpy.data.worlds.new('Cloth studio')
scene.world.color=(.12,.12,.12)
for loc,energy,size in [((1,-3,4),550,3),((-2,1,2),400,2)]:
    bpy.ops.object.light_add(type='AREA',location=loc)
    lamp=bpy.context.object
    lamp.data.energy,lamp.data.shape,lamp.data.size=energy,'DISK',size
    lamp.rotation_euler=(Vector((0,0,.65))-lamp.location).to_track_quat('-Z','Y').to_euler()
bpy.ops.object.camera_add(location=(1.2,-2.3,1.65))
camera=bpy.context.object
camera.rotation_euler=(Vector((0,0,.65))-camera.location).to_track_quat('-Z','Y').to_euler()
camera.data.type,camera.data.ortho_scale='ORTHO',1.65
scene.camera=camera
scene.render.engine='CYCLES'
scene.cycles.samples=32
scene.render.resolution_x,scene.render.resolution_y,scene.render.resolution_percentage=1000,900,100
for frame in (1,20 if hair else 35):
    scene.frame_set(frame)
    scene.render.filepath=str(OUT/f'{prefix}-{frame:02d}.png')
    bpy.ops.render.render(write_still=True)
scene.frame_set(1)
bpy.ops.object.select_all(action='DESELECT')
guides[0].select_set(True)
bpy.context.view_layer.objects.active=guides[0]
# Wire display is only for these previews; keep collision meshes as generated.
collider.modifiers.remove(wire)
collider.hide_render=True
source.hide_render=False
scene.scs_physics.mode='CLOTH'
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/f'{prefix}-demo.blend'))
print('CLOTH_PREVIEW_READY',prefix,flush=True)
