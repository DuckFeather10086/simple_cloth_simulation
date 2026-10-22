"""Build a small editable chain demonstration and render its real simulation."""
from pathlib import Path
import math
import sys
import bpy
from mathutils import Vector

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parent))
import simple_cloth_simulation as addon
from simple_cloth_simulation import chain_layout as chain, mesh_physics as mesh, physics_core as core

bpy.ops.wm.read_factory_settings(use_empty=True)
addon.register()
OUT = ROOT / 'out/unified-physics/demo'
OUT.mkdir(parents=True, exist_ok=True)
scene = bpy.context.scene
scene.frame_start, scene.frame_end = 1, 48
scene.render.engine = 'CYCLES'
scene.cycles.samples = 24
scene.render.resolution_x, scene.render.resolution_y, scene.render.resolution_percentage = 1200, 700, 100
scene.world = bpy.data.worlds.new('Studio')
scene.world.color = (.16, .16, .16)


def material(name, color, metal=0.):
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = (*color, 1)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get('Principled BSDF')
    bsdf.inputs['Base Color'].default_value = (*color, 1)
    bsdf.inputs['Metallic'].default_value = metal
    bsdf.inputs['Roughness'].default_value = .3
    return mat


steel = material('Brushed gold', (.58, .3, .055), .8)
blue = material('Specified Hook', (.02, .45, .8))
white = material('Labels', (.65, .78, .85))
floor_mat = material('Dark floor', (.022, .033, .045))

proxy = chain.create_proxy(bpy.context, 17, 1.6)
for i, vertex in enumerate(proxy.data.vertices):
    vertex.co.z = 1.05 - .22 * math.sin(i / 16 * math.pi)
mesh.prepare_chain(proxy, [0, 16], stiffness=.95, damping=3.)
middle = bpy.data.objects.new('指定 Hook · 中间第 009 节', None)
bpy.context.collection.objects.link(middle)
middle.location = proxy.data.vertices[8].co
middle.location.y = 0
bpy.context.view_layer.update()
mesh.add_hook(proxy, [8], target=middle)
middle.keyframe_insert('location', frame=1)
middle.location.z += .28
middle.keyframe_insert('location', frame=24)
middle.location.z -= .28
middle.keyframe_insert('location', frame=48)
scene.frame_set(1)
chain.mark_nodes(proxy)

bpy.ops.mesh.primitive_torus_add(major_segments=24, minor_segments=8, major_radius=.06, minor_radius=.009)
template = bpy.context.object
template.name = '链节模板 · Y 长轴'
for vertex in template.data.vertices:
    vertex.co.y *= 1.5
template.data.materials.append(steel)
template.location = (0, 0, -3)
template.hide_render = True
chain.build_links(proxy, template, pitch=.68)

for i in (0, 8, 16):
    marker = next(o for o in bpy.data.objects if o.get(chain.ROLE) == 'NODE' and o[chain.NODE] == i)
    bpy.ops.mesh.primitive_uv_sphere_add(segments=16, ring_count=8, radius=.021)
    ball = bpy.context.object
    ball.name = 'Preview anchor ' + str(i + 1)
    ball.parent = marker
    ball.location = (0, 0, 0)
    ball.data.materials.append(blue)

for i in range(17):
    marker = next(o for o in bpy.data.objects if o.get(chain.ROLE) == 'NODE' and o[chain.NODE] == i)
    font = bpy.data.curves.new('Node label', 'FONT')
    font.body = f'{i + 1:02d}'
    font.size = .026
    font.align_x = 'CENTER'
    text = bpy.data.objects.new('Preview node number', font)
    bpy.context.collection.objects.link(text)
    text.parent = marker
    text.location = (0, -.04, .065)
    text.rotation_euler.x = math.pi / 2
    text.data.materials.append(white)

font = bpy.data.curves.new('Title', 'FONT')
font.body = 'SIMPLE PHYSICS  /  CHAIN + CUSTOM HOOKS'
font.size = .048
text = bpy.data.objects.new('Preview title', font)
bpy.context.collection.objects.link(text)
text.location = (-.1, 0, 1.55)
text.rotation_euler.x = math.pi / 2
text.data.materials.append(white)

bpy.ops.mesh.primitive_plane_add(size=200, location=(0, 0, .25))
bpy.context.object.data.materials.append(floor_mat)

for location, energy, size in [((.3, -2, 3), 180, 3), ((2, 1, 2), 220, 2)]:
    bpy.ops.object.light_add(type='AREA', location=location)
    light = bpy.context.object
    light.data.energy, light.data.shape, light.data.size = energy, 'DISK', size
    light.rotation_euler = (Vector((.8, 0, .8)) - light.location).to_track_quat('-Z', 'Y').to_euler()
bpy.ops.object.camera_add(location=(1.1, -3.8, 2.0))
camera = bpy.context.object
camera.rotation_euler = (Vector((.8, 0, .95)) - camera.location).to_track_quat('-Z', 'Y').to_euler()
camera.data.type, camera.data.ortho_scale = 'ORTHO', 2.35
scene.camera = camera
scene.scs_physics.template = template
scene.scs_physics.target = middle
scene.scs_physics.mode = 'CHAIN'
core.cache_operation(bpy.context, [proxy])
for frame in (1, 24):
    scene.frame_set(frame)
    scene.render.filepath = str(OUT / f'chain-{frame:02d}.png')
    bpy.ops.render.render(write_still=True)
scene.frame_set(1)
bpy.ops.object.select_all(action='DESELECT')
proxy.select_set(True)
bpy.context.view_layer.objects.active = proxy
bpy.ops.wm.save_as_mainfile(filepath=str(OUT / 'chain-demo.blend'))
print('PHYSICS_PREVIEW_READY', OUT, flush=True)
