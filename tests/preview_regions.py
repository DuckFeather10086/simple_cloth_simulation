"""Render the saved one-Mesh region regression fixture, without recreating links."""
from pathlib import Path
import math
import sys
import bpy
from mathutils import Vector

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'out/unified-physics/regions'
continuous = '--continuous' in sys.argv
if continuous:
    sys.path.insert(0, str(ROOT.parent))
    import simple_cloth_simulation as addon
    from simple_cloth_simulation import region_physics as region, physics_core as core
    bpy.ops.wm.read_factory_settings(use_empty=True)
    addon.register()
    data = bpy.data.meshes.new('Existing Ribbon')
    data.from_pydata([(i * .04, (j - 1.5) * .03, 1.) for i in range(17) for j in range(4)], [],
                     [(i*4+j, (i+1)*4+j, (i+1)*4+j+1, i*4+j+1) for i in range(16) for j in range(3)])
    source = bpy.data.objects.new('One Mesh · local continuous region', data)
    bpy.context.collection.objects.link(source)
    bpy.context.scene.frame_start, bpy.context.scene.frame_end = 1, 20
    bpy.context.view_layer.objects.active = source
    source.select_set(True)
    proxy = region.create_region(bpy.context, source, list(range(16, 68)), count=8)
    core.cache_operation(bpy.context, [proxy])
else:
    bpy.ops.wm.open_mainfile(filepath=str(OUT / 'local-chain.blend'))
    source = bpy.data.objects['Existing Rings']
    source.name = 'One Mesh · 3 unchanged rings + 8 simulated rings'
scene = bpy.context.scene
prefix = 'continuous-chain' if continuous else 'local-chain'


def material(name, color, metal=0.):
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = (*color, 1)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get('Principled BSDF')
    bsdf.inputs['Base Color'].default_value = (*color, 1)
    bsdf.inputs['Metallic'].default_value = metal
    bsdf.inputs['Roughness'].default_value = .27
    return mat


source.data.materials.append(material('Unselected', (.025, .45, .64), .6))
source.data.materials.append(material('Selected region', (.65, .3, .04), .7))
for polygon in source.data.polygons:
    polygon.material_index = int(polygon.index >= (9 if continuous else 3 * 72))
    polygon.use_smooth = True
white = material('Text', (.7, .8, .9))
for body, z, size in [('ONE MESH  /  LOCAL CHAIN SIMULATION', 1.29, .032),
                      ('CYAN: unchanged   |   GOLD: selected region', 1.23, .023),
                      ('68 vertices, continuous ribbon, 8 proxy nodes' if continuous else
                       '792 vertices, 11 existing rings, 8 proxy nodes', 1.18, .02)]:
    font = bpy.data.curves.new('Label', 'FONT')
    font.body, font.size = body, size
    text = bpy.data.objects.new('Preview Label', font)
    bpy.context.collection.objects.link(text)
    text.location = (-.22, 0, z)
    text.rotation_euler.x = math.pi / 2
    font.materials.append(white)

bpy.ops.mesh.primitive_plane_add(size=200, location=(0, 0, .08))
bpy.context.object.data.materials.append(material('Floor', (.02, .03, .04)))
for loc, energy in [((.2, -2, 3), 130), ((1, 1, 2), 100)]:
    bpy.ops.object.light_add(type='AREA', location=loc)
    light = bpy.context.object
    light.data.energy, light.data.size = energy, 2
    light.rotation_euler = (Vector((.3, 0, .8)) - light.location).to_track_quat('-Z', 'Y').to_euler()
bpy.ops.object.camera_add(location=(1.3, -1.7, 1.65) if continuous else (.45, -3, 1.65))
camera = bpy.context.object
camera.rotation_euler = (Vector((.3, 0, .8)) - camera.location).to_track_quat('-Z', 'Y').to_euler()
camera.data.type, camera.data.ortho_scale = 'ORTHO', 1.65 if continuous else 1.35
scene.camera = camera
scene.render.engine = 'CYCLES'
scene.cycles.samples = 24
scene.render.resolution_x, scene.render.resolution_y, scene.render.resolution_percentage = 1200, 900, 100
for frame in (1, 12):
    scene.frame_set(frame)
    scene.render.filepath = str(OUT / f'{prefix}-{frame:02d}.png')
    bpy.ops.render.render(write_still=True)
scene.frame_set(1)
bpy.ops.object.select_all(action='DESELECT')
source.select_set(True)
bpy.context.view_layer.objects.active = source
bpy.ops.wm.save_as_mainfile(filepath=str(OUT / f'{prefix}-demo.blend'))
print('REGION_PREVIEW_READY', OUT, flush=True)
