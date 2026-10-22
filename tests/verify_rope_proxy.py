"""Verify an exported real edge mesh, save editable cages, and render a preview."""
from pathlib import Path
import json
import sys
import bpy
from mathutils import Matrix, Vector

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import simple_cloth_simulation as addon
from simple_cloth_simulation import rope_proxy as rope
addon.register()
bpy.ops.wm.read_factory_settings(use_empty=True)
path = Path(sys.argv[sys.argv.index('--') + 1]).resolve()
data = json.loads(path.read_text(encoding='utf8'))
mesh = bpy.data.meshes.new('Actual rope centerlines')
mesh.from_pydata(data['vertices'], data['edges'], [])
if 'rope_radius' in data:
    mesh.attributes.new('rope_radius','FLOAT','POINT').data.foreach_set('value',data['rope_radius'])
source = bpy.data.objects.new('Actual rope centerlines', mesh)
bpy.context.collection.objects.link(source)
source.matrix_world = Matrix(data['matrix'])
source.select_set(True)
bpy.context.view_layer.objects.active = source
proxies = rope.create(bpy.context, source)
assert len(proxies) == 2, len(proxies)
bpy.context.view_layer.update()
before = [v.co.copy() for v in source.evaluated_get(bpy.context.evaluated_depsgraph_get()).data.vertices]
jump = max((v.co - point).length for v, point in zip(source.data.vertices, before))
assert jump < 1e-5, jump
target = proxies[0]
for vertex in target.data.vertices:
    vertex.co.y += .025
target.data.update()
bpy.context.view_layer.update()
after = [v.co.copy() for v in source.evaluated_get(bpy.context.evaluated_depsgraph_get()).data.vertices]
group = source.vertex_groups[target['scs_rope_proxy_group']]
selected = {v.index for v in source.data.vertices if any(g.group == group.index for g in v.groups)}
assert max((a-b).length for i,(a,b) in enumerate(zip(after,before)) if i not in selected) < 1e-5
assert max((a-b).length for i,(a,b) in enumerate(zip(after,before)) if i in selected) > .02
for vertex in target.data.vertices:
    vertex.co.y -= .025
target.data.update()
out = path.parent
bpy.ops.wm.save_as_mainfile(filepath=str(out / 'actual-rope-cages.blend'))
report = dict(cages=len(proxies), vertices_each=[len(o.data.vertices) for o in proxies],
              faces_each=[len(o.data.polygons) for o in proxies], bind_jump=jump,
              independent_deformation=True, physics_added=False)
(out / 'actual-rope-result.json').write_text(json.dumps(report,indent=2),encoding='utf8')

# Only the preview copies have wireframe rendering geometry; editable cages stay unchanged.
source.matrix_world = Matrix.Identity(4)
for obj in proxies:
    obj.matrix_basis = Matrix.Identity(4)
palette = ((.07,.5,1.,1), (1.,.3,.06,1))
for proxy, color in zip(proxies, palette):
    display = bpy.data.objects.new('Preview wire ' + proxy.name, proxy.data.copy())
    bpy.context.collection.objects.link(display)
    display.color = color
    wire = display.modifiers.new('Preview wires', 'WIREFRAME')
    wire.thickness = .00025
    wire.use_replace = True
    wire.use_even_offset = True

graph = bpy.data.node_groups.new('Preview centerline tubes', 'GeometryNodeTree')
graph.interface.new_socket(name='Geometry',in_out='INPUT',socket_type='NodeSocketGeometry')
graph.interface.new_socket(name='Geometry',in_out='OUTPUT',socket_type='NodeSocketGeometry')
inp=graph.nodes.new('NodeGroupInput');outnode=graph.nodes.new('NodeGroupOutput')
curve=graph.nodes.new('GeometryNodeMeshToCurve');profile=graph.nodes.new('GeometryNodeCurvePrimitiveCircle')
profile.inputs['Resolution'].default_value=8;profile.inputs['Radius'].default_value=.0014
tube=graph.nodes.new('GeometryNodeCurveToMesh')
graph.links.new(inp.outputs['Geometry'],curve.inputs['Mesh'])
graph.links.new(curve.outputs['Curve'],tube.inputs['Curve'])
graph.links.new(profile.outputs['Curve'],tube.inputs['Profile Curve'])
graph.links.new(tube.outputs['Mesh'],outnode.inputs['Geometry'])
source.modifiers.new('Preview tubes','NODES').node_group=graph
source.color=(.7,.5,.2,1)
bpy.ops.object.camera_add(location=(.5,-.8,.35))
camera=bpy.context.object
camera.rotation_euler=(Vector((-.12,-.01,.18))-camera.location).to_track_quat('-Z','Y').to_euler()
camera.data.type='ORTHO';camera.data.ortho_scale=.53
scene=bpy.context.scene;scene.camera=camera
scene.render.engine='BLENDER_WORKBENCH'
scene.display.shading.light='STUDIO';scene.display.shading.color_type='OBJECT'
scene.display.shading.show_shadows=False;scene.display.shading.show_cavity=True
scene.display.shading.background_type='WORLD'
scene.world=bpy.data.worlds.new('Preview World');scene.world.color=(.015,.02,.03)
scene.render.resolution_x=900;scene.render.resolution_y=1000;scene.render.resolution_percentage=100
scene.render.filepath=str(path.parent/'actual-rope-cages.png')
bpy.ops.render.render(write_still=True)
print('ACTUAL_ROPE_PROXY_PASS',json.dumps(report),flush=True)
