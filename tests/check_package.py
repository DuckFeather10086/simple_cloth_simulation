"""Load and exercise the distributed archive without importing the checkout."""
from pathlib import Path
import sys
import tempfile
import zipfile
import bpy

archive = Path(sys.argv[sys.argv.index('--') + 1])
with tempfile.TemporaryDirectory(prefix='scs-package-') as temporary:
    package = Path(temporary) / 'simple_cloth_simulation'
    with zipfile.ZipFile(archive) as content:
        content.extractall(package)
    sys.path.insert(0, temporary)
    import simple_cloth_simulation as addon
    assert Path(addon.__file__).parent == package
    addon.register()
    bpy.ops.mesh.primitive_grid_add(x_subdivisions=9, y_subdivisions=3, size=1)
    obj = bpy.context.object
    selected = [v.index for v in obj.data.vertices if v.co.x > 0]
    proxy = addon.physics_ui.region.create_region(bpy.context, obj, selected, count=4)
    assert proxy['scs_region_source'] == obj
    addon.physics_ui.core.remove_simulations([proxy])
    bpy.ops.object.armature_add()
    rig=bpy.context.object
    obj.modifiers.new('Skin','ARMATURE').object=rig
    collider=addon.physics_ui.body.create_collider(bpy.context,obj,target_faces=32)
    assert collider['scs_body_collider_source']==obj
    addon.physics_ui.body.remove_colliders([collider])
    pins={v.index:1. for v in obj.data.vertices if v.co.x < -.4}
    addon.physics_ui.cloth.prepare_cloth(obj,pins)
    assert addon.physics_ui.core.solver_for(obj).type=='CLOTH'
    addon.physics_ui.core.remove_simulations([obj])
    path = addon.physics_ui.chain.create_proxy(bpy.context, 12, .5)
    cages = addon.physics_ui.rope.create(bpy.context, path)
    assert len(cages) == 1 and path.modifiers[0].is_bound
    addon.physics_ui.rope.remove(bpy.context, cages)
    addon.unregister()
print('STANDALONE_PACKAGE_PASS', archive.name, flush=True)
