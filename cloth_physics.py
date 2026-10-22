# SPDX-License-Identifier: GPL-2.0-or-later
"""Native cloth presets for woven bone guides and directly simulated meshes."""
from uuid import uuid4
from . import mesh_physics as mesh, physics_core as core
from . import skeletal_weaver as sw


def configure_cloth(mod, preset='FABRIC'):
    settings = mod.settings
    settings.quality = 12
    settings.mass = .08 if preset == 'HAIR' else .3
    settings.tension_stiffness = settings.compression_stiffness = 40.
    settings.shear_stiffness = 20.
    settings.bending_stiffness = 3. if preset == 'HAIR' else .5
    settings.tension_damping = settings.compression_damping = 8.
    settings.shear_damping = 8.
    settings.bending_damping = 1.
    settings.air_damping = 3.
    collision = mod.collision_settings
    collision.use_collision = True
    collision.collision_quality = 6
    collision.distance_min = .003
    collision.use_self_collision = False


def prepare_cloth(obj, pins, *, preset='FABRIC'):
    insertion = mesh.validate_mesh(obj)
    if not obj.data.polygons:
        raise ValueError('布片需要有面的网格；绳中心线请用链条模式')
    pins = {i: float(w) for i, w in pins.items() if w > 0}
    if not pins or any(i < 0 or i >= len(obj.data.vertices) or w > 1 for i, w in pins.items()):
        raise ValueError('请选择根部固定点，或指定有权重的固定组')
    if len(pins) == len(obj.data.vertices) and all(w >= 1 for w in pins.values()):
        raise ValueError('全部顶点已固定；请只选根部 / 固定边')
    token = uuid4().hex[:8]
    group = obj.vertex_groups.new(name='SCS Cloth Pin ' + token)
    for i, weight in pins.items():
        group.add([i], weight, 'REPLACE')
    mod = obj.modifiers.new('SCS Cloth ' + token, 'CLOTH')
    obj.modifiers.move(list(obj.modifiers).index(mod), insertion)
    configure_cloth(mod, preset)
    mod.settings.vertex_group_mass = group.name
    sw.set_cache_range(mod)
    obj[core.KIND], obj[core.SOLVER], obj[core.GROUPS] = 'CLOTH', mod.name, [group.name]
    return mod
