"""Dense curved rope preparation benchmark; keeps the source Mesh intact."""
from pathlib import Path
import cProfile
import json
import math
import pstats
import sys
import time
import subprocess
import types
import bpy

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from simple_cloth_simulation import region_physics as region, chain_layout as chain, rope_proxy as rope

bpy.ops.wm.read_factory_settings(use_empty=True)
out = Path(__file__).resolve().parents[1] / 'out/unified-physics/dense-regions'
out.mkdir(parents=True, exist_ok=True)
label = sys.argv[sys.argv.index('--') + 1] if '--' in sys.argv else 'current'
if '--baseline-ref' in sys.argv:
    revision = sys.argv[sys.argv.index('--baseline-ref') + 1]
    baseline = subprocess.check_output(['git', 'show', revision + ':region_physics.py'],
                                      cwd=Path(__file__).resolve().parents[1], encoding='utf8')
    region = types.ModuleType('simple_cloth_simulation._benchmark_baseline')
    exec(compile(baseline, 'baseline_region_physics.py', 'exec'), region.__dict__)
rows, sides = 1563, 64
vertices = []
for row in range(rows):
    x = row / (rows - 1)
    for side in range(sides):
        angle = math.tau * side / sides
        vertices.append((x, .05 * math.sin(x * math.pi) + .008 * math.cos(angle),
                         1 + .008 * math.sin(angle)))
faces = [(row * sides + side, row * sides + (side + 1) % sides,
          (row + 1) * sides + (side + 1) % sides, (row + 1) * sides + side)
         for row in range(rows - 1) for side in range(sides)]
data = bpy.data.meshes.new('Dense Rope')
data.from_pydata(vertices, [], faces)
source = bpy.data.objects.new('Dense Rope', data)
bpy.context.collection.objects.link(source)
source.select_set(True)
bpy.context.view_layer.objects.active = source
profile = cProfile.Profile()
started = time.perf_counter()
if '--plain' not in sys.argv:
    profile.enable()
if '--wire' in sys.argv:
    proxy = rope.create(bpy.context, source, count=16)[0]
else:
    proxy = region.create_region(bpy.context, source, range(len(vertices)), count=32,
                                 hook_preset='BOTH_ENDS')
    chain.mark_nodes(proxy)
profile.disable()
elapsed = time.perf_counter() - started
bpy.context.view_layer.update()
evaluated = source.evaluated_get(bpy.context.evaluated_depsgraph_get())
jump = max((v.co - data.vertices[i].co).length for i, v in enumerate(evaluated.data.vertices))
assert jump < 2e-5, jump
report = dict(vertices=len(vertices), faces=len(faces), nodes=len(proxy.data.vertices),
              seconds=elapsed, max_bind_jump=jump, profiled='--plain' not in sys.argv)
(out / (label + '.json')).write_text(json.dumps(report, indent=2))
if '--plain' not in sys.argv:
    with (out / (label + '-profile.txt')).open('w') as stream:
        pstats.Stats(profile, stream=stream).strip_dirs().sort_stats('cumulative').print_stats(25)
print('DENSE_REGION_BENCHMARK', json.dumps(report), flush=True)
