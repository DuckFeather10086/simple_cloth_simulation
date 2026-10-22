# SPDX-License-Identifier: GPL-2.0-or-later

import bpy
import bmesh
from bpy.props import FloatProperty, BoolProperty, PointerProperty, EnumProperty
from bpy.types import Operator, Panel, PropertyGroup
from mathutils import Vector, Matrix
from math import exp
from collections import defaultdict
from uuid import uuid4
from mathutils.kdtree import KDTree

bl_info = {
    "name": "Skeletal Weaver",
    "author": "Duck Feather",
    "version": (2, 4, 0),
    "blender": (5, 0, 0),
    "location": "Pose Mode > Sidebar > Skeletal Weaver",
    "description": "Generates helper meshes (grid/web) based on selected bones in an Armature",
    "warning": "",
    "doc_url": "",
    "category": "Rigging",
}

MESH_GUIDE_NAME = "Skeletal_Mesh_Guide"
WEAVER_PRE_CONSTRAINT_MATRIX = "skeletal_weaver_pre_constraint_matrix_basis"
WEAVER_CONSTRAINT_PREFIX = "SW_DampedTrack"
PIN_FALLOFF_BRUSH_NAME = "SkeletalWeaver_PinFalloff"
HOOK_GROUP_HEAD = "SW_Hook_Head"
HOOK_GROUP_TAIL = "SW_Hook_Tail"
OWNER_KEY = "skeletal_weaver_owner"
ROLE_KEY = "skeletal_weaver_role"
SOURCE_KEY = "skeletal_weaver_source"
BONES_KEY = "skeletal_weaver_bones"
TARGETS_KEY = "skeletal_weaver_targets"


# =============================================================================
# STEP 0: TOPOLOGY CLEANUP & CONSTRAINT REMOVAL
# =============================================================================

def snap_close_bones(selected_bones, snap_threshold=0.001):
    """
    Iterate through selected bones and snap Head/Tail points that are very close.
    If child.head is close to parent.tail, set child.use_connect = True.
    
    Note: This modifies edit bones, so must be called in EDIT mode.
    """
    selected_names = {b.name for b in selected_bones}
    
    for bone in selected_bones:
        if bone.parent and bone.parent.name in selected_names:
            distance = (bone.head - bone.parent.tail).length
            if distance < snap_threshold:
                bone.use_connect = True


def remove_damped_track_constraints(pose_bones):
    """Remove only explicitly identified plugin tracks, including broken targets."""
    count = 0
    for bone in pose_bones:
        owned = [c for c in bone.constraints if is_weaver_damped_track_constraint(bone, c)]
        for constraint in owned:
            bone.constraints.remove(constraint)
            count += 1
        if owned:
            restore_pre_constraint_pose(bone)
    return count


def remove_invalid_constraints(pose_bones):
    """Remove broken plugin tracks; targetless user constraints may be intentional."""
    count = 0
    for bone in pose_bones:
        broken = []
        for c in bone.constraints:
            if not is_weaver_damped_track_constraint(bone, c):
                continue
            target = c.target
            if target is None or (c.subtarget and (
                (target.type == 'MESH' and c.subtarget not in target.vertex_groups) or
                (target.type == 'ARMATURE' and c.subtarget not in target.pose.bones)
            )):
                broken.append(c)
        for c in broken:
            bone.constraints.remove(c)
            count += 1
        if broken and not any(is_weaver_damped_track_constraint(bone, c) for c in bone.constraints):
            restore_pre_constraint_pose(bone)
    return count


def select_bone_chain(armature_obj, start_bones):
    """
    Select all bones in the chain(s) starting from the given bones.
    Only traces downward through connected children.
    
    Returns the set of all selected bone names.
    """
    all_bones = set()
    
    for start_bone in start_bones:
        # Add the start bone itself
        all_bones.add(start_bone.name)
        
        # Trace downward through all connected children
        def trace_children(bone):
            for child in bone.children:
                if child.bone.use_connect:
                    all_bones.add(child.name)
                    trace_children(child)
        
        trace_children(start_bone)
    
    # Select the bones
    for bone in armature_obj.pose.bones:
        is_selected = bone.name in all_bones
        if hasattr(bone, "select"):
            # Blender 5.0+: bone selection lives on PoseBone.
            bone.select = is_selected
        else:
            # Blender <= 4.x: selection is on Bone.
            bone.bone.select = is_selected
    
    return all_bones


# =============================================================================
# MESH GROUP IDENTIFICATION
# =============================================================================

def identify_mesh_groups(selected_bones, armature_obj):
    """Weave sibling chains sharing an anchor; split at gaps and forks.

    A connected linear child continues its parent's column. At a fork all
    selected children start a new sheet anchored to that junction bone.
    Unselected ancestors stay anchors, never implicit simulation input.
    """
    names = {b.name for b in selected_bones}
    roots_by_parent = defaultdict(list)
    for bone in selected_bones:
        parent = bone.parent
        siblings = connected_children(parent, names) if parent else []
        if (parent is None or parent.name not in names or
                not bone.bone.use_connect or len(siblings) != 1):
            roots_by_parent[parent.name if parent else None].append(bone)
    groups = []
    for parent_name, roots in roots_by_parent.items():
        bones = set()
        for root in roots:
            bones.update(trace_chain(root, selected_bones))
        groups.append({'chain_roots': roots,
                       'parent_bone': armature_obj.pose.bones.get(parent_name) if parent_name else None,
                       'all_bones': bones})
    covered = [b.name for group in groups for b in group['all_bones']]
    if len(covered) != len(names) or set(covered) != names:
        raise ValueError("Bone grouping did not cover each selected bone exactly once")
    return groups


def connected_children(bone, selected_names):
    return [child for child in bone.children
            if child.name in selected_names and child.bone.use_connect]


def trace_chain_bones(start_bone, selected_bones):
    return set(trace_chain(start_bone, selected_bones))


# =============================================================================
# MATRIX BUILDING
# =============================================================================

def trace_chain(start_bone, selected_bones):
    names = {bone.name for bone in selected_bones}
    chain = [start_bone]
    while True:
        children = connected_children(chain[-1], names)
        if len(children) != 1:
            return chain
        chain.append(children[0])


def sort_chain_starts_by_x(chain_starts, armature_obj):
    """
    Sort chain starting bones by their local X coordinate.
    """
    return sorted(chain_starts, key=lambda b: b.bone.head_local.x)


def build_bone_matrix_for_group(mesh_group, armature_obj, selected_bones):
    """
    Build a 2D matrix bone_matrix[col][row] for a specific mesh group.
    
    Returns:
        - bone_matrix: 2D list where bone_matrix[col][row] is a bone or None
        - max_rows: Maximum number of rows
        - max_cols: Number of columns
    """
    chain_roots = mesh_group['chain_roots']
    
    if not chain_roots:
        return None, 0, 0
    
    chain_starts = sort_chain_starts_by_x(chain_roots, armature_obj)
    
    chains = []
    for start in chain_starts:
        chain = trace_chain(start, selected_bones)
        chains.append(chain)
    
    max_rows = max(len(chain) for chain in chains) if chains else 0
    max_cols = len(chains)
    
    bone_matrix = []
    for chain in chains:
        column = []
        for row_idx in range(max_rows):
            if row_idx < len(chain):
                column.append(chain[row_idx])
            else:
                column.append(None)
        bone_matrix.append(column)
    
    return bone_matrix, max_rows, max_cols


# =============================================================================
# COORDINATE EXTRACTION
# =============================================================================

def extract_coordinate_matrix(bone_matrix, armature_obj, max_rows, max_cols):
    """
    Extract world-space head coordinates for each bone in the matrix.
    Also extract the tail of the last bone in each chain immediately after the last head.
    
    Returns:
        - coord_matrix[col][row] = Vector or None
        - The matrix has max_rows + 1 rows (to include tail of last bones)
    """
    coord_matrix = []
    total_rows = max_rows + 1
    
    for col_idx in range(max_cols):
        column = []
        last_valid_bone = None
        
        for row_idx in range(max_rows):
            bone = bone_matrix[col_idx][row_idx]
            if bone is not None:
                head_world = armature_obj.matrix_world @ bone.head
                column.append(head_world.copy())
                last_valid_bone = bone
            else:
                break
        
        if last_valid_bone is not None:
            tail_world = armature_obj.matrix_world @ last_valid_bone.tail
            column.append(tail_world.copy())
        
        while len(column) < total_rows:
            column.append(None)
        
        coord_matrix.append(column)
    
    return coord_matrix


# =============================================================================
# MESH GENERATION
# =============================================================================

def ribbon_frames(coords, bone_matrix, armature_obj):
    valid = [co for co in coords if co is not None]
    first = bone_matrix[0][0]
    axis = (armature_obj.matrix_world @ first.matrix).to_3x3() @ Vector((1, 0, 0))
    frames = []
    previous_tangent = None
    for row in range(len(valid)):
        before = valid[max(0, row - 1)]
        after = valid[min(len(valid) - 1, row + 1)]
        tangent = after - before
        if tangent.length < 1e-8:
            tangent = valid[row] - before if row else after - valid[row]
        if tangent.length < 1e-8:
            raise ValueError("Zero-length or folded-back bone segment")
        tangent.normalize()
        if previous_tangent is not None:
            axis = previous_tangent.rotation_difference(tangent) @ axis
        axis -= tangent * axis.dot(tangent)
        if axis.length < 1e-8:
            reference = min((Vector((1, 0, 0)), Vector((0, 1, 0)), Vector((0, 0, 1))),
                            key=lambda v: abs(v.dot(tangent)))
            axis = reference - tangent * reference.dot(tangent)
        axis.normalize()
        frames.append((axis.copy(), axis.cross(tangent).normalized()))
        previous_tangent = tangent
    return frames


def create_ribbon_mesh(coord_matrix, bone_matrix, max_rows, mesh_name, ribbon_width, armature_obj):
    """
    Create a ribbon (thin rectangular strip) mesh for single-column chains.
    The ribbon is created by offsetting vertices perpendicular to the bone direction.
    
    Returns the created mesh object and coordinate vertex matrix.
    """
    mesh = bpy.data.meshes.new(mesh_name + "_mesh")
    bm = bmesh.new()
    
    total_rows = max_rows + 1
    # For ribbon: 2 columns (left and right offset)
    vert_matrix = [[None for _ in range(total_rows)] for _ in range(2)]
    
    # Get the single column coordinates
    coords = coord_matrix[0]
    
    frames = ribbon_frames(coords, bone_matrix, armature_obj)

    half_width = ribbon_width / 2.0
    
    # Create vertices with offset
    for row in range(total_rows):
        coord = coords[row]
        if coord is not None:
            # Left vertex (negative offset)
            left_pos = coord - frames[row][0] * half_width
            vert_left = bm.verts.new(left_pos)
            vert_matrix[0][row] = vert_left
            
            # Right vertex (positive offset)
            right_pos = coord + frames[row][0] * half_width
            vert_right = bm.verts.new(right_pos)
            vert_matrix[1][row] = vert_right
    
    bm.verts.ensure_lookup_table()
    
    # Create edges along both sides
    for col in range(2):
        for row in range(total_rows - 1):
            v1 = vert_matrix[col][row]
            v2 = vert_matrix[col][row + 1]
            if v1 is not None and v2 is not None:
                try:
                    bm.edges.new((v1, v2))
                except ValueError:
                    pass
    
    # Create horizontal edges and faces
    for row in range(total_rows):
        v_left = vert_matrix[0][row]
        v_right = vert_matrix[1][row]
        if v_left is not None and v_right is not None:
            try:
                bm.edges.new((v_left, v_right))
            except ValueError:
                pass
    
    # Create quad faces
    for row in range(total_rows - 1):
        v_tl = vert_matrix[0][row]
        v_tr = vert_matrix[1][row]
        v_bl = vert_matrix[0][row + 1]
        v_br = vert_matrix[1][row + 1]
        
        if all(v is not None for v in [v_tl, v_tr, v_bl, v_br]):
            try:
                bm.faces.new((v_tl, v_tr, v_br, v_bl))
            except ValueError:
                pass
    
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    
    # Extract coordinates before freeing
    coord_vert_matrix = [[None for _ in range(total_rows)] for _ in range(2)]
    for col in range(2):
        for row in range(total_rows):
            if vert_matrix[col][row] is not None:
                coord_vert_matrix[col][row] = vert_matrix[col][row].co.copy()
    
    bm.to_mesh(mesh)
    bm.free()
    mesh.update()
    
    obj = bpy.data.objects.new(mesh_name, mesh)
    bpy.context.collection.objects.link(obj)
    
    # Return with expanded bone_matrix for vertex group creation
    # Duplicate the single column for both sides
    expanded_bone_matrix = [bone_matrix[0], bone_matrix[0]]
    
    return obj, coord_vert_matrix, expanded_bone_matrix, 2


def create_extruded_ribbon_mesh(
    coord_matrix, bone_matrix, max_rows, mesh_name, ribbon_width, armature_obj
):
    """
    Same strip as create_ribbon_mesh, then extruded along face normals by ribbon_width
    so the cross-section is square (width × thickness).
    Returns mesh object, coord_vert_matrix (front shell coords), expanded bone matrix,
    column count, and vert_indices_matrix[col][row] = list of mesh vertex indices
    (front then back) for explicit weight assignment.
    """
    mesh = bpy.data.meshes.new(mesh_name + "_mesh")
    bm = bmesh.new()

    total_rows = max_rows + 1
    front = [[None for _ in range(total_rows)] for _ in range(2)]
    coords = coord_matrix[0]

    frames = ribbon_frames(coords, bone_matrix, armature_obj)

    half_width = ribbon_width / 2.0

    for row in range(total_rows):
        coord = coords[row]
        if coord is not None:
            front[0][row] = bm.verts.new(coord - frames[row][0] * half_width - frames[row][1] * half_width)
            front[1][row] = bm.verts.new(coord + frames[row][0] * half_width - frames[row][1] * half_width)

    bm.verts.ensure_lookup_table()

    back = [[None for _ in range(total_rows)] for _ in range(2)]
    for row in range(total_rows):
        for col in range(2):
            if front[col][row] is not None:
                back[col][row] = bm.verts.new(
                    front[col][row].co + frames[row][1] * ribbon_width
                )

    bm.verts.ensure_lookup_table()

    for row in range(total_rows - 1):
        for col in range(2):
            v1 = front[col][row]
            v2 = front[col][row + 1]
            if v1 is not None and v2 is not None:
                try:
                    bm.edges.new((v1, v2))
                except ValueError:
                    pass

    for row in range(total_rows - 1):
        for col in range(2):
            v1 = back[col][row]
            v2 = back[col][row + 1]
            if v1 is not None and v2 is not None:
                try:
                    bm.edges.new((v1, v2))
                except ValueError:
                    pass

    for row in range(total_rows):
        v_l = front[0][row]
        v_r = front[1][row]
        if v_l is not None and v_r is not None:
            try:
                bm.edges.new((v_l, v_r))
            except ValueError:
                pass

    for row in range(total_rows):
        v_l = back[0][row]
        v_r = back[1][row]
        if v_l is not None and v_r is not None:
            try:
                bm.edges.new((v_l, v_r))
            except ValueError:
                pass

    for row in range(total_rows - 1):
        v_tl = front[0][row]
        v_tr = front[1][row]
        v_bl = front[0][row + 1]
        v_br = front[1][row + 1]
        if all(v is not None for v in (v_tl, v_tr, v_bl, v_br)):
            try:
                bm.faces.new((v_tl, v_tr, v_br, v_bl))
            except ValueError:
                pass

    for row in range(total_rows - 1):
        v_tl = back[0][row + 1]
        v_tr = back[1][row + 1]
        v_bl = back[0][row]
        v_br = back[1][row]
        if all(v is not None for v in (v_tl, v_tr, v_bl, v_br)):
            try:
                bm.faces.new((v_tl, v_tr, v_br, v_bl))
            except ValueError:
                pass

    for row in range(total_rows - 1):
        v_fl = front[0][row]
        v_fr = front[0][row + 1]
        v_bl = back[0][row]
        v_br = back[0][row + 1]
        if all(v is not None for v in (v_fl, v_fr, v_br, v_bl)):
            try:
                bm.faces.new((v_fl, v_fr, v_br, v_bl))
            except ValueError:
                pass

        v_fl = front[1][row + 1]
        v_fr = front[1][row]
        v_bl = back[1][row + 1]
        v_br = back[1][row]
        if all(v is not None for v in (v_fl, v_fr, v_br, v_bl)):
            try:
                bm.faces.new((v_fl, v_fr, v_br, v_bl))
            except ValueError:
                pass

    first_row = None
    last_row = None
    for row in range(total_rows):
        if coords[row] is not None:
            if first_row is None:
                first_row = row
            last_row = row

    if first_row is not None and last_row is not None:
        f0 = front[0][first_row]
        f1 = front[1][first_row]
        b0 = back[0][first_row]
        b1 = back[1][first_row]
        if all(v is not None for v in (f0, f1, b1, b0)):
            try:
                bm.faces.new((f0, f1, b1, b0))
            except ValueError:
                pass

        f0 = front[0][last_row]
        f1 = front[1][last_row]
        b0 = back[0][last_row]
        b1 = back[1][last_row]
        if all(v is not None for v in (f0, b0, b1, f1)):
            try:
                bm.faces.new((f0, b0, b1, f1))
            except ValueError:
                pass

    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])

    coord_vert_matrix = [[None for _ in range(total_rows)] for _ in range(2)]
    for col in range(2):
        for row in range(total_rows):
            if front[col][row] is not None:
                coord_vert_matrix[col][row] = front[col][row].co.copy()

    bm.verts.ensure_lookup_table()
    bm.verts.index_update()
    vert_indices_matrix = [[[] for _ in range(total_rows)] for _ in range(2)]
    for col in range(2):
        for row in range(total_rows):
            if front[col][row] is not None:
                vert_indices_matrix[col][row].append(front[col][row].index)
            if back[col][row] is not None:
                vert_indices_matrix[col][row].append(back[col][row].index)

    bm.to_mesh(mesh)
    bm.free()
    mesh.update()

    obj = bpy.data.objects.new(mesh_name, mesh)
    bpy.context.collection.objects.link(obj)
    expanded_bone_matrix = [bone_matrix[0], bone_matrix[0]]
    return obj, coord_vert_matrix, expanded_bone_matrix, 2, vert_indices_matrix


def create_woven_mesh(coord_matrix, bone_matrix, max_rows, max_cols, mesh_name):
    """
    Generate mesh using BMesh with proper weaving logic.
    
    Returns the created mesh object and coordinate vertex matrix.
    """
    mesh = bpy.data.meshes.new(mesh_name + "_mesh")
    bm = bmesh.new()
    
    total_rows = max_rows + 1
    vert_matrix = [[None for _ in range(total_rows)] for _ in range(max_cols)]
    
    # VERTEX GENERATION
    for col in range(max_cols):
        for row in range(total_rows):
            coord = coord_matrix[col][row]
            if coord is not None:
                vert = bm.verts.new(coord)
                vert_matrix[col][row] = vert
    
    bm.verts.ensure_lookup_table()
    
    # VERTICAL EDGES (within each column)
    for col in range(max_cols):
        for row in range(total_rows - 1):
            v1 = vert_matrix[col][row]
            v2 = vert_matrix[col][row + 1]
            if v1 is not None and v2 is not None:
                try:
                    bm.edges.new((v1, v2))
                except ValueError:
                    pass
    
    bm.edges.ensure_lookup_table()
    
    # HORIZONTAL EDGES & FACES (between adjacent columns)
    for col in range(max_cols - 1):
        col_left = col
        col_right = col + 1
        
        last_valid_left = -1
        last_valid_right = -1
        
        for row in range(total_rows):
            if vert_matrix[col_left][row] is not None:
                last_valid_left = row
            if vert_matrix[col_right][row] is not None:
                last_valid_right = row
        
        for row in range(total_rows - 1):
            v_tl = vert_matrix[col_left][row]
            v_bl = vert_matrix[col_left][row + 1]
            v_tr = vert_matrix[col_right][row]
            v_br = vert_matrix[col_right][row + 1]
            
            valid_verts = [v for v in [v_tl, v_bl, v_tr, v_br] if v is not None]
            
            if len(valid_verts) == 4:
                try:
                    bm.faces.new((v_tl, v_tr, v_br, v_bl))
                except ValueError:
                    pass
            elif len(valid_verts) == 3:
                try:
                    bm.faces.new(valid_verts)
                except ValueError:
                    pass
        
        if last_valid_left != last_valid_right:
            if last_valid_left < last_valid_right:
                last_left_vert = vert_matrix[col_left][last_valid_left]
                if last_left_vert is not None:
                    for row in range(last_valid_left, last_valid_right):
                        v_r1 = vert_matrix[col_right][row]
                        v_r2 = vert_matrix[col_right][row + 1]
                        if v_r1 is not None and v_r2 is not None:
                            try:
                                bm.edges.new((last_left_vert, v_r1))
                            except ValueError:
                                pass
                            try:
                                bm.faces.new((last_left_vert, v_r1, v_r2))
                            except ValueError:
                                pass
            else:
                last_right_vert = vert_matrix[col_right][last_valid_right]
                if last_right_vert is not None:
                    for row in range(last_valid_right, last_valid_left):
                        v_l1 = vert_matrix[col_left][row]
                        v_l2 = vert_matrix[col_left][row + 1]
                        if v_l1 is not None and v_l2 is not None:
                            try:
                                bm.edges.new((last_right_vert, v_l1))
                            except ValueError:
                                pass
                            try:
                                bm.faces.new((last_right_vert, v_l2, v_l1))
                            except ValueError:
                                pass
    
    # HORIZONTAL EDGES (connect row 0 across columns)
    for col in range(max_cols - 1):
        v1 = vert_matrix[col][0]
        v2 = vert_matrix[col + 1][0]
        if v1 is not None and v2 is not None:
            try:
                bm.edges.new((v1, v2))
            except ValueError:
                pass
    
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    
    coord_vert_matrix = [[None for _ in range(total_rows)] for _ in range(max_cols)]
    for col in range(max_cols):
        for row in range(total_rows):
            if vert_matrix[col][row] is not None:
                coord_vert_matrix[col][row] = vert_matrix[col][row].co.copy()
    
    bmesh.ops.remove_doubles(bm, verts=bm.verts[:], dist=0.0001)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    bm.to_mesh(mesh)
    bm.free()
    mesh.update()
    
    obj = bpy.data.objects.new(mesh_name, mesh)
    bpy.context.collection.objects.link(obj)
    
    return obj, coord_vert_matrix


# =============================================================================
# VERTEX GROUPS & PARENTING
# =============================================================================

def evaluate_pin_falloff(
    normalized_distance, pattern, strength, min_weight, custom_curve_mapping=None
):
    """
    Compute pin weight where 0 = chain top, 1 = chain bottom.
    """
    t = max(0.0, min(1.0, normalized_distance))
    k = max(0.01, strength)

    if pattern == 'LINEAR':
        base_weight = 1.0 - t
    elif pattern == 'EXPONENTIAL':
        start = 1.0
        end = exp(-k)
        value = exp(-k * t)
        base_weight = (value - end) / (start - end) if start != end else 0.0
    elif pattern == 'ROOT':
        base_weight = 1.0 - (t ** 0.5)
    elif pattern == 'CUSTOM':
        if custom_curve_mapping is not None:
            base_weight = custom_curve_mapping.evaluate(custom_curve_mapping.curves[0], t)
        else:
            base_weight = 1.0 - t
    else:
        # Default: inverse-style drop (fast near top, slower near bottom).
        start = 1.0
        end = 1.0 / (1.0 + k)
        value = 1.0 / (1.0 + (k * t))
        base_weight = (value - end) / (start - end) if start != end else 0.0

    base_weight = max(0.0, min(1.0, base_weight))
    return min_weight + ((1.0 - min_weight) * base_weight)


def get_or_create_pin_falloff_brush():
    """
    Returns a dedicated hidden brush that stores GUI-editable falloff curve data.
    """
    brush = bpy.data.brushes.get(PIN_FALLOFF_BRUSH_NAME)
    if brush is None:
        brush = bpy.data.brushes.new(PIN_FALLOFF_BRUSH_NAME, mode='SCULPT')
        brush.use_fake_user = True
    return brush


def create_vertex_groups(
    mesh_obj, coord_vert_matrix, bone_matrix, max_rows, max_cols,
    pin_use_falloff=True,
    pin_falloff_pattern='INVERSE', pin_falloff_strength=4.0, pin_min_weight=0.05,
    pin_custom_curve_mapping=None,
    pin_group_name="Pin",
    vert_indices_matrix=None,
):
    """
    Create vertex groups for rigging (Pin/Goal mass + per-bone groups).
    pin_use_falloff: if True, pin weight uses falloff or TOP_ONLY pattern; if False,
    only top row = 1, rest = 0 (free swing), ignoring pattern.
    vert_indices_matrix: optional [col][row] -> list of mesh vertex indices (for extruded meshes).
    """
    total_rows = max_rows + 1

    pin_group = mesh_obj.vertex_groups.new(name=pin_group_name)
    mesh = mesh_obj.data
    pin_weights_by_vert = {}
    bone_assignment_by_vert = defaultdict(set)

    use_custom_curve = (
        pin_use_falloff
        and pin_falloff_pattern == 'CUSTOM'
        and pin_custom_curve_mapping is not None
    )
    if use_custom_curve:
        pin_custom_curve_mapping.initialize()

    tree = None
    if vert_indices_matrix is None:
        tree = KDTree(len(mesh.vertices))
        for vert in mesh.vertices:
            tree.insert(vert.co, vert.index)
        tree.balance()

    def pin_weight_for_row(row, distances):
        if not pin_use_falloff or pin_falloff_pattern == 'TOP_ONLY':
            return 1.0 if row == 0 else 0.0
        normalized_distance = distances[row] / distances[-1] if distances[-1] else 0.0
        return evaluate_pin_falloff(normalized_distance, pin_falloff_pattern,
                                    pin_falloff_strength, pin_min_weight, pin_custom_curve_mapping)

    for col in range(max_cols):
        valid_coords = [co for co in coord_vert_matrix[col] if co is not None]
        distances = [0.0]
        for a, b in zip(valid_coords, valid_coords[1:]):
            distances.append(distances[-1] + (b - a).length)
        for row in range(total_rows):
            if vert_indices_matrix is not None:
                idxs = vert_indices_matrix[col][row]
                if not idxs:
                    continue
            else:
                orig_coord = coord_vert_matrix[col][row]
                if orig_coord is None:
                    continue
                _, index, distance = tree.find(orig_coord)
                if distance > 0.00011:
                    raise ValueError("Cannot map a guide vertex after welding")
                idxs = [index]

            pin_weight = pin_weight_for_row(row, distances)
            for vert_idx in idxs:
                pin_weights_by_vert[vert_idx] = max(
                    pin_weights_by_vert.get(vert_idx, 0.0), pin_weight
                )

                if row > 0:
                    prev_row = row - 1
                    bone = None
                    if prev_row < max_rows:
                        bone = bone_matrix[col][prev_row]
                    if bone is None:
                        for r in range(prev_row - 1, -1, -1):
                            if r < max_rows:
                                bone = bone_matrix[col][r]
                                if bone is not None:
                                    break
                    if bone is not None:
                        bone_assignment_by_vert[vert_idx].add(bone.name)

    for vert_idx, weight in pin_weights_by_vert.items():
        pin_group.add([vert_idx], weight, 'REPLACE')

    target_groups = {}
    for vert_idx, group_names in bone_assignment_by_vert.items():
        for bone_name in sorted(group_names):
            if bone_name not in target_groups:
                target_groups[bone_name] = mesh_obj.vertex_groups.new(name=bone_name)
            target_groups[bone_name].add([vert_idx], 1.0, 'REPLACE')
    mesh_obj[TARGETS_KEY] = {name: group.name for name, group in target_groups.items()}


def add_cloth_modifier(mesh_obj):
    """
    Add cloth modifier to mesh with Pin vertex group.
    """
    cloth_mod = mesh_obj.modifiers.new("Cloth", 'CLOTH')
    cloth_mod.settings.vertex_group_mass = "Pin"
    set_cache_range(cloth_mod)
    return cloth_mod


def _set_softbody_goal_vertex_group(settings, group_name):
    if hasattr(settings, "vertex_group_goal"):
        settings.vertex_group_goal = group_name
    elif hasattr(settings, "goal_vertex_group"):
        settings.goal_vertex_group = group_name


def add_softbody_modifier(mesh_obj, goal_group_name="Goal"):
    """
    Soft body with Goal vertex group; goal spring stiffness at API maximum (~1).
    """
    mod = mesh_obj.modifiers.new("Softbody", 'SOFT_BODY')
    settings = mod.settings
    _set_softbody_goal_vertex_group(settings, goal_group_name)
    if hasattr(settings, "goal_spring"):
        settings.goal_spring = 0.999
    settings.use_goal = True
    settings.goal_min = 0.0
    settings.goal_max = 1.0
    set_cache_range(mod)
    return mod


def set_cache_range(mod):
    mod.point_cache.frame_start = bpy.context.scene.frame_start
    mod.point_cache.frame_end = bpy.context.scene.frame_end


def _unique_obj_name(prefix):
    name = prefix
    n = 1
    while name in bpy.data.objects:
        name = f"{prefix}.{n:03d}"
        n += 1
    return name


def _median_world_from_indices(mesh_obj, indices):
    me = mesh_obj.data
    acc = Vector()
    for i in indices:
        acc += mesh_obj.matrix_world @ me.vertices[i].co
    return acc / len(indices)


def setup_softbody_hook_presets(mesh_obj, preset, vert_indices_matrix):
    if preset == 'NONE' or not vert_indices_matrix:
        return
    if preset in ('BOTH_ENDS', 'HEAD_ONLY'):
        indices = [i for column in vert_indices_matrix for i in column[0]]
        create_hook(mesh_obj, indices, HOOK_GROUP_HEAD)
    if preset in ('BOTH_ENDS', 'TAIL_ONLY'):
        indices = [i for column in vert_indices_matrix for i in column[-1]]
        create_hook(mesh_obj, indices, HOOK_GROUP_TAIL)


def create_hook(mesh_obj, indices, group_name):
    """Bind a control before physics and make it follow the same upstream anchor."""
    indices = sorted(set(indices))
    if not indices:
        raise ValueError("No vertices selected")
    physics = next((m for m in mesh_obj.modifiers if m.type in {'CLOTH', 'SOFT_BODY'}), None)
    pin_name = ('Goal' if physics.type == 'SOFT_BODY' else 'Pin') if physics else 'Goal'
    goal = mesh_obj.vertex_groups.get(pin_name) or mesh_obj.vertex_groups.new(name=pin_name)
    goal.add(indices, 1.0, 'REPLACE')
    vg = mesh_obj.vertex_groups.new(name=group_name)
    vg.add(indices, 1.0, 'REPLACE')
    bpy.context.view_layer.update()
    empty = bpy.data.objects.new(_unique_obj_name('SW_Hook_Empty'), None)
    bpy.context.collection.objects.link(empty)
    empty.empty_display_type = 'PLAIN_AXES'
    empty.empty_display_size = max(mesh_obj.dimensions.length * 0.05, 0.01)
    empty[OWNER_KEY] = mesh_obj.get(OWNER_KEY, '')
    empty[ROLE_KEY] = 'HOOK'
    set_parent_keep_world(empty, mesh_obj.parent, mesh_obj.parent_type, mesh_obj.parent_bone,
                          Matrix.Translation(_median_world_from_indices(mesh_obj, indices)))
    hook = mesh_obj.modifiers.new(group_name, 'HOOK')
    hook.object = empty
    hook.vertex_group = vg.name
    hook.matrix_inverse = empty.matrix_world.inverted() @ mesh_obj.matrix_world
    if physics:
        mesh_obj.modifiers.move(len(mesh_obj.modifiers) - 1, list(mesh_obj.modifiers).index(physics))
    return empty, hook


def add_bone_constraints(mesh_obj, bone_matrix, max_rows, max_cols):
    """
    Add DAMPED_TRACK constraints to bones targeting their vertex groups.
    """
    added_constraints = []
    processed_bones = set()
    
    for col in range(max_cols):
        for row in range(max_rows):
            bone = bone_matrix[col][row]
            if bone is not None and bone.name not in processed_bones:
                processed_bones.add(bone.name)
                
                # Check if vertex group exists for this bone
                group_name = mesh_obj.get(TARGETS_KEY, {}).get(bone.name, bone.name)
                if group_name in mesh_obj.vertex_groups:
                    store_pre_constraint_pose(bone)

                    # Add DAMPED_TRACK constraint
                    constraint = bone.constraints.new('DAMPED_TRACK')
                    constraint.target = mesh_obj
                    constraint.subtarget = group_name
                    constraint.track_axis = 'TRACK_Y'
                    constraint.name = f"{WEAVER_CONSTRAINT_PREFIX}_{uuid4().hex[:16]}"
                    
                    added_constraints.append((bone.name, constraint.name))
    
    return added_constraints


def set_parent_keep_world(obj, parent, parent_type='OBJECT', parent_bone='', world=None):
    world = obj.matrix_world.copy() if world is None else world.copy()
    obj.parent = parent
    obj.parent_type = parent_type
    obj.parent_bone = parent_bone
    if parent is None:
        obj.matrix_world = world
        bpy.context.view_layer.update()
        return
    obj.matrix_parent_inverse = Matrix.Identity(4)
    obj.matrix_basis = Matrix.Identity(4)
    bpy.context.view_layer.update()
    # Sample Blender's actual posed bone-parent transform (tail offset included).
    # Store the full inverse, avoiding lossy TRS decomposition under nonuniform scale.
    parent_world = obj.matrix_world.copy()
    obj.matrix_parent_inverse = parent_world.inverted() @ world
    bpy.context.view_layer.update()


def parent_mesh_to_armature(mesh_obj, armature_obj, parent_bone):
    set_parent_keep_world(mesh_obj, armature_obj, 'BONE' if parent_bone else 'OBJECT',
                          parent_bone.name if parent_bone else '')


def _flatten_matrix4x4(matrix):
    return [matrix[r][c] for r in range(4) for c in range(4)]


def _matrix4x4_from_flat(values):
    return Matrix((
        (values[0], values[1], values[2], values[3]),
        (values[4], values[5], values[6], values[7]),
        (values[8], values[9], values[10], values[11]),
        (values[12], values[13], values[14], values[15]),
    ))


def store_pre_constraint_pose(pose_bone):
    """
    Save pose bone matrix_basis before Skeletal Weaver constraints are applied.
    """
    if WEAVER_PRE_CONSTRAINT_MATRIX in pose_bone:
        return
    pose_bone[WEAVER_PRE_CONSTRAINT_MATRIX] = _flatten_matrix4x4(pose_bone.matrix_basis.copy())


def restore_pre_constraint_pose(pose_bone):
    """
    Restore matrix_basis saved by store_pre_constraint_pose().
    """
    if WEAVER_PRE_CONSTRAINT_MATRIX not in pose_bone:
        return False

    matrix_data = list(pose_bone[WEAVER_PRE_CONSTRAINT_MATRIX])
    del pose_bone[WEAVER_PRE_CONSTRAINT_MATRIX]

    if len(matrix_data) != 16:
        return False

    pose_bone.matrix_basis = _matrix4x4_from_flat(matrix_data)
    return True


def is_weaver_damped_track_constraint(pose_bone, constraint, guide_object_names=None):
    """Reserved names are the ownership marker (Constraint has no ID properties).

    Old SW_DampedTrack_<bone> constraints use the same namespace. Do not infer
    ownership just from target/subtarget: users may track our guides themselves.
    """
    return constraint.type == 'DAMPED_TRACK' and constraint.name.startswith(WEAVER_CONSTRAINT_PREFIX + '_')


def delete_guide_objects(objects):
    owners = {obj.get(OWNER_KEY) for obj in objects if obj.get(OWNER_KEY)}
    hooks = {m.object for obj in objects for m in obj.modifiers
             if m.type == 'HOOK' and m.object and
             (m.object.get(ROLE_KEY) == 'HOOK' or m.object.name.startswith('SW_Hook_Empty'))}
    for obj in list(bpy.data.objects):
        if obj.get(ROLE_KEY) == 'HOOK' and obj.get(OWNER_KEY) in owners:
            hooks.add(obj)
    for obj in objects:
        data = obj.data
        bpy.data.objects.remove(obj, do_unlink=True)
        if isinstance(data, bpy.types.Mesh) and data.users == 0:
            bpy.data.meshes.remove(data)
    for hook in hooks:
        # A user may have deliberately shared the controller with another guide.
        if not any(m.type == 'HOOK' and m.object == hook
                   for obj in bpy.data.objects for m in obj.modifiers):
            bpy.data.objects.remove(hook, do_unlink=True)


def is_guide_object(obj):
    return obj.type == 'MESH' and (obj.get(ROLE_KEY) == 'GUIDE' or
        (obj.name.startswith(MESH_GUIDE_NAME) and ('Pin' in obj.vertex_groups or 'Goal' in obj.vertex_groups)))


def cleanup_previous_meshes():
    objects = [obj for obj in bpy.data.objects if is_guide_object(obj)]
    removed = restored = 0
    for armature in bpy.data.objects:
        if armature.type != 'ARMATURE' or armature.pose is None:
            continue
        for bone in armature.pose.bones:
            owned = [c for c in bone.constraints if is_weaver_damped_track_constraint(bone, c)]
            for c in owned:
                bone.constraints.remove(c)
                removed += 1
            if owned and restore_pre_constraint_pose(bone):
                restored += 1
    delete_guide_objects(objects)
    return len(objects), removed, restored


# =============================================================================
# MAIN OPERATOR
# =============================================================================

def run_generation(operator, context, generate, *, snap_bones=False):
    armature = context.active_object
    selected = list(context.selected_pose_bones or [])
    if not selected:
        operator.report({'WARNING'}, "Select bones in Pose Mode")
        return {'CANCELLED'}
    selection_before = [obj.name for obj in context.selected_objects]
    objects_before = set(bpy.data.objects)
    meshes_before = set(bpy.data.meshes)
    old_tracks = [(b, c, c.mute) for b in selected for c in b.constraints
                  if is_weaver_damped_track_constraint(b, c)]
    old_targets = {c.target for _, c, _ in old_tracks if c.target and is_guide_object(c.target)}
    # Guide-only builds have no track from which to discover the previous result.
    names = {b.name for b in selected}
    old_targets.update(obj for obj in bpy.data.objects
                       if is_guide_object(obj) and obj.get(SOURCE_KEY) == armature
                       and obj.get(BONES_KEY) and set(obj[BONES_KEY]).issubset(names))
    # Keep shared guides if unselected bones or other objects still use them.
    old_track_ptrs = {c.as_pointer() for _, c, _ in old_tracks}
    replaceable = {obj for obj in old_targets if not obj.children and not any(
        getattr(c, 'target', None) == obj and c.as_pointer() not in old_track_ptrs
        for source in bpy.data.objects
        for c in list(source.constraints) + ([c for b in source.pose.bones for c in b.constraints]
                                              if source.type == 'ARMATURE' else []))}
    pose_before = {b.name: b.matrix_basis.copy() for b in armature.pose.bones}
    stored_before = {b.name: list(b[WEAVER_PRE_CONSTRAINT_MATRIX])
                     for b in selected if WEAVER_PRE_CONSTRAINT_MATRIX in b}
    rest_before = {b.name: (b.head_local.copy(), b.tail_local.copy(), b.use_connect)
                   for b in armature.data.bones}
    constraints_before = {c.as_pointer() for b in armature.pose.bones for c in b.constraints}
    try:
        for _, c, _ in old_tracks:
            c.mute = True
        context.view_layer.update()
        result = generate(context)
        if result != {'FINISHED'}:
            raise ValueError("No simulation generated")
    except Exception as exc:
        if context.object and context.object.mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')
        for b in armature.pose.bones:
            for c in list(b.constraints):
                if c.as_pointer() not in constraints_before:
                    b.constraints.remove(c)
        for obj in set(bpy.data.objects) - objects_before:
            bpy.data.objects.remove(obj, do_unlink=True)
        for mesh in set(bpy.data.meshes) - meshes_before:
            if mesh.users == 0:
                bpy.data.meshes.remove(mesh)
        context.view_layer.objects.active = armature
        armature.select_set(True)
        if snap_bones:
            bpy.ops.object.mode_set(mode='EDIT')
            for b in armature.data.edit_bones:
                b.use_connect = False
            for b in armature.data.edit_bones:
                b.head, b.tail, _ = rest_before[b.name]
            for b in armature.data.edit_bones:
                b.use_connect = rest_before[b.name][2]
            bpy.ops.object.mode_set(mode='OBJECT')
        for b in armature.pose.bones:
            b.matrix_basis = pose_before[b.name]
        for b in selected:
            if b.name in stored_before:
                b[WEAVER_PRE_CONSTRAINT_MATRIX] = stored_before[b.name]
            elif WEAVER_PRE_CONSTRAINT_MATRIX in b:
                del b[WEAVER_PRE_CONSTRAINT_MATRIX]
        for _, c, mute in old_tracks:
            c.mute = mute
        bpy.ops.object.mode_set(mode='POSE')
        for name in selection_before:
            obj = bpy.data.objects.get(name)
            if obj is not None:
                obj.select_set(True)
        operator.report({'ERROR'}, str(exc))
        return {'CANCELLED'}
    for b, c, _ in old_tracks:
        b.constraints.remove(c)
    for bone in selected:
        if (WEAVER_PRE_CONSTRAINT_MATRIX in bone and
                not any(is_weaver_damped_track_constraint(bone, c) for c in bone.constraints)):
            del bone[WEAVER_PRE_CONSTRAINT_MATRIX]
    delete_guide_objects(list(replaceable))
    for name in selection_before:
        obj = bpy.data.objects.get(name)
        if obj is not None:
            obj.select_set(True)
    context.view_layer.update()
    return {'FINISHED'}


class SKELETALWEAVER_OT_weave(Operator):
    bl_idname = "armature.skeletal_weave"
    bl_label = "Weave & Parent Mesh"
    bl_description = "Generate helper meshes from selected bones and parent them to the armature"
    bl_options = {'REGISTER', 'UNDO'}
    
    @classmethod
    def poll(cls, context):
        if context.mode != 'POSE':
            return False
        if context.active_object is None:
            return False
        if context.active_object.type != 'ARMATURE':
            return False
        return True
    
    def execute(self, context):
        props = context.scene.skeletal_weaver_props
        if not props.create_vertex_groups and (props.use_physics_modifier or props.add_bone_constraints):
            self.report({'ERROR'}, "Enable Create Vertex Groups for physics or bone constraints")
            return {'CANCELLED'}
        return run_generation(self, context, self._execute, snap_bones=props.snap_bones)

    def _execute(self, context):
        armature_obj = context.active_object
        selected_bones = list(context.selected_pose_bones)
        
        if not selected_bones:
            self.report({'WARNING'}, "No bones selected in Pose Mode")
            return {'CANCELLED'}
        
        props = context.scene.skeletal_weaver_props
        
        # Sample the current pose without previous simulation feedback.
        # Previous tracks are muted and replaced transactionally by execute().
        
        selected_bone_names = [b.name for b in selected_bones]
        
        # STEP 0b: Topology Cleanup (optional)
        if props.snap_bones:
            bpy.ops.object.mode_set(mode='EDIT')
            edit_bones = [armature_obj.data.edit_bones[name] for name in selected_bone_names]
            snap_close_bones(edit_bones, props.snap_threshold)
            bpy.ops.object.mode_set(mode='POSE')
            selected_bones = [armature_obj.pose.bones[name] for name in selected_bone_names]
        
        # Identify mesh groups
        mesh_groups = identify_mesh_groups(selected_bones, armature_obj)
        
        if not mesh_groups:
            self.report({'WARNING'}, "Could not identify any mesh groups from selection")
            return {'CANCELLED'}
        
        if props.use_physics_modifier and props.physics_mode == 'SOFT_BODY':
            mesh_groups = [dict(group, chain_roots=[root], all_bones=set(trace_chain(root, selected_bones)))
                           for group in mesh_groups for root in group['chain_roots']]

        # Switch to object mode to create meshes
        bpy.ops.object.mode_set(mode='OBJECT')
        
        created_meshes = []
        
        # Process each mesh group
        for group_idx, mesh_group in enumerate(mesh_groups):
            # Build bone matrix for this group
            bone_matrix, max_rows, max_cols = build_bone_matrix_for_group(
                mesh_group, armature_obj, selected_bones
            )
            
            if bone_matrix is None or max_cols == 0:
                continue
            
            # Extract coordinates
            coord_matrix = extract_coordinate_matrix(
                bone_matrix, armature_obj, max_rows, max_cols
            )
            
            # Create mesh with unique name
            if group_idx == 0:
                mesh_name = MESH_GUIDE_NAME
            else:
                mesh_name = f"{MESH_GUIDE_NAME}_{group_idx}"
            
            # Check if single column - create ribbon instead
            use_softbody_here = (
                props.use_physics_modifier
                and props.physics_mode == 'SOFT_BODY'
                and max_cols == 1
            )
            vert_indices_matrix = None

            if max_cols == 1:
                if use_softbody_here:
                    mesh_obj, vert_matrix, expanded_bone_matrix, actual_cols, vert_indices_matrix = (
                        create_extruded_ribbon_mesh(
                            coord_matrix,
                            bone_matrix,
                            max_rows,
                            mesh_name,
                            props.ribbon_width,
                            armature_obj,
                        )
                    )
                else:
                    mesh_obj, vert_matrix, expanded_bone_matrix, actual_cols = create_ribbon_mesh(
                        coord_matrix,
                        bone_matrix,
                        max_rows,
                        mesh_name,
                        props.ribbon_width,
                        armature_obj,
                    )
                bone_matrix_for_groups = expanded_bone_matrix
                max_cols_for_groups = actual_cols
            else:
                mesh_obj, vert_matrix = create_woven_mesh(
                    coord_matrix, bone_matrix, max_rows, max_cols, mesh_name
                )
                bone_matrix_for_groups = bone_matrix
                max_cols_for_groups = max_cols

            pin_group_name = "Goal" if use_softbody_here else "Pin"

            # Create vertex groups
            if props.create_vertex_groups:
                custom_curve_mapping = None
                if props.pin_use_falloff and props.pin_falloff_pattern == 'CUSTOM':
                    custom_curve_mapping = get_or_create_pin_falloff_brush().curve_distance_falloff
                create_vertex_groups(
                    mesh_obj,
                    vert_matrix,
                    bone_matrix_for_groups,
                    max_rows,
                    max_cols_for_groups,
                    props.pin_use_falloff,
                    props.pin_falloff_pattern,
                    props.pin_falloff_strength,
                    props.pin_min_weight,
                    custom_curve_mapping,
                    pin_group_name=pin_group_name,
                    vert_indices_matrix=vert_indices_matrix,
                )

            mesh_obj[OWNER_KEY] = uuid4().hex
            mesh_obj[ROLE_KEY] = 'GUIDE'
            mesh_obj[SOURCE_KEY] = armature_obj
            mesh_obj[BONES_KEY] = sorted(b.name for b in mesh_group['all_bones'])

            # Parent to appropriate bone
            parent_mesh_to_armature(mesh_obj, armature_obj, mesh_group['parent_bone'])

            # Store for later processing
            created_meshes.append({
                'obj': mesh_obj,
                'parent': mesh_group['parent_bone'],
                'bone_matrix': bone_matrix,
                'max_rows': max_rows,
                'max_cols': max_cols,
                'verts': len(mesh_obj.data.vertices),
                'faces': len(mesh_obj.data.polygons),
                'vert_indices_matrix': vert_indices_matrix,
                'softbody_eligible': use_softbody_here,
            })

        # Add simulation modifiers
        if props.use_physics_modifier and props.physics_mode == 'CLOTH':
            for mesh_info in created_meshes:
                add_cloth_modifier(mesh_info['obj'])
        elif props.use_physics_modifier and props.physics_mode == 'SOFT_BODY':
            for mesh_info in created_meshes:
                if mesh_info.get('softbody_eligible'):
                    add_softbody_modifier(mesh_info['obj'], goal_group_name="Goal")
                    setup_softbody_hook_presets(
                        mesh_info['obj'],
                        props.softbody_hook_preset,
                        mesh_info.get('vert_indices_matrix'),
                    )
        
        # Add bone constraints if enabled
        total_constraints = 0
        if props.add_bone_constraints:
            for mesh_info in created_meshes:
                constraints = add_bone_constraints(
                    mesh_info['obj'], 
                    mesh_info['bone_matrix'],
                    mesh_info['max_rows'],
                    mesh_info['max_cols']
                )
                total_constraints += len(constraints)
        
        # Return to Pose Mode
        bpy.ops.object.select_all(action='DESELECT')
        armature_obj.select_set(True)
        context.view_layer.objects.active = armature_obj
        bpy.ops.object.mode_set(mode='POSE')
        
        # Report success
        if created_meshes:
            total_verts = sum(m['verts'] for m in created_meshes)
            total_faces = sum(m['faces'] for m in created_meshes)
            
            msg_parts = []
            for m in created_meshes:
                parent_name = m['parent'].name if m['parent'] else "Armature"
                msg_parts.append(f"'{m['obj'].name}' → {parent_name}")
            
            extra_info = []
            if props.use_physics_modifier and props.physics_mode == 'CLOTH':
                extra_info.append("Cloth")
            elif props.use_physics_modifier and props.physics_mode == 'SOFT_BODY':
                extra_info.append("Soft Body")
            if props.add_bone_constraints:
                extra_info.append(f"{total_constraints} constraints")
            
            extra_str = f" [{', '.join(extra_info)}]" if extra_info else ""
            
            self.report({'INFO'}, 
                        f"Created {len(created_meshes)} mesh(es) with {total_verts} verts, "
                        f"{total_faces} faces{extra_str}: {', '.join(msg_parts)}")
        else:
            self.report({'WARNING'}, "No meshes could be created")
            return {'CANCELLED'}
        
        return {'FINISHED'}


# =============================================================================
# UTILITY OPERATORS
# =============================================================================

class SKELETALWEAVER_OT_clean_bone_names(Operator):
    bl_idname = "armature.clean_bone_name_suffixes"
    bl_label = "Clean Bone Name Suffixes"
    bl_description = "Remove .001, .002, etc. suffixes from all bone names in the armature"
    bl_options = {'REGISTER', 'UNDO'}
    
    @classmethod
    def poll(cls, context):
        if context.active_object is None:
            return False
        if context.active_object.type != 'ARMATURE':
            return False
        return True
    
    def execute(self, context):
        import re
        
        armature_obj = context.active_object
        armature_data = armature_obj.data
        
        # Store original mode
        original_mode = context.mode
        rename_map = {}
        # Capture before Blender's bone renaming updates constraint paths.
        bindings = [(b.name, c.target, c.subtarget) for b in armature_obj.pose.bones
                    for c in b.constraints if is_weaver_damped_track_constraint(b, c) and c.target]
        
        # Switch to edit mode to rename bones
        if context.mode != 'EDIT_ARMATURE':
            bpy.ops.object.mode_set(mode='EDIT')
        
        # Pattern to match .001, .002, etc. at the end of the name
        suffix_pattern = re.compile(r'\.\d{3,}$')
        
        renamed_count = 0
        skipped_count = 0
        
        # Get all edit bones
        edit_bones = armature_data.edit_bones
        
        # First pass: collect bones to rename and check for conflicts
        bones_to_rename = []
        for bone in edit_bones:
            match = suffix_pattern.search(bone.name)
            if match:
                new_name = suffix_pattern.sub('', bone.name)
                bones_to_rename.append((bone, bone.name, new_name))
        
        # Second pass: rename bones, handling conflicts
        for bone, old_name, new_name in bones_to_rename:
            # Check if target name already exists (and is a different bone)
            if new_name in edit_bones and edit_bones[new_name] != bone:
                # Name conflict - skip this bone
                skipped_count += 1
            else:
                bone.name = new_name
                rename_map[old_name] = bone.name
                renamed_count += 1
        
        # Return to original mode
        bpy.ops.object.mode_set(mode='OBJECT')
        
        # Mesh target groups are not automatically renamed by Blender unless
        # they deform through this armature; guide meshes only use constraints.
        for old_name, mesh_obj, old_group in bindings:
            if old_name not in rename_map or mesh_obj.type != 'MESH':
                continue
            new_name = rename_map[old_name]
            group = mesh_obj.vertex_groups.get(old_group)
            if group is not None:
                group.name = new_name  # Blender resolves any Pin/Goal name collision.
                mapping = dict(mesh_obj.get(TARGETS_KEY, {}))
                mapping.pop(old_name, None)
                mapping[new_name] = group.name
                mesh_obj[TARGETS_KEY] = mapping
                for c in armature_obj.pose.bones[new_name].constraints:
                    if is_weaver_damped_track_constraint(armature_obj.pose.bones[new_name], c) and c.target == mesh_obj:
                        c.subtarget = group.name
        for mesh_obj in bpy.data.objects:
            if mesh_obj.get(SOURCE_KEY) == armature_obj and mesh_obj.get(BONES_KEY):
                mesh_obj[BONES_KEY] = [rename_map.get(name, name) for name in mesh_obj[BONES_KEY]]

        bpy.ops.object.mode_set(mode='EDIT' if original_mode == 'EDIT_ARMATURE' else original_mode)
        if skipped_count > 0:
            self.report({'INFO'}, 
                        f"Renamed {renamed_count} bones, skipped {skipped_count} (name conflicts)")
        else:
            self.report({'INFO'}, f"Renamed {renamed_count} bones")
        
        return {'FINISHED'}


class SKELETALWEAVER_OT_select_bone_chain(Operator):
    bl_idname = "armature.select_bone_chain"
    bl_label = "Select Bone Chain"
    bl_description = "Select all connected bones in the chain from the currently selected bone(s)"
    bl_options = {'REGISTER', 'UNDO'}
    
    @classmethod
    def poll(cls, context):
        if context.mode != 'POSE':
            return False
        if context.active_object is None:
            return False
        if context.active_object.type != 'ARMATURE':
            return False
        if not context.selected_pose_bones:
            return False
        return True
    
    def execute(self, context):
        armature_obj = context.active_object
        selected_bones = list(context.selected_pose_bones)
        
        # Select the full chain(s)
        selected_names = select_bone_chain(armature_obj, selected_bones)
        
        self.report({'INFO'}, f"Selected {len(selected_names)} bones in chain")
        
        return {'FINISHED'}


class SKELETALWEAVER_OT_clear_invalid_constraints(Operator):
    bl_idname = "armature.clear_invalid_constraints"
    bl_label = "Clear Invalid Constraints"
    bl_description = "Remove only plugin tracks with missing or invalid targets from selected bones"
    bl_options = {'REGISTER', 'UNDO'}
    
    @classmethod
    def poll(cls, context):
        if context.mode != 'POSE':
            return False
        if context.active_object is None:
            return False
        if context.active_object.type != 'ARMATURE':
            return False
        if not context.selected_pose_bones:
            return False
        return True
    
    def execute(self, context):
        selected_bones = list(context.selected_pose_bones)
        
        removed_count = remove_invalid_constraints(selected_bones)
        
        self.report({'INFO'}, f"Removed {removed_count} invalid constraints")
        
        return {'FINISHED'}


class SKELETALWEAVER_OT_cleanup_meshes(Operator):
    bl_idname = "armature.cleanup_weaver_meshes"
    bl_label = "Delete All Guide Meshes"
    bl_description = "Delete all plugin guides, their tracks and unused hook controls"
    bl_options = {'REGISTER', 'UNDO'}
    
    @classmethod
    def poll(cls, context):
        return any(is_guide_object(obj) or (obj.type == 'ARMATURE' and any(
            is_weaver_damped_track_constraint(b, c) for b in obj.pose.bones for c in b.constraints))
            for obj in bpy.data.objects)
    
    def execute(self, context):
        deleted_meshes, removed_constraints, restored_bones = cleanup_previous_meshes()
        self.report(
            {'INFO'},
            f"Deleted {deleted_meshes} guide mesh(es), removed {removed_constraints} constraint(s), "
            f"restored {restored_bones} bone pose(s)"
        )
        return {'FINISHED'}


class SKELETALWEAVER_OT_auto_merge(Operator):
    bl_idname = "armature.auto_merge_bones"
    bl_label = "Auto Merge Bones"
    bl_description = "Automatically connect selected bones to their parents if endpoints are close (auto merge)"
    bl_options = {'REGISTER', 'UNDO'}
    
    @classmethod
    def poll(cls, context):
        if context.mode != 'POSE':
            return False
        if context.active_object is None:
            return False
        if context.active_object.type != 'ARMATURE':
            return False
        if not context.selected_pose_bones:
            return False
        return True
    
    def execute(self, context):
        armature_obj = context.active_object
        selected_bones = list(context.selected_pose_bones)
        
        if not selected_bones:
            self.report({'WARNING'}, "No bones selected")
            return {'CANCELLED'}
        
        props = context.scene.skeletal_weaver_props
        
        # Store original mode
        original_mode = context.mode
        
        # Switch to edit mode to modify bone connections
        bpy.ops.object.mode_set(mode='EDIT')
        
        # Get edit bones
        selected_bone_names = [b.name for b in selected_bones]
        edit_bones = [armature_obj.data.edit_bones[name] for name in selected_bone_names]
        
        # Apply auto merge
        connected_count = 0
        for bone in edit_bones:
            if bone.parent and bone.parent.name in selected_bone_names:
                distance = (bone.head - bone.parent.tail).length
                if distance < props.snap_threshold:
                    if not bone.use_connect:
                        bone.use_connect = True
                        connected_count += 1
        
        # Return to original mode
        if original_mode == 'POSE':
            bpy.ops.object.mode_set(mode='POSE')
        elif original_mode == 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')
        
        if connected_count > 0:
            self.report({'INFO'}, f"Auto merged {connected_count} bone(s)")
        else:
            self.report({'INFO'}, "No bones needed merging (all already connected or too far apart)")
        
        return {'FINISHED'}


class SKELETALWEAVER_OT_add_hook_from_selection(Operator):
    bl_idname = "mesh.skeletal_weaver_add_hook_from_selection"
    bl_label = "Add Hook from Selection"
    bl_description = (
        "Selected vertices: assign Goal weight 1, new hook vertex group, Empty, and Hook modifier"
    )
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        obj = context.edit_object
        if obj is None or obj.type != 'MESH':
            return False
        if context.mode != 'EDIT_MESH':
            return False
        return is_guide_object(obj)

    def execute(self, context):
        obj = context.edit_object
        mesh = obj.data
        bm = bmesh.from_edit_mesh(mesh)
        bm.verts.ensure_lookup_table()
        selected = [v for v in bm.verts if v.select]
        if not selected:
            self.report({'WARNING'}, "No vertices selected")
            return {'CANCELLED'}
        indices = [v.index for v in selected]

        prev_mode = context.mode
        bpy.ops.object.mode_set(mode='OBJECT')

        create_hook(obj, indices, 'SW_Hook_User')

        context.view_layer.objects.active = obj
        if prev_mode == 'EDIT_MESH':
            bpy.ops.object.mode_set(mode='EDIT')

        self.report({'INFO'}, "Added pin weights, control, and Hook for %d vertices" % len(indices))
        return {'FINISHED'}


# =============================================================================
# UI PANEL
# =============================================================================

class SKELETALWEAVER_PT_main(Panel):
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Simple Cloth"
    bl_label = "骨链织网 · 高级设置"
    bl_options = {'DEFAULT_CLOSED'}
    bl_order = 2
    
    def draw(self, context):
        layout = self.layout
        props = context.scene.skeletal_weaver_props
        
        if context.mode != 'POSE':
            box = layout.box()
            box.alert = True
            box.label(text="⚠ Enter POSE mode to use", icon='ERROR')
            return
        
        if context.active_object is None or context.active_object.type != 'ARMATURE':
            box = layout.box()
            box.alert = True
            box.label(text="⚠ Select an Armature", icon='ERROR')
            return
        
        selected_count = len(context.selected_pose_bones) if context.selected_pose_bones else 0
        layout.label(text=f"Selected Bones: {selected_count}", icon='BONE_DATA')
        
        # Quick selection button at the top
        row = layout.row(align=True)
        row.operator("armature.select_bone_chain", text="Select Bone Chain", 
                     icon='LINKED')
        
        layout.separator()
        
        # Topology section
        box = layout.box()
        box.label(text="Topology", icon='SNAP_ON')
        col = box.column(align=True)
        col.operator("armature.auto_merge_bones", text="Auto Merge Bones", 
                     icon='LINKED')
        col.prop(props, "snap_bones")
        if props.snap_bones:
            col.prop(props, "snap_threshold")
        
        layout.separator()
        
        # Mesh section
        box = layout.box()
        box.label(text="Mesh Generation", icon='MESH_GRID')
        col = box.column(align=True)
        group_row = col.row()
        group_row.enabled = not props.create_vertex_groups or not (props.use_physics_modifier or props.add_bone_constraints)
        group_row.prop(props, "create_vertex_groups")
        col.prop(props, "ribbon_width")
        
        layout.separator()
        
        # Simulation section
        box = layout.box()
        box.label(text="Simulation & Rigging", icon='MOD_CLOTH')
        col = box.column(align=True)
        col.prop(props, "use_physics_modifier", text="Add Physics Modifier")
        if props.use_physics_modifier:
            col.prop(props, "physics_mode", text="Type")
            if props.physics_mode == 'SOFT_BODY':
                col.prop(props, "softbody_hook_preset")
        col.prop(props, "add_bone_constraints")
        col.separator()
        col.label(text="Pin / Goal weights", icon='FCURVE')
        col.prop(props, "pin_use_falloff", text="Pin Falloff (else top-only free swing)")
        if props.pin_use_falloff:
            col.prop(props, "pin_falloff_pattern", text="Pattern")
        show_falloff_params = (
            props.pin_use_falloff and props.pin_falloff_pattern != 'TOP_ONLY'
        )
        if show_falloff_params and props.pin_falloff_pattern == 'CUSTOM':
            brush = bpy.data.brushes.get(PIN_FALLOFF_BRUSH_NAME)
            if brush:
                col.template_curve_mapping(brush, "curve_distance_falloff")
            else:
                col.operator('armature.weaver_create_pin_curve')
        elif show_falloff_params and props.pin_falloff_pattern in {'INVERSE', 'EXPONENTIAL'}:
            col.prop(props, "pin_falloff_strength")
        if show_falloff_params:
            col.prop(props, "pin_min_weight")
        
        layout.separator()
        
        # Main button
        row = layout.row(align=True)
        row.scale_y = 1.5
        row.operator("armature.skeletal_weave", text="Weave & Parent Mesh", 
                     icon='OUTLINER_OB_MESH')
        
        layout.separator()
        
        # Utilities section
        box = layout.box()
        box.label(text="Utilities", icon='TOOL_SETTINGS')
        col = box.column(align=True)
        col.operator("armature.clear_invalid_constraints", text="Clear Broken Plugin Tracks",
                     icon='X')
        col.operator("armature.clean_bone_name_suffixes", text="Clean Bone Name Suffixes", 
                     icon='BRUSH_DATA')
        col.operator("armature.cleanup_weaver_meshes", text="Delete All Guide Meshes", 
                     icon='TRASH')


class SKELETALWEAVER_PT_edit_hooks(Panel):
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Simple Cloth"
    bl_label = "Guide Mesh Hooks"

    @classmethod
    def poll(cls, context):
        obj = context.edit_object
        return (
            obj is not None
            and obj.type == 'MESH'
            and is_guide_object(obj)
        )

    def draw(self, context):
        layout = self.layout
        layout.operator(
            "mesh.skeletal_weaver_add_hook_from_selection",
            text="Add Hook from Selection",
            icon='HOOK',
        )


# =============================================================================
# PROPERTIES
# =============================================================================

def update_pin_falloff_pattern(self, context):
    if self.pin_falloff_pattern == 'CUSTOM':
        get_or_create_pin_falloff_brush()


class SKELETALWEAVER_OT_create_pin_curve(Operator):
    bl_idname = 'armature.weaver_create_pin_curve'
    bl_label = 'Create Custom Curve'
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        get_or_create_pin_falloff_brush()
        return {'FINISHED'}


class SkeletalWeaverProperties(PropertyGroup):
    snap_bones: BoolProperty(
        name="Snap Bones",
        description="Snap close bone endpoints together before generating mesh",
        default=False,
    )
    
    snap_threshold: FloatProperty(
        name="Snap Threshold",
        description="Distance threshold for snapping bone endpoints",
        default=0.001,
        min=0.0001,
        max=0.1,
        unit='LENGTH',
        precision=4,
    )
    
    create_vertex_groups: BoolProperty(
        name="Create Vertex Groups",
        description="Create mass vertex group (Pin or Goal) and per-bone groups",
        default=True,
    )
    
    ribbon_width: FloatProperty(
        name="Ribbon Width",
        description="Width of ribbon mesh when only one chain is selected; also Soft Body extrusion thickness",
        default=0.02,
        min=0.001,
        max=1.0,
        unit='LENGTH',
    )

    use_physics_modifier: BoolProperty(
        name="Add Physics Modifier",
        description="After weave, add Cloth or Soft Body (turn off for guide mesh only)",
        default=True,
    )

    physics_mode: EnumProperty(
        name="Physics Type",
        description="Cloth sheets or Soft Body strips (one per chain)",
        items=[
            ('CLOTH', "Cloth", "Cloth simulation using Pin vertex group"),
            (
                'SOFT_BODY',
                "Soft Body",
                "One extruded strip per chain, with Goal and optional hooks",
            ),
        ],
        default='CLOTH',
    )

    softbody_hook_preset: EnumProperty(
        name="Soft Body Hooks",
        description="Preset hook empties on chain ends (Soft Body mode only)",
        items=[
            ('NONE', "None", "No automatic hook modifiers"),
            ('BOTH_ENDS', "Both Ends", "Hook modifiers on head and tail vertices"),
            ('HEAD_ONLY', "Head Only", "Hook modifier on chain head (parent row) only"),
            ('TAIL_ONLY', "Tail Only", "Hook modifier on chain tail only"),
        ],
        default='BOTH_ENDS',
    )
    add_bone_constraints: BoolProperty(
        name="Add Bone Constraints",
        description="Add DAMPED_TRACK constraints to bones targeting mesh vertices",
        default=True,
    )

    pin_use_falloff: BoolProperty(
        name="Pin Falloff",
        description="Use falloff for Pin group (weight decreases down chain). Off = only top vertices pinned (1), rest free (0)",
        default=True,
    )

    pin_falloff_pattern: EnumProperty(
        name="Pin Falloff Pattern",
        description="How pin/goal weight decreases from top (x=0) to bottom (x=1), when Pin Falloff is enabled",
        items=[
            (
                'TOP_ONLY',
                "Top Only",
                "Only the root row has full weight; all other rows 0 (same numeric result as turning Pin Falloff off)",
            ),
            ('INVERSE', "Inverse", "Fast drop near top, slower near bottom (recommended)"),
            ('EXPONENTIAL', "Exponential", "Strong early damping with smooth tail"),
            ('ROOT', "Root", "Very fast initial drop, then long soft tail"),
            ('LINEAR', "Linear", "Uniform decrease from top to bottom"),
            ('CUSTOM', "Custom Curve", "User-defined falloff curve via control points"),
        ],
        default='INVERSE',
        update=update_pin_falloff_pattern,
    )

    pin_falloff_strength: FloatProperty(
        name="Pin Falloff Strength",
        description="Controls how quickly the weight drops near the top",
        default=4.0,
        min=0.01,
        max=20.0,
        precision=2,
    )

    pin_min_weight: FloatProperty(
        name="Pin Min Weight",
        description="Minimum pin weight at the chain tail",
        default=0.05,
        min=0.0,
        max=1.0,
        precision=3,
        subtype='FACTOR',
    )


# =============================================================================
# REGISTRATION
# =============================================================================

classes = (
    SkeletalWeaverProperties,
    SKELETALWEAVER_OT_weave,
    SKELETALWEAVER_OT_clean_bone_names,
    SKELETALWEAVER_OT_select_bone_chain,
    SKELETALWEAVER_OT_clear_invalid_constraints,
    SKELETALWEAVER_OT_cleanup_meshes,
    SKELETALWEAVER_OT_auto_merge,
    SKELETALWEAVER_OT_add_hook_from_selection,
    SKELETALWEAVER_PT_main,
    SKELETALWEAVER_PT_edit_hooks,
    SKELETALWEAVER_OT_create_pin_curve,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    
    bpy.types.Scene.skeletal_weaver_props = PointerProperty(
        type=SkeletalWeaverProperties
    )


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
    
    del bpy.types.Scene.skeletal_weaver_props


if __name__ == "__main__":
    register()
