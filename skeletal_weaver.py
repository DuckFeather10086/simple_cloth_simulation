# SPDX-License-Identifier: GPL-2.0-or-later

import bpy
import bmesh
from bpy.props import FloatProperty, BoolProperty, PointerProperty
from bpy.types import Operator, Panel, PropertyGroup
from mathutils import Vector, Matrix
from collections import defaultdict

bl_info = {
    "name": "Skeletal Weaver",
    "author": "Duck Feather",
    "version": (2, 2, 0),
    "blender": (2, 80, 0),
    "location": "Pose Mode > Sidebar > Skeletal Weaver",
    "description": "Generates helper meshes (grid/web) based on selected bones in an Armature",
    "warning": "",
    "doc_url": "",
    "category": "Rigging",
}

MESH_GUIDE_NAME = "Skeletal_Mesh_Guide"


# =============================================================================
# STEP 0: TOPOLOGY CLEANUP
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


# =============================================================================
# MESH GROUP IDENTIFICATION
# =============================================================================

def identify_mesh_groups(selected_bones, armature_obj):
    """
    Identify separate mesh groups from selected bones.
    
    - "Primary group": bones whose parent is NOT in selection (or is None)
      These bones and their connected descendants form the first mesh.
    - "Secondary groups": bones whose parent IS in selection but use_connect = False
      These are grouped by their parent bone, each group forms a separate mesh.
    
    Returns:
        List of mesh group dicts, each containing:
        - 'chain_roots': list of bones that start chains in this group
        - 'parent_bone': the bone to parent the mesh to
        - 'all_bones': set of all bones in this group
    """
    selected_names = {b.name for b in selected_bones}
    
    primary_roots = []
    secondary_roots = defaultdict(list)
    
    for bone in selected_bones:
        if bone.parent is None or bone.parent.name not in selected_names:
            primary_roots.append(bone)
        elif not bone.bone.use_connect:
            secondary_roots[bone.parent.name].append(bone)
    
    mesh_groups = []
    
    if primary_roots:
        primary_group = {
            'chain_roots': primary_roots,
            'parent_bone': None,
            'all_bones': set(),
        }
        
        for root in primary_roots:
            chain = trace_chain_bones(root, selected_bones)
            primary_group['all_bones'].update(chain)
        
        topmost_root = max(primary_roots, key=lambda b: b.bone.head_local.z)
        primary_group['parent_bone'] = topmost_root.parent
        
        mesh_groups.append(primary_group)
    
    for parent_name, roots in secondary_roots.items():
        parent_bone = armature_obj.pose.bones.get(parent_name)
        
        secondary_group = {
            'chain_roots': roots,
            'parent_bone': parent_bone,
            'all_bones': set(),
        }
        
        for root in roots:
            chain = trace_chain_bones(root, selected_bones)
            secondary_group['all_bones'].update(chain)
        
        mesh_groups.append(secondary_group)
    
    return mesh_groups


def trace_chain_bones(start_bone, selected_bones):
    """
    Trace a chain from start_bone, following only CONNECTED children.
    Returns set of all bones in the chain.
    """
    selected_names = {b.name for b in selected_bones}
    chain = {start_bone}
    current = start_bone
    
    while True:
        next_bone = None
        for child in current.children:
            if child.name in selected_names and child.bone.use_connect:
                next_bone = child
                break
        
        if next_bone is None:
            break
        
        chain.add(next_bone)
        current = next_bone
    
    return chain


# =============================================================================
# MATRIX BUILDING
# =============================================================================

def trace_chain(start_bone, selected_bones):
    """
    Trace a chain from start_bone downward through selected CONNECTED children.
    Returns a list of bones in order from start to end.
    """
    selected_names = {b.name for b in selected_bones}
    chain = [start_bone]
    current = start_bone
    
    while True:
        next_bone = None
        for child in current.children:
            if child.name in selected_names and child.bone.use_connect:
                next_bone = child
                break
        
        if next_bone is None:
            break
        
        chain.append(next_bone)
        current = next_bone
    
    return chain


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
    
    # Calculate perpendicular offset direction for ribbon
    # Use the first bone's local X axis as the offset direction
    first_bone = bone_matrix[0][0]
    if first_bone is not None:
        # Get bone's world matrix to find its local X axis
        bone_world_matrix = armature_obj.matrix_world @ first_bone.bone.matrix_local
        # Local X axis in world space
        offset_dir = Vector((bone_world_matrix[0][0], bone_world_matrix[0][1], bone_world_matrix[0][2])).normalized()
    else:
        # Fallback: use world X axis
        offset_dir = Vector((1, 0, 0))
    
    half_width = ribbon_width / 2.0
    
    # Create vertices with offset
    for row in range(total_rows):
        coord = coords[row]
        if coord is not None:
            # Left vertex (negative offset)
            left_pos = coord - offset_dir * half_width
            vert_left = bm.verts.new(left_pos)
            vert_matrix[0][row] = vert_left
            
            # Right vertex (positive offset)
            right_pos = coord + offset_dir * half_width
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
    bmesh.ops.remove_doubles(bm, verts=bm.verts[:], dist=0.0001)
    
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
    bmesh.ops.remove_doubles(bm, verts=bm.verts[:], dist=0.0001)
    
    coord_vert_matrix = [[None for _ in range(total_rows)] for _ in range(max_cols)]
    for col in range(max_cols):
        for row in range(total_rows):
            if vert_matrix[col][row] is not None:
                coord_vert_matrix[col][row] = vert_matrix[col][row].co.copy()
    
    bm.to_mesh(mesh)
    bm.free()
    mesh.update()
    
    obj = bpy.data.objects.new(mesh_name, mesh)
    bpy.context.collection.objects.link(obj)
    
    return obj, coord_vert_matrix


# =============================================================================
# VERTEX GROUPS & PARENTING
# =============================================================================

def create_vertex_groups(mesh_obj, coord_vert_matrix, bone_matrix, max_rows, max_cols):
    """
    Create vertex groups for rigging.
    """
    total_rows = max_rows + 1
    
    pin_group = mesh_obj.vertex_groups.new(name="Pin")
    mesh = mesh_obj.data
    
    coord_to_idx = {}
    for idx, vert in enumerate(mesh.vertices):
        coord_key = (round(vert.co.x, 5), round(vert.co.y, 5), round(vert.co.z, 5))
        coord_to_idx[coord_key] = idx
    
    for col in range(max_cols):
        for row in range(total_rows):
            orig_coord = coord_vert_matrix[col][row]
            if orig_coord is not None:
                coord_key = (round(orig_coord.x, 5), round(orig_coord.y, 5), round(orig_coord.z, 5))
                
                if coord_key in coord_to_idx:
                    vert_idx = coord_to_idx[coord_key]
                    
                    # Row 0 is the first HEAD position - Pin only, no bone assignment
                    # (This vertex is pinned, no bone tracks to it)
                    if row == 0:
                        pin_group.add([vert_idx], 1.0, 'REPLACE')
                    else:
                        # Row N (N > 0) is the TAIL position of bone[N-1]
                        # So assign to bone_matrix[col][N-1]
                        prev_row = row - 1
                        bone = None
                        
                        if prev_row < max_rows:
                            bone = bone_matrix[col][prev_row]
                        
                        # If bone at prev_row is None, find last valid bone
                        if bone is None:
                            for r in range(prev_row - 1, -1, -1):
                                if r < max_rows:
                                    bone = bone_matrix[col][r]
                                    if bone is not None:
                                        break
                        
                        if bone is not None:
                            group_name = bone.name
                            if group_name not in mesh_obj.vertex_groups:
                                bone_group = mesh_obj.vertex_groups.new(name=group_name)
                            else:
                                bone_group = mesh_obj.vertex_groups[group_name]
                            bone_group.add([vert_idx], 1.0, 'REPLACE')


def add_cloth_modifier(mesh_obj):
    """
    Add cloth modifier to mesh with Pin vertex group.
    """
    cloth_mod = mesh_obj.modifiers.new("Cloth", 'CLOTH')
    cloth_mod.settings.vertex_group_mass = "Pin"
    return cloth_mod


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
                if bone.name in mesh_obj.vertex_groups:
                    # Add DAMPED_TRACK constraint
                    constraint = bone.constraints.new('DAMPED_TRACK')
                    constraint.target = mesh_obj
                    constraint.subtarget = bone.name
                    constraint.track_axis = 'TRACK_Y'
                    
                    added_constraints.append((bone.name, constraint.name))
    
    return added_constraints


def parent_mesh_to_armature(mesh_obj, armature_obj, parent_bone):
    """
    Parent the mesh to the armature with bone parenting.
    """
    original_matrix = mesh_obj.matrix_world.copy()
    mesh_obj.parent = armature_obj
    
    if parent_bone is not None:
        mesh_obj.parent_type = 'BONE'
        mesh_obj.parent_bone = parent_bone.name
        bone_matrix = armature_obj.matrix_world @ parent_bone.bone.matrix_local
        mesh_obj.matrix_parent_inverse = bone_matrix.inverted()
    else:
        mesh_obj.matrix_parent_inverse = armature_obj.matrix_world.inverted()
    
    mesh_obj.matrix_world = original_matrix


def cleanup_previous_meshes():
    """
    Remove previous Skeletal_Mesh_Guide objects if they exist.
    """
    objects_to_remove = [obj for obj in bpy.data.objects 
                         if obj.name.startswith(MESH_GUIDE_NAME)]
    
    for obj in objects_to_remove:
        mesh_data = obj.data
        bpy.data.objects.remove(obj, do_unlink=True)
        if mesh_data and mesh_data.users == 0:
            bpy.data.meshes.remove(mesh_data)


# =============================================================================
# MAIN OPERATOR
# =============================================================================

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
        armature_obj = context.active_object
        selected_bones = list(context.selected_pose_bones)
        
        if not selected_bones:
            self.report({'WARNING'}, "No bones selected in Pose Mode")
            return {'CANCELLED'}
        
        props = context.scene.skeletal_weaver_props
        
        # Clean up previous meshes
        cleanup_previous_meshes()
        
        selected_bone_names = [b.name for b in selected_bones]
        
        # STEP 0: Topology Cleanup (optional)
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
        
        # Switch to object mode to create meshes
        bpy.ops.object.mode_set(mode='OBJECT')
        
        created_meshes = []
        all_bone_matrices = []
        
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
            if max_cols == 1:
                mesh_obj, vert_matrix, expanded_bone_matrix, actual_cols = create_ribbon_mesh(
                    coord_matrix, bone_matrix, max_rows, mesh_name, 
                    props.ribbon_width, armature_obj
                )
                # Use expanded bone matrix for vertex groups
                bone_matrix_for_groups = expanded_bone_matrix
                max_cols_for_groups = actual_cols
            else:
                mesh_obj, vert_matrix = create_woven_mesh(
                    coord_matrix, bone_matrix, max_rows, max_cols, mesh_name
                )
                bone_matrix_for_groups = bone_matrix
                max_cols_for_groups = max_cols
            
            # Create vertex groups
            if props.create_vertex_groups:
                create_vertex_groups(mesh_obj, vert_matrix, bone_matrix_for_groups, 
                                   max_rows, max_cols_for_groups)
            
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
            })
        
        # Add cloth modifier if enabled
        if props.add_cloth_modifier:
            for mesh_info in created_meshes:
                add_cloth_modifier(mesh_info['obj'])
        
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
            if props.add_cloth_modifier:
                extra_info.append("Cloth")
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
# UI PANEL
# =============================================================================

class SKELETALWEAVER_PT_main(Panel):
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Skeletal Weaver"
    bl_label = "Skeletal Weaver"
    
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
        
        layout.separator()
        
        # Topology section
        box = layout.box()
        box.label(text="Topology", icon='SNAP_ON')
        col = box.column(align=True)
        col.prop(props, "snap_bones")
        if props.snap_bones:
            col.prop(props, "snap_threshold")
        
        layout.separator()
        
        # Mesh section
        box = layout.box()
        box.label(text="Mesh Generation", icon='MESH_GRID')
        col = box.column(align=True)
        col.prop(props, "create_vertex_groups")
        col.prop(props, "ribbon_width")
        
        layout.separator()
        
        # Simulation section
        box = layout.box()
        box.label(text="Simulation & Rigging", icon='MOD_CLOTH')
        col = box.column(align=True)
        col.prop(props, "add_cloth_modifier")
        col.prop(props, "add_bone_constraints")
        
        layout.separator()
        
        # Main button
        row = layout.row(align=True)
        row.scale_y = 1.5
        row.operator("armature.skeletal_weave", text="Weave & Parent Mesh", 
                     icon='OUTLINER_OB_MESH')


# =============================================================================
# PROPERTIES
# =============================================================================

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
        description="Create vertex groups for Pin and individual bones",
        default=True,
    )
    
    ribbon_width: FloatProperty(
        name="Ribbon Width",
        description="Width of ribbon mesh when only one chain is selected",
        default=0.02,
        min=0.001,
        max=1.0,
        unit='LENGTH',
    )
    
    add_cloth_modifier: BoolProperty(
        name="Add Cloth Modifier",
        description="Add cloth simulation modifier with Pin group",
        default=True,
    )
    
    add_bone_constraints: BoolProperty(
        name="Add Bone Constraints",
        description="Add DAMPED_TRACK constraints to bones targeting mesh vertices",
        default=True,
    )


# =============================================================================
# REGISTRATION
# =============================================================================

classes = (
    SkeletalWeaverProperties,
    SKELETALWEAVER_OT_weave,
    SKELETALWEAVER_PT_main,
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
