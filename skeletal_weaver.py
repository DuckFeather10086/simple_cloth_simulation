# SPDX-License-Identifier: GPL-2.0-or-later

import bpy
import bmesh
from bpy.props import FloatProperty, BoolProperty, PointerProperty
from bpy.types import Operator, Panel, PropertyGroup
from mathutils import Vector

bl_info = {
    "name": "Skeletal Weaver",
    "author": "Duck Feather",
    "version": (2, 0, 0),
    "blender": (2, 80, 0),
    "location": "Pose Mode > Sidebar > Skeletal Weaver",
    "description": "Generates a helper mesh (grid/web) based on selected bones in an Armature",
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
            # Check distance between child head and parent tail
            distance = (bone.head - bone.parent.tail).length
            if distance < snap_threshold:
                bone.use_connect = True


# =============================================================================
# STEP 1 & 2: MATRIX MAPPING
# =============================================================================

def find_root_bone(selected_bones):
    """
    Find the common root bone of the selection.
    The root is the selected bone whose parent is not selected (or has no parent).
    If multiple roots exist, find their common ancestor.
    """
    selected_names = {b.name for b in selected_bones}
    
    # Find all bones whose parent is not in selection
    roots = []
    for bone in selected_bones:
        if bone.parent is None or bone.parent.name not in selected_names:
            roots.append(bone)
    
    if len(roots) == 1:
        return roots[0]
    
    if len(roots) > 1:
        # Multiple roots - find common parent (which should be the actual root)
        # Check if all roots share a common parent
        parents = set()
        for root in roots:
            if root.parent:
                parents.add(root.parent.name)
        
        # If all roots share the same parent, that's our actual root
        if len(parents) == 1:
            parent_name = list(parents)[0]
            # Return the parent bone (from pose bones context)
            armature = roots[0].id_data
            return armature.pose.bones.get(parent_name)
        
        # Otherwise, return the first root (topmost by Z)
        roots.sort(key=lambda b: b.bone.head_local.z, reverse=True)
        return roots[0]
    
    return None


def get_chain_starting_bones(root_bone, selected_bones):
    """
    Get the starting bones for each chain (direct children of root that are selected).
    """
    selected_names = {b.name for b in selected_bones}
    
    chain_starts = []
    for child in root_bone.children:
        if child.name in selected_names:
            chain_starts.append(child)
    
    return chain_starts


def trace_chain(start_bone, selected_bones):
    """
    Trace a chain from start_bone downward through selected children.
    Returns a list of bones in order from start to end.
    """
    selected_names = {b.name for b in selected_bones}
    chain = [start_bone]
    current = start_bone
    
    while True:
        # Find selected child
        next_bone = None
        for child in current.children:
            if child.name in selected_names:
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
    def get_local_x(bone):
        return bone.bone.head_local.x
    
    return sorted(chain_starts, key=get_local_x)


def build_bone_matrix(selected_bones, armature_obj):
    """
    Build a 2D matrix bone_matrix[col][row] from selected bones.
    
    Returns:
        - bone_matrix: 2D list where bone_matrix[col][row] is a bone or None
        - root_bone: The common root bone
        - max_rows: Maximum number of rows
        - max_cols: Number of columns
    """
    # Handle edge case: if only one chain (no branching), handle it specially
    root_candidates = []
    selected_names = {b.name for b in selected_bones}
    
    for bone in selected_bones:
        if bone.parent is None or bone.parent.name not in selected_names:
            root_candidates.append(bone)
    
    # If we have multiple root candidates, they might share a common parent
    if len(root_candidates) > 1:
        # Check if they share a parent
        parent_names = set()
        for rc in root_candidates:
            if rc.parent:
                parent_names.add(rc.parent.name)
        
        if len(parent_names) == 1:
            # All share the same parent - use that as root
            parent_name = list(parent_names)[0]
            root_bone = armature_obj.pose.bones.get(parent_name)
            chain_starts = root_candidates
        else:
            # No common parent - treat first as root and others as chains
            root_bone = root_candidates[0]
            chain_starts = root_candidates
    elif len(root_candidates) == 1:
        # Single root - check if it has selected children
        root_bone = root_candidates[0]
        chain_starts = [c for c in root_bone.children if c.name in selected_names]
        
        # If no selected children, this bone is the entire chain
        if not chain_starts:
            chain_starts = [root_bone]
            root_bone = root_bone.parent  # Parent becomes root (may be None)
    else:
        return None, None, 0, 0
    
    # Sort chain starts by X coordinate
    chain_starts = sort_chain_starts_by_x(chain_starts, armature_obj)
    
    # Build chains
    chains = []
    for start in chain_starts:
        chain = trace_chain(start, selected_bones)
        chains.append(chain)
    
    # Find max rows
    max_rows = max(len(chain) for chain in chains) if chains else 0
    max_cols = len(chains)
    
    # Build matrix with None padding
    bone_matrix = []
    for col_idx, chain in enumerate(chains):
        column = []
        for row_idx in range(max_rows):
            if row_idx < len(chain):
                column.append(chain[row_idx])
            else:
                column.append(None)
        bone_matrix.append(column)
    
    return bone_matrix, root_bone, max_rows, max_cols


# =============================================================================
# STEP 3: COORDINATE EXTRACTION
# =============================================================================

def extract_coordinate_matrix(bone_matrix, armature_obj, max_rows, max_cols):
    """
    Extract world-space head coordinates for each bone in the matrix.
    Also extract the tail of the last bone in each chain for the final row.
    
    Returns:
        - coord_matrix[col][row] = Vector or None
        - The matrix has max_rows + 1 rows (to include tail of last bones)
    """
    coord_matrix = []
    
    for col_idx in range(max_cols):
        column = []
        last_valid_bone = None
        
        for row_idx in range(max_rows):
            bone = bone_matrix[col_idx][row_idx]
            if bone is not None:
                # Get world-space head position
                head_world = armature_obj.matrix_world @ bone.head
                column.append(head_world.copy())
                last_valid_bone = bone
            else:
                column.append(None)
        
        # Add tail of last valid bone as final point
        if last_valid_bone is not None:
            tail_world = armature_obj.matrix_world @ last_valid_bone.tail
            column.append(tail_world.copy())
        else:
            column.append(None)
        
        coord_matrix.append(column)
    
    return coord_matrix


# =============================================================================
# STEP 4: MESH GENERATION (DYNAMIC WEAVING)
# =============================================================================

def create_woven_mesh(coord_matrix, bone_matrix, max_rows, max_cols, armature_obj):
    """
    Generate mesh using BMesh with proper weaving logic.
    
    Returns the created mesh object.
    """
    # Create new mesh and bmesh
    mesh = bpy.data.meshes.new(MESH_GUIDE_NAME + "_mesh")
    bm = bmesh.new()
    
    # Vertex matrix to store bmesh vertex references
    # Has max_rows + 1 rows (including tail vertices)
    total_rows = max_rows + 1
    vert_matrix = [[None for _ in range(total_rows)] for _ in range(max_cols)]
    
    # ===================
    # VERTEX GENERATION
    # ===================
    for col in range(max_cols):
        for row in range(total_rows):
            coord = coord_matrix[col][row]
            if coord is not None:
                vert = bm.verts.new(coord)
                vert_matrix[col][row] = vert
    
    # CRITICAL: Ensure lookup table is updated
    bm.verts.ensure_lookup_table()
    
    # ===================
    # VERTICAL EDGES (WARP - within each column)
    # ===================
    for col in range(max_cols):
        for row in range(total_rows - 1):
            v1 = vert_matrix[col][row]
            v2 = vert_matrix[col][row + 1]
            if v1 is not None and v2 is not None:
                try:
                    bm.edges.new((v1, v2))
                except ValueError:
                    pass  # Edge already exists
    
    bm.edges.ensure_lookup_table()
    
    # ===================
    # HORIZONTAL EDGES & FACES (WEFT - between adjacent columns)
    # ===================
    for col in range(max_cols - 1):
        col_left = col
        col_right = col + 1
        
        # Find the last valid row index for each column
        last_valid_left = -1
        last_valid_right = -1
        
        for row in range(total_rows):
            if vert_matrix[col_left][row] is not None:
                last_valid_left = row
            if vert_matrix[col_right][row] is not None:
                last_valid_right = row
        
        # Process each row
        for row in range(total_rows - 1):
            v_tl = vert_matrix[col_left][row]      # top-left
            v_bl = vert_matrix[col_left][row + 1]  # bottom-left
            v_tr = vert_matrix[col_right][row]     # top-right
            v_br = vert_matrix[col_right][row + 1] # bottom-right
            
            # Count valid vertices
            valid_verts = [v for v in [v_tl, v_bl, v_tr, v_br] if v is not None]
            
            if len(valid_verts) == 4:
                # CASE A: All four vertices exist - create quad
                try:
                    # Order: counter-clockwise for correct normals
                    bm.faces.new((v_tl, v_tr, v_br, v_bl))
                except ValueError:
                    pass  # Face already exists
                    
            elif len(valid_verts) == 3:
                # CASE B: Three vertices - create triangle
                try:
                    bm.faces.new(valid_verts)
                except ValueError:
                    pass
        
        # Handle end cases where one column is shorter
        # Connect the last vertex of shorter column to the longer column
        if last_valid_left != last_valid_right:
            if last_valid_left < last_valid_right:
                # Left column is shorter - connect its last vertex to remaining right vertices
                last_left_vert = vert_matrix[col_left][last_valid_left]
                if last_left_vert is not None:
                    for row in range(last_valid_left, last_valid_right):
                        v_r1 = vert_matrix[col_right][row]
                        v_r2 = vert_matrix[col_right][row + 1]
                        if v_r1 is not None and v_r2 is not None:
                            try:
                                # Create edge from last left to right vertices
                                bm.edges.new((last_left_vert, v_r1))
                            except ValueError:
                                pass
                            # Create triangle
                            try:
                                bm.faces.new((last_left_vert, v_r1, v_r2))
                            except ValueError:
                                pass
            else:
                # Right column is shorter - connect its last vertex to remaining left vertices
                last_right_vert = vert_matrix[col_right][last_valid_right]
                if last_right_vert is not None:
                    for row in range(last_valid_right, last_valid_left):
                        v_l1 = vert_matrix[col_left][row]
                        v_l2 = vert_matrix[col_left][row + 1]
                        if v_l1 is not None and v_l2 is not None:
                            try:
                                # Create edge from last right to left vertices
                                bm.edges.new((last_right_vert, v_l1))
                            except ValueError:
                                pass
                            # Create triangle (reversed winding)
                            try:
                                bm.faces.new((last_right_vert, v_l2, v_l1))
                            except ValueError:
                                pass
    
    # ===================
    # HORIZONTAL EDGES (connect row 0 across columns)
    # ===================
    for col in range(max_cols - 1):
        v1 = vert_matrix[col][0]
        v2 = vert_matrix[col + 1][0]
        if v1 is not None and v2 is not None:
            try:
                bm.edges.new((v1, v2))
            except ValueError:
                pass
    
    # Recalculate normals
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    
    # Remove duplicate vertices
    bmesh.ops.remove_doubles(bm, verts=bm.verts[:], dist=0.0001)
    
    # CRITICAL: Extract coordinates from vert_matrix BEFORE freeing bmesh
    # Store coordinates instead of BMVert references
    coord_vert_matrix = [[None for _ in range(total_rows)] for _ in range(max_cols)]
    for col in range(max_cols):
        for row in range(total_rows):
            if vert_matrix[col][row] is not None:
                # Store a copy of the coordinate
                coord_vert_matrix[col][row] = vert_matrix[col][row].co.copy()
    
    # Write to mesh
    bm.to_mesh(mesh)
    bm.free()
    mesh.update()
    
    # Create object
    obj = bpy.data.objects.new(MESH_GUIDE_NAME, mesh)
    bpy.context.collection.objects.link(obj)
    
    return obj, coord_vert_matrix


# =============================================================================
# STEP 5: RIGGING & CONSTRAINTS
# =============================================================================

def create_vertex_groups(mesh_obj, coord_vert_matrix, bone_matrix, max_rows, max_cols):
    """
    Create vertex groups for rigging.
    - "Pin" group for all row_0 vertices
    - Individual groups for each bone
    
    Args:
        mesh_obj: The mesh object
        coord_vert_matrix: 2D list of Vector coordinates (not BMVerts)
        bone_matrix: 2D list of bones
        max_rows: Number of bone rows (not including tail row)
        max_cols: Number of columns
    """
    total_rows = max_rows + 1
    
    # First, create the Pin group
    pin_group = mesh_obj.vertex_groups.new(name="Pin")
    
    # We need to identify which vertices correspond to which matrix positions
    # Since remove_doubles may have merged vertices, we use coordinates to match
    
    mesh = mesh_obj.data
    
    # Build coordinate to vertex index mapping
    coord_to_idx = {}
    for idx, vert in enumerate(mesh.vertices):
        # Round coordinates to avoid floating point issues
        coord_key = (round(vert.co.x, 5), round(vert.co.y, 5), round(vert.co.z, 5))
        coord_to_idx[coord_key] = idx
    
    # Process each position in the matrix
    for col in range(max_cols):
        for row in range(total_rows):
            orig_coord = coord_vert_matrix[col][row]
            if orig_coord is not None:
                # coord_vert_matrix now contains Vector objects directly
                coord_key = (round(orig_coord.x, 5), round(orig_coord.y, 5), round(orig_coord.z, 5))
                
                if coord_key in coord_to_idx:
                    vert_idx = coord_to_idx[coord_key]
                    
                    # Add to Pin group if row 0
                    if row == 0:
                        pin_group.add([vert_idx], 1.0, 'REPLACE')
                    
                    # Add to bone-specific group
                    # Row indices 0 to max_rows-1 correspond to bone heads
                    # Row index max_rows corresponds to the tail of the last bone
                    if row < max_rows:
                        bone = bone_matrix[col][row]
                        if bone is not None:
                            # Create or get vertex group for this bone
                            group_name = bone.name
                            if group_name not in mesh_obj.vertex_groups:
                                bone_group = mesh_obj.vertex_groups.new(name=group_name)
                            else:
                                bone_group = mesh_obj.vertex_groups[group_name]
                            
                            bone_group.add([vert_idx], 1.0, 'REPLACE')
                    else:
                        # This is a tail vertex - assign to the last bone in this column
                        # Find the last valid bone in this column
                        for r in range(max_rows - 1, -1, -1):
                            bone = bone_matrix[col][r]
                            if bone is not None:
                                group_name = bone.name
                                if group_name not in mesh_obj.vertex_groups:
                                    bone_group = mesh_obj.vertex_groups.new(name=group_name)
                                else:
                                    bone_group = mesh_obj.vertex_groups[group_name]
                                bone_group.add([vert_idx], 1.0, 'REPLACE')
                                break


def parent_mesh_to_armature(mesh_obj, armature_obj, root_bone):
    """
    Parent the mesh to the armature with bone parenting.
    """
    # Store current world matrix
    original_matrix = mesh_obj.matrix_world.copy()
    
    # Set parent
    mesh_obj.parent = armature_obj
    
    if root_bone is not None:
        mesh_obj.parent_type = 'BONE'
        mesh_obj.parent_bone = root_bone.name
        
        # Calculate the bone's world matrix
        bone_matrix = armature_obj.matrix_world @ root_bone.bone.matrix_local
        
        # Restore world position by adjusting parent inverse
        mesh_obj.matrix_parent_inverse = bone_matrix.inverted()
    else:
        mesh_obj.matrix_parent_inverse = armature_obj.matrix_world.inverted()
    
    mesh_obj.matrix_world = original_matrix


def cleanup_previous_mesh():
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
    bl_description = "Generate a helper mesh from selected bones and parent it to the armature"
    bl_options = {'REGISTER', 'UNDO'}
    
    @classmethod
    def poll(cls, context):
        """Only active in POSE mode with an armature selected."""
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
        
        # Get properties
        props = context.scene.skeletal_weaver_props
        
        # Clean up previous mesh
        cleanup_previous_mesh()
        
        # Store selected bone names to restore selection later
        selected_bone_names = [b.name for b in selected_bones]
        
        # ===================
        # STEP 0: Topology Cleanup (optional, in edit mode)
        # ===================
        if props.snap_bones:
            bpy.ops.object.mode_set(mode='EDIT')
            edit_bones = [armature_obj.data.edit_bones[name] for name in selected_bone_names]
            snap_close_bones(edit_bones, props.snap_threshold)
            bpy.ops.object.mode_set(mode='POSE')
            # Re-get pose bones after mode switch
            selected_bones = [armature_obj.pose.bones[name] for name in selected_bone_names]
        
        # ===================
        # STEP 1 & 2: Build Bone Matrix
        # ===================
        bone_matrix, root_bone, max_rows, max_cols = build_bone_matrix(
            selected_bones, armature_obj
        )
        
        if bone_matrix is None or max_cols == 0:
            self.report({'WARNING'}, "Could not build bone matrix from selection")
            return {'CANCELLED'}
        
        # ===================
        # STEP 3: Extract Coordinates
        # ===================
        coord_matrix = extract_coordinate_matrix(
            bone_matrix, armature_obj, max_rows, max_cols
        )
        
        # ===================
        # STEP 4: Create Mesh
        # ===================
        # Switch to object mode to create mesh
        bpy.ops.object.mode_set(mode='OBJECT')
        
        mesh_obj, vert_matrix = create_woven_mesh(
            coord_matrix, bone_matrix, max_rows, max_cols, armature_obj
        )
        
        # ===================
        # STEP 5: Rigging & Vertex Groups
        # ===================
        if props.create_vertex_groups:
            create_vertex_groups(mesh_obj, vert_matrix, bone_matrix, max_rows, max_cols)
        
        # Parent to armature
        parent_mesh_to_armature(mesh_obj, armature_obj, root_bone)
        
        # ===================
        # Return to Pose Mode
        # ===================
        bpy.ops.object.select_all(action='DESELECT')
        armature_obj.select_set(True)
        context.view_layer.objects.active = armature_obj
        bpy.ops.object.mode_set(mode='POSE')
        
        # Report success
        root_name = root_bone.name if root_bone else "Armature"
        vert_count = len(mesh_obj.data.vertices)
        face_count = len(mesh_obj.data.polygons)
        
        self.report({'INFO'}, 
                    f"Created '{MESH_GUIDE_NAME}' with {vert_count} vertices, "
                    f"{face_count} faces, parented to '{root_name}'")
        
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
        
        # Mode warning
        if context.mode != 'POSE':
            box = layout.box()
            box.alert = True
            box.label(text="⚠ Enter POSE mode to use", icon='ERROR')
            return
        
        # Check for armature
        if context.active_object is None or context.active_object.type != 'ARMATURE':
            box = layout.box()
            box.alert = True
            box.label(text="⚠ Select an Armature", icon='ERROR')
            return
        
        # Selection info
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
