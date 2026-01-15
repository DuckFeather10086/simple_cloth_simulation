# SPDX-License-Identifier: GPL-2.0-or-later

import bpy
import bmesh
from bpy.props import FloatProperty, BoolProperty, PointerProperty
from bpy.types import Operator, Panel, PropertyGroup
from mathutils import Vector
from collections import defaultdict

bl_info = {
    "name": "Skeletal Weaver",
    "author": "Duck Feather",
    "version": (1, 0, 0),
    "blender": (2, 80, 0),
    "location": "Pose Mode > Sidebar > Skeletal Weaver",
    "description": "Generates a helper mesh (grid/web) based on selected bones in an Armature",
    "warning": "",
    "doc_url": "",
    "category": "Rigging",
}

MESH_GUIDE_NAME = "Skeletal_Mesh_Guide"


# =============================================================================
# PHASE A: Chain Identification
# =============================================================================

def identify_root_bones(selected_bones):
    """
    Identify "Root" bones: selected bones whose parent is not selected 
    or has no parent.
    
    Returns a list of root bone references.
    """
    selected_names = {b.name for b in selected_bones}
    roots = []
    
    for bone in selected_bones:
        # A bone is a root if it has no parent or its parent is not in selection
        if bone.parent is None or bone.parent.name not in selected_names:
            roots.append(bone)
    
    return roots


def trace_chain_from_root(root_bone, selected_bones):
    """
    Trace from a root bone downward through its children to form a sorted 
    list of points (Head and Tail coordinates).
    
    Returns a list of tuples: [(bone, head_pos, tail_pos), ...]
    """
    selected_names = {b.name for b in selected_bones}
    chain = []
    current_bone = root_bone
    
    while current_bone is not None:
        # Get world-space positions from pose bone
        armature = current_bone.id_data
        head_world = armature.matrix_world @ current_bone.head
        tail_world = armature.matrix_world @ current_bone.tail
        
        chain.append({
            'bone': current_bone,
            'head': head_world.copy(),
            'tail': tail_world.copy(),
        })
        
        # Find the next bone in chain (child that is selected)
        next_bone = None
        for child in current_bone.children:
            if child.name in selected_names:
                next_bone = child
                break
        
        current_bone = next_bone
    
    return chain


def build_chains(selected_bones):
    """
    Build all chains from the selected bones.
    
    Returns a list of chains, where each chain is a list of bone data dicts.
    """
    roots = identify_root_bones(selected_bones)
    chains = []
    
    for root in roots:
        chain = trace_chain_from_root(root, selected_bones)
        if chain:
            chains.append(chain)
    
    return chains


# =============================================================================
# PHASE B: Global Sorting
# =============================================================================

def get_bone_local_x(bone, armature_obj):
    """
    Get the local X coordinate of a bone's head in armature space.
    """
    # Get the bone's head position in armature local space
    return bone.bone.head_local.x


def sort_chains_by_local_x(chains, armature_obj):
    """
    Sort chains based on their root bone's local X-coordinate in armature space.
    """
    def get_chain_x(chain):
        if chain:
            root_bone = chain[0]['bone']
            return get_bone_local_x(root_bone, armature_obj)
        return 0
    
    return sorted(chains, key=get_chain_x)


# =============================================================================
# PHASE C: Transverse Weaving
# =============================================================================

def extract_points_from_chain(chain):
    """
    Extract all unique points from a chain.
    Returns list of Vector positions.
    """
    points = []
    for i, bone_data in enumerate(chain):
        # Add head (always)
        points.append(bone_data['head'].copy())
        # Add tail only for the last bone (to avoid duplicates at joints)
        if i == len(chain) - 1:
            points.append(bone_data['tail'].copy())
    return points


def find_closest_segment_endpoints(point, other_chain_points):
    """
    For a point P in Chain_i, find the closest segment in Chain_{i+1}
    and return the indices of both segment endpoints.
    """
    if len(other_chain_points) < 2:
        return [0] if other_chain_points else []
    
    min_dist = float('inf')
    closest_segment_idx = 0
    
    for i in range(len(other_chain_points) - 1):
        seg_start = other_chain_points[i]
        seg_end = other_chain_points[i + 1]
        
        # Find closest point on segment
        seg_vec = seg_end - seg_start
        seg_len_sq = seg_vec.length_squared
        
        if seg_len_sq < 0.0001:
            closest_on_seg = seg_start
        else:
            t = max(0, min(1, (point - seg_start).dot(seg_vec) / seg_len_sq))
            closest_on_seg = seg_start + t * seg_vec
        
        dist = (point - closest_on_seg).length
        
        if dist < min_dist:
            min_dist = dist
            closest_segment_idx = i
    
    return [closest_segment_idx, closest_segment_idx + 1]


def weave_chains(chains, threshold, generate_faces):
    """
    Iterate through adjacent pairs of sorted chains and create edges/faces.
    
    Returns vertices list and edges/faces list for mesh creation.
    """
    if len(chains) < 1:
        return [], [], []
    
    # Build global vertex list and chain point indices
    all_vertices = []
    chain_point_indices = []  # Maps chain_idx -> list of vertex indices
    
    for chain in chains:
        points = extract_points_from_chain(chain)
        start_idx = len(all_vertices)
        indices = []
        
        for pt in points:
            all_vertices.append(pt)
            indices.append(len(all_vertices) - 1)
        
        chain_point_indices.append(indices)
    
    edges = []
    faces = []
    
    # Create edges along each chain
    for indices in chain_point_indices:
        for i in range(len(indices) - 1):
            edges.append((indices[i], indices[i + 1]))
    
    # Weave between adjacent chain pairs
    for chain_idx in range(len(chains) - 1):
        left_indices = chain_point_indices[chain_idx]
        right_indices = chain_point_indices[chain_idx + 1]
        
        left_points = [all_vertices[i] for i in left_indices]
        right_points = [all_vertices[i] for i in right_indices]
        
        # Track connections for face generation
        connections = defaultdict(set)  # left_idx -> set of right_idx
        
        # For every point in left chain, find closest segment in right chain
        for li, left_pt in enumerate(left_points):
            seg_endpoints = find_closest_segment_endpoints(left_pt, right_points)
            
            for ri in seg_endpoints:
                if ri < len(right_points):
                    dist = (left_pt - right_points[ri]).length
                    if dist <= threshold:
                        edge = (left_indices[li], right_indices[ri])
                        if edge not in edges and (edge[1], edge[0]) not in edges:
                            edges.append(edge)
                            connections[li].add(ri)
        
        # For every point in right chain, find closest segment in left chain
        for ri, right_pt in enumerate(right_points):
            seg_endpoints = find_closest_segment_endpoints(right_pt, left_points)
            
            for li in seg_endpoints:
                if li < len(left_points):
                    dist = (right_pt - left_points[li]).length
                    if dist <= threshold:
                        edge = (left_indices[li], right_indices[ri])
                        if edge not in edges and (edge[1], edge[0]) not in edges:
                            edges.append(edge)
                            connections[li].add(ri)
        
        # Generate faces if requested
        if generate_faces:
            faces.extend(generate_weave_faces(
                left_indices, right_indices, 
                left_points, right_points, 
                connections, threshold
            ))
    
    return all_vertices, edges, faces


def generate_weave_faces(left_indices, right_indices, left_points, right_points, 
                          connections, threshold):
    """
    Generate triangular faces between two chains based on connections.
    Creates triangular fans to handle mismatched bone counts.
    """
    faces = []
    used_edges = set()
    
    # Build adjacency for both chains
    max_left = len(left_indices) - 1
    max_right = len(right_indices) - 1
    
    # Iterate through left chain segments
    for li in range(max_left):
        li_next = li + 1
        
        # Get connected right points for current and next left point
        right_conn_curr = sorted(connections.get(li, set()))
        right_conn_next = sorted(connections.get(li_next, set()))
        
        # Find overlapping and adjacent right indices
        all_right = sorted(set(right_conn_curr) | set(right_conn_next))
        
        if len(all_right) < 1:
            continue
        
        # Create triangles spanning the gap
        for i, ri in enumerate(all_right):
            # Triangle type 1: left[li], left[li+1], right[ri]
            if li in connections and ri in connections[li]:
                if li_next in connections and ri in connections[li_next]:
                    # Both left points connect to this right point - make triangle
                    face = (left_indices[li], left_indices[li_next], right_indices[ri])
                    if len(set(face)) == 3:
                        faces.append(face)
            
            # Triangle type 2: left[li], right[ri], right[ri+1]
            if i < len(all_right) - 1:
                ri_next = all_right[i + 1]
                if ri_next == ri + 1:  # Adjacent right points
                    if (li in connections and ri in connections[li] and 
                        li in connections and ri_next in connections[li]):
                        face = (left_indices[li], right_indices[ri], right_indices[ri_next])
                        if len(set(face)) == 3:
                            faces.append(face)
    
    # Create quad faces where possible
    quad_faces = []
    for li in range(max_left):
        li_next = li + 1
        right_curr = sorted(connections.get(li, set()))
        right_next = sorted(connections.get(li_next, set()))
        
        for ri in right_curr:
            ri_next = ri + 1
            if ri_next in right_next and ri_next <= max_right:
                # Check if we can form a quad
                if ri in right_next or ri_next in right_curr:
                    # Create quad: left[li], right[ri], right[ri+1], left[li+1]
                    quad = (left_indices[li], right_indices[ri], 
                           right_indices[ri_next], left_indices[li_next])
                    if len(set(quad)) == 4:
                        quad_faces.append(quad)
    
    # Prefer quads over triangles where they exist
    if quad_faces:
        return quad_faces
    
    return faces


# =============================================================================
# PARENT & ATTACHMENT LOGIC
# =============================================================================

def find_global_parent_bone(chains, selected_bones):
    """
    Find the parent bone of the "top-most" root bone in the selection.
    Top-most is determined by the highest Z coordinate of the bone head.
    """
    if not chains:
        return None
    
    # Find the root bone with highest Z coordinate (top-most)
    topmost_root = None
    highest_z = float('-inf')
    
    for chain in chains:
        if chain:
            root_bone = chain[0]['bone']
            z_coord = root_bone.bone.head_local.z
            if z_coord > highest_z:
                highest_z = z_coord
                topmost_root = root_bone
    
    if topmost_root is None:
        return None
    
    # Return the parent of the topmost root
    return topmost_root.parent


def cleanup_previous_mesh():
    """
    Remove previous Skeletal_Mesh_Guide objects if they exist.
    """
    objects_to_remove = [obj for obj in bpy.data.objects 
                         if obj.name.startswith(MESH_GUIDE_NAME)]
    
    for obj in objects_to_remove:
        # Remove mesh data
        mesh_data = obj.data
        bpy.data.objects.remove(obj, do_unlink=True)
        if mesh_data and mesh_data.users == 0:
            bpy.data.meshes.remove(mesh_data)


def create_mesh_with_bmesh(vertices, edges, faces, name):
    """
    Create a mesh object using bmesh.
    """
    # Create new mesh
    mesh = bpy.data.meshes.new(name + "_mesh")
    obj = bpy.data.objects.new(name, mesh)
    
    # Link to scene
    bpy.context.collection.objects.link(obj)
    
    # Create bmesh and add geometry
    bm = bmesh.new()
    
    # Add vertices
    bm_verts = []
    for v in vertices:
        bm_verts.append(bm.verts.new(v))
    
    bm.verts.ensure_lookup_table()
    
    # Add edges
    added_edges = set()
    for e in edges:
        if e[0] != e[1] and (e[0], e[1]) not in added_edges and (e[1], e[0]) not in added_edges:
            try:
                bm.edges.new((bm_verts[e[0]], bm_verts[e[1]]))
                added_edges.add((e[0], e[1]))
            except ValueError:
                pass  # Edge already exists
    
    bm.edges.ensure_lookup_table()
    
    # Add faces
    for f in faces:
        if len(set(f)) >= 3:
            try:
                face_verts = [bm_verts[i] for i in f]
                bm.faces.new(face_verts)
            except ValueError:
                pass  # Face already exists or invalid
    
    # Remove duplicate vertices
    bmesh.ops.remove_doubles(bm, verts=bm_verts, dist=0.0001)
    
    # Write to mesh
    bm.to_mesh(mesh)
    bm.free()
    
    mesh.update()
    
    return obj


def parent_mesh_to_bone(mesh_obj, armature_obj, parent_bone):
    """
    Parent the mesh to a specific bone in the armature while preserving 
    world transform.
    """
    if parent_bone is None:
        # If no parent bone, just parent to armature
        mesh_obj.parent = armature_obj
        mesh_obj.matrix_parent_inverse = armature_obj.matrix_world.inverted()
        return
    
    # Store current world matrix
    original_matrix = mesh_obj.matrix_world.copy()
    
    # Set parent
    mesh_obj.parent = armature_obj
    mesh_obj.parent_type = 'BONE'
    mesh_obj.parent_bone = parent_bone.name
    
    # Calculate the bone's world matrix
    bone_matrix = armature_obj.matrix_world @ parent_bone.bone.matrix_local
    
    # Restore world position by adjusting parent inverse
    mesh_obj.matrix_parent_inverse = bone_matrix.inverted()
    mesh_obj.matrix_world = original_matrix


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
        threshold = props.threshold
        generate_faces = props.generate_faces
        
        # Clean up previous mesh
        cleanup_previous_mesh()
        
        # Phase A: Build chains
        chains = build_chains(selected_bones)
        
        if not chains:
            self.report({'WARNING'}, "Could not identify any bone chains")
            return {'CANCELLED'}
        
        # Phase B: Sort chains by local X
        chains = sort_chains_by_local_x(chains, armature_obj)
        
        # Find parent bone before switching modes
        parent_bone = find_global_parent_bone(chains, selected_bones)
        
        # Phase C: Weave chains
        vertices, edges, faces = weave_chains(chains, threshold, generate_faces)
        
        if not vertices:
            self.report({'WARNING'}, "No geometry could be generated")
            return {'CANCELLED'}
        
        # Store original mode
        original_mode = context.mode
        
        # Switch to object mode to create mesh
        bpy.ops.object.mode_set(mode='OBJECT')
        
        # Create mesh using bmesh
        mesh_obj = create_mesh_with_bmesh(vertices, edges, faces, MESH_GUIDE_NAME)
        
        # Parent mesh to bone
        parent_mesh_to_bone(mesh_obj, armature_obj, parent_bone)
        
        # Select the armature and switch back to pose mode
        bpy.ops.object.select_all(action='DESELECT')
        armature_obj.select_set(True)
        context.view_layer.objects.active = armature_obj
        bpy.ops.object.mode_set(mode='POSE')
        
        # Report success
        parent_name = parent_bone.name if parent_bone else "Armature"
        self.report({'INFO'}, 
                    f"Created '{MESH_GUIDE_NAME}' with {len(vertices)} vertices, "
                    f"parented to '{parent_name}'")
        
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
        
        # Properties
        col = layout.column(align=True)
        col.prop(props, "threshold")
        col.prop(props, "generate_faces")
        
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
    threshold: FloatProperty(
        name="Threshold",
        description="Maximum distance to bridge gaps between chains",
        default=10.0,
        min=0.01,
        max=100.0,
        unit='LENGTH',
    )
    
    generate_faces: BoolProperty(
        name="Generate Faces",
        description="Generate filled faces instead of edges only",
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
