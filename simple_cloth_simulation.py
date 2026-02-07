# SPDX-License-Identifier: GPL-2.0-or-later

from enum import Enum
from bpy.app.handlers import persistent
from mathutils import (
    Vector,
    Euler,
)
from bpy.types import (
    Operator,
    Panel,
    PropertyGroup,
)
from bpy.props import (
    FloatProperty,
    IntProperty,
    BoolProperty,
    PointerProperty,
)
import bpy
bl_info = {
    "name": "Simple Cloth Simulation",
    "author": "Duck Feather",
    "version": (1, 0, 0),
    "blender": (2, 80, 0),
    "location": "Pose Mode > Sidebar > Simple Cloth Tab",
    "description": "Creates a cloth simulation mesh from selected bones",
    "warning": "",
    "doc_url": "",
    "category": "Animation",
}


class Rig_type(Enum):
    HORSE = 0
    SHARK = 1
    BIRD = 2
    CAT = 3
    BIPED = 4
    HUMAN = 5
    WOLF = 6
    QUAD = 7
    LEGACY = 8
    PITCHIPOY = 9
    OTHER = 10


rig_type = Rig_type.OTHER


class Idx_Store(object):
    def __init__(self, rig_type):
        self.rig_type = rig_type
        self.hand_r_merge = []
        self.hand_l_merge = []
        self.hands_pretty = []
        self.root = []

        if not self.rig_type == Rig_type.LEGACY and \
                not self.rig_type == Rig_type.HUMAN and \
                not self.rig_type == Rig_type.PITCHIPOY:
            return

        if self.rig_type == Rig_type.LEGACY:
            self.hand_l_merge = [7, 12, 16, 21, 26, 27]
            self.hand_r_merge = [30, 31, 36, 40, 45, 50]
            self.hands_pretty = [6, 29]
            self.root = [59]

        if self.rig_type == Rig_type.HUMAN or self.rig_type == Rig_type.PITCHIPOY:
            self.hand_l_merge = [9, 10, 15, 19, 24, 29]
            self.hand_r_merge = [32, 33, 37, 42, 47, 52]
            self.hands_pretty = [8, 31]
            self.root = [56]

    def get_all_idx(self):
        return self.hand_l_merge, self.hand_r_merge, self.hands_pretty, self.root

    def get_hand_l_merge_idx(self):
        return self.hand_l_merge

    def get_hand_r_merge_idx(self):
        return self.hand_r_merge

    def get_hands_pretty_idx(self):
        return self.hands_pretty

    def get_root_idx(self):
        return self.root


# initialize properties
def init_props():
    # additional check - this should be a rare case if the handler
    # wasn't removed for some reason and the add-on is not toggled on/off
    if hasattr(bpy.types.Scene, "simple_cloth_sim"):
        scn = bpy.context.scene.simple_cloth_sim

        scn.connect_mesh = False
        scn.connect_parents = False
        scn.generate_all = False
        scn.thickness = 0.8
        scn.finger_thickness = 0.25
        scn.apply_mod = True
        scn.parent_armature = True
        scn.sub_level = 1


# selects vertices
def select_vertices(mesh_obj, idx):
    bpy.context.view_layer.objects.active = mesh_obj
    mode = mesh_obj.mode
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='DESELECT')
    bpy.ops.object.mode_set(mode='OBJECT')

    for i in idx:
        mesh_obj.data.vertices[i].select = True

    selectedVerts = [v.index for v in mesh_obj.data.vertices if v.select]

    bpy.ops.object.mode_set(mode=mode)
    return selectedVerts


def identify_rig():
    if 'rigify_layers' not in bpy.context.object.data:
        return Rig_type.OTHER  # non recognized

    LEGACY_LAYERS_SIZE = 28
    layers = bpy.context.object.data['rigify_layers']

    for type, rig in enumerate(rigify_data):
        index = 0

        for props in layers:
            if len(layers) == LEGACY_LAYERS_SIZE and 'group_prop' not in props:

                if props['row'] != rig[index][0] or rig[index][1] is not None:
                    break

            elif (props['row'] != rig[index][0]) or (props['group_prop'] != rig[index][1]):
                break

            # SUCCESS if reach the end
            if index == len(layers) - 1:
                return Rig_type(type)

            index = index + 1

    return Rig_type.OTHER


def prepare_ignore_list(rig_type, bones):
    # detect the head, face, hands, breast, heels or other exceptionary bones to exclusion or customization
    common_ignore_list = ['eye', 'heel', 'breast', 'root']

    # edit these lists to suits your taste

    horse_ignore_list = ['chest', 'belly',
                         'pelvis', 'jaw', 'nose', 'skull', 'ear.']

    shark_ignore_list = ['jaw']

    bird_ignore_list = [
        'face', 'pelvis', 'nose', 'lip', 'jaw', 'chin', 'ear.', 'brow',
        'lid', 'forehead', 'temple', 'cheek', 'teeth', 'tongue', 'beak'
    ]
    cat_ignore_list = [
        'face', 'belly' 'pelvis.C', 'nose', 'lip', 'jaw', 'chin', 'ear.', 'brow',
        'lid', 'forehead', 'temple', 'cheek', 'teeth', 'tongue'
    ]
    biped_ignore_list = ['pelvis']

    human_ignore_list = [
        'face', 'pelvis', 'nose', 'lip', 'jaw', 'chin', 'ear.', 'brow',
        'lid', 'forehead', 'temple', 'cheek', 'teeth', 'tongue'
    ]
    wolf_ignore_list = [
        'face', 'pelvis', 'nose', 'lip', 'jaw', 'chin', 'ear.', 'brow',
        'lid', 'forehead', 'temple', 'cheek', 'teeth', 'tongue'
    ]
    quad_ignore_list = [
        'face', 'pelvis', 'nose', 'lip', 'jaw', 'chin', 'ear.', 'brow',
        'lid', 'forehead', 'temple', 'cheek', 'teeth', 'tongue'
    ]
    rigify_legacy_ignore_list = []

    pitchipoy_ignore_list = [
        'face', 'pelvis', 'nose', 'lip', 'jaw', 'chin', 'ear.', 'brow',
        'lid', 'forehead', 'temple', 'cheek', 'teeth', 'tongue'
    ]

    other_ignore_list = []

    ignore_list = common_ignore_list

    if rig_type == Rig_type.HORSE:
        ignore_list = ignore_list + horse_ignore_list
        print("RIDER OF THE APOCALYPSE")
    elif rig_type == Rig_type.SHARK:
        ignore_list = ignore_list + shark_ignore_list
        print("DEADLY JAWS")
    elif rig_type == Rig_type.BIRD:
        ignore_list = ignore_list + bird_ignore_list
        print("WINGS OF LIBERTY")
    elif rig_type == Rig_type.CAT:
        ignore_list = ignore_list + cat_ignore_list
        print("MEOW")
    elif rig_type == Rig_type.BIPED:
        ignore_list = ignore_list + biped_ignore_list
        print("HUMANOID")
    elif rig_type == Rig_type.HUMAN:
        ignore_list = ignore_list + human_ignore_list
        print("JUST A HUMAN AFTER ALL")
    elif rig_type == Rig_type.WOLF:
        ignore_list = ignore_list + wolf_ignore_list
        print("WHITE FANG")
    elif rig_type == Rig_type.QUAD:
        ignore_list = ignore_list + quad_ignore_list
        print("MYSTERIOUS CREATURE")
    elif rig_type == Rig_type.LEGACY:
        ignore_list = ignore_list + rigify_legacy_ignore_list
        print("LEGACY RIGIFY")
    elif rig_type == Rig_type.PITCHIPOY:
        ignore_list = ignore_list + pitchipoy_ignore_list
        print("PITCHIPOY")
    elif rig_type == Rig_type.OTHER:
        ignore_list = ignore_list + other_ignore_list
        print("rig non recognized...")

    return ignore_list


# generates edges from vertices used by skin modifier
def generate_edges(mesh, shape_object, bones, scale, connect_mesh=False, connect_parents=False,
                   head_ornaments=False, generate_all=False, chain_count=6):
    """
    This function adds vertices for all heads and tails
    """
    # scene preferences

    alternate_scale_list = []

    me = mesh
    verts = []
    edges = []
    idx = 0
    alternate_scale_idx_list = list()

    bone_chain_lentgh_map = []
    bone_chain_edges = []
    faces = []
    rig_type = identify_rig()

    parent_bone_name = ""

    # edge generator loop
    for b in bones:
        # look for rig's hands and their childs
        if 'hand' in b.name.lower():
            # prepare the list
            for c in b.children_recursive:
                alternate_scale_list.append(c.name)

        found = False

        if found and generate_all is False:
            continue

        # fix for drawing rootbone and relationship lines
        if 'root' in b.name.lower() and generate_all is False:
            continue

        vert1 = b.head
        vert2 = b.tail
        verts.append(vert1)
        verts.append(vert2)

        edges.append([idx, idx + 1])
        bone_chain_edges.append([idx, idx + 1])
        # faces.append((idx, idx + 1, idx+6, idx+6+1))

        if b.children is None:
            bone_chain_lentgh_map.append(bone_chain_edges)
            bone_chain_edges = []

        for a in alternate_scale_list:
            if b.name == a:
                alternate_scale_idx_list.append(idx)
                alternate_scale_idx_list.append(idx + 1)

        idx = idx + 2

    for i in range(0, len(bone_chain_edges)-chain_count):
        # for i in range(0,chain_count):
        if len(bone_chain_edges) >= i+chain_count:
            face = (bone_chain_edges[i][0], bone_chain_edges[i][1],
                    bone_chain_edges[i+chain_count][1], bone_chain_edges[i+chain_count][0])
            faces.append(face)

    # print("faces")
    # print(faces)
    me.from_pydata(verts, edges, faces)

    # Update mesh with new data
    me.update()

    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='DESELECT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.remove_doubles()
    bpy.ops.object.mode_set(mode='OBJECT')

    # set object scale exact as armature's scale
    shape_object.scale = scale

    idx = 0
    pin_group = []
    for b in bones:
        if idx == 0:
            parent_bone_name = b.parent.name

        if idx % (chain_count+1) == 0:
            pin_group.append(idx)
            idx = idx + 1

        # add vertex group
        group_name = 'group_'+str(idx)+"_"+b.name.lower()
        new_vertex_group = shape_object.vertex_groups.new(name=group_name)
        vertex_group_data = [idx]
        new_vertex_group.add(vertex_group_data, 1.0, 'REPLACE')

        # add constraints
        crc = b.constraints.new('DAMPED_TRACK')
        # give it a target bone
        crc.target = shape_object
        # note subtarget uses name not object.
        crc.subtarget = group_name
        crc.track_axis = "TRACK_Y"

        idx = idx + 1

    new_vertex_group = shape_object.vertex_groups.new(name="pin_group")
    new_vertex_group.add(pin_group, 1.0, 'REPLACE')

    # print("parent_bone_name")
    # print(parent_bone_name)

    return alternate_scale_idx_list, rig_type, parent_bone_name


def generate_plate(shape_object, size, thickness=0.8, finger_thickness=0.25, sub_level=1,
                   connect_mesh=True, connect_parents=False, generate_all=False, apply_mod=True,
                   alternate_scale_idx_list=[], rig_type=0, bones=[]):
    """
    This function adds modifiers for generated edges
    """
    total_bones_num = bpy.context.selected_pose_bones_from_active_object
    selected_bones_num = len(bones)

    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='DESELECT')

    # add skin modifier
    # shape_object.modifiers.new("Skin", 'SKIN')
    # bpy.ops.mesh.select_all(action='SELECT')

    override = bpy.context.copy()
    for area in bpy.context.screen.areas:
        if area.type == 'VIEW_3D':
            for region in area.regions:
                if region.type == 'WINDOW':
                    override['area'] = area
                    override['region'] = region
                    override['edit_object'] = bpy.context.edit_object
                    override['scene'] = bpy.context.scene
                    override['active_object'] = shape_object
                    override['object'] = shape_object
                    override['modifier'] = bpy.context.object.modifiers
                    break

    if connect_mesh:
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.select_all(action='DESELECT')
        bpy.ops.mesh.select_all(action='SELECT')
        bpy.ops.mesh.remove_doubles()

    idx_store = Idx_Store(rig_type)

    # fix rigify and pitchipoy hands topology
    if connect_mesh and connect_parents and generate_all is False and \
            (rig_type == Rig_type.LEGACY or rig_type == Rig_type.PITCHIPOY or rig_type == Rig_type.HUMAN) and \
            selected_bones_num == total_bones_num:
        # thickness will set palm vertex for both hands look pretty
        corrective_thickness = 2.5
        # left hand verts
        merge_idx = idx_store.get_hand_l_merge_idx()

        select_vertices(shape_object, merge_idx)
        bpy.ops.mesh.merge(type='CENTER')
        bpy.ops.transform.skin_resize(
            override,
            value=(corrective_thickness, corrective_thickness,
                   corrective_thickness),
            constraint_axis=(False, False, False), orient_type='GLOBAL',
            mirror=False,
            use_proportional_edit=False,
        )
        bpy.ops.mesh.select_all(action='DESELECT')

        # right hand verts
        merge_idx = idx_store.get_hand_r_merge_idx()

        select_vertices(shape_object, merge_idx)
        bpy.ops.mesh.merge(type='CENTER')
        bpy.ops.transform.skin_resize(
            override,
            value=(corrective_thickness, corrective_thickness,
                   corrective_thickness),
            constraint_axis=(False, False, False), orient_type='GLOBAL',
            mirror=False,
            use_proportional_edit=False,
        )

        # making hands even more pretty
        bpy.ops.mesh.select_all(action='DESELECT')
        hands_idx = idx_store.get_hands_pretty_idx()

        select_vertices(shape_object, hands_idx)
        # change the thickness to make hands look less blocky and more sexy
        corrective_thickness = 0.7
        bpy.ops.transform.skin_resize(
            override,
            value=(corrective_thickness, corrective_thickness,
                   corrective_thickness),
            constraint_axis=(False, False, False), orient_type='GLOBAL',
            mirror=False,
            use_proportional_edit=False,
        )
        bpy.ops.mesh.select_all(action='DESELECT')

    # todo optionally take root from rig's hip tail or head depending on scenario

    root_idx = idx_store.get_root_idx()

    if selected_bones_num == total_bones_num:
        root_idx = [0]

    if len(root_idx) > 0:
        select_vertices(shape_object, root_idx)
        bpy.ops.object.skin_root_mark(override)

    # skin in edit mode
    # add cloth modifier
    shape_object.modifiers.new("Cloth", 'CLOTH')
    # shape_object.modifiers["cloth"].levels = sub_level
    # shape_object.modifiers["cloth"].render_levels = sub_level
    bpy.context.object.modifiers["Cloth"].settings.vertex_group_mass = "pin_group"

    bpy.ops.object.mode_set(mode='OBJECT')

    # object mode apply all modifiers
    # if apply_mod:
    #     bpy.ops.object.modifier_apply(override, modifier="Skin")
    #     bpy.ops.object.modifier_apply(override, modifier="Subsurf")

    return {'FINISHED'}


def main(context):
    """
    This script will create a custom shape
    """

    # ### Check if selection is OK ###
    if len(context.selected_pose_bones) == 0 or \
            len(context.selected_objects) == 0 or \
            context.selected_objects[0].type != 'ARMATURE':
        return {'CANCELLED'}, "No bone selected or the Armature is hidden"

    scn = bpy.context.scene
    settings = scn.simple_cloth_sim

    # initialize the mesh object
    mesh_name = context.selected_objects[0].name + "_mesh"
    obj_name = context.selected_objects[0].name + "_object"
    armature_object = context.object

    origin = context.object.location
    bone_selection = context.selected_pose_bones
    oldLocation = None
    oldRotation = None
    oldScale = None
    armature_object = context.view_layer.objects.active
    armature_object.select_set(True)

    old_pose_pos = armature_object.data.pose_position
    bpy.ops.object.mode_set(mode='OBJECT')
    oldLocation = Vector(armature_object.location)
    oldRotation = Euler(armature_object.rotation_euler)
    oldScale = Vector(armature_object.scale)

    bpy.ops.object.rotation_clear(clear_delta=False)
    bpy.ops.object.location_clear(clear_delta=False)
    bpy.ops.object.scale_clear(clear_delta=False)
    # if settings.apply_mod and settings.parent_armature:
    #     armature_object.data.pose_position = 'REST'

    scale = bpy.context.object.scale
    size = bpy.context.object.dimensions[2]

    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.select_all(action='DESELECT')

    bpy.ops.object.add(type='MESH', enter_editmode=False, location=origin)

    # get the mesh object
    ob = context.view_layer.objects.active
    ob.name = obj_name
    me = ob.data
    me.name = mesh_name

    # this way we fit mesh and bvh with armature modifier correctly

    alternate_scale_idx_list, rig_type, parent_bone_name = generate_edges(
        me, ob, bone_selection, scale, settings.connect_mesh,
        settings.connect_parents, settings.head_ornaments,
        settings.generate_all, settings.sub_level
    )

    generate_plate(ob, size, settings.thickness, settings.finger_thickness, settings.sub_level,
                   settings.connect_mesh, settings.connect_parents, settings.generate_all,
                   settings.apply_mod, alternate_scale_idx_list, rig_type, bone_selection)

    # scn.objects.active = armature_object
    # ob.selected = True
    # armature_object.active = armature_object.bones[parent_bone_name]
    # bpy.ops.ob.parent_set(type='BONE_RELATIVE')
    # bpy.ops.object.parent_bone = parent_bone_name

    # ob.parent = armature_object
    # #armature_object.active = armature_object.bones
    # ob.parent_type = 'BONE_RELATIVE'
    # ob.parent_bone = parent_bone_name

    # parent mesh with armature only if modifiers are applied
    if settings.apply_mod and settings.parent_armature:
        bpy.ops.object.mode_set(mode='OBJECT')
        bpy.ops.object.select_all(action='DESELECT')
        ob.select_set(True)
        armature_object.select_set(True)
        bpy.context.view_layer.objects.active = armature_object

        # bones = bpy.data.armatures[armature_object.name].bones
        # armature_object.active = bones[parent_bone_name]
        print("BONE_RELATIVE")
        print(parent_bone_name)

        bpy.ops.object.parent_set(type='BONE')
        bpy.ops.object.parent_bone = parent_bone_name
        armature_object.data.pose_position = old_pose_pos
        armature_object.select_set(False)

    # # else:
    #     bpy.ops.object.mode_set(mode='OBJECT')
    #     ob.location = oldLocation
    #     ob.rotation_euler = oldRotation
    #     ob.scale = oldScale
    #     ob.select_set(False)
    #     armature_object.select_set(True)
    #     scn.objects.active = armature_object

    # skin in edit mode
    # add Subsurf modifier
    # shape_object.modifiers.new("Subsurf", 'SUBSURF')
    # shape_object.modifiers["Subsurf"].levels = sub_level
    # shape_object.modifiers["Subsurf"].render_levels = sub_level

    armature_object.location = oldLocation
    armature_object.rotation_euler = oldRotation
    armature_object.scale = oldScale
    bpy.ops.object.mode_set(mode='OBJECT')

    return {'FINISHED'}, me


class BONE_OT_custom_shape(Operator):
    bl_idname = "object.simple_cloth_simulation"
    bl_label = "Simple Cloth Simulation"
    bl_description = "Creates a cloth simulation mesh from the selected bones"
    bl_options = {'UNDO', 'INTERNAL'}

    @classmethod
    def poll(cls, context):
        return context.active_object is not None

    def execute(self, context):
        Mesh = main(context)
        if Mesh[0] == {'CANCELLED'}:
            self.report({'WARNING'}, Mesh[1])
            return {'CANCELLED'}
        else:
            self.report({'INFO'}, Mesh[1].name + " has been created")

            return {'FINISHED'}


class BONE_PT_custom_shape(Panel):
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Simple Cloth"
#    bl_context = "bone"
    bl_label = "Simple Cloth Simulation"
    bl_options = {'DEFAULT_CLOSED'}

    @classmethod
    def poll(cls, context):
        ob = context.object
        return ob and ob.mode == 'POSE'  # and context.bone

    def draw(self, context):
        layout = self.layout
        scn = context.scene.simple_cloth_sim

        row = layout.row()
        row.operator("object.simple_cloth_simulation",
                     text="Add Cloth Shape", icon='BONE_DATA')

        split = layout.split(factor=0.3)
        split.label(text="Thickness:")
        split.prop(scn, "thickness", text="Body", icon='MOD_SKIN')
        split.prop(scn, "finger_thickness", text="Fingers", icon='HAND')

        split = layout.split(factor=0.3)
        split.label(text="Mesh Density:")
        split.prop(scn, "sub_level", icon='MESH_ICOSPHERE')

        # row = layout.row()
        # split = layout.split(factor=0.3)
        # split.label(text="Parent Bone")
        # split.prop(scn, "parent_bones", icon='BONE_DATA')

        row = layout.row()
        row.prop(scn, "connect_mesh", icon='EDITMODE_HLT')
        row.prop(scn, "connect_parents", icon='CONSTRAINT_BONE')
        row = layout.row()
        row.prop(scn, "head_ornaments", icon='GROUP_BONE')
        row.prop(scn, "generate_all", icon='GROUP_BONE')
        row = layout.row()
        row.prop(scn, "apply_mod", icon='FILE_TICK')
        if scn.apply_mod:
            row = layout.row()
            row.prop(scn, "parent_armature", icon='POSE_HLT')


# define the scene properties in a group - call them with context.scene.simple_cloth_sim
class SimpleClothSimulation_Properties(PropertyGroup):
    sub_level: IntProperty(
        name="Chain Count",
        min=0, max=114514,
        default=1,
        description="Chain Count"
    )
    thickness: FloatProperty(
        name="Thickness",
        min=1, max=1,
        default=1,
        description="Adjust shape thickness const=1 can not change "
    )
    finger_thickness: FloatProperty(
        name="Finger Thickness",
        min=1, max=1,
        default=1,
        description="Adjust finger thickness relative to body const=1 can not change"
    )
    parent_bones: FloatProperty(
        name="Parent Bones",
        description="chooes the parent object for simulation object"
    )
    connect_mesh: BoolProperty(
        name="Solid Shape",
        default=False,
        description="Makes solid shape from bone chains"
    )
    connect_parents: BoolProperty(
        name="Fill Gaps",
        default=False,
        description="Fills the gaps between parented bones"
    )
    generate_all: BoolProperty(
        name="All Shapes",
        default=False,
        description="Generates shapes from all bones"
    )
    head_ornaments: BoolProperty(
        name="Head Ornaments",
        default=False,
        description="Includes head ornaments"
    )
    apply_mod: BoolProperty(
        name="Apply Modifiers",
        default=True,
        description="Applies Modifiers to mesh"
    )
    parent_armature: BoolProperty(
        name="Parent Armature",
        default=True,
        description="Applies mesh to Armature"
    )


# startup defaults

@persistent
def startup_init(dummy):
    init_props()


def register():
    bpy.utils.register_class(BONE_OT_custom_shape)
    bpy.utils.register_class(BONE_PT_custom_shape)
    bpy.utils.register_class(SimpleClothSimulation_Properties)

    bpy.types.Scene.simple_cloth_sim = PointerProperty(
        type=SimpleClothSimulation_Properties
    )
    # startup defaults
    bpy.app.handlers.load_post.append(startup_init)


def unregister():
    bpy.utils.unregister_class(BONE_OT_custom_shape)
    bpy.utils.unregister_class(BONE_PT_custom_shape)
    bpy.utils.unregister_class(SimpleClothSimulation_Properties)

    # cleanup the handler
    bpy.app.handlers.load_post.remove(startup_init)

    del bpy.types.Scene.simple_cloth_sim


if __name__ == "__main__":
    register()
