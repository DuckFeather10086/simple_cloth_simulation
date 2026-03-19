import bpy
import mathutils

# --- 配置：请确保你的物体名称和Shapekey名称正确 ---
object_name = "Genesis 9 Female Mesh"          # 你的G9物体名称
sk_basis = "Basic"                # 原生形态
sk_source_base = "RSSYG8FAreola Size 03"       # G8F基础形态 (减数)
vertex_group_name = ""            # Vertex Group名称，留空则不使用vertex group权重

# --- 导出配置 ---
export_prefix = "G9_"              # 要导出的shapekey前缀
export_path = ""                   # 导出路径，留空则使用Blender文件所在目录
export_scale = 2.0                 # 导出缩放比例
export_forward_axis = 'NEGATIVE_Z'         # 前向轴
export_up_axis = 'Y'               # 纵向轴

def calculate_shapekey_delta():
    obj = bpy.data.objects.get(object_name)
    if not obj or not obj.data.shape_keys:
        print("错误：未找到物体或物体没有Shapekeys")
        return

    sks = obj.data.shape_keys.key_blocks
    
    # 检查基础形态键是否存在
    if sk_basis not in sks or sk_source_base not in sks:
        print(f"错误：请确保物体拥有以下形态键: {sk_basis}, {sk_source_base}")
        return
    
    # 获取vertex group权重（如果指定了vertex group）
    vertex_weights = None
    if vertex_group_name:
        if vertex_group_name not in obj.vertex_groups:
            print(f"警告：未找到vertex group '{vertex_group_name}'，将不使用权重")
        else:
            vg = obj.vertex_groups[vertex_group_name]
            vertex_weights = [0.0] * len(obj.data.vertices)
            for i, vertex in enumerate(obj.data.vertices):
                # 查找顶点在该vertex group中的权重
                for group in vertex.groups:
                    if group.group == vg.index:
                        vertex_weights[i] = group.weight
                        break
    
    # 获取所有需要处理的形态键（排除basis和source_base）
    morph_keys = [key.name for key in sks if key.name != sk_basis and key.name != sk_source_base]
    
    if not morph_keys:
        print("错误：没有找到需要处理的形态键")
        return
    
    print(f"找到 {len(morph_keys)} 个形态键需要处理")
    
    # 获取基础数据
    basis_data = sks[sk_basis].data
    s_base_data = sks[sk_source_base].data
    
    # 对每个形态键进行处理
    processed_count = 0
    for sk_source_morph in morph_keys:
        if sk_source_morph not in sks:
            print(f"跳过：形态键 '{sk_source_morph}' 不存在")
            continue
        
        # 创建新的形态键来存储结果
        result_name = f"G9_{sk_source_morph}"
        result_key = obj.shape_key_add(name=result_name, from_mix=False)
        
        # 获取源形态键的顶点数据
        s_morph_data = sks[sk_source_morph].data
        
        # 核心向量运算（使用高精度计算）
        # 逻辑：Result = Basis + (Source_Morph - Source_Base) * vertex_group_weight
        for i in range(len(basis_data)):
            # 使用 mathutils.Vector 确保高精度计算，直接复制向量避免精度损失
            v_basis = mathutils.Vector(basis_data[i].co)
            v_s_base = mathutils.Vector(s_base_data[i].co)
            v_s_morph = mathutils.Vector(s_morph_data[i].co)
            
            # 计算偏移量（使用精确的向量减法，避免中间变量精度损失）
            delta = v_s_morph - v_s_base
            
            # 如果使用了vertex group，应用权重（使用精确的标量乘法）
            if vertex_weights is not None:
                weight = float(vertex_weights[i])
                delta = delta * weight
            
            # 计算最终结果（使用精确的向量加法）
            result = v_basis + delta
            
            # 应用到新形态键（直接赋值 Vector 对象以保持最高精度）
            result_key.data[i].co = result
        
        processed_count += 1
        print(f"已处理：{result_name}")

    print(f"计算成功！共生成 {processed_count} 个新的形态键。")

def convert_axis(axis_str):
    """将轴方向字符串转换为Blender 4.4所需的格式"""
    axis_map = {
        'X': 'X',
        'Y': 'Y',
        'Z': 'Z',
        '-X': 'NEGATIVE_X',
        '-Y': 'NEGATIVE_Y',
        '-Z': 'NEGATIVE_Z',
        'NEGATIVE_X': 'NEGATIVE_X',
        'NEGATIVE_Y': 'NEGATIVE_Y',
        'NEGATIVE_Z': 'NEGATIVE_Z'
    }
    return axis_map.get(axis_str.upper(), axis_str)

def export_shapekeys_to_obj():
    """批量导出指定前缀的shapekey为obj文件"""
    import os
    
    obj = bpy.data.objects.get(object_name)
    if not obj or not obj.data.shape_keys:
        print("错误：未找到物体或物体没有Shapekeys")
        return
    
    # 获取所有以指定前缀开头的shapekey
    sks = obj.data.shape_keys.key_blocks
    export_keys = [key.name for key in sks if key.name.startswith(export_prefix)]
    
    if not export_keys:
        print(f"错误：没有找到以 '{export_prefix}' 开头的形态键")
        return
    
    print(f"找到 {len(export_keys)} 个形态键需要导出")
    
    # 确定导出路径
    if not export_path:
        # 使用Blender文件所在目录
        blend_filepath = bpy.data.filepath
        if blend_filepath:
            base_path = os.path.dirname(blend_filepath)
        else:
            base_path = os.path.expanduser("~")
    else:
        base_path = export_path
    
    # 确保导出目录存在
    os.makedirs(base_path, exist_ok=True)
    
    # 对每个shapekey进行导出
    exported_count = 0
    for sk_name in export_keys:
        if sk_name not in sks:
            continue
            
        try:
            # 创建对象的浅拷贝
            obj_copy = obj.copy()
            obj_copy.data = obj.data.copy()
            bpy.context.collection.objects.link(obj_copy)
            
            # 确保拷贝对象被选中并激活
            bpy.context.view_layer.objects.active = obj_copy
            obj_copy.select_set(True)
            
            # 获取拷贝对象的shapekey并应用
            if obj_copy.data.shape_keys:
                sks_copy = obj_copy.data.shape_keys.key_blocks
                basis_key = sks_copy[0]  # Basis是第一个
                
                # 重置所有shapekey值
                for key in sks_copy:
                    key.value = 0.0
                
                # 设置当前shapekey为1.0
                if sk_name in sks_copy:
                    target_key = sks_copy[sk_name]
                    target_key.value = 1.0
                    
                    # 直接将shapekey的变形应用到mesh顶点
                    # 计算：final_vertex = basis_vertex + (target_vertex - basis_vertex) * value
                    mesh = obj_copy.data
                    for i, vertex in enumerate(mesh.vertices):
                        basis_co = mathutils.Vector(basis_key.data[i].co)
                        target_co = mathutils.Vector(target_key.data[i].co)
                        # 应用变形到顶点坐标
                        mesh.vertices[i].co = basis_co + (target_co - basis_co) * target_key.value
                    
                    # 删除所有shapekey，保留apply后的mesh
                    bpy.context.view_layer.objects.active = obj_copy
                    bpy.ops.object.shape_key_remove(all=True)
            
            # 构建导出文件路径
            filename = f"{sk_name}.obj"
            filepath = os.path.join(base_path, filename)
            
            # 转换轴方向格式为Blender 4.4所需格式
            forward_axis = convert_axis(export_forward_axis)
            up_axis = convert_axis(export_up_axis)
            
            # 导出obj文件 (Blender 4.4)
            bpy.ops.wm.obj_export(
                filepath=filepath,
                check_existing=False,
                export_selected_objects=True,
                export_animation=False,
                export_materials=False,
                export_uv=True,
                export_normals=True,
                export_colors=False,
                export_triangulated_mesh=False,
                export_curves_as_nurbs=False,
                export_vertex_groups=False,
                export_smooth_groups=False,
                global_scale=export_scale,
                path_mode='AUTO',
                forward_axis=forward_axis,
                up_axis=up_axis
            )
            
            # 删除临时拷贝对象
            bpy.data.objects.remove(obj_copy, do_unlink=True)
            
            exported_count += 1
            print(f"已导出：{filename}")
        except Exception as e:
            print(f"导出失败 {filename}: {str(e)}")
            # 清理临时对象（如果存在）
            if 'obj_copy' in locals():
                try:
                    bpy.data.objects.remove(obj_copy, do_unlink=True)
                except:
                    pass
    
    print(f"导出完成！共导出 {exported_count} 个obj文件到：{base_path}")

# 取消注释下面这行来执行导出
# export_shapekeys_to_obj()

calculate_shapekey_delta()