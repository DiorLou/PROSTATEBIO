import numpy as np
import pytransform3d.rotations as pyrot

def calculate_ultrasound_plane_normal(tcp_rx_deg, tcp_ry_deg, tcp_rz_deg):
    """
    根据工具末端的欧拉角，计算超声平面的法向量。

    定义：超声平面的法向量为当前TCP姿态的Y轴的反方向。

    Args:
        tcp_rx_deg (float): 工具端 Roll 角（绕世界X轴旋转），单位为度。
        tcp_ry_deg (float): 工具端 Pitch 角（绕世界Y轴旋转），单位为度。
        tcp_rz_deg (float): 工具端 Yaw 角（绕世界Z轴旋转），单位为度。

    Returns:
        np.ndarray: 超声平面的法向量（TCP Y轴的反方向）在世界坐标系中的表示 (3,).
    """

    # 1. 将输入的欧拉角从度转换为弧度
    euler_angles_rad = np.deg2rad([tcp_rx_deg, tcp_ry_deg, tcp_rz_deg])
    
    # 2. 将欧拉角转换为工具末端在世界坐标系中的旋转矩阵
    # 假设旋转顺序是 Roll (X), Pitch (Y), Yaw (Z) (SXYZ 外部旋转)
    tool_rotation_matrix = pyrot.matrix_from_euler(euler_angles_rad, 0, 1, 2, extrinsic=True)
    
    # 定义旋转角度（弧度）
    angle_offset_rad = np.deg2rad(-22.5) 
    
    # 3. 构造绕自身Z轴的旋转矩阵 (R_Z)
    # 旋转轴为 [0, 0, 1] (工具Z轴)
    z_axis_angle = np.array([0, 0, 1, angle_offset_rad])
    rotation_delta_matrix = pyrot.matrix_from_axis_angle(z_axis_angle)

    # 4. 右乘：将旋转应用到自身坐标系 (R_new = R_base * R_delta)
    # R_new 代表了新的超声平面姿态
    new_ultrasound_rotation_matrix = np.dot(tool_rotation_matrix, rotation_delta_matrix)
    
    # 3. 提取工具坐标系Y轴在世界坐标系中的方向向量
    # Y轴向量是旋转矩阵的第二列 (索引为 1)
    tool_y_axis_in_world_frame = new_ultrasound_rotation_matrix[:, 1]

    # 4. 超声平面的法向量 = 工具Y轴的反方向
    ultrasound_normal_in_world_frame = tool_y_axis_in_world_frame

    return ultrasound_normal_in_world_frame

def calculate_rotation_for_plane_alignment(a_point, o_point, e_point, initial_rpy_deg):
    """
    计算使超声平面与AOE平面重合所需的关节6旋转角度。

    Args:
        a_point (np.ndarray): 机器人 A 点的坐标 (3,).
        o_point (np.ndarray): 机器人 O 点（即 TCP 点）的坐标 (3,).
        e_point (np.ndarray): 机器人 End-effect 点的坐标 (3,).
        initial_rpy_deg (np.ndarray): 初始 TCP 姿态的欧拉角 [Rx, Ry, Rz]，单位为度。

    Returns:
        float: 关节6需要旋转的角度（单位：度）。
    """
    # 1. 计算 AOE 平面的法向量
    ea_vector = a_point - e_point
    eo_vector = o_point - e_point
    
    # 检查 A,O,E 三点是否共线
    if np.linalg.norm(np.cross(ea_vector, eo_vector)) < 1e-6:
        return 0.0, "AOE平面法向量为零，无法对齐。"
    
    aoe_plane_normal = np.cross(ea_vector, eo_vector)
    aoe_plane_normal = aoe_plane_normal / np.linalg.norm(aoe_plane_normal)

    # 2. 计算超声平面的法向量
    ultrasound_plane_normal = calculate_ultrasound_plane_normal(
        initial_rpy_deg[0], initial_rpy_deg[1], initial_rpy_deg[2])

    # 3. 计算旋转角度和方向
    initial_rpy_rad = np.deg2rad(initial_rpy_deg)
    tool_rotation_matrix = pyrot.matrix_from_euler(initial_rpy_rad, 0, 1, 2, extrinsic=True)
    tool_z_axis = tool_rotation_matrix[:, 2]

    projected_aoe_normal = aoe_plane_normal - np.dot(aoe_plane_normal, tool_z_axis) * tool_z_axis
    projected_ultrasound_normal = ultrasound_plane_normal - np.dot(ultrasound_plane_normal, tool_z_axis) * tool_z_axis
    
    # 避免除零错误
    if np.linalg.norm(projected_ultrasound_normal) < 1e-6 or np.linalg.norm(projected_aoe_normal) < 1e-6:
        return 0.0, "投影法向量为零，无需旋转。"

    cos_alpha = np.dot(projected_ultrasound_normal, projected_aoe_normal) / (np.linalg.norm(projected_ultrasound_normal) * np.linalg.norm(projected_aoe_normal))
    cos_alpha = np.clip(cos_alpha, -1.0, 1.0)
    angle_rad = np.arccos(cos_alpha)
    
    cross_product_direction = np.dot(np.cross(projected_ultrasound_normal, projected_aoe_normal), tool_z_axis)
    if cross_product_direction < 0:
        angle_rad = -angle_rad
        
    return np.rad2deg(angle_rad)

def calculate_new_rpy_for_b_point(a_point, o_point, b_point, initial_rpy_deg):
    """
    计算使超声平面包含病灶点B所需的新工具姿态（欧拉角）。

    该函数通过绕OA轴旋转超声平面来计算新的姿态。
    修改后的逻辑：旋转超声平面法向量，使其对齐 OAB 平面法向量在旋转平面上的投影。
    
    Args:
        a_point (np.ndarray): 机器人 A 点的坐标 (3,).
        o_point (np.ndarray): 机器人 O 点（即 TCP 点）的坐标 (3,).
        b_point (np.ndarray): 病灶点 B 的坐标 (3,).
        initial_rpy_deg (np.ndarray): 初始 TCP 姿态的欧拉角 [Rx, Ry, Rz]，单位为度。

    Returns:
        np.ndarray: 包含新姿态的欧拉角 [Rx, Ry, Rz]，单位为度。
    """
    a_point = np.array(a_point, dtype=np.float64)
    o_point = np.array(o_point, dtype=np.float64)
    b_point = np.array(b_point, dtype=np.float64)
    initial_rpy_deg = np.array(initial_rpy_deg, dtype=np.float64)
    
    # 1. 计算旋转轴：OA向量
    oa_vector = a_point - o_point
    if np.linalg.norm(oa_vector) < 1e-6:
        raise ValueError("OA向量为零，无法定义旋转轴。")

    oa_unit_vector = oa_vector / np.linalg.norm(oa_vector)

    # 2. 计算超声平面的初始法向量
    ultrasound_normal = calculate_ultrasound_plane_normal(
        initial_rpy_deg[0], initial_rpy_deg[1], initial_rpy_deg[2])
    
    # 3. 计算 OAB 平面法向量作为目标（N_OAB = OB x OA）
    ob_vector = b_point - o_point
    
    if np.linalg.norm(ob_vector) < 1e-6:
        # B点与O点重合，返回单位矩阵
        return [np.identity(4)]

    # N_OAB 是 OAB 平面的法向量，使用用户建议的顺序：OB x OA
    n_oab_vector = np.cross(oa_vector, ob_vector)
    
    if np.linalg.norm(n_oab_vector) < 1e-6:
        # O, A, B 三点共线，OAB 平面法向量为零
        return [np.identity(4)]

    # 4. 计算总旋转所需的角度
    # 将超声平面法向量和 OAB 法向量投影到垂直于OA轴的平面上
    projected_ultrasound_normal = ultrasound_normal - np.dot(ultrasound_normal, oa_unit_vector) * oa_unit_vector
    
    # 投影目标向量 (OAB平面法向量)
    projected_target_vector = n_oab_vector - np.dot(n_oab_vector, oa_unit_vector) * oa_unit_vector
    
    # 检查投影向量是否为零
    if np.linalg.norm(projected_ultrasound_normal) < 1e-6 or np.linalg.norm(projected_target_vector) < 1e-6:
        raise ValueError("投影向量为零，无法计算旋转。")

    # 使用反正切函数计算带方向的旋转角度
    y_axis_in_plane = np.cross(oa_unit_vector, projected_ultrasound_normal)
    
    x_component = np.dot(projected_target_vector, projected_ultrasound_normal)
    y_component = np.dot(projected_target_vector, y_axis_in_plane)

    # np.arctan2 能够计算出绕OA轴的总旋转角度 (弧度)
    rotation_angle_rad = np.arctan2(y_component, x_component)
    total_angle_deg = np.rad2deg(rotation_angle_rad)
    
    print("绕OA轴的旋转角度(度):", end='')
    print(total_angle_deg)
    
    # --- 步进角度生成逻辑 (根据用户要求修改为 0.5 度步长，跳过最后一个 0.5 倍数) ---
    step_size_deg = 0.5
    abs_angle_deg = abs(total_angle_deg)
    sign = np.sign(total_angle_deg) if abs(total_angle_deg) > 1e-6 else 1.0 

    step_angles_deg = []

    # 计算要生成的最大完整 0.5 度步长的索引 k_limit
    # 目标是生成 0.5*k 直到 0.5*k <= abs_angle_deg - step_size_deg (即 0.5*k 至少比总角小一个步长)
    max_full_step_deg = abs_angle_deg - step_size_deg
    
    # k_limit = floor(max_full_step_deg / step_size_deg)
    # 例如：5.8 -> 5.3 -> k_limit=10.0 -> k_limit=10
    # 例如：5.5 -> 5.0 -> k_limit=10.0 -> k_limit=10
    # 例如：0.4 -> -0.1 -> k_limit=-0.2 -> k_limit=0 (因为 int() 截断，实际上这里应该是 -1)
    k_limit = int(np.floor(max_full_step_deg / step_size_deg + 1e-6))
    
    # 1. 生成完整的 0.5 度步长
    for k in range(1, k_limit + 1):
        step_angles_deg.append(sign * k * step_size_deg)
        
    # 2. 最后一个步骤总是总角度 (除非总角度接近 0)
    if abs(total_angle_deg) > 1e-6:
        # 避免在 k_limit 恰好生成了 total_angle_deg 时重复添加
        # 只有在最后一个完整步长不等于总角度时才添加总角度
        if not (step_angles_deg and np.isclose(step_angles_deg[-1], total_angle_deg)):
             step_angles_deg.append(total_angle_deg)
             
    if not step_angles_deg:
        # 极小角度时确保至少有一个步骤 (0.0 或总角度)
        if abs(total_angle_deg) > 1e-6:
            step_angles_deg = [total_angle_deg]
        else:
             step_angles_deg = [0.0]

    print("step_angles_deg = ", end='')
    print(step_angles_deg)
    
    # 3. 构造累积旋转矩阵列表
    delta_rotation_matrices = []
    
    # 绕OA向量旋转指定角度
    for angle_deg in step_angles_deg:
        angle_rad = np.deg2rad(angle_deg)
        # 使用 Rodrigous 公式构造旋转矩阵 (绕OA向量旋转 angle_rad)
        rotation_axis_angle = np.array([oa_vector[0], oa_vector[1], oa_vector[2], angle_rad])
        delta_rotation_matrix = pyrot.matrix_from_axis_angle(rotation_axis_angle)
        delta_rotation_matrices.append(delta_rotation_matrix)
    
    return delta_rotation_matrices

def get_final_tcp_e_position_after_delta_rotation(initial_tcp_pose, delta_rotation_matrix, o_vars):
    
    small_p0 = initial_tcp_pose
    big_p0 =o_vars

    # 手动实现向量减法
    diff = [small_p0[i] - big_p0[i] for i in range(len(small_p0))]

    P_final = np.dot(delta_rotation_matrix[:3,:3], diff) + big_p0 # 修正：只使用旋转矩阵的前三行三列
    return P_final
    

if __name__ == '__main__':
    # --- 演示代码省略 ---
    pass