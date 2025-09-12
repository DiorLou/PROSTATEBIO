import numpy as np
import pytransform3d.rotations as pyrot

def calculate_ultrasound_plane_normal(tcp_rx_deg, tcp_ry_deg, tcp_rz_deg):
    """
    根据工具末端的欧拉角，计算超声平面的法向量。

    超声平面的法向量位于工具端X-Y平面上，与工具端Y轴夹角为22.5度。

    Args:
        tcp_rx_deg (float): 工具端 Roll 角（绕世界X轴旋转），单位为度。
        tcp_ry_deg (float): 工具端 Pitch 角（绕世界Y轴旋转），单位为度。
        tcp_rz_deg (float): 工具端 Yaw 角（绕世界Z轴旋转），单位为度。

    Returns:
        np.ndarray: 超声平面的法向量在世界坐标系中的表示 (3,).
    """
    # 1. 将输入的欧拉角从度转换为弧度
    euler_angles_rad = np.deg2rad([tcp_rx_deg, tcp_ry_deg, tcp_rz_deg])
    
    # 2. 将欧拉角转换为工具末端在世界坐标系中的旋转矩阵
    # 使用你提供的参数格式进行调用：
    # e: 欧拉角数组（弧度）
    # i, j, k: 旋转轴的索引 (0=X, 1=Y, 2=Z)
    # extrinsic: True 表示外部旋转
    # 假设旋转顺序是 Roll (X), Pitch (Y), Yaw (Z)
    tool_rotation_matrix = pyrot.matrix_from_euler(euler_angles_rad, 0, 1, 2, extrinsic=True)

    # 3. 定义超声平面的法向量在工具坐标系中的表示
    # 法向量位于X-Y平面，与Y轴夹角为22.5度。
    # 这意味着它绕工具坐标系的Z轴旋转了22.5度，从Y轴方向偏转。
    # x分量为 sin(22.5), y分量为 cos(22.5)
    
    angle_offset_rad = np.deg2rad(22.5)
    ultrasound_normal_in_tool_frame = np.array([np.sin(angle_offset_rad), np.cos(angle_offset_rad), 0.0])

    # 4. 将法向量从工具坐标系转换到世界坐标系
    # 这通过将向量与工具的旋转矩阵相乘来实现
    ultrasound_normal_in_world_frame = np.dot(tool_rotation_matrix, ultrasound_normal_in_tool_frame)

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

if __name__ == '__main__':
    # --- 演示如何使用此函数 ---

    print("--- 演示1：工具姿态为零（与世界坐标系对齐）---")
    # 当工具姿态为零时，超声法向量应为 [sin(22.5), cos(22.5), 0]
    rx, ry, rz = 0.0, 0.0, 0.0
    normal_vector_1 = calculate_ultrasound_plane_normal(rx, ry, rz)
    print(f"输入欧拉角 (Rx, Ry, Rz): {rx}, {ry}, {rz}")
    print(f"超声平面的法向量: {normal_vector_1}")
    print(f"验证: np.sin(np.deg2rad(22.5)) = {np.sin(np.deg2rad(22.5)):.6f}")
    print(f"验证: np.cos(np.deg2rad(22.5)) = {np.cos(np.deg2rad(22.5)):.6f}")

    print("\n--- 演示2：工具绕世界Z轴旋转90度 ---")
    # 当工具绕世界Z轴旋转90度时，工具的X轴变为世界坐标系的Y轴，Y轴变为世界坐标系的-X轴。
    # 此时，超声法向量应为 [-cos(22.5), sin(22.5), 0]
    rx, ry, rz = 0.0, 0.0, 90.0
    normal_vector_2 = calculate_ultrasound_plane_normal(rx, ry, rz)
    print(f"输入欧拉角 (Rx, Ry, Rz): {rx}, {ry}, {rz}")
    print(f"超声平面的法向量: {normal_vector_2}")

    print("\n--- 演示3：工具绕世界X轴旋转90度 ---")
    # 当工具绕世界X轴旋转90度时，工具的Y轴变为世界坐标系的Z轴。
    # 超声平面的法向量会从XY平面转到XZ平面。
    rx, ry, rz = 90.0, 0.0, 0.0
    normal_vector_3 = calculate_ultrasound_plane_normal(rx, ry, rz)
    print(f"输入欧拉角 (Rx, Ry, Rz): {rx}, {ry}, {rz}")
    print(f"超声平面的法向量: {normal_vector_3}")