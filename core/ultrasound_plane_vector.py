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