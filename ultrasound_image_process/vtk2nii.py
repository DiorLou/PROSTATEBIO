import vtk
import nibabel as nib
import numpy as np
import os


def vtk_lps_to_nii_lps(vtk_file_path, output_nii_path=None):
    """
    将LPS坐标系的VTK Volume文件转换为LPS坐标系的nii.gz文件
    修复：L(x)、P(y)方向反转问题，保持Z(z)方向正确

    参数：
        vtk_file_path (str): 输入VTK文件的完整路径（LPS坐标系结构化点数据）
        output_nii_path (str, optional): 输出nii.gz路径，默认同目录替换后缀

    返回：
        str: 输出nii.gz文件路径

    异常：
        FileNotFoundError: VTK文件不存在
        ValueError: VTK无标量数据
        Exception: 转换过程异常
    """
    # 处理输出路径
    if output_nii_path is None:
        output_nii_path = os.path.splitext(vtk_file_path)[0] + ".nii.gz"

    # 校验输入文件
    if not os.path.exists(vtk_file_path):
        raise FileNotFoundError(f"VTK文件不存在：{vtk_file_path}")

    try:
        # 1. 读取VTK数据
        vtk_reader = vtk.vtkStructuredPointsReader()
        vtk_reader.SetFileName(vtk_file_path)
        vtk_reader.Update()
        vtk_data = vtk_reader.GetOutput()

        if vtk_data is None:
            raise ValueError(f"无法解析VTK文件：{vtk_file_path}")

        # 提取VTK几何信息（LPS）
        dims = vtk_data.GetDimensions()  # (x, y, z) 对应L/P/Z
        spacing = vtk_data.GetSpacing()  # (x_spacing, y_spacing, z_spacing)
        origin = vtk_data.GetOrigin()  # (x_origin, y_origin, z_origin)

        # 2. 提取标量数据并重塑（无transpose，保持VTK原始维度）
        scalar_array = vtk_data.GetPointData().GetScalars()
        if scalar_array is None:
            raise ValueError("VTK文件无Volume标量数据")

        n_points = scalar_array.GetNumberOfTuples()
        np_data = np.array([scalar_array.GetTuple1(i) for i in range(n_points)], dtype=np.float32)
        np_volume = np_data.reshape(dims, order='F')  # 无transpose，保持(x,y,z)

        # 3. 构建修正后的LPS Affine矩阵（修复L/P方向）
        affine = np.eye(4)
        # 核心修正：L(x)、P(y)轴方向取反（符号改为负），Z(z)保持不变
        affine[0, 0] = -spacing[0]  # L→R 方向修正（原方向反，取负）
        affine[1, 1] = -spacing[1]  # P→A 方向修正（原方向反，取负）
        affine[2, 2] = spacing[2]  # Z方向正确，保持原符号

        # 修正原点：方向反转后，原点需偏移对应轴的总长度（dims-1)*spacing
        affine[0, 3] = -origin[0] #+ (dims[0] - 1) * spacing[0]  # x轴原点偏移
        affine[1, 3] = -origin[1] #+ (dims[1] - 1) * spacing[1]  # y轴原点偏移
        affine[2, 3] = origin[2]  # z轴原点不变

        # 4. 保存nii.gz
        nii_image = nib.Nifti1Image(np_volume, affine=affine)
        nib.save(nii_image, output_nii_path)

        print(f"转换成功！文件保存至：\n{output_nii_path}")
        print(f"修正后坐标系：L(x)、P(y)方向匹配原始VTK，Z(z)方向保持正确")
        return output_nii_path

    except Exception as e:
        raise Exception(f"转换失败：{str(e)}")


# -------------------------- 调用示例 --------------------------
if __name__ == "__main__":
    input_vtk = r"C:\Users\wangz\PycharmProjects\PythonProject2\ultrasound_images_20251015_150024_inv\Prostate_US_3D_XYZ_to_LPS.vtk"
    try:
        output_nii = vtk_lps_to_nii_lps(input_vtk)
    except Exception as e:
        print(f"转换失败：{e}")