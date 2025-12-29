import os
import re
import numpy as np
import cv2
import vtk
from vtk.util.numpy_support import numpy_to_vtk
import matplotlib.pyplot as plt


# -------------------------- 核心配置 --------------------------
IMAGE_FOLDER = ""
PIXEL_SPACING = 63.8 / 406  # 像素间距 (mm/像素)
ORIGIN_OFFSET = 10       # 物理原点偏移
ROTATION_START = None    # 将在运行时更新为 -x
ROTATION_END = None      # 将在运行时更新为 x
NUM_IMAGES = None        # 将在运行时更新为 2x + 1
INTERPOLATION_STEPS = 3  # 插值步数
VTK_FILENAME = "Prostate_US_3D_XYZ_to_LPS.vtk"
AXIAL_PREVIEW_FILENAME = "US_3D_LPS_Axial_Preview.png"

# -------------------------- 新增：外部调用接口 --------------------------
def run_pipeline(image_folder, rotation_x):
    """
    供主程序调用的入口函数
    :param image_folder: 图片保存的文件夹路径
    :param rotation_x: 旋转角度 x (即界面输入的 rotation range)
    """
    global IMAGE_FOLDER, ROTATION_START, ROTATION_END, NUM_IMAGES
    
    # 1. 更新配置参数
    IMAGE_FOLDER = image_folder
    ROTATION_START = -float(rotation_x)
    ROTATION_END = float(rotation_x)
    NUM_IMAGES = int(2 * rotation_x + 1)
    
    print(f"\n[RecUS] 启动处理流程...")
    print(f"  - 图像路径: {IMAGE_FOLDER}")
    print(f"  - 旋转范围: {ROTATION_START}° ~ {ROTATION_END}°")
    print(f"  - 预期图像数: {NUM_IMAGES}")
    
    # 为了防止多线程下 matplotlib 报错，强制使用非交互式后端
    try:
        plt.switch_backend('Agg')
    except:
        pass

    # 2. 执行主流程
    try:
        main()
        print("[RecUS] 处理流程执行完毕。")
    except Exception as e:
        print(f"[RecUS] 执行出错: {e}")

# -------------------------- 1. 图像读取与排序 --------------------------
def read_ultrasound_images(folder):
    image_files = [f for f in os.listdir(folder)
                   if f.endswith('.png') and re.match(r'^\d+\(', f)]

    def extract_index(filename):
        match = re.match(r'^(\d+)', filename)
        return int(match.group(1)) if match else -1

    image_files.sort(key=extract_index)
    images = []
    for file in image_files[:NUM_IMAGES]:
        try:
            img = cv2.imread(os.path.join(folder, file), cv2.IMREAD_GRAYSCALE)
            if img is not None:
                images.append(img)
        except Exception as e:
            print(f"读取图像 {file} 失败：{str(e)}")

    if len(images) != NUM_IMAGES:
        raise ValueError(f"需{NUM_IMAGES}张图像，实际{len(images)}张")

    img_height, img_width = images[0].shape
    print(f"图像读取完成：{len(images)}张，尺寸：{img_height}×{img_width}像素")
    return images, img_height, img_width


# -------------------------- 2. 旋转角度分配 --------------------------
def assign_rotation_angles(num_images):
    angles = np.linspace(ROTATION_START, ROTATION_END, num_images, endpoint=True)
    print(f"角度分配：{angles[0]}° ~ {angles[-1]}°（共{len(angles)}个角度）")
    return angles


# -------------------------- 3. 图像插值 --------------------------
def interpolate_images(images, angles, steps=1):
    if steps < 1:
        return images, angles

    dense_images = []
    dense_angles = []
    num_original = len(images)
    num_intervals = num_original - 1
    expected_total = num_original + num_intervals * steps

    print(f"\n插值配置：原始{num_original}张 → 预期总{expected_total}张")

    for i in range(num_intervals):
        start_angle, end_angle = angles[i], angles[i + 1]
        start_img, end_img = images[i], images[i + 1]

        interp_angles = np.linspace(start_angle, end_angle, num=steps + 2)[1:-1]
        interp_coeffs = np.linspace(0, 1, num=steps + 2)[1:-1]

        for coeff, angle in zip(interp_coeffs, interp_angles):
            interp_img = (start_img * (1 - coeff) + end_img * coeff).astype(np.uint8)
            dense_images.append(interp_img)
            dense_angles.append(angle)

    dense_images.append(images[-1])
    dense_angles.append(angles[-1])

    print(f"✅ 插值完成：总{len(dense_images)}张")
    return dense_images, np.array(dense_angles)


# -------------------------- 4. 三维重建（修改Z轴方向，对应调整X轴，LPS方向矩阵不变） --------------------------
def reconstruct_3d_volume(images, angles, pixel_spacing, origin_offset, img_height, img_width):
    # Z轴与原Xp相反（原Z→S，现Z→-S方向）
    # X轴因Z轴反转同步调整（保持坐标系一致性）
    Y_min, Y_max = np.inf, -np.inf  # Y→P（不变）
    for Yp in range(img_height):
        y_distance = origin_offset + (img_height - 1 - Yp) * pixel_spacing
        Y_phys = -y_distance * np.cos(np.radians(0))
        Y_min, Y_max = min(Y_min, Y_phys), max(Y_max, Y_phys)

    X_min, X_max = np.inf, -np.inf  # X→L（因Z轴反转调整）
    for angle in [ROTATION_START, ROTATION_END]:
        theta = np.radians(angle)
        for Yp in range(img_height):
            y_distance = origin_offset + (img_height - 1 - Yp) * pixel_spacing
            X_phys = y_distance * np.sin(theta)  # X轴方向调整（与Z轴反转对应）
            X_min, X_max = min(X_min, X_phys), max(X_max, X_phys)

    # Z轴修改：与原Xp相反（物理坐标反转）
    Z_max = (img_width - 1) * pixel_spacing
    Z_min = 0 * pixel_spacing  # 原Z_min → 现Z_max

    # 边界计算（不变）
    pad_ratio = 0.1
    X_range = (X_min - (X_max - X_min) * pad_ratio, X_max + (X_max - X_min) * pad_ratio)
    Y_range = (Y_min - (Y_max - Y_min) * pad_ratio, Y_max + (Y_max - Y_min) * pad_ratio)
    Z_range = (Z_min - (Z_max - Z_min) * pad_ratio, Z_max + (Z_max - Z_min) * pad_ratio)

    # 体素参数（XYZ顺序不变）
    x_spacing = y_spacing = z_spacing = pixel_spacing
    X_size = int(np.ceil((X_range[1] - X_range[0]) / x_spacing)) + 1
    Y_size = int(np.ceil((Y_range[1] - Y_range[0]) / y_spacing)) + 1
    Z_size = int(np.ceil((Z_range[1] - Z_range[0]) / z_spacing)) + 1
    volume = np.zeros((X_size, Y_size, Z_size), dtype=np.uint8)

    print(f"\n修改后体积参数（XYZ）：X→L={X_size}，Y→P={Y_size}，Z→-S={Z_size}体素")
    print(f"X轴：最小值 = {X_min:.4f} mm，最大值 = {X_max:.4f} mm")
    print(f"Y轴：最小值 = {Y_min:.4f} mm，最大值 = {Y_max:.4f} mm")
    print(f"Z轴：最小值 = {Z_min:.4f} mm，最大值 = {Z_max:.4f} mm")
    print("----------------------------------------")
    return volume, (x_spacing, y_spacing, z_spacing), (X_range, Y_range, Z_range), (img_height, img_width, angles)


# -------------------------- 5. 填充体素（同步修改Z轴物理坐标） --------------------------
def fill_volume(volume, images, angles, pixel_spacing, origin_offset, ranges, img_dims):
    X_range, Y_range, Z_range = ranges
    img_height, img_width, _ = img_dims
    x_spacing = pixel_spacing

    for angle, img in zip(angles, images):
        theta = np.radians(angle)
        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)

        for Yp in range(img_height):
            y_distance = origin_offset + (img_height - 1 - Yp) * pixel_spacing
            X_phys = y_distance * sin_theta  # X轴调整（与Z轴反转对应）
            Y_phys = -y_distance * cos_theta  # Y轴不变

            X_voxel = int(round((X_phys - X_range[0]) / x_spacing))
            Y_voxel = int(round((Y_phys - Y_range[0]) / x_spacing))
            if not (0 <= X_voxel < volume.shape[0] and 0 <= Y_voxel < volume.shape[1]):
                continue

            for Xp in range(img_width):
                Z_phys = (img_width - Xp) * pixel_spacing  # Z轴与原Xp相反（核心修改）
                Z_voxel = int(round((Z_phys - Z_range[0]) / x_spacing))
                if 0 <= Z_voxel < volume.shape[2] and img[Yp, Xp] > volume[X_voxel, Y_voxel, Z_voxel]:
                    volume[X_voxel, Y_voxel, Z_voxel] = img[Yp, Xp]

    print(f"体积填充完成，空洞率：{np.mean(volume == 0) * 100:.2f}%")
    return volume


# -------------------------- 6. 转换为LPS坐标系并导出VTK（LPS方向矩阵保持单位矩阵不变） --------------------------
def export_lps_vtk(volume_xyz, spacing, ranges, output_folder):
    x_spacing, y_spacing, z_spacing = spacing
    X_size, Y_size, Z_size = volume_xyz.shape
    X_range, Y_range, Z_range = ranges

    # 维度顺序转换为LPS存储习惯：(Z→S, Y→P, X→L)
    volume_lps = np.transpose(volume_xyz, (2, 1, 0))

    vtk_img = vtk.vtkImageData()
    vtk_img.SetDimensions(X_size, Y_size, Z_size)
    vtk_img.SetOrigin(X_range[0], Y_range[0], Z_range[0])
    vtk_img.SetSpacing(x_spacing, y_spacing, z_spacing)

    # 关键：LPS方向矩阵保持单位矩阵不变（用户要求）
    direction = vtk.vtkMatrix3x3()
    direction.Identity()  # 单位矩阵：X→L(1,0,0), Y→P(0,1,0), Z→S(0,0,1)
    vtk_img.SetDirectionMatrix(direction)

    # 数据转换与写入（不变）
    volume_contiguous = np.ascontiguousarray(volume_lps, dtype=np.uint8)
    vtk_array = numpy_to_vtk(
        volume_contiguous.flatten(),
        deep=True,
        array_type=vtk.VTK_UNSIGNED_CHAR
    )
    vtk_img.GetPointData().SetScalars(vtk_array)

    os.makedirs(output_folder, exist_ok=True)
    output_path = os.path.join(output_folder, VTK_FILENAME)
    writer = vtk.vtkDataSetWriter()
    writer.SetFileName(output_path)
    writer.SetInputData(vtk_img)
    writer.Write()

    if os.path.exists(output_path):
        print(f"\nLPS坐标系VTK保存至：{output_path}（方向矩阵保持单位矩阵）")
        return output_path, volume_lps
    else:
        raise FileNotFoundError("VTK生成失败")


# -------------------------- 7. 生成LPS Axial预览图（更新Z轴说明） --------------------------
def generate_lps_axial_preview(volume_lps, output_folder):
    Z_size, Y_size, X_size = volume_lps.shape
    Z_center = min(Z_size // 2, Z_size - 1)
    axial_plane = volume_lps[Z_center, :, :]

    os.makedirs(output_folder, exist_ok=True)
    preview_path = os.path.join(output_folder, AXIAL_PREVIEW_FILENAME)

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.imshow(axial_plane, cmap='gray')
    ax.set_title(f"LPS_Axial(Y→P × X→L, Z→S={Z_center})")  # 说明Z轴新方向
    ax.axis('off')

    plt.tight_layout()
    plt.savefig(preview_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"LPS Axial预览图保存至：{preview_path}")
# -------------------------- 主函数 --------------------------
def main():
    try:
        # 1. 读取与插值
        images, img_height, img_width = read_ultrasound_images(IMAGE_FOLDER)
        angles = assign_rotation_angles(len(images))
        interpolated_images, interpolated_angles = interpolate_images(images, angles, INTERPOLATION_STEPS)

        # 2. 按原始XYZ重建体积
        print("\n开始原始XYZ坐标系重建...")
        volume_xyz, spacing, ranges, img_dims = reconstruct_3d_volume(
            interpolated_images, interpolated_angles, PIXEL_SPACING, ORIGIN_OFFSET, img_height, img_width
        )

        # 3. 填充体素
        volume_xyz = fill_volume(volume_xyz, interpolated_images, interpolated_angles,
                                 PIXEL_SPACING, ORIGIN_OFFSET, ranges, img_dims)

        # 4. 转换为LPS并导出VTK
        print("\n转换为LPS坐标系并导出VTK...")
        vtk_path, volume_lps = export_lps_vtk(volume_xyz, spacing, ranges, IMAGE_FOLDER)

        # 5. 生成LPS预览图
        generate_lps_axial_preview(volume_lps, IMAGE_FOLDER)

        print("\n✅ 完成！体积先按原始XYZ重建，再转换为LPS坐标系导出，适配ITK-SNAP")


    except Exception as e:
        print(f"\n❌ 执行失败：{str(e)}")


if __name__ == "__main__":
    main()