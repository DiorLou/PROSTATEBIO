# ui/ultrasound_tab.py
import cv2
import os
import numpy as np
import time
from datetime import datetime
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QMessageBox, QSlider, QFileDialog, QLineEdit, QGridLayout
from PyQt5.QtCore import Qt, QTimer 
from PyQt5.QtGui import QImage, QPixmap # 已修正：QImage 和 QPixmap 应该从 QtGui 导入
from PyQt5.QtCore import pyqtSignal
import pytransform3d.rotations as pyrot
import threading  # [新增] 引入线程模块
import shutil
try:
    from ultrasound_image_process import RecUS
    from ultrasound_image_process import vtk2nii # 确保导入转换脚本
except ImportError:
    RecUS = None
    print("Warning: Could not import RecUS module. 3D reconstruction will be disabled.")

# Constants for motion direction (来自 main_window.py)
FORWARD = 1
BACKWARD = 0

class UltrasoundTab(QWidget):
    # 信号增加一个 bool 参数，True 表示是局部重建/需要后续动作
    scan_finished = pyqtSignal(bool)
    # 默认裁剪常量
    DEFAULT_LEFT_CROP   = 158
    DEFAULT_RIGHT_CROP  = 564
    DEFAULT_TOP_CROP    = 131
    DEFAULT_BOTTOM_CROP = 551

    # TCP_U -> default cropped ultrasound image calibration.
    # Image u follows -TCP_U z; image v follows +TCP_U y for the current
    # cropped ultrasound captures.
    MM_PER_PIXEL = 66.0 / 420.0
    TCP_U_ORIGIN_U_PX = 406.0
    TCP_U_ORIGIN_V_PX = 420.0 + 10.0 / MM_PER_PIXEL
    KINEMATIC_BOX_HALF_WIDTH_PX = 10.0
    
    def __init__(self, tcp_manager, parent=None):
        super().__init__(parent)
        self.tcp_manager = tcp_manager
        # 🌟 修复点 1: 显式存储主窗口实例
        self.main_window = parent 
        self.camera = None
        self.original_frame = None # 存储原始帧
        self.current_frame = None  # 用于存储裁剪后的帧
        self.image_timer = QTimer(self)
        self.image_timer.timeout.connect(self.update_frame)
        
        self.image_label = QLabel("Waiting for Ultrasound Image...")
        self.start_btn = QPushButton("Start Ultrasound Probe")
        self.stop_btn = QPushButton("Stop Ultrasound Probe")
        
        # --- [新增] 单次保存按钮 ---
        self.single_save_btn = QPushButton("Save Single Image")
        
        # --- [修改] 1. 按钮名称改为“开始保存图像” ---
        self.save_btn = QPushButton("Start Image Saving")
        
        # --- [新增] 实时保存相关变量和定时器 ---
        self.is_real_time_saving = False
        self.real_time_save_timer = QTimer(self)
        # 2. 定时器连接到新的保存函数
        self.real_time_save_timer.timeout.connect(self._save_real_time_frame) 
        self.real_time_save_folder = ""
        self.save_sequence_number = 0
        
        # --- 原始左右裁剪滑块 ---
        self.left_slider = QSlider(Qt.Horizontal)
        self.right_slider = QSlider(Qt.Horizontal)
        self.left_label = QLabel(f"Left Crop: {self.DEFAULT_LEFT_CROP}")
        self.right_label = QLabel(f"Right Crop: {self.DEFAULT_RIGHT_CROP}")
        
        # --- 新增上下裁剪滑块 ---
        self.top_slider = QSlider(Qt.Horizontal)
        self.bottom_slider = QSlider(Qt.Horizontal)
        self.top_label = QLabel(f"Top Crop: {self.DEFAULT_TOP_CROP}")
        self.bottom_label = QLabel(f"Bottom Crop: {self.DEFAULT_BOTTOM_CROP}")

        # 新增: 机器人旋转和拍照相关变量
        self.tcp_manager = tcp_manager
        self.is_rotating = False
        self.current_rotation_step = 0
        self.total_rotation_steps = 0 # 新增：总旋转步数
        self.save_folder = "" # 旋转保存的文件夹
        self.session_folder = None  # [新增] 用于存储 "时间戳_experimental results" 根目录

        # 新增: 旋转范围输入框和按钮
        self.rotation_range_input = QLineEdit("50") # 默认值 50
        self.left_x_btn = QPushButton("Ultrasound Probe Rotate Left x Deg")
        self.right_2x_btn = QPushButton("Ultrasound Probe Rotate Right 2x Deg")

        # [新增] 穿刺针旋转范围输入框和按钮
        self.needle_yaw_range_input = QLineEdit("10") # 默认值 10
        self.needle_left_x_btn = QPushButton("Needle Rotate Left x Deg")
        self.needle_right_2x_btn = QPushButton("Needle Rotate Right 2x Deg")
        self.init_ui()
        self.setup_connections()

    def _send_next_rotation_command(self):
        """发送下一个 1 度旋转命令，由 QTimer 延迟调用。"""
        # Direction FORWARD (1) is used for the continuous right turn
        command = f"MoveRelJ,0,5,{FORWARD},1;"
        self.tcp_manager.send_command(command)

    def init_ui(self):
        """构建超声图像标签页的UI。"""
        layout = QVBoxLayout(self)
        
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setStyleSheet("border: 2px solid grey;")
        # 将图像显示窗口固定为 360x640，防止窗口无限变大
        self.image_label.setFixedSize(360, 640) 

        image_layout = QHBoxLayout()
        image_layout.addStretch()
        image_layout.addWidget(self.image_label)
        image_layout.addStretch()

        layout.addLayout(image_layout, 1)

        crop_group = QWidget()
        crop_layout = QVBoxLayout(crop_group)
        
        # --- 水平裁剪 (左右) ---
        left_crop_layout = QHBoxLayout()
        # 滑块范围将在启动捕获后动态设置
        self.left_slider.setRange(0, 720)
        self.left_slider.setValue(self.DEFAULT_LEFT_CROP)
        left_crop_layout.addWidget(self.left_label)
        left_crop_layout.addWidget(self.left_slider)
        
        right_crop_layout = QHBoxLayout()
        self.right_slider.setRange(0, 720)
        self.right_slider.setValue(self.DEFAULT_RIGHT_CROP)
        right_crop_layout.addWidget(self.right_label)
        right_crop_layout.addWidget(self.right_slider)

        # --- 垂直裁剪 (上下) ---
        top_crop_layout = QHBoxLayout()
        self.top_slider.setRange(0, 1280)
        self.top_slider.setValue(self.DEFAULT_TOP_CROP)
        top_crop_layout.addWidget(self.top_label)
        top_crop_layout.addWidget(self.top_slider)

        bottom_crop_layout = QHBoxLayout()
        self.bottom_slider.setRange(0, 1280)
        self.bottom_slider.setValue(self.DEFAULT_BOTTOM_CROP)
        bottom_crop_layout.addWidget(self.bottom_label)
        bottom_crop_layout.addWidget(self.bottom_slider)

        crop_layout.addLayout(left_crop_layout)
        crop_layout.addLayout(right_crop_layout)
        crop_layout.addLayout(top_crop_layout)
        crop_layout.addLayout(bottom_crop_layout)

        layout.addWidget(crop_group)

        # --- 按钮和旋转控制布局 ---
        btn_layout = QHBoxLayout()
        
        # 设置所有旋转相关按钮的大小一致
        btn_width, btn_height = 250, 40
        self.left_x_btn.setFixedSize(btn_width, btn_height)
        self.right_2x_btn.setFixedSize(btn_width, btn_height)
        self.needle_left_x_btn.setFixedSize(btn_width, btn_height)
        self.needle_right_2x_btn.setFixedSize(btn_width, btn_height)

        # 创建网格布局用于精确对齐
        grid_ctrl_layout = QGridLayout()

        # 第一列：标签和输入框
        grid_ctrl_layout.addWidget(QLabel("Probe Range x:"), 0, 0)
        grid_ctrl_layout.addWidget(self.rotation_range_input, 0, 1)
        grid_ctrl_layout.addWidget(QLabel("Needle Range x:"), 1, 0)
        grid_ctrl_layout.addWidget(self.needle_yaw_range_input, 1, 1)

        # 第二列：左转按钮 (Probe 在上，Needle 在下)
        grid_ctrl_layout.addWidget(self.left_x_btn, 0, 2)
        grid_ctrl_layout.addWidget(self.needle_left_x_btn, 1, 2)

        # 第三列：右转按钮 (Probe 在上，Needle 在下)
        grid_ctrl_layout.addWidget(self.right_2x_btn, 0, 3)
        grid_ctrl_layout.addWidget(self.needle_right_2x_btn, 1, 3)

        # 将网格布局加入主按钮行
        btn_layout.addStretch()
        # (保留之前的 start/stop/save 等按钮)
        btn_layout.addWidget(self.start_btn)
        btn_layout.addWidget(self.stop_btn)
        btn_layout.addWidget(self.single_save_btn)
        btn_layout.addWidget(self.save_btn)
        btn_layout.addSpacing(20)
        btn_layout.addLayout(grid_ctrl_layout) # 插入新设计的对齐网格
        btn_layout.addStretch()
        
        layout.addLayout(btn_layout)

    def setup_connections(self):
        """连接信号和槽。"""
        self.start_btn.clicked.connect(self.start_capture)
        self.stop_btn.clicked.connect(self.stop_capture)
        # --- [新增] 单次保存按钮连接 ---
        self.single_save_btn.clicked.connect(self.save_image) 
        # --- [修改] 连接到新的 toggle 函数 ---
        self.save_btn.clicked.connect(self.toggle_real_time_save) 
        
        # 连接水平裁剪滑块
        self.left_slider.valueChanged.connect(self.update_crop_value)
        self.right_slider.valueChanged.connect(self.update_crop_value)
        
        # 连接垂直裁剪滑块
        self.top_slider.valueChanged.connect(self.update_crop_value)
        self.bottom_slider.valueChanged.connect(self.update_crop_value)
        
        # 机器人旋转按钮的连接 (使用新的方法)
        self.left_x_btn.clicked.connect(self.rotate_left_x)
        self.right_2x_btn.clicked.connect(self.rotate_and_capture_2x)

        # [新增] 穿刺针旋转按钮连接
        self.needle_left_x_btn.clicked.connect(self.rotate_needle_left_x)
        self.needle_right_2x_btn.clicked.connect(self.rotate_needle_right_2x)
        
        # 监听 TCP 消息，用于处理旋转反馈
        self.tcp_manager.message_received.connect(self.handle_incoming_message)
        
    def handle_incoming_message(self, message):
        """处理来自机器人的消息，用于驱动连续旋转逻辑。"""
        # 检查是否正在执行旋转扫描任务
        if not self.is_rotating:
            return

        # 筛选 MoveRelJ 指令的反馈
        if "MoveRelJ" in message:
            # 尝试获取主窗口右侧面板的日志函数，以便在界面显示日志
            log_func = print
            if self.main_window and hasattr(self.main_window, 'right_panel'):
                log_func = self.main_window.right_panel.log_message
            
            # 分情况处理并记录日志
            if "OK" in message:
                log_func(f"System: MoveRelJ OK received. Continuing... [{message.strip()}]")
                self.continue_rotation()
            else:
                # 如果没有 OK，可能是错误消息或异常状态，记录警告
                log_func(f"Warning: MoveRelJ received without OK! [{message.strip()}]")

    def update_crop_value(self, value):
        """更新裁剪滑块的标签文本，并确保上下左右边界的逻辑正确性。"""
        left_val = self.left_slider.value()
        right_val = self.right_slider.value()
        top_val = self.top_slider.value()
        bottom_val = self.bottom_slider.value()

        # 1. 检查水平边界 (Left < Right)
        if left_val > right_val:
            self.right_slider.setValue(left_val)
            right_val = left_val
        
        # 2. 检查垂直边界 (Top < Bottom)
        if top_val > bottom_val:
            self.bottom_slider.setValue(top_val)
            bottom_val = top_val

        # 3. 更新标签
        self.left_label.setText(f"Left Crop: {left_val}")
        self.right_label.setText(f"Right Crop: {right_val}")
        self.top_label.setText(f"Top Crop: {top_val}")
        self.bottom_label.setText(f"Bottom Crop: {bottom_val}")

    def start_capture(self):
        """开始捕获超声图像流。"""
        # 请在这里将 0 替换为您在第1步中找到的正确索引号
        self.camera = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        
        if not self.camera.isOpened():
            QMessageBox.critical(self, "Error", "Cannot open camera. Please check device connection.")
            self.stop_btn.setEnabled(False)
            self.start_btn.setEnabled(True)
            self.save_btn.setEnabled(False)
            self.single_save_btn.setEnabled(False) # [修改] 启用/禁用单次保存按钮
            # self.left_x_btn.setEnabled(False)
            self.right_2x_btn.setEnabled(False)
            self.image_label.setText("Cannot open camera.")
            self.camera = None
            return

        # 主动请求高分辨率
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        ret, frame = self.camera.read()
        if not ret:
            self.stop_capture()
            QMessageBox.critical(self, "Error", "Cannot read frame from camera.")
            return
            
        # 原始是 (720, 1280), 旋转 90 度后变成 (1280, 720)
        actual_height, actual_width = 1280, 720

        # --- 设置水平滑块范围 (宽度) ---
        self.left_slider.setRange(0, actual_width)
        self.right_slider.setRange(0, actual_width)
         
        # --- 设置垂直滑块范围 (高度) ---
        self.top_slider.setRange(0, actual_height)
        self.bottom_slider.setRange(0, actual_height)
 
        # 强制调用一次 update_crop_value 以更新标签和边界检查
        self.update_crop_value(0)

        self.image_timer.start(30)
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.save_btn.setEnabled(True)
        self.single_save_btn.setEnabled(True) # [修改] 启用/禁用单次保存按钮
        self.left_x_btn.setEnabled(True)
        self.right_2x_btn.setEnabled(True)
        self.image_label.setText("Capturing Image...")

    def stop_capture(self):
        """停止图像捕获。"""
        self.image_timer.stop()
        
        # [新增] 停止捕获时，如果正在实时保存，则停止
        if self.is_real_time_saving:
            self._stop_real_time_save()

        if self.camera:
            self.camera.release()
            self.camera = None
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.save_btn.setEnabled(False)
        self.single_save_btn.setEnabled(False) # [修改] 启用/禁用单次保存按钮
        self.left_x_btn.setEnabled(False)
        self.right_2x_btn.setEnabled(False)
        self.image_label.setText("Capture Stopped.")
    
    def update_frame(self):
        """从摄像头读取帧，旋转，裁剪并显示。"""
        ret, frame = self.camera.read()
        if not ret:
            self.stop_capture()
            return
        
        # 🌟 修改点 4: 将图像逆时针旋转 90 度
        # 旋转后尺寸从 1280x720 变为 720x1280
        self.original_frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        
        # 获取四向裁剪值
        left_crop = self.left_slider.value()
        right_crop = self.right_slider.value()
        top_crop = self.top_slider.value()
        bottom_crop = self.bottom_slider.value()
        
        # 裁剪旋转后的图像
        self.current_frame = self.original_frame[top_crop:bottom_crop, left_crop:right_crop]
        
        # 转换为 PyQt 格式
        rgb_image = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        
        pixmap = QPixmap.fromImage(qt_image).scaled(
            self.image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        self.image_label.setPixmap(pixmap)

    # --- [修改/新增] 实时保存逻辑 START ---
    
    def toggle_real_time_save(self):
        """切换实时图像保存状态。"""
        if not self.is_real_time_saving:
            self._start_real_time_save()
        else:
            self._stop_real_time_save()

    def _start_real_time_save(self):
        """开始实时图像保存：创建文件夹，设置序列号，启动定时器。"""
        if self.current_frame is None:
            QMessageBox.warning(self, "Warning", "No image available for saving. Please start ultrasound probe first.")
            return

        # 实时保存间隔 (ms)
        interval_ms = 300

        # 1. 构造新的文件夹名称
        # 名称格式: "实时保存图像_间隔300ms_20251107_HHMMSS"
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 使用下划线作为分隔符，避免系统兼容性问题
        new_folder_name = f"Realtime_Capture_Interval_{interval_ms}ms_{timestamp}"
        
        # 2. 创建根目录和子目录
        base_dir = os.path.join(os.getcwd(), "image") 
        if not os.path.isdir(base_dir):
            try:
                os.makedirs(base_dir, exist_ok=True)
            except OSError as e:
                QMessageBox.critical(self, "File System Error", f"Cannot create root save directory ('image'): {e}")
                return

        # 设置最终的保存文件夹路径
        self.real_time_save_folder = os.path.join(base_dir, new_folder_name)
        try:
            os.makedirs(self.real_time_save_folder, exist_ok=True)
        except OSError as e:
            QMessageBox.critical(self, "File System Error", f"Cannot create save directory: {e}")
            return
            
        self.save_sequence_number = 0
        self.is_real_time_saving = True
        
        # 3. 按钮名字变为“停止保存图像”，并改变颜色
        self.save_btn.setText("Stop Image Saving") 
        self.save_btn.setStyleSheet("background-color: salmon;") 
        
        # 4. 启动定时器 (默认 300ms)
        self.real_time_save_timer.start(interval_ms)
        self.main_window.status_bar.showMessage(f"Status: Started real-time image saving to: {self.real_time_save_folder}")

    def _stop_real_time_save(self):
        """停止实时图像保存：停止定时器，恢复按钮状态。"""
        self.real_time_save_timer.stop()
        self.is_real_time_saving = False
        
        # 2. 按钮变回“开始保存图像”
        self.save_btn.setText("Start Image Saving") 
        self.save_btn.setStyleSheet("")
        self.main_window.status_bar.showMessage("Status: Stopped real-time image saving.")

    def _save_real_time_frame(self):
        """定时器超时时调用，保存当前帧。"""
        if self.current_frame is None:
            print("Warning: No image frame available during real-time saving.")
            return

        robot_control_window = self.main_window
        raw_pose = getattr(robot_control_window, 'latest_tool_pose', None) if robot_control_window else None
        if raw_pose is None or len(raw_pose) != 6:
            print("Warning: Cannot get a valid TCP_E pose; frame not saved.")
            return
        else:
            tcp_name = getattr(robot_control_window, 'current_tcp_name', 'UNKNOWN')
            
            # 初始化目标 TCP_E 位姿（作为默认降级保障）
            pose_e = list(raw_pose) 
            
            try:
                # 获取 left_panel 实例利用辅助矩阵函数
                lp = getattr(robot_control_window, 'left_panel', None)
                
                if lp:
                    import pytransform3d.rotations as pyrot
                    
                    if tcp_name == "TCP_E":
                        # =========================================================
                        # 情况 A: 当前本身就是 TCP_E，反馈数据已是法兰中心位姿
                        # =========================================================
                        pose_e = list(raw_pose)
                        
                    elif tcp_name == "TCP_O":
                        # =========================================================
                        # 情况 B: 当前为 TCP_O，通过已确定的 T_E_O 反推绝对系下的 TCP_E 位姿
                        # 公式: T_Base_E = T_Base_Current(即T_Base_O) @ inv(T_E_O)
                        # =========================================================
                        T_Base_Current = lp.pose_to_matrix(raw_pose)
                        
                        # 提取系统内部已确定并生效的确定 z_offset
                        z_offset_calibrated = float(robot_control_window.right_panel.tcp_input_entries[2].text()) if robot_control_window.right_panel else 0.0
                        tcp_o_def_pose = [0.0, 0.0, z_offset_calibrated, 0.0, 0.0, 157.50]
                        
                        T_E_O = lp.pose_to_matrix(tcp_o_def_pose)
                        T_O_E = np.linalg.inv(T_E_O)
                        
                        # 矩阵相乘，得到基座系下的 TCP_E 矩阵
                        T_Base_E = T_Base_Current @ T_O_E
                        
                        # 从矩阵中重新解算绝对位置 [x, y, z] 和 绝对姿态 [Rx, Ry, Rz]
                        pos_e = T_Base_E[:3, 3]
                        rpy_e = np.rad2deg(pyrot.euler_from_matrix(T_Base_E[:3, :3], 0, 1, 2, extrinsic=True))
                        
                        # 拼接最终输出的统一 TCP_E 位姿
                        pose_e = [pos_e[0], pos_e[1], pos_e[2], rpy_e[0], rpy_e[1], rpy_e[2]]
                        
                    else:
                        print(f"Warning: Current TCP is {tcp_name}, expected TCP_E or TCP_O. No conversion applied.")
                else:
                    print("Warning: left_panel instance missing.")

            except Exception as e:
                print(f"Error transferring pose from {tcp_name} to TCP_E: {e}")
                # 异常发生时安全降级使用原始反馈，防止定时器高频保存崩溃
                pose_e = list(raw_pose) 
        
        # 格式化转换后的工具端位姿: (x,y,z,Rx,Ry,Rz)
        if pose_e and len(pose_e) == 6:
            pose_str = "(" + ",".join([f"{p:.2f}" for p in pose_e]) + ")"
        else:
            print("Warning: Cannot calculate a valid TCP_E pose; frame not saved.")
            return

        needle_kinematics = self._get_current_needle_kinematics_in_tcp_u()
        if needle_kinematics is None:
            print("Warning: Current J0-J3 or TCP definitions are not ready; saving with empty TipU/VecU.")
            tip_u_str = ""
            vector_u_str = ""
            box_str = ""
        else:
            tip_u, vector_u = needle_kinematics
            tip_u_str = "(" + ",".join([f"{p:.2f}" for p in tip_u]) + ")"
            vector_u_str = "(" + ",".join([f"{v:.6f}" for v in vector_u]) + ")"
            box_corners = self._get_kinematic_box_pixels(tip_u, vector_u)
            if box_corners is None:
                box_str = ""
            else:
                box_str = "(" + ",".join(
                    [f"{coord:.2f}" for corner in box_corners for coord in corner]
                ) + ")"

        # 文件名: 四位序号 + TCP_E位姿 + TCP_U针尖/向量 + 运动学预测框
        sequence_str = f"{self.save_sequence_number:04d}"
        new_filename = (
            f"{sequence_str}_E{pose_str}_TipU{tip_u_str}_VecU{vector_u_str}_Box{box_str}.png"
        )
        image_path = os.path.join(self.real_time_save_folder, new_filename)

        if cv2.imwrite(image_path, self.current_frame):
            self.save_sequence_number += 1
        else:
            print(f"Real-time image saving failed: {image_path}")
            self._stop_real_time_save()

    def _get_current_needle_kinematics_in_tcp_u(self):
        """Return the current needle tip and unit direction vector in TCP_U."""
        try:
            needle_tab = self.main_window.flexible_needle_tab
            left_panel = self.main_window.left_panel

            if left_panel.tcp_p_definition_pose is None or left_panel.tcp_u_definition_pose is None:
                return None

            current_values = []
            for axis in ("J0", "J1", "J2", "J3"):
                value_text = needle_tab.pos_labels[f"Cur{axis}"].text().strip()
                if not value_text or value_text == "--":
                    return None
                current_values.append(float(value_text))

            current_j0, current_j1, current_j2, current_j3 = current_values
            delta_j0 = current_j0 - needle_tab.manager.RESET_J0
            delta_j1 = current_j1 - needle_tab.manager.RESET_J1
            delta_j2 = current_j2 - needle_tab.manager.RESET_J2
            delta_j3 = current_j3 - needle_tab.manager.RESET_J3

            # J2 电机值为累计量，运动学模型需要相对量。
            joint_values = [delta_j0, delta_j1, delta_j2 - delta_j1, delta_j3]
            tip_p = needle_tab.robot.get_tip_of_needle(joint_values.copy())
            vector_p = needle_tab.robot.get_needle_vector(joint_values.copy())

            T_e_u = left_panel.pose_to_matrix(left_panel.tcp_u_definition_pose)
            T_e_p = left_panel.pose_to_matrix(left_panel.tcp_p_definition_pose)
            T_u_p = np.linalg.inv(T_e_u) @ T_e_p

            tip_u = (T_u_p @ np.append(tip_p, 1.0))[:3]
            vector_u = T_u_p[:3, :3] @ vector_p
            vector_norm = np.linalg.norm(vector_u)
            if vector_norm < 1e-9:
                return None
            vector_u = vector_u / vector_norm
            return tip_u, vector_u
        except (AttributeError, KeyError, TypeError, ValueError, np.linalg.LinAlgError) as e:
            print(f"Needle kinematics calculation error: {e}")
            return None

    def _get_kinematic_box_pixels(self, tip_u, vector_u):
        """Project the kinematic needle ray to the cropped image and return four box corners."""
        if self.current_frame is None:
            return None

        image_height, image_width = self.current_frame.shape[:2]
        if image_width <= 0 or image_height <= 0:
            return None

        origin_u = self.TCP_U_ORIGIN_U_PX
        origin_v = self.TCP_U_ORIGIN_V_PX

        tip_px = np.array([
            origin_u - float(tip_u[2]) / self.MM_PER_PIXEL,
            origin_v + float(tip_u[1]) / self.MM_PER_PIXEL,
        ], dtype=float)

        # The prediction box is meaningful only when the projected needle tip
        # itself is visible in the saved ultrasound image.  Do not let
        # clipLine turn an out-of-frame tip into an artificial boundary point.
        if not (
            0.0 <= tip_px[0] <= image_width - 1
            and 0.0 <= tip_px[1] <= image_height - 1
        ):
            return None

        direction_px = np.array([
            -float(vector_u[2]) / self.MM_PER_PIXEL,
            float(vector_u[1]) / self.MM_PER_PIXEL,
        ], dtype=float)
        direction_norm = np.linalg.norm(direction_px)
        if direction_norm < 1e-9:
            return None
        direction_px /= direction_norm

        # VecU points from the shaft towards the tip, so trace from the tip in
        # the opposite direction until the shaft centreline reaches the image
        # boundary.  The tip supplies two corners and that boundary point
        # supplies the other two corners.
        ray_length = 4.0 * np.hypot(image_width, image_height)
        shaft_px = tip_px - direction_px * ray_length
        clip_rect = (0, 0, image_width, image_height)
        visible, clipped_shaft, clipped_tip = cv2.clipLine(
            clip_rect,
            tuple(np.rint(shaft_px).astype(int)),
            tuple(np.rint(tip_px).astype(int)),
        )
        if not visible:
            return None

        start = np.asarray(clipped_shaft, dtype=float)
        # tip_px has already been verified to be inside the image. Keep its
        # floating-point projection so the two tip-side corners are determined
        # by the actual projected tip rather than clipLine's rounded endpoint.
        end = tip_px
        segment = end - start
        segment_norm = np.linalg.norm(segment)
        if segment_norm < 1e-9:
            return None

        segment /= segment_norm
        normal = np.array([-segment[1], segment[0]]) * self.KINEMATIC_BOX_HALF_WIDTH_PX
        corners = np.array([
            start + normal,
            end + normal,
            end - normal,
            start - normal,
        ])
        corners[:, 0] = np.clip(corners[:, 0], 0, image_width - 1)
        corners[:, 1] = np.clip(corners[:, 1], 0, image_height - 1)
        return corners.tolist()


    def save_image(self):
        """保存当前裁剪后的图像。（单次保存）"""
        if self.current_frame is None:
            QMessageBox.warning(self, "Warning", "No image available for saving. Please start ultrasound probe first.")
            return

        file_dialog = QFileDialog(self)
        file_dialog.setWindowTitle("Save Image")
        file_dialog.setNameFilter("Image Files (*.png *.jpg *.jpeg)")
        file_dialog.setAcceptMode(QFileDialog.AcceptSave)
        
        # 默认保存到当前目录
        file_dialog.setDirectory(os.getcwd()) 

        if file_dialog.exec_() == QFileDialog.Accepted:
            save_path = file_dialog.selectedFiles()[0]
            # 确保文件名后缀
            if not save_path.lower().endswith(('.png', '.jpg', '.jpeg')):
                save_path += '.png' 
                
            try:
                cv2.imwrite(save_path, self.current_frame)
                QMessageBox.information(self, "Success", f"Image saved to:\n{save_path}")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Error saving image: {e}")

    def cleanup(self):
        """在窗口关闭时进行清理。"""
        self.stop_capture()
        # [新增] 确保实时保存定时器停止
        self.real_time_save_timer.stop() 
        
    def _reset_rotation_buttons(self):
        """重新启用旋转相关的按钮。"""
        self.right_2x_btn.setEnabled(True)
        self.left_x_btn.setEnabled(True)
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(True)
        self.single_save_btn.setEnabled(True) # [修改] 启用/禁用单次保存按钮

    def _get_rotation_x_value(self):
        """
        从输入框获取旋转范围 x 的值。
        [修改] 仅支持正整数。
        """
        try:
            x_text = self.rotation_range_input.text()
            x = int(x_text) 
            
            # [修改] 限制 x 必须大于 0
            if x <= 0:
                QMessageBox.warning(self, "Input Error", "Rotation range must be a positive integer.")
                return None
            return x
        except ValueError:
            QMessageBox.critical(self, "Input Error", "Rotation range x must be a valid integer!")
            return None

    def _save_frame_at_step(self, step_degree):
        """根据当前机器臂姿态保存图像，并以当前旋转度数命名。"""
        robot_control_window = self.main_window
        if not robot_control_window:
            print("Error: Cannot get main window instance.")
            return False

        # 确保 latest_tool_pose 在主窗口中存在
        if not hasattr(robot_control_window, 'latest_tool_pose'):
             pose = [0.0] * 6
        else:
             pose = robot_control_window.latest_tool_pose
        
        # 格式化工具端位姿: (x,y,z,Rx,Ry,Rz)
        if pose and len(pose) == 6:
            pose_str = f"({pose[0]:.2f},{pose[1]:.2f},{pose[2]:.2f},{pose[3]:.2f},{pose[4]:.2f},{pose[5]:.2f})"
        else:
            pose_str = "POSE_NA"
            print("Warning: Cannot get valid tool pose data.")

        # 构造新的文件名: (旋转度数) + (机器臂末端位姿) + .png
        rotation_step_str = f"{step_degree:03d}"
        new_filename = f"{rotation_step_str}{pose_str}.png"
        path1 = os.path.join(self.save_folder, new_filename)
        path2 = os.path.join(self.project_save_folder, new_filename)

        if self.current_frame is not None:
            cv2.imwrite(path1, self.current_frame)
            cv2.imwrite(path2, self.current_frame)
            return True
        return False

    def rotate_left_x(self):
        """
        [修改版] 发送指令，使超声探头左转 x 度。
        并计算所有 A 点在 Volume 下的坐标，保存到 TXT。
        """
        x = self._get_rotation_x_value()
        if x is None:
            return
            
        # [修改] 移除负数处理逻辑，固定为左转 (BACKWARD)
        move_direction = BACKWARD
        move_angle = x
        
        # TCP_E 检查
        robot_control_window = self.main_window
        if not robot_control_window or not hasattr(robot_control_window, 'latest_tool_pose'):
            QMessageBox.warning(self, "Connection Error", "Cannot access robot pose data...")
            return

        if robot_control_window.current_tcp_name != "TCP_E":
            QMessageBox.critical(self, "TCP Error", f"Current TCP must be 'TCP_E'...")
            return
            
        # 1. 记录 tcp_e_in_ultrasound_zero_deg 并触发 volume_in_base 计算
        robot_control_window.tcp_e_in_ultrasound_zero_deg = list(robot_control_window.latest_tool_pose)
        pose_str = ", ".join([f"{p:.2f}" for p in robot_control_window.tcp_e_in_ultrasound_zero_deg])
        robot_control_window.status_bar.showMessage(f"Status: tcp_e_in_ultrasound_zero_deg recorded: [{pose_str}]")
        
        # 立即计算 volume_in_base
        robot_control_window.compute_and_store_volume_in_base()
            
        # 2. 获取所有 A 点(Volume系) 并保存到 TXT
        all_a_in_vol = robot_control_window.left_panel.calculate_all_a_points_in_volume()
        
        if all_a_in_vol:
            try:
                file_path = "A_points_in_volume.txt"
                with open(file_path, "w") as f:
                    for pt in all_a_in_vol:
                        # 格式: x, y, z (无括号，无编号，换行)
                        line = f"{pt[0]:.3f} {pt[1]:.3f} {pt[2]:.3f}\n"
                        f.write(line)
                print(f"Successfully saved {len(all_a_in_vol)} points to {file_path}")
                robot_control_window.status_bar.showMessage(f"Status: Saved {len(all_a_in_vol)} A points to TXT.")
            except Exception as e:
                print(f"Error saving A points to TXT: {e}")
                QMessageBox.warning(self, "File Error", f"Failed to save A_points_in_volume.txt: {e}")
        else:
            print("Warning: No A points calculated (History empty or Calc failed).")

        # 3. 发送旋转指令
        command = f"MoveRelJ,0,5,{move_direction},{move_angle};"
        
        if self.tcp_manager and self.tcp_manager.is_connected:
            self.tcp_manager.send_command(command)
            QMessageBox.information(self, "Command Sent", f"Sent rotate LEFT {move_angle} degrees command.\nA points saved to TXT.")
        else:
            QMessageBox.warning(self, "Connection Error", "Not connected to robot or TCP manager.")

    def rotate_and_capture_2x(self, custom_folder_name=None):
        """开始旋转采样，支持外部传入自定义文件夹名"""
        
        # 禁用按钮防止重复点击
        self.right_2x_btn.setEnabled(False)
        self.left_x_btn.setEnabled(False)
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(False)
        self.single_save_btn.setEnabled(False) # [修改] 启用/禁用单次保存按钮
        
        x = self._get_rotation_x_value()
        if x is None: return
        
        total_rotation = 2 * x
        desktop_path = r"C:\Users\hkclr_user\Desktop"

        # --- [修改逻辑开始] ---
        # 1. 如果是第一次点击（主文件夹尚未建立），建立实验主文件夹
        if self.session_folder is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            main_folder_name = f"{timestamp}_experimental results"
            self.session_folder = os.path.join(desktop_path, main_folder_name)
            os.makedirs(self.session_folder, exist_ok=True)

        # 2. 确定子文件夹名称
        if custom_folder_name:
            # 局部重建：使用传入的 generate_local_us_folder_name() 的结果
            sub_folder_name = custom_folder_name
            self.is_global_reconstruction = False
        else:
            # 全局重建：固定为 GlobalUS
            sub_folder_name = "GlobalUS"
            self.is_global_reconstruction = True

        # 3. 设置本次扫描的完整路径（在实验主文件夹内）
        self.save_folder = os.path.join(self.session_folder, sub_folder_name)
        os.makedirs(self.save_folder, exist_ok=True)
        
        # 4. 同时在项目目录建立副本文件夹（用于 RecUS 算法处理）
        # 保持原逻辑，但在子目录下建立
        self.project_save_folder = os.path.join(os.getcwd(), "image", os.path.basename(self.session_folder), sub_folder_name)
        os.makedirs(self.project_save_folder, exist_ok=True)
        # --- [修改逻辑结束] ---

        # 初始化步数变量
        self.total_rotation_steps = int(total_rotation)
        self.current_rotation_step = 0
        self.is_rotating = True
        
        # 立即保存第一张图片 (0度)
        # 注意: 第一次保存无需等待姿态更新，因为此时姿态已经是 MoveRelJ 之前的
        if not self._save_frame_at_step(0):
            QMessageBox.critical(self, "Save Error", "Cannot save image at initial position, please check camera or wait for image update.")
            self._reset_rotation_buttons()
            return

        # 发送第一条旋转指令 (1度)
        command = f"MoveRelJ,0,5,{FORWARD},1;" # Direction: FORWARD=1 (Right turn)
        QTimer.singleShot(100, lambda: self.tcp_manager.send_command(command))
        
        QMessageBox.information(self, "Task Started", f"Ultrasound probe started rotating right {total_rotation} degrees and capturing images.")

    def _continue_rotation_after_delay(self):
        """在等待 300ms 后执行保存图像和发送下一条指令的步骤。"""
        if not self.is_rotating:
            return

        # 2. 检查是否达到总旋转步数 (2x)
        if self.current_rotation_step < self.total_rotation_steps:
            # 2a. 保存当前位置的图像 (此时 latest_tool_pose 应该是最新的)
            self._save_frame_at_step(self.current_rotation_step)
            
            # 2b. 继续发送下一条旋转指令 (1度)
            command = f"MoveRelJ,0,5,{FORWARD},1;" # Direction: FORWARD=1
            
            # 使用 QTimer.singleShot 实现 300ms 延时 (非阻塞) - 确保 MoveRelJ,OK; 消息不会立即返回
            QTimer.singleShot(100, lambda: self.tcp_manager.send_command(command))            
        else:
            # 2c. 保存最后一张图像
            self._save_frame_at_step(self.current_rotation_step)
            
            self.is_rotating = False
            # 重新启用按钮
            self._reset_rotation_buttons()
            QMessageBox.information(self, "Task Completed", f"Completed rotating right {self.total_rotation_steps} degrees and saved {self.total_rotation_steps} images.")
            
            # 1. 调用 RecUS 
            rotation_x = float(self.rotation_range_input.text())
            if RecUS and rotation_x > 30:
                # 在项目目录下运行 pipeline
                RecUS.run_pipeline(self.project_save_folder, rotation_x)
                
                # 2. 调用 vtk2nii 转换
                vtk_file = os.path.join(self.project_save_folder, "Prostate_US_3D_XYZ_to_LPS.vtk")
                if os.path.exists(vtk_file):
                    try:
                        nii_path = vtk2nii.vtk_lps_to_nii_lps(vtk_file)
                        # 3. 将生成的文件复制到桌面文件夹
                        shutil.copy(vtk_file, self.save_folder)
                        shutil.copy(nii_path, self.save_folder)
                        # 仅在全局扫描 (is_global_reconstruction 为 True) 时执行
                        if self.is_global_reconstruction:
                            project_root_nii = os.path.join(os.getcwd(), os.path.basename(nii_path))
                            shutil.copy2(nii_path, project_root_nii)
                            if hasattr(self.main_window, 'right_panel'):
                                self.main_window.right_panel.log_message(f"System: Global NII file copied to project root: {os.path.basename(nii_path)}")
                        # 如果有预览图也拷贝
                        preview = os.path.join(self.project_save_folder, "US_3D_LPS_Axial_Preview.png")
                        if os.path.exists(preview): shutil.copy(preview, self.save_folder)
                    except Exception as e:
                        print(f"Conversion error: {e}")
                QMessageBox.information(self, "Completed", f"Reconstruction files saved to Desktop and Project folder.")
            else:
                QMessageBox.information(self, "Completed", f"Reconstruction skipped.")
            
            # 如果 is_global_reconstruction 为 False，说明是局部重建
            is_local = not self.is_global_reconstruction
            self.scan_finished.emit(is_local)
        
    def continue_rotation(self):
        """
        在接收到机器人反馈后，继续旋转并保存图像。（逻辑已更新，使用 total_rotation_steps）
        在保存图像前引入 300ms 延时，确保姿态数据最新。
        """
        if not self.is_rotating:
            return

        # 1. 移动完成后，增加旋转步数 (代表机器人现在的位置)
        self.current_rotation_step += 1 # Now 1, 2, 3...
        
        # --- 引入非阻塞延时 (300ms)，等待最新的机器人姿态更新 ---
        delay_ms = 500
        QTimer.singleShot(delay_ms, self._continue_rotation_after_delay)

    def rotate_needle_left_x(self):
        """针左转 x 度：修改 Yaw -> Apply All"""
        x = self._get_needle_x_value()
        if x is None: return

        # 使用您指定的 flexible_needle_tab
        needle_tab = self.main_window.flexible_needle_tab 
        try:
            # 获取当前显示的值
            current_yaw = float(needle_tab.yaw_display.text())
            new_yaw = current_yaw + x
            
            # 程序设置文本不会触发 editingFinished，需显式同步关节增量。
            needle_tab.yaw_display.setText(f"{new_yaw:.2f}")
            if not needle_tab.update_inputs_from_yaw_pitch():
                return
            needle_tab.apply_joint_increment()
            self.main_window.status_bar.showMessage(f"Status: Needle Left {x} deg Applied.")
        except Exception as e:
            print(f"Needle control error: {e}")

    def rotate_needle_right_2x(self):
        """针右转 2x 度：一次计算目标 Yaw 并发送运动指令。"""
        x = self._get_needle_x_value()
        if x is None: return

        needle_tab = self.main_window.flexible_needle_tab
        movement_status = needle_tab.movement_status_label.text().strip().lower()
        if not movement_status.endswith("ready"):
            self.main_window.status_bar.showMessage(
                "Status: Needle rotation ignored because Beckhoff is not Ready."
            )
            return

        try:
            total_rotation = 2 * x
            current_yaw = float(needle_tab.yaw_display.text())
            new_yaw = current_yaw - total_rotation

            needle_tab.yaw_display.setText(f"{new_yaw:.2f}")
            if not needle_tab.update_inputs_from_yaw_pitch():
                return
            needle_tab.apply_joint_increment()
            self.main_window.status_bar.showMessage(
                f"Status: Needle Right {total_rotation:.2f} deg Applied."
            )
        except (TypeError, ValueError) as e:
            print(f"Needle control error: {e}")

    def _get_needle_x_value(self):
        """
        从 needle_yaw_range_input 中安全获取针的单侧旋转角度 X (浮点数)。
        如果输入非法或为空，将弹出警告并返回 None。
        """
        # 确保控件已经存在
        if not hasattr(self, 'needle_yaw_range_input'):
            QMessageBox.critical(self, "错误", "未找到 needle_yaw_range_input 控件。")
            return None

        text_val = self.needle_yaw_range_input.text().strip()
        
        if not text_val:
            QMessageBox.warning(self, "提示", "针旋转角度 X 不能为空，请输入数值。")
            return None
            
        try:
            x_value = float(text_val)
            if x_value <= 0:
                QMessageBox.warning(self, "输入错误", "针旋转角度 X 必须大于 0。")
                return None
            return x_value
        except ValueError:
            QMessageBox.warning(self, "输入错误", "针旋转角度输入框内必须为有效的数字。")
            return None
