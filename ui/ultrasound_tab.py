# ui/ultrasound_tab.py
import cv2
import os
import numpy as np
import time
from datetime import datetime
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QMessageBox, QSlider, QFileDialog, QLineEdit
from PyQt5.QtCore import Qt, QTimer 
from PyQt5.QtGui import QImage, QPixmap # 已修正：QImage 和 QPixmap 应该从 QtGui 导入

# Constants for motion direction (来自 main_window.py)
FORWARD = 1
BACKWARD = 0

class UltrasoundTab(QWidget):
    # 默认裁剪常量
    DEFAULT_LEFT_CROP   = 407
    DEFAULT_RIGHT_CROP  = 638
    DEFAULT_TOP_CROP    = 131
    DEFAULT_BOTTOM_CROP = 643
    
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

        # 新增: 旋转范围输入框和按钮
        self.rotation_range_input = QLineEdit("45") # 默认值 45
        self.left_x_btn = QPushButton("Ultrasound Probe Rotate Left x Deg")
        self.right_2x_btn = QPushButton("Ultrasound Probe Rotate Right 2x Deg")

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
        # 将图像显示窗口固定为 640x480，防止窗口无限变大
        self.image_label.setFixedSize(640, 480) 

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
        self.left_slider.setRange(0, 1920)
        self.left_slider.setValue(self.DEFAULT_LEFT_CROP)
        left_crop_layout.addWidget(self.left_label)
        left_crop_layout.addWidget(self.left_slider)
        
        right_crop_layout = QHBoxLayout()
        self.right_slider.setRange(0, 1920) # 已修改为 1920
        self.right_slider.setValue(self.DEFAULT_RIGHT_CROP)    # 已修改为 1920
        right_crop_layout.addWidget(self.right_label)
        right_crop_layout.addWidget(self.right_slider)

        # --- 垂直裁剪 (上下) ---
        top_crop_layout = QHBoxLayout()
        self.top_slider.setRange(0, 1080)
        self.top_slider.setValue(self.DEFAULT_TOP_CROP)
        top_crop_layout.addWidget(self.top_label)
        top_crop_layout.addWidget(self.top_slider)

        bottom_crop_layout = QHBoxLayout()
        self.bottom_slider.setRange(0, 1080)
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
        self.start_btn.setFixedSize(160, 40)
        self.stop_btn.setFixedSize(155, 40)
        self.stop_btn.setEnabled(False)
        self.save_btn.setFixedSize(120, 40)
        self.save_btn.setEnabled(False)
        
        # [新增] 单次保存按钮的设置
        self.single_save_btn.setFixedSize(120, 40)
        self.single_save_btn.setEnabled(False) 
        
        # 旋转范围输入布局
        rotation_input_layout = QHBoxLayout()
        rotation_input_layout.addWidget(QLabel("Rotation Range x Deg:"))
        self.rotation_range_input.setFixedWidth(50) 
        rotation_input_layout.addWidget(self.rotation_range_input)
        
        # 旋转按钮设置
        self.left_x_btn.setFixedSize(250, 40)
        self.right_2x_btn.setFixedSize(250, 40)
        self.left_x_btn.setEnabled(True)
        self.right_2x_btn.setEnabled(False)

        btn_layout.addStretch()
        btn_layout.addWidget(self.start_btn)
        btn_layout.addSpacing(10)
        btn_layout.addWidget(self.stop_btn)
        btn_layout.addSpacing(10)
        
        # [新增] 添加保存单次图像按钮
        btn_layout.addWidget(self.single_save_btn) 
        btn_layout.addSpacing(10)
        
        btn_layout.addWidget(self.save_btn)
        
        # 添加旋转控制组
        btn_layout.addSpacing(20) 
        btn_layout.addLayout(rotation_input_layout) 
        btn_layout.addSpacing(10)
        btn_layout.addWidget(self.left_x_btn)
        btn_layout.addSpacing(10)
        btn_layout.addWidget(self.right_2x_btn)
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
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        ret, frame = self.camera.read()
        if not ret:
            self.stop_capture()
            QMessageBox.critical(self, "Error", "Cannot read frame from camera.")
            return
            
        actual_height, actual_width, _ = frame.shape

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
        """从摄像头读取帧，裁剪并显示。"""
        ret, frame = self.camera.read()
        if not ret:
            self.stop_capture()
            QMessageBox.critical(self, "Error", "Ultrasound image stream interrupted.")
            return
        
        self.original_frame = frame
        
        # 获取四向裁剪值
        left_crop = self.left_slider.value()
        right_crop = self.right_slider.value()
        top_crop = self.top_slider.value()
        bottom_crop = self.bottom_slider.value()
        
        # 裁剪图像并存储 (同时应用垂直和水平裁剪)
        # 注意: Python的切片顺序是 [行/Y轴, 列/X轴]
        self.current_frame = self.original_frame[top_crop:bottom_crop, left_crop:right_crop]
        
        # 将裁剪后的图像转换为 PyQt 格式，并等比例缩放到显示窗口
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

        # 获取定时器间隔 (ms)
        interval_ms = self.real_time_save_timer.interval()

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
        self.real_time_save_timer.start(300) 
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
        # 确保获取主窗口实例，并且它有 latest_tool_pose 属性
        if not robot_control_window or not hasattr(robot_control_window, 'latest_tool_pose'):
             pose = [0.0] * 6
             print("Warning: Cannot get latest tool pose. Using default pose.")
        else:
             pose = robot_control_window.latest_tool_pose
        
        # 格式化工具端位姿: (x,y,z,Rx,Ry,Rz)
        if pose and len(pose) == 6:
            # 姿态字符串格式: (x,y,z,Rx,Ry,Rz)，与旋转捕获命名格式一致
            pose_str = "(" + ",".join([f"{p:.2f}" for p in pose]) + ")"
        else:
            pose_str = "POSE_NA"
            print("Warning: Cannot get valid tool pose data.")

        # 序列号格式化，作为文件名开头的前缀，例如 0000, 0001, ... (使用 4 位格式)
        # 实时保存使用序列号作为“旋转度数”的位置
        rotation_step_str = f"{self.save_sequence_number:04d}"
        
        # 构造新的文件名: (序列号/步数) + (机器臂末端位姿) + .png
        new_filename = f"{rotation_step_str}{pose_str}.png"
        image_path = os.path.join(self.real_time_save_folder, new_filename)

        if cv2.imwrite(image_path, self.current_frame):
            self.save_sequence_number += 1
        else:
            print(f"Real-time image saving failed: {image_path}")
            self._stop_real_time_save() # 保存失败则停止
            # 移除原始的 QMessageBox.critical 调用，防止在定时器线程中阻塞 UI
            
    # --- [修改/新增] 实时保存逻辑 END ---


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
        [修改版] 从输入框获取旋转范围 x 的值。
        现在支持负数（输入负数表示反向旋转），但不支持 0。
        """
        try:
            x_text = self.rotation_range_input.text()
            x = int(x_text) 
            
            # [修改] 只要不等于 0 即可，允许负数
            if x == 0:
                QMessageBox.warning(self, "Input Error", "Rotation range cannot be 0.")
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
        image_path = os.path.join(self.save_folder, new_filename)

        # 立即保存当前图像
        if self.current_frame is not None:
            try:
                cv2.imwrite(image_path, self.current_frame)
                print(f"Image saved: {image_path}")
                return True
            except Exception as e:
                print(f"Error saving image: {e}")
                return False
        return False

    def rotate_left_x(self):
        """
        [修改版] 发送指令，使超声探头左转 x 度。
        如果 x 为负数（例如 -50），则执行右转 50 度。
        """
        x = self._get_rotation_x_value()
        if x is None:
            return
            
        # --- [新增] 方向与角度处理逻辑 ---
        # 默认方向为 BACKWARD (左转)
        move_direction = BACKWARD
        move_angle = x

        # 如果输入是负数 (例如 -50)
        if x < 0:
            move_direction = FORWARD  # 自动切换为 FORWARD (右转)
            move_angle = abs(x)       # 取绝对值 (50)
            print(f"Debug: Input is negative ({x}), switching to RIGHT turn {move_angle} deg.")
        # --------------------------------

        # 执行 TCP_E 检查和姿态记录 (保持原有逻辑不变)
        robot_control_window = self.main_window
        if not robot_control_window or not hasattr(robot_control_window, 'latest_tool_pose'):
            QMessageBox.warning(self, "Connection Error", "Cannot access robot pose data...")
            return

        if robot_control_window.current_tcp_name != "TCP_E":
            QMessageBox.critical(self, "TCP Error", f"Current TCP must be 'TCP_E'...")
            return
            
        # 1. 记录 TCP_E_Medical 并触发 TCP_U_Volume 计算
        robot_control_window.tcp_e_medical_value = list(robot_control_window.latest_tool_pose)
        pose_str = ", ".join([f"{p:.2f}" for p in robot_control_window.tcp_e_medical_value])
        robot_control_window.status_bar.showMessage(f"Status: TCP_E_Medical_value recorded: [{pose_str}]")
        
        # 立即计算 TCP_U_Volume
        robot_control_window.compute_and_store_tcp_u_volume()
            
        # 2. 计算 a_point_in_volume 并发送数据
        # 从 LeftPanel 获取刚刚计算好的 tcp_u_volume
        tcp_u_vol = robot_control_window.left_panel.tcp_u_volume
        # 调用 LeftPanel 的新方法计算 a_point_in_volume
        a_in_vol = robot_control_window.left_panel.calculate_a_point_in_u_volume()
        
        if tcp_u_vol is not None and a_in_vol is not None:
            self._send_volume_data_to_navigation(a_in_vol, tcp_u_vol)
        else:
            QMessageBox.warning(self, "Data Warning", "Failed to calculate A point or TCP_U_Volume...")

        # 3. 发送旋转指令 (使用上面计算好的 move_direction 和 move_angle)
        command = f"MoveRelJ,0,5,{move_direction},{move_angle};"
        
        if self.tcp_manager and self.tcp_manager.is_connected:
            self.tcp_manager.send_command(command)
            
            # 提示信息也做相应更新
            dir_str = "LEFT" if move_direction == BACKWARD else "RIGHT (Negative Input)"
            QMessageBox.information(self, "Command Sent", f"Sent rotate {dir_str} {move_angle} degrees command.\nNavigation data sent.")
        else:
            QMessageBox.warning(self, "Connection Error", "Not connected to robot or TCP manager.")

    def rotate_and_capture_2x(self):
        """开始 2x 度右转，并每1度保存一张图像。（已更新：先保存初始图片）"""
        x = self._get_rotation_x_value()
        if x is None:
            return
            
        total_rotation = 2 * x # Total rotation is 2x
        
        if not self.tcp_manager or not self.tcp_manager.is_connected:
            QMessageBox.warning(self, "Connection Error", "Not connected to robot or TCP manager.")
            return
        
        # 禁用按钮防止重复点击
        self.right_2x_btn.setEnabled(False)
        self.left_x_btn.setEnabled(False)
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(False)
        self.single_save_btn.setEnabled(False) # [修改] 启用/禁用单次保存按钮

        # 1. 定义 'image' 根目录并创建子文件夹
        base_dir = os.path.join(os.getcwd(), "image")
        if not os.path.isdir(base_dir):
            try:
                os.makedirs(base_dir)
            except OSError as e:
                QMessageBox.critical(self, "File System Error", f"Cannot create root save directory ('image'): {e}")
                self._reset_rotation_buttons()
                return

            gitignore_path = os.path.join(base_dir, ".gitignore")
            try:
                with open(gitignore_path, 'w') as f:
                    f.write("# Ignore all ultrasound image data (automatically generated by the program)\n")
                    f.write("ultrasound_images_*/\n") 
                    # [新增] 忽略实时保存文件夹
                    f.write("Realtime_Capture_Interval_*/\n") 
            except Exception as e:
                print(f"Warning: Cannot create .gitignore file: {e}")

        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.save_folder = os.path.join(base_dir, f"ultrasound_images_{timestamp}")
        
        try:
            os.makedirs(self.save_folder, exist_ok=True)
        except OSError as e:
            QMessageBox.critical(self, "File System Error", f"Cannot create save directory: {e}")
            self._reset_rotation_buttons()
            return

        # 2. 初始化步数变量
        self.total_rotation_steps = int(total_rotation) # 存储总步数 (2x)
        self.current_rotation_step = 0
        self.is_rotating = True
        
        # 3. [新增] 立即保存第一张图片 (0度)
        # 注意: 第一次保存无需等待姿态更新，因为此时姿态已经是 MoveRelJ 之前的
        if not self._save_frame_at_step(0):
            QMessageBox.critical(self, "Save Error", "Cannot save image at initial position, please check camera or wait for image update.")
            self._reset_rotation_buttons()
            return

        # 4. 发送第一条旋转指令 (1度)
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
        
    def _send_volume_data_to_navigation(self, a_pt, u_vol):
        """
        [新增] 格式化并发送 A点(Volume系) 和 TCP_U_Volume(Base系) 到导航服务器。
        发送格式示例: "UpdateVolumeData,Ax,Ay,Az,Ux,Uy,Uz,Urx,Ury,Urz;"
        """
        if self.main_window and hasattr(self.main_window, 'navigation_tab'):
            # 构造消息字符串 (保留3位小数)
            msg = f"UpdateVolumeData,{a_pt[0]:.3f},{a_pt[1]:.3f},{a_pt[2]:.3f}," \
                  f"{u_vol[0]:.3f},{u_vol[1]:.3f},{u_vol[2]:.3f}," \
                  f"{u_vol[3]:.3f},{u_vol[4]:.3f},{u_vol[5]:.3f};"
            
            # 1. 通过 NavigationTab 的 Manager 发送
            self.main_window.navigation_tab.nav_manager.send_command(msg)
            
            # 2. [修正] 在 Navigation Communication tab 页中记录日志
            # 直接调用 NavigationTab 的 log_message 方法显示在界面的文本框中
            self.main_window.navigation_tab.log_message(f"Sent Nav Data: {msg}")
        else:
            print("Error: Navigation Tab not accessible.")