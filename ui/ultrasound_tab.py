# ui/ultrasound_tab.py
import cv2
import os
import numpy as np
import time
from datetime import datetime
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QMessageBox, QSlider, QFileDialog, QLineEdit
from PyQt5.QtCore import Qt, QTimer 
from PyQt5.QtGui import QImage, QPixmap
import threading

# 尝试导入 RecUS
try:
    from ultrasound_image_process import RecUS
except ImportError:
    RecUS = None
    print("Warning: Could not import RecUS module. 3D reconstruction will be disabled.")

# 运动方向常量
FORWARD = 1
BACKWARD = 0

class UltrasoundTab(QWidget):
    # 🌟 基于 1280x720 逆时针旋转 90 度后的坐标系：宽度 720, 高度 1280
    DEFAULT_LEFT_CROP   = 0
    DEFAULT_RIGHT_CROP  = 720
    DEFAULT_TOP_CROP    = 0
    DEFAULT_BOTTOM_CROP = 1280
    
    def __init__(self, tcp_manager, parent=None):
        super().__init__(parent)
        self.tcp_manager = tcp_manager
        self.main_window = parent 
        self.camera = None
        self.original_frame = None 
        self.current_frame = None  
        self.image_timer = QTimer(self)
        self.image_timer.timeout.connect(self.update_frame)
        
        self.image_label = QLabel("Waiting for Ultrasound Image...")
        self.start_btn = QPushButton("Start Ultrasound Probe")
        self.stop_btn = QPushButton("Stop Ultrasound Probe")
        self.single_save_btn = QPushButton("Save Single Image")
        self.save_btn = QPushButton("Start Image Saving")
        
        self.is_real_time_saving = False
        self.real_time_save_timer = QTimer(self)
        self.real_time_save_timer.timeout.connect(self._save_real_time_frame) 
        self.real_time_save_folder = ""
        self.save_sequence_number = 0
        
        # 旋转后宽度为 720
        self.left_slider = QSlider(Qt.Horizontal)
        self.right_slider = QSlider(Qt.Horizontal)
        self.left_label = QLabel(f"Left Crop: {self.DEFAULT_LEFT_CROP}")
        self.right_label = QLabel(f"Right Crop: {self.DEFAULT_RIGHT_CROP}")
        
        # 旋转后高度为 1280
        self.top_slider = QSlider(Qt.Horizontal)
        self.bottom_slider = QSlider(Qt.Horizontal)
        self.top_label = QLabel(f"Top Crop: {self.DEFAULT_TOP_CROP}")
        self.bottom_label = QLabel(f"Bottom Crop: {self.DEFAULT_BOTTOM_CROP}")

        self.is_rotating = False
        self.current_rotation_step = 0
        self.total_rotation_steps = 0 
        self.save_folder = "" 

        self.rotation_range_input = QLineEdit("45") 
        self.left_x_btn = QPushButton("Ultrasound Probe Rotate Left x Deg")
        self.right_2x_btn = QPushButton("Ultrasound Probe Rotate Right 2x Deg")

        self.init_ui()
        self.setup_connections()

    def init_ui(self):
        """构建UI界面。"""
        layout = QVBoxLayout(self)
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setStyleSheet("border: 2px solid grey;")
        # 调整显示窗口尺寸以适应 720x1280 比例
        self.image_label.setFixedSize(450, 800) 

        image_layout = QHBoxLayout()
        image_layout.addStretch()
        image_layout.addWidget(self.image_label)
        image_layout.addStretch()
        layout.addLayout(image_layout, 1)

        crop_group = QWidget()
        crop_layout = QVBoxLayout(crop_group)
        
        self.left_slider.setRange(0, 720)
        self.left_slider.setValue(self.DEFAULT_LEFT_CROP)
        self.right_slider.setRange(0, 720)
        self.right_slider.setValue(self.DEFAULT_RIGHT_CROP)
        self.top_slider.setRange(0, 1280)
        self.top_slider.setValue(self.DEFAULT_TOP_CROP)
        self.bottom_slider.setRange(0, 1280)
        self.bottom_slider.setValue(self.DEFAULT_BOTTOM_CROP)

        for l, s in [(self.left_label, self.left_slider), (self.right_label, self.right_slider),
                     (self.top_label, self.top_slider), (self.bottom_label, self.bottom_slider)]:
            h_layout = QHBoxLayout()
            h_layout.addWidget(l)
            h_layout.addWidget(s)
            crop_layout.addLayout(h_layout)

        layout.addWidget(crop_group)

        btn_layout = QHBoxLayout()
        btn_layout.addStretch()
        btn_layout.addWidget(self.start_btn)
        btn_layout.addSpacing(10)
        btn_layout.addWidget(self.stop_btn)
        btn_layout.addSpacing(10)
        btn_layout.addWidget(self.single_save_btn) 
        btn_layout.addSpacing(10)
        btn_layout.addWidget(self.save_btn)
        btn_layout.addSpacing(20) 
        
        rotation_input_layout = QHBoxLayout()
        rotation_input_layout.addWidget(QLabel("Rotation Range x Deg:"))
        self.rotation_range_input.setFixedWidth(50) 
        rotation_input_layout.addWidget(self.rotation_range_input)
        
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
        self.single_save_btn.clicked.connect(self.save_image) 
        self.save_btn.clicked.connect(self.toggle_real_time_save) 
        self.left_slider.valueChanged.connect(self.update_crop_value)
        self.right_slider.valueChanged.connect(self.update_crop_value)
        self.top_slider.valueChanged.connect(self.update_crop_value)
        self.bottom_slider.valueChanged.connect(self.update_crop_value)
        self.left_x_btn.clicked.connect(self.rotate_left_x)
        self.right_2x_btn.clicked.connect(self.rotate_and_capture_2x)
        self.tcp_manager.message_received.connect(self.handle_incoming_message)

    def start_capture(self):
        """启动摄像头并设置分辨率。"""
        # 🌟 修复点：先尝试以默认方式打开
        self.camera = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        
        if not self.camera.isOpened():
            QMessageBox.critical(self, "Error", "Cannot open camera.")
            return

        # 🌟 成功打开后再设置分辨率
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        # 留出一点时间让硬件响应设置
        time.sleep(0.2)

        ret, frame = self.camera.read()
        if not ret:
            self.stop_capture()
            QMessageBox.critical(self, "Error", "Cannot read frame from camera.")
            return
            
        # 检查实际获取到的分辨率并在后台打印
        h, w = frame.shape[:2]
        print(f"DEBUG: Input Source Resolution: {w}x{h}")

        self.update_crop_value(0)
        self.image_timer.start(30)
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.save_btn.setEnabled(True)
        self.single_save_btn.setEnabled(True)
        self.image_label.setText("Capturing...")

    def update_frame(self):
        """读取帧、逆时针旋转 90 度并裁剪。"""
        ret, frame = self.camera.read()
        if not ret:
            self.stop_capture()
            return
        
        # 🌟 逆时针 90 度旋转：1280x720 -> 720(宽)x1280(高)
        rotated_frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        self.original_frame = rotated_frame
        
        lc, rc = self.left_slider.value(), self.right_slider.value()
        tc, bc = self.top_slider.value(), self.bottom_slider.value()
        
        # 裁剪旋转后的图像
        self.current_frame = self.original_frame[tc:bc, lc:rc]
        
        rgb_image = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        
        pixmap = QPixmap.fromImage(qt_image).scaled(
            self.image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        self.image_label.setPixmap(pixmap)

    def update_crop_value(self, value):
        """更新滑块值。"""
        left_val, right_val = self.left_slider.value(), self.right_slider.value()
        top_val, bottom_val = self.top_slider.value(), self.bottom_slider.value()

        if left_val > right_val:
            self.right_slider.setValue(left_val)
            right_val = left_val
        if top_val > bottom_val:
            self.bottom_slider.setValue(top_val)
            bottom_val = top_val

        self.left_label.setText(f"Left Crop: {left_val}")
        self.right_label.setText(f"Right Crop: {right_val}")
        self.top_label.setText(f"Top Crop: {top_val}")
        self.bottom_label.setText(f"Bottom Crop: {bottom_val}")

    def save_image(self):
        """保存单次图像。"""
        if self.current_frame is None:
            return

        file_dialog = QFileDialog(self)
        file_dialog.setAcceptMode(QFileDialog.AcceptSave)
        file_dialog.setNameFilter("Image Files (*.png *.jpg)")
        if file_dialog.exec_() == QFileDialog.Accepted:
            save_path = file_dialog.selectedFiles()[0]
            if not save_path.lower().endswith(('.png', '.jpg')):
                save_path += '.png'
            cv2.imwrite(save_path, self.current_frame)
            # 弹出成功提示并显示实际保存的尺寸
            h, w = self.current_frame.shape[:2]
            QMessageBox.information(self, "Success", f"Image saved: {w}x{h}")

    def stop_capture(self):
        """停止捕获。"""
        self.image_timer.stop()
        if self.is_real_time_saving:
            self._stop_real_time_save()
        if self.camera:
            self.camera.release()
            self.camera = None
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.save_btn.setEnabled(False)
        self.single_save_btn.setEnabled(False)
        self.image_label.setText("Capture Stopped.")

    def toggle_real_time_save(self):
        """切换实时保存状态。"""
        if not self.is_real_time_saving:
            self._start_real_time_save()
        else:
            self._stop_real_time_save()

    def _start_real_time_save(self):
        if self.current_frame is None: return
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.real_time_save_folder = os.path.join(os.getcwd(), "image", f"Realtime_{timestamp}")
        os.makedirs(self.real_time_save_folder, exist_ok=True)
        self.save_sequence_number = 0
        self.is_real_time_saving = True
        self.save_btn.setText("Stop Image Saving") 
        self.save_btn.setStyleSheet("background-color: salmon;") 
        self.real_time_save_timer.start(300) 

    def _stop_real_time_save(self):
        self.real_time_save_timer.stop()
        self.is_real_time_saving = False
        self.save_btn.setText("Start Image Saving") 
        self.save_btn.setStyleSheet("")

    def _save_real_time_frame(self):
        if self.current_frame is None: return
        pose = getattr(self.main_window, 'latest_tool_pose', [0.0]*6)
        pose_str = "(" + ",".join([f"{p:.2f}" for p in pose]) + ")"
        new_filename = f"{self.save_sequence_number:04d}{pose_str}.png"
        cv2.imwrite(os.path.join(self.real_time_save_folder, new_filename), self.current_frame)
        self.save_sequence_number += 1

    def handle_incoming_message(self, message):
        """处理机器人反馈消息。"""
        if self.is_rotating and "MoveRelJ" in message and "OK" in message:
            self.continue_rotation()

    def rotate_left_x(self):
        """向左旋转。"""
        try:
            x_text = self.rotation_range_input.text()
            x = int(x_text)
            self.tcp_manager.send_command(f"MoveRelJ,0,5,{BACKWARD},{x};")
        except ValueError:
            pass

    def rotate_and_capture_2x(self):
        """扫描旋转并保存图像。"""
        try:
            x = int(self.rotation_range_input.text())
            self.total_rotation_steps = 2 * x
            self.current_rotation_step = 0
            self.is_rotating = True
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.save_folder = os.path.join(os.getcwd(), "image", f"ultrasound_images_{timestamp}")
            os.makedirs(self.save_folder, exist_ok=True)
            self._save_frame_at_step(0)
            QTimer.singleShot(100, lambda: self.tcp_manager.send_command(f"MoveRelJ,0,5,{FORWARD},1;"))
        except ValueError:
            pass

    def continue_rotation(self):
        self.current_rotation_step += 1
        QTimer.singleShot(500, self._continue_rotation_after_delay)

    def _continue_rotation_after_delay(self):
        if not self.is_rotating: return
        if self.current_rotation_step <= self.total_rotation_steps:
            self._save_frame_at_step(self.current_rotation_step)
            if self.current_rotation_step < self.total_rotation_steps:
                self.tcp_manager.send_command(f"MoveRelJ,0,5,{FORWARD},1;")
            else:
                self.is_rotating = False
                self.start_btn.setEnabled(True)

    def _save_frame_at_step(self, step_degree):
        if self.current_frame is not None:
            pose = getattr(self.main_window, 'latest_tool_pose', [0.0]*6)
            pose_str = "(" + ",".join([f"{p:.2f}" for p in pose]) + ")"
            cv2.imwrite(os.path.join(self.save_folder, f"{step_degree:03d}{pose_str}.png"), self.current_frame)

    def cleanup(self):
        """清理资源。"""
        self.stop_capture()