# ui/ultrasound_tab.py
import cv2
import os
import numpy as np
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QMessageBox, QSlider, QFileDialog
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap

class UltrasoundTab(QWidget):
    """
    一个专门用于显示超声图像的标签页。
    它负责与USB摄像头通信，捕获图像流并将其显示在UI上。
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.camera = None
        self.original_frame = None # 存储原始帧
        self.current_frame = None  # 用于存储裁剪后的帧
        self.image_timer = QTimer(self)
        self.image_timer.timeout.connect(self.update_frame)
        
        self.image_label = QLabel("正在等待超声图像...")
        self.start_btn = QPushButton("开启超声探头")
        self.stop_btn = QPushButton("关闭超声探头")
        self.save_btn = QPushButton("保存图像")
        
        self.left_slider = QSlider(Qt.Horizontal)
        self.right_slider = QSlider(Qt.Horizontal)
        self.left_label = QLabel("左侧裁剪: 0")
        self.right_label = QLabel("右侧裁剪: 640")

        self.init_ui()
        self.setup_connections()

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
        
        left_crop_layout = QHBoxLayout()
        # 滑块范围将在启动捕获后动态设置
        self.left_slider.setRange(0, 640)
        self.left_slider.setValue(0)
        left_crop_layout.addWidget(self.left_label)
        left_crop_layout.addWidget(self.left_slider)
        
        right_crop_layout = QHBoxLayout()
        self.right_slider.setRange(0, 640)
        self.right_slider.setValue(640)
        right_crop_layout.addWidget(self.right_label)
        right_crop_layout.addWidget(self.right_slider)

        crop_layout.addLayout(left_crop_layout)
        crop_layout.addLayout(right_crop_layout)
        layout.addWidget(crop_group)

        btn_layout = QHBoxLayout()
        self.start_btn.setFixedSize(120, 40)
        self.stop_btn.setFixedSize(120, 40)
        self.stop_btn.setEnabled(False)
        self.save_btn.setFixedSize(120, 40)
        self.save_btn.setEnabled(False)

        btn_layout.addStretch()
        btn_layout.addWidget(self.start_btn)
        btn_layout.addSpacing(10)
        btn_layout.addWidget(self.stop_btn)
        btn_layout.addSpacing(10)
        btn_layout.addWidget(self.save_btn)
        btn_layout.addStretch()
        
        layout.addLayout(btn_layout)

    def setup_connections(self):
        """连接信号和槽。"""
        self.start_btn.clicked.connect(self.start_capture)
        self.stop_btn.clicked.connect(self.stop_capture)
        self.save_btn.clicked.connect(self.save_image)
        self.left_slider.valueChanged.connect(self.update_crop_value)
        self.right_slider.valueChanged.connect(self.update_crop_value)

    def update_crop_value(self, value):
        """更新裁剪滑块的标签文本，并确保左右边界的逻辑正确性。"""
        left_val = self.left_slider.value()
        right_val = self.right_slider.value()

        if left_val > right_val:
            self.right_slider.setValue(left_val)
            right_val = left_val
        
        self.left_label.setText(f"左侧裁剪: {left_val}")
        self.right_label.setText(f"右侧裁剪: {right_val}")

    def start_capture(self):
        """开始捕获超声图像流。"""
        # 请在这里将 0 替换为您在第1步中找到的正确索引号
        self.camera = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        
        if not self.camera.isOpened():
            QMessageBox.critical(self, "错误", "无法打开摄像头。请检查设备连接。")
            self.stop_btn.setEnabled(False)
            self.start_btn.setEnabled(True)
            self.save_btn.setEnabled(False)
            self.image_label.setText("无法打开摄像头。")
            self.camera = None
            return

        # 主动请求高分辨率
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        ret, frame = self.camera.read()
        if not ret:
            self.stop_capture()
            QMessageBox.critical(self, "错误", "无法从摄像头读取帧。")
            return
            
        actual_height, actual_width, _ = frame.shape

        self.left_slider.setRange(0, actual_width)
        self.right_slider.setRange(0, actual_width)
        self.right_slider.setValue(actual_width)
        self.left_label.setText(f"左侧裁剪: 0")
        self.right_label.setText(f"右侧裁剪: {actual_width}")

        self.image_timer.start(30)
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.save_btn.setEnabled(True)
        self.image_label.setText("正在捕获图像...")

    def stop_capture(self):
        """停止图像捕获。"""
        self.image_timer.stop()
        if self.camera:
            self.camera.release()
            self.camera = None
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.save_btn.setEnabled(False)
        self.image_label.setText("已停止捕获。")
    
    def update_frame(self):
        """从摄像头读取帧，裁剪并显示。"""
        ret, frame = self.camera.read()
        if not ret:
            self.stop_capture()
            QMessageBox.critical(self, "错误", "超声图像流已中断。")
            return
        
        self.original_frame = frame
        
        left_crop = self.left_slider.value()
        right_crop = self.right_slider.value()
        
        # 裁剪图像并存储，以供保存按钮使用
        self.current_frame = self.original_frame[:, left_crop:right_crop]
        
        # 将裁剪后的图像转换为 PyQt 格式，并等比例缩放到显示窗口
        rgb_image = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2RGB)

        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        
        pixmap = QPixmap.fromImage(qt_image).scaled(
            self.image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        self.image_label.setPixmap(pixmap)

    def save_image(self):
        """保存当前裁剪后的图像。"""
        if self.current_frame is None:
            QMessageBox.warning(self, "警告", "没有可保存的图像。请先开启超声探头。")
            return

        file_dialog = QFileDialog(self)
        file_dialog.setWindowTitle("保存图像")
        file_dialog.setNameFilter("图像文件 (*.png *.jpg *.jpeg)")
        file_dialog.setAcceptMode(QFileDialog.AcceptSave)
        file_dialog.setDirectory(os.path.expanduser('~'))

        if file_dialog.exec_() == QFileDialog.Accepted:
            save_path = file_dialog.selectedFiles()[0]
            try:
                cv2.imwrite(save_path, self.current_frame)
                QMessageBox.information(self, "成功", f"图像已保存至:\n{save_path}")
            except Exception as e:
                QMessageBox.critical(self, "错误", f"保存图像时出错: {e}")

    def cleanup(self):
        """在窗口关闭时进行清理。"""
        self.stop_capture()