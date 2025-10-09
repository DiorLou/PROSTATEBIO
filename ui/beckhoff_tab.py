import numpy as np
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, 
    QLabel, QLineEdit, QPushButton, QMessageBox, QGroupBox
)
from PyQt5.QtCore import Qt

class BeckhoffTab(QWidget):
    """
    Beckhoff通信和穿刺针运动学计算标签页。
    """
    def __init__(self, robot_kinematics, parent=None):
        super().__init__(parent)
        # 接收并存储 RobotKinematics 实例
        self.robot = robot_kinematics
        
        # UI 元素存储
        self.vector_inputs = [None] * 3  # X, Y, Z
        self.result_labels = {}         # J2, J3
        
        self.init_ui()
        self.setup_connections()

    def init_ui(self):
        """构建Beckhoff通信标签页的UI。"""
        main_layout = QVBoxLayout(self)

        # -----------------------------------------------------------------
        # 1. 穿刺针 Vector 定位模块
        # -----------------------------------------------------------------
        vector_group = QGroupBox("穿刺针Vector定位 (逆运动学)")
        vector_layout = QVBoxLayout(vector_group)

        # Vector 输入部分
        input_grid = QGridLayout()
        vector_labels = ["X:", "Y:", "Z:"]
        for i, label_text in enumerate(vector_labels):
            label = QLabel(label_text)
            text_box = QLineEdit("0.00")
            text_box.setPlaceholderText("请输入单位向量值")
            self.vector_inputs[i] = text_box
            input_grid.addWidget(label, 0, i * 2)
            input_grid.addWidget(text_box, 0, i * 2 + 1)
        
        self.input_vector_btn = QPushButton("计算关节值 (J2, J3)")
        
        vector_layout.addLayout(input_grid)
        vector_layout.addWidget(self.input_vector_btn, alignment=Qt.AlignCenter)
        
        # 结果显示部分
        result_group = QGroupBox("计算结果")
        result_layout = QGridLayout(result_group)
        
        result_labels = ["关节 J2 (度):", "关节 J3 (度):"]
        result_keys = ["J2", "J3"]
        
        for i, label_text in enumerate(result_labels):
            label = QLabel(label_text)
            value_label = QLabel("N/A")
            value_label.setStyleSheet("background-color: lightgrey; border: 1px inset grey;")
            self.result_labels[result_keys[i]] = value_label
            result_layout.addWidget(label, i, 0)
            result_layout.addWidget(value_label, i, 1)

        vector_layout.addWidget(result_group)
        
        main_layout.addWidget(vector_group)
        
        # -----------------------------------------------------------------
        # 2. Beckhoff通信示例 (占位符)
        # -----------------------------------------------------------------
        beckhoff_placeholder_group = QGroupBox("Beckhoff PLC 通信")
        beckhoff_placeholder_layout = QVBoxLayout(beckhoff_placeholder_group)
        beckhoff_placeholder_layout.addWidget(QLabel("此模块可用于集成 Beckhoff ADS 读写功能。"))
        
        main_layout.addWidget(beckhoff_placeholder_group)
        main_layout.addStretch()

    def setup_connections(self):
        """连接信号和槽。"""
        self.input_vector_btn.clicked.connect(self.calculate_joint_values)

    def calculate_joint_values(self):
        """读取 Vector 值，计算 J2 和 J3 关节角，并显示结果。"""
        try:
            x = float(self.vector_inputs[0].text())
            y = float(self.vector_inputs[1].text())
            z = float(self.vector_inputs[2].text())
            
            needle_vector = np.array([x, y, z])
            
            # 自动归一化处理
            norm = np.linalg.norm(needle_vector)
            if not np.isclose(norm, 1.0) and norm > 1e-6:
                 QMessageBox.warning(self, "警告", f"输入向量的模长为 {norm:.4f}，已自动归一化。")
                 needle_vector = needle_vector / norm
            elif norm < 1e-6:
                 QMessageBox.critical(self, "输入错误", "Vector 模长接近零，无法定义方向。")
                 return
            
            # 调用运动学逆解函数
            joint_values = self.robot.get_joint23_value(needle_vector)
            
            # get_joint23_value返回的是 [joint1_val, joint2_val_compensated2]
            # 根据 kinematics 文件的定义，它实际上返回的是 J2 和 J3 的值
            j2_val = joint_values[0]
            j3_val = joint_values[1]
            
            # 显示结果
            self.result_labels["J2"].setText(f"{j2_val:.4f}")
            self.result_labels["J3"].setText(f"{j3_val:.4f}")
            
            # 尝试更新主窗口的状态栏
            parent_window = self.parent()
            if hasattr(parent_window, 'status_bar'):
                parent_window.status_bar.showMessage(f"状态: 关节 J2={j2_val:.4f}, J3={j3_val:.4f} 计算完成。")
            
        except ValueError:
            QMessageBox.critical(self, "输入错误", "Vector X, Y, Z 必须是有效数字！")
        except Exception as e:
            QMessageBox.critical(self, "计算错误", f"逆运动学求解失败: {e}")