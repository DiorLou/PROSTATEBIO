import sys
import numpy as np
from PyQt5.QtWidgets import QMainWindow, QWidget, QHBoxLayout, QTabWidget, QStatusBar
from core.tcp_manager import TCPManager
from kinematics.prostate_biopsy_robot_kinematics import RobotKinematics
from ui.beckhoff_tab import BeckhoffTab
from ui.ultrasound_tab import UltrasoundTab
from ui.left_panel import LeftPanel
from ui.right_panel import RightPanel

class RobotControlWindow(QMainWindow):
    VARIABLE_NAMES = ['x0', 'x1', 'x2', 'x3']
    A_PARAMS = [245.472, 0, 0, 0]
    ALPHA_PARAMS = [0, -65, -30, 34]
    D_PARAMS = ['x0 - 174.830', 541.695, 0, 'x3 - 26.0']
    THETA_PARAMS = [30, 'x1', 'x2 + 85.96', 0]

    def __init__(self):
        super().__init__()
        self.setWindowTitle("UR-like Robot Advanced Control Interface (Split View)")
        self.setGeometry(100, 100, 1100, 700)

        # 1. 核心管理器
        self.tcp_manager = TCPManager()
        self.robot_kinematics = RobotKinematics(
            self.A_PARAMS, self.ALPHA_PARAMS, self.D_PARAMS, self.THETA_PARAMS, 
            self.VARIABLE_NAMES, angle_AOC=np.pi/12
        )

        # 2. UI 组件
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)

        # --- Tab 1: Robot Control ---
        robot_tab = QWidget()
        robot_layout = QHBoxLayout(robot_tab)
        
        # 左侧面板：运动、点位
        self.left_panel = LeftPanel(self.tcp_manager, self.robot_kinematics, self)
        # 右侧面板：控制、设置、通信
        self.right_panel = RightPanel(self.tcp_manager, self)
        
        robot_layout.addWidget(self.left_panel, 1)
        robot_layout.addWidget(self.right_panel, 1)
        self.tabs.addTab(robot_tab, "Robot Control")

        # --- Tab 2: Ultrasound ---
        # UltrasoundTab 需要访问 self.latest_tool_pose 等，通过 @property 提供兼容接口
        self.ultrasound_tab = UltrasoundTab(self.tcp_manager, self)
        self.tabs.addTab(self.ultrasound_tab, "Ultrasound Imaging")
        
        # --- Tab 3: Beckhoff ---
        self.beckhoff_tab = BeckhoffTab(self.robot_kinematics, self)
        self.tabs.addTab(self.beckhoff_tab, "Beckhoff Communication")
        
        self.status_bar.showMessage("Status: Ready")

    # --- 兼容性接口 (供 UltrasoundTab 使用) ---
    @property
    def latest_tool_pose(self):
        return self.left_panel.latest_tool_pose

    @property
    def current_tcp_name(self):
        return self.right_panel.current_tcp_name

    @property
    def tcp_e_medical_value(self):
        return self.left_panel.tcp_e_medical_value

    @tcp_e_medical_value.setter
    def tcp_e_medical_value(self, value):
        self.left_panel.tcp_e_medical_value = value

    def compute_and_store_tcp_u_volume(self):
        self.left_panel.compute_and_store_tcp_u_volume()

    def closeEvent(self, event):
        self.tcp_manager.disconnect()
        self.ultrasound_tab.cleanup()
        self.beckhoff_tab.cleanup()
        super().closeEvent(event)