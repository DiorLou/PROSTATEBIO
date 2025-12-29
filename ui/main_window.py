import sys
import numpy as np
from PyQt5.QtWidgets import QMainWindow, QWidget, QHBoxLayout, QTabWidget, QStatusBar
from core.tcp_manager import TCPManager
from kinematics.prostate_biopsy_robot_kinematics import RobotKinematics
# [修改] 导入 Manager 和 Tab 类, 以及新的 Flexible Tab
from ui.beckhoff_tab import BeckhoffTab, BeckhoffManager
from ui.flexible_needle_tab import FlexibleNeedleTab
from ui.ultrasound_tab import UltrasoundTab
from ui.left_panel import LeftPanel
from ui.right_panel import RightPanel
from ui.navigation_tab import NavigationTab

class RobotControlWindow(QMainWindow):
    VARIABLE_NAMES = ['x0', 'x1', 'x2', 'x3']
    A_PARAMS = [258.75, 0, 0, 0]
    ALPHA_PARAMS = [0, -65, -30, 34]
    D_PARAMS = ['x0 - 206.717', 571.008, 0, 'x3 - 37.318']
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
        
        # [NEW] 初始化单一的 Beckhoff Manager (通信逻辑核心)
        self.beckhoff_manager = BeckhoffManager(self)

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
        self.ultrasound_tab = UltrasoundTab(self.tcp_manager, self)
        self.tabs.addTab(self.ultrasound_tab, "Ultrasound Imaging")
        
        # --- Tab 3: Beckhoff (Instance 1) - Standard ---
        # 传递共享的 manager
        self.beckhoff_tab = BeckhoffTab(self.beckhoff_manager, self.robot_kinematics, self)
        self.tabs.addTab(self.beckhoff_tab, "Beckhoff Communication")
        
        # --- Tab 4: Navigation Communication ---
        self.navigation_tab = NavigationTab(self)
        self.tabs.addTab(self.navigation_tab, "Navigation Communication")
        
        # --- [NEW] Tab 5: Flexible needle steering (Instance 2) - Flexible Tab ---
        # 使用新的 FlexibleNeedleTab 类，但传递相同的 manager 实现状态同步
        self.flexible_needle_tab = FlexibleNeedleTab(self.beckhoff_manager, self.robot_kinematics, self)
        self.tabs.addTab(self.flexible_needle_tab, "Flexible needle steering")
        
        # [连接信号] 将 Beckhoff Tab 的位置更新信号连接到 Navigation Tab
        # 只需要连接其中一个 Tab 的信号即可，因为它们的数据源是一样的
        # 或者为了保险起见，我们可以监听 Manager 的信号，这里保持原样连接 beckhoff_tab 即可
        self.beckhoff_tab.beckhoff_position_update.connect(self.navigation_tab.update_needle_pose_in_volume)
        
        self.status_bar.showMessage("Status: Ready")

    # --- 兼容性接口 (供 UltrasoundTab 使用) ---
    @property
    def latest_tool_pose(self):
        return self.left_panel.latest_tool_pose

    @property
    def current_tcp_name(self):
        return self.right_panel.current_tcp_name

    @property
    def tcp_e_in_ultrasound_zero_deg(self):
        return self.left_panel.tcp_e_in_ultrasound_zero_deg
    
    @property
    def a_point_in_tcp_p(self):
        return self.left_panel.a_point_in_tcp_p

    @tcp_e_in_ultrasound_zero_deg.setter
    def tcp_e_in_ultrasound_zero_deg(self, value):
        self.left_panel.tcp_e_in_ultrasound_zero_deg = value

    def compute_and_store_volume_in_base(self):
        self.left_panel.compute_and_store_volume_in_base()

    def closeEvent(self, event):
        self.tcp_manager.disconnect()
        # 清理 Beckhoff Manager
        self.beckhoff_manager.cleanup()
        self.ultrasound_tab.cleanup()
        self.navigation_tab.cleanup()
        super().closeEvent(event)