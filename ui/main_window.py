# ui/main_window.py
import sys
import json
import os
import numpy as np
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QLineEdit, QPushButton,
    QTextEdit, QStatusBar, QGridLayout, QMessageBox,
    QCheckBox, QSlider, QTabWidget, QComboBox,
    # 新增用于B点选择对话框和文件读取的控件
    QDialog, QTableWidget, QTableWidgetItem, QHeaderView, QAbstractItemView,
    QRadioButton, QFileDialog 
)
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal
from PyQt5.QtGui import QCloseEvent, QFont, QImage, QPixmap
from core.tcp_manager import TCPManager
from core.ultrasound_plane import calculate_rotation_for_plane_alignment, calculate_new_rpy_for_b_point, get_final_tcp_e_position_after_delta_rotation
from kinematics.prostate_biopsy_robot_kinematics import RobotKinematics
from ui.beckhoff_tab import BeckhoffTab
import pytransform3d.rotations as pyrot
from ui.ultrasound_tab import UltrasoundTab

# Constants for motion direction
# 移动方向常量
FORWARD = 1
BACKWARD = 0

DATA_FILE_NAME = "saved_robot_data.json"

class BPointSelectionDialog(QDialog):
    """
    修改后的对话框：用于显示从TXT文件读取的B点列表，并允许用户多选。
    返回选中的所有点的数据列表。
    """
    
    # 信号：当用户选择并确认多个B点后发出，传递选定点的列表
    # 列表中每个元素包含: [p_b_in_u_pose (6), p_base_pose (6), rotation_angle_deg (1), index (int)]
    b_points_selected = pyqtSignal(list) 

    def __init__(self, b_point_data_list, parent=None):
        super().__init__(parent)
        self.setWindowTitle("B Point Localization Selection (Multiple)")
        self.setMinimumSize(900, 400)
        # b_point_data_list 的元素结构已修改为: (p_u_pose, p_base_pose, angle, index)
        self.b_point_data_list = b_point_data_list 
        self.checkboxes = [] # 修正: 初始化 checkboxes 列表

        main_layout = QVBoxLayout(self)

        # 1. 结果表格
        self.table = QTableWidget()
        self.table.setColumnCount(5) # 新增一列用于显示编号
        self.table.setHorizontalHeaderLabels(["Select", "Index", "B Point (TCP_U) Pose", "B Point (Base) Pose", "OA Axis Rotation Angle (Deg)"])
        # 允许内容自适应宽度，并限制只能选择行
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        # 允许单元格选择，但不自动改变选择状态
        self.table.setSelectionMode(QAbstractItemView.NoSelection) 

        # 填充表格
        self._populate_table()

        main_layout.addWidget(self.table)
        
        # 2. 按钮布局
        btn_layout = QHBoxLayout()
        self.ok_btn = QPushButton("Confirm Selection")
        self.cancel_btn = QPushButton(f"Cancel (Loaded {len(self.b_point_data_list)} points)")
        btn_layout.addStretch()
        btn_layout.addWidget(self.ok_btn)
        btn_layout.addWidget(self.cancel_btn)
        btn_layout.addStretch()
        
        main_layout.addLayout(btn_layout)

        # 3. 连接信号
        self.ok_btn.clicked.connect(self._accept_selection)
        self.cancel_btn.clicked.connect(self.reject)
        
        # 修正: 增加 cellClicked 连接，用于手动切换复选框状态
        self.table.cellClicked.connect(self._toggle_checkbox_on_click) 
        
        # 修正: 移除默认全选的循环，默认状态为不选中 (False)

    def _populate_table(self):
        """填充表格数据，使用 QCheckBox 实现多选。"""
        # 宽度常量
        SELECTION_COLUMN_WIDTH = 180 

        self.table.setRowCount(len(self.b_point_data_list))
        
        for row, data in enumerate(self.b_point_data_list):
            
            # Data: [p_u_pose (6), p_base_pose (6), angle (1), index (1)]
            p_u_pose, p_base_pose, angle, index = data 
            
            # 列 0: 复选框
            checkbox = QCheckBox()
            # 默认不选中
            checkbox.setChecked(False)
            self.checkboxes.append(checkbox) # 存储引用
            
            # 使用一个 QWidget 作为容器来居中复选框
            check_container = QWidget()
            check_layout = QHBoxLayout(check_container)
            check_layout.addWidget(checkbox)
            check_layout.setAlignment(Qt.AlignCenter)
            check_layout.setContentsMargins(0, 0, 0, 0)
            
            self.table.setCellWidget(row, 0, check_container)

            # 列 1: 编号
            self.table.setItem(row, 1, QTableWidgetItem(f"B{index}"))

            # 列 2: TCP_U 姿态
            p_u_str = f"({p_u_pose[0]:.2f}, {p_u_pose[1]:.2f}, {p_u_pose[2]:.2f}, {p_u_pose[3]:.2f}, {p_u_pose[4]:.2f}, {p_u_pose[5]:.2f})"
            self.table.setItem(row, 2, QTableWidgetItem(p_u_str))

            # 列 3: Base 姿态
            p_base_str = f"({p_base_pose[0]:.2f}, {p_base_pose[1]:.2f}, {p_base_pose[2]:.2f}, {p_base_pose[3]:.2f}, {p_base_pose[4]:.2f}, {p_base_pose[5]:.2f})"
            self.table.setItem(row, 3, QTableWidgetItem(p_base_str))
            
            # 列 4: OA 轴旋转角度
            self.table.setItem(row, 4, QTableWidgetItem(f"{angle:.2f}"))
        
        # 修正: 设置“选择”列的宽度与按钮一致
        self.table.setColumnWidth(0, SELECTION_COLUMN_WIDTH) 
        self.table.resizeColumnsToContents()

    def _toggle_checkbox_on_click(self, row, column):
        """手动处理单元格点击事件，切换对应行的复选框状态。"""
        # 检查点击是否在表格的有效范围内
        if 0 <= row < len(self.checkboxes):
            checkbox = self.checkboxes[row]
            # 切换状态
            checkbox.setChecked(not checkbox.isChecked())

    def _accept_selection(self):
        """确认多选，收集选中的点数据并发出信号。显式检查 isChecked()。"""
        selected_data_list = []
        
        # 遍历存储的复选框列表，显式读取其状态
        for row, checkbox in enumerate(self.checkboxes):
            if checkbox.isChecked(): 
                # 返回原始数据 (p_u_pose, p_base_pose, angle, index)
                selected_data_list.append(self.b_point_data_list[row]) 

        if not selected_data_list:
            QMessageBox.warning(self, "Warning", "Please select at least one B point.")
            return

        self.b_points_selected.emit(selected_data_list) 
        self.accept()

class RobotControlWindow(QMainWindow):
    """
    此类继承自 QMainWindow，是整个应用程序的图形界面。
    它负责构建UI，并与后台的 TCPManager/RobotKinematics 进行交互。
    """
    # --- Kinematics Constants: 机器人 DH 参数 --- #
    VARIABLE_NAMES = ['x0', 'x1', 'x2', 'x3']
    A_PARAMS = [247.204, 0, 0, 0]
    ALPHA_PARAMS = [0, -65, -30, 34]
    D_PARAMS = ['x0 - 184.845', 545.517, 0, 'x3 + 60.7']
    THETA_PARAMS = [30, 'x1', 'x2 + 85.96', 0]
    
    def __init__(self):
        super().__init__()
        # 设置窗口标题。
        self.setWindowTitle("UR-like Robot Advanced Control Interface (PyQt5)")
        # 设置窗口的初始位置和大小。
        self.setGeometry(100, 100, 900, 700)

        # --- 1. 初始化GUI所需的关节显示变量 ---
        self.num_joints = 6
        # 用于存储显示关节角度的 QLabel 控件引用。
        self.joint_vars = [None] * self.num_joints
        # 预留用于TCP变量的列表。
        self.cur_tcp_vars = [None] * 6
        self.tcp_vars = [None] * 6
        self._moving_joint_index = -1  # 正在连续微调的关节索引，-1表示没有。
        # 移动方向，1代表正向，0代表反向。
        self._moving_direction = BACKWARD
        self._moving_tcp_index = -1    # 正在连续微调的TCP坐标索引。
        
        # 新增用于判断是否已进入连续模式的标志
        self._is_continuous_mode_active = False # 关节连续模式状态
        self._is_continuous_tcp_mode_active = False # TCP连续模式状态
        
        # 新增用于记录A点和O点坐标的文本框列表
        self.a_vars = [None] * 3
        self.o_vars = [None] * 3
        self.e_vars = [None] * 3
        self.b_vars_in_base = [None] * 3 
        self.b_point_position_in_base = np.zeros(3)
        self.calculated_b_points = [] # <--- ADDED: 存储从文件读取并计算的所有B点数据
        self.b_point_dropdown = None  # <--- ADDED: B点选择下拉列表

        # 新增用于分步旋转的状态变量
        self.b_point_rotation_steps = []  # 累积旋转矩阵的列表
        self.current_b_point_step = -1    # 当前执行的步骤索引
        self.initial_tcp_pose_for_b_rot = None # 旋转序列开始时的初始姿态 (6D array)
        self.b_point_o_point = None       # O 点坐标 (3D array)

        
        # [修改] 用于存储从机器人读取的 [X, Y, Z, Rx, Ry, Rz]
        self.tcp_u_definition_pose = None  
        # [新增] 用于标记期待的响应类型 (替代按钮，改为连接时期待)
        self.temp_expected_response = None
        
        self.get_a_btn = None
        self.get_o_btn = None
        self.get_e_btn = None

        self.save_data_btn = None
        self.load_data_btn = None # <--- [新增] 读取按钮引用
        self.align_planes_btn = None
        self.rotate_ultrasound_plane_to_b_btn = None
        
        self.init_joint_pos_btn = None # 新增初始化关节按钮成员变量
        
        self.set_tcp_o_btn = None
        self.set_tcp_p_btn = None  # <--- 新增: TCP_P 按钮成员变量
        self.set_tcp_u_btn = None  # <--- 新增: TCP_U 按钮成员变量
        self.set_tcp_e_btn = None  # <--- 新增: TCP_E 按钮成员变量
        self.set_tcp_tip_btn = None
        self.read_tcp_o_btn = None
        self.read_tcp_tip_btn = None
        self.read_cur_tcp_btn = None
        self.set_cur_tcp_btn = None
        
        # 新增：Tool坐标系复选框
        self.tool_coord_checkbox = None 
        self.input_tool_pose_btn = None
        
        # 新增一个成员变量来存储最新的工具端姿态
        self.latest_tool_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # [新增] 用于存储医疗用途的 TCP_E 值 (点击左转按钮时记录)
        self.tcp_e_medical_value = None
        
        # [新增] 初始化 tcp_u_volume
        self.tcp_u_volume = None
        
        # QTimer 用于实现按住按钮连续微调关节的功能。
        self.continuous_move_timer = QTimer(self)
        # 将定时器的超时信号连接到 continuous_move 方法。
        self.continuous_move_timer.timeout.connect(self.continuous_move)
        
        # 新增用于延迟2秒启动连续模式的单次定时器
        self.start_continuous_move_timer = QTimer(self)
        self.start_continuous_move_timer.setSingleShot(True)
        self.start_continuous_move_timer.timeout.connect(self._start_joint_continuous_mode)
        
        # QTimer 用于实现按住按钮连续微调TCP的功能。
        self.continuous_tcp_move_timer = QTimer(self)
        self.continuous_tcp_move_timer.timeout.connect(self.continuous_tcp_move)
        
        # 新增：用于延迟2秒启动连续TCP模式的单次定时器
        self.start_continuous_tcp_move_timer = QTimer(self)
        self.start_continuous_tcp_move_timer.setSingleShot(True)
        self.start_continuous_tcp_move_timer.timeout.connect(self._start_tcp_continuous_mode)
        
        # 一个列表，用于存储用于设置TCP参数的 QLineEdit 控件的引用。
        self.tcp_input_entries = []

        # 新增：记录当前正在使用的TCP名称
        self.current_tcp_name = "TCP_E" 
        
        # --- 2. 初始化业务逻辑变量 ---
        self.power_state = 0  # 机器人的电源状态：0为未上电，1为已上电。
        self.enable_state = 0 # 机器人的使能状态：0为未使能，1为已使能。
        
        # 实例化 TCPManager 类，所有通信逻辑都通过它来调用。
        self.tcp_manager = TCPManager()
        
        # 实例化 RobotKinematics 类。
        self.robot_kinematics = RobotKinematics(
            self.A_PARAMS, 
            self.ALPHA_PARAMS, 
            self.D_PARAMS, 
            self.THETA_PARAMS, 
            self.VARIABLE_NAMES,
            angle_AOC=np.pi/12
        )
        
        # 将 tcp_manager 和父级实例 (self) 传递给 UltrasoundTab
        self.ultrasound_tab = UltrasoundTab(self.tcp_manager, self)
        
        # 实例化 BeckhoffTab 类。 # 新增
        self.beckhoff_tab = BeckhoffTab(self.robot_kinematics, self)

        # --- 3. 初始化UI ---
        self.init_ui()
        # --- 4. 连接信号和槽 ---
        self.setup_connections()

    def init_ui(self):
        """构建所有UI组件并使用布局管理器进行排列。"""
        # 使用 QTabWidget 作为主窗口的中心部件
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        # 创建第一个标签页，用于现有的机器人控制
        robot_control_tab = QWidget()
        main_layout = QHBoxLayout(robot_control_tab)

        # 左侧面板布局，包含电机控制和状态显示模块。
        left_panel_layout = QVBoxLayout()
        
        # 新增：一个水平布局容器来容纳电机微调和运动速率模块
        top_left_layout = QHBoxLayout()
        self.create_motor_group(top_left_layout)
        self.create_override_group(top_left_layout) # 新增：运动速率分组框
        
        left_panel_layout.addLayout(top_left_layout)
        self.create_tool_state_group(left_panel_layout)
        self.create_record_oae_state_group(left_panel_layout)
        self.create_b_point_group(left_panel_layout)
        self.create_data_persistence_group(left_panel_layout)
        
        # 在最底部添加一个弹簧，将所有内容向上推
        left_panel_layout.addStretch()
        main_layout.addLayout(left_panel_layout, 1)

        # 右侧面板布局，包含高级机器人控制和TCP通信模块。
        right_panel_layout = QVBoxLayout()
        self.create_ur_control_group(right_panel_layout)
        self.create_teach_mode_group(right_panel_layout) # 新增：示教功能分组框
        self.create_set_tcp_group(right_panel_layout) # 新增：工具坐标系设置分组框
        self.create_cur_tcp_group(right_panel_layout)
        self.create_tcp_group(right_panel_layout)
        main_layout.addLayout(right_panel_layout, 1)
        
        # 将机器人控制标签页添加到 QTabWidget
        self.tabs.addTab(robot_control_tab, "Robot Control")

        # 将超声图像标签页添加到 QTabWidget
        self.tabs.addTab(self.ultrasound_tab, "Ultrasound Imaging")
        
        # 将 Beckhoff 通信标签页添加到 QTabWidget
        self.tabs.addTab(self.beckhoff_tab, "Beckhoff Communication")

        # 状态栏，用于在底部显示简短的程序状态信息。
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Status: Ready")

    def setup_connections(self):
        """
        集中管理所有信号和槽的连接。
        将UI控件的操作连接到相应的处理方法，并将TCPManager的信号连接到UI更新方法。
        """
        self.connect_button.clicked.connect(self.connect_tcp)
        self.disconnect_button.clicked.connect(self.disconnect_tcp)
        self.send_button.clicked.connect(self.send_message)
        self.send_entry.returnPressed.connect(self.send_message)
        self.ur_power_btn.clicked.connect(self.toggle_power)
        self.ur_enable_btn.clicked.connect(self.toggle_enable)
        self.ur_init_btn.clicked.connect(self.send_ur_init_controller_command)
        self.ur_reset_btn.clicked.connect(self.send_ur_reset_command)
        self.ur_pause_btn.clicked.connect(self.send_ur_pause_command)
        self.ur_continue_btn.clicked.connect(self.send_ur_continue_command)
        self.ur_stop_btn.clicked.connect(self.send_ur_stop_command)
        self.teach_mode_checkbox.stateChanged.connect(self.handle_teach_mode_state_change)
        self.set_cur_tcp_btn.clicked.connect(self.send_set_tcp_command)
        self.read_cur_tcp_btn.clicked.connect(self.request_cur_tcp_info)
        self.read_tcp_o_btn.clicked.connect(self.read_tcp_o)
        self.set_tcp_o_btn.clicked.connect(self.set_tcp_o)
        self.set_tcp_p_btn.clicked.connect(self.set_tcp_p) # <--- 新增连接
        self.set_tcp_u_btn.clicked.connect(self.set_tcp_u) # <--- 新增连接
        self.set_tcp_e_btn.clicked.connect(self.set_tcp_e) # <--- 新增连接
        self.read_tcp_tip_btn.clicked.connect(self.read_tcp_tip)
        self.set_tcp_tip_btn.clicked.connect(self.set_tcp_tip)
        self.get_suitable_tcp_btn.clicked.connect(self.get_suitable_tcp)
        self.rotate_ultrasound_plane_to_b_btn.clicked.connect(self.rotate_ultrasound_plane_to_b)
        self.load_b_points_intcp_u_txt_btn.clicked.connect(self.read_b_points_in_tcp_u_from_file)
        self.get_a_btn.clicked.connect(self.get_a_point_position)
        self.get_o_btn.clicked.connect(self.get_o_point_position)
        self.get_e_btn.clicked.connect(self.get_e_point_position)
        self.align_planes_btn.clicked.connect(self.align_ultrasound_plane_to_aoe)
        self.init_joint_pos_btn.clicked.connect(self.send_init_joint_position_command)
        self.save_data_btn.clicked.connect(self.save_data)
        self.load_data_btn.clicked.connect(self.load_data)
        
        # 将 TCPManager 的信号连接到本窗口的槽函数
        self.tcp_manager.connection_status_changed.connect(self.update_ui_on_connection)
        self.tcp_manager.message_received.connect(self.handle_incoming_message)

    def connect_tcp(self):
        """将连接请求委托给 TCPManager。"""
        ip = self.ip_entry.text()
        try:
            port = int(self.port_entry.text())
            result = self.tcp_manager.connect(ip, port)
            
            # [修正] 如果连接成功，在发送 ReadActPos/ReadRobotState 等定时指令之后，立即发送读取 TCP_U 的命令
            if "连接成功" in result:
                command = "ReadTCPByName,0,TCP_U;"
                # 设置标志：期待下一个 ReadTCPByName 响应是 TCP_U 的定义
                self.temp_expected_response = "TCP_U_DEF" 
                self.tcp_manager.send_command(command)
                self.log_message(f"已自动发送命令: {command}")
            
            if "Fail" in result or "失败" in result:
                QMessageBox.critical(self, "Connection Error", result)
        except ValueError:
            QMessageBox.critical(self, "Input Error", "Port number must be a valid digit.")
            
    def disconnect_tcp(self):
        """将断开连接请求委托给 TCPManager。"""
        self.tcp_manager.disconnect()

    def send_message(self):
        """将发送用户手动输入消息的请求委托给 TCPManager。"""
        message = self.send_entry.text()
        if not message:
            return
        self.log_message(message)
        self.tcp_manager.send_command(message)
        self.send_entry.clear()
        
    def update_ui_on_connection(self, is_connected):
        """
        根据连接状态更新 UI 控件的可用性和文本。
        此槽函数由 tcp_manager.connection_status_changed 信号触发。
        """
        self.connect_button.setEnabled(not is_connected)
        self.disconnect_button.setEnabled(is_connected)
        self.send_entry.setEnabled(is_connected)
        self.send_button.setEnabled(is_connected)
        self.tcp_status_label.setText("TCP Status: Connected" if is_connected else "TCP Status: Disconnected")
        self.status_bar.showMessage("Status: Connected to Robot" if is_connected else "Status: Ready")
        if not is_connected:
            self.ur_stop_btn.setEnabled(False) # 断开连接时禁用急停按钮

    def handle_incoming_message(self, message):
        """
        解析接收到的消息，并根据消息类型分发到不同的处理方法。
        此槽函数由 tcp_manager.message_received 信号触发。
        """
        # self.log_message(message)
        if message.startswith("ReadActPos"):
            self.handle_real_time_message(message)
        elif message.startswith("ReadRobotState"):
            self.handle_robot_state_message(message)
        elif message.startswith("SetCurTCP"):
            self.log_message(message)
        elif message.startswith("ReadCurTCP"):
            self.log_message(message)
            self.handle_read_cur_tcp_message(message)
        elif message.startswith("ReadTCPByName"):
            self.log_message(message)
            # 1. 通用处理 (例如记录日志，但不解析数据到UI，因为不知道是哪个TCP)
            self.handle_read_tcp_byname_message(message)
            # 2. [修正] 特殊处理：检查是否是自动获取 TCP_U 定义的响应
            if self.temp_expected_response == "TCP_U_DEF":
                # 调用内部处理方法，它负责清除标记并解析数据
                self._handle_auto_tcp_u_definition_response(message)
        elif message.startswith("ReadEmergencyInfo"):
            self.handle_emergency_info_message(message)
        elif message.startswith("ReadOverride"):
            self.handle_override_message(message)
        elif message.startswith("WayPoint,OK"):
            self.log_message(message)
            self._continue_b_point_rotation() # <-- 调用分步继续函数
        elif message.startswith("WayPoint"):
            self.log_message(message)
        elif message.startswith("SetTCPByName"):
            self.log_message(message)
        elif message.startswith("MoveRelJ,OK"):
            # 如果是，通知 ultrasound_tab 继续下一步操作
            self.ultrasound_tab.continue_rotation()
        elif message.startswith("MoveRelJ"):
            # 说明指令未完成，开启报错
            self.log_message(message)

    def request_cur_tcp_info(self):
        """通过按钮点击发送指令，请求获取当前TCP坐标。"""
        self.log_message("ReadCurTCP,0;")
        self.tcp_manager.send_command("ReadCurTCP,0;")

    def send_set_tcp_command(self):
        """
        根据用户在输入框中输入的六个参数，格式化并发送设置TCP的指令。
        指令格式：ConfigTCP,nRbtID,sTcpName,dTcp_X,dTcp_Y,dTcp_Z,dTcp_Rx,dTcp_Ry,dTcp_Rz;
        """
        try:
            # 尝试将每个输入框的文本转换为浮点数。
            params = [float(entry.text()) for entry in self.tcp_input_entries]
            tcp_x, tcp_y, tcp_z, tcp_rx, tcp_ry, tcp_rz = params

            # 格式化指令字符串，并保留两位小数。
            # 使用新的 ConfigTCP 命令格式和 self.current_tcp_name
            sTcpName = self.current_tcp_name
            command = (
                f"ConfigTCP,0,{sTcpName},{tcp_x:.2f},{tcp_y:.2f},{tcp_z:.2f},"
                f"{tcp_rx:.2f},{tcp_ry:.2f},{tcp_rz:.2f};"
            )
            
            self.log_message(command)
            self.tcp_manager.send_command(command)
            self.status_bar.showMessage(f"Status: TCP parameters sent to {sTcpName}.")
        except ValueError:
            QMessageBox.critical(self, "Input Error", "TCP parameters must be valid numbers!")
        except IndexError:
            QMessageBox.critical(self, "Internal Error", "Incorrect number of TCP parameter input boxes.")

    def toggle_power(self):
        """根据当前电源状态切换按钮文本，并发送相应的上电或断电指令。"""
        if self.power_state == 0:
            # 未上电，发送上电指令。
            self.tcp_manager.send_command("Electrify;")
            self.ur_power_btn.setText("Powering On...")
            self.ur_power_btn.setEnabled(False) # 暂时禁用按钮，避免重复点击。
        else:
            # 已上电，发送断电指令 (OSCmd,1)。
            self.tcp_manager.send_command("OSCmd,1;")
            self.ur_power_btn.setText("Powering Off...")
            self.ur_power_btn.setEnabled(False)

    def toggle_enable(self):
        """根据当前使能状态切换按钮文本，并发送使能或去使能指令。"""
        if self.enable_state == 0:
            # 未使能，发送使能指令 (GrpEnable,0)。
            self.tcp_manager.send_command("GrpEnable,0;")
            self.ur_enable_btn.setText("Enabling...")
            self.ur_enable_btn.setEnabled(False)
        else:
            # 已使能，发送去使能指令 (GrpDisable,0)。
            self.tcp_manager.send_command("GrpDisable,0;")
            self.ur_enable_btn.setText("Disabling...")
            self.ur_enable_btn.setEnabled(False)

    def handle_teach_mode_state_change(self, state):
        """处理示教功能复选框的状态变化，并发送相应的TCP指令。"""
        if state == Qt.Checked:
            # 如果勾选，发送开启示教功能指令 (GrpOpenFreeDriver,0)。
            command = "GrpOpenFreeDriver,0;"
            self.tcp_manager.send_command(command)
            self.status_bar.showMessage("Status: Teach Mode Enabled.")
        else:
            # 如果取消勾选，发送关闭示教功能指令 (GrpCloseFreeDriver,0)。
            command = "GrpCloseFreeDriver,0;"
            self.tcp_manager.send_command(command)
            self.status_bar.showMessage("Status: Teach Mode Disabled.")

    def _on_override_slider_changed(self, value):
        """当滑条数值改变时，更新标签以显示当前的运动速率值。"""
        override_value = value / 100.0
        self.override_label.setText(f"Motion Speed: {override_value:.2f}")

    def _on_override_slider_released(self):
        """当用户释放滑条时，发送 SetOverride 指令到机器人。"""
        d_override = self.override_slider.value() / 100.0
        # 指令格式为: `SetOverride,nRbtID,dOverride;`
        command = f"SetOverride,0,{d_override:.2f};"
        self.tcp_manager.send_command(command)
        self.status_bar.showMessage(f"Status: Motion speed set to {d_override:.2f}")

    def handle_override_message(self, message):
        """解析 'ReadOverride' 消息，并更新当前运动速率显示。"""
        parts = message.strip(';').strip(',').split(',')
        if len(parts) == 3 and parts[1] == 'OK':
            try:
                dOverride = float(parts[2])
                self.current_override_value.setText(f"{dOverride:.2f}")
            except (ValueError, IndexError) as e:
                self.log_message(f"Warning: Cannot parse motion speed info: {message}, Error: {e}")
        else:
            self.log_message(f"Warning: Received invalid ReadOverride message: {message}")

    def handle_emergency_info_message(self, message):
        """解析 'ReadEmergencyInfo' 消息，并根据nESTO值更新急停按钮状态。"""
        parts = message.strip(';').strip(',').split(',')
        if len(parts) == 6 and parts[1] == 'OK':
            try:
                nESTO = int(parts[3])
                # 如果nESTO为0，说明未处于急停状态，急停按钮可用
                self.ur_stop_btn.setEnabled(nESTO == 0)
            except (ValueError, IndexError) as e:
                self.log_message(f"Warning: Cannot parse emergency stop info message: {message}, Error: {e}")
        else:
            self.log_message(f"Warning: Received invalid ReadEmergencyInfo message: {message}")

    def handle_real_time_message(self, message):
        """解析 'ReadActPos' 消息，并更新UI上的关节和末端坐标显示。"""
        # 移除消息末尾的分号并按逗号分割。
        parts = message.strip(';').strip(',').split(',')
        if len(parts) == 26 and parts[1] == 'OK':
            try:
                joint_params = [float(p) for p in parts[2:8]]
                for i in range(len(joint_params)):
                    if i < len(self.joint_vars):
                        # 直接使用解析出的值，不再进行转换
                        self.joint_vars[i].setText(f"{joint_params[i]:.2f}")

                # 更新机器人工具端姿态。
                tool_pose_params = [float(p) for p in parts[8:14]]
                self.tool_pose_labels["Tcp_X"].setText(f"{tool_pose_params[0]:.2f}")
                self.tool_pose_labels["Tcp_Y"].setText(f"{tool_pose_params[1]:.2f}")
                self.tool_pose_labels["Tcp_Z"].setText(f"{tool_pose_params[2]:.2f}")
                self.tool_pose_labels["Tcp_Rx"].setText(f"{tool_pose_params[3]:.2f}")
                self.tool_pose_labels["Tcp_Ry"].setText(f"{tool_pose_params[4]:.2f}")
                self.tool_pose_labels["Tcp_Rz"].setText(f"{tool_pose_params[5]:.2f}")
                # 更新最新工具端姿态
                self.latest_tool_pose = tool_pose_params

                self.status_bar.showMessage("Status: Joint, TCP, and Tool coordinates synchronized in real time.")
            except (ValueError, IndexError):
                self.log_message("Warning: Cannot parse ReadActPos message.")
        else:
            self.log_message(f"Warning: Received invalid ReadActPos message: {message}")

    def get_a_point_position(self):
        """
        [修改]：开始执行切换到 TCP_tip -> 延时 300ms -> 获取 A 点位置 的序列。
        """
        if not self.tcp_manager.is_connected:
            QMessageBox.warning(self, "Warning", "Robot is disconnected.")
            return
            
        self.status_bar.showMessage("Status: Starting 'Get A Point' sequence (Step 1/3: Switch to TCP_tip)...")
        
        # Step 1: 切换到 TCP_tip
        self.set_tcp_tip()
        
        # Step 2: 延时 300ms 后，执行核心逻辑
        QTimer.singleShot(300, self._finalize_get_a_point_position)

    def _finalize_get_a_point_position(self):
        """
        [新增]：序列 Step 3/3: 执行原有的核心逻辑，获取最新的姿态。
        """
        self.status_bar.showMessage("Status: Sequence Step 3/3: Get A Point Position...")

        if not self.latest_tool_pose:
            self.status_bar.showMessage("Warning: Cannot get position, please connect robot and wait for data update first.")
            return
        
        # Core Logic
        self.a_vars[0].setText(f"{self.latest_tool_pose[0]:.2f}")
        self.a_vars[1].setText(f"{self.latest_tool_pose[1]:.2f}")
        self.a_vars[2].setText(f"{self.latest_tool_pose[2]:.2f}")
        self.status_bar.showMessage("Status: A point position obtained.")

    def get_o_point_position(self):
        """
        [修改]：开始执行切换到 TCP_tip -> 延时 300ms -> 获取 O 点位置 的序列。
        """
        if not self.tcp_manager.is_connected:
            QMessageBox.warning(self, "Warning", "Robot is disconnected.")
            return
            
        self.status_bar.showMessage("Status: Starting 'Get O Point' sequence (Step 1/3: Switch to TCP_tip)...")
        
        # Step 1: 切换到 TCP_tip
        self.set_tcp_tip()
        
        # Step 2: 延时 300ms 后，执行核心逻辑
        QTimer.singleShot(300, self._finalize_get_o_point_position)

    def _finalize_get_o_point_position(self):
        """
        [新增]：序列 Step 3/3: 执行原有的核心逻辑，获取最新的姿态。
        """
        self.status_bar.showMessage("Status: Sequence Step 3/3: Get O Point Position...")

        if not self.latest_tool_pose:
            self.status_bar.showMessage("Warning: Cannot get position, please connect robot and wait for data update first.")
            return
        
        # Core Logic
        self.o_vars[0].setText(f"{self.latest_tool_pose[0]:.2f}")
        self.o_vars[1].setText(f"{self.latest_tool_pose[1]:.2f}")
        self.o_vars[2].setText(f"{self.latest_tool_pose[2]:.2f}")
        self.status_bar.showMessage("Status: O point position obtained.")
        
    def get_e_point_position(self):
        """将最新的工具端位置记录到End-Effect文本框中。"""
        if not self.latest_tool_pose:
            self.status_bar.showMessage("Warning: Cannot get position, please connect robot and wait for data update first.")
            return
        e_point = self.latest_tool_pose[0:3]
        self.e_vars[0].setText(f"{e_point[0]:.2f}")
        self.e_vars[1].setText(f"{e_point[1]:.2f}")
        self.e_vars[2].setText(f"{e_point[2]:.2f}")
        self.status_bar.showMessage("Status: End-Effect position obtained.")
        
    def get_current_tool_pose(self):
        """从UI中获取当前的工具姿态（位置和欧拉角）。"""
        try:
            x = float(self.latest_tool_pose[0])
            y = float(self.latest_tool_pose[1])
            z = float(self.latest_tool_pose[2])
            rx = float(self.latest_tool_pose[3])
            ry = float(self.latest_tool_pose[4])
            rz = float(self.latest_tool_pose[5])
            return np.array([x, y, z, rx, ry, rz])
        except (ValueError, IndexError):
            return None
        
    def align_ultrasound_plane_to_aoe(self):
        """
        [修改]：开始执行新的对齐序列：切换到 TCP_E，获取 End-Effect 位置，然后执行计算和旋转。
        """
        # 1. 检查连接状态 (可选但推荐)
        if not self.tcp_manager.is_connected:
            QMessageBox.warning(self, "Warning", "Robot is disconnected.")
            return

        self.status_bar.showMessage("Status: Starting 'Align Ultrasound Plane to AOE Plane' sequence (Step 0/3: Switch to TCP_E)...")
        
        # Step 0: 切换到 TCP_E (Sequence Start)
        self.set_tcp_e()
        
        # Step 1: 延时 200ms 后，获取 End-Effect 位置
        QTimer.singleShot(200, self._continue_align_ultrasound_plane_to_aoe)


    def _continue_align_ultrasound_plane_to_aoe(self):
        """
        [序列 Step 1]：获取 End-Effect 位置。
        """
        self.status_bar.showMessage("Status: Sequence Step 1/3: Get End-Effect Position...")

        # Step 1: 获取 End-Effect 位置
        self.get_e_point_position()
        
        # Step 2: 延时 200ms 后，执行核心计算和旋转逻辑
        QTimer.singleShot(200, self._finalize_align_ultrasound_plane_to_aoe)


    def _finalize_align_ultrasound_plane_to_aoe(self):
        """
        [序列 Step 2/3]：执行原有的核心计算和关节旋转指令。
        """
        self.status_bar.showMessage("Status: Sequence Step 2/3: Calculate and send joint rotation command...")

        # 将原有的 align_ultrasound_plane_to_aoe 核心逻辑放在这里
        try:
            a_point = np.array([float(self.a_vars[i].text()) for i in range(3)])
            o_point = np.array([float(self.o_vars[i].text()) for i in range(3)])
            e_point = np.array([float(self.e_vars[i].text()) for i in range(3)])
        except ValueError:
            self.status_bar.showMessage("Error: A, O, E point coordinates must be valid numbers.")
            QMessageBox.critical(self, "Error", "A, O, E point coordinates must be valid numbers.")
            return
        
        initial_tcp_pose = self.get_current_tool_pose()
        if initial_tcp_pose is None:
            self.status_bar.showMessage("Warning: Cannot get current robot pose.")
            return

        # 调用函数，计算关节6需要旋转的角度（带正负号）
        initial_rpy_deg = initial_tcp_pose[3:]
        result = calculate_rotation_for_plane_alignment(a_point, o_point, e_point, initial_rpy_deg)

        # Check if the result is an error message
        if isinstance(result, tuple):
            self.status_bar.showMessage(f"Error: {result[1]}")
            QMessageBox.warning(self, "Error", result[1])
            return
        
        rotation_angle_deg = result # Assign the float value if no error

        # 根据角度的正负来确定 nDirection 参数
        if rotation_angle_deg >= 0:
            nDirection = FORWARD  # 正向
        else:
            nDirection = BACKWARD # 反向

        # dDeltaVal 总是正值
        dDeltaVal = abs(rotation_angle_deg)

        # 构造 MoveRelJ 运动指令
        # 关节6的 nAxisID 为 5
        # 指令格式: MoveRelJ,nRbtID,nAxisID,nDirection,dDeltaVal;
        command = f"MoveRelJ,0,5,{nDirection},{dDeltaVal:.2f};"
        
        # 发送指令
        self.tcp_manager.send_command(command)
        self.status_bar.showMessage(f"Status: Sequence Step 3/3 Completed. Sent command to align ultrasound plane with AOE plane, Joint 6 rotated {rotation_angle_deg:.2f} degrees.")
        QMessageBox.information(self, "Sequence Completed", f"Rotation command sent, Joint 6 rotated {rotation_angle_deg:.2f} degrees.")

    def rotate_ultrasound_plane_to_b(self):
        """
        [修改]：开始执行新的对齐序列：先切换到 TCP_E，延时 200ms 后执行 B 点旋转的设置和启动。
        """
        # 1. 检查连接状态
        if not self.tcp_manager.is_connected:
            QMessageBox.warning(self, "Warning", "Robot is disconnected.")
            return
            
        self.status_bar.showMessage("Status: Starting 'Rotate Ultrasound Plane around OA to pass through B Point' sequence (Step 0/2: Switch to TCP_E)...")
        
        # Step 0: 切换到 TCP_E
        self.set_tcp_e()
        
        # Step 1: 延时 200ms 后，执行核心启动逻辑
        QTimer.singleShot(200, self._start_b_point_rotation_sequence)

    def _start_b_point_rotation_sequence(self):
        """
        [序列 Step 1/2]：执行原有的 B 点旋转计算，设置序列状态，并发送第一个 WayPoint 指令。
        """
        self.status_bar.showMessage("Status: Sequence Step 1/2: Calculate rotation steps and send first WayPoint...")
        
        try:
            a_point = np.array([float(self.a_vars[i].text()) for i in range(3)])
            o_point = np.array([float(self.o_vars[i].text()) for i in range(3)])
            
            # --- Input validation and B-point retrieval ---
            if np.all(self.b_point_position_in_base == 0):
                QMessageBox.warning(self, "Operation Failed", "Please enter or select a valid B point position first.")
                return
            b_point = self.b_point_position_in_base
            
            initial_tcp_pose = self.get_current_tool_pose()
            if initial_tcp_pose is None:
                self.status_bar.showMessage("Warning: Cannot get current robot pose.")
                return
            
            # 0. 重置并计算所有步骤的旋转矩阵
            self.b_point_rotation_steps = []
            self.current_b_point_step = -1
            self.initial_tcp_pose_for_b_rot = None
            self.b_point_o_point = None
            
            # 1. 调用函数，它返回一个 delta_rotation_matrix 列表
            delta_rotation_matrices = calculate_new_rpy_for_b_point(
                a_point, o_point, b_point, initial_tcp_pose[3:]
            )
            
            if not delta_rotation_matrices or (len(delta_rotation_matrices) == 1 and np.allclose(delta_rotation_matrices[0], np.identity(4))):
                 self.status_bar.showMessage("Warning: Rotation angle is close to 0 or calculation failed, no rotation needed.")
                 QMessageBox.information(self, "Completed", "Rotation angle is close to 0 or calculation failed, no rotation needed.")
                 return
                 
            # 2. 存储状态
            self.b_point_rotation_steps = delta_rotation_matrices
            self.initial_tcp_pose_for_b_rot = initial_tcp_pose
            self.b_point_o_point = o_point
            
            # 3. 启动第一个步骤 (通过将 step 设为 -1 并在 _continue 中递增到 0)
            self.current_b_point_step = -1
            
            # 4. 调用迭代器来发送第一个指令
            self._continue_b_point_rotation() 
            
            # 5. 更新 UI 状态
            self.status_bar.showMessage("Status: Sequence Step 2/2 Completed. Started sending WayPoint commands in steps to make the ultrasound plane contain B point.")
            QMessageBox.information(self, "Task Started", f"Started {len(delta_rotation_matrices)} step rotation task.")
            
        except ValueError as e:
            self.status_bar.showMessage(f"Error: Rotation calculation failed: {e}")
            self.log_message(f"CALCULATION ERROR (ValueError): {e}") 
            QMessageBox.critical(self, "Calculation Error", f"Rotation calculation failed: {e}")
        except Exception as e:
             self.status_bar.showMessage(f"Fatal Error: Pose calculation failed: {e}")
             self.log_message(f"FATAL ERROR: Calculation failed: {e}")
             QMessageBox.critical(self, "Fatal Error", f"Pose calculation failed: {e}")
             
    # I must assume this method exists and is the main iterator/handler
    def _continue_b_point_rotation(self):
        """
        当接收到 WayPoint 成功响应后，发送下一个旋转步骤的 WayPoint 指令。
        [修改]: 延迟增加到 400ms。
        """
        # 如果不在 B 点旋转模式，则忽略
        if not self.b_point_rotation_steps:
            return

        # 检查是否完成所有步骤
        self.current_b_point_step += 1
        
        if self.current_b_point_step < len(self.b_point_rotation_steps):
            # 1. 获取当前步骤的累积旋转矩阵
            delta_rotation_matrix = self.b_point_rotation_steps[self.current_b_point_step]
            
            # 2. 从初始姿态计算目标姿态
            initial_tcp_pose = self.initial_tcp_pose_for_b_rot
            o_point = self.b_point_o_point
            
            # 3. 计算最终位置 P_final 
            P_final = get_final_tcp_e_position_after_delta_rotation(
                initial_tcp_pose[:3], 
                delta_rotation_matrix, 
                o_point
            )
            
            # 4. 计算最终旋转矩阵 R_final 和 RPY
            initial_rpy_rad = np.deg2rad(initial_tcp_pose[3:])
            initial_rotation_matrix = pyrot.matrix_from_euler(initial_rpy_rad, 0, 1, 2, extrinsic=True)
            R_final = np.dot(delta_rotation_matrix[:3,:3], initial_rotation_matrix)
            final_rpy_rad = pyrot.euler_from_matrix(R_final, 0, 1, 2, extrinsic=True)
            final_rpy_deg = np.rad2deg(final_rpy_rad)
                                    
            # 5. 构造 WayPoint 命令
            dX_dY_dZ = P_final
            dRx_dRy_dRz = final_rpy_deg
            pos_str = ",".join([f"{val:.2f}" for val in dX_dY_dZ])
            rpy_str = ",".join([f"{val:.2f}" for val in dRx_dRy_dRz])
            pos_rpy_str = f"{pos_str},{rpy_str}"
            dJ_zero = ",".join(["0.00"] * 6)
            nRbtID = 0
            sTcpName = "TCP"
            sUcsName = "Base"
            dVelocity = 50
            dAcc = 250
            dRadius = 0 
            nMoveType = 0 
            nIsUseJoint = 0 
            nIsSeek = 0
            nIOBit = 0
            nIOState = 0
            strCmdID = "ID1"
            
            command = (
                f"WayPoint,{nRbtID},{pos_rpy_str},{dJ_zero},"
                f"{sTcpName},{sUcsName},{dVelocity},{dAcc},{dRadius},{nMoveType},"
                f"{nIsUseJoint},{nIsSeek},{nIOBit},{nIOState},{strCmdID};"
            )
            
            # 6. [修改] 发送指令前，加入 400ms 延时
            delay_ms = 400 # <-- NEW DELAY
            QTimer.singleShot(delay_ms, lambda: self.tcp_manager.send_command(command))

            # 提取累积旋转角度用于状态栏显示
            step_angle = np.rad2deg(pyrot.axis_angle_from_matrix(delta_rotation_matrix[:3, :3])[3])
            self.status_bar.showMessage(f"Status: Rotate Ultrasound Plane around OA (Step {self.current_b_point_step+1}/{len(self.b_point_rotation_steps)}, cumulative rotation {step_angle:.2f} degrees)...")
            
        else:
            # 7. 完成所有步骤
            self.b_point_rotation_steps = []
            self.current_b_point_step = -1
            self.initial_tcp_pose_for_b_rot = None
            self.b_point_o_point = None
            self.status_bar.showMessage("Status: Rotation of Ultrasound Plane around OA to B Point task completed.")
            QMessageBox.information(self, "Task Completed", "Rotation of Ultrasound Plane around OA to B Point task completed.")
        
    def handle_read_tcp_byname_message(self, message):
        """解析 'ReadTCPByName' 消息，并更新新的TCP显示框。"""
        parts = message.strip(';').strip(',').split(',')
        if len(parts) == 8 and parts[1] == 'OK':
            try:
                tcp_params = [f"{float(p):.2f}" for p in parts[2:]]
                for i in range(len(tcp_params)):
                    self.cur_tcp_vars[i].setText(tcp_params[i])
                self.status_bar.showMessage("Status: Current TCP coordinates updated.")
            except (ValueError, IndexError):
                self.log_message("Warning: Cannot parse ReadTCPByName message.")
        else:
            self.log_message(f"Warning: Received invalid ReadTCPByName message: {message}")
            
    def handle_read_cur_tcp_message(self, message):
        """解析 'ReadCurTCP' 消息，并更新新的TCP显示框。"""
        parts = message.strip(';').strip(',').split(',')
        if len(parts) == 8 and parts[1] == 'OK':
            try:
                tcp_params = [f"{float(p):.2f}" for p in parts[2:]]
                for i in range(len(tcp_params)):
                    self.cur_tcp_vars[i].setText(tcp_params[i])
                self.status_bar.showMessage("Status: Current TCP coordinates updated.")
            except (ValueError, IndexError):
                self.log_message("Warning: Cannot parse ReadCurTCP message.")
        else:
            self.log_message(f"Warning: Received invalid ReadCurTCP message: {message}")
            
    def handle_robot_state_message(self, message):
        """解析 'ReadRobotState' 消息，并更新UI上的电源和使能按钮状态。"""
        parts = message.strip(';').strip(',').split(',')
        if len(parts) == 15 and parts[1] == 'OK':
            try:
                # 协议中定义了每个状态参数的索引。
                nElectrify = int(parts[11])
                nEnableState = int(parts[3])
                self.update_power_button(nElectrify)
                self.update_enable_button(nEnableState)
            except (ValueError, IndexError):
                self.log_message("Warning: Cannot parse power/enable status in robot state message.")
        else:
            self.log_message(f"Warning: Received invalid ReadRobotState message: {message}")

    def update_power_button(self, nElectrify):
        """根据上电状态更新电源按钮的文本和颜色。"""
        self.power_state = nElectrify
        self.ur_power_btn.setEnabled(True)  # 重新启用按钮。
        if nElectrify == 1:
            self.ur_power_btn.setText("Power Off")
            self.ur_power_btn.setStyleSheet("background-color: lightgreen;")
        else:
            self.ur_power_btn.setText("Power On")
            self.ur_power_btn.setStyleSheet("background-color: salmon;")

    def update_enable_button(self, nEnableState):
        """根据使能状态更新使能按钮的文本和颜色。"""
        self.enable_state = nEnableState
        self.ur_enable_btn.setEnabled(True)
        if nEnableState == 1:
            self.ur_enable_btn.setText("Disable")
            self.ur_enable_btn.setStyleSheet("background-color: lightgreen;")
        else:
            self.ur_enable_btn.setText("Enable")
            self.ur_enable_btn.setStyleSheet("background-color: salmon;")

    def send_ur_init_controller_command(self):
        """发送初始化控制器指令。"""
        self.tcp_manager.send_command("StartMaster;")
        self.status_bar.showMessage("Status: Initialized Controller command sent.")

    def send_ur_reset_command(self):
        """发送机器人控制器复位指令。"""
        command = "GrpReset,0;" 
        self.tcp_manager.send_command(command)
        self.status_bar.showMessage("Status: Reset command sent.")

    def send_ur_pause_command(self):
        """发送机器人运动暂停指令。"""
        command = "GrpInterrupt,0;"
        self.tcp_manager.send_command(command)
        self.status_bar.showMessage("Status: Pause command sent.")

    def send_ur_continue_command(self):
        """发送机器人运动继续指令。"""
        command = "GrpContinue,0;"
        self.tcp_manager.send_command(command)
        self.status_bar.showMessage("Status: Continue command sent.")

    def send_ur_stop_command(self):
        """发送机器人急停指令。"""
        command = "GrpStop,0;" 
        self.tcp_manager.send_command(command)
        self.status_bar.showMessage("Status: Emergency Stop command sent.")

    def log_message(self, message):
        """将消息添加到接收文本框，并自动滚动到底部。"""
        self.recv_text.append(message)
        self.recv_text.verticalScrollBar().setValue(self.recv_text.verticalScrollBar().maximum())

    def motor_adjust(self, joint_index, direction):
        """发送 MoveRelJ 指令，实现单个关节的增量运动。"""
        # MoveRelJ 指令格式: MoveRelJ,nRbtID,nAxisID,nDirection,dDeltaVal;
        # dDeltaVal 是以度为单位的增量值。
        command = f"MoveRelJ,0,{joint_index},{direction},1;"
        self.tcp_manager.send_command(command)
        self.status_bar.showMessage(f"Status: Joint {joint_index+1} fine-tuning...")
    
    def _start_joint_continuous_mode(self):
        """延迟2秒后触发，启动关节的连续循环微调模式。"""
        self._is_continuous_mode_active = True
        # 立即发送第一个连续指令，然后启动循环定时器
        self.motor_adjust(self._moving_joint_index, self._moving_direction)
        self.continuous_move_timer.start(300) # 每300毫秒调用一次 continuous_move。

    def start_move(self, joint_index, direction):
        """开始微调关节运动：设置参数，并启动2秒延迟定时器。"""
        self._moving_joint_index = joint_index
        self._moving_direction = direction
        self._is_continuous_mode_active = False # 假定非连续模式
        self.start_continuous_move_timer.start(2000) # 2秒后触发连续模式

    def stop_move(self):
        """停止微调关节运动：停止定时器，并检查是否需要执行单步运动。"""
        # 停止所有相关定时器
        self.start_continuous_move_timer.stop()
        self.continuous_move_timer.stop()
        
        # 如果2秒内释放按钮（即未进入连续模式），则执行单次运动
        # 此时 _is_continuous_mode_active 仍为 False
        if not self._is_continuous_mode_active and self._moving_joint_index != -1:
            self.motor_adjust(self._moving_joint_index, self._moving_direction)

        self._moving_joint_index = -1
        self._moving_direction = BACKWARD
        self._is_continuous_mode_active = False
        self.status_bar.showMessage("Status: Joint fine-tuning stopped")

    def continuous_move(self):
        """由定时器周期性调用的函数，用于持续微调关节。"""
        if self._moving_joint_index != -1:
            self.motor_adjust(self._moving_joint_index, self._moving_direction)

    def tcp_adjust(self, tcp_index, direction):
        """
        发送 MoveRelL 指令，实现TCP坐标的增量运动。
        MoveRelL 指令格式: MoveRelL,nRbtID,nAxisId,nDirection,dDistance,nToolMotion;
        dDistance 是增量值（mm或度）。
        """
        # nRbtID 设为 0
        nRobotID = 0
        # ***** 关键修改: 根据 Tool 坐标系复选框状态设置 nToolMotion *****
        if self.tool_coord_checkbox and self.tool_coord_checkbox.isChecked():
            nToolMotion = 1 # 在 Tool 坐标系下运动
        else:
            nToolMotion = 0 # 在 Base 坐标系下运动
        
        # nDirection 的值根据传入的 direction 参数确定
        nDirection = direction # direction = 1 为正向，0 为反向
        
        # dDistance 的值根据微调类型确定
        if tcp_index < 3: # X, Y, Z
            dDistance = 1.0 # 1mm的增量
        else: # Rx, Ry, Rz
            dDistance = 1.0 # 1度的增量

        # nAxisId 的值根据 tcp_index 确定
        nAxisId = tcp_index
        
        command = f"MoveRelL,{nRobotID},{nAxisId},{nDirection},{dDistance:.2f},{nToolMotion};"
        self.tcp_manager.send_command(command)
        
        # 更新状态栏显示，这里可能需要根据 nAxisId 重新映射标签
        axis_labels = ["X", "Y", "Z", "Rx", "Ry", "Rz"]
        coord_sys = "Tool" if nToolMotion == 1 else "Base"
        self.status_bar.showMessage(f"Status: Tcp_{axis_labels[nAxisId]} fine-tuning in {coord_sys} coordinate system...")

    def _start_tcp_continuous_mode(self):
        """延迟2秒后触发，启动TCP的连续循环微调模式。"""
        self._is_continuous_tcp_mode_active = True
        # 立即发送第一个连续指令，然后启动循环定时器
        self.tcp_adjust(self._moving_tcp_index, self._moving_direction)
        self.continuous_tcp_move_timer.start(300) # 每300毫秒调用一次 continuous_tcp_move。

    def start_tcp_move(self, tcp_index, direction):
        """开始微调TCP坐标运动：设置参数，并启动2秒延迟定时器。"""
        self._moving_tcp_index = tcp_index
        self._moving_direction = direction
        self._is_continuous_tcp_mode_active = False # 假定非连续模式
        self.start_continuous_tcp_move_timer.start(2000) # 2秒后触发连续模式

    def stop_tcp_move(self):
        """停止微调TCP坐标运动：停止定时器，并检查是否需要执行单步运动。"""
        # 停止所有相关定时器
        self.start_continuous_tcp_move_timer.stop()
        self.continuous_tcp_move_timer.stop()
        
        # 如果2秒内释放按钮（即未进入连续模式），则执行单次运动
        # 此时 _is_continuous_tcp_mode_active 仍为 False
        if not self._is_continuous_tcp_mode_active and self._moving_tcp_index != -1:
            self.tcp_adjust(self._moving_tcp_index, self._moving_direction)

        self._moving_tcp_index = -1
        self._moving_direction = BACKWARD
        self._is_continuous_tcp_mode_active = False
        self.status_bar.showMessage("Status: TCP fine-tuning stopped")
    
    def send_init_joint_position_command(self):
        """发送指令，使机器人移动到预设的初始关节位置。"""
        # 预设的关节值
        J_VALS = [-213.88, -117.11, -73.94, -264.29, -94.03, 219.22]
        # 格式化关节值字符串，保留两位小数
        J_STR = ",".join([f"{j:.2f}" for j in J_VALS])
        
        # 固定参数：nRbtID=0, dX,dY,dZ,dRx,dRy,dRz 都为 0
        RBT_ID = 0
        POS_RPY_ZERO = ",".join(["0.00"] * 6)
        
        sTcpName = "TCP"
        sUcsName = "Base"
        dVelocity = 50
        dAcc = 360
        dRadius = 0
        nMoveType = 0
        nIsUseJoint = 1  # 关键：使用关节值模式
        nIsSeek = 0
        nIOBit = 0
        nIOState = 0
        strCmdID = "ID1"
        
        # 拼接 WayPoint 命令
        command = (
            f"WayPoint,{RBT_ID},{POS_RPY_ZERO},{J_STR},"
            f"{sTcpName},{sUcsName},{dVelocity},{dAcc},{dRadius},{nMoveType},"
            f"{nIsUseJoint},{nIsSeek},{nIOBit},{nIOState},{strCmdID};"
        )

        self.tcp_manager.send_command(command)
        self.status_bar.showMessage("Status: Initialized Joint Position command sent.")

    def continuous_tcp_move(self):
        """由定时器周期性调用的函数，用于持续微调TCP坐标。"""
        if self._moving_tcp_index != -1:
            self.tcp_adjust(self._moving_tcp_index, self._moving_direction)

    def closeEvent(self, a0: QCloseEvent):
        """重写窗口关闭事件，确保在程序退出前断开TCP连接并清理资源。"""
        self.tcp_manager.disconnect()
        self.ultrasound_tab.cleanup()
        # ***** 新增：调用 BeckhoffTab 的清理函数 *****
        self.beckhoff_tab.cleanup() 
        super().closeEvent(a0)

    def create_motor_group(self, layout):
        """
        创建左侧的电机微调模块，用于控制各个关节的运动。
        此方法已修改，以调整运动速率滑条的布局，使其与标签更靠近。
        """
        group = QGroupBox("Motor Fine-tuning Module (Forward Kinematics)")
        group_layout = QVBoxLayout(group)
        group_layout.addStretch()
        for i in range(self.num_joints):
            row_layout = QHBoxLayout()
            row_layout.addStretch()
            label = QLabel(f"Joint {i+1} (q{i+1}):")
            value_label = QLineEdit("0.00")
            value_label.setReadOnly(True)
            value_label.setStyleSheet("background-color: lightgrey; border: 1px inset grey;")
            value_label.setFixedWidth(70)
            value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter) # 右对齐
            self.joint_vars[i] = value_label  # 存储引用，以便后续更新显示。
            # 创建并连接“微调减小”按钮。
            btn_minus = QPushButton("-")
            btn_minus.setFixedWidth(30)
            # 使用 lambda 表达式来传递参数 (关节索引和方向)。
            # pressed 信号在按钮被按下时立即触发。
            btn_minus.pressed.connect(lambda j=i: self.start_move(j, BACKWARD))
            # released 信号在按钮被释放时触发。
            btn_minus.released.connect(self.stop_move)
            # 创建并连接“微调增加”按钮。
            btn_plus = QPushButton("+")
            btn_plus.setFixedWidth(30)
            btn_plus.pressed.connect(lambda j=i: self.start_move(j, FORWARD))
            btn_plus.released.connect(self.stop_move)
            row_layout.addWidget(label)
            row_layout.addWidget(value_label)
            row_layout.addSpacing(10)
            row_layout.addStretch()
            row_layout.addWidget(btn_minus)
            row_layout.addWidget(btn_plus)
            row_layout.addStretch()
            group_layout.addLayout(row_layout)
        group_layout.addStretch()
        # --- 新增：初始化关节位置按钮 ---
        init_btn_layout = QHBoxLayout()
        self.init_joint_pos_btn = QPushButton("Initialize Joint Position")
        init_btn_layout.addStretch()
        init_btn_layout.addWidget(self.init_joint_pos_btn)
        init_btn_layout.addStretch()
        group_layout.addLayout(init_btn_layout)

        group_layout.addStretch()
        layout.addWidget(group)

    def create_override_group(self, layout):
        """将运动速率滑块和显示做成一个新Groupbox。"""
        group = QGroupBox("Motion Speed")
        group_layout = QVBoxLayout(group)
        self.override_label = QLabel("Motion Speed: 0.01")
        self.override_label.setAlignment(Qt.AlignCenter)
        center_layout = QVBoxLayout()
        self.override_slider = QSlider(Qt.Vertical)
        self.override_slider.setMinimum(1)
        self.override_slider.setMaximum(100)
        self.override_slider.setValue(1)
        self.override_slider.setTickPosition(QSlider.TicksBothSides)
        self.override_slider.setTickInterval(10)
        self.override_slider.valueChanged.connect(self._on_override_slider_changed)
        self.override_slider.sliderReleased.connect(self._on_override_slider_released)
        center_layout.addWidget(self.override_slider, alignment=Qt.AlignCenter)
        current_override_read_layout = QVBoxLayout()
        self.current_override_label = QLabel("Current Motion Speed:")
        self.current_override_label.setAlignment(Qt.AlignCenter)
        self.current_override_value = QLineEdit("0.00")
        self.current_override_value.setReadOnly(True)
        self.current_override_value.setFixedWidth(60)
        self.current_override_value.setAlignment(Qt.AlignCenter)
        self.current_override_value.setStyleSheet("background-color: lightgrey;")
        current_override_read_layout.addWidget(self.current_override_label, alignment=Qt.AlignCenter)
        current_override_read_layout.addWidget(self.current_override_value, alignment=Qt.AlignCenter)
        group_layout.addStretch()
        group_layout.addWidget(self.override_label, alignment=Qt.AlignCenter)
        group_layout.addLayout(center_layout)
        group_layout.addLayout(current_override_read_layout)
         # 移除垂直拉伸因子，让group不再被拉伸
        group_layout.addStretch()
        layout.addWidget(group)

    def create_tool_state_group(self, layout):
        """创建用于显示机器人工具端实时状态并包含微调按钮的模块。（已修改为只读）"""
        group = QGroupBox("Robot Tool Real-time Status")
        group_layout = QGridLayout(group)
        self.tool_pose_labels = {}
        # 定义工具端姿态参数。格式: (基础标签文本, 变量键, 附加标签文本/None)
        tool_pose_info = [
            ("Tcp_X", "Tcp_X", None), 
            ("Tcp_Y", "Tcp_Y", None), 
            ("Tcp_Z", "Tcp_Z", None),
            ("Tcp_Rx", "Tcp_Rx", "Roll"), 
            ("Tcp_Ry", "Tcp_Ry", "Pitch"), 
            ("Tcp_Rz", "Tcp_Rz", "Yaw")
        ]
        
        # Row 索引: 0 (X, Y, Z标签/值), 1 (X, Y, Z按钮), 2 (Rx, Ry, Rz标签/值), 3 (Rx, Ry, Rz按钮)
        for i, (base_label_text, var_key, additional_label_text) in enumerate(tool_pose_info):
            
            # 确定行索引
            if i < 3: # X, Y, Z (位置参数)
                row_value = 0
                row_buttons = 1
            else: # Rx, Ry, Rz (姿态参数)
                row_value = 2
                row_buttons = 3 
            
            col_start = i % 3 * 2 # 0, 2, 4
            
            # 1. 构建最终的标签文本 (使用 \n 换行)
            final_label_text = base_label_text
            if additional_label_text:
                final_label_text += f"\n({additional_label_text})"
            
            # 2. 创建并配置 QLabel (标签部分)
            label = QLabel(final_label_text)
            label.setAlignment(Qt.AlignCenter)
            
            # 3. 创建并配置 QLineEdit (值显示/输入部分)
            value_label = QLineEdit("0.00")
            # --- 关键修改：设为只读，不再允许手动输入 ---
            value_label.setReadOnly(True) 
            value_label.setFixedWidth(70)
            value_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter) 
            # 关键修改：将背景色改为灰色，表示只读
            value_label.setStyleSheet("background-color: lightgrey; border: 1px inset grey;")
            self.tool_pose_labels[var_key] = value_label
            
            # ***** 关键修改: 强制设置固定高度，防止被多行标签拉伸 *****
            value_label.setFixedHeight(25) 
            
            # 4. 创建微调按钮布局
            btn_minus = QPushButton("-")
            btn_minus.setFixedWidth(30)
            btn_minus.pressed.connect(lambda idx=i: self.start_tcp_move(idx, BACKWARD))
            btn_minus.released.connect(self.stop_tcp_move) 
            btn_plus = QPushButton("+")
            btn_plus.setFixedWidth(30)
            btn_plus.pressed.connect(lambda idx=i: self.start_tcp_move(idx, FORWARD))
            btn_plus.released.connect(self.stop_tcp_move)
            
            btn_layout = QHBoxLayout()
            btn_layout.addStretch()
            btn_layout.addWidget(btn_minus)
            btn_layout.addWidget(btn_plus)
            btn_layout.addStretch()
            
            # 5. 将控件添加到网格布局
            group_layout.addWidget(label, row_value, col_start, alignment=Qt.AlignCenter)
            group_layout.addWidget(value_label, row_value, col_start + 1, alignment=Qt.AlignLeft) 
            group_layout.addLayout(btn_layout, row_buttons, col_start, 1, 2)
        
        # 6. 添加 Tool 坐标系复选框 (放在下一行，即第 4 行，跨越所有列)
        self.tool_coord_checkbox = QCheckBox("Move in Tool Coordinate System?")
        group_layout.addWidget(self.tool_coord_checkbox, 4, 0, 1, 6, alignment=Qt.AlignCenter)
        
        layout.addWidget(group)
        
    def create_record_oae_state_group(self, layout):
        """创建用于记录机器人A点、O点和end-effect点的模块。（已移除人工输入按钮，调整获取按钮位置）"""
        group = QGroupBox("Robot A, O, and End-Effect Positions")
        group_layout = QVBoxLayout(group)

        # 定义一个统一的按钮宽度，确保三个“获取”按钮大小一致
        BUTTON_WIDTH = 160
        
        # A点布局
        a_point_subgroup = QGroupBox("A点")
        a_point_layout = QGridLayout(a_point_subgroup)
        a_labels = ["A_x:", "A_y:", "A_z:"]
        for i, label_text in enumerate(a_labels):
            label = QLabel(label_text)
            text_box = QLineEdit("0.00")
            # --- 保持为可输入，以便人工调整或加载数据 ---
            text_box.setReadOnly(False)
            text_box.setStyleSheet("background-color: white;")
            # --- 结束修改 ---
            self.a_vars[i] = text_box
            a_point_layout.addWidget(label, 0, i * 2)
            a_point_layout.addWidget(text_box, 0, i * 2 + 1)
        
        # --- 按钮布局重组：将获取按钮放在 Z 坐标右边 ---
        self.get_a_btn = QPushButton("Get A Point Position") 
        self.get_a_btn.setFixedWidth(BUTTON_WIDTH) # 设置固定宽度
        a_point_layout.addWidget(self.get_a_btn, 0, 6) # 放置在 A_z 右侧
        # --- 按钮布局结束 ---
        
        group_layout.addWidget(a_point_subgroup)

        # O点布局
        o_point_subgroup = QGroupBox("O点")
        o_point_layout = QGridLayout(o_point_subgroup)
        o_labels = ["O_x:", "O_y:", "O_z:"]
        for i, label_text in enumerate(o_labels):
            label = QLabel(label_text)
            text_box = QLineEdit("0.00")
            # --- 保持为可输入，以便人工调整或加载数据 ---
            text_box.setReadOnly(False)
            text_box.setStyleSheet("background-color: white;")
            # --- 结束修改 ---
            self.o_vars[i] = text_box
            o_point_layout.addWidget(label, 0, i * 2)
            o_point_layout.addWidget(text_box, 0, i * 2 + 1)
        
        # --- 按钮布局重组：将获取按钮放在 Z 坐标右边 ---
        self.get_o_btn = QPushButton("Get O Point Position")
        self.get_o_btn.setFixedWidth(BUTTON_WIDTH) # 设置固定宽度
        o_point_layout.addWidget(self.get_o_btn, 0, 6) # 放置在 O_z 右侧
        # --- 按钮布局结束 ---
        
        group_layout.addWidget(o_point_subgroup)
        
        # End-Effect点布局
        e_point_subgroup = QGroupBox("End-Effect")
        e_point_layout = QGridLayout(e_point_subgroup)
        e_labels = ["E_x:", "E_y:", "E_z:"]
        for i, label_text in enumerate(e_labels):
            label = QLabel(label_text)
            text_box = QLineEdit("0.00")
            # --- 保持为可输入，以便人工调整或加载数据 ---
            text_box.setReadOnly(False)
            text_box.setStyleSheet("background-color: white;")
            # --- 结束修改 ---
            self.e_vars[i] = text_box
            e_point_layout.addWidget(label, 0, i * 2)
            e_point_layout.addWidget(text_box, 0, i * 2 + 1)
        
        # --- 按钮布局重组：将获取按钮放在 Z 坐标右边 ---
        self.get_e_btn = QPushButton("Get End-Effect Position")
        self.get_e_btn.setFixedWidth(BUTTON_WIDTH) # 设置固定宽度
        e_point_layout.addWidget(self.get_e_btn, 0, 6) # 放置在 E_z 右侧
        # --- 按钮布局结束 ---
        
        group_layout.addWidget(e_point_subgroup)
        
        # 在Groupbox底部添加一个水平布局，用于放置按钮
        button_layout = QHBoxLayout()
        self.align_planes_btn = QPushButton("Rotate the ultrasound plane to pass through the puncture point") # 创建按钮实例
        button_layout.addWidget(self.align_planes_btn)

        group_layout.addLayout(button_layout)

        group_layout.addStretch()
        layout.addWidget(group)
        
    def create_b_point_group(self, layout):
        """
        新增：创建病灶点B定位Group，包含 Base 坐标系结果显示和按钮。
        [修改]：移除了 B点 (TCP_U 坐标系) 输入组和手动转换按钮。
        """
        
        # 宽度常量：用于按钮和下拉列表的统一宽度
        BUTTON_WIDTH = 180 
        
        # 最外层的 GroupBox
        group = QGroupBox("Lesion B Point Localization")
        group_layout = QVBoxLayout(group)
        
        # --- 1. B点 (Base 坐标系) 显示/输入 & Dropdown ---
        b_point_base_subgroup = QGroupBox("B Point (Base Coordinate System)")
        b_point_base_layout = QGridLayout(b_point_base_subgroup)
        
        b_base_labels = ["B_x:", "B_y:", "B_z:"]
        for i, label_text in enumerate(b_base_labels):
            label = QLabel(label_text)
            text_box = QLineEdit("0.00")
            text_box.setReadOnly(False) 
            text_box.setStyleSheet("background-color: white; border: 1px inset grey;") 
            self.b_vars_in_base[i] = text_box
            b_point_base_layout.addWidget(label, 0, i * 2)
            b_point_base_layout.addWidget(text_box, 0, i * 2 + 1)

        # 添加下拉列表到 B_z 右侧 (B_z 对应的列索引为 5)
        self.b_point_dropdown = QComboBox()
        self.b_point_dropdown.setPlaceholderText("Please read TXT file first")
        self.b_point_dropdown.setFixedWidth(BUTTON_WIDTH) 
        b_point_base_layout.addWidget(self.b_point_dropdown, 0, 6) # Column 6 is right of B_z input

        self.b_point_dropdown.currentIndexChanged.connect(self._handle_b_point_dropdown_selection)
        
        group_layout.addWidget(b_point_base_subgroup)
        # --------------------------------------------------------

        # --- 2. 按钮布局 (Load TXT and Rotate) ---
        button_layout = QHBoxLayout()
        # 读取 TXT 文件按钮
        self.load_b_points_intcp_u_txt_btn = QPushButton("Read B Points in TCP_U from TXT File")
        button_layout.addWidget(self.load_b_points_intcp_u_txt_btn) 
        
        group_layout.addLayout(button_layout)
        
        # 旋转按钮
        self.rotate_ultrasound_plane_to_b_btn = QPushButton("Rotate the ultrasound plane to pass through the biopsy point")
        group_layout.addWidget(self.rotate_ultrasound_plane_to_b_btn)
        
        group_layout.addStretch()
        layout.addWidget(group)
                
    def create_ur_control_group(self, layout):
        """
        创建用于控制电源、使能、初始化和TCP设置的高级模块。
        此方法已修改，添加了“复位”、“暂停”、“继续”和“急停”按钮，并控制急停按钮的可用性。
        """
        ur_control_group = QGroupBox("E05-L Pro Device Control")
        ur_control_layout = QVBoxLayout(ur_control_group)
        # 电源/使能/初始化/复位/暂停/急停/继续 按钮布局。
        button_layout = QGridLayout()
        # 第一行按钮：电源和使能
        self.ur_power_btn = QPushButton("Power On")
        self.ur_enable_btn = QPushButton("Enable")
        # 第二行按钮：初始化和复位
        self.ur_init_btn = QPushButton("Initialize Controller")
        self.ur_reset_btn = QPushButton("Reset")
        # 第三行按钮：暂停和继续
        self.ur_pause_btn = QPushButton("Pause")
        self.ur_continue_btn = QPushButton("Continue")
        # 第四行按钮：急停
        self.ur_stop_btn = QPushButton("Emergency Stop")
        self.ur_stop_btn.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        # 默认情况下，急停按钮不可用，直到收到机器人状态信息
        self.ur_stop_btn.setEnabled(False)
        # 将按钮添加到网格布局
        button_layout.addWidget(self.ur_power_btn, 0, 0)
        button_layout.addWidget(self.ur_enable_btn, 0, 1)
        button_layout.addWidget(self.ur_init_btn, 1, 0)
        button_layout.addWidget(self.ur_reset_btn, 1, 1)
        button_layout.addWidget(self.ur_pause_btn, 2, 0)
        button_layout.addWidget(self.ur_continue_btn, 2, 1)
        button_layout.addWidget(self.ur_stop_btn, 3, 0, 1, 2) # 跨越两列
        ur_control_layout.addLayout(button_layout)

        layout.addWidget(ur_control_group)

    def create_teach_mode_group(self, layout):
        """新增：示教功能分组框，并将其移到左侧。"""
        group = QGroupBox("Teach Mode")
        group_layout = QHBoxLayout(group)
        self.teach_mode_checkbox = QCheckBox("Teach Mode On")
        group_layout.addWidget(self.teach_mode_checkbox)
        layout.addWidget(group)

    def create_set_tcp_group(self, layout):
        """
        新增：将工具坐标系设置移出并做成独立的Groupbox。
        已修改：将按钮分为两行。
        """
        tcp_group = QGroupBox("Tool Coordinate System Settings (TCP)")
        
        # 使用垂直布局作为主布局
        main_v_layout = QVBoxLayout(tcp_group)
        
        # 创建网格布局来容纳标签和输入框
        tcp_grid_layout = QGridLayout()
        tcp_labels = ["Tar_Tcp_X", "Tar_Tcp_Y", "Tar_Tcp_Z", "Tar_Tcp_Rx", "Tar_Tcp_Ry", "Tar_Tcp_Rz"]
        tcp_initial_values = ["0", "0", "0", "0", "0", "0"]
        self.tcp_input_entries = []
        for i, label_text in enumerate(tcp_labels):
            row, col = i // 3, (i % 3) * 2
            label = QLabel(label_text + ":")
            entry = QLineEdit(tcp_initial_values[i])
            self.tcp_input_entries.append(entry)
            tcp_grid_layout.addWidget(label, row, col)
            tcp_grid_layout.addWidget(entry, row, col + 1)
        
        # 添加网格布局到主垂直布局中
        main_v_layout.addLayout(tcp_grid_layout)

        # --- 按钮布局调整 ---
        
        # 1. TCP 切换按钮 (第一排)
        button_row1 = QHBoxLayout()
        self.set_tcp_o_btn = QPushButton("Switch to TCP_O")
        self.set_tcp_p_btn = QPushButton("Switch to TCP_P")
        self.set_tcp_u_btn = QPushButton("Switch to TCP_U")
        self.set_tcp_e_btn = QPushButton("Switch to TCP_E")
        self.set_tcp_tip_btn = QPushButton("Switch to TCP_tip")
        
        button_row1.addWidget(self.set_tcp_o_btn)
        button_row1.addWidget(self.set_tcp_p_btn) 
        button_row1.addWidget(self.set_tcp_u_btn)
        button_row1.addWidget(self.set_tcp_e_btn)
        button_row1.addWidget(self.set_tcp_tip_btn)
        
        main_v_layout.addLayout(button_row1)
        
        # 2. 操作按钮 (第二排)
        button_row2 = QHBoxLayout()
        self.set_cur_tcp_btn = QPushButton("Set Cur TCP")
        self.get_suitable_tcp_btn = QPushButton("Get Virtual RCM_O")
        
        button_row2.addWidget(self.set_cur_tcp_btn)
        button_row2.addWidget(self.get_suitable_tcp_btn)
        
        main_v_layout.addLayout(button_row2)
        
        layout.addWidget(tcp_group)

    def set_tcp_o(self):
        """发送设置TCP_O的命令。"""
        self.current_tcp_name = "TCP_O" 
        self.log_message("SetTCPByName,0,TCP_O;")
        self.tcp_manager.send_command("SetTCPByName,0,TCP_O;")
        self.status_bar.showMessage("Status: Sent command to set TCP_O.")

    def set_tcp_p(self):
        """发送设置TCP_P的命令。"""
        self.current_tcp_name = "TCP_P" 
        self.log_message("SetTCPByName,0,TCP_P;")
        self.tcp_manager.send_command("SetTCPByName,0,TCP_P;")
        self.status_bar.showMessage("Status: Sent command to set TCP_P.")

    def set_tcp_u(self):
        """发送设置TCP_U的命令。"""
        self.current_tcp_name = "TCP_U" 
        self.log_message("SetTCPByName,0,TCP_U;")
        self.tcp_manager.send_command("SetTCPByName,0,TCP_U;")
        self.status_bar.showMessage("Status: Sent command to set TCP_U.")

    def set_tcp_e(self):
        """发送设置TCP_E的命令。"""
        self.current_tcp_name = "TCP_E" 
        self.log_message("SetTCPByName,0,TCP_E;")
        self.tcp_manager.send_command("SetTCPByName,0,TCP_E;")
        self.status_bar.showMessage("Status: Sent command to set TCP_E.")

    def set_tcp_tip(self):
        """发送设置TCP_tip的命令。"""
        self.current_tcp_name = "TCP_tip" 
        self.log_message("SetTCPByName,0,TCP_tip;")
        self.tcp_manager.send_command("SetTCPByName,0,TCP_tip;")
        self.status_bar.showMessage("Status: Sent command to set TCP_tip.")
    
    def read_tcp_o(self):
        """发送读取TCP_O的命令。"""
        self.log_message("ReadTCPByName,0,TCP_O;")
        self.tcp_manager.send_command("ReadTCPByName,0,TCP_O;")
        self.status_bar.showMessage("Status: Sent command to read TCP_O.")
    
    def read_tcp_tip(self):
        """发送读取TCP_tip的命令。"""
        self.log_message("ReadTCPByName,0,TCP_tip;")
        self.tcp_manager.send_command("ReadTCPByName,0,TCP_tip;")
        self.status_bar.showMessage("Status: Sent command to read TCP_tip.")
        
    def _get_z_axis_vector(self, rpy_deg):
        """
        根据 Roll, Pitch, Yaw 角度（度），计算工具坐标系 Z 轴在基座标系中的方向向量。
        假设使用 SXYZ 外部（Extrinsic）欧拉角顺序。
        """
        rpy_rad = np.deg2rad(rpy_deg)
        rotation_matrix = pyrot.matrix_from_euler(rpy_rad, 0, 1, 2, extrinsic=True)
        # Z 轴向量是旋转矩阵的第三列
        return rotation_matrix[:, 2]
                
    def get_suitable_tcp(self):
        """
        [修改]：开始执行新的复杂指令序列：切换到 TCP_E，获取 End-Effect 位置，然后执行计算和 TCP 设置序列。
        """
        # 1. 检查连接状态 (可选但推荐)
        if not self.tcp_manager.is_connected:
            QMessageBox.warning(self, "Warning", "Robot is disconnected.")
            return

        self.status_bar.showMessage("Status: Starting 'Get Suitable Tar_Tcp_Z' sequence (Step 0/5: Switch to TCP_E)...")
        
        # Step 0: 切换到 TCP_E (Sequence Start)
        self.set_tcp_e()
        
        # Step 1: 延时 200ms 后，获取 End-Effect 位置
        QTimer.singleShot(200, self._suitable_tcp_sequence_step_1)


    def _suitable_tcp_sequence_step_1(self):
        """
        [序列 Step 1]：获取 End-Effect 位置，并执行投影距离计算和 UI 更新。
        """
        self.status_bar.showMessage("Status: Sequence Step 1/5: Get End-Effect Position and calculate projection distance...")

        # 1. 点击“获取End-Effect位置”
        self.get_e_point_position()
        
        # 2. 检查 E点和 O点是否有数据 (此时 End-Effect 应该已被最新的 latest_tool_pose 更新)
        try:
            o_point = np.array([float(self.o_vars[i].text()) for i in range(3)])
            e_point = np.array([float(self.e_vars[i].text()) for i in range(3)])
        except ValueError:
            QMessageBox.critical(self, "Sequence Interrupted", "Please enter or get valid coordinates for O point and End-Effect point first.")
            self.status_bar.showMessage("Error: Sequence interrupted, O or E point data is invalid.")
            return

        initial_tcp_pose = self.get_current_tool_pose()
        if initial_tcp_pose is None:
            QMessageBox.critical(self, "Sequence Interrupted", "Cannot get the latest robot pose.")
            self.status_bar.showMessage("Error: Sequence interrupted, cannot get robot pose.")
            return
            
        try:
            # 3. 执行计算逻辑 (更新 O_x, O_y, O_z 和 Tar_Tcp_Z/Rz)
            rpy_deg = initial_tcp_pose[3:]
            z_axis_vector = self._get_z_axis_vector(rpy_deg)
            p_ee = initial_tcp_pose[:3]
            vector_eo = o_point - e_point
            projection_distance = np.dot(vector_eo, z_axis_vector)

            # 更新 UI: Tar_Tcp_* 输入框
            self.tcp_input_entries[2].setText(f"{projection_distance:.2f}") # TCP_Z
            for i in range(len(self.tcp_input_entries)):
                if i != 2 and i != 5:
                    self.tcp_input_entries[i].setText("0.00")
            self.tcp_input_entries[5].setText("157.50") # TCP_Rz

            # 更新 UI: O 点位置
            p_o_new = p_ee + projection_distance * z_axis_vector
            self.o_vars[0].setText(f"{p_o_new[0]:.2f}")
            self.o_vars[1].setText(f"{p_o_new[1]:.2f}")
            self.o_vars[2].setText(f"{p_o_new[2]:.2f}")
        
            self.status_bar.showMessage(f"Status: Sequence Step 1/5 Completed. Projection distance is {projection_distance:.2f}.")

            # Step 2: 延时 200ms 后，切换到 TCP_O
            QTimer.singleShot(200, self._suitable_tcp_sequence_step_2)

        except Exception as e:
            QMessageBox.critical(self, "Sequence Interrupted", f"Calculation or UI update failed: {e}")
            self.status_bar.showMessage("Error: Sequence interrupted, calculation failed.")


    def _suitable_tcp_sequence_step_2(self):
        """
        [序列 Step 2]：切换到 TCP_O，并延时执行下一步。
        """
        self.status_bar.showMessage("Status: Sequence Step 2/5: Switch to TCP_O...")
        # Step 2: 切换到 TCP_O
        self.set_tcp_o()
        
        # Step 3: 延时 200ms 后，设置 Cur TCP
        QTimer.singleShot(200, self._continue_suitable_tcp_sequence)

    
    def _continue_suitable_tcp_sequence(self):
        """
        [序列 Step 3]：设置 Cur TCP，并延时执行下一步。
        (原有的 _continue_suitable_tcp_sequence)
        """
        self.status_bar.showMessage("Status: Sequence Step 3/5: Set Cur TCP...")
        try:
            # Step 3: 发送 ConfigTCP 指令 ("设置Cur Tcp")
            self.send_set_tcp_command()
            
            # Step 4: 启动延时 200ms，执行序列的最后一步
            QTimer.singleShot(200, self._finalize_suitable_tcp_sequence)
            
        except Exception as e:
            QMessageBox.critical(self, "Command Send Error", f"Sequence Interrupted (Set Cur TCP failed): {e}")
            self.status_bar.showMessage("Error: Sequence interrupted, Set Cur TCP failed.")

    def _finalize_suitable_tcp_sequence(self):
        """
        [序列 Step 4/5]：切换回 TCP_E，并弹出最终成功消息。
        (原有的 _finalize_suitable_tcp_sequence)
        """
        self.status_bar.showMessage("Status: Sequence Step 4/5: Switch back to TCP_E...")
        try:
            # Step 4: 切换回 TCP_E
            self.set_tcp_e()
            
            self.status_bar.showMessage("Status: Sequence Step 5/5 Completed.")
            QMessageBox.information(self, "Operation Successful", 
                                    "Robot command sequence (Switch TCP_E -> Get E Point -> Calculate -> Switch TCP_O -> Set Cur TCP -> Switch TCP_E) completed."
                                   )
        except Exception as e:
             QMessageBox.critical(self, "Command Send Error", f"Sequence Interrupted (Switch TCP_E failed): {e}")
             self.status_bar.showMessage("Error: Sequence interrupted, Switch TCP_E failed.")

    def create_cur_tcp_group(self, layout):
        """新增：创建用于显示当前TCP坐标的Groupbox。"""
        group = QGroupBox("Current TCP Settings")
        
        # 使用垂直布局作为主布局
        main_v_layout = QVBoxLayout(group)
        
        # 创建网格布局来容纳标签和输入框
        grid_layout = QGridLayout()
        tcp_labels = ["Cur_Tcp_X", "Cur_Tcp_Y", "Cur_Tcp_Z", "Cur_Tcp_Rx", "Cur_Tcp_Ry", "Cur_Tcp_Rz"]
        for i, label_text in enumerate(tcp_labels):
            row, col = i // 3, (i % 3) * 2
            label = QLabel(label_text + ":")
            display_entry = QLineEdit("0.00")
            display_entry.setReadOnly(True)
            display_entry.setStyleSheet("background-color: lightgrey;")
            self.cur_tcp_vars[i] = display_entry
            grid_layout.addWidget(label, row, col)
            grid_layout.addWidget(display_entry, row, col + 1)
        
        # 添加网格布局到主垂直布局中
        main_v_layout.addLayout(grid_layout)
        
        # 创建按钮并将其居中
        button_layout = QHBoxLayout()
        self.read_cur_tcp_btn = QPushButton("Read Cur TCP")
        self.read_tcp_o_btn = QPushButton("Read TCP_O")
        self.read_tcp_tip_btn = QPushButton("Read TCP_tip")
        
        button_layout.addWidget(self.read_cur_tcp_btn)
        button_layout.addWidget(self.read_tcp_o_btn)
        button_layout.addWidget(self.read_tcp_tip_btn)
        
        main_v_layout.addLayout(button_layout)
        
        layout.addWidget(group)

    def create_tcp_group(self, layout):
        """创建TCP连接和消息收发模块。"""
        group = QGroupBox("TCP Communication Module")
        group_layout = QVBoxLayout(group)
        # IP和端口输入框布局。
        ip_port_layout = QHBoxLayout()
        ip_port_layout.addWidget(QLabel("Remote IP:"))
        self.ip_entry = QLineEdit("192.168.10.10")
        self.ip_entry.setFixedWidth(200)
        ip_port_layout.addWidget(self.ip_entry)
        ip_port_layout.addWidget(QLabel("Port:"))
        self.port_entry = QLineEdit("10003")
        self.port_entry.setFixedWidth(80)
        ip_port_layout.addWidget(self.port_entry)
        ip_port_layout.addStretch()
        group_layout.addLayout(ip_port_layout)
        # 连接和断开按钮布局。
        btn_layout = QHBoxLayout()
        self.connect_button = QPushButton("Connect")
        self.disconnect_button = QPushButton("Disconnect")
        self.disconnect_button.setEnabled(False)
        btn_layout.addWidget(self.connect_button)
        btn_layout.addWidget(self.disconnect_button)
        group_layout.addLayout(btn_layout)
        # 连接状态标签。
        self.tcp_status_label = QLabel("TCP Status: Disconnected")
        self.tcp_status_label.setStyleSheet("color: blue;")
        group_layout.addWidget(self.tcp_status_label)
        # 接收消息文本框。
        group_layout.addWidget(QLabel("Received Messages:"))
        self.recv_text = QTextEdit()
        self.recv_text.setReadOnly(True)  # 设置为只读，用户无法编辑。
        self.recv_text.setStyleSheet("background-color: lightgrey;")
        group_layout.addWidget(self.recv_text) # Removed the index here
        # 发送消息输入框和按钮。
        group_layout.addWidget(QLabel("Send Message:"))
        send_layout = QHBoxLayout()
        self.send_entry = QLineEdit()
        self.send_entry.setEnabled(False)
        self.send_button = QPushButton("Send")
        self.send_button.setEnabled(False)
        send_layout.addWidget(self.send_entry)
        send_layout.addWidget(self.send_button)
        group_layout.addLayout(send_layout)
        
        # Corrected line to pass the widget directly
        group_layout.setStretchFactor(self.recv_text, 1)
        
        layout.addWidget(group)
            
    def create_data_persistence_group(self, layout):
        """新增：创建用于保存和读取当前数据的Group。"""
        group = QGroupBox("Save Current Data")
        group_layout = QHBoxLayout(group)
        
        self.save_data_btn = QPushButton("Save")
        self.load_data_btn = QPushButton("Load")
        
        group_layout.addStretch()
        group_layout.addWidget(self.save_data_btn)
        group_layout.addWidget(self.load_data_btn)
        group_layout.addStretch()
        
        layout.addWidget(group)
            
    def _get_ui_values(self, qline_edits):
        """尝试从 QLineEdit 列表中获取浮点数列表。失败返回 None 或抛出异常。"""
        try:
            return [float(le.text()) for le in qline_edits]
        except ValueError:
            return None
        
    def _get_ultrasound_normal(self, tcp_rpy_deg):
        """
        根据工具末端的欧拉角，计算超声平面的法向量。
        (核心逻辑复制自 core/ultrasound_plane.py:calculate_ultrasound_plane_normal)
        """
        euler_angles_rad = np.deg2rad(tcp_rpy_deg)
        tool_rotation_matrix = pyrot.matrix_from_euler(euler_angles_rad, 0, 1, 2, extrinsic=True)
        angle_offset_rad = np.deg2rad(-22.5) 
        z_axis_angle = np.array([0, 0, 1, angle_offset_rad])
        rotation_delta_matrix = pyrot.matrix_from_axis_angle(z_axis_angle)
        new_ultrasound_rotation_matrix = np.dot(tool_rotation_matrix, rotation_delta_matrix)
        tool_y_axis_in_world_frame = new_ultrasound_rotation_matrix[:, 1]
        
        return tool_y_axis_in_world_frame

    def _calculate_oa_rotation_angle(self, a_point, o_point, b_point, initial_rpy_deg, p_b_in_u_pose):
        """
        计算使超声平面包含B点所需绕OA轴的旋转角度 (度)。
        """
        # 1. 计算旋转轴：OA向量
        oa_vector = a_point - o_point
        
        if np.linalg.norm(oa_vector) < 1e-6:
            raise ValueError("几何错误：OA向量长度为零，无法定义旋转轴。") 

        oa_unit_vector = oa_vector / np.linalg.norm(oa_vector)

        # 2. 计算超声平面的初始法向量
        ultrasound_normal = self._get_ultrasound_normal(initial_rpy_deg)
        
        # 3. 计算 OAB 平面法向量作为目标
        ob_vector = b_point - o_point
        
        if np.linalg.norm(ob_vector) < 1e-6:
            return 0.0

        n_oab_vector = np.cross(oa_vector, ob_vector)
        
        if np.linalg.norm(n_oab_vector) < 1e-6:
            # O, A, B 共线
            return 0.0 

        # 4. 计算旋转所需的角度
        proj_u_norm = ultrasound_normal - np.dot(ultrasound_normal, oa_unit_vector) * oa_unit_vector
        proj_target = n_oab_vector - np.dot(n_oab_vector, oa_unit_vector) * oa_unit_vector
        
        if np.linalg.norm(proj_u_norm) < 1e-6 or np.linalg.norm(proj_target) < 1e-6:
            return 0.0 

        y_axis_in_plane = np.cross(oa_unit_vector, proj_u_norm)
        
        x_component = np.dot(proj_target, proj_u_norm)
        y_component = np.dot(proj_target, y_axis_in_plane)
        
        rotation_angle_rad = np.arctan2(y_component, x_component)

        # ----------------------------------------------------
        # 【统一调试打印】 (根据您的要求合并和调整格式)
        print("\n======================================")
        print("--- B点计算输入与几何向量分析 ---")
        print(f"当前 A 点 (Base): {a_point}")
        print(f"当前 O 点 (Base): {o_point}")
        print(f"当前 TCP E 姿态 (Base, RPY): {initial_rpy_deg}")
        print(f"待计算 B 点 (TCP_U 原始输入, 6D): {p_b_in_u_pose}")
        print(f"OA轴单位向量: {oa_unit_vector}")
        print(f"初始超声法向量 (N_US): {ultrasound_normal}")
        print(f"目标 OAB 法向量 (N_OAB): {n_oab_vector / np.linalg.norm(n_oab_vector)}")
        print(f"B 点位置 (Base 转换结果): {b_point}")
        print("\n--- 投影向量分析 ---")
        print(f"Proj(N_US): {proj_u_norm}")
        print(f"Proj(N_OAB): {proj_target}")
        print(f"绕OA轴的旋转角度(度): {np.rad2deg(rotation_angle_rad)}")
        print("======================================")
        # ----------------------------------------------------
        
        return np.rad2deg(rotation_angle_rad)

    def _calculate_b_point_data(self, p_b_in_u_pose, index):
        """
        执行单个B点的所有计算：Base坐标系姿态和OA轴旋转角度，并返回索引。
        p_b_in_u_pose: B点在TCP_U下的 [X, Y, Z, Rx, Ry, Rz] (度) 姿态。
        index: B点编号 (1-based integer).
        """
        # --- 0. 检查和读取依赖数据 ---
        if self.tcp_u_definition_pose is None:
            raise ValueError("未读取 TCP_U 定义，无法进行坐标变换。")
            
        # [修改]：使用 self.tcp_e_medical_value 代替实时的 latest_tool_pose
        if self.tcp_e_medical_value is None:
             raise ValueError("未记录 TCP_E_Medical_value，无法进行 B 点计算。请先点击'Ultrasound Probe Rotate Left x Deg'按钮来记录当前 TCP_E 姿态。")

        pose_base_to_e = np.array(self.tcp_e_medical_value) # 使用记录的值

        a_point_val = self._get_ui_values(self.a_vars)
        o_point_val = self._get_ui_values(self.o_vars)

        if a_point_val is None or o_point_val is None:
            raise ValueError("A点或O点坐标必须为有效数字。")

        a_point = np.array(a_point_val[:3])
        o_point = np.array(o_point_val[:3])
        initial_rpy_deg = pose_base_to_e[3:] # 提取 RPY 姿态角

        # --- 1. 计算 T_Base_to_B (Base 坐标系下的 B 点姿态) ---
        T_U_to_B = self._pose_to_matrix(p_b_in_u_pose)
        T_Base_to_E = self._pose_to_matrix(pose_base_to_e)
        T_E_to_U = self._pose_to_matrix(self.tcp_u_definition_pose)
        
        T_Base_to_B = T_Base_to_E @ T_E_to_U @ T_U_to_B
        
        p_b_in_base = T_Base_to_B[:3, 3] # B 点位置 (X, Y, Z)
        r_b_in_base_rpy = pyrot.euler_from_matrix(T_Base_to_B[:3, :3], 0, 1, 2, extrinsic=True) 
        p_b_in_base_pose = np.append(p_b_in_base, np.rad2deg(r_b_in_base_rpy))

        # --- 2. 计算 OA 轴旋转角度 ---
        # 传递所有需要的参数和用于打印调试信息的原始 B 点 TCP_U 姿态
        rotation_angle_deg = self._calculate_oa_rotation_angle(
            a_point, 
            o_point, 
            p_b_in_base, 
            initial_rpy_deg,
            p_b_in_u_pose 
        )
        
        return p_b_in_base_pose, rotation_angle_deg, index
    
    def read_b_points_in_tcp_u_from_file(self):
        """
        [修改]：读取TXT文件，解析B点(TCP_U)位置(X,Y,Z)，计算Base姿态和旋转角度，
               弹出多选对话框，并处理返回的选定点列表。
        """
        # 1. 检查 TCP_U 定义
        if self.tcp_u_definition_pose is None:
            QMessageBox.warning(self, "Warning", "Please connect robot and ensure TCP_U definition has been acquired.")
            return

        # [新增]：检查 TCP_E_Medical_value 是否存在
        if self.tcp_e_medical_value is None:
            QMessageBox.warning(self, "Warning", "Please click 'Ultrasound Probe Rotate Left x Deg' button first to record the necessary TCP_E_Medical_value for calculation.")
            return

        # 2. 检查 A, O 点是否已设置且不重合
        try:
            a_point_val = self._get_ui_values(self.a_vars)
            o_point_val = self._get_ui_values(self.o_vars)
            
            if a_point_val is None or o_point_val is None:
                QMessageBox.critical(self, "Data Error", "Please enter or get valid A and O point coordinates in the 'Robot A, O, and End-Effect Positions' module first.")
                return
            
            a_point_pos = np.array(a_point_val[:3])
            o_point_pos = np.array(o_point_val[:3])
            
            # 检查 A 点和 O 点是否重合（导致 OA 轴无法定义）
            if np.linalg.norm(a_point_pos - o_point_pos) < 1e-6:
                QMessageBox.critical(self, "Geometric Error", "A and O point coordinates coincide or are too close, unable to define OA rotation axis. Please re-enter or acquire valid coordinates.")
                return
                
        except ValueError:
            QMessageBox.critical(self, "Data Error", "A or O point coordinates must be valid numbers.")
            return

        # 3. 文件读取和解析 (使用文件对话框允许用户选择)
        FILE_NAME = "TCP_U_B_LIST.txt"
        
        # 优先查找当前目录下的文件
        file_path = os.path.join(os.getcwd(), FILE_NAME)
        
        if not os.path.exists(file_path):
             file_dialog = QFileDialog(self)
             file_dialog.setWindowTitle("Select B Point File")
             file_dialog.setNameFilter("Text Files (*.txt);;All Files (*)")
             file_dialog.setFileMode(QFileDialog.ExistingFile) 
             
             if file_dialog.exec_() == QFileDialog.Accepted:
                 selected_files = file_dialog.selectedFiles()
                 if not selected_files:
                     return
                 file_path = selected_files[0]
                 FILE_NAME = os.path.basename(file_path)
             else:
                 self.status_bar.showMessage("Status: User cancelled file selection.")
                 return
        
        b_point_data_list = []
        try:
            with open(file_path, 'r') as f:
                index = 1 # <--- ADDED: 从 1 开始编号
                for line_num, line in enumerate(f, 1):
                    line = line.strip()
                    if not line:
                        continue
                    
                    # 清理输入行，移除方括号和空格
                    line = line.strip('[] \t\n')
                    parts = line.split(',')

                    if len(parts) != 3:
                        self.log_message(f"Warning: File {FILE_NAME} line {line_num} format error, expected 3 position parameters (X, Y, Z), skipped: {line}")
                        continue
                    
                    try:
                        p_b_in_u_pos = [float(p.strip()) for p in parts]
                        p_b_in_u_pose = np.array(p_b_in_u_pos + [0.0, 0.0, 0.0])
                        
                        # 4. 执行计算 
                        # 传入当前 index
                        p_b_in_base_pose, rotation_angle_deg, calculated_index = self._calculate_b_point_data(p_b_in_u_pose, index) 
                        
                        # 存储: (TCP_U 姿态, Base 姿态, 旋转角度, 索引)
                        b_point_data_list.append((p_b_in_u_pose, p_b_in_base_pose, rotation_angle_deg, calculated_index))
                        
                        index += 1 # 递增 index
                        
                    except ValueError as e:
                        self.log_message(f"Calculation Error (File {FILE_NAME} line {line_num}): Failed to convert parameters to numbers or calculation failed: {e}")
                        
        except Exception as e:
            QMessageBox.critical(self, "File Read or Calculation Error", f"Error processing file: {e}")
            return

        if not b_point_data_list:
            QMessageBox.warning(self, "Warning", f"File {FILE_NAME} contains no valid B point data for calculation.")
            return

        # 5. 弹出多选对话框
        dialog = BPointSelectionDialog(b_point_data_list, self)
        # 关键：连接新的信号
        dialog.b_points_selected.connect(self._handle_b_points_list_selection) 
        dialog.exec_()
        
    def _handle_b_point_dropdown_selection(self, index):
        """
        槽函数：处理下拉列表选择事件，更新 Base 坐标系 B 点显示和内部状态。
        """
        if index < 0 or not self.calculated_b_points:
            # -1 索引通常是占位符或列表清空
            # 此时清空 Base 坐标显示
            self.b_point_position_in_base = np.zeros(3) 
            # 使用列表推导式直接写入 "0.00"
            for i in range(3):
                 self.b_vars_in_base[i].setText("0.00")
            return
        
        if index >= len(self.calculated_b_points):
            self.status_bar.showMessage("Warning: Dropdown index out of bounds.")
            return
        # data: [p_u_pose (6), p_base_pose (6), rotation_angle_deg (1), index (1)]
        selected_data = self.calculated_b_points[index]
        p_base_pose = selected_data[1] # Base 姿态 (6 elements: X, Y, Z, Rx, Ry, Rz)
        p_base_position = p_base_pose[0:3] # <--- 提取前 3 个位置数据 (X, Y, Z)
        rotation_angle = selected_data[2]
        b_index = selected_data[3]

        # 1. Update internal state (Base 坐标系下的 B 点位置)
        self.b_point_position_in_base = p_base_position
        
        # 2. 写入 B点 (Base坐标系) UI (红框处)
        try:
            # 确保将 Base 姿态中的 X, Y, Z 值分别写入对应的输入框
            self.b_vars_in_base[0].setText(f"{p_base_position[0]:.2f}")
            self.b_vars_in_base[1].setText(f"{p_base_position[1]:.2f}")
            self.b_vars_in_base[2].setText(f"{p_base_position[2]:.2f}")
        except Exception as e:
            self.status_bar.showMessage(f"Failed to write B point coordinates (UI reference error): {e}")
            return

        self.status_bar.showMessage(f"Status: Selected B{b_index}, Base coordinates updated (OA axis angle: {rotation_angle:.2f} degrees).")

    def _set_ui_values(self, var_list, data_list):
        """Helper to set QLineEdit values from a list of floats."""
        for var, data in zip(var_list, data_list):
            var.setText(f"{data:.2f}")
            
    def save_data(self):
        """保存当前的A/O/E点数据到JSON文件。（已移除 B点 TCP_U 输入的保存）"""
        # 1. 收集数据
        try:
            data = {}
            
            # A, O, E points
            data['a_point'] = self._get_ui_values(self.a_vars)
            data['o_point'] = self._get_ui_values(self.o_vars)
            data['e_point'] = self._get_ui_values(self.e_vars)
            # data['b_point_in_tcp_u'] = self._get_ui_values(self.b_vars_in_tcp_u) # <--- REMOVED

            if any(v is None for v in [data['a_point'], data['o_point'], data['e_point']]): # <--- ADJUSTED CHECK
                 raise ValueError("Invalid number in A/O/E point data.")

            # 新增：保存计算出的 B 点列表 (需要将 NumPy 数组转换为 List)
            # 结构: [p_u_pose (6), p_base_pose (6), angle (1), index (1)]
            calculated_b_points_serializable = []
            for p_u, p_base, angle, index in self.calculated_b_points:
                # Convert NumPy arrays to lists for JSON serialization
                p_u_list = p_u.tolist() if isinstance(p_u, np.ndarray) else list(p_u)
                p_base_list = p_base.tolist() if isinstance(p_base, np.ndarray) else list(p_base)
                calculated_b_points_serializable.append((p_u_list, p_base_list, float(angle), int(index)))
                
            data['calculated_b_points'] = calculated_b_points_serializable

        except ValueError as e:
            QMessageBox.critical(self, "Save Error", f"Data collection failed: {e}")
            return
        
        # 2. 写入文件
        try:
            with open(DATA_FILE_NAME, 'w') as f:
                json.dump(data, f, indent=4)
            self.status_bar.showMessage(f"Status: Data successfully saved to {DATA_FILE_NAME}.")
            QMessageBox.information(self, "Save Successful", f"Current data saved to file: {DATA_FILE_NAME}")
        except Exception as e:
            QMessageBox.critical(self, "File Write Error", f"Failed to save data to file: {e}")

    def load_data(self):
        """从JSON文件读取数据，并恢复到当前的A/O/E点输入框。（已移除 B点 TCP_U 输入的加载）"""
        if not os.path.exists(DATA_FILE_NAME):
            QMessageBox.warning(self, "Load Failed", f"Data file not found: {DATA_FILE_NAME}")
            self.status_bar.showMessage("Status: Last saved data file not found.")
            return
        
        # 1. 读取文件
        try:
            with open(DATA_FILE_NAME, 'r') as f:
                data = json.load(f)
        except Exception as e:
            QMessageBox.critical(self, "File Read Error", f"Failed to read data file: {e}")
            return

        # 2. 恢复数据到UI
        try:
            # A, O, E points
            self._set_ui_values(self.a_vars, data.get('a_point', [0.0]*3))
            self._set_ui_values(self.o_vars, data.get('o_point', [0.0]*3))
            self._set_ui_values(self.e_vars, data.get('e_point', [0.0]*3))
            
            # b_point_in_tcp_u_data = data.get('b_point_in_tcp_u', [0.0]*3) # <--- REMOVED
            # self._set_ui_values(self.b_vars_in_tcp_u, b_point_in_tcp_u_data) # <--- REMOVED
            
            # 3. 新增：恢复计算出的 B 点列表，并还原下拉列表
            calculated_b_points_loaded = data.get('calculated_b_points', [])
            if calculated_b_points_loaded:
                # 将加载的列表元素转换回 (np.ndarray, np.ndarray, float, int) 格式
                restored_b_points = []
                for p_u_list, p_base_list, angle, index in calculated_b_points_loaded:
                    restored_b_points.append((np.array(p_u_list), np.array(p_base_list), float(angle), int(index)))
                
                # 调用处理函数来填充下拉列表和设置默认选中项
                # 此函数会清空旧列表，填充新项，并触发 Base 坐标更新
                self._handle_b_points_list_selection(restored_b_points)
                self.status_bar.showMessage(f"Status: Data restored from {DATA_FILE_NAME}, B point list recovered.")
            else:
                # 如果没有保存的列表数据，则清空下拉列表
                self.calculated_b_points = []
                self.b_point_dropdown.clear()
                self.b_point_dropdown.setPlaceholderText("Data loaded from file")
                self.status_bar.showMessage(f"Status: Data restored from {DATA_FILE_NAME}.")

            QMessageBox.information(self, "Load Successful", "Last saved data restored.")
            
        except Exception as e:
            QMessageBox.critical(self, "Data Recovery Error", f"Failed to restore UI data: {e}")

    def _handle_auto_tcp_u_definition_response(self, response):
        """
        内部方法：处理由 'connect_tcp' 自动触发的 ReadTCPByName 响应，
        通过检查 self.temp_expected_response 标志来确保是正确的响应。
        """
        # 1. 立即清除标志，防止后续无关的 ReadTCPByName 消息误触发
        self.temp_expected_response = None 
        
        # 2. 检查消息格式是否符合 ReadTCPByName,OK/Fail,...
        if not response.startswith("ReadTCPByName"):
             return             
        parts = response.strip(';').strip(',').split(',')
        
        # 3. 检查长度和状态码 (期望 8 个部分，且第二个部分是 'OK')
        if len(parts) == 8 and parts[1].strip() == 'OK':
            try:
                # 参数索引从 2 到 7
                tcp_u_pose_str = parts[2:] 
                tcp_u_pose = [float(p) for p in tcp_u_pose_str]
                
                if len(tcp_u_pose) == 6:
                    self.tcp_u_definition_pose = tcp_u_pose
                    pose_str = ", ".join([f"{p:.2f}" for p in tcp_u_pose])
                    self.log_message(f"System: TCP_U Definition (relative to TCP_E) auto-stored: [{pose_str}]")
                    # [重要] 成功获取后，弹出提示，解决用户等待问题
                    self.status_bar.showMessage("Status: TCP_U definition successfully auto-acquired.")
                
            except Exception as e:
                self.log_message(f"Warning: Failed to auto-parse TCP_U definition data: {e}")
                
        elif len(parts) > 1 and parts[1].strip() == 'Fail':
            self.log_message(f"Warning: Auto-read of TCP_U definition failed: {response}")
            QMessageBox.warning(self, "TCP_U Warning", "Failed to auto-acquire TCP_U definition, functionality limited.")

    def _pose_to_matrix(self, pose_xyz_rpy):
        """
        Helper: Convert [X, Y, Z, Rx, Ry, Rz] (degrees) to 4x4 Homogeneous Matrix (T).
        使用 SXYZ Extrinsic 约定，与 core/ultrasound_plane.py 保持一致。
        """
        x, y, z, rx, ry, rz = pose_xyz_rpy
        
        # 1. Position vector (translation)
        translation = np.array([x, y, z])
        
        # 2. Rotation Matrix (SXYZ Extrinsic)
        rpy_rad = np.deg2rad([rx, ry, rz])
        # The order (0, 1, 2) corresponds to X, Y, Z axes
        rotation_matrix = pyrot.matrix_from_euler(rpy_rad, 0, 1, 2, extrinsic=True)
        
        # 3. Homogeneous Transformation Matrix (4x4)
        T = np.identity(4)
        T[:3, :3] = rotation_matrix
        T[:3, 3] = translation
        
        return T
    
    def _handle_b_points_list_selection(self, selected_points_list):
        """
        槽函数：接收选定的 B 点列表，存储所有点，并按指定格式填充到下拉列表中。
        同时默认选择第一个点，并触发选择处理逻辑。
        """
        # 1. 存储选定的点列表
        self.calculated_b_points = selected_points_list
        
        # 2. 填充下拉列表 (清空旧数据)
        self.b_point_dropdown.clear()
        
        if not self.calculated_b_points:
            self.b_point_dropdown.setPlaceholderText("No points selected")
            # 确保 Base 坐标系 UI 重置
            self.b_point_position_in_base = np.zeros(3) 
            self._set_ui_values(self.b_vars_in_base, [0.0]*3)
            self.status_bar.showMessage("Status: 0 B points selected.")
            return
            
        for data in selected_points_list:
            # data: [p_u_pose (6), p_base_pose (6), angle (1), index (1)]
            angle = data[2]
            index = data[3]
            # 格式: "B<编号>,<角度>度"
            display_text = f"B{index}, {angle:.2f}deg" 
            self.b_point_dropdown.addItem(display_text)

        # 3. 默认选中第一个点，这会触发 _handle_b_point_dropdown_selection 来更新红框区域
        self.b_point_dropdown.setCurrentIndex(0) 
        
        self.status_bar.showMessage(f"Status: {len(selected_points_list)} B points selected, list populated.")

    def _continue_b_point_rotation(self):
        """
        当接收到 WayPoint 成功响应后，发送下一个旋转步骤的 WayPoint 指令。
        """
        # 如果不在 B 点旋转模式，则忽略
        if not self.b_point_rotation_steps:
            return

        # 检查是否完成所有步骤
        self.current_b_point_step += 1
        
        if self.current_b_point_step < len(self.b_point_rotation_steps):
            # 1. 获取当前步骤的累积旋转矩阵
            delta_rotation_matrix = self.b_point_rotation_steps[self.current_b_point_step]
            
            # 2. 从初始姿态计算目标姿态
            initial_tcp_pose = self.initial_tcp_pose_for_b_rot
            o_point = self.b_point_o_point
            
            # 3. 计算最终位置 P_final (使用初始XYZ和O点)
            P_final = get_final_tcp_e_position_after_delta_rotation(
                initial_tcp_pose[:3], 
                delta_rotation_matrix, 
                o_point
            )
            
            # 4. 计算最终旋转矩阵 R_final 和 RPY
            initial_rpy_rad = np.deg2rad(initial_tcp_pose[3:])
            initial_rotation_matrix = pyrot.matrix_from_euler(initial_rpy_rad, 0, 1, 2, extrinsic=True)
            # R_final = R_delta * R_initial
            R_final = np.dot(delta_rotation_matrix[:3,:3], initial_rotation_matrix)
            final_rpy_rad = pyrot.euler_from_matrix(R_final, 0, 1, 2, extrinsic=True)
            final_rpy_deg = np.rad2deg(final_rpy_rad)
                                    
            # 5. 构造 WayPoint 命令
            dX_dY_dZ = P_final
            dRx_dRy_dRz = final_rpy_deg
            pos_str = ",".join([f"{val:.2f}" for val in dX_dY_dZ])
            rpy_str = ",".join([f"{val:.2f}" for val in dRx_dRy_dRz])
            pos_rpy_str = f"{pos_str},{rpy_str}"
            
            # WayPoint 参数 (使用关节移动 Type=0, 半径=0)
            dJ_zero = ",".join(["0.00"] * 6)
            nRbtID = 0
            sTcpName = "TCP"
            sUcsName = "Base"
            dVelocity = 50
            dAcc = 250
            dRadius = 0 # 半径设为 0
            nMoveType = 0 # 0 代表关节运动 (Joint movement)
            nIsUseJoint = 0 
            nIsSeek = 0
            nIOBit = 0
            nIOState = 0
            strCmdID = "ID1"
            
            command = (
                f"WayPoint,{nRbtID},{pos_rpy_str},{dJ_zero},"
                f"{sTcpName},{sUcsName},{dVelocity},{dAcc},{dRadius},{nMoveType},"
                f"{nIsUseJoint},{nIsSeek},{nIOBit},{nIOState},{strCmdID};"
            )
            
            # 6. 发送指令
            # self.tcp_manager.send_command(command)
            self.log_message(command)
            # 提取累积旋转角度用于状态栏显示
            step_angle = np.rad2deg(pyrot.axis_angle_from_matrix(delta_rotation_matrix[:3, :3])[3])
            self.log_message(f"Status: Rotate Ultrasound Plane around OA (Step {self.current_b_point_step+1}/{len(self.b_point_rotation_steps)}, cumulative rotation {step_angle:.2f} degrees)...")
            
        else:
            # 7. 完成所有步骤
            self.b_point_rotation_steps = []
            self.current_b_point_step = -1
            self.initial_tcp_pose_for_b_rot = None
            self.b_point_o_point = None
            self.status_bar.showMessage("Status: Rotation of Ultrasound Plane around OA to B Point task completed.")
            QMessageBox.information(self, "Task Completed", "Completed all steps for rotating ultrasound plane around OA axis to pass through B point.")
            
    def compute_and_store_tcp_u_volume(self):
        """
        根据当前的 tcp_e_medical_value 和 tcp_u_definition_pose，
        计算并记录 tcp_u_volume (Base坐标系下的 TCP_U 姿态)。
        计算公式: T_Base_U = T_Base_E * T_E_U
        """
        # 检查必要的数据是否存在
        if self.tcp_e_medical_value is None:
            self.log_message("Error: tcp_e_medical_value is None, cannot calculate tcp_u_volume.")
            return

        if self.tcp_u_definition_pose is None:
            self.log_message("Warning: TCP_U definition is missing (not read from robot yet). Cannot calculate tcp_u_volume.")
            self.tcp_u_volume = None
            return

        try:
            # 1. T_Base_E: 将 tcp_e_medical_value (Base->E) 转为 4x4 矩阵
            T_Base_E = self._pose_to_matrix(self.tcp_e_medical_value)
            
            # 2. T_E_U: 将 tcp_u_definition_pose (E->U) 转为 4x4 矩阵
            T_E_U = self._pose_to_matrix(self.tcp_u_definition_pose)
            
            # 3. T_Base_U = T_Base_E * T_E_U
            T_Base_U = np.dot(T_Base_E, T_E_U)
            
            # 4. 从矩阵中提取姿态 (XYZ + RPY)
            trans = T_Base_U[:3, 3]
            rot_matrix = T_Base_U[:3, :3]
            rpy_rad = pyrot.euler_from_matrix(rot_matrix, 0, 1, 2, extrinsic=True)
            rpy_deg = np.rad2deg(rpy_rad)
            
            # 5. 保存结果
            self.tcp_u_volume = np.concatenate((trans, rpy_deg)).tolist()
            
            # 打印日志
            u_vol_str = ", ".join([f"{v:.2f}" for v in self.tcp_u_volume])
            self.log_message(f"System: Calculated and stored tcp_u_volume: [{u_vol_str}]")
            
        except Exception as e:
            self.log_message(f"Error calculating tcp_u_volume: {e}")
            self.tcp_u_volume = None