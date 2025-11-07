# ui/main_window.py
import sys
import json
import os
import numpy as np
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QGroupBox, QLabel, QLineEdit, QPushButton,
                             QTextEdit, QStatusBar, QGridLayout, QMessageBox,
                             QCheckBox, QSlider, QTabWidget)
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal
from PyQt5.QtGui import QCloseEvent, QFont, QImage, QPixmap
from core.tcp_manager import TCPManager
from core.ultrasound_plane import calculate_rotation_for_plane_alignment, calculate_new_rpy_for_b_point, get_final_tcp_e_position_after_delta_rotation
from ui.ultrasound_tab import UltrasoundTab
from kinematics.prostate_biopsy_robot_kinematics import RobotKinematics
from ui.beckhoff_tab import BeckhoffTab
import pytransform3d.rotations as pyrot

# Constants for motion direction
# 移动方向常量
FORWARD = 1
BACKWARD = 0

DATA_FILE_NAME = "saved_robot_data.json"

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
        self.setWindowTitle("RRRRRR 机器人高级控制界面 (PyQt5版)")
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
        self.e_vars = [None] * 3 # 新增 end-effect 变量列表
        self.b_vars = [None] * 3
        self.b_point_position = np.zeros(3)
        
        self.get_a_btn = None
        self.get_o_btn = None
        self.get_e_btn = None
        self.set_a_btn_manual = None
        self.set_o_btn_manual = None
        self.set_e_btn_manual = None
        self.save_data_btn = None
        self.load_data_btn = None # <--- [新增] 读取按钮引用
        self.align_planes_btn = None
        self.b_point_btn = None
        self.rotate_b_point_btn = None
        
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
        self.create_b_point_group(left_panel_layout)  # 调用新的方法
        
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
        self.tabs.addTab(robot_control_tab, "机器人控制")

        # 将超声图像标签页添加到 QTabWidget
        self.tabs.addTab(self.ultrasound_tab, "超声图像")
        
        # 将 Beckhoff 通信标签页添加到 QTabWidget
        self.tabs.addTab(self.beckhoff_tab, "Beckhoff通信")

        # 状态栏，用于在底部显示简短的程序状态信息。
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("状态: 准备就绪")

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
        self.b_point_btn.clicked.connect(self.set_b_point_position)
        self.rotate_b_point_btn.clicked.connect(self.rotate_ultrasound_plane_to_b)
        self.get_a_btn.clicked.connect(self.get_a_point_position)
        self.get_o_btn.clicked.connect(self.get_o_point_position)
        self.get_e_btn.clicked.connect(self.get_e_point_position)
        self.set_a_btn_manual.clicked.connect(self.set_a_point_manually)
        self.set_o_btn_manual.clicked.connect(self.set_o_point_manually)
        self.set_e_btn_manual.clicked.connect(self.set_e_point_manually)
        self.align_planes_btn.clicked.connect(self.align_ultrasound_plane_to_aoe)
        self.init_joint_pos_btn.clicked.connect(self.send_init_joint_position_command)
        self.input_tool_pose_btn.clicked.connect(self.set_tool_pose_manually)
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
            if "失败" in result:
                QMessageBox.critical(self, "连接错误", result)
        except ValueError:
            QMessageBox.critical(self, "输入错误", "端口号必须是有效数字。")
            
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
        self.tcp_status_label.setText("TCP状态: 已连接" if is_connected else "TCP状态: 未连接")
        self.status_bar.showMessage("状态: 已连接到机器人" if is_connected else "状态: 准备就绪")
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
            self.handle_read_tcp_byname_message(message)
        elif message.startswith("ReadEmergencyInfo"):
            self.handle_emergency_info_message(message)
        elif message.startswith("ReadOverride"):
            self.handle_override_message(message)
        elif message.startswith("SetTCPByName"):
            self.log_message(message)
        elif message.startswith("MoveRelJ,OK"):
            # 如果是，通知 ultrasound_tab 继续下一步操作
            self.ultrasound_tab.continue_rotation()
        elif message.startswith("MoveRelJ"):
            # 说明指令未完成，开启报错
            QMessageBox.critical(self, "关节转动错误", "检查状态机是否空闲")

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
            self.status_bar.showMessage(f"状态: TCP参数已发送到 {sTcpName}。")
        except ValueError:
            QMessageBox.critical(self, "输入错误", "TCP参数必须是有效的数字！")
        except IndexError:
            QMessageBox.critical(self, "内部错误", "TCP参数输入框数量不正确。")

    def toggle_power(self):
        """根据当前电源状态切换按钮文本，并发送相应的上电或断电指令。"""
        if self.power_state == 0:
            # 未上电，发送上电指令。
            self.tcp_manager.send_command("Electrify;")
            self.ur_power_btn.setText("电源开启中...")
            self.ur_power_btn.setEnabled(False) # 暂时禁用按钮，避免重复点击。
        else:
            # 已上电，发送断电指令 (OSCmd,1)。
            self.tcp_manager.send_command("OSCmd,1;")
            self.ur_power_btn.setText("断电中...")
            self.ur_power_btn.setEnabled(False)

    def toggle_enable(self):
        """根据当前使能状态切换按钮文本，并发送使能或去使能指令。"""
        if self.enable_state == 0:
            # 未使能，发送使能指令 (GrpEnable,0)。
            self.tcp_manager.send_command("GrpEnable,0;")
            self.ur_enable_btn.setText("使能中...")
            self.ur_enable_btn.setEnabled(False)
        else:
            # 已使能，发送去使能指令 (GrpDisable,0)。
            self.tcp_manager.send_command("GrpDisable,0;")
            self.ur_enable_btn.setText("去使能中...")
            self.ur_enable_btn.setEnabled(False)

    def handle_teach_mode_state_change(self, state):
        """处理示教功能复选框的状态变化，并发送相应的TCP指令。"""
        if state == Qt.Checked:
            # 如果勾选，发送开启示教功能指令 (GrpOpenFreeDriver,0)。
            command = "GrpOpenFreeDriver,0;"
            self.tcp_manager.send_command(command)
            self.status_bar.showMessage("状态: 示教功能已开启。")
        else:
            # 如果取消勾选，发送关闭示教功能指令 (GrpCloseFreeDriver,0)。
            command = "GrpCloseFreeDriver,0;"
            self.tcp_manager.send_command(command)
            self.status_bar.showMessage("状态: 示教功能已关闭。")

    def _on_override_slider_changed(self, value):
        """当滑条数值改变时，更新标签以显示当前的运动速率值。"""
        override_value = value / 100.0
        self.override_label.setText(f"运动速率: {override_value:.2f}")

    def _on_override_slider_released(self):
        """当用户释放滑条时，发送 SetOverride 指令到机器人。"""
        d_override = self.override_slider.value() / 100.0
        # 指令格式为: `SetOverride,nRbtID,dOverride;`
        command = f"SetOverride,0,{d_override:.2f};"
        self.tcp_manager.send_command(command)
        self.status_bar.showMessage(f"状态: 已设置运动速率为 {d_override:.2f}")

    def handle_override_message(self, message):
        """解析 'ReadOverride' 消息，并更新当前运动速率显示。"""
        parts = message.strip(';').strip(',').split(',')
        if len(parts) == 3 and parts[1] == 'OK':
            try:
                dOverride = float(parts[2])
                self.current_override_value.setText(f"{dOverride:.2f}")
            except (ValueError, IndexError) as e:
                self.log_message(f"警告: 无法解析运动速率信息: {message}, 错误: {e}")
        else:
            self.log_message(f"警告: 接收到无效的 ReadOverride 消息: {message}")

    def handle_emergency_info_message(self, message):
        """解析 'ReadEmergencyInfo' 消息，并根据nESTO值更新急停按钮状态。"""
        parts = message.strip(';').strip(',').split(',')
        if len(parts) == 6 and parts[1] == 'OK':
            try:
                nESTO = int(parts[3])
                # 如果nESTO为0，说明未处于急停状态，急停按钮可用
                self.ur_stop_btn.setEnabled(nESTO == 0)
            except (ValueError, IndexError) as e:
                self.log_message(f"警告: 无法解析急停信息消息: {message}, 错误: {e}")
        else:
            self.log_message(f"警告: 接收到无效的 ReadEmergencyInfo 消息: {message}")

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

                self.status_bar.showMessage("状态: 关节、末端和工具端坐标已实时同步。")
            except (ValueError, IndexError):
                self.log_message("警告: 无法解析 ReadActPos 消息。")
        else:
            self.log_message(f"警告: 接收到无效的 ReadActPos 消息: {message}")

    def get_a_point_position(self):
        """将最新的工具端位置记录到A点文本框中。"""
        if not self.latest_tool_pose:
            self.status_bar.showMessage("警告: 无法获取位置，请先连接机器人并等待数据更新。")
            return
        
        self.a_vars[0].setText(f"{self.latest_tool_pose[0]:.2f}")
        self.a_vars[1].setText(f"{self.latest_tool_pose[1]:.2f}")
        self.a_vars[2].setText(f"{self.latest_tool_pose[2]:.2f}")
        self.status_bar.showMessage("状态: 已获取A点位置。")

    def get_o_point_position(self):
        """将最新的工具端位置记录到O点文本框中。"""
        if not self.latest_tool_pose:
            self.status_bar.showMessage("警告: 无法获取位置，请先连接机器人并等待数据更新。")
            return
        
        self.o_vars[0].setText(f"{self.latest_tool_pose[0]:.2f}")
        self.o_vars[1].setText(f"{self.latest_tool_pose[1]:.2f}")
        self.o_vars[2].setText(f"{self.latest_tool_pose[2]:.2f}")
        self.status_bar.showMessage("状态: 已获取O点位置。")
        
    def get_e_point_position(self):
        """将最新的工具端位置记录到End-Effect文本框中。"""
        if not self.latest_tool_pose:
            self.status_bar.showMessage("警告: 无法获取位置，请先连接机器人并等待数据更新。")
            return
        e_point = self.latest_tool_pose[0:3]
        self.e_vars[0].setText(f"{e_point[0]:.2f}")
        self.e_vars[1].setText(f"{e_point[1]:.2f}")
        self.e_vars[2].setText(f"{e_point[2]:.2f}")
        self.status_bar.showMessage("状态: 已获取End-Effect位置。")
        
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
        """计算并发送指令，以使超声平面与AOE平面重合。"""
        try:
            a_point = np.array([float(self.a_vars[i].text()) for i in range(3)])
            o_point = np.array([float(self.o_vars[i].text()) for i in range(3)])
            e_point = np.array([float(self.e_vars[i].text()) for i in range(3)])
        except ValueError:
            self.status_bar.showMessage("错误: A、O、E点坐标必须为有效数字。")
            return
        
        initial_tcp_pose = self.get_current_tool_pose()
        if initial_tcp_pose is None:
            self.status_bar.showMessage("警告: 无法获取当前的机器人姿态。")
            return

        # 调用函数，计算关节6需要旋转的角度（带正负号）
        initial_rpy_deg = initial_tcp_pose[3:]
        result = calculate_rotation_for_plane_alignment(a_point, o_point, e_point, initial_rpy_deg)

        # Check if the result is an error message
        if isinstance(result, tuple):
            self.status_bar.showMessage(f"错误: {result[1]}")
            QMessageBox.warning(self, "错误", result[1])
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
        self.status_bar.showMessage(f"状态: 已发送指令，使超声平面与AOE平面重合，关节6旋转了 {rotation_angle_deg:.2f} 度。")
        
    def rotate_ultrasound_plane_to_b(self):
        """
        计算并发送指令，以使超声平面绕OA轴旋转，使其经过B点。
        """
        try:
            a_point = np.array([float(self.a_vars[i].text()) for i in range(3)])
            o_point = np.array([float(self.o_vars[i].text()) for i in range(3)])
            if np.all(self.b_point_position == 0):
                QMessageBox.warning(self, "操作失败", "请先输入B点位置。")
                return
            b_point = self.b_point_position
        except ValueError:
            self.status_bar.showMessage("错误: A、O、B点坐标必须为有效数字。")
            return
        
        initial_tcp_pose = self.get_current_tool_pose()
        if initial_tcp_pose is None:
            self.status_bar.showMessage("警告: 无法获取当前的机器人姿态。")
            return

        try:
            # 计算新的欧拉角姿态
            delta_rotation_matrix = calculate_new_rpy_for_b_point(a_point, o_point, b_point, initial_tcp_pose[3:])
            # 1. 将输入的欧拉角从度转换为弧度
            euler_angles_rad = np.deg2rad([initial_tcp_pose[3], initial_tcp_pose[4], initial_tcp_pose[5]])
    
            # 2. 将欧拉角转换为工具末端在世界坐标系中的旋转矩阵
            # 假设旋转顺序是 Roll (X), Pitch (Y), Yaw (Z) (SXYZ 外部旋转)
            tool_rotation_matrix = pyrot.matrix_from_euler(euler_angles_rad, 0, 1, 2, extrinsic=True)

            R_final = np.dot(delta_rotation_matrix, tool_rotation_matrix)
            
            final_rpy_rad = pyrot.euler_from_matrix(R_final, 0, 1, 2, extrinsic=True)
            
            final_rpy_deg = np.rad2deg(final_rpy_rad)
            
            P_final = get_final_tcp_e_position_after_delta_rotation([initial_tcp_pose[0], initial_tcp_pose[1], initial_tcp_pose[2]], delta_rotation_matrix, [o_point[0], o_point[1], o_point[2]])
                        
            # print(delta_rpy_deg)
            
            # 3. 构造 WayPoint 命令 (绝对运动)
            
            nRbtID = 0
            
            # dX, dY, dZ, dRx, dRy, dRz 
            dX_dY_dZ = P_final
            dRx_dRy_dRz = final_rpy_deg
            
            # dJ1, dJ2, dJ3, dJ4, dJ5, dJ6 (全为0)
            dJ_zero = ",".join(["0.00"] * 6)
            
            # dX, dY, dZ, dRx, dRy, dRz 字符串
            pos_str = ",".join([f"{val:.2f}" for val in dX_dY_dZ])
            rpy_str = ",".join([f"{val:.2f}" for val in dRx_dRy_dRz])
            
            # 拼接 6个位置参数 + 6个姿态参数
            pos_rpy_str = f"{pos_str},{rpy_str}"
            
            sTcpName = "TCP"
            sUcsName = "Base"
            dVelocity = 50
            dAcc = 250
            dRadius = 50
            nMoveType = 1 # 1代表直线运动 (Linear movement)
            nIsUseJoint = 0 # 关键：不使用关节值模式
            nIsSeek = 0
            nIOBit = 0
            nIOState = 0
            strCmdID = "ID1"
            
            # WayPoint 命令格式: WayPoint,nRbtID,dX,dY,dZ,dRx,dRy,dRz,dJ1...dJ6,sTcpName,sUcsName,dVelocity,dAcc,dRadius,nMoveType,nIsUseJoint,nIsSeek,nIOBit,nIOState,strCmdID;
            command = (
                f"WayPoint,{nRbtID},{pos_rpy_str},{dJ_zero},"
                f"{sTcpName},{sUcsName},{dVelocity},{dAcc},{dRadius},{nMoveType},"
                f"{nIsUseJoint},{nIsSeek},{nIOBit},{nIOState},{strCmdID};"
            )
            
            self.log_message(f"Final WayPoint Command: {command}") # FINAL COMMAND LOG

            # 发送指令
            # self.tcp_manager.send_command(command)
            print(pos_rpy_str)
            self.status_bar.showMessage("状态: 已发送指令，使超声平面包含B点 (WayPoint)。")

        except ValueError as e:
            self.status_bar.showMessage(f"错误: {e}")
            self.log_message(f"CALCULATION ERROR (ValueError): {e}") # ERROR LOG
        except Exception as e:
             self.status_bar.showMessage(f"致命错误: 姿态计算失败: {e}")
             self.log_message(f"FATAL ERROR: Calculation or command preparation failed: {e}") # FATAL LOG
        self.log_message("--- 绕AO旋转超声平面至B点功能结束 ---") # END LOG
        
    def handle_read_tcp_byname_message(self, message):
        """解析 'ReadTCPByName' 消息，并更新新的TCP显示框。"""
        parts = message.strip(';').strip(',').split(',')
        if len(parts) == 8 and parts[1] == 'OK':
            try:
                tcp_params = [f"{float(p):.2f}" for p in parts[2:]]
                for i in range(len(tcp_params)):
                    self.cur_tcp_vars[i].setText(tcp_params[i])
                self.status_bar.showMessage("状态: 当前TCP坐标已更新。")
            except (ValueError, IndexError):
                self.log_message("警告: 无法解析 ReadTCPByName 消息。")
        else:
            self.log_message(f"警告: 接收到无效的 ReadTCPByName 消息: {message}")
            
    def handle_read_cur_tcp_message(self, message):
        """解析 'ReadCurTCP' 消息，并更新新的TCP显示框。"""
        parts = message.strip(';').strip(',').split(',')
        if len(parts) == 8 and parts[1] == 'OK':
            try:
                tcp_params = [f"{float(p):.2f}" for p in parts[2:]]
                for i in range(len(tcp_params)):
                    self.cur_tcp_vars[i].setText(tcp_params[i])
                self.status_bar.showMessage("状态: 当前TCP坐标已更新。")
            except (ValueError, IndexError):
                self.log_message("警告: 无法解析 ReadCurTCP 消息。")
        else:
            self.log_message(f"警告: 接收到无效的 ReadCurTCP 消息: {message}")
            
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
                self.log_message("警告: 无法解析机器人状态消息中的上电/使能状态。")
        else:
            self.log_message(f"警告: 接收到无效的 ReadRobotState 消息: {message}")

    def update_power_button(self, nElectrify):
        """根据上电状态更新电源按钮的文本和颜色。"""
        self.power_state = nElectrify
        self.ur_power_btn.setEnabled(True)  # 重新启用按钮。
        if nElectrify == 1:
            self.ur_power_btn.setText("电源断开")
            self.ur_power_btn.setStyleSheet("background-color: lightgreen;")
        else:
            self.ur_power_btn.setText("电源开启")
            self.ur_power_btn.setStyleSheet("background-color: salmon;")

    def update_enable_button(self, nEnableState):
        """根据使能状态更新使能按钮的文本和颜色。"""
        self.enable_state = nEnableState
        self.ur_enable_btn.setEnabled(True)
        if nEnableState == 1:
            self.ur_enable_btn.setText("去使能")
            self.ur_enable_btn.setStyleSheet("background-color: lightgreen;")
        else:
            self.ur_enable_btn.setText("使能")
            self.ur_enable_btn.setStyleSheet("background-color: salmon;")

    def send_ur_init_controller_command(self):
        """发送初始化控制器指令。"""
        self.tcp_manager.send_command("StartMaster;")
        self.status_bar.showMessage("状态: 已发送初始化控制器指令。")

    def send_ur_reset_command(self):
        """发送机器人控制器复位指令。"""
        command = "GrpReset,0;" 
        self.tcp_manager.send_command(command)
        self.status_bar.showMessage("状态: 已发送复位指令。")

    def send_ur_pause_command(self):
        """发送机器人运动暂停指令。"""
        command = "GrpInterrupt,0;"
        self.tcp_manager.send_command(command)
        self.status_bar.showMessage("状态: 已发送暂停指令。")

    def send_ur_continue_command(self):
        """发送机器人运动继续指令。"""
        command = "GrpContinue,0;"
        self.tcp_manager.send_command(command)
        self.status_bar.showMessage("状态: 已发送继续指令。")

    def send_ur_stop_command(self):
        """发送机器人急停指令。"""
        command = "GrpStop,0;" 
        self.tcp_manager.send_command(command)
        self.status_bar.showMessage("状态: 已发送急停指令。")

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
        self.status_bar.showMessage(f"状态: 关节{joint_index+1} 微调中...")
    
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
        self.status_bar.showMessage("状态: 关节微调停止")

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
        self.status_bar.showMessage(f"状态: {coord_sys}坐标系下 Tcp_{axis_labels[nAxisId]} 微调中...")

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
        self.status_bar.showMessage("状态: TCP微调停止")
    
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
        self.status_bar.showMessage("状态: 已发送初始化关节位置指令。")

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
        group = QGroupBox("电机微调模块 (正运动学)")
        group_layout = QVBoxLayout(group)
        group_layout.addStretch()
        for i in range(self.num_joints):
            row_layout = QHBoxLayout()
            row_layout.addStretch()
            label = QLabel(f"关节 {i+1} (q{i+1}):")
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
        self.init_joint_pos_btn = QPushButton("初始化关节位置")
        init_btn_layout.addStretch()
        init_btn_layout.addWidget(self.init_joint_pos_btn)
        init_btn_layout.addStretch()
        group_layout.addLayout(init_btn_layout)

        group_layout.addStretch()
        layout.addWidget(group)

    def create_override_group(self, layout):
        """将运动速率滑块和显示做成一个新Groupbox。"""
        group = QGroupBox("运动速率")
        group_layout = QVBoxLayout(group)
        self.override_label = QLabel("运动速率: 0.01")
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
        self.current_override_label = QLabel("当前运动速率:")
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
        """创建用于显示机器人工具端实时状态并包含微调按钮的模块。（已修改为可输入）"""
        group = QGroupBox("机器人工具端实时状态")
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
            # --- 关键修改：移除 setReadOnly(True) 使其可输入 ---
            value_label.setFixedWidth(70)
            value_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter) 
            # 关键修改：将背景色改为白色，表示可输入
            value_label.setStyleSheet("background-color: white; border: 1px inset grey;")
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
        self.tool_coord_checkbox = QCheckBox("Tool 坐标系")
        group_layout.addWidget(self.tool_coord_checkbox, 4, 0, 1, 6, alignment=Qt.AlignCenter)

        # 7. [新增] 添加“输入工具端位姿”按钮 (放在第 5 行)
        self.input_tool_pose_btn = QPushButton("输入工具端位姿")
        group_layout.addWidget(self.input_tool_pose_btn, 5, 0, 1, 6, alignment=Qt.AlignCenter) # 跨越所有 6 列
        
        layout.addWidget(group)
        
    def create_record_oae_state_group(self, layout):
        """创建用于记录机器人A点、O点和end-effect点的模块。（已修改为可输入）"""
        group = QGroupBox("机器人A、O和End-Effect位置")
        group_layout = QVBoxLayout(group)

        # A点布局
        a_point_subgroup = QGroupBox("A点")
        a_point_layout = QGridLayout(a_point_subgroup)
        a_labels = ["A_x:", "A_y:", "A_z:"]
        for i, label_text in enumerate(a_labels):
            label = QLabel(label_text)
            text_box = QLineEdit("0.00")
            # --- 修改：设置为可输入，并改变背景颜色 ---
            text_box.setReadOnly(False)
            text_box.setStyleSheet("background-color: white;")
            # --- 结束修改 ---
            self.a_vars[i] = text_box
            a_point_layout.addWidget(label, 0, i * 2)
            a_point_layout.addWidget(text_box, 0, i * 2 + 1)
        
        # --- 按钮布局重组 ---
        a_btn_layout = QHBoxLayout()
        self.set_a_btn_manual = QPushButton("人工输入A点位置") 
        self.get_a_btn = QPushButton("获取A点位置") 
        a_btn_layout.addWidget(self.set_a_btn_manual)
        a_btn_layout.addWidget(self.get_a_btn)
        a_point_layout.addLayout(a_btn_layout, 1, 0, 1, 6)
        # --- 按钮布局结束 ---
        
        group_layout.addWidget(a_point_subgroup)

        # O点布局
        o_point_subgroup = QGroupBox("O点")
        o_point_layout = QGridLayout(o_point_subgroup)
        o_labels = ["O_x:", "O_y:", "O_z:"]
        for i, label_text in enumerate(o_labels):
            label = QLabel(label_text)
            text_box = QLineEdit("0.00")
            # --- 修改：设置为可输入，并改变背景颜色 ---
            text_box.setReadOnly(False)
            text_box.setStyleSheet("background-color: white;")
            # --- 结束修改 ---
            self.o_vars[i] = text_box
            o_point_layout.addWidget(label, 0, i * 2)
            o_point_layout.addWidget(text_box, 0, i * 2 + 1)
        
        # --- 按钮布局重组 ---
        o_btn_layout = QHBoxLayout()
        self.set_o_btn_manual = QPushButton("人工输入O点位置")
        self.get_o_btn = QPushButton("获取O点位置")
        o_btn_layout.addWidget(self.set_o_btn_manual)
        o_btn_layout.addWidget(self.get_o_btn)
        o_point_layout.addLayout(o_btn_layout, 1, 0, 1, 6)
        # --- 按钮布局结束 ---
        
        group_layout.addWidget(o_point_subgroup)
        
        # End-Effect点布局
        e_point_subgroup = QGroupBox("End-Effect")
        e_point_layout = QGridLayout(e_point_subgroup)
        e_labels = ["E_x:", "E_y:", "E_z:"]
        for i, label_text in enumerate(e_labels):
            label = QLabel(label_text)
            text_box = QLineEdit("0.00")
            # --- 修改：设置为可输入，并改变背景颜色 ---
            text_box.setReadOnly(False)
            text_box.setStyleSheet("background-color: white;")
            # --- 结束修改 ---
            self.e_vars[i] = text_box
            e_point_layout.addWidget(label, 0, i * 2)
            e_point_layout.addWidget(text_box, 0, i * 2 + 1)
        
        # --- 按钮布局重组 ---
        e_btn_layout = QHBoxLayout()
        self.set_e_btn_manual = QPushButton("人工输入End-Effect位置")
        self.get_e_btn = QPushButton("获取End-Effect位置")
        e_btn_layout.addWidget(self.set_e_btn_manual)
        e_btn_layout.addWidget(self.get_e_btn)
        e_point_layout.addLayout(e_btn_layout, 1, 0, 1, 6)
        # --- 按钮布局结束 ---
        
        group_layout.addWidget(e_point_subgroup)
        
        # 在Groupbox底部添加一个水平布局，用于放置按钮
        button_layout = QHBoxLayout()
        self.align_planes_btn = QPushButton("使超声平面对齐AOE平面") # 创建按钮实例
        button_layout.addWidget(self.align_planes_btn)

        group_layout.addLayout(button_layout)

        group_layout.addStretch()
        layout.addWidget(group)
        
    def create_b_point_group(self, layout):
        """新增：创建病灶点B定位Group。"""
        
        # 最外层的 GroupBox
        group = QGroupBox("病灶点B定位")
        group_layout = QVBoxLayout(group)
        
        # 内部的子 GroupBox
        b_point_subgroup = QGroupBox("B点")
        b_point_layout = QGridLayout(b_point_subgroup)
        
        b_labels = ["B_x:", "B_y:", "B_z:"]
        for i, label_text in enumerate(b_labels):
            label = QLabel(label_text)
            text_box = QLineEdit("0.00")
            self.b_vars[i] = text_box
            b_point_layout.addWidget(label, 0, i * 2)
            b_point_layout.addWidget(text_box, 0, i * 2 + 1)
        
        self.b_point_btn = QPushButton("输入B点位置")
        b_point_layout.addWidget(self.b_point_btn, 1, 0, 1, 6, alignment=Qt.AlignCenter)
        
        # 将内部的子 GroupBox 控件添加到外部的布局中
        group_layout.addWidget(b_point_subgroup)
        
        # 在Groupbox底部添加一个水平布局，用于放置按钮
        button_layout = QHBoxLayout()
        self.rotate_b_point_btn = QPushButton("绕AO旋转超声平面使经过B点")
        button_layout.addWidget(self.rotate_b_point_btn)
        
        group_layout.addLayout(button_layout)
        
        group_layout.addStretch()
        layout.addWidget(group)
        
    def set_b_point_position(self):
        """
        从文本框中读取B点坐标，并将其存储。
        """
        try:
            b_x = float(self.b_vars[0].text())
            b_y = float(self.b_vars[1].text())
            b_z = float(self.b_vars[2].text())
            self.b_point_position = np.array([b_x, b_y, b_z])
            self.status_bar.showMessage(f"状态: B点位置 ({b_x}, {b_y}, {b_z}) 已输入。")
        except ValueError:
            self.status_bar.showMessage("错误: B点坐标必须为有效数字。")
            QMessageBox.critical(self, "输入错误", "B点坐标必须是有效的数字！")

    def create_ur_control_group(self, layout):
        """
        创建用于控制电源、使能、初始化和TCP设置的高级模块。
        此方法已修改，添加了“复位”、“暂停”、“继续”和“急停”按钮，并控制急停按钮的可用性。
        """
        ur_control_group = QGroupBox("E05-L Pro设备控制")
        ur_control_layout = QVBoxLayout(ur_control_group)
        # 电源/使能/初始化/复位/暂停/急停/继续 按钮布局。
        button_layout = QGridLayout()
        # 第一行按钮：电源和使能
        self.ur_power_btn = QPushButton("电源开启")
        self.ur_enable_btn = QPushButton("使能")
        # 第二行按钮：初始化和复位
        self.ur_init_btn = QPushButton("初始化控制器")
        self.ur_reset_btn = QPushButton("复位")
        # 第三行按钮：暂停和继续
        self.ur_pause_btn = QPushButton("暂停")
        self.ur_continue_btn = QPushButton("继续")
        # 第四行按钮：急停
        self.ur_stop_btn = QPushButton("急停")
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
        group = QGroupBox("示教功能")
        group_layout = QHBoxLayout(group)
        self.teach_mode_checkbox = QCheckBox("示教功能开启")
        group_layout.addWidget(self.teach_mode_checkbox)
        layout.addWidget(group)

    def create_set_tcp_group(self, layout):
        """
        新增：将工具坐标系设置移出并做成独立的Groupbox。
        已修改：将按钮分为两行。
        """
        tcp_group = QGroupBox("工具坐标系设置 (TCP)")
        
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
        self.set_tcp_o_btn = QPushButton("切换TCP_O")
        self.set_tcp_p_btn = QPushButton("切换TCP_P")
        self.set_tcp_u_btn = QPushButton("切换TCP_U")
        self.set_tcp_e_btn = QPushButton("切换TCP_E")
        self.set_tcp_tip_btn = QPushButton("切换TCP_tip")
        
        button_row1.addWidget(self.set_tcp_o_btn)
        button_row1.addWidget(self.set_tcp_p_btn) 
        button_row1.addWidget(self.set_tcp_u_btn)
        button_row1.addWidget(self.set_tcp_e_btn)
        button_row1.addWidget(self.set_tcp_tip_btn)
        
        main_v_layout.addLayout(button_row1)
        
        # 2. 操作按钮 (第二排)
        button_row2 = QHBoxLayout()
        self.set_cur_tcp_btn = QPushButton("设置Cur TCP")
        self.get_suitable_tcp_btn = QPushButton("获取合适的Tar_Tcp_Z以及更新O点位置")
        
        button_row2.addWidget(self.set_cur_tcp_btn)
        button_row2.addWidget(self.get_suitable_tcp_btn)
        
        main_v_layout.addLayout(button_row2)
        
        layout.addWidget(tcp_group)

    def set_tcp_o(self):
        """发送设置TCP_O的命令。"""
        self.current_tcp_name = "TCP_O" 
        self.log_message("SetTCPByName,0,TCP_O;")
        self.tcp_manager.send_command("SetTCPByName,0,TCP_O;")
        self.status_bar.showMessage("状态: 已发送设置TCP_O的命令。")

    def set_tcp_p(self):
        """发送设置TCP_P的命令。"""
        self.current_tcp_name = "TCP_P" 
        self.log_message("SetTCPByName,0,TCP_P;")
        self.tcp_manager.send_command("SetTCPByName,0,TCP_P;")
        self.status_bar.showMessage("状态: 已发送设置TCP_P的命令。")

    def set_tcp_u(self):
        """发送设置TCP_U的命令。"""
        self.current_tcp_name = "TCP_U" 
        self.log_message("SetTCPByName,0,TCP_U;")
        self.tcp_manager.send_command("SetTCPByName,0,TCP_U;")
        self.status_bar.showMessage("状态: 已发送设置TCP_U的命令。")

    def set_tcp_e(self):
        """发送设置TCP_E的命令。"""
        self.current_tcp_name = "TCP_E" 
        self.log_message("SetTCPByName,0,TCP_E;")
        self.tcp_manager.send_command("SetTCPByName,0,TCP_E;")
        self.status_bar.showMessage("状态: 已发送设置TCP_E的命令。")

    def set_tcp_tip(self):
        """发送设置TCP_tip的命令。"""
        self.current_tcp_name = "TCP_tip" 
        self.log_message("SetTCPByName,0,TCP_tip;")
        self.tcp_manager.send_command("SetTCPByName,0,TCP_tip;")
        self.status_bar.showMessage("状态: 已发送设置TCP_tip的命令。")
    
    def read_tcp_o(self):
        """发送读取TCP_O的命令。"""
        self.log_message("ReadTCPByName,0,TCP_O;")
        self.tcp_manager.send_command("ReadTCPByName,0,TCP_O;")
        self.status_bar.showMessage("状态: 已发送读取TCP_O的命令。")
    
    def read_tcp_tip(self):
        """发送读取TCP_tip的命令。"""
        self.log_message("ReadTCPByName,0,TCP_tip;")
        self.tcp_manager.send_command("ReadTCPByName,0,TCP_tip;")
        self.status_bar.showMessage("状态: 已发送读取TCP_tip的命令。")
        
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
        计算EO向量在关节六Z轴方向的投影距离，并设置到Tcp_Z。
        同时将其他五个TCP值设置成0，并将新的O点位置投影到E点沿着工具Z轴的延长线上。
        """
        # 1. 检查E点和O点是否有数据
        try:
            o_point = np.array([float(self.o_vars[i].text()) for i in range(3)])
            e_point = np.array([float(self.e_vars[i].text()) for i in range(3)])
        except ValueError:
            self.status_bar.showMessage("警告: 请先获取O点和End-effect点坐标。")
            return

        initial_tcp_pose = self.get_current_tool_pose()
        if initial_tcp_pose is None:
            QMessageBox.warning(self, "警告", "无法获取当前的机器人姿态来计算新的O点位置。")
            return
            
        try:
            # a. 获取 End-effect Z 轴向量 (V_Z) 和 P_EE
            rpy_deg = initial_tcp_pose[3:]
            z_axis_vector = self._get_z_axis_vector(rpy_deg)
            p_ee = initial_tcp_pose[:3]

            # b. 计算 EO 向量 (O - E)
            vector_eo = o_point - e_point
            
            # c. 计算投影距离 (D_proj): EO 向量在工具Z轴方向的投影。这个投影距离就是新的 TCP_Z
            projection_distance = np.dot(vector_eo, z_axis_vector)
            self.status_bar.showMessage(f"状态: EO向量在工具Z轴的投影距离为 {projection_distance:.2f}。")

            # 2. 设置TCP_Z和其他TCP值
            self.tcp_input_entries[2].setText(f"{projection_distance:.2f}")
            
            # 3. 将其他五个TCP值设置成0
            for i in range(len(self.tcp_input_entries)):
                if i != 2: # 索引2是Tcp_Z
                    self.tcp_input_entries[i].setText("0.00")

            # 4. 计算新的 O 点位置 (P_O_new)
            # P_O_new = P_EE + D_proj * V_Z
            p_o_new = p_ee + projection_distance * z_axis_vector
            
            # d. 更新 O 点 UI
            self.o_vars[0].setText(f"{p_o_new[0]:.2f}")
            self.o_vars[1].setText(f"{p_o_new[1]:.2f}")
            self.o_vars[2].setText(f"{p_o_new[2]:.2f}")
        
            QMessageBox.information(self, "操作成功", 
                                    f"TCP_Z值已设置为 {projection_distance:.2f}。\n"
                                    f"O点位置已更新至 ({p_o_new[0]:.2f}, {p_o_new[1]:.2f}, {p_o_new[2]:.2f}) (Base坐标系)。\n"
                                    "请点击“设置Cur TCP”按钮以应用更改。"
                                   )
        except Exception as e:
            QMessageBox.critical(self, "计算错误", f"计算TCP或新的O点位置时出错: {e}")
            self.status_bar.showMessage("错误: 计算TCP或新的O点位置时出错。")

    def create_cur_tcp_group(self, layout):
        """新增：创建用于显示当前TCP坐标的Groupbox。"""
        group = QGroupBox("当前TCP设置")
        
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
        self.read_cur_tcp_btn = QPushButton("读取Cur TCP")
        self.read_tcp_o_btn = QPushButton("读取TCP_O")
        self.read_tcp_tip_btn = QPushButton("读取TCP_tip")
        
        button_layout.addWidget(self.read_cur_tcp_btn)
        button_layout.addWidget(self.read_tcp_o_btn)
        button_layout.addWidget(self.read_tcp_tip_btn)
        
        main_v_layout.addLayout(button_layout)
        
        layout.addWidget(group)

    def create_tcp_group(self, layout):
        """创建TCP连接和消息收发模块。"""
        group = QGroupBox("TCP通信模块")
        group_layout = QVBoxLayout(group)
        # IP和端口输入框布局。
        ip_port_layout = QHBoxLayout()
        ip_port_layout.addWidget(QLabel("远程IP:"))
        self.ip_entry = QLineEdit("192.168.10.10")
        self.ip_entry.setFixedWidth(200)
        ip_port_layout.addWidget(self.ip_entry)
        ip_port_layout.addWidget(QLabel("端口:"))
        self.port_entry = QLineEdit("10003")
        self.port_entry.setFixedWidth(80)
        ip_port_layout.addWidget(self.port_entry)
        ip_port_layout.addStretch()
        group_layout.addLayout(ip_port_layout)
        # 连接和断开按钮布局。
        btn_layout = QHBoxLayout()
        self.connect_button = QPushButton("连接")
        self.disconnect_button = QPushButton("断开")
        self.disconnect_button.setEnabled(False)
        btn_layout.addWidget(self.connect_button)
        btn_layout.addWidget(self.disconnect_button)
        group_layout.addLayout(btn_layout)
        # 连接状态标签。
        self.tcp_status_label = QLabel("TCP状态: 未连接")
        self.tcp_status_label.setStyleSheet("color: blue;")
        group_layout.addWidget(self.tcp_status_label)
        # 接收消息文本框。
        group_layout.addWidget(QLabel("接收消息:"))
        self.recv_text = QTextEdit()
        self.recv_text.setReadOnly(True)  # 设置为只读，用户无法编辑。
        self.recv_text.setStyleSheet("background-color: lightgrey;")
        group_layout.addWidget(self.recv_text) # Removed the index here
        # 发送消息输入框和按钮。
        group_layout.addWidget(QLabel("发送消息:"))
        send_layout = QHBoxLayout()
        self.send_entry = QLineEdit()
        self.send_entry.setEnabled(False)
        self.send_button = QPushButton("发送")
        self.send_button.setEnabled(False)
        send_layout.addWidget(self.send_entry)
        send_layout.addWidget(self.send_button)
        group_layout.addLayout(send_layout)
        
        # Corrected line to pass the widget directly
        group_layout.setStretchFactor(self.recv_text, 1)
        
        layout.addWidget(group)
        
    def set_a_point_manually(self):
        """从A点文本框中读取坐标，并更新状态栏（确认手动输入）。"""
        try:
            x = float(self.a_vars[0].text())
            y = float(self.a_vars[1].text())
            z = float(self.a_vars[2].text())
            self.status_bar.showMessage(f"状态: 已人工输入A点位置 ({x:.2f}, {y:.2f}, {z:.2f})。")
        except ValueError:
            QMessageBox.critical(self, "输入错误", "A点坐标必须是有效的数字！")
            self.status_bar.showMessage("错误: A点坐标必须是有效的数字。")
            
    def set_o_point_manually(self):
        """从O点文本框中读取坐标，并更新状态栏（确认手动输入）。"""
        try:
            x = float(self.o_vars[0].text())
            y = float(self.o_vars[1].text())
            z = float(self.o_vars[2].text())
            self.status_bar.showMessage(f"状态: 已人工输入O点位置 ({x:.2f}, {y:.2f}, {z:.2f})。")
        except ValueError:
            QMessageBox.critical(self, "输入错误", "O点坐标必须是有效的数字！")
            self.status_bar.showMessage("错误: O点坐标必须是有效的数字。")
            
    def set_e_point_manually(self):
        """从End-Effect点文本框中读取坐标，并更新状态栏（确认手动输入）。"""
        try:
            x = float(self.e_vars[0].text())
            y = float(self.e_vars[1].text())
            z = float(self.e_vars[2].text())
            self.status_bar.showMessage(f"状态: 已人工输入End-Effect位置 ({x:.2f}, {y:.2f}, {z:.2f})。")
        except ValueError:
            QMessageBox.critical(self, "输入错误", "End-Effect点坐标必须是有效的数字！")
            self.status_bar.showMessage("错误: End-Effect点坐标必须是有效的数字。")
    
    def create_data_persistence_group(self, layout):
        """新增：创建用于保存和读取当前数据的Group。"""
        group = QGroupBox("保存当前数据")
        group_layout = QHBoxLayout(group)
        
        self.save_data_btn = QPushButton("保存")
        self.load_data_btn = QPushButton("读取")
        
        group_layout.addStretch()
        group_layout.addWidget(self.save_data_btn)
        group_layout.addWidget(self.load_data_btn)
        group_layout.addStretch()
        
        layout.addWidget(group)
            
    def set_tool_pose_manually(self):
        """
        从工具端位姿输入框中读取坐标，并更新内部状态 self.latest_tool_pose。
        """
        try:
            # 从可编辑的 QLineEdit 中读取值
            x = float(self.tool_pose_labels["Tcp_X"].text())
            y = float(self.tool_pose_labels["Tcp_Y"].text())
            z = float(self.tool_pose_labels["Tcp_Z"].text())
            rx = float(self.tool_pose_labels["Tcp_Rx"].text())
            ry = float(self.tool_pose_labels["Tcp_Ry"].text())
            rz = float(self.tool_pose_labels["Tcp_Rz"].text())
            
            new_pose = [x, y, z, rx, ry, rz]
            
            # 更新内部状态，用于后续的 AOE 平面等计算
            self.latest_tool_pose = new_pose
            
            self.status_bar.showMessage(f"状态: 已人工输入工具端位姿 ({x:.2f}, {y:.2f}, {z:.2f}, {rx:.2f}, {ry:.2f}, {rz:.2f})，可用于后续计算。")
        except ValueError:
            QMessageBox.critical(self, "输入错误", "工具端位姿坐标必须是有效的数字！")
            self.status_bar.showMessage("错误: 工具端位姿坐标必须是有效的数字。")
            
    def _get_ui_values(self, var_list):
        """Helper to get float values from a list of QLineEdits."""
        values = []
        for var in var_list:
            try:
                values.append(float(var.text()))
            except ValueError:
                return None
        return values
        
    def _set_ui_values(self, var_list, data_list):
        """Helper to set QLineEdit values from a list of floats."""
        for var, data in zip(var_list, data_list):
            var.setText(f"{data:.2f}")
            
    def save_data(self):
        """保存当前的工具端、A/O/E/B点数据到JSON文件。"""
        # 1. 收集数据
        try:
            data = {}
            
            # Tool End-Effector Status (Tcp_X ~ Tcp_Rz)
            tool_keys = ["Tcp_X", "Tcp_Y", "Tcp_Z", "Tcp_Rx", "Tcp_Ry", "Tcp_Rz"]
            tool_vars = [self.tool_pose_labels[k] for k in tool_keys]
            tool_pose = self._get_ui_values(tool_vars)
            if tool_pose is None:
                raise ValueError("工具端姿态数据无效，请确保所有字段为数字。")
            data['tool_pose'] = tool_pose

            # A, O, E, B points
            data['a_point'] = self._get_ui_values(self.a_vars)
            data['o_point'] = self._get_ui_values(self.o_vars)
            data['e_point'] = self._get_ui_values(self.e_vars)
            data['b_point'] = self._get_ui_values(self.b_vars)

            if any(v is None for v in [data['a_point'], data['o_point'], data['e_point'], data['b_point']]):
                 raise ValueError("A/O/E/B点数据中存在无效数字。")

        except ValueError as e:
            QMessageBox.critical(self, "保存错误", f"数据收集失败: {e}")
            return
        
        # 2. 写入文件
        try:
            with open(DATA_FILE_NAME, 'w') as f:
                json.dump(data, f, indent=4)
            self.status_bar.showMessage(f"状态: 数据已成功保存到 {DATA_FILE_NAME}。")
            QMessageBox.information(self, "保存成功", f"当前数据已保存到文件：{DATA_FILE_NAME}")
        except Exception as e:
            QMessageBox.critical(self, "文件写入错误", f"保存数据到文件时失败: {e}")


    def load_data(self):
        """从JSON文件读取数据，并恢复到当前的工具端、A/O/E/B点输入框。"""
        if not os.path.exists(DATA_FILE_NAME):
            QMessageBox.warning(self, "读取失败", f"未找到数据文件: {DATA_FILE_NAME}")
            self.status_bar.showMessage("状态: 未找到上次保存的数据文件。")
            return
        
        # 1. 读取文件
        try:
            with open(DATA_FILE_NAME, 'r') as f:
                data = json.load(f)
        except Exception as e:
            QMessageBox.critical(self, "文件读取错误", f"读取数据文件时失败: {e}")
            return

        # 2. 恢复数据到UI
        try:
            # Tool End-Effector Status
            tool_keys = ["Tcp_X", "Tcp_Y", "Tcp_Z", "Tcp_Rx", "Tcp_Ry", "Tcp_Rz"]
            tool_vars = [self.tool_pose_labels[k] for k in tool_keys]
            # 使用 .get 确保即使文件缺少键，也能使用默认值
            tool_pose_data = data.get('tool_pose', [0.0]*6) 
            self._set_ui_values(tool_vars, tool_pose_data)
            
            # 更新内部 self.latest_tool_pose (用于计算)
            self.latest_tool_pose = tool_pose_data
            
            # A, O, E, B points
            self._set_ui_values(self.a_vars, data.get('a_point', [0.0]*3))
            self._set_ui_values(self.o_vars, data.get('o_point', [0.0]*3))
            self._set_ui_values(self.e_vars, data.get('e_point', [0.0]*3))
            
            b_point_data = data.get('b_point', [0.0]*3)
            self._set_ui_values(self.b_vars, b_point_data)
            
            # 更新内部 self.b_point_position (需要 np.array)
            self.b_point_position = np.array(b_point_data)

            self.status_bar.showMessage(f"状态: 数据已从 {DATA_FILE_NAME} 恢复。")
            QMessageBox.information(self, "读取成功", "上次保存的数据已恢复。")
            
        except Exception as e:
            QMessageBox.critical(self, "数据恢复错误", f"恢复UI数据时失败: {e}")