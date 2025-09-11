# ui/main_window.py
import sys
import numpy as np
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QGroupBox, QLabel, QLineEdit, QPushButton,
                             QTextEdit, QStatusBar, QGridLayout, QMessageBox,
                             QCheckBox, QSlider)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QCloseEvent, QFont
from core.tcp_manager import TCPManager

class RobotControlWindow(QMainWindow):
    """
    此类继承自 QMainWindow，是整个应用程序的图形界面。
    它负责构建UI，并与后台的 TCPManager 进行交互。
    """
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
        self._moving_direction = 0     # 移动方向，1代表正向，-1代表反向。
        self._moving_tcp_index = -1    # 正在连续微调的TCP坐标索引。
        
        # 新增用于记录A点和O点坐标的文本框列表
        self.a_vars = [None] * 3
        self.o_vars = [None] * 3
        self.e_vars = [None] * 3 # 新增 end-effect 变量列表
        
        # 新增一个成员变量来存储最新的工具端姿态
        self.latest_tool_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # QTimer 用于实现按住按钮连续微调关节的功能。
        self.continuous_move_timer = QTimer(self)
        # 将定时器的超时信号连接到 continuous_move 方法。
        self.continuous_move_timer.timeout.connect(self.continuous_move)
        # QTimer 用于实现按住按钮连续微调TCP的功能。
        self.continuous_tcp_move_timer = QTimer(self)
        self.continuous_tcp_move_timer.timeout.connect(self.continuous_tcp_move)
        
        # 一个列表，用于存储用于设置TCP参数的 QLineEdit 控件的引用。
        self.tcp_input_entries = []

        # --- 2. 初始化业务逻辑变量 ---
        self.power_state = 0  # 机器人的电源状态：0为未上电，1为已上电。
        self.enable_state = 0 # 机器人的使能状态：0为未使能，1为已使能。
        
        # 实例化 TCPManager 类，所有通信逻辑都通过它来调用。
        self.tcp_manager = TCPManager()

        # --- 3. 初始化UI ---
        self.init_ui()
        # --- 4. 连接信号和槽 ---
        self.setup_connections()

    def init_ui(self):
        """构建所有UI组件并使用布局管理器进行排列。"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QHBoxLayout(central_widget)

        # 左侧面板布局，包含电机控制和状态显示模块。
        left_panel_layout = QVBoxLayout()
        
        # 新增：一个水平布局容器来容纳电机微调和运动速率模块
        top_left_layout = QHBoxLayout()
        self.create_motor_group(top_left_layout)
        self.create_override_group(top_left_layout) # 新增：运动速率分组框
        
        left_panel_layout.addLayout(top_left_layout)
        self.create_tool_state_group(left_panel_layout)
        self.create_record_oae_state_group(left_panel_layout)
        
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
        self.set_tcp_btn.clicked.connect(self.send_set_tcp_command)
        self.read_cur_tcp_btn.clicked.connect(self.request_cur_tcp_info)
        self.get_suitable_tcp_btn.clicked.connect(self.get_suitable_tcp)

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
        elif message.startswith("ReadCurTCP"):
            self.handle_read_tcp_message(message)
        elif message.startswith("ReadEmergencyInfo"):
            self.handle_emergency_info_message(message)
        elif message.startswith("ReadOverride"):
            self.handle_override_message(message)

    def request_cur_tcp_info(self):
        """通过按钮点击发送指令，请求获取当前TCP坐标。"""
        self.tcp_manager.send_command("ReadCurTCP,0;")

    def send_set_tcp_command(self):
        """
        根据用户在输入框中输入的六个参数，格式化并发送设置TCP的指令。
        指令格式：SetCurTCP,0,Tcp_X,Tcp_Y,Tcp_Z,Tcp_Rx,Tcp_Ry,Tcp_Rz;
        """
        try:
            # 尝试将每个输入框的文本转换为浮点数。
            params = [float(entry.text()) for entry in self.tcp_input_entries]
            tcp_x, tcp_y, tcp_z, tcp_rx, tcp_ry, tcp_rz = params

            # 格式化指令字符串，并保留两位小数。
            command = f"SetCurTCP,0,{tcp_x:.2f},{tcp_y:.2f},{tcp_z:.2f},{tcp_rx:.2f},{tcp_ry:.2f},{tcp_rz:.2f};"
            self.tcp_manager.send_command(command)
            self.status_bar.showMessage("状态: TCP参数已发送。")
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
        
    def handle_read_tcp_message(self, message):
        """解析 'ReadCurTCP' 消息，并更新新的TCP显示框。"""
        self.log_message(message)
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
        """发送 JOG 指令，实现单个关节的增量运动。"""
        # JOG 指令格式: JOG,nRbtID,nAxisID,dDeltaVal;
        # dDeltaVal 是以弧度为单位的增量值。
        step_rad = np.deg2rad(1.0)  # 设置每步为1度。
        command = f"JOG,0,{joint_index+1},{direction*step_rad:.4f};"
        self.tcp_manager.send_command(command)
        self.status_bar.showMessage(f"状态: 关节{joint_index+1} 微调中...")

    def start_move(self, joint_index, direction):
        """开始连续微调关节运动，启动定时器。"""
        self._moving_joint_index = joint_index
        self._moving_direction = direction
        self.motor_adjust(joint_index, direction)
        self.continuous_move_timer.start(50)  # 每50毫秒调用一次 continuous_move。

    def stop_move(self):
        """停止连续微调关节运动，停止定时器并发送停止指令。"""
        self.continuous_move_timer.stop()
        self._moving_joint_index = -1
        self._moving_direction = 0
        self.tcp_manager.send_command("StopJOG;")
        self.status_bar.showMessage("状态: 微调停止")

    def continuous_move(self):
        """由定时器周期性调用的函数，用于持续微调关节。"""
        if self._moving_joint_index != -1:
            self.motor_adjust(self._moving_joint_index, self._moving_direction)

    def tcp_adjust(self, tcp_index, direction):
        """发送 MoveRelL 指令，实现TCP坐标的增量运动。"""
        # MoveRelL 指令格式: MoveRelL,nRbtID,dX,dY,dZ,dRx,dRy,dRz;
        # dX, dY, dZ, dRx, dRy, dRz 都是增量值（mm或弧度）。
        if tcp_index < 3:  # X, Y, Z
            step = 1.0 * direction  # 1mm的步长。
            command_template = "MoveRelL,0,{:.2f},{:.2f},{:.2f},0,0,0;"
            values = [0.0, 0.0, 0.0]
            values[tcp_index] = step
            command = command_template.format(values[0], values[1], values[2])
        else:  # Rx, Ry, Rz
            step = np.deg2rad(1.0) * direction  # 1度的步长，转换为弧度。
            command_template = "MoveRelL,0,0,0,0,{:.4f},{:.4f},{:.4f};"
            values = [0.0, 0.0, 0.0]
            values[tcp_index - 3] = step
            command = command_template.format(values[0], values[1], values[2])

        self.tcp_manager.send_command(command)
        self.status_bar.showMessage(f"状态: Tcp_{['X','Y','Z','Rx','Ry','Rz'][tcp_index]} 微调中...")

    def start_tcp_move(self, tcp_index, direction):
        """开始连续微调TCP坐标，启动定时器。"""
        self._moving_tcp_index = tcp_index
        self._moving_direction = direction
        self.tcp_adjust(tcp_index, direction)
        self.continuous_tcp_move_timer.start(50)

    def stop_tcp_move(self,):
        """停止连续微调TCP坐标，停止定时器并发送停止指令。"""
        self.continuous_tcp_move_timer.stop()
        self._moving_tcp_index = -1
        self._moving_direction = 0
        self.tcp_manager.send_command("StopRelL;")
        self.status_bar.showMessage("状态: TCP微调停止")
    
    def continuous_tcp_move(self):
        """由定时器周期性调用的函数，用于持续微调TCP坐标。"""
        if self._moving_tcp_index != -1:
            self.tcp_adjust(self._moving_tcp_index, self._moving_direction)

    def closeEvent(self, a0: QCloseEvent):
        """重写窗口关闭事件，确保在程序退出前断开TCP连接并清理资源。"""
        self.tcp_manager.disconnect()
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
            label = QLabel(f"关节 {i+1} (q{i+1}):")
            value_label = QLabel("0.00")
            value_label.setFixedWidth(70)
            value_label.setAlignment(Qt.AlignRight)
            self.joint_vars[i] = value_label  # 存储引用，以便后续更新显示。
            # 创建并连接“微调减小”按钮。
            btn_minus = QPushButton("-")
            btn_minus.setFixedWidth(30)
            # 使用 lambda 表达式来传递参数 (关节索引和方向)。
            # pressed 信号在按钮被按下时立即触发。
            btn_minus.pressed.connect(lambda j=i: self.start_move(j, -1))
            # released 信号在按钮被释放时触发。
            btn_minus.released.connect(self.stop_move)
            # 创建并连接“微调增加”按钮。
            btn_plus = QPushButton("+")
            btn_plus.setFixedWidth(30)
            btn_plus.pressed.connect(lambda j=i: self.start_move(j, 1))
            btn_plus.released.connect(self.stop_move)
            row_layout.addWidget(label)
            row_layout.addWidget(value_label)
            row_layout.addSpacing(10)
            row_layout.addWidget(btn_minus)
            row_layout.addWidget(btn_plus)
            row_layout.addStretch()
            group_layout.addLayout(row_layout)
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
        """创建用于显示机器人工具端实时状态并包含微调按钮的模块。"""
        group = QGroupBox("机器人工具端实时状态")
        group_layout = QGridLayout(group)
        self.tool_pose_labels = {}
        # 定义工具端姿态参数。
        tool_pose_info = [
            ("Tcp_X", "Tcp_X"), ("Tcp_Y", "Tcp_Y"), ("Tcp_Z", "Tcp_Z"),
            ("Tcp_Rx", "Tcp_Rx"), ("Tcp_Ry", "Tcp_Ry"), ("Tcp_Rz", "Tcp_Rz")
        ]
        for i, (label_text, var_key) in enumerate(tool_pose_info):
            row = 0 if i < 3 else 2
            col = i % 3 * 2
            # 添加标签和显示值。
            label = QLabel(label_text)
            value_label = QLabel("0.00")
            value_label.setFixedWidth(70)
            value_label.setAlignment(Qt.AlignRight)
            value_label.setStyleSheet("background-color: lightgrey; border: 1px inset grey;")
            self.tool_pose_labels[var_key] = value_label
            # 添加微调减小按钮。
            btn_minus = QPushButton("-")
            btn_minus.setFixedWidth(30)
            btn_minus.pressed.connect(lambda idx=i: self.start_tcp_move(idx, -1))
            btn_minus.released.connect(self.stop_tcp_move)
            # 添加微调增加按钮。
            btn_plus = QPushButton("+")
            btn_plus.setFixedWidth(30)
            btn_plus.pressed.connect(lambda idx=i: self.start_tcp_move(idx, 1))
            btn_plus.released.connect(self.stop_tcp_move)
            btn_layout = QHBoxLayout()
            btn_layout.addStretch()
            btn_layout.addWidget(btn_minus)
            btn_layout.addWidget(btn_plus)
            btn_layout.addStretch()
            # 将控件添加到网格布局。
            group_layout.addWidget(label, row, col, alignment=Qt.AlignCenter)
            group_layout.addWidget(value_label, row, col + 1, alignment=Qt.AlignRight)
            group_layout.addLayout(btn_layout, row + 1, col, 1, 2)
        layout.addWidget(group)
        
    def create_record_oae_state_group(self, layout):
        """创建用于记录机器人A点、O点和end-effect点的模块。"""
        group = QGroupBox("机器人A、O和End-Effect位置")
        group_layout = QVBoxLayout(group)

        # A点布局
        a_point_subgroup = QGroupBox("A点")
        a_point_layout = QGridLayout(a_point_subgroup)
        a_labels = ["A_x:", "A_y:", "A_z:"]
        for i, label_text in enumerate(a_labels):
            label = QLabel(label_text)
            text_box = QLineEdit("0.00")
            text_box.setReadOnly(True)
            text_box.setStyleSheet("background-color: lightgrey;")
            self.a_vars[i] = text_box
            a_point_layout.addWidget(label, 0, i * 2)
            a_point_layout.addWidget(text_box, 0, i * 2 + 1)
        get_a_btn = QPushButton("获取A点位置")
        a_point_layout.addWidget(get_a_btn, 1, 0, 1, 6, alignment=Qt.AlignCenter)
        group_layout.addWidget(a_point_subgroup)

        # O点布局
        o_point_subgroup = QGroupBox("O点")
        o_point_layout = QGridLayout(o_point_subgroup)
        o_labels = ["O_x:", "O_y:", "O_z:"]
        for i, label_text in enumerate(o_labels):
            label = QLabel(label_text)
            text_box = QLineEdit("0.00")
            text_box.setReadOnly(True)
            text_box.setStyleSheet("background-color: lightgrey;")
            self.o_vars[i] = text_box
            o_point_layout.addWidget(label, 0, i * 2)
            o_point_layout.addWidget(text_box, 0, i * 2 + 1)
        get_o_btn = QPushButton("获取O点位置")
        o_point_layout.addWidget(get_o_btn, 1, 0, 1, 6, alignment=Qt.AlignCenter)
        group_layout.addWidget(o_point_subgroup)
        
        # End-Effect点布局
        e_point_subgroup = QGroupBox("End-Effect")
        e_point_layout = QGridLayout(e_point_subgroup)
        e_labels = ["E_x:", "E_y:", "E_z:"]
        for i, label_text in enumerate(e_labels):
            label = QLabel(label_text)
            text_box = QLineEdit("0.00")
            text_box.setReadOnly(True)
            text_box.setStyleSheet("background-color: lightgrey;")
            self.e_vars[i] = text_box
            e_point_layout.addWidget(label, 0, i * 2)
            e_point_layout.addWidget(text_box, 0, i * 2 + 1)
        get_e_btn = QPushButton("获取End-Effect位置")
        e_point_layout.addWidget(get_e_btn, 1, 0, 1, 6, alignment=Qt.AlignCenter)
        group_layout.addWidget(e_point_subgroup)

        # 连接按钮的点击事件
        get_a_btn.clicked.connect(self.get_a_point_position)
        get_o_btn.clicked.connect(self.get_o_point_position)
        get_e_btn.clicked.connect(self.get_e_point_position) # 连接新按钮

        group_layout.addStretch()
        layout.addWidget(group)

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
        """新增：将工具坐标系设置移出并做成独立的Groupbox。"""
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
        
        # 添加可拉伸的弹簧，将按钮推到底部
        main_v_layout.addStretch()

        # 创建一个水平布局来放置并居中按钮
        button_layout = QHBoxLayout()
        self.set_tcp_btn = QPushButton("设置TCP")
        self.get_suitable_tcp_btn = QPushButton("获取合适的TCP")
        button_layout.addWidget(self.set_tcp_btn)
        button_layout.addWidget(self.get_suitable_tcp_btn)
        
        # 将按钮布局添加到主垂直布局的底部
        main_v_layout.addLayout(button_layout)
        
        layout.addWidget(tcp_group)
        
    def get_suitable_tcp(self):
        """
        计算O点和End-effect点之间的距离，并设置到Tcp_Z。
        同时将其他五个TCP值设置成0。
        """
        # 1. 检查A点和O点是否有数据
        try:
            o_point = np.array([float(self.o_vars[i].text()) for i in range(3)])
            e_point = np.array([float(self.e_vars[i].text()) for i in range(3)])
        except (ValueError, IndexError):
            self.status_bar.showMessage("警告: 请先获取O点和End-effect点坐标。")
            return

        # 2. 计算两点之间的欧几里得距离
        distance = np.linalg.norm(e_point - o_point)
        self.status_bar.showMessage(f"状态: 计算出的O点和End-effect点距离为 {distance:.2f}。")

        # 3. 更新TCP设置输入框
        # 将Tcp_Z设置为计算出的距离
        self.tcp_input_entries[2].setText(f"{distance:.2f}")

        # 将其他五个值设置为0
        for i in range(len(self.tcp_input_entries)):
            if i != 2: # 索引2是Tcp_Z
                self.tcp_input_entries[i].setText("0.00")
        
        QMessageBox.information(self, "操作成功", f"TCP_Z值已设置为 {distance:.2f}。\n请点击“设置TCP”按钮以应用更改。")

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
        
        # 添加可拉伸的弹簧
        main_v_layout.addStretch()
        
        # 创建按钮并将其居中
        button_layout = QHBoxLayout()
        self.read_cur_tcp_btn = QPushButton("读取Cur TCP")
        button_layout.addStretch()
        button_layout.addWidget(self.read_cur_tcp_btn)
        button_layout.addStretch()
        
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
        group_layout.addWidget(self.recv_text, 1)
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
        group_layout.setStretch(3, 1) # 让接收消息文本框可以随窗口拉伸。
        layout.addWidget(group)