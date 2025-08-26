import sys
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QGroupBox, QLabel, QLineEdit,
                             QPushButton, QTextEdit, QStatusBar, QGridLayout,
                             QMessageBox, QCheckBox, QSlider)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QFont, QCloseEvent
import socket
import threading
import time

# --- TCP 消息接收线程 ---
class ReceiveThread(QThread):
    message_received = pyqtSignal(str)
    connection_lost = pyqtSignal()

    def __init__(self, client_socket, parent=None):
        super().__init__(parent)
        self.client_socket = client_socket
        self.is_running = True

    def run(self):
        while self.is_running:
            try:
                data = self.client_socket.recv(1024)
                if not data:
                    self.connection_lost.emit()
                    break
                message = data.decode('utf-8')
                self.message_received.emit(message)
            except (ConnectionResetError, socket.error) as e:
                self.connection_lost.emit()
                break
            except Exception as e:
                if self.is_running:
                    print(f"接收消息线程出错: {e}")
                    self.connection_lost.emit()
                break
        print("接收消息线程已退出。")

    def stop(self):
        self.is_running = False
        self.wait() # 确保线程安全退出

class RobotControlWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RRRRRR 机器人高级控制界面 (PyQt5版)")
        self.setGeometry(100, 100, 900, 700) # 调整窗口大小以容纳新模块

        # --- 1. 初始化GUI所需的关节显示变量 ---
        self.num_joints = 6
        self.joint_vars = [None] * self.num_joints
        self._moving_joint_index = -1
        self._moving_direction = 0
        self.continuous_move_timer = QTimer(self)
        self.continuous_move_timer.timeout.connect(self.continuous_move)
        self.tcp_input_entries = [] # 新增：用于存储TCP设置的输入框

        # --- 2. 初始化 TCP 变量 ---
        self.is_connected = False
        self.client_socket = None
        self.receive_thread = None
        self.power_state = 0 # 0: 未上电, 1: 已上电
        self.enable_state = 0 # 0: 未使能, 1: 已使能
        self.real_time_update_timer = QTimer(self)
        self.real_time_update_timer.timeout.connect(self.request_real_time_data)

        # --- 3. 初始化UI ---
        self.init_ui()

    def init_ui(self):
        """创建所有UI组件和布局"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QHBoxLayout(central_widget)

        # 左侧面板
        left_panel_layout = QVBoxLayout()
        self.create_motor_group(left_panel_layout)
        # 逆运动学模块已移除
        self.create_state_display_group(left_panel_layout)
        self.create_tool_state_group(left_panel_layout) # 新增：机器人工具端实时状态
        main_layout.addLayout(left_panel_layout, 1)

        # 右侧面板
        right_panel_layout = QVBoxLayout()
        self.create_ur_control_group(right_panel_layout) # 新增 UR 控制模块
        self.create_tcp_group(right_panel_layout)
        main_layout.addLayout(right_panel_layout, 1)

        # 状态栏
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("状态: 准备就绪")

    def create_motor_group(self, layout):
        """创建左侧的电机控制模块"""
        group = QGroupBox("电机微调模块 (正运动学)")
        group_layout = QHBoxLayout(group)
        left_rows_layout = QVBoxLayout()

        for i in range(self.num_joints):
            row_layout = QHBoxLayout()
            label = QLabel(f"关节 {i+1} (q{i+1}):")

            value_label = QLabel("0.00")
            value_label.setFixedWidth(50)
            value_label.setAlignment(Qt.AlignRight)
            self.joint_vars[i] = value_label

            btn_minus = QPushButton("-")
            btn_minus.setFixedWidth(30)
            btn_minus.clicked.connect(lambda _, j=i: self.send_joint_command(j, 0)) # 负方向
            
            btn_plus = QPushButton("+")
            btn_plus.setFixedWidth(30)
            btn_plus.clicked.connect(lambda _, j=i: self.send_joint_command(j, 1)) # 正方向

            row_layout.addWidget(label)
            row_layout.addWidget(value_label)
            row_layout.addSpacing(10)
            row_layout.addWidget(btn_minus)
            row_layout.addWidget(btn_plus)
            row_layout.addStretch()
            left_rows_layout.addLayout(row_layout)

        left_rows_layout.addStretch()

        # 在右侧预留区域添加运动速率滑条（对应图片中红框）
        slider_container = QVBoxLayout()
        slider_container.addStretch()

        self.override_label = QLabel("运动速率: 0.01")
        self.override_label.setAlignment(Qt.AlignCenter)

        self.override_slider = QSlider(Qt.Vertical)
        self.override_slider.setMinimum(1)      # 0.01
        self.override_slider.setMaximum(100)    # 1.00
        self.override_slider.setSingleStep(1)
        self.override_slider.setPageStep(5)
        self.override_slider.setValue(1)
        self.override_slider.setTickPosition(QSlider.TicksBothSides)
        self.override_slider.setTickInterval(10)
        self.override_slider.valueChanged.connect(self._on_override_slider_changed)
        self.override_slider.sliderReleased.connect(self._on_override_slider_released)

        slider_container.addWidget(self.override_label)
        slider_container.addWidget(self.override_slider)
        slider_container.addStretch()

        group_layout.addLayout(left_rows_layout, 1)
        group_layout.addLayout(slider_container)

        layout.addWidget(group)

    # 逆运动学模块已移除

    def _on_override_slider_changed(self, value):
        """滑条数值改变时，仅更新标签显示，不立即发指令。"""
        override_value = value / 100.0
        self.override_label.setText(f"运动速率: {override_value:.2f}")

    def _on_override_slider_released(self):
        """滑条释放时发送运动速率指令: SetOverride,nRbtID,dOverride;"""
        d_override = self.override_slider.value() / 100.0
        # 约定使用 0 号机器人ID
        command = f"SetOverride,0,{d_override:.2f};"
        try:
            self.send_ur_command(command)
            self.status_bar.showMessage(f"状态: 已设置运动速率为 {d_override:.2f}")
        except Exception:
            pass

    def create_state_display_group(self, layout):
        """创建底部用于显示实时状态的模块"""
        group = QGroupBox("机器人末端实时状态")
        group_layout = QGridLayout(group)

        self.pose_labels = {}
        # 调整布局以匹配图片
        pose_info = [
            ("X (mm)", "X"), ("Y (mm)", "Y"), ("Z (mm)", "Z"),
            ("Roll (°)", "Roll"), ("Pitch (°)", "Pitch"), ("Yaw (°)", "Yaw")
        ]

        # 将X,Y,Z放在第一行，Roll,Pitch,Yaw放在第二行
        for i, (label_text, var_key) in enumerate(pose_info):
            row = 0 if i < 3 else 1
            col = i % 3 * 2

            label = QLabel(label_text)
            value_label = QLabel("0.00")
            value_label.setFixedWidth(70)
            value_label.setAlignment(Qt.AlignRight)
            value_label.setStyleSheet("background-color: lightgrey; border: 1px inset grey;")
            self.pose_labels[var_key] = value_label

            group_layout.addWidget(label, row, col)
            group_layout.addWidget(value_label, row, col + 1)

        group_layout.setRowStretch(2, 1) # 撑开底部空间
        layout.addWidget(group)

    def create_tool_state_group(self, layout):
        """新增：创建用于显示机器人工具端实时状态的模块"""
        group = QGroupBox("机器人工具端实时状态")
        group_layout = QGridLayout(group)

        self.tool_pose_labels = {}
        tool_pose_info = [
            ("Tcp_X", "Tcp_X"), ("Tcp_Y", "Tcp_Y"), ("Tcp_Z", "Tcp_Z"),
            ("Tcp_Rx", "Tcp_Rx"), ("Tcp_Ry", "Tcp_Ry"), ("Tcp_Rz", "Tcp_Rz")
        ]

        # 将X,Y,Z放在第一行，Rx,Ry,Rz放在第二行
        for i, (label_text, var_key) in enumerate(tool_pose_info):
            row = 0 if i < 3 else 1
            col = i % 3 * 2

            label = QLabel(label_text)
            value_label = QLabel("0.00")
            value_label.setFixedWidth(70)
            value_label.setAlignment(Qt.AlignRight)
            value_label.setStyleSheet("background-color: lightgrey; border: 1px inset grey;")
            self.tool_pose_labels[var_key] = value_label

            group_layout.addWidget(label, row, col)
            group_layout.addWidget(value_label, row, col + 1)

        group_layout.setRowStretch(2, 1) # 撑开底部空间
        layout.addWidget(group)

    def create_ur_control_group(self, layout):
        """根据用户图片创建 UR 控制模块"""
        ur_control_group = QGroupBox("E05-L Pro设备控制")
        ur_control_layout = QVBoxLayout(ur_control_group)

        # 电源/使能/初始化按钮
        button_layout = QGridLayout()
        self.ur_power_btn = QPushButton("电源开启")
        self.ur_power_btn.clicked.connect(self.toggle_power)
        self.ur_enable_btn = QPushButton("使能")
        self.ur_enable_btn.clicked.connect(self.toggle_enable)
        self.ur_init_btn = QPushButton("初始化控制器")
        self.ur_init_btn.clicked.connect(self.send_ur_init_controller_command)
        button_layout.addWidget(self.ur_power_btn, 0, 0)
        button_layout.addWidget(self.ur_enable_btn, 0, 1)
        button_layout.addWidget(self.ur_init_btn, 1, 0, 1, 2)
        ur_control_layout.addLayout(button_layout)

        # 示教功能复选框
        self.teach_mode_checkbox = QCheckBox("示教功能")
        self.teach_mode_checkbox.stateChanged.connect(self.handle_teach_mode_state_change)
        ur_control_layout.addWidget(self.teach_mode_checkbox)

        # 工具坐标系设置 (新需求)
        tcp_group = QGroupBox("工具坐标系设置 (TCP)")
        tcp_grid_layout = QGridLayout(tcp_group)
        tcp_labels = ["Tcp_X", "Tcp_Y", "Tcp_Z", "Tcp_Rx", "Tcp_Ry", "Tcp_Rz"]
        tcp_initial_values = ["0", "0", "0", "0", "0", "0"]
        self.tcp_input_entries = []
        for i, label_text in enumerate(tcp_labels):
            row, col = i // 3, (i % 3) * 2
            label = QLabel(label_text + ":")
            entry = QLineEdit(tcp_initial_values[i])
            self.tcp_input_entries.append(entry)
            tcp_grid_layout.addWidget(label, row, col)
            tcp_grid_layout.addWidget(entry, row, col + 1)

        self.set_tcp_btn = QPushButton("设置TCP")
        self.set_tcp_btn.clicked.connect(self.send_set_tcp_command)
        tcp_grid_layout.addWidget(self.set_tcp_btn, 2, 0, 1, 2)

        ur_control_layout.addWidget(tcp_group)
        layout.addWidget(ur_control_group)

    def send_joint_command(self, joint_index, direction):
        """发送关节增量运动指令"""
        # nAxisId: 0-5 for joints 1-6
        # nDirection: 1 for positive, 0 for negative
        # 1: represents a single step, based on your request
        nAxisId = joint_index
        nDirection = direction
        
        command = f"MoveRelJ,0,{nAxisId},{nDirection},1;"
        self.send_ur_command(command)
        self.status_bar.showMessage(f"状态: 已发送关节 {joint_index + 1} 增量命令 ({'正向' if direction else '负向'})")

    def send_set_tcp_command(self):
        """
        根据用户输入的六个参数，发送设置TCP的指令。
        指令格式：SetCurTCP,0,Tcp_X,Tcp_Y,Tcp_Z,Tcp_Rx,Tcp_Ry,Tcp_Rz;
        """
        if not self.is_connected:
            QMessageBox.warning(self, "操作失败", "请先建立TCP连接。")
            return

        try:
            params = [float(entry.text()) for entry in self.tcp_input_entries]
            tcp_x, tcp_y, tcp_z, tcp_rx, tcp_ry, tcp_rz = params

            # 格式化指令字符串，保留两位小数
            command = f"SetCurTCP,0,{tcp_x:.2f},{tcp_y:.2f},{tcp_z:.2f},{tcp_rx:.2f},{tcp_ry:.2f},{tcp_rz:.2f};"
            self.send_ur_command(command)

            self.status_bar.showMessage("状态: TCP参数已发送。")

        except ValueError:
            QMessageBox.critical(self, "输入错误", "TCP参数必须是有效的数字！")
        except IndexError:
            QMessageBox.critical(self, "内部错误", "TCP参数输入框数量不正确。")


    def toggle_power(self):
        """根据当前电源状态切换电源开关"""
        if self.power_state == 0:
            # 未上电，发送上电指令
            self.send_ur_command("Electrify;")
            self.ur_power_btn.setText("电源开启中...")
            self.ur_power_btn.setEnabled(False)
        else:
            # 已上电，发送断电指令
            self.send_ur_command("OSCmd,1;")
            self.ur_power_btn.setText("断电中...")
            self.ur_power_btn.setEnabled(False)

    def toggle_enable(self):
        """根据当前使能状态切换使能/去使能"""
        if self.enable_state == 0:
            # 未使能，发送使能指令
            self.send_ur_command("GrpEnable,0;")
            self.ur_enable_btn.setText("使能中...")
            self.ur_enable_btn.setEnabled(False)
        else:
            # 已使能，发送去使能指令
            self.send_ur_command("GrpDisable,0;")
            self.ur_enable_btn.setText("去使能中...")
            self.ur_enable_btn.setEnabled(False)

    def handle_teach_mode_state_change(self, state):
        """处理示教功能复选框状态变化，并发送相应的TCP指令"""
        if state == Qt.Checked:
            # 勾选上时，发送开启示教功能指令
            command = "GrpOpenFreeDriver,0;"
            self.send_ur_command(command)
            self.status_bar.showMessage("状态: 示教功能已开启。")
        else:
            # 去掉勾选时，发送关闭示教功能指令
            command = "GrpCloseFreeDriver,0;"
            self.send_ur_command(command)
            self.status_bar.showMessage("状态: 示教功能已关闭。")

    def create_tcp_group(self, layout):
        """创建TCP连接和消息收发模块"""
        group = QGroupBox("TCP通信模块")
        group_layout = QVBoxLayout(group)

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

        btn_layout = QHBoxLayout()
        self.connect_button = QPushButton("连接")
        self.connect_button.clicked.connect(self.connect_tcp)
        self.disconnect_button = QPushButton("断开")
        self.disconnect_button.clicked.connect(self.disconnect_tcp)
        self.disconnect_button.setEnabled(False)
        btn_layout.addWidget(self.connect_button)
        btn_layout.addWidget(self.disconnect_button)
        group_layout.addLayout(btn_layout)

        self.tcp_status_label = QLabel("TCP状态: 未连接")
        self.tcp_status_label.setStyleSheet("color: blue;")
        group_layout.addWidget(self.tcp_status_label)

        group_layout.addWidget(QLabel("接收消息:"))
        self.recv_text = QTextEdit()
        self.recv_text.setReadOnly(True)
        self.recv_text.setStyleSheet("background-color: lightgrey;")
        group_layout.addWidget(self.recv_text, 1)

        group_layout.addWidget(QLabel("发送消息:"))
        send_layout = QHBoxLayout()
        self.send_entry = QLineEdit()
        self.send_entry.setEnabled(False)
        self.send_entry.returnPressed.connect(self.send_message)
        self.send_button = QPushButton("发送")
        self.send_button.clicked.connect(self.send_message)
        self.send_button.setEnabled(False)
        send_layout.addWidget(self.send_entry)
        send_layout.addWidget(self.send_button)
        group_layout.addLayout(send_layout)

        group_layout.setStretch(3, 1)
        layout.addWidget(group)

    def connect_tcp(self):
        """尝试建立TCP连接"""
        if self.is_connected:
            QMessageBox.information(self, "提示", "已连接，请勿重复操作。")
            return

        try:
            ip = self.ip_entry.text()
            port = int(self.port_entry.text())
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.settimeout(3)
            self.client_socket.connect((ip, port))
            self.client_socket.settimeout(None) # 连接成功后取消超时

            self.is_connected = True
            self.tcp_status_label.setText(f"TCP状态: 已连接到 {ip}:{port}")
            self.connect_button.setEnabled(False)
            self.disconnect_button.setEnabled(True)
            self.send_entry.setEnabled(True)
            self.send_button.setEnabled(True)
            self.log_message("系统: 连接成功！")

            self.receive_thread = ReceiveThread(self.client_socket)
            self.receive_thread.message_received.connect(self.handle_incoming_message)
            self.receive_thread.connection_lost.connect(self.handle_connection_lost)
            self.receive_thread.start()

            # 连接成功后，启动新的实时更新定时器
            self.real_time_update_timer.start(100)
            self.request_real_time_data() # 立即请求一次

        except socket.timeout:
            self.tcp_status_label.setText("TCP状态: 连接失败 (连接超时)")
            QMessageBox.critical(self, "连接错误", "连接超时，请检查IP和端口是否正确。")
        except socket.gaierror:
            self.tcp_status_label.setText("TCP状态: 连接失败 (IP地址错误)")
            QMessageBox.critical(self, "连接错误", "无法解析IP地址。")
        except ConnectionRefusedError:
            self.tcp_status_label.setText("TCP状态: 连接失败 (连接被拒绝)")
            QMessageBox.critical(self, "连接错误", "目标计算机拒绝连接，请检查IP和端口。")
        except ValueError:
            self.tcp_status_label.setText("TCP状态: 连接失败 (端口号错误)")
            QMessageBox.critical(self, "输入错误", "端口号必须是有效数字。")
        except Exception as e:
            self.tcp_status_label.setText(f"TCP状态: 连接失败 ({e})")
            QMessageBox.critical(self, "连接错误", f"发生未知错误: {e}")

    def disconnect_tcp(self):
        """断开TCP连接"""
        if self.is_connected:
            self.is_connected = False
            self.real_time_update_timer.stop() # 停止定时器
            if self.receive_thread and self.receive_thread.isRunning():
                self.receive_thread.stop()
            try:
                self.client_socket.close()
            except Exception as e:
                print(f"关闭套接字出错: {e}")

            self.tcp_status_label.setText("TCP状态: 已断开")
            self.connect_button.setEnabled(True)
            self.disconnect_button.setEnabled(False)
            self.send_entry.setEnabled(False)
            self.send_button.setEnabled(False)
            self.log_message("系统: 连接已断开。")

    def handle_connection_lost(self):
        """处理连接丢失信号"""
        if self.is_connected:
            self.log_message("系统: 连接被远程主机强制关闭或网络错误。")
            self.disconnect_tcp()

    def send_message(self):
        """发送消息"""
        if not self.is_connected:
            QMessageBox.warning(self, "发送失败", "请先建立TCP连接。")
            return
        message = self.send_entry.text()
        if not message:
            return
        try:
            self.client_socket.sendall(message.encode('utf-8'))
            self.log_message(f"发送: {message}")
            self.send_entry.clear()
        except Exception as e:
            self.log_message(f"错误: 发送失败 - {e}")
            self.disconnect_tcp()

    def handle_incoming_message(self, message):
        """分发接收到的消息"""
        self.log_message(message)
        if message.startswith("ReadActPos"):
            self.handle_real_time_message(message)
        elif message.startswith("ReadRobotState"):
            self.handle_robot_state_message(message)
        elif message.startswith("ReadCurTCP"):
            self.handle_read_tcp_message(message)

    def request_real_time_data(self):
        """发送指令以获取机器人实时关节和末端坐标"""
        self.send_ur_command("ReadActPos,0;")
        # 发送第二条指令，获取电源和使能状态
        self.send_ur_command("ReadRobotState,0;")


    def handle_real_time_message(self, message):
        """解析 ReadActPos 消息并更新关节和末端坐标显示"""
        parts = message.strip(';').split(',')
        if len(parts) >= 25 and parts[1] == 'OK':
            try:
                # 更新关节值
                joint_params = [float(p) for p in parts[2:8]]
                for i in range(len(joint_params)):
                    if i < len(self.joint_vars):
                        self.joint_vars[i].setText(f"{np.rad2deg(joint_params[i]):.2f}")

                # 更新机器人末端姿态
                pose_params = [float(p) for p in parts[8:14]]
                self.pose_labels["X"].setText(f"{pose_params[0]:.2f}")
                self.pose_labels["Y"].setText(f"{pose_params[1]:.2f}")
                self.pose_labels["Z"].setText(f"{pose_params[2]:.2f}")
                self.pose_labels["Roll"].setText(f"{pose_params[3]:.2f}")
                self.pose_labels["Pitch"].setText(f"{pose_params[4]:.2f}")
                self.pose_labels["Yaw"].setText(f"{pose_params[5]:.2f}")

                # 更新机器人工具端姿态
                tool_pose_params = [float(p) for p in parts[14:20]]
                self.tool_pose_labels["Tcp_X"].setText(f"{tool_pose_params[0]:.2f}")
                self.tool_pose_labels["Tcp_Y"].setText(f"{tool_pose_params[1]:.2f}")
                self.tool_pose_labels["Tcp_Z"].setText(f"{tool_pose_params[2]:.2f}")
                self.tool_pose_labels["Tcp_Rx"].setText(f"{tool_pose_params[3]:.2f}")
                self.tool_pose_labels["Tcp_Ry"].setText(f"{tool_pose_params[4]:.2f}")
                self.tool_pose_labels["Tcp_Rz"].setText(f"{tool_pose_params[5]:.2f}")


                self.status_bar.showMessage("状态: 关节、末端和工具端坐标已实时同步。")

            except (ValueError, IndexError):
                self.log_message("警告: 无法解析 ReadActPos 消息。")
        else:
            self.log_message(f"警告: 接收到无效的 ReadActPos 消息: {message}")


    def handle_read_tcp_message(self, message):
        """解析 ReadCurTCP 消息并更新TCP参数输入框"""
        parts = message.strip(';').split(',')
        if len(parts) == 8 and parts[1] == 'OK':
            try:
                tcp_params = [f"{float(p):.2f}" for p in parts[2:]]
                for i in range(len(tcp_params)):
                    if i < len(self.tcp_input_entries):
                        self.tcp_input_entries[i].setText(tcp_params[i])
                self.status_bar.showMessage("状态: TCP参数已从设备同步。")
            except (ValueError, IndexError):
                self.log_message("警告: 无法解析 ReadCurTCP 消息。")
        else:
            self.log_message(f"警告: 接收到无效的 ReadCurTCP 消息: {message}")

    def handle_robot_state_message(self, message):
        """解析 ReadRobotState 消息并更新UI"""
        parts = message.split(',')
        if len(parts) >= 14:
            try:
                nElectrify = int(parts[10])
                nEnableState = int(parts[2])
                self.update_power_button(nElectrify)
                self.update_enable_button(nEnableState)
            except (ValueError, IndexError):
                self.log_message("警告: 无法解析机器人状态消息中的上电/使能状态。")

    def update_power_button(self, nElectrify):
        """根据上电状态更新电源按钮的文本和使能状态"""
        self.power_state = nElectrify
        self.ur_power_btn.setEnabled(True)
        if nElectrify == 1:
            self.ur_power_btn.setText("电源断开")
            self.ur_power_btn.setStyleSheet("background-color: lightgreen;")
        else:
            self.ur_power_btn.setText("电源开启")
            self.ur_power_btn.setStyleSheet("background-color: salmon;")

    def update_enable_button(self, nEnableState):
        """根据使能状态更新使能按钮的文本和使能状态"""
        self.enable_state = nEnableState
        self.ur_enable_btn.setEnabled(True)
        if nEnableState == 1:
            self.ur_enable_btn.setText("去使能")
            self.ur_enable_btn.setStyleSheet("background-color: lightgreen;")
        else:
            self.ur_enable_btn.setText("使能")
            self.ur_enable_btn.setStyleSheet("background-color: salmon;")

    # --- 新增的 UR 控制功能函数 ---
    def send_ur_command(self, command):
        """发送 UR 控制指令"""
        if not self.is_connected:
            self.log_message("UR控制失败: 未建立TCP连接。")
            QMessageBox.warning(self, "操作失败", "请先建立TCP连接。")
            return
        try:
            # 确保命令以分号结尾
            if not command.endswith(';'):
                command += ';'
            self.client_socket.sendall(command.encode('utf-8'))
            self.log_message(f"发送 UR 指令: {command}")
        except Exception as e:
            self.log_message(f"UR指令发送失败: {e}")
            self.disconnect_tcp()

    def send_ur_init_controller_command(self):
        self.send_ur_command("StartMaster;")

    # --- 以下为原有代码，保持不变 ---
    def log_message(self, message):
        """向接收文本框中添加消息"""
        self.recv_text.append(message)
        self.recv_text.verticalScrollBar().setValue(self.recv_text.verticalScrollBar().maximum())

    # Forward kinematics and inverse kinematics related functions are removed.
    def motor_adjust(self, joint_index, direction):
        """微调单个关节角度，并发送命令"""
        # Note: This function now sends a command to the robot instead of updating a local model.
        step_rad = np.deg2rad(1.0)
        command = f"JOG,0,{joint_index+1},{direction*step_rad:.4f};"
        self.send_ur_command(command)
        self.status_bar.showMessage(f"状态: 关节{joint_index+1} 微调中...")

    def start_move(self, joint_index, direction):
        """开始连续微调"""
        self._moving_joint_index = joint_index
        self._moving_direction = direction
        self.motor_adjust(joint_index, direction)
        self.continuous_move_timer.start(50)

    def stop_move(self):
        """停止连续微调"""
        self.continuous_move_timer.stop()
        self._moving_joint_index = -1
        self._moving_direction = 0
        self.send_ur_command("StopJOG;")
        self.status_bar.showMessage("状态: 微调停止")

    def continuous_move(self):
        """连续微调函数，每隔一段时间调用一次自身"""
        if self._moving_joint_index != -1:
            self.motor_adjust(self._moving_joint_index, self._moving_direction)

    # Inverse kinematics related functions are removed.

    def closeEvent(self, a0: QCloseEvent):
        """重写关闭事件，执行清理工作"""
        self.disconnect_tcp()
        super().closeEvent(a0)

# --- 程序主入口 ---
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RobotControlWindow()
    window.show()
    sys.exit(app.exec_())