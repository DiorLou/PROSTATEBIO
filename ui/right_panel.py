from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel, QLineEdit, 
    QPushButton, QGridLayout, QCheckBox, QTextEdit, QMessageBox
)
from PyQt5.QtCore import QTimer, Qt
import numpy as np
import pytransform3d.rotations as pyrot

class RightPanel(QWidget):
    def __init__(self, tcp_manager, parent=None):
        super().__init__(parent)
        self.tcp_manager = tcp_manager
        self.main_window = parent
        
        self.tcp_input_entries = []
        self.cur_tcp_vars = [None] * 6
        self.current_tcp_name = "TCP_E"
        
        self.is_fine_tuning_process = False
        self.temp_expected_response = None

        self.init_ui()
        self.setup_connections()

    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # 1. 设备控制 (电源/急停)
        self.create_ur_control_group(layout)
        
        # 2. 示教模式
        self.create_teach_mode_group(layout)
        
        # 3. TCP 设置 (Tar TCP)
        self.create_set_tcp_group(layout)
        
        # 4. 当前 TCP 显示
        self.create_cur_tcp_group(layout)
        
        # 5. 通信模块
        self.create_tcp_group(layout)
        
        layout.addStretch()

    def create_ur_control_group(self, layout):
        group = QGroupBox("E05-L Pro Device Control")
        grid = QGridLayout(group)
        
        self.power_btn = QPushButton("Power On")
        self.enable_btn = QPushButton("Enable")
        self.init_btn = QPushButton("Initialize Controller")
        self.reset_btn = QPushButton("Reset")
        self.pause_btn = QPushButton("Pause")
        self.cont_btn = QPushButton("Continue")
        self.stop_btn = QPushButton("Emergency Stop")
        self.stop_btn.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        self.stop_btn.setEnabled(False)
        
        grid.addWidget(self.power_btn, 0, 0)
        grid.addWidget(self.enable_btn, 0, 1)
        grid.addWidget(self.init_btn, 1, 0)
        grid.addWidget(self.reset_btn, 1, 1)
        grid.addWidget(self.pause_btn, 2, 0)
        grid.addWidget(self.cont_btn, 2, 1)
        grid.addWidget(self.stop_btn, 3, 0, 1, 2)
        
        self.power_btn.clicked.connect(self.toggle_power)
        self.enable_btn.clicked.connect(self.toggle_enable)
        self.init_btn.clicked.connect(lambda: self.tcp_manager.send_command("StartMaster;"))
        self.reset_btn.clicked.connect(lambda: self.tcp_manager.send_command("GrpReset,0;"))
        self.pause_btn.clicked.connect(lambda: self.tcp_manager.send_command("GrpInterrupt,0;"))
        self.cont_btn.clicked.connect(lambda: self.tcp_manager.send_command("GrpContinue,0;"))
        self.stop_btn.clicked.connect(lambda: self.tcp_manager.send_command("GrpStop,0;"))
        
        layout.addWidget(group)

    def create_teach_mode_group(self, layout):
        group = QGroupBox("Teach Mode")
        h = QHBoxLayout(group)
        self.teach_chk = QCheckBox("Teach Mode On")
        self.teach_chk.stateChanged.connect(self.toggle_teach_mode)
        h.addWidget(self.teach_chk)
        layout.addWidget(group)

    def create_set_tcp_group(self, layout):
        group = QGroupBox("Tool Coordinate System Settings (TCP)")
        v_layout = QVBoxLayout(group)
        
        grid = QGridLayout()
        labels = ["Tar_Tcp_X", "Tar_Tcp_Y", "Tar_Tcp_Z", "Tar_Tcp_Rx", "Tar_Tcp_Ry", "Tar_Tcp_Rz"]
        for i, lbl in enumerate(labels):
            grid.addWidget(QLabel(lbl+":"), i//3, (i%3)*2)
            le = QLineEdit("0.00")
            self.tcp_input_entries.append(le)
            grid.addWidget(le, i//3, (i%3)*2+1)
        v_layout.addLayout(grid)
        
        h1 = QHBoxLayout()
        self.btn_tcp_o = QPushButton("Switch to TCP_O")
        self.btn_tcp_p = QPushButton("Switch to TCP_P")
        self.btn_tcp_u = QPushButton("Switch to TCP_U")
        self.btn_tcp_e = QPushButton("Switch to TCP_E")
        self.btn_tcp_tip = QPushButton("Switch to TCP_tip")
        for b in [self.btn_tcp_o, self.btn_tcp_p, self.btn_tcp_u, self.btn_tcp_e, self.btn_tcp_tip]:
            h1.addWidget(b)
            # 按钮点击时，Qt 会发送 checked 状态(False)，这会被 switch_tcp 的参数接收
            b.clicked.connect(self.switch_tcp)
            
        h2 = QHBoxLayout()
        self.btn_set_cur = QPushButton("Set Cur TCP")
        # 注意：这里连接的是默认参数的调用，即使用界面当前选中的 TCP
        self.btn_set_cur.clicked.connect(lambda: self.send_set_tcp_command())
        h2.addWidget(self.btn_set_cur)
        
        v_layout.addLayout(h1)
        v_layout.addLayout(h2)
        layout.addWidget(group)

    def create_cur_tcp_group(self, layout):
        group = QGroupBox("Current TCP Settings")
        
        v_layout = QVBoxLayout(group)
        
        grid = QGridLayout()
        labels = ["Cur_Tcp_X", "Cur_Tcp_Y", "Cur_Tcp_Z", "Cur_Tcp_Rx", "Cur_Tcp_Ry", "Cur_Tcp_Rz"]
        for i, lbl in enumerate(labels):
            grid.addWidget(QLabel(lbl+":"), i//3, (i%3)*2)
            le = QLineEdit("0.00")
            le.setReadOnly(True)
            le.setStyleSheet("background-color: lightgrey;")
            self.cur_tcp_vars[i] = le
            grid.addWidget(le, i//3, (i%3)*2+1)
        
        v_layout.addLayout(grid)
        
        h = QHBoxLayout()
        b_read = QPushButton("Read Cur TCP")
        b_read.clicked.connect(lambda: self.tcp_manager.send_command("ReadCurTCP,0;"))
        
        b_read_o = QPushButton("Read TCP_O")
        b_read_o.clicked.connect(lambda: self.tcp_manager.send_command("ReadTCPByName,0,TCP_O;"))
        
        b_read_tip = QPushButton("Read TCP_tip")
        b_read_tip.clicked.connect(lambda: self.tcp_manager.send_command("ReadTCPByName,0,TCP_tip;"))
        
        b_read_u = QPushButton("Read TCP_U")
        b_read_u.clicked.connect(lambda: self.tcp_manager.send_command("ReadTCPByName,0,TCP_U;"))
        
        h.addWidget(b_read)
        h.addWidget(b_read_o)
        h.addWidget(b_read_tip)
        h.addWidget(b_read_u)
        
        v_layout.addLayout(h)
        
        layout.addWidget(group)

    def create_tcp_group(self, layout):
        group = QGroupBox("TCP Communication Module")
        v = QVBoxLayout(group)
        
        h_ip = QHBoxLayout()
        self.ip_entry = QLineEdit("192.168.10.10")
        self.port_entry = QLineEdit("10003")
        self.ip_entry.setFixedWidth(200)
        self.port_entry.setFixedWidth(80)
        h_ip.addWidget(QLabel("Remote IP:"))
        h_ip.addWidget(self.ip_entry)
        h_ip.addWidget(QLabel("Port:"))
        h_ip.addWidget(self.port_entry)
        h_ip.addStretch()
        v.addLayout(h_ip)
        
        h_conn = QHBoxLayout()
        self.btn_conn = QPushButton("Connect")
        self.btn_disconn = QPushButton("Disconnect")
        self.btn_disconn.setEnabled(False)
        self.btn_conn.clicked.connect(self.connect_tcp)
        self.btn_disconn.clicked.connect(self.disconnect_tcp)
        h_conn.addWidget(self.btn_conn)
        h_conn.addWidget(self.btn_disconn)
        v.addLayout(h_conn)
        
        self.status_label = QLabel("TCP Status: Disconnected")
        self.status_label.setStyleSheet("color: blue;")
        v.addWidget(self.status_label)
        
        v.addWidget(QLabel("Received Messages:"))
        self.recv_text = QTextEdit()
        self.recv_text.setReadOnly(True)
        self.recv_text.setStyleSheet("background-color: lightgrey;")
        v.addWidget(self.recv_text)
        
        v.addWidget(QLabel("Send Message:"))
        h_send = QHBoxLayout()
        self.send_entry = QLineEdit()
        self.send_entry.setEnabled(False)
        self.btn_send = QPushButton("Send")
        self.btn_send.setEnabled(False)
        self.btn_send.clicked.connect(self.send_message)
        self.send_entry.returnPressed.connect(self.send_message)
        h_send.addWidget(self.send_entry)
        h_send.addWidget(self.btn_send)
        v.addLayout(h_send)
        
        layout.addWidget(group)

    def setup_connections(self):
        self.tcp_manager.connection_status_changed.connect(self.update_ui_on_connection)
        self.tcp_manager.message_received.connect(self.handle_incoming_message)

    def toggle_power(self):
        if "Power On" in self.power_btn.text():
            self.tcp_manager.send_command("Electrify;")
            self.power_btn.setText("Powering On...")
            self.power_btn.setEnabled(False)
        else:
            self.tcp_manager.send_command("OSCmd,1;")
            self.power_btn.setText("Powering Off...")
            self.power_btn.setEnabled(False)

    def toggle_enable(self):
        if "Enable" == self.enable_btn.text():
            self.tcp_manager.send_command("GrpEnable,0;")
            self.enable_btn.setText("Enabling...")
            self.enable_btn.setEnabled(False)
        else:
            self.tcp_manager.send_command("GrpDisable,0;")
            self.enable_btn.setText("Disabling...")
            self.enable_btn.setEnabled(False)

    def toggle_teach_mode(self, state):
        if state == Qt.Checked:
            # [修改] 开启示教模式时：
            # 1. 首先切换到 TCP_E
            self.switch_tcp("TCP_E")
            self.log_message("System: Auto-switched to TCP_E for Teach Mode. Waiting 300ms to open driver...")
            
            # 2. 延迟 300ms 后再发送开启自由驱动的指令
            # 这样确保 TCP 切换指令先被处理，且中间有间隔
            QTimer.singleShot(300, lambda: self.tcp_manager.send_command("GrpOpenFreeDriver,0;"))
        else:
            # 关闭示教模式时，直接发送关闭指令
            self.tcp_manager.send_command("GrpCloseFreeDriver,0;")

    def switch_tcp(self, tcp_name=None):
        """
        统一的 TCP 切换函数。
        逻辑：强制取消 Teach Mode 勾选，延迟 200ms 后发送切换 TCP 命令。
        """
        target_name = None

        # 情况1：如果是字符串，直接使用（代码调用）
        if isinstance(tcp_name, str):
            target_name = tcp_name
        
        # 情况2：如果是按钮触发（tcp_name不是字符串），从按钮文本解析
        else:
            btn = self.sender()
            if btn:
                target_name = btn.text().replace("Switch to ", "")
        
        if target_name:
            # 2. 强制关闭 Teach Mode 勾选
            # 这会触发 self.teach_chk.stateChanged 信号及其绑定的 toggle_teach_mode 逻辑
            if hasattr(self, 'teach_chk'):
                self.teach_chk.setChecked(False)
            
            # 3. 定义发送命令的闭包函数
            def send_switch_command():
                self.current_tcp_name = target_name
                cmd = f"SetTCPByName,0,{target_name};"
                self.tcp_manager.send_command(cmd)

            # 4. 使用 QTimer 延迟 200ms 执行
            # 这种方式不会阻塞主线程，界面依然可以响应
            QTimer.singleShot(200, send_switch_command)

    # [改进] 允许传入 tcp_name 参数，并在发送指令 200ms 后切换到 TCP_E
    def send_set_tcp_command(self, tcp_name=None):
        # 决定使用哪个名字：如果传入了就用传入的，否则用当前界面状态
        target_name = tcp_name if tcp_name else self.current_tcp_name
        
        try:
            params = [float(x.text()) for x in self.tcp_input_entries]
            # 使用 target_name 构造指令
            cmd = f"ConfigTCP,0,{target_name},{params[0]:.2f},{params[1]:.2f},{params[2]:.2f},{params[3]:.2f},{params[4]:.2f},{params[5]:.2f};"
            self.tcp_manager.send_command(cmd)
            self.log_message(cmd)
            
            # [新增] 发送配置指令后，延迟 100ms 自动切换回 TCP_E
            QTimer.singleShot(100, lambda: self.switch_tcp("TCP_E"))
            
        except:
            QMessageBox.critical(self, "Error", "Invalid TCP params")

    def connect_tcp(self):
        ip = self.ip_entry.text()
        try:
            port = int(self.port_entry.text())
            res = self.tcp_manager.connect(ip, port)
            
            if "成功" in res or "Connected" in res:
                self.tcp_manager.send_command("ReadTCPByName,0,TCP_U;")
                self.temp_expected_response = "TCP_U_DEF"
                
                # [修改] 使用统一的 switch_tcp 函数进行自动切换
                self.switch_tcp("TCP_E")
                self.log_message("System: Auto-switched to TCP_E on connect.")

                self.tcp_manager.send_command("SetOverride,0,0.20;")
                self.log_message("System: Auto-set Override to 0.20 on connect.")

                if self.main_window and hasattr(self.main_window, 'left_panel'):
                    self.main_window.left_panel.override_slider.setValue(20)
            else:
                QMessageBox.critical(self, "Connection Error", res)
        except ValueError:
            QMessageBox.critical(self, "Input Error", "Port must be digit.")

    def disconnect_tcp(self):
        self.tcp_manager.disconnect()

    def send_message(self):
        msg = self.send_entry.text()
        if msg: 
            self.log_message(msg)
            self.tcp_manager.send_command(msg)
            self.send_entry.clear()

    def update_ui_on_connection(self, connected):
        self.btn_conn.setEnabled(not connected)
        self.btn_disconn.setEnabled(connected)
        self.send_entry.setEnabled(connected)
        self.btn_send.setEnabled(connected)
        self.status_label.setText("TCP Status: Connected" if connected else "TCP Status: Disconnected")
        if not connected: self.stop_btn.setEnabled(False)

    def handle_incoming_message(self, msg):
        high_freq_msgs = ("ReadActPos", "ReadOverride", "ReadEmergencyInfo", "ReadRobotState", "ReadCurFSM")
        
        if not msg.startswith(high_freq_msgs): 
             self.log_message(msg) 
        
        if msg.startswith("ReadCurTCP") or msg.startswith("ReadTCPByName"):
            self._handle_tcp_msg(msg)
            if self.temp_expected_response == "TCP_U_DEF" and "ReadTCPByName" in msg:
                self._handle_tcp_u_def(msg)
            elif self.temp_expected_response == "TCP_TIP_DEF" and "ReadTCPByName" in msg:
                self._handle_tcp_tip_def(msg)
            elif self.temp_expected_response == "TCP_P_DEF" and "ReadTCPByName" in msg:
                self._handle_tcp_p_def(msg)
        elif msg.startswith("ReadRobotState"):
            self._update_state(msg)
        elif msg.startswith("ReadEmergencyInfo"):
            self._handle_emergency(msg)

    def log_message(self, msg):
        self.recv_text.append(msg)
        self.recv_text.verticalScrollBar().setValue(self.recv_text.verticalScrollBar().maximum())

    def _handle_tcp_msg(self, msg):
        parts = msg.strip(';').strip(',').split(',')
        if len(parts) >= 8 and parts[1] == 'OK':
            try:
                for i in range(6): self.cur_tcp_vars[i].setText(f"{float(parts[2+i]):.2f}")
            except Exception as e:
                print(f"Error _handle_tcp_msg: {e}")

    def _update_state(self, msg):
        parts = msg.strip(';').strip(',').split(',')
        if len(parts) >= 12 and parts[1] == 'OK':
            try:
                nElec = int(parts[11])
                nEn = int(parts[3])
                self.power_btn.setEnabled(True)
                self.enable_btn.setEnabled(True)
                self.power_btn.setText("Power Off" if nElec else "Power On")
                self.power_btn.setStyleSheet("background-color: lightgreen;" if nElec else "background-color: salmon;")
                self.enable_btn.setText("Disable" if nEn else "Enable")
                self.enable_btn.setStyleSheet("background-color: lightgreen;" if nEn else "background-color: salmon;")
            except Exception as e:
                print(f"Error _update_state: {e}")

    def _handle_emergency(self, msg):
        parts = msg.strip(';').strip(',').split(',')
        if len(parts) == 6 and parts[1] == 'OK':
            try:
                self.stop_btn.setEnabled(int(parts[3]) == 0)
            except Exception as e:
                print(f"Error _handle_emergency: {e}")

    def _handle_tcp_u_def(self, msg):
        self.temp_expected_response = None
        parts = msg.strip(';').strip(',').split(',')
        if len(parts) == 8 and parts[1] == 'OK':
            try:
                tcp_u = [float(p) for p in parts[2:]]
                if self.main_window and hasattr(self.main_window, 'left_panel'):
                    self.main_window.left_panel.tcp_u_definition_pose = tcp_u
                    self.log_message(f"System: TCP_U Definition stored: {tcp_u}")
                
                self.tcp_manager.send_command("ReadTCPByName,0,TCP_tip;")
                self.temp_expected_response = "TCP_TIP_DEF"
                self.log_message("System: Auto-requesting TCP_tip definition...")
                
            except Exception as e:
                print(f"Error _handle_tcp_u_def: {e}")
            
    def _handle_tcp_tip_def(self, msg):
        self.temp_expected_response = None
        parts = msg.strip(';').strip(',').split(',')
        if len(parts) == 8 and parts[1] == 'OK':
            try:
                tcp_tip = [float(p) for p in parts[2:]]
                if self.main_window and hasattr(self.main_window, 'left_panel'):
                    self.main_window.left_panel.tcp_tip_definition_pose = tcp_tip
                    self.log_message(f"System: TCP_tip Definition stored: {tcp_tip}")
                
                self.tcp_manager.send_command("ReadTCPByName,0,TCP_P;")
                self.temp_expected_response = "TCP_P_DEF"
                self.log_message("System: Auto-requesting TCP_P definition...")
                
            except Exception as e:
                self.log_message(f"Error parsing TCP_tip def: {e}")
                
    def _handle_tcp_p_def(self, msg):
        self.temp_expected_response = None
        parts = msg.strip(';').strip(',').split(',')
        if len(parts) == 8 and parts[1] == 'OK':
            try:
                tcp_p = [float(p) for p in parts[2:]]
                if self.main_window and hasattr(self.main_window, 'left_panel'):
                    self.main_window.left_panel.tcp_p_definition_pose = tcp_p
                    self.log_message(f"System: TCP_P Definition stored: {tcp_p}")
            except Exception as e:
                self.log_message(f"Error parsing TCP_P def: {e}")