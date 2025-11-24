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
        
        # 4. 当前 TCP 显示 (此处修复了按钮丢失问题)
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
            b.clicked.connect(self.switch_tcp)
            
        h2 = QHBoxLayout()
        self.btn_set_cur = QPushButton("Set Cur TCP")
        self.btn_set_cur.clicked.connect(self.send_set_tcp_command)
        self.btn_get_virtual = QPushButton("Get Virtual RCM_O")
        self.btn_get_virtual.clicked.connect(lambda: self.start_get_suitable_tcp_sequence(False))
        h2.addWidget(self.btn_set_cur)
        h2.addWidget(self.btn_get_virtual)
        
        v_layout.addLayout(h1)
        v_layout.addLayout(h2)
        layout.addWidget(group)

    def create_cur_tcp_group(self, layout):
        group = QGroupBox("Current TCP Settings")
        
        # 修复：使用 QVBoxLayout 作为主容器，这样可以垂直放入 Grid(数值) 和 HBox(按钮)
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
        
        # 将数值网格添加到垂直布局
        v_layout.addLayout(grid)
        
        h = QHBoxLayout()
        b_read = QPushButton("Read Cur TCP")
        b_read.clicked.connect(lambda: self.tcp_manager.send_command("ReadCurTCP,0;"))
        
        b_read_o = QPushButton("Read TCP_O")
        b_read_o.clicked.connect(lambda: self.tcp_manager.send_command("ReadTCPByName,0,TCP_O;"))
        
        b_read_tip = QPushButton("Read TCP_tip")
        b_read_tip.clicked.connect(lambda: self.tcp_manager.send_command("ReadTCPByName,0,TCP_tip;"))
        
        # 【新增】找回 Read TCP_U 按钮
        b_read_u = QPushButton("Read TCP_U")
        b_read_u.clicked.connect(lambda: self.tcp_manager.send_command("ReadTCPByName,0,TCP_U;"))
        
        h.addWidget(b_read)
        h.addWidget(b_read_o)
        h.addWidget(b_read_tip)
        h.addWidget(b_read_u)
        
        # 将按钮栏添加到垂直布局
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
        self.tcp_manager.message_received.connect(self.handle_message)

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
        cmd = "GrpOpenFreeDriver,0;" if state == Qt.Checked else "GrpCloseFreeDriver,0;"
        self.tcp_manager.send_command(cmd)

    def switch_tcp(self):
        btn = self.sender()
        name = btn.text().replace("Switch to ", "")
        self.current_tcp_name = name
        self.log_message(f"SetTCPByName,0,{name};")
        self.tcp_manager.send_command(f"SetTCPByName,0,{name};")

    def send_set_tcp_command(self):
        try:
            params = [float(x.text()) for x in self.tcp_input_entries]
            cmd = f"ConfigTCP,0,{self.current_tcp_name},{params[0]:.2f},{params[1]:.2f},{params[2]:.2f},{params[3]:.2f},{params[4]:.2f},{params[5]:.2f};"
            self.tcp_manager.send_command(cmd)
            self.log_message(cmd)
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

    def handle_message(self, msg):
        # 高频消息过滤
        high_freq_msgs = ("ReadActPos", "ReadOverride", "ReadEmergencyInfo", "ReadRobotState")
        
        if not msg.startswith(high_freq_msgs): 
             self.log_message(msg) 
        
        if msg.startswith("ReadCurTCP") or msg.startswith("ReadTCPByName"):
            self._handle_tcp_msg(msg)
            if self.temp_expected_response == "TCP_U_DEF" and "ReadTCPByName" in msg:
                self._handle_tcp_u_def(msg)
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
            except: pass

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
            except: pass

    def _handle_emergency(self, msg):
        parts = msg.strip(';').strip(',').split(',')
        if len(parts) == 6 and parts[1] == 'OK':
            try:
                self.stop_btn.setEnabled(int(parts[3]) == 0)
            except: pass

    def _handle_tcp_u_def(self, msg):
        self.temp_expected_response = None
        parts = msg.strip(';').strip(',').split(',')
        if len(parts) == 8 and parts[1] == 'OK':
            try:
                tcp_u = [float(p) for p in parts[2:]]
                if self.main_window and self.main_window.left_panel:
                    self.main_window.left_panel.tcp_u_definition_pose = tcp_u
                    self.log_message(f"System: TCP_U Definition stored: {tcp_u}")
            except: pass

    def start_get_suitable_tcp_sequence(self, is_fine_tune):
        if not self.tcp_manager.is_connected: return
        self.is_fine_tuning_process = is_fine_tune
        self.log_message("System: Starting Get Suitable TCP Sequence...")
        self.tcp_manager.send_command("SetTCPByName,0,TCP_E;")
        QTimer.singleShot(200, self._step_1_get_e)

    def _step_1_get_e(self):
        if self.main_window:
            self.main_window.left_panel.get_e_point_position()
            QTimer.singleShot(200, self._step_2_calc)

    def _step_2_calc(self):
        if not self.main_window: return
        try:
            lp = self.main_window.left_panel
            o_pt = np.array([float(v.text()) for v in lp.o_vars])
            e_pt = np.array([float(v.text()) for v in lp.e_vars])
            cur_pose = lp.latest_tool_pose
            if not cur_pose: return

            rpy_rad = np.deg2rad(cur_pose[3:])
            rot = pyrot.matrix_from_euler(rpy_rad, 0, 1, 2, extrinsic=True)
            z_vec = rot[:, 2]
            dist = np.dot(o_pt - e_pt, z_vec)
            
            for le in self.tcp_input_entries: le.setText("0.00")
            self.tcp_input_entries[2].setText(f"{dist:.2f}")
            self.tcp_input_entries[5].setText("157.50")
            
            p_o_new = cur_pose[:3] + dist * z_vec
            for i in range(3): lp.o_vars[i].setText(f"{p_o_new[i]:.2f}")
            
            self.tcp_manager.send_command("SetTCPByName,0,TCP_O;")
            QTimer.singleShot(200, self._step_3_set_cur)
        except Exception as e: 
            self.log_message(f"Error in calc: {e}")

    def _step_3_set_cur(self):
        self.send_set_tcp_command()
        QTimer.singleShot(200, self._finalize_sequence)

    def _finalize_sequence(self):
        if self.is_fine_tuning_process:
            if self.main_window and self.main_window.left_panel:
                lp = self.main_window.left_panel
                lp.is_fine_tuning_allowed = True
                lp.fine_tune_probe_btn.setStyleSheet("background-color: lightgreen;")
                QMessageBox.information(self, "Ready", "Fine tuning enabled (TCP_O).")
        else:
            self.tcp_manager.send_command("SetTCPByName,0,TCP_E;")
            QMessageBox.information(self, "Success", "Get Virtual RCM_O sequence completed.")
