from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel, QLineEdit, 
    QPushButton, QTextEdit, QMessageBox
)
from PyQt5.QtCore import Qt
from core.navigation_tcp_manager import NavigationTCPManager
import numpy as np
import pytransform3d.rotations as pyrot

class NavigationTab(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.main_window = parent # Explicitly store reference to Main Window
        self.nav_manager = NavigationTCPManager()
        self.init_ui()
        self.setup_connections()

    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # 创建 TCP 通信组，风格模仿 Robot Control
        self.create_tcp_group(layout)
        
        layout.addStretch()

    def create_tcp_group(self, layout):
        group = QGroupBox("TCP Communication Module (Navigation)")
        v = QVBoxLayout(group)
        
        # 1. IP 和 端口设置
        h_ip = QHBoxLayout()
        # 假设导航上位机的默认 IP，您可以根据实际情况修改
        self.ip_entry = QLineEdit("127.0.0.1") 
        self.port_entry = QLineEdit("9000")
        self.ip_entry.setFixedWidth(200)
        self.port_entry.setFixedWidth(80)
        
        h_ip.addWidget(QLabel("Remote IP:"))
        h_ip.addWidget(self.ip_entry)
        h_ip.addWidget(QLabel("Port:"))
        h_ip.addWidget(self.port_entry)
        h_ip.addStretch()
        v.addLayout(h_ip)
        
        # 2. 连接/断开按钮
        h_conn = QHBoxLayout()
        self.btn_conn = QPushButton("Connect")
        self.btn_disconn = QPushButton("Disconnect")
        self.btn_disconn.setEnabled(False)
        
        h_conn.addWidget(self.btn_conn)
        h_conn.addWidget(self.btn_disconn)
        v.addLayout(h_conn)
        
        # 3. 状态标签
        self.status_label = QLabel("TCP Status: Disconnected")
        self.status_label.setStyleSheet("color: blue; font-weight: bold;")
        v.addWidget(self.status_label)
        
        # 4. 接收消息区域
        v.addWidget(QLabel("Received Messages:"))
        self.recv_text = QTextEdit()
        self.recv_text.setReadOnly(True)
        self.recv_text.setStyleSheet("background-color: lightgrey;")
        v.addWidget(self.recv_text)
        
        # 5. 发送消息区域
        v.addWidget(QLabel("Send Message:"))
        h_send = QHBoxLayout()
        self.send_entry = QLineEdit()
        self.send_entry.setEnabled(False)
        self.send_entry.setPlaceholderText("Enter command for navigation software...")
        
        self.btn_send = QPushButton("Send")
        self.btn_send.setEnabled(False)
        
        h_send.addWidget(self.send_entry)
        h_send.addWidget(self.btn_send)
        v.addLayout(h_send)
        
        layout.addWidget(group)

    def setup_connections(self):
        # 按钮事件
        self.btn_conn.clicked.connect(self.connect_tcp)
        self.btn_disconn.clicked.connect(self.disconnect_tcp)
        self.btn_send.clicked.connect(self.send_message)
        self.send_entry.returnPressed.connect(self.send_message)
        
        # Manager 信号
        self.nav_manager.connection_status_changed.connect(self.update_ui_on_connection)
        self.nav_manager.message_received.connect(self.log_message)

    def connect_tcp(self):
        ip = self.ip_entry.text()
        try:
            port = int(self.port_entry.text())
            msg = self.nav_manager.connect(ip, port)
            
            if "Successful" not in msg and "Connected" not in msg:
                 QMessageBox.warning(self, "Connection Error", msg)
            else:
                 self.log_message(msg)
                 
        except ValueError:
            QMessageBox.critical(self, "Input Error", "Port must be a number.")

    def disconnect_tcp(self):
        self.nav_manager.disconnect()

    def send_message(self):
        msg = self.send_entry.text()
        if msg:
            self.log_message(f"[Sent]: {msg}")
            self.nav_manager.send_command(msg)
            # 是否清空输入框取决于个人习惯，这里清空
            self.send_entry.clear() 

    def update_ui_on_connection(self, connected):
        self.btn_conn.setEnabled(not connected)
        self.btn_disconn.setEnabled(connected)
        self.send_entry.setEnabled(connected)
        self.btn_send.setEnabled(connected)
        self.ip_entry.setEnabled(not connected)
        self.port_entry.setEnabled(not connected)
        
        self.status_label.setText("TCP Status: Connected" if connected else "TCP Status: Disconnected")
        self.status_label.setStyleSheet("color: green; font-weight: bold;" if connected else "color: red; font-weight: bold;")

    def log_message(self, msg):
        self.recv_text.append(msg)
        # 自动滚动到底部
        self.recv_text.verticalScrollBar().setValue(self.recv_text.verticalScrollBar().maximum())

    def cleanup(self):
        """窗口关闭时调用，确保断开连接"""
        self.nav_manager.disconnect()
        
    def update_needle_pose_in_volume(self, curr_j0, curr_j1, curr_j2, curr_j3):
        """
        槽函数：接收 Beckhoff Tab 发来的当前位置更新。
        计算针尖在 Volume 坐标系下的位姿并发送。
        """
        if not self.nav_manager.is_connected:
            return
            
        # 1. 获取主窗口引用
        mw = self.main_window
        if not mw: return
        
        # 2. 检查 LeftPanel 数据是否就绪
        lp = mw.left_panel
        if lp.tcp_p_definition_pose is None or lp.volume_in_base is None or not lp.latest_tool_pose:
            # 数据未准备好，跳过
            return
            
        # 3. 获取 Reset 值 (Beckhoff Tab)
        bt = mw.beckhoff_tab
        if bt is None: return
        
        # 4. 计算关节差值 (Joint Values)
        # 根据用户定义的输入顺序: [Current J1 - RESET_J1, Current J2 - RESET_J2, Current J3 - RESET_J3, Current J0 - RESET_J0]
        try:
            val_x0 = curr_j0 - bt.manager.RESET_J0
            val_x1 = curr_j1 - bt.manager.RESET_J1
            val_x2 = curr_j2 - bt.manager.RESET_J2
            val_x3 = curr_j3 - bt.manager.RESET_J3
            
            joint_values = [val_x0, val_x1, val_x2 - val_x1, val_x3]
        except Exception as e:
            print(f"joint_values error: {e}")
            return # 数据异常

        # 5. 计算针尖在 TCP_P 下的位姿 (T_TCP_P_Needle)
        # robot._forward 返回的是 SymPy 矩阵，需要转换
        try:
            # 强制将 SymPy 结果转换为标准 Python float 列表再转 NumPy
            res_sympy = mw.robot_kinematics._forward(joint_values)
            # 使用 res_sympy.tolist() 确保提取的是数值内容
            T_P_Needle = np.array(res_sympy.tolist(), dtype=np.float64) 
        except Exception as e:
            print(f"Kinematics conversion error: {e}")
            return

        # 6. 执行坐标系变换链
        # 目标: T_Vol_Needle = T_Vol_Base * T_Base_P * T_P_Needle
        try:
            # 辅助函数：Pose (x,y,z,rx,ry,rz) 转 4x4 矩阵
            def to_matrix(pose):
                # 确保 pose 是 flat 的 float 列表
                x, y, z, rx, ry, rz = [float(val) for val in pose]
                T = np.identity(4)
                # 确保旋转矩阵计算结果也是 float64
                R = pyrot.matrix_from_euler(np.deg2rad([rx, ry, rz]), 0, 1, 2, extrinsic=True)
                T[:3, :3] = R.astype(np.float64)
                T[:3, 3] = [x, y, z]
                return T
            
            # 获取基础变换矩阵
            T_Base_E = to_matrix(lp.latest_tool_pose)
            T_E_P = to_matrix(lp.tcp_p_definition_pose)
            T_Base_P = np.matmul(T_Base_E, T_E_P) # 建议使用 @ 或 np.matmul
            
            T_Base_Vol = to_matrix(lp.volume_in_base)
            T_Vol_Base = np.linalg.inv(T_Base_Vol)
            
            # 最终乘法：确保所有项都是 (4, 4) 维度
            T_Vol_Needle = T_Vol_Base @ T_Base_P @ T_P_Needle
            
            # 7. 提取位姿并发送
            pos = T_Vol_Needle[:3, 3]
            rot_mat = T_Vol_Needle[:3, :3]
            # 转换为 Euler 角 (rx, ry, rz)
            rpy = np.rad2deg(pyrot.euler_from_matrix(rot_mat, 0, 1, 2, extrinsic=True))
            
            """
            格式化并发送 针尖位姿(Volume系) 到导航服务器。
            发送格式示例: "UpdateNeedlePoseInVol,Nx,Ny,Nz,Nrx,Nry,Nrz;"
            """
            msg = f"{pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f},{rpy[0]:.3f},{rpy[1]:.3f},{rpy[2]:.3f}"
            self.nav_manager.send_command(msg)
            
        except Exception as e:
            print(f"Transform chain error: {e}")