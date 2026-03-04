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
        self.nav_manager_2 = NavigationTCPManager()
        self.realtime_send_enabled = True
        self.init_ui()
        self.setup_connections()

    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # 模块 1：默认端口 9000
        self.group1 = self.create_tcp_group(layout, "TCP Communication Module 1", "9000")
        # 模块 2：增加第二个模块，默认端口 8100
        self.group2 = self.create_tcp_group(layout, "TCP Communication Module 2", "8100")
        
        layout.addStretch()

    def create_tcp_group(self, layout, title, default_port):
        """重构为通用方法，返回控件字典以供绑定"""
        group = QGroupBox(title)
        v = QVBoxLayout(group)
        
        h_ip = QHBoxLayout()
        ip_entry = QLineEdit("127.0.0.1") 
        port_entry = QLineEdit(default_port)
        ip_entry.setFixedWidth(200)
        port_entry.setFixedWidth(80)
        h_ip.addWidget(QLabel("Remote IP:"))
        h_ip.addWidget(ip_entry)
        h_ip.addWidget(QLabel("Port:"))
        h_ip.addWidget(port_entry)
        h_ip.addStretch()
        v.addLayout(h_ip)
        
        h_conn = QHBoxLayout()
        btn_conn = QPushButton("Connect")
        btn_disconn = QPushButton("Disconnect")
        btn_disconn.setEnabled(False)
        h_conn.addWidget(btn_conn)
        h_conn.addWidget(btn_disconn)
        v.addLayout(h_conn)
        
        status_label = QLabel("TCP Status: Disconnected")
        status_label.setStyleSheet("color: blue; font-weight: bold;")
        v.addWidget(status_label)
        
        v.addWidget(QLabel("Received Messages:"))
        recv_text = QTextEdit()
        recv_text.setReadOnly(True)
        recv_text.setStyleSheet("background-color: lightgrey;")
        v.addWidget(recv_text)
        
        v.addWidget(QLabel("Send Message:"))
        h_send = QHBoxLayout()
        send_entry = QLineEdit()
        send_entry.setEnabled(False)
        send_entry.setPlaceholderText("Enter command...")
        btn_send = QPushButton("Send")
        btn_send.setEnabled(False)
        h_send.addWidget(send_entry)
        h_send.addWidget(btn_send)
        v.addLayout(h_send)
        
        layout.addWidget(group)
        return {
            "ip": ip_entry, "port": port_entry, "btn_conn": btn_conn,
            "btn_disconn": btn_disconn, "status": status_label,
            "recv": recv_text, "send_entry": send_entry, "btn_send": btn_send
        }

    def setup_connections(self):

        # 绑定模块 1 逻辑
        self.group1["btn_conn"].clicked.connect(lambda: self.connect_tcp(self.nav_manager, self.group1))
        self.group1["btn_disconn"].clicked.connect(self.nav_manager.disconnect)
        self.group1["btn_send"].clicked.connect(lambda: self.send_message(self.nav_manager, self.group1))
        self.nav_manager.connection_status_changed.connect(lambda c: self.update_ui_on_connection(self.group1, c))
        self.nav_manager.message_received.connect(lambda m: self.log_message(self.group1["recv"], m))

        # 绑定模块 2 逻辑
        self.group2["btn_conn"].clicked.connect(lambda: self.connect_tcp(self.nav_manager_2, self.group2))
        self.group2["btn_disconn"].clicked.connect(self.nav_manager_2.disconnect)
        self.group2["btn_send"].clicked.connect(lambda: self.send_message(self.nav_manager_2, self.group2))
        self.nav_manager_2.connection_status_changed.connect(lambda c: self.update_ui_on_connection(self.group2, c))
        self.nav_manager_2.message_received.connect(lambda m: self.log_message(self.group2["recv"], m))

    def connect_tcp(self, manager, widgets):
        ip = widgets["ip"].text()
        try:
            port = int(widgets["port"].text())
            msg = manager.connect(ip, port)
            if "Successful" not in msg and "Connected" not in msg:
                 QMessageBox.warning(self, "Connection Error", msg)
            else:
                 self.log_message(widgets["recv"], msg)
        except ValueError:
            QMessageBox.critical(self, "Input Error", "Port must be a number.")

    def disconnect_tcp(self):
        self.nav_manager.disconnect()

    def send_message(self, manager, widgets):
        msg = widgets["send_entry"].text()
        if msg:
            self.log_message(widgets["recv"], f"[Sent]: {msg}")
            manager.send_command(msg)
            widgets["send_entry"].clear()

    def update_ui_on_connection(self, widgets, connected):
        widgets["btn_conn"].setEnabled(not connected)
        widgets["btn_disconn"].setEnabled(connected)
        widgets["send_entry"].setEnabled(connected)
        widgets["btn_send"].setEnabled(connected)
        widgets["ip"].setEnabled(not connected)
        widgets["port"].setEnabled(not connected)
        
        widgets["status"].setText("TCP Status: Connected" if connected else "TCP Status: Disconnected")
        widgets["status"].setStyleSheet("color: green; font-weight: bold;" if connected else "color: red; font-weight: bold;")

    def log_message(self, text_edit, msg):
        text_edit.append(msg)
        text_edit.verticalScrollBar().setValue(text_edit.verticalScrollBar().maximum())

    def cleanup(self):
        self.nav_manager.disconnect()
        self.nav_manager_2.disconnect()

    def set_realtime_send_enabled(self, enabled: bool):
        """控制是否允许实时发送针尖位姿"""
        self.realtime_send_enabled = enabled
        
    def update_needle_pose_in_volume(self, curr_j0, curr_j1, curr_j2, curr_j3):
        """
        槽函数：接收 Beckhoff Tab 发来的当前位置更新。
        计算针尖在 Volume 坐标系下的位姿并发送。
        """
        if not self.realtime_send_enabled:
            return

        if not self.nav_manager_2.is_connected:
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
            发送格式示例: "Nx,Ny,Nz,Nrx,Nry,Nrz;"
            """
            msg = f"{pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f},{rpy[0]:.3f},{rpy[1]:.3f},{rpy[2]:.3f}"
            self.nav_manager_2.send_command(msg)
            
        except Exception as e:
            print(f"Transform chain error: {e}")

    def send_target_pose_once(self, vector, rcm_point):
        """
        计算目标方向向量在 Volume 系下的位姿并发送一次。
        vector: 目标方向向量 (np.array)
        rcm_point: 旋转中心点坐标 (np.array)
        """
        if not self.nav_manager_2.is_connected:
            return

        mw = self.main_window
        lp = mw.left_panel
        if lp.tcp_p_definition_pose is None or lp.volume_in_base is None or not lp.latest_tool_pose:
            return

        try:
            # 1. 构造 T_P_Target (在 TCP_P 系下的目标位姿)
            # 【核心修改】：通过最小旋转对齐 Z 轴，同时保持 X 轴与 TCP_P 一致
            def get_consistent_target_matrix(vec, pos):
                z_target = vec / np.linalg.norm(vec)
                
                # 参考 TCP_P 原本的 X 轴 [1, 0, 0] 构造正交基
                x_ref = np.array([1, 0, 0])
                
                # 计算 y = z_target cross x_ref
                y_target = np.cross(z_target, x_ref)
                if np.linalg.norm(y_target) < 1e-6:
                    # 如果 z 刚好指向 x，则改用 y 轴作为参考
                    y_ref = np.array([0, 1, 0])
                    x_target = np.cross(y_ref, z_target)
                    y_target = np.cross(z_target, x_target)
                else:
                    y_target /= np.linalg.norm(y_target)
                    x_target = np.cross(y_target, z_target)
                
                T = np.identity(4)
                T[:3, 0] = x_target / np.linalg.norm(x_target)
                T[:3, 1] = y_target / np.linalg.norm(y_target)
                T[:3, 2] = z_target
                T[:3, 3] = pos
                return T

            # 使用新逻辑计算 T_P_Target，确保其姿态相对于 TCP_P 是“正”的
            T_P_Target = get_consistent_target_matrix(vector, rcm_point)

            # 2. 坐标系变换链 (确保与 update_needle_pose_in_volume 逻辑闭环)
            def to_matrix(pose):
                x, y, z, rx, ry, rz = [float(val) for val in pose]
                T = np.identity(4)
                # 使用相同的旋转顺序 (0,1,2 - SXYZ) 和外旋定义
                R = pyrot.matrix_from_euler(np.deg2rad([rx, ry, rz]), 0, 1, 2, extrinsic=True)
                T[:3, :3] = R.astype(np.float64)
                T[:3, 3] = [x, y, z]
                return T

            T_Base_E = to_matrix(lp.latest_tool_pose)
            T_E_P = to_matrix(lp.tcp_p_definition_pose)
            T_Base_P = T_Base_E @ T_E_P # 这是当前探头的世界位姿
            
            T_Base_Vol = to_matrix(lp.volume_in_base)
            T_Vol_Base = np.linalg.inv(T_Base_Vol)
            
            # 最终变换计算：Volume -> Base -> TCP_P -> Target
            T_Vol_Target = T_Vol_Base @ T_Base_P @ T_P_Target
            
            # 3. 提取位姿并发送 (保持原格式，确信 Euler 提取顺序一致)
            pos = T_Vol_Target[:3, 3]
            rot_mat = T_Vol_Target[:3, :3]
            rpy = np.rad2deg(pyrot.euler_from_matrix(rot_mat, 0, 1, 2, extrinsic=True))
            
            msg = f"{pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f},{rpy[0]:.3f},{rpy[1]:.3f},{rpy[2]:.3f}"
            self.nav_manager_2.send_command(msg)
            print(f"Navigation: Target pose sent to volume (consistent): {msg}")

        except Exception as e:
            print(f"Error sending target pose: {e}")