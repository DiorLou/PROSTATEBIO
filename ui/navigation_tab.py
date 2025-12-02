from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel, QLineEdit, 
    QPushButton, QTextEdit, QMessageBox
)
from PyQt5.QtCore import Qt
from core.navigation_tcp_manager import NavigationTCPManager

class NavigationTab(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
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
        self.ip_entry = QLineEdit("192.168.1.100") 
        self.port_entry = QLineEdit("8000")
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