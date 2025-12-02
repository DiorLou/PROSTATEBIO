import socket
from PyQt5.QtCore import QObject, pyqtSignal
from .receive_thread import ReceiveThread

class NavigationTCPManager(QObject):
    """
    专用于导航软件通信的 TCP 管理器。
    去除了机器人特定的定时查询指令，仅保留基础连接和收发功能。
    """
    connection_status_changed = pyqtSignal(bool)
    message_received = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.is_connected = False
        self.client_socket = None
        self.receive_thread = None

    def connect(self, ip, port):
        if self.is_connected:
            return "Already connected."

        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.settimeout(3)
            self.client_socket.connect((ip, port))
            self.client_socket.settimeout(None)

            self.is_connected = True
            self.connection_status_changed.emit(True)
            self.message_received.emit("System: Connected to Navigation Host!")

            # 复用已有的 ReceiveThread
            self.receive_thread = ReceiveThread(self.client_socket)
            self.receive_thread.message_received.connect(self.message_received.emit)
            self.receive_thread.connection_lost.connect(self.disconnect)
            self.receive_thread.start()

            return "Connection Successful."

        except Exception as e:
            return f"Connection Failed: {e}"

    def disconnect(self):
        if not self.is_connected:
            return

        self.is_connected = False
        
        try:
            if self.client_socket:
                self.client_socket.close()
        except Exception as e:
            print(f"Socket close error: {e}")

        if self.receive_thread and self.receive_thread.isRunning():
            self.receive_thread.stop()

        self.connection_status_changed.emit(False)
        self.message_received.emit("System: Disconnected.")

    def send_command(self, command):
        if not self.is_connected:
            self.message_received.emit("Error: No Connection.")
            return
        try:
            # 根据导航软件的协议，决定是否需要加分号或换行符
            # 这里默认像机器人一样加分号，如果不加分号请删除下面这行
            # command += ';' 
            
            # 如果导航软件需要换行符，取消下面这行的注释
            # command += '\n'
            
            self.client_socket.sendall(command.encode('utf-8'))
        except Exception as e:
            self.message_received.emit(f"Send Failed: {e}")
            self.disconnect()