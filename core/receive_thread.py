import socket
from PyQt5.QtCore import QThread, pyqtSignal

class ReceiveThread(QThread):
    message_received = pyqtSignal(str)
    connection_lost = pyqtSignal()

    def __init__(self, client_socket, parent=None):
        super().__init__(parent)
        self.client_socket = client_socket
        self.is_running = True
        self.buffer = b""  # 初始化一个字节缓冲区

    def run(self):
        while self.is_running:
            try:
                data = self.client_socket.recv(1024)
                if not data:
                    self.connection_lost.emit()
                    break
                
                self.buffer += data  # 将接收到的数据追加到缓冲区
                
                # 循环检查缓冲区中是否存在完整的消息结束符
                while b";" in self.buffer:
                    # 使用 split(b";", 1) 来分割第一个完整消息和剩余数据
                    message, self.buffer = self.buffer.split(b";", 1)
                    # 将完整的消息解码并发出
                    self.message_received.emit(message.decode('utf-8') + ";")

            except (ConnectionResetError, socket.error):
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
        self.wait()