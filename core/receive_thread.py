# core/receive_thread.py
import socket
from PyQt5.QtCore import QThread, pyqtSignal

class ReceiveThread(QThread):
    """
    一个自定义的 QThread 类，用于在后台线程中接收TCP数据，
    以避免阻塞主GUI线程，保持界面的响应性。这是GUI编程中的一个重要模式。
    """
    # 定义一个信号：当收到新消息时发出，并附带消息内容。
    # 信号可以在线程间安全地传递数据。
    message_received = pyqtSignal(str)
    # 定义一个信号：当TCP连接丢失时发出。
    connection_lost = pyqtSignal()

    def __init__(self, client_socket, parent=None):
        super().__init__(parent)
        self.client_socket = client_socket  # 保存传入的TCP套接字实例。
        self.is_running = True  # 一个布尔标志，用于控制线程的主循环。

    def run(self):
        """线程的主循环，负责持续接收来自套接字的数据。"""
        while self.is_running:
            try:
                # 尝试接收最大1024字节的数据。这是一个阻塞调用。
                data = self.client_socket.recv(1024)
                if not data:
                    # 如果 recv() 返回一个空的字节串，这意味着对端（服务器）
                    # 已经优雅地关闭了连接。
                    self.connection_lost.emit()  # 发出连接丢失信号。
                    break  # 退出循环，线程将结束。
                # 将接收到的字节数据解码为UTF-8字符串。
                message = data.decode('utf-8')
                # 发出信号，将消息内容传递给主线程进行处理。
                self.message_received.emit(message)
            except (ConnectionResetError, socket.error):
                # 捕获常见的连接错误，例如对端强制关闭连接。
                self.connection_lost.emit()  # 发出连接丢失信号。
                break  # 退出循环。
            except Exception as e:
                # 捕获任何其他意外的异常，例如解码错误。
                if self.is_running:
                    print(f"接收消息线程出错: {e}")
                    self.connection_lost.emit()
                break
        print("接收消息线程已退出。")

    def stop(self):
        """提供一个安全的方法来从外部停止线程。"""
        self.is_running = False  # 设置标志为False，以退出循环。
        # wait() 方法会阻塞，直到线程的 run() 方法执行完毕，确保线程优雅地退出。
        self.wait()