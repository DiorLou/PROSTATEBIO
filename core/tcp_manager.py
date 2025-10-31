# core/tcp_manager.py
import socket
from PyQt5.QtCore import QObject, QTimer, pyqtSignal
from .receive_thread import ReceiveThread

class TCPManager(QObject):
    """
    一个用于管理所有TCP通信逻辑的类。
    它负责连接、断开、发送数据以及启动和停止定时器。
    通过信号与 UI 界面进行通信，避免了直接耦合。
    """
    # 定义信号以与 UI 交互
    connection_status_changed = pyqtSignal(bool)
    message_received = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.is_connected = False
        self.client_socket = None
        self.receive_thread = None
        
        # QTimer 用于定时向机器人发送请求实时数据的指令（ReadActPos），频率：50ms。
        self.actpos_update_timer = QTimer(self)
        self.actpos_update_timer.timeout.connect(self.request_actpos_data)
        
        # QTimer 用于定时请求机器人状态数据（ReadRobotState），频率：300ms。
        self.robotstate_update_timer = QTimer(self)
        self.robotstate_update_timer.timeout.connect(self.request_robot_state_data)
        
        # QTimer用于定时请求急停信息
        self.emergency_update_timer = QTimer(self)
        self.emergency_update_timer.timeout.connect(self.request_emergency_info)
        
        # 新增的定时器，用于实时读取运动速率
        self.override_update_timer = QTimer(self)
        self.override_update_timer.timeout.connect(self.request_override_info)
        
    def connect(self, ip, port):
        """尝试建立TCP连接到机器人控制器。"""
        if self.is_connected:
            return "已连接，请勿重复操作。"

        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.settimeout(3)  # 设置3秒的连接超时。
            self.client_socket.connect((ip, port))
            self.client_socket.settimeout(None)  # 连接成功后取消超时。

            self.is_connected = True
            self.connection_status_changed.emit(True)  # 发出连接成功信号
            self.message_received.emit("系统: 连接成功！")

            # 启动消息接收线程。
            self.receive_thread = ReceiveThread(self.client_socket)
            # 将线程的信号连接到本类的信号，再由本类的信号传递给主窗口。
            self.receive_thread.message_received.connect(self.message_received.emit)
            self.receive_thread.connection_lost.connect(self.disconnect)
            self.receive_thread.start()

            # 启动实时更新定时器 (ReadActPos)，50ms。
            self.actpos_update_timer.start(50)
            self.request_actpos_data()  # 立即请求一次数据，快速更新UI。
            
            # 启动机器人状态定时器 (ReadRobotState)，300ms。
            self.robotstate_update_timer.start(300)
            self.request_robot_state_data() # 立即请求一次数据。
            
            # 启动紧急状态更新定时器。
            self.emergency_update_timer.start(200)
            
            # 启动运动速率更新定时器。
            self.override_update_timer.start(200)
            
            return "连接成功。"

        except socket.timeout:
            return "连接失败 (连接超时)"
        except socket.gaierror:
            return "连接失败 (IP地址错误)"
        except ConnectionRefusedError:
            return "连接失败 (连接被拒绝)"
        except ValueError:
            return "连接失败 (端口号错误)"
        except Exception as e:
            return f"发生未知错误: {e}"

    def disconnect(self):
        """断开TCP连接，并执行所有必要的清理工作。"""
        if not self.is_connected:
            return

        self.is_connected = False
        self.actpos_update_timer.stop()
        self.robotstate_update_timer.stop() 
        self.emergency_update_timer.stop()
        self.override_update_timer.stop()
        
        # 优先关闭套接字，这会强制 recv() 抛出异常，让线程快速退出
        try:
            if self.client_socket:
                self.client_socket.close()  # 先关闭套接字
        except Exception as e:
            print(f"关闭套接字出错: {e}")

        # 然后再安全地等待线程退出
        if self.receive_thread and self.receive_thread.isRunning():
            self.receive_thread.stop() 

        self.connection_status_changed.emit(False)
        self.message_received.emit("系统: 连接已断开。")

    def send_command(self, command):
        """一个通用的方法，用于发送UR控制指令，并处理连接和错误。"""
        if not self.is_connected:
            self.message_received.emit("UR控制失败: 未建立TCP连接。")
            return
        try:
            if not command.endswith(';'):
                command += ';'  # 确保命令以分号结尾，符合协议。
            self.client_socket.sendall(command.encode('utf-8'))
            self.message_received.emit(f"发送 UR 指令: {command}")
        except Exception as e:
            self.message_received.emit(f"UR指令发送失败: {e}")
            self.disconnect()

    def request_actpos_data(self):
        """定时向机器人发送指令，请求获取实时关节和末端坐标数据 (ReadActPos)。 (50ms)"""
        self.send_command("ReadActPos,0;")

    def request_robot_state_data(self):
        """定时向机器人发送指令，请求获取机器人状态数据 (ReadRobotState)。 (300ms)"""
        self.send_command("ReadRobotState,0;")
    
    def request_emergency_info(self):
        """定时向机器人发送指令，请求获取急停状态。"""
        self.send_command("ReadEmergencyInfo,0;")

    def request_override_info(self):
        """定时向机器人发送指令，请求获取当前运动速率。"""
        self.send_command("ReadOverride,0;")