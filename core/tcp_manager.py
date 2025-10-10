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
    # [MODIFIED] message_received 信号现在连接到 message_received_handler，
    # 再由 handler 转发给 UI。
    message_received = pyqtSignal(str) 

    def __init__(self, parent=None):
        super().__init__(parent)
        self.is_connected = False
        self.client_socket = None
        self.receive_thread = None
        
        # QTimer 用于定时向机器人发送请求实时数据的指令。
        self.real_time_update_timer = QTimer(self)
        self.real_time_update_timer.timeout.connect(self.request_real_time_data)
        
        # QTimer用于定时请求急停信息
        self.emergency_update_timer = QTimer(self)
        self.emergency_update_timer.timeout.connect(self.request_emergency_info)
        
        # 新增的定时器，用于实时读取运动速率
        self.override_update_timer = QTimer(self)
        self.override_update_timer.timeout.connect(self.request_override_info)
        
        # [新增] FSM 状态检查和命令队列
        self.command_queue = [] # 命令队列
        self.is_fsm_standby = False # FSM状态是否为33
        
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
            # [MODIFIED] 将线程的信号连接到本类的消息处理函数，先处理 FSM 状态。
            self.receive_thread.message_received.connect(self.message_received_handler)
            self.receive_thread.connection_lost.connect(self.disconnect)
            self.receive_thread.start()

            # 启动实时更新定时器。
            self.real_time_update_timer.start(100)
            self.request_real_time_data()  # 立即请求一次数据，快速更新UI。
            
            # 启动紧急状态更新定时器。
            self.emergency_update_timer.start(200)
            
            # 启动运动速率更新定时器。
            self.override_update_timer.start(200)
            
            # [新增] 连接成功后立即请求一次 FSM 状态
            self.request_fsm_info()
            
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
        self.real_time_update_timer.stop()
        self.emergency_update_timer.stop()
        self.override_update_timer.stop()
        
        # [新增] 清理 FSM 状态和命令队列
        self.is_fsm_standby = False
        self.command_queue = []
        
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

    # [新增] 内部方法：立即发送命令，不经过 FSM 检查
    def _send_command_now(self, command, is_fsm_check=False):
        """内部方法：立即发送命令，不经过FSM检查和队列处理。"""
        if not self.is_connected:
            # 只有非FSM检查命令才需要报告错误给用户
            if not is_fsm_check:
                self.message_received.emit("UR控制失败: 未建立TCP连接。")
            return
        try:
            if not command.endswith(';'):
                command += ';'  # 确保命令以分号结尾，符合协议。
            self.client_socket.sendall(command.encode('utf-8'))
            # FSM检查命令不打印到 UI 日志
            if not is_fsm_check:
                self.message_received.emit(f"发送 UR 指令: {command}")
        except Exception as e:
            self.message_received.emit(f"UR指令发送失败: {e}")
            self.disconnect()

    # [新增] 处理命令队列
    def process_command_queue(self):
        """检查 FSM 状态，如果就绪且队列不空，发送队列中的下一个命令。"""
        if self.is_fsm_standby and self.command_queue:
            command_to_send = self.command_queue.pop(0)
            self.message_received.emit(f"系统: FSM 33就绪，发送队列命令: {command_to_send}")
            # 使用内部方法发送命令
            self._send_command_now(command_to_send)
            # 发送完后，再次请求 FSM 状态，等待下一个命令周期
            self.request_fsm_info() 

    # [新增] 处理接收消息
    def message_received_handler(self, message):
        """处理来自接收线程的消息，并在分发之前检查是否为 FSM 消息。"""
        # 优先处理FSM消息，因为这是内部逻辑
        if message.startswith("ReadCurFSM"):
            self.handle_fsm_message(message)
        
        # 仍然将所有消息转发给UI进行日志记录和进一步处理
        self.message_received.emit(message)
    
    # [新增] 处理 FSM 消息
    def handle_fsm_message(self, message):
        """解析 'ReadCurFSM' 消息，并更新 FSM 状态。"""
        parts = message.strip(';').strip(',').split(',')
        # 预期的格式: ReadCurFSM,OK,33,
        if len(parts) == 3 and parts[0] == 'ReadCurFSM' and parts[1] == 'OK':
            try:
                fsm_state = int(parts[2])
                was_standby = self.is_fsm_standby
                self.is_fsm_standby = (fsm_state == 33)
                
                if self.is_fsm_standby and not was_standby:
                    self.message_received.emit("系统: FSM 状态已切换至 33 (待机)。")
                    # 状态变为待机，尝试发送队列中的命令
                    self.process_command_queue()
            except (ValueError, IndexError):
                print(f"警告: 无法解析 FSM 状态信息: {message}")

    # [MODIFIED] 修改 send_command 以支持 FSM 队列
    def send_command(self, command, is_fsm_check=False):
        """
        一个通用的方法，用于发送UR控制指令，并处理连接和错误。
        新增FSM状态检查逻辑。
        """
        if not self.is_connected:
            self.message_received.emit("UR控制失败: 未建立TCP连接。")
            return
        
        # 1. 允许 FSM 检查命令立即发送
        if is_fsm_check:
            self._send_command_now(command, is_fsm_check=True)
            return

        # [SIMPLIFIED] 2. 非 FSM 检查命令，直接加入队列 (允许重复命令)
        # 警告: 这种方式会导致重复的幂等命令 (如 GrpEnable, SetCurTCP) 
        # 在机器人忙碌时被多次排队和执行，但代码更简洁。
        self.command_queue.append(command)
        self.message_received.emit(f"系统: 命令已加入队列等待 FSM 33: {command}")
            
        # 3. 尝试发送队列中的第一个命令 (如果 FSM 状态已就绪)
        self.process_command_queue()
        
        # 4. 如果队列不空且 FSM 未就绪，确保请求 FSM 状态
        if self.command_queue and not self.is_fsm_standby:
            self.request_fsm_info()


    def request_real_time_data(self):
        """定时向机器人发送指令，请求获取实时关节和末端坐标数据。"""
        # 这个方法由 real_time_update_timer 定时调用。
        # [MODIFIED] 始终允许发送状态查询命令
        self._send_command_now("ReadActPos,0;", is_fsm_check=True)
        self._send_command_now("ReadRobotState,0;", is_fsm_check=True)
        
        # [新增] 如果有命令等待，确保检查 FSM 状态
        if self.command_queue and not self.is_fsm_standby:
            self.request_fsm_info()

    # [新增] 专门用于请求 FSM 状态
    def request_fsm_info(self):
        """发送指令，请求获取 FSM 状态。"""
        self._send_command_now("ReadCurFSM,0;", is_fsm_check=True)
    
    # [MODIFIED] 修改状态查询函数，使其直接使用 _send_command_now 绕过 FSM 检查
    def request_emergency_info(self):
        """定时向机器人发送指令，请求获取急停状态。"""
        self._send_command_now("ReadEmergencyInfo,0;", is_fsm_check=True)

    # [MODIFIED] 修改状态查询函数，使其直接使用 _send_command_now 绕过 FSM 检查
    def request_override_info(self):
        """定时向机器人发送指令，请求获取当前运动速率。"""
        self._send_command_now("ReadOverride,0;", is_fsm_check=True)