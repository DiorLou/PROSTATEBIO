import numpy as np
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, 
    QLabel, QLineEdit, QPushButton, QMessageBox, QGroupBox
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QObject 
import pyads 
import threading
import time

# ====== ADS 配置和变量名（已修改，新增当前位置变量）======
AMS_NET_ID = "192.168.10.100.1.1"
PLC_IP     = "192.168.10.100"
PLC_PORT   = 851

# 目标位置变量名 (J2, J3)
J2_PV   = "MAIN.J2"              
J3_PV   = "MAIN.J3"              

# 运动使能变量名：需要持续 True，完成后置回 False
START_BOOL = "MAIN.Position_5"    

# 设置运动时间
MOTION_TIME = "MAIN.t5"   

# 新增：当前位置变量名 (从 WriteTest.py 复制)
POS1_PV = "MAIN.MotorCurPos[2]"   # 只读：当前位置1 (LREAL)
POS2_PV = "MAIN.MotorCurPos[3]"   # 只读：当前位置2 (LREAL)

# 目标与判据
# DONE_TOL   = 0.05                  # 判定完成阈值 (已移除)
READ_LREAL_T   = pyads.PLCTYPE_LREAL
WRITE_LREAL_T  = pyads.PLCTYPE_LREAL
WRITE_INT_T    = pyads.PLCTYPE_INT
# ----------------------------------------------------

class ADS:
    def __init__(self, ams, port, ip):
        self._c = pyads.Connection(ams, port, ip)
        self.connected = False
        self.lock = threading.Lock()

    def open(self):
        if self.connected:
            return True, "已连接"
        try:
            self._c.open()
            self._c.read_state()  # 健康检查
            self.connected = True
            return True, "ADS 连接成功"
        except Exception as e:
            self.connected = False
            return False, f"ADS 连接失败: {e}"

    def close(self):
        try:
            if self.connected:
                self._c.close()
        finally:
            self.connected = False

    def read_lreal(self, name: str) -> float:
        with self.lock:
            return float(self._c.read_by_name(name, READ_LREAL_T))

    def write_lreal(self, name: str, val: float):
        with self.lock:
            self._c.write_by_name(name, float(val), WRITE_LREAL_T)
            
    def write_int(self, name: str, val: int):
        with self.lock:
            self._c.write_by_name(name, int(val), WRITE_INT_T)

    def write_bool(self, name: str, val: bool):
        with self.lock:
            self._c.write_by_name(name, bool(val), pyads.PLCTYPE_BOOL)
            
    def read_bool(self, name: str) -> bool:
        with self.lock:
            # pyads 读取 PLCTYPE_BOOL 类型时返回 Python 布尔值
            return bool(self._c.read_by_name(name, pyads.PLCTYPE_BOOL))

# ====================================================================
# 新增：ADS 轮询线程 (负责实时读取位置和运动监控)
# ====================================================================
class ADSPollThread(QThread):
    """一个独立的线程，用于持续轮询读取当前位置，并监控运动到位状态。"""
    # 信号1：实时位置更新 (J2, J3)
    position_update = pyqtSignal(float, float)
    # 信号2：运动状态更新 (例如：就绪, 运动中, 运动完成)
    movement_status_update = pyqtSignal(str)
    
    def __init__(self, ads_client, parent=None):
        super().__init__(parent)
        self.ads = ads_client
        self.is_running = True
        self.moving = False          # 标记是否正在监控运动
        # self._stable_cnt = 0       # 已移除
        
        # 目标值 (由 BeckhoffTab 在触发运动前设置)
        self.target_j2 = 0.0
        self.target_j3 = 0.0
        self.motion_time = 0.0

    def start_monitoring_move(self, target_j2, target_j3):
        """设置目标值，并开始监控运动。（已移除 _stable_cnt 重置）"""
        self.target_j2 = target_j2
        self.target_j3 = target_j3
        self.moving = True

    def run(self):
        last_status = ""
        while self.is_running and self.ads.connected:
            try:
                # 1. 读取当前位置
                p1 = self.ads.read_lreal(POS1_PV)
                p2 = self.ads.read_lreal(POS2_PV)
                
                # 2. 发送信号更新 UI
                self.position_update.emit(p1, p2)

                if self.moving:
                    # 3. 判定到位：读取 PLC 中的 START_BOOL (MAIN.Position_5)
                    is_moving_flag = self.ads.read_bool(START_BOOL)
                    
                    if not is_moving_flag:
                        # 4. 到位：PLC 已将 Position_5 置回 False，运动完成。
                        self.moving = False
                        self.movement_status_update.emit("运动完成")
                        
                    else:
                        if last_status != "运动中":
                            self.movement_status_update.emit("运动中")
                            last_status = "运动中"
                else:
                    if last_status != "就绪":
                        self.movement_status_update.emit("就绪")
                        last_status = "就绪"

            except Exception as e:
                # 只有在连接仍然存在时才报告错误，否则是断开操作导致的
                if self.ads.connected: 
                    self.movement_status_update.emit(f"轮询读失败: {e}")
                break
            
            time.sleep(0.15) # 0.15 秒轮询间隔
        
        print("ADS 轮询线程已退出。")

    def stop(self):
        self.is_running = False
        self.wait()


# ====================================================================
# 修改：ADSThread (现在仅负责发送启动命令)
# ====================================================================
class ADSThread(QThread):
    """一个独立的线程用于执行 ADS 运动命令的发送。"""
    movement_status_update = pyqtSignal(str) # 负责发送执行状态

    def __init__(self, ads_client, target_j2, target_j3, target_time, parent=None):
        super().__init__(parent)
        self.ads = ads_client
        self.target_j2 = target_j2
        self.target_j3 = target_j3
        self.motion_time = target_time # 此处 target_time 期望为毫秒 (ms)

    def run(self):
        self.movement_status_update.emit("下发目标中...")
        try:
            # 1. 写入目标 J2/J3/TIME
            self.ads.write_lreal(J2_PV, self.target_j2)
            self.ads.write_lreal(J3_PV, self.target_j3)
            # MOTION_TIME (MAIN.t5) 期望单位为毫秒 (ms)
            self.ads.write_int(MOTION_TIME, self.motion_time) 
            
            # 2. 启动运动 (Position_5 = True)
            self.ads.write_bool(START_BOOL, True)
            self.movement_status_update.emit("运动指令已发送，准备开始监控...")
            
        except Exception as e:
            self.movement_status_update.emit(f"ADS 写入或启动失败: {e}")


# ====================================================================
# 修改：BeckhoffTab 类
# ====================================================================
class BeckhoffTab(QWidget):
    """
    Beckhoff通信和穿刺针运动学计算标签页。
    """
    def __init__(self, robot_kinematics, parent=None):
        super().__init__(parent)
        self.robot = robot_kinematics
        
        # ADS 客户端实例
        self.ads_client = ADS(AMS_NET_ID, PLC_PORT, PLC_IP)
        self.ads_command_thread = None  # 用于执行 ADS 写入命令的线程
        self.ads_poll_thread = None     # 新增：用于实时轮询和运动监控的线程

        # UI 元素存储
        self.vector_inputs = [None] * 3  
        self.result_labels = {}         
        self.ads_status_label = QLabel("ADS: 未连接") 
        self.pos_labels = {}            # 新增：实时位置显示标签
        self.motion_time_input = None   # 新增：目标运动时间输入框
        
        # 存储计算出的 J2, J3 值 (用于暂存，但实际目标以 QLineEdit 为准)
        self.latest_j2 = 0.0
        self.latest_j3 = 0.0
        
        # 恢复按钮成员变量引用
        self.reset_j2j3_btn = None 
        self.trigger_j2j3_btn = None 
        self.input_vector_btn = None
        
        # UI 初始化，调用 init_ui 会创建所有按钮实例
        self.init_ui()
        self.setup_connections()

    def init_ui(self):
        """构建Beckhoff通信标签页的UI。（关键修改：设置 QGridLayout 居左）"""
        main_layout = QVBoxLayout(self)

        # -----------------------------------------------------------------
        # 1. 穿刺针 Vector 定位模块
        # -----------------------------------------------------------------
        vector_group = QGroupBox("穿刺针Vector定位 (逆运动学)")
        vector_layout = QVBoxLayout(vector_group)

        # Vector 输入部分
        input_grid = QGridLayout()
        vector_labels = ["X:", "Y:", "Z:"]
        for i, label_text in enumerate(vector_labels):
            label = QLabel(label_text)
            text_box = QLineEdit("0.00")
            text_box.setPlaceholderText("请输入单位向量值")
            self.vector_inputs[i] = text_box
            input_grid.addWidget(label, 0, i * 2)
            input_grid.addWidget(text_box, 0, i * 2 + 1)
        
        self.input_vector_btn = QPushButton("计算关节值 (J2, J3)")
        
        vector_layout.addLayout(input_grid)
        vector_layout.addWidget(self.input_vector_btn, alignment=Qt.AlignCenter)
        
        # 结果显示部分
        result_group = QGroupBox("位置状态")
        
        # 创建一个 QHBoxLayout 来包装 QGridLayout，并将其左对齐
        result_content_layout = QHBoxLayout()
        result_layout = QGridLayout()
        
        # 目标位置显示 (改为 QLineEdit 实现可读写)
        result_labels = ["目标 J2 (度):", "目标 J3 (度):"]
        result_keys = ["J2", "J3"]
        
        # 行索引
        row = 0
        for i, label_text in enumerate(result_labels):
            label = QLabel(label_text)
            # 确保标签左对齐
            label.setAlignment(Qt.AlignVCenter) 
            
            # 目标输入框：设置固定宽度
            value_input = QLineEdit("0.00") 
            value_input.setReadOnly(False) 
            value_input.setFixedWidth(100) 
            value_input.setStyleSheet("background-color: white; border: 1px inset grey;")
            self.result_labels[result_keys[i]] = value_input
            
            # Column 0: Target Label
            result_layout.addWidget(label, row, 0)
            # Column 1: Target Input
            result_layout.addWidget(value_input, row, 1) 
            row += 1 # 递增行索引
            
        # *********** 根据用户要求新增：目标运动时间输入框 (在 J3 正下方) ***********
        
        # 目标运动时间输入
        time_label = QLabel("目标运动时间 (ms):")
        # 确保标签左对齐
        time_label.setAlignment(Qt.AlignVCenter) 
        
        self.motion_time_input = QLineEdit("500") # 默认 500ms
        self.motion_time_input.setReadOnly(False)
        self.motion_time_input.setFixedWidth(100)
        self.motion_time_input.setStyleSheet("background-color: white; border: 1px inset grey;")
        
        # Row 2, Col 0: Label (此时 row = 2)
        result_layout.addWidget(time_label, row, 0)
        # Row 2, Col 1: Input
        result_layout.addWidget(self.motion_time_input, row, 1) 
        row += 1 # 递增行索引
        
        # *************************************************************************

        # *** 新增固定间隔列 (Column 2) ***
        spacer_label = QLabel(" ")
        spacer_label.setFixedWidth(100) # 100像素的间隔
        # 更新间隔列，以包含新增的第三行 (共 row 行)
        for r in range(row):
            result_layout.addWidget(QLabel(" "), r, 2) 
            
        # 实时当前位置显示 (QLabel) - 保持不变，仍在 Col 3/4
        pos_labels = ["当前 J2 (度):", "当前 J3 (度):"]
        pos_keys = ["CurJ2", "CurJ3"]
        row_cur = 0
        for i, label_text in enumerate(pos_labels):
            label = QLabel(label_text)
            # 确保标签左对齐
            label.setAlignment(Qt.AlignVCenter)
            
            value_label = QLabel("--")
            
            # 当前值标签：设置固定宽度
            value_label.setFixedWidth(100) 
            
            value_label.setStyleSheet("background-color: #D4EDF7; border: 1px inset grey;") 
            self.pos_labels[pos_keys[i]] = value_label
            
            # Column 3: Current Label 
            result_layout.addWidget(label, row_cur, 3)
            # Column 4: Current Value
            result_layout.addWidget(value_label, row_cur, 4)
            row_cur += 1
            
        # 将 result_layout (QGridLayout) 添加到 result_content_layout (QHBoxLayout) 中
        result_content_layout.addLayout(result_layout)
        # 添加 stretch，将 QGridLayout 推到左侧
        result_content_layout.addStretch(1) 
        
        result_group.setLayout(result_content_layout) # 设置 GroupBox 的布局
        
        vector_layout.addWidget(result_group)
        
        main_layout.addWidget(vector_group)
        
        # -----------------------------------------------------------------
        # 2. Beckhoff PLC 通信模块 
        # -----------------------------------------------------------------
        beckhoff_comm_group = QGroupBox("Beckhoff PLC 通信")
        beckhoff_comm_layout = QVBoxLayout(beckhoff_comm_group)

        # 连接/状态行
        conn_layout = QHBoxLayout()
        self.connect_ads_btn = QPushButton("连接 ADS")
        self.disconnect_ads_btn = QPushButton("断开 ADS")
        self.disconnect_ads_btn.setEnabled(False)
        
        conn_layout.addWidget(self.connect_ads_btn)
        conn_layout.addWidget(self.disconnect_ads_btn)
        conn_layout.addWidget(self.ads_status_label)
        beckhoff_comm_layout.addLayout(conn_layout)

        # J2/J3 触发按钮行
        trigger_layout = QHBoxLayout()
        
        self.reset_j2j3_btn = QPushButton("重置 J2, J3")
        self.reset_j2j3_btn.setEnabled(False)
        
        self.trigger_j2j3_btn = QPushButton("运动到目标 J2, J3")
        self.trigger_j2j3_btn.setEnabled(False) 
        
        trigger_layout.addWidget(self.reset_j2j3_btn) 
        trigger_layout.addWidget(self.trigger_j2j3_btn) 
        beckhoff_comm_layout.addLayout(trigger_layout)
        
        # 新增一个 QLabel 用于显示运动状态
        self.movement_status_label = QLabel("运动状态: 待命")
        beckhoff_comm_layout.addWidget(self.movement_status_label)

        main_layout.addWidget(beckhoff_comm_group)
        main_layout.addStretch()

    def setup_connections(self):
        """连接信号和槽。"""
        self.input_vector_btn.clicked.connect(self.calculate_joint_values)
        self.connect_ads_btn.clicked.connect(self.connect_ads)
        self.disconnect_ads_btn.clicked.connect(self.disconnect_ads)
        self.trigger_j2j3_btn.clicked.connect(self.trigger_j2j3_move)
        self.reset_j2j3_btn.clicked.connect(self.trigger_j2j3_reset)

# ====================================================================
# 新增：轮询管理方法
# ====================================================================
    def _start_poll(self):
        """启动 ADS 轮询线程。"""
        if self.ads_poll_thread is None or not self.ads_poll_thread.isRunning():
            self.ads_poll_thread = ADSPollThread(self.ads_client)
            # 连接轮询线程的信号
            self.ads_poll_thread.position_update.connect(self._update_current_positions)
            self.ads_poll_thread.movement_status_update.connect(self.update_movement_status)
            self.ads_poll_thread.start()

    def _stop_poll(self):
        """停止 ADS 轮询线程。"""
        if self.ads_poll_thread and self.ads_poll_thread.isRunning():
            self.ads_poll_thread.stop()
            self.ads_poll_thread = None

    def _update_current_positions(self, j2_pos, j3_pos):
        """槽函数：接收并更新当前的 J2 和 J3 关节位置。"""
        # 实时显示当前位置
        self.pos_labels["CurJ2"].setText(f"{j2_pos:.3f}")
        self.pos_labels["CurJ3"].setText(f"{j3_pos:.3f}")


    def connect_ads(self):
        """尝试连接到 ADS。（新增：启动轮询线程）"""
        ok, msg = self.ads_client.open()
        self.ads_status_label.setText(f"ADS: {msg}")
        self.connect_ads_btn.setEnabled(not ok)
        self.disconnect_ads_btn.setEnabled(ok)
        self.trigger_j2j3_btn.setEnabled(ok)
        self.reset_j2j3_btn.setEnabled(ok)
        
        if ok:
            self._start_poll() # 启动轮询线程

    def disconnect_ads(self):
        """断开 ADS 连接。（已移除强制回落 START_BOOL 的代码）"""
        self._stop_poll() # 停止轮询线程

        self.ads_client.close()
        self.ads_status_label.setText("ADS: 已断开")
        self.connect_ads_btn.setEnabled(True)
        self.disconnect_ads_btn.setEnabled(False)
        self.trigger_j2j3_btn.setEnabled(False)
        self.reset_j2j3_btn.setEnabled(False)
        
        # 清除当前位置显示
        self.pos_labels["CurJ2"].setText("--")
        self.pos_labels["CurJ3"].setText("--")

    def calculate_joint_values(self):
        """读取 Vector 值，计算 J2 和 J3 关节角，并显示结果。"""
        try:
            x = float(self.vector_inputs[0].text())
            y = float(self.vector_inputs[1].text())
            z = float(self.vector_inputs[2].text())
            
            needle_vector = np.array([x, y, z])
            
            # 自动归一化处理
            norm = np.linalg.norm(needle_vector)
            if not np.isclose(norm, 1.0) and norm > 1e-6:
                 QMessageBox.warning(self, "警告", f"输入向量的模长为 {norm:.4f}，已自动归一化。")
                 needle_vector = needle_vector / norm
            elif norm < 1e-6:
                 QMessageBox.critical(self, "输入错误", "Vector 模长接近零，无法定义方向。")
                 return
            
            # 调用运动学逆解函数
            joint_values = self.robot.get_joint23_value(needle_vector)
            
            # 存储最新的 J2/J3 值
            self.latest_j2 = joint_values[0]
            self.latest_j3 = joint_values[1]
            
            # 显示结果：设置 QLineEdit 的文本
            self.result_labels["J2"].setText(f"{self.latest_j2:.4f}")
            self.result_labels["J3"].setText(f"{self.latest_j3:.4f}")
            
            self.movement_status_label.setText("运动状态: J2/J3 计算完成")

            # 尝试更新主窗口的状态栏
            parent_window = self.parent()
            if hasattr(parent_window, 'status_bar'):
                parent_window.status_bar.showMessage(f"状态: 关节 J2={self.latest_j2:.4f}, J3={self.latest_j3:.4f} 计算完成。")
            
        except ValueError:
            QMessageBox.critical(self, "输入错误", "Vector X, Y, Z 必须是有效数字！")
        except Exception as e:
            QMessageBox.critical(self, "计算错误", f"逆运动学求解失败: {e}")

    def update_movement_status(self, msg):
        """槽函数：更新运动状态标签。"""
        self.movement_status_label.setText(f"运动状态: {msg}")

# ====================================================================
# 修改：运动触发方法 (直接读取并使用 ms 值)
# ====================================================================
    def trigger_j2j3_move(self):
        """触发 J2/J3 关节运动。"""
        if not self.ads_client.connected:
            QMessageBox.warning(self, "警告", "ADS 未连接。请先连接 Beckhoff PLC。")
            return
            
        # 检查是否正在监控运动
        if self.ads_poll_thread and self.ads_poll_thread.moving:
            QMessageBox.warning(self, "警告", "正在执行运动监控任务，请稍候。")
            return

        # 1. 强制从 QLineEdit 中读取目标值
        try:
            target_j2 = float(self.result_labels["J2"].text())
            target_j3 = float(self.result_labels["J3"].text())
        except ValueError:
            QMessageBox.critical(self, "输入错误", "目标 J2/J3 必须是有效数字。")
            return
            
        # 2. 读取运动时间（ms），并直接使用
        try:
            target_time = float(self.motion_time_input.text())
            if target_time <= 0:
                raise ValueError("运动时间必须大于零。")
        except ValueError as e:
            QMessageBox.critical(self, "输入错误", "目标运动时间（毫秒）必须是有效数字且大于零！")
            return
        
        # 3. 启动 ADS 写入线程 (发送指令)
        self.ads_command_thread = ADSThread(self.ads_client, target_j2, target_j3, target_time) # 直接传递毫秒值
        self.ads_command_thread.movement_status_update.connect(self.update_movement_status)
        # 关键：指令线程结束后，启动轮询线程的运动监控
        self.ads_command_thread.finished.connect(lambda: self._start_poll_monitoring(target_j2, target_j3))
        self.ads_command_thread.start()

    def trigger_j2j3_reset(self):
        """触发 J2/J3 关节运动到预设的重置位置。"""
        if not self.ads_client.connected:
            QMessageBox.warning(self, "警告", "ADS 未连接。请先连接 Beckhoff PLC。")
            return
            
        if self.ads_poll_thread and self.ads_poll_thread.moving:
            QMessageBox.warning(self, "警告", "正在执行运动监控任务，请稍候。")
            return

        RESET_J2 = 71.46
        RESET_J3 = 23.9
        
        # 1. 读取运动时间（ms），并直接使用
        try:
            target_time = float(self.motion_time_input.text())
            if target_time <= 0:
                raise ValueError("运动时间必须大于零。")
        except ValueError:
            QMessageBox.critical(self, "输入错误", "目标运动时间（毫秒）必须是有效数字且大于零！")
            return

        # 2. 立即更新 QLineEdit 文本
        self.result_labels["J2"].setText(f"{RESET_J2:.4f}")
        self.result_labels["J3"].setText(f"{RESET_J3:.4f}")

        # 3. 启动 ADS 写入线程 (发送指令)
        self.ads_command_thread = ADSThread(self.ads_client, RESET_J2, RESET_J3, target_time) # 直接传递毫秒值
        self.ads_command_thread.movement_status_update.connect(self.update_movement_status)
        self.ads_command_thread.finished.connect(lambda: self._start_poll_monitoring(RESET_J2, RESET_J3))
        self.ads_command_thread.start()
        
        # 立即更新状态栏
        parent_window = self.parent()
        if hasattr(parent_window, 'status_bar'):
             parent_window.status_bar.showMessage(f"状态: 已发送重置指令: J2={RESET_J2:.2f}, J3={RESET_J3:.2f}。")

    def _start_poll_monitoring(self, target_j2, target_j3):
        """在运动指令发送完成后，启动轮询线程的运动监控。"""
        if self.ads_poll_thread:
            # 目标位置已在 ADSThread 中写入，现在通知轮询线程开始监控到位状态
            self.ads_poll_thread.start_monitoring_move(target_j2, target_j3)
            self.update_movement_status("运动启动，开始监控到位...")


    def cleanup(self):
        """在标签页关闭或主窗口关闭时调用，用于清理资源。（新增：停止轮询线程）"""
        self._stop_poll()
        self.disconnect_ads()
        if self.ads_command_thread and self.ads_command_thread.isRunning():
            self.ads_command_thread.wait() # 确保指令线程结束