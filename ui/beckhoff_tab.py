import numpy as np
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, 
    QLabel, QLineEdit, QPushButton, QMessageBox, QGroupBox
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QObject # 导入 QThread, pyqtSignal, QObject
import pyads # 确保 pyads 已导入
import threading
import time

# ====== ADS 配置和变量名（从 WriteTest.py 复制）======
AMS_NET_ID = "192.168.10.100.1.1"
PLC_IP     = "192.168.10.100"
PLC_PORT   = 851

# 目标位置变量名 (J2, J3)
J2_PV   = "MAIN.J2"              
J3_PV   = "MAIN.J3"              

# 运动使能变量名：需要持续 True，完成后置回 False
START_BOOL = "MAIN.Position_5"    

# 目标与判据
DONE_TOL   = 0.5                  # 判定完成阈值
READ_T     = pyads.PLCTYPE_LREAL
WRITE_T    = pyads.PLCTYPE_LREAL
# ----------------------------------------------------

# 从 WriteTest.py 拷贝 ADS 线程安全封装类
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
            return float(self._c.read_by_name(name, READ_T))

    def write_lreal(self, name: str, val: float):
        with self.lock:
            self._c.write_by_name(name, float(val), WRITE_T)

    def write_bool(self, name: str, val: bool):
        with self.lock:
            self._c.write_by_name(name, bool(val), pyads.PLCTYPE_BOOL)

# ... (在 BeckhoffTab 类之前添加 ADSThread)
class ADSThread(QThread):
    """一个独立的线程用于执行 ADS 运动和监控到位状态。"""
    movement_status_update = pyqtSignal(str)
    
    def __init__(self, ads_client, target_j2, target_j3, parent=None):
        super().__init__(parent)
        self.ads = ads_client
        self.target_j2 = target_j2
        self.target_j3 = target_j3
        self.is_running = True

    def run(self):
        self.movement_status_update.emit("下发目标中...")
        try:
            # 1. 写入目标 J2/J3
            self.ads.write_lreal(J2_PV, self.target_j2)
            self.ads.write_lreal(J3_PV, self.target_j3)
            
            # 2. 启动运动 (Position_5 = True)
            self.ads.write_bool(START_BOOL, True)
            self.movement_status_update.emit("运动启动，等待到位...")
            
            # 3. 监控到位状态（此处简化为等待固定时间）
            # 注意：WriteTest.py 中是通过轮询线程实时监控 Pos[2]/Pos[3] 来判定到位。
            # 在 PyQt 中，我们通常会在主窗口的 ADS 轮询线程中执行这个监控。
            # 为了实现您要求的“一键触发”，我们在这里只发送启动命令。
            # PLC 侧应该配置为接收到 True 后立即启动，并且在接收到 False 后停止。
            
            # ***************************************************************
            # 警告：要实现 WriteTest.py 的自动回落功能，需要一个持续的轮询线程
            # 来读取实际位置并判断 DONE_TOL。由于您只要求实现“触发”功能，
            # 我们假设 PLC 侧在运动结束后**不会**自动将 Position_5 拉回 False。
            # 并且我们现在只发送启动信号，不在这里实现复杂的到位判定。
            # ***************************************************************
            
            self.movement_status_update.emit("J2/J3 目标和启动信号已发送。请手动停止 Position_5。")
            
        except Exception as e:
            self.movement_status_update.emit(f"ADS 写入或启动失败: {e}")
            
    def stop(self):
        self.is_running = False
        self.wait()

class BeckhoffTab(QWidget):
    """
    Beckhoff通信和穿刺针运动学计算标签页。
    """
    def __init__(self, robot_kinematics, parent=None):
        super().__init__(parent)
        self.robot = robot_kinematics
        
        # ADS 客户端实例
        self.ads_client = ADS(AMS_NET_ID, PLC_PORT, PLC_IP)
        self.ads_thread = None  # 用于执行 ADS 写入的线程

        # UI 元素存储
        self.vector_inputs = [None] * 3  
        self.result_labels = {}         
        self.ads_status_label = QLabel("ADS: 未连接") # 新增 ADS 状态标签
        
        # 存储计算出的 J2, J3 值
        self.latest_j2 = 0.0
        self.latest_j3 = 0.0
        
        self.init_ui()
        self.setup_connections()

    def init_ui(self):
        """构建Beckhoff通信标签页的UI。"""
        main_layout = QVBoxLayout(self)

        # -----------------------------------------------------------------
        # 1. 穿刺针 Vector 定位模块
        # ... (保持不变) ...
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
        result_group = QGroupBox("计算结果")
        result_layout = QGridLayout(result_group)
        
        result_labels = ["关节 J2 (度):", "关节 J3 (度):"]
        result_keys = ["J2", "J3"]
        
        for i, label_text in enumerate(result_labels):
            label = QLabel(label_text)
            value_label = QLabel("N/A")
            value_label.setStyleSheet("background-color: lightgrey; border: 1px inset grey;")
            self.result_labels[result_keys[i]] = value_label
            result_layout.addWidget(label, i, 0)
            result_layout.addWidget(value_label, i, 1)

        vector_layout.addWidget(result_group)
        
        main_layout.addWidget(vector_group)
        
        # -----------------------------------------------------------------
        # 2. Beckhoff PLC 通信模块 (集成连接/断开/触发按钮)
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
        # ***** 新增按钮 *****
        self.trigger_j2j3_btn = QPushButton("运动到计算出的 J2, J3")
        self.trigger_j2j3_btn.setEnabled(False) 
        
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

    def connect_ads(self):
        """尝试连接到 ADS。"""
        ok, msg = self.ads_client.open()
        self.ads_status_label.setText(f"ADS: {msg}")
        self.connect_ads_btn.setEnabled(not ok)
        self.disconnect_ads_btn.setEnabled(ok)
        self.trigger_j2j3_btn.setEnabled(ok)
        
    def disconnect_ads(self):
        """断开 ADS 连接。"""
        try:
            # 尝试在断开前将 Position_5 置回 False
            if self.ads_client.connected:
                 self.ads_client.write_bool(START_BOOL, False)
        except Exception:
            pass # 忽略关闭时的错误
            
        self.ads_client.close()
        self.ads_status_label.setText("ADS: 已断开")
        self.connect_ads_btn.setEnabled(True)
        self.disconnect_ads_btn.setEnabled(False)
        self.trigger_j2j3_btn.setEnabled(False)


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
            
            # 显示结果
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

    def trigger_j2j3_move(self):
        """触发 J2/J3 关节运动（相当于 WriteTest.py 中的 start_move）。"""
        if not self.ads_client.connected:
            QMessageBox.warning(self, "警告", "ADS 未连接。请先连接 Beckhoff PLC。")
            return
            
        if self.ads_thread and self.ads_thread.isRunning():
            QMessageBox.warning(self, "警告", "正在执行其他 ADS 任务，请稍候。")
            return

        # 1. 检查是否有计算结果
        if self.latest_j2 == 0.0 and self.latest_j3 == 0.0 and self.result_labels["J2"].text() == "N/A":
             QMessageBox.warning(self, "警告", "请先计算 J2 和 J3 关节值。")
             return

        # 2. 启动 ADS 写入线程
        self.ads_thread = ADSThread(self.ads_client, self.latest_j2, self.latest_j3)
        self.ads_thread.movement_status_update.connect(self.update_movement_status)
        self.ads_thread.start()
        
    # 确保在 tab 关闭时断开 ADS 连接
    def cleanup(self):
        """在标签页关闭或主窗口关闭时调用，用于清理资源。"""
        self.disconnect_ads()
        if self.ads_thread and self.ads_thread.isRunning():
            self.ads_thread.stop()