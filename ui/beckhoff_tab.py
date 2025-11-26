import numpy as np
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, 
    QLabel, QLineEdit, QPushButton, QMessageBox, QGroupBox
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QObject 
import pyads 
import threading
import time

# ====== ADS 配置和变量名 ======
AMS_NET_ID = "192.168.10.100.1.1"
PLC_IP     = "192.168.10.100"
PLC_PORT   = 851

# 目标位置变量名 (J2, J3)
J2_PV   = "MAIN.J2"              
J3_PV   = "MAIN.J3"              

# 运动使能变量名：需要持续 True，完成后置回 False
START_BOOL = "MAIN.Position_5"    

# 新增：Beckhoff 电机使能变量
ENABLE_STATUS = "MAIN.MotorButtonEnable"

# 设置运动时间
MOTION_TIME = "MAIN.t5"   

# 新增：当前位置变量名
POS1_PV = "MAIN.MotorCurPos[2]"   # 只读：当前位置1 (LREAL)
POS2_PV = "MAIN.MotorCurPos[3]"   # 只读：当前位置2 (LREAL)

READ_LREAL_T   = pyads.PLCTYPE_LREAL
WRITE_LREAL_T  = pyads.PLCTYPE_LREAL
WRITE_INT_T    = pyads.PLCTYPE_INT
# ----------------------------------------------------

class ADS:
    def __init__(self, ams, port, ip):
        super().__init__()
        self._c = pyads.Connection(ams, port, ip)
        self.connected = False
        self.lock = threading.Lock()

    def open(self):
        if self.connected:
            return True, "Connected"
        try:
            self._c.open()
            self._c.read_state()  # 健康检查
            self.connected = True
            return True, "ADS Connection Successful"
        except Exception as e:
            self.connected = False
            return False, f"ADS Connection Failed: {e}"

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
            return bool(self._c.read_by_name(name, pyads.PLCTYPE_BOOL))

# ====================================================================
# ADS 轮询线程
# ====================================================================
class ADSPollThread(QThread):
    position_update = pyqtSignal(float, float)
    movement_status_update = pyqtSignal(str)
    enable_status_update = pyqtSignal(bool)
    
    def __init__(self, ads_client, parent=None):
        super().__init__(parent)
        self.ads = ads_client
        self.is_running = True
        self.moving = False 
        
        self.target_j2 = 0.0
        self.target_j3 = 0.0
        self.motion_time = 0.0

    def start_monitoring_move(self, target_j2, target_j3):
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
                self.position_update.emit(p1, p2)

                # 3. 读取使能状态
                is_enabled = self.ads.read_bool(ENABLE_STATUS)
                self.enable_status_update.emit(is_enabled)

                if self.moving:
                    is_moving_flag = self.ads.read_bool(START_BOOL)
                    if not is_moving_flag:
                        self.moving = False
                        self.movement_status_update.emit("Movement Completed")
                    else:
                        if last_status != "Moving":
                            self.movement_status_update.emit("Moving")
                            last_status = "Moving"
                else:
                    if last_status != "Ready":
                        self.movement_status_update.emit("Ready")
                        last_status = "Ready"

            except Exception as e:
                if self.ads.connected: 
                    self.movement_status_update.emit(f"Polling Read Failed: {e}")
                break
            time.sleep(0.15)
        print("ADS 轮询线程已退出。")

    def stop(self):
        self.is_running = False
        self.wait()

# ====================================================================
# ADSThread (发送启动命令)
# ====================================================================
class ADSThread(QThread):
    movement_status_update = pyqtSignal(str)

    def __init__(self, ads_client, target_j2, target_j3, target_time, parent=None):
        super().__init__(parent)
        self.ads = ads_client
        self.target_j2 = target_j2
        self.target_j3 = target_j3
        self.motion_time = target_time

    def run(self):
        self.movement_status_update.emit("Sending Target...")
        try:
            self.ads.write_lreal(J2_PV, self.target_j2)
            self.ads.write_lreal(J3_PV, self.target_j3)
            self.ads.write_int(MOTION_TIME, self.motion_time) 
            self.ads.write_bool(START_BOOL, True)
            self.movement_status_update.emit("Movement command sent, preparing to monitor...")
        except Exception as e:
            self.movement_status_update.emit(f"ADS Write or Start Failed: {e}")


# ====================================================================
# BeckhoffTab 类
# ====================================================================
class BeckhoffTab(QWidget):
    # 定义复位常量
    RESET_J2 = 67.569
    RESET_J3 = 20.347

    def __init__(self, robot_kinematics, parent=None):
        super().__init__(parent)
        self.robot = robot_kinematics
        
        self.ads_client = ADS(AMS_NET_ID, PLC_PORT, PLC_IP)
        self.ads_command_thread = None
        self.ads_poll_thread = None

        self.vector_inputs = [None] * 3  
        self.result_labels = {}         
        self.ads_status_label = QLabel("ADS: Disconnected") 
        self.pos_labels = {}
        self.motion_time_input = None
        self.enable_motor_btn = None
        
        self.inc_j2_input = None
        self.inc_j3_input = None
        self.apply_inc_btn = None
        
        self.latest_j2 = 0.0
        self.latest_j3 = 0.0
        
        self.reset_j2j3_btn = None 
        self.input_vector_btn = None
        
        self.init_ui()
        self.setup_connections()

    def init_ui(self):
        main_layout = QVBoxLayout(self)

        # -----------------------------------------------------------------
        # 1. 穿刺针 Vector 定位模块
        # -----------------------------------------------------------------
        vector_group = QGroupBox("Needle Vector Positioning (Inverse Kinematics)")
        vector_layout = QVBoxLayout(vector_group)

        input_grid = QGridLayout()
        vector_labels = ["X:", "Y:", "Z:"]
        for i, label_text in enumerate(vector_labels):
            label = QLabel(label_text)
            text_box = QLineEdit("0.00")
            text_box.setPlaceholderText("Enter Unit Vector Value")
            self.vector_inputs[i] = text_box
            input_grid.addWidget(label, 0, i * 2)
            input_grid.addWidget(text_box, 0, i * 2 + 1)
        
        # --- [修改] 按钮布局区域 ---
        btns_layout = QHBoxLayout()
        
        # 1. 创建 Calc ΔJ1 按钮 (放左边)
        self.calc_j1_btn = QPushButton("Calc ΔJ1")
        self.calc_j1_btn.setFixedWidth(100)
        
        # 2. 创建/修改 Calc ΔJ2, ΔJ3 按钮 (放右边，改名)
        self.input_vector_btn = QPushButton("Calc ΔJ2, ΔJ3")
        # self.input_vector_btn.setFixedWidth(120) # 可选：设置固定宽度

        btns_layout.addStretch()
        btns_layout.addWidget(self.calc_j1_btn)       # 左
        btns_layout.addWidget(self.input_vector_btn)  # 右
        btns_layout.addStretch()

        vector_layout.addLayout(input_grid)
        vector_layout.addLayout(btns_layout)
        
        # 结果显示部分
        result_group = QGroupBox("Position Control")
        result_content_layout = QHBoxLayout()
        result_layout = QGridLayout()
        
        # 隐藏的 Target 输入框
        self.result_labels["J2"] = QLineEdit("0.00")
        self.result_labels["J3"] = QLineEdit("0.00")
        self.motion_time_input = QLineEdit("5000") 

        # --- [修改] Increase J1 (Height) 输入 ---
        # 按钮已经移走了，这里只保留显示框
        inc_j1_label = QLabel("Increase J1 (mm):")
        inc_j1_label.setAlignment(Qt.AlignVCenter)
        self.inc_j1_input = QLineEdit("0.00") 
        self.inc_j1_input.setFixedWidth(100)
        
        result_layout.addWidget(inc_j1_label, 0, 0) 
        result_layout.addWidget(self.inc_j1_input, 0, 1)
        # 注意：这里不再添加 self.calc_j1_btn
        
        # --- Increase J2 输入 ---
        inc_j2_label = QLabel("Increase J2 (Deg):")
        inc_j2_label.setAlignment(Qt.AlignVCenter)
        self.inc_j2_input = QLineEdit("0.00") 
        self.inc_j2_input.setFixedWidth(100)
        result_layout.addWidget(inc_j2_label, 1, 0) 
        result_layout.addWidget(self.inc_j2_input, 1, 1) 
        
        # --- Increase J3 输入 ---
        inc_j3_label = QLabel("Increase J3 (Deg):")
        inc_j3_label.setAlignment(Qt.AlignVCenter)
        self.inc_j3_input = QLineEdit("0.00") 
        self.inc_j3_input.setFixedWidth(100)
        result_layout.addWidget(inc_j3_label, 2, 0) 
        result_layout.addWidget(self.inc_j3_input, 2, 1) 
        
        # --- 间隔 ---
        spacer_label = QLabel(" ")
        spacer_label.setFixedWidth(20) 
        result_layout.addWidget(spacer_label, 1, 2)
        
        # --- Current 显示 ---
        pos_labels = ["Current J2 (Deg):", "Current J3 (Deg):"]
        pos_keys = ["CurJ2", "CurJ3"]
        
        # Current J2
        cur_j2_label = QLabel(pos_labels[0])
        cur_j2_label.setAlignment(Qt.AlignVCenter)
        self.pos_labels[pos_keys[0]] = QLineEdit("--")
        self.pos_labels[pos_keys[0]].setReadOnly(True)
        self.pos_labels[pos_keys[0]].setFixedWidth(100)
        self.pos_labels[pos_keys[0]].setStyleSheet("background-color: #D4EDF7; border: 1px inset grey;")
        result_layout.addWidget(cur_j2_label, 1, 3)
        result_layout.addWidget(self.pos_labels[pos_keys[0]], 1, 4)
        
        # Current J3
        cur_j3_label = QLabel(pos_labels[1])
        cur_j3_label.setAlignment(Qt.AlignVCenter)
        self.pos_labels[pos_keys[1]] = QLineEdit("--")
        self.pos_labels[pos_keys[1]].setReadOnly(True)
        self.pos_labels[pos_keys[1]].setFixedWidth(100)
        self.pos_labels[pos_keys[1]].setStyleSheet("background-color: #D4EDF7; border: 1px inset grey;")
        result_layout.addWidget(cur_j3_label, 2, 3)
        result_layout.addWidget(self.pos_labels[pos_keys[1]], 2, 4)

        # --- 底部按钮 ---
        # 1. Apply Increment 按钮
        self.apply_inc_btn = QPushButton("Apply Increment (J2, J3)")
        result_layout.addWidget(self.apply_inc_btn, 3, 0, 1, 2) 

        # 2. Reset 按钮
        self.reset_j2j3_btn = QPushButton("Reset J2, J3")
        self.reset_j2j3_btn.setEnabled(False) 
        result_layout.addWidget(self.reset_j2j3_btn, 3, 3, 1, 2)
        
        result_content_layout.addLayout(result_layout)
        result_content_layout.addStretch(1) 
        
        result_group.setLayout(result_content_layout)
        vector_layout.addWidget(result_group)
        main_layout.addWidget(vector_group)
        
        # -----------------------------------------------------------------
        # 2. Beckhoff PLC 通信模块 
        # -----------------------------------------------------------------
        beckhoff_comm_group = QGroupBox("Beckhoff PLC Communication")
        beckhoff_comm_layout = QVBoxLayout(beckhoff_comm_group)

        # 连接/状态行
        conn_layout = QHBoxLayout()
        self.connect_ads_btn = QPushButton("Connect ADS")
        self.disconnect_ads_btn = QPushButton("Disconnect ADS")
        self.disconnect_ads_btn.setEnabled(False)
        
        # 使能按钮
        self.enable_motor_btn = QPushButton("Enable")
        self.enable_motor_btn.setEnabled(False)
        self.enable_motor_btn.setStyleSheet("background-color: lightgray;")
        
        conn_layout.addWidget(self.connect_ads_btn)
        conn_layout.addWidget(self.disconnect_ads_btn)
        conn_layout.addWidget(self.enable_motor_btn) 
        conn_layout.addWidget(self.ads_status_label)
        beckhoff_comm_layout.addLayout(conn_layout)

        # 运动状态
        self.movement_status_label = QLabel("Movement Status: Standby")
        beckhoff_comm_layout.addWidget(self.movement_status_label)

        main_layout.addWidget(beckhoff_comm_group)
        main_layout.addStretch()

    def setup_connections(self):
        self.input_vector_btn.clicked.connect(self.calculate_joint_values)
        self.connect_ads_btn.clicked.connect(self.connect_ads)
        self.disconnect_ads_btn.clicked.connect(self.disconnect_ads)
        # 移除了 self.trigger_j2j3_btn.clicked.connect(...)
        self.reset_j2j3_btn.clicked.connect(self.trigger_j2j3_reset)
        self.apply_inc_btn.clicked.connect(self.apply_joint_increment)
        self.enable_motor_btn.clicked.connect(self.toggle_motor_enable)

    def _start_poll(self):
        if self.ads_poll_thread is None or not self.ads_poll_thread.isRunning():
            self.ads_poll_thread = ADSPollThread(self.ads_client)
            self.ads_poll_thread.position_update.connect(self._update_current_positions)
            self.ads_poll_thread.movement_status_update.connect(self.update_movement_status)
            self.ads_poll_thread.enable_status_update.connect(self._update_enable_button)
            self.ads_poll_thread.start()

    def _stop_poll(self):
        if self.ads_poll_thread and self.ads_poll_thread.isRunning():
            self.ads_poll_thread.stop()
            self.ads_poll_thread = None

    def _update_current_positions(self, j2_pos, j3_pos):
        self.pos_labels["CurJ2"].setText(f"{j2_pos:.3f}")
        self.pos_labels["CurJ3"].setText(f"{j3_pos:.3f}")
        
    def _update_enable_button(self, is_enabled: bool):
        if is_enabled:
            self.enable_motor_btn.setText("Enabled")
            self.enable_motor_btn.setStyleSheet("background-color: lightgreen;")
        else:
            self.enable_motor_btn.setText("Enable")
            self.enable_motor_btn.setStyleSheet("background-color: lightgray;")

    def connect_ads(self):
        ok, msg = self.ads_client.open()
        self.ads_status_label.setText(f"ADS: {msg}")
        self.connect_ads_btn.setEnabled(not ok)
        self.disconnect_ads_btn.setEnabled(ok)
        # 移除了 self.trigger_j2j3_btn.setEnabled(ok)
        self.reset_j2j3_btn.setEnabled(ok)
        self.enable_motor_btn.setEnabled(ok)
        
        if ok:
            self._start_poll()

    def disconnect_ads(self):
        self._stop_poll()
        self.ads_client.close()
        self.ads_status_label.setText("ADS: Disconnected")
        self.connect_ads_btn.setEnabled(True)
        self.disconnect_ads_btn.setEnabled(False)
        # 移除了 self.trigger_j2j3_btn.setEnabled(False)
        self.reset_j2j3_btn.setEnabled(False)
        self.enable_motor_btn.setEnabled(False)
        
        self.pos_labels["CurJ2"].setText("--")
        self.pos_labels["CurJ3"].setText("--")

    def toggle_motor_enable(self):
        if not self.ads_client.connected:
            QMessageBox.warning(self, "Warning", "ADS is disconnected. Please connect Beckhoff PLC first.")
            return
        try:
            current_state = self.ads_client.read_bool(ENABLE_STATUS)
            new_state = not current_state
            self.ads_client.write_bool(ENABLE_STATUS, new_state)
            action = "Disable" if not new_state else "Enable"
            self.movement_status_label.setText(f"Movement Status: Sent {action} command ({new_state})")
        except Exception as e:
            QMessageBox.critical(self, "ADS Write Error", f"Failed to send Enable/Disable command: {e}")

    def calculate_joint_values(self):
        try:
            x = float(self.vector_inputs[0].text())
            y = float(self.vector_inputs[1].text())
            z = float(self.vector_inputs[2].text())
            needle_vector = np.array([x, y, z])
            norm = np.linalg.norm(needle_vector)
            if not np.isclose(norm, 1.0) and norm > 1e-6:
                 QMessageBox.warning(self, "Warning", f"Input vector magnitude is {norm:.4f}, automatically normalized.")
                 needle_vector = needle_vector / norm
            elif norm < 1e-6:
                 QMessageBox.critical(self, "Input Error", "Vector magnitude is close to zero, cannot define direction.")
                 return
            joint_values = self.robot.get_joint23_value(needle_vector)
            self.latest_j2 = joint_values[0]
            self.latest_j3 = joint_values[1] + joint_values[0]
            
            # --- [修改部分开始] ---
            # 原代码将结果放入了 result_labels (Target J2/J3)
            # self.result_labels["J2"].setText(f"{self.latest_j2:.4f}")
            # self.result_labels["J3"].setText(f"{self.latest_j3:.4f}")

            # 新代码：将结果放入 Increase J2/J3 文本框
            self.inc_j2_input.setText(f"{self.latest_j2:.4f}")
            self.inc_j3_input.setText(f"{self.latest_j3:.4f}")
            # --- [修改部分结束] ---

            self.movement_status_label.setText("Movement Status: J2/J3 Calculation Completed")
            parent_window = self.parent()
            if hasattr(parent_window, 'status_bar'):
                parent_window.status_bar.showMessage(f"Status: Joint J2={self.latest_j2:.4f}, J3={self.latest_j3:.4f} calculation completed.")
        except ValueError:
            QMessageBox.critical(self, "Input Error", "Vector X, Y, Z must be valid numbers!")
        except Exception as e:
            QMessageBox.critical(self, "Calculation Error", f"Inverse kinematics solution failed: {e}")

    def update_movement_status(self, msg):
        self.movement_status_label.setText(f"Movement Status: {msg}")

    def trigger_j2j3_move(self):
        """触发 J2/J3 关节运动。"""
        if not self.ads_client.connected:
            QMessageBox.warning(self, "Warning", "ADS is disconnected. Please connect Beckhoff PLC first.")
            return
        if self.ads_poll_thread and self.ads_poll_thread.moving:
            QMessageBox.warning(self, "Warning", "Movement monitoring task is in progress, please wait.")
            return

        try:
            target_j2 = float(self.result_labels["J2"].text())
            target_j3 = float(self.result_labels["J3"].text())
        except ValueError:
            QMessageBox.critical(self, "Input Error", "Target J2/J3 must be valid numbers (Internal Error).")
            return
            
        try:
            target_time = float(self.motion_time_input.text())
            if target_time <= 0:
                raise ValueError("运动时间必须大于零。")
        except ValueError as e:
            QMessageBox.critical(self, "Input Error", "Target motion time (ms) must be a valid number greater than zero!")
            return
        
        self.ads_command_thread = ADSThread(self.ads_client, target_j2, target_j3, target_time)
        self.ads_command_thread.movement_status_update.connect(self.update_movement_status)
        self.ads_command_thread.finished.connect(lambda: self._start_poll_monitoring(target_j2, target_j3))
        self.ads_command_thread.start()

    def trigger_j2j3_reset(self):
        if not self.ads_client.connected:
            QMessageBox.warning(self, "Warning", "ADS is disconnected. Please connect Beckhoff PLC first.")
            return
        if self.ads_poll_thread and self.ads_poll_thread.moving:
            QMessageBox.warning(self, "Warning", "Movement monitoring task is in progress, please wait.")
            return

        reset_j2 = self.RESET_J2
        reset_j3 = self.RESET_J3
        
        try:
            target_time = float(self.motion_time_input.text())
            if target_time <= 0:
                raise ValueError("运动时间必须大于零。")
        except ValueError:
            QMessageBox.critical(self, "Input Error", "Target motion time (ms) must be a valid number greater than zero!")
            return

        self.result_labels["J2"].setText(f"{reset_j2:.4f}")
        self.result_labels["J3"].setText(f"{reset_j3:.4f}")

        self.ads_command_thread = ADSThread(self.ads_client, reset_j2, reset_j3, target_time)
        self.ads_command_thread.movement_status_update.connect(self.update_movement_status)
        self.ads_command_thread.finished.connect(lambda: self._start_poll_monitoring(reset_j2, reset_j3))
        self.ads_command_thread.start()
        
        parent_window = self.parent()
        if hasattr(parent_window, 'status_bar'):
             parent_window.status_bar.showMessage(f"Status: Reset command sent: J2={reset_j2:.2f}, J3={reset_j3:.2f}.")

    def _start_poll_monitoring(self, target_j2, target_j3):
        if self.ads_poll_thread:
            self.ads_poll_thread.start_monitoring_move(target_j2, target_j3)
            self.update_movement_status("Movement started, starting monitoring completion...")

    def cleanup(self):
        self._stop_poll()
        self.disconnect_ads()
        if self.ads_command_thread and self.ads_command_thread.isRunning():
            self.ads_command_thread.wait()
            
    def apply_joint_increment(self):
        """
        点击“应用该增量”按钮时调用。
        计算新的目标值 (基于 RESET 值 + 增量)，然后【直接触发运动】。
        """
        try:
            inc_j2 = float(self.inc_j2_input.text())
            inc_j3 = float(self.inc_j3_input.text())

            new_target_j2 = self.RESET_J2 + inc_j2
            new_target_j3 = self.RESET_J3 + inc_j3

            self.result_labels["J2"].setText(f"{new_target_j2:.4f}")
            self.result_labels["J3"].setText(f"{new_target_j3:.4f}")
            
            self.movement_status_label.setText("Movement Status: Target calculated. Executing Move...")
            
            # [修改点] 直接调用触发函数，不再需要用户手动点击 Move 按钮
            self.trigger_j2j3_move()
            
        except ValueError:
            QMessageBox.critical(self, "Input Error", "Increment value must be valid numbers!")
        except Exception as e:
            QMessageBox.critical(self, "Calculation Error", f"An error occurred while applying increment: {e}")