import numpy as np
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, 
    QLabel, QLineEdit, QPushButton, QMessageBox, QGroupBox, 
    QFrame, QSizePolicy
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QObject, QTimer
import pyads 
import threading
import time

# ====== ADS Configuration ======
AMS_NET_ID = "192.168.10.100.1.1"
PLC_IP     = "192.168.10.100"
PLC_PORT   = 851

# Target Position Variables (J0, J1, J2, J3)
J0_PV   = "MAIN.MotorSetPos[0]"              
J1_PV   = "MAIN.MotorSetPos[1]"              
J2_PV   = "MAIN.MotorSetPos[2]"              
J3_PV   = "MAIN.MotorSetPos[3]"              

# Motion Enable Variable
START_BOOL = "MAIN.PositionT"    

# Motor Enable Status
ENABLE_STATUS = "MAIN.MotorButtonEnable"

# Motion Time
MOTION_TIME = "MAIN.t1"   

# Shoudong Mode Switch 
SHOUDONG_BOOL = "MAIN.SHOUDONG"

# Position Mode Switch
POSITION_BOOL = "MAIN.PosSetEable"

# Current Position Variables (LREAL)
POS_J0_PV = "MAIN.MotorCurPos[0]" 
POS_J1_PV = "MAIN.MotorCurPos[1]" 
POS_J2_PV = "MAIN.MotorCurPos[2]" 
POS_J3_PV = "MAIN.MotorCurPos[3]" 

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
            self._c.read_state()  # Health check
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
# ADS Poll Thread
# ====================================================================
class ADSPollThread(QThread):
    position_update = pyqtSignal(float, float, float, float)
    movement_status_update = pyqtSignal(str)
    enable_status_update = pyqtSignal(bool)
    
    def __init__(self, ads_client, parent=None):
        super().__init__(parent)
        self.ads = ads_client
        self.is_running = True
        self.moving = False 
        self.target_j0 = 0.0
        self.target_j1 = 0.0
        self.target_j2 = 0.0
        self.target_j3 = 0.0

    def start_monitoring_move(self, t0, t1, t2, t3):
        self.target_j0 = t0
        self.target_j1 = t1
        self.target_j2 = t2
        self.target_j3 = t3
        self.moving = True

    def run(self):
        last_status = ""
        while self.is_running and self.ads.connected:
            try:
                p0 = self.ads.read_lreal(POS_J0_PV)
                p1 = self.ads.read_lreal(POS_J1_PV)
                p2 = self.ads.read_lreal(POS_J2_PV)
                p3 = self.ads.read_lreal(POS_J3_PV)
                self.position_update.emit(p0, p1, p2, p3)

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

    def stop(self):
        self.is_running = False
        self.wait()

# ====================================================================
# ADS Command Thread
# ====================================================================
class ADSThread(QThread):
    movement_status_update = pyqtSignal(str)

    def __init__(self, ads_client, t0, t1, t2, t3, target_time, parent=None):
        super().__init__(parent)
        self.ads = ads_client
        self.target_j0 = t0
        self.target_j1 = t1
        self.target_j2 = t2
        self.target_j3 = t3
        self.motion_time = target_time

    def run(self):
        self.movement_status_update.emit("Sending Target...")
        try:
            self.ads.write_lreal(J0_PV, self.target_j0)
            self.ads.write_lreal(J1_PV, self.target_j1)
            self.ads.write_lreal(J2_PV, self.target_j2)
            self.ads.write_lreal(J3_PV, self.target_j3)
            self.ads.write_int(MOTION_TIME, self.motion_time) 
            self.ads.write_bool(START_BOOL, True)
            self.movement_status_update.emit("Movement command sent, preparing to monitor...")
        except Exception as e:
            self.movement_status_update.emit(f"ADS Write or Start Failed: {e}")

# ====================================================================
# Beckhoff Manager
# ====================================================================
class BeckhoffManager(QObject):
    """
    负责管理 ADS 连接、状态和线程的单一数据源。
    """
    # 信号定义
    connection_state_changed = pyqtSignal(bool, str) # connected, message
    position_update = pyqtSignal(float, float, float, float)
    movement_status_update = pyqtSignal(str)
    enable_status_update = pyqtSignal(bool)
    target_update = pyqtSignal(float, float, float, float, float)
    
    RESET_J0 = -9.891
    RESET_J1 = 293.489
    RESET_J2 = 304.709
    RESET_J3 = 153.415

    def __init__(self, parent=None):
        super().__init__(parent)
        self.ads_client = ADS(AMS_NET_ID, PLC_PORT, PLC_IP)
        self.ads_poll_thread = None
        self.ads_command_thread = None
        self.is_connected = False

    def connect_ads(self):
        ok, msg = self.ads_client.open()
        self.is_connected = ok
        self.connection_state_changed.emit(ok, msg)
        
        if ok:
            self.ads_client.write_bool(SHOUDONG_BOOL, False)
            self.ads_client.write_bool(POSITION_BOOL, True)
            self._start_poll()

    def disconnect_ads(self):
        self._stop_poll()
        self.ads_client.close()
        self.is_connected = False
        self.connection_state_changed.emit(False, "Disconnected")

    def _start_poll(self):
        if self.ads_poll_thread is None or not self.ads_poll_thread.isRunning():
            self.ads_poll_thread = ADSPollThread(self.ads_client)
            self.ads_poll_thread.position_update.connect(self.position_update.emit)
            self.ads_poll_thread.movement_status_update.connect(self.movement_status_update.emit)
            self.ads_poll_thread.enable_status_update.connect(self.enable_status_update.emit)
            self.ads_poll_thread.start()

    def _stop_poll(self):
        if self.ads_poll_thread and self.ads_poll_thread.isRunning():
            self.ads_poll_thread.stop()
            self.ads_poll_thread = None

    def toggle_motor_enable(self):
        if not self.is_connected:
            return
        try:
            current_state = self.ads_client.read_bool(ENABLE_STATUS)
            new_state = not current_state
            self.ads_client.write_bool(ENABLE_STATUS, new_state)
            action = "Disable" if not new_state else "Enable"
            self.movement_status_update.emit(f"Sent {action} command ({new_state})")
        except Exception as e:
            self.movement_status_update.emit(f"Failed to toggle enable: {e}")

    def move_robot(self, t0, t1, t2, t3, time_ms):
        if not self.is_connected:
            return
        if self.ads_poll_thread and self.ads_poll_thread.moving:
            self.movement_status_update.emit("Warning: Movement in progress.")
            return
        
        self.target_update.emit(t0, t1, t2, t3, time_ms)

        self.ads_command_thread = ADSThread(self.ads_client, t0, t1, t2, t3, time_ms)
        self.ads_command_thread.movement_status_update.connect(self.movement_status_update.emit)
        self.ads_command_thread.finished.connect(lambda: self._start_poll_monitoring(t0, t1, t2, t3))
        self.ads_command_thread.start()

    def _start_poll_monitoring(self, t0, t1, t2, t3):
        if self.ads_poll_thread:
            self.ads_poll_thread.start_monitoring_move(t0, t1, t2, t3)
            self.movement_status_update.emit("Movement started...")

    def cleanup(self):
        self.disconnect_ads()
        if self.ads_command_thread and self.ads_command_thread.isRunning():
            self.ads_command_thread.wait()

# ====================================================================
# BeckhoffTab Class (Standard / Original)
# ====================================================================
class BeckhoffTab(QWidget):
    beckhoff_position_update = pyqtSignal(float, float, float, float)
    D4_TROCAR = 23.2
    NEEDLE_TIP_OFFSET = 16.0
    D4_RESET_OFFSET = 2

    def __init__(self, manager: BeckhoffManager, robot_kinematics, parent=None):
        super().__init__(parent)
        self.main_window = parent
        self.manager = manager
        self.robot = robot_kinematics
        
        # [NEW] Steering Logic Variables
        self.current_yaw = 0.0
        self.current_pitch = 0.0
        
        self.vector_inputs = [None] * 3  
        self.result_labels = {}
        self.ads_status_label = QLabel("ADS: Disconnected") 
        self.pos_labels = {}
        self.motion_time_input = None
        
        self.inc_j0_input = None
        self.inc_j1_input = None
        self.inc_j2_input = None
        self.inc_j3_input = None
        
        # Flowchart Controls
        self.flow_trocar_in_p1_btn = None 
        self.flow_trocar_in_p2_btn = None
        self.flow_calc_btn = None
        self.flow_needle_in_btn = None
        self.flow_needle_out_btn = None
        self.flow_trocar_out_btn = None
        
        self.trocar_phase_1_state = 0
        self.phase_1_done = False 
        
        self.init_ui()
        self.setup_connections()
        self.on_connection_changed(self.manager.is_connected, "Synced")

    # --- Helper Functions for Arrows ---
    def _create_h_arrow_widget(self):
        w = QWidget()
        w.setFixedWidth(50) 
        layout = QHBoxLayout(w)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet("background-color: #444; min-height: 2px; max-height: 2px; border: none;")
        line.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        head = QLabel("►")
        head.setStyleSheet("color: #444; font-size: 14px; font-weight: bold; border: none; margin-left: -2px;")
        head.setAlignment(Qt.AlignCenter)
        head.setFixedWidth(15)
        layout.addWidget(line)
        layout.addWidget(head)
        return w

    def _create_v_arrow_widget(self):
        w = QWidget()
        layout = QVBoxLayout(w)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.setAlignment(Qt.AlignCenter)
        line = QFrame()
        line.setFrameShape(QFrame.VLine)
        line.setStyleSheet("background-color: #444; min-width: 2px; max-width: 2px; border: none;")
        line.setFixedWidth(2)
        line.setFixedHeight(12)
        head = QLabel("▼")
        head.setStyleSheet("color: #444; font-size: 12px; font-weight: bold; border: none; margin-top: -3px;")
        head.setAlignment(Qt.AlignCenter)
        head.setFixedHeight(12)
        head.setFixedWidth(20)
        layout.addWidget(line, 0, Qt.AlignHCenter)
        layout.addWidget(head, 0, Qt.AlignHCenter)
        return w

    def _create_loop_back_line(self):
        w = QWidget()
        layout = QHBoxLayout(w)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        head = QLabel("◄")
        head.setStyleSheet("color: #444; font-size: 14px; font-weight: bold; border: none; margin-right: -2px;")
        head.setAlignment(Qt.AlignCenter)
        head.setFixedWidth(15)
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet("background-color: #444; min-height: 2px; max-height: 2px; border: none;")
        line.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout.addWidget(head)
        layout.addWidget(line)
        return w
        
    def _create_up_arrow_widget(self):
        w = QWidget()
        layout = QVBoxLayout(w)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.setAlignment(Qt.AlignCenter)
        head = QLabel("▲")
        head.setStyleSheet("color: #444; font-size: 12px; font-weight: bold; border: none; margin-bottom: -3px;")
        head.setAlignment(Qt.AlignCenter)
        head.setFixedHeight(12)
        head.setFixedWidth(20)
        line = QFrame()
        line.setFrameShape(QFrame.VLine)
        line.setStyleSheet("background-color: #444; min-width: 2px; max-width: 2px; border: none;")
        line.setFixedWidth(2)
        line.setFixedHeight(12)
        layout.addWidget(head, 0, Qt.AlignHCenter)
        layout.addWidget(line, 0, Qt.AlignHCenter)
        return w

    def init_ui(self):
        main_layout = QVBoxLayout(self)

        # Vector Inputs
        for i in range(3):
            self.vector_inputs[i] = QLineEdit("0.00")

        # 1. Position Control (Integrated steering controls)
        result_group = QGroupBox("Position Control (J0 - J3)")
        result_content_layout = QHBoxLayout()
        result_layout = QGridLayout()
        
        self.result_labels["J0"] = QLineEdit("0.00")
        self.result_labels["J1"] = QLineEdit("0.00")
        self.result_labels["J2"] = QLineEdit("0.00")
        self.result_labels["J3"] = QLineEdit("0.00")
        self.motion_time_input = QLineEdit("5000") 

        for idx, axis in enumerate(["J0", "J1", "J2", "J3"]):
            if idx == 0: unit = "(mm)"
            elif idx == 1: unit = "(Deg)"
            elif idx == 2: unit = "(Deg)"
            elif idx == 3: unit = "(mm)"
            inc_label = QLabel(f"Δ {axis} {unit}:")
            inc_label.setAlignment(Qt.AlignVCenter)
            inc_input = QLineEdit("0.00")
            inc_input.setFixedWidth(100)
            cur_label = QLabel(f"Current {axis} {unit}:")
            cur_label.setAlignment(Qt.AlignVCenter)
            pos_output = QLineEdit("--")
            pos_output.setReadOnly(True)
            pos_output.setFixedWidth(100)
            pos_output.setStyleSheet("background-color: #D4EDF7; border: 1px inset grey;")
            
            self.pos_labels[f"Cur{axis}"] = pos_output
            if axis == "J0": self.inc_j0_input = inc_input
            elif axis == "J1": self.inc_j1_input = inc_input
            elif axis == "J2": self.inc_j2_input = inc_input
            elif axis == "J3": self.inc_j3_input = inc_input
            
            result_layout.addWidget(inc_label, idx, 0)
            result_layout.addWidget(inc_input, idx, 1)
            result_layout.addWidget(QLabel(" "), idx, 2)
            result_layout.addWidget(cur_label, idx, 3)
            result_layout.addWidget(pos_output, idx, 4)

        # Gap and steering controls
        result_layout.setColumnMinimumWidth(5, 40)
        
        # Descent Dist
        row0_container = QWidget()
        row0_layout = QHBoxLayout(row0_container); row0_layout.setContentsMargins(0,0,0,0)
        row0_layout.addWidget(QLabel("Descent Distance (mm)"))
        self.descent_dist_input = QLineEdit("10"); self.descent_dist_input.setFixedWidth(50)
        row0_layout.addWidget(self.descent_dist_input); row0_layout.addStretch()
        result_layout.addWidget(row0_container, 0, 6)
        
        # Rotate Deg
        row1_container = QWidget()
        row1_layout = QHBoxLayout(row1_container); row1_layout.setContentsMargins(0,0,0,0)
        row1_layout.addWidget(QLabel("Rotate x Deg"))
        self.rot_range_input = QLineEdit("10"); self.rot_range_input.setFixedWidth(50)
        row1_layout.addWidget(self.rot_range_input); row1_layout.addStretch()
        result_layout.addWidget(row1_container, 1, 6)
        
        self.btn_j0_decrease = QPushButton("Δ J0 decreases"); result_layout.addWidget(self.btn_j0_decrease, 2, 6)
        self.btn_rotate_reset = QPushButton("Rotate Probe and Reset"); result_layout.addWidget(self.btn_rotate_reset, 3, 6)

        # Yaw/Pitch Target & Current
        result_layout.addWidget(QLabel("Needle Yaw (°):"), 4, 0)
        yaw_l = QHBoxLayout()
        self.yaw_minus_btn = QPushButton("-"); self.yaw_plus_btn = QPushButton("+"); self.yaw_display = QLineEdit("0.0")
        self.yaw_minus_btn.setFixedWidth(30)
        self.yaw_plus_btn.setFixedWidth(30)
        self.yaw_display.setReadOnly(False); self.yaw_display.setFixedWidth(60)
        self.yaw_display.setStyleSheet("background-color: #ffffff; border: 1px solid #ced4da;") # [修改] 白色背景
        self.yaw_display.editingFinished.connect(self.update_inputs_from_yaw_pitch) # [新增] 输入完成触发计算
        yaw_l.addWidget(self.yaw_minus_btn); yaw_l.addWidget(self.yaw_display); yaw_l.addWidget(self.yaw_plus_btn); yaw_l.addStretch()
        result_layout.addLayout(yaw_l, 4, 1, 1, 2)
        
        result_layout.addWidget(QLabel("Current Yaw (°):"), 4, 3)
        self.cur_yaw_display = QLineEdit("--"); self.cur_yaw_display.setReadOnly(True); self.cur_yaw_display.setFixedWidth(100); self.cur_yaw_display.setStyleSheet("background-color: #E8F5E9;")
        result_layout.addWidget(self.cur_yaw_display, 4, 4)

        result_layout.addWidget(QLabel("Needle Pitch (°):"), 5, 0)
        pitch_l = QHBoxLayout()
        self.pitch_minus_btn = QPushButton("-"); self.pitch_plus_btn = QPushButton("+"); self.pitch_display = QLineEdit("0.0")
        self.pitch_minus_btn.setFixedWidth(30)
        self.pitch_plus_btn.setFixedWidth(30)
        self.pitch_display.setReadOnly(False); self.pitch_display.setFixedWidth(60)
        self.pitch_display.setStyleSheet("background-color: #ffffff; border: 1px solid #ced4da;") # [修改] 白色背景
        self.pitch_display.editingFinished.connect(self.update_inputs_from_yaw_pitch) # [新增] 输入完成触发计算
        pitch_l.addWidget(self.pitch_minus_btn); pitch_l.addWidget(self.pitch_display); pitch_l.addWidget(self.pitch_plus_btn); pitch_l.addStretch()
        result_layout.addLayout(pitch_l, 5, 1, 1, 2)
        
        result_layout.addWidget(QLabel("Current Pitch (°):"), 5, 3)
        self.cur_pitch_display = QLineEdit("--"); self.cur_pitch_display.setReadOnly(True); self.cur_pitch_display.setFixedWidth(100); self.cur_pitch_display.setStyleSheet("background-color: #E8F5E9;")
        result_layout.addWidget(self.cur_pitch_display, 5, 4)

        self.apply_inc_btn = QPushButton("Apply Increment (All)")
        result_layout.addWidget(self.apply_inc_btn, 6, 0, 1, 2) 

        self.reset_all_btn = QPushButton("Reset All (J0-J3)")
        self.reset_all_btn.setEnabled(False) 
        result_layout.addWidget(self.reset_all_btn, 6, 3, 1, 2)
        
        result_content_layout.addLayout(result_layout)
        result_content_layout.addStretch(1) 
        result_group.setLayout(result_content_layout)
        main_layout.addWidget(result_group)
        
        # 2. Biopsy Interface (Standard Flow - RETAINED ORIGINAL COMPACT LAYOUT)
        biopsy_group = QGroupBox("Biopsy Interface (Standard)")
        biopsy_layout = QGridLayout(biopsy_group)
        biopsy_layout.setSpacing(5) 
        biopsy_layout.setContentsMargins(10, 10, 10, 10)

        self.flow_trocar_in_p1_btn = QPushButton("Trocar Insertion Phase 1")
        self.flow_trocar_in_p2_btn = QPushButton("Trocar Insertion Phase 2")
        self.flow_calc_btn = QPushButton("Adjust Needle Dir") 
        self.flow_needle_in_btn = QPushButton("Needle Insertion")
        self.flow_needle_out_btn = QPushButton("Needle Retraction")
        self.flow_trocar_out_btn = QPushButton("Trocar Retraction")

        BTN_WIDTH = 160 
        for btn in [self.flow_trocar_in_p1_btn, self.flow_trocar_in_p2_btn, self.flow_calc_btn, 
                    self.flow_needle_in_btn, self.flow_needle_out_btn, 
                    self.flow_trocar_out_btn]:
            btn.setFixedWidth(BTN_WIDTH)

        def make_node_label(text):
            l = QLabel(text)
            l.setAlignment(Qt.AlignCenter)
            l.setStyleSheet("border: 2px solid #555; border-radius: 15px; padding: 5px; font-weight: bold; min-width: 80px; background-color: #f0f0f0;")
            l.setFixedSize(100, 40)
            return l

        self.flow_start_lbl = make_node_label("Start")
        self.flow_end_lbl = make_node_label("End")

        biopsy_layout.addWidget(self.flow_start_lbl, 0, 0, alignment=Qt.AlignCenter)
        biopsy_layout.addWidget(self._create_v_arrow_widget(), 1, 0, alignment=Qt.AlignCenter) 
        
        trocar_container = QWidget()
        trocar_layout = QVBoxLayout(trocar_container)
        trocar_layout.setContentsMargins(0, 0, 0, 0)
        trocar_layout.setSpacing(0)
        trocar_layout.addWidget(self.flow_trocar_in_p1_btn, alignment=Qt.AlignCenter)
        trocar_layout.addWidget(self._create_v_arrow_widget(), alignment=Qt.AlignCenter)
        trocar_layout.addWidget(self.flow_trocar_in_p2_btn, alignment=Qt.AlignCenter)
        biopsy_layout.addWidget(trocar_container, 2, 0, alignment=Qt.AlignCenter)
        biopsy_layout.addWidget(self._create_v_arrow_widget(), 3, 0, alignment=Qt.AlignCenter) 
        
        biopsy_layout.addWidget(self.flow_calc_btn, 4, 0, alignment=Qt.AlignCenter)
        biopsy_layout.addWidget(self._create_h_arrow_widget(), 4, 1) 
        biopsy_layout.addWidget(self.flow_needle_in_btn, 4, 2, alignment=Qt.AlignCenter) 
        biopsy_layout.addWidget(self._create_h_arrow_widget(), 4, 3) 
        biopsy_layout.addWidget(self.flow_needle_out_btn, 4, 4, alignment=Qt.AlignCenter) 

        biopsy_layout.addWidget(self._create_up_arrow_widget(), 5, 0, alignment=Qt.AlignCenter)
        biopsy_layout.addWidget(self._create_loop_back_line(), 5, 1, 1, 3) 
        biopsy_layout.addWidget(self._create_v_arrow_widget(), 5, 4, alignment=Qt.AlignCenter) 

        biopsy_layout.addWidget(self.flow_trocar_out_btn, 6, 4, alignment=Qt.AlignCenter) 
        biopsy_layout.addWidget(self._create_v_arrow_widget(), 7, 4, alignment=Qt.AlignCenter)
        biopsy_layout.addWidget(self.flow_end_lbl, 8, 4, alignment=Qt.AlignCenter)

        biopsy_layout.setColumnStretch(5, 1)
        main_layout.addWidget(biopsy_group)

        # 3. Beckhoff PLC Communication
        beckhoff_comm_group = QGroupBox("Beckhoff PLC Communication")
        beckhoff_comm_layout = QVBoxLayout(beckhoff_comm_group)

        conn_layout = QHBoxLayout()
        self.connect_ads_btn = QPushButton("Connect ADS")
        self.disconnect_ads_btn = QPushButton("Disconnect ADS")
        self.disconnect_ads_btn.setEnabled(False)
        self.enable_motor_btn = QPushButton("Enable")
        self.enable_motor_btn.setEnabled(False)
        self.enable_motor_btn.setStyleSheet("background-color: lightgray;")
        
        conn_layout.addWidget(self.connect_ads_btn)
        conn_layout.addWidget(self.disconnect_ads_btn)
        conn_layout.addWidget(self.enable_motor_btn) 
        conn_layout.addWidget(self.ads_status_label)
        beckhoff_comm_layout.addLayout(conn_layout)

        self.movement_status_label = QLabel("Movement Status: Standby")
        beckhoff_comm_layout.addWidget(self.movement_status_label)

        main_layout.addWidget(beckhoff_comm_group)
        main_layout.addStretch()

    def setup_connections(self):
        self.connect_ads_btn.clicked.connect(self.manager.connect_ads)
        self.disconnect_ads_btn.clicked.connect(self.manager.disconnect_ads)
        self.enable_motor_btn.clicked.connect(self.manager.toggle_motor_enable)
        self.reset_all_btn.clicked.connect(self.trigger_reset)
        
        self.apply_inc_btn.clicked.connect(self.apply_joint_increment)
        self.flow_calc_btn.clicked.connect(self.adjust_needle_direction)
        self.flow_trocar_in_p1_btn.clicked.connect(self.run_trocar_insertion_phase_1)
        self.flow_trocar_in_p2_btn.clicked.connect(self.run_trocar_insertion_phase_2)
        self.flow_needle_in_btn.clicked.connect(self.run_needle_insertion)
        self.flow_needle_out_btn.clicked.connect(self.run_needle_retraction)
        self.flow_trocar_out_btn.clicked.connect(self.run_trocar_retraction)

        self.manager.connection_state_changed.connect(self.on_connection_changed)
        self.manager.position_update.connect(self.on_position_update)
        self.manager.enable_status_update.connect(self.on_enable_status_update)
        self.manager.movement_status_update.connect(self.on_movement_status_update)
        self.manager.target_update.connect(self.on_target_update)
        self.manager.position_update.connect(self.beckhoff_position_update.emit)
        
        # [NEW] Additional Steering/Automation Connections
        self.yaw_minus_btn.clicked.connect(lambda: self.adjust_yaw_pitch_step("yaw", -1.0))
        self.yaw_plus_btn.clicked.connect(lambda: self.adjust_yaw_pitch_step("yaw", 1.0))
        self.pitch_minus_btn.clicked.connect(lambda: self.adjust_yaw_pitch_step("pitch", -1.0))
        self.pitch_plus_btn.clicked.connect(lambda: self.adjust_yaw_pitch_step("pitch", 1.0))
        self.btn_j0_decrease.clicked.connect(self.on_j0_decrease_clicked)
        self.btn_rotate_reset.clicked.connect(self.on_rotate_probe_reset_clicked)
        self.manager.position_update.connect(self.update_current_yaw_pitch)
        if hasattr(self.main_window, 'ultrasound_tab'):
            # [修改] 连接到带参数的槽函数
            self.main_window.ultrasound_tab.scan_finished.connect(self._on_scan_completed_logic)

    # =========================================================================
    # Steering & Angle Logic
    # =========================================================================
    def calculate_angles_from_vector(self, vector):
        """
        Calculate Yaw and Pitch relative to reference vector (0, 1, 0).
        Ref: 
          vx = -sin(y)*cos(p)
          vy = cos(y)*cos(p)
          vz = sin(p)
        Returns: (Yaw_deg, Pitch_deg, Roll_deg)
        """
        norm = np.linalg.norm(vector)
        if norm < 1e-6:
            return 0.0, 0.0, 0.0
        
        v = vector / norm
        
        # Pitch = arcsin(vz)
        pitch_rad = np.arcsin(v[2])
        
        # Yaw = arctan2(-vx, vy)
        yaw_rad = np.arctan2(-v[0], v[1])
        
        # Roll (placeholder, typically 0 for needle vector unless we have full orientation matrix)
        roll_rad = 0.0
        
        return np.rad2deg(yaw_rad), np.rad2deg(pitch_rad), np.rad2deg(roll_rad)

    def update_inputs_from_yaw_pitch(self):
        """
        [核心逻辑] 
        1. 根据 Yaw/Pitch 填写框计算目标方向向量。
        2. 更新 UI 上的向量显示框。
        3. 计算并应用电机增量。
        """
        try:
            # 获取角度并转换为弧度
            yaw_rad = np.radians(float(self.yaw_display.text()))
            pitch_rad = np.radians(float(self.pitch_display.text()))
            
            # 1. 根据参考逻辑构造目标方向向量
            vx = -np.sin(yaw_rad) * np.cos(pitch_rad)
            vy =  np.cos(yaw_rad) * np.cos(pitch_rad)
            vz =  np.sin(pitch_rad)
            
            # 2. 更新 vector_inputs 显示框 (vx, vy, vz)
            # 假设 self.vector_inputs 对应界面上的方向向量输入框
            self.vector_inputs[0].setText(f"{vx:.4f}")
            self.vector_inputs[1].setText(f"{vy:.4f}")
            self.vector_inputs[2].setText(f"{vz:.4f}")
            
            # 3. 计算电机增量 (j1, j2)
            # 内部调用 calculate_joint_values，它会读取 vector_inputs 并计算结果填入 inc_j1_input 等
            self.calculate_joint_values()
                
        except Exception as e:
            print(f"update_inputs_from_yaw_pitch error: {e}")

    def update_current_yaw_pitch(self, j0, j1, j2, j3):
        try:
            dj1 = j1 - self.manager.RESET_J1
            dj2 = (j2 - self.manager.RESET_J2) - (j1 - self.manager.RESET_J1)
            vector = self.robot.get_needle_vector([0, dj1, dj2, 0])
            yaw, pitch, _ = self.calculate_angles_from_vector(vector)
            self.cur_yaw_display.setText(f"{yaw:.1f}"); self.cur_pitch_display.setText(f"{pitch:.1f}")
        except Exception as e:
            print(f"Error updating current yaw/pitch: {e}")
            
    def adjust_yaw_pitch_step(self, axis, delta):
        """点击按钮增减填写框中的数值"""
        try:
            display = self.yaw_display if axis == "yaw" else self.pitch_display
            current_val = float(display.text())
            display.setText(f"{current_val + delta:.1f}")
            # 修改数值后，立即更新 Joint 电机输入框
            self.update_inputs_from_yaw_pitch()
            self.apply_joint_increment()
        except ValueError:
            pass

    # =========================================================================
    # Automation / Manual Controls
    # =========================================================================
    def on_j0_decrease_clicked(self):
        try:
            val = float(self.inc_j0_input.text()) - float(self.descent_dist_input.text())
            self.inc_j0_input.setText(f"{val:.4f}"); self.apply_joint_increment()
        except Exception as e:
            print(f"Error on_j0_decrease_clicked: {e}")
            
    def on_rotate_probe_reset_clicked(self):
        """局部图像重建入口"""
        folder_name = self.generate_local_us_folder_name()
        
        """
        自动化序列：
        1. Probe Left x
        2. (1s后) Probe Right 2X (Scan)
        3. (等待扫描完成)
        4. (1s后) Probe Left x (Reset)
        5. (1s后) J0 Increase (Descent Dist) & Apply
        """
        
        # 1. 探头左转
        self.rotate_probe(0, 1) 
        
        # 2. 1秒后触发带自定义名称的扫描
        QTimer.singleShot(4000, lambda: self._step2_start_scan_local(folder_name))
        
    def generate_local_us_folder_name(self):
        """构建复杂的局部重建文件夹名"""
        lp = self.main_window.left_panel
        
        # 1.1 转换 A、B 点到 Volume 坐标系
        raw_a_text = lp.a_point_dropdown.currentText()  # 例如 "A1: (-254.91, ...)"
        raw_b_text = lp.b_point_dropdown.currentText()  # 例如 "B1, 3.95deg"

        # 1.2 精确提取 ID
        # A 通过冒号分割获取 A1
        a_id = raw_a_text.split(':')[0].strip() if ':' in raw_a_text else "A"
        # B 通过逗号分割获取 B1
        b_id = raw_b_text.split(',')[0].strip() if ',' in raw_b_text else "B"
        
        a_vol = lp.get_current_a_in_volume() # 需在 left_panel 实现此转换
        b_vol = lp.get_current_b_in_volume() 
        
        a_str = f"{a_id}({a_vol[0]:.1f},{a_vol[1]:.1f},{a_vol[2]:.1f})"
        b_str = f"{b_id}({b_vol[0]:.1f},{b_vol[1]:.1f},{b_vol[2]:.1f})"

        # 2. 计算 RCM in Volume
        delta_j0 = float(self.inc_j0_input.text())
        rcm_in_p = self.robot.get_rcm_point([delta_j0, 0, 0, 0])
        rcm_vol = lp.transform_point_p_to_volume(rcm_in_p) # 需在 left_panel 实现
        rcm_str = f"RCM({rcm_vol[0]:.1f},{rcm_vol[1]:.1f},{rcm_vol[2]:.1f})"

        # 3. 获取 deg(x) 和 U 姿态
        # deg_x 来自点击过旋转对准后的内部变量
        deg_x = getattr(lp, 'last_calculated_rotation_to_b', 0) 
        
        # 计算 TCP_U 在 Volume 系下的姿态
        # T_Vol_U = T_Vol_Base * T_Base_E * T_E_U
        u_pose_vol = lp.get_current_tcp_u_in_volume() # 需在 left_panel 实现
        u_str = f"U({','.join([f'{p:.1f}' for p in u_pose_vol])})"

        return f"{a_str}{b_str}{rcm_str}deg({deg_x}){u_str}LocalUS"

    def _step2_start_scan_local(self, folder_name):
        if self.main_window and hasattr(self.main_window, 'ultrasound_tab'):
            us_tab = self.main_window.ultrasound_tab
            us_tab.rotation_range_input.setText(self.rot_range_input.text())
            # 传入自定义文件名
            us_tab.rotate_and_capture_2x(custom_folder_name=folder_name)

    def _on_scan_completed_logic(self, is_local_task):
        if is_local_task:
            # 只有局部重建才执行重置探头和增加 J0 的逻辑
            print("System: Local scan finished. Proceeding to reset sequence.")
            QTimer.singleShot(1000, self._step4_reset_probe)
        else:
            # 如果是全局扫描，什么都不做，或者只打个日志
            print("System: Global scan finished. No reset required.")

    def _step4_reset_probe(self):
        self.rotate_probe(0, 1) # Reset back
        QTimer.singleShot(1000, self._step5_increase_j0)

    def _step5_increase_j0(self):
        try:
            val = float(self.inc_j0_input.text()) + float(self.descent_dist_input.text())
            self.inc_j0_input.setText(f"{val:.4f}"); self.apply_joint_increment()
        except Exception as e:
            print(f"Error _step5_increase_j0: {e}")
            
    def rotate_probe(self, d, m):
        if self.main_window and hasattr(self.main_window, 'tcp_manager'):
            try:
                angle = float(self.rot_range_input.text()) * m
                self.main_window.tcp_manager.send_command(f"MoveRelJ,0,5,{d},{angle};")
            except Exception as e:
                print(f"Error rotate_probe: {e}")

    # =========================================================================
    # Original Shared UI Logic
    # =========================================================================
    def on_target_update(self, t0, t1, t2, t3, time_ms):
        self.result_labels["J0"].setText(f"{t0:.4f}")
        self.result_labels["J1"].setText(f"{t1:.4f}")
        self.result_labels["J2"].setText(f"{t2:.4f}")
        self.result_labels["J3"].setText(f"{t3:.4f}")
        if self.motion_time_input:
            self.motion_time_input.setText(str(int(time_ms)))

    def on_connection_changed(self, connected, msg):
        self.ads_status_label.setText(f"ADS: {msg}")
        self.connect_ads_btn.setEnabled(not connected)
        self.disconnect_ads_btn.setEnabled(connected)
        self.reset_all_btn.setEnabled(connected)
        self.enable_motor_btn.setEnabled(connected)
        if not connected:
             for key in ["CurJ0", "CurJ1", "CurJ2", "CurJ3"]:
                self.pos_labels[key].setText("--")

    def on_position_update(self, j0, j1, j2, j3):
        self.pos_labels["CurJ0"].setText(f"{j0:.3f}")
        self.pos_labels["CurJ1"].setText(f"{j1:.3f}")
        self.pos_labels["CurJ2"].setText(f"{j2:.3f}")
        self.pos_labels["CurJ3"].setText(f"{j3:.3f}")

    def on_enable_status_update(self, is_enabled):
        if is_enabled:
            self.enable_motor_btn.setText("Enabled")
            self.enable_motor_btn.setStyleSheet("background-color: lightgreen;")
        else:
            self.enable_motor_btn.setText("Enable")
            self.enable_motor_btn.setStyleSheet("background-color: lightgray;")

    def on_movement_status_update(self, msg):
        self.movement_status_label.setText(f"Movement Status: {msg}")
        if self.trocar_phase_1_state == 1 and "Movement Completed" in msg:
            self.trocar_phase_1_state = 2 
            vector = [0, 1, 0]
            target_yaw, target_pitch, _ = self.calculate_angles_from_vector(vector)
            self.yaw_display.setText(f"{target_yaw:.1f}")
            self.pitch_display.setText(f"{target_pitch:.1f}")
            
            # 执行统一的更新逻辑
            self.update_inputs_from_yaw_pitch()
            # 应用计算出的增量到电机目标位置
            self.apply_joint_increment()
            self.trocar_phase_1_state = 3 
        elif self.trocar_phase_1_state == 3 and "Movement Completed" in msg:
            self.trocar_phase_1_state = 0
            self.phase_1_done = True
            
    def calculate_joint_values(self):
        try:
            x = float(self.vector_inputs[0].text())
            y = float(self.vector_inputs[1].text())
            z = float(self.vector_inputs[2].text())
            needle_vector = np.array([x, y, z])
            norm = np.linalg.norm(needle_vector)
            if not np.isclose(norm, 1.0) and norm > 1e-6:
                 needle_vector = needle_vector / norm
            elif norm < 1e-6:
                 QMessageBox.critical(self, "Input Error", "Vector magnitude is close to zero.")
                 return
            joint_values = self.robot.get_joint23_value(needle_vector)
            actual_j1 = joint_values[0]
            actual_j2 = joint_values[1] + joint_values[0]
            self.inc_j1_input.setText(f"{actual_j1:.4f}")
            self.inc_j2_input.setText(f"{actual_j2:.4f}")
        except ValueError:
            QMessageBox.critical(self, "Input Error", "Vector X, Y, Z must be valid numbers!")
        except Exception as e:
            QMessageBox.critical(self, "Calculation Error", f"Inverse kinematics solution failed: {e}")

    def trigger_move(self):
        try:
            t0 = float(self.result_labels["J0"].text())
            t1 = float(self.result_labels["J1"].text())
            t2 = float(self.result_labels["J2"].text())
            t3 = float(self.result_labels["J3"].text())
            target_time = float(self.motion_time_input.text())
            if target_time <= 0: raise ValueError
        except ValueError:
            QMessageBox.critical(self, "Input Error", "Target values and time must be valid.")
            return
        
        self.manager.move_robot(t0, t1, t2, t3, target_time)

    def trigger_reset(self):
        try:
            target_time = float(self.motion_time_input.text())
            if target_time <= 0: raise ValueError
        except:
            QMessageBox.critical(self, "Input Error", "Time must be > 0")
            return

        r0 = self.manager.RESET_J0
        r1 = self.manager.RESET_J1
        r2 = self.manager.RESET_J2
        r3 = self.manager.RESET_J3 + self.D4_RESET_OFFSET

        self.result_labels["J0"].setText(f"{r0:.4f}")
        self.result_labels["J1"].setText(f"{r1:.4f}")
        self.result_labels["J2"].setText(f"{r2:.4f}")
        self.result_labels["J3"].setText(f"{r3:.4f}")

        self.manager.move_robot(r0, r1, r2, r3, target_time)

    def apply_joint_increment(self):
        try:
            inc_j0 = float(self.inc_j0_input.text())
            inc_j1 = float(self.inc_j1_input.text())
            inc_j2 = float(self.inc_j2_input.text())
            inc_j3 = float(self.inc_j3_input.text())

            new_target_j0 = self.manager.RESET_J0 + inc_j0
            new_target_j1 = self.manager.RESET_J1 + inc_j1
            new_target_j2 = self.manager.RESET_J2 + inc_j2
            new_target_j3 = self.manager.RESET_J3 + inc_j3

            self.result_labels["J0"].setText(f"{new_target_j0:.4f}")
            self.result_labels["J1"].setText(f"{new_target_j1:.4f}")
            self.result_labels["J2"].setText(f"{new_target_j2:.4f}")
            self.result_labels["J3"].setText(f"{new_target_j3:.4f}")
            
            self.trigger_move()
        except ValueError:
            QMessageBox.critical(self, "Input Error", "Increment values must be valid numbers!")

    def run_trocar_insertion_phase_1(self):
        self.phase_1_done = False
        parent = self.main_window 
        if not parent or not hasattr(parent, 'left_panel'):
            QMessageBox.warning(self, "Error", "Cannot access Left Panel data.")
            return
        if not parent.left_panel.a_point_in_tcp_p:
             QMessageBox.warning(self, "Data Missing", "A point in TCP_P is not defined.")
             return
        try:
            a_z = parent.left_panel.a_point_in_tcp_p[2]
            rcm_z = self.robot.get_rcm_point([0,0,0,0])[2]
            delta_j0 = a_z - rcm_z
        except Exception as e:
            QMessageBox.critical(self, "Calculation Error", f"Failed to calculate J1 delta: {e}")
            return
        self.inc_j0_input.setText(f"{delta_j0:.4f}")
        self.trocar_phase_1_state = 1
        self.apply_joint_increment()

    def run_trocar_insertion_phase_2(self):
        if not self.phase_1_done:
            QMessageBox.warning(self, "Sequence Error", "Please complete 'Trocar Insertion Phase 1' first.")
            return
        self.inc_j3_input.setText(f"{self.D4_TROCAR}")
        self.apply_joint_increment()
    
    def adjust_needle_direction(self):
        parent = self.main_window
        if not parent or not hasattr(parent, 'left_panel'): return
        b_point_p = parent.left_panel.b_point_in_tcp_p
        if b_point_p is None:
            QMessageBox.warning(self, "Data Missing", "Biopsy Point B in TCP_P is not defined.")
            return
        try:
            delta_j0 = float(self.inc_j0_input.text())
            rcm_point = self.robot.get_rcm_point([delta_j0, 0, 0, 0])
            vector = np.array(b_point_p) - rcm_point
            
            target_yaw, target_pitch, _ = self.calculate_angles_from_vector(vector)
            self.yaw_display.setText(f"{target_yaw:.1f}")
            self.pitch_display.setText(f"{target_pitch:.1f}")
            
            # 执行统一的更新逻辑
            self.update_inputs_from_yaw_pitch()
            # 应用计算出的增量到电机目标位置
            self.apply_joint_increment()
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed: {e}")

    def run_needle_insertion(self):
        try:
            delta_j0 = float(self.inc_j0_input.text())
            delta_j1 = float(self.inc_j1_input.text())
            delta_j2 = float(self.inc_j2_input.text())
            theoretical_j1 = delta_j1
            theoretical_j2 = delta_j2 - delta_j1
            
            parent = self.main_window
            b_point_p = parent.left_panel.b_point_in_tcp_p
            if b_point_p is None: return

            tip_pos = self.robot.get_tip_of_needle([delta_j0, theoretical_j1, theoretical_j2, 0])
            b_point_vec = np.array(b_point_p)
            d4 = np.linalg.norm(b_point_vec - tip_pos)
            d4 = d4 + self.NEEDLE_TIP_OFFSET
            self.inc_j3_input.setText(f"{d4:.4f}")
            self.apply_joint_increment()
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed: {e}")

    def run_needle_retraction(self):
        self.inc_j3_input.setText(f"{self.D4_TROCAR}")
        self.apply_joint_increment()

    def run_trocar_retraction(self):
        self.inc_j3_input.setText(f"{self.D4_RESET_OFFSET:.4f}")
        self.apply_joint_increment()

    def cleanup(self):
        pass