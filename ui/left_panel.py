import json
import os
import numpy as np
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel, QLineEdit, 
    QPushButton, QGridLayout, QMessageBox, QComboBox, QSlider, QDialog,
    QTableWidget, QTableWidgetItem, QHeaderView, QAbstractItemView, QCheckBox
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from core.ultrasound_plane import calculate_rotation_for_plane_alignment, calculate_new_rpy_for_b_point, get_final_tcp_e_position_after_delta_rotation
import pytransform3d.rotations as pyrot

# 常量
FORWARD = 1
BACKWARD = 0
DATA_FILE_NAME = "saved_robot_data.json"

class BPointSelectionDialog(QDialog):
    """
    B点选择对话框：用于显示从TXT文件读取的B点列表，并允许用户多选。
    """
    b_points_selected = pyqtSignal(list) 

    def __init__(self, b_point_data_list, parent=None):
        super().__init__(parent)
        self.setWindowTitle("B Point Localization Selection (Multiple)")
        self.setMinimumSize(900, 400)
        self.b_point_data_list = b_point_data_list 
        self.checkboxes = [] 
        self.init_ui()

    def init_ui(self):
        main_layout = QVBoxLayout(self)
        self.table = QTableWidget()
        self.table.setColumnCount(5) 
        self.table.setHorizontalHeaderLabels(["Select", "Index", "B Point (TCP_U) Pose", "B Point (Base) Pose", "OA Axis Rotation Angle (Deg)"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.NoSelection) 
        self._populate_table()
        main_layout.addWidget(self.table)
        
        btn_layout = QHBoxLayout()
        self.ok_btn = QPushButton("Confirm Selection")
        self.cancel_btn = QPushButton(f"Cancel (Loaded {len(self.b_point_data_list)} points)")
        btn_layout.addStretch()
        btn_layout.addWidget(self.ok_btn)
        btn_layout.addWidget(self.cancel_btn)
        btn_layout.addStretch()
        main_layout.addLayout(btn_layout)

        self.ok_btn.clicked.connect(self._accept_selection)
        self.cancel_btn.clicked.connect(self.reject)
        self.table.cellClicked.connect(self._toggle_checkbox_on_click) 

    def _populate_table(self):
        SELECTION_COLUMN_WIDTH = 180 
        self.table.setRowCount(len(self.b_point_data_list))
        
        for row, data in enumerate(self.b_point_data_list):
            p_u_pose, p_base_pose, angle, index = data 
            checkbox = QCheckBox()
            checkbox.setChecked(False)
            self.checkboxes.append(checkbox) 
            
            check_container = QWidget()
            check_layout = QHBoxLayout(check_container)
            check_layout.addWidget(checkbox)
            check_layout.setAlignment(Qt.AlignCenter)
            check_layout.setContentsMargins(0, 0, 0, 0)
            self.table.setCellWidget(row, 0, check_container)

            self.table.setItem(row, 1, QTableWidgetItem(f"B{index}"))
            p_u_str = f"({p_u_pose[0]:.2f}, {p_u_pose[1]:.2f}, {p_u_pose[2]:.2f}, {p_u_pose[3]:.2f}, {p_u_pose[4]:.2f}, {p_u_pose[5]:.2f})"
            self.table.setItem(row, 2, QTableWidgetItem(p_u_str))
            p_base_str = f"({p_base_pose[0]:.2f}, {p_base_pose[1]:.2f}, {p_base_pose[2]:.2f}, {p_base_pose[3]:.2f}, {p_base_pose[4]:.2f}, {p_base_pose[5]:.2f})"
            self.table.setItem(row, 3, QTableWidgetItem(p_base_str))
            self.table.setItem(row, 4, QTableWidgetItem(f"{angle:.2f}"))
        
        self.table.setColumnWidth(0, SELECTION_COLUMN_WIDTH) 
        self.table.resizeColumnsToContents()

    def _toggle_checkbox_on_click(self, row, column):
        if 0 <= row < len(self.checkboxes):
            self.checkboxes[row].setChecked(not self.checkboxes[row].isChecked())

    def _accept_selection(self):
        selected_data_list = []
        for row, checkbox in enumerate(self.checkboxes):
            if checkbox.isChecked(): 
                selected_data_list.append(self.b_point_data_list[row]) 

        if not selected_data_list:
            QMessageBox.warning(self, "Warning", "Please select at least one B point.")
            return

        self.b_points_selected.emit(selected_data_list) 
        self.accept()


class LeftPanel(QWidget):
    def __init__(self, tcp_manager, robot_kinematics, parent=None):
        super().__init__(parent)
        self.tcp_manager = tcp_manager
        self.robot_kinematics = robot_kinematics
        self.main_window = parent # 引用主窗口以便访问状态栏或右侧面板

        # 状态变量
        self.num_joints = 6
        self.joint_vars = [None] * self.num_joints
        self.tool_pose_labels = {}
        self.latest_tool_pose = [0.0] * 6 
        
        self.a_vars = [None] * 3
        self.o_vars = [None] * 3
        self.e_vars = [None] * 3
        self.b_vars_in_base = [None] * 3
        self.b_point_position_in_base = np.zeros(3)
        self.calculated_b_points = []
        
        # 点击左转x度的时候记录的 E 点位置
        self.tcp_e_medical_value = None
        # 录了超声平面刚刚对齐到 AOE 平面（即穿刺平面）时的机器臂状态
        self.tool_pose_in_puncture_position = None
        # 存储 TCP_U 的定义
        self.tcp_u_definition_pose = None 
        # 存储 TCP_tip 的定义
        self.tcp_tip_definition_pose = None  
        # 存储 TCP_P 的定义
        self.tcp_p_definition_pose = None
        # 点击左转x度的时候记录的 U 点位置
        self.tcp_u_volume = None
        # 存储计算后的 B 点在 TCP_P 坐标系下的位置
        self.b_point_in_tcp_p = None
        # 存储计算后的 A 点在 TCP_P 坐标系下的位置
        self.a_point_in_tcp_p = None

        # 旋转序列变量
        self.b_point_rotation_steps = []
        self.current_b_point_step = -1
        self.initial_tcp_pose_for_b_rot = None
        self.b_point_o_point = None

        # 微调控制变量
        self._moving_joint_index = -1
        self._moving_direction = BACKWARD
        self._is_continuous_mode_active = False
        self.continuous_move_timer = QTimer(self)
        self.continuous_move_timer.timeout.connect(self.continuous_move)
        self.start_continuous_move_timer = QTimer(self)
        self.start_continuous_move_timer.setSingleShot(True)
        self.start_continuous_move_timer.timeout.connect(self._start_joint_continuous_mode)

        # TCP微调变量
        self._moving_tcp_index = -1
        self._is_continuous_tcp_mode_active = False
        self.continuous_tcp_move_timer = QTimer(self)
        self.continuous_tcp_move_timer.timeout.connect(self.continuous_tcp_move)
        self.start_continuous_tcp_move_timer = QTimer(self)
        self.start_continuous_tcp_move_timer.setSingleShot(True)
        self.start_continuous_tcp_move_timer.timeout.connect(self._start_tcp_continuous_mode)
        self.is_fine_tuning_allowed = False
        self.is_fine_tuning_process = False # 标记是否正在执行 Fine tune probe 的序列

        # 状态机检查
        self.is_waiting_for_puncture_alignment = False
        self.fsm_check_timer = QTimer(self)
        self.fsm_check_timer.timeout.connect(self._check_fsm_status)

        self.init_ui()
        self.setup_connections()

    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # 1. 电机微调和速率
        top_left_layout = QHBoxLayout()
        self.create_motor_group(top_left_layout)
        self.create_override_group(top_left_layout)
        layout.addLayout(top_left_layout)
        
        # 2. 工具状态
        self.create_tool_state_group(layout)
        
        # 3. A/O/E 点
        self.create_record_oae_state_group(layout)
        
        # 4. B 点
        self.create_b_point_group(layout)
        
        # 5. 保存/加载
        self.create_data_persistence_group(layout)
        
        layout.addStretch()

    def create_motor_group(self, layout):
        group = QGroupBox("Motor Fine-tuning Module (Forward Kinematics)")
        v_layout = QVBoxLayout(group)
        v_layout.addStretch()
        for i in range(self.num_joints):
            row_layout = QHBoxLayout()
            row_layout.addStretch()
            label = QLabel(f"Joint {i+1} (q{i+1}):")
            value_label = QLineEdit("0.00")
            value_label.setReadOnly(True)
            value_label.setStyleSheet("background-color: lightgrey; border: 1px inset grey;")
            value_label.setFixedWidth(70)
            value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self.joint_vars[i] = value_label
            
            btn_minus = QPushButton("-")
            btn_minus.setFixedWidth(30)
            btn_minus.pressed.connect(lambda j=i: self.start_move(j, BACKWARD))
            btn_minus.released.connect(self.stop_move)
            
            btn_plus = QPushButton("+")
            btn_plus.setFixedWidth(30)
            btn_plus.pressed.connect(lambda j=i: self.start_move(j, FORWARD))
            btn_plus.released.connect(self.stop_move)
            
            row_layout.addWidget(label)
            row_layout.addWidget(value_label)
            row_layout.addSpacing(10)
            row_layout.addStretch()
            row_layout.addWidget(btn_minus)
            row_layout.addWidget(btn_plus)
            row_layout.addStretch()
            v_layout.addLayout(row_layout)
        
        v_layout.addStretch()
        init_btn_layout = QHBoxLayout()
        self.init_joint_pos_btn = QPushButton("Initialize Joint Position")
        self.init_joint_pos_btn.clicked.connect(self.send_init_joint_position_command)
        init_btn_layout.addStretch()
        init_btn_layout.addWidget(self.init_joint_pos_btn)
        init_btn_layout.addStretch()
        v_layout.addLayout(init_btn_layout)
        v_layout.addStretch()
        layout.addWidget(group)

    def create_override_group(self, layout):
        group = QGroupBox("Motion Speed")
        group_layout = QVBoxLayout(group)
        self.override_label = QLabel("Motion Speed: 0.01")
        self.override_label.setAlignment(Qt.AlignCenter)
        center_layout = QVBoxLayout()
        self.override_slider = QSlider(Qt.Vertical)
        self.override_slider.setMinimum(1)
        self.override_slider.setMaximum(100)
        self.override_slider.setValue(1)
        self.override_slider.setTickPosition(QSlider.TicksBothSides)
        self.override_slider.setTickInterval(10)
        self.override_slider.valueChanged.connect(self._on_override_slider_changed)
        self.override_slider.sliderReleased.connect(self._on_override_slider_released)
        center_layout.addWidget(self.override_slider, alignment=Qt.AlignCenter)
        current_override_read_layout = QVBoxLayout()
        self.current_override_label = QLabel("Current Motion Speed:")
        self.current_override_label.setAlignment(Qt.AlignCenter)
        self.current_override_value = QLineEdit("0.00")
        self.current_override_value.setReadOnly(True)
        self.current_override_value.setFixedWidth(60)
        self.current_override_value.setAlignment(Qt.AlignCenter)
        self.current_override_value.setStyleSheet("background-color: lightgrey;")
        current_override_read_layout.addWidget(self.current_override_label, alignment=Qt.AlignCenter)
        current_override_read_layout.addWidget(self.current_override_value, alignment=Qt.AlignCenter)
        group_layout.addStretch()
        group_layout.addWidget(self.override_label, alignment=Qt.AlignCenter)
        group_layout.addLayout(center_layout)
        group_layout.addLayout(current_override_read_layout)
        group_layout.addStretch()
        layout.addWidget(group)

    def create_tool_state_group(self, layout):
        group = QGroupBox("Robot Tool Real-time Status")
        group_layout = QGridLayout(group)
        tool_pose_info = [
            ("Tcp_X", "Tcp_X", None), ("Tcp_Y", "Tcp_Y", None), ("Tcp_Z", "Tcp_Z", None),
            ("Tcp_Rx", "Tcp_Rx", "Roll"), ("Tcp_Ry", "Tcp_Ry", "Pitch"), ("Tcp_Rz", "Tcp_Rz", "Yaw")
        ]
        
        for i, (base_label_text, var_key, additional_label_text) in enumerate(tool_pose_info):
            if i < 3: row_value, row_buttons = 0, 1
            else: row_value, row_buttons = 2, 3 
            col_start = (i % 3) * 2
            
            final_label_text = base_label_text + (f"\n({additional_label_text})" if additional_label_text else "")
            label = QLabel(final_label_text)
            label.setAlignment(Qt.AlignCenter)
            
            value_label = QLineEdit("0.00")
            value_label.setReadOnly(True) 
            value_label.setFixedWidth(70)
            value_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter) 
            value_label.setStyleSheet("background-color: lightgrey; border: 1px inset grey;")
            value_label.setFixedHeight(25) 
            self.tool_pose_labels[var_key] = value_label
            
            btn_minus = QPushButton("-")
            btn_minus.setFixedWidth(30)
            btn_minus.pressed.connect(lambda idx=i: self.start_tcp_move(idx, BACKWARD))
            btn_minus.released.connect(self.stop_tcp_move) 
            btn_plus = QPushButton("+")
            btn_plus.setFixedWidth(30)
            btn_plus.pressed.connect(lambda idx=i: self.start_tcp_move(idx, FORWARD))
            btn_plus.released.connect(self.stop_tcp_move)
            
            btn_layout = QHBoxLayout()
            btn_layout.addStretch()
            btn_layout.addWidget(btn_minus)
            btn_layout.addWidget(btn_plus)
            btn_layout.addStretch()
            
            group_layout.addWidget(label, row_value, col_start, alignment=Qt.AlignCenter)
            group_layout.addWidget(value_label, row_value, col_start + 1, alignment=Qt.AlignLeft) 
            group_layout.addLayout(btn_layout, row_buttons, col_start, 1, 2)
        
        self.fine_tune_probe_btn = QPushButton("Fine tune probe")
        self.fine_tune_probe_btn.clicked.connect(self.on_fine_tune_probe_clicked)
        group_layout.addWidget(self.fine_tune_probe_btn, 4, 0, 1, 6, alignment=Qt.AlignCenter)
        layout.addWidget(group)

    def create_record_oae_state_group(self, layout):
        group = QGroupBox("Robot A, O, and End-Effect Positions")
        group_layout = QVBoxLayout(group)
        BUTTON_WIDTH = 160
        
        def create_subgroup(title, prefix, vars_list, btn_text, slot):
            sub = QGroupBox(title)
            gl = QGridLayout(sub)
            labels = [f"{prefix}_x:", f"{prefix}_y:", f"{prefix}_z:"]
            for i, lt in enumerate(labels):
                gl.addWidget(QLabel(lt), 0, i*2)
                le = QLineEdit("0.00")
                le.setStyleSheet("background-color: white;")
                vars_list[i] = le
                gl.addWidget(le, 0, i*2+1)
            btn = QPushButton(btn_text)
            btn.setFixedWidth(BUTTON_WIDTH)
            btn.clicked.connect(slot)
            gl.addWidget(btn, 0, 6)
            return sub
        
        group_layout.addWidget(create_subgroup("A点", "A", self.a_vars, "Get A Point Position", self.get_a_point_position))
        group_layout.addWidget(create_subgroup("O点", "O", self.o_vars, "Get O Point Position", self.get_o_point_position))
        group_layout.addWidget(create_subgroup("End-Effect", "E", self.e_vars, "Get End-Effect Position", self.get_e_point_position))
        
        button_layout = QHBoxLayout()
        self.align_planes_btn = QPushButton("Rotate the ultrasound plane to pass through the puncture point")
        self.align_planes_btn.clicked.connect(self.align_ultrasound_plane_to_aoe)
        button_layout.addWidget(self.align_planes_btn)
        group_layout.addLayout(button_layout)
        group_layout.addStretch()
        layout.addWidget(group)

    def create_b_point_group(self, layout):
        group = QGroupBox("Lesion B Point Localization")
        group_layout = QVBoxLayout(group)
        BUTTON_WIDTH = 180 
        
        b_point_base_subgroup = QGroupBox("B Point (Base Coordinate System)")
        b_point_base_layout = QGridLayout(b_point_base_subgroup)
        b_base_labels = ["B_x:", "B_y:", "B_z:"]
        for i, label_text in enumerate(b_base_labels):
            label = QLabel(label_text)
            text_box = QLineEdit("0.00")
            text_box.setReadOnly(False) 
            text_box.setStyleSheet("background-color: white; border: 1px inset grey;") 
            self.b_vars_in_base[i] = text_box
            b_point_base_layout.addWidget(label, 0, i * 2)
            b_point_base_layout.addWidget(text_box, 0, i * 2 + 1)

        self.b_point_dropdown = QComboBox()
        self.b_point_dropdown.setPlaceholderText("Please read TXT file first")
        self.b_point_dropdown.setFixedWidth(BUTTON_WIDTH) 
        self.b_point_dropdown.currentIndexChanged.connect(self._handle_b_point_dropdown_selection)
        b_point_base_layout.addWidget(self.b_point_dropdown, 0, 6)
        group_layout.addWidget(b_point_base_subgroup)

        button_layout = QHBoxLayout()
        self.load_b_points_intcp_u_txt_btn = QPushButton("Read B Points in TCP_U from TXT File")
        self.load_b_points_intcp_u_txt_btn.clicked.connect(self.read_b_points_in_tcp_u_from_file)
        button_layout.addWidget(self.load_b_points_intcp_u_txt_btn) 
        group_layout.addLayout(button_layout)
        
        self.rotate_ultrasound_plane_to_b_btn = QPushButton("Rotate the ultrasound plane to pass through the biopsy point")
        self.rotate_ultrasound_plane_to_b_btn.clicked.connect(self.rotate_ultrasound_plane_to_b)
        group_layout.addWidget(self.rotate_ultrasound_plane_to_b_btn)
        group_layout.addStretch()
        layout.addWidget(group)

    def create_data_persistence_group(self, layout):
        group = QGroupBox("Save Current Data")
        group_layout = QHBoxLayout(group)
        self.save_data_btn = QPushButton("Save")
        self.load_data_btn = QPushButton("Load")
        self.save_data_btn.clicked.connect(self.save_data)
        self.load_data_btn.clicked.connect(self.load_data)
        group_layout.addStretch()
        group_layout.addWidget(self.save_data_btn)
        group_layout.addWidget(self.load_data_btn)
        group_layout.addStretch()
        layout.addWidget(group)

    def setup_connections(self):
        self.tcp_manager.message_received.connect(self.handle_incoming_message)

    def handle_incoming_message(self, message):
        if message.startswith("ReadActPos"):
            self._update_real_time_status(message)
        elif message.startswith("ReadOverride"):
            self._handle_override_message(message)
        elif message.startswith("WayPoint,OK"):
            self._continue_b_point_rotation()
        elif message.startswith("WayPoint"):
            self.main_window.right_panel.log_message(message)
        elif message.startswith("ReadCurFSM"):
            self.handle_fsm_message(message)

    def _update_real_time_status(self, message):
        parts = message.strip(';').strip(',').split(',')
        if len(parts) == 26 and parts[1] == 'OK':
            try:
                joint_params = [float(p) for p in parts[2:8]]
                for i in range(len(joint_params)):
                    if i < len(self.joint_vars):
                        self.joint_vars[i].setText(f"{joint_params[i]:.2f}")
                
                tool_pose_params = [float(p) for p in parts[8:14]]
                self.tool_pose_labels["Tcp_X"].setText(f"{tool_pose_params[0]:.2f}")
                self.tool_pose_labels["Tcp_Y"].setText(f"{tool_pose_params[1]:.2f}")
                self.tool_pose_labels["Tcp_Z"].setText(f"{tool_pose_params[2]:.2f}")
                self.tool_pose_labels["Tcp_Rx"].setText(f"{tool_pose_params[3]:.2f}")
                self.tool_pose_labels["Tcp_Ry"].setText(f"{tool_pose_params[4]:.2f}")
                self.tool_pose_labels["Tcp_Rz"].setText(f"{tool_pose_params[5]:.2f}")
                self.latest_tool_pose = tool_pose_params
            except (ValueError, IndexError):
                pass

    def _on_override_slider_changed(self, value):
        override_value = value / 100.0
        self.override_label.setText(f"Motion Speed: {override_value:.2f}")

    def _on_override_slider_released(self):
        d_override = self.override_slider.value() / 100.0
        command = f"SetOverride,0,{d_override:.2f};"
        self.tcp_manager.send_command(command)

    def _handle_override_message(self, message):
        parts = message.strip(';').strip(',').split(',')
        if len(parts) == 3 and parts[1] == 'OK':
            try:
                self.current_override_value.setText(f"{float(parts[2]):.2f}")
            except: pass

    # --- Motor Control ---
    def start_move(self, index, direction):
        self._moving_joint_index = index
        self._moving_direction = direction
        self._is_continuous_mode_active = False
        self.start_continuous_move_timer.start(2000)

    def stop_move(self):
        self.start_continuous_move_timer.stop()
        self.continuous_move_timer.stop()
        if not self._is_continuous_mode_active and self._moving_joint_index != -1:
            self.motor_adjust(self._moving_joint_index, self._moving_direction)
        self._moving_joint_index = -1
        self._is_continuous_mode_active = False

    def _start_joint_continuous_mode(self):
        self._is_continuous_mode_active = True
        self.motor_adjust(self._moving_joint_index, self._moving_direction)
        self.continuous_move_timer.start(300)

    def motor_adjust(self, index, direction):
        command = f"MoveRelJ,0,{index},{direction},1;"
        self.tcp_manager.send_command(command)

    def send_init_joint_position_command(self):
        J_STR = "-214.32,-36.68,70.4,-266.45,-90.83,-54.23"
        POS_ZERO = "0.00,0.00,0.00,0.00,0.00,0.00"
        cmd = f"WayPoint,0,{POS_ZERO},{J_STR},TCP,Base,50,360,0,0,1,0,0,0,ID1;"
        self.tcp_manager.send_command(cmd)

    # --- TCP Fine Tuning ---
    def start_tcp_move(self, index, direction):
        if not self.is_fine_tuning_allowed:
            QMessageBox.warning(self, "Operation Denied", "Fine tuning is disabled. Please click the 'Fine tune probe' button and wait for it to turn green.")
            return
        self._moving_tcp_index = index
        self._moving_direction = direction
        self._is_continuous_tcp_mode_active = False
        self.start_continuous_tcp_move_timer.start(2000)

    def stop_tcp_move(self):
        self.start_continuous_tcp_move_timer.stop()
        self.continuous_tcp_move_timer.stop()
        if not self._is_continuous_tcp_mode_active and self._moving_tcp_index != -1:
            self.tcp_adjust(self._moving_tcp_index, self._moving_direction)
        self._moving_tcp_index = -1
        self._is_continuous_tcp_mode_active = False

    def _start_tcp_continuous_mode(self):
        self._is_continuous_tcp_mode_active = True
        self.tcp_adjust(self._moving_tcp_index, self._moving_direction)
        self.continuous_tcp_move_timer.start(300)

    def tcp_adjust(self, index, direction):
        dDistance = 1.0
        command = f"MoveRelL,0,{index},{direction},{dDistance:.2f},1;"
        self.tcp_manager.send_command(command)

    def continuous_move(self):
        if self._moving_joint_index != -1: self.motor_adjust(self._moving_joint_index, self._moving_direction)

    def continuous_tcp_move(self):
        if self._moving_tcp_index != -1: self.tcp_adjust(self._moving_tcp_index, self._moving_direction)

    # --- Point Recording ---
    def get_a_point_position(self): self._start_point_record("A")
    def get_o_point_position(self): self._start_point_record("O")
    
    def _start_point_record(self, name):
        if not self.tcp_manager.is_connected: return
        if self.main_window and self.main_window.status_bar:
             # [修改] 提示切换到 TCP_E
             self.main_window.status_bar.showMessage(f"Status: Getting {name} Point (Switching TCP_E)...")
        
        # [修改] 发送切换到 TCP_E 的指令
        self.tcp_manager.send_command("SetTCPByName,0,TCP_E;")
        
        # 延时等待状态刷新后进行计算
        QTimer.singleShot(300, lambda: self._finalize_point_record(name))
        
    def _finalize_point_record(self, name):
        # 1. 检查必要数据是否存在
        if not self.latest_tool_pose:
            QMessageBox.warning(self, "Warning", "No real-time robot pose data.")
            return
        
        if not self.tcp_tip_definition_pose:
            QMessageBox.warning(self, "Warning", "TCP_tip definition missing. Please Connect to robot or Load data.")
            return

        try:
            # 2. 获取当前 TCP_E 在 Base 系下的位姿矩阵 (T_Base_E)
            # self.latest_tool_pose此时对应的是 TCP_E
            T_Base_E = self._pose_to_matrix(self.latest_tool_pose)

            # 3. 获取 TCP_tip 在 TCP_E 系下的相对位姿矩阵 (T_E_Tip)
            # 根据您的描述：tcp_tip_definition_pose 是基于 TCP_E 的
            T_E_Tip = self._pose_to_matrix(self.tcp_tip_definition_pose)

            # 4. 计算 TCP_tip 在 Base 系下的位姿 (T_Base_Tip)
            # 矩阵乘法：Base_Tip = Base_E * E_Tip
            T_Base_Tip = np.dot(T_Base_E, T_E_Tip)

            # 5. 提取位置 (x, y, z)
            tip_position_base = T_Base_Tip[:3, 3]

            # 6. 更新 UI 界面
            targets = self.a_vars if name == "A" else self.o_vars
            for i in range(3): 
                targets[i].setText(f"{tip_position_base[i]:.2f}")
            
            # 7. 状态栏反馈
            self.main_window.right_panel.log_message(f"Status: {name} point position calculated (Derived from TCP_E).")

        except Exception as e:
            QMessageBox.critical(self, "Calculation Error", f"Failed to calculate point: {e}")

    def get_e_point_position(self):
        if not self.latest_tool_pose: return
        for i in range(3): self.e_vars[i].setText(f"{self.latest_tool_pose[i]:.2f}")
        if self.main_window and self.main_window.status_bar:
             self.main_window.status_bar.showMessage("Status: End-Effect position obtained.")

    def get_current_tool_pose(self):
        try:
            return np.array(self.latest_tool_pose)
        except: return None
        
    # --- Alignment Logic ---
    def align_ultrasound_plane_to_aoe(self):
        if self.fine_tune_probe_btn: self.fine_tune_probe_btn.setStyleSheet("") 
        self.is_fine_tuning_allowed = False
        if not self.tcp_manager.is_connected: return
        self.tcp_manager.send_command("SetTCPByName,0,TCP_E;")
        QTimer.singleShot(200, self._continue_align_ultrasound_plane_to_aoe)

    def _continue_align_ultrasound_plane_to_aoe(self):
        self.get_e_point_position()
        QTimer.singleShot(200, self._finalize_align_ultrasound_plane_to_aoe)

    def _finalize_align_ultrasound_plane_to_aoe(self):
        try:
            a = np.array([float(v.text()) for v in self.a_vars])
            o = np.array([float(v.text()) for v in self.o_vars])
            e = np.array([float(v.text()) for v in self.e_vars])
            initial_pose = self.get_current_tool_pose()
            if initial_pose is None: return
            
            angle = calculate_rotation_for_plane_alignment(a, o, e, initial_pose[3:])
            if isinstance(angle, tuple): 
                QMessageBox.warning(self, "Error", angle[1])
                return
            
            nDirection = FORWARD if angle >= 0 else BACKWARD
            command = f"MoveRelJ,0,5,{nDirection},{abs(angle):.2f};"
            self.tcp_manager.send_command(command)
            
            if self.main_window and self.main_window.right_panel:
                self.main_window.right_panel.log_message(f"System: Sent rotation command ({abs(angle):.2f} deg). Waiting...")
            
            self.is_waiting_for_puncture_alignment = True
            QTimer.singleShot(500, lambda: self.fsm_check_timer.start(500))
        except ValueError:
            QMessageBox.critical(self, "Error", "Invalid Coordinates.")

    def _check_fsm_status(self): self.tcp_manager.send_command("ReadCurFSM,0;")

    def handle_fsm_message(self, message):
        if not self.is_waiting_for_puncture_alignment: return
        if ",33," in message:
            self.fsm_check_timer.stop()
            self.is_waiting_for_puncture_alignment = False
            QTimer.singleShot(300, self._finalize_puncture_point_recording)
            
    def _finalize_puncture_point_recording(self):
        if self.latest_tool_pose:
            self.tool_pose_in_puncture_position = list(self.latest_tool_pose)
            if self.main_window and self.main_window.right_panel:
                self.main_window.right_panel.log_message(f"System: Puncture Point Tool Pose Recorded: {self.tool_pose_in_puncture_position}")
            if self.main_window and self.main_window.status_bar:
                self.main_window.status_bar.showMessage("Status: Aligned to puncture point & Pose recorded.")

    def on_fine_tune_probe_clicked(self):
        # 情况 1: 当前已经是允许微调状态 (绿色) -> 执行关闭逻辑
        if self.is_fine_tuning_allowed:
            # 1. 恢复按钮默认颜色
            self.fine_tune_probe_btn.setStyleSheet("")
            
            # 2. 禁止微调
            self.is_fine_tuning_allowed = False
            self.is_fine_tuning_process = False 
            
            # 3. 切换回 TCP_E
            if self.tcp_manager.is_connected:
                self.tcp_manager.send_command("SetTCPByName,0,TCP_E;")
                if self.main_window and hasattr(self.main_window, 'right_panel'):
                    self.main_window.right_panel.log_message("System: Fine tune disabled. Switched back to TCP_E.")
            
            QMessageBox.information(self, "Status", "Fine tune disabled. Switched to TCP_E.")

        # 情况 2: 当前是初始状态 -> 准备执行开启序列
        else:
            # =================================================================
            # [新增] 限制：检查 O 点是否已获取 (判断是否全为 0)
            # =================================================================
            try:
                o_values = [float(v.text()) for v in self.o_vars]
                # 如果 O 点坐标全为 0，则视为未获取
                if all(v == 0.0 for v in o_values):
                     QMessageBox.warning(self, "Operation Denied", "Please 'Get O Point Position' (or Load Data) first.")
                     return
            except ValueError:
                 QMessageBox.warning(self, "Error", "Invalid O Point data found.")
                 return
            # =================================================================

            self.is_fine_tuning_process = True
            # 调用序列逻辑
            self.start_get_suitable_tcp_sequence(is_fine_tune=True)
    
    def start_get_suitable_tcp_sequence(self, is_fine_tune):
        if not self.tcp_manager.is_connected: return
        self.is_fine_tuning_process = is_fine_tune
        
        # 日志记录 (调用右侧面板)
        if self.main_window and hasattr(self.main_window, 'right_panel'):
            self.main_window.right_panel.log_message("System: Starting Get Suitable TCP Sequence (Logic in LeftPanel)...")
            
        # 1. 切换到 TCP_E
        self.tcp_manager.send_command("SetTCPByName,0,TCP_E;")
        QTimer.singleShot(200, self._step_1_get_e)

    def _step_1_get_e(self):
        # 2. 获取当前 E 点位置 (直接调用自身方法)
        self.get_e_point_position()
        QTimer.singleShot(200, self._step_2_calc)

    def _step_2_calc(self):
        # 3. 计算逻辑
        try:
            # 获取 O 点和 E 点数值 (自身变量)
            o_pt = np.array([float(v.text()) for v in self.o_vars])
            e_pt = np.array([float(v.text()) for v in self.e_vars])
            
            # 获取当前工具姿态 (自身变量)
            cur_pose = self.latest_tool_pose
            if not cur_pose: return

            # 计算 Z 轴方向上的投影距离
            rpy_rad = np.deg2rad(cur_pose[3:])
            rot = pyrot.matrix_from_euler(rpy_rad, 0, 1, 2, extrinsic=True)
            z_vec = rot[:, 2]
            dist = np.dot(o_pt - e_pt, z_vec)
            
            # --- 跨面板操作：更新 RightPanel 的 TCP 输入框 ---
            if self.main_window and hasattr(self.main_window, 'right_panel'):
                rp = self.main_window.right_panel
                # 重置输入框
                for le in rp.tcp_input_entries: le.setText("0.00")
                # 设置 Z 值 (Calculated distance)
                rp.tcp_input_entries[2].setText(f"{dist:.2f}")
                # 设置 Rz 值 (Fixed offset)
                rp.tcp_input_entries[5].setText("157.50")
            
            # 更新自身的 O 点显示 (新的 O 点是 E 点沿 Z 轴偏移后的位置)
            p_o_new = cur_pose[:3] + dist * z_vec
            for i in range(3): self.o_vars[i].setText(f"{p_o_new[i]:.2f}")
            
            # 发送切换到 TCP_O (此时 TCP_O 还是旧值，或者是全0，准备在下一步更新)
            # 注意：原始逻辑这里是为了确保后续 SetCurTCP 有效，或者先切到一个临时状态
            self.tcp_manager.send_command("SetTCPByName,0,TCP_O;")
            QTimer.singleShot(200, self._step_3_set_cur)
            
        except Exception as e: 
            if self.main_window and hasattr(self.main_window, 'right_panel'):
                self.main_window.right_panel.log_message(f"Error in calc: {e}")

    def _step_3_set_cur(self):
        # 4. 触发 RightPanel 的 "Set Cur TCP" 逻辑 (发送 ConfigTCP 指令)
        if self.main_window and hasattr(self.main_window, 'right_panel'):
            self.main_window.right_panel.send_set_tcp_command()
        QTimer.singleShot(200, self._finalize_sequence)
        
    def _finalize_sequence(self):
        if self.is_fine_tuning_process:
            # [关键修改] 额外强制切换到 TCP_O
            # 这是为了覆盖可能因 SetCurTCP 指令导致的 TCP 切换副作用
            self.tcp_manager.send_command("SetTCPByName,0,TCP_O;")
            
            # 允许微调 (启用 "+","-" 按钮逻辑)
            self.is_fine_tuning_allowed = True
            
            # 按钮变色 (绿色)
            self.fine_tune_probe_btn.setStyleSheet("background-color: lightgreen;")
            
            # 反馈
            if self.main_window and hasattr(self.main_window, 'right_panel'):
                self.main_window.right_panel.log_message("System: Fine tune enabled (Switched to TCP_O).")
            QMessageBox.information(self, "Ready", "Fine tuning enabled (TCP_O).")
            
        else:
            # 如果不是 fine tune 流程（虽然目前只有 fine tune 用这个），默认切回 TCP_E
            self.tcp_manager.send_command("SetTCPByName,0,TCP_E;")
            QMessageBox.information(self, "Success", "Sequence completed.")

    # --- B Point Logic ---
    def rotate_ultrasound_plane_to_b(self):
        if not self.tcp_manager.is_connected: return
        self.tcp_manager.send_command("SetTCPByName,0,TCP_E;")
        QTimer.singleShot(200, self._start_b_point_rotation_sequence)

    def _start_b_point_rotation_sequence(self):
        try:
            a = np.array([float(v.text()) for v in self.a_vars])
            o = np.array([float(v.text()) for v in self.o_vars])
            if np.all(self.b_point_position_in_base == 0):
                QMessageBox.warning(self, "Failed", "Invalid B point.")
                return
            b = self.b_point_position_in_base
            init_pose = self.get_current_tool_pose()
            
            # 计算旋转矩阵列表 deltas (这里会使用默认或你传入的 step_size_deg)
            deltas = calculate_new_rpy_for_b_point(a, o, b, init_pose[3:])
            
            if not deltas or (len(deltas)==1 and np.allclose(deltas[0], np.identity(4))):
                QMessageBox.information(self, "Info", "No rotation needed.")
                return
            
            self.b_point_rotation_steps = deltas
            self.initial_tcp_pose_for_b_rot = init_pose
            self.b_point_o_point = o
            
            # =========================================================================
            # [新增代码] 遍历所有步进，预计算并打印每一步的 E 点目标姿态
            # =========================================================================
            print(f"\n--- [预计算] 即将执行的 {len(deltas)} 个步进的目标 E 点姿态 ---")
            
            # 提前准备初始旋转矩阵 (R_init)
            init_rpy_rad = np.deg2rad(init_pose[3:])
            R_init = pyrot.matrix_from_euler(init_rpy_rad, 0, 1, 2, extrinsic=True)
            
            for i, delta in enumerate(deltas):
                # 1. 计算该步的位置 (Position)
                # 使用 core/ultrasound_plane.py 中已有的函数
                P_final = get_final_tcp_e_position_after_delta_rotation(init_pose[:3], delta, o)
                
                # 2. 计算该步的姿态 (Rotation RPY)
                # R_final = Delta_R * R_init
                R_final = np.dot(delta[:3, :3], R_init)
                final_rpy_deg = np.rad2deg(pyrot.euler_from_matrix(R_final, 0, 1, 2, extrinsic=True))
                
                # 3. 拼接并打印: [x, y, z, rx, ry, rz]
                full_pose = np.concatenate((P_final, final_rpy_deg))
                pose_str = ", ".join([f"{v:.4f}" for v in full_pose])
                
                # 打印到控制台
                print(f"Step {i+1}/{len(deltas)} Target E-Pose: [{pose_str}]")
                
                # (可选) 同时打印到软件界面的日志窗口，方便查看
                if self.main_window and hasattr(self.main_window, 'right_panel'):
                     self.main_window.right_panel.log_message(f"[Predict] Step {i+1}: [{pose_str}]")
            
            print("-----------------------------------------------------------------\n")
            # =========================================================================

            self.current_b_point_step = -1
            self._continue_b_point_rotation()
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Rotation Calc Failed: {e}")

    def _continue_b_point_rotation(self):
        if not self.b_point_rotation_steps: return
        self.current_b_point_step += 1
        if self.current_b_point_step < len(self.b_point_rotation_steps):
            delta = self.b_point_rotation_steps[self.current_b_point_step]
            P_final = get_final_tcp_e_position_after_delta_rotation(self.initial_tcp_pose_for_b_rot[:3], delta, self.b_point_o_point)
            
            init_rpy_rad = np.deg2rad(self.initial_tcp_pose_for_b_rot[3:])
            R_init = pyrot.matrix_from_euler(init_rpy_rad, 0, 1, 2, extrinsic=True)
            R_final = np.dot(delta[:3,:3], R_init)
            final_rpy_deg = np.rad2deg(pyrot.euler_from_matrix(R_final, 0, 1, 2, extrinsic=True))
            
            pos_rpy_str = ",".join([f"{v:.2f}" for v in np.concatenate((P_final, final_rpy_deg))])
            dJ_zero = ",".join(["0.00"] * 6)
            cmd = f"WayPoint,0,{pos_rpy_str},{dJ_zero},TCP,Base,50,250,0,1,0,0,0,0,ID1;"
            QTimer.singleShot(400, lambda: self.tcp_manager.send_command(cmd))
            print(cmd)
            
            step_angle = np.rad2deg(pyrot.axis_angle_from_matrix(delta[:3, :3])[3])
            if self.main_window and self.main_window.right_panel:
                self.main_window.right_panel.log_message(cmd)
                self.main_window.right_panel.log_message(f"Status: Rotating Step {self.current_b_point_step+1}/{len(self.b_point_rotation_steps)}, cumulative {step_angle:.2f} deg")
        else:
            self.b_point_rotation_steps = []
            # =================================================================
            # 旋转完成，计算 B 点，A 点在 TCP_P 坐标系下的位置
            # =================================================================
            QTimer.singleShot(300, lambda: self._calculate_b_point_in_tcp_p())
            self._calculate_a_point_in_tcp_p()
            
            QMessageBox.information(self, "Done", "Rotation to B point completed.\nB point in TCP_P calculated.")
            
    def _calculate_b_point_in_tcp_p(self):
        """计算 B 点在 TCP_P 坐标系下的位置。"""
        # 1. 检查必要数据
        if self.tcp_p_definition_pose is None:
            print("Error: TCP_P definition is missing.")
            return
        if self.latest_tool_pose is None:
            print("Error: Robot tool pose is missing.")
            return
        if self.b_point_position_in_base is None:
            print("Error: B point (Base) is not selected.")
            return

        try:
            # 2. 获取当前机器臂(TCP_E)在 Base 下的位姿矩阵 T_Base_E
            # 此时机器臂已经运动到了目标位置
            T_Base_E = self._pose_to_matrix(self.latest_tool_pose)

            # 3. 获取 TCP_P 在 TCP_E 下的位姿矩阵 T_E_P
            # 根据需求：tcp_p_definition_pose 记录的是相对于工具端(TCP_E)的姿态
            T_E_P = self._pose_to_matrix(self.tcp_p_definition_pose)

            # 4. 计算 TCP_P 在 Base 下的位姿矩阵 T_Base_P
            # T_Base_P = T_Base_E * T_E_P
            T_Base_P = np.dot(T_Base_E, T_E_P)

            # 5. 获取 B 点在 Base 下的坐标 P_B_Base (补齐为齐次坐标)
            # self.b_point_position_in_base 是 [x, y, z]
            b_point_base = np.append(self.b_point_position_in_base, 1.0)

            # 6. 计算 B 点在 TCP_P 下的坐标 P_B_P
            # P_B_P = inv(T_Base_P) * P_B_Base
            T_Base_P_Inv = np.linalg.inv(T_Base_P)
            b_point_p_homogeneous = np.dot(T_Base_P_Inv, b_point_base)
            
            # 提取 x, y, z
            self.b_point_in_tcp_p = b_point_p_homogeneous[:3].tolist()
            
            # 打印日志方便调试
            print(f"Calculated B point in TCP_P: {self.b_point_in_tcp_p}")
            if self.main_window and hasattr(self.main_window, 'right_panel'):
                self.main_window.right_panel.log_message(f"System: B point in TCP_P calculated: {self.b_point_in_tcp_p}")

        except Exception as e:
            print(f"Calculation Error: {e}")
            
    def _calculate_a_point_in_tcp_p(self):
        """[新增] 计算 A 点在 TCP_P 坐标系下的位置。"""
        # 1. 检查必要数据
        if self.tcp_p_definition_pose is None:
            print("Error (A_in_P): TCP_P definition is missing.")
            return
        if self.latest_tool_pose is None:
            print("Error (A_in_P): Robot tool pose is missing.")
            return
        
        # 获取 A 点基座坐标 (从界面输入框读取)
        try:
            a_vals = [float(v.text()) for v in self.a_vars]
            if all(v == 0 for v in a_vals):
                print("Warning (A_in_P): A point is (0,0,0), skipping calculation.")
                return
            a_point_base = np.array(a_vals + [1.0]) # 齐次坐标
        except ValueError:
            print("Error (A_in_P): Invalid A point values.")
            return

        try:
            # 2. 获取当前机器臂(TCP_E)在 Base 下的位姿矩阵 T_Base_E
            T_Base_E = self._pose_to_matrix(self.latest_tool_pose)

            # 3. 获取 TCP_P 在 TCP_E 下的位姿矩阵 T_E_P
            T_E_P = self._pose_to_matrix(self.tcp_p_definition_pose)

            # 4. 计算 TCP_P 在 Base 下的位姿矩阵 T_Base_P
            T_Base_P = np.dot(T_Base_E, T_E_P)

            # 5. 计算 A 点在 TCP_P 下的坐标 P_A_P
            # P_A_P = inv(T_Base_P) * P_A_Base
            T_Base_P_Inv = np.linalg.inv(T_Base_P)
            a_point_p_homogeneous = np.dot(T_Base_P_Inv, a_point_base)
            
            # 提取 x, y, z
            self.a_point_in_tcp_p = a_point_p_homogeneous[:3].tolist()
            
            # 打印日志
            print(f"Calculated A point in TCP_P: {self.a_point_in_tcp_p}")
            if self.main_window and hasattr(self.main_window, 'right_panel'):
                self.main_window.right_panel.log_message(f"System: A point in TCP_P calculated: {self.a_point_in_tcp_p}")

        except Exception as e:
            print(f"Calculation Error (A_in_P): {e}")

    def compute_and_store_tcp_u_volume(self):
        if self.tcp_e_medical_value is None or self.tcp_u_definition_pose is None: return
        try:
            T_Base_E = self._pose_to_matrix(self.tcp_e_medical_value)
            T_E_U = self._pose_to_matrix(self.tcp_u_definition_pose)
            T_Base_U = np.dot(T_Base_E, T_E_U)
            trans = T_Base_U[:3, 3]
            rpy_deg = np.rad2deg(pyrot.euler_from_matrix(T_Base_U[:3, :3], 0, 1, 2, extrinsic=True))
            self.tcp_u_volume = np.concatenate((trans, rpy_deg)).tolist()
            if self.main_window and self.main_window.right_panel:
                self.main_window.right_panel.log_message(f"System: tcp_u_volume: {self.tcp_u_volume}")
        except Exception as e: print(e)

    def _pose_to_matrix(self, pose):
        x, y, z, rx, ry, rz = pose
        T = np.identity(4)
        T[:3, :3] = pyrot.matrix_from_euler(np.deg2rad([rx, ry, rz]), 0, 1, 2, extrinsic=True)
        T[:3, 3] = np.array([x, y, z])
        return T

    def read_b_points_in_tcp_u_from_file(self):
        if self.tcp_u_definition_pose is None:
            QMessageBox.warning(self, "Warning", "TCP_U definition missing.")
            return
        if self.tcp_e_medical_value is None or self.tool_pose_in_puncture_position is None:
            QMessageBox.warning(self, "Missing Data", "Record tcp_e_medical_value and tool_pose_in_puncture_position first.")
            return
        
        # ... (Simplified file dialog logic similar to original) ...
        # Assuming file is read into b_point_data_list
        FILE_NAME = "TCP_U_B_LIST.txt"
        file_path = os.path.join(os.getcwd(), FILE_NAME)
        if not os.path.exists(file_path): return 

        b_point_data_list = []
        try:
            with open(file_path, 'r') as f:
                index = 1
                for line in f:
                    line = line.strip().strip('[] \t\n')
                    if not line: continue
                    parts = line.split(',')
                    if len(parts) != 3: continue
                    p_b_u = np.array([float(p) for p in parts] + [0.0]*3)
                    p_base_pose, angle, idx = self._calculate_b_point_data(p_b_u, index)
                    b_point_data_list.append((p_b_u, p_base_pose, angle, idx))
                    index += 1
        except Exception as e: 
            QMessageBox.critical(self, "Error", str(e))
            return

        dialog = BPointSelectionDialog(b_point_data_list, self)
        dialog.b_points_selected.connect(self._handle_b_points_list_selection)
        dialog.exec_()

    def _calculate_b_point_data(self, p_b_in_u_pose, index):
        if self.tcp_e_medical_value is None or self.tool_pose_in_puncture_position is None: raise ValueError("Missing poses")
        T_U_to_B = self._pose_to_matrix(p_b_in_u_pose)
        T_Base_to_E = self._pose_to_matrix(self.tcp_e_medical_value)
        T_E_to_U = self._pose_to_matrix(self.tcp_u_definition_pose)
        T_Base_to_B = T_Base_to_E @ T_E_to_U @ T_U_to_B
        
        p_b_in_base = T_Base_to_B[:3, 3]
        r_b_in_base_rpy = np.rad2deg(pyrot.euler_from_matrix(T_Base_to_B[:3, :3], 0, 1, 2, extrinsic=True))
        p_b_in_base_pose = np.append(p_b_in_base, r_b_in_base_rpy)
        
        a = np.array([float(v.text()) for v in self.a_vars])
        o = np.array([float(v.text()) for v in self.o_vars])
        angle = self._calculate_oa_rotation_angle(a, o, p_b_in_base, np.array(self.tool_pose_in_puncture_position)[3:], p_b_in_u_pose)
        return p_b_in_base_pose, angle, index

    def _calculate_oa_rotation_angle(self, a, o, b, initial_rpy, p_b_u):
        # (Logic copied from original main_window.py)
        oa = a - o
        if np.linalg.norm(oa) < 1e-6: return 0.0
        oa_u = oa / np.linalg.norm(oa)
        
        rpy_rad = np.deg2rad(initial_rpy)
        tool_rot = pyrot.matrix_from_euler(rpy_rad, 0, 1, 2, extrinsic=True)
        rot_delta = pyrot.matrix_from_axis_angle([0,0,1,np.deg2rad(-22.5)])
        u_normal = np.dot(tool_rot, rot_delta)[:, 1]
        
        ob = b - o
        if np.linalg.norm(ob) < 1e-6: return 0.0
        n_oab = np.cross(oa, ob)
        if np.linalg.norm(n_oab) < 1e-6: return 0.0
        
        proj_u = u_normal - np.dot(u_normal, oa_u) * oa_u
        proj_t = n_oab - np.dot(n_oab, oa_u) * oa_u
        
        if np.linalg.norm(proj_u) < 1e-6 or np.linalg.norm(proj_t) < 1e-6: return 0.0
        
        y_plane = np.cross(oa_u, proj_u)
        x_comp = np.dot(proj_t, proj_u)
        y_comp = np.dot(proj_t, y_plane)
        return np.rad2deg(np.arctan2(y_comp, x_comp))

    def _handle_b_points_list_selection(self, selected_list):
        self.calculated_b_points = selected_list
        self.b_point_dropdown.clear()
        if not selected_list:
            self.b_point_dropdown.setPlaceholderText("No points")
            return
        for data in selected_list:
            self.b_point_dropdown.addItem(f"B{data[3]}, {data[2]:.2f}deg")
        self.b_point_dropdown.setCurrentIndex(0)

    def _handle_b_point_dropdown_selection(self, idx):
        if idx < 0 or idx >= len(self.calculated_b_points): return
        p_base = self.calculated_b_points[idx][1][:3]
        self.b_point_position_in_base = p_base
        for i in range(3): self.b_vars_in_base[i].setText(f"{p_base[i]:.2f}")

    def _get_ui_values(self, vars_list):
        try: return [float(v.text()) for v in vars_list]
        except: return None
        
    def save_data(self):
        try:
            data = {
                # 原有的保存项
                'a_point': self._get_ui_values(self.a_vars),
                'o_point': self._get_ui_values(self.o_vars),
                'e_point': self._get_ui_values(self.e_vars),
                'calculated_b_points': [(d[0].tolist(), d[1].tolist(), d[2], d[3]) for d in self.calculated_b_points],
                
                # --- 新增保存的变量 ---
                'tcp_e_medical_value': self.tcp_e_medical_value,
                'tool_pose_in_puncture_position': self.tool_pose_in_puncture_position,
                'tcp_u_volume': self.tcp_u_volume,
                'b_point_in_tcp_p': self.b_point_in_tcp_p,
                'a_point_in_tcp_p': self.a_point_in_tcp_p
            }
            
            with open(DATA_FILE_NAME, 'w') as f: 
                json.dump(data, f, indent=4)
                
            QMessageBox.information(self, "Saved", f"Saved to {DATA_FILE_NAME}")
        except Exception as e: 
            QMessageBox.critical(self, "Error", str(e))

    def load_data(self):
        if not os.path.exists(DATA_FILE_NAME): return
        try:
            with open(DATA_FILE_NAME, 'r') as f: 
                data = json.load(f)
            
            # 1. 恢复界面上的 A, O, E 点数值
            for i in range(3):
                self.a_vars[i].setText(f"{data.get('a_point',[0]*3)[i]:.2f}")
                self.o_vars[i].setText(f"{data.get('o_point',[0]*3)[i]:.2f}")
                self.e_vars[i].setText(f"{data.get('e_point',[0]*3)[i]:.2f}")
            
            # 2. 恢复计算过的 B 点列表
            self.calculated_b_points = []
            for p_u, p_base, ang, idx in data.get('calculated_b_points', []):
                self.calculated_b_points.append((np.array(p_u), np.array(p_base), float(ang), int(idx)))
            
            # 3. 刷新 B 点下拉框
            self.b_point_dropdown.clear()
            for d in self.calculated_b_points:
                self.b_point_dropdown.addItem(f"B{d[3]}, {d[2]:.2f}deg")
                
            # --- 4. 恢复新增的变量 ---
            # 使用 .get() 方法以防止旧的 json 文件中没有这些键而报错
            self.tcp_e_medical_value = data.get('tcp_e_medical_value')
            self.tool_pose_in_puncture_position = data.get('tool_pose_in_puncture_position')
            self.tcp_u_volume = data.get('tcp_u_volume')
            self.b_point_in_tcp_p = data.get('b_point_in_tcp_p')
            self.a_point_in_tcp_p = data.get('a_point_in_tcp_p')

            # 可选：在控制台打印一下，确认加载成功
            print("Loaded additional robot states:")
            print(f"  - TCP_E Medical: {self.tcp_e_medical_value}")
            print(f"  - Puncture Pose: {self.tool_pose_in_puncture_position}")
            print(f"  - TCP_U Volume: {self.tcp_u_volume}")
            print(f"  - B_point_in_TCP_P: {self.b_point_in_tcp_p}")
            print(f"  - A_point_in_TCP_P: {self.a_point_in_tcp_p}")
            
            QMessageBox.information(self, "Success", "Data loaded successfully.")

        except Exception as e: 
            QMessageBox.critical(self, "Error", str(e))