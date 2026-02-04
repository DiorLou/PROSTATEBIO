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
import glob
import shutil

# 常量
FORWARD = 1
BACKWARD = 0
DATA_FILE_NAME = "saved_robot_data.json"

class BPointSelectionDialog(QDialog):
    """
    B点选择对话框：更新为显示 B(Volume) 原始坐标和计算后的 B(Base) 坐标。
    """
    b_points_selected = pyqtSignal(list) 

    def __init__(self, b_point_data_list, a_index, parent=None):
        super().__init__(parent)
        # [修改] 窗口标题增加当前所属的 A 点编号，方便用户核对
        self.setWindowTitle(f"B Point Selection for A{a_index} (Volume-based)") 
        self.setMinimumSize(1000, 450)
        self.b_point_data_list = b_point_data_list 
        self.checkboxes = [] 
        self.init_ui()

    def init_ui(self):
        main_layout = QVBoxLayout(self)
        self.table = QTableWidget()
        # [修改] 列数和表头，使其符合新的坐标体系
        self.table.setColumnCount(5) 
        self.table.setHorizontalHeaderLabels([
            "Select", 
            "B ID", 
            "B Point (Volume) [x, y, z]", 
            "B Point (Base) [x, y, z]", 
            "OA Rotation Angle (Deg)"
        ])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.NoSelection) 
        self._populate_table()
        main_layout.addWidget(self.table)
        
        btn_layout = QHBoxLayout()
        self.ok_btn = QPushButton("Confirm Selection")
        # [修改] 取消按钮显示当前加载的点位数量
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
        SELECTION_COLUMN_WIDTH = 100 
        self.table.setRowCount(len(self.b_point_data_list))
        
        for row, data in enumerate(self.b_point_data_list):
            # 这里的 data 结构需对应新函数传来的: (p_b_vol, p_base_pose, angle, b_idx)
            p_vol_xyz, p_base_pose, angle, b_idx = data 
            
            checkbox = QCheckBox()
            checkbox.setChecked(False)
            self.checkboxes.append(checkbox) 
            
            check_container = QWidget()
            check_layout = QHBoxLayout(check_container)
            check_layout.addWidget(checkbox)
            check_layout.setAlignment(Qt.AlignCenter)
            check_layout.setContentsMargins(0, 0, 0, 0)
            self.table.setCellWidget(row, 0, check_container)

            # [修改] 展示 B1-B16 的隐形编号
            self.table.setItem(row, 1, QTableWidgetItem(f"B{b_idx}"))
            
            # [修改] 展示 Volume 系下的原始坐标 (来自 biopsy_target_replan.txt 前三列)
            p_vol_str = f"({p_vol_xyz[0]:.2f}, {p_vol_xyz[1]:.2f}, {p_vol_xyz[2]:.2f})"
            self.table.setItem(row, 2, QTableWidgetItem(p_vol_str))
            
            # [修改] 展示转换到 Base 系后的坐标
            p_base_str = f"({p_base_pose[0]:.2f}, {p_base_pose[1]:.2f}, {p_base_pose[2]:.2f})"
            self.table.setItem(row, 3, QTableWidgetItem(p_base_str))
            
            # 展示计算出的旋转角度
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
        self.b_point_in_base = np.zeros(3)
        self.calculated_b_points = []
        
        # A Point List for Dropdown
        self.a_points_in_base_list = [] 
        
        # 点击左转x度的时候记录的 E 点位置
        self.tcp_e_in_ultrasound_zero_deg = None
        # 录了超声平面刚刚对齐到 AOE 平面（即穿刺平面）时的机器臂状态
        self.tcp_e_in_puncture_position = None
        # 存储 TCP_U 的定义
        self.tcp_u_definition_pose = None 
        # 存储 TCP_tip 的定义
        self.tcp_tip_definition_pose = None  
        # 存储 TCP_P 的定义
        self.tcp_p_definition_pose = None
        # 点击左转x度的时候记录的 U 点位置
        self.volume_in_base = None
        # 存储计算后的 B 点在 TCP_P 坐标系下的位置
        self.b_point_in_tcp_p = None
        # 存储计算后的 A 点在 TCP_P 坐标系下的位置
        self.a_point_in_tcp_p = None
        # 存储计算后的 A 点在 Volume 坐标系下的位置
        self.a_point_in_volume = None

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
            
            is_a_point = (prefix == "A")
            # [修改] 统一设为 1，确保所有组件在同一水平线上
            row_span = 1 
            
            labels = [f"{prefix}_x:", f"{prefix}_y:", f"{prefix}_z:"]
            for i, lt in enumerate(labels):
                lbl = QLabel(lt)
                # [修改] 保持在第 0 行
                gl.addWidget(lbl, 0, i*2, row_span, 1, Qt.AlignVCenter | Qt.AlignRight)
                
                le = QLineEdit("0.00")
                le.setStyleSheet("background-color: white;")
                vars_list[i] = le
                # [修改] 保持在第 0 行
                gl.addWidget(le, 0, i*2+1, row_span, 1, Qt.AlignVCenter)
                
            if not is_a_point:
                btn = QPushButton(btn_text)
                btn.setFixedWidth(BUTTON_WIDTH)
                btn.clicked.connect(slot)
                # 按钮在第 0 行
                gl.addWidget(btn, 0, 6)
            
            # [NEW] Add Dropdown for A Point
            if is_a_point:
                self.a_point_dropdown = QComboBox()
                self.a_point_dropdown.setPlaceholderText("History (Empty)")
                self.a_point_dropdown.setFixedWidth(BUTTON_WIDTH)
                self.a_point_dropdown.currentIndexChanged.connect(self._on_a_point_selected)
                # [修改] 将 row 从 1 改为 0，使其与左侧文本框在同一水平行
                gl.addWidget(self.a_point_dropdown, 0, 6, 1, 1, Qt.AlignVCenter)
                
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
        self.load_a_points_in_volume_txt_btn = QPushButton("Read A Points (Volume) from Replan File")
        self.load_a_points_in_volume_txt_btn.clicked.connect(self.read_a_points_volume_and_update_base)
        button_layout.addWidget(self.load_a_points_in_volume_txt_btn) 
        self.load_b_points_in_volume_txt_btn = QPushButton("Read B Points (Volume) from Replan File")
        self.load_b_points_in_volume_txt_btn.clicked.connect(self.read_b_points_volume_from_file_according_to_selected_a)
        button_layout.addWidget(self.load_b_points_in_volume_txt_btn) 
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
                print("Error _update_real_time_status")

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
            except Exception as e:
                print(f"Error _handle_override_message: {e}")

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
        J_STR = "-237.7,-53.88,57.53,94.2,-90.18,-45.6"
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

    # --- [NEW] Helper method to unify TCP switching ---
    def _switch_tcp(self, tcp_name):
        """
        统一的 TCP 切换内部辅助函数。
        尝试调用 RightPanel 的 switch_tcp 以保持状态同步；
        若不可用（else），则直接弹出报警，不进行强制切换。
        """
        # 1. 检查是否存在 main_window 及其 right_panel 组件
        if self.main_window and hasattr(self.main_window, 'right_panel'):
            # 调用 RightPanel 中封装好的 switch_tcp (包含取消勾选和 200ms 延时逻辑)
            self.main_window.right_panel.switch_tcp(tcp_name)
        
        else:
            # 2. 如果找不到 RightPanel，直接弹窗报警，不做任何切换操作
            QMessageBox.critical(
                self, 
                "切换失败", 
                f"无法执行 TCP 切换：未找到右侧控制面板 (RightPanel)。\n目标 TCP: {tcp_name}"
            )

    # --- Point Recording ---
    def get_a_point_position(self): self._start_point_record("A")
    def get_o_point_position(self): self._start_point_record("O")
    
    def _start_point_record(self, name):
        if not self.tcp_manager.is_connected: return
        
        # 延时等待状态刷新后进行计算
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
            T_Base_E = self.pose_to_matrix(self.latest_tool_pose)

            # 3. 获取 TCP_tip 在 TCP_E 系下的相对位姿矩阵 (T_E_Tip)
            # 根据您的描述：tcp_tip_definition_pose 是基于 TCP_E 的
            T_E_Tip = self.pose_to_matrix(self.tcp_tip_definition_pose)

            # 4. 计算 TCP_tip 在 Base 系下的位姿 (T_Base_Tip)
            # 矩阵乘法：Base_Tip = Base_E * E_Tip
            T_Base_Tip = np.dot(T_Base_E, T_E_Tip)

            # 5. 提取位置 (x, y, z)
            tip_position_base = T_Base_Tip[:3, 3]

            # 6. 更新 UI 界面
            targets = self.a_vars if name == "A" else self.o_vars
            for i in range(3): 
                targets[i].setText(f"{tip_position_base[i]:.2f}")
            
            # [NEW] Add to History if A Point
            if name == "O":
                # 2. 计算 A1 和 A2 (相对于 TCP_Tip 坐标系)
                # A1: X=+25, Y=-25, Z=0
                # A2: X=+25, Y=+25, Z=0
                p_a1_in_tip = np.array([25.0, -25.0, 0.0, 1.0])
                p_a2_in_tip = np.array([25.0, 25.0, 0.0, 1.0])

                # 转换到 Base 系
                p_a1_base = np.dot(T_Base_Tip, p_a1_in_tip)[:3]
                p_a2_base = np.dot(T_Base_Tip, p_a2_in_tip)[:3]

                # 3. 存储到历史记录 self.a_points_in_base_list
                # 清空旧的 A 点记录（根据需求“Get A Point”已停用，这里由 O 点驱动生成最新的 A1, A2）
                self.a_points_in_base_list = [p_a1_base.tolist(), p_a2_base.tolist()]
            
            # 7. 状态栏反馈
            self.main_window.right_panel.log_message(f"Status: {name} point position calculated (Derived from TCP_E). A1 & A2 calculated via TCP_Tip.")

        except Exception as e:
            QMessageBox.critical(self, "Calculation Error", f"Failed to calculate point: {e}")

    # [NEW] Slot for A Point Dropdown Selection
    def _on_a_point_selected(self, index):
        """Handle selection from A point history dropdown."""
        if index < 0 or index >= len(self.a_points_in_base_list):
            return
        
        # Retrieve point data
        selected_point = self.a_points_in_base_list[index]
        
        # Update UI text fields
        for i in range(3):
            self.a_vars[i].setText(f"{selected_point[i]:.2f}")

        # [新增] 检查 volume_in_base 是否存在，如果存在则自动计算并更新 A_in_Volume
        if self.volume_in_base is not None:
             # calculate_a_point_in_u_volume 使用界面上的输入框(a_vars)和 volume_in_base 进行计算
             new_a_in_vol = self.calculate_a_point_in_u_volume()
             if self.main_window and hasattr(self.main_window, 'right_panel'):
                 self.main_window.right_panel.log_message(f"System: A point selected. Updated A_in_Volume: {new_a_in_vol}")
        else:
             if self.main_window and hasattr(self.main_window, 'right_panel'):
                 self.main_window.right_panel.log_message("System: A point selected. volume_in_base not set, skipping A_in_Volume update.")

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
        
        # [修改] 统一接口切换到 TCP_E
        self._switch_tcp("TCP_E")
        
        # --- 阶段 2：延时后执行参数更新和对齐逻辑 ---
        QTimer.singleShot(200, self.reset_to_tcp_e_in_ultrasound_zero_deg) 

    def reset_to_tcp_e_in_ultrasound_zero_deg(self):
        zero_pose = self.tcp_e_in_ultrasound_zero_deg
        
        if zero_pose is None:
            QMessageBox.warning(self, "旋转失败", "未找到 0 度参考位姿。请先在 Ultrasound 标签页点击 'Rotate Left x Deg' 进行采样初始化。")
            return

        try:
            # --- 阶段 1：发送 WayPoint 指令复位到 0 度姿态 ---
            # 格式化位姿字符串 (x,y,z,Rx,Ry,Rz)
            pos_rpy_str = ",".join([f"{p:.2f}" for p in zero_pose])
            
            # 定义 6 个关节的增量为 0
            dJ_zero = ",".join(["0.00"] * 6)
            
            # 构造 WayPoint 指令
            # 参数含义：类型0(绝对运动), 位姿, 关节增量, 使用TCP坐标系, 参考Base基准, 速度50, 加速度250...
            reset_cmd = f"WayPoint,0,{pos_rpy_str},{dJ_zero},TCP,Base,50,250,0,1,0,0,0,0,ID1;"
            
            self.main_window.tcp_manager.send_command(reset_cmd)
            self.main_window.right_panel.log_message("Reset command sent. Waiting for robot to become idle...")

            # 设置标志位
            self.is_waiting_for_puncture_reset = True
            
            # 500ms 后启动定时器开始轮询状态
            QTimer.singleShot(500, lambda: self.fsm_check_timer.start(500))
            
        except Exception as e:
            print(f"reset_to_tcp_e_in_ultrasound_zero_deg error: {e}")
        

    def _continue_align_ultrasound_plane_to_aoe(self):
        # [NEW] 在这里调用更新函数，此时可以保证 TCP 是 TCP_E
        self._update_tcp_o_parameters_internally()
        
        # 获取最新的 E 点位置显示到 UI
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
        """
        处理状态机消息的统一入口
        ",33," 通常代表机械臂进入了 Idle (空闲) 状态
        """
        if ",33," in message:
            # 情况 A: 正在等待穿刺点对齐完成
            if hasattr(self, 'is_waiting_for_puncture_alignment') and self.is_waiting_for_puncture_alignment:
                self.fsm_check_timer.stop()
                self.is_waiting_for_puncture_alignment = False # 重置标志位
                self.main_window.right_panel.log_message("FSM: Puncture Alignment completed (Idle detected).")
                # 延迟执行对齐后的最终记录逻辑
                QTimer.singleShot(300, self._finalize_puncture_point_recording)
                return # 处理完毕

            # 情况 B: 正在等待穿刺复位（Reset）完成
            elif hasattr(self, 'is_waiting_for_puncture_reset') and self.is_waiting_for_puncture_reset:
                self.fsm_check_timer.stop()
                self.is_waiting_for_puncture_reset = False # 重置标志位
                self.main_window.right_panel.log_message("FSM: Puncture Reset completed (Idle detected).")
                # 延迟执行复位后的后续逻辑（例如继续对齐超声平面）
                QTimer.singleShot(300, self._continue_align_ultrasound_plane_to_aoe)
                return # 处理完毕
            
            else:
                self.main_window.right_panel.log_message("FSM: Error Occured.") 
                return # 处理完毕
            
    def _finalize_puncture_point_recording(self):
        if self.latest_tool_pose:
            self.tcp_e_in_puncture_position = list(self.latest_tool_pose)
            if self.main_window and self.main_window.right_panel:
                self.main_window.right_panel.log_message(f"System: tcp_e_in_puncture_position Recorded: {self.tcp_e_in_puncture_position}")
            
            if self.main_window and self.main_window.status_bar:
                self.main_window.status_bar.showMessage("Status: Aligned to puncture point & tcp_e_in_puncture_position recorded.")

    def _update_tcp_o_parameters_internally(self):
        """
        Performs the O-point projection calculation and updates TCP_O definition
        based on the current E-point (after alignment).
        Does NOT switch the active TCP to TCP_O.
        
        [IMPORTANT] This function assumes the robot is currently in TCP_E mode,
        so that 'self.latest_tool_pose' represents the actual E-point position.
        """
        try:
            # 1. Get current pose (Post-alignment E point)
            if not self.latest_tool_pose: return
            cur_pose = np.array(self.latest_tool_pose)
            e_pt = cur_pose[:3]
            
            # Update E vars in UI to reflect current position
            for i in range(3): self.e_vars[i].setText(f"{cur_pose[i]:.2f}")
            
            # 2. Get current O point guess
            try:
                o_pt = np.array([float(v.text()) for v in self.o_vars])
            except ValueError:
                return # Invalid O point, skip update

            # 3. Calculate projection distance (Dist from E to O along Z axis)
            rpy_rad = np.deg2rad(cur_pose[3:])
            rot = pyrot.matrix_from_euler(rpy_rad, 0, 1, 2, extrinsic=True)
            z_vec = rot[:, 2]
            
            dist = np.dot(o_pt - e_pt, z_vec)
            
            # 4. Update O point UI (Projected O)
            p_o_new = e_pt + dist * z_vec
            for i in range(3): self.o_vars[i].setText(f"{p_o_new[i]:.2f}")
            
            # 5. Update RightPanel Inputs and Configure TCP_O
            if self.main_window and hasattr(self.main_window, 'right_panel'):
                rp = self.main_window.right_panel
                
                # Update Text Fields for TCP Definition
                for le in rp.tcp_input_entries: le.setText("0.00")
                rp.tcp_input_entries[2].setText(f"{dist:.2f}")   # Z
                rp.tcp_input_entries[5].setText("157.50")        # Rz (Fixed offset)
                
                # [MODIFIED] Use the parameterized method to send ConfigTCP for "TCP_O"
                # without modifying RightPanel's internal state (current_tcp_name)
                rp.send_set_tcp_command(tcp_name="TCP_O")
                
                rp.log_message(f"System: Internal TCP_O Update -> Z:{dist:.2f}, Rz:157.50 (Active TCP unchanged)")
                
        except Exception as e:
            print(f"Error in internal TCP_O update: {e}")

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
                # [修改] 使用统一接口切换到 TCP_E
                self._switch_tcp("TCP_E")
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
        # [修改] 使用统一接口切换到 TCP_E
        self._switch_tcp("TCP_E")
        QTimer.singleShot(300, self._step_1_get_e)

    def _step_1_get_e(self):
        # 2. 获取当前 E 点位置 (直接调用自身方法)
        self.get_e_point_position()
        QTimer.singleShot(200, self._step_2_calc)

    def _step_2_calc(self):
        # 3. 计算逻辑 - 复用内部函数
        # 此时由于前一步是 start_get_suitable_tcp_sequence 已经切换了 TCP_E，所以这里是安全的
        self._update_tcp_o_parameters_internally()
        
        # 直接跳转到结束步骤，跳过原有的 _step_3_set_cur，因为内部函数已处理配置
        QTimer.singleShot(400, self._finalize_sequence)
        
    def _finalize_sequence(self):
        if self.is_fine_tuning_process:
            # [关键修改] 额外强制切换到 TCP_O
            # 这是为了覆盖可能因 SetCurTCP 指令导致的 TCP 切换副作用
            # [修改] 使用统一接口切换到 TCP_O
            self._switch_tcp("TCP_O")
            
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
            # [修改] 使用统一接口切换到 TCP_E
            self._switch_tcp("TCP_E")
            QMessageBox.information(self, "Success", "Sequence completed.")

    # --- B Point Logic ---
    def rotate_ultrasound_plane_to_b(self):
        if not self.tcp_manager.is_connected: return
        # [修改] 使用统一接口切换到 TCP_E
        self._switch_tcp("TCP_E")
        QTimer.singleShot(200, self._start_b_point_rotation_sequence)

    def _start_b_point_rotation_sequence(self):
        try:
            a = np.array([float(v.text()) for v in self.a_vars])
            o = np.array([float(v.text()) for v in self.o_vars])
            if np.all(self.b_point_in_base == 0):
                QMessageBox.warning(self, "Failed", "Invalid B point.")
                return
            b = self.b_point_in_base
            init_pose = self.get_current_tool_pose()
            
            # 计算旋转矩阵列表 deltas (这里会使用默认或你传入的 step_size_deg)
            deltas = calculate_new_rpy_for_b_point(a, o, b, init_pose[3:])
                   
            # if not deltas or (len(deltas)==1 and np.allclose(deltas[0], np.identity(4))):
            #     QMessageBox.information(self, "Info", "No rotation needed.")
            #     return
            
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
            
            # 1. 计算当前步进的 TCP_E 位置 (P_final)
            P_final = get_final_tcp_e_position_after_delta_rotation(self.initial_tcp_pose_for_b_rot[:3], delta, self.b_point_o_point)
            
            # 2. 计算当前步进的 TCP_E 姿态 (R_final)
            init_rpy_rad = np.deg2rad(self.initial_tcp_pose_for_b_rot[3:])
            R_init = pyrot.matrix_from_euler(init_rpy_rad, 0, 1, 2, extrinsic=True)
            R_final = np.dot(delta[:3,:3], R_init)
            final_rpy_deg = np.rad2deg(pyrot.euler_from_matrix(R_final, 0, 1, 2, extrinsic=True))
            
            # 构建当前步进的 T_Base_E 矩阵
            T_Base_E = np.identity(4)
            T_Base_E[:3, :3] = R_final
            T_Base_E[:3, 3] = P_final

            # 发送机器人运动指令
            pos_rpy_str = ",".join([f"{v:.2f}" for v in np.concatenate((P_final, final_rpy_deg))])
            dJ_zero = ",".join(["0.00"] * 6)
            cmd = f"WayPoint,0,{pos_rpy_str},{dJ_zero},TCP,Base,50,250,0,1,0,0,0,0,ID1;"
            if self.main_window and self.main_window.right_panel:
                self.main_window.right_panel.log_message(cmd)
            
            QTimer.singleShot(400, lambda: self.tcp_manager.send_command(cmd))
            
            step_angle = np.rad2deg(pyrot.axis_angle_from_matrix(delta[:3, :3])[3])
            if self.main_window and self.main_window.right_panel:
                self.main_window.right_panel.log_message(cmd)
                self.main_window.right_panel.log_message(f"Status: Rotating Step {self.current_b_point_step+1}/{len(self.b_point_rotation_steps)}, cumulative {step_angle:.2f} deg")
        else:
            # =================================================================
            # 旋转完成
            # =================================================================
            
            # 1. 预先计算理论上的最终 T_Base_E (用于发送给 Navigation，避免实时读取延迟)
            last_delta = self.b_point_rotation_steps[-1]
            
            # 计算最终位置 P_final
            P_final_calc = get_final_tcp_e_position_after_delta_rotation(self.initial_tcp_pose_for_b_rot[:3], last_delta, self.b_point_o_point)
            
            # 计算最终旋转 R_final
            init_rpy_rad = np.deg2rad(self.initial_tcp_pose_for_b_rot[3:])
            R_init = pyrot.matrix_from_euler(init_rpy_rad, 0, 1, 2, extrinsic=True)
            R_final_calc = np.dot(last_delta[:3,:3], R_init)
            
            # 构建 T_Base_E 矩阵
            T_Base_E_Final_Calc = np.identity(4)
            T_Base_E_Final_Calc[:3, :3] = R_final_calc
            T_Base_E_Final_Calc[:3, 3] = P_final_calc

            # 清空步骤
            self.b_point_rotation_steps = []

            # 2. 执行原有计算：B 点，A 点在 TCP_P 坐标系下的位置
            QTimer.singleShot(300, lambda: self._calculate_b_point_in_tcp_p())
            self._calculate_a_point_in_tcp_p()

            # 3. [修改逻辑] 计算并发送 Volume 系下的 RCM 和 TCP_U 到 Navigation
            # 传入计算好的 T_Base_E 矩阵
            self._calculate_and_send_rcm_and_tcp_u_to_nav(t_base_e_override=T_Base_E_Final_Calc)
            
            QMessageBox.information(self, "Done", "Rotation to B point completed.\nNavigation data sent.")
            
    def _calculate_and_send_rcm_and_tcp_u_to_nav(self, t_base_e_override=None):
        """
        [新增] 旋转结束后：
        1. 计算 delta_J1
        2. 计算 RCM (Volume系)
        3. 计算 TCP_U (Volume系)
        4. 获取 B点索引
        5. 发送组合数据: index, x, y, z, rx, ry, rz, rcm_x, rcm_y, rcm_z
        
        Args:
            t_base_e_override (np.ndarray, optional): 强制使用的 T_Base_E 矩阵 (4x4)。
                                                      如果不传，则使用 self.latest_tool_pose 转换。
        """
        # 0. 基础数据检查
        if not self.a_point_in_tcp_p:
            print("Error: A point in TCP_P missing.")
            return
        
        # 如果没有传入 override 且 latest_tool_pose 为空，则报错
        if t_base_e_override is None and self.latest_tool_pose is None:
            print("Error: Tool pose missing.")
            return
            
        if self.tcp_p_definition_pose is None or self.tcp_u_definition_pose is None or self.volume_in_base is None:
            print("Error: TCP definitions or volume_in_base missing.")
            return

        try:
            # 1. 计算 Delta J1
            # a_point_in_tcp_p 是 [x, y, z] 列表
            a_z = self.a_point_in_tcp_p[2]
            # rcm0 是 [x, y, z] numpy 数组
            rcm0_z = self.robot_kinematics.get_rcm_point([0,0,0,0])[2]
            delta_j0 = a_z - rcm0_z

            # 2. 计算 RCM 在 TCP_P 坐标系下的位置 (rcm_in_p)
            rcm_in_p = self.robot_kinematics.get_rcm_point([delta_j0, 0, 0, 0])
            rcm_p_homo = np.append(rcm_in_p, 1.0) # [x, y, z, 1]

            # 准备变换矩阵
            # [关键修改] 使用传入的 t_base_e_override 或当前的 self.latest_tool_pose
            if t_base_e_override is not None:
                T_Base_E = t_base_e_override
                if self.main_window and hasattr(self.main_window, 'right_panel'):
                     self.main_window.right_panel.log_message("System: Using calculated T_Base_E for Nav data.")
            else:
                T_Base_E = self.pose_to_matrix(self.latest_tool_pose)       # 当前机器人位姿

            T_E_P = self.pose_to_matrix(self.tcp_p_definition_pose)     # TCP_P 定义
            T_Base_P = np.dot(T_Base_E, T_E_P)                           # P -> Base
            
            T_Base_Vol = self.pose_to_matrix(self.volume_in_base)         # Volume -> Base
            T_Vol_Base = np.linalg.inv(T_Base_Vol)                       # Base -> Volume

            # 3. 将 RCM 转换到 Volume 坐标系 (P_rcm_vol = T_Vol_Base * T_Base_P * P_rcm_p)
            P_rcm_base = np.dot(T_Base_P, rcm_p_homo)
            P_rcm_vol = np.dot(T_Vol_Base, P_rcm_base)
            rcm_vol_xyz = P_rcm_vol[:3]

            # 4. 计算 TCP_U 在 Volume 坐标系下的位姿
            T_E_U = self.pose_to_matrix(self.tcp_u_definition_pose)
            T_Base_U = np.dot(T_Base_E, T_E_U)
            T_Vol_U = np.dot(T_Vol_Base, T_Base_U)
            
            tcp_u_vol_xyz = T_Vol_U[:3, 3]
            tcp_u_vol_rpy = np.rad2deg(pyrot.euler_from_matrix(T_Vol_U[:3, :3], 0, 1, 2, extrinsic=True))

            # 5. 获取 B 点索引 (从下拉框文本解析 "B1, ...")
            b_text = self.b_point_dropdown.currentText()
            b_index = 0
            if b_text.startswith("B"):
                try:
                    # 提取 "B" 和 "," 之间的数字
                    b_index = int(b_text.split(',')[0].replace("B", ""))
                except:
                    print("Warning: Could not parse B point index.")

            # 6. 构造发送消息
            # 格式: b_index, tcp_u_x, tcp_u_y, tcp_u_z, tcp_u_rx, tcp_u_ry, tcp_u_rz, rcm_x, rcm_y, rcm_z
            msg = f"{b_index},{tcp_u_vol_xyz[0]:.3f},{tcp_u_vol_xyz[1]:.3f},{tcp_u_vol_xyz[2]:.3f}," \
                  f"{tcp_u_vol_rpy[0]:.3f},{tcp_u_vol_rpy[1]:.3f},{tcp_u_vol_rpy[2]:.3f}," \
                  f"{rcm_vol_xyz[0]:.3f},{rcm_vol_xyz[1]:.3f},{rcm_vol_xyz[2]:.3f}"

            if self.main_window and hasattr(self.main_window, 'navigation_tab'):
                nav_tab = self.main_window.navigation_tab
    
                # [在这里添加检查]
                if not nav_tab.nav_manager.is_connected:
                    # 只在 RightPanel 的日志里提示一下，不触发任何 Error 信号
                    if hasattr(self.main_window, 'right_panel'):
                        self.main_window.right_panel.log_message("System: Navigation not connected, data sync skipped.")
                    return # 直接跳出，不再调用 send_command

                # 只有连接了才会执行这里
                nav_tab.nav_manager.send_command(msg)
                nav_tab.log_message(f"Sent Nav Data (Post-Rotate): {msg}")

        except Exception as e:
            print(f"Error calculating/sending RCM & TCP_U to Nav: {e}")
            
    def _calculate_b_point_in_tcp_p(self):
        """计算 B 点在 TCP_P 坐标系下的位置。"""
        # 1. 检查必要数据
        if self.tcp_p_definition_pose is None:
            print("Error: TCP_P definition is missing.")
            return
        if self.latest_tool_pose is None:
            print("Error: Robot tool pose is missing.")
            return
        if self.b_point_in_base is None:
            print("Error: B point (Base) is not selected.")
            return

        try:
            # 2. 获取当前机器臂(TCP_E)在 Base 下的位姿矩阵 T_Base_E
            # 此时机器臂已经运动到了目标位置
            T_Base_E = self.pose_to_matrix(self.latest_tool_pose)

            # 3. 获取 TCP_P 在 TCP_E 下的位姿矩阵 T_E_P
            # 根据需求：tcp_p_definition_pose 记录的是相对于工具端(TCP_E)的姿态
            T_E_P = self.pose_to_matrix(self.tcp_p_definition_pose)

            # 4. 计算 TCP_P 在 Base 下的位姿矩阵 T_Base_P
            # T_Base_P = T_Base_E * T_E_P
            T_Base_P = np.dot(T_Base_E, T_E_P)

            # 5. 获取 B 点在 Base 下的坐标 P_B_Base (补齐为齐次坐标)
            # self.b_point_in_base 是 [x, y, z]
            b_point_base = np.append(self.b_point_in_base, 1.0)

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
            T_Base_E = self.pose_to_matrix(self.latest_tool_pose)

            # 3. 获取 TCP_P 在 TCP_E 下的位姿矩阵 T_E_P
            T_E_P = self.pose_to_matrix(self.tcp_p_definition_pose)

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

    def compute_and_store_volume_in_base(self):
        if self.tcp_e_in_ultrasound_zero_deg is None or self.tcp_u_definition_pose is None: return
        try:
            T_Base_E = self.pose_to_matrix(self.tcp_e_in_ultrasound_zero_deg)
            T_E_U = self.pose_to_matrix(self.tcp_u_definition_pose)
            T_Base_U = np.dot(T_Base_E, T_E_U)
            trans = T_Base_U[:3, 3]
            rpy_deg = np.rad2deg(pyrot.euler_from_matrix(T_Base_U[:3, :3], 0, 1, 2, extrinsic=True))
            self.volume_in_base = np.concatenate((trans, rpy_deg)).tolist()
            if self.main_window and self.main_window.right_panel:
                self.main_window.right_panel.log_message(f"System: volume_in_base: {self.volume_in_base}")
        except Exception as e: print(e)
        
    def calculate_a_point_in_u_volume(self):
        """
        [新增] 计算 A 点在 U_Volume 坐标系下的位置。
        前提：volume_in_base 已被计算 (在点击左转x度时会触发计算)。
        
        Returns:
            list: [x, y, z] 坐标列表，如果计算失败则返回 None。
        """
        if self.volume_in_base is None:
            print("Error: volume_in_base is not computed.")
            return None
            
        # 1. 获取界面上的 A 点 (Base 系)
        try:
            a_vals = [float(v.text()) for v in self.a_vars]
            # 转换为齐次坐标 [x, y, z, 1.0]
            a_point_base = np.array(a_vals + [1.0])
        except ValueError:
            print("Error: Invalid A point inputs in UI.")
            return None

        try:
            # 2. 获取 Volume 在 Base 下的变换矩阵 T_Base_Volume
            T_Base_Volume = self.pose_to_matrix(self.volume_in_base)
            
            # 3. 计算 Base 在 Volume 下的变换矩阵 (求逆)
            T_Volume_Base = np.linalg.inv(T_Base_Volume)
            
            # 4. 坐标变换: P_A_Volume = T_Volume_Base * P_A_Base
            a_point_vol_homo = np.dot(T_Volume_Base, a_point_base)
            
            # 5. 将计算结果存储到 self.a_point_in_volume
            self.a_point_in_volume = a_point_vol_homo[:3].tolist()
            
            print(f"Calculated A point in Volume: {self.a_point_in_volume}")
            return self.a_point_in_volume
            
        except Exception as e:
            print(f"Error calculating A in Volume: {e}")
            return None
        
    def calculate_all_a_points_in_volume(self):
        """
        [新增] 计算历史记录中所有的 A 点在 U_Volume 坐标系下的位置。
        Returns:
            list: [[x, y, z], [x, y, z], ...] 坐标列表
        """
        if self.volume_in_base is None:
            print("Error: volume_in_base is not computed.")
            return []
            
        if not self.a_points_in_base_list:
            print("Warning: No A points in history list.")
            return []

        converted_points = []
        try:
            # 1. 获取 Volume 在 Base 下的变换矩阵 T_Base_Volume
            T_Base_Volume = self.pose_to_matrix(self.volume_in_base)
            
            # 2. 计算 Base 在 Volume 下的变换矩阵 (求逆)
            T_Volume_Base = np.linalg.inv(T_Base_Volume)
            
            # 3. 遍历列表进行转换
            for pt_base in self.a_points_in_base_list:
                # 转换为齐次坐标 [x, y, z, 1.0]
                pt_homo = np.array(pt_base + [1.0])
                # 坐标变换: P_Volume = T_Volume_Base * P_Base
                pt_vol_homo = np.dot(T_Volume_Base, pt_homo)
                # 存入结果
                converted_points.append(pt_vol_homo[:3].tolist())
            
            return converted_points
            
        except Exception as e:
            print(f"Error calculating all A points in Volume: {e}")
            return []
    
    def get_current_a_in_volume(self):
        """
        获取当前下拉框选中的 A 点，并将其从 Base 坐标系转换到 Volume 坐标系。
        """
        if not self.a_points_in_base_list or self.volume_in_base is None:
            return [0.0, 0.0, 0.0]
        
        # 获取当前 A 点下拉框选中的索引
        current_idx = self.a_point_dropdown.currentIndex()
        if current_idx < 0 or current_idx >= len(self.a_points_in_base_list):
            return [0.0, 0.0, 0.0]
            
        a_base = np.array(self.a_points_in_base_list[current_idx])
        return self.transform_point_base_to_volume(a_base)

    def get_current_b_in_volume(self):
        """
        获取当前界面显示的 B 点（Base系），并转换到 Volume 坐标系。
        """
        if self.volume_in_base is None:
            return [0.0, 0.0, 0.0]
            
        try:
            b_base = np.array([float(v.text()) for v in self.b_vars_in_base])
            return self.transform_point_base_to_volume(b_base)
        except ValueError:
            return [0.0, 0.0, 0.0]

    def transform_point_base_to_volume(self, point_base):
        """
        辅助方法：将一个点从 Base 坐标系转换到 Volume 坐标系。
        公式: P_vol = inv(T_base_vol) * P_base
        """
        if self.volume_in_base is None:
            return point_base
            
        # T_base_vol
        T_base_vol = self.pose_to_matrix(self.volume_in_base)
        # T_vol_base = inv(T_base_vol)
        T_vol_base = np.linalg.inv(T_base_vol)
        
        p_homo = np.append(point_base, 1.0)
        p_vol = np.dot(T_vol_base, p_homo)
        return p_vol[:3].tolist()

    def transform_point_p_to_volume(self, point_p):
        """
        辅助方法：将 TCP_P 坐标系下的点转换到 Volume 坐标系。
        用于计算 RCM in Volume。
        公式: P_vol = T_vol_base * T_base_e * T_e_p * P_p
        """
        if any(x is None for x in [self.volume_in_base, self.latest_tool_pose, self.tcp_p_definition_pose]):
            return point_p

        T_base_vol = self.pose_to_matrix(self.volume_in_base)
        T_vol_base = np.linalg.inv(T_base_vol)
        
        T_base_e = self.pose_to_matrix(self.latest_tool_pose)
        T_e_p = self.pose_to_matrix(self.tcp_p_definition_pose)
        
        # P -> E -> Base -> Volume
        T_vol_p = T_vol_base @ T_base_e @ T_e_p
        
        p_homo = np.append(point_p, 1.0)
        p_vol = np.dot(T_vol_p, p_homo)
        return p_vol[:3].tolist()

    def get_current_tcp_u_in_volume(self):
        """
        计算当前的 TCP_U 在 Volume 坐标系下的姿态 (x, y, z, rx, ry, rz)。
        TCP_U 的相对偏移由 self.tcp_u_definition_pose 提供。
        """
        if any(x is None for x in [self.volume_in_base, self.latest_tool_pose, self.tcp_u_definition_pose]):
            return [0.0] * 6

        # 1. 坐标系矩阵获取
        T_base_vol = self.pose_to_matrix(self.volume_in_base)
        T_vol_base = np.linalg.inv(T_base_vol)
        
        # 2. 计算 T_vol_u = T_vol_base * T_base_e * T_e_u
        T_base_e = self.pose_to_matrix(self.latest_tool_pose)
        T_e_u = self.pose_to_matrix(self.tcp_u_definition_pose)
        
        # U -> E -> Base -> Volume
        T_vol_u = T_vol_base @ T_base_e @ T_e_u
        
        # 3. 提取位置和欧拉角
        pos = T_vol_u[:3, 3]
        rot_mat = T_vol_u[:3, :3]
        # 这里的顺序需与您系统一致，通常为 rx, ry, rz (ZYX或XYZ)
        rpy = np.rad2deg(pyrot.euler_from_matrix(rot_mat, 0, 1, 2, extrinsic=True))
        
        return [pos[0], pos[1], pos[2], rpy[0], rpy[1], rpy[2]]

    def pose_to_matrix(self, pose):
        """
        将位姿列表 [x, y, z, rx, ry, rz] 转换为 4x4 齐次变换矩阵。
        """
        x, y, z, rx, ry, rz = pose
        T = np.identity(4)
        # 使用旋转顺序 0,1,2 (X, Y, Z) 对应 rx, ry, rz
        T[:3, :3] = pyrot.matrix_from_euler(np.deg2rad([rx, ry, rz]), 0, 1, 2, extrinsic=True)
        T[:3, 3] = [x, y, z]
        return T

    def read_a_points_volume_and_update_base(self, trigger_b_read=False):
        """
        从项目根目录读取 biopsy_entry_replan.txt，转换坐标系并更新 A 点下拉框。
        """

        file_path = "biopsy_entry_replan.txt"
        if not os.path.exists(file_path):
            QMessageBox.warning(self, "Error", f"File not found: {file_path}")
            return

        # 检查坐标转换矩阵 T_Base_Vol 是否就绪
        if self.volume_in_base is None:
            QMessageBox.warning(self, "Error", "Volume to Base transformation (T_Base_Vol) is not defined.")
            return

        try:
            # 位姿转 4x4 矩阵辅助函数
            def to_matrix(pose):
                x, y, z, rx, ry, rz = [float(val) for val in pose]
                T = np.identity(4)
                R = pyrot.matrix_from_euler(np.deg2rad([rx, ry, rz]), 0, 1, 2, extrinsic=True)
                T[:3, :3] = R
                T[:3, 3] = [x, y, z]
                return T

            T_Base_Vol = to_matrix(self.volume_in_base)
            
            new_a_base_list = []
            with open(file_path, 'r') as f:
                for line in f:
                    parts = line.strip().split()
                    if len(parts) >= 3:
                        # 转换 Volume 系坐标到 Base 系
                        p_vol = np.array([float(parts[0]), float(parts[1]), float(parts[2]), 1.0])
                        p_base = T_Base_Vol @ p_vol
                        new_a_base_list.append(p_base[:3].tolist())

            if not new_a_base_list:
                return

            # 1. 更新内存数据
            self.a_points_in_base_list = new_a_base_list
            
            # 2. 刷新 A 点下拉框，保持指定格式
            # 注意：此处使用 self.a_point_dropdown 对应你要求的变量名
            self.a_point_dropdown.blockSignals(True)
            self.a_point_dropdown.clear()
            for i, pt in enumerate(self.a_points_in_base_list):
                coord_str = f"({pt[0]:.2f}, {pt[1]:.2f}, {pt[2]:.2f})"
                self.a_point_dropdown.addItem(f"A{i+1}: {coord_str}")
            self.a_point_dropdown.blockSignals(False)
            
            # 3. 选中第一项并更新 a_vars
            self.a_point_dropdown.setCurrentIndex(0)
            if hasattr(self, 'on_a_point_selected'):
                self.on_a_point_selected(0)

            # 4. 如果是 Read B 触发的，继续执行后续逻辑
            if trigger_b_read:
                # 执行你原有的 B 点读取逻辑
                self.read_b_points_volume_from_file_according_to_selected_a()
            else:
                QMessageBox.information(self, "Success", f"Updated {len(new_a_base_list)} A points (Base frame).")

        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to process replan file: {e}")

    def read_b_points_volume_from_file_according_to_selected_a(self):
        """
        处理 biopsy_target_replan.txt 并生成包含 A点信息和 Volume姿态的详细记录文件。
        """
        # 1. 前置检查
        if self.volume_in_base is None:
            QMessageBox.warning(self, "Warning", "请先计算 volume_in_base（点击左转x度）。")
            return
        
        current_a_text = self.a_point_dropdown.currentText()
        if not current_a_text.startswith("A"):
            QMessageBox.warning(self, "Warning", "请先在下拉框选择一个 A 点。")
            return
        
        try:
            # 获取当前的 A 点编号
            current_a_idx = int(current_a_text.split(':')[0].replace("A", ""))
        except:
            current_a_idx = 1

        ORIGIN_FILE = "biopsy_target_replan.txt"
        if not os.path.exists(ORIGIN_FILE):
            QMessageBox.critical(self, "Error", f"File {ORIGIN_FILE} not found.")
            return

        # [可选]：为了防止残留文件干扰，拆分前可以先清理旧的子文件
        for old_file in glob.glob("biopsy_target_replan_for_a*.txt"):
            try: os.remove(old_file)
            except Exception as e:
                print(f"Error read_b_points_volume_from_file_according_to_selected_a: {e}")
        # 2. 拆分逻辑 (保持原样)
        split_data = {} 
        try:
            with open(ORIGIN_FILE, 'r') as f:
                for line_idx, line in enumerate(f):
                    line = line.strip()
                    if not line or line.startswith("["): continue 
                    parts = line.split()
                    if len(parts) < 4: continue
                    b_id = line_idx + 1
                    bx, by, bz = parts[0], parts[1], parts[2]
                    a_ref = int(parts[3])
                    if a_ref not in split_data: split_data[a_ref] = []
                    split_data[a_ref].append(f"{b_id} {bx} {by} {bz}")
            
            for a_key, lines in split_data.items():
                with open(f"biopsy_target_replan_for_a{a_key}.txt", 'w') as tf:
                    tf.write("\n".join(lines))
        except Exception as e:
            QMessageBox.critical(self, "File Process Error", str(e))
            return

        # 3. 加载当前 A 对应的 B 点并计算坐标
        target_file = f"biopsy_target_replan_for_a{current_a_idx}.txt"
        if not os.path.exists(target_file):
            QMessageBox.information(self, "Info", f"No B points associated with A{current_a_idx}.")
            return

        b_point_data_list = []
        try:
            with open(target_file, 'r') as f:
                for line in f:
                    parts = line.split()
                    if len(parts) != 4: continue
                    b_idx = int(parts[0])
                    p_b_vol = [float(parts[1]), float(parts[2]), float(parts[3])]
                    p_base_pose, angle = self._calculate_b_point_data_from_volume(p_b_vol)
                    b_point_data_list.append((p_b_vol, p_base_pose, angle, b_idx))
        except Exception as e:
            QMessageBox.critical(self, "读取失败", str(e))
            return

        # ================= [新增：增强版记录文件生成逻辑] =================
        try:
            # 获取当前 A 点的坐标信息
            a_base = self.a_points_in_base_list[current_a_idx - 1] # A点在 Base 系
            a_vol = self.transform_point_base_to_volume(np.array(a_base)) # A点在 Volume 系
            
            record_filename = f"B points in Volume for A{current_a_idx}.txt"
            with open(record_filename, 'w', encoding='utf-8') as rf:
                # 写入顶部摘要信息
                rf.write(f"=== Biopsy Target Report for A{current_a_idx} ===\n")
                rf.write(f"Volume in Base [x,y,z,rx,ry,rz]: {['%.3f' % x for x in self.volume_in_base]}\n")
                rf.write(f"A Point (Base)   [x,y,z]: {['%.3f' % x for x in a_base]}\n")
                rf.write(f"A Point (Volume) [x,y,z]: {['%.3f' % x for x in a_vol]}\n")
                rf.write("-" * 130 + "\n")
                
                # 写入表头，增加 A 点相关列
                header = (f"{'B ID':<6} | {'B point (Vol)':<30} | {'B point (Base)':<30} | "
                          f"{'A point (Vol)':<30} | {'A point (Base)':<30} | {'OA Angle':<10}\n")
                rf.write(header)
                rf.write("-" * 130 + "\n")
                
                # 格式化字符串准备
                fmt_coord = "(%.3f, %.3f, %.3f)"
                
                for p_vol, p_base, angle, b_idx in b_point_data_list:
                    line = (f"B{b_idx:<5} | "
                            f"{fmt_coord % tuple(p_vol):<30} | "
                            f"{fmt_coord % tuple(p_base[:3]):<30} | "
                            f"{fmt_coord % tuple(a_vol):<30} | "
                            f"{fmt_coord % tuple(a_base):<30} | "
                            f"{angle:.3f}\n")
                    rf.write(line)
            
            # 步骤 2: [新增] 拷贝文件到 session_folder
            # 由于 session_folder 定义在 ultrasound_tab 中，通过 main_window 访问
            if self.main_window and hasattr(self.main_window, 'ultrasound_tab'):
                ut = self.main_window.ultrasound_tab
                # 检查 ultrasound_tab 中是否有定义好的 session_folder
                if hasattr(ut, 'session_folder') and ut.session_folder:
                    if os.path.exists(ut.session_folder):
                        dest_path = os.path.join(ut.session_folder, record_filename)
                        shutil.copy2(record_filename, dest_path) # 使用 copy2 保留元数据
                        
                        # 在右侧面板打印同步成功的日志
                        if hasattr(self.main_window, 'right_panel'):
                            self.main_window.right_panel.log_message(f"System: File synced to session folder: {record_filename}")
            
            if self.main_window and hasattr(self.main_window, 'right_panel'):
                self.main_window.right_panel.log_message(f"System: Detailed record saved to {record_filename}")
                
        except Exception as e:
            print(f"Error saving/copying B point record: {e}")
        # ===============================================================

        # 4. 显示对话框
        dialog = BPointSelectionDialog(b_point_data_list, current_a_idx, self)
        dialog.b_points_selected.connect(self._handle_b_points_list_selection)
        dialog.exec_()

    def _calculate_b_point_data_from_volume(self, p_b_vol):
        """
        核心逻辑变动：从固定的 Volume 坐标系转换到 Base 坐标系。
        """
        if self.volume_in_base is None or self.tcp_e_in_puncture_position is None:
            raise ValueError("缺少 volume_in_base 或穿刺位姿数据")

        # 1. 坐标转换: P_base = T_base_vol * P_vol
        T_base_vol = self.pose_to_matrix(self.volume_in_base)
        p_b_vol_homo = np.array(p_b_vol + [1.0])
        p_b_base = np.dot(T_base_vol, p_b_vol_homo)[:3]
        
        # 组装到位姿列表 (x, y, z, rx, ry, rz)，旋转角设为 0 (对齐逻辑只用 xyz)
        p_b_base_pose = np.concatenate((p_b_base, [0.0, 0.0, 0.0]))

        # 2. 计算旋转角度 (对齐超声平面到 OA-OB 平面)
        a = np.array([float(v.text()) for v in self.a_vars])
        o = np.array([float(v.text()) for v in self.o_vars])
        
        # 复用原有的角度计算函数，此时 B 点已在 Base 系
        angle = self._calculate_oa_rotation_angle(
            a, o, p_b_base, 
            np.array(self.tcp_e_in_puncture_position)[3:], 
            [0]*6 # 旧的 p_b_u 参数在新逻辑中不再需要，传空
        )
        
        return p_b_base_pose, angle

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
        self.b_point_in_base = p_base
        for i in range(3): self.b_vars_in_base[i].setText(f"{p_base[i]:.2f}")

    def _get_ui_values(self, vars_list):
        try: return [float(v.text()) for v in vars_list]
        except: return None
        
    def save_data(self):
        try:
            # 定义一个安全转换函数
            def safe_tolist(obj):
                # 如果对象有 tolist 方法（是 numpy 数组），则调用；否则直接返回对象本身（已是列表）
                return obj.tolist() if hasattr(obj, 'tolist') else obj
            data = {
                # Save a,o,e Points in base History
                'a_point': self._get_ui_values(self.a_vars),
                'o_point': self._get_ui_values(self.o_vars),
                'e_point': self._get_ui_values(self.e_vars),
                # Save b Points in base
                'b_point_in_base': self._get_ui_values(self.b_vars_in_base),
                # Save b Points in volume & base History (即下拉框信息)
                'calculated_b_points': [(safe_tolist(d[0]), safe_tolist(d[1]), d[2], d[3]) for d in self.calculated_b_points],
                
                # Save A Points in base History
                'stored_a_points_in_base': self.a_points_in_base_list,
                # 保存 A 点和 B 点下拉列表的当前索引
                "index_a": self.a_point_dropdown.currentIndex(), 
                "index_b": self.b_point_dropdown.currentIndex(),

                'tcp_e_in_ultrasound_zero_deg': self.tcp_e_in_ultrasound_zero_deg,
                'tcp_e_in_puncture_position': self.tcp_e_in_puncture_position,
                'volume_in_base': self.volume_in_base,
                'b_point_in_tcp_p': self.b_point_in_tcp_p,
                'a_point_in_tcp_p': self.a_point_in_tcp_p,
                'a_point_in_volume': self.a_point_in_volume
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
            
            # 1. 恢复界面上的 A, O, E 点数值 (默认值)
            for i in range(3):
                self.a_vars[i].setText(f"{data.get('a_point',[0]*3)[i]:.2f}")
                self.o_vars[i].setText(f"{data.get('o_point',[0]*3)[i]:.2f}")
                self.e_vars[i].setText(f"{data.get('e_point',[0]*3)[i]:.2f}")
            
            # 仅恢复 Base 系下的 B 点界面数值
            b_base_vals = data.get('b_point_in_base', [0]*3)
            for i in range(3):
                self.b_vars_in_base[i].setText(f"{b_base_vals[i]:.2f}")
            
            # 2. 恢复计算过的 B 点列表及下拉框
            self.calculated_b_points = []
            for p_volume, p_base, ang, idx in data.get('calculated_b_points', []):
                self.calculated_b_points.append((np.array(p_volume), np.array(p_base), float(ang), int(idx)))
            
            self.b_point_dropdown.blockSignals(True)
            self.b_point_dropdown.clear()
            for d in self.calculated_b_points:
                self.b_point_dropdown.addItem(f"B{d[3]}, {d[2]:.2f}deg")
            self.b_point_dropdown.blockSignals(False)
                
            saved_index_b = data.get("index_b", -1)
            if 0 <= saved_index_b < self.b_point_dropdown.count():
                self.b_point_dropdown.setCurrentIndex(saved_index_b)
                
            # --- [核心修改：恢复 A 点列表历史] ---
            self.a_points_in_base_list = data.get('stored_a_points_in_base', [])
            
            # 暂时屏蔽信号，防止在填充下拉框时触发 currentIndexChanged 导致的冲突
            self.a_point_dropdown.blockSignals(True)
            self.a_point_dropdown.clear()
            for i, pt in enumerate(self.a_points_in_base_list):
                coord_str = f"({pt[0]:.2f}, {pt[1]:.2f}, {pt[2]:.2f})"
                self.a_point_dropdown.addItem(f"A{i+1}: {coord_str}")
            
            self.a_point_dropdown.blockSignals(False)
            
            saved_index_a = data.get("index_a", -1)
            if 0 <= saved_index_a < self.a_point_dropdown.count():
                self.a_point_dropdown.setCurrentIndex(saved_index_a)

            # ------------------------------------

            # 3. 恢复计算结果相关的变量
            self.tcp_e_in_ultrasound_zero_deg = data.get('tcp_e_in_ultrasound_zero_deg')
            self.tcp_e_in_puncture_position = data.get('tcp_e_in_puncture_position')
            self.volume_in_base = data.get('volume_in_base')
            # a_point_in_tcp_p 和 b_point_in_tcp_p 在超声平面穿过 B点后计算得来
            self.b_point_in_tcp_p = data.get('b_point_in_tcp_p')
            self.a_point_in_tcp_p = data.get('a_point_in_tcp_p')
            # 无需恢复计算过的 b_point_in_volume, 因为保存在了 biopsy_target_replan.txt 中
            # a_point_in_volume 是在 a 点下拉列表选中后得来（依赖于 self.volume_in_base 的存在）
            self.a_point_in_volume = data.get('a_point_in_volume')
            
            QMessageBox.information(self, "Success", "Data loaded successfully.")

        except Exception as e: 
            QMessageBox.critical(self, "Error", f"Failed to load data: {str(e)}")
