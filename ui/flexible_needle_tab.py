import numpy as np
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, 
    QLabel, QLineEdit, QPushButton, QMessageBox, QGroupBox, 
    QFrame, QSizePolicy
)
from PyQt5.QtCore import Qt, pyqtSignal
from ui.beckhoff_tab import BeckhoffManager, BeckhoffTab

class FlexibleNeedleTab(BeckhoffTab):
    """
    专用于 Flexible needle steering 的 Tab 页。
    流程：Start -> Trocar -> Adjust -> Needle Phase 1/2/3 -> Retract -> Trocar Out
    回路：Needle Retraction -> Loop Back -> Adjust Needle Dir (First)
    """
    
    # 定义绘图单元格的固定宽度
    CELL_WIDTH = 50 
    LINE_WIDTH = 2
    
    def __init__(self, manager: BeckhoffManager, robot_kinematics, parent=None):
        super().__init__(manager, robot_kinematics, parent)
        
    def init_ui(self):
        main_layout = QVBoxLayout(self)

        # -----------------------------------------------------------------
        # Need Vector Inputs
        # -----------------------------------------------------------------
        for i in range(3):
            self.vector_inputs[i] = QLineEdit("0.00")

        # -----------------------------------------------------------------
        # 1. Position Control
        # -----------------------------------------------------------------
        result_group = QGroupBox("Position Control (J0 - J3)")
        result_content_layout = QHBoxLayout()
        result_layout = QGridLayout()
        
        self.result_labels["J0"] = QLineEdit("0.00")
        self.result_labels["J1"] = QLineEdit("0.00")
        self.result_labels["J2"] = QLineEdit("0.00")
        self.result_labels["J3"] = QLineEdit("0.00")
        self.motion_time_input = QLineEdit("5000") 

        for idx, axis in enumerate(["J0", "J1", "J2", "J3"]):
            unit = "(mm)" if idx < 2 else "(Deg)"
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

        self.apply_inc_btn = QPushButton("Apply Increment (All)")
        result_layout.addWidget(self.apply_inc_btn, 4, 0, 1, 2) 

        self.reset_all_btn = QPushButton("Reset All (J0-J3)")
        self.reset_all_btn.setEnabled(False) 
        result_layout.addWidget(self.reset_all_btn, 4, 3, 1, 2)
        
        result_content_layout.addLayout(result_layout)
        result_content_layout.addStretch(1) 
        result_group.setLayout(result_content_layout)
        main_layout.addWidget(result_group)
        
        # -----------------------------------------------------------------
        # 2. Biopsy Interface (Flexible Steering Flow)
        # -----------------------------------------------------------------
        biopsy_group = QGroupBox("Biopsy Interface (Flexible Needle Steering)")
        biopsy_layout = QGridLayout(biopsy_group)
        biopsy_layout.setSpacing(0) 
        biopsy_layout.setContentsMargins(20, 20, 20, 20)

        # 创建按钮
        self.flow_trocar_in_p1_btn = QPushButton("Trocar Insertion Phase 1")
        self.flow_trocar_in_p2_btn = QPushButton("Trocar Insertion Phase 2")
        self.flow_trocar_out_btn = QPushButton("Trocar Retraction")

        self.flow_calc_1_btn = QPushButton("Adjust Needle Dir")
        self.flow_needle_in_p1_btn = QPushButton("Needle Insertion Phase 1")
        
        self.flow_calc_2_btn = QPushButton("Adjust Needle Dir")
        self.flow_needle_in_p2_btn = QPushButton("Needle Insertion Phase 2")
        
        self.flow_calc_3_btn = QPushButton("Adjust Needle Dir")
        self.flow_needle_in_p3_btn = QPushButton("Needle Insertion Phase 3")
        
        self.flow_needle_out_btn = QPushButton("Needle Retraction")

        BTN_WIDTH = 160
        BTN_HEIGHT = 35
        for btn in [self.flow_trocar_in_p1_btn, self.flow_trocar_in_p2_btn, self.flow_trocar_out_btn,
                    self.flow_calc_1_btn, self.flow_needle_in_p1_btn,
                    self.flow_calc_2_btn, self.flow_needle_in_p2_btn,
                    self.flow_calc_3_btn, self.flow_needle_in_p3_btn,
                    self.flow_needle_out_btn]:
            btn.setFixedWidth(BTN_WIDTH)
            btn.setFixedHeight(BTN_HEIGHT)

        def make_node_label(text):
            l = QLabel(text)
            l.setAlignment(Qt.AlignCenter)
            l.setStyleSheet("border: 2px solid #555; border-radius: 15px; padding: 5px; font-weight: bold; background-color: #f0f0f0;")
            l.setFixedSize(100, 35)
            return l

        self.flow_start_lbl = make_node_label("Start")
        self.flow_end_lbl = make_node_label("End")

        # -------------------------------------------------------------
        # 布局逻辑
        # -------------------------------------------------------------
        
        # [关键] 设置第0列最小宽度为固定值，确保所有垂直线组件宽度一致
        biopsy_layout.setColumnMinimumWidth(0, self.CELL_WIDTH) 
        biopsy_layout.setColumnMinimumWidth(2, 40) 
        biopsy_layout.setColumnMinimumWidth(4, 40)
        
        # 强制包含线条的行的高度，防止压缩
        for r in [6, 8, 10]: 
            biopsy_layout.setRowMinimumHeight(r, 30)

        # Row 0-3: Start & Trocar
        biopsy_layout.addWidget(self.flow_start_lbl, 0, 1, alignment=Qt.AlignCenter)
        biopsy_layout.addWidget(self._create_v_arrow_widget(), 1, 1, alignment=Qt.AlignCenter)

        trocar_container = QWidget()
        trocar_layout = QVBoxLayout(trocar_container)
        trocar_layout.setContentsMargins(0, 0, 0, 0)
        trocar_layout.setSpacing(5) 
        trocar_layout.addWidget(self.flow_trocar_in_p1_btn, alignment=Qt.AlignCenter)
        trocar_layout.addWidget(self._create_v_arrow_widget(), alignment=Qt.AlignCenter)
        trocar_layout.addWidget(self.flow_trocar_in_p2_btn, alignment=Qt.AlignCenter)
        biopsy_layout.addWidget(trocar_container, 2, 1, 2, 1, alignment=Qt.AlignCenter) 
        
        biopsy_layout.addWidget(self._create_v_arrow_widget(), 4, 1, alignment=Qt.AlignCenter)

        # Row 5: Adjust 1 -> Phase 1 (Loop Re-entry Point)
        # (5, 0) Entry Arrow
        biopsy_layout.addWidget(self._create_entry_arrow_widget(), 5, 0)
        biopsy_layout.addWidget(self.flow_calc_1_btn, 5, 1, alignment=Qt.AlignCenter)
        biopsy_layout.addWidget(self._create_h_arrow_widget(), 5, 2)
        biopsy_layout.addWidget(self.flow_needle_in_p1_btn, 5, 3, alignment=Qt.AlignCenter)
        
        # Row 6: Down Arrows & Loop Line (Vertical Up)
        biopsy_layout.addWidget(self._create_v_line_widget(), 6, 0) 
        biopsy_layout.addWidget(self._create_v_arrow_widget(), 6, 3, alignment=Qt.AlignCenter)

        # Row 7: Adjust 2 -> Phase 2
        biopsy_layout.addWidget(self._create_v_line_widget(), 7, 0) 
        biopsy_layout.addWidget(self.flow_calc_2_btn, 7, 1, alignment=Qt.AlignCenter)
        biopsy_layout.addWidget(self._create_h_arrow_widget(), 7, 2)
        biopsy_layout.addWidget(self.flow_needle_in_p2_btn, 7, 3, alignment=Qt.AlignCenter)

        # Row 8: Down Arrows & Loop Line (Vertical Up)
        biopsy_layout.addWidget(self._create_v_line_widget(), 8, 0) 
        biopsy_layout.addWidget(self._create_v_arrow_widget(), 8, 3, alignment=Qt.AlignCenter)

        # Row 9: Adjust 3 -> Phase 3 -> Needle Retraction
        biopsy_layout.addWidget(self._create_v_line_widget(), 9, 0) 
        biopsy_layout.addWidget(self.flow_calc_3_btn, 9, 1, alignment=Qt.AlignCenter)
        biopsy_layout.addWidget(self._create_h_arrow_widget(), 9, 2)
        biopsy_layout.addWidget(self.flow_needle_in_p3_btn, 9, 3, alignment=Qt.AlignCenter)
        biopsy_layout.addWidget(self._create_h_arrow_widget(), 9, 4)
        biopsy_layout.addWidget(self.flow_needle_out_btn, 9, 5, alignment=Qt.AlignCenter)

        # Row 10: Loop Path (Bottom Row)
        # (10, 0): Corner (Right -> Up)
        biopsy_layout.addWidget(self._create_corner_LL_widget(), 10, 0)
        # (10, 1-4): Horizontal Lines
        biopsy_layout.addWidget(self._create_h_line_widget(), 10, 1)
        biopsy_layout.addWidget(self._create_h_line_widget(), 10, 2)
        biopsy_layout.addWidget(self._create_h_line_widget(), 10, 3)
        biopsy_layout.addWidget(self._create_h_line_widget(), 10, 4)
        # (10, 5): T-Junction (Top -> Bottom, Left -> In)
        biopsy_layout.addWidget(self._create_t_junction_widget(), 10, 5)

        # Row 11+: Trocar Retraction Flow
        biopsy_layout.addWidget(self._create_v_arrow_widget(), 11, 5, alignment=Qt.AlignCenter)
        biopsy_layout.addWidget(self.flow_trocar_out_btn, 12, 5, alignment=Qt.AlignCenter)
        biopsy_layout.addWidget(self._create_v_arrow_widget(), 13, 5, alignment=Qt.AlignCenter)
        biopsy_layout.addWidget(self.flow_end_lbl, 14, 5, alignment=Qt.AlignCenter)

        # Extra stretch
        biopsy_layout.setColumnStretch(6, 1) 
        main_layout.addWidget(biopsy_group)

        # -----------------------------------------------------------------
        # 3. Beckhoff PLC Communication
        # -----------------------------------------------------------------
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
        # 基础连接
        self.connect_ads_btn.clicked.connect(self.manager.connect_ads)
        self.disconnect_ads_btn.clicked.connect(self.manager.disconnect_ads)
        self.enable_motor_btn.clicked.connect(self.manager.toggle_motor_enable)
        self.reset_all_btn.clicked.connect(self.trigger_reset)
        self.apply_inc_btn.clicked.connect(self.apply_joint_increment)
        
        # 流程图连接
        self.flow_calc_1_btn.clicked.connect(self.adjust_needle_direction)
        self.flow_calc_2_btn.clicked.connect(self.adjust_needle_direction)
        self.flow_calc_3_btn.clicked.connect(self.adjust_needle_direction)
        
        self.flow_trocar_in_p1_btn.clicked.connect(self.run_trocar_insertion_phase_1)
        self.flow_trocar_in_p2_btn.clicked.connect(self.run_trocar_insertion_phase_2)
        self.flow_trocar_out_btn.clicked.connect(self.run_trocar_retraction)
        
        self.flow_needle_in_p1_btn.clicked.connect(self.run_needle_insertion_phase_1)
        self.flow_needle_in_p2_btn.clicked.connect(self.run_needle_insertion_phase_2)
        self.flow_needle_in_p3_btn.clicked.connect(self.run_needle_insertion_phase_3)
        self.flow_needle_out_btn.clicked.connect(self.run_needle_retraction)

        # 信号连接
        self.manager.connection_state_changed.connect(self.on_connection_changed)
        self.manager.position_update.connect(self.on_position_update)
        self.manager.enable_status_update.connect(self.on_enable_status_update)
        self.manager.movement_status_update.connect(self.on_movement_status_update)
        self.manager.target_update.connect(self.on_target_update)
        self.manager.position_update.connect(self.beckhoff_position_update.emit)

    # =========================================================================
    # Visual Helper Functions for Drawing Lines
    # [FIX] 使用 Spacers 代替 Alignment 标志，确保垂直线能够填满高度
    # =========================================================================

    def _get_line_style(self):
        return f"background-color: #444; border: none;"

    def _create_v_line_widget(self):
        """标准垂直线"""
        w = QWidget()
        w.setFixedWidth(self.CELL_WIDTH) # 固定宽度
        
        # [FIX] 使用 HBox + Spacers 确保垂直线居中且不被压缩
        lay = QHBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        
        line = QFrame()
        line.setFixedWidth(self.LINE_WIDTH)
        line.setStyleSheet(self._get_line_style())
        line.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding) # 垂直方向拉伸
        
        lay.addStretch(1)  # 左弹簧
        lay.addWidget(line)
        lay.addStretch(1)  # 右弹簧
        
        return w

    def _create_h_line_widget(self):
        """标准水平线"""
        w = QWidget()
        w.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # [FIX] 使用 VBox + Spacers 确保水平线居中且不被压缩
        lay = QVBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        
        lay.addStretch(1) # 上弹簧
        
        line = QFrame()
        line.setFixedHeight(self.LINE_WIDTH)
        line.setStyleSheet(self._get_line_style())
        line.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        lay.addWidget(line)
        
        lay.addStretch(1) # 下弹簧
        
        return w

    def _create_entry_arrow_widget(self):
        """
        Entry Point (Row 5, Col 0). '└>' shape.
        Line comes from Bottom, turns Right.
        """
        w = QWidget()
        w.setFixedWidth(self.CELL_WIDTH)
        main_layout = QVBoxLayout(w)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        
        # 1. Top Part (Stretch) - Empty
        main_layout.addStretch(1)
        
        # 2. Middle Part (Height=Fixed) - Horizontal Arrow
        mid_container = QWidget()
        mid_lay = QHBoxLayout(mid_container)
        mid_lay.setContentsMargins(0,0,0,0)
        mid_lay.setSpacing(0)
        
        mid_lay.addStretch(1) # Left Spacer (50%)
        
        # Right Arrow Line + Head
        arrow_box = QWidget()
        arrow_lay = QHBoxLayout(arrow_box)
        arrow_lay.setContentsMargins(0,0,0,0)
        arrow_lay.setSpacing(0)
        
        h_line = QFrame()
        h_line.setFixedHeight(self.LINE_WIDTH)
        h_line.setStyleSheet(self._get_line_style())
        h_line.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        
        head = QLabel("►")
        head.setStyleSheet("color: #444; font-size: 14px; font-weight: bold; margin-bottom: 2px;")
        head.setAlignment(Qt.AlignCenter)
        
        arrow_lay.addWidget(h_line)
        arrow_lay.addWidget(head)
        
        mid_lay.addWidget(arrow_box, 1) # Right Stretch
        
        main_layout.addWidget(mid_container)
        
        # 3. Bottom Part (Stretch) - Vertical Line
        bot_container = QWidget()
        bot_lay = QHBoxLayout(bot_container)
        bot_lay.setContentsMargins(0,0,0,0)
        bot_lay.setSpacing(0)
        
        v_line = QFrame()
        v_line.setFixedWidth(self.LINE_WIDTH)
        v_line.setStyleSheet(self._get_line_style())
        v_line.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)
        
        # [FIX] Use spacers for centering
        bot_lay.addStretch(1)
        bot_lay.addWidget(v_line)
        bot_lay.addStretch(1)
        
        main_layout.addWidget(bot_container, 1) # Stretch 1
        
        return w

    def _create_corner_LL_widget(self):
        """
        Bottom-Left Corner (Row 10, Col 0). '┘' shape.
        Line from Right, turns Up.
        """
        w = QWidget()
        w.setFixedWidth(self.CELL_WIDTH)
        main_layout = QVBoxLayout(w)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        
        # 1. Top Part (Stretch) - Vertical Line
        top_container = QWidget()
        top_lay = QHBoxLayout(top_container)
        top_lay.setContentsMargins(0,0,0,0)
        top_lay.setSpacing(0)
        
        v_line = QFrame()
        v_line.setFixedWidth(self.LINE_WIDTH)
        v_line.setStyleSheet(self._get_line_style())
        v_line.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)
        
        # [FIX] Use spacers for centering
        top_lay.addStretch(1)
        top_lay.addWidget(v_line)
        top_lay.addStretch(1)
        
        main_layout.addWidget(top_container, 1) # Stretch 1
        
        # 2. Middle Part (Height=Fixed) - Horizontal Line (Right side)
        mid_container = QWidget()
        mid_lay = QHBoxLayout(mid_container)
        mid_lay.setContentsMargins(0,0,0,0)
        mid_lay.setSpacing(0)
        
        mid_lay.addStretch(1) # Left Spacer (50%)
        
        h_line = QFrame()
        h_line.setFixedHeight(self.LINE_WIDTH)
        h_line.setStyleSheet(self._get_line_style())
        h_line.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        
        mid_lay.addWidget(h_line, 1) # Right line
        
        main_layout.addWidget(mid_container)
        
        # 3. Bottom Part (Stretch) - Empty
        main_layout.addStretch(1)
        
        return w

    def _create_t_junction_widget(self):
        """
        T-Junction (Row 10, Col 5). '┤' shape.
        """
        w = QWidget()
        w.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # Use HBox to split Left/Center/Right
        layout = QHBoxLayout(w)
        layout.setContentsMargins(0,0,0,0)
        layout.setSpacing(0)
        
        # Left: H-Line (Using standard H-line creation for alignment)
        left_w = self._create_h_line_widget()
        
        # Center: V-Line
        center_w = QWidget()
        c_lay = QHBoxLayout(center_w)
        c_lay.setContentsMargins(0,0,0,0)
        v_line = QFrame()
        v_line.setFixedWidth(self.LINE_WIDTH)
        v_line.setStyleSheet(self._get_line_style())
        v_line.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)
        c_lay.addWidget(v_line, alignment=Qt.AlignCenter)
        
        # Right: Spacer
        right_w = QWidget()
        right_w.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        layout.addWidget(left_w, 1)
        layout.addWidget(center_w, 0)
        layout.addWidget(right_w, 1)
        
        return w

    # ... [Same Kinematics Helper Functions] ...
    def _calculate_current_d4(self):
        """辅助函数：计算当前状态下到 B 点的剩余距离 d4"""
        try:
            delta_j1 = float(self.inc_j1_input.text())
            delta_j2 = float(self.inc_j2_input.text()) if self.inc_j2_input.text() else 0.0
            delta_j3 = float(self.inc_j3_input.text()) if self.inc_j3_input.text() else 0.0
            theoretical_j2 = delta_j2
            theoretical_j3 = delta_j3 - delta_j2
            
            parent = self.main_window
            b_point_p = parent.left_panel.b_point_in_tcp_p
            if b_point_p is None:
                QMessageBox.warning(self, "Error", "B point in TCP_P is missing.")
                return None

            tip_pos = self.robot.get_tip_of_needle([delta_j1, theoretical_j2, theoretical_j3, 0])
            b_point_vec = np.array(b_point_p)
            d4 = np.linalg.norm(b_point_vec - tip_pos)
            return d4
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to calc d4: {e}")
            return None

    def run_needle_insertion_phase_1(self):
        d4 = self._calculate_current_d4()
        if d4 is not None:
            # Phase 1: 走 1/3
            val = d4 / 3.0
            self.inc_j0_input.setText(f"{val:.4f}")
            self.apply_joint_increment()

    def run_needle_insertion_phase_2(self):
        d4 = self._calculate_current_d4()
        if d4 is not None:
            # Phase 2: 走当前剩余的 1/2
            val = d4 / 2.0
            self.inc_j0_input.setText(f"{val:.4f}")
            self.apply_joint_increment()

    def run_needle_insertion_phase_3(self):
        d4 = self._calculate_current_d4()
        if d4 is not None:
            # Phase 3: 走完剩余全部
            val = d4
            self.inc_j0_input.setText(f"{val:.4f}")
            self.apply_joint_increment()