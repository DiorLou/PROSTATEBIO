import numpy as np
import cv2
import math
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, 
    QLabel, QLineEdit, QPushButton, QMessageBox, QGroupBox, 
    QFrame, QSizePolicy
)
from PyQt5.QtGui import QPixmap, QImage, QPainter, QPen, QColor, QFont
from PyQt5.QtCore import Qt, pyqtSignal, QPoint, QTimer
from ui.beckhoff_tab import BeckhoffManager, BeckhoffTab

# =============================================================================
# [修改] 自定义图像标记控件 (NeedleImageViewer)
# =============================================================================
class NeedleImageViewer(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.pixmap = None
        self.points = []  # [Start, End]
        self.setMinimumSize(350, 250) 
        self.setStyleSheet("background-color: #000; border: 1px solid #555;")
        self.parent_tab = parent 

    def set_image(self, pixmap):
        self.pixmap = pixmap
        self.points = []
        self.update()
        if self.parent_tab:
            self.parent_tab.update_marking_info()

    def getScaledImageInfo(self):
        if not self.pixmap:
            return None, 0, 0, 0, 0
        scaled_pixmap = self.pixmap.scaled(
            self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        x = (self.width() - scaled_pixmap.width()) // 2
        y = (self.height() - scaled_pixmap.height()) // 2
        return scaled_pixmap, x, y, scaled_pixmap.width(), scaled_pixmap.height()
    
    # [新增] 计算图像上绘制线段的角度 (度)
    # 假设图像坐标系：左上角(0,0)，X向右，Y向下。
    # 物理坐标系（超声切面）：通常假设 X向右，Z向上(Pitch方向)。
    # 因此，图像上的 -dy 对应物理上的 dz。
    def get_angle(self):
        if len(self.points) < 2:
            return None
        
        p1 = self.points[0]
        p2 = self.points[1]
        
        dx = p2.x() - p1.x()
        dy = p2.y() - p1.y()
        
        # 计算角度，注意 y 轴反转 (图像y向下为正，物理z向上为正)
        # 结果范围 (-180, 180)
        angle_rad = math.atan2(-dy, dx)
        return math.degrees(angle_rad)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        if not self.pixmap:
            painter.setPen(QColor(200, 200, 200))
            painter.drawText(self.rect(), Qt.AlignCenter, "No Image Captured")
            return

        scaled_pixmap, x, y, scaled_w, scaled_h = self.getScaledImageInfo()
        painter.drawPixmap(x, y, scaled_pixmap)

        if len(self.points) >= 1:
            scale_x = scaled_w / self.pixmap.width()
            scale_y = scaled_h / self.pixmap.height()

            pt1 = self.points[0]
            sx1 = int(x + pt1.x() * scale_x)
            sy1 = int(y + pt1.y() * scale_y)

            pen = QPen(QColor(255, 0, 0), 6)
            painter.setPen(pen)
            painter.drawPoint(sx1, sy1)
            
            painter.setFont(QFont("Arial", 10))
            painter.setPen(QColor(255, 0, 0))
            painter.drawText(sx1 + 10, sy1 - 10, "Start")

            if len(self.points) == 2:
                pt2 = self.points[1]
                sx2 = int(x + pt2.x() * scale_x)
                sy2 = int(y + pt2.y() * scale_y)

                pen = QPen(QColor(0, 255, 255), 6)
                painter.setPen(pen)
                painter.drawPoint(sx2, sy2)

                painter.setPen(QColor(0, 255, 255))
                painter.drawText(sx2 + 10, sy2 - 10, "End")

                pen = QPen(QColor(0, 255, 0), 2, Qt.DashLine)
                painter.setPen(pen)
                painter.drawLine(sx1, sy1, sx2, sy2)

    def mousePressEvent(self, event):
        if self.pixmap and event.button() == Qt.LeftButton:
            scaled_pixmap, x, y, scaled_w, scaled_h = self.getScaledImageInfo()
            if (event.x() >= x and event.x() <= x + scaled_w and
                    event.y() >= y and event.y() <= y + scaled_h):
                rel_x = (event.x() - x) / scaled_w
                rel_y = (event.y() - y) / scaled_h
                original_x = int(rel_x * self.pixmap.width())
                original_y = int(rel_y * self.pixmap.height())

                if len(self.points) < 2:
                    self.points.append(QPoint(original_x, original_y))
                else:
                    self.points[0] = self.points[1]
                    self.points[1] = QPoint(original_x, original_y)
                self.update()
                if self.parent_tab:
                    self.parent_tab.update_marking_info()

    def resizeEvent(self, event):
        self.update()
        super().resizeEvent(event)

# =============================================================================
# FlexibleNeedleTab
# =============================================================================

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
        # [NEW] Initialize Yaw and Pitch state
        self.current_yaw = 0.0
        self.current_pitch = 0.0
        
        # [NEW] Error Correction Variables
        self.last_commanded_pitch = None # 上一次 adjust needle dir 计算出的理论 Pitch
        
        # [新增] 自动化序列相关的 Timer
        self.scan_polling_timer = QTimer(self)
        self.scan_polling_timer.timeout.connect(self._check_scan_status)

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

        # --- J0 - J3 Rows ---
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

        # [修改] 设置第5列作为间隔，拉开与右侧组件的距离
        result_layout.setColumnMinimumWidth(5, 40)

        # --- [NEW] Simplified Manual Controls (Column 6, 4 Components) ---
        
        # 1. Descent Distance (Label + Input)
        row0_container = QWidget()
        row0_layout = QHBoxLayout(row0_container)
        row0_layout.setContentsMargins(0,0,0,0)
        
        lbl_descent = QLabel("Descent Distance (mm)")
        self.descent_dist_input = QLineEdit("10")
        self.descent_dist_input.setAlignment(Qt.AlignCenter)
        self.descent_dist_input.setFixedWidth(50)
        
        row0_layout.addWidget(lbl_descent)
        row0_layout.addWidget(self.descent_dist_input)
        row0_layout.addStretch()
        result_layout.addWidget(row0_container, 0, 6)

        # 2. Rotate x Deg (Label + Input)
        row1_container = QWidget()
        row1_layout = QHBoxLayout(row1_container)
        row1_layout.setContentsMargins(0,0,0,0)
        
        lbl_rotate = QLabel("Rotate x Deg")
        self.rot_range_input = QLineEdit("10") # 默认值 10
        self.rot_range_input.setAlignment(Qt.AlignCenter)
        self.rot_range_input.setFixedWidth(50) 
        
        row1_layout.addWidget(lbl_rotate)
        row1_layout.addWidget(self.rot_range_input)
        row1_layout.addStretch()
        result_layout.addWidget(row1_container, 1, 6)

        # 3. Button: Δ J1 decreases
        self.btn_j1_decrease = QPushButton("Δ J1 decreases")
        self.btn_j1_decrease.clicked.connect(self.on_j1_decrease_clicked)
        result_layout.addWidget(self.btn_j1_decrease, 2, 6)

        # 4. Button: Rotate Probe and Reset (Automation)
        self.btn_rotate_reset = QPushButton("Rotate Probe and Reset")
        self.btn_rotate_reset.clicked.connect(self.on_rotate_probe_reset_clicked)
        result_layout.addWidget(self.btn_rotate_reset, 3, 6)


        # --- [NEW] Yaw / Pitch Control Rows ---
        # Row 4: Needle Yaw
        yaw_label = QLabel("Needle Yaw (°):")
        self.yaw_display = QLineEdit("0.0")
        self.yaw_display.setReadOnly(True)
        self.yaw_display.setFixedWidth(60)
        self.yaw_display.setStyleSheet("background-color: #f0f0f0;")
        
        yaw_btn_layout = QHBoxLayout()
        self.yaw_minus_btn = QPushButton("-")
        self.yaw_plus_btn = QPushButton("+")
        self.yaw_minus_btn.setFixedWidth(30)
        self.yaw_plus_btn.setFixedWidth(30)
        yaw_btn_layout.addWidget(self.yaw_minus_btn)
        yaw_btn_layout.addWidget(self.yaw_display)
        yaw_btn_layout.addWidget(self.yaw_plus_btn)
        yaw_btn_layout.addStretch()

        result_layout.addWidget(yaw_label, 4, 0)
        result_layout.addLayout(yaw_btn_layout, 4, 1, 1, 2)

        # Row 5: Needle Pitch
        pitch_label = QLabel("Needle Pitch (°):")
        self.pitch_display = QLineEdit("0.0")
        self.pitch_display.setReadOnly(True)
        self.pitch_display.setFixedWidth(60)
        self.pitch_display.setStyleSheet("background-color: #f0f0f0;")
        
        pitch_btn_layout = QHBoxLayout()
        self.pitch_minus_btn = QPushButton("-")
        self.pitch_plus_btn = QPushButton("+")
        self.pitch_minus_btn.setFixedWidth(30)
        self.pitch_plus_btn.setFixedWidth(30)
        pitch_btn_layout.addWidget(self.pitch_minus_btn)
        pitch_btn_layout.addWidget(self.pitch_display)
        pitch_btn_layout.addWidget(self.pitch_plus_btn)
        pitch_btn_layout.addStretch()
        
        result_layout.addWidget(pitch_label, 5, 0)
        result_layout.addLayout(pitch_btn_layout, 5, 1, 1, 2)

        # Buttons (moved to row 6)
        self.apply_inc_btn = QPushButton("Apply Increment (All)")
        result_layout.addWidget(self.apply_inc_btn, 6, 0, 1, 2) 

        self.reset_all_btn = QPushButton("Reset All (J0-J3)")
        self.reset_all_btn.setEnabled(False) 
        result_layout.addWidget(self.reset_all_btn, 6, 3, 1, 2)
        
        result_content_layout.addLayout(result_layout)
        result_content_layout.addStretch(1) 
        result_group.setLayout(result_content_layout)
        main_layout.addWidget(result_group)
        
        # -----------------------------------------------------------------
        # Middle Layout
        # -----------------------------------------------------------------
        middle_layout = QHBoxLayout()

        # === LEFT: Biopsy Interface ===
        biopsy_group = QGroupBox("Biopsy Interface (Flexible Needle Steering)")
        biopsy_layout = QGridLayout(biopsy_group)
        biopsy_layout.setSpacing(0) 
        biopsy_layout.setContentsMargins(5, 10, 5, 10) 

        self.flow_trocar_in_p1_btn = QPushButton("Trocar Insertion\nPhase 1")
        self.flow_trocar_in_p2_btn = QPushButton("Trocar Insertion\nPhase 2")
        self.flow_trocar_out_btn = QPushButton("Trocar\nRetraction")

        self.flow_calc_1_btn = QPushButton("Adjust\nNeedle Dir")
        self.flow_needle_in_p1_btn = QPushButton("Needle Insertion\nPhase 1")
        
        self.flow_calc_2_btn = QPushButton("Adjust\nNeedle Dir")
        self.flow_needle_in_p2_btn = QPushButton("Needle Insertion\nPhase 2")
        
        self.flow_calc_3_btn = QPushButton("Adjust\nNeedle Dir")
        self.flow_needle_in_p3_btn = QPushButton("Needle Insertion\nPhase 3")
        
        self.flow_needle_out_btn = QPushButton("Needle\nRetraction")

        # 压缩宽度
        BTN_WIDTH = 115
        BTN_HEIGHT = 40 
        
        for btn in [self.flow_trocar_in_p1_btn, self.flow_trocar_in_p2_btn, self.flow_trocar_out_btn,
                    self.flow_calc_1_btn, self.flow_needle_in_p1_btn,
                    self.flow_calc_2_btn, self.flow_needle_in_p2_btn,
                    self.flow_calc_3_btn, self.flow_needle_in_p3_btn,
                    self.flow_needle_out_btn]:
            btn.setFixedWidth(BTN_WIDTH)
            btn.setFixedHeight(BTN_HEIGHT)
            btn.setStyleSheet("font-size: 10px; text-align: center;")

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
        
        biopsy_layout.setColumnMinimumWidth(0, self.CELL_WIDTH) 
        biopsy_layout.setColumnMinimumWidth(2, 25) 
        biopsy_layout.setColumnMinimumWidth(4, 25)
        
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

        # Row 5: Adjust 1 -> Phase 1
        biopsy_layout.addWidget(self._create_entry_arrow_widget(), 5, 0)
        biopsy_layout.addWidget(self.flow_calc_1_btn, 5, 1, alignment=Qt.AlignCenter)
        biopsy_layout.addWidget(self._create_h_arrow_widget(), 5, 2)
        biopsy_layout.addWidget(self.flow_needle_in_p1_btn, 5, 3, alignment=Qt.AlignCenter)
        
        # Row 6: Down Arrows
        biopsy_layout.addWidget(self._create_v_line_widget(), 6, 0) 
        biopsy_layout.addWidget(self._create_v_arrow_widget(), 6, 3, alignment=Qt.AlignCenter)

        # Row 7: Adjust 2 -> Phase 2
        biopsy_layout.addWidget(self._create_v_line_widget(), 7, 0) 
        biopsy_layout.addWidget(self.flow_calc_2_btn, 7, 1, alignment=Qt.AlignCenter)
        biopsy_layout.addWidget(self._create_h_arrow_widget(), 7, 2)
        biopsy_layout.addWidget(self.flow_needle_in_p2_btn, 7, 3, alignment=Qt.AlignCenter)

        # Row 8: Down Arrows
        biopsy_layout.addWidget(self._create_v_line_widget(), 8, 0) 
        biopsy_layout.addWidget(self._create_v_arrow_widget(), 8, 3, alignment=Qt.AlignCenter)

        # Row 9: Adjust 3 -> Phase 3 -> Needle Retraction
        biopsy_layout.addWidget(self._create_v_line_widget(), 9, 0) 
        biopsy_layout.addWidget(self.flow_calc_3_btn, 9, 1, alignment=Qt.AlignCenter)
        biopsy_layout.addWidget(self._create_h_arrow_widget(), 9, 2)
        biopsy_layout.addWidget(self.flow_needle_in_p3_btn, 9, 3, alignment=Qt.AlignCenter)
        biopsy_layout.addWidget(self._create_h_arrow_widget(), 9, 4)
        biopsy_layout.addWidget(self.flow_needle_out_btn, 9, 5, alignment=Qt.AlignCenter)

        # Row 10: Loop Path
        biopsy_layout.addWidget(self._create_corner_LL_widget(), 10, 0)
        biopsy_layout.addWidget(self._create_h_line_widget(), 10, 1)
        biopsy_layout.addWidget(self._create_h_line_widget(), 10, 2)
        biopsy_layout.addWidget(self._create_h_line_widget(), 10, 3)
        biopsy_layout.addWidget(self._create_h_line_widget(), 10, 4)
        biopsy_layout.addWidget(self._create_t_junction_widget(), 10, 5)

        # Row 11+: Trocar Retraction Flow
        biopsy_layout.addWidget(self._create_v_arrow_widget(), 11, 5, alignment=Qt.AlignCenter)
        biopsy_layout.addWidget(self.flow_trocar_out_btn, 12, 5, alignment=Qt.AlignCenter)
        biopsy_layout.addWidget(self._create_v_arrow_widget(), 13, 5, alignment=Qt.AlignCenter)
        biopsy_layout.addWidget(self.flow_end_lbl, 14, 5, alignment=Qt.AlignCenter)

        biopsy_layout.setColumnStretch(6, 1) 
        middle_layout.addWidget(biopsy_group, 60) # 60% Width

        # === RIGHT: Needle Image Marking Group ===
        img_group = QGroupBox("Needle Image Marking")
        img_layout = QVBoxLayout(img_group)

        # 1. Image Viewer
        self.needle_viewer = NeedleImageViewer(self)
        img_layout.addWidget(self.needle_viewer, 1)

        # 2. Info Labels
        info_layout = QGridLayout()
        # [修改] 增加水平间距以拉开两列
        info_layout.setHorizontalSpacing(30)
        
        self.mark_start_lbl = QLabel("Start: --")
        self.mark_end_lbl = QLabel("End: --")
        
        # [修改] 增加角度、误差、Next Target相关显示
        self.mark_vec_lbl = QLabel("Vector (px): --")
        self.actual_angle_lbl = QLabel("Actual Angle: --°")
        self.theo_angle_lbl = QLabel("Theo Angle: --°")
        self.error_angle_lbl = QLabel("Error Angle: --°")
        self.next_angle_lbl = QLabel("Next Target: --°")
        
        # [新增] 系数 K 输入
        k_lbl = QLabel("Coeff k:")
        self.k_input = QLineEdit("0.4")
        self.k_input.setFixedWidth(50)
        
        info_layout.addWidget(self.mark_start_lbl, 0, 0)
        info_layout.addWidget(self.mark_end_lbl, 0, 1)
        info_layout.addWidget(self.mark_vec_lbl, 1, 0, 1, 2)
        
        info_layout.addWidget(self.actual_angle_lbl, 2, 0)
        info_layout.addWidget(self.theo_angle_lbl, 2, 1)
        
        # [修改] Error Angle 独占一行 (第3行)
        info_layout.addWidget(self.error_angle_lbl, 3, 0, 1, 2)
        
        # [修改] 第4行: 左侧放 Next Target, 右侧放 Coeff k
        # 将 Next Target 放在第0列 (Actual Angle下方)
        info_layout.addWidget(self.next_angle_lbl, 4, 0)
        
        # 将 Coeff k 放在第1列 (Theo Angle下方)
        k_layout = QHBoxLayout()
        k_layout.addWidget(k_lbl)
        k_layout.addWidget(self.k_input)
        k_layout.addStretch() # 保持靠左 (靠近label)
        
        info_layout.addLayout(k_layout, 4, 1)
        
        img_layout.addLayout(info_layout)

        # 3. Control Buttons
        btn_layout = QHBoxLayout()
        self.capture_img_btn = QPushButton("Capture from Ultrasound")
        self.clear_marks_btn = QPushButton("Clear Marks")
        self.capture_img_btn.setStyleSheet("background-color: #E1F5FE; font-weight: bold;")
        
        btn_layout.addWidget(self.capture_img_btn)
        btn_layout.addWidget(self.clear_marks_btn)
        img_layout.addLayout(btn_layout)

        middle_layout.addWidget(img_group, 40) # 40% Width
        main_layout.addLayout(middle_layout)

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
        # Shared Connections
        self.connect_ads_btn.clicked.connect(self.manager.connect_ads)
        self.disconnect_ads_btn.clicked.connect(self.manager.disconnect_ads)
        self.enable_motor_btn.clicked.connect(self.manager.toggle_motor_enable)
        self.reset_all_btn.clicked.connect(self.trigger_reset)
        self.apply_inc_btn.clicked.connect(self.apply_joint_increment)
        
        # Specific Connections
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

        # [NEW] Yaw/Pitch Connections
        self.yaw_minus_btn.clicked.connect(lambda: self.change_needle_angle(-1, 0))
        self.yaw_plus_btn.clicked.connect(lambda: self.change_needle_angle(1, 0))
        self.pitch_minus_btn.clicked.connect(lambda: self.change_needle_angle(0, -1))
        self.pitch_plus_btn.clicked.connect(lambda: self.change_needle_angle(0, 1))

        self.manager.connection_state_changed.connect(self.on_connection_changed)
        self.manager.position_update.connect(self.on_position_update)
        self.manager.enable_status_update.connect(self.on_enable_status_update)
        self.manager.movement_status_update.connect(self.on_movement_status_update)
        self.manager.target_update.connect(self.on_target_update)
        self.manager.position_update.connect(self.beckhoff_position_update.emit)

        # Image Connections
        self.capture_img_btn.clicked.connect(self.capture_image_from_ultrasound)
        self.clear_marks_btn.clicked.connect(self.clear_marks)
        
    # [修改] 重写 adjust_needle_direction 以加入误差补偿逻辑
    def adjust_needle_direction(self):
        parent = self.main_window
        if not parent or not hasattr(parent, 'left_panel'): return
        b_point_p = parent.left_panel.b_point_in_tcp_p
        if b_point_p is None:
            QMessageBox.warning(self, "Data Missing", "Biopsy Point B in TCP_P is not defined.")
            return
        
        try:
            # 1. 计算理论上的基础向量 (Raw Target Vector)
            # RCM点会随着 Trocar insertion 改变 (delta_j1)
            delta_j1 = float(self.inc_j1_input.text())
            rcm_point = self.robot.get_rcm_point([delta_j1, 0, 0, 0])
            raw_vector = np.array(b_point_p) - rcm_point
            
            # 归一化
            norm = np.linalg.norm(raw_vector)
            if norm < 1e-6:
                QMessageBox.critical(self, "Error", "Vector magnitude zero.")
                return
            raw_unit_vec = raw_vector / norm
            
            # 2. 将 Raw Vector 转换为 Pitch 和 Yaw
            # 根据 change_needle_angle 中的公式:
            # vx = -sin(y)*cos(p), vy = cos(y)*cos(p), vz = sin(p)
            # 所以 p = arcsin(vz)
            raw_pitch_rad = np.arcsin(raw_unit_vec[2])
            raw_pitch_deg = np.rad2deg(raw_pitch_rad)
            
            # y = arctan2(-vx, vy)
            raw_yaw_rad = np.arctan2(-raw_unit_vec[0], raw_unit_vec[1])
            raw_yaw_deg = np.rad2deg(raw_yaw_rad)
            
            # 3. 检查是否有图像标记以及上一次的指令，以计算误差
            measured_angle = self.needle_viewer.get_angle()
            
            target_pitch_deg = raw_pitch_deg # 默认目标为几何计算值
            
            # 读取用户输入的系数 K
            try:
                k_val = float(self.k_input.text())
            except ValueError:
                k_val = 0.4
                self.k_input.setText("0.4")
            
            if (measured_angle is not None and self.last_commanded_pitch is not None):
                # 误差 = 上次理论 - 实际测量 (代表我们少了多少度)
                # 例如：目标30，实际25，误差+5。我们需要补偿让下次更高。
                # 您的公式：角度调整为 (k * 误差) + 原本Target
                error_deg = self.last_commanded_pitch - measured_angle
                
                correction = k_val * error_deg
                target_pitch_deg = raw_pitch_deg + correction
                
                # 更新显示
                self.error_angle_lbl.setText(f"Error: {error_deg:.2f}°")
                if self.main_window:
                    self.main_window.status_bar.showMessage(f"Applied Correction: {raw_pitch_deg:.1f} + {k_val}*{error_deg:.1f} = {target_pitch_deg:.1f}°")
            else:
                 self.error_angle_lbl.setText("Error Angle: --°")
            
            # 4. 根据修正后的 Pitch 和 原本的 Yaw 重建向量
            # 我们假设误差主要发生在 Pitch (超声切面内)
            final_pitch_rad = np.deg2rad(target_pitch_deg)
            final_yaw_rad = np.deg2rad(raw_yaw_deg)
            
            new_vx = -np.sin(final_yaw_rad) * np.cos(final_pitch_rad)
            new_vy = np.cos(final_yaw_rad) * np.cos(final_pitch_rad)
            new_vz = np.sin(final_pitch_rad)
            
            # 5. 更新 UI 和 内部状态
            self.vector_inputs[0].setText(f"{new_vx:.4f}")
            self.vector_inputs[1].setText(f"{new_vy:.4f}")
            self.vector_inputs[2].setText(f"{new_vz:.4f}")
            
            self.current_yaw = raw_yaw_deg
            self.current_pitch = target_pitch_deg
            self.yaw_display.setText(f"{self.current_yaw:.1f}")
            self.pitch_display.setText(f"{self.current_pitch:.1f}")
            
            # 6. 保存本次的 Target Pitch
            self.last_commanded_pitch = target_pitch_deg
            self.theo_angle_lbl.setText(f"Theo Angle: {self.last_commanded_pitch:.2f}°")
            self.next_angle_lbl.setText(f"Next Target: {self.last_commanded_pitch:.2f}°")
            
            # 7. 触发逆运动学计算
            self.calculate_joint_values()
            
            # 自动应用增量 (可选，根据流程是否需要自动执行)
            self.apply_joint_increment()
            
        except Exception as e:
             QMessageBox.critical(self, "Error", f"Failed: {e}")

    # =========================================================================
    # [NEW] Yaw/Pitch Logic
    # =========================================================================
    def change_needle_angle(self, yaw_delta, pitch_delta):
        """
        Adjust yaw or pitch, recalculate vector, and update ΔJ2/ΔJ3.
        """
        self.current_yaw += yaw_delta
        self.current_pitch += pitch_delta
        
        # Update display
        self.yaw_display.setText(f"{self.current_yaw:.1f}")
        self.pitch_display.setText(f"{self.current_pitch:.1f}")
        
        # Calculate new vector
        # Base vector (0, 1, 0)
        # Yaw rotates around Z-axis (creating X component)
        # Pitch rotates around X-axis (creating Z component)
        # x = -sin(Yaw) * cos(Pitch)
        # y = cos(Yaw) * cos(Pitch)
        # z = sin(Pitch)
        
        yaw_rad = np.deg2rad(self.current_yaw)
        pitch_rad = np.deg2rad(self.current_pitch)
        
        vx = -np.sin(yaw_rad) * np.cos(pitch_rad)
        vy = np.cos(yaw_rad) * np.cos(pitch_rad)
        vz = np.sin(pitch_rad)
        
        # Update Vector Inputs (UI only)
        self.vector_inputs[0].setText(f"{vx:.4f}")
        self.vector_inputs[1].setText(f"{vy:.4f}")
        self.vector_inputs[2].setText(f"{vz:.4f}")
        
        # Recalculate Joints (this updates inc_j2/inc_j3 inputs automatically)
        self.calculate_joint_values()

        # [新增] 自动 Apply Increment
        self.apply_joint_increment()

    # =========================================================================
    # [新增] Manual Controls Logic (J1 Decrease & Auto Rotate)
    # =========================================================================
    
    def on_j1_decrease_clicked(self):
        """让 Δ J1 减少指定的 'Descent Distance' 并执行"""
        try:
            current_inc = float(self.inc_j1_input.text())
            decrease_amount = float(self.descent_dist_input.text())
            
            new_val = current_inc - decrease_amount
            self.inc_j1_input.setText(f"{new_val:.4f}")
            self.apply_joint_increment()
            
        except ValueError:
            QMessageBox.warning(self, "Input Error", "Please check J1 or Descent Distance input format.")

    def on_rotate_probe_reset_clicked(self):
        """
        自动化序列：
        1. Probe Left x
        2. (1s后) Probe Right 2X (Scan)
        3. (等待扫描完成)
        4. (1s后) Probe Left x (Reset)
        5. (1s后) J1 Increase (Descent Dist) & Apply
        """
        # 1. Probe Left x
        self.rotate_probe(direction=0, multiplier=1) # 0=Left/Backward
        
        # 2. 1s 后执行扫描 (Right 2x)
        QTimer.singleShot(1000, self._step2_start_scan)

    def _step2_start_scan(self):
        # 触发 Ultrasound Tab 的扫描功能
        if self.main_window and hasattr(self.main_window, 'ultrasound_tab'):
             try:
                 # 同步角度
                 x_val = self.rot_range_input.text()
                 self.main_window.ultrasound_tab.rotation_range_input.setText(x_val)
                 
                 # 调用扫描
                 self.main_window.ultrasound_tab.rotate_and_capture_2x()
                 
                 # 启动轮询 Timer，等待扫描结束
                 self.scan_polling_timer.start(500) # 每500ms检查一次
                 
             except Exception as e:
                 QMessageBox.critical(self, "Error", f"Failed to start scan: {e}")
        else:
             QMessageBox.warning(self, "Warning", "Ultrasound Tab not found. Sequence aborted.")

    def _check_scan_status(self):
        """轮询检查扫描是否完成"""
        if self.main_window and hasattr(self.main_window, 'ultrasound_tab'):
            # 如果 is_rotating 变为 False，说明扫描结束
            if not self.main_window.ultrasound_tab.is_rotating:
                self.scan_polling_timer.stop()
                
                # 扫描完成，1s 后执行复位
                QTimer.singleShot(1000, self._step4_reset_probe)

    def _step4_reset_probe(self):
        # Probe Left x (从 +x 回到 0)
        self.rotate_probe(direction=0, multiplier=1) # 0=Left
        
        # 1s 后执行 J1 增加
        QTimer.singleShot(1000, self._step5_increase_j1)

    def _step5_increase_j1(self):
        # J1 Increase by Descent Distance
        try:
            current_inc = float(self.inc_j1_input.text())
            increase_amount = float(self.descent_dist_input.text())
            new_val = current_inc + increase_amount
            self.inc_j1_input.setText(f"{new_val:.4f}")
            self.apply_joint_increment()
            
            if self.main_window:
                self.main_window.status_bar.showMessage("Auto Sequence Completed: Scanned, Reset, and Advanced Needle.")
                
        except ValueError:
            pass

    def rotate_probe(self, direction, multiplier):
        if not self.main_window or not hasattr(self.main_window, 'tcp_manager'):
            return
        try:
            deg = float(self.rot_range_input.text())
            if deg <= 0: raise ValueError
        except ValueError:
            return
        angle = deg * multiplier
        command = f"MoveRelJ,0,5,{direction},{angle};"
        self.main_window.tcp_manager.send_command(command)

    # =========================================================================
    # Visual Helper Functions for Drawing Lines (恢复原始代码)
    # =========================================================================

    def _get_line_style(self):
        return f"background-color: #444; border: none;"

    def _create_v_line_widget(self):
        """标准垂直线"""
        w = QWidget()
        w.setFixedWidth(self.CELL_WIDTH) 
        lay = QHBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        line = QFrame()
        line.setFixedWidth(self.LINE_WIDTH)
        line.setStyleSheet(self._get_line_style())
        line.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding) 
        lay.addStretch(1)
        lay.addWidget(line)
        lay.addStretch(1)
        return w

    def _create_h_line_widget(self):
        """标准水平线"""
        w = QWidget()
        w.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        lay = QVBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        lay.addStretch(1) 
        line = QFrame()
        line.setFixedHeight(self.LINE_WIDTH)
        line.setStyleSheet(self._get_line_style())
        line.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        lay.addWidget(line)
        lay.addStretch(1) 
        return w

    def _create_entry_arrow_widget(self):
        """Entry Point (Row 5, Col 0). '└>' shape."""
        w = QWidget()
        w.setFixedWidth(self.CELL_WIDTH)
        main_layout = QVBoxLayout(w)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        
        main_layout.addStretch(1)
        
        mid_container = QWidget()
        mid_lay = QHBoxLayout(mid_container)
        mid_lay.setContentsMargins(0,0,0,0)
        mid_lay.setSpacing(0)
        mid_lay.addStretch(1) 
        
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
        
        mid_lay.addWidget(arrow_box, 1) 
        main_layout.addWidget(mid_container)
        
        bot_container = QWidget()
        bot_lay = QHBoxLayout(bot_container)
        bot_lay.setContentsMargins(0,0,0,0)
        bot_lay.setSpacing(0)
        v_line = QFrame()
        v_line.setFixedWidth(self.LINE_WIDTH)
        v_line.setStyleSheet(self._get_line_style())
        v_line.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)
        bot_lay.addStretch(1)
        bot_lay.addWidget(v_line)
        bot_lay.addStretch(1)
        
        main_layout.addWidget(bot_container, 1) 
        return w

    def _create_corner_LL_widget(self):
        """Bottom-Left Corner (Row 10, Col 0). '┘' shape."""
        w = QWidget()
        w.setFixedWidth(self.CELL_WIDTH)
        main_layout = QVBoxLayout(w)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        
        top_container = QWidget()
        top_lay = QHBoxLayout(top_container)
        top_lay.setContentsMargins(0,0,0,0)
        top_lay.setSpacing(0)
        v_line = QFrame()
        v_line.setFixedWidth(self.LINE_WIDTH)
        v_line.setStyleSheet(self._get_line_style())
        v_line.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)
        top_lay.addStretch(1)
        top_lay.addWidget(v_line)
        top_lay.addStretch(1)
        
        main_layout.addWidget(top_container, 1)
        
        mid_container = QWidget()
        mid_lay = QHBoxLayout(mid_container)
        mid_lay.setContentsMargins(0,0,0,0)
        mid_lay.setSpacing(0)
        mid_lay.addStretch(1) 
        h_line = QFrame()
        h_line.setFixedHeight(self.LINE_WIDTH)
        h_line.setStyleSheet(self._get_line_style())
        h_line.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        mid_lay.addWidget(h_line, 1) 
        main_layout.addWidget(mid_container)
        main_layout.addStretch(1)
        return w

    def _create_t_junction_widget(self):
        """T-Junction (Row 10, Col 5). '┤' shape."""
        w = QWidget()
        w.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout = QHBoxLayout(w)
        layout.setContentsMargins(0,0,0,0)
        layout.setSpacing(0)
        left_w = self._create_h_line_widget()
        center_w = QWidget()
        c_lay = QHBoxLayout(center_w)
        c_lay.setContentsMargins(0,0,0,0)
        v_line = QFrame()
        v_line.setFixedWidth(self.LINE_WIDTH)
        v_line.setStyleSheet(self._get_line_style())
        v_line.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)
        c_lay.addWidget(v_line, alignment=Qt.AlignCenter)
        right_w = QWidget()
        right_w.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout.addWidget(left_w, 1)
        layout.addWidget(center_w, 0)
        layout.addWidget(right_w, 1)
        return w

    # =========================================================================
    # Kinematics Logic
    # =========================================================================
    def _calculate_current_d4(self):
        try:
            delta_j1 = float(self.inc_j1_input.text())
            delta_j2 = float(self.inc_j2_input.text()) if self.inc_j2_input.text() else 0.0
            delta_j3 = float(self.inc_j3_input.text()) if self.inc_j3_input.text() else 0.0
            theoretical_j2 = delta_j2
            theoretical_j3 = delta_j3 - delta_j2
            
            parent = self.main_window
            if not hasattr(parent, 'left_panel'): return None
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

    # Image Slots
    def capture_image_from_ultrasound(self):
        if self.main_window and hasattr(self.main_window, 'ultrasound_tab'):
            frame = self.main_window.ultrasound_tab.current_frame
            if frame is not None:
                rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_image.shape
                bytes_per_line = ch * w
                qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
                pixmap = QPixmap.fromImage(qt_image)
                
                self.needle_viewer.set_image(pixmap)
                self.main_window.status_bar.showMessage("Image Captured from Ultrasound Tab.")
            else:
                QMessageBox.warning(self, "Warning", "No image available. Please start the probe in Ultrasound Tab.")
        else:
            QMessageBox.warning(self, "Error", "Cannot access Ultrasound Tab.")

    def clear_marks(self):
        self.needle_viewer.points = []
        self.needle_viewer.update()
        self.update_marking_info()

    def update_marking_info(self):
        points = self.needle_viewer.points
        
        # 1. Update Points Info
        if len(points) >= 1:
            self.mark_start_lbl.setText(f"Start: ({points[0].x()}, {points[0].y()})")
        else:
            self.mark_start_lbl.setText("Start: --")

        if len(points) == 2:
            self.mark_end_lbl.setText(f"End: ({points[1].x()}, {points[1].y()})")
            vec_x = points[1].x() - points[0].x()
            vec_y = points[1].y() - points[0].y()
            self.mark_vec_lbl.setText(f"Vector (px): ({vec_x}, {vec_y})")
        else:
            self.mark_end_lbl.setText("End: --")
            self.mark_vec_lbl.setText("Vector (px): --")
            
        # 2. Update Angle Info
        # 获取实际测量角度
        actual_ang = self.needle_viewer.get_angle()
        
        if actual_ang is not None:
            self.actual_angle_lbl.setText(f"Actual Angle: {actual_ang:.2f}°")
            
            # 计算并显示误差
            if self.last_commanded_pitch is not None:
                self.theo_angle_lbl.setText(f"Theo Angle: {self.last_commanded_pitch:.2f}°")
                err = self.last_commanded_pitch - actual_ang
                self.error_angle_lbl.setText(f"Error Angle: {err:.2f}°")
            else:
                self.theo_angle_lbl.setText(f"Theo Angle: --°")
                self.error_angle_lbl.setText(f"Error Angle: --°")
        else:
            self.actual_angle_lbl.setText("Actual Angle: --°")
            # 保持 Theoretical 不变或清除，取决于需求，这里暂不清除 Theo
            if self.last_commanded_pitch is not None:
                 self.theo_angle_lbl.setText(f"Theo Angle: {self.last_commanded_pitch:.2f}°")
            else:
                 self.theo_angle_lbl.setText(f"Theo Angle: --°")
            self.error_angle_lbl.setText("Error Angle: --°")