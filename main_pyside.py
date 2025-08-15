import sys
import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox
import socket
import threading
import time

# --- 机器人工具箱导入 ---
try:
    from roboticstoolbox import DHRobot, PrismaticMDH, RevoluteMDH
    from spatialmath import SE3

except ImportError as e:
    root = tk.Tk()
    root.withdraw()
    messagebox.showerror(
        "库导入失败",
        f"无法导入核心库，请检查安装。\n\n具体错误: {e}"
    )
    sys.exit()

class RobotControlWindow(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("RRRRRR 机器人高级控制界面 (Tkinter稳定版)")
        self.geometry("900x550")

        # --- 1. 定义新的6关节机器人DH模型 ---
        self.create_robot()

        # --- 2. 初始化机器人状态，关节数改为6 ---
        self.current_q = np.zeros(self.robot.n)
        self._after_id = None
        self._moving_joint_index = None
        self._moving_direction = 0

        # --- 3. 初始化UI变量，关节数量改为6 ---
        self.joint_vars = [tk.StringVar(value="0.00") for _ in range(self.robot.n)]
        self.pose_vars = {
            "X": tk.StringVar(value="0.00"),
            "Y": tk.StringVar(value="0.00"),
            "Z": tk.StringVar(value="0.00"),
            "Roll": tk.StringVar(value="0.00"),
            "Pitch": tk.StringVar(value="0.00"),
            "Yaw": tk.StringVar(value="0.00")
        }
        # IK输入变量数量保持6个不变，但Roll,Pitch,Yaw现在是可用的
        self.ik_input_vars = [tk.StringVar(value=v) for v in ["15", "0", "15", "0", "0", "0"]]
        self.ik_result_var = tk.StringVar(value="逆解结果将显示在这里。")
        self.status_var = tk.StringVar(value="状态: 准备就绪")

        # --- 4. 初始化 TCP 变量 ---
        self.is_connected = False
        self.client_socket = None
        self.ip_var = tk.StringVar(value="127.0.0.1")
        self.port_var = tk.StringVar(value="12345")
        self.tcp_status_var = tk.StringVar(value="TCP状态: 未连接")

        # --- 5. 初始化UI ---
        self.init_ui()

        # --- 6. 首次更新显示 ---
        self.update_state_display()

    def create_robot(self):
        """创建新的6关节RRRRRR机器人DH模型"""
        # 使用一个简化的6关节机器人模型
        L1_d, L2_a, L3_a, L4_d, L6_d = 10, 15, 15, 10, 5
        
        L1 = RevoluteMDH(alpha=np.pi/2, d=L1_d, a=0, qlim=[-np.pi, np.pi])
        L2 = RevoluteMDH(alpha=0, d=0, a=L2_a, qlim=[-np.pi, np.pi])
        L3 = RevoluteMDH(alpha=np.pi/2, d=0, a=L3_a, qlim=[-np.pi, np.pi])
        L4 = RevoluteMDH(alpha=-np.pi/2, d=L4_d, a=0, qlim=[-np.pi, np.pi])
        L5 = RevoluteMDH(alpha=np.pi/2, d=0, a=0, qlim=[-np.pi, np.pi])
        L6 = RevoluteMDH(alpha=0, d=L6_d, a=0, qlim=[-np.pi, np.pi])
        
        self.robot = DHRobot([L1, L2, L3, L4, L5, L6], name="RRRRRR_Arm")
        print("机器人模型已更新为6关节RRRRRR:\n", self.robot)

    def init_ui(self):
        """创建所有UI组件和布局"""
        style = ttk.Style(self)
        style.theme_use('clam')

        main_frame = ttk.Frame(self, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        left_panel = ttk.Frame(main_frame)
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))

        right_panel = ttk.Frame(main_frame)
        right_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(10, 0))

        motor_group = self.create_motor_group(left_panel)
        motor_group.pack(fill=tk.X, expand=True, pady=(0, 10))

        action_group = self.create_action_group(left_panel)
        action_group.pack(fill=tk.X, expand=True, pady=(10, 10))

        state_group = self.create_state_display_group(left_panel)
        state_group.pack(fill=tk.BOTH, expand=True)

        self.create_tcp_group(right_panel)

        status_bar = ttk.Label(self, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)

    def create_motor_group(self, parent):
        """创建左侧的电机控制模块，会根据self.robot.n自动生成6个关节"""
        group = ttk.LabelFrame(parent, text="电机微调模块 (正运动学)", padding="10")
        for i in range(self.robot.n):
            row_frame = ttk.Frame(group)
            row_frame.pack(fill=tk.X, pady=2)
            joint_type = "d" if self.robot.links[i].isprismatic else "q"
            # 关节数量会根据self.robot.n自动变为6
            ttk.Label(row_frame, text=f"关节 {i+1} ({joint_type}{i+1}):").pack(side=tk.LEFT)
            ttk.Label(row_frame, textvariable=self.joint_vars[i], width=7, anchor=tk.E).pack(side=tk.LEFT, padx=5)
            btn_minus = ttk.Button(row_frame, text="-", width=3)
            btn_minus.pack(side=tk.LEFT)
            btn_minus.bind('<ButtonPress-1>', lambda e, j=i, d=-1: self.start_move(j, d))
            btn_minus.bind('<ButtonRelease-1>', self.stop_move)
            btn_plus = ttk.Button(row_frame, text="+", width=3)
            btn_plus.pack(side=tk.LEFT)
            btn_plus.bind('<ButtonPress-1>', lambda e, j=i, d=1: self.start_move(j, d))
            btn_plus.bind('<ButtonRelease-1>', self.stop_move)
        return group

    def create_action_group(self, parent):
        """创建逆解功能区，Roll/Pitch/Yaw输入框现在可用"""
        group = ttk.LabelFrame(parent, text="逆运动学解算模块", padding="10")
        grid_frame = ttk.Frame(group)
        grid_frame.pack(fill=tk.X)
        labels = ["X (mm)", "Y (mm)", "Z (mm)", "Roll (°)", "Pitch (°)", "Yaw (°)"]
        for i, label_text in enumerate(labels):
            row, col = i // 3, (i % 3) * 2
            ttk.Label(grid_frame, text=label_text).grid(row=row, column=col, sticky=tk.W, padx=5, pady=2)
            entry = ttk.Entry(grid_frame, textvariable=self.ik_input_vars[i], width=8)
            entry.grid(row=row, column=col + 1, sticky=tk.EW, padx=5, pady=2)
            # 移除禁用代码，所有输入框都将可用
            # if i >= 3:
            #     entry.config(state='disabled')
        ttk.Button(group, text="执行逆解", command=self.run_inverse_kinematics).pack(pady=10)
        ttk.Label(group, textvariable=self.ik_result_var, justify=tk.CENTER).pack()
        return group

    def create_state_display_group(self, parent):
        """创建底部用于显示实时状态的模块，包含六个信息，并调整布局"""
        group = ttk.LabelFrame(parent, text="机器人末端实时状态", padding="10")
        group.pack_propagate(False)

        grid_frame = ttk.Frame(group)
        grid_frame.pack(expand=True, fill=tk.BOTH)

        pose_info = [
            ("X (mm)", "X"),
            ("Y (mm)", "Y"),
            ("Z (mm)", "Z")
        ]
        
        orientation_info = [
            ("Roll (°)", "Roll"),
            ("Pitch (°)", "Pitch"),
            ("Yaw (°)", "Yaw")
        ]
        
        for i, (label_text, var_key) in enumerate(pose_info):
            ttk.Label(grid_frame, text=label_text, font=('Helvetica', 10, 'bold')).grid(row=i, column=0, sticky=tk.W, padx=5, pady=2)
            ttk.Label(grid_frame, textvariable=self.pose_vars[var_key], width=10, anchor=tk.E, relief=tk.SUNKEN).grid(row=i, column=1, sticky=tk.EW, padx=5, pady=2)
            
        for i, (label_text, var_key) in enumerate(orientation_info):
            ttk.Label(grid_frame, text=label_text, font=('Helvetica', 10, 'bold')).grid(row=i, column=2, sticky=tk.W, padx=5, pady=2)
            ttk.Label(grid_frame, textvariable=self.pose_vars[var_key], width=10, anchor=tk.E, relief=tk.SUNKEN).grid(row=i, column=3, sticky=tk.EW, padx=5, pady=2)
            
        return group

    def create_tcp_group(self, parent):
        """创建TCP连接和消息收发模块"""
        group = ttk.LabelFrame(parent, text="TCP通信模块", padding="10")
        group.pack(fill=tk.BOTH, expand=True)

        ip_port_frame = ttk.Frame(group)
        ip_port_frame.pack(fill=tk.X, pady=5)
        ttk.Label(ip_port_frame, text="远程IP:").pack(side=tk.LEFT)
        ttk.Entry(ip_port_frame, textvariable=self.ip_var, width=15).pack(side=tk.LEFT, padx=5)
        ttk.Label(ip_port_frame, text="端口:").pack(side=tk.LEFT)
        ttk.Entry(ip_port_frame, textvariable=self.port_var, width=8).pack(side=tk.LEFT, padx=5)

        btn_frame = ttk.Frame(group)
        btn_frame.pack(fill=tk.X, pady=5)
        self.connect_button = ttk.Button(btn_frame, text="连接", command=self.connect_tcp)
        self.connect_button.pack(side=tk.LEFT, expand=True, padx=(0, 5))
        self.disconnect_button = ttk.Button(btn_frame, text="断开", command=self.disconnect_tcp, state=tk.DISABLED)
        self.disconnect_button.pack(side=tk.LEFT, expand=True, padx=(5, 0))

        ttk.Label(group, textvariable=self.tcp_status_var, foreground='blue').pack(fill=tk.X, pady=5)

        ttk.Label(group, text="接收消息:").pack(fill=tk.X, anchor=tk.W)
        self.recv_text = tk.Text(group, height=10, width=40, state=tk.DISABLED, bg='lightgrey')
        self.recv_text.pack(fill=tk.BOTH, expand=True, pady=5)

        ttk.Label(group, text="发送消息:").pack(fill=tk.X, anchor=tk.W)
        send_frame = ttk.Frame(group)
        send_frame.pack(fill=tk.X, pady=5)
        self.send_entry = ttk.Entry(send_frame, state=tk.DISABLED)
        self.send_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        self.send_button = ttk.Button(send_frame, text="发送", command=self.send_message, state=tk.DISABLED)
        self.send_button.pack(side=tk.LEFT)

    def connect_tcp(self):
        """尝试建立TCP连接"""
        if self.is_connected:
            messagebox.showinfo("提示", "已连接，请勿重复操作。")
            return
        try:
            ip = self.ip_var.get()
            port = int(self.port_var.get())
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((ip, port))
            self.is_connected = True
            self.tcp_status_var.set(f"TCP状态: 已连接到 {ip}:{port}")
            self.connect_button.config(state=tk.DISABLED)
            self.disconnect_button.config(state=tk.NORMAL)
            self.send_entry.config(state=tk.NORMAL)
            self.send_button.config(state=tk.NORMAL)
            self.log_message("系统: 连接成功！")
            self.receive_thread = threading.Thread(target=self.receive_messages, daemon=True)
            self.receive_thread.start()
        except socket.gaierror:
            self.tcp_status_var.set("TCP状态: 连接失败 (IP地址错误)")
            messagebox.showerror("连接错误", "无法解析IP地址。")
        except ConnectionRefusedError:
            self.tcp_status_var.set("TCP状态: 连接失败 (连接被拒绝)")
            messagebox.showerror("连接错误", "目标计算机拒绝连接，请检查IP和端口。")
        except ValueError:
            self.tcp_status_var.set("TCP状态: 连接失败 (端口号错误)")
            messagebox.showerror("输入错误", "端口号必须是有效数字。")
        except Exception as e:
            self.tcp_status_var.set(f"TCP状态: 连接失败 ({e})")
            messagebox.showerror("连接错误", f"发生未知错误: {e}")

    def disconnect_tcp(self):
        """断开TCP连接"""
        if self.is_connected:
            self.client_socket.close()
            self.is_connected = False
            self.tcp_status_var.set("TCP状态: 已断开")
            self.connect_button.config(state=tk.NORMAL)
            self.disconnect_button.config(state=tk.DISABLED)
            self.send_entry.config(state=tk.DISABLED)
            self.send_button.config(state=tk.DISABLED)
            self.log_message("系统: 连接已断开。")

    def send_message(self):
        """发送消息"""
        if not self.is_connected:
            messagebox.showwarning("发送失败", "请先建立TCP连接。")
            return
        message = self.send_entry.get()
        if not message:
            return
        try:
            self.client_socket.sendall(message.encode('utf-8'))
            self.log_message(f"发送: {message}")
            self.send_entry.delete(0, tk.END)
        except Exception as e:
            self.log_message(f"错误: 发送失败 - {e}")
            self.disconnect_tcp()

    def receive_messages(self):
        """在独立的线程中接收消息"""
        while self.is_connected:
            try:
                data = self.client_socket.recv(1024)
                if not data:
                    self.after(0, self.disconnect_tcp)
                    break
                message = data.decode('utf-8')
                self.after(0, self.log_message, f"接收: {message}")
            except ConnectionResetError:
                self.after(0, self.log_message, "错误: 连接被远程主机强制关闭。")
                self.after(0, self.disconnect_tcp)
                break
            except Exception as e:
                if self.is_connected:
                    self.after(0, self.log_message, f"错误: 接收消息失败 - {e}")
                    self.after(0, self.disconnect_tcp)
                break

    def log_message(self, message):
        """向接收文本框中添加消息"""
        self.recv_text.config(state=tk.NORMAL)
        self.recv_text.insert(tk.END, message + "\n")
        self.recv_text.see(tk.END)
        self.recv_text.config(state=tk.DISABLED)

    def update_state_display(self):
        """更新GUI上的关节值和末端姿态显示"""
        for i, q_val in enumerate(self.current_q):
            if self.robot.links[i].isprismatic:
                self.joint_vars[i].set(f"{q_val:.2f}")
            else:
                self.joint_vars[i].set(f"{np.rad2deg(q_val):.2f}")

        T = self.robot.fkine(self.current_q)
        x, y, z = T.t
        rpy_rad = T.rpy('zyx')
        roll = np.rad2deg(rpy_rad[0])
        pitch = np.rad2deg(rpy_rad[1])
        yaw = np.rad2deg(rpy_rad[2])
        
        self.pose_vars["X"].set(f"{x:.2f}")
        self.pose_vars["Y"].set(f"{y:.2f}")
        self.pose_vars["Z"].set(f"{z:.2f}")
        self.pose_vars["Roll"].set(f"{roll:.2f}")
        self.pose_vars["Pitch"].set(f"{pitch:.2f}")
        self.pose_vars["Yaw"].set(f"{yaw:.2f}")

    def motor_adjust(self, joint_index, direction):
        """微调单个关节角度，并更新显示"""
        step = 0.5 if self.robot.links[joint_index].isprismatic else np.deg2rad(1.0)
        q_new = self.current_q[joint_index] + direction * step
        
        if self.robot.links[joint_index].qlim:
            q_min, q_max = self.robot.links[joint_index].qlim
            q_new = np.clip(q_new, q_min, q_max)
            
        self.current_q[joint_index] = q_new
        self.update_state_display()
        self.status_var.set(f"状态: 关节{joint_index+1} 微调中...")
        
    def start_move(self, joint_index, direction):
        """开始连续微调"""
        self._moving_joint_index = joint_index
        self._moving_direction = direction
        self.continuous_move()

    def stop_move(self, event):
        """停止连续微调"""
        if self._after_id:
            self.after_cancel(self._after_id)
        self._after_id = None
        self._moving_joint_index = None
        self._moving_direction = 0
        self.status_var.set("状态: 微调停止")

    def continuous_move(self):
        """连续微调函数，每隔一段时间调用一次自身"""
        if self._moving_joint_index is not None:
            self.motor_adjust(self._moving_joint_index, self._moving_direction)
            self._after_id = self.after(50, self.continuous_move)

    def run_inverse_kinematics(self):
        """执行逆运动学解算"""
        try:
            x = float(self.ik_input_vars[0].get())
            y = float(self.ik_input_vars[1].get())
            z = float(self.ik_input_vars[2].get())
            roll = np.deg2rad(float(self.ik_input_vars[3].get()))
            pitch = np.deg2rad(float(self.ik_input_vars[4].get()))
            yaw = np.deg2rad(float(self.ik_input_vars[5].get()))
            
            # 创建目标位姿矩阵
            T_target = SE3.Trans(x, y, z) * SE3.RPY(roll, pitch, yaw)
            
            q_result, *_ = self.robot.ikine_LM(T_target, q0=self.current_q)
            
            if q_result is not None:
                self.current_q = q_result.q
                self.update_state_display()
                
                result_str = "逆解成功！\n"
                for i, q_val in enumerate(self.current_q):
                    result_str += f"关节{i+1}: q={np.rad2deg(q_val):.2f}°\n"
                
                self.ik_result_var.set(result_str)
                self.status_var.set("状态: 成功执行逆解，关节值已更新。")
            else:
                self.ik_result_var.set("警告: 无法找到逆解，请检查目标位置是否可达。")
                self.status_var.set("状态: 逆解失败。")
                
        except ValueError:
            messagebox.showerror("输入错误", "请输入有效的数字！")
        except Exception as e:
            messagebox.showerror("解算错误", f"逆解过程中发生错误: {e}")

def on_closing(app):
    """窗口关闭时的清理操作"""
    if app.is_connected:
        app.disconnect_tcp()
    app.destroy()

# --- 程序主入口 ---
if __name__ == "__main__":
    app = RobotControlWindow()
    app.protocol("WM_DELETE_WINDOW", lambda: on_closing(app))
    app.mainloop()