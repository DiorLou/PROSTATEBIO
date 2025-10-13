# ads_move_start_hold.py
# -*- coding: utf-8 -*-
import tkinter as tk
from tkinter import ttk
import threading
import time
import pyads

# ====== 连接参数（按需修改） ======
AMS_NET_ID = "192.168.10.100.1.1"
PLC_IP     = "192.168.10.100"
PLC_PORT   = 851

# ====== 变量名（按需修改） ======
POS1_PV = "MAIN.MotorCmdPos[2]"   # 只读：当前位置1 (LREAL)
POS2_PV = "MAIN.MotorCmdPos[3]"   # 只读：当前位置2 (LREAL)

J2_PV   = "MAIN.J2"               # 可写：目标位1 (LREAL)
J3_PV   = "MAIN.J3"               # 可写：目标位2 (LREAL)

START_BOOL = "MAIN.Position_5"    # 运动使能：需要持续 True，完成后置回 False

# ====== 目标与判据 ======
DEFAULT_T1 = 5.00
DEFAULT_T2 = 9.00
DONE_TOL   = 0.5                  # 判定完成阈值
READ_T     = pyads.PLCTYPE_LREAL
WRITE_T    = pyads.PLCTYPE_LREAL

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


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ADS 双轴目标保持启动示例（J2/J3 + Position_5）")
        self.geometry("560x310+0+0")

        self.ads = ADS(AMS_NET_ID, PLC_PORT, PLC_IP)

        # UI 变量
        self.pos1_var   = tk.StringVar(value="--")
        self.pos2_var   = tk.StringVar(value="--")
        self.target1_sv = tk.StringVar(value=f"{DEFAULT_T1:.2f}")
        self.target2_sv = tk.StringVar(value=f"{DEFAULT_T2:.2f}")
        self.status_sv  = tk.StringVar(value="未连接")

        # 线程控制
        self.poll_stop = threading.Event()
        self.poll_thread = None
        self.moving = False
        self._stable_cnt = 0

        self._build_ui()

    def _build_ui(self):
        root = ttk.Frame(self, padding=10)
        root.pack(fill=tk.BOTH, expand=True)

        top = ttk.Frame(root)
        top.pack(fill=tk.X, pady=(0,8))
        self.conn_btn = ttk.Button(top, text="连接 ADS", command=self.toggle_conn)
        self.conn_btn.pack(side=tk.LEFT)
        ttk.Label(top, textvariable=self.status_sv, width=45, anchor="w").pack(side=tk.LEFT, padx=10)

        grid = ttk.Frame(root)
        grid.pack(fill=tk.X, pady=8)

        ttk.Label(grid, text="Pos1  (MotorCmdPos[2]):").grid(row=0, column=0, sticky="e", padx=6, pady=6)
        ttk.Label(grid, textvariable=self.pos1_var, width=12).grid(row=0, column=1, sticky="w")

        ttk.Label(grid, text="Pos2  (MotorCmdPos[3]):").grid(row=1, column=0, sticky="e", padx=6, pady=6)
        ttk.Label(grid, textvariable=self.pos2_var, width=12).grid(row=1, column=1, sticky="w")

        ttk.Label(grid, text="目标1 (J2):").grid(row=0, column=2, sticky="e", padx=10)
        self.t1_entry = ttk.Entry(grid, textvariable=self.target1_sv, width=10, justify="center")
        self.t1_entry.grid(row=0, column=3, sticky="w")

        ttk.Label(grid, text="目标2 (J3):").grid(row=1, column=2, sticky="e", padx=10)
        self.t2_entry = ttk.Entry(grid, textvariable=self.target2_sv, width=10, justify="center")
        self.t2_entry.grid(row=1, column=3, sticky="w")

        btns = ttk.Frame(root)
        btns.pack(fill=tk.X, pady=10)

        self.start_btn = ttk.Button(btns, text="开始运动（保持 Position_5=True）", command=self.start_move, state=tk.DISABLED)
        self.start_btn.pack(side=tk.LEFT, padx=5)

        self.quit_btn = ttk.Button(btns, text="退出", command=self.on_close)
        self.quit_btn.pack(side=tk.RIGHT, padx=5)

    # 连接/断开
    def toggle_conn(self):
        if not self.ads.connected:
            ok, msg = self.ads.open()
            self.status_sv.set(msg)
            if ok:
                self.conn_btn.config(text="断开 ADS")
                self.start_btn.config(state=tk.NORMAL)
                self._start_poll()
            else:
                self.conn_btn.config(text="连接 ADS")
                self.start_btn.config(state=tk.DISABLED)
        else:
            # 断开前，尽量把 Position_5 拉回 False，防止悬挂
            try:
                self.ads.write_bool(START_BOOL, False)
            except Exception:
                pass
            self._stop_poll()
            self.ads.close()
            self.status_sv.set("已断开")
            self.conn_btn.config(text="连接 ADS")
            self.start_btn.config(state=tk.DISABLED)

    # 实时轮询读取
    def _start_poll(self):
        self.poll_stop.clear()
        def worker():
            last_status = ""
            while not self.poll_stop.is_set() and self.ads.connected:
                try:
                    p1 = self.ads.read_lreal(POS1_PV)
                    p2 = self.ads.read_lreal(POS2_PV)
                    self._safe_set(self.pos1_var, f"{p1:.3f}")
                    self._safe_set(self.pos2_var, f"{p2:.3f}")

                    if self.moving:
                        # 判定到位（两轴都在阈值内，连续两次）
                        t1, t2 = self._get_targets()
                        done1 = abs(p1 - t1) <= DONE_TOL
                        done2 = abs(p2 - t2) <= DONE_TOL
                        if done1 and done2:
                            self._stable_cnt += 1
                        else:
                            self._stable_cnt = 0

                        if self._stable_cnt >= 2:
                            # 到位：立即把 Position_5 置回 False，并结束
                            try:
                                self.ads.write_bool(START_BOOL, False)
                            except Exception as e:
                                self._safe_set(self.status_sv, f"置位回落失败：{e}")
                            self.moving = False
                            self._stable_cnt = 0
                            self._safe_set(self.status_sv, "运动完成")
                        else:
                            if last_status != "运动中":
                                self._safe_set(self.status_sv, "运动中")
                                last_status = "运动中"
                    else:
                        if last_status != "就绪":
                            self._safe_set(self.status_sv, "就绪")
                            last_status = "就绪"

                except Exception as e:
                    self._safe_set(self.status_sv, f"读失败: {e}")
                time.sleep(0.15)
        self.poll_thread = threading.Thread(target=worker, daemon=True)
        self.poll_thread.start()

    def _stop_poll(self):
        self.poll_stop.set()
        if self.poll_thread and self.poll_thread.is_alive():
            try:
                self.poll_thread.join(timeout=0.5)
            except Exception:
                pass
        self.poll_thread = None

    def _safe_set(self, var, text):
        try:
            self.after(0, lambda: var.set(text))
        except Exception:
            pass

    def _get_targets(self):
        try:
            t1 = float(self.target1_sv.get())
        except Exception:
            t1 = DEFAULT_T1
            self._safe_set(self.target1_sv, f"{t1:.2f}")
        try:
            t2 = float(self.target2_sv.get())
        except Exception:
            t2 = DEFAULT_T2
            self._safe_set(self.target2_sv, f"{t2:.2f}")
        return t1, t2

    # 开始运动：写 J2/J3，然后将 Position_5 置 True（保持）
    def start_move(self):
        if not self.ads.connected:
            self.status_sv.set("未连接")
            return
        if self.moving:
            self.status_sv.set("正在运动中…")
            return

        t1, t2 = self._get_targets()

        def job():
            try:
                self._safe_set(self.status_sv, "下发目标中…")
                self.ads.write_lreal(J2_PV, t1)
                self.ads.write_lreal(J3_PV, t2)
                # 保持 True，让 PLC 执行运动；到位后轮询线程会自动 False
                self.ads.write_bool(START_BOOL, True)
                self.moving = True
                self._stable_cnt = 0
                self._safe_set(self.status_sv, "运动中")
            except Exception as e:
                self._safe_set(self.status_sv, f"下发失败：{e}")
        threading.Thread(target=job, daemon=True).start()

    def on_close(self):
        try:
            # 收尾：尽量拉低启动位
            if self.ads.connected:
                try:
                    self.ads.write_bool(START_BOOL, False)
                except Exception:
                    pass
            self._stop_poll()
            self.ads.close()
        finally:
            self.destroy()

if __name__ == "__main__":
    app = App()
    app.mainloop()
