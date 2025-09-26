# ads_toggle_ui.py
# 功能：连接到 Beckhoff（ADS），实时显示变量 MAIN.A，并提供按钮一键 0/1 切换
# 依赖：pip install pyads

import tkinter as tk
from tkinter import ttk, messagebox
import pyads
import threading
import time

# ====== 配置区（按需修改） ==========================================
AMS_NET_ID = "192.168.10.100.1.1"   # 目标PLC的 AMS NetId（务必与目标机一致）
ADS_PORT   = 851                    # 851=Runtime1, 852=Runtime2...
REMOTE_IP  = "192.168.10.100"       # 目标PLC IP

VAR_NAME   = "MAIN.A"               # PLC 变量名（例：MAIN.A）
PLCTYPE    = pyads.PLCTYPE_BOOL     # 如为 INT，请改 pyads.PLCTYPE_INT
POLL_MS    = 200                    # 轮询周期（毫秒）
# ===================================================================

class AdsClient:
    """小心处理连接的安全封装，避免析构期异常"""
    def __init__(self, ams, port, ip):
        self.ams = ams
        self.port = port
        self.ip = ip
        self.plc = None
        self.lock = threading.Lock()

    def open(self):
        with self.lock:
            if self.plc is None:
                self.plc = pyads.Connection(self.ams, self.port, self.ip)
                self.plc.open()

    def is_open(self):
        with self.lock:
            return self.plc is not None and getattr(self.plc, "_open", False)

    def read_by_name(self, name, typ):
        with self.lock:
            return self.plc.read_by_name(name, typ)

    def write_by_name(self, name, value, typ):
        with self.lock:
            self.plc.write_by_name(name, value, typ)

    def close(self):
        with self.lock:
            try:
                if self.plc is not None and getattr(self.plc, "_open", False):
                    self.plc.close()
            finally:
                self.plc = None


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ADS 变量监视与切换")
        self.geometry("360x180")
        self.resizable(False, False)

        self.client = AdsClient(AMS_NET_ID, ADS_PORT, REMOTE_IP)

        # UI
        frm = ttk.Frame(self, padding=14)
        frm.pack(fill="both", expand=True)

        self.var_status = tk.StringVar(value="未连接")
        self.var_value  = tk.StringVar(value="-")

        ttk.Label(frm, text=f"目标: {REMOTE_IP}  |  AMS: {AMS_NET_ID}  |  端口: {ADS_PORT}").pack(anchor="w")
        ttk.Label(frm, text=f"变量: {VAR_NAME}（{'BOOL' if PLCTYPE==pyads.PLCTYPE_BOOL else 'INT'}）").pack(anchor="w", pady=(4,0))

        row = ttk.Frame(frm)
        row.pack(fill="x", pady=8)
        ttk.Label(row, text="当前值：", width=10).pack(side="left")
        self.lbl_val = ttk.Label(row, textvariable=self.var_value, font=("Consolas", 16))
        self.lbl_val.pack(side="left")

        btn_row = ttk.Frame(frm)
        btn_row.pack(fill="x", pady=8)
        self.btn_toggle = ttk.Button(btn_row, text="切换 0⇄1", command=self.on_toggle, width=16, state="disabled")
        self.btn_toggle.pack(side="left", padx=(0,8))
        self.btn_reconnect = ttk.Button(btn_row, text="重连", command=self.try_connect, width=10)
        self.btn_reconnect.pack(side="left")

        ttk.Label(frm, textvariable=self.var_status, foreground="#666").pack(anchor="w", pady=(8,0))

        # 连接并启动轮询
        self.after(50, self.try_connect)
        self.after(POLL_MS, self.poll)

        # 关闭处理
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def try_connect(self):
        try:
            if not self.client.is_open():
                self.client.open()
            self.var_status.set("已连接")
            self.btn_toggle.config(state="normal" if PLCTYPE==pyads.PLCTYPE_BOOL else "normal")
        except Exception as e:
            self.var_status.set(f"连接失败：{e}")
            self.btn_toggle.config(state="disabled")

    def poll(self):
        """定时读取变量并刷新UI"""
        try:
            if self.client.is_open():
                val = self.client.read_by_name(VAR_NAME, PLCTYPE)
                self.var_value.set(str(int(val)) if PLCTYPE==pyads.PLCTYPE_BOOL else str(val))
                self.var_status.set("在线")
                self.btn_toggle.config(state="normal")
            else:
                self.var_status.set("未连接")
                self.btn_toggle.config(state="disabled")
        except Exception as e:
            # 读失败：显示错误，按钮禁用
            self.var_status.set(f"读失败：{e}")
            self.btn_toggle.config(state="disabled")
        finally:
            self.after(POLL_MS, self.poll)

    def on_toggle(self):
        """若 A 为 0 写 1；若 A 为 1 写 0"""
        try:
            if not self.client.is_open():
                self.var_status.set("未连接，无法写入")
                return
            cur = self.client.read_by_name(VAR_NAME, PLCTYPE)
            if PLCTYPE == pyads.PLCTYPE_BOOL:
                new_val = not bool(cur)
            else:
                # INT 的话：0/非0 在 0⇄1 之间切换
                new_val = 0 if int(cur) != 0 else 1
            self.client.write_by_name(VAR_NAME, new_val, PLCTYPE)
            # 立即刷新一次
            now = self.client.read_by_name(VAR_NAME, PLCTYPE)
            self.var_value.set(str(int(now)) if PLCTYPE==pyads.PLCTYPE_BOOL else str(now))
            self.var_status.set("写入成功")
        except Exception as e:
            self.var_status.set(f"写失败：{e}")

    def on_close(self):
        try:
            self.client.close()
        finally:
            self.destroy()


if __name__ == "__main__":
    # 可选：先验证本机 AMS（不强制）
    try:
        pyads.open_port()
        addr = pyads.get_local_address()
        # 打印一次，方便排错；出现异常也不阻塞UI
        netid = str(getattr(addr, "netId", None) or getattr(addr, "amsNetId", None))
        print("Local AMS:", netid, "ADS port:", getattr(addr, "port", None))
    except Exception as e:
        print("本机ADS初始化警告：", e)
    App().mainloop()
