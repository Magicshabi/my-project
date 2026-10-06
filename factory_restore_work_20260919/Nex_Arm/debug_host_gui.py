#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import queue
import struct
import threading
import time
import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk

import serial
import serial.tools.list_ports


# ---------------- Protocol constants ----------------
FRAME_HEADER_1 = 0xFF
FRAME_HEADER_2 = 0xFF
TARGET_ID = 0xFF

CMD_ARM_SERVO_SINGLE = 51
CMD_GET_CUR_COORDS = 11
CMD_COORDINATE_SET = 8
CMD_MOVE_INC = 82
CMD_SET_PID_PARAM = 59
CMD_GET_PID_PARAM = 60
CMD_SERVO_READ_OVERLOAD = 71
CMD_SERVO_WRITE_OVERLOAD = 72
CMD_SERVO_READ_BAUD = 73
CMD_SERVO_WRITE_BAUD = 74
CMD_SERVO_READ_MAX_TORQUE = 75
CMD_SERVO_WRITE_MAX_TORQUE = 76
CMD_SERVO_CALI_POS = 88

SERVO_IDS = [1, 2, 3, 4, 5, 6]
BAUD_CODE_TO_BPS = {
    0: 1000000,
    1: 500000,
    2: 250000,
    4: 115200,
    5: 76800,
    6: 57600,
    7: 38400,
}
BAUD_BPS_TO_CODE = {v: k for k, v in BAUD_CODE_TO_BPS.items()}

DEFAULT_CENTER_POS = 2048
DEFAULT_MOVE_TIME_MS = 800
SERIAL_BAUD_HOST = 1000000
CONFIG_FILE = "debug_host_gui_config.json"


def checksum_inv8(data_bytes):
    return (~sum(data_bytes)) & 0xFF


class NexArmSerialClient:
    def __init__(self):
        self._ser = None
        self._rx_thread = None
        self._stop_event = threading.Event()
        self._write_lock = threading.Lock()
        self._queue_lock = threading.Lock()
        self._cmd_queues = {}
        self._buffer = bytearray()
        self._frame_callback = None

    def set_frame_callback(self, cb):
        self._frame_callback = cb

    def is_open(self):
        return self._ser is not None and self._ser.is_open

    def open(self, port, baudrate=SERIAL_BAUD_HOST, timeout=0.02):
        self.close()
        self._ser = serial.Serial(port, baudrate, timeout=timeout)
        self._stop_event.clear()
        self._rx_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._rx_thread.start()

    def close(self):
        self._stop_event.set()
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=0.3)
        self._rx_thread = None

        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        self._buffer = bytearray()
        with self._queue_lock:
            self._cmd_queues = {}

    def send(self, cmd, args=None, target_id=TARGET_ID):
        if args is None:
            args = []
        if not self.is_open():
            raise RuntimeError("串口未连接")

        length = 2 + len(args)
        body = [target_id, length, cmd] + list(args)
        chk = checksum_inv8(body)
        frame = bytes([FRAME_HEADER_1, FRAME_HEADER_2] + body + [chk])

        with self._write_lock:
            self._ser.write(frame)
        return frame

    def request(self, cmd, args=None, timeout=0.5, predicate=None, retries=1, target_id=TARGET_ID):
        if args is None:
            args = []
        for _ in range(max(1, retries)):
            self.clear_cmd_queue(cmd)
            self.send(cmd, args, target_id=target_id)
            frame = self.wait_response(cmd, timeout=timeout, predicate=predicate)
            if frame is not None:
                return frame
        return None

    def wait_response(self, cmd, timeout=0.5, predicate=None):
        q = self._get_cmd_queue(cmd)
        deadline = time.time() + timeout
        while time.time() < deadline:
            remain = max(0.001, deadline - time.time())
            try:
                frame = q.get(timeout=remain)
            except queue.Empty:
                return None
            if predicate is None or predicate(frame):
                return frame
        return None

    def clear_cmd_queue(self, cmd):
        q = self._get_cmd_queue(cmd)
        while True:
            try:
                q.get_nowait()
            except queue.Empty:
                break

    def _get_cmd_queue(self, cmd):
        with self._queue_lock:
            q = self._cmd_queues.get(cmd)
            if q is None:
                q = queue.Queue()
                self._cmd_queues[cmd] = q
            return q

    def _emit_frame(self, frame):
        q = self._get_cmd_queue(frame["cmd"])
        q.put(frame)
        if self._frame_callback:
            try:
                self._frame_callback(frame)
            except Exception:
                pass

    def _reader_loop(self):
        while not self._stop_event.is_set():
            try:
                if not self._ser:
                    time.sleep(0.01)
                    continue

                n = self._ser.in_waiting
                if n <= 0:
                    data = self._ser.read(1)
                else:
                    data = self._ser.read(n)
                if not data:
                    continue
                self._buffer.extend(data)
                self._parse_buffer()
            except Exception as e:
                if self._frame_callback:
                    self._frame_callback(
                        {
                            "id": 0x00,
                            "cmd": 0x00,
                            "args": [],
                            "length": 0,
                            "raw": b"",
                            "error": str(e),
                        }
                    )
                time.sleep(0.03)

    def _parse_buffer(self):
        while True:
            if len(self._buffer) < 6:
                return

            if self._buffer[0] != FRAME_HEADER_1 or self._buffer[1] != FRAME_HEADER_2:
                del self._buffer[0]
                continue

            length = self._buffer[3]
            if length < 2 or length > 250:
                del self._buffer[0]
                continue

            total_len = length + 4
            if len(self._buffer) < total_len:
                return

            raw = bytes(self._buffer[:total_len])
            del self._buffer[:total_len]

            body = raw[2:-1]
            chk = raw[-1]
            if checksum_inv8(body) != chk:
                if self._frame_callback:
                    self._frame_callback(
                        {
                            "id": raw[2],
                            "cmd": raw[4],
                            "args": list(raw[5:-1]) if length > 2 else [],
                            "length": length,
                            "raw": raw,
                            "error": "checksum mismatch",
                        }
                    )
                continue

            args_len = length - 2
            args = list(raw[5 : 5 + args_len]) if args_len > 0 else []
            frame = {
                "id": raw[2],
                "length": length,
                "cmd": raw[4],
                "args": args,
                "raw": raw,
                "ts": time.time(),
            }
            self._emit_frame(frame)


class DebugHostApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("调试上位机")
        self.geometry("1250x820")

        self.client = NexArmSerialClient()
        self.client.set_frame_callback(self._on_frame_rx_from_thread)

        self.ui_queue = queue.Queue()
        self.scanned_baud = {}
        self.servo_widgets = {}
        self._busy = False

        self._cfg = self._load_config()
        self._build_ui()
        self.refresh_ports()
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(50, self._poll_ui_queue)

    # ---------------- Config ----------------
    def _load_config(self):
        cfg = {
            "pid_p": 32,
            "pid_i": 0,
            "pid_d": 20,
            "overload_torque": 80,
            "overload_time": 20,
            "coord_x": 200,
            "coord_y": 0,
            "coord_z": 200,
            "coord_pitch": 0,
            "coord_roll": 0,
            "coord_claw": 0,
            "coord_time": 800,
            "servo_pos": {str(i): DEFAULT_CENTER_POS for i in SERVO_IDS},
            "servo_time": {str(i): DEFAULT_MOVE_TIME_MS for i in SERVO_IDS},
        }
        if os.path.exists(CONFIG_FILE):
            try:
                with open(CONFIG_FILE, "r", encoding="utf-8") as f:
                    loaded = json.load(f)
                cfg.update(loaded)
            except Exception:
                pass
        return cfg

    def _save_config(self):
        try:
            cfg = {
                "pid_p": int(self.var_pid_p.get()),
                "pid_i": int(self.var_pid_i.get()),
                "pid_d": int(self.var_pid_d.get()),
                "overload_torque": int(self.var_ov_torque.get()),
                "overload_time": int(self.var_ov_time.get()),
                "coord_x": int(self.var_coord_x.get()),
                "coord_y": int(self.var_coord_y.get()),
                "coord_z": int(self.var_coord_z.get()),
                "coord_pitch": int(self.var_coord_pitch.get()),
                "coord_roll": int(self.var_coord_roll.get()),
                "coord_claw": int(self.var_coord_claw.get()),
                "coord_time": int(self.var_coord_time.get()),
                "servo_pos": {},
                "servo_time": {},
            }
            for sid in SERVO_IDS:
                cfg["servo_pos"][str(sid)] = int(self.servo_widgets[sid]["pos"].get())
                cfg["servo_time"][str(sid)] = int(self.servo_widgets[sid]["time"].get())
            with open(CONFIG_FILE, "w", encoding="utf-8") as f:
                json.dump(cfg, f, ensure_ascii=False, indent=2)
        except Exception as e:
            self.log(f"[WARN] 保存本地配置失败: {e}")

    # ---------------- UI ----------------
    def _build_ui(self):
        root = ttk.Frame(self, padding=8)
        root.pack(fill="both", expand=True)

        # Serial connect row
        conn = ttk.LabelFrame(root, text="串口连接")
        conn.pack(fill="x", pady=4)
        ttk.Label(conn, text="端口").pack(side="left", padx=4)
        self.port_cb = ttk.Combobox(conn, width=20, state="readonly")
        self.port_cb.pack(side="left", padx=4)
        ttk.Button(conn, text="刷新", command=self.refresh_ports).pack(side="left", padx=4)
        self.btn_connect = ttk.Button(conn, text="连接", command=self.toggle_connect)
        self.btn_connect.pack(side="left", padx=8)
        ttk.Label(conn, text=f"上位机串口波特率固定: {SERIAL_BAUD_HOST}").pack(side="left", padx=10)
        self.lbl_status = ttk.Label(conn, text="未连接")
        self.lbl_status.pack(side="right", padx=8)

        # Utility row
        tools = ttk.LabelFrame(root, text="快速操作")
        tools.pack(fill="x", pady=4)
        self.btn_recover_baud = ttk.Button(tools, text="一键恢复全部舵机波特率为1M", command=self.recover_all_baud_1m)
        self.btn_recover_baud.pack(side="left", padx=4, pady=4)
        self.btn_scan = ttk.Button(tools, text="读取当前波特率(1~6)", command=self.scan_all_baud)
        self.btn_scan.pack(side="left", padx=4, pady=4)
        self.btn_set_mid = ttk.Button(tools, text="设置中位(1~6)", command=self.set_middle_all)
        self.btn_set_mid.pack(side="left", padx=4, pady=4)
        self.btn_restore_mid = ttk.Button(tools, text="恢复中位(1~6)", command=self.restore_middle_all)
        self.btn_restore_mid.pack(side="left", padx=4, pady=4)
        self.lbl_scan = ttk.Label(tools, text="扫描结果: -")
        self.lbl_scan.pack(side="right", padx=8)

        # Param row
        params = ttk.LabelFrame(root, text="参数批量设置(1~6)")
        params.pack(fill="x", pady=4)

        ttk.Label(params, text="PID P").grid(row=0, column=0, padx=4, pady=4, sticky="e")
        self.var_pid_p = tk.IntVar(value=int(self._cfg.get("pid_p", 32)))
        ttk.Spinbox(params, from_=0, to=255, textvariable=self.var_pid_p, width=6).grid(row=0, column=1, padx=4, pady=4)

        ttk.Label(params, text="PID I").grid(row=0, column=2, padx=4, pady=4, sticky="e")
        self.var_pid_i = tk.IntVar(value=int(self._cfg.get("pid_i", 0)))
        ttk.Spinbox(params, from_=0, to=255, textvariable=self.var_pid_i, width=6).grid(row=0, column=3, padx=4, pady=4)

        ttk.Label(params, text="PID D").grid(row=0, column=4, padx=4, pady=4, sticky="e")
        self.var_pid_d = tk.IntVar(value=int(self._cfg.get("pid_d", 20)))
        ttk.Spinbox(params, from_=0, to=255, textvariable=self.var_pid_d, width=6).grid(row=0, column=5, padx=4, pady=4)

        self.btn_set_pid = ttk.Button(params, text="一键设置PID(1~6)", command=self.set_pid_all)
        self.btn_set_pid.grid(row=0, column=6, padx=8, pady=4)

        ttk.Label(params, text="最小启动力(过载阈值)").grid(row=1, column=0, padx=4, pady=4, sticky="e")
        self.var_ov_torque = tk.IntVar(value=int(self._cfg.get("overload_torque", 80)))
        ttk.Spinbox(params, from_=0, to=255, textvariable=self.var_ov_torque, width=6).grid(
            row=1, column=1, padx=4, pady=4
        )
        ttk.Label(params, text="保护时间").grid(row=1, column=2, padx=4, pady=4, sticky="e")
        self.var_ov_time = tk.IntVar(value=int(self._cfg.get("overload_time", 20)))
        ttk.Spinbox(params, from_=0, to=255, textvariable=self.var_ov_time, width=6).grid(
            row=1, column=3, padx=4, pady=4
        )
        self.btn_set_ov = ttk.Button(params, text="一键设置最小启动力(1~6)", command=self.set_overload_all)
        self.btn_set_ov.grid(row=1, column=6, padx=8, pady=4)

        ttk.Button(params, text="读取PID(单个)", command=self.get_pid_one).grid(row=0, column=7, padx=8, pady=4)
        ttk.Button(params, text="读取最小启动力(单个)", command=self.get_overload_one).grid(row=1, column=7, padx=8, pady=4)
        ttk.Button(params, text="设置PID(单个)", command=self.set_pid_one).grid(row=0, column=8, padx=8, pady=4)
        ttk.Button(params, text="设置最小启动力(单个)", command=self.set_overload_one).grid(row=1, column=8, padx=8, pady=4)
        self.var_read_sid = tk.IntVar(value=1)
        ttk.Spinbox(params, from_=1, to=6, textvariable=self.var_read_sid, width=4).grid(row=0, column=9, rowspan=2, padx=4, pady=4)

        # Servo control table
        servo_frame = ttk.LabelFrame(root, text="单舵机控制(1~6)")
        servo_frame.pack(fill="x", pady=4)

        headers = ["舵机ID", "目标位置", "时间(ms)", "动作", "回中位", "读位置"]
        for col, title in enumerate(headers):
            ttk.Label(servo_frame, text=title).grid(row=0, column=col, padx=6, pady=4)

        for row, sid in enumerate(SERVO_IDS, start=1):
            ttk.Label(servo_frame, text=str(sid)).grid(row=row, column=0, padx=6, pady=4)

            pos_var = tk.IntVar(value=int(self._cfg.get("servo_pos", {}).get(str(sid), DEFAULT_CENTER_POS)))
            time_var = tk.IntVar(value=int(self._cfg.get("servo_time", {}).get(str(sid), DEFAULT_MOVE_TIME_MS)))

            pos_sp = ttk.Spinbox(servo_frame, from_=0, to=4095, textvariable=pos_var, width=8)
            pos_sp.grid(row=row, column=1, padx=6, pady=4)

            time_sp = ttk.Spinbox(servo_frame, from_=20, to=60000, textvariable=time_var, width=8)
            time_sp.grid(row=row, column=2, padx=6, pady=4)

            ttk.Button(servo_frame, text="执行", command=lambda i=sid: self.move_single_servo(i)).grid(
                row=row, column=3, padx=6, pady=4
            )
            ttk.Button(servo_frame, text="回中位", command=lambda i=sid: self.move_servo_to_center(i)).grid(
                row=row, column=4, padx=6, pady=4
            )
            ttk.Button(servo_frame, text="读位置", command=lambda i=sid: self.read_single_position(i)).grid(
                row=row, column=5, padx=6, pady=4
            )

            self.servo_widgets[sid] = {
                "pos": pos_var,
                "time": time_var,
            }

        # Log output
        log_frame = ttk.LabelFrame(root, text="日志")
        log_frame.pack(fill="both", expand=True, pady=4)
        self.text_log = scrolledtext.ScrolledText(log_frame, height=20)
        self.text_log.pack(fill="both", expand=True, padx=4, pady=4)

        # Coordinate debug
        coord_frame = ttk.LabelFrame(root, text="坐标调试")
        coord_frame.pack(fill="x", pady=4)

        self.var_coord_x = tk.IntVar(value=int(self._cfg.get("coord_x", 200)))
        self.var_coord_y = tk.IntVar(value=int(self._cfg.get("coord_y", 0)))
        self.var_coord_z = tk.IntVar(value=int(self._cfg.get("coord_z", 200)))
        self.var_coord_pitch = tk.IntVar(value=int(self._cfg.get("coord_pitch", 0)))
        self.var_coord_roll = tk.IntVar(value=int(self._cfg.get("coord_roll", 0)))
        self.var_coord_claw = tk.IntVar(value=int(self._cfg.get("coord_claw", 0)))
        self.var_coord_time = tk.IntVar(value=int(self._cfg.get("coord_time", 800)))

        coord_fields = [
            ("X", self.var_coord_x, 0),
            ("Y", self.var_coord_y, 2),
            ("Z", self.var_coord_z, 4),
            ("Pitch", self.var_coord_pitch, 6),
            ("Roll", self.var_coord_roll, 8),
            ("Claw", self.var_coord_claw, 10),
            ("Time(ms)", self.var_coord_time, 12),
        ]
        for label, var, col in coord_fields:
            ttk.Label(coord_frame, text=label).grid(row=0, column=col, padx=4, pady=4, sticky="e")
            ttk.Spinbox(coord_frame, textvariable=var, width=8).grid(row=0, column=col + 1, padx=4, pady=4)

        ttk.Button(coord_frame, text="绝对发送", command=self.send_coord_absolute).grid(row=1, column=0, padx=4, pady=4)
        ttk.Button(coord_frame, text="增量发送", command=self.send_coord_increment_all).grid(row=1, column=1, padx=4, pady=4)
        ttk.Button(coord_frame, text="X+", command=lambda: self.move_coord_axis("x", 1)).grid(row=1, column=2, padx=3, pady=4)
        ttk.Button(coord_frame, text="X-", command=lambda: self.move_coord_axis("x", -1)).grid(row=1, column=3, padx=3, pady=4)
        ttk.Button(coord_frame, text="Y+", command=lambda: self.move_coord_axis("y", 1)).grid(row=1, column=4, padx=3, pady=4)
        ttk.Button(coord_frame, text="Y-", command=lambda: self.move_coord_axis("y", -1)).grid(row=1, column=5, padx=3, pady=4)
        ttk.Button(coord_frame, text="Z+", command=lambda: self.move_coord_axis("z", 1)).grid(row=1, column=6, padx=3, pady=4)
        ttk.Button(coord_frame, text="Z-", command=lambda: self.move_coord_axis("z", -1)).grid(row=1, column=7, padx=3, pady=4)
        ttk.Button(coord_frame, text="P+", command=lambda: self.move_coord_axis("pitch", 1)).grid(row=1, column=8, padx=3, pady=4)
        ttk.Button(coord_frame, text="P-", command=lambda: self.move_coord_axis("pitch", -1)).grid(row=1, column=9, padx=3, pady=4)
        ttk.Button(coord_frame, text="R+", command=lambda: self.move_coord_axis("roll", 1)).grid(row=1, column=10, padx=3, pady=4)
        ttk.Button(coord_frame, text="R-", command=lambda: self.move_coord_axis("roll", -1)).grid(row=1, column=11, padx=3, pady=4)
        ttk.Button(coord_frame, text="C+", command=lambda: self.move_coord_axis("claw", 1)).grid(row=1, column=12, padx=3, pady=4)
        ttk.Button(coord_frame, text="C-", command=lambda: self.move_coord_axis("claw", -1)).grid(row=1, column=13, padx=3, pady=4)

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports:
            self.port_cb.current(0)

    def toggle_connect(self):
        if self.client.is_open():
            self.client.close()
            self.btn_connect.config(text="连接")
            self.lbl_status.config(text="未连接")
            self.log("串口已断开")
            return

        port = self.port_cb.get().strip()
        if not port:
            messagebox.showwarning("提示", "请先选择串口")
            return

        try:
            self.client.open(port, baudrate=SERIAL_BAUD_HOST)
        except Exception as e:
            messagebox.showerror("连接失败", str(e))
            return

        self.btn_connect.config(text="断开")
        self.lbl_status.config(text=f"已连接: {port}")
        self.log(f"已连接 {port} @ {SERIAL_BAUD_HOST}")

    # ---------------- Background helper ----------------
    def run_task(self, name, fn):
        if self._busy:
            messagebox.showinfo("提示", "当前有任务执行中，请稍候")
            return

        if not self.client.is_open():
            messagebox.showwarning("提示", "请先连接串口")
            return

        self._busy = True
        self.log(f"[TASK] {name} 开始")

        def worker():
            try:
                fn()
                self.ui_queue.put(("task_done", f"[TASK] {name} 完成"))
            except Exception as e:
                self.ui_queue.put(("task_done", f"[TASK] {name} 异常: {e}"))

        threading.Thread(target=worker, daemon=True).start()

    # ---------------- RX frame callback (thread) ----------------
    def _on_frame_rx_from_thread(self, frame):
        self.ui_queue.put(("frame", frame))

    # ---------------- UI queue poll ----------------
    def _poll_ui_queue(self):
        while True:
            try:
                item_type, payload = self.ui_queue.get_nowait()
            except queue.Empty:
                break

            if item_type == "frame":
                self._handle_frame_ui(payload)
            elif item_type == "task_done":
                self._busy = False
                self.log(payload)
                self._save_config()
            elif item_type == "scan_label":
                self.lbl_scan.config(text=payload)
            elif item_type == "log":
                self._append_log(payload)
        self.after(50, self._poll_ui_queue)

    def _handle_frame_ui(self, frame):
        if frame.get("error"):
            self.log(f"[RX-ERR] {frame.get('error')}")
            return

        cmd = frame["cmd"]
        if cmd == CMD_GET_CUR_COORDS:
            return

        args = frame["args"]
        raw_hex = frame["raw"].hex(" ").upper()
        self.log(f"[RX] cmd=0x{cmd:02X} args={args} raw={raw_hex}")

    # ---------------- Logging ----------------
    def log(self, text):
        if threading.current_thread() is not threading.main_thread():
            self.ui_queue.put(("log", text))
            return
        self._append_log(text)

    def _append_log(self, text):
        ts = time.strftime("%H:%M:%S")
        self.text_log.insert("end", f"[{ts}] {text}\n")
        self.text_log.see("end")

    # ---------------- Helpers ----------------
    @staticmethod
    def _hex_list(data):
        return " ".join(f"{b:02X}" for b in data)

    @staticmethod
    def _match_servo_id_in_args(args, sid):
        return sid in args if args else False

    def _send_and_log(self, cmd, args):
        frame = self.client.send(cmd, args)
        self.log(f"[TX] cmd=0x{cmd:02X} args={args} raw={frame.hex(' ').upper()}")

    def _request_with_log(self, cmd, args, timeout=0.5, predicate=None, retries=1):
        tx_len = 2 + len(args)
        tx = [TARGET_ID, tx_len, cmd] + list(args)
        chk = checksum_inv8(tx)
        self.log(f"[TX] cmd=0x{cmd:02X} args={args} raw={self._hex_list([FRAME_HEADER_1, FRAME_HEADER_2] + tx + [chk])}")
        return self.client.request(cmd, args, timeout=timeout, predicate=predicate, retries=retries)

    @staticmethod
    def _parse_baud_code_from_response(servo_id, args):
        if not args:
            return None
        if len(args) == 1:
            return args[0] if args[0] in BAUD_CODE_TO_BPS else None

        # Most likely: [id, baud]
        if args[0] == servo_id and args[1] in BAUD_CODE_TO_BPS:
            return args[1]
        # Some firmware might return [baud, id]
        if args[1] == servo_id and args[0] in BAUD_CODE_TO_BPS:
            return args[0]

        # Fallback: pick first byte that looks like known baud code
        for b in args:
            if b in BAUD_CODE_TO_BPS:
                return b
        return None

    # ---------------- Requested features ----------------
    def scan_all_baud(self):
        def task():
            results = {}
            for sid in SERVO_IDS:
                frame = self._request_with_log(
                    CMD_SERVO_READ_BAUD,
                    [sid],
                    timeout=0.6,
                    predicate=lambda f, s=sid: self._match_servo_id_in_args(f["args"], s) or len(f["args"]) >= 1,
                    retries=2,
                )

                if frame is None:
                    results[sid] = None
                    self.log(f"[SCAN] 舵机 {sid}: 未响应")
                    continue

                code = self._parse_baud_code_from_response(sid, frame["args"])
                results[sid] = code
                if code is None:
                    self.log(f"[SCAN] 舵机 {sid}: 收到响应但无法解析波特率, args={frame['args']}")
                else:
                    self.log(f"[SCAN] 舵机 {sid}: 波特率码={code}, 波特率={BAUD_CODE_TO_BPS[code]}")
                time.sleep(0.05)

            self.scanned_baud = results
            summary = []
            for sid in SERVO_IDS:
                code = results.get(sid)
                if code is None:
                    summary.append(f"{sid}:--")
                else:
                    summary.append(f"{sid}:{BAUD_CODE_TO_BPS[code]}")
            self.log("[SCAN] " + ", ".join(summary))
            self.ui_queue.put(("scan_label", "扫描结果: " + ", ".join(summary)))

        self.run_task("读取当前波特率", task)

    def recover_all_baud_1m(self):
        def task():
            target_code = BAUD_BPS_TO_CODE[1000000]
            frame = self._request_with_log(
                CMD_SERVO_WRITE_BAUD,
                [0xFE, target_code],
                timeout=10.0,
                retries=1,
                predicate=lambda f: len(f["args"]) >= 3 and f["args"][0] == 0xFE,
            )

            if frame is None:
                self.log("[RECOVER] AT32未返回结果，请确认ESP32/AT32已烧录最新固件")
                return

            mask = frame["args"][2]
            recovered = [str(sid) for sid in SERVO_IDS if mask & (1 << (sid - 1))]
            if recovered:
                self.log("[RECOVER] 已恢复到1M的舵机: " + ", ".join(recovered))
            else:
                self.log("[RECOVER] 未检测到可恢复的舵机")
            self.log("[RECOVER] AT32舵机总线已回到1M，可点击读取当前波特率确认")

        self.run_task("一键恢复全部舵机波特率为1M", task)

    def set_middle_all(self):
        def task():
            for sid in SERVO_IDS:
                # Assumed payload format: [servo_id]
                self._send_and_log(CMD_SERVO_CALI_POS, [sid])
                self.log(f"[MID-SET] 舵机 {sid}: 已下发设置中位")
                time.sleep(0.08)

        self.run_task("设置中位", task)

    def restore_middle_all(self):
        def task():
            for sid in SERVO_IDS:
                pos = DEFAULT_CENTER_POS
                t_ms = int(self.servo_widgets[sid]["time"].get())
                args = list(struct.pack("<BhH", sid, pos, t_ms))
                self._send_and_log(CMD_ARM_SERVO_SINGLE, args)
                self.log(f"[MID-RESTORE] 舵机 {sid}: 回中位 {pos}, t={t_ms}ms")
                time.sleep(0.08)

        self.run_task("恢复中位", task)

    def set_pid_all(self):
        def task():
            p = int(self.var_pid_p.get()) & 0xFF
            i = int(self.var_pid_i.get()) & 0xFF
            d = int(self.var_pid_d.get()) & 0xFF
            for sid in SERVO_IDS:
                # Assumed payload format: [servo_id, P, I, D]
                args = [sid, p, i, d]
                self._send_and_log(CMD_SET_PID_PARAM, args)
                self.log(f"[PID] 舵机 {sid}: P={p} I={i} D={d}")
                time.sleep(0.08)

        self.run_task("一键设置PID", task)

    def set_overload_all(self):
        def task():
            torque = int(self.var_ov_torque.get()) & 0xFF
            tval = int(self.var_ov_time.get()) & 0xFF
            for sid in SERVO_IDS:
                args = [sid, torque, tval]
                self._send_and_log(CMD_SERVO_WRITE_OVERLOAD, args)
                self.log(f"[OVLD] 舵机 {sid}: torque={torque} time={tval}")
                time.sleep(0.08)

        self.run_task("一键设置最小启动力", task)

    def set_pid_one(self):
        def task():
            sid = int(self.var_read_sid.get())
            p = int(self.var_pid_p.get()) & 0xFF
            i = int(self.var_pid_i.get()) & 0xFF
            d = int(self.var_pid_d.get()) & 0xFF
            self._send_and_log(CMD_SET_PID_PARAM, [sid, p, i, d])
            self.log(f"[PID-SET] 舵机 {sid}: P={p} I={i} D={d}")

        self.run_task("设置PID(单个)", task)

    def set_overload_one(self):
        def task():
            sid = int(self.var_read_sid.get())
            torque = int(self.var_ov_torque.get()) & 0xFF
            tval = int(self.var_ov_time.get()) & 0xFF
            self._send_and_log(CMD_SERVO_WRITE_OVERLOAD, [sid, torque, tval])
            self.log(f"[OVLD-SET] 舵机 {sid}: torque={torque} time={tval}")

        self.run_task("设置最小启动力(单个)", task)

    def get_pid_one(self):
        def task():
            sid = int(self.var_read_sid.get())
            frame = self._request_with_log(CMD_GET_PID_PARAM, [sid], timeout=0.6, retries=2)
            if frame is None:
                self.log(f"[PID-GET] 舵机 {sid}: 无响应")
                return
            self.log(f"[PID-GET] 舵机 {sid}: args={frame['args']}")

        self.run_task("读取PID", task)

    def get_overload_one(self):
        def task():
            sid = int(self.var_read_sid.get())
            frame = self._request_with_log(CMD_SERVO_READ_OVERLOAD, [sid], timeout=0.6, retries=2)
            if frame is None:
                self.log(f"[OVLD-GET] 舵机 {sid}: 无响应")
                return
            self.log(f"[OVLD-GET] 舵机 {sid}: args={frame['args']}")

        self.run_task("读取最小启动力", task)

    def _coord_values(self):
        return (
            int(self.var_coord_x.get()),
            int(self.var_coord_y.get()),
            int(self.var_coord_z.get()),
            int(self.var_coord_pitch.get()),
            int(self.var_coord_roll.get()),
            int(self.var_coord_claw.get()),
            int(self.var_coord_time.get()),
        )

    def send_coord_absolute(self):
        def task():
            x, y, z, pitch, roll, claw, t_ms = self._coord_values()
            args = list(struct.pack("<hhhhhhh", pitch, x, y, z, roll, claw, t_ms))
            self._send_and_log(CMD_COORDINATE_SET, args)
            self.log(f"[COORD-SET] x={x} y={y} z={z} pitch={pitch} roll={roll} claw={claw} t={t_ms}ms")

        self.run_task("设置坐标", task)

    def send_coord_increment_all(self):
        def task():
            dx, dy, dz, dp, dr, dc, t_ms = self._coord_values()
            args = list(struct.pack("<hhhhhhh", dx, dy, dz, dp, dr, dc, t_ms))
            self._send_and_log(CMD_MOVE_INC, args)
            self.log(f"[COORD-INC] dx={dx} dy={dy} dz={dz} dp={dp} dr={dr} dc={dc} t={t_ms}ms")

        self.run_task("坐标增量控制", task)

    def move_coord_axis(self, axis, sign):
        def task():
            x, y, z, pitch, roll, claw, t_ms = self._coord_values()
            dx = dy = dz = dp = dr = dc = 0
            if axis == "x":
                dx = sign * x
            elif axis == "y":
                dy = sign * y
            elif axis == "z":
                dz = sign * z
            elif axis == "pitch":
                dp = sign * pitch
            elif axis == "roll":
                dr = sign * roll
            elif axis == "claw":
                dc = sign * claw
            args = list(struct.pack("<hhhhhhh", dx, dy, dz, dp, dr, dc, t_ms))
            self._send_and_log(CMD_MOVE_INC, args)
            self.log(f"[COORD-STEP] axis={axis} sign={sign} dx={dx} dy={dy} dz={dz} dp={dp} dr={dr} dc={dc} t={t_ms}ms")

        self.run_task(f"坐标{axis}{'+' if sign > 0 else '-'}", task)

    def move_single_servo(self, sid):
        def task():
            pos = int(self.servo_widgets[sid]["pos"].get())
            t_ms = int(self.servo_widgets[sid]["time"].get())
            args = list(struct.pack("<BhH", sid, pos, t_ms))
            self._send_and_log(CMD_ARM_SERVO_SINGLE, args)
            self.log(f"[MOVE] 舵机 {sid}: pos={pos}, t={t_ms}ms")

        self.run_task(f"单舵机{sid}执行", task)

    def move_servo_to_center(self, sid):
        self.servo_widgets[sid]["pos"].set(DEFAULT_CENTER_POS)
        self.move_single_servo(sid)

    def read_single_position(self, sid):
        def task():
            frame = self._request_with_log(
                CMD_GET_CUR_COORDS,
                [],
                timeout=0.7,
                retries=2,
                predicate=lambda f: len(f["args"]) >= 24,
            )
            if frame is None:
                self.log(f"[READ-POS] 舵机 {sid}: 无响应")
                return

            args = frame["args"]
            if len(args) >= 24:
                idx = 12 + (sid - 1) * 2
                pos = struct.unpack("<h", bytes(args[idx : idx + 2]))[0]
                self.log(f"[READ-POS] 舵机 {sid}: 当前位置 {pos}")
            else:
                self.log(f"[READ-POS] 舵机 {sid}: 响应长度不足, args={args}")

        self.run_task(f"读取舵机{sid}位置", task)

    def on_close(self):
        self._save_config()
        self.client.close()
        self.destroy()


if __name__ == "__main__":
    app = DebugHostApp()
    app.mainloop()
