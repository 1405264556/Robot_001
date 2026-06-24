#!/usr/bin/env python3
"""
e-puck Monitor - EPFL e-puck Mini robot monitoring and control application.
Python/tkinter rewrite of the original Borland C++ Builder application.

All serial I/O runs in a background thread. The UI thread only updates
widgets via ``root.after()`` using cached data from the background thread.
User commands (LED, motor, camera) are written through a thread-safe queue.
"""

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import struct
import threading
import math
import time
import os
import sys
import queue
import re

# ── Binary command bytes ──────────────────────────────────────────────
CMD_IMAGE       = 0xB7   # -'I'
CMD_LED         = 0xB4   # -'L'
CMD_SENSOR      = 0xB2   # -'N'
CMD_ACCEL       = 0xBF   # -'A'
CMD_MOTOR       = 0xBC   # -'D'
CMD_EXIT_BINARY = 0x00

SIZE_IMAGE = 185
POLL_INTERVAL_MS = 50

# Known Bluetooth-friendly names for e-puck
EPUCK_PORT_KEYWORDS = ['epuck', 'e-puck', 'bluetooth', 'bt', 'serial', 'spp', 'com']

# Typical baud rates to try when scanning
SCAN_BAUD_RATES = [115200, 57600, 9600]


def _test_connection(port_name, baud=115200, timeout=1.0):
    """
    Try to open a port and verify an e-puck is on the other end.
    Returns (True, info_string) or (False, error_string).
    """
    ser = None
    try:
        ser = serial.Serial(
            port=port_name, baudrate=baud,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.3, write_timeout=0.3,
        )
        # Flush any stale data
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # Send CR to wake up the robot
        ser.write(b'\r')
        ser.flush()
        time.sleep(0.15)

        # Send ASCII selector query: "C\r" → robot should reply "c,N\n"
        ser.reset_input_buffer()
        ser.write(b"C\r")
        ser.flush()
        ser.timeout = timeout
        response = ser.readline()
        response_str = response.decode('ascii', errors='replace').strip().lower()

        if response_str.startswith('c,') or response_str.startswith('c '):
            ser.close()
            return True, f"在 {port_name} 找到 e-puck（{baud} 波特）"

        # Also try binary-mode ping
        ser.reset_input_buffer()
        ser.write(bytes([CMD_SENSOR, CMD_ACCEL, 0x00]))
        ser.flush()
        ser.timeout = 0.5
        data = ser.read(20)
        if len(data) >= 6:
            ser.close()
            return True, f"在 {port_name} 找到 e-puck（{baud} 波特，二进制响应）"

        ser.close()
        return False, f"{port_name} 无 e-puck 响应（{baud} 波特）"
    except (serial.SerialException, OSError) as e:
        if ser:
            try:
                ser.close()
            except Exception:
                pass
        return False, f"无法打开 {port_name}：{e}"


class EPuckMonitor:
    def __init__(self, root):
        self.root = root
        self.root.title("e-puck 监视器")
        self.root.geometry("800x720")
        self.root.minsize(780, 650)

        # ── data directory (beside the exe, not on C:) ────────────────
        if getattr(sys, 'frozen', False):
            self.app_dir = os.path.dirname(sys.executable)
        else:
            self.app_dir = os.path.dirname(os.path.abspath(__file__))
        self.data_dir = os.path.join(self.app_dir, "epuck_data")
        os.makedirs(self.data_dir, exist_ok=True)

        # ── serial state ──────────────────────────────────────────────
        self.ser = None
        self.connected = False
        self.paused = False
        self._connecting = False   # guard against double-connect while scanning

        # ── background polling ────────────────────────────────────────
        self._poll_thread = None
        self._stop_event = threading.Event()
        self._cmd_queue = queue.Queue()
        self._data_lock = threading.Lock()
        self._cached_sensors = None
        self._cached_micro = None
        self._cached_image = None

        # ── test mode ─────────────────────────────────────────────────
        self.testing = False
        self.test_counter = 0

        # ── port list cache (description → device mapping) ────────────
        self._port_info = {}   # display_name → device_path

        # ── build UI ──────────────────────────────────────────────────
        self._build_ui()
        self._set_controls_state(tk.DISABLED)
        self._ui_timer = None
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)

    # ══════════════════════════════════════════════════════════════════
    #  UI construction
    # ══════════════════════════════════════════════════════════════════
    def _build_ui(self):
        main = ttk.Frame(self.root, padding=6)
        main.pack(fill=tk.BOTH, expand=True)

        # ── LEFT column ───────────────────────────────────────────────
        left = ttk.Frame(main)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 6))

        # Connection
        conn = ttk.LabelFrame(left, text="串口连接", padding=4)
        conn.pack(fill=tk.X, pady=(0, 4))

        ttk.Label(conn, text="端口：").grid(row=0, column=0, padx=(0, 4))
        self.port_var = tk.StringVar(value="")
        self.port_cb = ttk.Combobox(conn, textvariable=self.port_var, width=38)
        self.port_cb.grid(row=0, column=1, padx=(0, 4))
        self.port_cb.bind('<FocusIn>', lambda e: self._refresh_ports())

        btn_frame = ttk.Frame(conn)
        btn_frame.grid(row=0, column=2, columnspan=2)
        ttk.Button(btn_frame, text="↻", width=3,
                   command=self._refresh_ports).pack(side=tk.LEFT, padx=1)
        self.btn_scan = ttk.Button(btn_frame, text="自动检测",
                                   command=self._auto_detect)
        self.btn_scan.pack(side=tk.LEFT, padx=1)

        self.btn_connect = ttk.Button(conn, text="连接",
                                      command=self._toggle_connect, width=12)
        self.btn_connect.grid(row=1, column=0, columnspan=4, pady=4)

        self._status_var = tk.StringVar(value="请选择端口，或点击「自动检测」查找机器人")
        status_lbl = ttk.Label(conn, textvariable=self._status_var,
                               foreground='#555', font=('', 8), wraplength=300)
        status_lbl.grid(row=2, column=0, columnspan=4, sticky='w')

        # Camera
        camf = ttk.LabelFrame(left, text="摄像头", padding=4)
        camf.pack(fill=tk.X, pady=(0, 4))
        self.cam_canvas = tk.Canvas(camf, width=SIZE_IMAGE, height=SIZE_IMAGE,
                                     bg='white', highlightthickness=1,
                                     highlightbackground='#888')
        self.cam_canvas.pack(pady=(0, 4))

        ctrl1 = ttk.Frame(camf)
        ctrl1.pack(fill=tk.X)
        self.rotate_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(ctrl1, text="旋转 -90°", variable=self.rotate_var).pack(side=tk.LEFT)
        ttk.Label(ctrl1, text=" 宽:").pack(side=tk.LEFT)
        self.w_var = tk.StringVar(value="40")
        ttk.Entry(ctrl1, textvariable=self.w_var, width=5).pack(side=tk.LEFT)
        ttk.Label(ctrl1, text=" 高:").pack(side=tk.LEFT)
        self.h_var = tk.StringVar(value="40")
        ttk.Entry(ctrl1, textvariable=self.h_var, width=5).pack(side=tk.LEFT)
        ttk.Label(ctrl1, text=" 缩放:").pack(side=tk.LEFT)
        self.zoom_var = tk.StringVar(value="8")
        ttk.Entry(ctrl1, textvariable=self.zoom_var, width=5).pack(side=tk.LEFT)

        ctrl2 = ttk.Frame(camf)
        ctrl2.pack(fill=tk.X, pady=2)
        self.color_var = tk.StringVar(value="color")
        ttk.Radiobutton(ctrl2, text="彩色", variable=self.color_var,
                        value="color").pack(side=tk.LEFT, padx=(0, 8))
        ttk.Radiobutton(ctrl2, text="灰度", variable=self.color_var,
                        value="bw").pack(side=tk.LEFT)

        ctrl3 = ttk.Frame(camf)
        ctrl3.pack(fill=tk.X, pady=2)
        self.btn_get_img = ttk.Button(ctrl3, text="拍照",
                                      command=self._request_image)
        self.btn_get_img.pack(side=tk.LEFT, padx=(0, 4))
        self.btn_cam_para = ttk.Button(ctrl3, text="发送参数",
                                       command=self._send_cam_params)
        self.btn_cam_para.pack(side=tk.LEFT, padx=(0, 4))
        self.cont_var = tk.BooleanVar(value=False)
        self.cb_cont = ttk.Checkbutton(ctrl3, text="连续采集",
                                       variable=self.cont_var,
                                       command=self._toggle_continuous)
        self.cb_cont.pack(side=tk.LEFT)

        # Virtual joystick
        motf = ttk.LabelFrame(left, text="电机控制 – 摇杆操控", padding=4)
        motf.pack(fill=tk.BOTH, expand=True)

        joy_size = 210
        self.joy_cx = joy_size // 2
        self.joy_cy = joy_size // 2
        self.joy_radius = 82
        self.joy_knob_r = 16

        self.joy_canvas = tk.Canvas(motf, width=joy_size, height=joy_size,
                                     bg='#f5f5f5', highlightthickness=0)
        self.joy_canvas.pack()

        # Outer ring
        self.joy_canvas.create_oval(
            self.joy_cx - self.joy_radius, self.joy_cy - self.joy_radius,
            self.joy_cx + self.joy_radius, self.joy_cy + self.joy_radius,
            fill='#e8e8e8', outline='#999', width=3, tags="base")
        self.joy_canvas.create_oval(
            self.joy_cx - self.joy_radius + 6, self.joy_cy - self.joy_radius + 6,
            self.joy_cx + self.joy_radius - 6, self.joy_cy + self.joy_radius - 6,
            fill='', outline='#ddd', width=1, tags="base")
        # Crosshair
        g = self.joy_radius - 14
        self.joy_canvas.create_line(self.joy_cx - g, self.joy_cy,
                                     self.joy_cx + g, self.joy_cy,
                                     fill='#ddd', tags="base")
        self.joy_canvas.create_line(self.joy_cx, self.joy_cy - g,
                                     self.joy_cx, self.joy_cy + g,
                                     fill='#ddd', tags="base")
        # Center pip
        self.joy_canvas.create_oval(self.joy_cx - 4, self.joy_cy - 4,
                                     self.joy_cx + 4, self.joy_cy + 4,
                                     fill='#ccc', outline='', tags="base")
        # Direction labels
        for angle, text in [(270, '↑前'), (90, '↓后'), (180, '←左'), (0, '右→')]:
            rad = math.radians(angle)
            x = self.joy_cx + (self.joy_radius + 16) * math.cos(rad)
            y = self.joy_cy - (self.joy_radius + 16) * math.sin(rad)
            self.joy_canvas.create_text(x, y, text=text, fill='#888', font=('', 7))

        # Knob
        self.joy_knob = self.joy_canvas.create_oval(
            self.joy_cx - self.joy_knob_r, self.joy_cy - self.joy_knob_r,
            self.joy_cx + self.joy_knob_r, self.joy_cy + self.joy_knob_r,
            fill='#4a90d9', outline='#2d5f8a', width=2, tags="knob")

        # Speed readout
        spf = ttk.Frame(motf)
        spf.pack(fill=tk.X, pady=(6, 0))
        ttk.Label(spf, text="左轮:").pack(side=tk.LEFT)
        self.spd_l_var = tk.StringVar(value="0")
        ttk.Label(spf, textvariable=self.spd_l_var, width=6, anchor='e',
                  foreground='#555').pack(side=tk.LEFT, padx=(0, 14))
        ttk.Label(spf, text="右轮:").pack(side=tk.LEFT)
        self.spd_r_var = tk.StringVar(value="0")
        ttk.Label(spf, textvariable=self.spd_r_var, width=6, anchor='e',
                  foreground='#555').pack(side=tk.LEFT)
        ttk.Label(spf, text="  (±1000)", foreground='#aaa', font=('', 7)).pack(side=tk.LEFT)

        ttk.Label(motf, text="拖动摇杆控制方向与速度，松手自动停止",
                  foreground='gray').pack()

        # Bind events
        self.joy_canvas.tag_bind("knob", "<B1-Motion>", self._on_joy_drag)
        self.joy_canvas.tag_bind("knob", "<ButtonRelease-1>", self._on_joy_release)
        self.joy_canvas.tag_bind("base", "<Button-1>", self._on_joy_click)
        self.joy_canvas.bind("<B1-Motion>", self._on_joy_drag)
        self.joy_canvas.bind("<ButtonRelease-1>", self._on_joy_release)

        # ── RIGHT column ──────────────────────────────────────────────
        right = ttk.Frame(main)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        top_bar = ttk.Frame(right)
        top_bar.pack(fill=tk.X, pady=(0, 4))
        self.btn_pause = ttk.Button(top_bar, text="暂停",
                                    command=self._toggle_pause)
        self.btn_pause.pack(side=tk.LEFT, padx=(0, 4))
        self.btn_test = ttk.Button(top_bar, text="测试全部执行器",
                                   command=self._toggle_test)
        self.btn_test.pack(side=tk.LEFT)

        # Robot diagram
        robotf = ttk.LabelFrame(right, text="接近传感器 & LED", padding=4)
        robotf.pack(fill=tk.X, pady=(0, 4))
        self.robot_canvas = tk.Canvas(robotf, width=280, height=280,
                                       bg='#f5f5f5', highlightthickness=0)
        self.robot_canvas.pack()
        self._draw_robot()

        # Sensor readings
        sensef = ttk.LabelFrame(right, text="传感器读数", padding=4)
        sensef.pack(fill=tk.BOTH, expand=True, pady=(0, 4))

        acc_col = ttk.Frame(sensef)
        acc_col.pack(side=tk.LEFT, fill=tk.BOTH, padx=(0, 8))
        ttk.Label(acc_col, text="加速度计方向").pack()
        self.acc_canvas = tk.Canvas(acc_col, width=185, height=185, bg='white',
                                     highlightthickness=1,
                                     highlightbackground='#ccc')
        self.acc_canvas.pack()
        self._draw_pie(90)
        ttk.Label(acc_col, text="倾斜度").pack()
        self.incl_bar = ttk.Progressbar(acc_col, orient=tk.VERTICAL,
                                        length=150, maximum=180, value=0)
        self.incl_bar.pack(pady=(0, 8))

        mic_col = ttk.Frame(sensef)
        mic_col.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))
        ttk.Label(mic_col, text="麦克风").pack()
        ttk.Label(mic_col, text="幅值", font=('', 8)).pack()
        mics = ttk.Frame(mic_col)
        mics.pack()
        self.mic1 = ttk.Progressbar(mics, orient=tk.VERTICAL,
                                    length=105, maximum=500)
        self.mic1.pack(side=tk.LEFT, padx=3)
        self.mic2 = ttk.Progressbar(mics, orient=tk.VERTICAL,
                                    length=105, maximum=500)
        self.mic2.pack(side=tk.LEFT, padx=3)
        self.mic3 = ttk.Progressbar(mics, orient=tk.VERTICAL,
                                    length=105, maximum=500)
        self.mic3.pack(side=tk.LEFT, padx=3)

        prox_col = ttk.Frame(sensef)
        prox_col.pack(side=tk.LEFT, fill=tk.Y)
        ttk.Label(prox_col, text="接近传感器 (0-4095)").pack()
        self.prox_labels = []
        self.prox_bars = []
        for i in range(8):
            row = ttk.Frame(prox_col)
            row.pack(fill=tk.X, pady=0)
            ttk.Label(row, text=f"IR{i}:", width=4, anchor='e').pack(side=tk.LEFT)
            bar = ttk.Progressbar(row, orient=tk.HORIZONTAL,
                                  length=120, maximum=4095)
            bar.pack(side=tk.LEFT, padx=2)
            lbl = ttk.Label(row, text="0", width=5, anchor='w')
            lbl.pack(side=tk.LEFT)
            self.prox_bars.append(bar)
            self.prox_labels.append(lbl)

        # Info
        infof = ttk.LabelFrame(right, text="机器人信息", padding=4)
        infof.pack(fill=tk.X)
        ttk.Label(infof, text="选择器：").grid(row=0, column=0, padx=(0, 2))
        self.sel_var = tk.StringVar(value="0")
        ttk.Entry(infof, textvariable=self.sel_var, width=6,
                  state='readonly').grid(row=0, column=1, padx=(0, 10))
        ttk.Label(infof, text="IR校验：").grid(row=0, column=2, padx=(0, 2))
        self.ir_check_var = tk.StringVar(value="0")
        ttk.Entry(infof, textvariable=self.ir_check_var, width=8,
                  state='readonly').grid(row=0, column=3, padx=(0, 10))
        ttk.Label(infof, text="IR地址：").grid(row=0, column=4, padx=(0, 2))
        self.ir_addr_var = tk.StringVar(value="0")
        ttk.Entry(infof, textvariable=self.ir_addr_var, width=8,
                  state='readonly').grid(row=0, column=5, padx=(0, 10))
        ttk.Label(infof, text="IR数据：").grid(row=0, column=6, padx=(0, 2))
        self.ir_data_var = tk.StringVar(value="0")
        ttk.Entry(infof, textvariable=self.ir_data_var, width=8,
                  state='readonly').grid(row=0, column=7)

        # Initial refresh
        self._refresh_ports()

    # ══════════════════════════════════════════════════════════════════
    #  Robot diagram
    # ══════════════════════════════════════════════════════════════════
    def _draw_robot(self):
        c = self.robot_canvas
        cx, cy, r = 140, 140, 115
        c.create_oval(cx - r, cy - r, cx + r, cy + r,
                      fill='#e8e8e8', outline='#666', width=2)

        self._led_rects = []
        self._led_vars = [tk.BooleanVar(value=False) for _ in range(8)]
        angles = [270, 315, 0, 45, 90, 135, 180, 225]
        for i, a in enumerate(angles):
            rad = math.radians(a)
            x = cx + r * 0.92 * math.cos(rad)
            y = cy - r * 0.92 * math.sin(rad)
            rect = c.create_rectangle(x - 7, y - 7, x + 7, y + 7,
                                       fill='#888', outline='#333',
                                       tags=("led", f"led{i}"))
            c.create_text(x, y - 11, text=str(i), font=('', 7), fill='#333')
            c.tag_bind(f"led{i}", "<Button-1>",
                       lambda e, n=i: self._toggle_led(n))
            self._led_rects.append(rect)

        self._prox_robot_rects = []
        prox_angles = [292, 337, 22, 67, 112, 157, 202, 247]
        for a in prox_angles:
            rad = math.radians(a)
            x = cx + r * 0.85 * math.cos(rad)
            y = cy - r * 0.85 * math.sin(rad)
            rect = c.create_rectangle(x - 12, y - 3, x + 12, y + 3,
                                       fill='#4c4', outline='#060', tags="prox")
            self._prox_robot_rects.append(rect)

        self._body_led_var = tk.BooleanVar(value=False)
        body_cb = tk.Checkbutton(self.robot_canvas, text="机身LED",
                                 variable=self._body_led_var,
                                 command=self._toggle_body_led)
        c.create_window(200, 20, window=body_cb, anchor='e')

        self._front_led_var = tk.BooleanVar(value=False)
        front_cb = tk.Checkbutton(self.robot_canvas, text="前灯LED",
                                  variable=self._front_led_var,
                                  command=self._toggle_front_led)
        c.create_window(200, 45, window=front_cb, anchor='e')

    def _update_robot_leds(self):
        for i, var in enumerate(self._led_vars):
            color = '#ff0' if var.get() else '#888'
            self.robot_canvas.itemconfig(self._led_rects[i], fill=color)

    def _update_prox_robot(self, values):
        max_val = 4095.0
        for i, v in enumerate(values):
            ratio = min(v / max_val, 1.0)
            if ratio < 0.5:
                r, g = int(255 * ratio * 2), 200
            else:
                r, g = 255, int(200 * (1 - ratio) * 2)
            self.robot_canvas.itemconfig(self._prox_robot_rects[i],
                                         fill=f'#{r:02x}{g:02x}00')

    # ══════════════════════════════════════════════════════════════════
    #  Accelerometer pie gauge
    # ══════════════════════════════════════════════════════════════════
    def _draw_pie(self, orientation_deg):
        c = self.acc_canvas
        c.delete("pie")
        cx, cy, rr = 92, 92, 80
        start = 90 - (orientation_deg - 2)
        c.create_arc(cx - rr, cy - rr, cx + rr, cy + rr,
                     start=start, extent=4,
                     fill='red', outline='red', tags="pie")
        for deg in range(0, 360, 30):
            rad = math.radians(90 - deg)
            x1 = cx + (rr - 10) * math.cos(rad)
            y1 = cy - (rr - 10) * math.sin(rad)
            x2 = cx + rr * math.cos(rad)
            y2 = cy - rr * math.sin(rad)
            c.create_line(x1, y1, x2, y2, fill='#aaa', tags="pie_tick")

    # ══════════════════════════════════════════════════════════════════
    #  Port management
    # ══════════════════════════════════════════════════════════════════
    def _refresh_ports(self, *args):
        """List all COM ports with descriptions."""
        ports = sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)
        self._port_info.clear()
        entries = []
        for p in ports:
            # Build a human-readable label
            desc = p.description or ""
            # Clean up description
            if desc and p.device in desc:
                desc = desc.replace(p.device, "").strip(" ()")
            # Try to get manufacturer
            manufacturer = getattr(p, 'manufacturer', '') or ''
            hwid = getattr(p, 'hwid', '') or ''
            vid_pid = ""
            if p.vid is not None and p.pid is not None:
                vid_pid = f" [VID:{p.vid:04X} PID:{p.pid:04X}]"

            # Build display string
            parts = [p.device]
            if desc:
                parts.append(f"- {desc}")
            if manufacturer and manufacturer != desc:
                parts.append(f"({manufacturer})")
            if vid_pid:
                parts.append(vid_pid)

            label = " ".join(parts)
            self._port_info[label] = p.device
            entries.append(label)

        self.port_cb['values'] = entries

        # Smart default: pick Bluetooth port if visible
        bt_entry = None
        for e in entries:
            el = e.lower()
            if 'bluetooth' in el or 'bt ' in el or 'spp' in el:
                bt_entry = e
                break
        if bt_entry:
            self.port_var.set(bt_entry)
        elif entries:
            self.port_var.set(entries[0])
        else:
            self.port_var.set("未检测到COM端口")
            self._set_status("未检测到串口，蓝牙是否已配对？")

    def _get_selected_port(self):
        """Resolve the combobox selection to a device path."""
        sel = self.port_var.get()
        return self._port_info.get(sel, sel)

    # ══════════════════════════════════════════════════════════════════
    #  Auto-detect e-puck
    # ══════════════════════════════════════════════════════════════════
    def _auto_detect(self):
        """Scan all COM ports to find an e-puck robot."""
        if self.connected or self._connecting:
            return

        self.btn_scan.config(state=tk.DISABLED, text="正在扫描…")
        self.btn_connect.config(state=tk.DISABLED)
        self._set_status("正在所有COM端口上搜索e-puck…")

        def scan_thread():
            ports = list(serial.tools.list_ports.comports())
            total = len(ports)
            results = []

            for i, p in enumerate(ports):
                self.root.after(0, lambda idx=i, t=total:
                                self._set_status(
                                    f"正在扫描… ({idx + 1}/{t}) {p.device}"))
                ok, msg = _test_connection(p.device)
                results.append((p.device, ok, msg))
                if ok:
                    # Found it — stop early
                    break

            # Back on UI thread
            def done():
                self.btn_scan.config(state=tk.NORMAL, text="自动检测")
                self.btn_connect.config(state=tk.NORMAL)

                found = [(dev, msg) for dev, ok, msg in results if ok]
                if found:
                    dev, msg = found[0]
                    for label, device in self._port_info.items():
                        if device == dev:
                            self.port_var.set(label)
                            break
                    self._set_status(f"已找到：{msg}")
                    messagebox.showinfo("找到 e-puck",
                                        f"已检测到 e-puck 机器人！\n\n"
                                        f"端口：{dev}\n\n"
                                        f"请点击「连接」开始使用。")
                else:
                    self._set_status(
                        "未找到 e-puck。请检查：机器人是否开机？蓝牙是否配对？端口是否正确？"
                    )
                    detail_lines = [f"  {dev}: {msg}"
                                    for dev, ok, msg in results]
                    messagebox.showwarning(
                        "未找到 e-puck",
                        "在所有COM端口上均未检测到 e-puck 机器人。\n\n"
                        "请检查：\n"
                        "• e-puck 是否已开机（绿色LED亮起）\n"
                        "• 蓝牙是否已在 Windows 设置中配对\n"
                        "• 是否正确选择了COM端口\n\n"
                        "扫描结果：\n" +
                        ("\n".join(detail_lines) if detail_lines else
                         "  无可用COM端口"))

            self.root.after(0, done)

        threading.Thread(target=scan_thread, daemon=True).start()

    # ══════════════════════════════════════════════════════════════════
    #  Connection
    # ══════════════════════════════════════════════════════════════════
    def _toggle_connect(self):
        if self._connecting:
            return
        if self.connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self._get_selected_port()
        if not port or port.startswith("未检测"):
            self._set_status("未选择端口，请点击「自动检测」或手动选择端口。")
            return

        self._connecting = True
        self.btn_connect.config(state=tk.DISABLED, text="正在连接…")
        self.btn_scan.config(state=tk.DISABLED)
        self._set_status(f"正在连接 {port}…")

        def do_connect():
            try:
                ser = serial.Serial(
                    port=port, baudrate=115200,
                    bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.3, write_timeout=0.3,
                )

                # ── Handshake ─────────────────────────────────────────
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                ser.write(b'\r')
                ser.flush()
                time.sleep(0.15)

                # ── Verify robot responds ─────────────────────────────
                ser.reset_input_buffer()
                ser.write(b"C\r")
                ser.flush()
                ser.timeout = 1.5
                response = ser.readline()
                response_str = response.decode('ascii', errors='replace').strip()

                if response_str.startswith('c,') or response_str.startswith('c '):
                    # Robot responded correctly
                    pass
                elif len(response) > 0:
                    # Got something, might be noise
                    pass
                else:
                    # No response — try binary ping
                    ser.reset_input_buffer()
                    ser.write(bytes([CMD_SENSOR, CMD_ACCEL, 0x00]))
                    ser.flush()
                    ser.timeout = 1.0
                    data = ser.read(20)
                    if len(data) < 4:
                        ser.close()
                        self.root.after(0, self._connect_failed,
                                        f"端口 {port} 已打开，但 e-puck 无响应。\n\n"
                                        "请检查：\n"
                                        "• 机器人是否已开机\n"
                                        "• 是否烧录了 BTcom 固件\n"
                                        "• COM 端口是否正确（可尝试「自动检测」）")
                        return

                # ── Robot confirmed — finish setup ────────────────────
                self.ser = ser
                self.connected = True

                # Initialise robot state
                ser.write(bytes([CMD_LED, 8, 0])); ser.flush()
                time.sleep(0.02)
                ser.write(bytes([CMD_EXIT_BINARY])); ser.flush()
                time.sleep(0.01)
                ser.write(b"T,0\r"); ser.flush()
                time.sleep(0.01)
                ser.write(b"B,0\r"); ser.flush()
                time.sleep(0.01)
                ser.write(b"F,0\r"); ser.flush()
                ser.reset_input_buffer()

                # UI updates must happen on main thread
                self.root.after(0, self._connect_success, port)
            except (serial.SerialException, OSError) as e:
                self.root.after(0, self._connect_failed,
                                f"无法打开端口 {port}\n\n{str(e)}")

        threading.Thread(target=do_connect, daemon=True).start()

    def _connect_success(self, port):
        self._connecting = False
        self.btn_connect.config(state=tk.NORMAL, text="断开连接")
        self.btn_scan.config(state=tk.DISABLED)
        self.port_cb.config(state=tk.DISABLED)
        self._set_controls_state(tk.NORMAL)
        self._start_polling()
        self._set_status(f"已连接 {port} — e-puck 就绪")

    def _connect_failed(self, msg):
        self._connecting = False
        self.connected = False
        self.ser = None
        self.btn_connect.config(state=tk.NORMAL, text="连接")
        self.btn_scan.config(state=tk.NORMAL)
        self.port_cb.config(state='readonly')
        self._set_status("连接失败")
        messagebox.showerror("连接失败", msg)

    def _disconnect(self):
        self._stop_polling()
        if self.ser:
            try:
                self.ser.write(bytes([CMD_MOTOR, 0, 0, 0, 0]))
                self.ser.flush()
                time.sleep(0.01)
                self.ser.write(bytes([CMD_EXIT_BINARY]))
                self.ser.flush()
                time.sleep(0.01)
                self.ser.write(b"T,0\r")
                self.ser.flush()
                self.ser.reset_input_buffer()
                self.ser.close()
            except (serial.SerialException, OSError):
                pass
        self.ser = None
        self.connected = False
        self.testing = False
        self.btn_connect.config(text="连接")
        self.btn_scan.config(state=tk.NORMAL)
        self.port_cb.config(state='readonly')
        self.btn_test.config(text="测试全部执行器")
        self._set_controls_state(tk.DISABLED)
        self._set_status("已断开连接")

    def _set_controls_state(self, state):
        for w in [self.btn_get_img, self.btn_cam_para, self.btn_test,
                  self.btn_pause, self.joy_canvas]:
            try:
                w.config(state=state)
            except tk.TclError:
                pass
        self.cb_cont.config(state=state)
        for child in self.robot_canvas.winfo_children():
            if isinstance(child, tk.Checkbutton):
                child.config(state=state)

    # ══════════════════════════════════════════════════════════════════
    #  Background polling thread
    # ══════════════════════════════════════════════════════════════════
    def _start_polling(self):
        self._stop_event.clear()
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()
        self._ui_timer = self.root.after(POLL_INTERVAL_MS, self._ui_update_tick)

    def _stop_polling(self):
        self._stop_event.set()
        if self._poll_thread and self._poll_thread.is_alive():
            self._poll_thread.join(timeout=1.5)
        self._poll_thread = None
        if self._ui_timer:
            self.root.after_cancel(self._ui_timer)
            self._ui_timer = None

    def _poll_loop(self):
        micro_tick = 0
        while not self._stop_event.is_set() and self.connected:
            try:
                self._drain_cmd_queue()
                if not self.paused:
                    self._bg_poll_sensors()
                    micro_tick += 1
                    if micro_tick >= 2:
                        micro_tick = 0
                        self._bg_poll_micro()
            except Exception:
                pass
            if self._stop_event.wait(0.05):
                break
        self._drain_cmd_queue()

    def _drain_cmd_queue(self):
        while True:
            try:
                cmd = self._cmd_queue.get_nowait()
            except queue.Empty:
                break
            try:
                cmd_type = cmd[0]
                if cmd_type == 'led':
                    self._raw_write(bytes([CMD_LED, cmd[1], cmd[2]]))
                elif cmd_type in ('body_led', 'front_led', 'raw_ascii'):
                    self._raw_write(bytes([CMD_EXIT_BINARY]))
                    self._raw_write(cmd[1].encode('ascii'))
                elif cmd_type == 'motor':
                    left, right = cmd[1], cmd[2]
                    self._raw_write(bytes([
                        CMD_MOTOR,
                        left & 0xFF, (left >> 8) & 0xFF,
                        right & 0xFF, (right >> 8) & 0xFF,
                    ]))
                elif cmd_type == 'cam_params':
                    self._raw_write(bytes([CMD_EXIT_BINARY]))
                    self._raw_write(cmd[1].encode('ascii'))
                elif cmd_type == 'image':
                    self._bg_capture_image()
                elif cmd_type == 'raw_binary':
                    self._raw_write(bytes(cmd[1]))
            except Exception:
                pass

    def _raw_write(self, data):
        if self.ser and self.connected:
            try:
                self.ser.write(data)
                self.ser.flush()
            except (serial.SerialException, OSError):
                pass

    def _bg_read_exact(self, n, timeout=0.3):
        if not self.ser or not self.connected:
            return b''
        try:
            self.ser.timeout = timeout
            buf = bytearray()
            deadline = time.time() + timeout
            while len(buf) < n and time.time() < deadline:
                chunk = self.ser.read(n - len(buf))
                if chunk:
                    buf.extend(chunk)
            return bytes(buf)
        except (serial.SerialException, OSError):
            return b''

    def _bg_readline(self, timeout=0.3):
        if not self.ser or not self.connected:
            return ''
        try:
            self.ser.timeout = timeout
            line = self.ser.readline()
            return line.decode('ascii', errors='replace').strip()
        except (serial.SerialException, OSError):
            return ''

    def _bg_poll_sensors(self):
        try:
            self.ser.reset_input_buffer()
            self._raw_write(bytes([CMD_SENSOR, CMD_ACCEL, 0x00]))
            data = self._bg_read_exact(16, timeout=0.2)
            if len(data) < 16:
                return
            prox = [data[i * 2] + data[i * 2 + 1] * 256 for i in range(8)]

            floats = []
            for _ in range(3):
                fb = self._bg_read_exact(4, timeout=0.15)
                if len(fb) < 4:
                    break
                try:
                    floats.append(struct.unpack('<f', fb)[0])
                except struct.error:
                    floats.append(0.0)

            if len(floats) >= 3:
                ori = max(0.0, min(360.0, floats[1]))
                inc = max(0.0, min(180.0, floats[2]))
                with self._data_lock:
                    self._cached_sensors = (prox, ori, inc)
        except Exception:
            pass

    def _bg_poll_micro(self):
        try:
            self.ser.reset_input_buffer()
            self._raw_write(bytes([CMD_EXIT_BINARY]))
            time.sleep(0.005)
            self._raw_write(b"U\r")
            line = self._bg_readline(timeout=0.15)
            mic1 = mic2 = mic3 = None
            if line.startswith('u,'):
                parts = line.split(',')
                if len(parts) >= 4:
                    try:
                        mic1, mic2, mic3 = (int(parts[1]), int(parts[2]),
                                            int(parts[3]))
                    except ValueError:
                        pass

            self.ser.reset_input_buffer()
            self._raw_write(bytes([CMD_EXIT_BINARY]))
            time.sleep(0.005)
            self._raw_write(b"C\r")
            line = self._bg_readline(timeout=0.15)
            selector = None
            if line.startswith('c,'):
                parts = line.split(',')
                if len(parts) >= 2:
                    try:
                        selector = int(parts[1])
                    except ValueError:
                        pass

            self.ser.reset_input_buffer()
            self._raw_write(bytes([CMD_EXIT_BINARY]))
            time.sleep(0.005)
            self._raw_write(b"G\r")
            line = self._bg_readline(timeout=0.15)
            ir_check = ir_addr = ir_data = None
            if 'IR check' in line:
                nums = re.findall(r'0x([0-9a-fA-F]+)', line)
                if len(nums) >= 3:
                    try:
                        ir_check = int(nums[0], 16)
                        ir_addr = int(nums[1], 16)
                        ir_data = int(nums[2], 16)
                    except ValueError:
                        pass

            with self._data_lock:
                self._cached_micro = (mic1, mic2, mic3, selector,
                                      ir_check, ir_addr, ir_data)
        except Exception:
            pass

    def _bg_capture_image(self):
        try:
            self.ser.reset_input_buffer()
            self._raw_write(bytes([CMD_IMAGE, 0x00]))
            hdr = self._bg_read_exact(3, timeout=0.5)
            if len(hdr) < 3:
                return
            img_type, w, h = hdr[0], hdr[1], hdr[2]
            pix_count = (w * h) if img_type == 0 else (w * h * 2)
            pixels = self._bg_read_exact(pix_count, timeout=1.5)
            if len(pixels) < pix_count:
                return
            with self._data_lock:
                self._cached_image = (img_type, w, h, bytes(pixels))
        except Exception:
            pass

    # ══════════════════════════════════════════════════════════════════
    #  UI update tick (main thread, non-blocking)
    # ══════════════════════════════════════════════════════════════════
    def _ui_update_tick(self):
        try:
            with self._data_lock:
                sensors = self._cached_sensors
                self._cached_sensors = None
                micro = self._cached_micro
                self._cached_micro = None
                image = self._cached_image
                self._cached_image = None

            if sensors is not None:
                prox, orientation, inclination = sensors
                for i in range(8):
                    self.prox_bars[i]['value'] = prox[i]
                    self.prox_labels[i].config(text=str(prox[i]))
                self._update_prox_robot(prox)
                self._draw_pie(orientation)
                self.incl_bar['value'] = inclination

            if micro is not None:
                mic1, mic2, mic3, sel, ir_chk, ir_adr, ir_dat = micro
                if mic1 is not None:
                    self.mic1['value'] = mic1
                    self.mic2['value'] = mic2
                    self.mic3['value'] = mic3
                if sel is not None:
                    self.sel_var.set(str(sel))
                if ir_chk is not None:
                    self.ir_check_var.set(str(ir_chk))
                    self.ir_addr_var.set(str(ir_adr))
                    self.ir_data_var.set(str(ir_dat))

            if image is not None:
                self._display_image(*image)
        except Exception:
            pass
        finally:
            if self.connected:
                self._ui_timer = self.root.after(POLL_INTERVAL_MS,
                                                  self._ui_update_tick)

    # ══════════════════════════════════════════════════════════════════
    #  Camera display (main thread)
    # ══════════════════════════════════════════════════════════════════
    def _display_image(self, img_type, w, h, pixels):
        c = self.cam_canvas
        c.delete("img")
        if w <= 0 or h <= 0:
            return
        if h < w:
            disp_w, disp_h = SIZE_IMAGE, int(SIZE_IMAGE * h / w)
        else:
            disp_h, disp_w = SIZE_IMAGE, int(SIZE_IMAGE * w / h)
        rotate = self.rotate_var.get()
        if rotate:
            w, h = h, w

        if img_type == 0:
            for row in range(h):
                for col in range(w):
                    src_idx = (w * (row + 1) - 1 - col) if rotate else (row * w + col)
                    if src_idx >= len(pixels):
                        continue
                    gray = pixels[src_idx] & 0xFF
                    color = f'#{gray:02x}{gray:02x}{gray:02x}'
                    x1, y1 = col * disp_w // w, row * disp_h // h
                    x2, y2 = (col + 1) * disp_w // w, (row + 1) * disp_h // h
                    c.create_rectangle(x1, y1, x2, y2, fill=color,
                                       outline=color, tags="img")
        elif img_type == 1:
            for row in range(h):
                for col in range(w):
                    src_idx = (w * 2 * (row + 1) - 2 - col * 2) if rotate else (row * 2 * w + col * 2)
                    if src_idx + 1 >= len(pixels):
                        continue
                    hi, lo = pixels[src_idx], pixels[src_idx + 1]
                    r = hi & 0xF8
                    g = ((hi & 0x07) << 5) | ((lo & 0xE0) >> 3)
                    b = (lo & 0x1F) << 3
                    color = f'#{min(r,255):02x}{min(g,255):02x}{min(b,255):02x}'
                    x1, y1 = col * disp_w // w, row * disp_h // h
                    x2, y2 = (col + 1) * disp_w // w, (row + 1) * disp_h // h
                    c.create_rectangle(x1, y1, x2, y2, fill=color,
                                       outline=color, tags="img")

    # ══════════════════════════════════════════════════════════════════
    #  User actions (queue commands to background thread)
    # ══════════════════════════════════════════════════════════════════
    def _queue_cmd(self, *args):
        if self.connected:
            self._cmd_queue.put(args)

    def _request_image(self):
        self._queue_cmd('image')
        if self.cont_var.get():
            self._camera_tick_schedule()

    def _camera_tick_schedule(self):
        if not self.connected or not self.cont_var.get():
            return
        self._queue_cmd('image')
        self.root.after(500, self._camera_tick_schedule)

    def _toggle_continuous(self):
        if self.cont_var.get():
            self._camera_tick_schedule()

    def _send_cam_params(self):
        color = "1" if self.color_var.get() == "color" else "0"
        cmd = f"J,{color},{self.w_var.get()},{self.h_var.get()},{self.zoom_var.get()}\r"
        self._queue_cmd('cam_params', cmd)

    def _toggle_led(self, n):
        self._led_vars[n].set(not self._led_vars[n].get())
        self._update_robot_leds()
        self._queue_cmd('led', n, 2)

    def _toggle_body_led(self):
        self._queue_cmd('body_led', "B,2\r")

    def _toggle_front_led(self):
        self._queue_cmd('front_led', "F,2\r")

    def _joy_knob_pos(self, x, y):
        """Clamp knob position to joystick radius, return (dx, dy)."""
        dx = x - self.joy_cx
        dy = y - self.joy_cy
        dist = math.sqrt(dx * dx + dy * dy)
        if dist > self.joy_radius:
            dx = dx * self.joy_radius / dist
            dy = dy * self.joy_radius / dist
        return dx, dy

    def _move_knob(self, dx, dy):
        """Draw knob at offset (dx, dy) from center."""
        r = self.joy_knob_r
        self.joy_canvas.coords(self.joy_knob,
                                self.joy_cx + dx - r, self.joy_cy + dy - r,
                                self.joy_cx + dx + r, self.joy_cy + dy + r)

    def _send_joy_speeds(self, dx, dy):
        """Convert joystick offset to motor speeds and send."""
        max_speed = 1000
        fwd = -dy / self.joy_radius   # +1 = forward
        turn = dx / self.joy_radius   # +1 = right
        left = int((fwd + turn) * max_speed)
        right = int((fwd - turn) * max_speed)
        left = max(-max_speed, min(max_speed, left))
        right = max(-max_speed, min(max_speed, right))
        self.spd_l_var.set(str(left))
        self.spd_r_var.set(str(right))
        self._queue_cmd('motor', left, right)

    def _on_joy_drag(self, event):
        if not self.connected:
            return
        dx, dy = self._joy_knob_pos(event.x, event.y)
        self._move_knob(dx, dy)
        self._send_joy_speeds(dx, dy)

    def _on_joy_click(self, event):
        if not self.connected:
            return
        dx, dy = self._joy_knob_pos(event.x, event.y)
        self._move_knob(dx, dy)
        self._send_joy_speeds(dx, dy)

    def _on_joy_release(self, event):
        self._move_knob(0, 0)
        self.spd_l_var.set("0")
        self.spd_r_var.set("0")
        if self.connected:
            self._queue_cmd('motor', 0, 0)

    # ══════════════════════════════════════════════════════════════════
    #  Pause / Test
    # ══════════════════════════════════════════════════════════════════
    def _toggle_pause(self):
        self.paused = not self.paused
        self.btn_pause.config(text="继续" if self.paused else "暂停")

    def _toggle_test(self):
        if self.testing:
            self.testing = False
            self.btn_test.config(text="测试全部执行器")
        else:
            self.testing = True
            self.test_counter = 0
            self.btn_test.config(text="停止测试")
            self._test_tick()

    def _test_tick(self):
        if not self.connected or not self.testing:
            return
        tc = self.test_counter
        testspeed = 300
        if (tc & 1) == 0:
            self._queue_cmd('raw_binary', [CMD_LED, 8, 0])
        else:
            self._queue_cmd('raw_binary', [CMD_LED, 8, 1])
        if ((tc // 4) & 1) == 0:
            self._queue_cmd('motor', testspeed, -testspeed)
        else:
            self._queue_cmd('motor', -testspeed, testspeed)
        self._queue_cmd('raw_ascii', "B,2\r")
        self._queue_cmd('raw_ascii', "F,2\r")
        self._queue_cmd('raw_ascii', f"T,{2 if (tc & 1) == 0 else 1}\r")
        self.test_counter += 1
        self.root.after(200, self._test_tick)

    # ══════════════════════════════════════════════════════════════════
    def _set_status(self, msg):
        self._status_var.set(msg)
        self.root.title(f"e-puck 监视器 - {msg}")

    def _on_closing(self):
        self.testing = False
        if self.connected:
            self._stop_polling()
            if self.ser:
                try:
                    self.ser.write(bytes([CMD_MOTOR, 0, 0, 0, 0]))
                    self.ser.flush()
                    time.sleep(0.01)
                    self.ser.write(bytes([CMD_EXIT_BINARY]))
                    self.ser.flush()
                    time.sleep(0.01)
                    self.ser.write(b"T,0\r")
                    self.ser.flush()
                    self.ser.close()
                except Exception:
                    pass
        self.root.destroy()


def main():
    root = tk.Tk()
    EPuckMonitor(root)
    root.mainloop()


if __name__ == '__main__':
    main()
