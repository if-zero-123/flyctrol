# -*- coding: utf-8 -*-
"""Windows GUI for debugging the NAZE32 custom flight firmware."""

from __future__ import annotations

import queue
import sys
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from tkinter import BooleanVar, IntVar, StringVar, filedialog, messagebox
import tkinter as tk
from tkinter import ttk

from cli_parser import FlightCliParser
from serial_link import available_ports, create_backend, probe_port


APP_TITLE = "NAZE32 飞控调试上位机"


class FlightDebugGui(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title(APP_TITLE)
        self.geometry("1120x760")
        self.minsize(960, 680)

        self.events: "queue.Queue[tuple[str, str]]" = queue.Queue()
        self.backend = None
        self.parser = FlightCliParser()
        self.rx_buffer = ""
        self.raw_log: list[str] = []
        self.port_items: list[tuple[str, str]] = []
        self.polling = False
        self.receiver_polling = False
        self.self_test_running = False
        self.rx_bytes = 0
        self.tx_count = 0
        self.last_rx_time = 0.0
        self.last_tx_time = 0.0
        self.no_rx_warning_shown = False
        self.cli_ready = False
        self.connect_started_at = 0.0
        self.prompt_probe_sent_at = 0.0
        self.command_queue: deque[str] = deque()
        self.waiting_for_prompt = False
        self.command_inflight = ""
        self.command_deadline = 0.0
        self.command_gap_until = 0.0
        self.command_sent_rx_count = 0
        self.command_quiet_deadline = 0.0
        self.command_history: list[str] = []
        self.command_history_index = 0

        self.connected_text = StringVar(value="未连接")
        self.link_stats_text = StringVar(value="TX 0 条 | RX 0 字节 | 未收到数据")
        self.selected_port = StringVar(value="")
        self.manual_command = StringVar(value="")
        self.terminal_mode = BooleanVar(value=True)
        self.motor_permille = IntVar(value=80)
        self.props_removed = BooleanVar(value=False)
        self.advanced_motor = BooleanVar(value=False)
        self.allow_failsafe_motor_test = BooleanVar(value=False)
        self.receiver_status_text = StringVar(value="等待接收机数据")
        self.receiver_map_text = StringVar(value="AETR: CH1 Roll, CH2 Pitch, CH3 Thr, CH4 Yaw, CH5 ARM, CH6 BARO")
        self.receiver_norm_text = StringVar(value="-")
        self.receiver_channel_values: list[IntVar] = []
        self.receiver_channel_labels: list[StringVar] = []

        self._build_ui()
        self.refresh_ports()
        self.after(50, self._process_events)
        self.after(100, self._command_watchdog)
        self.after(300, self._refresh_state_views)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        top = ttk.Frame(self, padding=(10, 8))
        top.pack(fill="x")
        ttk.Label(top, text=APP_TITLE, font=("Microsoft YaHei UI", 15, "bold")).pack(side="left")
        ttk.Label(top, textvariable=self.connected_text, foreground="#0f766e").pack(side="right")

        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill="both", expand=True, padx=10, pady=(0, 10))

        self._build_connect_tab()
        self._build_self_test_tab()
        self._build_monitor_tab()
        self._build_receiver_tab()
        self._build_motor_tab()
        self._build_checklist_tab()

    def _build_connect_tab(self) -> None:
        tab = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(tab, text="连接/日志")

        controls = ttk.Frame(tab)
        controls.pack(fill="x")
        ttk.Label(controls, text="串口:").pack(side="left")
        self.port_combo = ttk.Combobox(controls, textvariable=self.selected_port, state="readonly", width=42)
        self.port_combo.pack(side="left", padx=6)
        ttk.Button(controls, text="刷新", command=self.refresh_ports).pack(side="left", padx=3)
        ttk.Button(controls, text="连接", command=self.connect).pack(side="left", padx=3)
        ttk.Button(controls, text="断开", command=self.disconnect).pack(side="left", padx=3)
        ttk.Button(controls, text="串口诊断", command=self.run_serial_diagnosis).pack(side="left", padx=3)
        ttk.Button(controls, text="自动探测", command=self.start_auto_probe).pack(side="left", padx=3)
        ttk.Button(controls, text="保存日志", command=self.save_log).pack(side="right", padx=3)
        ttk.Button(controls, text="保存CSV", command=self.save_csv).pack(side="right", padx=3)

        hint = "连接后等待启动文本；命令按静默窗口串行发送，电机 STOP ALL 可随时抢发。"
        ttk.Label(tab, text=hint).pack(fill="x", pady=(8, 4))
        ttk.Label(tab, textvariable=self.link_stats_text, foreground="#1d4ed8").pack(fill="x", pady=(0, 4))

        cmd = ttk.Frame(tab)
        cmd.pack(fill="x", pady=(0, 6))
        ttk.Label(cmd, text="手动命令:").pack(side="left")
        entry = ttk.Entry(cmd, textvariable=self.manual_command)
        entry.pack(side="left", fill="x", expand=True, padx=6)
        entry.bind("<Return>", lambda _e: self.send_manual())
        entry.bind("<Up>", self._history_prev)
        entry.bind("<Down>", self._history_next)
        ttk.Button(cmd, text="发送", command=self.send_manual).pack(side="left")
        ttk.Button(cmd, text="慢速发送多行", command=self.send_multiline_manual).pack(side="left", padx=3)
        ttk.Checkbutton(cmd, text="终端模式", variable=self.terminal_mode).pack(side="left", padx=6)

        text_frame = ttk.Frame(tab)
        text_frame.pack(fill="both", expand=True)
        self.log_text = tk.Text(text_frame, wrap="word", height=24, font=("Consolas", 10))
        yscroll = ttk.Scrollbar(text_frame, orient="vertical", command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=yscroll.set)
        self.log_text.pack(side="left", fill="both", expand=True)
        yscroll.pack(side="right", fill="y")

    def _build_self_test_tab(self) -> None:
        tab = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(tab, text="自检")

        ttk.Button(tab, text="一键自检", command=self.run_self_test).pack(anchor="w")
        self.self_test_summary = ttk.Label(tab, text="点击一键自检后，会自动执行 status/clock/i2cscan/imu/baro/rc/batt/heap/tasks。")
        self.self_test_summary.pack(fill="x", pady=8)

        self.check_labels: dict[str, ttk.Label] = {}
        grid = ttk.Frame(tab)
        grid.pack(fill="x")
        checks = [
            ("boot", "启动输出"),
            ("clock", "系统时钟"),
            ("i2c", "I2C: 0x68/0x76"),
            ("imu", "MPU6050 姿态"),
            ("baro", "BMP280 气压"),
            ("rc", "CRSF 接收机"),
            ("batt", "电池电压"),
            ("failsafe", "安全状态"),
            ("heap", "RTOS 堆"),
        ]
        for i, (key, label) in enumerate(checks):
            ttk.Label(grid, text=label + ":").grid(row=i, column=0, sticky="w", pady=4)
            value = ttk.Label(grid, text="等待", foreground="#92400e")
            value.grid(row=i, column=1, sticky="w", padx=8, pady=4)
            self.check_labels[key] = value

        ttk.Label(tab, text="提示: 电机测试前必须确认自检里 failsafe=正常、RC=正常、电池不 critical。").pack(anchor="w", pady=(16, 0))

    def _build_monitor_tab(self) -> None:
        tab = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(tab, text="实时监控")

        controls = ttk.Frame(tab)
        controls.pack(fill="x")
        ttk.Button(controls, text="开始轮询", command=self.start_polling).pack(side="left", padx=3)
        ttk.Button(controls, text="停止轮询", command=self.stop_polling).pack(side="left", padx=3)
        ttk.Button(controls, text="固件 log on", command=lambda: self.send_command("log on")).pack(side="left", padx=12)
        ttk.Button(controls, text="固件 log off", command=lambda: self.send_command("log off")).pack(side="left", padx=3)

        self.monitor_vars = {k: StringVar(value="-") for k in [
            "armed", "failsafe", "rc", "imu", "baro", "althold", "battery", "att", "baro_detail", "sticks", "motors"
        ]}
        rows = [
            ("解锁", "armed"),
            ("Failsafe", "failsafe"),
            ("RC", "rc"),
            ("IMU", "imu"),
            ("BARO", "baro"),
            ("定高", "althold"),
            ("电池", "battery"),
            ("姿态", "att"),
            ("气压/高度", "baro_detail"),
            ("摇杆", "sticks"),
            ("电机输出", "motors"),
        ]
        panel = ttk.LabelFrame(tab, text="实时状态", padding=10)
        panel.pack(fill="both", expand=True, pady=10)
        for i, (label, key) in enumerate(rows):
            ttk.Label(panel, text=label + ":", width=12).grid(row=i, column=0, sticky="w", pady=5)
            ttk.Label(panel, textvariable=self.monitor_vars[key], font=("Consolas", 11)).grid(row=i, column=1, sticky="w", pady=5)

    def _build_receiver_tab(self) -> None:
        tab = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(tab, text="接收机")

        controls = ttk.Frame(tab)
        controls.pack(fill="x")
        ttk.Button(controls, text="刷新接收机", command=lambda: self.queue_commands(["rcmap", "rc"])).pack(side="left", padx=3)
        ttk.Button(controls, text="开始监控", command=self.start_receiver_polling).pack(side="left", padx=3)
        ttk.Button(controls, text="停止监控", command=self.stop_receiver_polling).pack(side="left", padx=3)
        ttk.Label(controls, textvariable=self.receiver_status_text, foreground="#1d4ed8").pack(side="left", padx=12)

        ttk.Label(tab, textvariable=self.receiver_map_text).pack(anchor="w", pady=(10, 4))
        ttk.Label(tab, textvariable=self.receiver_norm_text, font=("Consolas", 11)).pack(anchor="w", pady=(0, 10))

        channel_box = ttk.LabelFrame(tab, text="CRSF 通道", padding=10)
        channel_box.pack(fill="x")
        names = ["CH1 Roll", "CH2 Pitch", "CH3 Thr", "CH4 Yaw", "CH5 ARM", "CH6 BARO", "CH7 AUX3", "CH8 AUX4"]
        self.receiver_channel_values = []
        self.receiver_channel_labels = []
        for row, name in enumerate(names):
            value = IntVar(value=0)
            label = StringVar(value=f"{name}: -")
            self.receiver_channel_values.append(value)
            self.receiver_channel_labels.append(label)
            ttk.Label(channel_box, text=name, width=12).grid(row=row, column=0, sticky="w", pady=4)
            ttk.Progressbar(channel_box, maximum=100, variable=value, length=520).grid(row=row, column=1, sticky="ew", padx=8, pady=4)
            ttk.Label(channel_box, textvariable=label, font=("Consolas", 10), width=24).grid(row=row, column=2, sticky="w", pady=4)
        channel_box.columnconfigure(1, weight=1)

        note = ttk.Label(
            tab,
            text="控制逻辑: CH1/2/4 归一化为 -1000..1000 姿态/偏航输入，CH3 为 0..1000 油门，CH5 高于约1500 解锁请求，CH6 高于约1500 开启 BARO 模式。",
            wraplength=900,
        )
        note.pack(anchor="w", pady=(14, 0))

    def _build_motor_tab(self) -> None:
        tab = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(tab, text="电机测试")

        warning = ttk.Label(
            tab,
            text="危险: 必须拆桨。测试按钮会自动 motor unlock，然后低占空比转动约 4.5 秒并自动 stop。",
            foreground="#b91c1c",
            font=("Microsoft YaHei UI", 10, "bold"),
        )
        warning.pack(anchor="w")

        ttk.Checkbutton(tab, text="我确认已经拆掉全部螺旋桨", variable=self.props_removed, command=self._refresh_motor_buttons).pack(anchor="w", pady=8)

        row = ttk.Frame(tab)
        row.pack(fill="x", pady=4)
        ttk.Label(row, text="测试占空比 permille:").pack(side="left")
        self.motor_spin = ttk.Spinbox(row, from_=0, to=150, textvariable=self.motor_permille, width=8, command=self._clamp_motor_value)
        self.motor_spin.pack(side="left", padx=6)
        ttk.Checkbutton(row, text="高级模式(最高300)", variable=self.advanced_motor, command=self._toggle_advanced_motor).pack(side="left", padx=12)
        ttk.Checkbutton(row, text="台架模式(忽略RC/电池failsafe)", variable=self.allow_failsafe_motor_test).pack(side="left", padx=12)

        self.motor_status = StringVar(value="等待自检状态")
        ttk.Label(tab, textvariable=self.motor_status).pack(anchor="w", pady=(4, 12))

        buttons = ttk.Frame(tab)
        buttons.pack(anchor="w")
        self.motor_buttons = []
        for i in range(1, 5):
            b = ttk.Button(buttons, text=f"测试 M{i}", command=lambda n=i: self.test_motor(n))
            b.grid(row=0, column=i - 1, padx=6, pady=6)
            self.motor_buttons.append(b)
        ttk.Button(buttons, text="STOP ALL", command=self.stop_all_motors).grid(row=0, column=4, padx=16, pady=6)

        arm_box = ttk.LabelFrame(tab, text="解锁命令", padding=10)
        arm_box.pack(fill="x", pady=20)
        ttk.Button(arm_box, text="请求 ARM", command=self.request_arm).pack(side="left", padx=4)
        ttk.Button(arm_box, text="DISARM / 停机", command=self.request_disarm).pack(side="left", padx=4)
        ttk.Label(arm_box, text="默认调试不需要 ARM；首飞前才使用，并保持油门最低。").pack(side="left", padx=12)

    def _build_checklist_tab(self) -> None:
        tab = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(tab, text="首飞检查")
        ttk.Button(tab, text="运行首飞前自检", command=self.run_self_test).pack(anchor="w")

        items = [
            "已拆桨完成全部电机方向测试",
            "I2C 可见 0x68 和 0x76",
            "IMU 姿态方向正确: 前倾 pitch 方向符合预期",
            "RC 四个主通道和 ARM 开关方向正确",
            "电池电压读数合理且不 critical",
            "Failsafe 测试: 关闭遥控器后电机停止",
            "安装桨前确认 motor stop / disarm 可立即停机",
            "首次离地只用小油门、Angle 模式、低高度短时测试",
        ]
        self.checklist_vars: list[BooleanVar] = []
        box = ttk.LabelFrame(tab, text="检查项", padding=10)
        box.pack(fill="x", pady=10)
        for item in items:
            var = BooleanVar(value=False)
            self.checklist_vars.append(var)
            ttk.Checkbutton(box, text=item, variable=var).pack(anchor="w", pady=3)

        ttk.Button(tab, text="保存首飞检查报告", command=self.save_checklist_report).pack(anchor="w", pady=8)

    def refresh_ports(self) -> None:
        self.port_items = available_ports()
        self.port_combo["values"] = [label for _, label in self.port_items]
        labels = {label for _, label in self.port_items}
        if self.selected_port.get() not in labels:
            self.selected_port.set("")
        real_ports = [(port, label) for port, label in self.port_items if port != "FAKE"]
        if real_ports and not self.selected_port.get():
            self.selected_port.set(real_ports[0][1])
        elif self.port_items and not self.selected_port.get():
            self.selected_port.set(self.port_items[0][1])

    def _selected_port_id(self) -> str:
        selected = self.selected_port.get()
        for port, label in self.port_items:
            if selected == label:
                return port
        return ""

    def connect(self) -> None:
        if self.backend is not None:
            self.disconnect()
        port = self._selected_port_id()
        if not port:
            messagebox.showerror("连接失败", "请先选择真实 COM 口；演示模式请选择 FAKE。")
            return
        if port == "FAKE":
            if not messagebox.askyesno("演示模式", "FAKE 只是假串口演示，不能验证真实飞控。仍要连接 FAKE 吗？"):
                return
        try:
            self.backend = create_backend(port, self.events)
            self.backend.connect(port)
            self.rx_bytes = 0
            self.tx_count = 0
            self.last_rx_time = 0.0
            self.last_tx_time = 0.0
            self.no_rx_warning_shown = False
            self.cli_ready = False
            self.connect_started_at = time.monotonic()
            self.prompt_probe_sent_at = 0.0
            self.command_queue.clear()
            self.waiting_for_prompt = False
            self.command_inflight = ""
            self.command_deadline = 0.0
            self.command_gap_until = 0.0
            self.command_sent_rx_count = 0
            self.command_quiet_deadline = 0.0
            self._update_link_stats()
            self.connected_text.set(f"已连接: {port}，等待 CLI")
            self.append_log(f"\n[HOST] connected {port}, waiting for CLI ready/prompt\n")
            self.after(1300, self._probe_cli_prompt)
            self.after(3200, self._warn_if_no_rx_after_connect)
        except Exception as exc:
            self.backend = None
            messagebox.showerror("连接失败", str(exc))

    def disconnect(self) -> None:
        self.command_queue.clear()
        self.waiting_for_prompt = False
        self.command_inflight = ""
        self.command_sent_rx_count = 0
        self.command_quiet_deadline = 0.0
        self.cli_ready = False
        self.polling = False
        self.receiver_polling = False
        if self.backend is not None:
            self.backend.close()
            self.backend = None
        self.connected_text.set("未连接")
        self._update_link_stats()

    def send_command(self, command: str, *, priority: bool = False, clear_pending: bool = False) -> None:
        command = command.strip()
        if not command:
            return
        if self.backend is None or not self.backend.connected:
            self.append_log(f"\n[HOST] 未连接，无法发送: {command}\n")
            return
        if clear_pending:
            self.command_queue.clear()
            self.waiting_for_prompt = False
            self.command_inflight = ""
        if priority:
            self._send_command_now(command, wait_for_prompt=False)
            return
        self.command_queue.append(command)
        self._pump_command_queue()

    def queue_commands(self, commands: list[str], *, clear_pending: bool = False) -> None:
        if clear_pending:
            self.command_queue.clear()
            self.waiting_for_prompt = False
            self.command_inflight = ""
        for command in commands:
            command = command.strip()
            if command:
                self.command_queue.append(command)
        self._pump_command_queue()

    def _send_command_now(self, command: str, *, wait_for_prompt: bool = True) -> None:
        if self.backend is None or not self.backend.connected:
            return
        self.last_tx_time = time.monotonic()
        self.waiting_for_prompt = wait_for_prompt
        self.command_inflight = command if wait_for_prompt else ""
        self.command_sent_rx_count = self.rx_bytes
        duration = self._command_delay_s(command)
        self.command_deadline = self.last_tx_time + duration
        self.command_quiet_deadline = self.last_tx_time + min(duration, 0.22)
        if not wait_for_prompt and hasattr(self.backend, "send_now"):
            self.backend.send_now(command)
        else:
            self.backend.send(command)

    def _pump_command_queue(self) -> None:
        if self.backend is None or not self.backend.connected:
            return
        if not self.cli_ready:
            now = time.monotonic()
            if self.connect_started_at > 0.0 and (now - self.connect_started_at) < 1.2:
                self._probe_cli_prompt()
                self.after(150, self._pump_command_queue)
                return
            self._mark_cli_ready("timeout")
        if self.waiting_for_prompt:
            return
        if not self.command_queue:
            return
        now = time.monotonic()
        if now < self.command_gap_until:
            self.after(int((self.command_gap_until - now) * 1000) + 10, self._pump_command_queue)
            return
        self._send_command_now(self.command_queue.popleft(), wait_for_prompt=True)

    def queue_slow_lines(self, text: str, delay_ms: int = 80) -> None:
        lines = [line.strip() for line in text.replace("\r", "\n").split("\n")]
        commands = [line for line in lines if line]
        if not commands:
            return

        def push_one(index: int = 0) -> None:
            if index >= len(commands):
                return
            self.send_command(commands[index])
            self.after(delay_ms, lambda: push_one(index + 1))

        push_one()

    @staticmethod
    def _command_delay_s(command: str) -> float:
        head = command.split(" ", 1)[0]
        if head in ("motor", "motors", "arm", "disarm", "log"):
            return 0.28
        if head in ("status", "clock", "imu", "baro", "rc", "rcmap", "batt", "heap", "pid"):
            return 0.45
        if head == "help":
            return 0.9
        if head == "tasks":
            return 1.4
        if head == "i2cscan":
            return 1.3
        return 0.6

    def _probe_cli_prompt(self) -> None:
        if self.backend is None or not self.backend.connected or self.cli_ready:
            return
        now = time.monotonic()
        if self.prompt_probe_sent_at > 0.0 and (now - self.prompt_probe_sent_at) < 2.0:
            return
        if self.connect_started_at > 0.0 and (now - self.connect_started_at) < 1.0:
            return
        self.prompt_probe_sent_at = now
        self.backend.send("")
        self.append_log("\n[HOST] 等待 CLI 提示符，发送空回车探测\n")
        self.after(2100, self._probe_cli_prompt)

    def _mark_cli_ready(self, reason: str = "prompt") -> None:
        if self.cli_ready:
            return
        self.cli_ready = True
        if self.backend is not None and self.backend.connected:
            self.connected_text.set(f"已连接: {self._selected_port_id()}，CLI 就绪")
        if reason == "timeout":
            self.append_log("\n[HOST] 未捕获完整提示符，已按延时模式发送队列命令\n")
        else:
            self.append_log("\n[HOST] CLI ready，开始按队列发送命令\n")
        self._pump_command_queue()

    def _mark_command_complete(self, reason: str) -> None:
        if self.waiting_for_prompt:
            self.waiting_for_prompt = False
            self.command_inflight = ""
            self.command_deadline = 0.0
            self.command_gap_until = time.monotonic() + 0.04
            self.after(40, self._pump_command_queue)

    def _command_watchdog(self) -> None:
        if self.waiting_for_prompt:
            now = time.monotonic()
            got_response = self.rx_bytes > self.command_sent_rx_count
            if got_response and now > self.command_quiet_deadline:
                self._mark_command_complete("response")
            elif now > self.command_deadline:
                self._mark_command_complete("delay")
        self.after(100, self._command_watchdog)

    def run_serial_diagnosis(self) -> None:
        if self.backend is None or not self.backend.connected:
            messagebox.showwarning("串口诊断", "请先选择 COM 口并点击连接。")
            return
        before = self.rx_bytes
        self.append_log("\n[HOST] 串口诊断: 发送 help/status，并等待飞控回复...\n")
        self.queue_commands(["help", "status"], clear_pending=True)
        self.after(2500, lambda b=before: self._finish_serial_diagnosis(b))

    def start_auto_probe(self) -> None:
        if self.backend is not None:
            self.disconnect()
        self.refresh_ports()
        ports = [(p, label) for p, label in self.port_items if p != "FAKE"]
        if not ports:
            messagebox.showwarning("自动探测", "没有发现真实 COM 口。请先插入板载 USB-C 或 USB-TTL。")
            return
        self.append_log("\n[HOST] 自动探测开始：逐个打开 COM，等待启动文本并尝试 help/status...\n")
        threading.Thread(target=self._auto_probe_worker, args=(ports,), daemon=True).start()

    def _auto_probe_worker(self, ports: list[tuple[str, str]]) -> None:
        for port, label in ports:
            self.events.put(("info", f"probe {label}"))
            try:
                text = probe_port(port)
            except Exception as exc:
                self.events.put(("info", f"{port} 探测失败: {exc}"))
                continue

            if text:
                preview = text.replace("\r", "").strip()
                self.events.put(("info", f"{port} 收到 {len(text)} 字节: {preview[:160]}"))
            else:
                self.events.put(("info", f"{port} 没有收到字节"))

            if self._looks_like_flight_controller(text):
                self.events.put(("probe_found", label))
                return
        self.events.put(("probe_done", "not_found"))

    @staticmethod
    def _looks_like_flight_controller(text: str) -> bool:
        return any(marker in text for marker in (
            "NAZE32 custom firmware boot",
            "PRECLK USART1",
            "CLOCK OK",
            "HAL USART1 OK",
            "ERROR_HANDLER",
            "cmd: help status",
            "armed=",
        ))

    def _finish_serial_diagnosis(self, rx_before: int) -> None:
        if self.rx_bytes > rx_before:
            self.append_log(f"\n[HOST] 串口诊断: 已收到 {self.rx_bytes - rx_before} 字节，串口 RX 正常。\n")
            return
        self._show_no_rx_help("串口能打开，但诊断命令没有收到任何字节。")

    def send_manual(self) -> None:
        cmd = self.manual_command.get().strip()
        self.manual_command.set("")
        if not cmd:
            return
        self.command_history.append(cmd)
        self.command_history_index = len(self.command_history)
        if "\n" in cmd or "\r" in cmd:
            self.queue_slow_lines(cmd)
        else:
            self.send_command(cmd)

    def send_multiline_manual(self) -> None:
        text = self.manual_command.get()
        if "\n" in text or "\r" in text:
            self.manual_command.set("")
            self.queue_slow_lines(text, delay_ms=100)
            return

        dialog = tk.Toplevel(self)
        dialog.title("慢速发送多行命令")
        dialog.geometry("520x360")
        dialog.transient(self)
        dialog.grab_set()
        box = tk.Text(dialog, wrap="none", font=("Consolas", 10))
        box.pack(fill="both", expand=True, padx=10, pady=(10, 6))
        if text:
            box.insert("1.0", text)

        buttons = ttk.Frame(dialog)
        buttons.pack(fill="x", padx=10, pady=(0, 10))

        def do_send() -> None:
            payload = box.get("1.0", "end").strip()
            dialog.destroy()
            self.manual_command.set("")
            self.queue_slow_lines(payload, delay_ms=100)

        ttk.Button(buttons, text="发送", command=do_send).pack(side="right", padx=4)
        ttk.Button(buttons, text="取消", command=dialog.destroy).pack(side="right", padx=4)

    def _history_prev(self, _event) -> str:
        if not self.command_history:
            return "break"
        self.command_history_index = max(0, self.command_history_index - 1)
        self.manual_command.set(self.command_history[self.command_history_index])
        return "break"

    def _history_next(self, _event) -> str:
        if not self.command_history:
            return "break"
        self.command_history_index = min(len(self.command_history), self.command_history_index + 1)
        if self.command_history_index >= len(self.command_history):
            self.manual_command.set("")
        else:
            self.manual_command.set(self.command_history[self.command_history_index])
        return "break"

    def run_self_test(self) -> None:
        if self.self_test_running:
            return
        self.self_test_running = True
        self.self_test_summary.configure(text="自检运行中...")
        rx_before = self.rx_bytes
        commands = ["status", "i2cscan", "imu", "baro", "rcmap", "rc", "batt", "heap", "tasks", "pid"]
        commands.insert(1, "clock")
        self.queue_commands(commands, clear_pending=True)
        self.after(500, self._check_self_test_done)
        self.after(5000, lambda b=rx_before: self._warn_if_self_test_no_rx(b))

    def _check_self_test_done(self) -> None:
        if not self.self_test_running:
            return
        if not self.command_queue and not self.waiting_for_prompt:
            self._finish_self_test()
            return
        self.after(200, self._check_self_test_done)

    def _finish_self_test(self) -> None:
        self.self_test_running = False
        self._refresh_self_test()
        self.self_test_summary.configure(text="自检完成。若有红色项，请先处理后再测试电机。")

    def start_polling(self) -> None:
        self.polling = True
        self._poll_once()

    def stop_polling(self) -> None:
        self.polling = False

    def _poll_once(self) -> None:
        if not self.polling:
            return
        if self.terminal_mode.get() and self.notebook.tab(self.notebook.select(), "text") == "连接/日志":
            self.after(1500, self._poll_once)
            return
        if not self.command_queue and not self.waiting_for_prompt:
            self.queue_commands(["status", "imu", "baro", "control", "rc", "batt"])
        self.after(700, self._refresh_state_views)
        self.after(1500, self._poll_once)

    def start_receiver_polling(self) -> None:
        self.receiver_polling = True
        self.queue_commands(["rcmap", "rc"])
        self._receiver_poll_once()

    def stop_receiver_polling(self) -> None:
        self.receiver_polling = False

    def _receiver_poll_once(self) -> None:
        if not self.receiver_polling:
            return
        if self.terminal_mode.get() and self.notebook.tab(self.notebook.select(), "text") == "连接/日志":
            self.after(250, self._receiver_poll_once)
            return
        if not self.command_queue and not self.waiting_for_prompt:
            self.queue_commands(["rc"])
        self.after(250, self._receiver_poll_once)

    def test_motor(self, motor: int) -> None:
        if not self._motor_test_allowed():
            return
        value = self._clamp_motor_value()
        self.polling = False
        self.receiver_polling = False
        unlock_cmd = "motor unlock bench" if self.allow_failsafe_motor_test.get() else "motor unlock"
        self.send_command(unlock_cmd, priority=True, clear_pending=True)
        self.after(220, lambda n=motor, v=value: self.send_command(f"motor {n} {v}", priority=True))
        self.after(500, lambda: self.queue_commands(["status"], clear_pending=False))
        self.after(4500, self.stop_all_motors)

    def stop_all_motors(self) -> None:
        self.waiting_for_prompt = False
        self.command_inflight = ""
        self.send_command("motor stop", priority=True, clear_pending=True)
        self.after(80, lambda: self.send_command("motors 0", priority=True))
        self.after(250, lambda: self.queue_commands(["status"], clear_pending=False))

    def request_arm(self) -> None:
        if messagebox.askyesno("确认 ARM", "确认要发送 arm？请确保油门最低、飞控水平、周围安全。"):
            self.send_command("arm")

    def request_disarm(self) -> None:
        self.send_command("disarm", priority=True, clear_pending=True)
        self.after(50, self.stop_all_motors)

    def save_log(self) -> None:
        path = filedialog.asksaveasfilename(
            title="保存原始日志",
            defaultextension=".txt",
            filetypes=[("Text", "*.txt"), ("All files", "*.*")],
            initialfile=f"flight_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt",
        )
        if path:
            Path(path).write_text("".join(self.raw_log), encoding="utf-8")

    def save_csv(self) -> None:
        path = filedialog.asksaveasfilename(
            title="保存状态 CSV",
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv"), ("All files", "*.*")],
            initialfile=f"flight_state_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
        )
        if path:
            self.parser.save_csv(Path(path))

    def save_checklist_report(self) -> None:
        path = filedialog.asksaveasfilename(
            title="保存首飞检查报告",
            defaultextension=".txt",
            filetypes=[("Text", "*.txt"), ("All files", "*.*")],
            initialfile=f"preflight_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt",
        )
        if not path:
            return
        state = self.parser.latest()
        lines = [
            "NAZE32 首飞检查报告",
            f"生成时间: {datetime.now().isoformat(timespec='seconds')}",
            "",
            "检查项:",
        ]
        for i, var in enumerate(self.checklist_vars, start=1):
            lines.append(f"{i}. {'[OK]' if var.get() else '[  ]'}")
        lines.extend(["", "最新解析状态:", repr(state)])
        Path(path).write_text("\n".join(lines), encoding="utf-8")

    def append_log(self, text: str) -> None:
        self.raw_log.append(text)
        self.log_text.insert("end", text)
        self.log_text.see("end")

    def _process_events(self) -> None:
        try:
            while True:
                kind, text = self.events.get_nowait()
                if kind == "rx":
                    self.rx_bytes += len(text.encode("utf-8", errors="replace"))
                    self.last_rx_time = time.monotonic()
                    self._update_link_stats()
                    self.append_log(text)
                    self._feed_parser(text)
                    has_prompt = self._chunk_has_prompt(text)
                    if has_prompt:
                        self.parser.state["prompt_seen"] = True
                    if has_prompt or ("CLI ready" in text) or ("init: tasks" in text):
                        self._mark_cli_ready()
                elif kind == "tx":
                    self.tx_count += 1
                    self._update_link_stats()
                    if text:
                        self.append_log(f"\n>> {text}\n")
                    else:
                        self.append_log("\n>> <ENTER>\n")
                elif kind == "error":
                    self.append_log(f"\n[ERROR] {text}\n")
                elif kind == "probe_found":
                    self.selected_port.set(text)
                    self.append_log(f"\n[HOST] 自动探测找到飞控串口: {text}\n")
                    messagebox.showinfo("自动探测", f"找到飞控串口:\n{text}\n\n已自动选中，请点击连接。")
                elif kind == "probe_done":
                    self.append_log("\n[HOST] 自动探测结束：没有找到飞控输出。请确认固件已下载、板子供电、COM 驱动正常。\n")
                    messagebox.showwarning("自动探测", "没有找到飞控串口。")
                else:
                    self.append_log(f"\n[HOST] {text}\n")
        except queue.Empty:
            pass
        self.after(50, self._process_events)

    def _update_link_stats(self) -> None:
        if self.backend is None or not self.backend.connected:
            self.link_stats_text.set(f"TX {self.tx_count} 条 | RX {self.rx_bytes} 字节 | 未连接")
            return
        if self.last_rx_time > 0.0:
            age = max(0.0, time.monotonic() - self.last_rx_time)
            rx_text = f"最近接收 {age:.1f}s 前"
        else:
            rx_text = "未收到数据"
        pending = len(self.command_queue) + (1 if self.waiting_for_prompt else 0)
        self.link_stats_text.set(f"TX {self.tx_count} 条 | RX {self.rx_bytes} 字节 | {rx_text} | 待发/等待 {pending}")

    def _warn_if_no_rx_after_connect(self) -> None:
        if self.backend is None or not self.backend.connected:
            return
        if self.rx_bytes == 0 and not self.no_rx_warning_shown:
            self.no_rx_warning_shown = True
            self._show_no_rx_help("已连接 3 秒，但没有收到飞控任何字节。")

    def _warn_if_self_test_no_rx(self, rx_before: int) -> None:
        if self.backend is None or not self.backend.connected:
            return
        if self.rx_bytes == rx_before:
            self._show_no_rx_help("一键自检命令已经发出，但飞控没有回任何字节。")

    def _show_no_rx_help(self, reason: str) -> None:
        text = (
            f"{reason}\n\n"
            "这通常不是上位机解析失败，而是串口 RX 没有数据。\n"
            "请按顺序检查:\n"
            "1. USB-TTL RX 接飞控 PA9，USB-TTL TX 接飞控 PA10，GND 必须共地。\n"
            "2. 不要接 PA2/PA3；那组是 CRSF 接收机串口，波特率 420000。\n"
            "3. 串口参数固定 115200, 8N1, 无流控。\n"
            "4. 连接后上位机会等待启动文本，并每隔约 2 秒发一次空回车探测提示符；不用卡复位时机。\n"
            "5. 如果用外接 USB-TTL，可把 USB-TTL 的 TX 和 RX 短接做回环测试；若回环也没字符，说明选错 COM 或转接器/驱动有问题。\n"
            "6. 如果用板载 USB-C，优先选择插入该 USB-C 后新增的 COM 口。"
        )
        self.append_log("\n[HOST] " + text.replace("\n", "\n[HOST] ") + "\n")
        messagebox.showwarning("没有收到串口数据", text)

    @staticmethod
    def _chunk_has_prompt(text: str) -> bool:
        return ">" in text

    def _feed_parser(self, text: str) -> None:
        normalized = text.replace("\r\n", "\n").replace("\r", "\n")
        self.rx_buffer += normalized
        while "\n" in self.rx_buffer:
            line, self.rx_buffer = self.rx_buffer.split("\n", 1)
            self.parser.parse_line(line)

    def _refresh_state_views(self) -> None:
        self._update_link_stats()
        self._refresh_self_test()
        self._refresh_monitor()
        self._refresh_receiver()
        self._refresh_motor_buttons()
        self.after(500, self._refresh_state_views)

    def _refresh_self_test(self) -> None:
        s = self.parser.latest()
        status = s.get("status", {})
        imu = s.get("imu", {})
        baro = s.get("baro", {})
        althold = s.get("althold", {})
        rc = s.get("rc", {})
        batt = s.get("battery", {})
        i2c = s.get("i2c", {})
        heap = s.get("heap", {})
        clock = s.get("clock", {})

        self._set_check("boot", bool(s.get("boot_seen")), "已看到 boot" if s.get("boot_seen") else "等待 boot")
        clock_ok = isinstance(clock, dict) and clock.get("sys_hz")
        self._set_check("clock", bool(clock_ok) and not bool(clock.get("fallback")), self._clock_text(clock))
        addrs = i2c.get("addresses", []) if isinstance(i2c, dict) else []
        self._set_check("i2c", "0x68" in addrs and "0x76" in addrs, f"{' '.join(addrs) if addrs else '等待'}")
        self._set_check("imu", bool(imu.get("ok")) if isinstance(imu, dict) else False, f"ok={int(bool(imu.get('ok')))}" if isinstance(imu, dict) and imu else "等待")
        self._set_check("baro", bool(baro.get("ok")) if isinstance(baro, dict) else False, f"ok={int(bool(baro.get('ok')))}" if isinstance(baro, dict) and baro else "等待")
        self._set_check("rc", bool(rc.get("connected")) and not bool(rc.get("failsafe")) if isinstance(rc, dict) else False, self._rc_text(rc))
        battery_ok = isinstance(batt, dict) and batt.get("voltage_mv") and not batt.get("critical")
        self._set_check("batt", bool(battery_ok), self._battery_text(batt))
        failsafe_ok = isinstance(status, dict) and status.get("failsafe") is False
        self._set_check("failsafe", bool(failsafe_ok), f"failsafe={int(status.get('failsafe', 1))}" if isinstance(status, dict) and status else "等待")
        self._set_check("heap", bool(heap), f"free={heap.get('free')} min={heap.get('min')} rxdrop={heap.get('rxdrop', 0)}" if isinstance(heap, dict) and heap else "等待")

    def _refresh_monitor(self) -> None:
        s = self.parser.latest()
        status = s.get("status", {})
        imu = s.get("imu", {})
        baro = s.get("baro", {})
        althold = s.get("althold", {})
        rc = s.get("rc", {})
        batt = s.get("battery", {})
        att = s.get("attitude", {})
        motors = s.get("motors", {})

        if isinstance(status, dict):
            self.monitor_vars["armed"].set(self._yes_no(status.get("armed")))
            self.monitor_vars["failsafe"].set(self._yes_no(status.get("failsafe")))
            self.monitor_vars["rc"].set(self._ok_bad(status.get("rc_ok")))
            self.monitor_vars["imu"].set(self._ok_bad(status.get("imu_ok") if "imu_ok" in status else imu.get("ok") if isinstance(imu, dict) else None))
            self.monitor_vars["baro"].set(self._ok_bad(status.get("baro_ok") if "baro_ok" in status else baro.get("ok") if isinstance(baro, dict) else None))
        if isinstance(althold, dict) and althold:
            self.monitor_vars["althold"].set(self._althold_text(althold))
        self.monitor_vars["battery"].set(self._battery_text(batt))
        if isinstance(att, dict):
            self.monitor_vars["att"].set(f"roll={att.get('roll', '-')} pitch={att.get('pitch', '-')} yaw={att.get('yaw', '-')}")
        if isinstance(baro, dict):
            self.monitor_vars["baro_detail"].set(
                f"pressure={baro.get('pressure_pa', '-')}Pa altitude={baro.get('altitude_cm', '-')}cm "
                f"vel={baro.get('velocity_cms', '-')}cm/s"
            )
        if isinstance(rc, dict):
            self.monitor_vars["sticks"].set(
                f"r={rc.get('roll', '-')} p={rc.get('pitch', '-')} y={rc.get('yaw', '-')} t={rc.get('throttle', '-')}"
            )
        if isinstance(motors, dict):
            self.monitor_vars["motors"].set(str(motors.get("m", "-")))

    def _refresh_receiver(self) -> None:
        s = self.parser.latest()
        rc = s.get("rc", {})
        rcmap = s.get("rcmap", {})
        if isinstance(rcmap, dict) and rcmap:
            self.receiver_map_text.set(
                "AETR 映射: "
                f"Roll={rcmap.get('roll', 'CH1')} Pitch={rcmap.get('pitch', 'CH2')} "
                f"Thr={rcmap.get('throttle', 'CH3')} Yaw={rcmap.get('yaw', 'CH4')} "
                f"ARM={rcmap.get('arm', 'CH5')} BARO={rcmap.get('baro', 'CH6')}"
            )
        if not isinstance(rc, dict) or not rc:
            self.receiver_status_text.set("等待接收机数据")
            return

        self.receiver_status_text.set(
            f"connected={int(bool(rc.get('connected')))} failsafe={int(bool(rc.get('failsafe')))} "
            f"ARM={int(bool(rc.get('arm')))} BARO={int(bool(rc.get('baro')))} age={rc.get('age_ms', '-')}ms"
        )
        self.receiver_norm_text.set(
            f"roll={rc.get('roll', '-'):>5} pitch={rc.get('pitch', '-'):>5} "
            f"yaw={rc.get('yaw', '-'):>5} throttle={rc.get('throttle', '-'):>4}"
        )

        raw = rc.get("raw", [])
        if not isinstance(raw, list):
            raw = []
        names = ["Roll", "Pitch", "Thr", "Yaw", "ARM", "BARO", "AUX3", "AUX4"]
        for index, value_var in enumerate(self.receiver_channel_values):
            raw_value = int(raw[index]) if index < len(raw) else 0
            percent = self._raw_channel_percent(raw_value)
            value_var.set(percent)
            if index < len(self.receiver_channel_labels):
                self.receiver_channel_labels[index].set(f"CH{index + 1} {names[index]}: {raw_value:4d}  {percent:3d}%")

    def _set_check(self, key: str, ok: bool, text: str) -> None:
        label = self.check_labels[key]
        label.configure(text=("OK: " if ok else "检查: ") + text, foreground=("#15803d" if ok else "#b91c1c"))

    def _toggle_advanced_motor(self) -> None:
        self.motor_spin.configure(to=300 if self.advanced_motor.get() else 150)
        self._clamp_motor_value()

    def _clamp_motor_value(self) -> int:
        max_value = 300 if self.advanced_motor.get() else 150
        try:
            value = int(self.motor_permille.get())
        except Exception:
            value = 0
        value = max(0, min(max_value, value))
        self.motor_permille.set(value)
        return value

    def _motor_test_allowed(self) -> bool:
        if not self.props_removed.get():
            messagebox.showwarning("禁止测试", "请先勾选确认已经拆掉全部螺旋桨。")
            return False
        if self.backend is None or not self.backend.connected:
            messagebox.showwarning("禁止测试", "串口未连接。")
            return False
        status = self.parser.latest().get("status", {})
        if isinstance(status, dict) and status.get("failsafe") is True and not self.allow_failsafe_motor_test.get():
            messagebox.showwarning("禁止测试", "当前 failsafe=1。请先确认接收机在线、油门最低、飞控未处于失控保护。")
            return False
        return True

    def _refresh_motor_buttons(self) -> None:
        state = "normal" if self.props_removed.get() else "disabled"
        for button in self.motor_buttons:
            button.configure(state=state)
        s = self.parser.latest()
        status = s.get("status", {})
        if isinstance(status, dict) and status:
            self.motor_status.set(
                f"armed={int(bool(status.get('armed')))} failsafe={int(bool(status.get('failsafe', True)))} "
                f"motor_test={int(bool(status.get('motor_test')))}"
            )
        else:
            self.motor_status.set("等待 status")

    def _on_close(self) -> None:
        try:
            self.stop_all_motors()
            time.sleep(0.05)
        except Exception:
            pass
        self.disconnect()
        self.destroy()

    @staticmethod
    def _yes_no(value: object) -> str:
        if value is True:
            return "是"
        if value is False:
            return "否"
        return "-"

    @staticmethod
    def _ok_bad(value: object) -> str:
        if value is True:
            return "OK"
        if value is False:
            return "异常"
        return "-"

    @staticmethod
    def _althold_text(althold: object) -> str:
        if not isinstance(althold, dict) or not althold:
            return "等待"
        return (
            f"req={int(bool(althold.get('requested')))} "
            f"ready={int(bool(althold.get('ready')))} "
            f"active={int(bool(althold.get('active')))} "
            f"tilt={int(bool(althold.get('tilt_ok', True)))} "
            f"thr={int(bool(althold.get('throttle_ok', True)))} "
            f"hold={althold.get('hold_altitude_cm', althold.get('altitude_cm', '-'))}cm "
            f"vel={althold.get('velocity_cms', '-')}cm/s "
            f"out={althold.get('output', '-')}"
        )

    @staticmethod
    def _battery_text(batt: object) -> str:
        if not isinstance(batt, dict) or not batt:
            return "等待"
        adc = batt.get("adc_mv", "-")
        cells = batt.get("cells", "-")
        return f"{batt.get('voltage_mv', '-')}mV adc={adc}mV {cells}S {batt.get('percent', '-')}% low={int(bool(batt.get('low')))} critical={int(bool(batt.get('critical')))}"

    @staticmethod
    def _rc_text(rc: object) -> str:
        if not isinstance(rc, dict) or not rc:
            return "等待"
        return f"connected={int(bool(rc.get('connected')))} failsafe={int(bool(rc.get('failsafe')))} age={rc.get('age_ms', '-')}"

    @staticmethod
    def _raw_channel_percent(raw_value: int) -> int:
        percent = int((raw_value - 172) * 100 / (1811 - 172))
        return max(0, min(100, percent))

    @staticmethod
    def _clock_text(clock: object) -> str:
        if not isinstance(clock, dict) or not clock:
            return "等待"
        mhz = int(clock.get("sys_hz", 0)) / 1000000.0
        return f"src={clock.get('src', '-')} pll={clock.get('pll', '-')} sys={mhz:.1f}MHz fallback={int(bool(clock.get('fallback')))}"


def main() -> int:
    app = FlightDebugGui()
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
