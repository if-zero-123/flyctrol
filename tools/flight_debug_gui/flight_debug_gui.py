# -*- coding: utf-8 -*-
"""Windows GUI for debugging the NAZE32 custom flight firmware."""

from __future__ import annotations

import queue
import sys
import threading
import time
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
        self.self_test_running = False
        self.rx_bytes = 0
        self.tx_count = 0
        self.last_rx_time = 0.0
        self.last_tx_time = 0.0
        self.no_rx_warning_shown = False

        self.connected_text = StringVar(value="未连接")
        self.link_stats_text = StringVar(value="TX 0 条 | RX 0 字节 | 未收到数据")
        self.selected_port = StringVar(value="")
        self.manual_command = StringVar(value="")
        self.motor_permille = IntVar(value=80)
        self.props_removed = BooleanVar(value=False)
        self.advanced_motor = BooleanVar(value=False)

        self._build_ui()
        self.refresh_ports()
        self.after(50, self._process_events)
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

        hint = (
            "连接后不用抢复位；固件会每秒打印 hb tick=...。板载 USB-C 通常就是调试串口。"
        )
        ttk.Label(tab, text=hint).pack(fill="x", pady=(8, 4))
        ttk.Label(tab, textvariable=self.link_stats_text, foreground="#1d4ed8").pack(fill="x", pady=(0, 4))

        cmd = ttk.Frame(tab)
        cmd.pack(fill="x", pady=(0, 6))
        ttk.Label(cmd, text="手动命令:").pack(side="left")
        entry = ttk.Entry(cmd, textvariable=self.manual_command)
        entry.pack(side="left", fill="x", expand=True, padx=6)
        entry.bind("<Return>", lambda _e: self.send_manual())
        ttk.Button(cmd, text="发送", command=self.send_manual).pack(side="left")

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
        self.self_test_summary = ttk.Label(tab, text="点击一键自检后，会自动执行 status/i2cscan/imu/baro/rc/batt/heap/tasks。")
        self.self_test_summary.pack(fill="x", pady=8)

        self.check_labels: dict[str, ttk.Label] = {}
        grid = ttk.Frame(tab)
        grid.pack(fill="x")
        checks = [
            ("boot", "启动输出"),
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
            "armed", "failsafe", "rc", "imu", "baro", "battery", "att", "baro_detail", "sticks", "motors"
        ]}
        rows = [
            ("解锁", "armed"),
            ("Failsafe", "failsafe"),
            ("RC", "rc"),
            ("IMU", "imu"),
            ("BARO", "baro"),
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
        if self.port_items and not self.selected_port.get():
            self.selected_port.set(self.port_items[0][1])

    def _selected_port_id(self) -> str:
        selected = self.selected_port.get()
        for port, label in self.port_items:
            if selected == label:
                return port
        return "FAKE"

    def connect(self) -> None:
        if self.backend is not None:
            self.disconnect()
        port = self._selected_port_id()
        try:
            self.backend = create_backend(port, self.events)
            self.backend.connect(port)
            self.rx_bytes = 0
            self.tx_count = 0
            self.last_rx_time = 0.0
            self.last_tx_time = 0.0
            self.no_rx_warning_shown = False
            self._update_link_stats()
            self.connected_text.set(f"已连接: {port}")
            self.append_log(f"\n[HOST] connected {port}\n")
            self.after(500, lambda: self.send_command("help"))
            self.after(3200, self._warn_if_no_rx_after_connect)
        except Exception as exc:
            self.backend = None
            messagebox.showerror("连接失败", str(exc))

    def disconnect(self) -> None:
        if self.backend is not None:
            self.backend.close()
            self.backend = None
        self.connected_text.set("未连接")
        self._update_link_stats()

    def send_command(self, command: str) -> None:
        if not command:
            return
        if self.backend is None or not self.backend.connected:
            self.append_log(f"\n[HOST] 未连接，无法发送: {command}\n")
            return
        self.last_tx_time = time.monotonic()
        self.backend.send(command)

    def run_serial_diagnosis(self) -> None:
        if self.backend is None or not self.backend.connected:
            messagebox.showwarning("串口诊断", "请先选择 COM 口并点击连接。")
            return
        before = self.rx_bytes
        self.append_log("\n[HOST] 串口诊断: 发送 help/status，并等待飞控回复...\n")
        self.send_command("help")
        self.after(350, lambda: self.send_command("status"))
        self.after(2500, lambda b=before: self._finish_serial_diagnosis(b))

    def start_auto_probe(self) -> None:
        if self.backend is not None:
            self.disconnect()
        self.refresh_ports()
        ports = [(p, label) for p, label in self.port_items if p != "FAKE"]
        if not ports:
            messagebox.showwarning("自动探测", "没有发现真实 COM 口。请先插入板载 USB-C 或 USB-TTL。")
            return
        self.append_log("\n[HOST] 自动探测开始：逐个打开 COM，等待 hb/help/status...\n")
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
            "hb tick=",
            "cmd: help status tasks",
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
        self.send_command(cmd)

    def run_self_test(self) -> None:
        if self.self_test_running:
            return
        self.self_test_running = True
        self.self_test_summary.configure(text="自检运行中...")
        rx_before = self.rx_bytes
        commands = ["status", "i2cscan", "imu", "baro", "rc", "batt", "heap", "tasks", "pid"]
        for index, cmd in enumerate(commands):
            self.after(index * 300, lambda c=cmd: self.send_command(c))
        self.after(len(commands) * 350 + 600, self._finish_self_test)
        self.after(len(commands) * 350 + 800, lambda b=rx_before: self._warn_if_self_test_no_rx(b))

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
        for index, cmd in enumerate(["status", "imu", "baro", "rc", "batt"]):
            self.after(index * 120, lambda c=cmd: self.send_command(c))
        self.after(700, self._refresh_state_views)
        self.after(1000, self._poll_once)

    def test_motor(self, motor: int) -> None:
        if not self._motor_test_allowed():
            return
        value = self._clamp_motor_value()
        self.send_command("motor unlock")
        self.after(250, lambda: self.send_command(f"motor {motor} {value}"))
        self.after(4500, self.stop_all_motors)

    def stop_all_motors(self) -> None:
        self.send_command("motor stop")
        self.after(80, lambda: self.send_command("motors 0"))

    def request_arm(self) -> None:
        if messagebox.askyesno("确认 ARM", "确认要发送 arm？请确保油门最低、飞控水平、周围安全。"):
            self.send_command("arm")

    def request_disarm(self) -> None:
        self.send_command("disarm")
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
                elif kind == "tx":
                    self.tx_count += 1
                    self._update_link_stats()
                    self.append_log(f"\n>> {text}\n")
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
        self.link_stats_text.set(f"TX {self.tx_count} 条 | RX {self.rx_bytes} 字节 | {rx_text}")

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
            "4. 固件已加入每秒 hb 心跳；不用卡复位时机，连接后等 3-5 秒 RX 字节也应该增加。\n"
            "5. 如果用外接 USB-TTL，可把 USB-TTL 的 TX 和 RX 短接做回环测试；若回环也没字符，说明选错 COM 或转接器/驱动有问题。\n"
            "6. 如果用板载 USB-C，优先选择插入该 USB-C 后新增的 COM 口。"
        )
        self.append_log("\n[HOST] " + text.replace("\n", "\n[HOST] ") + "\n")
        messagebox.showwarning("没有收到串口数据", text)

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
        self._refresh_motor_buttons()
        self.after(500, self._refresh_state_views)

    def _refresh_self_test(self) -> None:
        s = self.parser.latest()
        status = s.get("status", {})
        imu = s.get("imu", {})
        baro = s.get("baro", {})
        rc = s.get("rc", {})
        batt = s.get("battery", {})
        i2c = s.get("i2c", {})
        heap = s.get("heap", {})

        self._set_check("boot", bool(s.get("boot_seen")), "已看到 boot" if s.get("boot_seen") else "等待 boot")
        addrs = i2c.get("addresses", []) if isinstance(i2c, dict) else []
        self._set_check("i2c", "0x68" in addrs and "0x76" in addrs, f"{' '.join(addrs) if addrs else '等待'}")
        self._set_check("imu", bool(imu.get("ok")) if isinstance(imu, dict) else False, f"ok={int(bool(imu.get('ok')))}" if isinstance(imu, dict) and imu else "等待")
        self._set_check("baro", bool(baro.get("ok")) if isinstance(baro, dict) else False, f"ok={int(bool(baro.get('ok')))}" if isinstance(baro, dict) and baro else "等待")
        self._set_check("rc", bool(rc.get("connected")) and not bool(rc.get("failsafe")) if isinstance(rc, dict) else False, self._rc_text(rc))
        battery_ok = isinstance(batt, dict) and batt.get("voltage_mv") and not batt.get("critical")
        self._set_check("batt", bool(battery_ok), self._battery_text(batt))
        failsafe_ok = isinstance(status, dict) and status.get("failsafe") is False
        self._set_check("failsafe", bool(failsafe_ok), f"failsafe={int(status.get('failsafe', 1))}" if isinstance(status, dict) and status else "等待")
        self._set_check("heap", bool(heap), f"free={heap.get('free')} min={heap.get('min')}" if isinstance(heap, dict) and heap else "等待")

    def _refresh_monitor(self) -> None:
        s = self.parser.latest()
        status = s.get("status", {})
        imu = s.get("imu", {})
        baro = s.get("baro", {})
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
        self.monitor_vars["battery"].set(self._battery_text(batt))
        if isinstance(att, dict):
            self.monitor_vars["att"].set(f"roll={att.get('roll', '-')} pitch={att.get('pitch', '-')} yaw={att.get('yaw', '-')}")
        if isinstance(baro, dict):
            self.monitor_vars["baro_detail"].set(f"pressure={baro.get('pressure_pa', '-')}Pa altitude={baro.get('altitude_cm', '-')}cm")
        if isinstance(rc, dict):
            self.monitor_vars["sticks"].set(
                f"r={rc.get('roll', '-')} p={rc.get('pitch', '-')} y={rc.get('yaw', '-')} t={rc.get('throttle', '-')}"
            )
        if isinstance(motors, dict):
            self.monitor_vars["motors"].set(str(motors.get("m", "-")))

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
        if not isinstance(status, dict) or status.get("failsafe") is not False:
            messagebox.showwarning("禁止测试", "当前 failsafe 未确认正常，请先运行自检并确认 RC/IMU/电池。")
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
    def _battery_text(batt: object) -> str:
        if not isinstance(batt, dict) or not batt:
            return "等待"
        return f"{batt.get('voltage_mv', '-')}mV {batt.get('percent', '-')}% low={int(bool(batt.get('low')))} critical={int(bool(batt.get('critical')))}"

    @staticmethod
    def _rc_text(rc: object) -> str:
        if not isinstance(rc, dict) or not rc:
            return "等待"
        return f"connected={int(bool(rc.get('connected')))} failsafe={int(bool(rc.get('failsafe')))} age={rc.get('age_ms', '-')}"


def main() -> int:
    app = FlightDebugGui()
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
