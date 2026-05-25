# -*- coding: utf-8 -*-
"""Threaded serial and fake-demo backends for the flight debug GUI."""

from __future__ import annotations

import queue
import threading
import time
from typing import List, Optional, Tuple

try:
    import serial
    from serial.tools import list_ports
except Exception:  # pragma: no cover - handled at runtime in the GUI
    serial = None
    list_ports = None


Event = Tuple[str, str]


def available_ports() -> List[Tuple[str, str]]:
    ports: List[Tuple[str, str]] = []
    if list_ports is None:
        return [("FAKE", "FAKE - 演示模式，不连接真实飞控")]
    for p in sorted(list_ports.comports(), key=lambda item: item.device):
        ports.append((p.device, f"{p.device} - {p.description}"))
    ports.append(("FAKE", "FAKE - 演示模式，不连接真实飞控"))
    return ports


def probe_port(port: str, duration_s: float = 2.5) -> str:
    if serial is None:
        raise RuntimeError("缺少 pyserial，请运行: python -m pip install pyserial")

    chunks: List[str] = []
    with serial.Serial(
        port=port,
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.05,
        write_timeout=0.3,
    ) as ser:
        start = time.monotonic()
        sent_help = False
        sent_status = False
        while (time.monotonic() - start) < duration_s:
            elapsed = time.monotonic() - start
            if not sent_help and elapsed > 0.2:
                ser.write(b"help\r\n")
                sent_help = True
            if not sent_status and elapsed > 0.8:
                ser.write(b"status\r\n")
                sent_status = True
            data = ser.read(256)
            if data:
                chunks.append(data.decode("utf-8", errors="replace"))
    return "".join(chunks)


class SerialBackend:
    def __init__(self, events: "queue.Queue[Event]") -> None:
        self.events = events
        self.tx: "queue.Queue[str]" = queue.Queue()
        self.thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        self.connected = False

    def connect(self, port: str) -> None:
        raise NotImplementedError

    def send(self, command: str) -> None:
        if self.connected:
            self.tx.put(command)

    def send_now(self, command: str) -> None:
        self.send(command)

    def close(self) -> None:
        self.stop_event.set()
        self.connected = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)


class RealSerialBackend(SerialBackend):
    def __init__(self, events: "queue.Queue[Event]") -> None:
        super().__init__(events)
        self.ser = None

    def connect(self, port: str) -> None:
        if serial is None:
            raise RuntimeError("缺少 pyserial，请运行: python -m pip install pyserial")
        self.ser = serial.Serial(
            port=port,
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
            write_timeout=0.3,
        )
        self.stop_event.clear()
        self.connected = True
        self.thread = threading.Thread(target=self._run, name="flight-serial", daemon=True)
        self.thread.start()

    def send_now(self, command: str) -> None:
        if self.ser is None or not self.connected:
            return
        try:
            self.ser.write((command + "\r\n").encode("ascii", errors="ignore"))
            self.ser.flush()
            self.events.put(("tx", command))
        except Exception as exc:
            self.events.put(("error", f"发送失败: {exc}"))

    def close(self) -> None:
        try:
            if self.connected:
                self.send_now("motor stop")
                time.sleep(0.05)
        finally:
            super().close()
            if self.ser is not None:
                try:
                    self.ser.close()
                finally:
                    self.ser = None

    def _run(self) -> None:
        self.events.put(("info", "串口已连接"))
        while not self.stop_event.is_set():
            try:
                while True:
                    command = self.tx.get_nowait()
                    self.ser.write((command + "\r\n").encode("ascii", errors="ignore"))
                    self.ser.flush()
                    self.events.put(("tx", command))
            except queue.Empty:
                pass
            except Exception as exc:
                self.events.put(("error", f"发送失败: {exc}"))
                break

            try:
                data = self.ser.read(256)
                if data:
                    self.events.put(("rx", data.decode("utf-8", errors="replace")))
            except Exception as exc:
                self.events.put(("error", f"读取失败: {exc}"))
                break
        self.connected = False
        self.events.put(("info", "串口已断开"))


class FakeSerialBackend(SerialBackend):
    def connect(self, port: str) -> None:
        self.stop_event.clear()
        self.connected = True
        self.thread = threading.Thread(target=self._run, name="flight-fake-serial", daemon=True)
        self.thread.start()

    def _emit(self, text: str) -> None:
        self.events.put(("rx", text))

    def send_now(self, command: str) -> None:
        if self.connected:
            self.tx.put(command)

    def _run(self) -> None:
        self._emit("\r\nNAZE32 custom firmware boot\r\nCLI ready: type help\r\n> ")
        last_telem = time.monotonic()
        logging = False
        while not self.stop_event.is_set():
            try:
                cmd = self.tx.get(timeout=0.05)
                self.events.put(("tx", cmd))
                response, logging = self._response(cmd.strip(), logging)
                self._emit(response + "\r\n> ")
            except queue.Empty:
                pass
            if logging and (time.monotonic() - last_telem) > 0.5:
                last_telem = time.monotonic()
                self._emit(
                    "st arm=0 fs=0 rc=1 imu=1 baro=1 air=0 crash=0 thr=0 batt=3990mV\r\n"
                    "att cd r=24 p=-16 y=110 baro=8cm vel=0cm/s p=100820Pa\r\n"
                    "mot 0 0 0 0\r\n> "
                )
        self.connected = False
        self.events.put(("info", "演示串口已断开"))

    @staticmethod
    def _response(cmd: str, logging: bool) -> Tuple[str, bool]:
        if cmd == "":
            return "", logging
        if cmd == "help":
            return (
                "cmd: help status clock tasks heap i2cscan imu baro rc rcmap batt battdiag\r\n"
                "cmd: motormap control yawdir normal|reverse mixcheck [r_milli p_milli y_milli thr]\r\n"
                "cmd: motoridle [0-200], motormax [700-1000], motor unlock|stop|<1-4> <permille>\r\n"
                "cmd: pid [roll|pitch|yaw <P> <I> <D>|default] uses BF-style units\r\n"
                "cmd: trim [roll_cdeg pitch_cdeg], leveltrim, gyrocal acccal imucal arm disarm log on|off reboot",
                logging,
            )
        if cmd == "status":
            return (
                "armed=0 failsafe=0 rc=1 imu=1 baro=1 mode angle=1 baro=1 air=0 crash=0\r\n"
                "althold req=1 ready=1 active=0 baro_ok=1 imu_ok=1 tilt_ok=1 armed=0 alt=8cm vel=0cm/s out=0\r\n"
                "uptime=12345ms throttle=0 motor_test=0\r\n"
                "failsafe_flags=0x0000 last_disarm=0x0000 motor_idle=120 motor_max=1000"
            ), logging
        if cmd == "clock":
            return "clock src=PLL pll=HSE sys=72000000Hz hclk=72000000Hz pclk1=36000000Hz pclk2=72000000Hz fallback=0", logging
        if cmd == "i2cscan":
            return "i2c: 0x68 0x76", logging
        if cmd == "imu":
            return "imu ok=1 acc_mg=12,-28,998 gyro_cdps=3,-2,1 temp=31.25C\r\natt cd roll=24 pitch=-16 yaw=110", logging
        if cmd == "baro":
            return (
                "baro ok=1 temp=29.88C pressure=100820Pa altitude=8cm vel=0cm/s\r\n"
                "baro hold active=0 velctl=0 hold=0cm err=0cm target_vel=0cm/s base=0 corr=0 out=0"
            ), logging
        if cmd == "rcmap":
            return "rcmap order=AETR roll=CH1 pitch=CH2 throttle=CH3 yaw=CH4 arm=CH5 baro=CH6 raw_min=172 raw_mid=992 raw_max=1811", logging
        if cmd == "rc":
            return "rc connected=1 failsafe=0 arm=0 baro=0 age=12ms\r\nstick r=0 p=0 y=0 t=0 raw=992,992,172,992,988,988,172,172,172,172,172,172,172,172,172,172", logging
        if cmd == "batt":
            return "batt raw=948 adc=764mV voltage=8404mV cells=2 percent=100 low=0 critical=0", logging
        if cmd in ("battdiag", "adc"):
            return (
                "battdiag path=BAT-R29(100k)-ADC_BATT/PA4-R28(10k)-GND\r\n"
                "battdiag ratio=11/1 cells=2 vref=3300mV adcmax=4095\r\n"
                "battdiag expected_raw empty=744 low=789 full=946\r\n"
                "battdiag now raw=948 adc=764mV voltage=8404mV"
            ), logging
        if cmd == "motormap":
            return (
                "motormap QuadX nose-forward:\r\n"
                "motormap layout: M4 front-left, M2 front-right, M3 rear-left, M1 rear-right\r\n"
                "motormap output: M1=PA8/CN6 M2=PA11/CN4 M3=PB6/CN3 M4=PB7/CN1\r\n"
                "motormap spin: standard M1/M4=CW, M2/M3=CCW viewed from top\r\n"
                "motormap correction: right-low -> M1/M2 up, nose-low -> M2/M4 up"
            ), logging
        if cmd in ("control", "ctrl"):
            return (
                "control status arm=0 fs=0 flags=0x0000 rc=1 imu=1 imu_age=2ms yawdir=-1\r\n"
                "control sp_cd r=0 p=0 yawrate_cdps=0 thr=0 air=0 baro=1\r\n"
                "control att_cd r=24 p=-16 y=110 healthy=1\r\n"
                "control trim_cd r=0 p=0\r\n"
                "control out_milli r=0 p=0 y=0 alt=0\r\n"
                "control alt active=0 velctl=0 hold=0cm err=0cm vel=0 target=0 base=0 corr=0\r\n"
                "control rate_sp_dps r=0 p=0 y=0 gyro_dps r=0 p=0 y=0\r\n"
                "control pid_milli r=0,0,0 p=0,0,0 y=0,0,0\r\n"
                "control mot_permille M1=0 M2=0 M3=0 M4=0"
            ), logging
        if cmd.startswith("yawdir"):
            parts = cmd.split()
            value = parts[1] if len(parts) > 1 else "reverse"
            if value in ("normal", "1"):
                return "yawdir=normal (1)", logging
            if value in ("reverse", "-1"):
                return "yawdir=reverse (-1)", logging
            return "usage: yawdir normal|reverse", logging
        if cmd.startswith("mixcheck"):
            return FakeSerialBackend._mixcheck_response(cmd), logging
        if cmd.startswith("motoridle"):
            parts = cmd.split()
            return f"motoridle={parts[1] if len(parts) > 1 else 120} permille", logging
        if cmd.startswith("motormax"):
            parts = cmd.split()
            return f"motormax={parts[1] if len(parts) > 1 else 1000} permille", logging
        if cmd.startswith("trim"):
            parts = cmd.split()
            roll = parts[1] if len(parts) > 1 else "0"
            pitch = parts[2] if len(parts) > 2 else "0"
            return f"trim roll_cd={roll} pitch_cd={pitch}", logging
        if cmd == "leveltrim":
            return "trim roll_cd=0 pitch_cd=0", logging
        if cmd == "heap":
            return "heap free=7816 min=7040 rxdrop=0", logging
        if cmd == "tasks":
            return "name          state prio stack num\r\nstabilize     B     5    92    1\r\ncli           R     1    211   7", logging
        if cmd == "pid":
            return (
                "roll bf P=60 I=70 D=17 gain kp=1921u ki=17106u kd=8u\r\n"
                "pitch bf P=80 I=90 D=18 gain kp=2562u ki=21994u kd=9u\r\n"
                "yaw bf P=200 I=45 D=0 gain kp=6405u ki=10997u kd=0u"
            ), logging
        if cmd.startswith("pid "):
            return "pid ok", logging
        if cmd == "motor unlock":
            return "motor test unlocked bench=0", logging
        if cmd == "motor unlock bench":
            return "motor test unlocked bench=1", logging
        if cmd.startswith("motor ") and cmd != "motor stop":
            parts = cmd.split()
            return f"motor {parts[1]}={parts[2] if len(parts) > 2 else 0}", logging
        if cmd in ("motor stop", "motors 0"):
            return "motors stopped", logging
        if cmd == "log on":
            return "log=1", True
        if cmd == "log off":
            return "log=0", False
        if cmd == "arm":
            return "arm requested", logging
        if cmd == "disarm":
            return "disarmed", logging
        return "unknown command", logging

    @staticmethod
    def _mixcheck_response(cmd: str) -> str:
        parts = cmd.split()

        def arg(index: int, default: int) -> int:
            try:
                return int(parts[index])
            except (IndexError, ValueError):
                return default

        r = arg(1, 0) / 1000.0
        p = arg(2, 0) / 1000.0
        y = arg(3, 0) / 1000.0
        throttle = max(0, min(1000, arg(4, 200)))
        t = throttle / 1000.0
        idle = 0.12
        motors = [
            t - r + p - y,
            t - r - p + y,
            t + r + p + y,
            t + r - p - y,
        ]
        min_v = min(motors)
        max_v = max(motors)
        if min_v < idle:
            motors = [m + (idle - min_v) for m in motors]
        max_v = max(motors)
        if max_v > 1.0:
            motors = [m - (max_v - 1.0) for m in motors]
        motors = [max(idle, min(1.0, m)) for m in motors]
        values = [int(m * 1000.0) for m in motors]
        return (
            f"mixcheck in_milli r={int(r * 1000)} p={int(p * 1000)} y={int(y * 1000)} thr={throttle}\r\n"
            f"mixcheck M1={values[0]} M2={values[1]} M3={values[2]} M4={values[3]}\r\n"
            "mixcheck +roll=>M3/M4 up, +pitch=>M1/M3 up, +yaw=>M2/M3 up"
        )


def create_backend(port: str, events: "queue.Queue[Event]") -> SerialBackend:
    if port == "FAKE":
        return FakeSerialBackend(events)
    return RealSerialBackend(events)
