# -*- coding: utf-8 -*-
"""Parser for the NAZE32 custom firmware text CLI."""

from __future__ import annotations

import csv
import re
from copy import deepcopy
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional


_INT = r"(-?\d+)"


class FlightCliParser:
    def __init__(self) -> None:
        self.state: Dict[str, object] = {
            "boot_seen": False,
            "prompt_seen": False,
            "status": {},
            "imu": {},
            "attitude": {},
            "baro": {},
            "rc": {},
            "rcmap": {
                "order": "AETR",
                "roll": "CH1",
                "pitch": "CH2",
                "throttle": "CH3",
                "yaw": "CH4",
                "arm": "CH5",
                "baro": "CH6",
            },
            "battery": {},
            "i2c": {},
            "heap": {},
            "clock": {},
            "pid": {},
            "motors": {},
            "last_line": "",
            "last_update": "",
        }
        self.rows: List[Dict[str, object]] = []

    def parse_lines(self, lines: Iterable[str]) -> None:
        for line in lines:
            self.parse_line(line)

    def parse_line(self, line: str) -> None:
        text = line.strip()
        if not text:
            return

        text = text.lstrip("> ").strip()
        if not text:
            self.state["prompt_seen"] = True
            return

        self.state["last_line"] = text
        self.state["last_update"] = datetime.now().strftime("%H:%M:%S")

        if "NAZE32 custom firmware boot" in text:
            self.state["boot_seen"] = True
            return

        if text.startswith("armed="):
            self._parse_status(text)
            self._add_snapshot()
            return
        if text.startswith("uptime="):
            self._parse_uptime(text)
            self._add_snapshot()
            return
        if text.startswith("i2c:"):
            self._parse_i2c(text)
            return
        if text.startswith("imu ok="):
            self._parse_imu(text)
            self._add_snapshot()
            return
        if text.startswith("att cd "):
            self._parse_attitude(text)
            self._add_snapshot()
            return
        if text.startswith("baro ok="):
            self._parse_baro(text)
            self._add_snapshot()
            return
        if text.startswith("rc connected="):
            self._parse_rc_status(text)
            self._add_snapshot()
            return
        if text.startswith("rcmap "):
            self._parse_rcmap(text)
            return
        if text.startswith("stick "):
            self._parse_rc_sticks(text)
            self._add_snapshot()
            return
        if text.startswith("batt raw="):
            self._parse_battery(text)
            self._add_snapshot()
            return
        if text.startswith("heap free="):
            self._parse_heap(text)
            return
        if text.startswith("clock src="):
            self._parse_clock(text)
            return
        if text.startswith(("roll kp=", "pitch kp=", "yaw kp=")):
            self._parse_pid(text)
            return
        if text.startswith("st arm="):
            self._parse_telemetry_status(text)
            self._add_snapshot()
            return
        if text.startswith("att cd r="):
            self._parse_telemetry_attitude(text)
            self._add_snapshot()
            return
        if text.startswith("mot "):
            self._parse_motors(text)
            return

    def latest(self) -> Dict[str, object]:
        return deepcopy(self.state)

    def save_csv(self, path: Path) -> None:
        fields = [
            "time",
            "armed",
            "failsafe",
            "rc",
            "imu",
            "baro",
            "battery_mv",
            "roll",
            "pitch",
            "yaw",
            "throttle",
        ]
        with path.open("w", newline="", encoding="utf-8-sig") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self.rows)

    def _parse_status(self, text: str) -> None:
        m = re.search(
            rf"armed=(\d+) failsafe=(\d+) rc=(\d+) imu=(\d+) baro=(\d+) "
            rf"mode angle=(\d+) baro=(\d+)",
            text,
        )
        if not m:
            return
        status = self._dict("status")
        status.update(
            {
                "armed": bool(int(m.group(1))),
                "failsafe": bool(int(m.group(2))),
                "rc_ok": bool(int(m.group(3))),
                "imu_ok": bool(int(m.group(4))),
                "baro_ok": bool(int(m.group(5))),
                "angle_mode": bool(int(m.group(6))),
                "baro_mode": bool(int(m.group(7))),
            }
        )
        self.state["status"] = status

    def _parse_uptime(self, text: str) -> None:
        m = re.search(r"uptime=(\d+)ms throttle=(\d+) motor_test=(\d+)", text)
        if not m:
            return
        status = self._dict("status")
        status.update(
            {
                "uptime_ms": int(m.group(1)),
                "throttle": int(m.group(2)),
                "motor_test": bool(int(m.group(3))),
            }
        )
        self.state["status"] = status

    def _parse_i2c(self, text: str) -> None:
        addresses = re.findall(r"0x[0-9A-Fa-f]{2}", text)
        lowered = [a.lower() for a in addresses]
        self.state["i2c"] = {
            "addresses": addresses,
            "mpu6050": "0x68" in lowered,
            "bmp280": "0x76" in lowered,
        }

    def _parse_imu(self, text: str) -> None:
        m = re.search(
            rf"imu ok=(\d+) acc_mg={_INT},{_INT},{_INT} "
            rf"gyro_cdps={_INT},{_INT},{_INT} temp={_INT}\.(\d+)C",
            text,
        )
        if not m:
            return
        self.state["imu"] = {
            "ok": bool(int(m.group(1))),
            "acc_mg": [int(m.group(i)) for i in range(2, 5)],
            "gyro_cdps": [int(m.group(i)) for i in range(5, 8)],
            "temp_c": float(f"{m.group(8)}.{m.group(9)}"),
        }

    def _parse_attitude(self, text: str) -> None:
        m = re.search(rf"att cd roll={_INT} pitch={_INT} yaw={_INT}", text)
        if not m:
            return
        self.state["attitude"] = {
            "roll": int(m.group(1)) / 100.0,
            "pitch": int(m.group(2)) / 100.0,
            "yaw": int(m.group(3)) / 100.0,
        }

    def _parse_baro(self, text: str) -> None:
        m = re.search(rf"baro ok=(\d+) temp={_INT}\.(\d+)C pressure={_INT}Pa altitude={_INT}cm", text)
        if not m:
            return
        self.state["baro"] = {
            "ok": bool(int(m.group(1))),
            "temp_c": float(f"{m.group(2)}.{m.group(3)}"),
            "pressure_pa": int(m.group(4)),
            "altitude_cm": int(m.group(5)),
        }

    def _parse_rc_status(self, text: str) -> None:
        m = re.search(r"rc connected=(\d+) failsafe=(\d+) arm=(\d+) baro=(\d+) age=(\d+)ms", text)
        if not m:
            return
        rc = self._dict("rc")
        rc.update(
            {
                "connected": bool(int(m.group(1))),
                "failsafe": bool(int(m.group(2))),
                "arm": bool(int(m.group(3))),
                "baro": bool(int(m.group(4))),
                "age_ms": int(m.group(5)),
            }
        )
        self.state["rc"] = rc

    def _parse_rc_sticks(self, text: str) -> None:
        m = re.search(rf"stick r={_INT} p={_INT} y={_INT} t=(\d+) raw=([0-9,\-]+)", text)
        if not m:
            return
        rc = self._dict("rc")
        rc.update(
            {
                "roll": int(m.group(1)),
                "pitch": int(m.group(2)),
                "yaw": int(m.group(3)),
                "throttle": int(m.group(4)),
                "raw": [int(x) for x in m.group(5).split(",") if x],
            }
        )
        self.state["rc"] = rc

    def _parse_rcmap(self, text: str) -> None:
        pairs = dict(re.findall(r"(\w+)=([A-Za-z0-9_]+)", text))
        if pairs:
            self.state["rcmap"] = pairs

    def _parse_battery(self, text: str) -> None:
        m = re.search(r"batt raw=(\d+) voltage=(\d+)mV percent=(\d+) low=(\d+) critical=(\d+)", text)
        if not m:
            return
        self.state["battery"] = {
            "raw": int(m.group(1)),
            "voltage_mv": int(m.group(2)),
            "percent": int(m.group(3)),
            "low": bool(int(m.group(4))),
            "critical": bool(int(m.group(5))),
        }

    def _parse_heap(self, text: str) -> None:
        m = re.search(r"heap free=(\d+) min=(\d+)(?: rxdrop=(\d+))?", text)
        if m:
            self.state["heap"] = {
                "free": int(m.group(1)),
                "min": int(m.group(2)),
                "rxdrop": int(m.group(3) or 0),
            }

    def _parse_clock(self, text: str) -> None:
        m = re.search(
            r"clock src=([A-Z0-9_]+) pll=([A-Z0-9_]+) sys=(\d+)Hz "
            r"hclk=(\d+)Hz pclk1=(\d+)Hz pclk2=(\d+)Hz fallback=(\d+)",
            text,
        )
        if not m:
            return
        self.state["clock"] = {
            "src": m.group(1),
            "pll": m.group(2),
            "sys_hz": int(m.group(3)),
            "hclk_hz": int(m.group(4)),
            "pclk1_hz": int(m.group(5)),
            "pclk2_hz": int(m.group(6)),
            "fallback": bool(int(m.group(7))),
        }

    def _parse_pid(self, text: str) -> None:
        m = re.search(r"(roll|pitch|yaw) kp=(-?\d+) ki=(-?\d+) kd=(-?\d+) milli", text)
        if not m:
            return
        pid = self._dict("pid")
        pid[m.group(1)] = {
            "kp": int(m.group(2)),
            "ki": int(m.group(3)),
            "kd": int(m.group(4)),
        }
        self.state["pid"] = pid

    def _parse_telemetry_status(self, text: str) -> None:
        m = re.search(r"st arm=(\d+) fs=(\d+) rc=(\d+) imu=(\d+) baro=(\d+) thr=(\d+) batt=(\d+)mV", text)
        if not m:
            return
        status = self._dict("status")
        status.update(
            {
                "armed": bool(int(m.group(1))),
                "failsafe": bool(int(m.group(2))),
                "rc_ok": bool(int(m.group(3))),
                "imu_ok": bool(int(m.group(4))),
                "baro_ok": bool(int(m.group(5))),
                "throttle": int(m.group(6)),
            }
        )
        self.state["status"] = status
        batt = self._dict("battery")
        batt["voltage_mv"] = int(m.group(7))
        self.state["battery"] = batt

    def _parse_telemetry_attitude(self, text: str) -> None:
        m = re.search(rf"att cd r={_INT} p={_INT} y={_INT} baro={_INT}cm p={_INT}Pa", text)
        if not m:
            return
        self.state["attitude"] = {
            "roll": int(m.group(1)) / 100.0,
            "pitch": int(m.group(2)) / 100.0,
            "yaw": int(m.group(3)) / 100.0,
        }
        baro = self._dict("baro")
        baro.update({"altitude_cm": int(m.group(4)), "pressure_pa": int(m.group(5))})
        self.state["baro"] = baro

    def _parse_motors(self, text: str) -> None:
        m = re.search(r"mot (-?\d+) (-?\d+) (-?\d+) (-?\d+)", text)
        if m:
            self.state["motors"] = {"m": [int(m.group(i)) for i in range(1, 5)]}

    def _add_snapshot(self) -> None:
        status = self._dict("status")
        battery = self._dict("battery")
        att = self._dict("attitude")
        row = {
            "time": datetime.now().isoformat(timespec="seconds"),
            "armed": self._fmt_bool(status.get("armed")),
            "failsafe": self._fmt_bool(status.get("failsafe")),
            "rc": self._fmt_bool(status.get("rc_ok")),
            "imu": self._fmt_bool(status.get("imu_ok") if "imu_ok" in status else self._dict("imu").get("ok")),
            "baro": self._fmt_bool(status.get("baro_ok") if "baro_ok" in status else self._dict("baro").get("ok")),
            "battery_mv": battery.get("voltage_mv", ""),
            "roll": att.get("roll", ""),
            "pitch": att.get("pitch", ""),
            "yaw": att.get("yaw", ""),
            "throttle": status.get("throttle", self._dict("rc").get("throttle", "")),
        }
        if not self.rows or self.rows[-1] != row:
            self.rows.append(row)
            if len(self.rows) > 5000:
                self.rows = self.rows[-5000:]

    def _dict(self, key: str) -> Dict[str, object]:
        value = self.state.get(key)
        return dict(value) if isinstance(value, dict) else {}

    @staticmethod
    def _fmt_bool(value: Optional[object]) -> str:
        if value is True:
            return "1"
        if value is False:
            return "0"
        return ""
