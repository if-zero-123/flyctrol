# -*- coding: utf-8 -*-
import unittest

from cli_parser import FlightCliParser
from flight_debug_gui import FlightDebugGui


SAMPLE = """
NAZE32 custom firmware boot
armed=0 failsafe=0 rc=1 imu=1 baro=1 mode angle=1 baro=1 air=0 crash=0
althold req=1 ready=1 active=0 baro_ok=1 imu_ok=1 tilt_ok=1 armed=0 alt=8cm vel=0cm/s out=0 thr_ok=0 thr_min=300
uptime=12345ms throttle=0 motor_test=0
i2c: 0x68 0x76
imu ok=1 acc_mg=12,-28,998 gyro_cdps=3,-2,1 temp=31.25C
att cd roll=24 pitch=-16 yaw=110
baro ok=1 temp=29.88C pressure=100820Pa altitude=8cm vel=0cm/s
baro hold active=0 velctl=0 hold=0cm err=0cm target_vel=0cm/s base=0 corr=0 out=0
rc connected=1 failsafe=0 arm=0 baro=0 age=12ms
rcmap order=AETR roll=CH1 pitch=CH2 throttle=CH3 yaw=CH4 arm=CH5 baro=CH6 raw_min=172 raw_mid=992 raw_max=1811
stick r=0 p=0 y=0 t=0 raw=992,992,172,992,988,988,172,172,172,172,172,172,172,172,172,172
batt raw=948 adc=764mV voltage=8404mV cells=2 percent=100 low=0 critical=0
heap free=7816 min=7040 rxdrop=0
clock src=PLL pll=HSE sys=72000000Hz hclk=72000000Hz pclk1=36000000Hz pclk2=72000000Hz fallback=0
roll kp=3500 ki=0 kd=45 milli
pitch kp=3500 ki=0 kd=45 milli
yaw kp=1800 ki=0 kd=0 milli
mot 0 0 0 0
"""


class ParserTest(unittest.TestCase):
    def test_parse_sample_cli_output(self) -> None:
        parser = FlightCliParser()
        parser.parse_lines(SAMPLE.splitlines())
        state = parser.latest()

        self.assertTrue(state["boot_seen"])
        self.assertEqual(state["i2c"]["addresses"], ["0x68", "0x76"])
        self.assertTrue(state["status"]["rc_ok"])
        self.assertFalse(state["status"]["failsafe"])
        self.assertTrue(state["imu"]["ok"])
        self.assertAlmostEqual(state["attitude"]["roll"], 0.24)
        self.assertEqual(state["baro"]["pressure_pa"], 100820)
        self.assertEqual(state["baro"]["velocity_cms"], 0)
        self.assertTrue(state["althold"]["requested"])
        self.assertTrue(state["althold"]["ready"])
        self.assertFalse(state["althold"]["active"])
        self.assertFalse(state["althold"]["throttle_ok"])
        self.assertEqual(state["althold"]["throttle_min"], 300)
        self.assertEqual(state["battery"]["voltage_mv"], 8404)
        self.assertEqual(state["battery"]["cells"], 2)
        self.assertEqual(state["clock"]["pll"], "HSE")
        self.assertEqual(state["clock"]["sys_hz"], 72000000)
        self.assertFalse(state["clock"]["fallback"])
        self.assertEqual(state["heap"]["free"], 7816)
        self.assertEqual(state["heap"]["rxdrop"], 0)
        self.assertEqual(state["rc"]["throttle"], 0)
        self.assertEqual(state["rcmap"]["order"], "AETR")
        self.assertEqual(len(state["rc"]["raw"]), 16)
        self.assertEqual(state["pid"]["roll"]["kp"], 3500)
        self.assertEqual(state["motors"]["m"], [0, 0, 0, 0])
        self.assertGreater(len(parser.rows), 0)


class _FakeVar:
    def __init__(self) -> None:
        self.value = None

    def set(self, value: str) -> None:
        self.value = value


class _FakeParser:
    def latest(self) -> dict:
        parser = FlightCliParser()
        parser.parse_lines(SAMPLE.splitlines())
        return parser.latest()


class GuiRefreshTest(unittest.TestCase):
    def test_refresh_monitor_uses_althold_state_without_tk_window(self) -> None:
        gui = object.__new__(FlightDebugGui)
        gui.parser = _FakeParser()
        gui.monitor_vars = {
            key: _FakeVar()
            for key in (
                "armed",
                "failsafe",
                "rc",
                "imu",
                "baro",
                "althold",
                "battery",
                "att",
                "baro_detail",
                "sticks",
                "motors",
            )
        }

        FlightDebugGui._refresh_monitor(gui)

        self.assertIn("req=1", gui.monitor_vars["althold"].value)
        self.assertIn("ready=1", gui.monitor_vars["althold"].value)
        self.assertIn("active=0", gui.monitor_vars["althold"].value)


if __name__ == "__main__":
    unittest.main()
