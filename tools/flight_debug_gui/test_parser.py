# -*- coding: utf-8 -*-
import unittest

from cli_parser import FlightCliParser


SAMPLE = """
NAZE32 custom firmware boot
armed=0 failsafe=0 rc=1 imu=1 baro=1 mode angle=1 baro=0
uptime=12345ms throttle=0 motor_test=0
i2c: 0x68 0x76
imu ok=1 acc_mg=12,-28,998 gyro_cdps=3,-2,1 temp=31.25C
att cd roll=24 pitch=-16 yaw=110
baro ok=1 temp=29.88C pressure=100820Pa altitude=8cm
rc connected=1 failsafe=0 arm=0 baro=0 age=12ms
stick r=0 p=0 y=0 t=0 raw=992,992,172,992,988,988
batt raw=1125 voltage=3990mV percent=76 low=0 critical=0
heap free=7816 min=7040
hb tick=12345ms heap=7800 arm=0 fs=0 rc=1 imu=1 baro=1 thr=0 batt=3990mV
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
        self.assertEqual(state["battery"]["voltage_mv"], 3990)
        self.assertEqual(state["heartbeat"]["tick_ms"], 12345)
        self.assertEqual(state["heap"]["free"], 7800)
        self.assertEqual(state["rc"]["throttle"], 0)
        self.assertEqual(state["pid"]["roll"]["kp"], 3500)
        self.assertEqual(state["motors"]["m"], [0, 0, 0, 0])
        self.assertGreater(len(parser.rows), 0)


if __name__ == "__main__":
    unittest.main()
