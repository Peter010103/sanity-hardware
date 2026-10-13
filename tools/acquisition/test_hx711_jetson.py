"""Logic/protocol tests with a fake GPIO device; no hardware dependency."""
import argparse
import contextlib
import csv
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import hx711_jetson as hx


class FakeClock:
    def __init__(self):
        self.ns = 0
    def perf_counter_ns(self):
        self.ns += 200
        return self.ns
    def monotonic_ns(self):
        return self.ns
    def monotonic(self):
        return self.ns / 1e9
    def time_ns(self):
        return self.ns + 100000000000
    def sleep(self, seconds):
        self.ns += int(seconds * 1e9)


class FakeGPIO:
    BOARD, OUT, IN, HIGH, LOW = "BOARD", "OUT", "IN", 1, 0
    def __init__(self, clock, word=0):
        self.clock, self.word = clock, word
        self.level = 0
        self.pulses = 0
        self.read_started = False
        self.cleaned = False
        self.stuck = None
        self.slow = False
    def setmode(self, mode):
        assert mode == "BOARD"
    def setup(self, pin, direction, **kwargs):
        pass
    def output(self, pin, level):
        if level and not self.level and self.read_started:
            self.pulses += 1
            if self.slow:
                self.clock.ns += 61000
        self.level = level
    def input(self, pin):
        if self.stuck is not None:
            return self.stuck
        if not self.read_started:
            self.read_started = True
            return 0
        if self.pulses >= 25:
            return 1
        if self.pulses:
            return (self.word >> (24 - self.pulses)) & 1
        return 0
    def cleanup(self, pins):
        self.cleaned = True


def samples(values):
    return [{"raw_counts": x} for x in values]


class Tests(unittest.TestCase):
    def sensor(self, word=0):
        clock = FakeClock()
        gpio = FakeGPIO(clock, word)
        return hx.HX711(gpio, clock=clock), gpio

    def test_signed_extremes(self):
        for word, expected in [(0, 0), (0x7fffff, 8388607),
                               (0x800000, -8388608), (0xffffff, -1)]:
            self.assertEqual(hx.signed24(word), expected)

    def test_wire_protocol(self):
        for word in (0, 0x123456, 0xabcdef, 0xffffff):
            with self.subTest(word=word):
                sensor, gpio = self.sensor(word)
                result = sensor.read()
                self.assertEqual(result["raw_counts"], hx.signed24(word))
                self.assertEqual(gpio.pulses, 25)
                self.assertEqual(gpio.level, 0)
                sensor.close()
                self.assertTrue(gpio.cleaned)

    def test_timeout(self):
        sensor, gpio = self.sensor()
        gpio.stuck = 1
        with self.assertRaises(TimeoutError):
            sensor.read()
        self.assertEqual(gpio.pulses, 0)

    def test_stuck_low_rejected(self):
        sensor, gpio = self.sensor()
        gpio.stuck = 0
        with self.assertRaisesRegex(RuntimeError, "did not return HIGH"):
            sensor.read()

    def test_long_clock_rejected_and_low(self):
        sensor, gpio = self.sensor()
        gpio.slow = True
        with self.assertRaises(hx.TimingError):
            sensor.read()
        self.assertEqual(gpio.level, 0)

    def test_calibration_both_electrical_polarities(self):
        for delta in (-1000, 1000):
            cal = hx.make_calibration(samples([100, 101, 99]),
                                      samples([100+delta]*3), 2, {}, {})
            self.assertAlmostEqual(hx.force_from_counts(100+delta, cal), 2)
            self.assertAlmostEqual(hx.force_from_counts(100-delta, cal), -2)

    def test_opposite_reference_direction(self):
        cal = hx.make_calibration(samples([0]*3), samples([1000]*3), -1, {}, {})
        self.assertEqual(hx.force_from_counts(-1000, cal), 1)

    def test_invalid_reference(self):
        for force in (0, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                hx.make_calibration(samples([0]*3), samples([1000]*3), force, {}, {})

    def test_rails_and_insufficient_signal(self):
        with self.assertRaises(ValueError):
            hx.stats(samples([0, -8388608]))
        with self.assertRaises(ValueError):
            hx.make_calibration(samples([100]*3), samples([101]*3), 1, {}, {})

    def test_discard_stale_conversion(self):
        sensor = unittest.mock.Mock()
        sensor.read.side_effect = samples([999, 10, 20])
        self.assertEqual(hx.collect(sensor, 2), samples([10, 20]))

    def test_no_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory) / "cal.json"
            hx.write_json_new(p, {"x": 1})
            with self.assertRaises(FileExistsError):
                hx.write_json_new(p, {"x": 2})
            self.assertEqual(json.loads(p.read_text()), {"x": 1})

    def test_roundtrip_calibration(self):
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory) / "cal.json"
            cal = hx.make_calibration(samples([0]*3), samples([1000]*3), 1, {}, {})
            hx.write_json_new(p, cal)
            self.assertEqual(hx.load_calibration(p), cal)

    def test_fault_preserves_partial_log(self):
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory) / "data.csv"
            s = dict(raw_counts=123, ready_observed_monotonic_ns=1,
                     read_complete_monotonic_ns=2, unix_time_ns=3,
                     max_clock_high_bound_us=4)
            sensor = unittest.mock.Mock()
            sensor.read.side_effect = [s, s, hx.TimingError("timing")]
            args = argparse.Namespace(output=str(p), duration=None, test_id="test",
                                      scope="whole_aircraft", configuration_id="cfg")
            with contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaises(hx.TimingError):
                    hx.log(sensor, args, None, {})
            with p.open(newline="", encoding="utf-8") as file:
                rows = list(csv.DictReader(file))
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["raw_counts"], "123")
            self.assertEqual(rows[0]["loaded_voltage_V"], "")
            self.assertEqual(rows[0]["force_N"], "")
            summary = json.loads(Path(str(p) + ".summary.json").read_text())
            self.assertEqual(summary["status"], "failed")
            self.assertEqual(summary["samples_written"], 1)

    def test_timing_recovery_returns_only_valid_sample(self):
        sensor = unittest.mock.Mock()
        sensor.read.side_effect = [hx.TimingError("92.4 us"), {"raw_counts": 123}]
        recovery = hx.RecoveringHX711(sensor)
        with contextlib.redirect_stderr(io.StringIO()) as warnings:
            sample = recovery.read()
        self.assertEqual(sample["raw_counts"], 123)
        self.assertEqual(sample["timing_rejections_before_sample"], 1)
        self.assertEqual(sample["timing_rejections_total"], 1)
        sensor.reset.assert_called_once()
        self.assertIn("92.4 us", warnings.getvalue())

    def test_sustained_timing_failures_stop(self):
        sensor = unittest.mock.Mock()
        sensor.read.side_effect = hx.TimingError("timing")
        recovery = hx.RecoveringHX711(sensor, retries=3)
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaisesRegex(hx.TimingError, "stopping"):
                recovery.read()
        self.assertEqual(recovery.rejected_total, 4)
        self.assertEqual(recovery.resets_total, 3)

    def test_other_errors_are_not_retried(self):
        for error in (TimeoutError("DOUT"), OSError("GPIO")):
            sensor = unittest.mock.Mock()
            sensor.read.side_effect = error
            recovery = hx.RecoveringHX711(sensor)
            with self.assertRaises(type(error)):
                recovery.read()
            sensor.reset.assert_not_called()

    def test_csv_records_recovery_gap_and_rejection_count(self):
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory) / "data.csv"
            sample = dict(raw_counts=123, ready_observed_monotonic_ns=1,
                          read_complete_monotonic_ns=2, unix_time_ns=3,
                          max_clock_high_bound_us=4)
            sensor = unittest.mock.Mock()
            sensor.read.side_effect = [dict(sample), hx.TimingError("92.4 us"),
                                      dict(sample), KeyboardInterrupt()]
            recovery = hx.RecoveringHX711(sensor)
            args = argparse.Namespace(output=str(p), duration=None, test_id="test",
                                      scope="whole_aircraft", configuration_id="cfg")
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                hx.log(recovery, args, None, {})
            with p.open(newline="", encoding="utf-8") as file:
                rows = list(csv.DictReader(file))
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["raw_counts"], "123")
            self.assertEqual(rows[0]["timing_rejections_before_sample"], "1")
            summary = json.loads(Path(str(p) + ".summary.json").read_text())
            self.assertEqual(summary["timing_rejections_total"], 1)
            self.assertEqual(summary["adc_recovery_resets"], 1)


if __name__ == "__main__":
    unittest.main()
