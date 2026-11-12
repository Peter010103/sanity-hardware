"""Protocol tests; no flight controller or powered motors required."""
import os
import pty
import struct
import threading
import time
import unittest

import betaflight_msp as bf


def reply(command, payload=b"", direction=b">"):
    frame = b"$M" + direction + bytes((len(payload), command)) + payload
    checksum = 0
    for byte in frame[3:]:
        checksum ^= byte
    return frame + bytes((checksum,))


class FakeTransport:
    def __init__(self, parts):
        self.parts, self.writes = iter(parts), []
    def write(self, data, deadline):
        self.writes.append(data)
    def read(self, deadline):
        try:
            return next(self.parts)
        except StopIteration:
            raise TimeoutError("no reply")


class Tests(unittest.TestCase):
    def test_fragmented_frame_and_noise(self):
        frame = reply(1, b"\0\1\x2e")
        transport = FakeTransport([b"noise" + frame[:1], frame[1:4], frame[4:]])
        self.assertEqual(bf.MSP(transport).query(1), b"\0\1\x2e")
        self.assertEqual(transport.writes, [b"$M<\0\1\1"])

    def test_bad_checksum_and_wrong_command_are_ignored(self):
        bad = bytearray(reply(1, b"bad"))
        bad[-1] ^= 1
        transport = FakeTransport([bytes(bad) + reply(2, b"BTFL") + reply(1, b"good")])
        self.assertEqual(bf.MSP(transport).query(1), b"good")

    def test_motor_and_configuration_writes_are_blocked(self):
        for command in (200, 214, 250, 208, 99):
            transport = FakeTransport([])
            with self.assertRaises(ValueError):
                bf.MSP(transport).query(command)
            self.assertEqual(transport.writes, [])

    def test_negative_reply(self):
        with self.assertRaises(bf.MSPError):
            bf.MSP(FakeTransport([reply(130, direction=b"!")])).query(130)

    def test_battery_signed_current_and_precise_voltage(self):
        payload = struct.pack("<BHBHhBH", 2, 530, 74, 120, -25, 0, 743)
        battery = bf.battery(payload)
        self.assertEqual(battery["voltage_V"], 7.43)
        self.assertEqual(battery["current_A"], -0.25)
        self.assertEqual(battery["detected_cells"], 2)
        self.assertEqual(bf.battery(payload[:9])["voltage_V"], 7.4)
        with self.assertRaises(bf.MSPError):
            bf.battery(payload[:5])

    def test_arm_flag_uses_box_mapping(self):
        payload = struct.pack("<HHHIB", 125, 0, 33, 4, 0)
        self.assertTrue(bf.status(payload, [1, 2, 0])["armed"])
        self.assertFalse(bf.status(payload, [0, 1, 2])["armed"])
        self.assertIsNone(bf.status(payload, None)["armed"])

    def test_linux_tty_transport_with_emulated_fc(self):
        master, slave = pty.openpty()
        transport = None
        try:
            transport = bf.USBSerial(os.ttyname(slave))
            received = []
            def responder():
                packet = b""
                while len(packet) < 6:
                    packet += os.read(master, 6 - len(packet))
                received.append(packet)
                os.write(master, reply(2, b"BTFL"))
            worker = threading.Thread(target=responder, daemon=True)
            worker.start()
            self.assertEqual(bf.MSP(transport).query(2), b"BTFL")
            worker.join(timeout=1)
            self.assertEqual(received, [bf.request_frame(2)])
        finally:
            if transport is not None:
                transport.close()
            os.close(master)
            os.close(slave)


if __name__ == "__main__":
    unittest.main()
