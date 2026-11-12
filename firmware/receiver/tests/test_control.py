"""Cross-language wire/firmware tests; PYTHONPATH must include sanity_zenoh."""

import ctypes as c
import functools
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest

from sanity_zenoh.protocol import CTBR, DIRECT, ReceiverStatus, command_packet, neutral


class ControlTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        root = Path(__file__).resolve().parents[1]
        output = Path(cls.temp.name) / "control.so"
        subprocess.run(
            [
                "cc",
                "-std=c11",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-pedantic",
                "-fsanitize=undefined",
                "-fno-sanitize-recover=all",
                "-shared",
                "-fPIC",
                "-I",
                str(root / "main"),
                str(root / "main/control.c"),
                str(root / "main/bench.c"),
                str(root / "tests/wrapper.c"),
                "-o",
                str(output),
            ],
            check=True,
        )
        cls.lib = c.CDLL(str(output))
        cls.lib.test_reset.argtypes = [c.c_uint, c.c_ulonglong]
        cls.lib.test_receive.argtypes = [c.c_char_p, c.c_uint, c.c_uint]
        cls.lib.test_receive_msp.argtypes = [c.c_char_p, c.c_uint, c.c_uint, c.c_int]
        cls.lib.test_bench_feed.argtypes = [c.c_char_p, c.c_uint, c.c_uint]

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def setUp(self):
        self.lib.test_reset(CTBR, 123)

    def send(
        self,
        seq=0,
        enabled=False,
        values=None,
        now=1000,
        lease=1000,
        ttl=100,
        epoch=123,
        session=9,
        mode=CTBR,
    ):
        packet = command_packet(
            1, mode, enabled, epoch, session, seq, lease, ttl, values or neutral(mode)
        )
        return self.lib.test_receive(packet, len(packet), now)

    def output(self, now):
        values = (c.c_ushort * 4)()
        expired = self.lib.test_output(now, values)
        return bool(expired), tuple(values)

    def test_unified_mode_claim_is_disarmed_disabled_fresh_and_fences_old_epoch(self):
        def claim(mode, enabled=False, disarmed=True, epoch=123, lease=1000, seq=0):
            packet = command_packet(1, mode, enabled, epoch, 9, seq, lease, 100, neutral(mode))
            return self.lib.test_receive_msp(packet, len(packet), 1000, disarmed)
        self.assertFalse(claim(DIRECT, enabled=True))
        self.assertFalse(claim(DIRECT, disarmed=False))
        self.assertFalse(claim(DIRECT, lease=800))
        self.assertTrue(self.send())
        self.assertTrue(self.send(seq=1, enabled=True))
        self.assertFalse(claim(DIRECT))  # Cannot switch while enabled, even if FC says disarmed.
        self.assertTrue(self.send(seq=2, enabled=False))
        self.assertFalse(claim(DIRECT))  # First disabled claim announces the next epoch.
        raw = (c.c_ubyte * 48)(); self.lib.test_status(1000, raw)
        status = ReceiverStatus.decode(bytes(raw))
        self.assertEqual((status.mode, status.epoch), (DIRECT, 124))
        self.assertFalse(claim(CTBR, enabled=True, seq=3))  # Buffered old-mode packet.
        self.assertFalse(claim(DIRECT, epoch=123))
        self.assertTrue(claim(DIRECT, epoch=124))
        self.assertTrue(claim(DIRECT, epoch=124, enabled=True, seq=1))

    def test_msp_rc_frame_is_aetr_and_disabled_arm_low(self):
        values = (c.c_ushort * 4)(1600, 1400, 1550, 1200)
        raw = (c.c_ubyte * 22)()
        self.assertEqual(self.lib.sz_rc_msp(values, True, raw), 22)
        packet = bytes(raw)
        self.assertEqual(packet[:5], b'$M<\x10\xc8')
        self.assertEqual(struct.unpack('<8H', packet[5:-1]), (1600,1400,1200,1550,2000,2000,1000,1000))
        self.assertEqual(packet[-1], functools.reduce(int.__xor__, packet[3:-1], 0))
        self.lib.sz_rc_msp(values, False, raw)
        self.assertEqual(struct.unpack('<8H', bytes(raw)[5:-1]), (1500,1500,1000,1500,1000,2000,1000,1000))

    def test_flight_lease_expiry_latches_even_when_next_packet_has_fresh_lease(self):
        self.assertTrue(self.send(seq=0,now=106986,lease=106986,ttl=92))
        self.assertTrue(self.send(seq=1,enabled=True,now=106987,lease=106986,ttl=92))
        self.assertTrue(self.send(seq=1565,enabled=True,values=(1500,1500,1500,1334),now=107070,lease=106986,ttl=92))
        self.assertEqual(self.output(107073),(False,(1500,1500,1500,1334)))
        # The new command itself is fresh, but the previous authority expired.
        self.assertFalse(self.send(seq=1566,enabled=True,values=(1500,1500,1500,1335),now=107079,lease=107052,ttl=91))
        self.assertEqual(self.output(107079),(True,(1500,1500,1500,1000)))
        self.assertTrue(self.send(seq=1567,now=107080,lease=107079,ttl=100))
        self.assertFalse(self.send(seq=1568,enabled=True,values=(1500,1500,1500,1335),now=107081,lease=107080))

    def test_disable_zero_enable_and_thrust(self):
        self.assertFalse(self.send(enabled=True))
        self.assertTrue(self.send())
        self.assertFalse(
            self.send(seq=1, enabled=True, values=(1500, 1500, 1500, 1300))
        )
        self.assertTrue(self.send(seq=1, enabled=True))
        self.assertTrue(self.send(seq=2, enabled=True, values=(1600, 1400, 1510, 1300)))
        self.assertEqual(self.output(1020), (False, (1600, 1400, 1510, 1300)))

    def test_one_second_lease_holds_through_short_stalls_and_latches_at_expiry(self):
        for mode in (CTBR, DIRECT):
            for lease in (1000, 0xFFFFFFF0):
                with self.subTest(mode=mode, lease=lease):
                    self.lib.test_reset(mode, 123)
                    self.assertTrue(self.send(mode=mode, lease=lease, now=lease, ttl=1000))
                    self.assertTrue(self.send(seq=1, mode=mode, enabled=True, lease=lease, now=lease, ttl=1000))
                    values = (1600, 1400, 1510, 1300) if mode == CTBR else (1250, 1300, 1400, 1450)
                    self.assertTrue(self.send(seq=2, mode=mode, enabled=True, values=values, lease=lease, now=lease, ttl=1000))
                    for age in (100, 200, 500, 999):
                        self.assertEqual(self.output((lease+age)&0xFFFFFFFF), (False, values))
                    expired = (lease+1000)&0xFFFFFFFF
                    self.assertEqual(self.output(expired), (True, neutral(mode)))
                    self.assertFalse(self.send(seq=3, mode=mode, enabled=True, lease=expired, now=expired, ttl=1000))
                    self.assertTrue(self.send(seq=3, mode=mode, lease=expired, now=expired, ttl=1000))
                    self.assertTrue(self.send(seq=4, mode=mode, enabled=True, lease=expired, now=expired, ttl=1000))

    def test_protocol_rejects_leases_above_one_second(self):
        with self.assertRaises(ValueError):
            self.send(ttl=1001)
        packet = bytearray(command_packet(1, CTBR, False, 123, 9, 0, 1000, 1000, neutral(CTBR)))
        struct.pack_into('>H', packet, 28, 1001)
        self.assertFalse(self.lib.test_receive(bytes(packet), len(packet), 1000))

    def test_voltage_telemetry_outside_fit_preserves_direct_bench_interlock(self):
        for centivolts in (740, 824):
            self.lib.test_bench_reset(); self.configure_bench()
            self.feed(36, struct.pack('<I', 1 << 14))
            self.feed(200, b'')
            self.feed(130, bytes((2,)) + bytes(8) + struct.pack('<H', centivolts))
            self.assertEqual(self.lib.test_bench_voltage(1001), centivolts*10)
            self.assertTrue(self.lib.test_bench_ctbr_ready(1001))
            self.assertFalse(self.lib.test_bench_ready(1001))
            self.assertEqual(self.lib.test_bench_voltage(1150), 0)
        for payload in (bytes(11), bytes((2,))+bytes(8)+struct.pack('<H',7334), bytes((2,))):
            self.lib.test_bench_reset(); self.configure_bench()
            self.feed(130, payload)
            self.assertEqual(self.lib.test_bench_voltage(1001), 0)
            self.assertFalse(self.lib.test_bench_ready(1001))

    def test_old_increasing_commands_do_not_refresh(self):
        self.assertTrue(self.send())
        self.assertTrue(self.send(seq=1, enabled=True))
        self.assertFalse(self.send(seq=2, enabled=True, now=1100))
        self.assertEqual(self.output(1100), (True, neutral(CTBR)))
        self.assertFalse(self.send(seq=3, enabled=True, now=1100, lease=1100))
        self.assertTrue(self.send(seq=3, now=1100, lease=1100))
        self.assertTrue(self.send(seq=4, enabled=True, now=1101, lease=1100))

    def test_monotonic_receive_evaluation_keeps_exact_deadline_across_wrap(self):
        # A caller must sample now while holding the same lock as last_rx_ms.
        # Check the valid monotonic path, including a wrapped receiver clock;
        # the real hardware comparison verifies the caller's lock ordering.
        for lease in [990, 0xFFFFFFF0]:
            self.lib.test_reset(CTBR, 123)
            received=(lease+11)&0xFFFFFFFF
            self.assertTrue(self.send(lease=lease,now=received))
            self.assertTrue(self.send(seq=1,enabled=True,lease=lease,now=received))
            self.assertEqual(self.output(received),(False,neutral(CTBR)))
            self.assertFalse(self.output((lease+99)&0xFFFFFFFF)[0])
            self.assertTrue(self.output((lease+100)&0xFFFFFFFF)[0])

    def test_duplicate_reorder_epoch_and_session(self):
        self.assertTrue(self.send(seq=10))
        self.assertFalse(self.send(seq=10))
        self.assertFalse(self.send(seq=9))
        self.assertFalse(self.send(seq=11, epoch=124))
        self.assertFalse(self.send(seq=11, session=10, enabled=True))
        self.assertTrue(self.send(seq=0, session=10))

    def test_tick_and_sequence_wrap(self):
        self.assertTrue(self.send(seq=0xFFFFFFFF, lease=0xFFFFFFF0, now=0xFFFFFFF1))
        self.assertTrue(self.send(seq=0, enabled=True, lease=0xFFFFFFF0, now=20))
        self.assertFalse(self.send(seq=1, lease=40, now=20))
        self.assertEqual(self.output(85), (True, neutral(CTBR)))

    def test_direct_and_wrong_mode(self):
        self.assertFalse(self.send(mode=DIRECT))
        self.lib.test_reset(DIRECT, 123)
        self.assertTrue(self.send(mode=DIRECT))
        self.assertFalse(
            self.send(seq=1, mode=DIRECT, enabled=True, values=(1300,) * 4)
        )
        self.assertTrue(self.send(seq=1, mode=DIRECT, enabled=True))
        self.assertTrue(
            self.send(seq=2, mode=DIRECT, enabled=True, values=(1250, 1333, 1450, 1000))
        )
        self.assertEqual(self.output(1030)[1], (1250, 1333, 1450, 1000))
        self.assertEqual(self.output(1100), (True, neutral(DIRECT)))

    def test_malformed_packets(self):
        valid = command_packet(1, CTBR, False, 123, 9, 0, 1000, 100, neutral(CTBR))
        for n in range(len(valid)):
            self.assertFalse(self.lib.test_receive(valid[:n], n, 1000))
        for offset, value in (
            (0, 0),
            (4, 2),
            (5, 3),
            (6, 2),
            (7, 2),
            (28, 255),
            (29, 0),
            (30, 1),
            (32, 0),
        ):
            bad = bytearray(valid)
            bad[offset] = value
            self.assertFalse(self.lib.test_receive(bytes(bad), len(bad), 1000), offset)
        self.assertFalse(self.lib.test_receive(valid + b"X", len(valid) + 1, 1000))

    def test_status_and_serial_encodings(self):
        self.send(seq=99)
        data = (c.c_ubyte * 48)()
        self.lib.test_status(1010, data)
        status = ReceiverStatus.decode(bytes(data))
        self.assertEqual(
            (status.epoch, status.session, status.sequence, status.voltage_mv),
            (123, 9, 99, 7800),
        )
        self.assertFalse(status.failsafe)
        v = (c.c_ushort * 4)(1250, 1300, 1400, 1450)
        packet = (c.c_ubyte * 25)()
        self.lib.sz_motors(v, packet)
        raw = bytes(packet[:14])
        self.assertEqual(raw[:5], b"$M<\x08\xd6")
        self.assertEqual(struct.unpack("<4H", raw[5:13]), tuple(v))
        self.assertEqual(functools.reduce(int.__xor__, raw[3:], 0), 0)
        v = (c.c_ushort * 4)(1600, 1400, 1510, 1300)
        self.lib.sz_sbus(v, 1, 0, packet)
        bits = int.from_bytes(bytes(packet[1:23]), "little")
        channels = tuple((bits >> (11 * i)) & 2047 for i in range(6))
        expected = tuple(
            192 + (x - 1000) * 1601 // 1000
            for x in (1600, 1400, 1300, 1510, 2000, 2000)
        )
        self.assertEqual(channels, expected)

    def feed(self, cmd, payload, now=1000, corrupt=False):
        body = bytes((len(payload), cmd)) + payload
        packet = (
            b"$M>"
            + body
            + bytes((functools.reduce(int.__xor__, body, 0) ^ int(corrupt),))
        )
        self.lib.test_bench_feed(packet, len(packet), now)

    def configure_bench(self):
        self.feed(1, bytes((0, 1, 46)))
        self.feed(2, b"BTFL")
        self.feed(3, bytes((4, 5, 1)))
        self.feed(36, bytes(4))
        self.feed(90, bytes((0, 0, 0, 6, 0, 0, 0, 0)))
        self.feed(131, struct.pack("<HHHB", 1070, 2000, 1000, 4))
        self.feed(119, bytes((7, 0, 1)))  # ARM is bit 1, not bit 0.
        self.feed(101, struct.pack("<HHHIB", 125, 0, 0, 1, 0))
        self.feed(130, bytes((2,)) + bytes(8) + struct.pack("<H", 780))
        self.feed(214, b"")

    def test_msp_ctbr_requires_receiver_feature_and_rc_ack_but_allows_armed_fc(self):
        self.lib.test_bench_reset(); self.configure_bench()
        self.feed(200, b'')
        self.assertFalse(self.lib.test_bench_ctbr_ready(1001))
        self.feed(36, struct.pack('<I', 1 << 14))
        self.assertTrue(self.lib.test_bench_ctbr_ready(1001))
        self.feed(101, struct.pack('<HHHIB', 125, 0, 0, 2, 0))  # ARM bit 1.
        self.feed(130, bytes(11))  # No battery: not a direct-thrust readiness pass.
        self.assertTrue(self.lib.test_bench_ctbr_ready(1001))
        self.assertFalse(self.lib.test_bench_ready(1001))
        self.assertFalse(self.lib.test_bench_ctbr_ready(1150))
        self.feed(36, struct.pack('<I', (1 << 14) | (1 << 3)))
        self.assertFalse(self.lib.test_bench_ctbr_ready(1001))

    def test_msp_bench_interlocks_and_ack_timeout(self):
        self.lib.test_bench_reset()
        self.assertFalse(self.lib.test_bench_ready(1000))
        self.configure_bench()
        self.assertTrue(self.lib.test_bench_ready(1001))
        self.assertFalse(self.lib.test_bench_ready(1150))
        self.feed(101, struct.pack("<HHHIB", 125, 0, 0, 2, 0))
        self.assertFalse(self.lib.test_bench_ready(1001))
        self.configure_bench()
        self.feed(36, struct.pack("<I", 1 << 12))
        self.assertFalse(self.lib.test_bench_ready(1001))
        self.configure_bench()
        self.feed(130, bytes((2,)) + bytes(8) + struct.pack("<H", 740))
        self.assertFalse(self.lib.test_bench_ready(1001))
        self.configure_bench()
        self.feed(130, bytes((2,)) + bytes(8) + struct.pack("<H", 7334))
        self.assertFalse(self.lib.test_bench_ready(1001))  # No uint16 mV overflow.
        self.lib.test_bench_reset()
        self.feed(2, b"BTFL", corrupt=True)
        self.assertFalse(self.lib.test_bench_ready(1001))


if __name__ == "__main__":
    unittest.main()
