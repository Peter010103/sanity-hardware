"""Motor bench guards and stop-delivery tests using mocked FCs; never actuates hardware."""
import contextlib
import copy
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

import betaflight_motors as bm

PROFILE = {"identity": {"firmware_version": "4.5.1", "msp_api_version": "1.46"},
           "board": {"board_name": "BETAFPVF405"}, "uid_payload_hex": "example-uid",
           "features": {"reversible_3d_enabled": False},
           "motor_protocol": {"name": "DSHOT300"},
           "motor_config": {"motor_count": 4, "min_command_setting": 1000,
                            "max_throttle_setting": 2000}}


class Tests(unittest.TestCase):
    def test_packet_four_little_endian_outputs_and_checksum(self):
        packet = bm.motor_packet([1000, 1100, 1000, 1000], 4, 1000, 2000, 1100)
        self.assertEqual(packet[:5], b"$M<\x08\xd6")
        self.assertEqual(struct.unpack("<4H", packet[5:-1]), (1000, 1100, 1000, 1000))
        checksum = 0
        for value in packet[3:]:
            checksum ^= value
        self.assertEqual(checksum, 0)

    def test_invalid_vectors_and_ceiling(self):
        for values, ceiling in [([1000]*3, 1100), ([999]*4, 1100),
                                ([1101]*4, 1100), ([1000]*4, 2001)]:
            with self.assertRaises(ValueError):
                bm.motor_packet(values, 4, 1000, 2000, ceiling)

    def test_wrong_controller_or_changed_configuration_rejected(self):
        for key, value in [("uid_payload_hex", "another-uid"),
                           ("motor_protocol", {"name": "DSHOT600"}),
                           ("features", {"reversible_3d_enabled": True})]:
            live = copy.deepcopy(PROFILE)
            live[key] = value
            with self.assertRaises(ValueError):
                bm.validate_setup(live, PROFILE)

    def test_armed_and_unknown_armed_state_rejected(self):
        client = Mock()
        for flags, box_ids in [(1, [0]), (0, [])]:
            client.query.return_value = struct.pack("<HHHIB", 125, 0, 33, flags, 0)
            with self.assertRaises(RuntimeError):
                bm.require_disarmed(client, box_ids)

    def run_main(self, execute, cells, exchange_side_effect=None):
        with tempfile.TemporaryDirectory() as directory:
            profile = Path(directory)/"fc_profile.json"
            profile.write_text(json.dumps(PROFILE))
            args = ["betaflight_motors.py", "--port", "/dev/fake", "--profile", str(profile),
                    "--values", "1100", "1000", "1000", "1000",
                    "--ceiling", "1100", "--duration", "1"]
            if execute:
                args += ["--execute", "--bench-ready"]
            client = Mock()
            client.query.side_effect = lambda command: (bytes([0]) if command == 119 else
                struct.pack("<BHBHhBH", cells, 530, 74, 0, 0, 0, 740))
            client._exchange.side_effect = exchange_side_effect
            transport = Mock()
            with patch.object(sys, "argv", args), patch.object(bm, "USBSerial", return_value=transport), \
                 patch.object(bm, "MSP", return_value=client), \
                 patch.object(bm, "inspect_controller", return_value=copy.deepcopy(PROFILE)), \
                 patch.object(bm, "require_disarmed"), patch.object(bm.signal, "signal"), \
                 contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                result = bm.main()
            transport.close.assert_called_once()
            return result, client

    def test_dry_run_sends_no_motor_packets(self):
        result, client = self.run_main(False, 0)
        self.assertEqual(result, 0)
        client._exchange.assert_not_called()

    def test_missing_battery_prevents_all_motor_writes(self):
        result, client = self.run_main(True, 0)
        self.assertEqual(result, 1)
        client._exchange.assert_not_called()

    def test_lost_command_ack_still_attempts_stop(self):
        result, client = self.run_main(True, 2, [TimeoutError("lost ack"), b"", b"", b""])
        self.assertEqual(result, 1)
        packets = [call.args[0] for call in client._exchange.call_args_list]
        self.assertEqual(len(packets), 4)
        for packet in packets[1:]:
            self.assertEqual(struct.unpack("<4H", packet[5:-1]), (1000, 1000, 1000, 1000))


if __name__ == "__main__":
    unittest.main()
