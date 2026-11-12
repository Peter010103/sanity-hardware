"""UART4 configuration must preserve USB and unrelated serial ports."""
import importlib.util
from pathlib import Path
import struct
import unittest

spec = importlib.util.spec_from_file_location('uart4_mode', Path(__file__).resolve().parents[1]/'scripts/uart4_mode.py')
uart4 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(uart4)


class Uart4ModeTests(unittest.TestCase):
    def setUp(self):
        self.records = [[20, 1, 5, 4, 0, 5], [0, 0, 5, 4, 0, 5],
                        [3, 64, 4, 4, 0, 5], [4, 0, 5, 4, 0, 5]]

    def test_msp_only_changes_uart4_function_and_msp_baud_with_exact_rollback(self):
        original = [r.copy() for r in self.records]
        result = uart4.plan(self.records, 'msp-bench')
        self.assertEqual(self.records, original)
        self.assertEqual(result['commands'], ['serial 3 1 115200 57600 0 115200', 'save'])
        self.assertEqual(result['restore_commands'], ['serial 3 64 57600 57600 0 115200', 'save'])
        self.assertEqual([r for r in result['expected'] if r[0] != 3], [r for r in original if r[0] != 3])
        self.assertEqual(uart4.plan(result['expected'], 'ctbr')['expected'][2][1], 64)
        self.assertFalse(uart4.plan(result['expected'], 'msp-bench')['changes_required'])

    def test_rejects_shared_functions_missing_usb_and_other_serial_receiver(self):
        for mask in [65, 2, 128]:
            with self.subTest(mask=mask):
                self.records[2][1] = mask
                with self.assertRaises(ValueError):
                    uart4.plan(self.records, 'msp-bench')
        self.records[2][1] = 1
        self.records[3][1] = 64
        with self.assertRaises(ValueError):
            uart4.plan(self.records, 'ctbr')
        self.records[0][1] = 0
        with self.assertRaises(ValueError):
            uart4.plan(self.records, 'msp-bench')

    def test_unified_msp_selects_one_receiver_preserving_other_features(self):
        before = (1 << 3) | (1 << 10) | (1 << 18)
        result = uart4.plan(self.records,'msp',before)
        self.assertEqual(result['expected_features'],(1 << 14) | (1 << 10) | (1 << 18))
        self.assertIn('set serial_update_rate_hz = 1000',result['commands'])
        self.assertIn('feature -RX_SERIAL',result['commands'])
        self.assertIn('feature RX_MSP',result['commands'])
        self.assertIn('feature RX_SERIAL',result['restore_commands'])
        self.assertIn('feature -RX_MSP',result['restore_commands'])
        with self.assertRaises(ValueError):
            uart4.plan(self.records,'msp')

    def test_serial_decode_rejects_truncation_duplicates_and_unknown_baud(self):
        raw = b''.join(struct.pack('<BHBBBB', *r) for r in self.records)
        self.assertEqual(uart4.serial_records(raw), self.records)
        for invalid in [b'', raw[:-1], raw+raw[:7], struct.pack('<BHBBBB', 20, 1, 99, 4, 0, 5)]:
            with self.assertRaises(ValueError):
                uart4.serial_records(invalid)


if __name__ == '__main__':
    unittest.main()
