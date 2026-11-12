import ctypes as c
import json
import math
from pathlib import Path
import random
import struct
import subprocess
import tempfile
import time
import unittest

from sanity_zenoh.fc_telemetry import BATCH_HEADER, RECORD_HEADER, decode_fc_telemetry


def msp(command, payload=b"", error=False):
    checksum = len(payload) ^ command
    for byte in payload:
        checksum ^= byte
    return b"$M" + (b"!" if error else b">") + bytes((len(payload), command)) + payload + bytes((checksum,))


class FCTelemetryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        root = Path(__file__).resolve().parents[1]
        output = Path(cls.temp.name)/"fc.so"
        subprocess.run(["cc","-std=c11","-Wall","-Wextra","-Werror","-pedantic","-fsanitize=undefined",
            "-fno-sanitize-recover=all","-shared","-fPIC","-I",str(root/"main"),str(root/"main/fc_telemetry.c"),
            str(root/"main/control.c"),str(root/"tests/fc_telemetry_wrapper.c"),"-o",str(output)],check=True)
        cls.lib = c.CDLL(str(output))
        cls.lib.test_fc_feed.argtypes = [c.c_char_p,c.c_uint,c.c_uint64]
        cls.lib.test_fc_query.argtypes = [c.c_uint,c.c_uint64]
        cls.lib.test_fc_output.argtypes = [c.c_uint64]
        cls.lib.test_fc_packet.argtypes = [c.c_void_p,c.c_uint]

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def setUp(self):
        self.lib.test_fc_reset()

    def packet(self):
        out = c.create_string_buffer(1400)
        count = self.lib.test_fc_packet(out,len(out))
        self.assertGreater(count,0)
        return out.raw[:count]

    def feed(self, command, payload=b"", now=200000, error=False):
        frame = msp(command,payload,error)
        self.assertEqual(self.lib.test_fc_feed(frame,len(frame),now),1)
        return decode_fc_telemetry(self.packet(),1)["records"][0]

    def test_controller_query_response_passes_real_esp_parser(self):
        from sanity_zenoh.controller_status import wrapped_query,crc8,COMMAND
        out=(c.c_ubyte*6)();self.lib.sz_fc_controller_query_payload(out)
        self.assertEqual(bytes(out),wrapped_query()[5:-1])
        raw=bytearray(80);raw[0]=1;raw[42]=3
        raw[44:56]=bytes.fromhex('56003f001451333231343731')
        inner=struct.pack('<BHH',0,COMMAND,80)+raw
        wrapped=inner+bytes([crc8(inner)])
        self.lib.test_fc_query(255,190000)
        record=self.feed(255,wrapped)
        self.assertEqual(record['controller_state']['fc_uid_msp'],'56003f001451333231343731')
        self.assertEqual(record['query_write_begin_esp_us'],190000)
        bad=bytearray(wrapped);bad[-1]^=1
        self.assertIn('decode_error',self.feed(255,bytes(bad)))

    def test_signed_gyro_units_axes_and_64_bit_timing(self):
        start = 2**40
        self.assertEqual(self.lib.test_fc_query(102,start),1)
        event = self.feed(102,struct.pack("<9h",1,-2,3,-600,500,-32768,7,8,-9),start+7000)
        self.assertEqual(event["query_write_begin_esp_us"],start)
        self.assertEqual(event["response_parsed_esp_us"],start+7000)
        self.assertEqual(event["gyro_counts_fc"],[-600,500,-32768])
        self.assertEqual(event["gyro_dps_fc"],[-600*2000/32768,500*2000/32768,-2000])
        self.assertAlmostEqual(event["gyro_rad_s_fc"][0],math.radians(-600*2000/32768))
        self.assertIn("no model-frame",event["frame"])
        self.assertEqual(event["epoch"],0x123456789abcdef0)

    def test_timing_query_has_independent_backoff_and_no_authority_mailbox(self):
        from sanity_zenoh.controller_status import crc8
        out=(c.c_ubyte*6)();self.lib.sz_fc_timing_query_payload(out)
        self.assertEqual(bytes(out)[:5],b'\0\x14\x7f\0\0')
        self.assertEqual(bytes(out)[5],crc8(bytes(out)[:5]))
        self.assertEqual(self.lib.test_fc_query(255,1000),1)
        self.assertEqual(self.lib.test_fc_query(254,2000),1)
        payload=struct.pack('<BHH',0,0x7f14,80)+bytes(80)
        payload+=bytes([crc8(payload)])
        frame=msp(255,payload)
        self.assertEqual(self.lib.test_fc_feed(frame,len(frame),3000),1)
        self.assertFalse(self.lib.test_fc_is_controller())
        self.assertEqual(self.lib.test_fc_query_start(),2000)
        self.assertEqual(self.lib.test_fc_query(255,4000),0) # controller still pending
        self.assertEqual(self.lib.test_fc_query(254,4000),1) # only timing was completed

    def test_old_firmware_optional_error_cannot_backoff_controller(self):
        self.assertEqual(self.lib.test_fc_query(255,1000),1)
        self.assertEqual(self.lib.test_fc_query(254,2000),1)
        frame=msp(255,error=True)
        self.assertEqual(self.lib.test_fc_feed(frame,len(frame),3000),1)
        self.assertFalse(self.lib.test_fc_is_controller())
        self.assertEqual(self.lib.test_fc_query(254,4000),0) # optional backoff
        from sanity_zenoh.controller_status import crc8
        payload=struct.pack('<BHH',0,0x7f11,80)+bytes(80)
        payload+=bytes([crc8(payload)]);frame=msp(255,payload)
        self.assertEqual(self.lib.test_fc_feed(frame,len(frame),5000),1)
        self.assertTrue(self.lib.test_fc_is_controller())
        self.assertEqual(self.lib.test_fc_query_start(),1000)
        self.assertEqual(self.lib.test_fc_query(255,6000),1)

    def test_motor_output_and_rc_are_separate_from_motor_cache_writes(self):
        event = self.feed(104,struct.pack("<8H",1100,1200,1300,1400,0,0,0,0))
        self.assertEqual(event["motor_outputs_external_fc"][:4],[1100,1200,1300,1400])
        self.lib.test_fc_output(2**40)
        event = decode_fc_telemetry(self.packet())["records"][0]
        self.assertEqual(event["command_sequence"],99)
        self.assertEqual(event["transmitted_rc_channels"][:4],[1550,1450,1350,1600])
        self.assertEqual(event["uart_write_return_esp_us"]-event["uart_write_begin_esp_us"],100)
        self.assertNotIn("motor_outputs_external_fc",event)
        event = self.feed(105,struct.pack("<8H",*[1500]*8))
        self.assertEqual(len(event["received_rc_channels_fc"]),8)

    def test_rpm_quality_and_estimate_require_all_four_motors(self):
        payload = bytes((4,))+b"".join(struct.pack("<IHBHHH",12000+i*1000,100,25,760,40,12) for i in range(4))
        self.feed(139,payload)
        event = decode_fc_telemetry(self.packet(),rpm_kT=3e-8)["records"][0]
        expected = 3e-8 * sum((rpm*2*math.pi/60)**2 for rpm in (12000,13000,14000,15000))
        self.assertAlmostEqual(event["estimated_collective_N"],expected)
        bad = bytearray(payload);struct.pack_into("<H",bad,5,10000)
        self.feed(139,bytes(bad))
        self.assertIsNone(decode_fc_telemetry(self.packet(),rpm_kT=3e-8)["records"][0]["estimated_collective_N"])
        self.feed(139,b"\x00")
        self.assertIn("decode_error",decode_fc_telemetry(self.packet())["records"][0])

    def test_optional_query_timeouts_backoff_and_error_do_not_change_control(self):
        self.assertEqual(self.lib.test_fc_query(139,1000),1)
        self.assertEqual(self.lib.test_fc_query(139,100999),0)
        self.assertEqual(self.lib.test_fc_query(139,101000),2)
        self.assertTrue(decode_fc_telemetry(self.packet())["records"][0]["query_timed_out"])
        self.assertEqual(self.lib.test_fc_query(139,1100999),0)
        self.assertEqual(self.lib.test_fc_query(139,1101000),1)
        event = self.feed(139,now=1102000,error=True)
        self.assertTrue(event["msp_error"])
        self.assertEqual(self.lib.test_fc_query(139,1103000),0)
        self.assertEqual(self.lib.test_fc_query(102,1103000),1)

    def test_corrupt_partial_oversized_and_unknown_frames_recover(self):
        good = msp(102,struct.pack("<9h",*[0]*9))
        corrupt = bytearray(good);corrupt[-1]^=1
        self.assertEqual(self.lib.test_fc_feed(bytes(corrupt),len(corrupt),1000),0)
        self.assertEqual(self.lib.test_fc_feed(good[:8],8,2000),0)
        self.assertEqual(self.lib.test_fc_feed(good[8:],len(good)-8,2001),1)
        self.assertGreater(self.lib.test_fc_malformed(),0)
        for frame in (msp(139,bytes(255)),msp(42,b"abc")):
            self.assertEqual(self.lib.test_fc_feed(frame,len(frame),3000),0)
        self.assertEqual(self.lib.test_fc_feed(good,len(good),4000),1)
        self.lib.test_fc_feed(b"$M>",3,5000)
        self.assertEqual(self.lib.test_fc_feed(good,len(good),106000),1)

    def test_wire_truncation_invalid_headers_and_bad_field_lengths(self):
        self.feed(102,struct.pack("<9h",*[0]*9))
        good = self.packet()
        for length in range(len(good)):
            with self.assertRaises(ValueError):decode_fc_telemetry(good[:length])
        for offset in (0,4,5):
            bad = bytearray(good);bad[offset]=0
            with self.assertRaises(ValueError):decode_fc_telemetry(bytes(bad))
        with self.assertRaises(ValueError):decode_fc_telemetry(good,expected_device=2)
        with self.assertRaises(ValueError):decode_fc_telemetry(good+b"x")
        self.feed(102,b"\0"*16)
        self.assertIn("decode_error",decode_fc_telemetry(self.packet())["records"][0])
        self.assertEqual(self.lib.test_fc_packet(c.create_string_buffer(12),12),0)

    def test_health_distinguishes_brownout_and_network_reconnection(self):
        for code,name in ((1,"POWERON"),(9,"BROWNOUT"),(14,"PWR_GLITCH"),(4,"PANIC")):
            self.lib.test_fc_health(code)
            health = decode_fc_telemetry(self.packet())["records"][0]["esp_health"]
            self.assertEqual(health["reset_reason"],name)
            self.assertEqual(health["boot_id"],0xfedcba9876543210)
            self.assertEqual(health["boot_count"],3)
            self.assertEqual(health["wifi_disconnects"],2)
            self.assertEqual(health["wifi_rssi_dBm"],-62)
            self.assertEqual(health["uptime_us"],10000000000)
            self.assertIsNone(health["esp_supply_voltage_V"])

    def test_sender_records_raw_and_decoded_without_changing_interlocks(self):
        from sanity_zenoh.client import SanitySender
        from sanity_zenoh.recording import EventRecorder
        self.feed(102,struct.pack("<9h",*[0]*9))
        class Sample:
            payload = self.packet()
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/"events.jsonl"
            recorder = EventRecorder(path)
            sender = SanitySender.__new__(SanitySender)
            sender.device=1;sender.telemetry_kT=None;sender.on_event=recorder
            sender.enabled=True;sender.rearm_required=False;sender.stop_reason=None
            sender.receive_telemetry(Sample())
            Sample.payload=b"bad"
            sender.receive_telemetry(Sample())
            recorder.close()
            records=[json.loads(line) for line in path.read_text().splitlines()]
            self.assertIn("telemetry",records[0]);self.assertIn("decode_error",records[1])
            self.assertIn("unix_ns",records[0]);self.assertIn("host_ns",records[0])
            self.assertTrue(sender.enabled);self.assertFalse(sender.rearm_required)

    def test_real_zenoh_loopback_reaches_existing_sender_recording(self):
        import socket
        import zenoh
        from sanity_zenoh.client import SanitySender
        self.feed(102,struct.pack("<9h",1,2,3,4,5,6,7,8,9))
        payload = self.packet()
        with socket.socket() as sock:
            sock.bind(("127.0.0.1",0));port=sock.getsockname()[1]
        endpoint=f"tcp/127.0.0.1:{port}"
        events=[]
        sender=SanitySender(device=1,listen=endpoint,on_event=events.append)
        config=zenoh.Config();config.insert_json5("mode",'"client"')
        config.insert_json5("connect/endpoints",json.dumps([endpoint]))
        config.insert_json5("scouting/multicast/enabled","false")
        config.insert_json5("scouting/gossip/enabled","false")
        peer=zenoh.open(config)
        try:
            deadline=time.monotonic()+3
            while not any(e["kind"]=="zenoh_rx_fc_telemetry" for e in events) and time.monotonic()<deadline:
                peer.put("sanity/01/telemetry/fc",payload);time.sleep(.02)
            event=next(e for e in events if e["kind"]=="zenoh_rx_fc_telemetry")
            self.assertEqual(event["payload_hex"],payload.hex())
            self.assertEqual(event["telemetry"]["records"][0]["gyro_counts_fc"],[4,5,6])
            self.assertEqual(event["telemetry"]["records"][0]["gyro_dps_fc"],[x*2000/32768 for x in (4,5,6)])
            self.assertFalse(sender.enabled)
        finally:
            peer.close();sender.close()


if __name__ == "__main__":
    unittest.main()
