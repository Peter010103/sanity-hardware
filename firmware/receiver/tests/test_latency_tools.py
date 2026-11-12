"""The physical timing harness must retain its bounded, stop-capable interface."""
import importlib.util
from pathlib import Path
import unittest
import sys
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from sanity_zenoh.protocol import CTBR,DIRECT,COMMAND,command_packet,neutral
spec=importlib.util.spec_from_file_location('measure_command_latency',Path(__file__).resolve().parents[1]/'scripts/measure_command_latency.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class LatencyToolTests(unittest.TestCase):
    def test_wire_guard_limits_zero_thrust_ctbr_and_single_motor_counts(self):
        def allowed(mode,enabled,values):
            return module.allowed_packet(COMMAND.unpack(command_packet(1,mode,enabled,123,9,0,1000,100,values)),mode)
        self.assertTrue(allowed(CTBR,True,(1400,1500,1500,1000)))
        self.assertFalse(allowed(CTBR,True,(1400,1500,1500,1001)))
        self.assertFalse(allowed(CTBR,True,(1601,1500,1500,1000)))
        self.assertTrue(allowed(DIRECT,True,(1000,1300,1000,1000)))
        self.assertFalse(allowed(DIRECT,True,(1250,1250,1000,1000)))
        self.assertFalse(allowed(DIRECT,True,(1301,1000,1000,1000)))
        self.assertTrue(allowed(DIRECT,False,neutral(DIRECT)))

    @staticmethod
    def reply(command,payload=b'',direction=b'>'):
        frame=module.msp_frame(command,payload)
        return frame[:2]+direction+frame[3:]

    def test_batch_handles_fragmented_reordered_replies_and_bad_checksums(self):
        good=self.reply(101,b'status')
        bad=good[:-1]+bytes((good[-1]^1,))
        stream=b'noise'+bad+self.reply(104,b'motors')+self.reply(101,b'status')
        chunks=iter([stream[:4],stream[4:11],stream[11:19],stream[19:]])
        writes=[]
        transport=SimpleNamespace(write=lambda data,deadline:writes.append(data),read=lambda deadline:next(chunks))
        client=module.bf.MSP(transport,timeout=1)
        result=module.query_batch(client,[101,104])
        self.assertEqual(writes,[module.bf.request_frame(101)+module.bf.request_frame(104)])
        self.assertEqual(result[101][0],b'status')
        self.assertEqual(result[104][0],b'motors')
        self.assertEqual(result[101][1],result[104][1])
        self.assertLessEqual(result[104][2],result[101][2])
        self.assertEqual(client.buffer,bytearray())

    def test_batch_rejects_duplicate_ids_and_fc_errors(self):
        client=module.bf.MSP(SimpleNamespace(write=lambda data,deadline:None,read=lambda deadline:self.reply(101,direction=b'!')))
        with self.assertRaises(ValueError):module.query_batch(client,[101,101])
        with self.assertRaises(module.bf.MSPError):module.query_batch(client,[101])

    def test_usb_control_frames_only_inhibit_or_explicit_stop(self):
        self.assertEqual(module.msp_frame(99,b'\x01'),b'$M<\x01\x63\x01\x63')
        stop=module.msp_frame(214,b'\xe8\x03'*4)
        self.assertEqual(stop,b'$M<\x08\xd6'+b'\xe8\x03'*4+b'\xde')


if __name__=='__main__':unittest.main()
