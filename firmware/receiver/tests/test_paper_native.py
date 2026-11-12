"""Actual ESP C decoder/lease/forwarder, matched against independent wire vectors."""
import ctypes as c
import json
from pathlib import Path
import struct
from test_control import ControlTests
from sanity_zenoh.protocol import PAPER_NATIVE, PaperDemand, paper_command_packet
from sanity_zenoh.controller_status import crc8
from sanity_zenoh.paper_status import decode_wrapped

class PaperNativeTests(ControlTests):
    def test_v2_model_diagnostics_and_v1_compatibility(self):
        def wrap(p):
            raw=struct.pack('<BHH',0,0x7f16,80)+p
            return raw+bytes([crc8(raw)])
        p=bytearray(80);p[:8]=b'SIPS\x02\x01\x01\x01'
        struct.pack_into('>II',p,8,0x50494e31,1000);p[54]=255
        p[76:80]=bytes([0x8f,15,5,10]);struct.pack_into('>H',p,50,11)
        raw=wrap(p)
        self.assertTrue(self.lib.test_paper_capability(raw,len(raw)))
        d=decode_wrapped(raw)
        self.assertEqual((d['version'],d['model_reason'],d['degraded_mask'],d['recovering_mask'],d['seeded_mask'],d['reason']),(2,0x8f,15,5,10,11))
        for offset,value in [(4,3),(76,16),(77,16),(78,16),(79,16)]:
            bad=bytearray(p);bad[offset]=value;raw=wrap(bad)
            self.assertFalse(self.lib.test_paper_capability(raw,len(raw)))
            with self.assertRaises(ValueError):decode_wrapped(raw)
        raw=bytearray(wrap(p));raw[-1]^=1
        self.assertFalse(self.lib.test_paper_capability(bytes(raw),len(raw)))
        with self.assertRaises(ValueError):decode_wrapped(raw)
        p[4]=1;p[76:80]=bytes(4);struct.pack_into('>H',p,50,0)
        raw=wrap(p);self.assertTrue(self.lib.test_paper_capability(raw,len(raw)))
        self.assertIsNone(decode_wrapped(raw)['model_reason'])
        p[76]=1;raw=wrap(p)
        self.assertFalse(self.lib.test_paper_capability(raw,len(raw)))
        with self.assertRaises(ValueError):decode_wrapped(raw)
    def setUp(self):self.lib.test_reset(PAPER_NATIVE,123)
    def packet(self,seq=0,enabled=False,lease=1000,ttl=100,demand=PaperDemand(),request=False,epoch=123,session=9):
        return paper_command_packet(1,enabled,epoch,session,seq,lease,ttl,demand,request)
    def accept(self,**kw):
        p=self.packet(**kw);return bool(self.lib.test_receive(p,len(p),1000))
    def test_native_disabled_claim_enabled_zero_positive_expiry(self):
        self.assertFalse(self.accept(seq=0,enabled=True))
        self.assertTrue(self.accept())
        self.assertTrue(self.accept(seq=1,enabled=True))
        self.assertTrue(self.accept(seq=2,enabled=True,demand=PaperDemand(.8,(1,2,3),(.1,-.2,.3)),request=True))
        out=(c.c_ubyte*76)();p=self.packet(seq=2,enabled=True,demand=PaperDemand(.8,(1,2,3),(.1,-.2,.3)),request=True)
        self.assertEqual(self.lib.test_paper_frame(p,len(p),1055,out),76)
        payload=bytes(out)[10:-2]
        self.assertEqual(payload[:5],b'SIP1\x01');self.assertEqual(struct.unpack_from('>I',payload,24)[0],45)
        self.assertEqual(struct.unpack_from('>3f',payload,40),(1,2,3))
        self.assertEqual(self.lib.test_paper_frame(p,len(p),1100,out),0)
        self.assertTrue(self.output(1100)[0]);self.assertFalse(self.lib.test_enabled())
        self.assertFalse(self.accept(seq=3,enabled=True,lease=1100))
    def test_partial_fallback_write_and_expiry_during_uart_delay(self):
        out=(c.c_ubyte*76)();p=self.packet(enabled=True)
        for written in (-1,0,1,21,23):
            self.assertEqual(self.lib.test_paper_after_rc(p,len(p),1000,22,written,out),0)
        self.assertEqual(self.lib.test_paper_after_rc(p,len(p),1000,22,22,out),76)
        self.assertEqual(self.lib.test_paper_after_rc(p,len(p),1100,22,22,out),0)
        stop=self.packet()
        self.assertEqual(self.lib.test_paper_after_rc(stop,len(stop),2000,22,-1,out),76)
    def test_capability_lateness_does_not_stop_active_negotiation(self):
        p=bytearray(80);p[:8]=b'SIPS\x01\x01\x01\x01';struct.pack_into('>II',p,8,0x50494e31,1000)
        raw=struct.pack('<BHH',0,0x7f16,80)+p;raw+=bytes([crc8(raw)])
        self.lib.test_paper_observe(b'',0,123,1000);self.assertFalse(self.lib.test_paper_admit(123,1000))
        self.lib.test_paper_observe(raw,len(raw),123,1000)
        self.assertTrue(self.lib.test_paper_admit(123,1149));self.assertFalse(self.lib.test_paper_admit(123,1150))
        self.lib.test_paper_claim(1)
        self.assertTrue(self.lib.test_paper_admit(123,5000));self.assertTrue(self.lib.test_paper_forward(123,1))
        self.assertFalse(self.lib.test_paper_admit(124,5000));self.assertFalse(self.lib.test_paper_forward(124,1))
        self.lib.test_paper_observe(b'',0,123,5000)
        self.assertFalse(self.lib.test_paper_admit(123,5000));self.assertFalse(self.lib.test_paper_forward(123,1))
    def test_independent_root_golden(self):
        vectors=json.loads(Path(__file__).with_name('paper_native_golden.json').read_text())
        out=(c.c_ubyte*76)()
        for name,row in vectors['frames'].items():
            p=bytes.fromhex(row['network_hex']);expected=bytes.fromhex(row['fc_hex'])
            now=0xf0000001+(113 if name=='enabled' else 0)
            self.assertEqual(self.lib.test_paper_frame(p,len(p),now,out),76)
            self.assertEqual(bytes(out)[10:-2],expected)
            fields=struct.unpack('>4sBBBBQIIIHBBI7f',p)
            demand=PaperDemand(fields[-7],tuple(fields[-6:-3]),tuple(fields[-3:]))
            self.assertEqual(p,paper_command_packet(15,bool(fields[4]&1),fields[5],fields[6],fields[7],fields[8],fields[9],demand,bool(fields[4]&2)))
    def test_malformed_native_headers_and_values(self):
        base=self.packet()
        for offset,value in [(4,4),(5,1),(7,2),(7,4),(30,2),(31,1),(32,0)]:
            p=bytearray(base);p[offset]=value
            self.assertFalse(self.lib.test_receive(bytes(p),len(p),1000),(offset,value))
        for offset,value in [(36,float('nan')),(36,4.1),(40,float('inf')),(40,1001),(52,2.4),(60,-2.7)]:
            p=bytearray(self.packet(enabled=True));struct.pack_into('>f',p,offset,value)
            self.assertFalse(self.lib.test_receive(bytes(p),len(p),1000))
        self.assertFalse(self.lib.test_receive(base[:-1],63,1000))
    def test_identity_wrap_duplicate_reconnect_and_stale_claim(self):
        self.assertTrue(self.accept(seq=0xffffffff))
        self.assertTrue(self.accept(seq=0,enabled=True))
        self.assertFalse(self.accept(seq=0,enabled=True))
        self.assertFalse(self.accept(seq=0xffffffff,enabled=True))
        self.assertFalse(self.accept(seq=1,enabled=True,epoch=124))
        self.lib.test_reset(PAPER_NATIVE,124)
        self.assertFalse(self.accept(seq=1,enabled=True,epoch=124))
        self.assertFalse(self.accept(epoch=124,lease=800))
        self.assertTrue(self.accept(epoch=124))
    def test_capability_absent_old_fw_wrong_backend_and_consumed_identity(self):
        p=bytearray(80);p[:8]=b'SIPS\x01\x01\x01\x03';struct.pack_into('>II',p,8,0x50494e31,1000)
        struct.pack_into('>Q6I2H',p,16,123,9,4,9,3,123456,123400,55,0);p[54]=255
        def wrap(data):
            v=struct.pack('<BHH',0,0x7f16,len(data))+data;return v+bytes([crc8(v)])
        raw=wrap(p);self.assertTrue(self.lib.test_paper_capability(raw,len(raw)))
        decoded=decode_wrapped(raw);self.assertEqual(decoded['accepted_sequence'],4);self.assertEqual(decoded['consumed_sequence'],3)
        for offset,value in [(5,0),(6,0),(7,0),(8,0)]:
            bad=bytearray(p);bad[offset]=value;raw=wrap(bad)
            self.assertFalse(self.lib.test_paper_capability(raw,len(raw)))
        self.assertFalse(self.lib.test_paper_capability(b'',0))

# Inherited legacy tests expect CTBR in setUp; explicitly keep legacy coverage in
# their own suites, and run only the new native methods here.
for name in list(vars(ControlTests)):
    if name.startswith('test_') and name not in PaperNativeTests.__dict__:setattr(PaperNativeTests,name,None)
