import ctypes as c
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest

REQ=struct.Struct('>4sBBBBQIIIHH')


class MspTuneTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory();folder=Path(cls.temp.name);main=Path(__file__).resolve().parents[1]/'main'
        (folder/'wrap.c').write_text('''#include "msp_tune.h"
#include <string.h>
static struct sz_tune_rpc rpc;static struct sz_state state;static struct sz_tune_context ctx;static uint64_t clock_us;
void init(void){sz_tune_init(&rpc);sz_init(&state,1,1,123);struct sz_command q={.epoch=123,.session=9,.sequence=1,.lease_ms=1000,.ttl_ms=1000,.device=1,.mode=1,.values={1500,1500,1500,1000}};sz_accept(&state,&q,1000);ctx=(struct sz_tune_context){.state=&state,.now_ms=1000,.profile=0,.fc_ready=true,.fc_disarmed=true};clock_us=1000000;}
void mode(int enabled,int disarmed,unsigned now){state.enabled=enabled;ctx.fc_disarmed=disarmed;ctx.now_ms=now;clock_us=(uint64_t)now*1000;}
void ready(int value){ctx.fc_ready=value;}
int submit(const unsigned char *p,unsigned n){return sz_tune_submit(&rpc,p,n,&ctx,clock_us);}
unsigned poll(unsigned char *out){return sz_tune_poll(&rpc,&ctx,clock_us,out);}
void response(unsigned cmd,const unsigned char *p,unsigned n,int error){struct sz_fc_record r={.command=cmd,.length=n,.type=error?3:2,.command_rx_us=clock_us+1000,.end_us=clock_us+2000};memcpy(r.payload,p,n);sz_tune_response(&rpc,&r,clock_us+2000);}
unsigned reply(unsigned char *out){memcpy(out,rpc.reply,rpc.reply_length);return rpc.reply_length;}
''')
        subprocess.run(['cc','-std=c11','-shared','-fPIC','-Wall','-Wextra','-Werror','-fsanitize=undefined','-fno-sanitize-recover=all','-I',str(main),str(folder/'wrap.c'),str(main/'msp_tune.c'),str(main/'control.c'),'-o',str(folder/'test.so')],check=True)
        cls.lib=c.CDLL(str(folder/'test.so'));cls.lib.submit.argtypes=[c.c_char_p,c.c_uint];cls.lib.response.argtypes=[c.c_uint,c.c_char_p,c.c_uint,c.c_int]

    @classmethod
    def tearDownClass(cls):cls.temp.cleanup()

    def setUp(self):
        self.lib.init();self.identifier=0;self.pid=bytes([74,92,39,77,97,46,80,90,0,50,50,50,40,40,40]);a=bytearray(61);a[39]=39;a[40]=46;a[47]=100;a[52]=15;struct.pack_into('<3H',a,32,138,143,120);self.advanced=bytes(a)

    def request(self,command,payload=b'',**kw):
        self.identifier+=1
        raw=REQ.pack(b'SZM1',1,1,command,len(payload),kw.get('epoch',123),kw.get('session',9),kw.get('identifier',self.identifier),kw.get('lease',1000),800,0)+payload
        return raw,self.lib.submit(raw,len(raw))

    def reply_code(self):
        out=(c.c_ubyte*128)();n=self.lib.reply(out);return bytes(out[:n])[7]

    def poll(self):
        out=(c.c_ubyte*134)();n=self.lib.poll(out);return bytes(out[:n])

    def baseline(self):
        for cmd,payload in ((112,self.pid),(94,self.advanced)):
            self.request(cmd);frame=self.poll();self.assertEqual(frame[4],cmd);self.lib.response(cmd,payload,len(payload),0);self.assertEqual(self.reply_code(),0)

    def test_ownership_expiry_and_whitelist(self):
        for kw in ({'session':8},{'epoch':124},{'lease':100}):
            self.assertFalse(self.request(112,**kw)[1]);self.assertFalse(self.poll())
        self.assertFalse(self.request(245)[1]);self.assertEqual(self.reply_code(),5)

    def test_bounded_gain_write_is_acked_and_duplicate_not_reapplied(self):
        self.baseline();self.lib.mode(1,0,1000);p=bytearray(self.pid);p[0]=70
        raw,ok=self.request(202,bytes(p));self.assertTrue(ok);self.assertEqual(self.poll()[4],202);self.lib.response(202,b'',0,0)
        self.assertEqual(self.reply_code(),0);self.assertTrue(self.lib.submit(raw,len(raw)));self.assertFalse(self.poll())

    def test_i_yaw_other_settings_and_large_gain_changes_rejected(self):
        self.baseline();self.lib.mode(1,0,1000)
        for index,value in ((1,91),(6,79),(0,120),(2,50)):
            p=bytearray(self.pid);p[index]=value;self.assertFalse(self.request(202,bytes(p))[1]);self.assertEqual(self.reply_code(),2)
        for index,value in ((36,121),(47,90),(52,1)):
            p=bytearray(self.advanced);p[index]=value;self.assertFalse(self.request(95,bytes(p))[1])

    def test_feedforward_is_bounded_and_does_not_touch_other_advanced_fields(self):
        self.baseline();self.lib.mode(1,0,1000);p=bytearray(self.advanced);struct.pack_into('<H',p,32,124)
        self.assertTrue(self.request(95,bytes(p))[1]);self.assertEqual(self.poll()[4],95);self.lib.response(95,b'',0,0);self.assertEqual(self.reply_code(),0)

    def test_save_denied_while_armed_and_rechecked_at_uart_execution(self):
        self.baseline();self.lib.mode(1,0,1000);self.assertFalse(self.request(250)[1])
        self.lib.mode(0,1,1000);self.assertTrue(self.request(250)[1]);self.lib.mode(1,0,1000);self.assertFalse(self.poll());self.assertEqual(self.reply_code(),2)

    def test_expired_queued_write_never_reaches_uart(self):
        self.baseline();self.lib.mode(1,0,1000);p=bytearray(self.pid);p[0]=70;self.request(202,bytes(p));self.lib.mode(1,0,2001);self.assertFalse(self.poll());self.assertEqual(self.reply_code(),2)

    def test_owner_loss_restores_original_only_after_fc_disarmed(self):
        self.baseline();self.lib.mode(1,0,1000);p=bytearray(self.pid);p[0]=70;self.request(202,bytes(p));self.poll();self.lib.response(202,b'',0,0)
        self.lib.mode(0,0,2001);self.assertFalse(self.poll());self.lib.mode(0,1,2001);frame=self.poll();self.assertEqual(frame[4],202);self.assertEqual(frame[5:-1],self.pid);self.lib.response(202,b'',0,0);self.assertFalse(self.poll())

    def test_lost_write_ack_still_restores_original_after_lease_loss(self):
        self.baseline();self.lib.mode(1,0,1000);p=bytearray(self.pid);p[0]=70;self.request(202,bytes(p));self.poll()
        self.lib.mode(1,0,1151);self.poll();self.assertEqual(self.reply_code(),3)
        self.lib.mode(0,1,2001);frame=self.poll();self.assertEqual(frame[4],202);self.assertEqual(frame[5:-1],self.pid)

    def test_disarmed_eeprom_save_allows_slow_reply_and_duplicate_during_fc_pause(self):
        self.baseline();raw,ok=self.request(250);self.assertTrue(ok);self.assertEqual(self.poll()[4],250)
        self.lib.ready(0);self.lib.mode(0,1,1151);self.assertTrue(self.lib.submit(raw,len(raw)));self.assertFalse(self.poll());self.assertEqual(self.reply_code(),0)
        self.lib.ready(1);self.lib.response(250,b'',0,0);self.assertEqual(self.reply_code(),0)

    def test_old_owner_cannot_overwrite_cached_acknowledgement(self):
        self.baseline();self.assertEqual(self.reply_code(),0);self.assertFalse(self.request(112,session=8)[1]);self.assertEqual(self.reply_code(),0)

    def test_paired_d_min_requires_matching_d_ceiling_and_bounds(self):
        self.baseline();self.lib.mode(1,0,1000)
        advanced=bytearray(self.advanced);advanced[39]=41
        self.assertFalse(self.request(95,bytes(advanced))[1])
        pid=bytearray(self.pid);pid[2]=41
        self.assertTrue(self.request(202,bytes(pid))[1]);self.poll();self.lib.response(202,b'',0,0)
        self.assertTrue(self.request(95,bytes(advanced))[1]);self.poll();self.lib.response(95,b'',0,0)
        for index,value in ((39,44),(40,45),(41,1),(51,30)):
            invalid=bytearray(advanced);invalid[index]=value
            self.assertFalse(self.request(95,bytes(invalid))[1])
        self.lib.mode(0,1,2001)
        self.assertEqual(self.poll()[5:-1],self.pid);self.lib.response(202,b'',0,0)
        self.assertEqual(self.poll()[5:-1],self.advanced);self.lib.response(95,b'',0,0)
        self.assertFalse(self.poll())

    def test_boost_changes_are_downward_bounded_and_original_restore_allowed(self):
        self.baseline();self.lib.mode(1,0,1000)
        for value,allowed in ((16,False),(0,False),(10,True),(5,True),(15,True)):
            advanced=bytearray(self.advanced);advanced[52]=value
            self.assertEqual(bool(self.request(95,bytes(advanced))[1]),allowed)
            if allowed:self.poll();self.lib.response(95,b'',0,0)

    def test_automatic_restore_waits_for_receiver_disabled_even_if_fc_disarmed(self):
        self.baseline();pid=bytearray(self.pid);pid[0]=70
        self.request(202,bytes(pid));self.poll();self.lib.response(202,b'',0,0)
        self.lib.mode(1,1,2001);self.assertFalse(self.poll())
        self.lib.mode(0,1,2001);self.assertEqual(self.poll()[5:-1],self.pid)

    def test_capability_advertises_paired_d_min_and_boost(self):
        self.request(0);out=(c.c_ubyte*128)();n=self.lib.reply(out)
        self.assertEqual(bytes(out[32:n]),bytes((1,46,4,5,1,0,1,3)))

    def test_bad_reply_and_timeout_are_not_success(self):
        self.request(112);self.poll();self.lib.response(112,b'bad!',4,0);self.assertEqual(self.reply_code(),5)
        self.request(94);self.poll();self.lib.mode(0,1,1151);self.poll();self.assertEqual(self.reply_code(),3)


if __name__=='__main__':unittest.main()
