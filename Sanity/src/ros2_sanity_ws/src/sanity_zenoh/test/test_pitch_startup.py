from types import SimpleNamespace
import unittest
from unittest.mock import patch

from sanity_zenoh.pitch_mission import wait_for_pitch_receiver
from sanity_zenoh.protocol import ReceiverStatus


class PitchStartupTests(unittest.TestCase):
    def setUp(self):
        self.now=0.;self.sent=[]
        self.status=ReceiverStatus(mode=1,device=1,flags=52,epoch=1,receiver_ms=100,session=17,sequence=0,rejected=0,voltage_mv=8000,fc_age_ms=1)
        self.sender=SimpleNamespace(session_id=17,ctbr=lambda *a,**k:self.sent.append((a,k)),receiver_status=lambda:self.status)
        self.clock=SimpleNamespace(monotonic=lambda:self.now,sleep=self.advance)

    def advance(self,seconds):
        self.now+=seconds

    def test_ack_after_old_ten_second_limit_is_accepted_disarmed(self):
        self.sender.receiver_status=lambda:self.status if self.now>=10.05 else None
        events=[]
        with patch('sanity_zenoh.pitch_mission.time',self.clock):
            status=wait_for_pitch_receiver(self.sender,1.,on_event=events.append)
        self.assertIs(status,self.status);self.assertGreater(self.now,10.);self.assertLess(self.now,30.)
        self.assertTrue(all(a==(False,0.,(0.,0.,0.)) and k=={'valid_for_s':1.} for a,k in self.sent))
        self.assertEqual(events[-1]['kind'],'startup_receiver_ready')

    def test_missing_udp_status_has_specific_timeout(self):
        self.sender.receiver_status=lambda:None
        with patch('sanity_zenoh.pitch_mission.time',self.clock),self.assertRaisesRegex(ValueError,'no fresh UDP command/status'):
            wait_for_pitch_receiver(self.sender,1.,timeout_s=.03)
        self.assertTrue(all(not a[0] for a,_ in self.sent))

    def test_wrong_session_enabled_or_stale_fc_cannot_pass(self):
        for changes,reason in [({'session':18},'sender session'),({'flags':53},'disabled output'),({'fc_age_ms':200},'UART4 status stale')]:
            self.now=0.;self.status=ReceiverStatus(**{**vars(self.status),**changes})
            with self.subTest(changes=changes),patch('sanity_zenoh.pitch_mission.time',self.clock),self.assertRaisesRegex(ValueError,reason):
                wait_for_pitch_receiver(self.sender,1.,timeout_s=.02)
            self.status=ReceiverStatus(mode=1,device=1,flags=52,epoch=1,receiver_ms=100,session=17,sequence=0,rejected=0,voltage_mv=8000,fc_age_ms=1)


if __name__=='__main__':unittest.main()
