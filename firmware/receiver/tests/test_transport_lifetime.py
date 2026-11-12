"""The real Pico publication path must share teardown's lifetime lock."""
import json,os,socket,subprocess,unittest
import zenoh

@unittest.skipUnless(os.environ.get('SANITY_TRANSPORT_LIFETIME_EMULATOR'),'Build the native lifetime regression executable')
class TransportLifetimeTests(unittest.TestCase):
    def test_drop_does_not_wait_or_touch_transport_while_teardown_holds_lock(self):
        for protocol,kind in [('tcp',socket.SOCK_STREAM),('udp',socket.SOCK_DGRAM)]:
            with self.subTest(protocol=protocol):
                with socket.socket(socket.AF_INET,kind) as sock:sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
                endpoint=f'{protocol}/127.0.0.1:{port}';config=zenoh.Config();config.insert_json5('mode','"peer"');config.insert_json5('listen/endpoints',json.dumps([endpoint]));config.insert_json5('scouting/multicast/enabled','false');config.insert_json5('scouting/gossip/enabled','false')
                with zenoh.open(config) as session:
                    received=[];subscriber=session.declare_subscriber('guard/probe',lambda sample:received.append(bytes(sample.payload)))
                    try:
                        result=subprocess.run([os.environ['SANITY_TRANSPORT_LIFETIME_EMULATOR'],endpoint],capture_output=True,text=True,timeout=12)
                        self.assertEqual(result.returncode,0,result.stdout+result.stderr);self.assertIn('"guarded_drop":true',result.stdout);self.assertNotIn(b'blocked',received);self.assertIn(b'after',received)
                    finally:subscriber.undeclare()

if __name__=='__main__':unittest.main()
