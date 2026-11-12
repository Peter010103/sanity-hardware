"""Real pinned-Pico reconnect regression, restricted to loopback."""
import json
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import time
import unittest
import zenoh


@unittest.skipUnless(os.environ.get('SANITY_SESSION_WATCH_EMULATOR'),'Build/set session_watch_emulator with connectivity enabled')
class SessionWatchTests(unittest.TestCase):
    def test_four_host_sessions_reconnect_without_restarting_receiver(self):
        self.check_host_restarts('tcp')


    def test_four_udp_control_host_sessions_reconnect_without_restarting_receiver(self):
        self.check_host_restarts('udp')


    def test_partial_initialization_cleanup(self):
        for protocol in ('tcp','udp'):
            with socket.socket(socket.AF_INET,socket.SOCK_DGRAM if protocol=='udp' else socket.SOCK_STREAM) as sock:
                sock.bind(('127.0.0.1',0));endpoint=f'{protocol}/127.0.0.1:{sock.getsockname()[1]}'
            config=zenoh.Config();config.insert_json5('mode','"peer"');config.insert_json5('listen/endpoints',json.dumps([endpoint]));config.insert_json5('scouting/multicast/enabled','false');config.insert_json5('scouting/gossip/enabled','false')
            with zenoh.open(config) as session:
                sub=session.declare_subscriber('restart/**',lambda sample:None)
                try:
                    for stage in range(5):
                        with self.subTest(protocol=protocol,stage=stage):
                            result=subprocess.run([os.environ['SANITY_SESSION_WATCH_EMULATOR'],endpoint,str(stage)],capture_output=True,text=True,timeout=20)
                            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
                            self.assertIn('"event":"partial_closed"',result.stdout)
                finally:sub.undeclare()


    def check_host_restarts(self, protocol):
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM if protocol=='udp' else socket.SOCK_STREAM) as sock:
            sock.bind(('127.0.0.1',0));endpoint=f'{protocol}/127.0.0.1:{sock.getsockname()[1]}'
        peer=None;session=None;sub=None;trace_sub=None;counts=[]
        with tempfile.TemporaryDirectory() as folder:
            log=Path(folder)/'peer.jsonl'
            with log.open('w') as stream:
                try:
                    for cycle in range(4):
                        config=zenoh.Config();config.insert_json5('mode','"peer"');config.insert_json5('listen/endpoints',json.dumps([endpoint]));config.insert_json5('connect/endpoints','[]');config.insert_json5('scouting/multicast/enabled','false');config.insert_json5('scouting/gossip/enabled','false')
                        session=zenoh.open(config);received=[]
                        sub=session.declare_subscriber('restart/probe',lambda sample:received.append(bytes(sample.payload)))
                        trace_sub=session.declare_subscriber('restart/trace',lambda sample:None)
                        if peer is None:peer=subprocess.Popen([os.environ['SANITY_SESSION_WATCH_EMULATOR'],endpoint],stdout=stream,stderr=subprocess.STDOUT)
                        deadline=time.monotonic()+15.
                        while len(received)<5 and time.monotonic()<deadline:
                            self.assertIsNone(peer.poll(),'Receiver must remain running');session.put('restart/input',b'callback');time.sleep(.02)
                        self.assertGreaterEqual(len(received),5,f'No telemetry after host restart {cycle}')
                        counts.append(len(received));sub.undeclare();sub=None;trace_sub.undeclare();trace_sub=None;session.close();session=None
                    text=log.read_text();opened=[json.loads(line) for line in text.splitlines() if line.startswith('{') and json.loads(line).get('event')=='opened']
                    self.assertGreaterEqual(len(opened),4,text)
                    self.assertIn('"session_closed":false',text)
                    closed=[json.loads(line) for line in text.splitlines() if line.startswith('{') and json.loads(line).get('event')=='closed']
                    self.assertTrue(any(row['callbacks']>0 for row in closed),text)
                finally:
                    if trace_sub:trace_sub.undeclare()
                    if sub:sub.undeclare()
                    if session:session.close()
                    if peer and peer.poll() is None:peer.terminate();peer.wait(timeout=5)


if __name__=='__main__':unittest.main()
