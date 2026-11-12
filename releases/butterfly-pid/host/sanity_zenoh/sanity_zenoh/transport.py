import json
import threading

import zenoh


class SanityTransport:
    def __init__(self, listen='tcp/0.0.0.0:7447', connect=None,
                 telemetry_listen=None, telemetry_connect=None):
        self.lock = threading.Lock()
        self.devices = set()
        self.closed = False
        self.session = self.open_session(listen, connect)
        self.telemetry_session = None
        try:
            if telemetry_listen or telemetry_connect:
                self.telemetry_session = self.open_session(telemetry_listen, telemetry_connect)
        except BaseException:
            self.session.close()
            raise


    @staticmethod
    def open_session(listen, connect):
        config = zenoh.Config()
        config.insert_json5('mode', json.dumps('client' if connect else 'peer'))
        config.insert_json5('scouting/multicast/enabled', 'false')
        config.insert_json5('scouting/gossip/enabled', 'false')
        endpoints=list(listen) if isinstance(listen,(list,tuple)) else [listen]
        config.insert_json5('listen/endpoints', json.dumps([] if connect else endpoints))
        config.insert_json5('connect/endpoints', json.dumps([connect] if connect else []))
        return zenoh.open(config)


    def register(self, device):
        with self.lock:
            if self.closed or device in self.devices:
                raise ValueError('Transport closed or duplicate device registration')
            self.devices.add(device)


    def release(self, device):
        with self.lock:
            self.devices.discard(device)


    def close(self):
        with self.lock:
            if self.closed:
                return
            if self.devices:
                raise RuntimeError('Close all drone senders before the shared transport')
            self.closed = True
        if self.telemetry_session is not None:
            self.telemetry_session.close()
        self.session.close()
