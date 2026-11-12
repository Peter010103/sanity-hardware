"""Explicit UDP unicast carrying the existing Sanity packets without a new envelope.

This implements the small session interface used by SanitySender; its ownership,
leases, epoch handling, latest-command slot and STOP acknowledgement are reused.
"""
from collections import Counter
import ipaddress
import re
import socket
import threading
from types import SimpleNamespace


class _Entity:
    def __init__(self, owner, key, callback=None):
        self.owner, self.key, self.callback = owner, key, callback
        self.closed = False
        self.active = 0

    def put(self, payload):
        if self.closed or self.callback is not None:
            raise RuntimeError('Publisher closed or invalid')
        self.owner.send(self.key, bytes(payload))

    def undeclare(self):
        with self.owner.lock:
            if self.closed:
                return
            self.closed = True
            if self.callback is not None:
                self.owner.subscribers.get(self.key, set()).discard(self)
            if threading.current_thread() is not self.owner.thread:
                while self.active:self.owner.idle.wait()


def _address(value):
    host, port = value.rsplit(':', 1)
    ip = ipaddress.IPv4Address(host)
    if ip.is_unspecified or ip.is_multicast or str(ip) == '255.255.255.255' or not 1 <= int(port) <= 65535:
        raise ValueError('Explicit unicast IPv4 address and port required')
    return str(ip), int(port)


def packet_key(raw):
    if len(raw) < 6:
        raise ValueError('Short datagram')
    magic = raw[:4]
    if magic in (b'SZS1', b'SZS2'):
        from sanity_zenoh.protocol import ReceiverStatus
        device = ReceiverStatus.decode(raw).device
        suffix = 'status'
    elif magic in (b'SZT1', b'SZF1', b'SZR1'):
        if raw[4] != 1:
            raise ValueError('Unsupported datagram version')
        device = raw[5]
        suffix = {b'SZT1': 'diagnostics/trace', b'SZF1': 'telemetry/fc', b'SZR1': 'diagnostics/pong'}[magic]
    else:
        raise ValueError('Unsupported receiver datagram')
    if not 1 <= device <= 255:
        raise ValueError('Invalid receiver identity')
    return device, f'sanity/{device:02d}/{suffix}'


class SanityDatagramTransport:
    backend = 'udp-unicast'

    def __init__(self, listen, peers):
        self.lock = threading.RLock()
        self.idle = threading.Condition(self.lock)
        self.devices, self.subscribers = set(), {}
        self.closed = False
        self.peers = {int(device): _address(endpoint) for device, endpoint in peers.items()}
        if len(self.peers) != len(peers):
            raise ValueError("Duplicate normalized device IDs")
        if not self.peers or any(not 1 <= device <= 255 for device in self.peers):
            raise ValueError('Distinct configured device IDs required')
        if len(set(self.peers.values())) != len(self.peers):
            raise ValueError('Receiver endpoints must be unique')
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.setsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF,8192)
            self.socket.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,32768)
            self.socket.bind(_address(listen))
            self.socket.settimeout(.05)
        except BaseException:
            self.socket.close()
            raise
        self.stats = Counter()
        self.session = self
        self.telemetry_session = self
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self._receive, name='sanity_udp_receiver', daemon=True)
        self.thread.start()

    def register(self, device):
        with self.lock:
            if self.closed or device not in self.peers or device in self.devices:
                raise ValueError('Transport closed, unconfigured or duplicate device')
            self.devices.add(device)

    def release(self, device):
        with self.lock:
            self.devices.discard(device)

    def _key(self, key):
        match = re.fullmatch(r'sanity/([0-9]{2,3})/(.+)', key)
        if not match or int(match[1]) not in self.devices:
            raise ValueError('Key must target a registered receiver')
        if match[1] != f"{int(match[1]):02d}":
            raise ValueError("Noncanonical device key")
        return int(match[1]), match[2]

    def declare_subscriber(self, key, callback):
        with self.lock:
            self._key(key)
            entity = _Entity(self, key, callback)
            self.subscribers.setdefault(key, set()).add(entity)
            return entity

    def declare_publisher(self, key, **_options):
        with self.lock:
            self._key(key)
            return _Entity(self, key)

    def send(self, key, raw):
        with self.lock:
            if self.closed:
                raise RuntimeError('Transport closed')
            device, suffix = self._key(key)
            destination = self.peers[device]
        if suffix.startswith('command/'):
            from sanity_zenoh.protocol import COMMAND, CTBR, DIRECT
            if len(raw) != COMMAND.size:
                raise ValueError('Command datagram length')
            header = COMMAND.unpack(raw)
            expected = 'command/ctbr' if header[2] == CTBR else 'command/direct_thrust' if header[2] == DIRECT else ''
            if header[0] not in (b'SZC1', b'SZC2', b'SZC3') or header[3] != device or suffix != expected:
                raise ValueError('Command routing mismatch')
        elif suffix == 'diagnostics/ping':
            if len(raw) != 20 or raw[:5] != b'SZP1\x01' or raw[5] != device:
                raise ValueError('Probe routing mismatch')
        else:
            # Tuning and telemetry-toggle RPCs intentionally stay on the Zenoh path.
            raise ValueError('UDP baseline supports commands and read-only probes only')
        try:
            self.socket.sendto(raw, socket.MSG_DONTWAIT, destination)
            self.stats['tx_packets'] += 1
        except (BlockingIOError, socket.timeout):
            self.stats['tx_drops'] += 1

    def _receive(self):
        while not self.stopping.is_set():
            try:
                raw, address = self.socket.recvfrom(1501)
            except socket.timeout:
                continue
            except OSError:
                if self.stopping.is_set():
                    return
                self.stats['socket_errors'] += 1
                continue
            try:
                if len(raw) > 1400:
                    raise ValueError('Oversize datagram')
                device, key = packet_key(raw)
                with self.lock:
                    if device not in self.devices or address != self.peers.get(device):
                        raise ValueError('Unconfigured source or cross-device packet')
                    callbacks = list(self.subscribers.get(key, ()))
                self.stats['rx_packets'] += 1
                for entity in callbacks:
                    with self.lock:
                        if entity.closed:continue
                        entity.active += 1
                    try:entity.callback(SimpleNamespace(payload=raw))
                    finally:
                        with self.lock:
                            entity.active -= 1
                            self.idle.notify_all()
            except (ValueError, TypeError):
                self.stats['rx_rejected'] += 1
            except Exception:
                self.stats['callback_errors'] += 1

    def close(self):
        with self.lock:
            if self.closed:
                return
            if self.devices:
                raise RuntimeError('Close all drone senders before shared transport')
            self.closed = True
        self.stopping.set()
        self.thread.join(timeout=.5)
        self.socket.close()
