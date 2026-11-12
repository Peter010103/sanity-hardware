"""Bounded recording alongside host-controlled hover; never supplies control state.

Callbacks copy bundles. A writer saves original records and decodes only the
latest state for startup/logging checks. Full CRC/semantic validation is offline.
"""
import json
import queue
import struct
import threading
import time
from pathlib import Path
from .estimator_protocol import decode_record


class HoverObservations:
    def __init__(self, path, metadata, capacity=256):
        self.queue = queue.Queue(maxsize=capacity)
        self.lock = threading.Lock()
        self.stopping = threading.Event()
        self.dropped = 0
        self.error = None
        self.records = 0
        self.latest = None
        self.latest_ns = 0
        self.file = Path(path).open('xb')
        header = json.dumps(metadata, sort_keys=True).encode()
        self.file.write(b'SELOG1\r\n' + struct.pack('<I', len(header)) + header)
        self.thread = threading.Thread(target=self._write, name='hover_observations', daemon=True)
        self.thread.start()

    def receive(self, sample):
        self.put(bytes(sample.payload), time.monotonic_ns())

    def put(self, raw, host_ns):
        if self.stopping.is_set():
            return
        if not 28 <= len(raw) <= 4096:
            with self.lock: self.error = 'Invalid observation bundle length'
            return
        try:
            self.queue.put_nowait((bytes(raw), host_ns))
        except queue.Full:
            with self.lock: self.dropped += 1

    def _write(self):
        try:
            while not self.stopping.is_set() or not self.queue.empty():
                try:
                    bundle, stamp = self.queue.get(timeout=.05)
                except queue.Empty:
                    continue
                try:
                    cursor = count = 0
                    state = None
                    while cursor < len(bundle):
                        if count >= 16 or len(bundle)-cursor < 28 or bundle[cursor:cursor+5] != b'SE01\x01':
                            raise ValueError('Malformed observation bundle')
                        size = struct.unpack_from('<H', bundle, cursor+6)[0]
                        if not 28 <= size <= 256 or cursor+size > len(bundle):
                            raise ValueError('Truncated observation record')
                        raw = bundle[cursor:cursor+size]
                        self.file.write(struct.pack('<IQ', size, stamp) + raw)
                        if raw[5] == 3: state = raw
                        count += 1
                        cursor += size
                    latest = decode_record(state) if state is not None else None
                    with self.lock:
                        self.records += count
                        if latest is not None:
                            self.latest, self.latest_ns = latest, stamp
                finally:
                    self.queue.task_done()
        except Exception as error:
            with self.lock: self.error = str(error)

    def snapshot(self, now_ns=None, expected_epoch=None):
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        with self.lock:
            state = self.latest
            age = (now_ns-self.latest_ns)/1e9 if state is not None else None
            recent = age is not None and 0 <= age < .25
            # Logging admission, not a replacement for the ESKF's 40 ms gate.
            # The healthy bit and effective state age remain recorded as-is.
            initialized = bool(state and state['flags'] & 1 and state['flags'] & 4 and state['fused'] > 0
                               and (expected_epoch is None or state['epoch']==expected_epoch))
            return dict(records=self.records, dropped=self.dropped, error=self.error,
                        state_record_age_s=age, initialized_and_clock_valid=initialized,
                        ready=recent and initialized and not self.dropped and not self.error,
                        latest=state)

    def close(self):
        self.stopping.set()
        self.thread.join(timeout=3.)
        if self.thread.is_alive():
            with self.lock: self.error = self.error or 'Observation writer did not stop'
        else:
            self.file.close()
