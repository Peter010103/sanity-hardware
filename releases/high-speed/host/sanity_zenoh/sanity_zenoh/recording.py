"""Bounded nonblocking submission; optional process isolation for JSON/disk work."""
import json
from pathlib import Path
import pickle
import queue
import select
import socket
import struct
import subprocess
import sys
import threading
import time


class EventRecorder:
    def __init__(self, path, *, isolated=False, stamp=True, capacity=None):
        self.queue = queue.Queue(maxsize=capacity or (4096 if isolated else 100000))
        self.error = None
        self.count = 0
        self.isolated, self.stamp = isolated, stamp
        self.process = None
        self.closed_count = None
        self._closing = False
        self._closed = False
        self.socket = self.stream = self.status_thread = self.stderr = None
        if isolated:
            parent, child = socket.socketpair()
            self.socket = parent
            self.stderr = Path(str(path)+'.writer.log').open('w')
            try:
                self.process = subprocess.Popen([sys.executable, '-u', str(Path(__file__).with_name('recording_process.py')),
                    '--fd',str(child.fileno()),'--path',str(path)],pass_fds=(child.fileno(),),
                    stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=self.stderr)
                child.close()
                if not select.select([self.process.stdout],[],[],10.)[0]:
                    raise RuntimeError('Recording process startup timed out')
                first = json.loads(self.process.stdout.readline())
                if first.get('ready') is not True:
                    raise RuntimeError(first.get('error','Recording process did not start'))
                self.status_thread = threading.Thread(target=self.watch_status,name='recording_status',daemon=True)
                self.status_thread.start()
            except BaseException:
                child.close(); parent.close()
                if self.process:
                    if self.process.poll() is None:self.process.terminate()
                    self.process.wait(timeout=3.)
                    self.process.stdout.close()
                self.stderr.close()
                raise
        else:
            self.stream = Path(path).open('x',buffering=1)
        self.thread = threading.Thread(target=self.run,name='flight_event_writer',daemon=True)
        self.thread.start()

    def __call__(self, event):
        if self.error or self._closing:
            return
        try:
            row = {'host_ns':time.monotonic_ns(),'unix_ns':time.time_ns(),**event} if self.stamp else event
            self.queue.put_nowait(row)
        except queue.Full:
            self.error = 'Recording queue full'

    def watch_status(self):
        try:
            for line in self.process.stdout:
                status = json.loads(line)
                if status.get('error'):self.error = status['error']
                if status.get('closed'):self.closed_count = status['count']
            if not self._closing and self.error is None:
                self.error = 'Recording process exited before close'
        except Exception as error:
            self.error = str(error)

    def run(self):
        try:
            while (event := self.queue.get()) is not None:
                if self.isolated:
                    payload = pickle.dumps(event, protocol=pickle.HIGHEST_PROTOCOL)
                    self.socket.sendall(struct.pack('!I',len(payload))+payload)
                else:
                    self.stream.write(json.dumps(event,allow_nan=False)+'\n')
                self.count += 1
            if self.isolated:self.socket.sendall(struct.pack('!I',0))
        except Exception as error:
            self.error = str(error)

    def close(self):
        if self._closed:return
        self._closing = True
        try:
            if self.thread.is_alive():
                try:self.queue.put(None,timeout=2.)
                except queue.Full:self.error = self.error or 'Recording close queue full'
                self.thread.join(timeout=5.)
            if self.thread.is_alive():
                self.error = self.error or 'Recording drain timed out'
                if self.socket:
                    try:self.socket.shutdown(socket.SHUT_RDWR)
                    except OSError:pass
                self.thread.join(timeout=1.)
            if self.isolated:
                try:self.process.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    self.error = self.error or 'Recording process close timed out'
                    self.process.terminate()
                    try:self.process.wait(timeout=3.)
                    except subprocess.TimeoutExpired:
                        self.process.kill();self.process.wait(timeout=3.)
                self.status_thread.join(timeout=2.)
                if self.process.returncode != 0 or self.closed_count != self.count:
                    self.error = self.error or 'Recording process did not acknowledge every row'
        finally:
            if self.socket:self.socket.close()
            if self.process:self.process.stdout.close()
            if self.stderr:self.stderr.close()
            if self.stream:self.stream.close()
            self._closed = True
