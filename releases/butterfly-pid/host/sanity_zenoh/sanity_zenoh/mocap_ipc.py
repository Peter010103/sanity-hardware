"""One bounded, locked latest-state slot per body; readers never wait for a writer."""
import fcntl
import mmap
import os
from pathlib import Path
import pickle
import struct

SIZE = 8192


class LatestStateSlot:
    def __init__(self, path, *, writable=False, create=False):
        flags = os.O_RDWR if writable else os.O_RDONLY
        if create:flags |= os.O_CREAT | os.O_EXCL
        self.fd = os.open(Path(path),flags,0o600)
        if create:os.ftruncate(self.fd,SIZE)
        if os.fstat(self.fd).st_size != SIZE:
            os.close(self.fd);raise ValueError('Invalid mocap slot size')
        self.mapping = mmap.mmap(self.fd,SIZE,access=mmap.ACCESS_WRITE if writable else mmap.ACCESS_READ)
        self.writable = writable
        self.cached = {'ready':False,'error':'Waiting for isolated mocap estimator'}

    def write(self, packet):
        if not self.writable:raise ValueError('Read-only state slot')
        payload = pickle.dumps(packet,protocol=pickle.HIGHEST_PROTOCOL)
        if len(payload)>SIZE-4:raise ValueError('Oversized mocap state')
        try:fcntl.flock(self.fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:return False
        try:
            self.mapping[4:4+len(payload)] = payload
            struct.pack_into('!I',self.mapping,0,len(payload))
        finally:fcntl.flock(self.fd,fcntl.LOCK_UN)
        return True

    def snapshot(self):
        try:fcntl.flock(self.fd,fcntl.LOCK_SH|fcntl.LOCK_NB)
        except BlockingIOError:return dict(self.cached)
        try:
            length = struct.unpack_from('!I',self.mapping,0)[0]
            if length>SIZE-4:raise ValueError('Invalid mocap slot payload size')
            payload = self.mapping[4:4+length] if length else None
        finally:fcntl.flock(self.fd,fcntl.LOCK_UN)
        if payload:
            packet = pickle.loads(payload)
            if not isinstance(packet,dict):raise ValueError('Invalid mocap packet')
            self.cached = packet
        # Re-reading a slot never changes the original source/receipt clocks.
        return dict(self.cached)

    def close(self):
        self.mapping.close();os.close(self.fd)
