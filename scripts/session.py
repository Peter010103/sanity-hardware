"""Track and clean only child processes owned by one invocation (Linux)."""
import ctypes
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time


def process_rows():
    rows={}
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():continue
        try:
            stat=(path/'stat').read_text();parts=stat[stat.rfind(')')+2:].split()
            rows[int(path.name)]=dict(pid=int(path.name),ppid=int(parts[1]),start_ticks=int(parts[19]),
                command=(path/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace').strip())
        except (OSError,ValueError,IndexError):pass
    return rows


def inventory():
    import re
    pattern=re.compile(r'ros2|rosbag|zenoh|natnet|mocap|pose.*bridge|sanity.*(?:hover|mpc|controller)|(?:sensor|preview).*server',re.I)
    return dict(host=socket.gethostname(),created_unix=time.time(),processes=[row for row in process_rows().values() if row['pid']!=os.getpid() and pattern.search(row['command'])],scope='Read-only. Pre-existing processes are preserved; identify ownership before stopping them.')


class OwnedSession:
    def __init__(self,path):
        self.path=Path(path);self.owned={};self.pidfds={};self.child=None;self.interrupts=0
        self.previous={sig:signal.signal(sig,self.interrupt) for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP)}
        # Reparent orphaned descendants here so new process groups remain owned/reapable.
        self.libc=ctypes.CDLL(None,use_errno=True)
        if self.libc.prctl(36,1,0,0,0)!=0:raise OSError(ctypes.get_errno(),'Cannot enable child subreaper')

    def interrupt(self,number,frame):
        self.interrupts+=1
        if self.child is not None and self.child.poll() is None:
            self.child.send_signal(signal.SIGINT if self.interrupts==1 and number==signal.SIGINT else signal.SIGTERM)
        else:raise KeyboardInterrupt

    def update(self):
        rows=process_rows();parents={os.getpid(),*(pid for pid,row in self.owned.items() if pid in rows and rows[pid]['start_ticks']==row['start_ticks'])}
        while True:
            additions={pid:row for pid,row in rows.items() if row['ppid'] in parents and pid not in parents}
            if not additions:break
            for pid in additions:
                try:self.pidfds[pid]=os.pidfd_open(pid)
                except ProcessLookupError:pass
            self.owned.update(additions);parents.update(additions)
        for pid,row in self.owned.items():
            current=rows.get(pid);row['exited']=current is None or current['start_ticks']!=row['start_ticks']
        self.path.write_text(json.dumps(dict(host=socket.gethostname(),launcher_pid=os.getpid(),processes=list(self.owned.values())),indent=2)+'\n')
        return [pid for pid,row in self.owned.items() if not row['exited']]

    def run(self,command,*,cwd,env):
        self.child=subprocess.Popen(command,cwd=cwd,env=env,start_new_session=True)
        while self.child.poll() is None:
            self.update();time.sleep(.1)
        self.update()
        return self.child.returncode

    def close(self):
        # These PIDs were observed as descendants, and identities are rechecked before signals.
        for sig,seconds in ((signal.SIGINT,5.),(signal.SIGTERM,3.),(signal.SIGKILL,2.)):
            active=self.update()
            if not active:break
            for pid in active:
                try:
                    if pid in self.pidfds:signal.pidfd_send_signal(self.pidfds[pid],sig)
                except ProcessLookupError:pass
            deadline=time.monotonic()+seconds
            while time.monotonic()<deadline:
                try:
                    while os.waitpid(-1,os.WNOHANG)[0]:pass
                except ChildProcessError:pass
                if not self.update():break
                time.sleep(.05)
        remaining=self.update()
        for sig,handler in self.previous.items():signal.signal(sig,handler)
        for fd in self.pidfds.values():os.close(fd)
        self.libc.prctl(36,0,0,0,0)
        if remaining:raise RuntimeError('Owned processes did not exit: '+str(remaining))
