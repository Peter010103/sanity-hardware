"""Lifecycle of the separate ROS estimator; no command or Zenoh capability."""
import json
import os
from pathlib import Path
import select
import subprocess
import sys
import threading
import time

from sanity_zenoh.mocap_ipc import LatestStateSlot


class FleetMocapProcess:
    def __init__(self, config, nodes, output):
        self.readers={};self.process=None;self.status_thread=None;self.error=None;self._closing=False
        self.log=(output/'mocap_worker.log').open('w')
        folder=output/'mocap_state';folder.mkdir()
        setup={'topic':config.mocap['topic'],'parent_pid':os.getpid(),'drones':[]}
        try:
            for device,node in nodes.items():
                path=folder/(str(device)+'.state')
                creator=LatestStateSlot(path,writable=True,create=True);creator.close()
                self.readers[device]=LatestStateSlot(path)
                setup['drones'].append({'id':device,'parameters':node.values,'slot':str(path),
                    'log':str(node.output/'tf.jsonl')})
            cfg=output/'mocap_worker_config.json';cfg.write_text(json.dumps(setup,indent=2)+'\n')
            self.process=subprocess.Popen([sys.executable,'-u',str(Path(__file__).with_name('fleet_mocap_process.py')),
                '--config',str(cfg)],stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=self.log)
            deadline=time.monotonic()+30.
            while True:
                remaining=deadline-time.monotonic()
                if remaining<=0 or not select.select([self.process.stdout],[],[],remaining)[0]:
                    raise RuntimeError('Isolated mocap process startup timed out')
                line=self.process.stdout.readline()
                if not line:raise RuntimeError('Isolated mocap process exited; see mocap_worker.log')
                try:status=json.loads(line)
                except ValueError:
                    self.log.write(line.decode(errors='replace'));continue
                if status.get('ready'):
                    self.info=status;break
                if status.get('error'):raise RuntimeError(str(status))
            self.status_thread=threading.Thread(target=self.watch,name='mocap_process_status',daemon=True)
            self.status_thread.start()
        except BaseException:
            self.close();raise

    def watch(self):
        try:
            for line in self.process.stdout:
                try:status=json.loads(line)
                except ValueError:
                    self.log.write(line.decode(errors='replace'));continue
                if status.get('error'):self.error=str(status)
            if not self._closing:self.error=self.error or 'Isolated mocap process exited'
        except Exception as error:
            if not self._closing:self.error=str(error)

    def close(self):
        self._closing=True
        if self.process:
            if self.process.poll() is None:
                self.process.terminate()
                try:self.process.wait(timeout=15.)
                except subprocess.TimeoutExpired:
                    self.process.kill();self.process.wait(timeout=3.)
                    self.error=self.error or 'Isolated mocap shutdown timed out'
            if self.process.returncode not in (0,None):self.error=self.error or f'Isolated mocap exit {self.process.returncode}'
            if self.status_thread:self.status_thread.join(timeout=2.)
            self.process.stdout.close()
        for reader in self.readers.values():reader.close()
        self.readers={};self.log.close()
