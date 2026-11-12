from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import select
import subprocess
import time
import numpy as np


class PlateWorker:
    def __init__(self, config, output, python):
        self.log=(output/'solver.log').open('w');self.buffer=b'';self.sequence=0
        aircraft=output/'aircraft_models.json';aircraft.write_text(json.dumps(config.models,indent=2))
        geometry=output/'controller_geometry.json';geometry.write_text(json.dumps(config.physical.dictionary(),indent=2))
        environment={**os.environ,'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','MKL_NUM_THREADS':'1',
            'PYTHONPATH':str(config.workspace/'loco_mjx')+os.pathsep+os.environ.get('PYTHONPATH','')}
        self.process=subprocess.Popen([str(python),'-u','-m','experiments.sanity_plate_transport.flight_worker',
            '--geometry',str(geometry),'--aircraft',str(aircraft),'--cache',str(config.workspace/'.cache/sanity_plate_flight')],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.log,env=environment)
        reader=config.affinity.get('reader',[])
        self.pool=ThreadPoolExecutor(max_workers=1,initializer=os.sched_setaffinity if reader else None,initargs=(0,set(reader)) if reader else ());self.pending=None
        try:
            if config.affinity.get('solver'):os.sched_setaffinity(self.process.pid,set(config.affinity['solver']))
            self.info=self.read(600.)
            if not self.info.get('ready'):raise RuntimeError('Plate worker failed startup: '+str(self.info))
        except BaseException:self.close();raise


    def read(self, timeout):
        deadline=time.monotonic()+timeout
        while b'\n' not in self.buffer:
            remaining=deadline-time.monotonic()
            if remaining<=0 or not select.select([self.process.stdout],[],[],remaining)[0]:raise TimeoutError('Plate solver response timeout')
            chunk=os.read(self.process.stdout.fileno(),65536)
            if not chunk:raise RuntimeError('Plate solver exited; inspect solver.log')
            self.buffer+=chunk
            if len(self.buffer)>1000000:raise ValueError('Oversized plate solver response')
        line,self.buffer=self.buffer.split(b'\n',1);result=json.loads(line)
        result['host_received_s']=time.monotonic();return result


    def submit(self, packet):
        if self.pending is not None and not self.pending.done():raise RuntimeError('Only one plate MPC request may be outstanding')
        self.sequence+=1;packet=dict(packet,sequence=self.sequence)
        self.process.stdin.write((json.dumps(packet,allow_nan=False)+'\n').encode());self.process.stdin.flush()
        self.pending=self.pool.submit(self.read,2.)
        return self.pending,self.sequence


    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait()
        self.pool.shutdown(wait=True,cancel_futures=True);self.log.close()


def admit_solution(result, sequence, latency_s, selection_s, now_s, model, budget_s):
    if result.get('sequence')!=sequence:return None,None,'wrong_sequence'
    if 'error' in result:return None,str(result['error']),'solver_error'
    if not np.isfinite(latency_s) or not 0<=latency_s<=budget_s:return None,None,'deadline'
    if not -.003<=now_s-selection_s<=.006:return None,None,'selection_slot'
    command=np.asarray(result.get('command'),dtype=float)
    if result.get('command_space')=='world_force':
        if command.shape!=(3*model.config.drones,) or not np.isfinite(command).all() or not result.get('mpc',{}).get('accepted'):
            return None,'Malformed plate world-force command','invalid_command'
        forces=command.reshape(-1,3)
        if np.any(forces[:,2]<.05-1e-6) or np.max(np.linalg.norm(forces,axis=1))>model.config.maximum_force_N+.003:
            return None,'Plate world-force command outside limits','invalid_command'
        return command.copy(),None,'accepted'
    if command.shape!=(model.nu,) or not np.isfinite(command).all() or not result.get('mpc',{}).get('accepted') or np.any(command<model.input_lower-1e-6) or np.any(command>model.input_upper+1e-6):
        return None,'Malformed or unaccepted plate MPC command','invalid_command'
    return command.copy(),None,'accepted'
