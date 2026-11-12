import argparse
from concurrent.futures import ThreadPoolExecutor
import gc
import json
import os
from pathlib import Path
import select
import subprocess
import sys
import time
import threading

import numpy as np


def pin_cpu_affinity(cpus):
    if cpus:
        if not isinstance(cpus,list) or any(type(cpu) is not int or cpu<0 for cpu in cpus) or any(cpu>=os.cpu_count() for cpu in cpus):
            raise ValueError('CPU affinity must select available logical CPU numbers')
        for thread in Path('/proc/self/task').iterdir():
            try:os.sched_setaffinity(int(thread.name),set(cpus))
            except ProcessLookupError:pass
        if os.sched_getaffinity(0)!=set(cpus):raise ValueError('Requested CPU affinity is restricted by the host')
    return sorted(os.sched_getaffinity(0))


def move_background_threads(cpus, preserve=()):
    if not cpus:return
    if not isinstance(cpus,list) or any(type(cpu) is not int or not 0<=cpu<os.cpu_count() for cpu in cpus):
        raise ValueError('Background affinity must select available logical CPU numbers')
    current=threading.get_native_id()
    for entry in Path('/proc/self/task').iterdir():
        tid=int(entry.name)
        if tid==current or tid in preserve:continue
        try:os.sched_setaffinity(tid,set(cpus))
        except ProcessLookupError:pass


def next_control_slot(now_ns, deadline_ns):
    return int(deadline_ns+max(0,(now_ns-deadline_ns)//20_000_000+1)*20_000_000)


class CableWorker:
    def __init__(self, python, experiment, config, output, cpu_affinity=None, response_cpu_affinity=None):
        self.log = (output/'solver.log').open('w')
        self.process = subprocess.Popen([str(python),'-u',str(Path(__file__).resolve()),'--experiment',str(experiment),'--config',str(config),'--cpu-affinity',*(str(cpu) for cpu in (cpu_affinity or []))],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.log,env={**os.environ,'OMP_NUM_THREADS':'1','OPENBLAS_NUM_THREADS':'1','MKL_NUM_THREADS':'1'})
        self.buffer = b''
        try:
            self.info = self.read(600.)
            if 'error' in self.info:
                raise RuntimeError(self.info['error'])
        except BaseException:
            self.process.terminate();self.process.wait(timeout=5);self.log.close()
            raise
        self.outstanding = None
        self.executor = ThreadPoolExecutor(max_workers=1,thread_name_prefix='cable_mpc',initializer=os.sched_setaffinity if response_cpu_affinity else None,initargs=(0,set(response_cpu_affinity)) if response_cpu_affinity else ())


    def read(self, timeout):
        deadline = time.monotonic()+timeout
        while b'\n' not in self.buffer:
            remaining = deadline-time.monotonic()
            if remaining<=0 or not select.select([self.process.stdout],[],[],remaining)[0]:
                raise TimeoutError('Cable solver process timeout')
            chunk = os.read(self.process.stdout.fileno(),65536)
            if not chunk:
                raise RuntimeError('Cable solver process exited; see solver.log')
            self.buffer += chunk
            if len(self.buffer)>1_000_000:
                raise ValueError('Oversized cable solver reply')
        line,self.buffer = self.buffer.split(b'\n',1)
        return json.loads(line)


    def send(self, state, preview, delay_s, previous_command, cable_length_m=None, payload_mass_kg=None):
        started = time.perf_counter()
        request = {'state':np.asarray(state).tolist(),'preview':np.asarray(preview).tolist(),'test_delay_s':delay_s,'previous_command':None if previous_command is None else np.asarray(previous_command).tolist()}
        if cable_length_m is not None:request['cable_length_m']=float(cable_length_m)
        if payload_mass_kg is not None:request['payload_mass_kg']=float(payload_mass_kg)
        self.process.stdin.write((json.dumps(request,allow_nan=False)+'\n').encode());self.process.stdin.flush()
        return started


    def read_result(self, started):
        result = self.read(2.)
        result['completed_wall_s'] = time.perf_counter()
        result['roundtrip_ms'] = (result['completed_wall_s']-started)*1000
        result['request_dispatch_ms'] = (result.get('worker_started_wall_s',started)-started)*1000
        result['reply_dispatch_ms'] = (result['completed_wall_s']-result.get('worker_finished_wall_s',result['completed_wall_s']))*1000
        return result


    def infer(self, state, preview, delay_s=0., previous_command=None, cable_length_m=None, payload_mass_kg=None):
        return self.read_result(self.send(state,preview,delay_s,previous_command,cable_length_m,payload_mass_kg))


    def submit(self, state, preview, delay_s=0., previous_command=None, cable_length_m=None, payload_mass_kg=None):
        if self.outstanding is not None and not self.outstanding.done():
            raise RuntimeError('Only one cable MPC request may be outstanding')
        started=self.send(state,preview,delay_s,previous_command,cable_length_m,payload_mass_kg)
        self.outstanding=self.executor.submit(self.read_result,started)
        return self.outstanding


    def close(self):
        self.executor.shutdown(wait=False,cancel_futures=True)
        if self.process.poll() is None:
            self.process.terminate()
            try:self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait(timeout=3)
        self.process.stdin.close();self.process.stdout.close();self.log.close()


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description='Isolated cable MPC, finite JSON state/preview protocol.')
    parser.add_argument('--experiment',type=Path,required=True)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--cpu-affinity',type=int,nargs='*',default=[])
    args=parser.parse_args()
    affinity=pin_cpu_affinity(args.cpu_affinity)
    protocol=os.fdopen(os.dup(sys.stdout.fileno()),'w',buffering=1)
    os.dup2(sys.stderr.fileno(),sys.stdout.fileno())
    sys.path.insert(0,str(args.experiment.resolve().parent))
    from sanity_cable_mpc.controller import CableMPC
    cfg=json.loads(args.config.read_text())
    simulation=bool(cfg.get('simulation',False))
    controller=CableMPC({**cfg,'simulation':False})
    state,command=controller.model.equilibrium()
    for _ in range(6):controller.command(state,np.tile(np.r_[state,command],(controller.nodes+1,1)))
    controller.info['cpu_affinity']=affinity
    controller.previous=None
    gc.collect();gc.disable()
    protocol.write(json.dumps(controller.info,allow_nan=False)+'\n')
    for line in sys.stdin:
        request=json.loads(line)
        worker_started=time.perf_counter();cpu_started=time.process_time()
        try:
            delay=float(request.get('test_delay_s',0.))
            if delay:
                if not simulation or not 0<=delay<=.25:
                    raise ValueError('Fault injection is simulation-only')
                time.sleep(delay)
            if cfg.get('runtime_cable_length',False):
                length=float(request['cable_length_m'])
                mass=float(request['payload_mass_kg'])
                if length!=controller.model.length or mass!=controller.model.mp:
                    controller.model.set_cable_parameters(length,mass);controller.previous=None
            command=controller.command(request['state'],request['preview'],previous_command=request.get('previous_command'))
            controller.last['cable_length_m']=float(controller.model.length)
            controller.last['payload_mass_kg']=float(controller.model.mp)
            result={'command':command.tolist(),'mpc':controller.last}
        except (ValueError,RuntimeError,KeyError) as error:
            result={'error':str(error),'mpc':controller.last}
        if 'cpu_started' in locals():
            result['mpc']['worker_cpu_ms']=(time.process_time()-cpu_started)*1000
            result['mpc']['linearization_ms']=1000*float(controller.solver.get_stats('time_lin'))
            result['mpc']['qp_ms']=1000*float(controller.solver.get_stats('time_qp'))
            result['mpc']['qp_iterations']=np.asarray(controller.solver.get_stats('qp_iter')).ravel().tolist()
        result['worker_started_wall_s']=worker_started
        result['worker_finished_wall_s']=time.perf_counter()
        protocol.write(json.dumps(result,allow_nan=False)+'\n')
