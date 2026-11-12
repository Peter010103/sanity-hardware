"""Independent bounded UDP/health/shadow logging task, with no flight actions."""
import hashlib
import importlib.util
import json
from pathlib import Path
import queue
import socket
import sys
import threading
import time

from sanity_zenoh.cable_engagement import EngagementEstimator, configuration
from sanity_zenoh.recording import EventRecorder


def load_helpers(directory):
    """Use the prepared health helper and decoder without copying their logic."""
    directory=Path(directory).resolve()
    loaded=[]
    for name,file in [('hx711_timing','hx711_timing.py'),('_sanity_strain_decoder','calibrate_onboard_tension.py')]:
        path=directory/file
        old=sys.modules.get(name)
        if old is not None and Path(old.__file__).resolve()!=path:raise ValueError('Conflicting strain helper import: '+name)
        if old is None:
            spec=importlib.util.spec_from_file_location(name,path)
            module=importlib.util.module_from_spec(spec);sys.modules[name]=module
            try:spec.loader.exec_module(module)
            except BaseException:
                sys.modules.pop(name,None);raise
        else:module=old
        loaded.append(module)
    return loaded[0].HX711Health,loaded[1].decode_packet


class CableStrainShadow:
    """Receiver owns mutable estimator state; flight only reads latest snapshot.

    Context provider must return an independent raw-geometry snapshot. No
    reference/controller/sender object is accepted by this class.
    """
    def __init__(self, settings, helper_directory, output, context_provider):
        self.cfg=configuration(settings)
        if self.cfg['mode']!='shadow':raise ValueError('Shadow task requires mode=shadow')
        health_type,self.decode=load_helpers(helper_directory)
        health=health_type(**{k:self.cfg[k] for k in ('max_source_gap_s','max_receive_age_s','max_queue_age_s')})
        self.estimator=EngagementEstimator(health,self.cfg)
        self.context_provider=context_provider
        self.stop_event=threading.Event();self.requests=queue.Queue(maxsize=1)
        self.socket=None;self.thread=None;self.error=None
        self.latest={'mode':'shadow','state':'UNKNOWN','reasons':['waiting_for_shadow_task'],'control_affected':False}
        self.raw_log=EventRecorder(Path(output)/'strain_raw.jsonl',capacity=4096)
        try:self.decision_log=EventRecorder(Path(output)/'cable_engagement.jsonl',capacity=4096)
        except BaseException:
            self.raw_log.close();raise
        self.helpers={str(Path(helper_directory).resolve()/name):hashlib.sha256((Path(helper_directory)/name).read_bytes()).hexdigest() for name in ('hx711_timing.py','calibrate_onboard_tension.py')}
        (Path(output)/'cable_engagement_config.json').write_text(json.dumps(dict(settings=self.cfg,helper_sha256=self.helpers,scope='Shadow only; no flight gating; thresholds and timing bounds unvalidated'),indent=2)+'\n')

    def start(self):
        self.thread=threading.Thread(target=self.run,name='sanity_strain_shadow',daemon=True)
        self.thread.start()

    def confirm_unloaded(self):
        """Explicit operator request, processed asynchronously by receiver task."""
        if self.stop_event.is_set() or self.error:return False,'Shadow task unavailable'
        if not self.latest.get('health',{}).get('health_ok',False):return False,'Healthy live strain stream required'
        try:self.requests.put_nowait(time.monotonic_ns())
        except queue.Full:return False,'An unloaded confirmation is already pending'
        return True,'Queued confirmation: support/remove ALL sensor loads, maintain unloading through baseline capture; inspect cable_engagement.jsonl for completion'

    def run(self):
        next_check=0
        try:
            self.socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            # Exclusive ownership: never share a unicast stream with a recorder.
            self.socket.bind((self.cfg['bind'],self.cfg['port']));self.socket.settimeout(.02)
            while not self.stop_event.is_set():
                now=time.monotonic_ns()
                try:
                    confirmation=self.requests.get_nowait()
                except queue.Empty:confirmation=None
                if confirmation is not None:
                    ok,message=self.estimator.confirm_unloaded(now)
                    self.decision_log(dict(kind='unloaded_confirmation',host_monotonic_ns=now,request_ns=confirmation,accepted=ok,message=message))
                try:
                    data,address=self.socket.recvfrom(8193)
                except socket.timeout:data=None
                if data is not None:
                    arrival=time.monotonic_ns();record=dict(kind='strain_packet',host_monotonic_ns=arrival,source_ip=address[0],datagram_utf8=data.decode('utf8',errors='replace'),accepted=False)
                    try:
                        if address[0]!=self.cfg['source_ip']:raise ValueError('unexpected_source_ip')
                        if len(data)>8192:raise ValueError('oversized_packet')
                        packet=self.decode(data,self.cfg['device_id'])
                        health=self.estimator.observe(packet,arrival)
                        record.update(accepted=True,health=health)
                    except (ValueError,TypeError,KeyError,UnicodeError) as error:
                        self.estimator.invalidate('decoder_or_source_error: '+str(error));record['reason']=str(error)
                    self.raw_log(record)
                now=time.monotonic_ns()
                # Runs on wall time even without UDP, ROS callbacks, or inference.
                if now>=next_check:
                    if self.raw_log.error or self.decision_log.error:self.estimator.invalidate('shadow_recording_error')
                    try:
                        geometry,phase,path_clear=self.context_provider()
                    except Exception as error:
                        geometry={'valid':False,'reasons':['shadow_context_error: '+str(error)]};phase='unknown';path_clear=False
                    diagnostic=self.estimator.evaluate(now,geometry,phase,path_clear)
                    diagnostic.update(kind='shadow_decision',host_monotonic_ns=now,recording_errors=dict(raw=self.raw_log.error,decisions=self.decision_log.error))
                    self.latest=diagnostic
                    self.decision_log(diagnostic)
                    next_check=now+50_000_000
        except Exception as error:
            self.error=type(error).__name__+': '+str(error)
            self.estimator.invalidate('shadow_task_failed: '+self.error)
            self.latest={'mode':'shadow','state':'UNKNOWN','reasons':['shadow_task_failed: '+self.error],'control_affected':False}
            self.decision_log(dict(kind='shadow_task_failed',**self.latest))
        finally:
            if self.socket is not None:self.socket.close()

    def close(self):
        self.stop_event.set()
        if self.thread is not None:self.thread.join(timeout=2.)
        self.raw_log.close();self.decision_log.close()
        return dict(mode='shadow',control_affected=False,error=self.error,recording_errors=dict(raw=self.raw_log.error,decisions=self.decision_log.error),last_state=self.latest)
