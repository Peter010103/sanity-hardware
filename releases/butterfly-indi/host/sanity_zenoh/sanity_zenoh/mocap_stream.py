"""ROS-independent mocap publisher, sharing a caller-owned Zenoh session.

No command publisher, arming capability, FC writes, or source-pose replay.
"""
import math
import secrets
import socket
import threading
import time
import zenoh
from .mocap_protocol import CLOCK_REQUEST, CLOCK_REPLY, ClockBounds, decode_status, state_packet, MAX_AGE_US


class MocapStreamPublisher:
    def __init__(self, session, device=9, maximum_rate_hz=100., on_event=None, on_record=None, clock_rate_hz=5., clock_endpoint=None):
        if type(device) is not int or not 1<=device<=255 or not math.isfinite(maximum_rate_hz) or not 1<=maximum_rate_hz<=200:
            raise ValueError('Device 1..255 and rate 1..200 Hz required')
        if not math.isfinite(clock_rate_hz) or not 1<=clock_rate_hz<=50:
            raise ValueError('Clock probe rate 1..50 Hz required')
        self.clock_interval_s=1./clock_rate_hz
        self.clock_endpoint=clock_endpoint;self.clock_socket=None
        self.device=device;self.on_record=on_record;self.on_event=on_event;self.lock=threading.RLock();self.stopping=threading.Event()
        self.session_id=secrets.randbelow(2**32-1)+1;self.sequence=0;self.clock=None;self.pending={}
        self.last_source_ns=0;self.last_send_us=0;self.interval_us=math.ceil(1e6/maximum_rate_hz)
        self.status=None;self.status_received_us=0;self.sent=0;self.skipped=0;self.errors=0;self.last_error=None
        self.entities=[]
        root=f'sanity/{device:02d}/state/mocap'
        options=dict(priority=zenoh.Priority.DATA_HIGH,congestion_control=zenoh.CongestionControl.DROP,
                     reliability=zenoh.Reliability.BEST_EFFORT,express=True)
        try:
            self.publisher=session.declare_publisher(root,**options);self.entities.append(self.publisher)
            if clock_endpoint:
                if not clock_endpoint.startswith('udp/'):raise ValueError('Clock endpoint must be udp/IPv4:port')
                host,port=clock_endpoint[4:].rsplit(':',1);socket.inet_pton(socket.AF_INET,host)
                if not 1<=int(port)<=65535:raise ValueError('Invalid UDP clock port')
                self.clock_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
                self.clock_socket.connect((host,int(port)))
                self.clock_socket.settimeout(min(.02,self.clock_interval_s))
            else:
                self.clock_publisher=session.declare_publisher(root+'/clock/request',**options);self.entities.append(self.clock_publisher)
                self.clock_sub=session.declare_subscriber(root+'/clock/reply',self.receive_clock);self.entities.append(self.clock_sub)
            self.status_sub=session.declare_subscriber(root+'/status',self.receive_status);self.entities.append(self.status_sub)
        except BaseException:
            for entity in reversed(self.entities):entity.undeclare()
            if self.clock_socket is not None:self.clock_socket.close()
            raise
        self.thread=threading.Thread(target=self.run_clock,name='mocap_clock',daemon=True);self.thread.start()

    @staticmethod
    def now_us():return time.monotonic_ns()//1000

    def emit(self,event):
        if self.on_event:self.on_event(event)

    def run_clock(self):
        while not self.stopping.is_set():
            started=time.monotonic()
            now=self.now_us();nonce=secrets.randbelow(2**64-1)+1
            with self.lock:
                self.pending={n:h for n,h in self.pending.items() if now-h<2000000}
                self.pending[nonce]=now
            try:
                request=CLOCK_REQUEST.pack(b'SZMQ',1,self.device,0,nonce,now)
                if self.clock_socket is None:self.clock_publisher.put(request)
                else:
                    self.clock_socket.send(request)
                    # Continue receiving between probes. A late reply must not
                    # remain queued until the following 50/200 ms probe cycle.
                    deadline=started+self.clock_interval_s
                    while not self.stopping.is_set():
                        remaining=deadline-time.monotonic()
                        if remaining<=0:break
                        self.clock_socket.settimeout(min(.02,remaining))
                        try:
                            raw=self.clock_socket.recv(CLOCK_REPLY.size+1)
                            received=self.now_us()
                            self.receive_clock_bytes(raw,received)
                        except socket.timeout:pass
            except Exception as error:
                with self.lock:self.errors+=1;self.last_error=str(error)
            self.stopping.wait(max(0.,self.clock_interval_s-(time.monotonic()-started)))

    def receive_clock(self,sample):
        now=self.now_us()
        self.receive_clock_bytes(bytes(sample.payload),now)

    def receive_clock_bytes(self,raw,now):
        try:
            if len(raw)!=CLOCK_REPLY.size:raise ValueError('Wrong mocap clock reply size')
            magic,version,device,reserved,epoch,nonce,host,received,sent=CLOCK_REPLY.unpack(raw)
            if (magic,version,device,reserved)!=(b'SZMR',1,self.device,0):raise ValueError('Invalid mocap clock reply')
            with self.lock:
                expected=self.pending.pop(nonce,None)
                if expected!=host or now-host>2000000:raise ValueError('Unmatched/expired clock reply')
                candidate=ClockBounds.exchange(epoch,host,received,sent,now)
                if self.clock is None or self.clock.epoch!=epoch:
                    self.clock=candidate;self.session_id=secrets.randbelow(2**32-1)+1;self.sequence=0
                    self.pending.clear()
                else:
                    try:self.clock=self.clock.intersect(candidate)
                    except ValueError:
                        self.clock=None
                        raise
                combined_width=self.clock.high-self.clock.low
            self.emit(dict(kind='mocap_clock',epoch=epoch,offset_lower_us=candidate.low,
                           offset_upper_us=candidate.high,interval_width_us=candidate.high-candidate.low,
                           host_send_us=host,esp_receive_us=received,esp_send_us=sent,host_receive_us=now,
                           combined_interval_width_us=combined_width,clock_transport=self.clock_endpoint or 'zenoh'))
        except (ValueError,TypeError,OverflowError) as error:
            with self.lock:self.errors+=1;self.last_error=str(error)

    def receive_status(self,sample):
        try:
            status=decode_status(bytes(sample.payload),self.device)
            with self.lock:self.status=status;self.status_received_us=self.now_us()
            self.emit(dict(kind='mocap_receiver_status',**status))
        except (ValueError,TypeError) as error:
            with self.lock:self.errors+=1;self.last_error=str(error)

    def send_estimate(self,packet,ros_now_ns,host_now_us=None):
        """Called once per newly accepted TF. Never retransmit a held pose."""
        now=self.now_us() if host_now_us is None else host_now_us
        with self.lock:
            try:
                if not packet.get('ready') or not packet.get('frames_confirmed') or packet.get('orientation_held'):
                    raise ValueError('Calibrated, ready, unheld mocap state required')
                stamp=int(packet['stamp_ns']);age_ns=int(ros_now_ns)-stamp
                if not 0<=age_ns<=MAX_AGE_US*1000:raise ValueError('TF source stale or future')
                if stamp<=self.last_source_ns:raise ValueError('Duplicate or reversed TF source timestamp')
                if now-self.last_send_us<self.interval_us:
                    self.skipped+=1;return False
                if self.clock is None:raise ValueError('Waiting for bounded ESP clock exchange')
                capture=now-math.ceil(age_ns/1000)
                lower,upper=self.clock.capture(capture,now)
                # State estimator layout is p3,q_wxyz4,v3,world-angular-rate3.
                raw=state_packet(self.device,self.clock.epoch,self.session_id,self.sequence,
                                 stamp,lower,upper,packet['state'][:10])
                if self.on_record:self.on_record(dict(kind='mocap_original_source',host_now_us=now,ros_now_ns=int(ros_now_ns),source_packet=packet,encoded_hex=raw.hex()))
                self.publisher.put(raw)
                self.sequence=(self.sequence+1)&0xffffffff;self.last_source_ns=stamp;self.last_send_us=now;self.sent+=1;self.last_error=None
                return True
            except (ValueError,KeyError,TypeError,OverflowError) as error:
                self.skipped+=1;self.last_error=str(error);return False

    def snapshot(self):
        with self.lock:
            age=self.now_us()-self.status_received_us if self.status else None
            return dict(sent=self.sent,skipped=self.skipped,errors=self.errors,last_error=self.last_error,
                        clock_interval_width_us=self.clock.high-self.clock.low if self.clock else None,
                        clock_probe_rate_hz=1./self.clock_interval_s,
                        clock_transport=self.clock_endpoint or 'zenoh',
                        receiver=self.status,receiver_status_age_us=age,
                        receiver_usable=bool(self.status and self.status['usable'] and age<=200000),
                        scope='Observation-only cache; no FC forwarding or control consumer')

    def close(self):
        if self.stopping.is_set():return
        self.stopping.set();self.thread.join(timeout=3.)
        if self.thread.is_alive():raise RuntimeError('Mocap clock thread did not stop')
        if self.clock_socket is not None:self.clock_socket.close()
        for entity in reversed(self.entities):entity.undeclare()
