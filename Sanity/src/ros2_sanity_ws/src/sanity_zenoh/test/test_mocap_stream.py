"""Cross-language freshness/replay tests and native pinned-Pico integration."""
import ctypes
import json
import os
from pathlib import Path
import socket
import struct
import subprocess
import tempfile
import time
import pytest

from sanity_zenoh.mocap_protocol import ClockBounds, MAX_AGE_US, STATE, decode_status, state_packet
from sanity_zenoh.mocap_stream import MocapStreamPublisher
from sanity_zenoh.transport import SanityTransport
from sanity_zenoh.protocol import command_packet, ReceiverStatus, CTBR

ROOT=Path(__file__).resolve().parents[6]
CORE=ROOT/'firmware/receiver/main'
VALUE=(1.,2.,3.,1.,0.,0.,0.,.1,.2,.3)

@pytest.fixture(scope='module')
def native():
    with tempfile.TemporaryDirectory() as directory:
        p=Path(directory);(p/'wrapper.c').write_text('''
#include "mocap_state.h"
static struct sz_mocap_cache cache;
void reset(unsigned device,unsigned long long epoch){sz_mocap_init(&cache,device,epoch);}
int accept(const unsigned char *bytes,unsigned n,unsigned long long now){
 struct sz_mocap_sample s;
 if(!sz_mocap_decode(bytes,n,cache.device,&s)){cache.rejected++;return 0;}
 return sz_mocap_accept(&cache,&s,now);
}
void status(unsigned long long now,unsigned char *out){sz_mocap_status(&cache,now,out);}
''')
        subprocess.run(['cc','-std=c11','-shared','-fPIC','-Wall','-Wextra','-Werror','-I',str(CORE),
                        str(p/'wrapper.c'),str(CORE/'mocap_state.c'),'-lm','-o',str(p/'core.so')],check=True)
        lib=ctypes.CDLL(str(p/'core.so'))
        lib.reset.argtypes=[ctypes.c_uint,ctypes.c_ulonglong]
        lib.accept.argtypes=[ctypes.c_char_p,ctypes.c_uint,ctypes.c_ulonglong];lib.accept.restype=ctypes.c_int
        lib.status.argtypes=[ctypes.c_ulonglong,ctypes.c_void_p]
        yield lib

def packet(**change):
    v=dict(device=9,epoch=7,session=11,sequence=1,source_ros_ns=1000000000,
           lower_us=100000,upper_us=102000,state=VALUE);v.update(change);return state_packet(**v)

def status(native,now):
    out=ctypes.create_string_buffer(48);native.status(now,out);return decode_status(out.raw,9)

def test_exact_wire_and_expiry(native):
    native.reset(9,7);p=packet();assert len(p)==88
    assert native.accept(p,len(p),110000)==1
    assert status(native,110000)['usable']
    assert status(native,140000)['usable']
    assert not status(native,140001)['usable']
    assert not status(native,109999)['usable'] # caller time must not precede receipt

@pytest.mark.parametrize('offset,value',[(0,0),(4,2),(5,8),(6,2),(7,1)])
def test_reject_header_and_frame(native,offset,value):
    native.reset(9,7);p=bytearray(packet());p[offset]=value
    assert not native.accept(bytes(p),len(p),110000)
    assert status(native,110000)['accepted']==0

@pytest.mark.parametrize('index,value',[(0,float('nan')),(2,float('inf')),(3,0.),(7,101.)])
def test_reject_nonfinite_and_bad_quaternion(native,index,value):
    native.reset(9,7);p=bytearray(packet());struct.pack_into('>f',p,48+index*4,value)
    assert not native.accept(bytes(p),len(p),110000)

def test_replay_frozen_source_does_not_refresh(native):
    native.reset(9,7);p=packet();assert native.accept(p,len(p),110000)
    for changed in [p,packet(sequence=2),packet(sequence=0,source_ros_ns=1000000001,lower_us=100001)]:
        assert not native.accept(changed,len(changed),120000)
    s=status(native,140001);assert not s['usable'] and s['received_esp_us']==110000 and s['rejected']==3

def test_bounds_epoch_wrap_and_publisher_takeover(native):
    native.reset(9,7)
    for p in [packet(epoch=8),packet(lower_us=120000,upper_us=121000),packet(lower_us=1,upper_us=2)]:
        assert not native.accept(p,len(p),110000)
    p=packet(sequence=2**32-1);assert native.accept(p,len(p),110000)
    p=packet(sequence=0,source_ros_ns=1000000001,lower_us=100001);assert native.accept(p,len(p),110001)
    p=packet(session=12,source_ros_ns=1000000002,lower_us=100002);assert not native.accept(p,len(p),110002)
    p=packet(session=12,source_ros_ns=1000000003,lower_us=150000,upper_us=152000);assert native.accept(p,len(p),155000)
    native.reset(9,9);assert not native.accept(p,len(p),155000)

def test_clock_interval_contains_offset_for_asymmetric_delay():
    for forward,reverse,processing in [(100,900,50),(900,100,500),(5000,200,20000)]:
        h0=1000000;offset=555000;e1=h0+forward+offset;e2=e1+processing;h3=h0+forward+processing+reverse
        clock=ClockBounds.exchange(7,h0,e1,e2,h3)
        lo,hi=clock.capture(h3-100,h3)
        assert lo<=h3-100+offset<=hi
        with pytest.raises(ValueError):clock.capture(h3,h3+2000001)
    with pytest.raises(ValueError):ClockBounds.exchange(7,100,500,600,150)

def test_clock_uncertainty_cannot_be_hidden_in_state():
    with pytest.raises(ValueError):packet(lower_us=100000,upper_us=121000)
    clock=ClockBounds.exchange(7,1000000,1000000,1000010,1040000)
    with pytest.raises(ValueError):clock.capture(1040000,1040000)

@pytest.mark.parametrize('skew_ppm',[-200.,0.,200.])
def test_combined_clock_contains_true_time_with_drift_and_asymmetric_queues(skew_ppm):
    import random
    random.seed(19)
    def esp(host):return round(555000+host*(1+skew_ppm/1e6))
    combined=None
    for k in range(1000):
        h0=1_000_000+50_000*k
        forward=random.randint(80,9000);reverse=random.randint(80,9000)
        processing=random.randint(10,20000);h3=h0+forward+processing+reverse
        candidate=ClockBounds.exchange(7,h0,esp(h0+forward),esp(h0+forward+processing),h3)
        combined=candidate if combined is None else combined.intersect(candidate)
        capture=h3-random.randint(0,40000)
        lo,hi=combined.capture(capture,h3)
        assert lo<=esp(capture)<=hi

def test_combined_clock_uses_causal_bounds_without_symmetric_delay_assumption():
    offset=555000
    first=ClockBounds.exchange(7,1000000,1000100+offset,1000200+offset,1006200)
    second=ClockBounds.exchange(7,1200000,1206000+offset,1206100+offset,1206200)
    assert min(first.high-first.low,second.high-second.low)>6000
    combined=first.intersect(second)
    assert combined.low<=offset<=combined.high
    assert combined.high-combined.low<300

def test_combined_clock_rejects_inconsistency_and_expires_old_information():
    first=ClockBounds(7,100,200,1000000)
    with pytest.raises(ValueError,match='Inconsistent'):first.intersect(ClockBounds(7,1000,1100,1100000))
    with pytest.raises(ValueError,match='Reversed'):first.intersect(ClockBounds(7,100,200,999999))
    expired=ClockBounds(7,1000,1100,3000001)
    assert first.intersect(expired) is expired
    restarted=ClockBounds(8,-1000,-900,1000001)
    assert first.intersect(restarted) is restarted

@pytest.mark.parametrize('rate',[0,51,float('nan'),float('inf')])
def test_clock_probe_rate_rejected_before_network_setup(rate):
    with pytest.raises(ValueError,match='Clock probe rate'):
        MocapStreamPublisher(None,clock_rate_hz=rate)

@pytest.mark.parametrize('processing_delay_s',[0.,.03])
def test_udp_clock_native_reply_nonce_validation_and_pose_encoding(native,processing_delay_s):
    import threading
    from sanity_zenoh.mocap_protocol import CLOCK_REQUEST
    native.sz_mocap_clock_reply.argtypes=[ctypes.c_uint8]+[ctypes.c_uint64]*5+[ctypes.c_void_p]
    native.sz_mocap_clock_request.argtypes=[ctypes.c_void_p,ctypes.c_size_t,ctypes.c_uint8,
                                          ctypes.POINTER(ctypes.c_uint64),ctypes.POINTER(ctypes.c_uint64)]
    native.sz_mocap_clock_request.restype=ctypes.c_bool
    server=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);server.bind(('127.0.0.1',0));server.settimeout(.05)
    stop=threading.Event();requests=[];puts=[];entities=[]
    class Entity:
        def __init__(self,key):self.key=key;self.closed=False;entities.append(self)
        def put(self,raw):puts.append((self.key,raw))
        def undeclare(self):self.closed=True
    class Session:
        def declare_publisher(self,key,**kwargs):return Entity(key)
        def declare_subscriber(self,key,*args):return Entity(key)
    def respond():
        while not stop.is_set():
            try:raw,peer=server.recvfrom(100)
            except socket.timeout:continue
            received=time.monotonic_ns()//1000+555000
            nonce=ctypes.c_uint64();host=ctypes.c_uint64()
            assert native.sz_mocap_clock_request(raw,len(raw),15,ctypes.byref(nonce),ctypes.byref(host))
            requests.append(raw);reply=ctypes.create_string_buffer(48)
            time.sleep(processing_delay_s)
            # The first correctly formatted packet deliberately has an unmatched nonce.
            native.sz_mocap_clock_reply(15,7,nonce.value+(len(requests)==1),host.value,received,
                                       time.monotonic_ns()//1000+555000,reply)
            server.sendto(reply.raw,peer)
    thread=threading.Thread(target=respond);thread.start()
    stream=None
    try:
        stream=MocapStreamPublisher(Session(),15,clock_rate_hz=20.,clock_endpoint=f'udp/127.0.0.1:{server.getsockname()[1]}')
        deadline=time.monotonic()+2
        while stream.clock is None and time.monotonic()<deadline:time.sleep(.01)
        assert stream.clock and stream.clock.epoch==7
        assert stream.clock.high-stream.clock.low<10000
        assert stream.errors==1 and len(requests)>=2
        now=time.time_ns();host=time.monotonic_ns()//1000
        assert stream.send_estimate(dict(ready=True,frames_confirmed=True,stamp_ns=now,state=VALUE),now,host)
        key,raw=puts[-1];fields=STATE.unpack(raw)
        assert key=='sanity/15/state/mocap' and fields[5]==7
        assert fields[9]<=host+555000<=fields[10]
        assert len(entities)==2 # pose publisher + status subscriber, no control / Zenoh clock keys
        stop.set();thread.join(timeout=1)
        with stream.lock:
            now=time.time_ns()
            assert not stream.send_estimate(dict(ready=True,frames_confirmed=True,stamp_ns=now,state=VALUE),now,
                                            stream.clock.host_us+stream.clock.ttl_us+1)
            assert stream.last_error=='Expired clock bound or future capture'
        for invalid in [requests[-1]+b'X',requests[-1][:-1],CLOCK_REQUEST.pack(b'SZMQ',1,14,0,1,1)]:
            n=ctypes.c_uint64();h=ctypes.c_uint64()
            assert not native.sz_mocap_clock_request(invalid,len(invalid),15,ctypes.byref(n),ctypes.byref(h))
    finally:
        if stream:stream.close()
        stop.set();thread.join(timeout=1);server.close()
    assert all(e.closed for e in entities)

def available_port(kind=socket.SOCK_STREAM):
    with socket.socket(socket.AF_INET,kind) as s:s.bind(('127.0.0.1',0));return s.getsockname()[1]

@pytest.mark.parametrize('backend',['tcp','udp'])
def test_pico_receive_alongside_commands_and_forwarded_pc_publisher(backend):
    exe=os.environ.get('SANITY_MOCAP_EMULATOR')
    if not exe:pytest.skip('Set SANITY_MOCAP_EMULATOR to native target')
    endpoint=f'{backend}/127.0.0.1:{available_port(socket.SOCK_DGRAM if backend=="udp" else socket.SOCK_STREAM)}'
    local=f'tcp/127.0.0.1:{available_port()}'
    transport=SanityTransport([endpoint,local]);producer=SanityTransport(connect=local)
    stream=MocapStreamPublisher(producer.session,9);controls=[]
    subscriber=transport.session.declare_subscriber('sanity/09/status',lambda s:controls.append(ReceiverStatus.decode(bytes(s.payload))))
    pub=transport.session.declare_publisher('sanity/09/command/ctbr')
    with tempfile.TemporaryFile(mode='w+') as log:
        process=subprocess.Popen([exe,endpoint,'12345'],stdout=log,stderr=log)
        try:
            deadline=time.monotonic()+8
            while time.monotonic()<deadline:
                now=time.time_ns()
                stream.send_estimate(dict(ready=True,frames_confirmed=True,stamp_ns=now,state=VALUE),now)
                if controls:
                    c=controls[-1];pub.put(command_packet(9,CTBR,False,c.epoch,9,stream.sent,c.receiver_ms,1000,(1500,1500,1500,1000)))
                if stream.status and stream.status['accepted']>=8 and stream.status['usable'] and controls and controls[-1].session==9:break
                time.sleep(.01)
            assert stream.status and stream.status['accepted']>=8 and stream.status['usable'],stream.snapshot()
            assert controls[-1].session==9 and not controls[-1].enabled
            old=stream.sent;packet_state=dict(ready=True,frames_confirmed=True,stamp_ns=now,state=VALUE)
            assert not stream.send_estimate(packet_state,now+100000000)
            time.sleep(.15)
            assert stream.sent==old and stream.status and not stream.status['usable']
        finally:
            stream.close();pub.undeclare();subscriber.undeclare();producer.close();transport.close()
            process.terminate();process.wait(timeout=5)

def test_ros_tf_to_pico_and_frozen_source_expiry():
    rclpy=pytest.importorskip('rclpy')
    exe=os.environ.get('SANITY_MOCAP_EMULATOR')
    if not exe:pytest.skip('Set SANITY_MOCAP_EMULATOR')
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from tf2_msgs.msg import TFMessage
    from geometry_msgs.msg import TransformStamped
    from sanity_zenoh.mocap_stream_node import MocapStreamNode
    endpoint=f'tcp/127.0.0.1:{available_port()}'
    rclpy.init(domain_id=211)
    node=None;publisher_node=None;process=None
    with tempfile.TemporaryFile(mode='w+') as log:
        try:
            node=MocapStreamNode(parameter_overrides=[Parameter('frames_confirmed',value=True),
                Parameter('listen',value=[endpoint]),Parameter('tf_topic',value='/mocap_stream_test/tf')])
            assert node.status_pub.topic_name=='/sanity09/mocap_stream_status'
            publisher_node=Node('mocap_stream_test_source')
            pub=publisher_node.create_publisher(TFMessage,'/mocap_stream_test/tf',QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
            process=subprocess.Popen([exe,endpoint,'99999'],stdout=log,stderr=log)
            end=time.monotonic()+8
            while time.monotonic()<end:
                tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='sanity09'
                tf.header.stamp=publisher_node.get_clock().now().to_msg();tf.transform.translation.z=1.;tf.transform.rotation.w=1.
                pub.publish(TFMessage(transforms=[tf]));rclpy.spin_once(node,timeout_sec=.005)
                s=node.stream.snapshot()
                if s['receiver_usable'] and s['receiver']['accepted']>=10:break
                time.sleep(.005)
            assert node.stream.snapshot()['receiver_usable'],node.stream.snapshot()
            sent=node.stream.sent
            # Keep publishing the same transform: receipt activity must not refresh capture time.
            end=time.monotonic()+.2
            while time.monotonic()<end:
                pub.publish(TFMessage(transforms=[tf]));rclpy.spin_once(node,timeout_sec=.005);time.sleep(.005)
            assert node.stream.sent==sent
            assert not node.stream.snapshot()['receiver_usable']
        finally:
            if node:node.destroy_node()
            if publisher_node:publisher_node.destroy_node()
            if rclpy.ok():rclpy.shutdown()
            if process:process.terminate();process.wait(timeout=5)


def test_ros_entity_failure_closes_observation_transport(monkeypatch):
    rclpy=pytest.importorskip('rclpy')
    from rclpy.parameter import Parameter
    import sanity_zenoh.mocap_stream_node as module
    closed=[]
    class Transport:
        session=object()
        def __init__(self,*args):pass
        def close(self):closed.append('transport')
    class Stream:
        def __init__(self,*args,**kwargs):pass
        def close(self):closed.append('stream')
    original_publisher=module.MocapStreamNode.create_publisher
    def failed_publisher(self,*args,**kwargs):
        if len(args)>1 and str(args[1]).endswith('/mocap_stream_status'):
            raise RuntimeError('synthetic ROS publisher failure')
        return original_publisher(self,*args,**kwargs)
    monkeypatch.setattr(module,'SanityTransport',Transport)
    monkeypatch.setattr(module,'MocapStreamPublisher',Stream)
    monkeypatch.setattr(module.MocapStreamNode,'create_publisher',failed_publisher)
    rclpy.init(domain_id=212)
    try:
        with pytest.raises(RuntimeError,match='synthetic ROS publisher'):
            module.MocapStreamNode(parameter_overrides=[Parameter('frames_confirmed',value=True)])
        assert closed==['stream','transport']
    finally:
        if rclpy.ok():rclpy.shutdown()
