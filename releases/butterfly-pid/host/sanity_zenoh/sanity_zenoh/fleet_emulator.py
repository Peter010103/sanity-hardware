import ipaddress
import json
import secrets
import struct
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation
import zenoh

from sanity_zenoh.protocol import COMMAND, STATUS_V2, neutral
from sanity_zenoh.transport_test import TRACE_HEADER, TRACE_RECORD
from sanity_zenoh.fc_telemetry import BATCH_HEADER, RECORD_HEADER


class SimulatedReceiver:
    """Loopback protocol/rigid-body fixture; not the flight firmware or calibrated plant."""
    def __init__(self, drone, endpoints, calibration_id, mass, tf_publisher=None, ros_node=None, mocap=None, disturbance_world_m_s2=(0.,0.,0.)):
        for value in endpoints.values():
            host=value.split('/',1)[1].rsplit(':',1)[0]
            if not ipaddress.ip_address(host).is_loopback:
                raise ValueError('Emulators cannot connect to physical/external endpoints')
        self.disturbance_world_m_s2=np.asarray(disturbance_world_m_s2,dtype=float)
        if (self.disturbance_world_m_s2.shape!=(3,) or not np.isfinite(self.disturbance_world_m_s2).all()
                or np.linalg.norm(self.disturbance_world_m_s2)>1.):
            raise ValueError('Finite bounded simulated acceleration bias required')
        self.device, self.calibration_id, self.mass = drone['id'],calibration_id,mass
        self.drone, self.mocap = drone,mocap or {'parent_frame':'map','world_from_map_xyzw':[0.,0.,0.,1.]}
        self.tf_publisher,self.ros_node=tf_publisher,ros_node
        self.lock=threading.RLock()
        self.epoch=secrets.randbelow(2**63-1)+1
        self.session_id,self.sequence,self.rejected=0,0,0
        self.enabled,self.failsafe=False,True
        self.deadline=-float('inf')
        self.values=np.zeros(4)
        self.position=np.r_[drone['home_xy'],.025]
        self.velocity=np.zeros(3)
        self.rotation=Rotation.identity()
        self.rates=np.zeros(3)
        self.force=0.
        self.hold_until=0.
        self.pending=[]
        self.accepted=[]
        self.command_count=0
        self.trace=[]
        self.stopping=threading.Event()
        self.sessions=[]
        self.closed=False
        try:
            for key in ('listen','telemetry_listen'):
                config=zenoh.Config()
                config.insert_json5('mode','"peer"')
                config.insert_json5('scouting/multicast/enabled','false')
                config.insert_json5('scouting/gossip/enabled','false')
                config.insert_json5('listen/endpoints','[]')
                config.insert_json5('connect/endpoints',json.dumps([endpoints[key]]))
                self.sessions.append(zenoh.open(config))
            root=f'sanity/{self.device:02d}'
            self.subscriber=self.sessions[0].declare_subscriber(root+'/command/ctbr',self.receive)
            self.status_pub=self.sessions[0].declare_publisher(root+'/status')
            self.telemetry_pub=self.sessions[1].declare_publisher(root+'/telemetry/fc')
            self.trace_pub=self.sessions[1].declare_publisher(root+'/diagnostics/trace')
            self.thread=threading.Thread(target=self.run,daemon=True,name=f'loopback_receiver_{self.device}')
            self.thread.start()
        except BaseException:
            for session in self.sessions:session.close()
            raise


    def restart(self):
        with self.lock:
            self.epoch+=1
            self.enabled,self.failsafe=False,True
            self.session_id,self.sequence=0,0
            self.values[:]=0.
            self.pending.clear()


    def receive(self, sample):
        raw=bytes(sample.payload)
        with self.lock:
            self.pending.append(raw)
            if time.monotonic()>=self.hold_until:self.flush()


    def flush(self):
        pending,self.pending=self.pending,[]
        for raw in pending:
            try:
                h=COMMAND.unpack(raw)
                now=time.monotonic()
                receiver_ms=int(now*1000)&0xffffffff
                if h[0] not in (b'SZC2',b'SZC3') or h[2]!=1 or h[3]!=self.device or h[5]!=self.epoch:
                    raise ValueError('Incorrect route/version/epoch')
                if h[10]!=(self.calibration_id&0xffff):raise ValueError('Calibration mismatch')
                if not 0<=((receiver_ms-h[8])&0xffffffff)<h[9]<=1000:raise ValueError('Expired lease')
                same=h[6]==self.session_id
                if same and ((h[7]-self.sequence)&0xffffffff)>=0x80000000:raise ValueError('Reordered command')
                if self.enabled and not same:raise ValueError('Another sender owns output')
                enabled=bool(h[4])
                roll,pitch,yaw,thrust=struct.unpack_from('>3hH',raw,32)
                values=np.array([thrust/10000.,roll/(1000 if h[1]==3 else 10000),pitch/(1000 if h[1]==3 else 10000),yaw/(1000 if h[1]==3 else 10000)])
                if enabled and self.failsafe:raise ValueError('Explicit disabled claim required')
                if enabled and not self.enabled and np.any(values):raise ValueError('Enabled-zero handshake required')
                if not enabled and np.any(values):raise ValueError('Disabled values nonzero')
                self.enabled,self.failsafe=enabled,False
                self.session_id,self.sequence=h[6],h[7]
                self.deadline=now+h[9]/1000.
                self.values=values if enabled else np.zeros(4)
                us=int(now*1e6)
                self.trace.append((1,1,1,0,self.session_id,self.sequence,0,self.epoch,us,us,0,0,0,0))
                self.accepted.append({'host_ns':time.monotonic_ns(),'device':self.device,'session':self.session_id,'sequence':self.sequence,'enabled':enabled,'ctbr':self.values.tolist()})
                self.command_count+=1
            except (ValueError,struct.error):self.rejected+=1


    def publish_status(self):
        with self.lock:
            flags=4|int(self.enabled)|(2 if self.failsafe else 0)
            packet=STATUS_V2.pack(b'SZS2',2,1,self.device,flags,self.epoch,int(time.monotonic()*1000)&0xffffffff,
                self.session_id,self.sequence,self.rejected,7800,0,self.calibration_id,*neutral(1))
        self.status_pub.put(packet)


    def publish_diagnostics(self):
        with self.lock:
            records,self.trace=self.trace[:20],self.trace[20:]
            epoch,session,sequence=self.epoch,self.session_id,self.sequence
        if records:self.trace_pub.put(TRACE_HEADER.pack(b'SZT1',1,self.device,64,len(records),0)+b''.join(TRACE_RECORD.pack(*record) for record in records))
        us=int(time.monotonic()*1e6)
        battery=bytearray(11)
        battery[0]=2
        struct.pack_into('<H',battery,9,780)
        record=RECORD_HEADER.pack(2,130,0,11,epoch,session,sequence,us,us,us)+battery
        self.telemetry_pub.put(BATCH_HEADER.pack(b'SZF1',1,self.device,1,0)+record)


    def step(self, dt):
        with self.lock:
            if self.enabled and time.monotonic()>=self.deadline:
                self.enabled,self.failsafe=False,True
                self.values[:]=0.
            values=self.values.copy()
        for _ in range(max(1,int(np.ceil(dt/.005)))):
            step=dt/max(1,int(np.ceil(dt/.005)))
            self.rates+=(values[1:]-self.rates)*(-np.expm1(-step/.04))
            self.force+=(values[0]-self.force)*(-np.expm1(-step/.045))
            self.rotation=self.rotation*Rotation.from_rotvec(self.rates*step)
            acceleration=self.rotation.apply([0.,0.,self.force/self.mass])+np.array([0.,0.,-9.81])-.05*self.velocity
            if self.position[2]>.03:acceleration+=self.disturbance_world_m_s2
            self.velocity+=acceleration*step
            self.position+=self.velocity*step
            if self.position[2]<.025:
                self.position[2]=.025
                self.velocity[2]=max(0.,self.velocity[2])
                self.velocity[:2]*=np.exp(-step*15.)


    def transform(self):
        from geometry_msgs.msg import TransformStamped
        from tf2_msgs.msg import TFMessage
        world=Rotation.from_quat(self.mocap['world_from_map_xyzw'])
        marker=world.inv()*self.rotation*Rotation.from_quat(self.drone['marker_to_base_xyzw']).inv()
        position=world.inv().apply(self.position)-marker.apply(self.drone['marker_to_base_xyz'])
        message=TransformStamped()
        message.header.frame_id=self.mocap['parent_frame']
        message.child_frame_id=self.drone['tracked_frame']
        message.header.stamp=self.ros_node.get_clock().now().to_msg()
        message.transform.translation.x,message.transform.translation.y,message.transform.translation.z=map(float,position)
        message.transform.rotation.x,message.transform.rotation.y,message.transform.rotation.z,message.transform.rotation.w=map(float,marker.as_quat())
        return message


    def run(self):
        previous=time.monotonic()
        next_status,next_diagnostic=previous,previous
        while not self.stopping.wait(.005):
            now=time.monotonic()
            with self.lock:
                if now>=self.hold_until:self.flush()
            self.step(now-previous)
            previous=now
            if self.tf_publisher is not None:
                from tf2_msgs.msg import TFMessage
                self.tf_publisher.publish(TFMessage(transforms=[self.transform()]))
            if now>=next_status:self.publish_status();next_status=now+.02
            if now>=next_diagnostic:self.publish_diagnostics();next_diagnostic=now+.10


    def close(self):
        if self.closed:return
        self.closed=True
        self.stopping.set()
        self.thread.join(timeout=2.)
        for entity in (self.subscriber,self.status_pub,self.telemetry_pub,self.trace_pub):entity.undeclare()
        for session in self.sessions:session.close()
