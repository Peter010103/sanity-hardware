"""Short, causal state projection to the asynchronous MPC selection epoch.

This is a nominal prediction from measured pose/velocity and selected commands,
not an estimator, a mocap-loss bridge or a relaxed input freshness check.
"""
from bisect import bisect_right
import math
import numpy as np
from indi_rate_model import RateModelHistory,continuous_matrices

class CommandPrediction:
    def __init__(self,configuration,mass,thrust_effectiveness=1.,drag=(0.,0.,0.),command_delay_s=0.):
        self.history=RateModelHistory(configuration)
        self.A,self.B,self.C=continuous_matrices(configuration)
        self.mass=float(mass);self.eta=float(thrust_effectiveness)
        if not 0<self.mass<1 or not .8<=self.eta<=1.2:raise ValueError('Invalid prediction mass/effectiveness')
        self.drag=np.asarray(drag,dtype=float);self.command_delay_s=float(command_delay_s)
        if self.drag.shape!=(3,) or not np.isfinite(self.drag).all() or np.any(self.drag<0) or np.any(self.drag>5) or not 0<=self.command_delay_s<=.03:raise ValueError('Invalid effective damping/command delay')
        self.commands=[];self.last={}

    def reset(self):
        self.commands=[];self.last={}

    def commit(self,stamp_ns,command):
        u=np.asarray(command,dtype=float)
        if type(stamp_ns) is not int or stamp_ns<=0 or u.shape!=(4,) or not np.isfinite(u).all():raise ValueError('Timestamped finite selected CTBR required')
        stamp_ns+=int(round(self.command_delay_s*1e9))
        if self.commands and stamp_ns<=self.commands[-1][0]:raise ValueError('Selected command timestamps must increase')
        if not self.commands:self.history.reset(stamp_ns-100_000_000)
        self.history.commit(stamp_ns,u[1:]);self.commands.append((stamp_ns,u.copy()))
        # Keep a prior held-command anchor as well as two seconds of history.
        while len(self.commands)>2 and self.commands[1][0]<stamp_ns-2_000_000_000:self.commands.pop(0)

    def held(self,stamp):
        if not self.commands:return np.zeros(4)
        index=bisect_right([t for t,_ in self.commands],stamp)-1
        return self.commands[index][1] if index>=0 else np.zeros(4)

    @staticmethod
    def rotation_z(q):
        return np.array([2*(q[1]*q[3]+q[0]*q[2]),2*(q[2]*q[3]-q[0]*q[1]),1-2*(q[1]**2+q[2]**2)])

    def derivative(self,x,u):
        q=x[3:7];hidden=x[10:];w=self.C@hidden
        qdot=.5*np.r_[-np.dot(q[1:],w),q[0]*w+np.cross(q[1:],w)]
        acc=self.rotation_z(q)*(self.eta*u[0]/self.mass)-[0.,0.,9.81]
        if np.any(self.drag):
            w,xq,y,z=q
            rotation=np.array([[1-2*(y*y+z*z),2*(xq*y-w*z),2*(xq*z+w*y)],[2*(xq*y+w*z),1-2*(xq*xq+z*z),2*(y*z-w*xq)],[2*(xq*z-w*y),2*(y*z+w*xq),1-2*(xq*xq+y*y)]])
            acc-=rotation@(self.drag*(rotation.T@x[7:10]))
        return np.r_[x[7:10],qdot,acc,self.A@hidden+self.B@u[1:]]

    def predict(self,state,source_stamp_ns,target_stamp_ns):
        s=np.asarray(state,dtype=float)
        if s.shape!=(13,) or not np.isfinite(s).all() or abs(np.linalg.norm(s[3:7])-1)>.001:raise ValueError('Finite measured source state required')
        if any(type(t) is not int or t<=0 for t in [source_stamp_ns,target_stamp_ns]):raise ValueError('Typed prediction timestamps required')
        dt=(target_stamp_ns-source_stamp_ns)*1e-9
        if not 0<=dt<=.060001+self.command_delay_s:raise ValueError('Projection limited to fresh40ms input plus explicit selection/command delay')
        if not self.commands:
            # Startup shadow computation only, before the first selected commit.
            self.history.reset(source_stamp_ns-100_000_000)
        hidden=self.history.state_at(source_stamp_ns)
        x=np.r_[s[:10],hidden]
        times=[source_stamp_ns]+[t for t,_ in self.commands if source_stamp_ns<t<target_stamp_ns]+[target_stamp_ns]
        for begin,end in zip(times,times[1:]):
            h_total=(end-begin)*1e-9;steps=max(1,int(math.ceil(h_total/.005)));h=h_total/steps;u=self.held(begin)
            for _ in range(steps):
                k1=self.derivative(x,u);k2=self.derivative(x+h*k1/2,u);k3=self.derivative(x+h*k2/2,u);k4=self.derivative(x+h*k3,u)
                x+=h*(k1+2*k2+2*k3+k4)/6;x[3:7]/=np.linalg.norm(x[3:7])
        # World rates only complete the existing state contract; MPC receives
        # the six nominal response states explicitly and never consumes mocap rates.
        from scipy.spatial.transform import Rotation
        world_rate=Rotation.from_quat(x[[4,5,6,3]]).apply(self.C@x[10:])
        self.last=dict(source_stamp_ns=source_stamp_ns,target_stamp_ns=target_stamp_ns,projection_s=dt,selected_history_only=True,nominal_command_delay_s=self.command_delay_s,
                       source_state=s.tolist(),predicted_state=np.r_[x[:10],world_rate].tolist(),rate_model_state=x[10:].tolist())
        return np.r_[x[:10],world_rate],x[10:].copy()
