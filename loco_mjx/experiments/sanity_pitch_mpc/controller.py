"""Explicitly scoped full-attitude CTBR MPC; transport is owned by its runner."""
import argparse
import contextlib
import ctypes
import fcntl
import hashlib
import json
import os
import platform
from pathlib import Path
from scipy.spatial.transform import Rotation
import sys
import time

import casadi as ca
import numpy as np


class PitchLoopMPC:
    def __init__(self, config):
        if config.get('scope') not in ('simulation_only_pitch_loop','experimental_pitch_loop'):raise ValueError('Explicit simulation or experimental pitch-loop scope required')
        self.config=config
        self.mass=config['mass_kg'];self.inertia=np.asarray(config['inertia_kg_m2']);self.inverse_inertia=np.linalg.inv(self.inertia)
        self.nodes=config.get('nodes',20);self.dt=config.get('prediction_dt_s',.04)
        self.bounds=np.array([[0.,config.get('maximum_thrust_N',2.85)],[-2.3,2.3],[-config.get('pitch_limit_rad_s',9.5),config.get('pitch_limit_rad_s',9.5)],[-2.6,2.6]])
        if not 5<=self.nodes<=50 or not .01<=self.dt<=.05 or not .1<self.bounds[2,1]<=np.radians(600):raise ValueError('Invalid simulation horizon/rate limit')
        self.preview_times=np.arange(self.nodes+1)*self.dt
        self.iterations=config.get('rti_iterations',2)
        mode=config.get('rate_prediction','legacy')
        if mode not in ('legacy','flight'):raise ValueError('Unknown pitch rate prediction mode')
        self.flight_rate=None
        if mode=='flight':
            from flight_prediction import FlightRatePrediction
            self.flight_rate=FlightRatePrediction(config)
        self.nx=22 if self.flight_rate is not None else 14
        x=ca.SX.sym('x',self.nx);u=ca.SX.sym('u',4);q=x[3:7];w=x[10:13]
        rotation=ca.horzcat(ca.vertcat(1-2*(q[2]**2+q[3]**2),2*(q[1]*q[2]+q[0]*q[3]),2*(q[1]*q[3]-q[0]*q[2])),
            ca.vertcat(2*(q[1]*q[2]-q[0]*q[3]),1-2*(q[1]**2+q[3]**2),2*(q[2]*q[3]+q[0]*q[1])),
            ca.vertcat(2*(q[1]*q[3]+q[0]*q[2]),2*(q[2]*q[3]-q[0]*q[1]),1-2*(q[1]**2+q[2]**2)))
        qdot=.5*ca.vertcat(-ca.dot(q[1:],w),q[0]*w+ca.cross(q[1:],w))
        extra=[]
        if self.flight_rate is None:
            desired=ca.DM(config['rate_kv'])*(u[1:]-w)
        else:
            acceleration=[];lowpass=[];delay_derivatives=[]
            for axis,(gain,wn,damping,zero,delay_tau) in enumerate(self.flight_rate.coefficients):
                filtered=x[14+axis];delays=x[16+axis*3:19+axis*3]
                derivative=(w[axis]/gain-filtered)/zero
                acceleration.append(gain*((1-2*damping*wn*zero)*derivative+zero*wn**2*(delays[2]-filtered)))
                lowpass.append(derivative)
                delay_derivatives.extend([(u[axis+1]-delays[0])/delay_tau,(delays[0]-delays[1])/delay_tau,(delays[1]-delays[2])/delay_tau])
            yaw_acceleration=config['rate_kv'][2]*(u[3]-w[2])/self.inertia[2,2]
            desired=ca.DM(self.inertia)@ca.vertcat(*acceleration,yaw_acceleration)+ca.cross(w,ca.DM(self.inertia)@w)
            extra=[*lowpass,*delay_derivatives]
        limits=ca.DM(config['torque_limits_Nm'])
        torque=ca.fmin(ca.fmax(desired,-limits),limits)
        deviations=ca.DM(config['allocation_per_m'])@torque
        self.force_range=np.asarray(config['physical_force_range_N'],dtype=float)
        if self.force_range.shape!=(2,) or not np.isfinite(self.force_range).all() or not 0<=self.force_range[0]<self.force_range[1]:raise ValueError('Finite ascending physical force bounds required')
        force_parameters=ca.SX.sym('force_bounds',2);force_min,force_max=force_parameters[0],force_parameters[1]
        capacity=config['torque_reserve_fraction']*ca.fmax(ca.fmin(x[13]-force_min,force_max-x[13]),0.)/4
        scale=ca.fmin(1.,capacity/ca.fmax(ca.mmax(ca.fabs(deviations)),1e-8))
        omega_dot=ca.DM(self.inverse_inertia)@(torque*scale-ca.cross(w,ca.DM(self.inertia)@w))
        target=ca.fmin(ca.fmax(u[0],0.),force_max)
        tau=ca.if_else(target>=x[13],config['collective_tau_up_s'],config['collective_tau_down_s'])
        parameterized_dynamics=ca.Function('pitch_dynamics',[x,u,force_parameters],[ca.vertcat(x[7:10],qdot,rotation[:,2]*x[13]/self.mass-ca.DM([0.,0.,9.81]),omega_dot,(target-x[13])/tau,*extra)])
        self.dynamics=ca.Function('bound_pitch_dynamics',[x,u],[parameterized_dynamics(x,u,ca.DM(self.force_range))])
        next_x=x
        substeps=max(2,int(np.ceil(self.dt/.005))) if self.flight_rate is not None else 2
        for _ in range(substeps):
            h=self.dt/substeps;k1=parameterized_dynamics(next_x,u,force_parameters);k2=parameterized_dynamics(next_x+h*k1/2,u,force_parameters);k3=parameterized_dynamics(next_x+h*k2/2,u,force_parameters);k4=parameterized_dynamics(next_x+h*k3,u,force_parameters)
            next_x=next_x+h*(k1+2*k2+2*k3+k4)/6
            next_x=ca.vertcat(next_x[:3],next_x[3:7]/ca.norm_2(next_x[3:7]),next_x[7:])
        self.step=ca.Function('pitch_step',[x,u],[ca.substitute(next_x,force_parameters,ca.DM(self.force_range))]);self.rollout=self.step.mapaccum(self.nodes)
        import acados_template
        from acados_template import AcadosModel,AcadosOcp,AcadosOcpSolver
        os.environ.setdefault('ACADOS_SOURCE_DIR',str(Path(acados_template.__file__).resolve().parents[3]))
        ocp=AcadosOcp();lib=Path(ocp.code_gen_options.acados_lib_path)
        for name in ('libblasfeo.so','libhpipm.so','libacados.so'):ctypes.CDLL(str(lib/name),mode=ctypes.RTLD_GLOBAL)
        compile_fields=('mass_kg','inertia_kg_m2','rate_kv','torque_limits_Nm','allocation_per_m','torque_reserve_fraction',
            'collective_tau_up_s','collective_tau_down_s','nodes','prediction_dt_s','rate_prediction','rate_prediction_timestamp',
            'state_cost','control_cost','terminal_multiplier','solver_type','rti_iterations')
        key={'configuration':{k:config[k] for k in compile_fields if k in config},
            'flight_rate_profile_sha256':self.flight_rate.info['sha256'] if self.flight_rate is not None else None,
            'flight_prediction_source_sha256':hashlib.sha256(Path(__file__).with_name('flight_prediction.py').read_bytes()).hexdigest() if self.flight_rate is not None else None,
            'source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'acados_sha256':hashlib.sha256((lib/'libacados.so').read_bytes()).hexdigest(),
            'casadi_version':ca.__version__,'machine':platform.machine(),'runtime_parameters':'reference18,physical_force_min_max2; input bounds set on load'}
        digest=hashlib.sha256(json.dumps(key,sort_keys=True).encode()).hexdigest()
        cache=Path.home()/'.cache/sanity_pitch_mpc'/digest;cache.mkdir(parents=True,exist_ok=True)
        ocp.model=AcadosModel();ocp.model.name='pitch_loop_'+digest[:12];ocp.name=ocp.model.name
        p=ca.SX.sym('reference_and_force_bounds',20);qr=p[3:7]
        attitude=2*(qr[0]*q[1:]-q[0]*qr[1:]-ca.cross(qr[1:],q[1:]))
        residual=ca.vertcat(x[:3]-p[:3],x[7:10]-p[7:10],attitude,w-p[10:13],x[13]-p[13])
        ocp.model.x=x;ocp.model.u=u;ocp.model.p=p;ocp.model.disc_dyn_expr=ca.substitute(next_x,force_parameters,p[18:20])
        ocp.model.cost_y_expr=ca.vertcat(residual,u-p[14:18]);ocp.model.cost_y_expr_e=residual
        weights=np.array(config.get('state_cost',[80.,80.,120.,8.,8.,12.,12.,12.,12.,.1,.1,.1,.1]))
        self.control_cost=config.get('control_cost',[.01,.03,.03,.03])
        ocp.cost.cost_type='NONLINEAR_LS';ocp.cost.cost_type_e='NONLINEAR_LS'
        ocp.cost.W=2*np.diag(np.r_[weights,self.control_cost]);ocp.cost.W_e=2*config.get('terminal_multiplier',2.)*np.diag(weights)
        ocp.cost.yref=np.zeros(17);ocp.cost.yref_e=np.zeros(13)
        hover=self.augment(np.r_[0.,0.,.5,1.,0.,0.,0.,np.zeros(6),self.mass*9.81])
        ocp.parameter_values=np.r_[hover[:14],self.mass*9.81,0.,0.,0.,self.force_range]
        ocp.constraints.idxbu=np.arange(4);ocp.constraints.lbu=self.bounds[:,0];ocp.constraints.ubu=self.bounds[:,1]
        ocp.constraints.x0=hover
        ocp.constraints.idxbx=np.array([2]);ocp.constraints.lbx=np.array([.12]);ocp.constraints.ubx=np.array([2.])
        ocp.constraints.idxbx_e=np.array([2]);ocp.constraints.lbx_e=np.array([.12]);ocp.constraints.ubx_e=np.array([2.])
        ocp.solver_options.N_horizon=self.nodes;ocp.solver_options.tf=self.nodes*self.dt
        ocp.solver_options.integrator_type='DISCRETE';ocp.solver_options.nlp_solver_type=config.get('solver_type','SQP_RTI')
        if config.get('solver_type')=='SQP':
            ocp.solver_options.nlp_solver_max_iter=20
            ocp.solver_options.nlp_solver_tol_stat=1e-3
            ocp.solver_options.nlp_solver_tol_eq=1e-5
            ocp.solver_options.nlp_solver_tol_ineq=1e-5
            ocp.solver_options.nlp_solver_tol_comp=1e-5
            ocp.solver_options.globalization='MERIT_BACKTRACKING'
            self.iterations=1
        ocp.solver_options.hessian_approx='GAUSS_NEWTON';ocp.solver_options.qp_solver='PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.qp_solver_cond_N=self.nodes;ocp.solver_options.qp_solver_iter_max=100
        ocp.solver_options.cost_scaling=np.ones(self.nodes+1);ocp.solver_options.print_level=0
        ocp.code_gen_options.code_export_directory=str(cache/'generated');ocp.code_gen_options.json_file='acados_ocp.json'
        with (cache/'build.lock').open('w') as lock,contextlib.redirect_stdout(sys.stderr):
            fcntl.flock(lock,fcntl.LOCK_EX)
            built=(cache/'build.json').exists() and (cache/'generated'/('libacados_ocp_solver_'+ocp.model.name+'.so')).exists()
            print(('Loading cached' if built else 'Generating and compiling')+' pitch MPC solver: '+str(cache),file=sys.stderr,flush=True)
            self.solver=AcadosOcpSolver(ocp,generate=not built,build=not built,verbose=False,check_reuse_possible=False)
            (cache/'build.json').write_text(json.dumps(key,indent=2)+'\n')
        for i in range(self.nodes):
            self.solver.constraints_set(i,'lbu',self.bounds[:,0])
            self.solver.constraints_set(i,'ubu',self.bounds[:,1])
        self.previous=None;self.last={}
        self.info={'scope':config['scope'],'solver':'acados/'+config.get('solver_type','SQP_RTI')+'/HPIPM','configuration':config,'cache':str(cache),'cache_hit':bool(built),'runtime_force_bounds_N':self.force_range.tolist(),
            'state':'p_world3,q_wxyz4,v_world3,omega_body3,collective_force_N1; flight mode adds2 rate low-pass and6 internal-delay states','internal_state_size':self.nx,'preview':'state14,feedforward_CTBR4',
            'prediction':'full attitude; collective lag; identified roll/pitch response in flight mode, assumed servo in legacy mode; physical torque/headroom limits',
            'flight_rate_prediction':self.flight_rate.info if self.flight_rate is not None else None,
            'not_identified':'High-rate dynamics remain a simulation-only extrapolation of provisional rate servo assumptions.',
            'no_hardware_interface':True,'bounds':self.bounds.tolist()}


    def augment(self,state,rate_observer=None):
        value=np.asarray(state,dtype=float)
        if value.shape==(self.nx,):return value.copy()
        if value.shape!=(14,):raise ValueError('Expected measured state14 or complete prediction state')
        if self.flight_rate is None:return value.copy()
        if rate_observer is None:
            gains=np.array([v[0] for v in self.flight_rate.coefficients]);lowpass=value[10:12]/gains
            return np.r_[value,lowpass,np.repeat(lowpass,3)]
        if rate_observer['profile_sha256']!=self.flight_rate.info['sha256']:raise ValueError('Flight-rate observer/prediction profile mismatch')
        latent=np.asarray(rate_observer['state'],dtype=float)
        if latent.shape!=(2,5) or not np.isfinite(latent).all():raise ValueError('Finite flight-rate observer state[2,5] required')
        value=value.copy();value[10:12]=np.sum(self.flight_rate.c[:,:5]*latent,axis=1)
        return np.r_[value,latent[:,0],latent[:,2:5].ravel()]


    def forecast(self,state,command,duration):
        value=np.array(state,dtype=float,copy=True)
        steps=max(1,int(np.ceil(duration/.005)));h=duration/steps
        for _ in range(steps):
            k1=np.asarray(self.dynamics(value,command)).ravel()
            k2=np.asarray(self.dynamics(value+h*k1/2,command)).ravel()
            k3=np.asarray(self.dynamics(value+h*k2/2,command)).ravel()
            k4=np.asarray(self.dynamics(value+h*k3,command)).ravel()
            value+=h*(k1+2*k2+2*k3+k4)/6
            value[3:7]/=np.linalg.norm(value[3:7])
        return value


    def command(self,state,preview):
        started=time.perf_counter();initial=self.augment(state);refs=np.array(preview,dtype=float,copy=True)
        if initial.shape!=(self.nx,) or refs.shape!=(self.nodes+1,18) or not np.isfinite(initial).all() or not np.isfinite(refs).all():raise ValueError('Finite state14/preview[N+1,18] required')
        if abs(np.linalg.norm(initial[3:7])-1.)>.01 or np.max(abs(np.linalg.norm(refs[:,3:7],axis=1)-1.))>.01:raise ValueError('Unit body-to-world quaternions required')
        initial[3:7]/=np.linalg.norm(initial[3:7]);refs[:,3:7]/=np.linalg.norm(refs[:,3:7],axis=1)[:,None]
        controls=np.clip(refs[:-1,14:18],self.bounds[:,0],self.bounds[:,1])
        if self.previous is not None:
            for axis in range(4):controls[:,axis]=np.interp(self.preview_times[:-1]+.02,self.preview_times[:-1],self.previous[:,axis])
        rollout=np.c_[initial,np.asarray(self.rollout(initial,controls.T))].T
        for i in range(self.nodes+1):
            self.solver.set(i,'x',rollout[i]);self.solver.set(i,'p',np.r_[refs[i],self.force_range])
            if i<self.nodes:self.solver.set(i,'u',controls[i])
        self.solver.set(0,'lbx',initial);self.solver.set(0,'ubx',initial)
        core=0.;status=0
        for _ in range(self.iterations):
            before=time.perf_counter();status=self.solver.solve();core+=time.perf_counter()-before
            if status:break
        optimal=np.array([self.solver.get(i,'u') for i in range(self.nodes)])
        if not np.isfinite(optimal).all():
            self.previous=None
            self.last={'valid':False,'status':int(status),'total_ms':(time.perf_counter()-started)*1000,'core_ms':core*1000,'reason':'nonfinite_solver_controls'}
            return np.zeros(4)
        predicted=np.c_[initial,np.asarray(self.rollout(initial,optimal.T))].T
        if not np.isfinite(predicted).all():
            self.previous=None
            self.last={'valid':False,'status':int(status),'total_ms':(time.perf_counter()-started)*1000,'core_ms':core*1000,'reason':'nonfinite_nonlinear_rollout'}
            return np.zeros(4)
        violation=max(float(np.max(self.bounds[:,0]-optimal)),float(np.max(optimal-self.bounds[:,1])),0.)
        raw=np.array([self.solver.get(i,'x') for i in range(self.nodes+1)])
        dynamic_defect=float(np.max(abs(raw-predicted)))
        residuals=self.solver.get_residuals().tolist()
        total=(time.perf_counter()-started)*1000
        height_violation=max(float(.12-predicted[1:,2].min()),float(predicted[1:,2].max()-2.),0.)
        valid=status==0 and np.isfinite(optimal).all() and np.isfinite(predicted).all() and violation<1e-4 and height_violation<1e-4 and total<=25.
        self.last={'valid':bool(valid),'status':int(status),'total_ms':total,'core_ms':core*1000,'input_violation':violation,'rollout_vs_raw_state_max_error':dynamic_defect,'nlp_residuals':residuals,'height_violation':height_violation,'within_25ms':total<=25.,
            'predicted_height_min_m':float(predicted[:,2].min()),'predicted_height_max_m':float(predicted[:,2].max())}
        self.previous=optimal if valid else None
        return np.clip(optimal[0],self.bounds[:,0],self.bounds[:,1])


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--config',type=Path,required=True);args=parser.parse_args()
    protocol=os.fdopen(os.dup(sys.stdout.fileno()),'w',buffering=1)
    os.dup2(sys.stderr.fileno(),sys.stdout.fileno())
    with contextlib.redirect_stdout(sys.stderr):
        controller=PitchLoopMPC(json.loads(args.config.read_text()))
        state=np.r_[0.,0.,.5,1.,0.,0.,0.,np.zeros(6),controller.mass*9.81]
        preview=np.tile(np.r_[state,controller.mass*9.81,0.,0.,0.],(controller.nodes+1,1))
        for _ in range(4):controller.command(state,preview)
        controller.previous=None
    print(json.dumps(controller.info),file=protocol,flush=True)
    for line in sys.stdin:
        request=json.loads(line)
        if request['op']=='close':break
        if request['op']=='reset':controller.previous=None;controller.solver.reset();reply={'reset':True}
        else:
            prepare_started=time.perf_counter()
            prepared=controller.augment(request['state'],request.get('rate_observer'))
            pose_delay=request.get('pose_filter_delay_s',{'position':0.,'attitude':0.})
            if not all(np.isfinite(pose_delay[k]) and 0<=pose_delay[k]<=.04 for k in ('position','attitude')):raise ValueError('Invalid pose-filter prediction delay')
            if pose_delay['attitude']:
                rotation=Rotation.from_quat(prepared[[4,5,6,3]])*Rotation.from_rotvec(pose_delay['attitude']*prepared[10:13])
                prepared[3:7]=rotation.as_quat()[[3,0,1,2]]
            compensation=request.get('filter_compensation_s',0.)
            if compensation:
                derivative=np.asarray(controller.dynamics(prepared,request['sample_command'])).ravel()
                prepared[7:10]+=compensation*derivative[7:10]
                if controller.flight_rate is None:prepared[10:13]+=compensation*derivative[10:13]
            prepared[:3]+=pose_delay['position']*prepared[7:10]
            for duration,selected in request.get('prediction_segments',[]):
                prepared=controller.forecast(prepared,selected,duration)
            preparation_ms=(time.perf_counter()-prepare_started)*1000
            command=controller.command(prepared,request['preview'])
            controller.last['observation_preparation_ms']=preparation_ms
            controller.last['total_ms']+=preparation_ms
            controller.last['within_25ms']=controller.last['total_ms']<=25.
            controller.last['valid']=controller.last['valid'] and controller.last['within_25ms']
            reply={'ctbr':command.tolist(),'mpc':controller.last,'prepared_state':prepared.tolist()}
        print(json.dumps(reply,allow_nan=False),file=protocol,flush=True)
