"""Compact trajectory-to-CTBR NMPC, inspired by the Agilicious formulation."""

import contextlib
import ctypes
import fcntl
import hashlib
import json
import os
import platform
import sys
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import casadi as ca
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).resolve().parent))
from mpc_disturbance import HorizontalAccelerationBias, validate_configuration
from indi_rate_model import validate_configuration as validate_rate_model, continuous_matrices


class MpcSolveError(RuntimeError):
    """No successful, feasible RTI command within the configured runtime budget."""


class CtbrMPC:
    def __init__(self, model_path, configuration):
        self.model_path = Path(model_path).resolve()
        model = ET.parse(self.model_path).getroot()
        if model.findall('include'):
            raise ValueError('MPC requires the complete, non-included Sanity CTBR XML')
        self.mass = sum(float(v.attrib['mass']) for v in model.findall('.//body/inertial'))
        motors = model.find('actuator')
        expected = ['body_thrust', 'roll_rate', 'pitch_rate', 'yaw_rate']
        if motors is None or [v.attrib.get('name') for v in motors] != expected:
            raise ValueError('Expected the Sanity CTBR actuator order')
        self.bounds = np.array([np.fromstring(v.attrib['ctrlrange'], sep=' ') for v in motors])
        gravity = np.fromstring(model.find('option').attrib.get('gravity', '0 0 -9.81'), sep=' ')
        if not np.allclose(gravity[:2], 0) or gravity[2] >= 0:
            raise ValueError('Expected world Z up with downward gravity')
        self.gravity = -float(gravity[2])
        allowed = {'nodes', 'prediction_dt_s', 'position_weights', 'velocity_weights', 'attitude_weights',
                   'input_weights', 'terminal_multiplier', 'tilt_limit_deg', 'drag_per_s', 'qp_max_iterations', 'max_solve_time_s', 'disturbance_compensation', 'rate_response', 'thrust_effectiveness', 'command_delay_s', 'rate_acceleration_weights'}
        if set(configuration) - allowed:
            raise ValueError('Unknown MPC configuration keys: ' + str(set(configuration) - allowed))
        self.nodes = configuration.get('nodes', 10)
        self.dt = configuration.get('prediction_dt_s', .1)
        self.tilt_limit = configuration.get('tilt_limit_deg', 30.)
        self.qp_max_iterations = configuration.get('qp_max_iterations', 50)
        self.max_solve_time = configuration.get('max_solve_time_s', .025)
        self.terminal_multiplier = configuration.get('terminal_multiplier', 2.)
        self.qp = np.array(configuration.get('position_weights', [30., 30., 60.]))
        self.qv = np.array(configuration.get('velocity_weights', [4., 4., 6.]))
        self.qa = np.array(configuration.get('attitude_weights', [1., 1., 2.]))
        self.ru = np.array(configuration.get('input_weights', [.02, .1, .1, .1]))
        self.drag = np.array(configuration.get('drag_per_s', [0., 0., 0.]))
        self.rate_acceleration_weights=np.asarray(configuration.get('rate_acceleration_weights',[0.,0.,0.]),dtype=float)
        if self.rate_acceleration_weights.shape!=(3,) or not np.isfinite(self.rate_acceleration_weights).all() or np.any(self.rate_acceleration_weights<0) or np.any(self.rate_acceleration_weights>.1):raise ValueError('Invalid rate acceleration weights')
        self.disturbance_configuration=validate_configuration(configuration.get('disturbance_compensation',{}))
        self.disturbance_enabled=self.disturbance_configuration['enabled']
        self.rate_configuration=validate_rate_model(configuration.get('rate_response',{}))
        self.rate_enabled=self.rate_configuration['enabled']
        self.maximum_feasibility_attempts=3 if self.rate_enabled else 1
        self.nx=16 if self.rate_enabled else 10
        self.thrust_effectiveness=configuration.get('thrust_effectiveness',1.)
        self.command_delay_s=configuration.get('command_delay_s',0.)
        if isinstance(self.command_delay_s,bool) or not np.isfinite(self.command_delay_s) or not 0<=self.command_delay_s<=.03:raise ValueError('Command delay must be finite in0..30ms')
        if isinstance(self.thrust_effectiveness,bool) or not np.isfinite(self.thrust_effectiveness) or not .8<=self.thrust_effectiveness<=1.2:
            raise ValueError('Explicit thrust effectiveness must be finite in0.8..1.2')
        if not isinstance(self.nodes, int) or not 5 <= self.nodes <= 40 or not isinstance(self.qp_max_iterations, int) or not 1 <= self.qp_max_iterations <= 100:
            raise ValueError('MPC nodes must be integer [5,40]; QP iterations integer [1,100]')
        if not np.isfinite([self.mass, self.dt, self.gravity, self.tilt_limit, self.max_solve_time, self.terminal_multiplier]).all() or not 0 < self.mass < 1 or not .02 <= self.dt <= .1 or not 5 < self.tilt_limit <= 75 or not .001 <= self.max_solve_time <= .035 or self.terminal_multiplier <= 0:
            raise ValueError('Invalid mass, horizon, tilt or runtime budget')
        for value, size in [(self.qp, 3), (self.qv, 3), (self.qa, 3), (self.ru, 4), (self.drag, 3)]:
            if value.shape != (size,) or not np.isfinite(value).all() or np.any(value < 0):
                raise ValueError('Finite nonnegative MPC weights/drag required')
        if self.bounds.shape != (4, 2) or not np.isfinite(self.bounds).all() or np.any(self.bounds[:, 0] >= self.bounds[:, 1]) or not self.bounds[0, 0] <= self.mass * self.gravity <= self.bounds[0, 1]:
            raise ValueError('Invalid CTBR bounds or hover outside thrust envelope')
        if np.any(self.qp<=0) or np.any(self.ru<=0):
            raise ValueError("Position and input weights must be positive")
        self.preview_times = np.arange(self.nodes + 1) * self.dt
        x = ca.SX.sym('x', self.nx)
        u = ca.SX.sym('u', 4)
        bias = ca.SX.sym('world_acceleration_bias',3)
        q = x[3:7]
        if self.rate_enabled:
            rate_A,rate_B,rate_C=continuous_matrices(self.rate_configuration)
            w=ca.DM(rate_C)@x[10:16]
            rate_dot=ca.DM(rate_A)@x[10:16]+ca.DM(rate_B)@u[1:4]
        else:w=u[1:4]
        qdot = .5 * ca.vertcat(-ca.dot(q[1:4], w), q[0] * w + ca.cross(q[1:4], w))
        rotation = ca.horzcat(
            ca.vertcat(1-2*(q[2]**2+q[3]**2), 2*(q[1]*q[2]+q[0]*q[3]), 2*(q[1]*q[3]-q[0]*q[2])),
            ca.vertcat(2*(q[1]*q[2]-q[0]*q[3]), 1-2*(q[1]**2+q[3]**2), 2*(q[2]*q[3]+q[0]*q[1])),
            ca.vertcat(2*(q[1]*q[3]+q[0]*q[2]), 2*(q[2]*q[3]-q[0]*q[1]), 1-2*(q[1]**2+q[2]**2)))
        acceleration = rotation[:, 2] * (self.thrust_effectiveness*u[0]) - ca.DM([0., 0., self.gravity]) - rotation @ (ca.DM(self.drag) * (rotation.T @ x[7:10])) + bias
        derivative=ca.vertcat(x[7:10],qdot,acceleration,rate_dot) if self.rate_enabled else ca.vertcat(x[7:10],qdot,acceleration)
        dynamics = ca.Function('ctbr_dynamics', [x, u, bias], [derivative])
        # Resolve the identified rate poles inside each prediction interval.
        # Legacy ideal-rate predictions retain their original integration.
        substeps=max(1,int(np.ceil(self.dt/.01))) if self.rate_enabled else 1
        h=self.dt/substeps;next_x=x
        for _ in range(substeps):
            k1=dynamics(next_x,u,bias);k2=dynamics(next_x+h*k1/2,u,bias)
            k3=dynamics(next_x+h*k2/2,u,bias);k4=dynamics(next_x+h*k3,u,bias)
            next_x=next_x+h*(k1+2*k2+2*k3+k4)/6
        next_x[3:7] /= ca.norm_2(next_x[3:7])
        nominal_next = ca.substitute(next_x,bias,ca.SX.zeros(3,1))
        self.step = ca.Function('ctbr_step', [x, u], [nominal_next])
        self.rollout = self.step.mapaccum(self.nodes)
        self.step_batch = self.step.map(self.nodes)
        self.disturbance_step = ca.Function('ctbr_step_bias',[x,u,bias],[next_x])
        self.disturbance_rollout = self.disturbance_step.mapaccum(self.nodes)
        self.disturbance_step_batch = self.disturbance_step.map(self.nodes)
        try:
            import acados_template
            from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
        except ImportError as error:
            raise RuntimeError('acados is required; run Sanity/src/ros2_sanity_ws/scripts/install_sanity_acados.py') from error
        os.environ.setdefault('ACADOS_SOURCE_DIR', str(Path(acados_template.__file__).resolve().parents[3]))
        ocp = AcadosOcp()
        lib_path = Path(ocp.code_gen_options.acados_lib_path)
        self.acados_library_sha256 = hashlib.sha256((lib_path/'libacados.so').read_bytes()).hexdigest()
        for library in ('libblasfeo.so', 'libhpipm.so', 'libacados.so'):
            ctypes.CDLL(str(lib_path/library), mode=ctypes.RTLD_GLOBAL)
        self.acados_source = str(Path(os.environ['ACADOS_SOURCE_DIR']).resolve())
        install_receipt = Path(self.acados_source)/'sanity_install.json'
        self.acados_build = json.loads(install_receipt.read_text()) if install_receipt.exists() else {'source':self.acados_source,'version':'unrecorded external build'}
        key = {'source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'model_sha256':hashlib.sha256(self.model_path.read_bytes()).hexdigest(), 'configuration':configuration,
            'rate_model_source_sha256':hashlib.sha256(Path(__file__).with_name('indi_rate_model.py').read_bytes()).hexdigest(),
            'acados_library_sha256':self.acados_library_sha256, 'acados_source':self.acados_source,
            'casadi':ca.__version__, 'machine':platform.machine()}
        self.cache_key = hashlib.sha256(json.dumps(key,sort_keys=True).encode()).hexdigest()
        cache_root = Path(os.environ.get('SANITY_MPC_CACHE_DIR', str(Path(os.environ.get('XDG_CACHE_HOME',Path.home()/'.cache'))/'sanity_ctbr_mpc')))
        cache = cache_root/self.cache_key
        cache.mkdir(parents=True, exist_ok=True)
        ocp.model = AcadosModel()
        ocp.model.name = 'sanity_ctbr_' + self.cache_key[:12]
        ocp.name = ocp.model.name
        reference = ca.SX.sym('reference',14)
        ocp.model.x = x
        ocp.model.u = u
        ocp.model.p = ca.vertcat(reference,bias) if self.disturbance_enabled else reference
        ocp.model.disc_dyn_expr = next_x if self.disturbance_enabled else nominal_next
        qr = reference[3:7]; q = x[3:7]
        attitude_error = qr[0]*q[1:4]-q[0]*qr[1:4]-ca.cross(qr[1:4],q[1:4])
        residual = ca.vertcat(x[:3]-reference[:3], x[7:10]-reference[7:10], 2*attitude_error)
        regularize_rate=self.rate_enabled and np.any(self.rate_acceleration_weights)
        if regularize_rate:
            modeled_alpha=ca.DM(rate_C)@(ca.DM(rate_A)@x[10:16]+ca.DM(rate_B)@u[1:4])
            ocp.model.cost_y_expr=ca.vertcat(residual,u-reference[10:14],modeled_alpha)
        else:ocp.model.cost_y_expr = ca.vertcat(residual, u-reference[10:14])
        ocp.model.cost_y_expr_e = residual
        ocp.cost.cost_type = 'NONLINEAR_LS'
        ocp.cost.cost_type_e = 'NONLINEAR_LS'
        ocp.cost.W = 2*np.diag(np.r_[self.qp,self.qv,self.qa,self.ru,self.rate_acceleration_weights] if regularize_rate else np.r_[self.qp,self.qv,self.qa,self.ru])
        ocp.cost.W_e = 2*self.terminal_multiplier*np.diag(np.r_[self.qp,self.qv,self.qa])
        ocp.cost.yref = np.zeros(16 if regularize_rate else 13)
        ocp.cost.yref_e = np.zeros(9)
        ocp.parameter_values = np.r_[0.,0.,1.,1.,0.,0.,0.,0.,0.,0.,self.gravity/self.thrust_effectiveness,0.,0.,0.]
        if self.disturbance_enabled:ocp.parameter_values=np.r_[ocp.parameter_values,np.zeros(3)]
        cb = self.bounds.copy(); cb[0] /= self.mass
        ocp.constraints.idxbu = np.arange(4)
        ocp.constraints.lbu = cb[:,0]
        ocp.constraints.ubu = cb[:,1]
        ocp.constraints.x0 = np.r_[0.,0.,1.,1.,0.,0.,0.,0.,0.,0.,np.zeros(self.nx-10)]
        ocp.model.con_h_expr = 1-2*(x[4]**2+x[5]**2)
        ocp.model.con_h_expr_e = ocp.model.con_h_expr
        ocp.model.con_h_expr_0 = ca.SX.zeros(0,1)
        ocp.constraints.lh = np.array([np.cos(np.radians(self.tilt_limit))])
        ocp.constraints.uh = np.ones(1)
        ocp.constraints.lh_e = ocp.constraints.lh.copy()
        ocp.constraints.uh_e = np.ones(1)
        ocp.solver_options.N_horizon = self.nodes
        ocp.solver_options.tf = self.nodes*self.dt
        ocp.solver_options.integrator_type = 'DISCRETE'
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.qp_solver_cond_N = self.nodes
        ocp.solver_options.qp_solver_iter_max = self.qp_max_iterations
        ocp.solver_options.qp_solver_tol_stat = 1e-6
        ocp.solver_options.qp_solver_tol_eq = 1e-6
        ocp.solver_options.qp_solver_tol_ineq = 1e-6
        ocp.solver_options.qp_solver_tol_comp = 1e-6
        ocp.solver_options.cost_scaling = np.ones(self.nodes+1)
        ocp.solver_options.print_level = 0
        ocp.code_gen_options.code_export_directory = str(cache/'generated')
        ocp.code_gen_options.json_file = 'acados_ocp.json'
        with (cache/'build.lock').open('w') as lock, contextlib.redirect_stdout(sys.stderr):
            fcntl.flock(lock,fcntl.LOCK_EX)
            built = (cache/'build.json').exists() and (cache/'generated'/('libacados_ocp_solver_'+ocp.model.name+'.so')).exists()
            self.solver = AcadosOcpSolver(ocp, generate=not built, build=not built, verbose=False, check_reuse_possible=False)
            (cache/'build.json').write_text(json.dumps(key,indent=2)+'\n')
        self.cache_path = str(cache)
        self.previous_controls = None
        self.last = {}
        self.info = {'controller': 'mpc', 'implementation': 'Agilicious-inspired CTBR NMPC; generated acados C, direct CTBR model',
            'solver': 'acados/SQP_RTI/HPIPM', 'hessian': 'GAUSS_NEWTON',
            'acados_source':self.acados_source, 'acados_library_sha256':self.acados_library_sha256, 'acados_build':self.acados_build,
            'generated_solver_cache':self.cache_path, 'generated_solver_key':self.cache_key,
            'wrapper':'Python ctypes; optimization and linear algebra in compiled C', 'casadi_version': ca.__version__, 'model_path': str(self.model_path),
            'model_sha256': hashlib.sha256(self.model_path.read_bytes()).hexdigest(), 'mass_kg': self.mass,
            'ctrlrange': self.bounds.tolist(), 'policy_dt_s': .02, 'history_length_steps': 0, 'imu_position': [0., 0., 0.],
            'reference_point': 'root', 'state': 'p_world,q_wxyz,v_world plus6 causal command-response states' if self.rate_enabled else 'p_world,q_wxyz,v_world; ideal body rates',
            'prediction_state_count':self.nx, 'rate_response_configuration':self.rate_configuration, 'thrust_effectiveness':self.thrust_effectiveness, 'command_delay_s':self.command_delay_s,
            'action': 'total thrust N; body rates rad/s, x-forward y-left z-up', 'configuration': {'nodes':self.nodes,'prediction_dt_s':self.dt,'tilt_limit_deg':self.tilt_limit,
                'qp_max_iterations':self.qp_max_iterations,'max_solve_time_s':self.max_solve_time,'terminal_multiplier':self.terminal_multiplier,
                'position_weights':self.qp.tolist(),'velocity_weights':self.qv.tolist(),'attitude_weights':self.qa.tolist(),
                'input_weights':self.ru.tolist(),'drag_per_s':self.drag.tolist(),'rate_acceleration_weights':self.rate_acceleration_weights.tolist(),'disturbance_compensation':self.disturbance_configuration},
            'disturbance_compensation':{**self.disturbance_configuration,'observer_source_sha256':hashlib.sha256(Path(__file__).with_name('mpc_disturbance.py').read_bytes()).hexdigest(),'frame':'model world XY','units':'m/s^2','scope':'Bounded effective constant acceleration bias; not identified drag or actuator dynamics'},
            'prediction_dt_s': self.dt, 'nodes': self.nodes, 'horizon_s': self.nodes*self.dt,
            'preview_times_s': self.preview_times.tolist(), 'rate_response': 'Identified second-order roll/pitch response; conservative first-order yaw' if self.rate_enabled else 'Ideal body-rate tracking; no identified FC rate dynamics or delay',
            'drag_scope': 'Explicit body-frame effective damping; not unique aerodynamic identification',
            'command_slew_limit': None, 'warm_start': 'Previous controls interpolated by 20 ms and rolled out from current state; identified-rate INDI may relinearize the same observation up to twice after an infeasible first RTI step; ideal-rate PID retains one step; no output filtering',
            'maximum_feasibility_attempts':self.maximum_feasibility_attempts,
            'feasibility_refinement_scope':'INDI identified-rate model only; unchanged 1e-4 feasibility tolerance and total compute budget; no additional control-history shift',
            'feasibility_contract':'Check raw input/tilt inequalities and nonlinear rollout tilt; RTI nonlinear dynamics defects recorded separately, not treated as convergence to an NLP optimum'}


    def make_disturbance_observer(self):
        return HorizontalAccelerationBias(self.disturbance_configuration,self.mass,self.gravity,self.drag)


    def command(self, state, preview, disturbance_world=None, rate_state=None):
        total_started = time.perf_counter()
        state = np.asarray(state, dtype=float); preview = np.asarray(preview, dtype=float)
        if state.shape != (13,) or preview.shape != (self.nodes+1, 3, 3) or not np.isfinite(state).all() or not np.isfinite(preview).all():
            raise ValueError('MPC expects finite root state13 and [N+1,position/velocity/acceleration,xyz] preview')
        if abs(np.linalg.norm(state[3:7])-1.) > .01:
            raise ValueError('MPC requires a unit body-to-world quaternion')
        disturbance=np.zeros(3) if disturbance_world is None else np.asarray(disturbance_world,dtype=float)
        if (disturbance.shape!=(3,) or not np.isfinite(disturbance).all() or abs(disturbance[2])>1e-12
                or np.linalg.norm(disturbance[:2])>self.disturbance_configuration['max_bias_m_s2']+1e-9
                or (not self.disturbance_enabled and np.any(disturbance))):
            raise ValueError('Expected a bounded XY acceleration bias with compensation enabled')
        bias_horizon=np.tile(disturbance[:,None],(1,self.nodes))
        initial = state[:10].copy(); initial[3:7] /= np.linalg.norm(initial[3:7])
        if self.rate_enabled:
            rate_state=np.asarray(rate_state,dtype=float)
            if rate_state.shape!=(6,) or not np.isfinite(rate_state).all():raise ValueError('Six causal rate-response states required')
            initial=np.r_[initial,rate_state]
        refs = np.zeros((self.nodes+1, 17 if self.disturbance_enabled else 14)); refs[:, :3] = preview[:, 0]; refs[:, 7:10] = preview[:, 1]
        if self.disturbance_enabled:refs[:,14:17]=disturbance
        rotations = []
        for i, base_specific in enumerate(preview[:, 2] + [0., 0., self.gravity] - disturbance):
            specific=base_specific.copy()
            if np.any(self.drag):
                for _ in range(4):
                    z=specific/np.linalg.norm(specific);y=np.cross(z,[1.,0.,0.]);y/=np.linalg.norm(y)
                    rotation=np.column_stack([np.cross(y,z),y,z])
                    specific=base_specific+rotation@(self.drag*(rotation.T@preview[i,1]))
            norm = np.linalg.norm(specific)
            if norm < 1e-6 or specific[2] <= 0:
                raise ValueError('Reference acceleration requires positive upward specific force')
            z = specific/norm; y = np.cross(z, [1., 0., 0.]); y /= np.linalg.norm(y)
            rotations.append(Rotation.from_matrix(np.column_stack([np.cross(y, z), y, z])))
            refs[i, 3:7] = rotations[-1].as_quat()[[3, 0, 1, 2]]
            refs[i, 10] = norm/self.thrust_effectiveness
        for i in range(self.nodes):
            refs[i, 11:14] = (rotations[i].inv()*rotations[i+1]).as_rotvec()/self.dt
        refs[-1, 11:14] = refs[-2, 11:14]
        controls = refs[:-1, 10:14].copy()
        if self.previous_controls is not None:
            for axis in range(4):
                controls[:, axis] = np.interp(self.preview_times[:-1]+.02, self.preview_times[:-1], self.previous_controls[:, axis])
        cb = self.bounds.copy(); cb[0] /= self.mass
        controls = np.clip(controls, cb[:, 0], cb[:, 1])
        if self.previous_controls is None:
            self.solver.reset()
        states = np.column_stack([initial, np.asarray(self.disturbance_rollout(initial,controls.T,bias_horizon) if self.disturbance_enabled else self.rollout(initial, controls.T))]).T
        started = time.perf_counter()
        for i in range(self.nodes+1):
            self.solver.set(i,'p',refs[i])
            self.solver.set(i,'x',states[i])
            if i<self.nodes:
                self.solver.set(i,'u',controls[i])
        self.solver.set(0,'lbx',initial)
        self.solver.set(0,'ubx',initial)
        core_elapsed = 0.
        feasibility_attempts = []
        for attempt in range(self.maximum_feasibility_attempts):
            core_started = time.perf_counter()
            status = self.solver.solve()
            core_elapsed += time.perf_counter()-core_started
            predicted = np.array([self.solver.get(i,'x') for i in range(self.nodes+1)])
            optimal = np.array([self.solver.get(i,'u') for i in range(self.nodes)])
            rollout = np.column_stack([initial,np.asarray(self.disturbance_rollout(initial,optimal.T,bias_horizon) if self.disturbance_enabled else self.rollout(initial,optimal.T))]).T
            defect = float(np.max(abs(predicted[1:]-np.asarray(self.disturbance_step_batch(predicted[:-1].T,optimal.T,bias_horizon) if self.disturbance_enabled else self.step_batch(predicted[:-1].T,optimal.T)).T)))
            tilt_z = 1-2*(np.r_[predicted[1:,4],rollout[1:,4]]**2+np.r_[predicted[1:,5],rollout[1:,5]]**2)
            violation = float(max(np.max(cb[:,0]-optimal),np.max(optimal-cb[:,1]),
                np.max(np.cos(np.radians(self.tilt_limit))-tilt_z),np.max(tilt_z-1),0.))
            finite_attempt = bool(np.isfinite(predicted).all() and np.isfinite(optimal).all() and np.isfinite(rollout).all())
            feasibility_attempts.append(dict(attempt=attempt+1,solver_status=int(status),constraint_violation=violation,
                dynamics_defect=defect,finite=finite_attempt,elapsed_ms=(time.perf_counter()-total_started)*1000))
            if status!=0 or not finite_attempt or violation<=1e-4 or time.perf_counter()-total_started>self.max_solve_time-.005:
                break
            # Relinearize the SAME observation and preview at its nonlinear
            # rollout; do not advance history or relax any admission checks.
            for i in range(self.nodes+1):
                self.solver.set(i,'x',rollout[i])
                if i<self.nodes:self.solver.set(i,'u',optimal[i])
        finite = bool(np.isfinite(predicted).all() and np.isfinite(optimal).all() and np.isfinite(rollout).all())
        cost = float(self.solver.get_cost())
        qp_iterations = np.asarray(self.solver.get_stats('qp_iter')).ravel().tolist()
        residuals = self.solver.get_residuals(recompute=True).tolist()
        elapsed = time.perf_counter()-started
        command = optimal[0].copy(); command[0] *= self.mass
        command = np.clip(command,self.bounds[:,0],self.bounds[:,1])
        self.last = {'success':False, 'solver_success':status==0,
            'status':{0:'ACADOS_SUCCESS',1:'ACADOS_NAN_DETECTED',2:'ACADOS_MAXITER',3:'ACADOS_MINSTEP',4:'ACADOS_QP_FAILURE',7:'ACADOS_TIMEOUT'}.get(status,'ACADOS_STATUS_'+str(status)),
            'status_code':int(status), 'solve_ms':elapsed*1000, 'core_solve_ms':core_elapsed*1000,
            'acados_time_ms':float(self.solver.get_stats('time_tot'))*1000,
            'nonlinear_refinement_count':len(feasibility_attempts), 'feasibility_attempts':feasibility_attempts,
            'initial_constraint_violation':feasibility_attempts[0]['constraint_violation'],
            'initial_dynamics_defect':feasibility_attempts[0]['dynamics_defect'],
            'iterations':int(self.solver.get_stats('sqp_iter')), 'qp_iterations':qp_iterations,
            'constraint_violation':violation, 'dynamics_defect':defect, 'kkt_residuals':residuals,
            'disturbance_world_m_s2':disturbance.tolist(), 'cost':cost,
            'rate_response_state':rate_state.tolist() if self.rate_enabled else None, 'thrust_effectiveness':self.thrust_effectiveness, 'predicted_state':rollout.tolist(),
            'predicted_ctbr':(optimal*[self.mass,1.,1.,1.]).tolist(), 'requested_ctbr':command.tolist(),
            'total_ms':0., 'deadline_scope':'Full command computation, including Python preparation, transfers and validation'}
        # Retain the raw linearized states for refinement/failure diagnosis;
        # ordinary feasible ticks need only the existing nonlinear rollout.
        if len(feasibility_attempts)>1 or status!=0 or not finite or violation>1e-4 or time.perf_counter()-total_started>self.max_solve_time:
            self.last['solver_predicted_state']=predicted.tolist()
        total_elapsed = time.perf_counter()-total_started
        valid = bool(status==0 and total_elapsed<=self.max_solve_time and finite and violation<=1e-4)
        self.last.update(success=valid,total_ms=total_elapsed*1000,
                         refinement_rescued=bool(valid and len(feasibility_attempts)>1 and feasibility_attempts[0]['constraint_violation']>1e-4))
        if valid:
            self.previous_controls = optimal
        else:
            self.previous_controls = None
            raise MpcSolveError(f"MPC solution rejected: {self.last['status']}; {total_elapsed*1000:.2f} ms total; violation {violation:.3g}")
        return command
