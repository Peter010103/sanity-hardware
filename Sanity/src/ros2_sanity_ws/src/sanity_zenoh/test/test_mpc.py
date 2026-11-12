from sanity_zenoh.standalone_paths import load_json
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest

import numpy as np
from scipy.spatial.transform import Rotation
from sanity_zenoh.async_policy import AsyncPolicy

ROOT = Path(__file__).resolve().parents[6]
SOURCE = ROOT/'loco_mjx/experiments/sanity_ctbr_sim2sim/mpc_controller.py'
MODEL = ROOT/'loco_mjx/loco_mjx/environments/robots/sanity_ctbr/data/plane.xml'
CONFIG = ROOT/'Sanity/src/ros2_sanity_ws/src/sanity_zenoh/config/sanity09_mpc.json'
spec = importlib.util.spec_from_file_location('sanity_mpc_tests', SOURCE)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class MpcTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mpc = module.CtbrMPC(MODEL, load_json(CONFIG))


    def test_hover_units_and_finite_horizon_constraints(self):
        state = np.array([0.,0.,1.,1.,0.,0.,0.,0.,0.,0.,0.,0.,0.])
        preview = np.tile([[0.,0.,1.],[0.,0.,0.],[0.,0.,0.]], (self.mpc.nodes+1,1,1))
        command = self.mpc.command(state, preview)
        self.assertTrue(self.mpc.last['solver_success'])
        self.assertAlmostEqual(command[0], self.mpc.mass*9.81, places=5)
        np.testing.assert_allclose(command[1:],0.,atol=1e-8)
        predicted = np.asarray(self.mpc.last['predicted_state'])
        self.assertLess(np.max(abs(predicted[:,:3]-state[:3])),1e-5)
        self.assertLess(self.mpc.last['constraint_violation'],1e-5)
        q = Rotation.from_quat(predicted[:,[4,5,6,3]])
        self.assertGreaterEqual(q.as_matrix()[:,2,2].min(),np.cos(np.radians(30.))-1e-5)


    def test_world_axis_commands_and_quaternion_sign_invariance(self):
        preview = np.tile([[.1,0.,1.],[0.,0.,0.],[0.,0.,0.]],(self.mpc.nodes+1,1,1))
        for yaw in (0., np.pi/2):
            rotation = Rotation.from_euler('z',yaw)
            state = np.r_[0.,0.,1.,rotation.as_quat()[[3,0,1,2]],np.zeros(6)]
            self.mpc.previous_controls=None
            for _ in range(8):command = self.mpc.command(state,preview)
            world_rates = rotation.apply(command[1:])
            self.assertGreater(world_rates[1],0.)
            predicted=np.asarray(self.mpc.last['predicted_state'])
            self.assertGreater(predicted[-1,0],.08)
            self.assertLess(abs(predicted[-1,1]),.01)
            self.assertTrue(np.all(command>=self.mpc.bounds[:,0]))
            self.assertTrue(np.all(command<=self.mpc.bounds[:,1]))
            first = command.copy()
            state[3:7] *= -1
            self.mpc.previous_controls=None
            for _ in range(8):second = self.mpc.command(state,preview)
            np.testing.assert_allclose(first,second,atol=2e-4)


    def test_invalid_inputs_never_produce_a_command(self):
        for state, preview in [([0.]*13, np.zeros((21,3,3))), ([0.]*12, np.zeros((21,3,3))),
                ([np.nan]*13,np.zeros((21,3,3))), ([0,0,1,1,0,0,0,0,0,0,0,0,0],np.zeros((6,3,3)))]:
            with self.assertRaises(ValueError):self.mpc.command(state, preview)
        for config in ({'nodes':0},{'drag_per_s':[-1.,0.,0.]},{'tilt_limit_deg':90.},{'slew_limit':1.}):
            with self.assertRaises(ValueError):module.CtbrMPC(MODEL,config)


    def test_late_solution_raises_instead_of_returning_a_command(self):
        from unittest.mock import patch
        state=[0.,0.,1.,1.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
        preview=[[[0.,0.,1.],[0.,0.,0.],[0.,0.,0.]]]*(self.mpc.nodes+1)
        with patch.object(self.mpc,'max_solve_time',0.):
            with self.assertRaises(module.MpcSolveError):self.mpc.command(state,preview)
        self.assertFalse(self.mpc.last['success'])
        self.assertTrue(self.mpc.last['solver_success'])


    def test_qp_failure_never_returns_a_command(self):
        from unittest.mock import patch
        state=[0.,0.,1.,1.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
        preview=[[[0.,0.,1.],[0.,0.,0.],[0.,0.,0.]]]*(self.mpc.nodes+1)
        with patch.object(self.mpc.solver,'solve',return_value=4):
            with self.assertRaises(module.MpcSolveError):self.mpc.command(state,preview)
        self.assertFalse(self.mpc.last['success'])
        self.assertEqual(self.mpc.last['status'],'ACADOS_QP_FAILURE')
        self.assertIsNone(self.mpc.previous_controls)


    def test_deadline_covers_python_preparation_and_validation(self):
        from unittest.mock import patch
        state=[0.,0.,1.,1.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
        preview=[[[0.,0.,1.],[0.,0.,0.],[0.,0.,0.]]]*(self.mpc.nodes+1)
        rollout=self.mpc.rollout
        def delayed(*args):
            time.sleep(.03)
            return rollout(*args)
        with patch.object(self.mpc,'rollout',side_effect=delayed):
            with self.assertRaises(module.MpcSolveError):self.mpc.command(state,preview)
        self.assertTrue(self.mpc.last['solver_success'])
        self.assertGreater(self.mpc.last['total_ms'],60.)
        self.assertLess(self.mpc.last['core_solve_ms'],25.)
        self.assertFalse(self.mpc.last['success'])


    def test_raw_out_of_bounds_solution_is_rejected_before_command_clipping(self):
        from unittest.mock import patch
        state=[0.,0.,1.,1.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
        preview=[[[0.,0.,1.],[0.,0.,0.],[0.,0.,0.]]]*(self.mpc.nodes+1)
        get=self.mpc.solver.get
        def invalid(stage,field):
            value=get(stage,field)
            if field=='u':value[0]=1000.
            return value
        with patch.object(self.mpc.solver,'get',side_effect=invalid):
            with self.assertRaises(module.MpcSolveError):self.mpc.command(state,preview)
        self.assertGreater(self.mpc.last['constraint_violation'],1.)
        self.assertFalse(self.mpc.last['success'])


    def test_cached_solver_loads_without_recompilation(self):
        from acados_template import AcadosOcpSolver
        from unittest.mock import patch
        with patch.object(AcadosOcpSolver,'build',side_effect=AssertionError('Unexpected rebuild')):
            other=module.CtbrMPC(MODEL,load_json(CONFIG))
        self.assertEqual(other.cache_key,self.mpc.cache_key)
        self.assertEqual(other.info['solver'],'acados/SQP_RTI/HPIPM')
        self.assertEqual(other.info['configuration']['qp_max_iterations'],50)
        self.assertEqual(other.info['command_slew_limit'],None)


    def test_async_mpc_records_preview_and_selected_outgoing_ctbr_without_actor(self):
        with tempfile.TemporaryDirectory() as directory:
            worker = AsyncPolicy(Path(sys.executable),SOURCE,CONFIG,Path(directory),controller='mpc',mpc_model=MODEL)
            try:
                self.assertEqual(worker.info['controller'],'mpc')
                self.assertNotIn('checkpoint',worker.info)
                for tick in range(4):
                    observation={'tick':tick,'state_stamp_ns':time.monotonic_ns(),
                        'state':[0.,0.,1.,1.,0.,0.,0.,0.,0.,0.,0.,0.,0.],'goal':None,
                        'reference_preview':[[[0.,0.,1.],[0.,0.,0.],[0.,0.,0.]]]*(worker.info['nodes']+1)}
                    worker.observe(observation)
                    deadline=time.monotonic()+2
                    while (worker.latest() is None or worker.latest()['tick']!=tick) and not worker.error and time.monotonic()<deadline:
                        time.sleep(.001)
                    self.assertIsNone(worker.error)
                    self.assertTrue(worker.latest()['valid'])
                    worker.commit({'tick':tick,'selected_ctbr':[.3+tick*.01,0.,0.,0.]})
            finally:
                worker.close()
            history=[json.loads(line) for line in (Path(directory)/'history.jsonl').read_text().splitlines()]
            self.assertEqual(len(history),4)
            for tick,row in enumerate(history):
                self.assertEqual(row['committed']['ticks'],tick+1)
                self.assertEqual(row['committed']['ctbr'],row['selection']['selected_ctbr'])
                self.assertNotIn('actor_inputs',row['result'])
                self.assertEqual(row['result']['controller_inputs']['reference_preview'],row['observation']['reference_preview'])


    def test_ros_sequence_selects_mpc_and_commits_physical_ctbr_without_actor_action(self):
        try:
            import rclpy
            from sanity_zenoh.ros_sequence import SequenceNode
        except ImportError:
            self.skipTest('Source the ROS overlay for the controller integration test')
        from types import SimpleNamespace
        from unittest.mock import patch
        class Sender:
            def __init__(self,**kwargs):
                self.stop_reason=None
                self.shutdown_acknowledged=True
                self.shutdown_error=None
                self.shutdown_status={}
            def close(self):pass
        with tempfile.TemporaryDirectory() as directory:
            output=Path(directory)/'controller'
            args=['--ros-args','-p','controller:=mpc','-p','loco_root:='+str(ROOT/'loco_mjx'),
                '-p','mpc_config:='+str(CONFIG),'-p','output:='+str(output)]
            rclpy.init(args=args)
            node=None
            try:
                with patch('sanity_zenoh.ros_sequence.SanitySender',Sender):node=SequenceNode()
                self.assertEqual(node.worker.info['controller'],'mpc')
                self.assertNotIn('checkpoint',node.worker.info)
                now=[31_000_000_000]
                node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=now[0]))
                state=np.array([0.,0.,1.,1.,0.,0.,0.,0.,0.,0.,0.,0.,0.])
                node.measured_state=lambda:(state,.002,{'stamp_ns':now[0]-2_000_000})
                node.reference.sample=lambda t:{'position':[0.,0.,1.],'velocity':[0.]*3,'acceleration':[0.]*3,'phase':'circle_start_hold'}
                node.reference.handover_ready_time=0.
                node.supervisor.blend_start=0.
                node.stage='running'
                node.started_ns=1_000_000_000
                node.home_xy=np.zeros(2)
                node.ground_root=.02
                node.values['enable_output']=True  # Sender is mocked; never open a hardware transport.
                for tick in range(3):
                    node.observe()
                    deadline=time.monotonic()+2
                    while (node.worker.latest() is None or node.worker.latest()['tick']!=tick) and not node.worker.error and time.monotonic()<deadline:
                        time.sleep(.001)
                    node.control_tick()
                    self.assertIsNone(node.error)
                    self.assertEqual(node.tick,tick+1)
                    self.assertGreater(node.slot['ctbr'][0],.6)
                    self.assertNotIn('selected_action',node.worker.selections.get(tick,{}))
                    now[0]+=20_000_000
                node.close()
                receipt=load_json(output / 'receipt.json')
                self.assertEqual(receipt['history_processed'],3)
                rows=[json.loads(line) for line in (output/'history.jsonl').read_text().splitlines()]
                self.assertEqual(len(rows),3)
                self.assertTrue(all(r['committed']['ctbr']==r['selection']['selected_ctbr'] for r in rows))
                (Path(directory)/'experiment_receipt.json').write_text(json.dumps({'rosbag_returncode':0}))
                assessed=subprocess.run([sys.executable,str(ROOT/'Sanity/src/ros2_sanity_ws/scripts/assess_landing.py'),directory],capture_output=True,text=True)
                self.assertNotIn('Traceback',assessed.stderr)
                summary=load_json(Path(directory) / 'summary.json')
                self.assertTrue(summary['checks']['mpc_inputs_recorded'])
                self.assertTrue(summary['checks']['all_selected_history_committed'])
                self.assertTrue(summary['checks']['history_matches_selected_command'])
                self.assertTrue(summary['checks']['mpc_selected_results_valid'])
                self.assertNotIn('raw_actor_inputs_recorded',summary['checks'])
            finally:
                if node:
                    if not (output/'receipt.json').exists():node.close()
                    node.destroy_node()
                rclpy.shutdown()


    @unittest.skip("Optional training simulator is outside the standalone flight package")
    def test_simulator_uses_the_same_isolated_deployment_worker(self):
        spec=importlib.util.spec_from_file_location('sanity_sim_worker_client',SOURCE.with_name('run.py'))
        simulator=importlib.util.module_from_spec(spec);spec.loader.exec_module(simulator)
        with tempfile.TemporaryDirectory() as directory:
            client=simulator.ActorProcess(Path(sys.executable),CONFIG,Path(directory)/'worker.log',mpc_model=MODEL)
            try:
                self.assertEqual(client.info['solver'],'acados/SQP_RTI/HPIPM')
                self.assertEqual(client.call('reset'),{'reset':True})
                result=client.call('infer',state=[0.,0.,1.,1.,0.,0.,0.,0.,0.,0.,0.,0.,0.],
                    reference_preview=[[[0.,0.,1.],[0.,0.,0.],[0.,0.,0.]]]*(client.info['nodes']+1))
                self.assertTrue(result['valid'])
                self.assertLess(result['mpc']['total_ms'],25.)
                self.assertAlmostEqual(result['ctbr'][0],client.info['mass_kg']*9.81,places=5)
                for tick in range(3):
                    reply=client.call('commit',ctbr=[.5,0.,0.,0.])
                    self.assertEqual(reply,{'ctbr':[.5,0.,0.,0.],'ticks':tick+1})
                client.call('reset')
                self.assertEqual(client.call('commit',ctbr=[0.,0.,0.,0.])['ticks'],1)
            finally:client.close()


    def test_mpc_plan_has_no_checkpoint_and_rejects_policy_mix(self):
        args=[sys.executable,'-m','sanity_zenoh.experiment','--workspace',str(ROOT),'--controller','mpc','--plan-only']
        result=subprocess.run(args,capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stderr)
        plan=json.loads(result.stdout)
        self.assertIsNone(plan['checkpoint'])
        self.assertEqual(plan['controller'],'mpc')
        result=subprocess.run([*args,'--residual'],capture_output=True,text=True)
        self.assertNotEqual(result.returncode,0)
        self.assertIn('standalone CTBR baseline',result.stderr)


if __name__=='__main__':unittest.main()
