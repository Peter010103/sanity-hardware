from sanity_zenoh.standalone_paths import load_json
"""Compiled MPC and causal IPC checks; never open an aircraft transport."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest

import numpy as np
from scipy.spatial.transform import Rotation
from sanity_zenoh.async_policy import AsyncPolicy

ROOT=Path(__file__).resolve().parents[6]
SOURCE=ROOT/'loco_mjx/experiments/sanity_ctbr_sim2sim/mpc_controller.py'
MODEL=ROOT/'loco_mjx/loco_mjx/environments/robots/sanity_ctbr/data/plane.xml'
CONFIG=ROOT/'Sanity/src/ros2_sanity_ws/src/sanity_zenoh/config/sanity09_mpc_bias.json'
spec=importlib.util.spec_from_file_location('sanity_bias_integration',SOURCE)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def balanced_hover(mass,bias):
    specific=np.array([0.,0.,9.81])-np.asarray(bias)
    z=specific/np.linalg.norm(specific);y=np.cross(z,[1.,0.,0.]);y/=np.linalg.norm(y)
    rotation=Rotation.from_matrix(np.column_stack([np.cross(y,z),y,z]))
    return np.r_[0.,0.,1.,rotation.as_quat()[[3,0,1,2]],np.zeros(6)],np.r_[mass*np.linalg.norm(specific),np.zeros(3)]


class BiasMpcIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.controller=module.CtbrMPC(MODEL,load_json(CONFIG))
        cls.preview=np.tile([[0.,0.,1.],[0.,0.,0.],[0.,0.,0.]],(cls.controller.nodes+1,1,1))

    def test_disturbance_in_dynamics_and_force_reference_has_correct_sign(self):
        controller=self.controller
        for bias in ([.3,-.2,0.],[-.25,.35,0.]):
            state,expected=balanced_hover(controller.mass,bias)
            controller.previous_controls=None
            for _ in range(6):command=controller.command(state,self.preview,disturbance_world=bias)
            np.testing.assert_allclose(command,expected,atol=1e-5)
            predicted=np.asarray(controller.last['predicted_state'])
            np.testing.assert_allclose(predicted,np.tile(state[:10],(controller.nodes+1,1)),atol=2e-6)
            self.assertLess(controller.last['constraint_violation'],1e-6)
            np.testing.assert_allclose(controller.last['disturbance_world_m_s2'],bias)

    def test_zero_estimate_matches_original_mpc(self):
        config=load_json(CONFIG);config.pop('disturbance_compensation')
        original=module.CtbrMPC(MODEL,config)
        state=[0.,0.,1.,1.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
        preview=self.preview.copy();preview[:,0,0]=.1
        self.controller.previous_controls=None
        for _ in range(8):
            first=original.command(state,preview)
            second=self.controller.command(state,preview,disturbance_world=[0.,0.,0.])
        np.testing.assert_allclose(first,second,atol=1e-6)
        self.assertIsNone(self.controller.info['command_slew_limit'])
        self.assertEqual(original.bounds.tolist(),self.controller.bounds.tolist())
        with self.assertRaises(ValueError):original.command(state,preview,disturbance_world=[.1,0.,0.])

    def test_invalid_estimate_cannot_change_output(self):
        state=[0.,0.,1.,1.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
        for bias in ([.6,.6,0.],[0.,0.,.1],[float('nan'),0.,0.],[0.,0.]):
            with self.assertRaises(ValueError):self.controller.command(state,self.preview,disturbance_world=bias)

    def test_worker_learns_only_selected_pd_commands_and_records_context(self):
        with tempfile.TemporaryDirectory() as directory:
            worker=AsyncPolicy(Path(sys.executable),SOURCE,CONFIG,Path(directory),controller='mpc',mpc_model=MODEL)
            try:
                state,selected=balanced_hover(worker.info['mass_kg'],[.3,-.2,0.])
                context={'allow_learning':True,'height_above_ground_m':1.,'reference_velocity':[0.]*3,'reference_acceleration':[0.]*3}
                preview=self.preview.copy();preview[:,0,0]=.1  # Shadow MPC differs from selected PD hover.
                for tick in range(200):
                    stamp=1_000_000_000+tick*20_000_000
                    worker.observe({'tick':tick,'state_stamp_ns':stamp,'state':state.tolist(),'goal':None,
                        'reference_preview':preview.tolist(),'disturbance_context':context})
                    deadline=time.monotonic()+2.
                    while (worker.latest() is None or worker.latest()['tick']!=tick) and not worker.error and time.monotonic()<deadline:time.sleep(.001)
                    self.assertIsNone(worker.error)
                    self.assertEqual(worker.latest()['tick'],tick)
                    worker.commit({'tick':tick,'selected_ctbr':selected.tolist(),'controller_context':{
                        'selected_stamp_ns':stamp+10_000_000,'disturbance_learning_allowed':True}})
                deadline=time.monotonic()+2.
                while worker.processed<200 and not worker.error and time.monotonic()<deadline:time.sleep(.001)
                self.assertEqual(worker.processed,200)
            finally:worker.close()
            rows=[json.loads(line) for line in (Path(directory)/'history.jsonl').read_text().splitlines()]
            self.assertEqual(len(rows),200)
            final=rows[-1]['result']['mpc']
            self.assertGreater(final['disturbance_world_m_s2'][0],.2)
            self.assertLess(final['disturbance_world_m_s2'][1],-.12)
            self.assertGreater(final['disturbance_observer']['updates'],10)
            self.assertIn('Observer plus full MPC',final['deadline_scope'])
            self.assertGreaterEqual(final['observer_ms'],0.)
            self.assertTrue(any(np.linalg.norm(np.asarray(row['result']['ctbr'])-selected)>.1 for row in rows))
            for tick,row in enumerate(rows):
                self.assertEqual(row['committed']['ticks'],tick+1)
                self.assertEqual(row['committed']['ctbr'],selected.tolist())
                self.assertEqual(row['result']['controller_inputs']['state_stamp_ns'],row['observation']['state_stamp_ns'])
                self.assertEqual(row['selection']['controller_context']['selected_stamp_ns'],row['observation']['state_stamp_ns']+10_000_000)


if __name__=='__main__':unittest.main()
