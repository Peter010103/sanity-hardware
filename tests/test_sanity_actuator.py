"""Offline model consistency, causal delay and timestep invariance checks."""
import math
import unittest
from sanity_actuator import SanityModel,SpinningModel


class Tests(unittest.TestCase):
    def test_delay_and_closed_form_response(self):
        m=SanityModel();low=[1250]*4;high=[1450]*4;v=7.8
        a=m.steady_omega(low,v);b=m.steady_omega(high,v)
        sim=SpinningModel(m,low,v)
        self.assertEqual(sim.step(m.L/2,high,v),a)
        w=sim.step(m.L/2+.08,high,v)
        expected=[f+(x-f)*math.exp(-.08/m.tau) for x,f in zip(a,b)]
        for x,y in zip(w,expected):self.assertAlmostEqual(x,y,places=9)

    def test_partitioning_timesteps_preserves_solution(self):
        m=SanityModel();a=SpinningModel(m,[1250]*4,7.8);b=SpinningModel(m,[1250]*4,7.8)
        wa=a.step(.2,[1450]*4,7.8)
        for _ in range(200):wb=b.step(.001,[1450]*4,7.8)
        for x,y in zip(wa,wb):self.assertAlmostEqual(x,y,places=8)

    def test_out_of_domain_input_and_startup_refused(self):
        m=SanityModel()
        for u,v in [([1750]*4,7.8),([1000]*4,7.8),([1350]*4,6.5)]:
            with self.assertRaises(ValueError):m.steady_omega(u,v)
        with self.assertRaises(ValueError):SpinningModel(m,[1250]*4,7.8,[0]*4)
        with self.assertRaises(ValueError):m.thrusts([float('nan')]*4)

    def test_square_law_uses_each_rotor_separately(self):
        m=SanityModel();w=[1600,2200,2800,3100]
        measured=sum(m.thrusts(w))
        k=m.force['k_N_per_rad_s_squared']
        self.assertAlmostEqual(measured,k*sum(x*x for x in w),places=12)
        self.assertGreater(measured,k*4*(sum(w)/4)**2)
        self.assertEqual(m.thrusts([0]*4),(0,0,0,0))


if __name__=='__main__':unittest.main()
