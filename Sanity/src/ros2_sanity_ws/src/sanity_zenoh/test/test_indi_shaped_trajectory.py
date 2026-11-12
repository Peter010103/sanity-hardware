from sanity_zenoh.standalone_paths import load_json
"""Reference shaping invariants; no ROS, worker, transport or hardware."""
import json
import math
from pathlib import Path

import numpy as np
import pytest

from sanity_zenoh.indi_hover import make_mission
from sanity_zenoh.indi_lemniscate import AngularLemniscate, IndiShapedAngularLemniscate, IndiShapedTransfer, QuinticTransfer
from sanity_zenoh.mpc_benchmark_config import build_config

ROOT=next(p for p in Path(__file__).resolve().parents if (p/'loco_mjx').is_dir())
PACKAGE=Path(__file__).resolve().parents[1]
BASE=PACKAGE/'config/sanity15_indi_mpc_constant_arclength.json'


@pytest.mark.parametrize('speed',[.8,1.,1.5])
def test_shaping_respects_limits_preserves_peak_speed_and_completes_all_three_laps(speed):
    curve=IndiShapedAngularLemniscate([.1,-.2,1.],peak_speed=speed)
    points=[curve.sample(float(t)) for t in np.linspace(0,curve.duration,10001)]
    assert max(np.linalg.norm(p['velocity']) for p in points)<=speed+1e-12
    assert max(np.linalg.norm(p['acceleration']) for p in points)<=1.5+1e-12
    assert max(np.linalg.norm(p['jerk']) for p in points)<=2.+1e-12
    assert curve.design_metadata['conservative_acceleration_bound_m_s2']<=1.5+1e-12
    assert curve.design_metadata['conservative_jerk_bound_m_s3']<=2.+1e-12
    for lap in (1,2):
        crossing=2*math.pi*lap/curve.omega+.5*curve.ramp
        sample=curve.sample(crossing)
        np.testing.assert_allclose(sample['position'],curve.center,atol=1e-12)
        assert np.linalg.norm(sample['velocity'])==pytest.approx(speed)
    last=curve.sample(curve.duration)
    assert last['completed_laps']==3 and last['lap_progress']==3.
    assert last['position']==curve.center.tolist()
    assert last['velocity']==last['acceleration']==last['jerk']==[0.,0.,0.]


def test_high_speed_ramp_is_lengthened_but_cruise_geometry_is_unchanged():
    old=AngularLemniscate([0.,0.,1.],3.2,1.5,2.,laps=3)
    new=IndiShapedAngularLemniscate([0.,0.,1.],3.2,1.5,2.,laps=3)
    assert 4.<new.ramp<4.5
    assert new.omega==old.omega
    assert new.duration-old.duration==pytest.approx(new.ramp-old.ramp)
    for theta in np.linspace(1.,2*math.pi*3-1.,301):
        a=old.sample(theta/old.omega+old.ramp/2)
        b=new.sample(theta/new.omega+new.ramp/2)
        for field in ('position','velocity','acceleration','jerk'):
            np.testing.assert_allclose(a[field],b[field],atol=1e-12)


def test_reference_has_consistent_analytic_derivatives_and_c3_ramp_joins():
    curve=IndiShapedAngularLemniscate([0.,0.,1.])
    for t in np.linspace(.001,curve.duration-.001,301):
        h=1e-5;a=curve.sample(t-h);b=curve.sample(t+h);c=curve.sample(t)
        for state,derivative,tol in [('position','velocity',2e-8),('velocity','acceleration',2e-8),('acceleration','jerk',3e-7)]:
            np.testing.assert_allclose((np.array(b[state])-a[state])/(2*h),c[derivative],atol=tol)
    for join in (0.,curve.ramp,curve.duration-curve.ramp,curve.duration):
        a=curve.sample(join-1e-7);b=curve.sample(join+1e-7)
        for field in ('position','velocity','acceleration','jerk'):
            np.testing.assert_allclose(a[field],b[field],atol=1e-6)


@pytest.mark.parametrize('kwargs',[dict(maximum_acceleration_m_s2=.1),dict(maximum_jerk_m_s3=.1),dict(maximum_jerk_m_s3=float('nan')),dict(maximum_acceleration_m_s2=True)])
def test_impossible_or_invalid_reference_limits_are_rejected(kwargs):
    with pytest.raises(ValueError):IndiShapedAngularLemniscate([0.,0.,1.],**kwargs)


@pytest.mark.parametrize('speed',[.8,1.,1.5])
@pytest.mark.parametrize('inner',['pid','indi'])
def test_public_builder_exposes_shaping_as_explicit_comparable_option(speed,inner):
    base=load_json(BASE)
    cfg=build_config(base,pattern='lemniscate',inner=inner,speed=speed,schedule='indi_shaped_angular',ground_m=.037)
    mission=make_mission(cfg,ROOT/'loco_mjx');curve=mission._make_curve([0.,0.,1.037])
    assert isinstance(curve,IndiShapedAngularLemniscate)
    assert cfg['benchmark_provenance']['reference_design']==curve.design_metadata
    assert cfg['limits']['maximum_run_s']==max(120,math.ceil(curve.duration+60))<=180
    assert cfg['trajectory']['center_xy_m']==base['trajectory']['center_xy_m']
    # Nothing silently changes a prior recipe or rewrites its requested ramp.
    assert cfg['trajectory']['ramp_s']==2.
    prior=build_config(base,pattern='lemniscate',inner=inner,speed=speed,schedule='variable_angular',ground_m=.037)
    prior_curve=make_mission(prior,ROOT/'loco_mjx')._make_curve([0.,0.,1.037])
    assert type(prior_curve) is AngularLemniscate and prior_curve.ramp==2.
    assert 'reference_design' not in prior['benchmark_provenance']


def test_shaping_does_not_silently_apply_to_circle():
    with pytest.raises(ValueError):build_config(load_json(BASE),pattern='circle',inner='indi',speed=1.5,schedule='indi_shaped_angular',ground_m=.037)


@pytest.mark.parametrize('end',[[0.,0.,1.],[.5,.2,1.],[3.,1.,1.]])
def test_shaped_transfer_has_exact_limits_and_zero_v_a_jerk_at_both_ends(end):
    c=IndiShapedTransfer([0.,0.,1.],end)
    samples=[c.sample(float(t)) for t in np.linspace(0,c.duration,1001)]
    for key,limit in [('velocity',.4),('acceleration',1.5),('jerk',2.)]:
        assert max(np.linalg.norm(s[key]) for s in samples)<=limit+1e-12
        assert samples[0][key]==samples[-1][key]==[0.,0.,0.]
    np.testing.assert_allclose(samples[0]['position'],c.start,atol=1e-15)
    np.testing.assert_allclose(samples[-1]['position'],c.end,atol=1e-15)
    for t in np.linspace(.001,c.duration-.001,51):
        h=1e-5;a=c.sample(t-h);b=c.sample(t+h);d=c.sample(t)
        for key,derivative in [('position','velocity'),('velocity','acceleration'),('acceleration','jerk')]:
            np.testing.assert_allclose((np.array(b[key])-a[key])/(2*h),d[derivative],atol=1e-7)


def test_explicit_shaped_mission_entry_lap_exit_are_c3_and_prior_transfers_stay_quintic():
    base=load_json(BASE)
    cfg=build_config(base,pattern='lemniscate',inner='indi',speed=1.5,schedule='indi_shaped_angular',ground_m=.037)
    m=make_mission(cfg,ROOT/'loco_mjx');home=np.array([.5,.2,1.037]);center=np.array([0.,0.,1.037])
    entry=m._make_transfer(home,center);lap=m._make_curve(center);exit=m._make_transfer(center,home)
    assert isinstance(entry,IndiShapedTransfer) and isinstance(exit,IndiShapedTransfer)
    for before,after in [(entry,lap),(lap,exit)]:
        a=before.sample(before.duration);b=after.sample(0.)
        for key in ('position','velocity','acceleration','jerk'):np.testing.assert_allclose(a[key],b[key],atol=1e-12)
        for key in ('position','velocity','acceleration','jerk'):
            np.testing.assert_allclose(before.sample(before.duration-1e-7)[key],after.sample(1e-7)[key],atol=1e-6)
    old=build_config(base,pattern='lemniscate',inner='indi',speed=1.5,schedule='variable_angular',ground_m=.037)
    assert type(make_mission(old,ROOT/'loco_mjx')._make_transfer(home,center)) is QuinticTransfer
