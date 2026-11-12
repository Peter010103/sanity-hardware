"""Analytic geometry, default compatibility and real launcher snapshot tests."""
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from sanity_zenoh.indi_lemniscate import AngularLemniscate,IndiShapedAngularLemniscate,ButterflyReference
from sanity_zenoh.high_speed_lemniscate import make_reference,reference_limits
from sanity_zenoh.mpc_benchmark_config import build_config
from sanity_zenoh.indi_hover import make_mission
OUT=Path(__file__).resolve().parents[1]
ROOT=next(p for p in OUT.parents if (p/'loco_mjx').is_dir())
PKG=OUT/'host/sanity_zenoh'
BASE=json.loads((PKG/'config/sanity15_indi_mpc_constant_arclength.json').read_text())
spec=importlib.util.spec_from_file_location('width_candidate_launcher',OUT/'run_high_speed_lemniscate.py')
launcher=importlib.util.module_from_spec(spec);spec.loader.exec_module(launcher)
oldspec=importlib.util.spec_from_file_location('original_gerono_reference',ROOT/'releases/high-speed/host/sanity_zenoh/sanity_zenoh/indi_lemniscate.py')
old=importlib.util.module_from_spec(oldspec);oldspec.loader.exec_module(old)

@pytest.mark.parametrize('cls,name',[(AngularLemniscate,'AngularLemniscate'),(IndiShapedAngularLemniscate,'IndiShapedAngularLemniscate')])
def test_omitted_width_and_explicit_legacy_width_preserve_every_sample_exactly(cls,name):
    kwargs={} if cls is AngularLemniscate else reference_limits(5.)
    a=getattr(old,name)([.1,.2,1.],2.,5.,2.,laps=3,**kwargs)
    b=cls([.1,.2,1.],2.,5.,2.,laps=3,**kwargs)
    c=cls([.1,.2,1.],2.,5.,2.,laps=3,width_m=2.,**kwargs)
    assert a.duration==b.duration==c.duration and a.omega==b.omega==c.omega
    for t in np.linspace(-.1,a.duration+.1,401):
        for key in ('position','velocity','acceleration','jerk','theta_rad','lap_progress','completed_laps'):
            assert a.sample(float(t))[key]==b.sample(float(t))[key]==c.sample(float(t))[key]

@pytest.mark.parametrize('height',[0.,.3,-.3])
def test_inclined_derivatives_and_analytic_norm_bounds(height):
    kwargs=reference_limits(5.,width_m=3.,vertical_amplitude_m=height)
    c=IndiShapedAngularLemniscate([0.,0.,1.],2.,5.,2.,laps=1,width_m=3.,vertical_amplitude_m=height,**kwargs)
    b=np.sqrt([4.+9.+4.*height**2,4.+36.+16.*height**2,4.+144.+64.*height**2])
    np.testing.assert_allclose(c.design_metadata['derivative_norm_bounds_m'],b,atol=1e-14)
    assert c.omega==pytest.approx(5./b[0])
    for t in np.linspace(.002,c.duration-.002,101):
        h=1e-5;a=c.sample(t-h);d=c.sample(t+h);v=c.sample(t)
        for field,derivative,tol in [('position','velocity',1e-7),('velocity','acceleration',1e-7),('acceleration','jerk',2e-6)]:
            np.testing.assert_allclose((np.array(d[field])-a[field])/(2*h),v[derivative],atol=tol)
    samples=[c.sample(t) for t in np.linspace(0.,c.duration,1001)]
    assert max(np.linalg.norm(x['acceleration']) for x in samples)<=kwargs['maximum_acceleration_m_s2']
    assert max(np.linalg.norm(x['jerk']) for x in samples)<=kwargs['maximum_jerk_m_s3']


def test_planar_four_by_three_has_exact_extents_peak_five_and_three_laps():
    c=make_reference(5.,width_m=3.)
    for theta,expected in [(2*math.pi+math.pi/2,[2.,0.,1.]),(2*math.pi+math.pi/4,[math.sqrt(2.),1.5,1.]),(3*math.pi+math.pi/4,[-math.sqrt(2.),1.5,1.])]:
        s=c.sample(theta/c.omega+c.ramp/2)
        np.testing.assert_allclose(s['position'],expected,atol=1e-13)
    crossing=c.sample(2*math.pi/c.omega+c.ramp/2)
    assert np.linalg.norm(crossing['velocity'])==pytest.approx(5.)
    assert c.sample(c.duration)['completed_laps']==3


def test_butterfly_alternates_complete_planes_with_six_eights_and_c3_rest_handoffs():
    c=make_reference(5.,width_m=3.,butterfly_height_m=.3,butterfly_cycles=3)
    assert isinstance(c,ButterflyReference) and c.laps==6 and c.cycles==3
    assert c.duration==pytest.approx(6*c.segment_duration)
    for index in range(6):
        segment=c.planes[index%2]
        t=index*c.segment_duration+math.pi/segment.omega+segment.ramp/2
        assert np.linalg.norm(c.sample(t)['velocity'])==pytest.approx(5.)
        for theta in (.75*math.pi,1.25*math.pi):
            local=theta/segment.omega+segment.ramp/2
            sample=c.sample(index*c.segment_duration+local)
            assert sample['position'][2]-1.==pytest.approx((1 if index%2==0 else -1)*.3*math.sin(2*theta))
        join=(index+1)*c.segment_duration
        s=c.sample(join)
        np.testing.assert_allclose(s['position'],c.center,atol=1e-12)
        for key in ('velocity','acceleration','jerk'):np.testing.assert_allclose(s[key],0.,atol=1e-10)
        for key in ('position','velocity','acceleration','jerk'):
            np.testing.assert_allclose(c.sample(join-1e-7)[key],c.sample(join+1e-7)[key],atol=1e-5)
    end=c.sample(c.duration)
    assert end['completed_cycles']==3 and end['completed_laps']==6 and end['lap_progress']==6.

@pytest.mark.parametrize('width',[0.,-1.,float('nan'),True])
def test_invalid_width_is_rejected(width):
    with pytest.raises(ValueError):make_reference(5.,width_m=width)


def test_circle_rejects_explicit_width_and_remains_unchanged_without_it():
    kw=dict(pattern='circle',inner='indi',speed=1.5,schedule='constant_arclength',ground_m=.03)
    plain=build_config(BASE,**kw)
    with pytest.raises(ValueError):build_config(BASE,width_m=3.,**kw)
    assert plain['trajectory']['pattern']=='circle' and 'width_m' not in plain['trajectory']


def test_actual_factory_provenance_limits_and_snapshot_include_geometry(tmp_path):
    args=SimpleNamespace(test='trajectory',pattern='lemniscate',inner='indi',speed=5.,schedule='indi_shaped_angular',radius_m=2.,width_m=3.,butterfly_height_m=.3,butterfly_cycles=3)
    cfg=launcher.make_config(args,.03);m=make_mission(cfg,ROOT/'loco_mjx');curve=m._make_curve([0.,0.,1.03])
    assert isinstance(curve,ButterflyReference) and cfg['trajectory']['laps']==6
    assert cfg['benchmark_provenance']['requested_laps']==6
    assert cfg['benchmark_provenance']['butterfly_cycles']==3
    assert cfg['benchmark_provenance']['footprint_xy_m']==[4.,3.]
    assert cfg['benchmark_provenance']['reference_design']==curve.design_metadata
    original_config=copy.deepcopy(cfg);args.butterfly_height_m=0.;args.width_m=None
    oldcfg=launcher.make_config(args,.03)
    for key in ('limits','benchmark_envelope','ground_indi','liftoff_options','spool_leveling','mpc'):
        assert original_config[key]==oldcfg[key]
    launcher.snapshot_sources(tmp_path,cfg)
    manifest=json.loads((tmp_path/'source_manifest.json').read_text())['sha256']
    for name in ('indi_lemniscate.py','high_speed_lemniscate.py','mpc_benchmark_config.py'):
        source=PKG/'sanity_zenoh'/name;relative=str(source.relative_to(ROOT))
        assert manifest[relative]==hashlib.sha256(source.read_bytes()).hexdigest()
    assert any(name.endswith('scripts/run_mpc.py') for name in manifest)
    for name in ('mpc_controller.py','indi_rate_model.py','mpc_disturbance.py','mpc_5mps.json','model_5mps.xml'):
        source=OUT/name;relative=str(source.relative_to(ROOT))
        assert manifest[relative]==hashlib.sha256(source.read_bytes()).hexdigest()


def test_output_directory_records_width_incline_and_cycles(tmp_path,monkeypatch):
    args=SimpleNamespace(test='trajectory',pattern='lemniscate',inner='indi',speed=5.,schedule='indi_shaped_angular',radius_m=2.,width_m=3.,butterfly_height_m=.3,butterfly_cycles=3,output=None,bench_check=False)
    cfg=launcher.make_config(args,.03);monkeypatch.setattr(launcher,'ROOT',tmp_path)
    (tmp_path/'outputs').mkdir();path=launcher.create_run_directory(args,cfg)
    assert '_R2m_W3m_butterflyH0.3m_C3_' in path.name


def test_confirmed_vertical_range_is_relative_to_ground_and_ceiling_is_not_raised():
    args=SimpleNamespace(test='trajectory',pattern='lemniscate',inner='indi',speed=5.,schedule='indi_shaped_angular',radius_m=2.,width_m=3.,butterfly_height_m=.3,butterfly_cycles=3)
    cfg=launcher.make_config(args,.03);m=make_mission(cfg,ROOT/'loco_mjx');c=m._make_curve([0.,0.,1.03])
    np.testing.assert_allclose(c.design_metadata['height_bounds_m'],[.73,1.33])
    assert cfg['limits']['maximum_altitude_m']==1.5
    args.butterfly_height_m=.5
    with pytest.raises(ValueError):launcher.make_config(args,.03)
