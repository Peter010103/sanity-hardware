from sanity_zenoh.standalone_paths import load_json
"""Public launcher readiness and child-ownership contracts; no lab services."""
import importlib.util
import json
import os
from pathlib import Path
import sys
import signal
import threading
import time

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

ROOT=next(p for p in Path(__file__).resolve().parents if (p/'loco_mjx').is_dir())
spec=importlib.util.spec_from_file_location('benchmark_launcher',ROOT/'Sanity/src/ros2_sanity_ws/scripts/run_sanity_mpc_benchmark.py')
launcher=importlib.util.module_from_spec(spec);spec.loader.exec_module(launcher)


def bench_poses(path,*,tilt=5.,speed=0.,span=.0001,ready=True,frame='sanity15'):
    quaternion=Rotation.from_euler('x',tilt,degrees=True).as_quat()[[3,0,1,2]].tolist()
    rows=[]
    for i in range(50):
        rows.append(dict(kind='mocap_original_source',host_monotonic_ns=8_000_000_000+i*40_000_000,
            source_packet=dict(ready=ready,frames_confirmed=True,tracked_frame=frame,
                state=[i/49*span,0.,.04,*quaternion,speed,0.,0.,0.,0.,0.])))
    path.write_text(''.join(json.dumps(x)+'\n' for x in rows))
    return rows[-1]['host_monotonic_ns']+100_000_000


def test_floor_uses_confirmed_pose_and_wxyz_orientation(tmp_path):
    p=tmp_path/'events.jsonl';now=bench_poses(p)
    result=launcher.refreshed_ground(p,'sanity15',now)
    assert result['ground_m']==pytest.approx(.04)
    assert result['maximum_tilt_deg']==pytest.approx(5.)
    assert result['samples']==50 and result['last_pose_age_s']==pytest.approx(.1)


@pytest.mark.parametrize('options',[dict(tilt=11.),dict(speed=.11),dict(span=.02),dict(ready=False),dict(frame='sanity09')])
def test_unready_or_moving_pose_cannot_supply_flight_floor(tmp_path,options):
    p=tmp_path/'events.jsonl';now=bench_poses(p,**options)
    with pytest.raises(ValueError):launcher.refreshed_ground(p,'sanity15',now)


def test_stale_and_future_bench_poses_are_rejected(tmp_path):
    p=tmp_path/'events.jsonl';now=bench_poses(p)
    for value in (now+6_000_000_000,now-2_000_000_000):
        with pytest.raises(ValueError):launcher.refreshed_ground(p,'sanity15',value)


def test_short_or_reordered_pose_window_is_rejected(tmp_path):
    p=tmp_path/'events.jsonl';now=bench_poses(p)
    rows=[json.loads(s) for s in p.read_text().splitlines()]
    for sample in (rows[-20:],rows[:10]+[rows[11],rows[10]]+rows[12:]):
        p.write_text(''.join(json.dumps(x)+'\n' for x in sample))
        with pytest.raises(ValueError):launcher.refreshed_ground(p,'sanity15',now)


def test_plan_only_never_creates_live_process_owner(monkeypatch,capsys):
    monkeypatch.setattr(launcher,'OwnedCommands',lambda *_:pytest.fail('Plan must not start a process owner'))
    monkeypatch.setattr(sys,'argv',['launcher','--inner','pid','--pattern','circle','--speed','0.8'])
    assert launcher.main()==0
    result=json.loads(capsys.readouterr().out)
    assert result['pattern']=='circle' and result['inner_controller']=='pid'
    assert result['network_or_hardware_access'] is False


def test_small_lemniscate_option_reaches_saved_config_and_readable_run_name(monkeypatch,capsys,tmp_path):
    from types import SimpleNamespace
    monkeypatch.setattr(launcher,'OwnedCommands',lambda *_:pytest.fail('Plan must not start a process owner'))
    output=tmp_path/'preview'
    monkeypatch.setattr(sys,'argv',['launcher','--inner','indi','--pattern','lemniscate',
        '--schedule','indi_shaped_angular','--speed','1.5','--radius-m','2','--plan-only','--output',str(output)])
    assert launcher.main()==0
    result=json.loads(capsys.readouterr().out)
    config=load_json(output / 'configuration.json')
    assert result['trajectory']['radius_m']==config['trajectory']['radius_m']==2.
    assert result['network_or_hardware_access'] is False
    monkeypatch.setattr(launcher,'ROOT',tmp_path)
    args=SimpleNamespace(output=None,test='trajectory',pattern='lemniscate',inner='indi',speed=1.5,radius_m=2.,bench_check=False)
    run=launcher.create_run_directory(args,config)
    assert '_R2m_' in run.name


@pytest.mark.parametrize('test_name',['pd-hover','roll-pulse','pitch-pulse','yaw-pulse','roll-sine','pitch-sine','mpc-hover'])
def test_diagnostic_plan_has_no_trajectory_or_live_process(monkeypatch,capsys,test_name):
    monkeypatch.setattr(launcher,'OwnedCommands',lambda *_:pytest.fail('Plan must not start a process owner'))
    monkeypatch.setattr(sys,'argv',['launcher','--test',test_name,'--inner','indi','--plan-only'])
    assert launcher.main()==0
    result=json.loads(capsys.readouterr().out)
    assert result['test']==test_name and result['diagnostic']
    assert result['trajectory'] is None and result['speed_m_s'] is None and result['pattern'] is None
    assert result['network_or_hardware_access'] is False
    if test_name!='mpc-hover':
        assert result['mpc'] is None
        assert result['benchmark_envelope'] is None


@pytest.mark.parametrize('extra',[['--speed','1.5'],['--pattern','circle'],['--schedule','variable_angular'],['--radius-m','2']])
def test_diagnostic_rejects_misleading_trajectory_arguments(monkeypatch,extra):
    monkeypatch.setattr(sys,'argv',['launcher','--test','roll-pulse','--inner','pid',*extra])
    with pytest.raises(SystemExit) as exc:launcher.main()
    assert exc.value.code==2


def test_owned_failed_child_is_reaped_and_reported(tmp_path):
    owner=launcher.OwnedCommands(tmp_path)
    try:
        with pytest.raises(RuntimeError):owner.run('failed',[sys.executable,'-c','raise SystemExit(7)'],2.)
        assert owner.rows[0]['returncode']==7 and owner.rows[0]['remaining_group_pids']==[]
        with pytest.raises(ProcessLookupError):os.killpg(owner.rows[0]['pgid'],0)
    finally:owner.close()


def test_owned_timeout_interrupts_and_reaps_child(tmp_path):
    owner=launcher.OwnedCommands(tmp_path)
    try:
        with pytest.raises(RuntimeError):owner.run('timeout',[sys.executable,'-c','import time;time.sleep(60)'],.1)
        assert owner.rows[0]['timed_out'] and owner.rows[0]['remaining_group_pids']==[]
        with pytest.raises(ProcessLookupError):os.killpg(owner.rows[0]['pgid'],0)
    finally:owner.close()


def test_cancellation_blocks_next_stage_even_if_child_exits_zero(tmp_path):
    owner=launcher.OwnedCommands(tmp_path)
    def cancel_when_ready():
        end=time.monotonic()+3.
        while time.monotonic()<end:
            p=tmp_path/'cancellable.log'
            if p.exists() and 'READY' in p.read_text():break
            time.sleep(.005)
        owner.interrupt(signal.SIGINT,None)
    thread=threading.Thread(target=cancel_when_ready)
    thread.start()
    try:
        command=[sys.executable,'-u','-c',
            'import signal,time,sys;signal.signal(signal.SIGINT,lambda *_:sys.exit(0));print("READY");time.sleep(60)']
        with pytest.raises(KeyboardInterrupt):owner.run('cancellable',command,5.)
        thread.join(timeout=3)
        assert owner.rows[0]['returncode']==0 and owner.rows[0]['remaining_group_pids']==[]
        with pytest.raises(KeyboardInterrupt):owner.run('must_not_start',[sys.executable,'-c','pass'],2.)
        assert len(owner.rows)==1
    finally:thread.join(timeout=3);owner.close()


def test_ledger_write_failure_still_reaps_newly_started_child(tmp_path,monkeypatch):
    owner=launcher.OwnedCommands(tmp_path);original=owner.save;calls=[]
    def fail_once():
        calls.append(True)
        if len(calls)==1:raise OSError('ledger unavailable')
        original()
    monkeypatch.setattr(owner,'save',fail_once)
    try:
        with pytest.raises(OSError):owner.run('ledger_failure',[sys.executable,'-c','import time;time.sleep(60)'],5.)
        assert owner.rows[0]['remaining_group_pids']==[]
        with pytest.raises(ProcessLookupError):os.killpg(owner.rows[0]['pgid'],0)
    finally:owner.close()


@pytest.mark.parametrize('bad_floor',[False,True])
def test_disabled_bench_or_bad_floor_prevents_flight(monkeypatch,tmp_path,bad_floor):
    called=[]
    class Owner:
        def __init__(self,output):self.output=output;self.rows=[]
        def run(self,name,command,timeout):
            called.append(name)
            if name=='disabled_bench':
                if not bad_floor:raise RuntimeError('Receiver not ready')
                p=self.output/'bench';p.mkdir()
                (p/'result.json').write_text(json.dumps(dict(success=True,disabled_only=True,shutdown_acknowledged=True)))
                (p/'events.jsonl').write_text('')
        def close(self):pass
    monkeypatch.setattr(launcher,'OwnedCommands',Owner)
    output=tmp_path/'case'
    monkeypatch.setattr(sys,'argv',['launcher','--inner','indi','--fly','--frames-confirmed','--output',str(output)])
    assert launcher.main()==1
    assert called==['clock_preflight','disabled_bench']
    result=load_json(output / 'run_summary.json')
    assert not result['success'] and not result['flight_started']


def test_shorter_launch_still_requires_clock_disabled_bench_and_fresh_floor(monkeypatch,tmp_path):
    called=[]
    class Owner:
        def __init__(self,output):self.output=output;self.rows=[]
        def run(self,name,command,timeout):
            called.append((name,[str(x) for x in command]))
            if name=='disabled_bench':
                p=self.output/'bench';p.mkdir()
                (p/'result.json').write_text(json.dumps(dict(success=True,disabled_only=True,shutdown_acknowledged=True)))
                now=bench_poses(p/'events.jsonl')
                monkeypatch.setattr(launcher.time,'monotonic_ns',lambda:now)
            elif name=='flight':
                p=self.output/'flight';p.mkdir()
                (p/'result.json').write_text(json.dumps(dict(success=True,reason='test_complete')))
        def close(self):pass
    monkeypatch.setattr(launcher,'OwnedCommands',Owner)
    monkeypatch.setattr(launcher,'snapshot_sources',lambda *_:None)
    monkeypatch.setattr(launcher,'lab_process_inventory',lambda:dict(processes=[]))
    output=tmp_path/'case'
    monkeypatch.setattr(sys,'argv',['launcher','--test','pd-hover','--inner','indi','--fly','--frames-confirmed','--output',str(output)])
    assert launcher.main()==0
    assert [n for n,_ in called]==['clock_preflight','disabled_bench','flight']
    bench=called[1][1]
    assert bench[bench.index('--minimum-bench-seconds')+1]=='3'
    config=load_json(output / 'configuration.json')
    assert config['limits']['ground_m']==pytest.approx(.04)
    assert config['ground_indi']['warmup_s']==.6
    assert config['ground_indi']['spool_s']==1.5
