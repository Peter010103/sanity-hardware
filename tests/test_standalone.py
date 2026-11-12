"""Public recipe, same-time analysis and descendant cleanup regressions."""
import json
from pathlib import Path
import os
import subprocess
import sys
from types import SimpleNamespace
import pytest

from configure_lab import ROOT,configuration,load_lab
from recipes import RECIPES
from session import OwnedSession
from analyze_flight import read_controls,metrics


@pytest.mark.parametrize('recipe', sorted(RECIPES))
def test_public_recipe_plans_from_an_unrelated_working_directory(tmp_path,recipe):
    # Plan subprocesses may load code/config, but never start ROS or a sender.
    output=tmp_path/'plan'
    r=subprocess.run([sys.executable,str(ROOT/'scripts/run_mpc.py'),recipe,'--output',str(output),'--plan-only'],cwd=tmp_path,text=True,capture_output=True,timeout=20)
    assert r.returncode==0,r.stderr
    plan=json.loads((output/'launch_plan.json').read_text())
    assert plan['network_or_hardware_access'] is False
    assert '--plan-only' in plan['command']
    assert not (output/'owned_processes.json').exists()


@pytest.mark.parametrize('flags',[['--fly'],['--fly','--frames-confirmed'],['--bench-check']])
def test_live_requires_an_explicit_local_configuration_before_startup(tmp_path,flags):
    output=tmp_path/'never-created'
    r=subprocess.run([sys.executable,str(ROOT/'scripts/run_mpc.py'),'indi-lemniscate-r2-3','--output',str(output),*flags],text=True,capture_output=True,timeout=10)
    assert r.returncode==2 and not output.exists()


def test_paired_pid_uses_the_same_aircraft_identity_as_indi():
    lab=load_lab(ROOT/'config/lab.example.json')
    pid=configuration(lab,RECIPES['paired-pid-0p8']);indi=configuration(lab,RECIPES['indi-lemniscate-r2-3'])
    assert pid['device']==indi['device']==15
    assert pid['limits']['expected_uid']==indi['limits']['expected_uid']
    assert pid['mpc']['model']==indi['mpc']['model']


def test_adopted_child_in_a_new_session_is_stopped_and_reaped(tmp_path):
    session=OwnedSession(tmp_path/'owned.json')
    code='import subprocess,sys,time; subprocess.Popen([sys.executable,"-c","import time; time.sleep(60)"],start_new_session=True); time.sleep(0.2)'
    try:assert session.run([sys.executable,'-c',code],cwd=tmp_path,env=os.environ.copy())==0
    finally:session.close()
    rows=json.loads((tmp_path/'owned.json').read_text())['processes']
    assert len(rows)>=2 and all(row['exited'] for row in rows)


def test_analysis_keeps_recorded_times_and_does_not_fit_away_tracking_error(tmp_path):
    path=tmp_path/'control.jsonl'
    rows=[]
    for i in range(3):
        rows.append(dict(time_s=i*.02,state=[i+.1,0,1,1,0,0,0,1,0,0,0,0,0],reference=dict(position=[i,0,1],velocity=[1,0,0]),policy_weight=1. if i else 0.,phase='tracking',command=[.7,0,0,0]))
    path.write_text(''.join(json.dumps(r)+'\n' for r in rows))
    source,controls=read_controls(tmp_path);result=metrics(controls)
    assert source==path and [r['time_s'] for r in controls]==[0.,.02,.04]
    assert result['moving_full_mpc_samples']==2
    assert result['xyz_position_rmse_m']==pytest.approx(.1)
    assert result['velocity_3d_rmse_m_s']==0.
