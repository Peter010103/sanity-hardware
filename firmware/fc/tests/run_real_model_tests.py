#!/usr/bin/env python3
"""Actual selected speed model/servo grids under both transfer policies."""
import json
from pathlib import Path
import subprocess

HERE=Path(__file__).resolve().parent
SRC=HERE.parent/'source/src/main/flight/sanity_indi'
logs=[]
for policy in (0,1):
    exe=HERE/f'test_real_model_{policy}'
    cmd=['gcc','-std=c11','-O2','-fno-fast-math','-ffp-contract=off',
         '-Wall','-Wextra','-Werror',f'-DSP_FF_ACCEPT_TRANSFER={policy}',
         '-I'+str(SRC),str(HERE/'test_real_model.c'),str(SRC/'paper_core.c'),
         '-lm','-o',str(exe)]
    subprocess.run(cmd,check=True)
    run=subprocess.run([str(exe)],capture_output=True,text=True)
    (HERE/f'real_model_grid_{policy}.csv').write_text(run.stdout)
    logs.append(dict(policy=policy,command=cmd,returncode=run.returncode,
                     stderr=run.stderr,grid_rows=len(run.stdout.splitlines())-1))
    (HERE/'real_model_results.json').write_text(json.dumps(logs,indent=2)+'\n')
    run.check_returncode()
    print(f'PASS actual model grid policy{policy}: {logs[-1]["grid_rows"]} motor rows')
