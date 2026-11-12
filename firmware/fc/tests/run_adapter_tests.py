#!/usr/bin/env python3
"""Actual candidate adapter regression + speed-model diagnostics, native only."""
from pathlib import Path
import json
import subprocess

HERE = Path(__file__).resolve().parent
import argparse
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--source',type=Path,required=True)
bf=parser.parse_args().source.resolve()
native = HERE/'adapter'
asroot = bf/'lib/main/ActiveSetCtlAlloc/src'
files = ['common/setupWLS.c', 'common/solveActiveSet.c', 'solveActiveSet_qr.c',
         'solveActiveSet_qr_naive.c', 'solveActiveSet_chol.c', 'lib/chol_math.c',
         'lib/qr_updates.c', 'lib/qr_wrapper.c', 'lib/qr_solve/qr_solve.c',
         'lib/qr_solve/r8lib_min.c', 'lib/sparse_math.c']
logs=[]
for sanitized in (False, True):
    exe=native/('test_adapter_sanitized' if sanitized else 'test_adapter')
    cmd=['gcc','-O1' if sanitized else '-O2','-fno-fast-math','-fno-finite-math-only',
         '-ffp-contract=off','-fno-lto','-std=gnu17','-Wall','-Wextra','-Werror',
         '-DAS_N_U=4','-DAS_N_V=4','-DAS_SINGLE_FLOAT',
         '-I'+str(native),'-I'+str(native/'adapter_stubs'),'-I'+str(bf/'src/main'),
         '-I'+str(bf/'src/main/flight'),'-I'+str(asroot/'common'),'-I'+str(asroot/'lib'),
         str(native/'test_adapter.c'),str(bf/'src/main/flight/sanity_indi.c'),
         str(bf/'src/main/flight/sanity_indi/core.c'),
         *[str(bf/'src/main/flight/sanity_indi'/f) for f in
           ['rate_feedforward.c','paper_core.c','paper_adapter.c','paper_native.c','paper_rpm.c','supervisor.c']],
         *[str(asroot/f) for f in files],'-lm','-o',str(exe)]
    if sanitized:cmd+=['-fsanitize=address,undefined','-fno-omit-frame-pointer']
    for command in (cmd,[str(exe)]):
        r=subprocess.run(command,cwd=native,text=True,capture_output=True)
        logs.append(dict(command=command,returncode=r.returncode,stdout=r.stdout,stderr=r.stderr))
        (HERE/'adapter_test_results.json').write_text(json.dumps(logs,indent=2)+'\n')
        print(r.stdout,end='');print(r.stderr,end='');r.check_returncode()
