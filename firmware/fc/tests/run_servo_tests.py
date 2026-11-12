#!/usr/bin/env python3
"""Offline tests of actual candidate servo; leaves no background processes."""
import hashlib
import json
from pathlib import Path
import subprocess

HERE = Path(__file__).resolve().parent
SRC = HERE.parent / 'source/src/main/flight/sanity_indi'
results = []
for policy in (0, 1):
    for sanitized in (False, True):
        exe = HERE / f'test_servo_{policy}{"_sanitized" if sanitized else ""}'
        command = ['gcc', '-std=c11', '-O1' if sanitized else '-O2',
                   '-fno-fast-math', '-fno-finite-math-only', '-ffp-contract=off',
                   '-Wall', '-Wextra', '-Werror',
                   '-DSP_SPEED_MODEL_HEADER="speed_model_test_stub.h"',
                   f'-DSP_FF_ACCEPT_TRANSFER={policy}', '-I'+str(SRC), '-I'+str(HERE),
                   str(HERE/'test_servo.c'), str(SRC/'paper_core.c'), '-lm', '-o', str(exe)]
        if sanitized:
            command += ['-fsanitize=address,undefined', '-fno-omit-frame-pointer']
        for cmd in (command, [str(exe)]):
            run = subprocess.run(cmd, text=True, capture_output=True)
            results.append(dict(command=cmd, returncode=run.returncode,
                                stdout=run.stdout, stderr=run.stderr))
            print(run.stdout, end=''); print(run.stderr, end='')
            (HERE/'servo_test_results.json').write_text(json.dumps(results, indent=2)+'\n')
            run.check_returncode()
hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
          [SRC/'paper_core.c', SRC/'paper_core.h', HERE/'test_servo.c',
           HERE/'speed_model_test_stub.h', Path(__file__)]}
(HERE/'servo_test_hashes.json').write_text(json.dumps(hashes, indent=2)+'\n')
