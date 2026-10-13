#!/usr/bin/env python3
"""Compare a reproduced fit with distributed coefficients; no hardware I/O."""
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
old=json.loads((ROOT/'data/processed/actuator/fit_report.json').read_text())
new=json.loads((ROOT/'build/actuator_fit/fit_report.json').read_text())
checks={}
for key in ['static_thrust','extended_static_thrust','spinning_dynamics','steady_speed']:
    a=old['models'][key];b=new['models'][key]
    assert a['candidate']==b['candidate'],key
    assert np.allclose(a['fitted_parameters'],b['fitted_parameters'],rtol=1e-9,atol=1e-12),key
    checks[key]='matches'
for a,b in zip(old['models']['end_to_end_test'],new['models']['end_to_end_test'],strict=True):
    assert a['run']==b['run']
    for quantity in ['rpm','observed_loadcell_force_N']:
        assert abs(a[quantity]['rmse']-b[quantity]['rmse'])<1e-8,(a['run'],quantity)
checks['heldout_metrics']='match'
exported=json.loads((ROOT/'build/actuator_fit/actuators.json').read_text())
runtime=json.loads((ROOT/'parameters/actuators.json').read_text())
assert exported == runtime
checks['runtime_parameters']='match'
(ROOT/'build/refit_verification.json').write_text(json.dumps(checks,indent=2)+'\n')
print(json.dumps(checks,indent=2))
