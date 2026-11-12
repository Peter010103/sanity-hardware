"""Persist the wrapper's verified startup clock receipt before physical connections."""
import json
import os
from pathlib import Path
import time


def record_clock_preflight(output, required=False):
    raw = os.environ.get('SANITY_CLOCK_PREFLIGHT_JSON')
    if not raw:
        if required:
            raise ValueError('Clock preflight missing; launch through the mission run_sanity_*.sh wrapper')
        return None
    report = json.loads(raw)
    age = (time.monotonic_ns()-int(report['verified_monotonic_ns']))/1e9
    if (report.get('passed') is not True or report.get('server')!=os.environ.get('SANITY_MOCAP_NTP_SERVER','10.0.0.251')
            or not 0<=age<=120 or len(report.get('samples', []))!=3
            or any(not 0<=sample['conservative_absolute_offset_ms']<1. for sample in report['samples'])):
        raise ValueError('Clock preflight invalid/expired; rerun the mission wrapper')
    Path(output,'clock_sync.json').write_text(json.dumps(report,indent=2)+'\n')
    return report
