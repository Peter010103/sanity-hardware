"""Incremental, offline-only analysis of archived Sanity15 control diagnostics."""
from pathlib import Path
import argparse
import csv
import datetime
import fcntl
import hashlib
import json
import sys
from sanity_control_analysis import TESTS,analyse,discover,sha,write_json

ROOT=Path(__file__).resolve().parents[4]

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs-root',type=Path,default=ROOT/'outputs')
    parser.add_argument('--output',type=Path,default=ROOT/'outputs/sanity15_control_analysis_20261110')
    parser.add_argument('--decoder',type=Path,default=ROOT/'outputs/sanity15_rate_feedforward_20261110/decoder/obj/blackbox_decode')
    parser.add_argument('--no-plots',action='store_true')
    args=parser.parse_args();output=args.output.resolve();output.mkdir(parents=True,exist_ok=True)
    if not args.decoder.is_file():parser.error('Offline decoder is missing; specify --decoder')
    with (output/'.analysis.lock').open('a') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:parser.error('Another offline analysis is using this output directory')
        return run_analysis(args,output)

def run_analysis(args,output):
    inventory=discover(args.runs_root);datasets={};errors=[]
    for row in inventory:
        if row['state']!='archived':continue
        try:
            data=analyse(row,args.decoder.resolve(),output/'cache')
            datasets[row['run']]=data;row['state']='analysed'
            write_json(output/'runs'/row['run']/'metrics.json',data['metrics'])
            print('Analysed',row['run'],'recording',data['metrics']['selected_recording'],flush=True)
        except (OSError,ValueError,KeyError,IndexError) as exc:
            row['state']='analysis_needs_review';row['error']=str(exc);errors.append(dict(run=row['run'],error=str(exc)))
            print('Needs review:',row['run'],str(exc),file=sys.stderr,flush=True)
    selected={}
    for row in sorted(inventory,key=lambda r:(r['started_unix'],r['run'])):
        if row['success'] and row['run'] in datasets:selected[row['test']+'/'+row['inner']]=row['run']
    # A completed baseline must not silently become a mixture of gain revisions
    # after the next tuning flights. A new output directory remains incremental.
    selection_path=output/'baseline_selection.json'
    if selection_path.is_file():
        pinned=json.loads(selection_path.read_text())['selected']
        for key,name in pinned.items():
            if name not in datasets or key!=datasets[name]['metrics']['test']+'/'+datasets[name]['metrics']['inner']:
                raise ValueError('Pinned baseline recording unavailable or mismatched: '+name)
        selected=pinned
    missing=[test+'/'+inner for test in TESTS for inner in ('pid','indi') if test+'/'+inner not in selected]
    harmonic_rows=[]
    for key,name in selected.items():
        m=datasets[name]['metrics']
        for h in m['harmonics']:harmonic_rows.append(dict(run=name,test=m['test'],inner=m['inner'],**h))
    result=dict(version=1,created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        scope='Offline recorded-data analysis; no network, ROS, serial, aircraft commands or parameter changes',
        signal_scope=dict(rates='Native FRD. Host body FLU is converted by diag(1,-1,-1).',
            demand='FC p46..48 /1e6 rad/s after compatibility RC processing',
            early_gyro='FC p28..30 /1e6 rad/s after downsampling/RPM filtering, before ordinary BF filters',
            H='FC p31..33 /1e6 rad/s actual synchronization output; session setting12Hz; not PID internal feedback',
            filtering='No added filtering, smoothing, resampling, interpolation or fitted time-shift correction',
            sine_estimator='Least-squares sinusoid+cosinusoid+offset+linear trend, on raw FC samples; estimation only, never substituted into plots',
            PID_shadow='INDI target/allocator/servo values under PID are unused; omitted from PID actuation/tracking metrics',
            sampling='Every fourth1kHz control tick; nominal250Hz, actual about248Hz. p124 is previous CONTROL tick, not previous recorded row.'),
        analysed_successful_cases=len(selected),expected_cases=12,collection_complete=not missing,
        selection_policy='pinned pre-tuning baseline' if selection_path.is_file() else 'latest successful archived run per case',
        pending_cases=missing,inventory=inventory,selected=selected,errors=errors,
        runs={name:d['metrics'] for name,d in datasets.items()},harmonic_response=harmonic_rows,
        limitations=['One or few flights per case; closed-loop PD correction and initial motion differ.',
            'Status alignment is approximate; same-FC demand/gyro metrics do not use cross-device delay fits.',
            'Harmonic gain/phase are descriptive closed-loop ratios, not unique open-loop plant identification.',
            'Low harmonic amplitude or large residual motion is flagged, not silently removed.',
            'These tests use desktop mocap for control; onboard ESKF logging is not a control input.',
            'Baseline collection is complete. Candidate tuning requires new matched recordings before improvement can be claimed.'])
    takeoff=output/'takeoff_review/host_attempts.json'
    if takeoff.is_file():
        result['takeoff_review']=json.loads(takeoff.read_text())
    write_json(output/'results.json',result)
    if harmonic_rows:
        fields=['run','test','inner','frequency_hz','demand_amplitude_rad_s','gyro_amplitude_rad_s','gain','phase_deg','demand_residual_RMS_rad_s','gyro_residual_RMS_rad_s','samples','weak_demand','response_harmonic_below_residual']
        with (output/'sine_frequency_response.csv').open('w') as stream:
            writer=csv.DictWriter(stream,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(harmonic_rows)
    if not args.no_plots:
        from sanity_control_analysis_plots import make_report
        make_report(output,result,datasets)
    sources={}
    for name in ['analyze_sanity_control_tests.py','sanity_control_analysis.py','sanity_control_analysis_plots.py','sanity_control_report_findings.py','analyze_sanity_control_tests.sh']:
        path=Path(__file__).with_name(name)
        if path.exists():sources[str(path.relative_to(ROOT))]=sha(path)
    write_json(output/'provenance.json',dict(decoder=str(args.decoder.resolve()),decoder_sha256=sha(args.decoder),
        analysis_source_sha256=sources,raw_receipt_verification='All analyzed raw files match their stored byte count and SHA256. No FC was contacted.',
        cached_decode='Reused only when raw/decoder and decoded CSV hashes agree. New/changed archives decode separately.'))
    print(f'{len(selected)}/12 cases analysed; {len(missing)} pending; {len(errors)} need review.',flush=True)
    print('Results:',output/'results.json',flush=True)
    return 1 if errors else 0

if __name__=='__main__':raise SystemExit(main())
