#!/usr/bin/env python3
"""Reproduce the packaged bifilar dataset, figures and inertia correction."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

import numpy as np

from calculate_inertia import component_correction, parallel_axis

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
DATA=ROOT/'data/processed/inertia/20261019'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'build/inertia')
    parser.add_argument('--update-results',action='store_true',help='Refresh the curated processed JSON and report figures after reproducing')
    args=parser.parse_args();out=args.output.resolve()
    for script,extra in [
        ('visualize_fixture.py',['--save-only','--output',str(out/'preview')]),
        ('visualize_trial_mountings.py',['--save-only','--output',str(out/'preview')]),
        ('plot_collection_timeseries.py',['--output',str(out/'timeseries')]),
        ('calculate_inertia.py',['--output',str(out)])]:
        subprocess.run([sys.executable,str(HERE/script),*extra],check=True)
    inputs=json.loads((DATA/'measurement_inputs.json').read_text())
    estimates=json.loads((out/'inertia_estimates.json').read_text())
    hashes={}
    for trial in inputs['trials']:
        path=ROOT/trial['raw_run_directory']/'poses.csv'
        hashes[trial['axis_label']]=hashlib.sha256(path.read_bytes()).hexdigest()==trial['pose_csv_sha256']
    if not all(hashes.values()):raise ValueError('Raw recording hash mismatch')
    correction=component_correction(inputs)
    G=np.array(correction['combined_COM_F_m']);gd=np.array(correction['assumed_drone_COM_F_m'])
    mp=correction['equivalent_uniform_plywood_mass_kg'];mm=inputs['mass_ledger']['marker_mass_each_kg_approx']
    gp=np.array(inputs['fixture_F']['centroid_F_m']);markers=np.array([m['center_F_m'] for m in inputs['markers'].values()])
    about_drone=mp*np.array(inputs['fixture_F']['centroidal_inertia_per_kg_m2'])+mp*parallel_axis(gp-gd)+sum(mm*parallel_axis(r-gd) for r in markers)
    independent=about_drone-correction['component_mass_sum_kg']*parallel_axis(G-gd)
    if not np.allclose(independent,correction['total_subtraction_F_kg_m2'],atol=1e-16,rtol=0):
        raise ValueError('Parallel-axis identity failed')
    import xml.etree.ElementTree as ET
    block=ET.parse(ROOT/'models/sanity.urdf').getroot().find("link[@name='base_link']/inertial")
    mass=float(block.find('mass').get('value'));inertia=block.find('inertia')
    selected=inputs['model_update']
    if not np.isclose(mass,selected['mass_kg']):raise ValueError('URDF mass differs from selected dataset')
    tensor=np.diag([float(inertia.get(k)) for k in ['ixx','iyy','izz']])
    if not np.allclose(np.diag(tensor),selected['inertia_diagonal_kg_m2'],atol=0,rtol=1e-12):raise ValueError('URDF inertia differs from selected dataset')
    rounded=np.array([float(f"{estimates['axes'][axis]['main']['drone_inertia_candidate_kg_m2']:.3g}") for axis in ['roll','pitch','yaw']])
    if not np.array_equal(np.diag(tensor),rounded):raise ValueError('Rounded reproduced estimates differ from selected URDF')
    moments=np.linalg.eigvalsh(tensor)
    if not (np.all(moments>0) and np.all(moments.sum()-2*moments>0)):raise ValueError('Nonphysical selected inertia')
    for name,value in estimates['axes'].items():
        if not estimates['period_sensitivity_review']['axes'][name]['working_period_supported']:
            raise ValueError('Working period not supported for '+name)
    if estimates['axes']['roll']['main']['small_angle_fit_passed']:raise ValueError('Roll waveform gate was unexpectedly relaxed')
    validation={'raw_hashes_unchanged':hashes,'independent_parallel_axis_identity':True,
                'selected_URDF_parameters_match':True,'URDF_matches_rounded_reproduced_estimates':True,'inertia_positive_and_triangle_valid':True,
                'all_periods_robust_to_outliers':True,'waveform_failures_retained':True,
                'calibrated_geometry_uncertainty_claimed':False,'hardware_access':False}
    (out/'validation.json').write_text(json.dumps(validation,indent=2)+'\n')
    if args.update_results:
        for name in ['inertia_estimates.json','fit_sensitivity_check.json','validation.json']:
            shutil.copy2(out/name,DATA/name)
        for name in ['registration.json','nominal_fixture_properties.json']:
            shutil.copy2(out/'preview'/name,DATA/name)
        for source,target in [
            (out/'preview/fixture_drone_and_markers_assembly.png','setup.png'),
            (out/'preview/trial_mounting_sequence.png','mountings.png'),
            (out/'timeseries/collection_zoom_0_10s.png','timeseries.png'),
            (out/'fit_diagnostics.png','fits.png'),
            (out/'inertia_breakdown.png','inertia.png')]:
            shutil.copy2(source,ROOT/'media/inertia'/target)
    print('Reproduction and consistency checks passed: '+str(out),flush=True)


if __name__=='__main__':main()
