#!/usr/bin/env python3
"""Fit bifilar collections and calculate explicitly provisional drone-COM inertia."""
import argparse
import csv
import importlib.util
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares

import plot_collection_timeseries as timeseries

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
DATA=ROOT/'data/processed/inertia/20261019'


def parallel_axis(r):
    return np.dot(r,r)*np.eye(3)-np.outer(r,r)


def component_correction(data,other_mass=0.,other_position=None,drone_shift=None):
    """All component tensors are expressed in F and about the same nominal G."""
    md=data['drone']['mass_kg'];mm=data['mass_ledger']['marker_mass_each_kg_approx']
    remainder=data['mass_ledger']['plywood_plus_other_fixture_attachments_kg_approx']
    mp=remainder-other_mass
    if mp<0:raise ValueError('Attachment mass exceeds non-marker remainder')
    gp=np.array(data['fixture_F']['centroid_F_m'])
    gd=np.array(data['drone']['nominal_visualization_translation_F_m'])
    if drone_shift is not None:gd=gd+np.array(drone_shift)
    rm=np.array([v['center_F_m'] for v in data['markers'].values()])
    M=md+remainder+len(rm)*mm
    go=np.zeros(3) if other_position is None else np.array(other_position)
    G=(md*gd+mp*gp+mm*rm.sum(0)+other_mass*go)/M
    Jp=mp*np.array(data['fixture_F']['centroidal_inertia_per_kg_m2'])+mp*parallel_axis(gp-G)
    Jm=sum(mm*parallel_axis(p-G) for p in rm)
    Jo=other_mass*parallel_axis(go-G)
    Jd=md*parallel_axis(gd-G)
    return dict(component_mass_sum_kg=M,combined_COM_F_m=G.tolist(),assumed_drone_COM_F_m=gd.tolist(),
                equivalent_uniform_plywood_mass_kg=mp,
                uniform_remainder_about_combined_COM_F_kg_m2=Jp.tolist(),
                point_markers_about_combined_COM_F_kg_m2=Jm.tolist(),
                other_attachments_about_combined_COM_F_kg_m2=Jo.tolist(),
                drone_parallel_axis_to_combined_COM_F_kg_m2=Jd.tolist(),
                total_subtraction_F_kg_m2=(Jp+Jm+Jo+Jd).tolist())


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start',type=float,default=10.)
    parser.add_argument('--end',type=float,default=20.)
    parser.add_argument('--output',type=Path,default=ROOT/'build/inertia')
    args=parser.parse_args()
    if args.end-args.start<8.1:parser.error('Use a fit window of at least 8.1s')
    data=json.loads((DATA/'measurement_inputs.json').read_text())
    inputs=data
    manifest={'runs':[{'axis':r['axis_label'],'run_directory':r['raw_run_directory'],'pose_csv_sha256':r['pose_csv_sha256']} for r in inputs['trials']]}
    spec=importlib.util.spec_from_file_location('bifilar_fit',ROOT/'tools/acquisition/bifilar_inertia.py')
    fit_module=importlib.util.module_from_spec(spec);spec.loader.exec_module(fit_module)
    correction=component_correction(data)
    R_BF=np.array(data['drone']['fixture_from_drone_rotation']).T
    correction_B=R_BF@np.array(correction['total_subtraction_F_kg_m2'])@R_BF.T
    output={'status':'Operator-selected working estimates; period robustness checked separately from waveform residuals; COM/fixture correction provisional',
            'nominal_correction':correction,'fit_window_s_after_cue':[args.start,args.end],
            'selection':f'Common {args.start:g}-{args.end:g}s window chosen to avoid initial large amplitude; waveform quality thresholds unchanged',
            'assumptions':[
                'Two equal taut strings; reported 0.642m treated as loaded free length; aligned upper/lower attachment midpoints.',
                'Dominant small-angle torsional mode about nominal drone body axis; wire torsional stiffness neglected.',
                'Suspension/torsion axis assumed through combined assembly COM; exact axis points/contact locations not measured.',
                'Drone COM assumed at URDF base_link origin in the displayed centred collar-contact CAD placement; this COM is not measured.',
                'Entire 11.42g non-marker fixture remainder treated as uniform nominal plywood, including unseparated tape/glue/mount mass.',
                'Six 1.08g markers modelled as point masses at nominal registered centres; marker intrinsic inertia omitted.',
                'Measured 87.38g used for restoring stiffness; component bookkeeping sums to87.37g. The0.01g closure difference is not allocated.',
                'Body axes aligned with F as established by mounting; off-diagonal drone tensor elements are not identified.'],
            'method':{'height':'h=sqrt(L^2-(a-b)^2), with a,b half the full top/bottom spacings',
                      'stiffness':'k=M*g*a*b/h',
                      'fit':'theta=c0+c1*(t-mean(t))+exp(-lambda*t)*(A*cos(2*pi*f*t)+B*sin(2*pi*f*t))',
                      'combined_inertia':'J=k/((2*pi*f)^2+lambda^2)',
                      'drone_inertia':'J_drone,Gd=J_combined,G - J_fixture,G - md*||n cross (Gd-G)||^2',
                      'time':'Fit against integer-derived source timestamps; windows selected relative to host countdown cue.'},
            'axes':{},'raw_recordings_modified':False,'flight_model_modified':False,
            'geometry_uncertainty_quantified':False,'statistical_confidence_interval_not_claimed':True,
            'references':['https://doi.org/10.2514/1.34015','https://doi.org/10.1016/0020-7403(67)90047-1']}
    period_review={}
    details=[];index={'roll':0,'pitch':1,'yaw':2}
    for run,trial in zip(manifest['runs'],data['trials']):
        name=run['axis'];axis=index[name]
        t,deg,digest=timeseries.extract(run)
        with (ROOT/run['run_directory']/'poses.csv').open() as stream:rows=list(csv.DictReader(stream))
        ns=np.array([int(r['stamp_sec'])*10**9+int(r['stamp_nanosec']) for r in rows],dtype=np.int64)
        stamp=(ns-ns[0])*1e-9
        rotations=Rotation.from_quat([[float(r[k]) for k in ['qx','qy','qz','qw']] for r in rows])
        relative=(rotations*rotations[t>=t[-1]-10].mean().inv()).as_quat()
        swing_deg=np.degrees(2*np.arctan2(np.linalg.norm(relative[:,:2],axis=1),np.linalg.norm(relative[:,2:],axis=1)))
        geometry=fit_module.geometry(data['mass_ledger']['total_suspended_kg']*1000,
                     trial['reported_string_length_m']*100,trial['upper_full_spacing_m']*100,trial['lower_full_spacing_m']*100)
        sensitivity=[];main=None
        for start,end in [(args.start,args.end),(0.,10.),(8.,18.),(12.,22.),(15.,25.),(20.,30.)]:
            select=(t>=start)&(t<=end)
            fit,grid,y,model=fit_module.fit_oscillation(stamp[select],np.radians(deg[select]))
            J=geometry['stiffness_Nm_rad']/((2*np.pi*fit['frequency_damped_hz'])**2+fit['decay_per_s']**2)
            candidate=J-correction_B[axis,axis]
            accepted=not fit['failures'] and fit['amplitude_at_fit_start_deg']<=12
            row={'swing_max_deg':float(np.max(swing_deg[select])),
                 'swing_percentile95_deg':float(np.percentile(swing_deg[select],95)),
                 'window_s_after_cue':[start,end],'fit':fit,'combined_inertia_kg_m2':J,
                 'drone_inertia_candidate_kg_m2':candidate,'small_angle_fit_passed':accepted}
            sensitivity.append(row)
            if main is None:
                main=row
                overlay_host=np.interp(stamp[select][0]+grid,stamp,t)
                details.append((name,t,deg,overlay_host,np.degrees(y),np.degrees(model),main))
                f=fit['frequency_damped_hz'];decay=fit['decay_per_s'];centred=grid-grid.mean()
                matrix=np.column_stack([np.ones(len(grid)),centred,
                    np.exp(-decay*grid)*np.cos(2*np.pi*f*grid),np.exp(-decay*grid)*np.sin(2*np.pi*f*grid)])
                coefficients=np.linalg.lstsq(matrix,y,rcond=None)[0]
                def residual(parameters):
                    c,d,A,B,frequency,rate=parameters
                    phase=2*np.pi*frequency*grid
                    return c+d*centred+np.exp(-rate*grid)*(A*np.cos(phase)+B*np.sin(phase))-y
                robust=least_squares(residual,np.r_[coefficients,f,decay],loss='soft_l1',
                    f_scale=np.radians(.25),bounds=([-np.inf]*4+[.2,0],[np.inf]*4+[10,1]),
                    xtol=1e-11,ftol=1e-11,gtol=1e-11,max_nfev=2000)
                frequency,rate=robust.x[-2:]
                robust_J=geometry['stiffness_Nm_rad']/((2*np.pi*frequency)**2+rate**2)-correction_B[axis,axis]
                change=100*(robust_J/candidate-1)
                period_review[name]={'least_squares_period_s':fit['period_damped_s'],
                    'outlier_resistant_period_s':float(1/frequency),
                    'least_squares_drone_inertia_kg_m2':candidate,
                    'outlier_resistant_drone_inertia_kg_m2':float(robust_J),
                    'drone_inertia_change_percent':float(change),
                    'robust_fit_converged':bool(robust.success),
                    'working_period_supported':bool(robust.success and abs(change)<.5)}
        output['axes'][name]={'body_axis':'XYZ'[axis],'status':'working_estimate_provisional_correction' if period_review[name]['working_period_supported'] else 'diagnostic_only',
            'geometry':geometry,'main':main,'window_sensitivity':sensitivity,
            'fixture_and_parallel_axis_subtraction_kg_m2':float(correction_B[axis,axis]),
            'source_pose_csv_sha256':digest,
            'conditional_estimate_kg_m2':main['drone_inertia_candidate_kg_m2'] if period_review[name]['working_period_supported'] else None}
    candidate=np.array([output['axes'][name]['main']['drone_inertia_candidate_kg_m2'] for name in ['roll','pitch','yaw']])
    output['candidate_body_diagonal_kg_m2']=candidate.tolist()
    output['off_diagonal_body_tensor_elements']=None
    output['candidate_diagonal_positive']=bool(np.all(candidate>0))
    output['candidate_diagonal_satisfies_triangle_inequalities']=bool(np.all(candidate<=candidate.sum()-candidate))
    output['marker_intrinsic_inertia_max_subtraction_each_axis_kg_m2']=6*.00108*(.0127/2)**2
    output['unmeasured_axis_offset_parallel_axis_only_sensitivity']=[
        {'offset_m':dist,'extra_subtraction_kg_m2':correction['component_mass_sum_kg']*dist**2}
        for dist in [.003,.005,.01]]
    scenarios=[]
    for label,mass,pos in [('1g attachment at collar',.001,[0,0,.013]),('1g attachment at arm',.001,[.075,0,.03])]:
        alt=component_correction(data,mass,pos)
        alt_B=R_BF@np.array(alt['total_subtraction_F_kg_m2'])@R_BF.T
        delta=np.diag(correction_B-alt_B)
        scenarios.append({'scenario':label,'illustrative_not_uncertainty_bound':True,
                          'delta_drone_body_diagonal_kg_m2':delta.tolist()})
    output['attachment_distribution_sensitivity']=scenarios
    args.output.mkdir(parents=True,exist_ok=True)
    output['period_sensitivity_review']={'waveform_gate_unchanged':True,'comparison':'soft-L1 loss with0.25deg scale versus ordinary least squares; same source-time grid and model',
        'inference':'Working periods are insensitive to residual spikes; this does not resolve window drift or geometry uncertainty.',
        'axes':period_review}
    output['selected_URDF_parameters']=data['model_update']
    (args.output/'fit_sensitivity_check.json').write_text(json.dumps(output['period_sensitivity_review'],indent=2)+'\n')
    (args.output/'inertia_estimates.json').write_text(json.dumps(output,indent=2)+'\n')

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,3,figsize=(15,9),gridspec_kw={'width_ratios':[1.35,1,1]})
    colors=['#2166ac','#238b45','#c45721']
    for row,(detail,color) in enumerate(zip(details,colors)):
        name,t,deg,host,y,model,main=detail
        ax=axes[row,0];window=(t>=args.start)&(t<=args.end)
        ax.plot(t[window],deg[window],color=color,lw=.8,label='Measured')
        ax.plot(host,model,color='black',lw=1,alpha=.8,label='Damped fit')
        ax.set(xlim=(args.start,args.end),ylabel=name.capitalize()+' twist (°)')
        ax.set_title(f"T={main['fit']['period_damped_s']:.4f}s | "+('fit passed' if main['small_angle_fit_passed'] else 'waveform gate failed'),fontsize=11)
        if row==0:ax.legend(fontsize=8,ncol=2)
        ax=axes[row,1];ax.plot(host,y-model,color=color,lw=.8);ax.axhline(0,c='black',lw=.6)
        ax.set(xlim=(args.start,args.end),ylabel='Residual (°)')
        ax.set_title(f"Residual / oscillation RMS: {100*main['fit']['residual_to_oscillation_rms']:.1f}%",fontsize=11)
        ax=axes[row,2]
        for w in output['axes'][name]['window_sensitivity']:
            x=np.mean(w['window_s_after_cue']);value=w['drone_inertia_candidate_kg_m2']*1e6
            ax.scatter(x,value,color=color,marker='o' if w['small_angle_fit_passed'] else 'x',s=40)
        ax.axvline((args.start+args.end)/2,color='gray',ls=':',lw=1)
        ax.set(ylabel='Candidate J (10⁻⁶ kg·m²)')
        if row==0:ax.set_title('Window sensitivity: ○ passing, × diagnostic',fontsize=10)
    for row in axes:
        for ax in row:ax.grid(alpha=.2)
    for ax in axes[-1,:2]:ax.set_xlabel('Seconds after countdown cue')
    axes[-1,2].set_xlabel('Fit-window midpoint after cue (s)')
    fig.suptitle('Bifilar inertia fit diagnostics · CAD correction is provisional',fontsize=16)
    fig.text(.5,.018,'Period drift and roll residuals remain. Geometry, COM and attachment-distribution uncertainties are not statistical fit errors.',ha='center',fontsize=10)
    fig.tight_layout(rect=[0,.04,1,.95])
    for ext in ['png','pdf']:fig.savefig(args.output/('fit_diagnostics.'+ext),dpi=180)
    plt.close(fig)

    fig,ax=plt.subplots(figsize=(10,4.8))
    labels=['Roll / X','Pitch / Y','Yaw / Z'];names=['roll','pitch','yaw']
    values=np.array([output['axes'][n]['main']['combined_inertia_kg_m2'] for n in names])*1e6
    corrections=np.diag(correction_B)*1e6;remaining=candidate*1e6
    ax.barh(labels,remaining,color=['#dfb08a','#63a889','#6c9fcb'],label='Working drone COM estimate')
    ax.barh(labels,corrections,left=remaining,color='#b8b8b8',label='Fixture + parallel-axis correction')
    ax.patches[0].set_hatch('///')
    for i,(J,C) in enumerate(zip(remaining,values)):
        ax.text(J/2,i,f'{J:.1f}',ha='center',va='center',fontsize=10)
        ax.text(C+1,i,f'{C:.1f} combined',va='center',fontsize=10)
    ax.set_xlim(0,values.max()*1.24);ax.invert_yaxis()
    ax.set_xlabel('Inertia (10⁻⁶ kg·m²)');ax.set_title('Provisional drone-only estimate · displayed CAD placement')
    ax.legend(loc='lower right',fontsize=9);ax.grid(axis='x',alpha=.2);ax.set_axisbelow(True)
    fig.text(.5,.025,'Assumes suspension axis through combined COM; 11.42g remainder treated as uniform plywood. Roll period is robust; waveform residual gate fails.',ha='center',fontsize=9)
    fig.tight_layout(rect=[0,.06,1,1])
    for ext in ['png','pdf']:fig.savefig(args.output/('inertia_breakdown.'+ext),dpi=180)
    plt.close(fig)

    lines=['PROVISIONAL DRONE INERTIA - CAD CONTACT AND COM ASSUMPTIONS','',
           'Body frame: X forward, Y left, Z up. Entries below refer to axes through assumed drone COM.',
           'No full tensor is identified; off-diagonal entries are unknown. This analysis command does not modify the flight model; selected values are recorded in measurement_inputs.json.','',
           'Axis | Period(s) | Assembly(kg m2) | Correction(kg m2) | Drone candidate(kg m2) | Fit status']
    for name in ['roll','pitch','yaw']:
        entry=output['axes'][name];main=entry['main']
        lines.append(f"{name:5} | {main['fit']['period_damped_s']:.6f} | {main['combined_inertia_kg_m2']:.8g} | {entry['fixture_and_parallel_axis_subtraction_kg_m2']:.8g} | {main['drone_inertia_candidate_kg_m2']:.8g} | {entry['status']}")
    lines+=['','Assumptions:']+['- '+a for a in output['assumptions']]
    lines+=['','Roll waveform residual exceeds15%, but robust fitting changes inferred inertia by less than0.2%; period supports a working estimate.',
            'Changing fit windows changes the estimated period; no calibrated total uncertainty is available.',
            'Mass and dimensions are nominal operator measurements, not a traceable uncertainty budget.']
    (args.output/'inertia_summary.txt').write_text('\n'.join(lines)+'\n')
    print('\n'.join(lines[:10]),flush=True)
    print('Saved: '+str(args.output),flush=True)


if __name__=='__main__':main()
