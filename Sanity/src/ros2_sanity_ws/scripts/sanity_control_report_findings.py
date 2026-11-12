"""Main interpretation pages for the completed, recorded baseline collection.

No fitted or filtered replacement signals are produced here. The numerical
summary is derived from the same archived metrics as the appendix plots.
"""
import matplotlib.pyplot as plt
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from sanity_control_analysis import read_csv,sha


def table(fig, rect, rows, labels, widths, size=12, scale=1.8):
    ax=fig.add_axes(rect);ax.axis('off')
    tab=ax.table(cellText=rows,colLabels=labels,colWidths=widths,
                 cellLoc='left',loc='center')
    tab.auto_set_font_size(False);tab.set_fontsize(size);tab.scale(1,scale)
    for (r,c),cell in tab.get_celld().items():
        cell.set_edgecolor('#d4d9dc')
        if r==0:cell.set_facecolor('#eaf0f3');cell.set_text_props(weight='bold')
    return tab


def interpretation_pages(output,result,pdf,paragraph,save):
    fig=plt.figure(figsize=(13.6,8.3))
    fig.text(.07,.95,'What the complete comparison establishes',fontsize=23,weight='bold',va='top')
    paragraph(fig,.07,.875,
        'INDI follows slow commands, but its roll and pitch response falls substantially behind PID as command frequency rises. '
        'This is visible between delivered demand and measured gyro on the same FC clock; a host-to-FC alignment error cannot account for it.',width=136,size=13)
    rows=[]
    for axis in ['roll','pitch']:
        for f in [.5,1.,2.]:
            pair=[]
            for inner in ['pid','indi']:
                h=next((h for h in result['harmonic_response'] if h['test']==axis+'-sine' and h['inner']==inner and h['frequency_hz']==f),None)
                pair.append('pending' if h is None else f"{h['gain']:.2f} / {h['phase_deg']:+.1f}°")
            rows.append([f'{axis.capitalize()} · {f:g} Hz',*pair])
    table(fig,[.07,.46,.86,.30],rows,['Excitation','PID gain / phase','INDI gain / phase'],[.4,.3,.3])
    paragraph(fig,.07,.425,
        'Gain near one means similar oscillation amplitude, not timely tracking. At 2 Hz, INDI produces roughly 90% of the demanded amplitude but about 57–58° of lag. '
        'The short, closed-loop experiments support a response deficit; they do not uniquely identify a motor delay or prove that one gain is the only cause.',width=141,size=12)
    paragraph(fig,.07,.285,
        'There is also real low-frequency motion: the INDI roll-pulse run was already moving before the first pulse (rate-error RMS 0.170 rad/s, versus PID 0.064). '
        'Its position RMS was 0.148 m versus 0.039 m. Other INDI position errors were much closer to PID, and the INDI roll-sine position error was smaller. '
        'A single overall position score would hide the consistent rate-phase deficit.',width=141,size=12)
    paragraph(fig,.07,.11,
        'The test controller was PD hover with additive rate excitation. These twelve flights are inner-loop diagnostics, not a completed MPC + PID versus MPC + INDI trajectory comparison. '
        'Battery voltage, initial motion and outer PD corrections differ; matched repeats are required after a change.',width=145,size=11.5)
    save(fig,pdf,output,'00a_interpretation')

    fig=plt.figure(figsize=(13.6,8.3))
    fig.text(.07,.95,'What the present INDI rate gain means',fontsize=23,weight='bold',va='top')
    paragraph(fig,.07,.875,
        'The deployed CTBR compatibility path converts body-rate error into desired angular acceleration using the following law. '
        'The measured trace recovers these gains directly; stored CLI values 100,100,80 have a scale factor of 0.1.',width=140,size=13)
    fig.text(.10,.725,r'$\vec{\alpha}_c=K_\omega(\vec{\omega}_c-\vec{\omega}_e),\qquad K_\omega=\mathrm{diag}(10,10,8)\;\mathrm{s}^{-1}$',fontsize=22,color='black')
    paragraph(fig,.07,.65,
        r'Here $\omega_c$ is the delivered FC body-rate command, $\omega_e$ is the early measured gyro rate, and $\alpha_c$ is the acceleration requested from the INDI allocator. '
        'The measured rotor speeds and gyro acceleration then close the incremental inversion loop. This rate-error law has no command-derivative feedforward in the current path.',width=139,size=12)
    paragraph(fig,.07,.51,
        'If the acceleration request were achieved exactly, a single axis would have the ideal response below. The 10/s roll and pitch gains alone imply a 100 ms time constant. '
        'This is an explanatory ideal model, not a fit or a prediction of the aircraft after retuning.',width=139,size=12)
    fig.text(.10,.405,r'$\frac{\omega(s)}{\omega_c(s)}=\frac{K}{s+K},\qquad \tau=K^{-1},\qquad \angle G(j2\pi f)=-\tan^{-1}(2\pi f/K)$',fontsize=20,color='black')
    paragraph(fig,.07,.33,
        'The 12 Hz synchronization filter H is inside the acceleration/motor-state inversion. Both channels use the actual matching flight filter. '
        'It is not simply an extra low-pass filter in series with the whole rate response, so its phase cannot be added to the ideal equation as a diagnosed total delay.',width=139,size=12)
    paragraph(fig,.07,.19,
        'The first airborne tuning lever is therefore a modest increase in roll/pitch rate-error gain, with yaw, H, the motor-speed servo, 1 kHz execution and outer controllers held fixed. '
        'Faster gain also amplifies gyro error and model imperfections. Takeoff behavior must be corrected and verified separately before interpreting faster airborne tests.',width=139,size=12)
    fig.text(.07,.055,'Source: deployed paper_adapter.c compatibility acceleration demand; units verified against recorded p25–30 and p46–48.',fontsize=10.5)
    save(fig,pdf,output,'00b_rate_loop')

    attempts=result.get('takeoff_review',{}).get('attempts',[])
    if attempts:
        fig=plt.figure(figsize=(13.6,8.3))
        fig.text(.07,.95,'Takeoff reliability is a separate failure',fontsize=23,weight='bold',va='top')
        paragraph(fig,.07,.875,
            'The roll-sine archive contains eight failed INDI starts followed by one completed flight. Every failure occurred before the deliberate sine input. '
            'The successful flight is recording 09; all earlier recordings are preserved and individually associated with their host logs.',width=139,size=13)
        rows=[]
        for i,a in enumerate(attempts):
            fr=a.get('first_reason',{}) or {}
            reason=fr.get('reason','complete')
            rows.append([f'{i+1:02d}', 'Completed' if a['success'] else reason.replace('soft_flight_envelope','Host envelope').replace('fc_indi_fault_timing','FC timing fault'),
                         f"{a['max_height_before_first_reason_m']:.3f}",f"{a['max_tilt_before_first_reason_deg']:.1f}"])
        table(fig,[.07,.35,.86,.39],rows,['Recording','Outcome','Max height (m)','Max tilt (°)'],[.15,.43,.21,.21],size=11.5,scale=1.65)
        paragraph(fig,.07,.29,
            'Height and tilt are maxima before the first stop/landing reason, relative to the captured ground height. Seven attempts entered a host envelope landing; one reported an FC timing fault. '
            'These are stop reasons, not proof of the physical initiating cause. Counter increments in several failed logs require time ordering against the motion.',width=141,size=12)
        paragraph(fig,.07,.145,
            'The repair target is the ground-to-flight transition: authority, collective thrust, attitude correction and motor response at release from ground contact. '
            'The airborne rate tuning does not, by itself, validate this transition. The comparison keeps the existing position, tilt and timing limits; a successful retry is insufficient evidence of reliable takeoff.',width=141,size=12)
        save(fig,pdf,output,'00c_takeoff')

        fig=plt.figure(figsize=(13.6,8.3))
        fig.text(.07,.95,'Takeoff diagnosis: ground support enters the inversion',fontsize=22,weight='bold',va='top')
        paragraph(fig,.07,.875,
            'Six failed starts reached 10° of tilt before the planned climb, with less than 3 mm of height gain and zero commanded body rates. '
            'In recording 03, the modeled roll torque grew from about −1.34 to −2.26 mN m over the first 0.6 s after INDI acknowledgement while the aircraft remained near 1.5° tilt.',width=139,size=12.5)
        fig.text(.10,.735,r'$J\vec{\alpha}+\vec{\omega}\times J\vec{\omega}=\vec{\tau}_{\rm motor}+\vec{\tau}_{\rm floor}+\vec{\tau}_{\rm other},\qquad \vec{\tau}_c=\vec{\tau}_H+J(\vec{\alpha}_c-\vec{\alpha}_H)$',fontsize=21,color='black')
        paragraph(fig,.07,.67,
            r'$J$ is body inertia; all vectors use FC FRD coordinates. Floor reaction torque can oppose motor torque while angular acceleration stays small. '
            r'The second is the deployed INDI law. Its measured-speed torque baseline $\tau_H$ is carried into the next request. '
            'While the floor prevents rotation, this can retain and accumulate motor imbalance rather than cancel it.',width=139,size=12)
        paragraph(fig,.07,.51,
            'This is a specific mechanism supported by both source and time traces, not a claim that every failed attempt has one cause. '
            'Attempts 01 and 04 failed without any timing-counter increment. In 03, 05, 06, 07 and 08, the first deadline/late increment occurred after the aircraft had already reached 10° tilt. '
            'Reducing timing restrictions cannot repair these earlier tilts.',width=139,size=12)
        paragraph(fig,.07,.335,
            'The PID comparisons also show why simply demanding zero modeled torque is inadequate: successful airborne flights require nonzero modeled motor torque. '
            'In six flights, the pitch torque residual divided by modeled total thrust ranged from −4.30 to −1.98 mm at similar collective. Torque divided by force has units of length. This quantity combines model error and disturbance; it is not a calibrated centre-of-gravity measurement. '
            'A single pooled trim was therefore rejected.',width=139,size=12)
        paragraph(fig,.07,.145,
            'Freezing a ground trim prevents recursive growth, but the same-flight check found ground-to-air torque discrepancies as large as 2.74 mN m in roll and 5.28 mN m in pitch. '
            'That candidate was rejected for deployment. The selected approach shortens supported INDI operation: warm up under PID, then begin the climb immediately after guarded INDI acknowledgement. The next page records the latest flight evidence and launch revision.',width=139,size=12)
        save(fig,pdf,output,'00d_contact_diagnosis')

        fig=plt.figure(figsize=(13.6,8.3))
        fig.text(.07,.95,'Revised launch: brief thrust boost after INDI acknowledgement',fontsize=21,weight='bold',va='top')
        paragraph(fig,.07,.875,
            'Three new attempts at 14:19–14:21 on 10 November did not complete takeoff. Two slid more than 0.6 m while still under PID. '
            'The third reached 7° tilt at full INDI acknowledgement and 16° only 0.10 s later, still below 4 mm height. The extended 0.7-weight ground hold is therefore withdrawn.',width=139,size=12.5)
        table(fig,[.07,.44,.86,.32],
              [['PID ground spool target','0.7 × weight','0.4 × weight'],
               ['After full INDI acknowledgement','Start climb reference','Start climb + boost'],
               ['Temporary thrust floor','None','1.2 × weight'],
               ['Upward collective slew','0.5 N/s','3 N/s during boost'],
               ['Boost end','Not applicable','Liftoff or 0.5 s'],
               ['Downward / ordinary slew','0.5 N/s','0.5 N/s']],
              ['Setting / phase','Previous candidate','Revised candidate'],[.44,.28,.28],size=11.5,scale=1.8)
        paragraph(fig,.07,.385,
            'The boost is a temporary minimum collective request, not a motor step or a rate impulse. It begins only after full, healthy INDI authority is acknowledged. '
            'Thrust rises through a bounded ramp; the existing total-thrust cap and normal downward slew still apply. Ground warmup and the handover receive no boost.',width=139,size=12)
        paragraph(fig,.07,.245,
            'The floor and faster ramp end permanently on existing mocap liftoff confirmation, timeout, landing or fallback. Downward slew may keep thrust elevated beyond that instant. '
            'Public diagnostics confirm liftoff at 2 cm for 40 ms. Rate limits and flight envelopes remain unchanged. PID keeps its original sequence; no FC or ESP flash is required.',width=139,size=12)
        paragraph(fig,.07,.105,
            'Saved rate gains remain 15,15,8 /s; H remains 12 Hz and FC control 1 kHz. These three attempts have host/mocap evidence only; their motor response has not yet been decoded. '
            'The revised boost is checked offline, but improved physical takeoff or tracking is unmeasured. First collect one PD hover and inspect its launch before further excitation tests.',width=139,size=11.5)
        save(fig,pdf,output,'00e_engineering')


def takeoff_evidence_page(output,pdf,save):
    """Compact recorded onset comparison; verify the independent review inputs."""
    root=Path(__file__).resolve().parents[4]
    source=root/'outputs/sanity15_indi_tuning_20261110/takeoff/plot_provenance.json'
    if not source.is_file():return
    records=json.loads(source.read_text())
    fig,axes=plt.subplots(3,2,figsize=(13.6,8.3),sharex=True)
    fig.subplots_adjust(left=.08,right=.97,bottom=.16,top=.82,hspace=.35,wspace=.22)
    torque_extrema=[]
    for col,r in enumerate(records):
        csv,events=Path(r['csv']),Path(r['events'])
        if sha(csv)!=r['csv_sha256'] or sha(events)!=r['events_sha256']:
            raise ValueError('Takeoff evidence source changed')
        t0=r['host_takeoff_monotonic_s'];offset=r['fc_to_host_offset_s']
        rows=[row for line in events.open() if (row:=json.loads(line))['kind']=='hover_control' and row.get('state') and -.35<=row['control_monotonic_s']-t0<=3]
        end=next((row['control_monotonic_s']-t0 for row in rows if row['command'].get('reason')),3.)
        rows=[row for row in rows if row['control_monotonic_s']-t0<=end]
        ht=np.array([row['control_monotonic_s']-t0 for row in rows]);state=np.array([row['state'] for row in rows])
        tilt=np.degrees(np.arccos(np.clip(Rotation.from_quat(state[:,[4,5,6,3]]).as_matrix()[:,2,2],-1,1)))
        a=read_csv(csv);t=a[:,1]*1e-6+offset-t0;mask=(t>=-.35)&(t<=end);p=a[mask,2:];t=t[mask]
        axes[0,col].plot(ht,tilt,color='#216b8c',lw=1.2)
        axes[0,col].set(title='INDI · failed start 03' if col==0 else 'PID · successful start',ylabel='Mocap tilt (°)',ylim=(0,20))
        axes[1,col].plot(t,p[:,28]*1e-6,color='#216b8c',lw=.75,label='Early gyro')
        axes[1,col].plot(t,p[:,31]*1e-6,color='#bc5b30',lw=1.1,label='Actual flight H output')
        axes[1,col].plot(t,p[:,46]*1e-6,color='#333333',lw=1,label='FC delivered demand')
        axes[1,col].set(ylabel='Roll rate (rad/s)',ylim=(-1.5,1.5))
        axes[1,col].legend(fontsize=8,frameon=False,loc='upper left')
        axes[2,col].plot(t,p[:,37]*1e-6,color='#216b8c',lw=1,label='Roll modeled torque')
        axes[2,col].plot(t,p[:,38]*1e-6,color='#bc5b30',lw=1,label='Pitch modeled torque')
        torque_extrema.extend([np.min(p[:,37:39])*1e-6,np.max(p[:,37:39])*1e-6])
        axes[2,col].set(ylabel='Torque from H rotor speeds\n(mN m)',ylim=(-3.5,1.5),xlabel='Time from host takeoff phase (s)')
        axes[2,col].legend(fontsize=8,frameon=False,loc='upper left')
        for ax in axes[:,col]:
            ax.set_xlim(-.35,3);ax.grid(alpha=.18);ax.axvline(2,color='#333333',ls=':',lw=1)
            if end<3:ax.axvline(end,color='#a33131',ls='--',lw=1)
    # Equal scales must still contain every recorded torque sample, including
    # the PID release transient; do not crop the comparison to the INDI range.
    lo,hi=min(torque_extrema),max(torque_extrema)
    margin=.05*(hi-lo)
    for ax in axes[2]:ax.set_ylim(np.floor((lo-margin)*2)/2,np.ceil((hi+margin)*2)/2)
    fig.suptitle('Appendix · takeoff failure develops before climb',x=.08,ha='left',fontsize=21,y=.97)
    fig.text(.08,.905,'Dotted: planned climb at 2 s. Red dashed: first abort. Equal axes; raw samples and recorded H only.',fontsize=11.5)
    fig.text(.08,.075,'Modeled torque uses filtered measured rotor speeds; it is not a direct torque sensor or a PID actuator target.',fontsize=10.5)
    fig.text(.08,.04,'Host/FC origins use status correspondence (tens of ms uncertainty). No fitted time shift, interpolation or extra filtering.',fontsize=10.5)
    save(fig,pdf,output,'98_takeoff_onset')
