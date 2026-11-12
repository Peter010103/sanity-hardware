"""Recorded-signal plots and incremental PDF; no analysis-only signal filters."""
import textwrap
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from sanity_control_analysis import TESTS

BLUE='#216b8c';ORANGE='#bc5b30';GRAY='#333333';GOLD='#9b7b17'
plt.rcParams.update({'font.family':'Avenir','font.size':11.5,'axes.titlesize':13,
    'axes.labelsize':11.5,'legend.fontsize':9,'pdf.fonttype':3,'mathtext.fontset':'stix',
    'axes.spines.top':False,'axes.spines.right':False})

def paragraph(fig,x,y,text,width=142,size=12):
    fig.text(x,y,'\n'.join(textwrap.wrap(text,width)),va='top',fontsize=size,linespacing=1.45)

def pending(ax,label='Awaiting matching archived flight'):
    ax.text(.5,.5,label,transform=ax.transAxes,ha='center',va='center',fontsize=12,color='#666666')
    ax.set_axis_off()

def get_data(result,datasets,test,mode):
    name=result['selected'].get(test+'/'+mode)
    return datasets.get(name) if name else None

def limit_for(datasets,axis,command=False):
    maximum=.1
    for d in datasets:
        if d is None:continue
        if command:maximum=max(maximum,float(np.max(abs(d['host_command'][:,axis]))),float(np.max(abs(d['injection'][:,axis]))))
        else:
            use=(d['t']>=0)&(d['t']<=d['duration'])
            maximum=max(maximum,float(np.max(abs(d['p'][use][:,[28+axis,31+axis,46+axis]])*1e-6)))
    return np.ceil(maximum*1.1*20)/20

def rate_lines(ax,d,axis,mask,time):
    p=d['p'][mask]
    ax.plot(time,p[:,28+axis]*1e-6,color=BLUE,lw=.7,label='Measured early gyro')
    ax.plot(time,p[:,31+axis]*1e-6,color=ORANGE,lw=1.1,label='Actual flight H output')
    ax.plot(time,p[:,46+axis]*1e-6,color=GRAY,lw=1.1,label='FC delivered demand')
    ax.grid(True,alpha=.18)

def pulse_shading(ax):
    for t in [3,5.3,7.6,9.9]:ax.axvspan(t,t+.3,color=GOLD,alpha=.1,lw=0)

def save(fig,pdf,output,name):
    fig.savefig(output/(name+'.png'),dpi=160)
    fig.savefig(output/(name+'.pdf'))
    pdf.savefig(fig);plt.close(fig)

def metric(result,datasets,test,mode,axis):
    d=get_data(result,datasets,test,mode)
    return 'pending' if d is None else f"{d['metrics']['rate_error_RMS_rad_s'][axis]:.3f}"

def make_report(output,result,datasets):
    baseline=result.get('selection_policy')=='pinned pre-tuning baseline'
    with PdfPages(output/'control_test_comparison.pdf') as pdf:
        fig=plt.figure(figsize=(13.6,8.3))
        fig.text(.07,.95,'Sanity15 · control-test comparison',fontsize=24,weight='bold',va='top')
        state=('Completed baseline comparison' if baseline else 'Completed selected comparison') if result['collection_complete'] else 'Interim analysis; paired collection in progress'
        fig.text(.07,.885,f"{state} · {result['analysed_successful_cases']}/12 cases",fontsize=14)
        introduction=('Incremental Nonlinear Dynamic Inversion (INDI) is slower than PID in the paired rate tests. Main findings and the latest launch revision come first; recorded-signal plots and methods follow in the appendix. The comparison plots retain the original flight settings.' if baseline else
            'Recorded comparison of Incremental Nonlinear Dynamic Inversion (INDI) and PID. This output selects the latest successful archived run per case; check each run configuration before interpreting a mixture of revisions. Baseline-specific conclusions are not copied into this report.')
        paragraph(fig,.07,.83,introduction,width=144,size=13)
        rows=[]
        for axis,name in enumerate(['roll','pitch','yaw']):
            rows.append([f'PD hover · {name}',metric(result,datasets,'pd-hover','pid',axis),metric(result,datasets,'pd-hover','indi',axis)])
        for test,axis in [('roll-pulse',0),('pitch-pulse',1),('yaw-pulse',2),('roll-sine',0),('pitch-sine',1)]:
            rows.append([test,metric(result,datasets,test,'pid',axis),metric(result,datasets,test,'indi',axis)])
        ax=fig.add_axes([.07,.37,.86,.35]);ax.axis('off')
        table=ax.table(cellText=rows,colLabels=['Same-tap demand − early gyro RMS (rad/s)','PID','INDI'],
                       colWidths=[.68,.16,.16],cellLoc='left',loc='center')
        table.auto_set_font_size(False);table.set_fontsize(11.5);table.scale(1,1.6)
        for (r,c),cell in table.get_celld().items():
            cell.set_edgecolor('#d4d9dc')
            if r==0:cell.set_facecolor('#eaf0f3');cell.set_text_props(weight='bold')
        paragraph(fig,.07,.31,'Read this as a baseline: the larger INDI rate errors and phase lag motivate a controlled gain change; the repeated failed takeoffs require a separate transition repair.' if baseline else
            'These metrics describe the selected recordings. Inspect demand amplitude, initial motion, controller configuration and timing before attributing a difference to the inner controller.',size=12)
        analysed=[datasets[n]['metrics'] for n in result['selected'].values()]
        changed=sum(any(v!=0 for v in m['recording_counter_deltas'].values()) for m in analysed)
        paragraph(fig,.07,.23,f"Execution check: {changed}/{len(analysed)} selected recordings show a new deadline, late-interval or log-drop counter increment. The cache and results retain recording-match evidence, authority, rotor validity, clipping, voltage and timing statistics for each run.",size=12)
        if any(datasets[name]['metrics'].get('collection_notes') for name in result['selected'].values()):
            paragraph(fig,.07,.145,'Collection note: repeated INDI takeoff failures are retained separately. Successful-airborne metrics do not explain those failures; the takeoff review has priority before further INDI flights.',size=11)
        paragraph(fig,.07,.065,'RMS compares common measured motion, not the literal PID internal error. H is recorded in flight; no extra smoothing or response-based time shift is used.',width=153,size=10.5)
        save(fig,pdf,output,'00_overview')

        if result['collection_complete'] and baseline:
            from sanity_control_report_findings import interpretation_pages
            interpretation_pages(output,result,pdf,paragraph,save)

        fig,axes=plt.subplots(2,3,figsize=(14,8.3),sharex=True)
        fig.subplots_adjust(left=.075,right=.98,bottom=.17,top=.83,hspace=.37,wspace=.26)
        pair=[get_data(result,datasets,'pd-hover',mode) for mode in ['pid','indi']]
        for row,(mode,d) in enumerate(zip(['PID','INDI'],pair)):
            for axis,name in enumerate(['Roll','Pitch','Yaw']):
                ax=axes[row,axis]
                if d is None:pending(ax);continue
                mask=(d['t']>=0)&(d['t']<=d['duration']);rate_lines(ax,d,axis,mask,d['t'][mask])
                limit=limit_for(pair,axis);ax.set(title=f'{mode} · {name}',ylabel='Rate, native FRD (rad/s)',ylim=(-limit,limit),xlim=(0,d['duration']))
                if row==1:ax.set_xlabel('Diagnostic elapsed (s)')
        if pair[0] is not None:axes[0,0].legend(frameon=False,fontsize=8,loc='upper right')
        fig.suptitle('Appendix A · PD hover baseline',x=.075,ha='left',fontsize=21,y=.97)
        fig.text(.075,.905,'These flights contain no deliberate excitation. Baseline motion is distinct from stationary sensor-noise characterization.',fontsize=11.5)
        fig.text(.075,.085,'Rates use the same FC clock within each flight; time origins between flights come from approximate host/FC status correspondence.',fontsize=10.5)
        fig.text(.075,.05,'Control used desktop mocap. The onboard estimator was logged separately and is not assessed as a control input by these plots.',fontsize=10.5)
        save(fig,pdf,output,'01_pd-hover')

        for test,axis in [('roll-pulse',0),('pitch-pulse',1),('yaw-pulse',2)]:
            pair=[get_data(result,datasets,test,mode) for mode in ['pid','indi']]
            fig,axes=plt.subplots(2,2,figsize=(13.6,8.3))
            fig.subplots_adjust(left=.075,right=.97,bottom=.20,top=.83,wspace=.24,hspace=.39)
            for row,(mode,d) in enumerate(zip(['PID','INDI'],pair)):
                if d is None:
                    for ax in axes[row]:pending(ax)
                    continue
                ax=axes[row,0];mask=(d['t']>=0)&(d['t']<=d['duration'])
                rate_lines(ax,d,axis,mask,d['t'][mask]);limit=limit_for(pair,axis)
                ax.set(title=f'{mode} · delivered demand and response',ylabel='Rate, native FRD (rad/s)',ylim=(-limit,limit))
                ax=axes[row,1]
                ax.plot(d['host_t'],d['base'][:,axis],color=BLUE,lw=1,label='PD base rate')
                ax.plot(d['host_t'],d['injection'][:,axis],color=GOLD,lw=1.3,label='Requested injection')
                ax.plot(d['host_t'],d['host_command'][:,axis],color=GRAY,lw=1,label='Selected total rate')
                limit=limit_for(pair,axis,command=True)
                ax.set(title=f'{mode} · host components converted to FRD',ylabel='Rate, native FRD (rad/s)',ylim=(-limit,limit))
                for ax in axes[row]:pulse_shading(ax);ax.grid(True,alpha=.18);ax.set_xlim(0,13.2)
            if pair[0] is not None:
                for ax in axes[0]:ax.legend(frameon=False,fontsize=8.5)
            for ax in axes[1]:ax.set_xlabel('Diagnostic elapsed (s)')
            fig.suptitle('Appendix · '+test.replace('-',' ').capitalize()+' · recorded signals',x=.075,ha='left',fontsize=21,y=.97)
            fig.text(.075,.905,'Shading marks requested pulse timing. Host pitch/yaw signs are flipped into FRD; both FC series already share FRD.',fontsize=11)
            stats=[]
            for mode,d in zip(['PID','INDI'],pair):
                if d is not None:
                    m=d['metrics'];stats.append(f"{mode}: rate error RMS {m['rate_error_RMS_rad_s'][axis]:.3f} rad/s; position RMS {m['position_RMS_m']:.3f} m; clipped ticks {m['host_clipped_ticks']}")
            fig.text(.075,.115,'\n'.join(stats),fontsize=10.5,linespacing=1.5,va='top')
            fig.text(.075,.045,'PD hover remains closed around these injections. Pulse boundaries use approximate status alignment; response traces are never delay-corrected.',fontsize=10.3)
            save(fig,pdf,output,test)

        for test,axis in [('roll-sine',0),('pitch-sine',1)]:
            pair=[get_data(result,datasets,test,mode) for mode in ['pid','indi']]
            frequencies=sorted({h['frequency_hz'] for d in pair if d is not None for h in d['metrics']['harmonics']}) or [.5,1.,2.]
            for page_index in range(0,len(frequencies),3):
                subset=frequencies[page_index:page_index+3]
                fig,axes=plt.subplots(2,3,figsize=(14,8.3),sharex=True)
                fig.subplots_adjust(left=.075,right=.98,bottom=.21,top=.80,hspace=.44,wspace=.25)
                for row,(mode,d) in enumerate(zip(['PID','INDI'],pair)):
                    for col in range(3):
                        ax=axes[row,col]
                        if col>=len(subset):ax.set_axis_off();continue
                        frequency=subset[col]
                        h=next((h for h in d['metrics']['harmonics'] if h['frequency_hz']==frequency),None) if d is not None else None
                        if h is None:pending(ax,'Frequency absent from this recording');continue
                        if h.get('complete') is False:pending(ax,'Excitation incomplete; no estimate');continue
                        start=h['FC_phase_window_s'][0]-1
                        relative=d['t']-start;mask=(relative>=0)&(relative<=6)
                        rate_lines(ax,d,axis,mask,relative[mask])
                        fit='weak demand' if h['gain'] is None else f"gain {h['gain']:.2f}; phase {h['phase_deg']:+.1f}°"
                        amplitude=d['metrics'].get('sine_recipe',{}).get('excitation',{}).get('amplitude_rad_s',.15)
                        limit=limit_for(pair,axis)
                        ax.set(title=f'{mode} · {frequency:g} Hz · injection {amplitude:g} rad/s\n{fit}',ylabel='Rate, native FRD (rad/s)',ylim=(-limit,limit),xlim=(0,6))
                        ax.axvspan(1,5,color=GOLD,alpha=.09,lw=0)
                        if row==1:ax.set_xlabel('Time within sine segment (s)')
                if pair[0] is not None:axes[0,0].legend(frameon=False,fontsize=8,loc='lower right')
                fig.suptitle('Appendix · '+test.replace('-',' ').capitalize()+' · frequency segments',x=.075,ha='left',fontsize=21,y=.97)
                fig.text(.075,.905,'Raw recorded curves. Gain/phase use the central 4 s shaded interval. Frequencies come from each recorded recipe.',fontsize=11)
                fig.text(.075,.11,'Offset, trend and sine/cosine are estimated on actual FC timestamps. No fitted curve replaces the measured signal.',fontsize=10.5)
                fig.text(.075,.07,'Gain is gyro harmonic / FC-demand harmonic; positive phase is a lead. Injection sizes are shown; compare matched recipes.',fontsize=10.3)
                fig.text(.075,.035,'Closed-loop descriptive ratios from a short record; compare residual motion, battery state and delivered demand.',fontsize=10.3)
                save(fig,pdf,output,test+('' if page_index==0 else f'_page{page_index//3+1}'))

        if result.get('selection_policy')=='pinned pre-tuning baseline':
            from sanity_control_report_findings import takeoff_evidence_page
            takeoff_evidence_page(output,pdf,save)
        fig=plt.figure(figsize=(13.6,8.3))
        fig.text(.07,.95,'Appendix · methods and interpretation limits',fontsize=23,weight='bold',va='top')
        sections=[(.86,'Recording association','Each host flight is matched to a decoded FC recording using status timestamp, controller sequence and authority mode together. Timestamp overlap alone is insufficient after power cycles. Candidate scores and the selected CSV are retained in results.json; ambiguous matches stop analysis for that run.'),
                  (.71,'Signal definitions and filters','Demand is FC p46–48 / 1e6 after RC processing. Measured early gyro is p28–30 / 1e6, after downsampling/RPM filtering and before ordinary BF gyro filters. H is p31–33 / 1e6: the actual synchronization-filter output (session setting 12 Hz), not PID internal feedback. All use native FRD; host FLU converts by diag(1, −1, −1). No analysis-only filter, smoothing, interpolation or fitted delay correction is applied.'),
                  (.51,'Frequency estimates','A sinusoid, cosine, offset and linear trend are fitted by least squares to raw FC samples at the known excitation frequency, over the central four seconds of each six-second segment. Complex harmonic ratios give descriptive gain and phase. Fitted curves are not plotted, and residual motion is reported rather than hidden.'),
                  (.27,'Authority, actuation and sampling','INDI rotor targets p73–76 are compared with measured speeds p65–68 only when INDI owns the motors. Under PID these targets and allocator/servo diagnostics are unused shadow computations. Final outputs p105–108 are actual motor commands in both modes. p112–114 are cumulative all-tick timing/drop counters. The trace samples every fourth control tick; p124 refers to the previous control tick, not the previous stored row.'),
                  (.14,'Scope','One or few flights per case, different PD corrections and changing battery conditions limit direct comparisons. Archive and decoder hashes, run selection, voltage, clipping and timing are retained. '+('The main text gives the completed baseline interpretation; no plot in this report represents a post-tuning flight.' if baseline else 'Run selection and per-flight configurations determine which controller revision each plot represents.'))]
        for y,title,body in sections:
            fig.text(.07,y,title,fontsize=13,weight='bold',va='top');paragraph(fig,.07,y-.035,body,width=147,size=11)
        fig.text(.10,.375,r'$y(t)=c_0+c_1(t-\bar{t})+a\sin(2\pi ft)+b\cos(2\pi ft),\quad C=b-ja,\quad G=C_{\mathrm{gyro}}/C_{\mathrm{demand}}$',fontsize=17,color='black')
        save(fig,pdf,output,'99_methods')
