"""Offline UART-to-gyro sine response assessment. No ROS, Zenoh or aircraft I/O."""
import argparse
import csv
import json
import math
from pathlib import Path
import struct
import numpy as np


def sinusoid_fit(time,values,frequency):
    matrix=np.column_stack((np.ones(len(time)),np.sin(2*np.pi*frequency*time),np.cos(2*np.pi*frequency*time)))
    coef=np.linalg.lstsq(matrix,values,rcond=None)[0];fit=matrix@coef
    variance=float(np.sum((values-np.mean(values))**2));residual=float(np.sum((values-fit)**2))
    return dict(amplitude=float(np.hypot(coef[1],coef[2])),phase=float(np.arctan2(coef[2],coef[1])),
                bias=float(coef[0]),r_squared=1.-residual/variance if variance>1e-12 else 0.)


def assess_sine_run(folder,make_plots=True):
    folder=Path(folder);wire={};outputs=[];gyro=[];begins={};ends={};cancelled={}
    with (folder/'events.jsonl').open() as stream:
        for line in stream:
            event=json.loads(line);kind=event['kind']
            if kind=='probe_begin':begins[event['trial_index']]=event
            elif kind=='probe_end':ends[event['trial_index']]=event
            elif kind=='probe_cancelled':cancelled[event['trial_index']]=event
            elif kind=='zenoh_tx_command':
                raw=bytes.fromhex(event['payload_hex'])
                if raw[:4] in (b'SZC2',b'SZC3'):
                    wire[struct.unpack_from('>QII',raw,8)]=event['host_ns']
            for record in event.get('telemetry',{}).get('records',[]):
                if 'transmitted_rc_channels' in record:outputs.append(record)
                if 'gyro_rad_s_fc' in record:gyro.append(record)
    outputs.sort(key=lambda r:r['uart_write_begin_esp_us']);gyro.sort(key=lambda r:r['response_parsed_esp_us'])
    rows=[];traces=[]
    for index,begin in sorted(begins.items()):
        row=dict(trial_index=index,axis=begin['axis'],repeat=begin['repeat'],frequency_hz=begin['frequency_hz'],
            requested_injection_amplitude_rad_s=begin['amplitude_rad_s'],completed=index in ends,valid=False,reasons=[])
        rows.append(row)
        if index not in ends:
            row['reasons'].append('Cancelled: '+cancelled[index].get('reason','unknown') if index in cancelled else 'No completed block');continue
        end=ends[index];f=begin['frequency_hz'];axis=begin['rate_axis']-1
        linked=[]
        for record in outputs:
            host=wire.get((record['epoch'],record['command_session'],record['command_sequence']))
            if host is not None and begin['host_ns']<=host<end['host_ns']:linked.append((record,host))
        if len(linked)<10:row['reasons'].append('Insufficient matched UART commands');continue
        epochs={r['epoch'] for r,_ in linked}
        if len(epochs)!=1:row['reasons'].append('Receiver epoch changed during block');continue
        epoch=next(iter(epochs));t0=linked[0][0]['uart_write_begin_esp_us']/1e6
        commands=np.array([(r['uart_write_begin_esp_us']/1e6,(r['transmitted_rc_channels'][axis]-1500)*math.radians(600)/500) for r,_ in linked])
        samples=np.array([(r['response_parsed_esp_us']/1e6,r['gyro_rad_s_fc'][axis]) for r in gyro if r['epoch']==epoch and commands[0,0]<=r['response_parsed_esp_us']/1e6<=commands[-1,0]])
        steady=[r for r,h in linked if begin['host_ns']+1e9/f<=h<begin['host_ns']+4e9/f]
        trace=dict(trial_index=index,command_time=(commands[:,0]-t0).tolist(),command=commands[:,1].tolist(),
                   gyro_time=(samples[:,0]-t0).tolist() if len(samples) else [],gyro=samples[:,1].tolist() if len(samples) else [],
                   ramp_s=1/f,duration_s=begin['duration_s'])
        traces.append(trace)
        if len(steady)<10 or len(samples)<10:row['reasons'].append('Insufficient steady-cycle telemetry');continue
        lo=steady[0]['uart_write_begin_esp_us']/1e6;hi=steady[-1]['uart_write_begin_esp_us']/1e6
        selected=samples[(samples[:,0]>=lo)&(samples[:,0]<=hi)]
        if len(selected)<20 or selected[-1,0]-selected[0,0]<2.5/f:row['reasons'].append('Fewer than2.5 usable steady cycles');continue
        ages=[(r['uart_write_begin_esp_us']-r['command_received_esp_us'])/1000 for r in steady]
        command_gap=1000*max(np.diff([r['uart_write_begin_esp_us']/1e6 for r in steady]))
        gyro_gap=1000*max(np.diff(selected[:,0]))
        row.update(maximum_command_age_ms=max(ages),maximum_uart_gap_ms=float(command_gap),maximum_gyro_gap_ms=float(gyro_gap),steady_samples=len(selected))
        if max(ages)>40.:row['reasons'].append('Command age exceeded40ms')
        if command_gap>40.:row['reasons'].append('UART command gap exceeded40ms')
        if gyro_gap>35.:row['reasons'].append('Gyro gap exceeded35ms')
        lookup=np.clip(np.searchsorted(commands[:,0],selected[:,0],side='right')-1,0,len(commands)-1)
        command=commands[lookup,1];time=selected[:,0]-t0
        incoming=sinusoid_fit(time,command,f);outgoing=sinusoid_fit(time,selected[:,1],f)
        if incoming['amplitude']<.03:row['reasons'].append('Actual command fundamental below0.03rad/s')
        if incoming['r_squared']<.7:row['reasons'].append('Command poorly described by single frequency')
        if outgoing['r_squared']<.7:row['reasons'].append('Gyro poorly described by single frequency')
        phase=(outgoing['phase']-incoming['phase']+np.pi)%(2*np.pi)-np.pi
        row.update(command_amplitude_rad_s=incoming['amplitude'],measured_amplitude_rad_s=outgoing['amplitude'],
            amplitude_ratio=outgoing['amplitude']/incoming['amplitude'] if incoming['amplitude']>1e-9 else None,
            phase_deg=float(np.degrees(phase)),phase_equivalent_lag_ms=float(-phase/(2*np.pi*f)*1000),
            command_bias_rad_s=incoming['bias'],measured_bias_rad_s=outgoing['bias'],
            command_r_squared=incoming['r_squared'],measured_r_squared=outgoing['r_squared'])
        row['valid']=not row['reasons']
    report=dict(blocks=rows,scope='Offline fundamental response over three middle cycles: total actual UART-derived rate command versus FC gyro. Live PD remains active. No additional response filtering. Phase-equivalent lag is not an identified pure delay or communication delay. 600deg/s linear mapping; response timestamps and50Hz gyro limit precision. A usable block is not a flight-safety certificate.')
    (folder/'sine_summary.json').write_text(json.dumps(report,indent=2)+'\n')
    keys=list(dict.fromkeys(key for row in rows for key in row))
    if rows:
        with (folder/'sine_summary.csv').open('w') as stream:
            writer=csv.DictWriter(stream,fieldnames=keys);writer.writeheader();writer.writerows(rows)
    if make_plots and traces:plot_responses(folder,rows,traces)
    return report


def plot_responses(folder,rows,traces):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family':'Avenir','font.size':10,'lines.linewidth':.8,'grid.linestyle':':',
        'axes.spines.top':False,'axes.spines.right':False,'legend.handlelength':1.2})
    columns=min(3,len(traces));nrows=math.ceil(len(traces)/columns)
    fig,axes=plt.subplots(nrows,columns,figsize=(5*columns,2.8*nrows),squeeze=False,layout='constrained')
    for ax,trace in zip(axes.flat,traces):
        row=next(r for r in rows if r['trial_index']==trace['trial_index']);duration=trace['duration_s'];ramp=trace['ramp_s']
        ax.axvspan(ramp,duration-ramp,color='#edf3f9',zorder=0)
        ax.step(trace['command_time'],trace['command'],where='post',ls='--',label='UART rate command')
        ax.plot(trace['gyro_time'],trace['gyro'],label='FC gyro')
        detail=f"gain={row['amplitude_ratio']:.2f}, phase={row['phase_deg']:.1f}°" if row.get('amplitude_ratio') is not None else 'insufficient fit data'
        ax.set_title(f"{row['axis'].capitalize()} · {row['frequency_hz']:g} Hz · repeat {row['repeat']}\n{detail}"+(' · quality failed' if not row['valid'] else ''),pad=8)
        ax.set_xlabel('Time in block [s]');ax.set_ylabel('Angular rate [rad/s]');ax.grid(True);ax.legend(frameon=False,fontsize=8,loc='best')
    for ax in list(axes.flat)[len(traces):]:ax.set_visible(False)
    fig.savefig(folder/'sine_response.pdf');fig.savefig(folder/'sine_response.png',dpi=180);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('recording',type=Path);args=parser.parse_args()
    report=assess_sine_run(args.recording);print(json.dumps(report,indent=2))

if __name__=='__main__':main()
