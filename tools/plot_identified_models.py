#!/usr/bin/env python3
"""Export the three curated figures from local campaign archives; no hardware I/O.

The larger campaign archives are intentionally not shipped in this repository.
Usage: python tools/plot_identified_models.py --fullrange /path/to/sanity_fullrange_id_20261020
The matching sanity_regulated_id_20261020 archive must be beside it, or pass --regulated.
"""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from scipy.spatial import Delaunay

ROOT=Path(__file__).resolve().parents[1]
read=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
COL=['#28699c','#dc8d21','#25917c','#9a629f'];INK='#182b3b';VS=[7.6,7.8,8.,8.2]

def main():
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--fullrange',type=Path,required=True);parser.add_argument('--regulated',type=Path);parser.add_argument('--preview-dir',type=Path);args=parser.parse_args();R=args.fullrange;L=args.regulated or R.parent/'sanity_regulated_id_20261020'
 for style in ['Regular','Heavy']:
  font=Path.home()/('.local/share/fonts/Avenir '+style+'.ttf')
  if font.exists():font_manager.fontManager.addfont(str(font))
 plt.rcParams.update({'font.family':['Avenir','DejaVu Sans'],'font.size':10,'axes.labelsize':10,'axes.titlesize':12,'legend.fontsize':8,'xtick.labelsize':9,'ytick.labelsize':9,'axes.spines.top':False,'axes.spines.right':False,'axes.grid':True,'grid.alpha':.18,'svg.fonttype':'path','svg.hashsalt':'sanity-curated-20261020','mathtext.fontset':'dejavusans'})
 selected_path=ROOT/'parameters/identified_actuators.json';models=read(selected_path)['models'];out=ROOT/'media/identification';out.mkdir(parents=True,exist_ok=True);assets=[]
 def save(fig,name):
  path=out/name;fig.savefig(path,metadata={'Date':None})
  path.write_text('\n'.join(line.rstrip() for line in path.read_text().splitlines())+'\n')
  if args.preview_dir:
   args.preview_dir.mkdir(parents=True,exist_ok=True);fig.savefig(args.preview_dir/(Path(name).stem+'.png'),dpi=160)
  plt.close(fig);assets.append(dict(path=str(path.relative_to(ROOT)),sha256=sha(path)))
 surface=read(R/'thrust_command_voltage_surface.json');points=surface['points'];u=np.array([p['command'] for p in points]);v=np.array([p['FC_voltage_V'] for p in points]);T=np.array([p['thrust_N'] for p in points]);m=models['direct_motor_thrust'];n=m['normalization']
 def thrust(command,V):
  q=(command-n['command_center'])/n['command_scale'];z=(V-n['voltage_center_V'])/n['voltage_scale_V'];return sum(c*q**i*z**j for c,(i,j) in zip(m['coefficients_N'],m['coefficient_powers_q_z']))
 fig=plt.figure(figsize=(11.5,4.8));ax=fig.add_subplot(121,projection='3d');x,y=np.meshgrid(np.linspace(min(u),max(u),45),np.linspace(min(v),max(v),35));mask=Delaunay(m['domain']['convex_hull_command_voltage_V']).find_simplex(np.c_[x.ravel(),y.ravel()]).reshape(x.shape)>=0;z=thrust(x,y);z[~mask]=np.nan
 ax.plot_surface(x,y,z,cmap='viridis',alpha=.68,linewidth=0,antialiased=True,rcount=35,ccount=45,rasterized=False);ax.scatter(u,v,T,s=7,color=INK,alpha=.35,depthshade=False,edgecolors='none');ax.set(title='Direct motor command + FC voltage',xlabel='MSP motor count',ylabel='Delivered FC voltage [V]',zlabel='Rotor thrust [N]');ax.view_init(24,-127);ax.set_box_aspect((1.5,1,.9));ax.xaxis.labelpad=7;ax.yaxis.labelpad=8
 ax=fig.add_subplot(122);speed_points=read(R/'thrust_omega_voltage_surface.json')['points'];k=models['rotor_thrust_from_speed']['kT_N_per_rad_s_squared']
 for motor in range(1,5):
  a=[p for p in speed_points if p['motor']==motor];ax.scatter([p['omega_rad_s']/1000 for p in a],[p['thrust_N'] for p in a],color=COL[motor-1],s=12,alpha=.25,label=f'M{motor}')
 x=np.linspace(.079,5.884,250);ax.plot(x,k*(x*1000)**2,color=INK,lw=2,label='Shared speed law');ax.set(title='Separate measured-speed relationship',xlabel=r'Angular speed $\omega$ [10³ rad/s]',ylabel='Rotor thrust [N]');ax.legend();fig.subplots_adjust(left=.035,right=.985,bottom=.18,top=.88,wspace=.28);save(fig,'direct-thrust.svg')
 all_points=read(R/'results.json')['plateaus']+read(L/'results.json')['plateaus'];m=models['ctbr_acro100_collective'];c=np.array(m['coefficients_N']);n=m['normalization']
 def collective(rc,V):
  q=(rc-n['command_center'])/n['command_scale'];z=(V-n['voltage_center_V'])/n['voltage_scale_V'];return sum(c[i]*q**i for i in range(4))+z*sum(c[4+j]*q**j for j in range(3))
 fig,axes=plt.subplots(1,2,figsize=(11.5,4.4));ax=axes[0]
 for j,V in enumerate(VS):
  a=[p for p in all_points if p['role']=='acro_collective' and p['compensation']==100 and p['steady_motor_speed'] and p['source_setpoint_V']==V];ax.scatter([p['input_counts'][0] for p in a],[p['thrust_N'] for p in a],s=15,alpha=.30,color=COL[j]);d=next(d for d in m['domain_by_source_V'] if d['source_V']==V);x=np.linspace(d['x_min'],d['x_max'],250);ax.plot(x,collective(x,V),lw=1.9,color=COL[j],label=f'{V:.1f} V source')
 ax.axhline(.06947*9.80665,color=INK,ls=':',lw=1,label='Hover load');ax.set(title='Acro collective · compensation 100',xlabel='RC throttle [count]',ylabel='Aircraft-total thrust [N]');ax.legend()
 z=np.load(R/'audited_step_traces.npz',allow_pickle=False);meta=json.loads(str(z['metadata_json']));ax=axes[1]
 for direction,color in [('up',COL[0]),('down',COL[1])]:
  a=[e for e in meta if e['cohort']=='front' and e['mode']=='acro' and e['compensation']==100 and e['source_V']==8 and e['signal']=='thrust' and e['direction']==direction and set([e['from_count'],e['to_count']])=={1200,1300}];fit=next(f for f in models['observed_step_responses']['fits'] if f['mode']=='acro' and f['signal']=='thrust' and f['direction']==direction)
  for e in a:ax.plot(z[e['key']+'_t']*1000,z[e['key']+'_normalized'],color=color,alpha=.23,lw=.9)
  t=np.linspace(-.05,.35,300);h=-np.expm1(-np.maximum(t-fit['delay_s'],0)/fit['time_constant_s']);ax.plot(t*1000,h,color=color,lw=2,label=direction.capitalize()+' model')
 ax.set(title='Observed force-step shape · 8.0 V source',xlabel='Time after host ACK [ms]',ylabel='Normalised transition progress',xlim=(-50,350),ylim=(-.15,1.15));ax.legend();fig.subplots_adjust(left=.075,right=.985,bottom=.17,top=.88,wspace=.28);save(fig,'ctbr-collective.svg')
 yaw=read(L/'provisional_yaw_model.json');fig,ax=plt.subplots(figsize=(7.2,4.1))
 for j,V in enumerate(VS):
  a=[p for p in yaw['contrasts'] if p['source_V']==V];ax.scatter([p['signed_squared_speed_difference']/1e6 for p in a],[1000*p['torque_difference_Nm'] for p in a],color=COL[j],alpha=.45,s=28,label=f'{V:.1f} V source')
 x=np.array([min(p['signed_squared_speed_difference'] for p in yaw['contrasts']),max(p['signed_squared_speed_difference'] for p in yaw['contrasts'])]);ax.plot(x/1e6,1000*models['yaw_reaction_torque']['kQ_Nm_per_rad_s_squared']*x,color=INK,lw=2,label='Shared balanced-contrast fit');ax.set(title='Provisional yaw-reaction coefficient',xlabel=r'Signed squared-speed contrast [10⁶ (rad/s)²]',ylabel='Measured torque contrast [mN m]');ax.legend(fontsize=8);fig.subplots_adjust(left=.12,right=.98,bottom=.18,top=.88);save(fig,'yaw-torque.svg')
 sources=[R/'thrust_command_voltage_surface.json',R/'thrust_omega_voltage_surface.json',R/'results.json',L/'results.json',R/'audited_step_traces.npz',L/'provisional_yaw_model.json'];manifest=dict(scope='Three curated vector figures; raw/full analysis archives remain external. No device/network access.',models_sha256=sha(selected_path),plotting_script_sha256=sha(Path(__file__)),sources=[dict(archive=p.parent.name+'/'+p.name,sha256=sha(p)) for p in sources],assets=assets);(out/'provenance.json').write_text(json.dumps(manifest,indent=2)+'\n');print('Exported',len(assets),'vector SVGs;',sum((ROOT/a['path']).stat().st_size for a in assets),'bytes')
if __name__=='__main__':main()
