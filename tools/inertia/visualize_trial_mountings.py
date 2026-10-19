#!/usr/bin/env python3
"""Show the operator-confirmed yaw/pitch/roll mounting sequence."""
import argparse
import json
from pathlib import Path

import numpy as np
import visualize_fixture as fixture

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
DATA=ROOT/'data/processed/inertia/20261019'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--save-only',action='store_true')
    parser.add_argument('--output',type=Path,default=ROOT/'build/inertia/preview')
    args=parser.parse_args()
    import matplotlib
    if args.save_only:matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    data=json.loads((DATA/'measurement_inputs.json').read_text())
    R_AF=np.array(data['assembly_A']['rotation_A_from_F'])
    t_AF=np.array(data['assembly_A']['translation_A_from_F_m'])*1000
    R_AB=np.array(data['drone']['rotation_A_from_B'])
    t_FB=np.array(data['drone']['nominal_visualization_translation_F_m'])*1000
    profiles,hole,radius=fixture.read_outlines(ROOT/'data/raw/bifilar/20261019/u_mount_rev4.dxf')
    faces,colors=fixture.solid_faces(profiles,hole,radius)
    faces_A=[face@R_AF.T+t_AF for face in faces]
    parts=fixture.load_drone_visuals(ROOT/data['drone']['visual_model'])
    centers_F=np.array([v['center_F_m'] for v in data['markers'].values()])*1000
    centers_A=centers_F@R_AF.T+t_AF
    u,v=np.meshgrid(np.linspace(0,2*np.pi,17),np.linspace(0,np.pi,11))
    sphere=np.array([6.35*np.cos(u)*np.sin(v),6.35*np.sin(u)*np.sin(v),6.35*np.cos(v)])
    fig=plt.figure(figsize=(18,7))
    titles=['1. Yaw: drone Z axis vertical','2. Pitch: drone Y axis vertical','3. Roll: drone X axis vertical']
    captions=['Initial mounting','Then +90° about assembly +X','Then −90° about assembly’s own +Z']
    palette=plt.get_cmap('tab10')
    for i,trial in enumerate(data['trials']):
        ax=fig.add_subplot(1,3,i+1,projection='3d')
        R_CA=np.array(trial['nominal_rotation_C_from_A'])
        R_CB=R_CA@R_AB
        vertical_B=np.array(trial['nominal_measurement_axis_direction_B'])
        if not np.allclose(R_CB@vertical_B,[0,0,1]):raise ValueError('Axis mapping inconsistent')
        wood=Poly3DCollection([face@R_CA.T for face in faces_A],facecolors=colors,
                             edgecolors='#765536',linewidths=.15,alpha=.45,rasterized=True)
        ax.add_collection3d(wood)
        for part in parts:
            xyz=((part['vertices_B_mm']+t_FB)@R_AF.T+t_AF)@R_CA.T
            tri=xyz[part['triangles']]
            normal=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])
            normal/=np.maximum(np.linalg.norm(normal,axis=1,keepdims=True),1e-12)
            light=np.array([-.3,-.5,1.]);light/=np.linalg.norm(light)
            shade=.55+.45*np.abs(normal@light)
            rgba=part['rgba'][None,:]*np.column_stack([shade,shade,shade,np.ones(len(shade))])
            ax.add_collection3d(Poly3DCollection(tri,facecolors=rgba,edgecolors='none',rasterized=True))
        for j,(name,center) in enumerate(zip(data['markers'],centers_A@R_CA.T)):
            ax.plot_surface(*(sphere+center[:,None,None]),color=palette(j),alpha=.85,linewidth=0)
            ax.text(*(center+[3,3,4]),name,color=palette(j),fontsize=8,fontweight='bold')
        # An offset triad shows body directions; it is not a COM marker.
        anchor=np.array([-58.,-55.,-48.])
        for name,vector,color in zip(['X_B','Y_B','Z_B'],R_CB.T,['crimson','forestgreen','royalblue']):
            ax.quiver(*anchor,*vector,length=25,color=color,linewidth=2,arrow_length_ratio=.18)
            ax.text(*(anchor+vector*30),name,color=color,fontsize=9,fontweight='bold')
        ax.quiver(74,-65,-30,0,0,1,length=65,color='black',linewidth=2,arrow_length_ratio=.12)
        ax.text(74,-65,40,'up',fontsize=9)
        ax.set(xlim=(-95,95),ylim=(-95,95),zlim=(-95,95))
        ax.set_box_aspect((1,1,1));ax.view_init(elev=23,azim=-58)
        ax.mouse_init(rotate_btn=1,zoom_btn=3)
        ax.set_axis_off();ax.set_title(titles[i],fontsize=13,pad=0)
        ax.text2D(.5,.03,captions[i],transform=ax.transAxes,ha='center',fontsize=11)
    fig.suptitle('Bifilar pendulum mountings: yaw → pitch → roll',fontsize=18,y=.95)
    fig.text(.5,.87,'The entire fixture, drone and marker cluster rotate together. Black arrows show the fixed vertical direction.',ha='center',fontsize=11)
    fig.text(.5,.055,'Nominal operator-described orientations; body direction triads are offset for clarity. String contacts and physical COM are not shown.',ha='center',fontsize=10)
    fig.subplots_adjust(left=.01,right=.99,bottom=.12,top=.82,wspace=.01)
    args.output.mkdir(parents=True,exist_ok=True)
    for ext in ['png','pdf']:
        target=args.output/('trial_mounting_sequence.'+ext)
        fig.savefig(target,dpi=170)
        print('Saved: '+str(target),flush=True)
    if args.save_only:plt.close(fig)
    else:plt.show()


if __name__=='__main__':main()
