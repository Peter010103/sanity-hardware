#!/usr/bin/env python3
"""Interactive mocap marker viewer, rotated into the clarified drone frame by default."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np


MARKERS = {
    'm1': [-75.749, -4.849, -9.692],
    'm2': [73.359, -6.420, 8.518],
    'm3': [-0.173, 7.680, 0.514],
    'm4': [-47.680, 7.599, 0.189],
    'm5': [-10.988, -3.145, 45.108],
    'm6': [7.232, -1.045, -44.637],
}


def frame_basis(markers, option='confirmed'):
    points={k:np.asarray(v,dtype=float) for k,v in markers.items()}
    if not all(k in points for k in ['m2','m3','m4','m5','m6']):
        raise ValueError('Drone-frame construction needs m2, m3, m4, m5 and m6')
    x_axis=points['m3']-points['m4']
    if np.linalg.norm(x_axis)<1e-9:raise ValueError('m3 and m4 must be distinct')
    x_axis/=np.linalg.norm(x_axis)
    y_axis=points['m5']-points['m6']
    y_axis-=x_axis*np.dot(y_axis,x_axis)
    if np.linalg.norm(y_axis)<1e-9:raise ValueError('X and Y marker references must not be parallel')
    y_axis/=np.linalg.norm(y_axis)
    if option in ['confirmed','keep-x','none']:y_axis=-y_axis
    if option=='keep-y':x_axis=-x_axis
    z_axis=np.cross(x_axis,y_axis)
    basis=np.column_stack([x_axis,y_axis,z_axis])
    if not np.allclose(basis.T@basis,np.eye(3)) or not np.isclose(np.linalg.det(basis),1.):
        raise ValueError('Failed to construct a right-handed orthonormal frame')
    if option in ['confirmed','none']:
        if not (np.dot(points['m2'],x_axis)>0 and np.dot(points['m6'],y_axis)>0 and np.dot(points['m3'],z_axis)>0):
            raise ValueError('Marker sides contradict +X/m2, +Y/m6, +Z/m3')
    return basis


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--units',default='mm',help='Coordinate-unit label; supplied values assumed millimetres')
    parser.add_argument('--markers-json',type=Path,help='Optional marker-name to [x,y,z] JSON, or a markers.json export')
    parser.add_argument('--frame',choices=['drone','mocap'],default='drone',help='Plot coordinate frame (default: drone)')
    parser.add_argument('--projections',action='store_true',help='Also show/save XY, XZ and YZ views in the selected frame')
    parser.add_argument('--drone-axes',choices=['confirmed','none','stated','keep-x','keep-y'],default='confirmed',
                        help='Axis interpretation; none hides arrows. Default: exact +X m4->m3, +Y m6 side, +Z top')
    parser.add_argument('--save-only',action='store_true',help='Save figures without opening interactive windows')
    parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parents[2]/'build/inertia/markers')
    args=parser.parse_args()
    import matplotlib
    if args.save_only:matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button

    payload=json.loads(args.markers_json.read_text()) if args.markers_json else MARKERS
    markers=payload.get('markers',payload)
    names=list(markers)
    supplied=np.array(list(markers.values()),dtype=float)
    if not names or supplied.shape!=(len(names),3) or not np.isfinite(supplied).all():
        parser.error('Marker data must contain finite [x,y,z] coordinates')
    basis=None
    if args.frame=='drone' or args.drone_axes!='none':
        try:basis=frame_basis(markers,args.drone_axes)
        except ValueError as exc:parser.error(str(exc))
    # Columns of basis are drone axes in supplied coordinates. Row vectors use p @ basis.
    drone=supplied@basis if basis is not None else None
    xyz=drone if args.frame=='drone' else supplied
    args.output.mkdir(parents=True,exist_ok=True)
    palette=plt.get_cmap('tab10');colors=[palette(i%10) for i in range(len(names))]
    span=max(float(np.ptp(xyz,axis=0).max()),1.);offset=span*.014
    fig=plt.figure(figsize=(10,8));ax=fig.add_subplot(111,projection='3d')
    for name,point,color in zip(names,xyz,colors):
        ax.scatter(*point,color=color,s=65,depthshade=False)
        ax.text(*(point+offset),name,color=color,fontsize=12,fontweight='bold')
    ax.scatter(0,0,0,c='black',marker='+',s=100)
    ax.text(-3*offset,-3*offset,-3*offset,'Motive pivot',color='black',fontsize=9)
    if args.drone_axes!='none':
        arrows=np.eye(3) if args.frame=='drone' else basis
        for label,vector,color in zip(['+X','+Y','+Z'],arrows.T,['crimson','forestgreen','royalblue']):
            ax.quiver(0,0,0,*vector,length=span*.25,color=color,arrow_length_ratio=.15,linewidth=2)
            ax.text(*(vector*span*.29),label,color=color,fontweight='bold')
    if 'm5' in names and 'm6' in names:
        ax.plot(*xyz[[names.index('m5'),names.index('m6')]].T,color='0.5',ls='--',lw=1)
    center=(np.maximum(xyz.max(axis=0),0)+np.minimum(xyz.min(axis=0),0))/2;radius=span*.60
    ax.set(xlim=(center[0]-radius,center[0]+radius),ylim=(center[1]-radius,center[1]+radius),
           zlim=(center[2]-radius,center[2]+radius),xlabel=f'X ({args.units})',
           ylabel=f'Y ({args.units})',zlabel=f'Z ({args.units})')
    ax.set_box_aspect((1,1,1));ax.view_init(elev=24,azim=-58);ax.mouse_init(rotate_btn=1,zoom_btn=3)
    subtitle='+X along m4 → m3; +Y toward m6; +Z toward top' if args.drone_axes in ['confirmed','none'] else 'Alternative axis interpretation: '+args.drone_axes
    if args.frame=='mocap':subtitle='Supplied coordinates; drone arrows shown' if args.drone_axes!='none' else 'Supplied coordinates; no axis remapping'
    ax.set_title(f'Marker coordinates in {args.frame} frame\n{subtitle}',pad=18)
    fig.subplots_adjust(bottom=.12,top=.9)
    fig.text(.5,.025,f'Units: {args.units}. Recorded Motive pivot preserved; no added translation. Equal axis scale.',ha='center',fontsize=9)
    suffix=args.frame+('' if args.drone_axes in ['confirmed','none'] else '_'+args.drone_axes)
    stem='markers_3d_'+suffix
    fig.savefig(args.output/(stem+'.png'),dpi=180);fig.savefig(args.output/(stem+'.pdf'))
    buttons=[]
    if not args.save_only:
        fig.text(.5,.965,'Left drag: rotate  |  Right drag: zoom',ha='center',fontsize=10)
        for i,(label,elev,azim) in enumerate([('3D',24,-58),('XY',90,-90),('XZ',0,-90),('YZ',0,0)]):
            button=Button(fig.add_axes([.30+i*.105,.065,.09,.035]),label)
            button.on_clicked(lambda event,e=elev,a=azim:(ax.view_init(elev=e,azim=a),fig.canvas.draw_idle()))
            buttons.append(button)
    if args.projections:
        projection,axes=plt.subplots(1,3,figsize=(13,4.7),layout='constrained')
        for subplot,(i,j,label) in zip(axes,[(0,1,'XY'),(0,2,'XZ'),(1,2,'YZ')]):
            for name,point,color in zip(names,xyz,colors):
                subplot.scatter(point[i],point[j],color=color,s=40)
                subplot.annotate(name,(point[i],point[j]),xytext=(4,5),textcoords='offset points',color=color)
            subplot.scatter(0,0,c='black',marker='+',s=70)
            subplot.set(xlabel=f'{"XYZ"[i]} ({args.units})',ylabel=f'{"XYZ"[j]} ({args.units})',title=label)
            subplot.set_aspect('equal',adjustable='box');subplot.grid(alpha=.25);subplot.margins(.18)
        projection.suptitle(f'Marker coordinates — {args.frame} frame projections')
        projection.savefig(args.output/('markers_projections_'+suffix+'.png'),dpi=180)
        projection.savefig(args.output/('markers_projections_'+suffix+'.pdf'))
    (args.output/'markers.json').write_text(json.dumps({'units':args.units,'coordinate_frame':'Supplied Motive pivot frame; original numeric values preserved','markers':markers},indent=2)+'\n')
    with (args.output/'markers.csv').open('w',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['marker','x','y','z','units'])
        writer.writerows([name,*point,args.units] for name,point in zip(names,supplied))
    if basis is not None:
        alignment={'status':'Approximate orthogonal marker basis for visualization; raw recordings and control unchanged',
                   'option':args.drone_axes,'origin_in_supplied_coordinates':[0.,0.,0.],
                   'origin_definition':'Recorded Motive pivot, centroid/X shift already applied in Motive',
                   'rotation_supplied_from_drone':basis.tolist(),'rotation_drone_from_supplied':basis.T.tolist(),
                   'transform_column_vector':'p_drone = rotation_drone_from_supplied @ p_supplied',
                   'additional_translation_applied':False,'motive_pivot_shift_already_applied':True,
                   'origin_is_confirmed_center_of_mass':False,'metric_orientation_calibration_confirmed':False,
                   'basis_method':'X exactly along m4->m3; Y from m5->m6 projected perpendicular to X; Z=X cross Y (confirmed option)',
                   'axis_priority':'X', 'x_reference':'m4->m3 exact direction',
                   'y_reference':'m5->m6 with X component removed',
                   'directions':({'positive_x':'m4 side','positive_y':'m5 side','positive_z':'m3/top side'} if args.drone_axes=='keep-y' else ({'positive_x':'m2 side','positive_y':'m5 side','positive_z':'away from top'} if args.drone_axes=='stated' else {'positive_x':'m2 side','positive_y':'m6 side','positive_z':'m3/top side'}))}
        (args.output/('axis_overlay_'+args.drone_axes+'.json')).write_text(json.dumps(alignment,indent=2)+'\n')
        data={'units':args.units,'coordinate_frame':'Drone basis: '+args.drone_axes,'markers':dict(zip(names,drone.tolist())),
              'rotation_drone_from_supplied':basis.T.tolist(),'additional_translation_applied':False}
        (args.output/('markers_drone'+('' if args.drone_axes in ['confirmed','none'] else '_'+args.drone_axes)+'.json')).write_text(json.dumps(data,indent=2)+'\n')
        with (args.output/('markers_drone_'+args.drone_axes+'.csv')).open('w',newline='') as stream:
            writer=csv.writer(stream);writer.writerow(['marker','x','y','z','units']);writer.writerows([name,*point,args.units] for name,point in zip(names,drone))
    print(f'Markers: {len(names)} | Display frame: {args.frame} | Units: {args.units}\nFigures and coordinates: {args.output}',flush=True)
    if args.save_only:plt.close('all')
    else:plt.show()


if __name__=='__main__':
    main()
