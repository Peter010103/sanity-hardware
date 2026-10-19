#!/usr/bin/env python3
"""Visualize Sanity, rev4 plywood and the confirmed nominal marker arrangement."""

import argparse
import csv
import hashlib
import importlib.util
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
DATA=ROOT/'data/processed/inertia/20261019'


def read_outlines(path):
    lines=path.read_text().splitlines()
    if len(lines)%2:raise ValueError('Malformed DXF group pairs')
    pairs=[(int(lines[i]),lines[i+1].strip()) for i in range(0,len(lines),2)]
    units=next((pairs[i+1][1] for i,pair in enumerate(pairs[:-1]) if pair==(9,'$INSUNITS')),None)
    if units!='4':raise ValueError('Expected DXF units millimetres ($INSUNITS=4)')
    section=None;entities=[];current=None
    for i,(code,value) in enumerate(pairs):
        if code==0 and value=='SECTION':section=pairs[i+1][1]
        elif code==0 and value=='ENDSEC':
            if section=='ENTITIES' and current:entities.append(current);current=None
            section=None
        elif section=='ENTITIES':
            if code==0:
                if current:entities.append(current)
                current={'type':value,'groups':[]}
            elif current is not None:current['groups'].append((code,value))
    polygons=[];circles=[]
    for entity in entities:
        groups=entity['groups']
        if entity['type']=='LWPOLYLINE':
            if not int(next(v for c,v in groups if c==70))&1:raise ValueError('Open cut outline')
            if any(c==42 and float(v)!=0 for c,v in groups):raise ValueError('Unexpected arc bulge')
            points=np.column_stack([[float(v) for c,v in groups if c==10],[float(v) for c,v in groups if c==20]])
            polygons.append(points)
        elif entity['type']=='CIRCLE':
            circles.append((np.array([float(next(v for c,v in groups if c==10)),float(next(v for c,v in groups if c==20))]),float(next(v for c,v in groups if c==40))))
        else:raise ValueError('Unsupported DXF entity: '+entity['type'])
    if len(polygons)!=3 or len(circles)!=1:raise ValueError('Expected two U outlines and one collar aperture/circle')
    profiles=sorted([p for p in polygons if np.isclose(np.ptp(p[:,0]),160)],key=lambda p:p[:,1].min())
    if len(profiles)!=2:raise ValueError('Cannot identify the two160mm U profiles')
    local=[]
    for points in profiles:
        if not np.isclose(np.ptp(points[:,1]),50):raise ValueError('Expected50mm U height')
        local.append(points-np.array([(points[:,0].min()+points[:,0].max())/2,points[:,1].min()]))
    hole=next(p for p in polygons if not np.isclose(np.ptp(p[:,0]),160))-circles[0][0]
    radius=circles[0][1]
    if not np.isclose(radius,15):raise ValueError('Expected30mm collar diameter')
    return local,hole,radius


def polygon_moments(points):
    x,y=points.T;u=np.roll(x,-1);v=np.roll(y,-1);cross=x*v-u*y
    moments=np.array([cross.sum()/2,((x+u)*cross).sum()/6,((y+v)*cross).sum()/6,
        ((x*x+x*u+u*u)*cross).sum()/12,((y*y+y*v+v*v)*cross).sum()/12,
        ((2*x*y+x*v+u*y+2*u*v)*cross).sum()/24])
    return moments if moments[0]>0 else -moments


def nominal_properties(profiles,hole,radius,thickness=3.):
    volumes=[];firsts=[];seconds=[];parts=[]
    for horizontal,points in enumerate(profiles):
        area,mx,mz,mxx,mzz,mxz=polygon_moments(points);volume=area*thickness
        first=np.zeros(3);second=np.zeros((3,3))
        first[horizontal]=thickness*mx;first[2]=thickness*mz
        second[horizontal,horizontal]=thickness*mxx;second[2,2]=thickness*mzz
        second[horizontal,2]=second[2,horizontal]=thickness*mxz
        second[1-horizontal,1-horizontal]=area*thickness**3/12
        volumes.append(volume);firsts.append(first);seconds.append(second)
        parts.append({'part':'U_X' if horizontal==0 else 'U_Y','area_mm2':area,'volume_mm3':volume})
    hole_m=polygon_moments(hole);area=np.pi*radius**2-hole_m[0];volume=area*thickness
    z=10+thickness/2
    first=np.array([-thickness*hole_m[1],-thickness*hole_m[2],volume*z])
    second=np.array([[thickness*(np.pi*radius**4/4-hole_m[3]),-thickness*hole_m[5],0],
                     [-thickness*hole_m[5],thickness*(np.pi*radius**4/4-hole_m[4]),0],
                     [0,0,volume*(z*z+thickness**2/12)]])
    volumes.append(volume);firsts.append(first);seconds.append(second)
    parts.append({'part':'collar','area_mm2':area,'volume_mm3':volume,'bottom_z_mm':10,'top_z_mm':13})
    volume=sum(volumes);center=sum(firsts)/volume;second=sum(seconds)
    inertia=np.trace(second)*np.eye(3)-second-volume*(np.dot(center,center)*np.eye(3)-np.outer(center,center))
    coefficients=inertia/volume*1e-6
    supplied_center=np.array([0,0,.0126384]);supplied_diag=np.array([.001555166,.001555166,.002800631])
    matches=bool(np.allclose(center/1000,supplied_center,atol=1e-9,rtol=0) and np.allclose(coefficients,np.diag(supplied_diag),atol=5e-10,rtol=0))
    return {'uniform_density':True,'nominal_thickness_m':thickness/1000,'parts':parts,
            'total_volume_m3':volume*1e-9,'centroid_F_m':(center/1000).tolist(),
            'centroidal_inertia_per_kg_m2':coefficients.tolist(),
            'matches_operator_supplied_coefficients_within_rounding':matches,
            'plywood_mass_kg':None,'mass_dependent_inertia_not_computed':True}


def solid_faces(profiles,hole,radius,thickness=3.):
    faces=[];colors=[]
    for horizontal,points in enumerate(profiles):
        sides=[]
        for sign in [-1,1]:
            face=np.zeros((len(points),3));face[:,horizontal]=points[:,0]
            face[:,1-horizontal]=sign*thickness/2;face[:,2]=points[:,1]
            sides.append(face);faces.append(face);colors.append('#bd8b51' if horizontal==0 else '#d8b17b')
        for i in range(len(points)):
            j=(i+1)%len(points);faces.append(np.array([sides[0][i],sides[0][j],sides[1][j],sides[1][i]]));colors.append('#b58750')
    # Tessellated circular edge is for display only. Nominal properties use exact circle integrals.
    angles=np.unique(np.r_[np.linspace(0,2*np.pi,129),np.mod(np.arctan2(hole[:,1],hole[:,0]),2*np.pi)])
    cx=np.maximum(abs(np.cos(angles)),1e-12);sy=np.maximum(abs(np.sin(angles)),1e-12)
    inner=np.maximum(np.minimum(9/cx,1.5/sy),np.minimum(1.5/cx,9/sy))
    inner_xy=np.column_stack([inner*np.cos(angles),inner*np.sin(angles)])
    outer_xy=radius*np.column_stack([np.cos(angles),np.sin(angles)])
    for i in range(len(angles)-1):
        for height in [10,13]:
            face=np.column_stack([np.array([inner_xy[i],outer_xy[i],outer_xy[i+1],inner_xy[i+1]]),np.full(4,height)])
            faces.append(face);colors.append('#e2c28c')
        for boundary in [inner_xy,outer_xy]:
            face=np.array([[*boundary[i],10],[*boundary[i+1],10],[*boundary[i+1],13],[*boundary[i],13]])
            faces.append(face);colors.append('#b78c55')
    return faces,colors


def mass_ledger_g():
    return {'total_suspended':87.38,'drone_separately_weighed':69.47,
            'fixture_markers_tape_glue_mounts':17.90,'component_sum':87.37,
            'total_minus_component_sum':.01,'six_marker_sum_approx':6*1.08,
            'plywood_plus_other_fixture_attachments_residual_approx':11.42,
            'plywood_only':None,'tape_glue_mounts_excluding_markers':None,
            'note':'Independent measured readings retained; 0.01g closure difference, uncertainty unspecified.'}


def load_drone_visuals(urdf):
    """Read base_link visual OBJ meshes, retaining URDF scale, pose and colour."""
    urdf=urdf.resolve()
    root=ET.parse(urdf).getroot();base=root.find("link[@name='base_link']")
    parts=[]
    for visual in base.findall('visual'):
        mesh=visual.find('geometry/mesh')
        if mesh is None:raise ValueError('Expected mesh visual in Sanity URDF')
        path=urdf.parent/mesh.get('filename');vertices=[];triangles=[]
        for line in path.read_text().splitlines():
            fields=line.split()
            if not fields:continue
            if fields[0]=='v':vertices.append([float(x) for x in fields[1:4]])
            elif fields[0]=='f':
                idx=[int(x.split('/')[0]) for x in fields[1:]]
                idx=[i-1 if i>0 else len(vertices)+i for i in idx]
                triangles.extend([[idx[0],idx[i],idx[i+1]] for i in range(1,len(idx)-1)])
        v=np.array(vertices)*np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
        origin=visual.find('origin');R=np.eye(3);t=np.zeros(3)
        if origin is not None:
            t=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
            r,p,y=np.fromstring(origin.get('rpy','0 0 0'),sep=' ')
            Rx=np.array([[1,0,0],[0,np.cos(r),-np.sin(r)],[0,np.sin(r),np.cos(r)]])
            Ry=np.array([[np.cos(p),0,np.sin(p)],[0,1,0],[-np.sin(p),0,np.cos(p)]])
            Rz=np.array([[np.cos(y),-np.sin(y),0],[np.sin(y),np.cos(y),0],[0,0,1]])
            R=Rz@Ry@Rx
        v=(v@R.T+t)*1000
        color=visual.find('material/color')
        rgba=np.fromstring(color.get('rgba'),sep=' ') if color is not None else np.array([.3,.3,.3,1])
        parts.append({'name':visual.get('name'),'vertices_B_mm':v,
                      'triangles':np.asarray(triangles),'rgba':rgba,
                      'source':str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
    return parts


def drone_mount(parts,offset):
    # B axes parallel F. In A this is a 180-degree roll, not a reflection.
    vertices=np.vstack([p['vertices_B_mm'] for p in parts])
    in_collar=(np.linalg.norm(vertices[:,:2],axis=1)<=15)
    in_aperture=((abs(vertices[:,0])<=9)&(abs(vertices[:,1])<=1.5))|((abs(vertices[:,1])<=9)&(abs(vertices[:,0])<=1.5))
    support_z=float(vertices[in_collar&~in_aperture,2].min())
    t=np.array([0.,0.,13.-support_z])+np.asarray(offset)
    pose={'rotation_F_from_B':np.eye(3).tolist(),'translation_F_from_B_mm':t.tolist(),
          'operator_confirmed':'Front along +X_A; +Z_B down, battery side toward collar; centred collar contact.',
          'translation_status':'Nominal CAD-derived contact placement, not a measured COM or calibrated rigid transform',
          'contact_method':'Lowest battery-side external mesh vertex over solid collar footprint placed at Z_F=13mm',
          'mesh_support_z_B_mm':support_z,'nominal_contact_plane_F_mm':13.,
          'offset_override_F_mm':list(offset),'body_origin_is_measured_COM':False,
          'meshes':[{'source':p['source'],'sha256':p['sha256']} for p in parts],
          'used_for_inertia_correction':False}
    return t,pose


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dxf',type=Path,default=ROOT/'data/raw/bifilar/20261019/u_mount_rev4.dxf')
    parser.add_argument('--output',type=Path,default=ROOT/'build/inertia/preview')
    parser.add_argument('--view',choices=['3d','xy','xz','yz'],default='3d',help='Initial view; side projections are useful for inspecting collar contact')
    parser.add_argument('--frame',choices=['assembly','fixture'],default='assembly',help='Display frame: assembly A (+Z toward m3) or original fixture F')
    parser.add_argument('--m3-xy',type=float,nargs=2,default=[0.,0.],metavar=('X_MM','Y_MM'),help='Override nominal m3 contact location in F; default0,0 is operator confirmed')
    parser.add_argument('--urdf',type=Path,default=ROOT/'models/sanity.urdf')
    parser.add_argument('--no-drone',action='store_true',help='Hide the drone mesh')
    parser.add_argument('--drone-offset',type=float,nargs=3,default=[0.,0.,0.],metavar=('X_MM','Y_MM','Z_MM'),help='Visual offset in F from the centred nominal collar-contact placement')
    parser.add_argument('--no-markers',action='store_true',help='Show only the nominal plywood geometry')
    parser.add_argument('--save-only',action='store_true',help='Save previews without opening a desktop window')
    args=parser.parse_args()
    import matplotlib
    if args.save_only:matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button,CheckButtons
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    profiles,hole,radius=read_outlines(args.dxf)
    properties=nominal_properties(profiles,hole,radius)
    if not properties['matches_operator_supplied_coefficients_within_rounding']:raise ValueError('DXF nominal properties disagree with supplied reference')
    args.output.mkdir(parents=True,exist_ok=True)
    properties['dxf_sha256']=hashlib.sha256(args.dxf.read_bytes()).hexdigest()
    (args.output/'nominal_fixture_properties.json').write_text(json.dumps(properties,indent=2)+'\n')
    path=ROOT/'tools/acquisition/visualize_markers.py'
    spec=importlib.util.spec_from_file_location('marker_view',path);marker_view=importlib.util.module_from_spec(spec);spec.loader.exec_module(marker_view)
    names=list(marker_view.MARKERS);supplied=np.array(list(marker_view.MARKERS.values()))
    basis=marker_view.frame_basis(marker_view.MARKERS)
    body=supplied@basis
    body_to_F=np.diag([1.,-1.,-1.])
    target=np.array([*args.m3_xy,-6.35])
    centers=(body-body[names.index('m3')])@body_to_F.T+target
    confirmed=bool(np.allclose(args.m3_xy,[0,0]))
    t_FM=target-body_to_F@body[names.index('m3')]
    R_AF=body_to_F.T
    t_AF=-R_AF@t_FM
    parts=load_drone_visuals(args.urdf)
    t_FB,drone_pose=drone_mount(parts,args.drone_offset)
    drone_pose['rotation_A_from_B']=R_AF.tolist()
    drone_pose['translation_A_from_B_mm']=(R_AF@t_FB+t_AF).tolist()
    registration={'status':'Operator-confirmed nominal layout; uncertainty not quantified; inertia correction deferred' if confirmed else 'Unconfirmed m3 XY override for visualization',
                  'operator_confirmed_nominal_layout':confirmed,
                  'registration_uncertainty_quantified':False,
                  'source_marker_frame':'Supplied Motive pivot; pivot shift already included',
                  'marker_diameter_mm':12.7,'marker_mass_g_approx':1.08,
                  'construction':['Fixture +X follows m4->m3; assembly +X is the same.','Fixture +Z is opposite assembly +Z; assembly +Z points toward m3/top.',
                                 'm3 nominal centre is [X,Y,-6.35]mm; default X=Y=0 placement was confirmed by operator.',
                                 'All six marker centres move as a rigid cluster; m3/m4 spheres touch bottom plane Z_F=0.',
                                 'Marker sphere shape used for display only; internal mass distribution unknown.',
                                 'Drone centred over collar with battery-side contact; nominal CAD contact offset, COM unresolved.'],
                  'm3_xy_nominal_mm':args.m3_xy,'rotation_F_from_mocap':(body_to_F@basis.T).tolist(),
                  'assembly_A':{'origin':'Recorded Motive pivot, not a measured centre of mass',
                                'positive_x':'m4 toward m3; m2 positive side','positive_y':'m6 side','positive_z':'m3/top side'},
                  'rotation_A_from_F':R_AF.tolist(),'translation_A_from_F_mm':t_AF.tolist(),
                  'display_frame':args.frame,
                  'translation_F_from_mocap_mm':t_FM.tolist(),
                  'markers_F_mm':dict(zip(names,centers.tolist())),
                  'used_for_inertia_correction':False,'raw_recordings_modified':False,
                  'mass_ledger_g':mass_ledger_g(),'drone':drone_pose}
    (args.output/'registration.json').write_text(json.dumps(registration,indent=2)+'\n')
    with (args.output/'marker_centers_F_mm.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(['marker','x_F_mm','y_F_mm','z_F_mm']);writer.writerows([name,*point] for name,point in zip(names,centers))
    fig=plt.figure(figsize=(12,9));ax=fig.add_subplot(111,projection='3d')
    faces,colors=solid_faces(profiles,hole,radius)
    if args.frame=='assembly':faces=[face@R_AF.T+t_AF for face in faces]
    centers_display=centers@R_AF.T+t_AF if args.frame=='assembly' else centers
    wood=Poly3DCollection(faces,facecolors=colors,edgecolors='#765536',linewidths=.18,alpha=.55)
    ax.add_collection3d(wood)
    drone_actors=[]
    for part in parts:
        v=part['vertices_B_mm']+t_FB
        if args.frame=='assembly':v=v@R_AF.T+t_AF
        tri=v[part['triangles']]
        normal=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])
        normal/=np.maximum(np.linalg.norm(normal,axis=1,keepdims=True),1e-12)
        light=np.array([-.3,-.5,1.]);light/=np.linalg.norm(light)
        shade=.55+.45*np.abs(normal@light)
        colors=part['rgba'][None,:]*np.column_stack([shade,shade,shade,np.ones(len(shade))])
        actor=Poly3DCollection(tri,facecolors=colors,edgecolors='none',linewidths=0,rasterized=True)
        actor.set_visible(not args.no_drone);ax.add_collection3d(actor);drone_actors.append(actor)
    origin_B=R_AF@t_FB+t_AF if args.frame=='assembly' else t_FB
    # Separate direction triad to avoid hiding the central collar and battery.
    origin_B+=R_AF@np.array([-53.,50.,-14.]) if args.frame=='assembly' else np.array([-53.,50.,-14.])
    R_display_B=R_AF if args.frame=='assembly' else np.eye(3)
    for name,vector,color in zip(['front / +X_B','+Y_B','+Z_B'],R_display_B.T,['crimson','forestgreen','royalblue']):
        arrow=ax.quiver(*origin_B,*vector,length=23,color=color,arrow_length_ratio=.15,linewidth=2.2)
        label=ax.text(*(origin_B+vector*26),name,color=color,fontsize=9,fontweight='bold')
        for actor in [arrow,label]:actor.set_visible(not args.no_drone);drone_actors.append(actor)
    actors=[];labels=[];palette=plt.get_cmap('tab10')
    u,v=np.meshgrid(np.linspace(0,2*np.pi,25),np.linspace(0,np.pi,15))
    sphere=np.array([6.35*np.cos(u)*np.sin(v),6.35*np.sin(u)*np.sin(v),6.35*np.cos(v)])
    for i,(name,center) in enumerate(zip(names,centers_display)):
        xyz=sphere+center[:,None,None]
        actor=ax.plot_surface(*xyz,color=palette(i),alpha=.85,linewidth=0,shade=True)
        label=ax.text(*(center+np.array([3,3,-9])),name,color=palette(i),fontsize=11,fontweight='bold')
        actor.set_visible(not args.no_markers);label.set_visible(not args.no_markers)
        actors.append(actor);labels.append(label)
    G=np.array(properties['centroid_F_m'])*1000
    if args.frame=='assembly':G=R_AF@G+t_AF
    ax.scatter(*G,color='crimson',marker='*',s=110)
    ax.text(*(G+np.array([3,3,1])),'plywood G',color='crimson',fontsize=9)
    frame_letter='A' if args.frame=='assembly' else 'F'
    for label,vector,color in zip(['X_'+frame_letter,'Y_'+frame_letter,'Z_'+frame_letter],np.eye(3),['crimson','forestgreen','royalblue']):
        ax.quiver(0,0,0,*vector,length=32,color=color,arrow_length_ratio=.15,linewidth=1.8)
        ax.text(*(vector*37),label,color=color,fontweight='bold')
    ax.scatter(0,0,0,c='black',marker='+',s=70)
    ax.set(xlim=(-88,88),ylim=(-88,88),zlim=(-57,17) if args.frame=='assembly' else (-17,57),
           xlabel='X_'+frame_letter+' (mm)',ylabel='Y_'+frame_letter+' (mm)',zlabel='Z_'+frame_letter+' (mm)')
    elev,azim={'3d':(23,-58),'xy':(90,-90),'xz':(0,-90),'yz':(0,0)}[args.view]
    ax.set_box_aspect((176,176,74));ax.view_init(elev=elev,azim=azim);ax.mouse_init(rotate_btn=1,zoom_btn=3)
    if args.save_only and args.view!='3d':
        hidden_axis={'xy':ax.zaxis,'xz':ax.yaxis,'yz':ax.xaxis}[args.view]
        hidden_axis.set_ticks([]);hidden_axis.set_label_text('')
    ax.set_title('Sanity drone + revision-4 fixture + markers\n'+('Assembly A: +Z toward m3, +Y toward m6' if args.frame=='assembly' else 'Fixture F: +Z toward open upright ends'),pad=16)
    fig.subplots_adjust(bottom=.18,top=.88,left=.04,right=.96)
    fig.text(.5,.945,'Drone 69.47 g | Fixture package 17.90 g | Measured total 87.38 g (sum difference 0.01 g)',ha='center',fontsize=10)
    fig.text(.5,.05,('Operator-confirmed nominal layout: m3 centred below the base in F; m3/m4 touch the underside.\nDrone front +X_A; battery side up toward collar; +Z_B down. CAD contact offset is nominal, not a measured COM.' if confirmed else 'Unconfirmed m3 XY override; other geometry is nominal.\nNominal CAD drone contact placement; glue/mount geometry not shown.'),ha='center',fontsize=10)
    stem=('fixture_and_markers_' if args.no_drone else 'fixture_drone_and_markers_')+args.frame
    if args.view!='3d':stem+='_'+args.view
    fig.savefig(args.output/(stem+'.png'),dpi=180)
    fig.savefig(args.output/(stem+'.pdf'))
    controls=[]
    if not args.save_only:
        fig.text(.5,.985,'Left drag: rotate | Right drag: zoom | Views use the selected coordinate frame',ha='center',fontsize=10)
        for i,(label,e,a) in enumerate([('3D',23,-58),('XY',90,-90),('XZ',0,-90),('YZ',0,0)]):
            button=Button(fig.add_axes([.31+i*.105,.115,.09,.035]),label)
            button.on_clicked(lambda event,e=e,a=a:(ax.view_init(elev=e,azim=a),fig.canvas.draw_idle()))
            controls.append(button)
        checkbox=CheckButtons(fig.add_axes([.04,.09,.16,.09]),['Plywood','Markers','Drone'],[True,not args.no_markers,not args.no_drone])
        def toggle(label):
            if label=='Plywood':wood.set_visible(not wood.get_visible())
            elif label=='Drone':
                visible=not drone_actors[0].get_visible()
                for actor in drone_actors:actor.set_visible(visible)
            else:
                visible=not actors[0].get_visible()
                for actor in actors+labels:actor.set_visible(visible)
            fig.canvas.draw_idle()
        checkbox.on_clicked(toggle);controls.append(checkbox)
    print('Saved: '+str(args.output/(stem+'.png')),flush=True)
    print('DXF centroid/inertia coefficients match supplied values. Plywood mass remains unknown.',flush=True)
    print(('Confirmed nominal placement' if confirmed else 'Unconfirmed override')+': m3 at '+str(target.tolist())+' mm in F. Raw measurement data unchanged.',flush=True)
    if args.save_only:plt.close('all')
    else:plt.show()


if __name__=='__main__':main()
