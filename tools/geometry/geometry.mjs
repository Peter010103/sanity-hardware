import fs from 'node:fs/promises';
import path from 'node:path';
import crypto from 'node:crypto';
import zlib from 'node:zlib';
export const I=()=>[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1];
export const mul=(a,b)=>Array.from({length:16},(_,i)=>[0,1,2,3].reduce((s,k)=>s+a[Math.floor(i/4)*4+k]*b[k*4+i%4],0));
export const xyz=(m,p)=>[0,1,2].map(i=>m[i*4]*p[0]+m[i*4+1]*p[1]+m[i*4+2]*p[2]+m[i*4+3]);
export const nums=s=>s.trim().split(/\s+/).map(Number);
export const children=(n,t)=>n.children.filter(x=>x.tag===t);
export const child=(n,t)=>children(n,t)[0];
export const all=(n,t)=>[...(n.tag===t?[n]:[]),...n.children.flatMap(x=>all(x,t))];
export const sha=b=>crypto.createHash('sha256').update(b).digest('hex');
export const norm=a=>{const l=Math.hypot(...a);return l?a.map(x=>x/l):[0,0,1];};
export const sub=(a,b)=>a.map((x,i)=>x-b[i]);
export const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
export const fmt=x=>Number(x.toFixed(10)).toString();
export function bounds(v){const lo=[Infinity,Infinity,Infinity],hi=[-Infinity,-Infinity,-Infinity];for(const p of v)for(let i=0;i<3;i++){lo[i]=Math.min(lo[i],p[i]);hi[i]=Math.max(hi[i],p[i]);}return {lo,hi,size:hi.map((x,i)=>x-lo[i]),center:hi.map((x,i)=>(x+lo[i])/2)};}
export function parseXML(s){
 if(/<!DOCTYPE|<!ENTITY/.test(s))throw Error('DTD forbidden');s=s.replace(/<!--[\s\S]*?-->/g,'');
 const root={tag:'document',children:[],text:''},stack=[root];
 for(const t of s.match(/<[^>]*>|[^<]+/g)||[]){
  if(t.startsWith('<?'))continue;
  if(t.startsWith('</')){if(stack.length<2||stack.pop().tag!==t.slice(2,-1).trim())throw Error('Unbalanced XML');}
  else if(t.startsWith('<')){const m=t.match(/^<([\w:.-]+)/);if(!m)throw Error('Unsupported XML');const a=Object.fromEntries([...t.matchAll(/([\w:.-]+)="([^"]*)"/g)].map(x=>[x[1],x[2]]));const n={tag:m[1],a,children:[],text:''};stack.at(-1).children.push(n);if(!t.endsWith('/>'))stack.push(n);}
  else stack.at(-1).text+=t;
 }
 if(stack.length!==1||root.children.length!==1)throw Error('Unclosed/multiple-root XML');return root.children[0];
}
export function loadDAE(text){
 const doc=parseXML(text),asset=child(doc,'asset');
 if(child(asset,'up_axis').text.trim()!=='Z_UP'||+child(asset,'unit').a.meter!==1)throw Error('Unexpected source unit/axis');
 if(all(doc,'image').length||all(doc,'texture').length)throw Error('Texture handling required');
 const effects=Object.fromEntries(all(doc,'effect').map(e=>[e.a.id,nums(child(all(e,'diffuse')[0],'color').text)]));
 const materials=Object.fromEntries(all(doc,'material').map((m,i)=>[m.a.id,{name:'material_'+i,rgba:effects[child(m,'instance_effect').a.url.slice(1)]}]));
 const geos=Object.fromEntries(all(doc,'geometry').map(g=>[g.a.id,g])),pieces=[],hierarchy=[],rebase=I();rebase[11]=0.0028;
 function walk(n,parent=I(),anc=''){
  const local=child(n,'matrix')?nums(child(n,'matrix').text):I(),m=mul(parent,local),body=mul(rebase,m),nodePath=anc+'/'+(n.a.name||n.tag);
  if(children(n,'rotate').length||children(n,'scale').length||children(n,'translate').length)throw Error('Unsupported transform');
  hierarchy.push({path:nodePath,local_matrix:local,world_matrix:m});
  for(const ins of children(n,'instance_geometry')){
   const g=geos[ins.a.url.slice(1)],mesh=child(g,'mesh'),sources={};
   for(const s of children(mesh,'source')){const a=nums(child(s,'float_array').text);if(a.length!==+child(s,'float_array').a.count)throw Error('Array count');sources[s.a.id]=a;}
   const raw=sources[child(child(mesh,'vertices'),'input').a.source.slice(1)],v=Array.from({length:raw.length/3},(_,i)=>xyz(body,raw.slice(3*i,3*i+3)));
   const bindings=Object.fromEntries(all(ins,'instance_material').map(x=>[x.a.symbol,x.a.target.slice(1)])),triangles=[];
   for(const pr of mesh.children.filter(x=>['triangles','polylist','polygons','trifans','tristrips'].includes(x.tag))){
    if(pr.tag!=='triangles')throw Error('Nontriangular primitive');
    const inputs=children(pr,'input'),stride=1+Math.max(...inputs.map(x=>+x.a.offset)),p=nums(child(pr,'p').text),vi=inputs.find(x=>x.a.semantic==='VERTEX'),ni=inputs.find(x=>x.a.semantic==='NORMAL'),normal=ni?sources[ni.a.source.slice(1)]:null;
    if(p.length!==+pr.a.count*3*stride)throw Error('Triangle count');
    for(let k=0;k<p.length;k+=3*stride){const face=[],ns=[];
     for(let j=0;j<3;j++){const idx=p[k+j*stride+(+vi.a.offset)];if(!v[idx])throw Error('Invalid vertex');face.push(idx);
      if(normal){const ni3=3*p[k+j*stride+(+ni.a.offset)],r=body.slice();r[3]=r[7]=r[11]=0;ns.push(norm(xyz(r,normal.slice(ni3,ni3+3))));}}
     if(!normal){const n=norm(cross(sub(v[face[1]],v[face[0]]),sub(v[face[2]],v[face[0]])));ns.push(n,n,n);}
     triangles.push({face,normals:ns,material:bindings[pr.a.material]||pr.a.material});
    }
   }
   const det=body[0]*(body[5]*body[10]-body[6]*body[9])-body[1]*(body[4]*body[10]-body[6]*body[8])+body[2]*(body[4]*body[9]-body[5]*body[8]);
   if(det<=0)throw Error('Mirrored/singular transform');
   pieces.push({name:n.a.name,path:nodePath,geometry:g.a.id,source_vertices:raw,vertices:v,triangles,bounds:bounds(v),body_matrix:body,determinant:det});
  }
  for(const c of children(n,'node'))walk(c,m,nodePath);
 }walk(all(doc,'visual_scene')[0]);
 return {pieces,hierarchy,materials,bounds:bounds(pieces.flatMap(p=>p.vertices)),source_to_body:rebase};
}
export function placeRotors(model){
 const shaftCoordinate=0.0284610479427585;
 for(const p of model.pieces.filter(p=>/^Body1\.00[1-4]$/.test(p.name))){
  const delta=[0,1].map(k=>Math.sign(p.body_matrix[k*4+3])*shaftCoordinate-p.body_matrix[k*4+3]);
  p.source_to_body_translation_m=[...delta,0];
  p.vertices=p.vertices.map(v=>[v[0]+delta[0],v[1]+delta[1],v[2]]);
  p.body_matrix[3]+=delta[0];p.body_matrix[7]+=delta[1];p.bounds=bounds(p.vertices);
 }
 model.bounds=bounds(model.pieces.flatMap(p=>p.vertices));
 return model;
}
export async function build(sourceDir,out,reportDir){
 // The URDF owns the physical parameters; mesh conversion preserves them.
 const existingUrdf=await fs.readFile(path.join(out,'sanity.urdf'),'utf8');
 const baseXml=existingUrdf.match(/<link name="base_link">([\s\S]*?)<\/link>/)?.[1];
 const inertialXml=baseXml?.match(/<inertial>[\s\S]*?<\/inertial>/)?.[0];
 if(!inertialXml)throw Error('models/sanity.urdf must define base_link inertial properties');
 const files=await fs.readdir(sourceDir),daes=files.filter(f=>/\.dae$/i.test(f));if(daes.length!==1)throw Error('Need exactly one DAE in source dir');
 const model=loadDAE(await fs.readFile(path.join(sourceDir,daes[0]),'utf8'));
 placeRotors(model);
 for(const d of ['meshes/visual','meshes/collision','sources'])await fs.mkdir(path.join(out,d),{recursive:true});
 await fs.mkdir(reportDir,{recursive:true});
 const inventory=[];
 for(const f of files.filter(f=>/\.(dae|step|stp|usdz|blend)$/i.test(f))){const b=await fs.readFile(path.join(sourceDir,f));if(path.resolve(sourceDir)!==path.resolve(out,'sources'))await fs.writeFile(path.join(out,'sources',f),b);inventory.push({source_name:f,bytes:b.length,sha256:sha(b),rights:'CC-BY-4.0'});}
 const visuals=[];let mtl='# Source DAE diffuse colors; no textures\n';
 for(const [id,mat] of Object.entries(model.materials)){
  mtl+='newmtl '+mat.name+'\nKd '+mat.rgba.slice(0,3).join(' ')+'\nd '+mat.rgba[3]+'\n\n';
  const vs=[],ns=[],faces=[],index=new Map();
  for(const p of model.pieces)for(const t of p.triangles.filter(t=>t.material===id)){
   const f=[];for(let j=0;j<3;j++){const v=p.vertices[t.face[j]],n=t.normals[j],key=[...v,...n].map(fmt).join(',');let i=index.get(key);if(i===undefined){i=vs.length+1;index.set(key,i);vs.push(v);ns.push(n);}f.push(i);}faces.push(f);
  }
  const obj='# Metres; base_link; geometry only\nmtllib materials.mtl\nusemtl '+mat.name+'\n'+vs.map(v=>'v '+v.map(fmt).join(' ')).join('\n')+'\n'+ns.map(n=>'vn '+n.map(fmt).join(' ')).join('\n')+'\n'+faces.map(f=>'f '+f.map(i=>i+'//'+i).join(' ')).join('\n')+'\n';
  await fs.writeFile(path.join(out,'meshes/visual',mat.name+'.obj'),obj);
  visuals.push('<visual name="'+mat.name+'"><geometry><mesh filename="meshes/visual/'+mat.name+'.obj" scale="1 1 1"/></geometry><material name="'+mat.name+'"><color rgba="'+mat.rgba.join(' ')+'"/></material></visual>');
 }
 await fs.writeFile(path.join(out,'meshes/visual/materials.mtl'),mtl.trimEnd()+'\n');
 const {lo,hi}=model.bounds,box=Array.from({length:8},(_,i)=>[i&1?hi[0]:lo[0],i&2?hi[1]:lo[1],i&4?hi[2]:lo[2]]),bf=[[0,2,3],[0,3,1],[4,5,7],[4,7,6],[0,1,5],[0,5,4],[2,6,7],[2,7,3],[0,4,6],[0,6,2],[1,3,7],[1,7,5]];
 await fs.writeFile(path.join(out,'meshes/collision/envelope.obj'),'# Conservative envelope; fills guard openings and corners\n'+box.map(v=>'v '+v.map(fmt).join(' ')).join('\n')+'\n'+bf.map(f=>'f '+f.map(i=>i+1).join(' ')).join('\n')+'\n');
 const names=['rear_right','front_right','front_left','rear_left'];
 const rotors=['Body1.001','Body1.002','Body1.003','Body1.004'].map((name,i)=>{const p=model.pieces.find(p=>p.name===name);return {id:i+1,name:names[i],source_propeller:name,position_m:[p.body_matrix[3],p.body_matrix[7],p.body_matrix[11]],thrust_axis_body:[0,0,1],position_status:'CAD motor shaft; hardware frame registration unconfirmed',hardware_output:null,rotation_direction:null};});
 let urdf='<?xml version="1.0"?>\n<!-- SI units. All aircraft mass and inertia are on base_link; rotor links define massless frames. -->\n<robot name="sanity">\n<link name="base_link">\n'+inertialXml+'\n'+visuals.join('\n')+'\n<collision name="conservative_envelope"><geometry><mesh filename="meshes/collision/envelope.obj" scale="1 1 1"/></geometry></collision>\n</link>\n';
 for(const r of rotors)urdf+='<link name="rotor_'+r.id+'_'+r.name+'"><inertial><mass value="0"/><inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/></inertial></link>\n<joint name="rotor_'+r.id+'_frame" type="fixed"><parent link="base_link"/><child link="rotor_'+r.id+'_'+r.name+'"/><origin xyz="'+r.position_m.map(fmt).join(' ')+'" rpy="0 0 0"/></joint>\n';
 await fs.writeFile(path.join(out,'sanity.urdf'),urdf+'</robot>\n');
 const prop=model.pieces.find(p=>p.name==='Body1.001').source_vertices;let propRadius=0;for(let i=0;i<prop.length;i+=3)propRadius=Math.max(propRadius,Math.hypot(prop[i],prop[i+1])*.01);
 const report={propeller_visual_max_radius_m:propRadius,propeller_visual_swept_diameter_m:2*propRadius,converter_version:'2.0.0',source_file:daes[0],units:'m',body_origin:'CoM at base_link origin, confirmed by operator',axis_convention:'right-handed +X provisional forward along battery, +Y left, +Z up; physical nose unverified',source_to_body:model.source_to_body,bounds_m:model.bounds,triangle_count:model.pieces.reduce((s,p)=>s+p.triangles.length,0),collision_triangles:12,materials:model.materials,rotors,hierarchy:model.hierarchy,pieces:model.pieces.map(({vertices,triangles,source_vertices,...p})=>({...p,vertex_count:vertices.length,triangle_count:triangles.length}))};
 await fs.writeFile(path.join(reportDir,'geometry.json'),JSON.stringify(report,null,2)+'\n');
 await fs.writeFile(path.join(out,'sources/inventory.json'),JSON.stringify(inventory,null,2)+'\n');
 return {model,report,inventory};
}
export function crc32(b){let c=0xffffffff;for(const x of b){c^=x;for(let k=0;k<8;k++)c=(c>>>1)^((c&1)?0xedb88320:0);}return (c^0xffffffff)>>>0;}
export function png(w,h,pixels){
 const chunk=(name,b)=>{const n=Buffer.from(name),head=Buffer.alloc(4),tail=Buffer.alloc(4);head.writeUInt32BE(b.length);tail.writeUInt32BE(crc32(Buffer.concat([n,b])));return Buffer.concat([head,n,b,tail]);};
 const head=Buffer.alloc(13);head.writeUInt32BE(w);head.writeUInt32BE(h,4);head[8]=8;head[9]=2;
 const raw=Buffer.alloc((w*3+1)*h);for(let y=0;y<h;y++)pixels.copy(raw,y*(w*3+1)+1,y*w*3,(y+1)*w*3);
 return Buffer.concat([Buffer.from([137,80,78,71,13,10,26,10]),chunk('IHDR',head),chunk('IDAT',zlib.deflateSync(raw)),chunk('IEND',Buffer.alloc(0))]);
}
export async function render(model,dir){
 const views={top:[[1,0,0],[0,1,0],[0,0,1]],side:[[1,0,0],[0,0,1],[0,-1,0]],isometric:[[0.707,-0.707,0],[0.408,0.408,0.816],[-0.577,-0.577,0.577]]},images={};
 for(const [name,axes] of Object.entries(views)){
  const w=800,h=640,pix=Buffer.alloc(w*h*3,242),depth=new Float64Array(w*h).fill(-Infinity),scale=4500,proj=p=>axes.map((a,i)=>p.reduce((s,x,j)=>s+x*a[j],0)*(i<2?scale:1));
  for(const pc of model.pieces)for(const t of pc.triangles){
   const raw=t.face.map(i=>pc.vertices[i]),v=raw.map(p=>{const [x,y,z]=proj(p);return [400+x,320-y,z];}),n=norm(cross(sub(raw[1],raw[0]),sub(raw[2],raw[0]))),light=0.4+0.6*Math.abs(n[0]*0.3+n[1]*0.4+n[2]*0.866);
   const col=model.materials[t.material].rgba.slice(0,3).map(x=>Math.round(255*Math.pow(x,1/2.2)*light));
   const [a,b,c]=v,den=(b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1]);if(Math.abs(den)<1e-9)continue;
   const x0=Math.max(0,Math.floor(Math.min(...v.map(x=>x[0])))),x1=Math.min(w-1,Math.ceil(Math.max(...v.map(x=>x[0])))),y0=Math.max(0,Math.floor(Math.min(...v.map(x=>x[1])))),y1=Math.min(h-1,Math.ceil(Math.max(...v.map(x=>x[1]))));
   for(let y=y0;y<=y1;y++)for(let x=x0;x<=x1;x++){const u=((b[1]-c[1])*(x-c[0])+(c[0]-b[0])*(y-c[1]))/den,vv=((c[1]-a[1])*(x-c[0])+(a[0]-c[0])*(y-c[1]))/den,ww=1-u-vv;if(u<-1e-7||vv<-1e-7||ww<-1e-7)continue;const z=u*a[2]+vv*b[2]+ww*c[2],i=y*w+x;if(z>depth[i]){depth[i]=z;for(let k=0;k<3;k++)pix[3*i+k]=col[k];}}
  }
  const line=(a,b,col)=>{const steps=Math.max(1,Math.ceil(Math.hypot(b[0]-a[0],b[1]-a[1])));for(let i=0;i<=steps;i++){const x=Math.round(a[0]+(b[0]-a[0])*i/steps),y=Math.round(a[1]+(b[1]-a[1])*i/steps);if(x>=0&&x<w&&y>=0&&y<h)for(let k=0;k<3;k++)pix[(y*w+x)*3+k]=col[k];}};
  const screen=p=>{const [x,y]=proj(p);return [400+x,320-y];},{lo,hi}=model.bounds,box=Array.from({length:8},(_,i)=>[i&1?hi[0]:lo[0],i&2?hi[1]:lo[1],i&4?hi[2]:lo[2]]);
  for(let i=0;i<8;i++)for(let k=0;k<3;k++){const j=i^(1<<k);if(j>i)line(screen(box[i]),screen(box[j]),[20,155,195]);}
  for(let k=0;k<3;k++){const p=[0,0,0];p[k]=0.020;line(screen([0,0,0]),screen(p),[[230,50,40],[35,170,40],[45,75,230]][k]);}
  const data=png(w,h,pix);await fs.writeFile(path.join(dir,'render_'+name+'.png'),data);images[name]=data;
 }return images;
}
