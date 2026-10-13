import fs from 'node:fs/promises';
import path from 'node:path';
import {parseXML,children,child,all,bounds,sha} from './geometry.mjs';
export async function validate(out,reportDir){
 const urdfPath=path.join(out,'sanity.urdf'),doc=parseXML(await fs.readFile(urdfPath,'utf8'));
 if(doc.tag!=='robot')throw Error('URDF root');
 const links=children(doc,'link').map(x=>x.a.name),joints=children(doc,'joint'),childNames=joints.map(j=>child(j,'child').a.link);
 if(new Set(links).size!==links.length||new Set(childNames).size!==childNames.length)throw Error('Duplicate link/parent');
 const roots=links.filter(l=>!childNames.includes(l));if(roots.length!==1||roots[0]!=='base_link')throw Error('Root not free base_link');
 for(const j of joints){if(j.a.type!=='fixed'||child(j,'parent').a.link!=='base_link'||!links.includes(child(j,'child').a.link))throw Error('Unexpected topology');}
 for(const link of children(doc,'link')){
  const inertial=child(link,'inertial');if(!inertial)throw Error('Missing link inertial properties');
  const mass=Number(child(inertial,'mass').a.value);
  if(!Number.isFinite(mass)||(link.a.name==='base_link'?mass<=0:mass!==0))throw Error('Mass must be on base_link only');
 }

 const meshes=[],vertices=[],collisionVertices=[];
 for(const mesh of all(doc,'mesh')){
  const ref=mesh.a.filename;if(path.isAbsolute(ref)||ref.includes('://'))throw Error('Nonportable ref');
  const p=path.resolve(path.dirname(urdfPath),ref);if(!p.startsWith(path.resolve(out)+path.sep))throw Error('Asset escapes package');
  const s=await fs.readFile(p,'utf8'),vs=[],normals=[],faces=[];
  for(const line of s.split('\n')){
   const [tag,...parts]=line.trim().split(/\s+/);
   if(tag==='v'){const v=parts.map(Number);if(v.length!==3||!v.every(Number.isFinite))throw Error('Invalid vertex');vs.push(v);}
   if(tag==='vn')normals.push(parts.map(Number));
   if(tag==='f')faces.push(parts.map(p=>p.split('/').map(x=>x===''?null:Number(x))));
   if(tag==='mtllib'){const mp=path.resolve(path.dirname(p),parts.join(' '));if(!mp.startsWith(path.resolve(out)+path.sep))throw Error('Material escapes');await fs.access(mp);const mtl=await fs.readFile(mp,'utf8');if(/^map_/m.test(mtl))throw Error('Unexpected texture');}
  }
  for(const face of faces){if(face.length!==3)throw Error('Not triangles');for(const corner of face){if(!vs[corner[0]-1])throw Error('Invalid index');if(corner[2]&&!normals[corner[2]-1])throw Error('Invalid normal');}}
  (ref.includes('/collision/')?collisionVertices:vertices).push(...vs);
  meshes.push({reference:ref,vertices:vs.length,triangles:faces.length});
 }
 const vb=bounds(vertices),cb=bounds(collisionVertices);for(let k=0;k<3;k++)if(vb.lo[k]<cb.lo[k]-1e-9||vb.hi[k]>cb.hi[k]+1e-9)throw Error('Collision envelope misses visual');
 const expected=JSON.parse(await fs.readFile(path.join(reportDir,'geometry.json'),'utf8'));
 for(let k=0;k<3;k++)if(Math.abs(vb.size[k]-expected.bounds_m.size[k])>1e-8)throw Error('Dimensions changed');
 if(Math.abs(vb.size[0]-0.1119187)>1e-6||Math.abs(vb.size[2]-0.0295)>1e-7)throw Error('Scale error');
 const signs=expected.rotors.map(r=>{const [x,y]=r.position_m;return {rotor:r.id,unit_thrust_N:[0,0,1],moment_about_body_origin_Nm:[y,-x,0],yaw_reaction:'unknown; not tested'};});
 const result={geometry:{status:'PASS custom XML/topology/asset/index/bounds checks',links:links.length,joints:joints.length,world_joint:false,meshes,bounds_m:vb,collision:'conservative AABB; contains every visual vertex; no contact fidelity claim',force_sign_algebra:signs},scope:'Geometry and mass-assignment checks. Native dynamics and inertia checks are in build/validation/contribution_validation.json.',physical:{status:'Physical parameters are in models/sanity.urdf; rotor frames are massless.'},simulation:{status:'Native force-balance, moment-sign and scalar-response checks are in tools/validate_models.py.'},hardware:{status:'Existing bench records included; no hardware access by validation; no flight-policy validation.'},parser_limit:'Custom XML parser here; separate PyBullet native import in contribution_validation.json.'};
 await fs.writeFile(path.join(reportDir,'validation.json'),JSON.stringify(result,null,2)+'\n');return result;
}
