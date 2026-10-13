import fs from 'node:fs/promises';
import path from 'node:path';
import {USDCParser} from './vendor/USDCParser.mjs';
import {loadDAE,I,mul,xyz,sha} from './geometry.mjs';
export function unzipStored(b){
 const entries=[];for(let k=0;k<b.length-46;k++)if(b.readUInt32LE(k)===0x02014b50){
 const n=b.readUInt16LE(k+28),e=b.readUInt16LE(k+30),c=b.readUInt16LE(k+32),o=b.readUInt32LE(k+42),size=b.readUInt32LE(k+20),name=b.subarray(k+46,k+46+n).toString(),method=b.readUInt16LE(k+10);
 if(method!==0)throw Error('Expected stored USDZ');const start=o+30+b.readUInt16LE(o+26)+b.readUInt16LE(o+28);entries.push({name,data:b.subarray(start,start+size)});k+=45+n+e+c;}return entries;
}
export async function audit(out,reportDir){
 const files=await fs.readdir(path.join(out,'sources'));
 const dae=files.find(f=>/\.dae$/i.test(f)),usdz=files.find(f=>/\.usdz$/i.test(f)),stepname=files.find(f=>/\.step$/i.test(f));
 const model=loadDAE(await fs.readFile(path.join(out,'sources',dae),'utf8'));
 const entries=unzipStored(await fs.readFile(path.join(out,'sources',usdz))),parser=new USDCParser(),b=entries[0].data;
 // Explicit wrapper also works across JavaScript VM realms.
 const specs=parser.parseData({buffer:b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength)}).specsByPath;
 const comparisons=[];
 for(const pc of model.pieces){
  const raw=pc.source_vertices,candidates=Object.entries(specs).filter(([p,v])=>p.endsWith('.points')&&v.fields.default.length===raw.length).map(([p,v])=>({path:p,max_error_mesh_units:raw.reduce((m,x,i)=>Math.max(m,Math.abs(x-v.fields.default[i])),0)})).sort((a,b)=>a.max_error_mesh_units-b.max_error_mesh_units);
  if(!candidates.length)throw Error('No matching USD prototype');const best=candidates[0];best.max_error_m=best.max_error_mesh_units*0.01;
  comparisons.push({dae_component:pc.name,...best});
 }
 const step=await fs.readFile(path.join(out,'sources',stepname),'utf8'),se=Object.fromEntries([...step.matchAll(/#(\d+)=([\s\S]*?);/g)].map(m=>[m[1],m[2]]));
 const tuple=s=>s.match(/\(([-+.0-9Ee,]+)\)\)?$/)?.[1].split(',').map(Number);
 const placement=id=>{const refs=[...se[id].matchAll(/#(\d+)/g)].map(m=>m[1]);return {origin_mm:tuple(se[refs[0]]),axis:tuple(se[refs[1]]),reference_axis:tuple(se[refs[2]])};};
 const hubs=[14505,14506,14507,14508].map(id=>({entity:id,...placement(id)}));
 const motors=Object.entries(se).filter(([id,s])=>s.startsWith('CYLINDRICAL_SURFACE')&&s.endsWith(',6.925)')).map(([id,s])=>({entity:id,...placement(s.match(/#(\d+)/)[1])}));
 const transforms=hubs.map((h,i)=>{
 const us=specs['/Sanity/MeshInstance_'+(i+4)+'.xformOp:transform'].fields.default;
 return {instance:i+4,usd_translation_mm:us.slice(12,15),step_translation_mm:h.origin_mm,max_step_usd_translation_error_mm:Math.max(...h.origin_mm.map((x,j)=>Math.abs(x-us[12+j])))};
 });
 const report={usd:{crate_version:parser.version,entries:entries.map(e=>({name:e.name,bytes:e.data.length})),stage:specs['/'].fields,prototype_count:Object.values(specs).filter(v=>v.fields.typeName==='Mesh').length,physics_property_paths:Object.keys(specs).filter(p=>/physics|mass|inertia|density/i.test(p)),comparison_limit:'r183 USDCParser does not decode ReferenceListOp (11 warnings); compare numeric prototype arrays and explicit instance transforms, not general USD stage composition'},comparisons,step:{product:'Sanity_simplified v4',units:'mm',hubs,motors,transforms,densities_kg_m3:{Motors:{material:'Aluminum',value:2700},Propellers:{material:'Steel',value:7850},Frame:{material:'ABS Plastic',value:1060},Duct:{material:'Nylon 12 (Formlabs Fuse 1)',value:1015},Battery:{material:'ABS Plastic',value:1060}},density_status:'CAD assignments not verified as physical; not used for mass/inertia'},findings:{max_usd_dae_vertex_difference_m:Math.max(...comparisons.map(x=>x.max_error_m)),propeller_diagonal_m:0.08,motor_diagonal_m:0.0805,dae_from_step:'Rz(+90 deg) * (0.001 * p_step_mm) + [0,0,0.01] m',body_from_step:'Rz(+90 deg) * (0.001 * p_step_mm) + [0,0,0.0128] m',same_propeller_prototype:true,rotation_direction_not_inferred:true}};
 await fs.writeFile(path.join(reportDir,'source_comparison.json'),JSON.stringify(report,null,2)+'\n');return report;
}
