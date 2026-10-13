import fs from 'node:fs/promises';
import path from 'node:path';
import {loadDAE,placeRotors,parseXML,all,child,fmt,render} from './geometry.mjs';
export async function roundtrip(out,reportDir){
 const urdf=parseXML(await fs.readFile(path.join(out,'sanity.urdf'),'utf8'));
 const model=placeRotors(loadDAE(await fs.readFile(path.join(out,'sources/sanity.dae'),'utf8')));
 const actual={pieces:[],materials:{},bounds:model.bounds};
 const results=[];
 for(const visual of all(urdf,'visual')){
 const mesh=all(visual,'mesh')[0],material=child(visual,'material'),name=material.a.name,rgba=child(material,'color').a.rgba.split(/\s+/).map(Number);
 const data=await fs.readFile(path.resolve(out,mesh.a.filename),'utf8'),v=[],n=[],tri=[];
 for(const line of data.split('\n')){const [tag,...args]=line.trim().split(/\s+/);if(tag==='v')v.push(args.map(Number));else if(tag==='vn')n.push(args.map(Number));else if(tag==='f'){const f=args.map(x=>x.split('/').map(Number));tri.push({face:f.map(x=>x[0]-1),normals:f.map(x=>n[x[2]-1]),material:name});}}
 const materialID=Object.entries(model.materials).find(([id,m])=>m.name===name)[0];
 const expected=[];
 for(const p of model.pieces)for(const t of p.triangles.filter(x=>x.material===materialID))expected.push(t.face.map((i,j)=>[...p.vertices[i],...t.normals[j]].map(fmt).join(',')).join(';'));
 const observed=tri.map(t=>t.face.map((i,j)=>[...v[i],...t.normals[j]].map(fmt).join(',')).join(';'));
 if(expected.length!==observed.length||expected.some((s,i)=>s!==observed[i]))throw Error('Triangle/normal roundtrip differs: '+name);
 results.push({material:name,triangle_count:tri.length,ordered_triangles_and_normals_identical_at_1e_10_serialization:true});
 actual.pieces.push({name,vertices:v,triangles:tri});actual.materials[name]={rgba};
 }
 await render(actual,reportDir);await fs.writeFile(path.join(reportDir,'roundtrip.json'),JSON.stringify(results,null,2)+'\n');return results;
}
