# -*- coding: utf-8 -*-
"""
report_template.py — Die HTML-Vorlage des Befund-Reports
========================================================
Ein einzelnes HTML mit Three.js-Viewer (laedt pointcloud.ply und das CAD
nach), Zonentabelle, Antast-Funktion per Klick — und der Schleif-
Animation, die UNSER Konzept zeigt: Der Schleifstift steht fest im Raum,
das BAUTEIL wird bewegt (invertierte Kinematik), mit Abheben zwischen
den Strichen. Die __PLATZHALTER__ fuellt ausgaben.build_html().
"""

HTML_TEMPLATE = r"""<!DOCTYPE html>
<html lang="de"><head><meta charset="UTF-8"><title>Bauteil-Befund __PART_ID__</title>
<style>
:root{--bg:#0e1015;--panel:#181b22;--panel2:#1f232c;--text:#e8eaed;--muted:#8b94a5;
--line:#262a33;--pass:#22c55e;--fail:#ef4444;--accent:#3b82f6;--grind:#ff7a18}
*{box-sizing:border-box}html,body{height:100%}
body{margin:0;font-family:'Inter',-apple-system,system-ui,sans-serif;background:var(--bg);color:var(--text);font-size:14px}
header{padding:14px 24px;border-bottom:1px solid var(--line);display:flex;align-items:center;gap:20px;background:var(--panel)}
header h1{font-size:18px;margin:0;font-weight:600}
header .meta{color:var(--muted);font-size:12px;font-variant-numeric:tabular-nums}
.verdict{font-weight:700;font-size:14px;padding:6px 14px;border-radius:5px;letter-spacing:.5px}
.verdict.pass{background:rgba(34,197,94,.15);color:var(--pass)}
.verdict.fail{background:rgba(239,68,68,.15);color:var(--fail)}
main{display:grid;grid-template-columns:1fr 420px;height:calc(100vh - 51px)}
#viewer{position:relative;background:var(--bg)}
.toolbar{position:absolute;top:12px;left:12px;display:flex;gap:5px;z-index:5;flex-wrap:wrap}
.toolbar button{padding:5px 10px;background:rgba(24,27,34,.85);color:var(--muted);border:1px solid var(--line);
border-radius:4px;font-size:11px;cursor:pointer;font-family:inherit}
.toolbar button:hover{color:var(--text);border-color:#3a3f49}
.toolbar button.on{color:var(--grind);border-color:var(--grind)}
.legend{position:absolute;top:12px;right:12px;background:rgba(24,27,34,.85);border:1px solid var(--line);
border-radius:5px;padding:9px 11px;font-size:11px;color:var(--muted);z-index:5;line-height:1.7}
.legend .row{display:flex;align-items:center;gap:7px}
.legend .sw{width:11px;height:11px;border-radius:2px;border:1px solid #2a2e36}
.hint{position:absolute;left:12px;bottom:12px;color:var(--muted);font-size:11px;user-select:none;
background:rgba(0,0,0,.4);padding:6px 10px;border-radius:4px;z-index:5}
.probe{position:absolute;display:none;pointer-events:none;z-index:10;background:rgba(10,12,16,.95);
border:1px solid var(--line);border-radius:5px;padding:8px 11px;font-size:12px;font-variant-numeric:tabular-nums;
box-shadow:0 4px 18px rgba(0,0,0,.5)}
.probe .zn{font-weight:600;margin-bottom:3px}.probe .dv{font-size:15px;font-weight:700}
.err{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;color:#f59e0b;
font-size:13px;text-align:center;padding:40px;z-index:3}
.sidebar{padding:18px 20px;overflow-y:auto;border-left:1px solid var(--line);background:var(--panel)}
.sidebar h2{font-size:11px;text-transform:uppercase;letter-spacing:1px;color:var(--muted);margin:22px 0 10px;font-weight:600}
.sidebar h2:first-child{margin-top:0}
table{width:100%;border-collapse:collapse;font-size:12px}
th{text-align:left;color:var(--muted);font-weight:500;padding:6px 4px;border-bottom:1px solid var(--line)}
td{padding:8px 4px;border-bottom:1px solid #20232b;vertical-align:middle;font-variant-numeric:tabular-nums}
td.num{text-align:right}td.name{font-weight:500}
.dot{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:6px;vertical-align:middle}
.status{display:inline-block;padding:2px 7px;border-radius:3px;font-size:10px;font-weight:600}
.status.pass{background:rgba(34,197,94,.18);color:var(--pass)}
.status.fail{background:rgba(239,68,68,.18);color:var(--fail);cursor:pointer}
.status.empty{background:rgba(139,148,165,.15);color:var(--muted)}
.toggles{display:flex;flex-direction:column;gap:2px}
.toggle{display:flex;align-items:center;gap:8px;padding:5px 6px;border-radius:4px;cursor:pointer;font-size:12px}
.toggle:hover{background:var(--panel2)}.toggle input{accent-color:var(--accent);margin:0}
.sw{width:11px;height:11px;border-radius:2px;border:1px solid #2a2e36}
.controls{display:flex;gap:5px;margin-bottom:8px}
.controls button{padding:4px 9px;background:transparent;color:var(--muted);border:1px solid var(--line);
border-radius:4px;font-size:11px;cursor:pointer;font-family:inherit}.controls button:hover{color:var(--text)}
.quality{font-size:12px;color:var(--muted);line-height:1.7}.quality .num{color:var(--text);font-variant-numeric:tabular-nums}
.q-row{display:flex;justify-content:space-between;padding:2px 0}
.region{font-size:12px;padding:7px 8px;border:1px solid var(--line);border-radius:5px;margin-bottom:6px;cursor:pointer}
.region:hover{border-color:var(--grind)}
.region .rh{display:flex;justify-content:space-between;font-weight:600}
.region .rm{color:var(--muted);font-size:11px;margin-top:2px}
</style></head><body>
<header><h1>Bauteil-Befund</h1>
<div class="verdict __VERDICT_CLASS__">__VERDICT_TEXT__</div>
<div class="meta">__META_LINE__</div></header>
<main>
<div id="viewer">
  <div class="toolbar">
    <button data-view="iso">Iso</button><button data-view="top">Oben</button>
    <button data-view="front">Vorn</button><button data-view="side">Seite</button>
    <button id="btn-grind" class="on">Schleifpfad</button><button id="btn-anim">▶ Schleifen abspielen</button><button id="btn-shot">PNG</button>
  </div>
  <div id="anim-status" style="position:absolute;top:48px;left:12px;z-index:5;display:none;
    background:rgba(24,27,34,.9);border:1px solid var(--grind);border-radius:5px;padding:6px 11px;
    font-size:12px;color:var(--text)"></div>
  <div class="legend">
    <div class="row"><span class="sw" style="background:__CORRECT_COLOR__"></span>Gruen = i.O. (in Toleranz)</div>
    <div class="row">Defekte in Zonenfarbe (siehe Tabelle)</div>
    <div class="row"><span class="sw" style="background:#ff7a18"></span>Schleifpfad</div>
    <div class="row" style="margin-top:3px;color:#5d6677">Punkt anklicken = antasten</div>
  </div>
  <div class="hint">Linksklick: rotieren · Rechtsklick: schieben · Mausrad: zoom · Klick auf Punkt: antasten</div>
  <div class="probe" id="probe"></div>
</div>
<aside class="sidebar">
  <h2>Zonen-Toleranzen</h2>
  <table><thead><tr><th>Zone</th><th class="num">Tol</th><th class="num">Max</th>
  <th class="num">Mittel</th><th class="num">P95</th><th></th></tr></thead><tbody id="zone-rows"></tbody></table>
  <h2>Ansicht</h2>
  <div class="controls"><button id="show-all">Alle</button><button id="show-fail">Nur N.i.O.</button>
  <button id="show-none">Keine</button></div>
  <div id="zone-toggles" class="toggles"></div>
  <h2>Schleifregionen (__N_REGIONS__)</h2>
  <div id="region-list"></div>
  <h2>Scan-Qualitaet</h2>
  <div class="quality">
    <div class="q-row"><span>Punkte (nach Filter)</span><span class="num">__N_TOTAL__</span></div>
    <div class="q-row"><span>Verworfen (&gt;__MAXDIST_MM__mm)</span><span class="num">__N_DROPPED__</span></div>
    <div class="q-row"><span>Merge-Score zum CAD</span><span class="num">__QUALITY_MM__ mm</span></div>
  </div>
</aside></main>
<script type="importmap">
{"imports":{"three":"https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js",
"three/addons/":"https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"}}</script>
<script type="module">
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {PLYLoader} from 'three/addons/loaders/PLYLoader.js';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';

const ZONES=__ZONES__, REGIONS=__REGIONS__, CAD_FILE="__CAD_FILE__", CAD_SCALE=__CAD_SCALE__;
const CAD_FORMAT="__CAD_FORMAT__", POINT_SIZE=__POINT_SIZE__, MESH_OPACITY=__MESH_OPACITY__;
function b64(s,T){const b=atob(s);const u=new Uint8Array(b.length);for(let i=0;i<b.length;i++)u[i]=b.charCodeAt(i);return new T(u.buffer);}
const ZONE_IDX=b64(__ZONE_IDX__,Int32Array), ZONE_DEV=b64(__ZONE_DEV__,Float32Array);

const container=document.getElementById('viewer'), probe=document.getElementById('probe');
function showErr(m){const e=document.createElement('div');e.className='err';e.innerHTML=m;container.appendChild(e);}
const renderer=new THREE.WebGLRenderer({antialias:true,preserveDrawingBuffer:true});
renderer.setPixelRatio(window.devicePixelRatio);renderer.setSize(container.clientWidth,container.clientHeight);
container.appendChild(renderer.domElement);
const scene=new THREE.Scene();scene.background=new THREE.Color(0x0e1015);
const camera=new THREE.PerspectiveCamera(45,container.clientWidth/container.clientHeight,0.0005,5);
camera.position.set(0.15,0.10,0.15);
const controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;controls.dampingFactor=0.12;
scene.add(new THREE.AmbientLight(0xffffff,0.55));
const dl=new THREE.DirectionalLight(0xffffff,0.65);dl.position.set(1,1,1);scene.add(dl);
const dl2=new THREE.DirectionalLight(0xffffff,0.25);dl2.position.set(-1,-0.5,-1);scene.add(dl2);

let bbCenter=new THREE.Vector3(),camDist=0.2;
const partGroup=new THREE.Group();scene.add(partGroup);   // bewegtes Bauteil (Mesh+Wolke+Pfad)
function onCadLoaded(g){
  g.scale(CAD_SCALE,CAD_SCALE,CAD_SCALE);g.computeVertexNormals();
  partGroup.add(new THREE.Mesh(g,new THREE.MeshPhongMaterial({color:0x4a5160,transparent:true,opacity:MESH_OPACITY,side:THREE.DoubleSide,depthWrite:false})));
  g.computeBoundingBox();g.boundingBox.getCenter(bbCenter);
  const s=new THREE.Vector3();g.boundingBox.getSize(s);camDist=Math.max(s.x,s.y,s.z)*2;setView('iso');
}
const onCadErr=()=>showErr("CAD nicht ladbar. Bitte ueber das Python-Skript starten (Server).");
if(CAD_FORMAT==='ply')new PLYLoader().load(CAD_FILE,onCadLoaded,undefined,onCadErr);
else new STLLoader().load(CAD_FILE,onCadLoaded,undefined,onCadErr);

// Eine Punktwolke laden, im JS nach Zone aufteilen (Toggle + Antastung)
const zoneObjects={};
const cloudPts=[];   // {x,y,z, attr, j} je Punkt (CAD-Frame) -> Gruenfaerben beim Schleifen
new PLYLoader().load('pointcloud.ply',(geom)=>{
  const pos=geom.getAttribute('position'), col=geom.getAttribute('color');
  const groups={};
  for(let i=0;i<ZONE_IDX.length;i++){(groups[ZONE_IDX[i]]=groups[ZONE_IDX[i]]||[]).push(i);}
  for(const zi in groups){
    const idxs=groups[zi], p=new Float32Array(idxs.length*3), c=new Float32Array(idxs.length*3), dv=new Float32Array(idxs.length);
    for(let j=0;j<idxs.length;j++){const k=idxs[j];
      p[j*3]=pos.getX(k);p[j*3+1]=pos.getY(k);p[j*3+2]=pos.getZ(k);
      c[j*3]=col.getX(k);c[j*3+1]=col.getY(k);c[j*3+2]=col.getZ(k);dv[j]=ZONE_DEV[k];}
    const g=new THREE.BufferGeometry();
    g.setAttribute('position',new THREE.BufferAttribute(p,3));
    const cAttr=new THREE.BufferAttribute(c,3);
    g.setAttribute('color',cAttr);
    const pts=new THREE.Points(g,new THREE.PointsMaterial({size:POINT_SIZE,vertexColors:true}));
    pts.frustumCulled=false;pts.userData={zoneName:ZONES[zi].name,dev:dv};
    partGroup.add(pts);zoneObjects[ZONES[zi].name]=pts;
    for(let j=0;j<idxs.length;j++)cloudPts.push({x:p[j*3],y:p[j*3+1],z:p[j*3+2],attr:cAttr,j:j});
  }
},undefined,()=>showErr("Punktwolke nicht ladbar. Bitte ueber das Python-Skript starten (Server)."));

// Punkte im Radius um einen (geschliffenen) Wegpunkt gruen faerben
const GR=[0x22/255,0xc5/255,0x5e/255];
function markGround(p,radius){
  const r2=radius*radius, touched=new Set();
  for(const cp of cloudPts){
    const dx=cp.x-p.x,dy=cp.y-p.y,dz=cp.z-p.z;
    if(dx*dx+dy*dy+dz*dz<=r2){cp.attr.setXYZ(cp.j,GR[0],GR[1],GR[2]);touched.add(cp.attr);}
  }
  touched.forEach(a=>a.needsUpdate=true);
}

// Schleifpfade (in partGroup -> bewegen sich mit dem Bauteil)
const grindGroup=new THREE.Group();partGroup.add(grindGroup);
const wpSpheres=[];   // Wegpunkt-Kugeln in Abfahrreihenfolge (fuer Fortschrittsfaerbung)
REGIONS.forEach(r=>{
  const pts=r.waypoints.map(w=>new THREE.Vector3(w.xyz[0]*0.001,w.xyz[1]*0.001,w.xyz[2]*0.001));
  if(pts.length>1){const lg=new THREE.BufferGeometry().setFromPoints(pts);
    grindGroup.add(new THREE.Line(lg,new THREE.LineBasicMaterial({color:0xff7a18})));}
  pts.forEach(p=>{const s=new THREE.Mesh(new THREE.SphereGeometry(0.0005,8,8),new THREE.MeshBasicMaterial({color:0xff7a18}));s.position.copy(p);grindGroup.add(s);wpSpheres.push(s);});
});

// Marker fuer schlimmste Punkte (in partGroup -> bewegen mit dem Bauteil)
const markers=[];
function clearMarkers(){markers.forEach(m=>partGroup.remove(m));markers.length=0;}
function addMarker(xyz,color){const m=new THREE.Mesh(new THREE.SphereGeometry(0.0008,16,16),new THREE.MeshBasicMaterial({color}));m.position.set(xyz[0],xyz[1],xyz[2]);partGroup.add(m);markers.push(m);}
ZONES.forEach(z=>{if(z.pass===false&&z.worst_xyz)addMarker(z.worst_xyz,0xffffff);});

// Tabelle + Toggles
const rowsEl=document.getElementById('zone-rows'),togglesEl=document.getElementById('zone-toggles');
const fmt=v=>v===null?'-':v.toFixed(2);
ZONES.forEach((z,i)=>{
  const tr=document.createElement('tr');let sc='empty',st='-';
  if(z.pass===true){sc='pass';st='i.O.';}if(z.pass===false){sc='fail';st='N.i.O.';}
  tr.innerHTML=`<td class="name"><span class="dot" style="background:${z.color}"></span>${z.name}</td>
  <td class="num">${z.tolerance_mm.toFixed(2)}</td><td class="num">${fmt(z.max_mm)}</td>
  <td class="num">${fmt(z.mean_mm)}</td><td class="num">${fmt(z.p95_mm)}</td>
  <td><span class="status ${sc}" data-fz="${z.pass===false?i:''}">${st}</span></td>`;
  rowsEl.appendChild(tr);
  const lbl=document.createElement('label');lbl.className='toggle';
  lbl.innerHTML=`<input type="checkbox" checked data-zone="${z.name}"><span class="sw" style="background:${z.color}"></span>
  <span>${z.name}</span><span style="color:var(--muted);margin-left:auto;font-size:11px">${z.n_points||0}</span>`;
  togglesEl.appendChild(lbl);
});
rowsEl.addEventListener('click',e=>{const s=e.target.closest('.status.fail');if(!s||!s.dataset.fz)return;
  const z=ZONES[+s.dataset.fz];if(z&&z.worst_xyz)flyTo(z.worst_xyz);});
togglesEl.addEventListener('change',e=>{const cb=e.target;if(!cb.dataset||!cb.dataset.zone)return;
  const o=zoneObjects[cb.dataset.zone];if(o)o.visible=cb.checked;});
function setAll(f){togglesEl.querySelectorAll('input').forEach(c=>{const v=f(c.dataset.zone);c.checked=v;const o=zoneObjects[c.dataset.zone];if(o)o.visible=v;});}
document.getElementById('show-all').onclick=()=>setAll(()=>true);
document.getElementById('show-none').onclick=()=>setAll(()=>false);
document.getElementById('show-fail').onclick=()=>setAll(n=>{const z=ZONES.find(zz=>zz.name===n);return z&&z.pass===false;});

// Region-Liste
const rl=document.getElementById('region-list');
if(REGIONS.length===0){rl.innerHTML='<div style="color:var(--muted);font-size:12px">keine</div>';}
REGIONS.forEach(r=>{
  const d=document.createElement('div');d.className='region';
  d.innerHTML=`<div class="rh"><span>#${r.id} ${r.zone}</span><span style="color:var(--grind)">${r.type}</span></div>
  <div class="rm">${r.waypoints.length} Wegpunkte · max Abtrag ${r.max_removal_mm.toFixed(2)}mm</div>`;
  d.onclick=()=>{if(r.waypoints.length){const w=r.waypoints[Math.floor(r.waypoints.length/2)];flyTo([w.xyz[0]*0.001,w.xyz[1]*0.001,w.xyz[2]*0.001]);}};
  rl.appendChild(d);
});

// Ansichten / Screenshot / Schleifpfad-Toggle
function setView(v){const c=bbCenter,d=camDist;
  const pos={iso:[c.x+d,c.y+d*0.55,c.z+d],top:[c.x,c.y+d*1.6,c.z+0.0001],front:[c.x,c.y,c.z+d*1.6],side:[c.x+d*1.6,c.y,c.z]}[v];
  camera.position.set(pos[0],pos[1],pos[2]);controls.target.copy(c);controls.update();}
document.querySelectorAll('.toolbar [data-view]').forEach(b=>b.onclick=()=>setView(b.dataset.view));
document.getElementById('btn-shot').onclick=()=>{renderer.render(scene,camera);
  const a=document.createElement('a');a.href=renderer.domElement.toDataURL('image/png');a.download='befund_'+('__PART_ID__'||'teil')+'.png';a.click();};
const bg=document.getElementById('btn-grind');
bg.onclick=()=>{grindGroup.visible=!grindGroup.visible;bg.classList.toggle('on',grindGroup.visible);};
function flyTo(xyz){const p=new THREE.Vector3(xyz[0],xyz[1],xyz[2]);
  const dir=camera.position.clone().sub(controls.target).normalize();
  controls.target.copy(p);camera.position.copy(p.clone().add(dir.multiplyScalar(0.03)));controls.update();clearMarkers();addMarker(xyz,0xffff00);}

// Antastung
const ray=new THREE.Raycaster();ray.params.Points.threshold=0.0007;const mouse=new THREE.Vector2();
let downXY=null,dragging=false;
renderer.domElement.addEventListener('pointerdown',e=>{downXY=[e.clientX,e.clientY];dragging=false;});
renderer.domElement.addEventListener('pointermove',e=>{if(downXY&&(Math.abs(e.clientX-downXY[0])+Math.abs(e.clientY-downXY[1]))>4)dragging=true;});
renderer.domElement.addEventListener('pointerup',e=>{if(dragging){downXY=null;return;}downXY=null;
  const r=renderer.domElement.getBoundingClientRect();
  mouse.x=((e.clientX-r.left)/r.width)*2-1;mouse.y=-((e.clientY-r.top)/r.height)*2+1;
  ray.setFromCamera(mouse,camera);
  const hits=ray.intersectObjects(Object.values(zoneObjects).filter(o=>o.visible),false);
  if(!hits.length){probe.style.display='none';return;}
  const h=hits[0],zn=h.object.userData.zoneName,dv=h.object.userData.dev[h.index];
  const z=ZONES.find(x=>x.name===zn),bad=Math.abs(dv)>z.tolerance_mm;
  probe.innerHTML=`<div class="zn"><span class="dot" style="background:${z.color}"></span>${zn}</div>
  <div class="dv" style="color:${bad?'#ef4444':'#22c55e'}">${dv>=0?'+':''}${dv.toFixed(3)} mm</div>
  <div style="color:var(--muted);font-size:11px">Tol ${z.tolerance_mm.toFixed(2)} mm · ${bad?'ausser Tol.':'i.O.'}</div>`;
  probe.style.left=Math.min(e.clientX+12,window.innerWidth-180)+'px';probe.style.top=(e.clientY+12)+'px';probe.style.display='block';});
renderer.domElement.addEventListener('dblclick',()=>probe.style.display='none');

// === Schleif-Animation: feste Spitze, Bauteil bewegt sich (wie real) ===
// Industrieller Schleifstift fest im Raum. Pro Wegpunkt wird das Bauteil so
// gedreht/verschoben, dass der Punkt an der Spitze liegt und seine Flaechen-
// normale zur Spitze zeigt. Zwischen Strichen wird ABGEHOBEN, umorientiert und
// wieder ANGEFAHREN, damit die Spitze nie quer durchs Bauteil faehrt.
const TIP=new THREE.Vector3(0,0,0);          // feste Spitzenposition (Weltkoordinaten)
const NW=new THREE.Vector3(0,1,0);           // Kontaktnormale zeigt nach oben zur Spitze
const RETRACT=0.022;                         // 22mm Abheben zwischen Strichen
let tipObj=null,bitObj=null;
function buildTip(){
  tipObj=new THREE.Group();
  const metal=new THREE.MeshStandardMaterial({color:0xc2c6ce,metalness:0.95,roughness:0.3});
  const dark =new THREE.MeshStandardMaterial({color:0x21252b,metalness:0.5,roughness:0.55});
  const abr  =new THREE.MeshStandardMaterial({color:0x6b7280,metalness:0.15,roughness:0.95});
  const accent=new THREE.MeshStandardMaterial({color:0xff7a18,metalness:0.3,roughness:0.5});
  // rotierende Schleifspitze (gerundeter Stift), Kontakt bei y=0 an TIP
  bitObj=new THREE.Group();
  const bit=new THREE.Mesh(new THREE.CylinderGeometry(0.0019,0.0015,0.010,20),abr);bit.position.y=0.005;
  const cap=new THREE.Mesh(new THREE.SphereGeometry(0.0015,16,12),abr);
  bitObj.add(bit);bitObj.add(cap);tipObj.add(bitObj);
  // Spannzange (metallisch, konisch)
  const collet=new THREE.Mesh(new THREE.CylinderGeometry(0.0036,0.0021,0.011,20),metal);collet.position.y=0.0155;tipObj.add(collet);
  // Motorkoerper + Akzentring + Endkappe (industrieller Handschleifer)
  const body=new THREE.Mesh(new THREE.CylinderGeometry(0.0062,0.0062,0.046,28),dark);body.position.y=0.044;tipObj.add(body);
  const ring=new THREE.Mesh(new THREE.CylinderGeometry(0.0064,0.0064,0.004,28),accent);ring.position.y=0.03;tipObj.add(ring);
  const endc=new THREE.Mesh(new THREE.CylinderGeometry(0.0048,0.0062,0.007,28),dark);endc.position.y=0.069;tipObj.add(endc);
  tipObj.position.copy(TIP);tipObj.visible=false;scene.add(tipObj);
}
buildTip();

// Pose des Bauteils, die Wegpunkt p an die Spitze bringt und Normale -> NW dreht
function poseFor(p,n){
  const q=new THREE.Quaternion().setFromUnitVectors(n,NW);
  const pos=TIP.clone().sub(p.clone().applyQuaternion(q));   // TIP - R*p
  return {q,pos};
}
function liftedOf(pp){return {q:pp.q.clone(),pos:pp.pos.clone().add(NW.clone().multiplyScalar(-RETRACT))};}
function resetPart(){partGroup.quaternion.identity();partGroup.position.set(0,0,0);partGroup.updateMatrixWorld(true);}

// Bewegungssequenz: pro Region  [abgehoben-an] -> Kontakte... -> [abgehoben-ab]
// Kontakt-Posen schleifen (faerben Punkte gruen); abgehobene Posen sind Leerfahrt.
const MOTION=[];
REGIONS.forEach((r,ri)=>{
  const wps=r.waypoints.map(w=>({p:new THREE.Vector3(w.xyz[0]*0.001,w.xyz[1]*0.001,w.xyz[2]*0.001),
                                 n:new THREE.Vector3(w.normal[0],w.normal[1],w.normal[2]).normalize()}));
  if(!wps.length)return;
  const first=poseFor(wps[0].p,wps[0].n);
  MOTION.push({...liftedOf(first),contact:false,ri,zone:r.zone,wi:0,nw:wps.length});
  wps.forEach((wp,wi)=>{const cp=poseFor(wp.p,wp.n);
    MOTION.push({q:cp.q,pos:cp.pos,contact:true,cpos:wp.p,ri,zone:r.zone,wi,nw:wps.length});});
  const last=poseFor(wps[wps.length-1].p,wps[wps.length-1].n);
  MOTION.push({...liftedOf(last),contact:false,ri,zone:r.zone,wi:wps.length-1,nw:wps.length});
});

let anim={on:false,i:0,t:0,from:null};
const SEG_CONTACT=0.45, SEG_TRAVEL=0.7;      // s pro Segment (Schleifen langsamer, Leerfahrt zuegig)
const statusEl=document.getElementById('anim-status');
const animBtn=document.getElementById('btn-anim');

function animStart(){
  if(MOTION.length===0){statusEl.textContent='Keine Schleifregionen';statusEl.style.display='block';
    setTimeout(()=>statusEl.style.display='none',1500);return;}
  anim.on=true;anim.i=0;anim.t=0;anim.from={q:partGroup.quaternion.clone(),pos:partGroup.position.clone()};
  tipObj.visible=true;grindGroup.visible=true;
  wpSpheres.forEach(s=>s.material.color.set(0xff7a18));
  animBtn.textContent='⏸ Pause';animBtn.classList.add('on');
  controls.target.copy(new THREE.Vector3(TIP.x,TIP.y-0.025,TIP.z));
  camera.position.set(TIP.x+camDist*1.1,TIP.y+camDist*0.45,TIP.z+camDist*1.1);controls.update();
}
function animStop(){
  anim.on=false;tipObj.visible=false;resetPart();
  animBtn.textContent='▶ Schleifen abspielen';animBtn.classList.remove('on');
  statusEl.style.display='none';setView('iso');
}
animBtn.onclick=()=>{ anim.on?animStop():animStart(); };

function animTick(dt){
  if(tipObj.visible)bitObj.rotation.y+=dt*40;     // Schleifspitze dreht sichtbar
  if(!anim.on||MOTION.length===0)return;
  const m=MOTION[anim.i];
  const seg=m.contact?SEG_CONTACT:SEG_TRAVEL;
  anim.t+=dt/seg;const u=Math.min(anim.t,1);
  partGroup.quaternion.copy(anim.from.q.clone().slerp(m.q,u));
  partGroup.position.copy(anim.from.pos.clone().lerp(m.pos,u));
  partGroup.updateMatrixWorld(true);
  statusEl.style.display='block';
  statusEl.innerHTML=`Region ${m.ri+1}/${REGIONS.length} · <b>${m.zone}</b> · `+
    (m.contact?`schleift Wegpunkt ${m.wi+1}/${m.nw}`:`positioniert ...`);
  if(u>=1){
    if(m.contact){markGround(m.cpos,0.0028);}          // Punkte gruen wo geschliffen
    anim.from={q:partGroup.quaternion.clone(),pos:partGroup.position.clone()};
    anim.i++;anim.t=0;
    if(anim.i>=MOTION.length){
      statusEl.innerHTML='✓ Fertig — alle Stellen geschliffen';
      anim.on=false;animBtn.textContent='▶ Schleifen abspielen';animBtn.classList.remove('on');
      setTimeout(()=>{if(!anim.on){tipObj.visible=false;resetPart();statusEl.style.display='none';setView('iso');}},2800);
    }
  }
}

let _last=performance.now();
(function animate(now){requestAnimationFrame(animate);
  const dt=Math.min((now-_last)/1000,0.05);_last=now;
  animTick(dt);controls.update();renderer.render(scene,camera);})(_last);
addEventListener('resize',()=>{camera.aspect=container.clientWidth/container.clientHeight;
  camera.updateProjectionMatrix();renderer.setSize(container.clientWidth,container.clientHeight);});
</script></body></html>
"""

