import * as THREE from 'three';
import {OrbitControls} from './vendor/OrbitControls.js';

const $ = id => document.getElementById(id);
const host = $('viewport');
let renderer;
try { renderer = new THREE.WebGLRenderer({antialias:true}); }
catch (error) { $('error').textContent='浏览器无法创建 WebGL，请启用硬件加速或换用支持 WebGL 的浏览器。'; throw error; }
renderer.setPixelRatio(Math.min(devicePixelRatio,2));
renderer.outputColorSpace=THREE.SRGBColorSpace;
host.prepend(renderer.domElement);
const scene=new THREE.Scene(); scene.background=new THREE.Color('#edf1f4');
const camera=new THREE.PerspectiveCamera(42,1,.01,200); camera.up.set(0,0,1);
const controls=new OrbitControls(camera,renderer.domElement); controls.enableDamping=true;
const target=new THREE.Vector3(-.6,.5,.9);
function view(name){
  camera.up.set(0,0,1);
  const positions={orbit:[6,-8,5.8],front:[-.6,-9,2.4],top:[-.6,.5,11]};
  camera.position.fromArray(positions[name]);
  if(name==='top')camera.up.set(0,1,0);
  controls.target.copy(target);controls.update();
}
view('orbit');
document.querySelectorAll('[data-view]').forEach(button=>button.onclick=()=>view(button.dataset.view));
scene.add(new THREE.HemisphereLight(0xffffff,0x657582,2.5));
const light=new THREE.DirectionalLight(0xffffff,3);light.position.set(3,-4,8);scene.add(light);
const grid=new THREE.GridHelper(10,50,0x8fa3af,0xd5dee3);grid.rotation.x=Math.PI/2;grid.position.z=-.008;scene.add(grid);
scene.add(new THREE.AxesHelper(.5));
const toolArrow=new THREE.ArrowHelper(new THREE.Vector3(0,0,-1),new THREE.Vector3(),.35,0x197ac2,.07,.035);toolArrow.visible=false;scene.add(toolArrow);
const lengthArrow=new THREE.ArrowHelper(new THREE.Vector3(1,0,0),new THREE.Vector3(),.38,0xe34a42,.06,.025);
const widthArrow=new THREE.ArrowHelper(new THREE.Vector3(0,1,0),new THREE.Vector3(),.30,0x26a65b,.06,.025);
scene.add(lengthArrow,widthArrow);lengthArrow.visible=widthArrow.visible=false;
let model=null,bodies=[],shells=[],sceneRun=null,frozen=false,lastFrame=-1,metadata=null;
const traceGeometry=new THREE.BufferGeometry();
const traceArray=new Float32Array(30000*3);traceGeometry.setAttribute('position',new THREE.BufferAttribute(traceArray,3));traceGeometry.setDrawRange(0,0);
const trace=new THREE.Line(traceGeometry,new THREE.LineBasicMaterial({color:0xd08b35}));trace.frustumCulled=false;scene.add(trace);
let traceCount=0,lastPoint=null;
function disposeModel(){
  if(model){scene.remove(model);model.traverse(node=>{node.geometry?.dispose();node.material?.dispose();});}
  toolArrow.visible=lengthArrow.visible=widthArrow.visible=false;
  model=null;bodies=[];shells=[];traceCount=0;lastPoint=null;lastFrame=-1;traceGeometry.setDrawRange(0,0);
}
function shellOpacity(){for(const mesh of shells){mesh.material.transparent=$('shell').checked;mesh.material.opacity=$('shell').checked?.18:1;mesh.material.depthWrite=!$('shell').checked;}}
$('shell').onchange=shellOpacity;$('trace').onchange=()=>trace.visible=$('trace').checked;
async function loadScene(run){
  const [metaResponse,binResponse]=await Promise.all([fetch('/data/scene.json'),fetch('/data/geometry.bin')]);
  if(!metaResponse.ok||!binResponse.ok)throw Error('模型仍在初始化');
  const meta=await metaResponse.json(),buffer=await binResponse.arrayBuffer();
  disposeModel();model=new THREE.Group();metadata=meta;
  bodies=meta.bodies.map((name,i)=>{const group=new THREE.Group();group.name=name;model.add(group);return group;});
  for(const shape of meta.shapes){
    const geometry=new THREE.BufferGeometry();
    geometry.setAttribute('position',new THREE.BufferAttribute(new Float32Array(buffer,shape.vertex_offset,shape.vertex_count),3));
    geometry.setIndex(new THREE.BufferAttribute(new Uint32Array(buffer,shape.index_offset,shape.index_count),1));geometry.computeVertexNormals();
    const color=new THREE.Color().setRGB(...shape.color);
    const mesh=new THREE.Mesh(geometry,new THREE.MeshStandardMaterial({color,metalness:.12,roughness:.65,flatShading:shape.name.endsWith("_collision_box"),side:THREE.DoubleSide}));
    mesh.position.fromArray(shape.pose);mesh.quaternion.fromArray(shape.pose,3);
    mesh.name=shape.name;(shape.body<0?model:bodies[shape.body]).add(mesh);
    if(shape.name.includes('container'))shells.push(mesh);
  }
  scene.add(model);shellOpacity();sceneRun=run;
}
function applyState(state){
  if(state.box_pose){
    const visible=Math.max(...state.box_pose.slice(0,3).map(Math.abs))<30;
    for(const [arrow,direction] of [[lengthArrow,state.box_length_axis],[widthArrow,state.box_width_axis]]){arrow.visible=visible;arrow.position.fromArray(state.box_pose);arrow.setDirection(new THREE.Vector3().fromArray(direction).normalize());}
  }
  if(metadata&&state.tcp_normal_world){toolArrow.visible=true;toolArrow.position.fromArray(state.body_q[metadata.tcp]);toolArrow.setDirection(new THREE.Vector3().fromArray(state.tcp_normal_world).normalize());}
  for(let i=0;i<bodies.length;i++){
    const pose=state.body_q[i];if(!pose)continue;
    bodies[i].position.fromArray(pose);bodies[i].quaternion.fromArray(pose,3);
    bodies[i].visible=Math.max(...pose.slice(0,3).map(Math.abs))<30;
  }
  if(state.frame!==lastFrame&&metadata){
    const p=state.body_q[metadata.tcp];
    if(p&&traceCount<30000&&(!lastPoint||Math.hypot(...p.slice(0,3).map((v,i)=>v-lastPoint[i]))>.002)){
      traceArray.set(p.slice(0,3),traceCount++*3);lastPoint=p.slice(0,3);
      traceGeometry.attributes.position.needsUpdate=true;traceGeometry.setDrawRange(0,traceCount);
    }
    lastFrame=state.frame;
  }
}
$('freeze').onclick=()=>{frozen=!frozen;$('freeze').textContent=frozen?'恢复实时画面':'冻结画面';$('badge').textContent=frozen?'画面已冻结 · 仿真继续运行':'世界坐标 · Z 向上 · 米';};
async function action(path){const r=await fetch(path,{method:'POST',headers:{'X-Viewer-Request':'1'}});if(!r.ok)throw Error('操作失败：'+r.status);return r.json();}
$('mode').onchange=()=>{$('start').textContent=({'stacked-four':'开始四箱翻转仿真','stacked-width':'开始堆叠插入仿真','flip-horizontal':'开始翻转仿真','upright':'开始四箱仿真'})[$('mode').value];};
$('start').onclick=async()=>{try{$('start').disabled=true;const started=await action(({'stacked-four':'/api/start-stack-four','stacked-width':'/api/start-stack','flip-horizontal':'/api/start-flip','upright':'/api/start'})[$('mode').value]);if(started.started){disposeModel();sceneRun=null;$('count').textContent='0';$('stage').textContent='初始化中';$('joints').textContent='等待仿真状态';}$('error').textContent='';frozen=false;$('freeze').textContent='冻结画面';$('badge').textContent='世界坐标 · Z 向上 · 米';}catch(e){$('error').textContent=e.message;$('start').disabled=false;}};
$('stop').onclick=async()=>{try{await action('/api/stop');$('message').textContent='正在停止仿真并保存记录…';}catch(e){$('error').textContent=e.message;}};
async function poll(){
  try{
    const response=await fetch('/api/status');if(!response.ok)throw Error('服务响应异常');
    const status=await response.json(),s=status.state;
    $('start').disabled=status.active;$('mode').disabled=status.active;$('stop').disabled=!status.active;$('freeze').disabled=!s;
    $('run').textContent=status.run||'—';
    if(status.run&&sceneRun!==status.run&&s){await loadScene(status.run);}
    $('status').textContent=status.active?(s?'实时运行':'初始化中'):s?.completed?'仿真完成':status.run?'仿真已停止':'等待启动';
    if(s){
      $('total').textContent=s.total;$('tilt').textContent=`${(s.tcp_horizontal_error_deg??0).toFixed(2)}°（水平为 0°）`;$('count').textContent=s.completed_cycles;$('stage').textContent=s.stage;
      $('boxAxes').textContent=`长边离水平 ${(s.length_horizontal_error_deg??0).toFixed(2)}° · 宽边离竖直 ${(s.width_vertical_error_deg??0).toFixed(2)}°`;
      $('joints').textContent=s.q.map((q,i)=>`J${i+1}   ${(q*180/Math.PI).toFixed(2).padStart(8)}°`).join('\n');
      if(!frozen&&sceneRun===status.run)applyState(s);
      const age=(Date.now()/1000-s.timestamp).toFixed(1);
      $('message').textContent=status.active?`第 ${s.cycle||1} 箱 · 状态更新于 ${age} 秒前。规划时模型可能保持不动。`:s.completed?'本轮装箱完成。可自由查看最终布局，或重新开始。':'仿真停止，保留最后状态与运行记录。';
      if(s.error)$('error').textContent=s.error;
    }else if(status.active){$('message').textContent='正在加载模型与规划器，首次启动请稍候…';}
    if(!status.active&&status.run&&(!s||status.exit_code))$('error').textContent=status.log_tail||'仿真未输出状态，请检查运行日志。';
  }catch(error){$('status').textContent='连接等待';$('message').textContent=error.message;}
  setTimeout(poll,50);
}
poll();
new ResizeObserver(()=>{const w=host.clientWidth,h=host.clientHeight;renderer.setSize(w,h);camera.aspect=w/h;camera.updateProjectionMatrix();}).observe(host);
function animate(){requestAnimationFrame(animate);controls.update();renderer.render(scene,camera);}animate();
