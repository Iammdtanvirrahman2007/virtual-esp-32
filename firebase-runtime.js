import { initializeApp } from "https://www.gstatic.com/firebasejs/12.19.0/firebase-app.js";
import { getAuth, signInAnonymously } from "https://www.gstatic.com/firebasejs/12.19.0/firebase-auth.js";
import { getFirestore, doc, onSnapshot, setDoc, serverTimestamp } from "https://www.gstatic.com/firebasejs/12.19.0/firebase-firestore.js";

const firebaseConfig = {
  apiKey: "AIzaSyCJTsM3JOuh8tkjGrZPXzR1VVIx8lwANEo",
  authDomain: "robocontrol-snmyl.firebaseapp.com",
  projectId: "robocontrol-snmyl",
  storageBucket: "robocontrol-snmyl.firebasestorage.app",
  messagingSenderId: "753891876454",
  appId: "1:753891876454:web:52f5911a74c6a4057a17a7"
};

const app = initializeApp(firebaseConfig);
const auth = getAuth(app);
const db = getFirestore(app);
const robotId = "VESP32-01";

const world = {
  width: 600, height: 400, cell: 20,
  robot: { x: 60, y: 60, heading: 0 },
  battery: 100, motorL: 0, motorR: 0, collision: false,
  frontDistance: 250, path: [],
  obstacles: [
    { x:180, y:40, w:40, h:160 },
    { x:320, y:200, w:180, h:40 },
    { x:100, y:280, w:160, h:40 },
    { x:430, y:60, w:40, h:100 }
  ]
};

let lastSeq = 0;
let currentCommand = "STOP";
let lastPublish = 0;
let running = false;
let commandUnsub = null;
let sketchRunning=false;
let sketchTimer=null;
let sketchLines=[];
let sketchIndex=0;
const virtualPins=new Map();
function serialPrint(value){window.dispatchEvent(new CustomEvent("esp-serial",{detail:String(value)}));}
function codeValue(expr){
  const m=String(expr||"").match(/[-+]?\d+(?:\\.\d+)?/); return m?Number(m[0]):0;
}
function executeSketchLine(line){
  const s=line.trim().replace(/;$/,"");
  if(!s||s.startsWith("//")||s.startsWith("#"))return;
  let m=s.match(/Serial\\.(?:println|print)\((.*)\)$/);
  if(m){serialPrint(m[1].replace(/^["']|["']$/g,""));return}
  m=s.match(/analogWrite\(\s*(\d+)\s*,\s*(-?\d+)\s*\)/);
  if(m){virtualPins.set(Number(m[1]),Number(m[2]));const p=Number(m[1]),v=clamp(Number(m[2]),0,255);if(p===25)world.motorL=v/255*100;if(p===27)world.motorR=v/255*100;return}
  m=s.match(/digitalWrite\(\s*(\d+)\s*,\s*(HIGH|LOW)\s*\)/i);
  if(m){virtualPins.set(Number(m[1]),m[2].toUpperCase()==="HIGH"?1:0);const p=Number(m[1]);if(p===26&&world.motorL!==0)world.motorL=Math.abs(world.motorL)*(m[2].toUpperCase()==="HIGH"?1:-1);if(p===14&&world.motorR!==0)world.motorR=Math.abs(world.motorR)*(m[2].toUpperCase()==="HIGH"?1:-1);return}
  m=s.match(/delay\(\s*(\d+)\s*\)/);if(m){sketchIndex++;sketchTimer=setTimeout(runSketchStep,Math.min(Number(m[1]),2000));return}
}
function runSketchStep(){if(!sketchRunning)return;if(sketchIndex>=sketchLines.length)sketchIndex=0;const line=sketchLines[sketchIndex++];sketchTimer=null;executeSketchLine(line);if(sketchRunning&&!sketchTimer)sketchTimer=setTimeout(runSketchStep,20)}
function runSketch(code){
  stopSketch();
  const cleaned=String(code||"");
  if(!/void\s+setup\s*\(/.test(cleaned)||!/void\s+loop\s*\(/.test(cleaned)){window.dispatchEvent(new CustomEvent("robot-error",{detail:{message:"ESP32 code must contain setup() and loop()"}}));return}
  const loop=cleaned.match(/void\s+loop\s*\(\s*\)\s*\{([\s\\S]*?)\\}/);if(!loop)return;
  sketchLines=loop[1].split(/\r?\n/);sketchIndex=0;sketchRunning=true;window.dispatchEvent(new CustomEvent("esp-serial",{detail:"--- ESP32 sketch started ---"}));runSketchStep();
}
function stopSketch(){sketchRunning=false;if(sketchTimer){clearTimeout(sketchTimer);sketchTimer=null}world.motorL=0;world.motorR=0;currentCommand="STOP";window.dispatchEvent(new CustomEvent("esp-serial",{detail:"--- ESP32 sketch stopped ---"}))}


const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
const rad=d=>d*Math.PI/180;
const normalize=d=>((d%360)+360)%360;

function rayDistance(){
  const r=world.robot, a=rad(r.heading), max=250;
  let best=max;
  const candidates=[];
  if(Math.cos(a)>0) candidates.push((world.width-r.x)/Math.cos(a));
  if(Math.cos(a)<0) candidates.push((0-r.x)/Math.cos(a));
  if(Math.sin(a)>0) candidates.push((world.height-r.y)/Math.sin(a));
  if(Math.sin(a)<0) candidates.push((0-r.y)/Math.sin(a));
  for(const d of candidates) if(d>=0) best=Math.min(best,d);
  for(const o of world.obstacles){
    const dx=Math.cos(a),dy=Math.sin(a);
    const invX=1/(Math.abs(dx)<1e-9?1e-9:dx);
    const invY=1/(Math.abs(dy)<1e-9?1e-9:dy);
    const tx1=(o.x-r.x)*invX,tx2=(o.x+o.w-r.x)*invX;
    const ty1=(o.y-r.y)*invY,ty2=(o.y+o.h-r.y)*invY;
    const tmin=Math.max(Math.min(tx1,tx2),Math.min(ty1,ty2));
    const tmax=Math.min(Math.max(tx1,tx2),Math.max(ty1,ty2));
    if(tmax>=Math.max(0,tmin)) best=Math.min(best,tmin>=0?tmin:tmax);
  }
  return clamp(best,0,max);
}

function hits(x,y){
  if(x<8||x>world.width-8||y<8||y>world.height-8)return true;
  return world.obstacles.some(o=>x+8>o.x&&x-8<o.x+o.w&&y+8>o.y&&y-8<o.y+o.h);
}

function applyCommand(command,value){
  const v=clamp(Number(value)||0,-100,100);
  currentCommand=String(command||"STOP").toUpperCase();
  if(currentCommand==="FORWARD"||currentCommand==="DRIVE_FORWARD"){world.motorL=v;world.motorR=v}
  else if(currentCommand==="BACKWARD"||currentCommand==="DRIVE_BACKWARD"){world.motorL=-Math.abs(v);world.motorR=-Math.abs(v)}
  else if(currentCommand==="TURN_LEFT"){world.motorL=-Math.abs(v);world.motorR=Math.abs(v)}
  else if(currentCommand==="TURN_RIGHT"){world.motorL=Math.abs(v);world.motorR=-Math.abs(v)}
  else if(currentCommand==="SET_MOTOR_L"){world.motorL=v}
  else if(currentCommand==="SET_MOTOR_R"){world.motorR=v}
  else {world.motorL=0;world.motorR=0}
}

function physicsTick(dt){
  const l=world.motorL,r=world.motorR;
  const avg=(l+r)/2;
  const turn=(r-l)*0.75;
  world.robot.heading=normalize(world.robot.heading+turn*dt);
  const distance=avg*0.75*dt;
  const a=rad(world.robot.heading);
  const nx=world.robot.x+Math.cos(a)*distance;
  const ny=world.robot.y+Math.sin(a)*distance;
  world.collision=hits(nx,ny);
  if(!world.collision){world.robot.x=nx;world.robot.y=ny}
  else {world.motorL=0;world.motorR=0;currentCommand="STOP"}
  if(Math.abs(l)+Math.abs(r)>1) world.battery=clamp(world.battery-0.002*dt,0,100);
  world.frontDistance=rayDistance();
  world.path.push({x:world.robot.x,y:world.robot.y});
  if(world.path.length>300)world.path.shift();
}

async function publish(){
  const state={
    connected:true, firmware:"virtual-esp32", version:"cloud-1.0",
    currentCommand, speed:Math.round((Math.abs(world.motorL)+Math.abs(world.motorR))/2),
    battery:world.battery,
    sensors:{front_distance:world.frontDistance,collision:world.collision,battery:world.battery},
    actuators:{motor_l:world.motorL,motor_r:world.motorR},
    pose:{...world.robot},
    world:{...world,robot:{...world.robot},path:world.path.slice(-300)},
    heartbeat:Date.now(), updatedAt:serverTimestamp()
  };
  await setDoc(doc(db,"robots",robotId,"state","current"),state,{merge:true});
}

async function start(){
  if(running)return;
  running=true;
  await signInAnonymously(auth);
  await setDoc(doc(db,"robots",robotId),{
    id:robotId,name:"Virtual ESP32 Rover",type:"wheeled",
    firmware:"virtual-esp32",online:true,updatedAt:serverTimestamp()
  },{merge:true});

  commandUnsub=onSnapshot(doc(db,"robots",robotId,"control","current"),snap=>{
    if(!snap.exists())return;
    const c=snap.data();
    if(Number(c.seq||0)<=lastSeq)return;
    lastSeq=Number(c.seq||0);
    applyCommand(c.command,c.value);
    window.dispatchEvent(new CustomEvent("robot-command",{detail:c}));
  },err=>window.dispatchEvent(new CustomEvent("robot-error",{detail:err})));

  let last=performance.now();
  let lastPublishAt=0;
  const loop=async now=>{
    const dt=Math.min((now-last)/1000,0.1);last=now;
    physicsTick(dt);
    if(now-lastPublishAt>500){lastPublishAt=now;try{await publish()}catch(e){window.dispatchEvent(new CustomEvent("robot-error",{detail:e}))}}
    window.dispatchEvent(new CustomEvent("robot-state",{detail:structuredClone(world)}));
    requestAnimationFrame(loop);
  };
  requestAnimationFrame(loop);
}

window.VirtualESP32={world,start,stop(){commandUnsub?.();running=false},publish,runSketch:runSketch,stopSketch:stopSketch};
start().catch(e=>window.dispatchEvent(new CustomEvent("robot-error",{detail:e})));
