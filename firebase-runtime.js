import { initializeApp } from "https://www.gstatic.com/firebasejs/12.19.0/firebase-app.js";
import { getAuth, signInAnonymously } from "https://www.gstatic.com/firebasejs/12.19.0/firebase-auth.js";
import { getFirestore, doc, onSnapshot, setDoc, serverTimestamp } from "https://www.gstatic.com/firebasejs/12.19.0/firebase-firestore.js";

const firebaseConfig = {
  apiKey: "AIzaSyCJTsM3JOuh8tkjGrZPXzR1VVIx8lwANEo",
  authDomain: "robocontrol-snmyl.firebaseapp.com",
  projectId: "robocontrol-snmyl",
  storageBucket: "robocontrol-snmyl.firebasestorage.app",
  messagingSenderId: "753891876454",
  appId: "1:753891876454:web:52f5911a74c6a4057a7"
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
  speedLimit: 100,
  obstacles: [
    { x:180, y:40, w:40, h:160 },
    { x:320, y:200, w:180, h:40 },
    { x:100, y:280, w:160, h:40 },
    { x:430, y:60, w:40, h:100 }
  ]
};

let lastSeq = 0;
let currentCommand = "STOP";
let currentCommandId = "";
let currentCommandAt = 0;
let running = false;
let commandUnsub = null;
let animationFrameId = null;
let lastCommandSeenAt = 0;
const COMMAND_TTL_MS = 1800;
const TELEMETRY_INTERVAL_MS = 5000;
const hardwareDescription = {
  protocol: "1.1",
  type: "wheeled",
  firmware: "virtual-esp32",
  version: "cloud-1.2",
  sensors: [
    { id: "front_distance", type: "ultrasonic", unit: "cm", min: 0, max: 250 },
    { id: "collision", type: "digital", unit: "bool" },
    { id: "battery", type: "battery", unit: "%" }
  ],
  actuators: [
    { id: "motor_l", type: "motor", unit: "%" },
    { id: "motor_r", type: "motor", unit: "%" }
  ],
  controls: ["FORWARD", "BACKWARD", "TURN_LEFT", "TURN_RIGHT", "STOP", "SET_SPEED"],
  map: { width: 600, height: 400, cell: 20 }
};

const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
const rad=d=>d*Math.PI/180;
const normalize=d=>((d%360)+360)%360;

function ackCommand(c, accepted, reason = "") {
  return setDoc(doc(db,"robots",robotId,"state","current"), {
    commandAck: {
      id: String(c?.id || ""),
      seq: Number(c?.seq || 0),
      command: String(c?.command || ""),
      accepted,
      reason,
      at: Date.now()
    }
  }, {merge:true});
}

function applySpeedLimit(value){
  world.speedLimit=clamp(Number(value)||0,0,100);
  world.motorL=clamp(world.motorL,-world.speedLimit,world.speedLimit);
  world.motorR=clamp(world.motorR,-world.speedLimit,world.speedLimit);
  window.dispatchEvent(new CustomEvent("robot-speed",{detail:world.speedLimit}));
}

function safetyWatchdog(nowMs){
  if(currentCommand!=="STOP" && lastCommandSeenAt && nowMs-lastCommandSeenAt>COMMAND_TTL_MS){
    world.motorL=0;
    world.motorR=0;
    currentCommand="STOP";
    currentCommandId="";
    lastCommandSeenAt=0;
    window.dispatchEvent(new CustomEvent("robot-safety",{detail:"command timeout: motors stopped"}));
  }
}

function rayDistance(){
  const r=world.robot,a=rad(r.heading),max=250;
  let best=max;
  const candidates=[];
  if(Math.cos(a)>0)candidates.push((world.width-r.x)/Math.cos(a));
  if(Math.cos(a)<0)candidates.push((0-r.x)/Math.cos(a));
  if(Math.sin(a)>0)candidates.push((world.height-r.y)/Math.sin(a));
  if(Math.sin(a)<0)candidates.push((0-r.y)/Math.sin(a));
  for(const d of candidates)if(d>=0)best=Math.min(best,d);
  for(const o of world.obstacles){
    const dx=Math.cos(a),dy=Math.sin(a);
    const invX=1/(Math.abs(dx)<1e-9?1e-9:dx),invY=1/(Math.abs(dy)<1e-9?1e-9:dy);
    const tx1=(o.x-r.x)*invX,tx2=(o.x+o.w-r.x)*invX;
    const ty1=(o.y-r.y)*invY,ty2=(o.y+o.h-r.y)*invY;
    const tmin=Math.max(Math.min(tx1,tx2),Math.min(ty1,ty2));
    const tmax=Math.min(Math.max(tx1,tx2),Math.max(ty1,ty2));
    if(tmax>=Math.max(0,tmin))best=Math.min(best,tmin>=0?tmin:tmax);
  }
  return clamp(best,0,max);
}

function hits(x,y){
  if(x<8||x>world.width-8||y<8||y>world.height-8)return true;
  return world.obstacles.some(o=>x+8>o.x&&x-8<o.x+o.w&&y+8>o.y&&y-8<o.y+o.h);
}

function applyCommand(command,value){
  const v=clamp(Number(value)||0,-100,100)*(world.speedLimit/100);
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
  const l=world.motorL,r=world.motorR,avg=(l+r)/2,turn=(r-l)*0.75;
  world.robot.heading=normalize(world.robot.heading+turn*dt);
  const distance=avg*0.75*dt,a=rad(world.robot.heading);
  const nx=world.robot.x+Math.cos(a)*distance,ny=world.robot.y+Math.sin(a)*distance;
  world.collision=hits(nx,ny);
  if(!world.collision){world.robot.x=nx;world.robot.y=ny}
  else{
    world.motorL=0;
    world.motorR=0;
    currentCommand="STOP";
    currentCommandId="";
    lastCommandSeenAt=0;
  }
  if(Math.abs(l)+Math.abs(r)>1)world.battery=clamp(world.battery-0.002*dt,0,100);
  world.frontDistance=rayDistance();
  world.path.push({x:world.robot.x,y:world.robot.y});
  if(world.path.length>300)world.path.shift();
}

function buildState(){
  return {
    connected:true,firmware:"virtual-esp32",version:"cloud-1.2",
    hardware:hardwareDescription,
    currentCommand,speed:Math.round((Math.abs(world.motorL)+Math.abs(world.motorR))/2),
    speedLimit:world.speedLimit,battery:world.battery,
    sensors:{front_distance:world.frontDistance,collision:world.collision,battery:world.battery},
    actuators:{motor_l:world.motorL,motor_r:world.motorR},
    pose:{...world.robot},
    world:{...world,robot:{...world.robot},path:world.path.slice(-300)},
    heartbeat:Date.now(),commandId:currentCommandId,
    commandAgeMs:lastCommandSeenAt?Date.now()-lastCommandSeenAt:null,
    updatedAt:serverTimestamp()
  };
}

async function publish(){
  await setDoc(doc(db,"robots",robotId,"state","current"),buildState(),{merge:true});
}

async function sendManualCommand(command, value=0){
  await signInAnonymously(auth);
  const current = await new Promise((resolve, reject) => {
    let done = false;
    let unsubscribe = () => {};
    unsubscribe = onSnapshot(doc(db,"robots",robotId,"control","current"),snap=>{
      if(!done){done=true;unsubscribe();resolve(snap.exists()?snap.data():null)}
    },error=>{
      if(!done){done=true;unsubscribe();reject(error)}
    });
  });
  const seq=Math.max(lastSeq,Number(current?.seq||0))+1;
  const id="manual-"+Date.now()+"-"+seq;
  const normalized=String(command||"STOP").trim().toUpperCase();
  if(!hardwareDescription.controls.includes(normalized)){
    throw new Error("Unsupported manual command: "+normalized);
  }
  const numeric=Number(value);
  const safeValue=Number.isFinite(numeric)?clamp(numeric,-100,100):0;
  const payload={seq,id,command:normalized,value:safeValue,priority:normalized==="STOP"?1000:100,ttl:normalized==="STOP"?5000:1500,issuedAtMs:Date.now(),source:"virtual-esp32-manual",protocol:"1.1"};
  await setDoc(doc(db,"robots",robotId,"control","current"),payload);
  logManual("TX "+payload.command+" "+payload.value);
}
function logManual(message){ window.dispatchEvent(new CustomEvent("robot-command",{detail:{command:message}})); }

async function start(){
  if(running)return;
  running=true;
  try {
    await signInAnonymously(auth);
    // Keep all robot records under the same collection path used by the controller and Firestore rules.
    await setDoc(doc(db,"robots",robotId),{
      id:robotId,name:"Virtual ESP32 Rover",type:"wheeled",
      firmware:"virtual-esp32",protocol:"1.1",
      hardware:hardwareDescription,online:true,updatedAt:serverTimestamp()
    },{merge:true});
    commandUnsub=onSnapshot(doc(db,"robots",robotId,"control","current"),snap=>{
      if(!snap.exists())return;
      const c=snap.data();
      if(Number(c.seq||0)<=lastSeq)return;
      lastSeq=Number(c.seq||0);
      const issuedAt=Number(c.issuedAtMs||0);
      const ttl=clamp(Number(c.ttl||COMMAND_TTL_MS),250,10000);
      if(issuedAt && Date.now()-issuedAt>ttl){
        ackCommand(c,false,"COMMAND_EXPIRED").catch(e=>window.dispatchEvent(new CustomEvent("robot-error",{detail:e})));
        window.dispatchEvent(new CustomEvent("robot-error",{detail:{message:"Command expired"}}));
        return;
      }
      const normalizedCommand=String(c.command||"").trim().toUpperCase();
      if(!hardwareDescription.controls.includes(normalizedCommand)){
        ackCommand(c,false,"UNSUPPORTED_COMMAND").catch(e=>window.dispatchEvent(new CustomEvent("robot-error",{detail:e})));
        window.dispatchEvent(new CustomEvent("robot-error",{detail:{message:"Unsupported command: "+normalizedCommand}}));
        return;
      }
      if(normalizedCommand==="SET_SPEED")applySpeedLimit(c.value);
      else applyCommand(normalizedCommand,c.value);
      currentCommandId=String(c.id||"");
      currentCommandAt=Date.now();
      lastCommandSeenAt=currentCommandAt;
      ackCommand(c,true).catch(e=>window.dispatchEvent(new CustomEvent("robot-error",{detail:e})));
      window.dispatchEvent(new CustomEvent("robot-ack",{detail:{id:String(c.id||""),seq:Number(c.seq||0),command:String(c.command||""),accepted:true,at:Date.now()}}));
      window.dispatchEvent(new CustomEvent("robot-command",{detail:c}));
    },err=>window.dispatchEvent(new CustomEvent("robot-error",{detail:err})));

    let last=performance.now(),lastPublishAt=0;
    const loop=async now=>{
      if(!running)return;
      const dt=Math.min((now-last)/1000,0.1);
      last=now;
      physicsTick(dt);
      const nowMs=Date.now();
      safetyWatchdog(nowMs);
      if(now-lastPublishAt>TELEMETRY_INTERVAL_MS){
        lastPublishAt=now;
        try{await publish()}catch(e){window.dispatchEvent(new CustomEvent("robot-error",{detail:e}))}
      }
      window.dispatchEvent(new CustomEvent("robot-state",{detail:structuredClone(world)}));
      if(running)animationFrameId=requestAnimationFrame(loop);
    };
    animationFrameId=requestAnimationFrame(loop);
  } catch(e) {
    running=false;
    commandUnsub?.();
    commandUnsub=null;
    window.dispatchEvent(new CustomEvent("robot-error",{detail:e}));
  }
}

window.VirtualESP32={world,start,stop(){
  running=false;
  if(animationFrameId!==null)cancelAnimationFrame(animationFrameId);
  animationFrameId=null;
  commandUnsub?.();
  commandUnsub=null;
  world.motorL=0;
  world.motorR=0;
  currentCommand="STOP";
  currentCommandId="";
  lastCommandSeenAt=0;
  publish().catch(e=>window.dispatchEvent(new CustomEvent("robot-error",{detail:e})));
},publish,setSpeedLimit:applySpeedLimit,sendManualCommand};
start().catch(e=>window.dispatchEvent(new CustomEvent("robot-error",{detail:e})));
