// The web page served by Web.ino: plain HTML, CSS and JavaScript in one
// string, edited here directly (no build step). It needs no internet: a phone
// on the "TireBalancer" hotspot has none.
#pragma once

static const char WEB_PAGE[] PROGMEM = R"HTML(<!doctype html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>Tire Balancer</title>
<style>
:root{--bg:#f3f4f6;--card:#fff;--text:#111827;--muted:#6b7280;--line:#d1d5db;--accent:#2563eb;--accent-text:#fff;
--ok:#15803d;--bad:#b91c1c;--warn-bg:#fef3c7;--warn-line:#d97706;--safe-bg:#dcfce7;--safe-line:#16a34a;--btn:#e5e7eb;--con:#0b1020;--con-text:#d1d5db}
@media (prefers-color-scheme:dark){:root{--bg:#0f1115;--card:#1a1d24;--text:#e5e7eb;--muted:#9ca3af;--line:#374151;--accent:#3b82f6;
--ok:#22c55e;--bad:#ef4444;--warn-bg:#3a2a07;--warn-line:#d97706;--safe-bg:#0f2e1a;--safe-line:#16a34a;--btn:#2a2f3a;--con:#05070d}}
*{box-sizing:border-box}
html,body{margin:0;background:var(--bg);color:var(--text);font:16px/1.4 system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
header{position:sticky;top:0;z-index:5;background:var(--card);border-bottom:1px solid var(--line)}
.bar{display:flex;align-items:center;gap:10px;padding:8px 16px;max-width:760px;margin:0 auto}
.bar h1{font-size:17px;margin:0;flex:1}
.pill{font-size:13px;padding:3px 10px;border-radius:99px;background:var(--btn);white-space:nowrap}
.pill.up{background:var(--ok);color:#fff}.pill.down{background:var(--bad);color:#fff}
#stop{background:var(--bad);color:#fff;font-weight:700;border:0;border-radius:10px;padding:10px 18px;font-size:16px}
.ack{display:flex;align-items:center;gap:10px;padding:8px 16px;max-width:760px;margin:0 auto;border-top:1px solid var(--line)}
.ack label{display:flex;align-items:center;gap:10px;flex:1;font-weight:600;cursor:pointer}
.ack input{width:26px;height:26px;flex:none}
#ackWrap{background:var(--warn-bg);border-left:6px solid var(--warn-line)}
#ackWrap.on{background:var(--safe-bg);border-left-color:var(--safe-line)}
#ackWrap small{color:var(--muted);font-weight:400;display:block}
nav{display:flex;overflow-x:auto;max-width:760px;margin:0 auto;padding:0 8px}
nav button{flex:1;background:none;border:0;border-bottom:3px solid transparent;color:var(--muted);padding:10px 8px;font-size:15px;white-space:nowrap}
nav button.sel{color:var(--accent);border-bottom-color:var(--accent);font-weight:600}
main{max-width:760px;margin:0 auto;padding:12px 16px 40px}
section{display:none}section.sel{display:block}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px;margin-bottom:12px}
.card h2{font-size:15px;margin:0 0 10px;color:var(--muted);font-weight:600;text-transform:uppercase;letter-spacing:.04em}
button{font:inherit;cursor:pointer}
.btn{background:var(--btn);color:var(--text);border:1px solid var(--line);border-radius:10px;padding:12px 14px;min-height:48px}
.btn.primary{background:var(--accent);color:var(--accent-text);border-color:var(--accent);font-weight:600}
.btn.danger{background:var(--bad);color:#fff;border-color:var(--bad)}
.btn.big{width:100%;font-size:20px;min-height:64px}
.btn:disabled{opacity:.4;cursor:not-allowed}
.row{display:flex;gap:10px;align-items:center;flex-wrap:wrap}
.row>.grow{flex:1}
.grid3{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}
.sw{display:flex;align-items:center;justify-content:space-between;gap:10px;padding:8px 0;border-bottom:1px solid var(--line)}
.sw:last-child{border-bottom:0}
.sw small{display:block;color:var(--muted)}
.tog{position:relative;width:52px;height:30px;flex:none}
.tog input{opacity:0;width:0;height:0;position:absolute}
.tog span{position:absolute;inset:0;background:var(--line);border-radius:99px;transition:.15s}
.tog span:before{content:"";position:absolute;width:24px;height:24px;left:3px;top:3px;background:#fff;border-radius:50%;transition:.15s}
.tog input:checked+span{background:var(--accent)}
.tog input:checked+span:before{transform:translateX(22px)}
.tog input:disabled+span{opacity:.4}
.seg{display:inline-flex;border:1px solid var(--line);border-radius:10px;overflow:hidden}
.seg button{background:var(--card);color:var(--text);border:0;padding:8px 14px;min-height:40px}
.seg button.sel{background:var(--accent);color:var(--accent-text)}
.slider{padding:6px 0}
.slider .top{display:flex;justify-content:space-between;color:var(--muted);font-size:14px}
.slider .top b{color:var(--text)}
.slider .row{flex-wrap:nowrap}
.slider input[type=range]{flex:1;width:100%;height:36px}
.slider .btn{min-width:48px;padding:6px}
.result{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;text-align:center}
.result div{background:var(--bg);border-radius:10px;padding:10px 4px}
.result b{display:block;font-size:24px}
.result small{color:var(--muted)}
input[type=text]{font:inherit;font-size:20px;padding:10px 12px;border:2px solid var(--line);border-radius:10px;background:var(--card);color:var(--text);width:100%;min-height:48px}
input[type=text]:focus{border-color:var(--accent);outline:none}
.muted{color:var(--muted)}.okc{color:var(--ok)}.badc{color:var(--bad)}
.list button{display:block;width:100%;text-align:left;margin-top:8px}
#log{background:var(--con);color:var(--con-text);font:12px/1.35 ui-monospace,Consolas,monospace;height:52vh;overflow:auto;border-radius:10px;padding:8px;white-space:pre-wrap;word-break:break-all}
#log .s{color:#93c5fd}#log .r{color:#d1d5db}#log .d{color:#9ca3af}#log .e{color:#fbbf24}
.chips{display:flex;flex-wrap:wrap;gap:6px;margin-top:8px}
.chips .btn{min-height:36px;padding:6px 10px;font-size:14px}
table{width:100%;border-collapse:collapse}td{padding:7px 4px;border-bottom:1px solid var(--line)}td:first-child{color:var(--muted)}
.overlay{position:fixed;inset:0;background:rgba(0,0,0,.55);display:none;align-items:center;justify-content:center;z-index:20;padding:16px}
.overlay.show{display:flex}
.dlg{background:var(--card);border-radius:14px;padding:20px;max-width:420px;width:100%;text-align:center}
.dlg h3{margin:0 0 8px}
.dlg .row{justify-content:center;margin-top:16px}
.spin{width:44px;height:44px;border:5px solid var(--line);border-top-color:var(--accent);border-radius:50%;margin:0 auto 12px;animation:r 1s linear infinite}
@keyframes r{to{transform:rotate(360deg)}}
#toast{position:fixed;left:50%;bottom:20px;transform:translateX(-50%);background:#111827;color:#fff;padding:10px 16px;border-radius:10px;opacity:0;transition:opacity .2s;z-index:30;pointer-events:none;max-width:90vw}
#toast.show{opacity:.95}
</style></head><body>
<header>
  <div class="bar"><h1>Tire Balancer</h1><span id="link" class="pill">...</span><button id="stop" title="Stop everything">STOP</button></div>
  <div id="ackWrap"><div class="ack"><label><input type="checkbox" id="ack"><span>I can see the machine and it is clear<small>Needed for anything that moves the platform or switches the laser on. Clears itself after 5 minutes unused.</small></span></label></div></div>
  <nav id="tabs"><button data-t="level">Level</button><button data-t="tire">Tire Info</button><button data-t="balance">Balance</button><button data-t="debug">Debug</button><button data-t="console">Console</button><button data-t="info">Info</button></nav>
</header>
<main>

<section id="t-level">
  <div class="card">
    <button class="btn primary big" data-a="level" data-g>Level</button>
    <div class="sw" style="margin-top:8px"><div>Auto Stop<small>Stop once level. Off: keep holding level until Cancel.</small></div><label class="tog"><input type="checkbox" data-sw="autoStop"><span></span></label></div>
  </div>
  <p class="muted">Level weighs the platform first and picks its gains for that load, so it works with or without a tire.</p>
</section>

<section id="t-tire">
  <div class="card">
    <h2>Tire size, as printed on the tire</h2>
    <div class="row"><input type="text" id="size" class="grow" placeholder="e.g. 215/65R15 or 4.80-12" autocomplete="off" autocapitalize="characters" spellcheck="false"><button class="btn primary" id="tireEnter">Enter</button></div>
    <p id="sizeInfo" class="muted" style="margin:8px 0 0">Metric 215/65R15 &middot; Inch 4.80-12 &middot; Flotation 31x10.50R15</p>
  </div>
  <div class="card">
    <h2>In use</h2>
    <div id="tireNow" class="muted">No tire entered yet.</div>
    <div class="sw"><div>Weight type<small id="wrInfo"></small></div><div class="seg" id="stickSeg"><button data-v="0">Clip-on</button><button data-v="1">Stick-on</button></div></div>
  </div>
  <div class="card list"><h2>Recent sizes</h2><div id="recents" class="muted">None yet.</div></div>
</section>

<section id="t-balance">
  <div class="card">
    <div class="grid3">
      <button class="btn" data-a="tare">Tare</button>
      <button class="btn primary" data-a="balance" data-g>Balance</button>
      <button class="btn" data-a="laser" data-g>Laser</button>
    </div>
    <p class="muted" style="margin:10px 0 0">Tare with the arms empty, once per power-up.</p>
  </div>
  <div class="card">
    <h2>Result</h2>
    <div class="result">
      <div><small>Add</small><b id="rMass">--</b><small id="rMassU">oz</small></div>
      <div><small>At angle</small><b id="rAngle">--</b><small>deg</small></div>
      <div><small>Total</small><b id="rTotal">--</b><small id="rTotalU">kg</small></div>
    </div>
  </div>
  <div class="card">
    <h2>Units</h2>
    <div class="sw"><div>Added weight</div><div class="seg" id="gSeg"><button data-v="0">oz</button><button data-v="1">g</button></div></div>
    <div class="sw"><div>Total weight</div><div class="seg" id="lbSeg"><button data-v="0">kg</button><button data-v="1">lbs</button></div></div>
  </div>
</section>

<section id="t-debug">
  <div class="card">
    <h2>Laser</h2>
    <div class="sw"><div>Manual Laser<small>Point the laser by hand</small></div><label class="tog"><input type="checkbox" data-sw="manualLaser" id="mlSw"><span></span></label></div>
    <div class="slider" data-sl="laserAngle" data-min="0" data-max="360" data-g><div class="top"><span>Laser angle</span><b></b></div>
      <div class="row"><button class="btn" data-step="-1">-</button><input type="range" min="0" max="360" step="1"><button class="btn" data-step="1">+</button></div></div>
    <button class="btn" data-a="set90" data-g style="width:100%">Set 90 (the dot is on the far side)</button>
  </div>
  <div class="card">
    <h2>Level trim</h2>
    <div class="slider" data-sl="pitchTrim" data-min="-10" data-max="10" data-tenths><div class="top"><span>Pitch trim</span><b></b></div>
      <div class="row"><button class="btn" data-step="-1">-</button><input type="range" min="-10" max="10" step="1"><button class="btn" data-step="1">+</button></div></div>
    <div class="slider" data-sl="rollTrim" data-min="-10" data-max="10" data-tenths><div class="top"><span>Roll trim</span><b></b></div>
      <div class="row"><button class="btn" data-step="-1">-</button><input type="range" min="-10" max="10" step="1"><button class="btn" data-step="1">+</button></div></div>
  </div>
  <div class="card">
    <h2>Leveling speed</h2>
    <div class="slider" data-sp="minSpeed" data-min="0" data-max="255"><div class="top"><span>Min speed (0-255)</span><b></b></div>
      <div class="row"><button class="btn" data-step="-1">-</button><input type="range" min="0" max="255" step="1"><button class="btn" data-step="1">+</button></div></div>
    <div class="slider" data-sp="maxSpeed" data-min="0" data-max="255"><div class="top"><span>Max speed (0-255)</span><b></b></div>
      <div class="row"><button class="btn" data-step="-1">-</button><input type="range" min="0" max="255" step="1"><button class="btn" data-step="1">+</button></div></div>
    <div class="slider" data-sp="kp" data-min="10" data-max="400"><div class="top"><span>Level gain Kp</span><b></b></div>
      <div class="row"><button class="btn" data-step="-1">-</button><input type="range" min="10" max="400" step="1"><button class="btn" data-step="1">+</button></div></div>
    <div class="row" style="margin-top:6px"><button class="btn primary" id="speedEnter">Enter Speed</button><span id="speedNote" class="muted"></span></div>
  </div>
  <div class="card">
    <h2>Switches</h2>
    <div class="sw"><div>Mass Comp<small>Weigh first and pick gains for the load</small></div><label class="tog"><input type="checkbox" data-sw="massComp"><span></span></label></div>
    <div class="sw"><div>Test Weight<small>Bench test: fake load, skews results</small></div><label class="tog"><input type="checkbox" data-sw="testMass"><span></span></label></div>
  </div>
  <div class="card">
    <h2>Machine</h2>
    <button class="btn" id="reach" data-g style="width:100%">Find Reach (platform limits)</button>
    <button class="btn danger" id="restore" style="width:100%;margin-top:10px">Restore Factory Defaults</button>
  </div>
</section>

<section id="t-console">
  <div class="card">
    <div id="log"></div>
    <div class="row" style="margin-top:8px">
      <label><input type="checkbox" id="fDebug" checked> # lines</label>
      <label><input type="checkbox" id="fPing"> Ping/Pong</label>
      <label><input type="checkbox" id="fScroll" checked> Follow</label>
      <span class="grow"></span>
      <button class="btn" id="logClear">Clear</button><button class="btn" id="logSave">Save</button>
    </div>
    <div class="row" style="margin-top:8px"><input type="text" id="cmd" class="grow" placeholder="Command to the Nano, e.g. Status" autocomplete="off" autocapitalize="off" spellcheck="false" style="font-size:16px"><button class="btn primary" id="cmdSend">Send</button></div>
    <div class="chips">
      <button class="btn" data-c="Status">Status</button>
      <button class="btn" data-c="LevelDebugOn">Level debug on</button><button class="btn" data-c="LevelDebugOff">off</button>
      <button class="btn" data-c="SensorDebugOn">Sensor debug on</button><button class="btn" data-c="SensorDebugOff">off</button>
      <button class="btn" data-c="ScaleDebugOn">Scale debug on</button><button class="btn" data-c="ScaleDebugOff">off</button>
    </div>
    <p class="muted" style="margin:8px 0 0">&gt; sent to the Nano, &lt; from the Nano, ! screen events. Commands that can move something need the safety tick. Commands: docs/Protocol.md.</p>
  </div>
</section>

<section id="t-info">
  <div class="card"><h2>Link to the Nano</h2><table id="infoLink"></table></div>
  <div class="card"><h2>Screen</h2><table id="infoScreen"></table></div>
</section>
</main>

<div class="overlay" id="busy"><div class="dlg"><div class="spin"></div><h3 id="busyText">Working...</h3><div class="row"><button class="btn danger" id="busyCancel">Cancel</button></div></div></div>
<div class="overlay" id="dlg"><div class="dlg"><h3 id="dlgTitle"></h3><div id="dlgText"></div><div class="row" id="dlgBtns"></div></div></div>
<div id="toast"></div>

<script>
"use strict";
const $=s=>document.querySelector(s), $$=s=>[...document.querySelectorAll(s)];
const ACK_MS=300000, POLL_MS=300;
let S=null, ack=false, ackUsed=0, logSeq=0, boot=null, fails=0, seenNote=null, seenMsg=null;
const LOG=[], LOG_MAX=2000, hold={};   // hold[name] = time until server values are ignored (being edited)

// ---------- helpers
function store(k,v){try{localStorage.setItem(k,v)}catch(e){}}
function load(k){try{return localStorage.getItem(k)}catch(e){return null}}
let toastT;
function toast(t){const e=$('#toast');e.textContent=t;e.classList.add('show');clearTimeout(toastT);toastT=setTimeout(()=>e.classList.remove('show'),2500)}
function dialog(title,text,btns){
  $('#dlgTitle').textContent=title;$('#dlgText').textContent=text;
  const b=$('#dlgBtns');b.innerHTML='';
  (btns||[['OK']]).forEach(([label,fn,cls])=>{const x=document.createElement('button');x.className='btn '+(cls||'');x.textContent=label;
    x.onclick=()=>{$('#dlg').classList.remove('show');if(fn)fn()};b.appendChild(x)});
  $('#dlg').classList.add('show');
}
function held(n){return hold[n]&&hold[n]>Date.now()}
function holdFor(n,ms){hold[n]=Date.now()+ms}

// ---------- talking to the screen
async function act(a,v,extra){
  const p=new URLSearchParams(Object.assign({a,v:v==null?'':String(v),ack:ack?1:0},extra||{}));
  if(ack)ackUsed=Date.now();
  try{
    const r=await fetch('/api/do',{method:'POST',body:p});
    const j=await r.json();
    if(!j.ok)dialog('Not done',j.err);
    return j;
  }catch(e){toast('No connection to the machine');return {ok:false}}
}

async function poll(){
  const ctl=new AbortController(), t=setTimeout(()=>ctl.abort(),2500);
  try{
    const r=await fetch('/api/state?log='+logSeq,{cache:'no-store',signal:ctl.signal});
    const j=await r.json();
    fails=0;render(j);
  }catch(e){
    if(++fails>=4){setLink('No WiFi link','down');if(ack)setAck(false,'Connection lost: safety tick cleared')}
  }
  clearTimeout(t);
  setTimeout(poll,document.hidden?1500:POLL_MS);
}

// ---------- safety tick
function setAck(on,why){
  ack=on;ackUsed=Date.now();$('#ack').checked=on;
  $('#ackWrap').classList.toggle('on',on);
  $$('[data-g]').forEach(e=>{
    const ctl=e.matches('.slider')?e.querySelectorAll('input,button'):[e];
    ctl.forEach(c=>c.disabled=!on)});
  updateManualLaserSwitch();
  if(why)toast(why);
}
$('#ack').onchange=e=>setAck(e.target.checked);
setInterval(()=>{if(ack&&Date.now()-ackUsed>ACK_MS)setAck(false,'Safety tick cleared after 5 minutes unused')},5000);
function updateManualLaserSwitch(){const sw=$('#mlSw');sw.disabled=!ack&&!sw.checked}   // can always switch it off

// ---------- tabs
function showTab(t){$$('nav button').forEach(b=>b.classList.toggle('sel',b.dataset.t==t));$$('main section').forEach(s=>s.classList.toggle('sel',s.id=='t-'+t));store('tab',t);if(t=='console')drawLog(true)}
$$('nav button').forEach(b=>b.onclick=()=>showTab(b.dataset.t));
showTab(load('tab')||'level');

// ---------- buttons, switches, segments
$('#stop').onclick=()=>act('stop');
$$('[data-a]').forEach(b=>b.onclick=()=>act(b.dataset.a));
$$('[data-sw]').forEach(i=>i.onchange=()=>{holdFor(i.dataset.sw,1500);act(i.dataset.sw,i.checked?1:0)});
function seg(id,name){$$('#'+id+' button').forEach(b=>b.onclick=()=>{holdFor(name,1500);setSeg(id,b.dataset.v);act(name,b.dataset.v)})}
function setSeg(id,v){$$('#'+id+' button').forEach(b=>b.classList.toggle('sel',b.dataset.v==String(v)))}
seg('stickSeg','stick');seg('gSeg','grams');seg('lbSeg','lbs');

$('#reach').onclick=()=>dialog('Find platform limits','The platform tilts up to 25 deg to each limit (about 45 s), then levels. Remove the tire first and keep clear.',
  [['Start',()=>act('reach'),'primary'],['Cancel']]);
$('#restore').onclick=()=>dialog('Restore factory defaults','Put the Debug settings, units and Clip/Stick-on back to the factory values? Calibration and Recents are kept.',
  [['Restore',()=>act('restore'),'danger'],['Cancel']]);
$('#busyCancel').onclick=()=>{const b=S&&S.busy;act(b=='balance'?'balanceCancel':'levelCancel')};

// ---------- sliders
function sliderText(el,v){return el.hasAttribute('data-tenths')?(v/10).toFixed(1)+'°':String(v)}
function sliderShow(el,v){el.querySelector('input').value=v;el.querySelector('b').textContent=sliderText(el,v)}
// Live sliders (trims, laser angle): sent while dragging, at most every 120 ms
$$('[data-sl]').forEach(el=>{
  const n=el.dataset.sl, inp=el.querySelector('input'), min=+el.dataset.min, max=+el.dataset.max;
  let timer=null,last=0;
  const send=v=>{holdFor(n,1500);sliderShow(el,v);const now=Date.now();clearTimeout(timer);
    if(now-last>120){last=now;act(n,v)}else timer=setTimeout(()=>{last=Date.now();act(n,+inp.value)},120)};
  inp.oninput=()=>send(+inp.value);
  el.querySelectorAll('[data-step]').forEach(b=>b.onclick=()=>{
    if(n=='laserAngle'){holdFor(n,1500);sliderShow(el,Math.min(max,Math.max(min,+inp.value+ +b.dataset.step)));act('laserStep',b.dataset.step)}
    else send(Math.min(max,Math.max(min,+inp.value+ +b.dataset.step)))});
});
// Speed sliders: only sent with Enter Speed, like the touch screen
const spEdited={};
$$('[data-sp]').forEach(el=>{
  const n=el.dataset.sp, inp=el.querySelector('input'), min=+el.dataset.min, max=+el.dataset.max;
  const set=v=>{spEdited[n]=true;sliderShow(el,v);speedNote()};
  inp.oninput=()=>set(+inp.value);
  el.querySelectorAll('[data-step]').forEach(b=>b.onclick=()=>set(Math.min(max,Math.max(min,+inp.value+ +b.dataset.step))));
});
function spVal(n){return +document.querySelector('[data-sp="'+n+'"] input').value}
function speedNote(){$('#speedNote').textContent=Object.keys(spEdited).length?'Not sent yet':''}
$('#speedEnter').onclick=async()=>{const j=await act('speed','',{min:spVal('minSpeed'),max:spVal('maxSpeed'),kp:spVal('kp')});
  if(j.ok){for(const k in spEdited)delete spEdited[k];speedNote()}};

// ---------- tire size
let checkT=null;
$('#size').oninput=()=>{holdFor('box',30000);clearTimeout(checkT);const v=$('#size').value.trim();
  if(!v){$('#sizeInfo').textContent='Metric 215/65R15 · Inch 4.80-12 · Flotation 31x10.50R15';$('#sizeInfo').className='muted';return}
  checkT=setTimeout(async()=>{const p=new URLSearchParams({a:'tireCheck',v});
    try{const j=await (await fetch('/api/do',{method:'POST',body:p})).json();
      $('#sizeInfo').textContent=j.ok?j.info:j.err;$('#sizeInfo').className=j.ok?'okc':'badc'}catch(e){}},300)};
$('#size').onkeydown=e=>{if(e.key=='Enter')$('#tireEnter').click()};
$('#tireEnter').onclick=async()=>{const v=$('#size').value.trim();if(!v){$('#size').focus();return}
  const j=await act('tire',v);if(j.ok){hold.box=0;$('#size').blur()}};

// ---------- console
$('#cmdSend').onclick=async()=>{const c=$('#cmd').value.trim();if(!c)return;const j=await act('cmd',c);if(j.ok)$('#cmd').value=''};
$('#cmd').onkeydown=e=>{if(e.key=='Enter')$('#cmdSend').click()};
$$('[data-c]').forEach(b=>b.onclick=()=>act('cmd',b.dataset.c));
$('#logClear').onclick=()=>{LOG.length=0;drawLog(true)};
$('#logSave').onclick=()=>{const blob=new Blob([LOG.join('\n')+'\n'],{type:'text/plain'});const a=document.createElement('a');
  a.href=URL.createObjectURL(blob);a.download='tire-balancer-log.txt';a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000)};
['fDebug','fPing'].forEach(id=>$('#'+id).onchange=()=>drawLog(true));
function logShown(l){const t=l.slice(1);
  if(!$('#fPing').checked&&(t.startsWith('Ping')||t.startsWith('Pong')))return false;
  if(!$('#fDebug').checked&&t.startsWith('#'))return false;return true}
function logLine(l){const d=document.createElement('div');const k=l[0],t=l.slice(1);
  d.className=k=='>'?'s':k=='!'?'e':t.startsWith('#')?'d':'r';d.textContent=k+' '+t;return d}
function drawLog(all,added){
  const box=$('#log');if(!$('#t-console').classList.contains('sel'))return;
  if(all){box.innerHTML='';LOG.filter(logShown).slice(-600).forEach(l=>box.appendChild(logLine(l)))}
  else (added||[]).filter(logShown).forEach(l=>box.appendChild(logLine(l)));
  while(box.childNodes.length>800)box.removeChild(box.firstChild);
  if($('#fScroll').checked)box.scrollTop=box.scrollHeight;
}

// ---------- drawing the state
function setLink(t,c){const e=$('#link');e.textContent=t;e.className='pill '+(c||'')}
function fmt(v,d){return Number(v).toFixed(d)}
function ago(ms){return ms<0?'never':ms<1000?ms+' ms ago':(ms/1000).toFixed(1)+' s ago'}
function upTime(s){const h=Math.floor(s/3600),m=Math.floor(s/60)%60;return (h?h+' h ':'')+m+' min '+(s%60)+' s'}
function table(id,rows){$(id).innerHTML=rows.map(r=>'<tr><td>'+r[0]+'</td><td>'+r[1]+'</td></tr>').join('')}
function esc(s){return String(s).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]))}

function render(j){
  if(boot!==j.boot){if(boot!==null){LOG.push('!(screen restarted)')}boot=j.boot;seenNote=seenMsg=null;if(j.seq<logSeq)logSeq=0}
  S=j;
  // link
  if(j.quiet)setLink('Paused: noise','down');else if(j.link)setLink('Nano OK','up');else setLink('No Nano link','down');

  // busy overlay
  const busyText={level:'Leveling...',reach:'Testing boundaries...',balance:'Balancing...',tare:'Taring...'}[j.busy];
  $('#busy').classList.toggle('show',!!busyText);
  if(busyText){$('#busyText').textContent=busyText;$('#busyCancel').style.display=j.busy=='tare'?'none':''}

  // switches
  $$('[data-sw]').forEach(i=>{if(!held(i.dataset.sw))i.checked=!!j[i.dataset.sw]});
  updateManualLaserSwitch();
  if(!held('stick'))setSeg('stickSeg',j.stick);
  if(!held('grams'))setSeg('gSeg',j.grams);
  if(!held('lbs'))setSeg('lbSeg',j.lbs);

  // sliders
  $$('[data-sl]').forEach(el=>{const n=el.dataset.sl;if(!held(n))sliderShow(el,j[n])});
  $$('[data-sp]').forEach(el=>{const n=el.dataset.sp;if(!spEdited[n])sliderShow(el,j[n])});

  // result
  if(j.have){
    $('#rMass').textContent=j.grams?fmt(j.massG,1):fmt(j.massG/28.3495,2);
    $('#rAngle').textContent=fmt(j.angle,1);
    $('#rTotal').textContent=j.lbs?fmt(j.totalKg*2.20462,2):fmt(j.totalKg,2);
  }else['#rMass','#rAngle','#rTotal'].forEach(s=>$(s).textContent='--');
  $('#rMassU').textContent=j.grams?'g':'oz';$('#rTotalU').textContent=j.lbs?'lbs':'kg';

  // tire
  if(!held('box')&&document.activeElement!==$('#size')&&$('#size').value!==j.box){$('#size').value=j.box}
  $('#tireNow').innerHTML=j.entered?'<b>'+esc(j.tire)+'</b><br><span class="muted">'+esc(j.tireInfo||'')+'</span>':'No tire entered yet.';
  $('#wrInfo').textContent=j.entered?'Weight radius '+fmt(j.weightR,0)+' mm':'';
  const rec=$('#recents');const key=j.recents.join('|');
  if(rec.dataset.k!==key){rec.dataset.k=key;
    if(!j.recents.length){rec.textContent='None yet.';rec.className='muted'}
    else{rec.className='';rec.innerHTML='';j.recents.forEach(s=>{const b=document.createElement('button');b.className='btn';b.textContent=s;
      b.onclick=()=>{$('#size').value=s;$('#size').dispatchEvent(new Event('input'));toast('Press Enter to use it')};rec.appendChild(b)})}}

  // pop-ups from the touch screen side (errors, notes)
  if(seenNote===null)seenNote=j.note[0];else if(j.note[0]>seenNote){seenNote=j.note[0];toast(j.note[1])}
  if(seenMsg===null)seenMsg=j.msg[0];else if(j.msg[0]>seenMsg){seenMsg=j.msg[0];dialog(j.msg[1],j.msg[2])}

  // log
  if(j.log.length){LOG.push(...j.log);if(LOG.length>LOG_MAX)LOG.splice(0,LOG.length-LOG_MAX);drawLog(false,j.log)}
  logSeq=j.seq;

  // info
  table('#infoLink',[['State',j.quiet?'<span class="badc">Paused: noise on RX</span>':j.link?'<span class="okc">Nano OK</span>':'<span class="badc">No link</span>'],
    ['Last heard',ago(j.heard)],['Bytes received',j.rx],['Bad lines',j.bad],['Commands queued',j.txq],['Nano firmware',j.nv?'V'+esc(j.nv):'unknown (send Status)']]);
  table('#infoScreen',[['Screen firmware','V'+esc(j.fw)],['Running for',upTime(j.up)],['Free memory',Math.round(j.heap/1024)+' KB (lowest '+Math.round(j.minHeap/1024)+' KB)'],
    ['Phones connected',j.sta],['WiFi','TireBalancer (open), 192.168.4.1']]);
}

setAck(false);
poll();
</script>
</body></html>)HTML";
