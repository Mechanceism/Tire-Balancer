// The web page served by Web.ino: plain HTML, CSS and JavaScript in one
// string, edited here directly (no build step). It needs no internet: a phone
// on the "TireBalancer" hotspot has none.
//
// Live data (charts, bubble levels, reach map, live load cells) is read from
// the Nano's own debug lines, which arrive in the log with the time the
// screen heard them. Each poll says which streams this page is showing
// (want=...), and the screen switches those on and the rest off.
#pragma once

static const char WEB_PAGE[] PROGMEM = R"HTML(<!doctype html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>Tire Balancer</title>
<style>
:root{--bg:#f3f4f6;--card:#fcfcfb;--text:#111827;--muted:#6b7280;--line:#d1d5db;--accent:#2563eb;--accent-text:#fff;
--ok:#15803d;--bad:#b91c1c;--warn-bg:#fef3c7;--warn-line:#d97706;--safe-bg:#dcfce7;--safe-line:#16a34a;--btn:#e5e7eb;--con:#0b1020;--con-text:#d1d5db;
--s1:#2a78d6;--s2:#eb6834;--grid:#e1e0d9;--axis:#c3c2b7;--ink2:#52514e;--ink3:#898781;
--good:#0ca30c;--warning:#fab219;--critical:#d03b3b;--tire:#4a4a47;--wheel:#e1e0d9;--arm:#898781}
@media (prefers-color-scheme:dark){:root{--bg:#0d0d0d;--card:#1a1a19;--text:#f0efec;--muted:#9ca3af;--line:#383835;--accent:#3b82f6;
--ok:#22c55e;--bad:#ef4444;--warn-bg:#3a2a07;--warn-line:#d97706;--safe-bg:#0f2e1a;--safe-line:#16a34a;--btn:#2a2a28;--con:#05070d;
--s1:#3987e5;--s2:#d95926;--grid:#2c2c2a;--axis:#383835;--ink2:#c3c2b7;--ink3:#898781;--tire:#5c5c58;--wheel:#2c2c2a;--arm:#898781}}
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
.head{display:flex;align-items:center;gap:10px;flex-wrap:wrap;margin-bottom:10px}.head h2{margin:0;flex:1}
button{font:inherit;cursor:pointer}
.btn{background:var(--btn);color:var(--text);border:1px solid var(--line);border-radius:10px;padding:12px 14px;min-height:48px}
.btn.primary{background:var(--accent);color:var(--accent-text);border-color:var(--accent);font-weight:600}
.btn.danger{background:var(--bad);color:#fff;border-color:var(--bad)}
.btn.big{width:100%;font-size:20px;min-height:64px}
.btn.sm{min-height:36px;padding:6px 10px;font-size:14px}
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
.muted{color:var(--muted)}.okc{color:var(--ok)}.badc{color:var(--bad)}.small{font-size:14px}
.list button{display:block;width:100%;text-align:left;margin-top:8px}
#log{background:var(--con);color:var(--con-text);font:12px/1.35 ui-monospace,Consolas,monospace;height:52vh;overflow:auto;border-radius:10px;padding:8px;white-space:pre-wrap;word-break:break-all}
#log .s{color:#93c5fd}#log .r{color:#d1d5db}#log .d{color:#9ca3af}#log .e{color:#fbbf24}
.chips{display:flex;flex-wrap:wrap;gap:6px;margin-top:8px}
.chips .btn{min-height:36px;padding:6px 10px;font-size:14px}
table{width:100%;border-collapse:collapse}td{padding:7px 4px;border-bottom:1px solid var(--line);vertical-align:top}td:first-child{color:var(--muted)}
td.mono{font:13px ui-monospace,Consolas,monospace;word-break:break-all}
.overlay{position:fixed;inset:0;background:rgba(0,0,0,.55);display:none;align-items:center;justify-content:center;z-index:20;padding:16px}
.overlay.show{display:flex}
.dlg{background:var(--card);border-radius:14px;padding:20px;max-width:420px;width:100%;text-align:center}
.dlg h3{margin:0 0 8px}
.dlg .row{justify-content:center;margin-top:16px}
.spin{width:44px;height:44px;border:5px solid var(--line);border-top-color:var(--accent);border-radius:50%;margin:0 auto 12px;animation:r 1s linear infinite}
@keyframes r{to{transform:rotate(360deg)}}
#toast{position:fixed;left:50%;bottom:20px;transform:translateX(-50%);background:#111827;color:#fff;padding:10px 16px;border-radius:10px;opacity:0;transition:opacity .2s;z-index:30;pointer-events:none;max-width:90vw}
#toast.show{opacity:.95}
/* live view */
.pair{display:grid;grid-template-columns:1fr 1fr;gap:12px;align-items:start}
@media (max-width:520px){.pair{grid-template-columns:1fr}}
.kv{display:grid;grid-template-columns:auto 1fr;column-gap:12px;font-size:15px;align-content:start;border-top:1px solid var(--line)}
.kv>*{padding:6px 0;border-bottom:1px solid var(--line)}
.kv span{color:var(--muted)}.kv b{font-weight:600;font-variant-numeric:tabular-nums;text-align:right}
.kv .end{grid-column:1/-1;border-bottom:0;text-align:right}
th{text-align:left;font-size:13px;font-weight:600;color:var(--muted);text-transform:uppercase;letter-spacing:.04em;padding:12px 4px 4px;border-bottom:1px solid var(--line)}
details.cmds{border-top:1px solid var(--line)}details.cmds summary{padding:10px 0;font-weight:600;cursor:pointer}
.cmd{display:grid;grid-template-columns:1fr auto;gap:2px 10px;align-items:center;padding:8px 0;border-top:1px solid var(--line)}
.cmd code{font:600 14px ui-monospace,Consolas,monospace}.cmd code small{font-weight:400;color:var(--muted)}
.cmd p{grid-column:1;margin:0;font-size:13px;color:var(--muted)}
.cmd .acts2{grid-column:2;grid-row:1/span 2;display:flex;gap:6px}
.cmd .acts2 .btn{min-height:36px;padding:6px 10px;font-size:14px}
svg text{font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
.diag{width:100%;height:auto;display:block;max-width:360px;margin:0 auto}
.cap{text-align:center;font-size:13px;color:var(--muted);margin:4px 0 0}
.chip{display:inline-flex;align-items:center;gap:6px;font-size:14px;font-weight:600;padding:3px 10px;border-radius:99px;border:1px solid var(--line)}
.chip i{font-style:normal;width:18px;height:18px;border-radius:50%;color:#fff;display:inline-flex;align-items:center;justify-content:center;font-size:12px}
.chip.good i{background:var(--good)}.chip.warning i{background:var(--warning);color:#0b0b0b}.chip.critical i{background:var(--critical)}
.acts{display:grid;grid-template-columns:auto 1fr auto;gap:6px 10px;align-items:center;font-size:14px;margin:10px 0}
.acts span{color:var(--muted)}.acts b{font-variant-numeric:tabular-nums;min-width:40px;text-align:right}
.track{position:relative;height:10px;background:var(--grid);border-radius:5px}
.track:after{content:"";position:absolute;left:50%;top:-3px;bottom:-3px;width:1px;background:var(--axis)}
.fill{position:absolute;top:0;bottom:0;border-radius:5px}
.chart{position:relative;margin-top:14px}
.chart .title{font-size:14px;font-weight:600;margin-bottom:2px}
.legend{display:flex;flex-wrap:wrap;gap:4px 14px;font-size:13px;color:var(--ink2);margin-bottom:4px}
.legend i{display:inline-block;width:14px;height:3px;border-radius:2px;vertical-align:middle;margin-right:5px}
.legend b{color:var(--text);font-weight:600;font-variant-numeric:tabular-nums;margin-left:4px}
.chart canvas{width:100%;height:170px;display:block;touch-action:pan-y}
.tip{position:absolute;pointer-events:none;background:var(--card);border:1px solid var(--line);border-radius:8px;padding:6px 8px;font-size:13px;box-shadow:0 4px 12px rgba(0,0,0,.15);display:none;white-space:nowrap;z-index:2}
.tip i{display:inline-block;width:10px;height:3px;border-radius:2px;vertical-align:middle;margin-right:5px}
/* balance report: only #print is printed, always black on white */
#print{display:none}
@media print{
  html,body{background:#fff!important;color:#111!important}
  body>*:not(#print){display:none!important}
  #print{display:block;font:11pt/1.35 system-ui,-apple-system,Segoe UI,Roboto,sans-serif;color:#111}
  #print h1{font-size:18pt;margin:0 0 2pt}#print p{margin:0 0 8pt;color:#444}
  #print svg{display:block;width:70mm;height:auto;margin:4mm auto 0}#print .cap{text-align:center;font-size:9pt}
  #print table{width:100%;border-collapse:collapse}
  #print td{padding:3pt 4pt;border-bottom:.5pt solid #bbb;color:#111}#print td:first-child{color:#555;width:42%}
  #print th{color:#555;border-bottom:.5pt solid #888;padding:9pt 4pt 2pt}
  #print tr{break-inside:avoid}
  @page{margin:14mm}
}
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
    <div class="small muted" id="runs" style="margin-top:6px"></div>
  </div>

  <div class="card">
    <div class="head"><h2>Live view</h2><div class="seg" id="liveSeg"><button data-v="level">Leveling</button><button data-v="sensor">Sensors</button><button data-v="off">Off</button></div></div>
    <p id="liveWait" class="muted small" style="margin:0"></p>

    <div id="liveLevel">
      <div class="pair">
        <div><svg class="diag" id="bubP" viewBox="-132 -120 264 240"></svg><p class="cap">Platform, off level. The dot sits on the high side.</p></div>
        <div class="kv">
          <span>Pitch (far side up +)</span><b id="vP">--</b>
          <span>Roll (right side up +)</span><b id="vR">--</b>
          <span>Frame tilt</span><b id="vB">--</b>
          <span>Reach margin</span><b id="vM">--</b>
          <span>Skipped samples</span><b id="vX">--</b>
          <div class="end" id="mChip"></div>
        </div>
      </div>
      <div class="acts">
        <span>Actuator 1 (150&deg;)</span><div class="track"><div class="fill" id="a1f"></div></div><b id="a1v">--</b>
        <span>Actuator 2 (30&deg;)</span><div class="track"><div class="fill" id="a2f"></div></div><b id="a2v">--</b>
      </div>
      <div class="chart" id="chAng"></div>
      <div class="chart" id="chAct"></div>
      <div class="row" style="margin-top:10px"><button class="btn sm" id="lvPause">Pause</button><button class="btn sm" id="lvClear">Clear</button><button class="btn sm" id="lvCsv">Save CSV</button></div>
      <p class="small muted" style="margin:6px 0 0">The charts show the last 20 s; Save CSV has the last 2 minutes. Tap or drag on a chart to read it.</p>
    </div>

    <div id="liveSensor">
      <div class="pair">
        <div><svg class="diag" id="bubT" viewBox="-132 -120 264 240"></svg><p class="cap">Platform (accelerometer only), off level</p></div>
        <div><svg class="diag" id="bubF" viewBox="-132 -120 264 240"></svg><p class="cap">Frame, against true level</p></div>
      </div>
      <div class="kv" style="margin:12px 0">
        <span>Platform pitch / roll (off level)</span><b id="vT">--</b>
        <span>Platform gyro X / Y (deg/s)</span><b id="vG">--</b>
        <span>Frame pitch / roll</span><b id="vF">--</b>
      </div>
      <h2 style="margin-top:14px">Reach map</h2>
      <svg class="diag" id="reach" viewBox="-128 -122 256 254" style="max-width:420px"></svg>
      <p class="cap" id="reachCap">How far the frame may tilt, and which way, for the platform still to reach level. The dot is the frame now.</p>
    </div>
  </div>
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
    <p class="muted" style="margin:10px 0 0">Tare with the arms empty, once per power-up. After a Balance the laser switches itself off after 30 s; press Laser to take over.</p>
    <p class="small" id="laserNote" style="margin:6px 0 0"></p>
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
    <div class="head"><h2>Wheel, seen from above</h2><label class="row small" style="gap:8px">Live load cells<span class="tog"><input type="checkbox" id="liveCells"><span></span></span></label></div>
    <svg class="diag" id="wheel" viewBox="-150 -142 300 286" style="max-width:420px"></svg>
    <p class="cap" id="wheelCap"></p>
  </div>
  <div class="card" id="repCard" style="display:none">
    <h2>Balance report</h2>
    <table id="repTable"></table>
    <div class="row" style="margin-top:10px"><button class="btn" id="repCsv">Save CSV</button><button class="btn" id="repPrint">Print / Save PDF</button></div>
    <p class="muted small" style="margin:8px 0 0">Date and time come from this phone. To save as PDF, pick "Save as PDF" as the printer. If nothing happens, open http://192.168.4.1 in the phone's own browser: the WiFi sign-in window cannot save files or print.</p>
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
    <div class="row" style="margin-top:6px"><button class="btn primary" id="speedEnter">Enter Speed</button><span id="speedNote" class="muted"></span></div>
  </div>
  <div class="card">
    <h2>Switches</h2>
    <div class="sw"><div>Mass Comp<small>Weigh first and pick gains for the load</small></div><label class="tog"><input type="checkbox" data-sw="massComp"><span></span></label></div>
    <div class="sw"><div>Test Weight<small>Bench test: fake load, skews results</small></div><label class="tog"><input type="checkbox" data-sw="testMass"><span></span></label></div>
  </div>
  <div class="card">
    <h2>Machine</h2>
    <button class="btn" id="reachBtn" data-g style="width:100%">Find Reach (platform limits)</button>
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
    <p class="muted small" style="margin:8px 0 0">&gt; sent to the Nano, &lt; from the Nano, ! screen events. The live views switch the debug outputs on and off by themselves while they are shown.</p>
  </div>
  <div class="card">
    <h2>Commands</h2>
    <p class="muted small" style="margin:0 0 6px">As in docs/Protocol.md. Buttons send at once; a command that takes a value goes into the box above for you to type the number, then Send. Anything that can move the platform or switch the laser on needs the safety tick.</p>
    <div id="cmdList"></div>
  </div>
</section>

<section id="t-info">
  <div class="card"><h2>Link to the Nano</h2><table id="infoLink"></table></div>
  <div class="card"><h2>Screen</h2><table id="infoScreen"></table></div>
  <div class="card"><div class="head"><h2>Nano settings (last Status)</h2><button class="btn sm" data-c="Status">Read again</button></div><table id="infoStatus"></table></div>
</section>
</main>

<div class="overlay" id="busy"><div class="dlg"><div class="spin"></div><h3 id="busyText">Working...</h3><div class="row"><button class="btn danger" id="busyCancel">Cancel</button></div></div></div>
<div class="overlay" id="dlg"><div class="dlg"><h3 id="dlgTitle"></h3><div id="dlgText"></div><div class="row" id="dlgBtns"></div></div></div>
<div id="toast"></div>
<div id="print"></div>

<script>
"use strict";
const $=s=>document.querySelector(s), $$=s=>[...document.querySelectorAll(s)];
const ACK_MS=300000, POLL_MS=300, MIX=0.5774, START_MARGIN=1.5, RUN_MARGIN=0.3;
let S=null, ack=false, ackUsed=0, logSeq=0, boot=null, fails=0, seenNote=null, seenMsg=null, curTab='level';
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
let CSS_OVR=null;   // fixed colours while drawing for print
function css(n){return (CSS_OVR&&CSS_OVR[n])||getComputedStyle(document.documentElement).getPropertyValue(n).trim()}
function fmt(v,d){return Number(v).toFixed(d)}
function sgn(v,d){return (v>0?'+':'')+fmt(v,d)}
function esc(s){return String(s).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]))}
function saveFile(name,text,type){const blob=new Blob([text],{type});const a=document.createElement('a');
  a.href=URL.createObjectURL(blob);a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000)}
const NS='http://www.w3.org/2000/svg';
function svgEl(parent,tag,attrs,text){const e=document.createElementNS(NS,tag);for(const k in attrs)e.setAttribute(k,attrs[k]);if(text!=null)e.textContent=text;parent.appendChild(e);return e}
function setA(e,attrs){for(const k in attrs)e.setAttribute(k,attrs[k])}

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

// Which Nano debug streams this page is showing right now
function wants(){
  if(document.hidden)return '';
  const w=[];
  if(curTab=='level'&&liveMode!='off')w.push(liveMode);
  if(curTab=='balance'&&$('#liveCells').checked)w.push('scale');
  return w.join(',');
}

async function poll(){
  const ctl=new AbortController(), t=setTimeout(()=>ctl.abort(),2500);
  try{
    const r=await fetch('/api/state?log='+logSeq+'&want='+wants(),{cache:'no-store',signal:ctl.signal});
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
function showTab(t){curTab=t;$$('nav button').forEach(b=>b.classList.toggle('sel',b.dataset.t==t));$$('main section').forEach(s=>s.classList.toggle('sel',s.id=='t-'+t));store('tab',t);
  if(t=='console')drawLog(true);if(t=='level')drawLive();if(t=='balance')drawWheel()}
$$('nav button').forEach(b=>b.onclick=()=>showTab(b.dataset.t));

// ---------- buttons, switches, segments
$('#stop').onclick=()=>act('stop');
$$('[data-a]').forEach(b=>b.onclick=()=>act(b.dataset.a));
$$('[data-sw]').forEach(i=>i.onchange=()=>{holdFor(i.dataset.sw,1500);act(i.dataset.sw,i.checked?1:0)});
function seg(id,name){$$('#'+id+' button').forEach(b=>b.onclick=()=>{holdFor(name,1500);setSeg(id,b.dataset.v);act(name,b.dataset.v)})}
function setSeg(id,v){$$('#'+id+' button').forEach(b=>b.classList.toggle('sel',b.dataset.v==String(v)))}
seg('stickSeg','stick');seg('gSeg','grams');seg('lbSeg','lbs');

$('#reachBtn').onclick=()=>dialog('Find platform limits','The platform tilts up to 25 deg to each limit (about 45 s), then levels. Remove the tire first and keep clear.',
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
$('#speedEnter').onclick=async()=>{const j=await act('speed','',{min:spVal('minSpeed'),max:spVal('maxSpeed')});
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

// Command list, as in docs/Protocol.md. [name, description, kind]
//   kind: '' = send as is; ['On','Off',...] = one button per suffix; {u:'unit'} = takes a value
// SAFE must match SAFE_COMMANDS in Web.ino (the rest need the safety tick).
const SAFE=new Set(['Ping','Status','LevelDebugOn','LevelDebugOff','SensorDebugOn','SensorDebugOff','ScaleDebugOn','ScaleDebugOff','Stop','LevelCancel','BalanceCancel','LaserOff','ManualLaserOff','Ack']);
const OO=['On','Off'];
const CMDS=[
 ['Actions',[
  ['Level','Start leveling. Ends with LevelCompleted (Auto Stop on) or runs until cancelled.'],
  ['LevelCancel','Stop leveling.'],
  ['Tare','Zero the load cells (arms empty). Needed once after every power-up before Balance or Cal.'],
  ['SetZero','Save the factory zero (empty platform, arms on) that Level weighs against for Mass Comp. Redo if the arms change.'],
  ['FindReach','Moves the platform to all four end-stops, saves how far it can tilt, then levels. About 45 s; remove the tire.'],
  ['Balance','Weigh, calculate, switch the laser on. Ends with BalanceCompleted.'],
  ['BalanceCancel','Abort a Tare / Balance / Cal that is still measuring.'],
  ['Stop','Stop everything (actuators and measuring).'],
  ['Ping','The Nano answers Pong.'],
  ['Ack','"Got your event, stop repeating it."'],
  ['Status','Print all settings (only while idle).'],
  ['Save','Store the tuning values in EEPROM.'],
  ['Defaults','Back to the built-in values (and save them).']]],
 ['Switches',[
  ['AutoStop','Off = keep hunting for level forever.',OO],
  ['MassComp','Weigh first, then use leveling gains for that load.',OO],
  ['TestMass','Add TestMass kg to every load cell (bench testing).',OO],
  ['Laser','Laser and servo pointing at the last balance result.',['On','Off','Toggle']],
  ['ManualLaser','Point at LaserAngle instead of the result.',OO],
  ['LevelDebug','# P: R: B: A: X: M: lines, 10 a second.',OO],
  ['SensorDebug','# T: F: G: Z: lines, 10 a second.',OO],
  ['ScaleDebug','# W: T: Ang: M: lines, 2 a second.',OO]]],
 ['Values',[
  ['RimD','Rim diameter (not saved).',{u:'mm'}],
  ['TreadW','Tread / section width (not saved).',{u:'mm'}],
  ['SideWall','Aspect ratio (not saved).',{u:'%'}],
  ['WeightR','Radius where the balance weight goes; 0 = rim radius (not saved).',{u:'mm'}],
  ['MinSpeed','Lowest PWM that still moves the actuators.',{u:'PWM 0-255'}],
  ['MaxSpeed','Speed cap (150 for the 45 W supply).',{u:'PWM 0-255'}],
  ['Ramp','How fast an actuator may speed up; 0 = instantly.',{u:'PWM/s'}],
  ['Kp','PID proportional gain.',{u:'PWM/deg'}],
  ['Ki','PID integral gain.',{u:'PWM/deg·s'}],
  ['Kd','PID derivative gain (0 = PI).',{u:'PWM/(deg/s)'}],
  ['LoadRef','Load the *Load gains were tuned at.',{u:'kg'}],
  ['KpLoad','Kp with LoadRef kg on the platform.',{u:''}],
  ['KiLoad','Ki with LoadRef kg on the platform.',{u:''}],
  ['KdLoad','Kd with LoadRef kg on the platform.',{u:''}],
  ['MinSpeedLoad','MinSpeed with LoadRef kg on the platform.',{u:'PWM'}],
  ['Gyro','Gyro blending; 0 = accelerometer only, 0.98 tuned.',{u:'0-0.995'}],
  ['Alpha','Angle smoothing, only used with Gyro0.',{u:'0-0.99'}],
  ['LevelTol','Pitch and roll must both be inside this to count as level.',{u:'deg'}],
  ['PitchOffset','Platform sensor pitch when truly level.',{u:'deg'}],
  ['RollOffset','Platform sensor roll when truly level.',{u:'deg'}],
  ['PitchTrim','Fine adjustment, added to the offset (not saved).',{u:'deg'}],
  ['RollTrim','Fine adjustment, added to the offset (not saved).',{u:'deg'}],
  ['LaserOffset','Servo zero vs. platform zero.',{u:'deg'}],
  ['LaserAngle','Target in manual laser mode (not saved).',{u:'deg'}],
  ['LaserSet','"The laser now points at the real n degrees" (saves).',{u:'deg'}],
  ['TestMass','Amount added per load cell while TestMass is on.',{u:'kg'}],
  ['CalA','Calibrate load cell A (210°) with this mass on its arm.',{u:'kg'}],
  ['CalB','Calibrate load cell B (90°).',{u:'kg'}],
  ['CalC','Calibrate load cell C (330°).',{u:'kg'}],
  ['JogLeft','Run actuator 1 (left, 150°) for JogTime; + = up.',{u:'signed PWM'}],
  ['JogRight','Run actuator 2 (right, 30°) for JogTime; + = up.',{u:'signed PWM'}],
  ['JogTime','Jog length, 100-40000.',{u:'ms'}],
  ['TopAxisX','Platform MPU: sensor axis along machine +X (0°).',{u:'±1..3'}],
  ['TopAxisY','Platform MPU: sensor axis along machine +Y (90°).',{u:'±1..3'}],
  ['BaseAxisX','Frame MPU: sensor axis along machine +X.',{u:'±1..3'}],
  ['BaseAxisY','Frame MPU: sensor axis along machine +Y.',{u:'±1..3'}]]],
];
(function(){
  const box=$('#cmdList');
  CMDS.forEach(([sec,list],i)=>{
    const d=document.createElement('details');d.className='cmds';d.open=i==0;
    d.innerHTML='<summary>'+sec+' <span class="muted small">('+list.length+')</span></summary>';
    list.forEach(([name,desc,kind])=>{
      const r=document.createElement('div');r.className='cmd';
      const value=kind&&!Array.isArray(kind);
      r.innerHTML='<code>'+name+(value?'<small>&lt;n&gt; '+esc(kind.u)+'</small>':Array.isArray(kind)?'<small>'+kind.join(' / ')+'</small>':'')+'</code><p>'+esc(desc)+'</p><div class="acts2"></div>';
      const a=r.querySelector('.acts2');
      const btn=(label,cmd)=>{const b=document.createElement('button');b.className='btn';b.textContent=label;
        if(cmd&&!SAFE.has(cmd))b.dataset.g='';b.onclick=()=>{if(cmd)act('cmd',cmd);else{const c=$('#cmd');c.value=name;c.focus();c.setSelectionRange(name.length,name.length);c.scrollIntoView({block:'center'})}};a.appendChild(b)};
      if(value)btn('Type value',null);
      else if(Array.isArray(kind))kind.forEach(s=>btn(s,name+s));
      else btn('Send',name);
      d.appendChild(r)});
    box.appendChild(d)});
})();
$('#logClear').onclick=()=>{LOG.length=0;drawLog(true)};
$('#logSave').onclick=()=>saveFile('tire-balancer-log.txt',LOG.map(l=>fmt(l.t/1000,3).padStart(10)+'  '+l.s[0]+' '+l.s.slice(1)).join('\n')+'\n','text/plain');
['fDebug','fPing'].forEach(id=>$('#'+id).onchange=()=>drawLog(true));
function logShown(l){const t=l.s.slice(1);
  if(!$('#fPing').checked&&(t.startsWith('Ping')||t.startsWith('Pong')))return false;
  if(!$('#fDebug').checked&&t.startsWith('#'))return false;return true}
function logLine(l){const d=document.createElement('div');const k=l.s[0],t=l.s.slice(1);
  d.className=k=='>'?'s':k=='!'?'e':t.startsWith('#')?'d':'r';d.textContent=k+' '+t;return d}
function drawLog(all,added){
  const box=$('#log');if(curTab!='console')return;
  if(all){box.innerHTML='';LOG.filter(logShown).slice(-600).forEach(l=>box.appendChild(logLine(l)))}
  else (added||[]).filter(logShown).forEach(l=>box.appendChild(logLine(l)));
  while(box.childNodes.length>800)box.removeChild(box.firstChild);
  if($('#fScroll').checked)box.scrollTop=box.scrollHeight;
}

// =====================================================================
// Live data: parsed from the Nano's debug lines in the log
// =====================================================================
const LV=[], SN=[], KEEP_MS=120000, WIN_MS=20000;
let lastScale=null, liveMode='off', paused=false, pausedEnd=0, espNow=0;   // live view starts off every time
const RUNS=[], MARKS=[]; let runStart=null;
const RE_L=/^# P:(-?[\d.]+) R:(-?[\d.]+) B:(-?[\d.]+) A:(-?\d+),(-?\d+) X:(\d+) M:(-?[\d.]+)/;
const RE_S=/^# T:(-?[\d.]+),(-?[\d.]+) F:(-?[\d.]+),(-?[\d.]+) G:(-?\d+),(-?\d+)/;
const RE_W=/^# W:(-?[\d.]+),(-?[\d.]+),(-?[\d.]+) T:(-?[\d.]+)(?: Ang:(-?[\d.]+) M:(-?[\d.]+))?/;

function trim(arr,t){while(arr.length&&arr[0].t<t-KEEP_MS)arr.shift()}
function addRun(t,err){
  if(!runStart)return;
  const dur=(t-runStart.t)/1000;
  RUNS.unshift({what:runStart.what,dur,err});RUNS.length=Math.min(RUNS.length,5);
  MARKS.push({t,label:err?'stopped':'done '+fmt(dur,1)+' s'});
  runStart=null;drawRuns();
}
function ingest(l){
  const k=l.s[0], s=l.s.slice(1), t=l.t;
  if(k=='>'){
    if(s=='Level'||s=='FindReach'){runStart={t,what:s=='Level'?'Level':'Find Reach'};MARKS.push({t,label:runStart.what})}
    else if((s=='LevelCancel'||s=='Stop')&&runStart)addRun(t,'cancelled');
    return;
  }
  if(k!='<')return;
  let m;
  if(s.startsWith('# P:')&&(m=RE_L.exec(s))){LV.push({t,p:+m[1],r:+m[2],b:+m[3],a1:+m[4],a2:+m[5],x:+m[6],m:+m[7]});trim(LV,t)}
  else if(s.startsWith('# T:')&&(m=RE_S.exec(s))){SN.push({t,tp:+m[1],tr:+m[2],fp:+m[3],fr:+m[4],gx:+m[5]/131,gy:+m[6]/131});trim(SN,t)}
  else if(s.startsWith('# W:')&&(m=RE_W.exec(s))){lastScale={t,w:[+m[1],+m[2],+m[3]],T:+m[4],ang:m[5]!=null?+m[5]:null,g:m[6]!=null?+m[6]:null}}
  else if(s.startsWith('LevelCompleted'))addRun(t,null);
  else if(s.startsWith('Err ')&&runStart)addRun(t,s.slice(4).split('*')[0]);
  while(MARKS.length&&MARKS[0].t<t-KEEP_MS)MARKS.shift();
}
function drawRuns(){
  $('#runs').textContent=RUNS.length?'Recent runs (from the press): '+RUNS.map(r=>r.what+' '+(r.err?r.err:fmt(r.dur,1)+' s')).join(' · '):'';
}

// ---------- Nano Status lines (remembered by the screen)
function status(re){if(!S)return null;for(const l of S.status){const m=re.exec(l);if(m)return m}return null}
function statusNums(){
  const o=status(/^PitchOffset (-?[\d.]+) RollOffset (-?[\d.]+)/), t=status(/LevelTol (-?[\d.]+)/), r=status(/^Reach (.+)/);
  let reach=null;
  if(r){const c=r[1].trim().split(/\s+/).map(p=>p.split(',').map(Number));if(c.length==4&&c.every(p=>p.length==2&&p.every(isFinite)))reach=c}
  return {po:o?+o[1]:null,ro:o?+o[2]:null,tol:t?+t[1]:0.1,reach};
}

// ---------- bubble level (svg): sqrt scale so 0.1 deg and 5 deg both show
const BUB_R=100, BUB_MAX=5;
function bubbleInit(svg,top,bottom){
  svg.innerHTML='';
  svgEl(svg,'circle',{r:BUB_R,fill:'none',stroke:css('--axis'),'stroke-width':1});
  [0.1,1,2].forEach(d=>{const r=BUB_R*Math.sqrt(d/BUB_MAX);svgEl(svg,'circle',{r,fill:'none',stroke:css('--grid'),'stroke-width':1});
    svgEl(svg,'text',{x:r*0.71+2,y:-r*0.71-2,'font-size':9,fill:css('--ink3')},d+'°')});
  svgEl(svg,'line',{x1:-BUB_R,y1:0,x2:BUB_R,y2:0,stroke:css('--grid'),'stroke-width':1});
  svgEl(svg,'line',{x1:0,y1:-BUB_R,x2:0,y2:BUB_R,stroke:css('--grid'),'stroke-width':1});
  svgEl(svg,'text',{x:0,y:-BUB_R-6,'font-size':11,'text-anchor':'middle',fill:css('--ink3')},top);
  svgEl(svg,'text',{x:0,y:BUB_R+15,'font-size':11,'text-anchor':'middle',fill:css('--ink3')},bottom);
  svgEl(svg,'text',{x:BUB_R+4,y:4,'font-size':11,fill:css('--ink3')},'0°');
  svgEl(svg,'text',{x:-BUB_R-4,y:4,'font-size':11,'text-anchor':'end',fill:css('--ink3')},'180°');
  svg._dot=svgEl(svg,'circle',{r:8,fill:css('--s1'),stroke:css('--card'),'stroke-width':2,visibility:'hidden'});
  svg._val=svgEl(svg,'text',{x:0,y:BUB_R+30,'font-size':12,'text-anchor':'middle',fill:css('--ink2')},'');
}
function bubbleSet(svg,pitch,roll){
  if(!svg._dot)return;
  if(pitch==null){setA(svg._dot,{visibility:'hidden'});return}
  const mag=Math.hypot(pitch,roll), rr=BUB_R*Math.sqrt(Math.min(mag,BUB_MAX)/BUB_MAX);
  const x=mag?roll/mag*rr:0, y=mag?-pitch/mag*rr:0;
  setA(svg._dot,{cx:x,cy:y,visibility:'visible',fill:mag>BUB_MAX?css('--critical'):css('--s1')});
}

// ---------- line chart (canvas): one unit per chart, legend + hover readout
function makeChart(box,title,series,opt){
  box.innerHTML='<div class="title">'+title+'</div><div class="legend"></div><canvas></canvas><div class="tip"></div>';
  const cv=box.querySelector('canvas'), leg=box.querySelector('.legend'), tip=box.querySelector('.tip');
  leg.innerHTML=series.map(s=>'<span><i style="background:var('+s.c+')"></i>'+s.label+'<b data-k="'+s.k+'">--</b></span>').join('');
  const ch={data:[],t1:0,hoverX:null};
  const pos=e=>{const r=cv.getBoundingClientRect();ch.hoverX=e.clientX-r.left;ch.draw()};
  cv.addEventListener('pointerdown',pos);cv.addEventListener('pointermove',pos);
  cv.addEventListener('pointerleave',()=>{ch.hoverX=null;ch.draw()});
  ch.draw=function(){
    const W=cv.clientWidth,H=cv.clientHeight,dpr=window.devicePixelRatio||1;
    if(!W)return;
    if(cv.width!=Math.round(W*dpr)||cv.height!=Math.round(H*dpr)){cv.width=Math.round(W*dpr);cv.height=Math.round(H*dpr)}
    const g=cv.getContext('2d');g.setTransform(dpr,0,0,dpr,0,0);g.clearRect(0,0,W,H);
    const L=44,R=10,T=8,B=20,pw=W-L-R,ph=H-T-B, t1=ch.t1, t0=t1-WIN_MS;
    const d=ch.data.filter(p=>p.t>=t0-500&&p.t<=t1);
    // y range: the data, at least +-minHalf, always including 0
    let lo=-opt.minHalf,hi=opt.minHalf;
    d.forEach(p=>series.forEach(s=>{lo=Math.min(lo,p[s.k]);hi=Math.max(hi,p[s.k])}));
    if(opt.clamp){lo=Math.max(lo,-opt.clamp);hi=Math.min(hi,opt.clamp)}
    const raw=(hi-lo)/4,p10=Math.pow(10,Math.floor(Math.log10(raw))),f=raw/p10,step=(f<1.5?1:f<3?2:f<7?5:10)*p10;
    lo=Math.floor(lo/step)*step;hi=Math.ceil(hi/step)*step;
    const X=t=>L+(t-t0)/WIN_MS*pw, Y=v=>T+(hi-v)/(hi-lo)*ph;
    g.font='11px system-ui,sans-serif';g.textBaseline='middle';
    // band (level tolerance)
    if(opt.band){const b=opt.band();g.fillStyle=css('--s1');g.globalAlpha=.1;g.fillRect(L,Y(b),pw,Y(-b)-Y(b));g.globalAlpha=1}
    // grid + y ticks
    for(let v=lo;v<=hi+step/2;v+=step){const y=Math.round(Y(v))+.5;g.strokeStyle=Math.abs(v)<step/2?css('--axis'):css('--grid');g.lineWidth=1;
      g.beginPath();g.moveTo(L,y);g.lineTo(L+pw,y);g.stroke();g.fillStyle=css('--ink3');g.textAlign='right';g.fillText(fmt(v,step<1?(step<0.1?2:1):0),L-6,y)}
    // x ticks
    g.textAlign='center';g.textBaseline='alphabetic';g.fillStyle=css('--ink3');
    [-20,-15,-10,-5,0].forEach(s=>{const x=X(t1+s*1000);g.fillText(s?s+' s':'now',x,H-4)});
    // run marks; a label that would run into the previous one goes a row lower
    g.textAlign='left';g.textBaseline='top';const rowEnd=[-1e9,-1e9,-1e9];
    MARKS.forEach(mk=>{if(mk.t<t0||mk.t>t1)return;const x=Math.round(X(mk.t))+.5;g.strokeStyle=css('--axis');g.beginPath();g.moveTo(x,T);g.lineTo(x,T+ph);g.stroke();
      const tw=g.measureText(mk.label).width;let row=rowEnd.findIndex(e=>x+3>e);if(row<0)row=0;rowEnd[row]=x+3+tw+6;
      const lx=Math.min(x+3,L+pw-tw);g.fillStyle=css('--card');g.fillRect(lx-1,T+1+row*13,tw+2,12);g.fillStyle=css('--ink2');g.fillText(mk.label,lx,T+1+row*13)});
    // lines (break at gaps over 0.5 s)
    g.save();g.beginPath();g.rect(L,T,pw,ph);g.clip();
    series.forEach(s=>{g.strokeStyle=css(s.c);g.lineWidth=2;g.lineJoin=g.lineCap='round';g.beginPath();let prev=null;
      d.forEach(p=>{const x=X(p.t),y=Y(Math.max(lo,Math.min(hi,p[s.k])));if(!prev||p.t-prev.t>500)g.moveTo(x,y);else g.lineTo(x,y);prev=p});g.stroke()});
    g.restore();
    // end dots + legend values (latest sample)
    const last=d[d.length-1];
    if(last&&t1-last.t<1500)series.forEach(s=>{g.beginPath();g.arc(X(last.t),Y(Math.max(lo,Math.min(hi,last[s.k]))),4,0,7);g.fillStyle=css(s.c);g.fill();g.lineWidth=2;g.strokeStyle=css('--card');g.stroke()});
    series.forEach(s=>{leg.querySelector('[data-k="'+s.k+'"]').textContent=last?opt.fmt(last[s.k]):'--'});
    // hover: crosshair + readout of the nearest sample
    if(ch.hoverX!=null&&d.length){
      const th=t0+(ch.hoverX-L)/pw*WIN_MS;let best=d[0];d.forEach(p=>{if(Math.abs(p.t-th)<Math.abs(best.t-th))best=p});
      const x=Math.round(X(best.t))+.5;g.strokeStyle=css('--ink3');g.lineWidth=1;g.beginPath();g.moveTo(x,T);g.lineTo(x,T+ph);g.stroke();
      series.forEach(s=>{g.beginPath();g.arc(x,Y(Math.max(lo,Math.min(hi,best[s.k]))),4,0,7);g.fillStyle=css(s.c);g.fill();g.lineWidth=2;g.strokeStyle=css('--card');g.stroke()});
      tip.innerHTML='<div class="muted">'+fmt((best.t-t1)/1000,1)+' s</div>'+series.map(s=>'<div><i style="background:var('+s.c+')"></i>'+s.label+' <b>'+opt.fmt(best[s.k])+'</b></div>').join('');
      tip.style.display='block';const tw=tip.offsetWidth;tip.style.left=Math.min(Math.max(0,x+10+(x+10+tw>W?-tw-20:0)),W-tw)+'px';tip.style.top=(box.querySelector('canvas').offsetTop+10)+'px';
    }else tip.style.display='none';
  };
  return ch;
}
const chAng=makeChart($('#chAng'),'Platform off level (deg)',[{k:'p',label:'Pitch',c:'--s1'},{k:'r',label:'Roll',c:'--s2'}],
  {minHalf:0.3,clamp:30,fmt:v=>sgn(v,2)+'°',band:()=>statusNums().tol});
const chAct=makeChart($('#chAct'),'Actuator drive (PWM, + = up)',[{k:'a1',label:'Actuator 1 (150°)',c:'--s1'},{k:'a2',label:'Actuator 2 (30°)',c:'--s2'}],
  {minHalf:50,clamp:255,fmt:v=>sgn(v,0)});

$$('#liveSeg button').forEach(b=>b.onclick=()=>{liveMode=b.dataset.v;drawLive()});
$('#lvPause').onclick=()=>{paused=!paused;pausedEnd=espNow;$('#lvPause').textContent=paused?'Resume':'Pause';drawLive()};
$('#lvClear').onclick=()=>{LV.length=0;SN.length=0;MARKS.length=0;drawLive()};
$('#lvCsv').onclick=()=>{
  const rows=['time_s,pitch_off_deg,roll_off_deg,frame_tilt_deg,act1_pwm,act2_pwm,reach_margin_deg,skipped'];
  LV.forEach(p=>rows.push([fmt(p.t/1000,3),p.p,p.r,p.b,p.a1,p.a2,p.m,p.x].join(',')));
  saveFile('tire-balancer-level.csv',rows.join('\n')+'\n','text/csv')};

function chip(el,margin){
  if(margin==null){el.innerHTML='';return}
  const c=margin>=START_MARGIN?['good','✓','Level can start']:margin>=RUN_MARGIN?['warning','!','Close to the limit']:['critical','✕','Too steep to level'];
  el.innerHTML='<span class="chip '+c[0]+'"><i>'+c[1]+'</i>'+c[2]+'</span>';
}
function actBar(fill,val,v){
  const f=Math.max(-255,Math.min(255,v))/255*50;
  fill.style.left=(f<0?50+f:50)+'%';fill.style.width=Math.abs(f)+'%';fill.style.background=css(v<0?'--s2':'--s1');val.textContent=sgn(v,0);
}

let bubInit=false;
function drawLive(){
  setSeg('liveSeg',liveMode);
  $('#liveLevel').style.display=liveMode=='level'?'':'none';
  $('#liveSensor').style.display=liveMode=='sensor'?'':'none';
  if(curTab!='level')return;
  if(!bubInit){bubInit=true;bubbleInit($('#bubP'),'far side (90°)','screen side (270°)');bubbleInit($('#bubT'),'far side','screen side');bubbleInit($('#bubF'),'far side','screen side');reachInit()}
  const on=S&&S.streams;
  const fresh=(arr)=>arr.length&&espNow-arr[arr.length-1].t<1500;
  let wait='';
  if(liveMode=='off')wait='Off. Choose Leveling (bubble level, actuators, charts) or Sensors (platform and frame, reach map); the Nano then sends live data while this view is open.';
  else if(liveMode=='level'&&!fresh(LV))wait=S&&!S.link?'No link to the Nano.':'Waiting for data from the Nano...';
  else if(liveMode=='sensor'&&!fresh(SN))wait=S&&!S.link?'No link to the Nano.':'Waiting for data from the Nano...';
  $('#liveWait').textContent=wait;
  if(liveMode=='level'){
    const p=LV[LV.length-1], ok=p&&fresh(LV);
    bubbleSet($('#bubP'),ok?p.p:null,ok?p.r:null);
    $('#vP').textContent=ok?sgn(p.p,2)+'°':'--';$('#vR').textContent=ok?sgn(p.r,2)+'°':'--';
    $('#vB').textContent=ok?fmt(p.b,1)+'°':'--';$('#vM').textContent=ok?fmt(p.m,1)+'°':'--';$('#vX').textContent=ok?p.x:'--';
    chip($('#mChip'),ok?p.m:null);
    actBar($('#a1f'),$('#a1v'),ok?p.a1:0);actBar($('#a2f'),$('#a2v'),ok?p.a2:0);if(!ok){$('#a1v').textContent='--';$('#a2v').textContent='--'}
    const t1=paused?pausedEnd:espNow;
    chAng.data=LV;chAng.t1=t1;chAng.draw();chAct.data=LV;chAct.t1=t1;chAct.draw();
  }else if(liveMode=='sensor'){
    const p=SN[SN.length-1], ok=p&&fresh(SN), st=statusNums();
    const tp=(st.po||0)+(S?S.pitchTrim/10:0), tr=(st.ro||0)+(S?S.rollTrim/10:0);
    bubbleSet($('#bubT'),ok?p.tp-tp:null,ok?p.tr-tr:null);
    bubbleSet($('#bubF'),ok?p.fp:null,ok?p.fr:null);
    $('#vT').textContent=ok?sgn(p.tp-tp,2)+'° / '+sgn(p.tr-tr,2)+'°':'--';
    $('#vF').textContent=ok?sgn(p.fp,1)+'° / '+sgn(p.fr,1)+'°':'--';
    $('#vG').textContent=ok?sgn(p.gx,1)+' / '+sgn(p.gy,1):'--';
    reachDraw(ok?p:null,st,tp,tr);
  }
}

// ---------- reach map: frame tilt plane (x = roll, right side up +; y = pitch, far side up +)
// The Nano stores the reach area as 4 corners in actuator-corner angles
// c1 = pitch - MIX*roll, c2 = pitch + MIX*roll of platform relative to frame.
// The frame may sit at f when (target - f) is inside that area.
const RM=25, RS=100/RM;   // map shows +-25 deg
function reachInit(){
  const svg=$('#reach');svg.innerHTML='';
  for(let v=-25;v<=25;v+=5){const a=v*RS,c=v?css('--grid'):css('--axis');
    svgEl(svg,'line',{x1:a,y1:-100,x2:a,y2:100,stroke:c,'stroke-width':1});svgEl(svg,'line',{x1:-100,y1:a,x2:100,y2:a,stroke:c,'stroke-width':1});
    if(v&&v%10==0){svgEl(svg,'text',{x:a,y:114,'font-size':9,'text-anchor':'middle',fill:css('--ink3')},v+'°');svgEl(svg,'text',{x:-104,y:-a+3,'font-size':9,'text-anchor':'end',fill:css('--ink3')},v+'°')}}
  svgEl(svg,'text',{x:0,y:-108,'font-size':11,'text-anchor':'middle',fill:css('--ink3')},'far side up (90°)');
  svgEl(svg,'text',{x:0,y:127,'font-size':11,'text-anchor':'middle',fill:css('--ink3')},'screen side up (270°)');
  svgEl(svg,'text',{x:0,y:0,'font-size':11,'text-anchor':'middle',fill:css('--ink3'),transform:'translate(113,0) rotate(90)'},'right side up (0°)');
  svg._area=svgEl(svg,'polygon',{fill:css('--s1'),'fill-opacity':.1,stroke:css('--s1'),'stroke-width':2,'stroke-linejoin':'round'});
  svg._start=svgEl(svg,'polygon',{fill:'none',stroke:css('--s1'),'stroke-width':1,'stroke-opacity':.6});
  svg._dot=svgEl(svg,'circle',{r:6,fill:css('--s2'),stroke:css('--card'),'stroke-width':2,visibility:'hidden'});
}
// Corner space -> frame plane point, for the target (tp, tr)
function cToFrame(c,tp,tr){const p=(c[0]+c[1])/2, r=(c[1]-c[0])/(2*MIX);return [tr-r,tp-p]}
// Same margin as the Nano's reachMarginFor()
function reachMargin(reach,x,y){
  let area=0;for(let i=0;i<4;i++){const j=(i+1)&3;area+=reach[i][0]*reach[j][1]-reach[j][0]*reach[i][1]}
  const s=area>0?1:-1;let m=1e6;
  for(let i=0;i<4;i++){const j=(i+1)&3,ex=reach[j][0]-reach[i][0],ey=reach[j][1]-reach[i][1],len=Math.hypot(ex,ey);if(len<0.01)continue;
    m=Math.min(m,s*(ex*(y-reach[i][1])-ey*(x-reach[i][0]))/len)}
  return m;
}
// The area shrunk by d (corner space), for the "Level can start" line
function inset(reach,d){
  let area=0;for(let i=0;i<4;i++){const j=(i+1)&3;area+=reach[i][0]*reach[j][1]-reach[j][0]*reach[i][1]}
  const s=area>0?1:-1, lines=[];
  for(let i=0;i<4;i++){const j=(i+1)&3,ex=reach[j][0]-reach[i][0],ey=reach[j][1]-reach[i][1],len=Math.hypot(ex,ey)||1,nx=-s*ey/len,ny=s*ex/len;
    lines.push([reach[i][0]+nx*d,reach[i][1]+ny*d,ex,ey])}
  const out=[];
  for(let i=0;i<4;i++){const a=lines[(i+3)&3],b=lines[i],den=a[2]*b[3]-a[3]*b[2];if(Math.abs(den)<1e-9)return null;
    const u=((b[0]-a[0])*b[3]-(b[1]-a[1])*b[2])/den;out.push([a[0]+a[2]*u,a[1]+a[3]*u])}
  return out;
}
function reachDraw(p,st,tp,tr){
  const svg=$('#reach');if(!svg._area)return;
  const P=pts=>pts.map(c=>{const f=cToFrame(c,tp,tr);return (f[0]*RS).toFixed(1)+','+(-f[1]*RS).toFixed(1)}).join(' ');
  if(!st.reach){setA(svg._area,{points:''});setA(svg._start,{points:''});$('#reachCap').textContent='Waiting for the reach area from the Nano (Status)...'}
  else{setA(svg._area,{points:P(st.reach)});const ins=inset(st.reach,START_MARGIN);setA(svg._start,{points:ins?P(ins):''})}
  if(!p){setA(svg._dot,{visibility:'hidden'});return}
  setA(svg._dot,{cx:Math.max(-RM,Math.min(RM,p.fr))*RS,cy:-Math.max(-RM,Math.min(RM,p.fp))*RS,visibility:'visible'});
  if(st.reach){
    const x=(tp-MIX*tr)-(p.fp-MIX*p.fr), y=(tp+MIX*tr)-(p.fp+MIX*p.fr), m=reachMargin(st.reach,x,y);
    const c=m>=START_MARGIN?'Level can start':m>=RUN_MARGIN?'Close to the limit: Level would refuse':'Too steep: the platform cannot reach level';
    $('#reachCap').innerHTML='Margin <b>'+fmt(m,1)+'°</b> &middot; '+c+'. Outer line: the platform\'s limit; inner line: Level starts only inside it. The dot is the frame now.';
  }
}

// =====================================================================
// Wheel drawing (Balance tab): seen from above, standing at the screen
// =====================================================================
const ARMS=[['A',210],['B',90],['C',330]];
function polar(r,deg){const a=deg*Math.PI/180;return [r*Math.cos(a),-r*Math.sin(a)]}
$('#liveCells').onchange=()=>{lastScale=null;drawWheel()};
// d = {ang, g, kg, cgMm, w:[kg A,B,C] or null}; draws into svg with the
// current colours (or the light ones while printing, see CSS_OVR)
function wheelSVG(svg,j,d){
  svg.innerHTML='';
  const known=!!j.entered, rim=known?j.rimMm:300, side=known?j.sideMm:110;
  const R=100, k=R/(rim/2+side), rr=rim/2*k, cr=(rim/2+side/2)*k, wr=(known?j.weightR:rim/2+10)*k;
  // tire and wheel
  svgEl(svg,'circle',{r:(R+rr)/2,fill:'none',stroke:css('--tire'),'stroke-width':R-rr});
  svgEl(svg,'circle',{r:rr,fill:css('--wheel')});
  svgEl(svg,'circle',{r:6,fill:css('--axis')});
  // angle ticks every 30 deg
  for(let a=0;a<360;a+=30){const p1=polar(R+3,a),p2=polar(R+(a%90?7:11),a);svgEl(svg,'line',{x1:p1[0],y1:p1[1],x2:p2[0],y2:p2[1],stroke:css('--ink3'),'stroke-width':1})}
  [[0,'0°'],[180,'180°']].forEach(([a,t])=>{const p=polar(R+22,a);svgEl(svg,'text',{x:p[0],y:p[1]+4,'font-size':11,'text-anchor':'middle',fill:css('--ink3')},t)});
  svgEl(svg,'text',{x:0,y:R+26,'font-size':11,'text-anchor':'middle',fill:css('--ink3')},'270° (screen)');
  // arms with load cell readings (B at 90 deg: its label above the tire, the angle in it)
  ARMS.forEach(([n,a],i)=>{const p0=polar(10,a),p1=polar(cr,a);
    svgEl(svg,'line',{x1:p0[0],y1:p0[1],x2:p1[0],y2:p1[1],stroke:css('--arm'),'stroke-width':4,'stroke-linecap':'round'});
    svgEl(svg,'rect',{x:p1[0]-5,y:p1[1]-5,width:10,height:10,rx:2,fill:css('--arm')});
    const kgTxt=d.w?(j.lbs?fmt(d.w[i]*2.20462,2)+' lbs':fmt(d.w[i],2)+' kg'):'';
    const name=a==90?'B (90°)':n, top=a==90;
    const lp=top?[0,-R-(kgTxt?28:16)]:polar(R+30,a);
    svgEl(svg,'text',{x:lp[0],y:lp[1],'font-size':12,'font-weight':600,'text-anchor':'middle',fill:css('--ink2')},name);
    if(kgTxt)svgEl(svg,'text',{x:lp[0],y:lp[1]+13,'font-size':11,'text-anchor':'middle',fill:css('--ink2')},kgTxt)});
  if(d.ang==null)return;
  // centre of gravity (heavy side), drawn larger than life
  if(d.cgMm!=null){const r=Math.min(45,6+16*Math.log2(1+d.cgMm)),c=polar(r,d.ang+180);
    svgEl(svg,'line',{x1:0,y1:0,x2:c[0],y2:c[1],stroke:css('--s2'),'stroke-width':2});
    svgEl(svg,'circle',{cx:c[0],cy:c[1],r:6,fill:css('--s2'),stroke:css('--card'),'stroke-width':2})}
  // the laser line and the weight
  const w=polar(wr,d.ang),wl=polar(R,d.ang);
  svgEl(svg,'line',{x1:0,y1:0,x2:wl[0],y2:wl[1],stroke:css('--s1'),'stroke-width':1.5,'stroke-opacity':.7});
  svgEl(svg,'rect',{x:-9,y:-5,width:18,height:10,rx:3,fill:css('--s1'),stroke:css('--card'),'stroke-width':2,transform:'translate('+w[0]+','+w[1]+') rotate('+(-d.ang+90)+')'});
  const gTxt=d.g!=null?(j.grams?fmt(d.g,1)+' g':fmt(d.g/28.3495,2)+' oz'):'';
  const tp=polar(wr-22,d.ang);
  if(gTxt)svgEl(svg,'text',{x:tp[0],y:tp[1]+4,'font-size':12,'font-weight':600,'text-anchor':'middle',fill:css('--text'),stroke:css('--wheel'),'stroke-width':3,'paint-order':'stroke'},gTxt);
}
// What the Balance tab drawing shows: live load cells, or the last Balance
function drawWheel(){
  const j=S;if(!j||curTab!='balance')return;
  const live=$('#liveCells').checked, sc_=live&&lastScale&&espNow-lastScale.t<2500?lastScale:null;
  const known=!!j.entered, rim=known?j.rimMm:300, side=known?j.sideMm:110;
  const d={ang:null,g:null,kg:null,cgMm:null,w:sc_?sc_.w:(!live&&SNAP&&SNAP.resW)?SNAP.resW:null};
  if(sc_){
    d.kg=sc_.T;let x=0,y=0;ARMS.forEach(([n,a],i)=>{x+=sc_.w[i]*Math.cos(a*Math.PI/180);y+=sc_.w[i]*Math.sin(a*Math.PI/180)});
    if(d.kg>0.5){d.ang=(Math.atan2(y,x)*180/Math.PI+540)%360;if(known)d.cgMm=(rim/2+side/2)*Math.hypot(x,y)/d.kg}
    if(sc_.ang!=null){d.ang=sc_.ang;d.g=sc_.g}
  }else if(j.have){d.ang=j.angle;d.g=j.massG;d.kg=j.totalKg;if(known&&d.kg>0)d.cgMm=d.g/1000*j.weightR/d.kg}
  wheelSVG($('#wheel'),j,d);
  let cap;
  if(d.ang!=null){
    const gTxt=d.g!=null?(j.grams?fmt(d.g,1)+' g':fmt(d.g/28.3495,2)+' oz'):'';
    cap=(sc_?'Live: ':'Last Balance: ')+(gTxt?'add '+gTxt+' ':'add weight ')+'at '+fmt(d.ang,1)+'°'+(d.kg!=null?', total '+(j.lbs?fmt(d.kg*2.20462,2)+' lbs':fmt(d.kg,2)+' kg'):'')+'.';
    if(d.cgMm!=null)cap+=' Centre of gravity '+fmt(d.cgMm,2)+' mm off centre, toward the heavy side (dot, drawn larger).';
  }else cap=live?(sc_?'Live: not enough load on the arms.':'Waiting for load cell readings...'):'Press Balance to see where the weight goes.';
  if(!known)cap+=' Enter the tire size to draw it to scale.';
  $('#wheelCap').textContent=cap;
}

// =====================================================================
// Balance report: CSV or print / PDF, with this phone's date and time.
// SNAP is a copy of the state taken when this page first saw the result,
// so changing the tire or Clip/Stick-on afterwards does not change it.
// =====================================================================
let SNAP=null, snapId=null, snapTime=null;
function snapResult(j){
  if(j.resId!==snapId){snapTime=snapId===null?null:new Date();snapId=j.resId;SNAP=j.have?JSON.parse(JSON.stringify(j)):null;return}
  if(SNAP){if(j.resW&&!SNAP.resW)SNAP.resW=j.resW;if(j.resLvl&&!SNAP.resLvl)SNAP.resLvl=j.resLvl}
}
// Rows: [section] or [item, value]
function reportRows(){
  const j=SNAP;if(!j)return null;
  const r=[], oz=g=>fmt(g/28.3495,2)+' oz', lb=kg=>fmt(kg*2.20462,2)+' lbs';
  const when=snapTime?snapTime.toLocaleString():'Before this page was opened (report made '+new Date().toLocaleString()+')';
  r.push(['Balance']);
  r.push(['Date and time',when]);
  r.push(['Add weight',fmt(j.massG,1)+' g ('+oz(j.massG)+')']);
  r.push(['At angle',fmt(j.angle,1)+'° (counter-clockwise from the right side, 0°; where the laser points)']);
  r.push(['Total weight',fmt(j.totalKg,2)+' kg ('+lb(j.totalKg)+')']);
  if(j.entered&&j.totalKg>0)r.push(['Centre of gravity off centre',fmt(j.massG/1000*j.weightR/j.totalKg,2)+' mm, toward '+fmt((j.angle+180)%360,1)+'°']);
  r.push(['Tire']);
  if(j.entered){
    r.push(['Size',j.tire]);
    r.push(['Section width',fmt(j.widthMm,0)+' mm']);
    r.push(['Sidewall height',fmt(j.sideMm,0)+' mm']);
    r.push(['Rim diameter',fmt(j.rimMm,0)+' mm ('+fmt(j.rimMm/25.4,1)+' in)']);
    r.push(['Overall diameter',fmt(j.rimMm+2*j.sideMm,0)+' mm ('+fmt((j.rimMm+2*j.sideMm)/25.4,1)+' in)']);
    r.push(['Weight type',j.stick?'Stick-on':'Clip-on']);
    r.push(['Weight radius',fmt(j.weightR,0)+' mm']);
  }else r.push(['Size','not entered']);
  if(j.resW){r.push(['Load cells at balance']);ARMS.forEach(([n,a],i)=>r.push(['Arm '+n+' ('+a+'°)',fmt(j.resW[i],3)+' kg']))}
  if(j.resLvl){r.push(['Platform at balance']);r.push(['Pitch off level',sgn(j.resLvl[0],2)+'°']);r.push(['Roll off level',sgn(j.resLvl[1],2)+'°']);r.push(['Frame tilt',fmt(j.resLvl[2],1)+'°'])}
  r.push(['Machine']);
  r.push(['Mass Comp',j.massComp?'On':'Off']);
  r.push(['Test Weight',j.testMass?'ON - bench test, result not real':'Off']);
  r.push(['Firmware','Screen V'+j.fw+', Nano '+(j.nv?'V'+j.nv:'unknown')]);
  return r;
}
function drawReport(){
  const rows=reportRows();
  $('#repCard').style.display=rows?'':'none';if(!rows)return;
  $('#repTable').innerHTML=rows.map(x=>x.length==1?'<tr><th colspan="2">'+esc(x[0])+'</th></tr>':'<tr><td>'+esc(x[0])+'</td><td>'+esc(x[1])+'</td></tr>').join('');
}
function stamp(){const d=snapTime||new Date(),p=n=>String(n).padStart(2,'0');return d.getFullYear()+p(d.getMonth()+1)+p(d.getDate())+'-'+p(d.getHours())+p(d.getMinutes())}
$('#repCsv').onclick=()=>{
  const rows=reportRows();if(!rows)return;let sec='';const q=s=>'"'+String(s).replace(/"/g,'""')+'"';
  const out=['section,item,value'];rows.forEach(x=>{if(x.length==1)sec=x[0];else out.push([sec,x[0],x[1]].map(q).join(','))});
  saveFile('tire-balance-'+stamp()+'.csv',out.join('\r\n')+'\r\n','text/csv')};
// Printing uses the light colours whatever the phone's theme
const PRINT_COLORS={'--tire':'#4a4a47','--wheel':'#e1e0d9','--axis':'#c3c2b7','--ink3':'#898781','--ink2':'#52514e','--arm':'#898781','--s1':'#2a78d6','--s2':'#eb6834','--card':'#ffffff','--text':'#111111'};
$('#repPrint').onclick=()=>{
  const rows=reportRows();if(!rows)return;const j=SNAP,p=$('#print');
  p.innerHTML='<h1>Tire balance report</h1><p>'+esc(snapTime?snapTime.toLocaleString():new Date().toLocaleString())+(j.entered?' &middot; '+esc(j.tire):'')+'</p>'+
    '<svg viewBox="-150 -142 300 286"></svg><p class="cap">Seen from above, standing at the screen. Blue: where the weight goes. Orange: centre of gravity (drawn larger).</p><table></table>';
  CSS_OVR=PRINT_COLORS;try{wheelSVG(p.querySelector('svg'),j,{ang:j.angle,g:j.massG,kg:j.totalKg,cgMm:j.entered&&j.totalKg>0?j.massG/1000*j.weightR/j.totalKg:null,w:j.resW||null})}finally{CSS_OVR=null}
  p.querySelector('table').innerHTML=rows.map(x=>x.length==1?'<tr><th colspan="2">'+esc(x[0])+'</th></tr>':'<tr><td>'+esc(x[0])+'</td><td>'+esc(x[1])+'</td></tr>').join('');
  const t=document.title;document.title='tire-balance-'+stamp();window.print();document.title=t;
};

// ---------- drawing the state
function setLink(t,c){const e=$('#link');e.textContent=t;e.className='pill '+(c||'')}
function ago(ms){return ms<0?'never':ms<1000?ms+' ms ago':(ms/1000).toFixed(1)+' s ago'}
function upTime(s){const h=Math.floor(s/3600),m=Math.floor(s/60)%60;return (h?h+' h ':'')+m+' min '+(s%60)+' s'}
function table(id,rows){$(id).innerHTML=rows.map(r=>'<tr><td>'+r[0]+'</td><td'+(r[2]?' class="mono"':'')+'>'+r[1]+'</td></tr>').join('')}

function render(j){
  if(boot!==j.boot){if(boot!==null){LOG.push({t:0,s:'!(screen restarted)'});LV.length=SN.length=MARKS.length=0;runStart=null}boot=j.boot;seenNote=seenMsg=null;snapId=null;if(j.seq<logSeq)logSeq=0}
  S=j;espNow=j.now;
  // link
  if(j.quiet)setLink('Paused: noise','down');else if(j.link)setLink('Nano OK','up');else setLink('No Nano link','down');

  // busy overlay
  const busyText={level:'Leveling...',reach:'Testing boundaries...',balance:'Balancing...',tare:'Taring...'}[j.busy];
  $('#busy').classList.toggle('show',!!busyText&&!(curTab=='level'&&liveMode!='off'));
  if(busyText){$('#busyText').textContent=busyText;$('#busyCancel').style.display=j.busy=='tare'?'none':''}
  levelBusy(busyText,j.busy);

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
  $('#laserNote').textContent=j.laserOffIn>0?'Laser switches off in '+j.laserOffIn+' s.':'';

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

  // log (+ live data parsed from it)
  if(j.log.length){
    const added=j.log.map((s,i)=>({s,t:j.lt[i]}));
    added.forEach(ingest);
    LOG.push(...added);if(LOG.length>LOG_MAX)LOG.splice(0,LOG.length-LOG_MAX);drawLog(false,added)}
  logSeq=j.seq;
  snapResult(j);
  drawLive();drawWheel();drawReport();

  // info
  table('#infoLink',[['State',j.quiet?'<span class="badc">Paused: noise on RX</span>':j.link?'<span class="okc">Nano OK</span>':'<span class="badc">No link</span>'],
    ['Last heard',ago(j.heard)],['Bytes received',j.rx],['Bad lines',j.bad],['Commands queued',j.txq],['Nano firmware',j.nv?'V'+esc(j.nv):'unknown (send Status)'],
    ['Debug streams on',['Level','Sensor','Scale'].filter((n,i)=>j.streams[i]).join(', ')||'none']]);
  table('#infoScreen',[['Screen firmware','V'+esc(j.fw)],['Running for',upTime(j.up)],['Free memory',Math.round(j.heap/1024)+' KB (lowest '+Math.round(j.minHeap/1024)+' KB)'],
    ['Phones connected',j.sta],['WiFi','TireBalancer (open), 192.168.4.1']]);
  const rows=statusRows(j.status);
  $('#infoStatus').innerHTML=rows.length?rows.map(r=>r.length==1?'<tr><th colspan="2">'+r[0]+'</th></tr>':'<tr><td>'+r[0]+'</td><td>'+esc(r[1])+'</td></tr>').join('')
    :'<tr><td>Not read yet - press Read again (the Nano answers when idle).</td></tr>';
}
// The Nano's Status lines, one setting per row: [group] or [label, value]
function statusRows(lines){
  const out=[], find=re=>{for(const l of lines){const m=re.exec(l);if(m)return m}return null};
  const N=(v,d)=>{const x=parseFloat(v);return isFinite(x)?(d==null?String(x):x.toFixed(d)):v}, onoff=v=>v=='1'?'On':'Off';
  const group=(title,items)=>{const r=items.filter(x=>x);if(r.length){out.push([title]);out.push(...r)}};
  let m;
  group('Leveling',[
    (m=find(/^PitchOffset (\S+) RollOffset (\S+)/))&&['Pitch offset',N(m[1],2)+'°'], m&&['Roll offset',N(m[2],2)+'°'],
    (m=find(/^MinSpeed (\S+) MaxSpeed (\S+)/))&&['Min speed',N(m[1])+' PWM'], m&&['Max speed',N(m[2])+' PWM'],
    (m=find(/^Gyro (\S+) Ramp (\S+)/))&&['Ramp',N(m[2])+' PWM/s'], m&&['Gyro blending',N(m[1],2)],
    (m=find(/^Alpha (\S+) LevelTol (\S+)/))&&['Level tolerance',N(m[2],2)+'°'], m&&['Alpha (angle smoothing)',N(m[1],2)],
  ]);
  group('Leveling gains, no load',[
    (m=find(/^Kp (\S+) Ki (\S+) Kd (\S+)/))&&['Kp',N(m[1])], m&&['Ki',N(m[2])], m&&['Kd',N(m[3])],
  ]);
  m=find(/^At (\S+) kg: Kp (\S+) Ki (\S+) Kd (\S+) MinSpeed (\S+)/);
  group(m?'Leveling gains at '+N(m[1])+' kg (Mass Comp)':'Leveling gains, loaded',[
    m&&['Kp',N(m[2])], m&&['Ki',N(m[3])], m&&['Kd',N(m[4])], m&&['Min speed',N(m[5])+' PWM'],
  ]);
  m=find(/^Reach (\S+) (\S+) (\S+) (\S+)/);
  group('Reach area (corner angles c1, c2)',m?[1,2,3,4].map(i=>['Corner '+i,m[i].replace(',',', ')+'°']):[]);
  m=find(/^Axes top (\S+) base (\S+)/);
  group('MPU mounting',[m&&['Platform (TopAxisX, Y)',m[1].replace(',',', ')], m&&['Frame (BaseAxisX, Y)',m[2].replace(',',', ')]]);
  group('Load cells',[
    (m=find(/^Cal (\S+) (\S+) (\S+)/))&&['Calibration A (210°)',m[1]], m&&['Calibration B (90°)',m[2]], m&&['Calibration C (330°)',m[3]],
    (m=find(/^Zero (\S+) (\S+) (\S+)/))&&['Factory zero A',m[1]], m&&['Factory zero B',m[2]], m&&['Factory zero C',m[3]],
    (m=find(/TestMass (\S+)/))&&['Test mass',N(m[1],2)+' kg'],
  ]);
  group('Laser',[(m=find(/^LaserOffset (\S+)/))&&['Laser offset',N(m[1],1)+'°']]);
  m=find(/^Tire mm RimD (\S+) TreadW (\S+) SideWall (\S+) WeightR (\S+)/);
  group('Tire (as the Nano has it)',m?[['Rim diameter',m[1]+' mm'],['Tread width',m[2]+' mm'],['Sidewall (aspect ratio)',m[3]+' %'],['Weight radius',m[4]+' mm']]:[]);
  m=find(/^AutoStop (\S+) MassComp (\S+) TestMassOn (\S+)/);
  group('Switches',m?[['Auto Stop',onoff(m[1])],['Mass Comp',onoff(m[2])],['Test Weight',onoff(m[3])]]:[]);
  return out;
}
// While leveling and the live view is open, show progress on the Level tab
// instead of covering the charts with the busy pop-up.
function levelBusy(text,busy){
  let el=$('#lvBusy');
  if(!el){el=document.createElement('div');el.id='lvBusy';el.className='row';el.style.marginTop='10px';
    el.innerHTML='<span class="spin" style="width:22px;height:22px;border-width:3px;margin:0"></span><b class="grow" id="lvBusyT"></b><button class="btn danger sm" id="lvBusyC">Cancel</button>';
    $('#t-level .card').appendChild(el);$('#lvBusyC').onclick=()=>$('#busyCancel').click()}
  const show=!!text&&curTab=='level'&&liveMode!='off';
  el.style.display=show?'':'none';if(show){$('#lvBusyT').textContent=text;$('#lvBusyC').style.display=busy=='tare'?'none':''}
}

window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change',()=>{bubInit=false;drawLive();drawWheel()});
window.addEventListener('resize',()=>{drawLive()});
showTab(load('tab')||'level');
setAck(false);
poll();
</script>
</body></html>)HTML";
