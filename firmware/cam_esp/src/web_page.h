#pragma once
#include <Arduino.h>

const char WEB_PAGE[] PROGMEM = R"BBWEB(<!doctype html>
<html lang="vi"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>BalanceBot · Điều khiển</title>
<style>
:root{--bg:#f2f5f4;--ink:#142a2b;--muted:#607573;--line:#d9e3df;--green:#12775c;--red:#b83b38;--shadow:0 8px 30px #17352b08}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.5 system-ui,-apple-system,Segoe UI,sans-serif}button,input{font:inherit}button{cursor:pointer;border:1px solid var(--line);border-radius:13px;background:#fff;color:var(--ink);padding:12px 18px;font-weight:650;touch-action:manipulation}button:disabled{opacity:.4;cursor:not-allowed}button:focus-visible,input:focus-visible,summary:focus-visible{outline:3px solid #2eac85;outline-offset:4px}button:active:not(:disabled),button.held{transform:translateY(1px);background:#d9f3e7;border-color:var(--green)}button.primary{background:var(--green);color:white;border-color:var(--green)}button.danger{background:var(--red);color:white;border-color:var(--red)}button.small{padding:9px 13px;font-size:13px}
.wrap{max-width:1040px;margin:auto;padding:24px 24px 42px}.top{display:flex;align-items:center;justify-content:space-between;gap:16px}.brand{font-size:24px;font-weight:800;letter-spacing:-.8px}.brand span{color:var(--green)}.eyebrow{color:var(--muted);font-size:11px;letter-spacing:1.8px;font-weight:750;text-transform:uppercase}.badge{border-radius:30px;padding:7px 13px;background:#e5ebe8;color:#526962;font-size:12px;font-weight:700;white-space:nowrap}.badge.good{background:#dff3e8;color:#11633f}.badge.bad{background:#fae4e3;color:#a43230}.toolbar{display:flex;align-items:center;justify-content:space-between;gap:16px;margin:22px 0 16px;padding:18px 20px;background:#fff;border:1px solid var(--line);border-radius:18px;box-shadow:var(--shadow)}.toolbar strong{font-size:18px}.sub{font-size:12px;color:var(--muted);margin-top:3px}.actions{display:flex;gap:10px;flex-wrap:wrap}.message{font-size:13px;min-height:24px;margin:0 2px 14px;color:var(--muted)}.grid{display:grid;grid-template-columns:1.05fr 1fr;gap:18px}.card{background:white;border:1px solid var(--line);border-radius:20px;padding:22px;box-shadow:var(--shadow)}h2{font-size:16px;margin:0 0 4px}p{margin:0}.hint{color:var(--muted);font-size:12px}.pad{position:relative;width:280px;height:280px;border-radius:50%;margin:24px auto;background:radial-gradient(circle,#f7faf8 37%,#edf3f0 38%,#edf3f0 39%,#f7faf8 40%);border:1px solid var(--line);box-shadow:inset 0 0 0 10px #fff;user-select:none}.pad button{position:absolute;width:76px;height:67px;padding:7px;border-radius:16px;touch-action:none;user-select:none;-webkit-user-select:none}.pad button b{display:block;font-size:23px;line-height:1.1}.pad button span{font-size:12px}.pad .up{left:101px;top:14px}.pad .down{left:101px;bottom:14px}.pad .left{left:13px;top:106px}.pad .right{right:13px;top:106px}.pad .stop{left:102px;top:107px;width:74px;height:65px;border-radius:50%;font-size:11px;background:#223e38;color:white;border:0}.slider{margin-top:16px}.slider label{display:flex;justify-content:space-between;gap:10px;font-size:13px}.slider output{font-weight:750;color:var(--green)}input[type=range]{width:100%;accent-color:var(--green);margin:13px 0 0;min-height:24px}.spin{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin:24px 0 20px}.spin button{min-height:126px;touch-action:none;user-select:none;-webkit-user-select:none}.spin b{display:block;font-size:42px;line-height:1.25;color:var(--green)}.spin span{display:block;font-size:13px}.spin small{font-size:11px;color:var(--muted)}.metrics{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;margin-top:20px}.metric{background:#f5f8f6;padding:13px 10px;border-radius:12px}.metric small{font-size:11px;color:var(--muted)}.metric strong{display:block;font-size:19px;margin-top:3px;font-variant-numeric:tabular-nums}.metric em{font-style:normal;font-size:10px;color:var(--muted)}.tools{display:flex;gap:9px;margin-top:16px;flex-wrap:wrap}.settings{margin-top:18px;padding:0;overflow:hidden}summary{list-style:none;cursor:pointer;display:flex;justify-content:space-between;align-items:center;padding:20px 22px;font-size:15px;font-weight:700}summary::-webkit-details-marker{display:none}summary span{font-size:12px;font-weight:500;color:var(--muted)}.settings[open] summary{border-bottom:1px solid var(--line)}.settings-body{padding:20px 22px}.note{padding:12px 14px;background:#fff6df;border:1px solid #eddfb4;border-radius:12px;font-size:12px;margin-bottom:18px;color:#735619}fieldset{margin:0 0 20px;padding:0;border:0}legend{font-size:13px;font-weight:750;margin-bottom:12px}.fields{display:grid;grid-template-columns:repeat(3,1fr);gap:14px}.field label{display:block;font-size:12px;font-weight:700;margin-bottom:5px}.field input{width:100%;border:1px solid #cddbd4;padding:10px;border-radius:9px;color:var(--ink);background:#fff}.field small{display:block;font-size:11px;color:var(--muted);margin-top:5px;min-height:30px}.checks{display:flex;flex-wrap:wrap;gap:16px;margin:6px 0 20px;font-size:12px}.checks input{accent-color:var(--green)}.foot{margin-top:18px;font-size:11px;color:var(--muted);display:flex;justify-content:space-between;gap:15px}.save-info{font-size:12px;color:var(--muted);margin-top:12px}a{color:var(--green)}
.joy-card{grid-column:1/-1}.stick{position:relative;width:250px;height:250px;border-radius:50%;margin:20px auto;background:radial-gradient(circle,#f7faf8 60%,#edf3f0 61%);border:1px solid var(--line);box-shadow:inset 0 0 0 10px #fff;touch-action:none;user-select:none;-webkit-user-select:none}.stick .ring{position:absolute;left:50%;top:50%;width:100px;height:100px;transform:translate(-50%,-50%);border-radius:50%;border:2px dashed #cfe0d8}.stick .knob{position:absolute;left:50%;top:50%;width:94px;height:94px;transform:translate(-50%,-50%);border-radius:50%;background:#fff;border:1px solid var(--line);box-shadow:0 6px 18px #17352b22;display:flex;align-items:center;justify-content:center;color:var(--green);font-size:26px;transition:transform .09s ease-out}.stick.active .knob{transition:none}.stick.off{opacity:.4;pointer-events:none}
@media(max-width:700px){.stick{width:220px;height:220px}.stick .knob{width:84px;height:84px}}
@media(max-width:700px){.wrap{padding:16px 14px 28px}.grid{grid-template-columns:1fr}.toolbar{padding:14px;align-items:flex-start}.toolbar strong{font-size:16px}.actions{gap:7px}.toolbar button{padding:11px 13px}.brand{font-size:22px}.card{padding:18px}.pad{margin:18px auto}.fields{grid-template-columns:1fr 1fr}.settings{padding:0}.settings-body{padding:16px}.foot{flex-direction:column;gap:4px}.spin{margin:18px 0}.spin button{min-height:105px}summary{padding:18px}.metrics{margin-top:16px}}
@media(max-width:350px){.pad{transform:scale(.9);margin:-1px auto;width:280px}.card{padding:12px}.fields{grid-template-columns:1fr}.toolbar{flex-wrap:wrap}}
</style></head><body><main class="wrap">
<header class="top"><div><div class="eyebrow">Bảng điều khiển xe cân bằng</div><div class="brand">Balance<span>Bot</span></div></div><span id="connection" class="badge">Đang kết nối…</span></header>
<section class="toolbar"><div><strong id="mode">Đang đọc trạng thái</strong><div id="armNote" class="sub">Chờ dữ liệu từ xe</div></div><div class="actions"><button id="arm" class="primary" disabled>ARM</button><button id="disarm" class="danger" disabled>DISARM</button></div></section>
<p id="message" class="message" role="status" aria-live="polite">Kết nối Wi-Fi của xe và giữ trang này ở phía trước.</p>
<div class="grid">
<section class="card"><h2>Di chuyển</h2><p class="hint">Giữ nút để chạy · Nhả nút để giảm tốc dừng</p>
<div class="pad" aria-label="Vùng tròn điều khiển bốn hướng">
<button class="up drive" data-direction="forward" aria-label="Tiến" disabled><b>↑</b><span>Tiến</span></button>
<button class="left drive" data-direction="left" aria-label="Rẽ trái theo cung" disabled><b>←</b><span>Trái</span></button>
<button class="stop" id="stop" disabled>DỪNG</button>
<button class="right drive" data-direction="right" aria-label="Rẽ phải theo cung" disabled><b>→</b><span>Phải</span></button>
<button class="down drive" data-direction="backward" aria-label="Lùi" disabled><b>↓</b><span>Lùi</span></button></div>
<p class="hint">Trái / phải: tiến chậm và rẽ theo cung. Xe hai bánh không đi ngang.</p>
<div class="slider"><label for="speed">Tốc độ tiến / lùi <output id="speedOut">120 tick/s</output></label><input id="speed" type="range" min="50" max="3600" step="10" value="120"></div>
</section>
<section class="card"><h2>Xoay tại chỗ</h2><p class="hint">Không đặt tốc độ tiến / lùi khi xoay</p>
<div class="spin"><button class="drive" data-direction="ccw" disabled><b>↺</b><span>Ngược chiều kim đồng hồ</span><small>Xoay trái</small></button><button class="drive" data-direction="cw" disabled><b>↻</b><span>Thuận chiều kim đồng hồ</span><small>Xoay phải</small></button></div>
<div class="slider"><label for="turn">Tốc độ rẽ / xoay <output id="turnOut">15°/s</output></label><input id="turn" type="range" min="5" max="90" step="1" value="15"></div>
<div class="metrics"><div class="metric"><small>Góc nghiêng</small><strong id="pitch">—</strong><em>độ</em></div><div class="metric"><small>Tốc độ bánh</small><strong id="velocity">—</strong><em>tick/s</em></div><div class="metric"><small>Sai số hướng</small><strong id="yawError">—</strong><em>độ</em></div></div>
<div class="tools"><button id="zero" class="small" disabled>Lấy mốc hiện tại</button><button id="calibrate" class="small" disabled>Calib lại MPU</button></div>
<p class="hint" style="margin-top:10px">Calib khi Disarm và xe bất động. Disarm tắt motor; hãy đỡ xe.</p>
</section>
<section class="card joy-card"><h2>Cần điều hướng đa chiều</h2><p class="hint">Kéo cần theo hướng muốn đi · Thả tay hoặc đẩy về tâm để dừng</p>
<div class="stick" id="stick" aria-label="Cần điều khiển đa hướng"><div class="ring"></div><div class="knob" id="stickKnob"><b>✥</b></div></div>
<p class="hint">Lên/xuống: tiến/lùi · Trái/phải: rẽ theo cung · Chéo: kết hợp cả hai. Đẩy hết cỡ = trần tốc độ (3600 tick/s) và trần xoay (90°/s); vùng giữa cần là vùng chết.</p>
</section></div>
<details class="card settings"><summary>Thông số điều khiển <span>Chỉnh khi Disarm ＋</span></summary><div class="settings-body">
<div class="note">Đổi từng thông số một. “Áp dụng” đổi bộ số đang chạy; “Lưu khởi động” lưu bộ số đã áp dụng qua lần tắt nguồn. Mọi thao tác chỉnh / lưu yêu cầu Disarm.</div>
<form id="settingsForm"><div id="fieldGroups"></div><div class="checks"><label><input type="checkbox" name="positionOn"> Giữ vị trí</label><label><input type="checkbox" name="headingOn"> Giữ hướng MPU</label><label><input type="checkbox" name="autoArmBoot"> Tự Arm sau calib / khởi động</label></div>
<div class="actions"><button id="apply" type="submit" class="primary" disabled>Áp dụng</button><button id="save" type="button" disabled>Lưu khởi động</button><button id="reload" type="button" disabled>Đọc lại từ xe</button><button id="defaults" type="button" disabled>Khôi phục mặc định</button></div><p id="saveInfo" class="save-info">Chưa đọc thông số.</p></form>
</div></details>
<footer class="foot"><span>Wi-Fi cục bộ · Không cần Internet · Web v2</span><span>Mất lệnh giữ nút 600 ms → giảm tốc và giữ cân bằng</span></footer>
</main><script>
'use strict';
const $=id=>document.getElementById(id), form=$('settingsForm');
const groups=[['Cân bằng — bộ số mới nhất',[
 ['kx','kx · Vị trí','0.000001','lệnh/tick · kéo xe về mốc'],['kv','kv · Tốc độ','0.00001','lệnh/(tick/s) · phản hồi tốc độ'],
 ['ka','ka · Góc pitch','0.001','lệnh/độ · Ktheta = −ka'],['kg','kg · Gyro X','0.00001','lệnh/(độ/s) · Komega = −kg'],
 ['balanceOffset','Offset cân bằng','0.05','độ · góc đứng cân bằng']]],
 ['Giữ hướng và motor',[
 ['headingP','P hướng','0.001','lệnh/độ · kéo về hướng mục tiêu'],['headingD','D hướng','0.0001','lệnh/(độ/s) · dập tốc độ xoay'],
 ['yawLimit','Giới hạn lệnh quay','0.01','0–0.3 · cân bằng vẫn ưu tiên'],['deadzoneL','Vùng chết trái','0.0001','0–0.5 · quá cao có thể gây giật'],
 ['deadzoneR','Vùng chết phải','0.0001','0–0.5 · chỉnh riêng bên phải'],['trimL','Trim trái','0.01','0.5–1.5 · nhân lệnh motor'],['trimR','Trim phải','0.01','0.5–1.5 · nhân lệnh motor']]],
 ['Chuyển động — tăng / giảm tốc mềm',[
 ['maxSpeed','Trần tốc độ bánh','10','tick/s · giới hạn thanh tốc độ'],['maxYawRate','Trần tốc độ xoay','1','độ/s · giới hạn thanh xoay'],
 ['speedAccel','Gia tốc tiến / lùi','10','tick/s² · giảm để khởi hành êm'],['speedDecel','Giảm tốc tiến / lùi','10','tick/s² · tăng để dừng nhanh hơn'],
 ['yawAccel','Gia tốc xoay','5','độ/s² · giảm nếu bắt đầu giật'],['yawDecel','Giảm tốc xoay','5','độ/s² · giảm nếu dừng xoay lắc'],
 ['positionLead','Giới hạn mốc vị trí','10','tick · tránh mốc chạy quá xa'],['headingLead','Giới hạn mốc hướng','1','độ · tránh tích lũy yêu cầu xoay']]]];
for(const [title,fields] of groups){const fieldset=document.createElement('fieldset'),legend=document.createElement('legend'),grid=document.createElement('div');legend.textContent=title;grid.className='fields';fieldset.append(legend,grid);
 for(const [key,label,step,hint] of fields){const box=document.createElement('div');box.className='field';const l=document.createElement('label'),input=document.createElement('input'),small=document.createElement('small');l.textContent=label;l.htmlFor=key;input.id=key;input.name=key;input.type='number';input.step='any';input.dataset.suggestedStep=step;input.required=true;input.disabled=true;small.textContent=hint;box.append(l,input,small);grid.append(box);}
 $('fieldGroups').append(fieldset);}
let token='',boot='',seq=0,state=null,online=false,lastSeen=0,held=null,driveBusy=false,pollBusy=false,pingBusy=false,claimBusy=false;
let loadedRevision=0,dirty=false,refreshAfterApply=false,requestBusy=false;
let stickPointer=null,stickActive=false;
const say=text=>{$('message').textContent=text;};
async function api(path,data=null,timeout=900){
 const ctrl=new AbortController(),timer=setTimeout(()=>ctrl.abort(),timeout);
 try{const response=await fetch(path,{method:data?'POST':'GET',body:data?new URLSearchParams(data):undefined,cache:'no-store',signal:ctrl.signal});
 const result=await response.json();if(!response.ok){const err=new Error(result.error||'Yêu cầu bị từ chối');err.status=response.status;throw err;}return result;
 }finally{clearTimeout(timer);}
}
function auth(extra={}){return {boot,token,seq:++seq,epoch:state?state.epoch:0,issued:state?state.at:0,...extra};}
function lockUi(){
 const own=online&&!!token,editable=own&&state&&!state.armed&&!state.autoArm&&!requestBusy;
 $('arm').disabled=!own||!state||state.armed||!state.ready||requestBusy;
 $('disarm').disabled=!online;
 for(const b of document.querySelectorAll('.drive'))b.disabled=!own||!state||!state.armed||requestBusy||(!state.headingOn&&['left','right','cw','ccw'].includes(b.dataset.direction));
 $('stick').classList.toggle('off',!own||!state||!state.armed||requestBusy);
 $('stop').disabled=!own;$('zero').disabled=!own||!state||state.moving||requestBusy;$('calibrate').disabled=!editable;
 for(const e of form.elements)e.disabled=!editable;
 $('reload').disabled=!online||requestBusy;
 $('save').disabled=!editable||dirty;
 $('speed').disabled=!!held;$('turn').disabled=!!held;
}
function dropHold(send=true){
 const old=held;held=null;document.querySelectorAll('.held').forEach(e=>e.classList.remove('held'));
 if(old&&old.stick)stickReset();
 if(old&&send&&token){const data=auth({action:'stop',epoch:old.epoch});api('/api/action',data,650).catch(()=>{});}
 lockUi();
}
function lost(text){dropHold(true);online=false;say(text);$('connection').textContent='Mất kết nối';$('connection').className='badge bad';lockUi();}
function showState(s){
 if(boot&&boot!==s.boot){dropHold(false);token='';seq=0;loadedRevision=0;}
 boot=s.boot;state=s;online=true;lastSeen=performance.now();
 if(held&&(!s.armed||held.epoch!==s.epoch))dropHold(false);
 $('mode').textContent=s.armed?(s.moving?'Đang di chuyển / giảm tốc':'Đang giữ cân bằng'):(s.autoArm?'Đang chờ tự Arm':'Đã Disarm');
 $('armNote').textContent=s.autoArm?('Giữ thẳng '+((s.armHold||700)/1000)+' s để tự Arm'):(s.armed?'Nhả nút để dừng chuyển động':'Motor tắt · Có thể chỉnh thông số');
 $('connection').textContent=token?'Đã kết nối':'Chế độ xem';$('connection').className='badge '+(token?'good':'');
 $('pitch').textContent=s.pitch.toFixed(2);$('velocity').textContent=s.v.toFixed(0);$('yawError').textContent=(s.yaw-s.yawRef).toFixed(1);
 if(!requestBusy)say(s.message);
 $('saveInfo').textContent=s.saved?'Bộ số hiện tại đã lưu qua khởi động.':'Bộ số hiện tại ở RAM / mặc định. Bấm Lưu khởi động để lưu lâu dài.';
 if((loadedRevision===0||(refreshAfterApply&&s.revision!==loadedRevision))&&!requestBusy)readSettings().catch(()=>{});
 lockUi();
}
let settingsBusy=false;
async function readSettings(){
 if(settingsBusy)return;settingsBusy=true;
 try{const data=await api('/api/settings');for(const f of data.fields){const e=form.elements.namedItem(f.key);if(!e)continue;
 if(e.type==='checkbox')e.checked=!!f.value;else {e.value=String(f.value);e.min=f.min;e.max=f.max;}}
 loadedRevision=data.revision;dirty=false;refreshAfterApply=false;
 $('speed').max=data.values.maxSpeed;$('turn').max=data.values.maxYawRate;
 $('speed').value=Math.min(Number($('speed').value),data.values.maxSpeed);$('turn').value=Math.min(Number($('turn').value),data.values.maxYawRate);sliderLabels();lockUi();
 }finally{settingsBusy=false;}
}
async function claim(){
 if(claimBusy||token||!boot||document.hidden)return;claimBusy=true;
 try{const r=await api('/api/session',{boot});token=r.token;seq=0;await poll();lockUi();}
 catch(e){if(e.status!==409)say(e.message);}
 finally{claimBusy=false;}
}
async function poll(){
 if(pollBusy||document.hidden)return;pollBusy=true;
 try{showState(await api('/api/state'));}catch{lost('Không đọc được xe. Lệnh chạy sẽ hết hạn và giảm tốc dừng.');}
 finally{pollBusy=false;}
}
async function ping(){
 if(pingBusy||!token||document.hidden)return;pingBusy=true;
 try{await api('/api/ping',{boot,token});}catch(e){dropHold(true);if(e.status===403)token='';lost('Mất phiên điều khiển — hãy kết nối lại và nhấn lại nút.');}
 finally{pingBusy=false;}
}
async function drive(){
 if(!held||driveBusy)return;driveBusy=true;const h=held;
 try{await api('/api/drive',auth({epoch:h.epoch,v:h.v,w:h.w}),500);}
 catch(e){if(held===h){dropHold(true);say('Lệnh chạy bị ngắt: '+e.message);}}
 finally{driveBusy=false;}
}
for(const button of document.querySelectorAll('.drive')){
 button.addEventListener('pointerdown',e=>{
 e.preventDefault();if(button.disabled)return;
 if(held){dropHold(true);return;}
 const v=Number($('speed').value),w=Number($('turn').value),d=button.dataset.direction;
 const map={forward:[v,0],backward:[-v,0],left:[v*.6,w],right:[v*.6,-w],ccw:[0,w],cw:[0,-w]};
 held={pointer:e.pointerId,button,epoch:state.epoch,v:map[d][0],w:map[d][1]};button.classList.add('held');button.setPointerCapture(e.pointerId);lockUi();drive();
 });
 for(const event of ['pointerup','pointercancel','lostpointercapture'])button.addEventListener(event,e=>{if(held&&held.pointer===e.pointerId)dropHold(true);});
 button.addEventListener('contextmenu',e=>e.preventDefault());
}
window.addEventListener('blur',()=>dropHold(true));
const stickEl=$('stick'),knobEl=$('stickKnob'),DEAD=0.18;
function stickMove(e){
 const r=stickEl.getBoundingClientRect(),cx=r.left+r.width/2,cy=r.top+r.height/2;
 let dx=e.clientX-cx,dy=e.clientY-cy;
 const max=r.width/2-20,len=Math.hypot(dx,dy)||1e-6;
 if(len>max){dx=dx/len*max;dy=dy/len*max;}
 knobEl.style.transform=`translate(calc(-50% + ${dx}px),calc(-50% + ${dy}px))`;
 const n=Math.min(len/max,1);if(n<DEAD)return null;
 const t=(n-DEAD)/(1-DEAD),ang=Math.atan2(-dx,-dy);
 const v=Math.round(t*Math.cos(ang)*Number($('speed').max));
 const w=Math.round(t*Math.sin(ang)*Number($('turn').max));
 return [v,w];
}
function stickEnd(send){
 stickPointer=null;stickActive=false;stickEl.classList.remove('active');
 stickReset();
 if(held&&held.stick)dropHold(send);
}
function stickReset(){
 knobEl.style.transform='translate(-50%,-50%)';
}
stickEl.addEventListener('pointerdown',e=>{
 e.preventDefault();if(stickEl.classList.contains('off')||held)return;
 stickPointer=e.pointerId;stickActive=true;stickEl.classList.add('active');
 stickEl.setPointerCapture(e.pointerId);
 const m=stickMove(e);if(m){held={pointer:e.pointerId,button:stickEl,stick:true,epoch:state.epoch,v:m[0],w:m[1]};lockUi();drive();}
});
stickEl.addEventListener('pointermove',e=>{
 if(e.pointerId!==stickPointer||!stickActive)return;
 const m=stickMove(e);
 if(!m){if(held&&held.stick)dropHold(true);return;}
 if(!held){held={pointer:e.pointerId,button:stickEl,stick:true,epoch:state.epoch,v:m[0],w:m[1]};lockUi();drive();}
 else if(held.stick&&(held.v!==m[0]||held.w!==m[1])){held.v=m[0];held.w=m[1];}
});
for(const ev of ['pointerup','pointercancel','lostpointercapture'])stickEl.addEventListener(ev,e=>{
 if(e.pointerId===stickPointer)stickEnd(true);
});
stickEl.addEventListener('contextmenu',e=>e.preventDefault());
document.addEventListener('visibilitychange',()=>{if(document.hidden)dropHold(true);else{poll();ping();}});
window.addEventListener('pagehide',()=>{if(held&&token){const h=held;held=null;navigator.sendBeacon('/api/action',new URLSearchParams(auth({action:'stop',epoch:h.epoch})));}});
window.addEventListener('keydown',e=>{if(e.key==='Escape'){$('stop').click();}});
function sliderLabels(){$('speedOut').textContent=$('speed').value+' tick/s';$('turnOut').textContent=$('turn').value+'°/s';}
$('speed').oninput=sliderLabels;$('turn').oninput=sliderLabels;
async function action(name){
 if(name==='disarm')dropHold(false);else if(name==='stop')dropHold(false);
 if(!online||(!token&&name!=='disarm'))return;
 requestBusy=true;lockUi();say('Đang gửi yêu cầu…');
 try{await api('/api/action',auth({action:name}));say('Đã gửi yêu cầu; chờ trạng thái từ xe.');if(name==='defaults')refreshAfterApply=true;}
 catch(e){say(e.message);}finally{requestBusy=false;lockUi();setTimeout(poll,60);}
}
for(const id of ['arm','disarm','stop','zero','calibrate','save','defaults'])$(id).onclick=()=>action(id);
$('reload').onclick=()=>readSettings().catch(e=>say(e.message));
form.addEventListener('input',()=>{dirty=true;lockUi();});
form.addEventListener('submit',async e=>{
 e.preventDefault();if(!form.reportValidity()||requestBusy)return;
 const values={};for(const el of form.elements)if(el.name)values[el.name]=el.type==='checkbox'?(el.checked?1:0):el.value;
 requestBusy=true;lockUi();
 try{await api('/api/settings',auth(values));refreshAfterApply=true;say('Đã gửi bộ số; chờ xe áp dụng.');}
 catch(err){say(err.message);}finally{requestBusy=false;lockUi();setTimeout(poll,60);}
});
setInterval(drive,100);setInterval(poll,250);setInterval(ping,500);
setInterval(()=>{if(online&&performance.now()-lastSeen>1200)lost('Dữ liệu xe đã cũ — đã nhả điều khiển.');if(!token)claim();},1000);
poll().then(claim);sliderLabels();lockUi();
</script></body></html>
)BBWEB";
