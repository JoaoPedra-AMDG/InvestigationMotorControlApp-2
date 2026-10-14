const $ = id => document.getElementById(id);
const tabs = ['connections','manual','tuning','matrix','results'];
let previousLog = '', previousPrompt = '', busy = false;
function showError(error) { const el=$('error'); el.textContent=error.message||String(error); el.hidden=false; }
function clearError() { $('error').hidden=true; $('error').textContent=''; }
function showTab() {
  const tab=tabs.includes(location.hash.slice(1))?location.hash.slice(1):'connections';
  tabs.forEach(name=>{ $(name).hidden=name!==tab; document.querySelector(`[data-tab="${name}"]`).classList.toggle('active',name===tab); });
  if(tab==='results') refreshResults().catch(showError);
  if(['connections','manual','matrix'].includes(tab)) refreshLive().catch(showError);
}
addEventListener('hashchange',showTab); showTab();
async function get(url) { if(location.protocol==='file:')throw Error('Open this site through the local server. Run start.ps1, then visit http://127.0.0.1:8765.'); const response=await fetch(url,{cache:'no-store'}); if(!response.ok)throw Error((await response.json()).error||'Request failed'); return response.json(); }
async function post(data) { if(location.protocol==='file:')throw Error('Open this site through the local server. Run start.ps1, then visit http://127.0.0.1:8765.'); const response=await fetch('/api/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)}); const body=await response.json(); if(!response.ok)throw Error(body.error||'Request failed'); return body; }
async function command(data) { if(busy)return; busy=true; clearError(); try { const status=await post(data); renderStatus(status); } catch(error){showError(error)} finally{busy=false} }
function renderStatus(status) {
  $('state').textContent=status.prompt?'Action needed':status.running?'Script running':status.exit_code===null?'Idle':status.exit_code===0?'Completed':'Stopped with error';
  const connections=status.connections||{};
  for(const role of ['test','load']){
    const connection=connections[role]||{state:'disconnected',detail:''};
    const label=connection.state==='connected'?'connected':connection.state==='connecting'?'connecting…':connection.state==='error'?'error':'disconnected';
    const chip=$(role+'-connection'); chip.textContent=(role==='test'?'Test: ':'Load: ')+label; chip.dataset.state=connection.state;
    $(role+'-connection-detail').textContent=connection.detail|| (status.running?'USB handed to the test script.':'USB disconnected.');
  }
  const connectionBusy=['test','load'].some(role=>connections[role]?.state==='connecting');
  const anyConnected=['test','load'].some(role=>connections[role]?.open);
  const allConnected=['test','load'].every(role=>connections[role]?.state==='connected');
  $('connect').disabled=status.running||connectionBusy||allConnected;
  $('disconnect').disabled=status.running||connectionBusy||!anyConnected;
  $('clear-errors').disabled=status.running||connectionBusy||!allConnected;
  $('connection-notice').textContent=status.connection_notice||'';
  $('connection-notice').hidden=!status.connection_notice;
  $('run-detail').textContent=status.running?`${status.kind} workflow started ${new Date(status.started).toLocaleString()}`:status.exit_code===null?'No workflow is running.':`Last ${status.kind} workflow exited with code ${status.exit_code}.`;
  const value=[...status.lines,status.partial].filter(Boolean).join('\n');
  if(value!==previousLog){ const log=$('log'); const nearBottom=log.scrollTop+log.clientHeight>=log.scrollHeight-80; log.textContent=value||'No script output yet.'; if(nearBottom)log.scrollTop=log.scrollHeight; previousLog=value; }
  const prompt=status.prompt||''; $('prompt-card').hidden=!prompt; $('prompt-text').textContent=prompt;
  $('answer-ready').hidden=!prompt.includes('type ready');
  $('answer-yes').hidden=prompt.includes('type ready');
  $('answer-no').hidden=prompt.includes('type ready');
  if(prompt && prompt!==previousPrompt)previousPrompt=prompt;
  if(!prompt)previousPrompt='';
  for(const id of ['run-point','run-matrix','run-sequence'])$(id).disabled=status.running;
  $('stop').disabled=!status.running&&!anyConnected;
}
async function refreshStatus() { try{renderStatus(await get('/api/status'))}catch(error){showError(error)} }
$('refresh-status').onclick=refreshStatus;
$('connect').onclick=()=>command({action:'connect'});
$('disconnect').onclick=()=>command({action:'disconnect'});
$('clear-errors').onclick=()=>command({action:'clear_errors'});
$('stop').onclick=()=>command({action:'stop'});
$('answer-yes').onclick=()=>command({action:'answer',value:'yes'});
$('answer-no').onclick=()=>command({action:'answer',value:'no'});
$('answer-ready').onclick=()=>command({action:'answer',value:'ready'});
$('run-point').onclick=()=>command({action:'start',kind:'point',mode:$('point-mode').value,rpm:$('point-rpm').value,amps:$('point-amps').value,bandwidth:$('point-bandwidth').value});
$('run-matrix').onclick=()=>command({action:'start',kind:'matrix',rpms:$('matrix-rpms').value,loads:$('matrix-loads').value,modes:$('matrix-modes').value,bandwidth:$('matrix-bandwidth').value});
$('run-sequence').onclick=()=>command({action:'start',kind:'sequence'});
get('/api/info').then(info=>{ $('test-serial').textContent=info.serials.test||'Not found'; $('load-serial').textContent=info.serials.load||'Not found'; }).catch(showError);
refreshStatus(); setInterval(refreshStatus,500);

function csvRows(text) {
  const rows=[]; let row=[],value='',quoted=false;
  for(let i=0;i<text.length;i++){ const c=text[i]; if(c==='"'){if(quoted&&text[i+1]==='"'){value+='"';i++}else quoted=!quoted;}else if(c===','&&!quoted){row.push(value);value='';}else if((c==='\n'||c==='\r')&&!quoted){if(c==='\r'&&text[i+1]==='\n')i++;row.push(value);if(row.some(x=>x!==''))rows.push(row);row=[];value='';}else value+=c; }
  if(value||row.length){row.push(value);rows.push(row)} return rows;
}
function plot(canvas, rows, headers, keys, scales, labels) {
  const width=canvas.clientWidth, height=240, dpr=devicePixelRatio||1;
  canvas.width=Math.round(width*dpr);canvas.height=Math.round(height*dpr);
  const ctx=canvas.getContext('2d');ctx.scale(dpr,dpr);ctx.fillStyle='#fff';ctx.fillRect(0,0,width,height);
  const indexes=keys.map(k=>headers.indexOf(k)), time=headers.indexOf('time_s');
  const colors=['#285a70','#c27831','#2e746c'];
  if(!canvas.dataset.legendReady){labels.forEach((label,k)=>{if(indexes[k]<0)return;const legend=document.createElement('span');legend.textContent='● '+label+'  ';legend.style.color=colors[k];canvas.before(legend)});canvas.dataset.legendReady='true'}
  if(time<0||indexes.every(i=>i<0)){ctx.fillStyle='#526973';ctx.fillText('These channels are not in the CSV.',18,45);return}
  const values=[];for(const row of rows){const t=Number(row[time]);if(!Number.isFinite(t))continue;for(let k=0;k<indexes.length;k++){const i=indexes[k];if(i<0||row[i]==null||row[i]==='')continue;const y=Number(row[i])*scales[k];if(Number.isFinite(y))values.push({t,y,k})}}
  if(!values.length){ctx.fillStyle='#526973';ctx.font='13px Segoe UI';ctx.fillText('Waiting for live readings',18,45);return}
  const minT=Math.min(...values.map(p=>p.t)),maxT=Math.max(...values.map(p=>p.t));
  let minY=Math.min(...values.map(p=>p.y)),maxY=Math.max(...values.map(p=>p.y)); if(maxY===minY){minY-=1;maxY+=1}
  const left=55,right=width-12,top=20,bottom=height-32;
  const x=t=>left+(t-minT)/Math.max(.000001,maxT-minT)*(right-left), y=v=>bottom-(v-minY)/(maxY-minY)*(bottom-top);
  ctx.strokeStyle='#e4eceb';ctx.fillStyle='#576f76';ctx.font='11px Segoe UI';
  for(let i=0;i<4;i++){const val=minY+(maxY-minY)*i/3;ctx.beginPath();ctx.moveTo(left,y(val));ctx.lineTo(right,y(val));ctx.stroke();ctx.fillText(val.toFixed(2),4,y(val)+4)}
  ctx.fillText(minT.toFixed(3)+' s',left,bottom+19);ctx.fillText(maxT.toFixed(3)+' s',right-48,bottom+19);
  for(let k=0;k<keys.length;k++){if(indexes[k]<0)continue;ctx.strokeStyle=colors[k];ctx.lineWidth=1.4;ctx.beginPath();let began=false;for(const p of values){if(p.k!==k)continue;if(!began){ctx.moveTo(x(p.t),y(p.y));began=true}else ctx.lineTo(x(p.t),y(p.y))}ctx.stroke()}
}
async function preview(file) {
  clearError(); const title=file.path.split('/').pop(); $('preview-title').textContent=title;
  const target=$('preview');target.replaceChildren();const url='/result?path='+encodeURIComponent(file.path);
  const download=document.createElement('a');download.href=url;download.textContent='Open or download original file';download.className='preview-link';target.append(download);
  if(file.path.endsWith('.svg')){const img=document.createElement('img');img.src=url;img.alt=title;target.append(img);return}
  if(file.path.endsWith('.csv')){const text=await (await fetch(url)).text();const all=csvRows(text);const headers=all.shift()||[];const stride=Math.max(1,Math.ceil(all.length/2500)),rows=all.filter((_,i)=>i%stride===0);const summary=document.createElement('p');summary.textContent=`${all.length} original samples · showing every ${stride} sample(s)`;target.append(summary);for(const [name,keys,scales,labels] of [['Position',['position_turn','estimated_position_turn','measured_position_turn'],[1,1,1],['Position','Estimated','Measured']],['Speed',['velocity_turn_s'],[60],['Speed rpm']],['Three-phase currents',['phase_a_A','phase_b_A','phase_c_A'],[1,1,1],['Phase A','Phase B','Phase C']]]){const h=document.createElement('h3');h.textContent=name;target.append(h);const group=document.createElement('div');target.append(group);const canvas=document.createElement('canvas');group.append(canvas);plot(canvas,rows,headers,keys,scales,labels)}return}
  const text=await(await fetch(url)).text();const pre=document.createElement('pre');pre.textContent=text.slice(0,30000);target.append(pre);
}
let manualRuns=[],spectrumRange='low_frequency',spectrumRequest=0;
function renderSpectrumImage() {
  const run=manualRuns.find(item=>item.folder===$('manual-spectrum-run').value);
  if(!run)return;
  const url='/result?path='+encodeURIComponent(`${run.folder}/pair_psd_${spectrumRange}.svg`);
  $('manual-spectrum-plot').src=url;
  $('spectrum-open').href=url;
  $('spectrum-low').setAttribute('aria-pressed',String(spectrumRange==='low_frequency'));
  $('spectrum-full').setAttribute('aria-pressed',String(spectrumRange==='full_range'));
}
async function displayManualSpectrum() {
  const run=manualRuns.find(item=>item.folder===$('manual-spectrum-run').value);
  if(!run)return;
  const request=++spectrumRequest;
  const analysis=await get('/result?path='+encodeURIComponent(`${run.folder}/analysis.json`));
  if(request!==spectrumRequest)return;
  const tbody=$('spectrum-metrics').querySelector('tbody');tbody.replaceChildren();
  const fmt=value=>Number.isFinite(Number(value))?Number(value).toExponential(3):'—';
  const fundamentals=[];
  let resolutionWarning=false;
  for(const role of ['test','load']){
    const phases=analysis.boards?.[role]?.phases||{};
    const first=phases.a;
    if(first)fundamentals.push(`${role} ${Number(first.expected_fundamental_Hz).toFixed(1)} Hz (band ${first.band_Hz.map(v=>Number(v).toFixed(1)).join('–')} Hz)`);
    for(const phase of ['a','b','c']){
      const metric=phases[phase];if(!metric)continue;
      resolutionWarning ||= !metric.trustworthy_fundamental_comparison;
      const tr=document.createElement('tr');
      for(const value of [`${role} / ${phase.toUpperCase()}`,fmt(metric.in_band_variance_A2),fmt(metric.out_of_band_variance_A2),
        metric.in_band_fraction==null?'—':(100*metric.in_band_fraction).toFixed(1)+'%',
        metric.in_to_out_ratio_dB!=null&&Number.isFinite(Number(metric.in_to_out_ratio_dB))?Number(metric.in_to_out_ratio_dB).toFixed(1):'—']){
        const td=document.createElement('td');td.textContent=value;tr.append(td);
      }
      tbody.append(tr);
    }
  }
  $('spectrum-note').textContent=`Electrical fundamental: ${fundamentals.join('; ')}. The green band in each plot is compared with all other positive frequencies.${resolutionWarning?' Frequency resolution is insufficient for a reliable in/out comparison.':''}`;
  $('manual-spectrum-empty').hidden=true;
  $('manual-spectrum-content').hidden=false;
  renderSpectrumImage();
}
async function refreshManualSpectra(runs) {
  const select=$('manual-spectrum-run'),previous=select.value;
  manualRuns=runs;select.replaceChildren();select.disabled=!runs.length;
  if(!runs.length){const option=document.createElement('option');option.textContent='No completed manual captures yet';select.append(option);$('manual-spectrum-empty').hidden=false;$('manual-spectrum-content').hidden=true;return}
  for(const run of runs){const option=document.createElement('option');option.value=run.folder;option.textContent=`${new Date(run.captured_at).toLocaleString()} · ${run.mode} · ${run.rpm} rpm · ${run.amps} A`;select.append(option)}
  select.value=runs.some(run=>run.folder===previous)?previous:runs[0].folder;
  await displayManualSpectrum();
}
$('manual-spectrum-run').onchange=()=>displayManualSpectrum().catch(showError);
$('spectrum-low').onclick=()=>{spectrumRange='low_frequency';renderSpectrumImage()};
$('spectrum-full').onclick=()=>{spectrumRange='full_range';renderSpectrumImage()};
async function refreshResults() {
  const [files,runs]=await Promise.all([get('/api/results'),get('/api/manual-spectra')]);
  await refreshManualSpectra(runs);
  const list=$('result-files');list.replaceChildren();
  if(!files.length){list.textContent='No result files yet.';return}
  let lastFolder='';for(const file of files){const folder=file.path.includes('/')?file.path.slice(0,file.path.lastIndexOf('/')):'results';if(folder!==lastFolder){const heading=document.createElement('div');heading.className='folder';heading.textContent=folder;list.append(heading);lastFolder=folder}const button=document.createElement('button');button.textContent=file.path.split('/').pop();button.title=file.path;button.onclick=()=>preview(file).catch(showError);list.append(button)}
}
$('refresh-results').onclick=()=>refreshResults().catch(showError);

async function refreshLive() {
  const visible=['connections','manual','matrix'].filter(id=>!$(id).hidden);
  if(!visible.length)return;
  const data=await get('/api/live');
  const start=data.rows.length?Number(data.rows[0].host_monotonic_s):0;
  if(!$('connections').hidden){
    const age=data.sampled_at?(Date.now()-Date.parse(data.sampled_at))/1000:null;
    $('monitor-source').textContent=data.source?`${data.source}${age!==null?` · last reading ${Math.max(0,age).toFixed(1)} s ago`:''}${age!==null&&age>4?' · readings may be stale':''}`:'Connect both motors to see live readings.';
    for(const role of ['test','load']){
      const sample=data.connections&&data.connections[role]?.state!=='connected'?undefined:[...data.rows].reverse().find(row=>row.boards?.[role])?.boards[role];
      const target=$(role+'-monitor-metrics');
      const axisState=sample?.state==null?'—':sample.state===1?'IDLE (1)':sample.state===8?'Closed loop (8)':String(sample.state);
      const errorText=value=>value==null?'—':Number(value)===0?'None (0x0)':'0x'+Number(value).toString(16);
      const metrics=[['Axis state',axisState],['Armed',sample?.armed==null?'—':sample.armed?'Yes':'No'],['Position · turns',sample?.position],['Speed · rpm',sample?.rpm],['Iq · A',sample?.iq],['Id · A',sample?.id],['Phase A · A',sample?.phase_a],['Phase B · A',sample?.phase_b],['Phase C · A',sample?.phase_c],['Bus voltage · V',sample?.vbus],['Bus current · A',sample?.ibus],['FET temperature · °C',sample?.fet_C],['Active errors',errorText(sample?.errors)],['Last disarm',errorText(sample?.disarm)]];
      if(target.children.length!==metrics.length){
        target.replaceChildren();
        for(const [label] of metrics){const item=document.createElement('div');item.className='metric';const name=document.createElement('span');name.textContent=label;item.append(name,document.createElement('strong'));target.append(item)}
      }
      metrics.forEach(([label,value],index)=>{target.children[index].querySelector('strong').textContent=typeof value==='number'?(Number.isFinite(value)?value.toFixed(label==='Axis state'?0:2):'—'):(value??'—')});
    }
  }
  for(const id of visible){
    const section=$(id),container=section.querySelector('.live-charts');
    section.querySelector('.live-source').textContent=data.source?`${data.source}. These readings are sequential USB polls, not synchronized high-rate captures.`:'Connect both motors or start a test to see live readings.';
    if(!container.children.length){
      for(const role of ['test','load']){
        const card=document.createElement('div');card.className='live-board';card.dataset.role=role;
        const heading=document.createElement('h3');heading.textContent=role==='test'?'Test motor · opposing torque':'Load motor · velocity control';card.append(heading);
        for(const title of ['Position · turns','Speed · rpm','Three-phase voltage · V (commanded, phase to neutral)','Three-phase currents · A']){
          const label=document.createElement('h4');label.textContent=title;card.append(label,document.createElement('canvas'));
        }
        container.append(card);
      }
    }
    for(const role of ['test','load']){
      const card=container.querySelector(`[data-role="${role}"]`);
      const headers=['time_s','position','rpm','voltage_a','voltage_b','voltage_c','phase_a','phase_b','phase_c'];
      const rows=data.rows.map(row=>{
        const sample=row.boards?.[role]||{};
        const alpha=Number(sample.v_alpha),beta=Number(sample.v_beta);
        const hasVoltage=sample.v_alpha!=null&&sample.v_beta!=null&&Number.isFinite(alpha)&&Number.isFinite(beta);
        const voltage=hasVoltage?[alpha,-alpha/2+Math.sqrt(3)*beta/2,-alpha/2-Math.sqrt(3)*beta/2]:[null,null,null];
        return [Number(row.host_monotonic_s)-start,sample.position,sample.rpm,...voltage,sample.phase_a,sample.phase_b,sample.phase_c];
      });
      const charts=[
        ['Position · turns',['position'],['Position']],
        ['Speed · rpm',['rpm'],['Speed']],
        ['Three-phase voltage · V (commanded, phase to neutral)',['voltage_a','voltage_b','voltage_c'],['Phase A','Phase B','Phase C']],
        ['Three-phase currents · A',['phase_a','phase_b','phase_c'],['Phase A','Phase B','Phase C']]];
      const canvases=card.querySelectorAll('canvas');
      charts.forEach(([,keys,spanLabels],index)=>plot(canvases[index],rows,headers,keys,keys.map(()=>1),spanLabels));
    }
  }
}
refreshLive().catch(showError);setInterval(()=>refreshLive().catch(showError),1000);
