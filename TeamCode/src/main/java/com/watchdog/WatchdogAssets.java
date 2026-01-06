package com.watchdog;

class WatchdogAssets {
    static String indexHtml() {
        return "<!DOCTYPE html><html><head><meta charset='utf-8'><title>WATCHDOG" +
                "</title><style>" + stylesCss() + "</style></head><body>" +
                "<header class='wd-header'>" +
                "<div class='wd-logo-title'>" +
                "<img src='/logo.svg' alt='WATCHDOG logo' class='wd-logo'/>" +
                "<div><h1>Logs</h1><p class='wd-subtitle'>Non-blocking FTC logging dashboard</p></div>" +
                "</div>" +
                "</header>" +
                "<main>" +
                "<section class='wd-controls'>" +
                "<label>Channel <input id='channel' placeholder='optional' value=''></label>" +
                "<label>Limit <input id='limit' type='number' value='100'></label>" +
                "<button onclick='loadLogs()'>Refresh</button>" +
                "<a href='/download' class='wd-download'>Download DB</a>" +
                "</section>" +
                "<section class='wd-tabs'>" +
                "<button id='tab-logs' class='wd-tab wd-tab-active' onclick='showTab(\"logs\")'>Logs</button>" +
                "<button id='tab-replay' class='wd-tab' onclick='showTab(\"replay\")'>Pose Replay</button>" +
                "</section>" +
                "<section id='panel-logs' class='wd-panel wd-panel-active'>" +
                "<div class='wd-table-wrapper'>" +
                "<table><thead><tr><th>ID</th><th>Timestamp</th><th>Channel</th><th>Payload</th><th>Tags</th></tr></thead><tbody id='rows'></tbody></table>" +
                "</div>" +
                "</section>" +
                "<section id='panel-replay' class='wd-panel'>" +
                "<div class='wd-replay-layout'>" +
                "<div class='wd-runs'>" +
                "<h2>Recorded runs</h2>" +
                "<table><thead><tr><th>Run ID</th><th>Start</th><th>End</th><th>Points</th><th></th></tr></thead><tbody id='runRows'></tbody></table>" +
                "</div>" +
                "<div class='wd-replay-main'>" +
                "<div class='wd-replay-controls'>" +
                "<label>Run ID <input id='runId' placeholder='select from table'></label>" +
                "<label>Max points <input id='poseLimit' type='number' value='1000'></label>" +
                "<button onclick='loadReplay()'>Load path</button>" +
                "<button onclick='exportRun()'>Download .wd</button>" +
                "</div>" +
                "<div class='wd-player-controls'>" +
                "<button onclick='playerRewind()'>&lt;&lt;</button>" +
                "<button onclick='playerToggle()' id='btnPlay'>Play</button>" +
                "<button onclick='playerFastForward()'>&gt;&gt;</button>" +
                "<input id='timeSlider' type='range' min='0' max='1' step='0.001' value='0' onchange='playerSeek(this.value)'/>" +
                "<span id='timeLabel'></span>" +
                "</div>" +
                "<div class='wd-replay-wrapper'>" +
                "<img id='fieldImg' src='/field.png' class='wd-field-img' alt='Field'/>" +
                "<canvas id='fieldCanvas' class='wd-field-canvas'></canvas>" +
                "</div>" +
                "</div>" +
                "</div>" +
                "</section>" +
                "</main>" +
                "<script>" + indexScriptJs() + "</script>" +
                "</body></html>";
    }

    static String stylesCss() {
        return "body{font-family:Arial,Helvetica,sans-serif;margin:24px;background:#0f141a;color:#f5f7fa;}" +
                "h1{margin:0;font-size:24px;}" +
                "table{border-collapse:collapse;width:100%;margin-top:8px;font-size:13px;}" +
                "th,td{border:1px solid #2b323b;padding:6px 8px;}" +
                "th{background:#1b222c;text-align:left;}" +
                "tr:nth-child(even){background:#151b23;}" +
                "tr:nth-child(odd){background:#11161d;}" +
                "input{margin-left:4px;margin-right:12px;padding:2px 4px;background:#11161d;border:1px solid #2b323b;color:#f5f7fa;}" +
                "button{padding:4px 10px;margin-right:4px;border-radius:3px;border:1px solid #3b82f6;background:#2563eb;color:white;cursor:pointer;}" +
                "button:hover{background:#1d4ed8;}" +
                ".wd-header{display:flex;justify-content:space-between;align-items:center;margin-bottom:16px;}" +
                ".wd-logo-title{display:flex;align-items:center;gap:10px;}" +
                ".wd-logo{height:40px;width:auto;display:block;}" +
                ".wd-subtitle{margin:0;font-size:12px;color:#9ca3af;}" +
                ".wd-controls{display:flex;align-items:center;flex-wrap:wrap;gap:8px;margin-bottom:12px;}" +
                ".wd-download{color:#9ca3af;text-decoration:none;border:1px solid #374151;padding:3px 8px;border-radius:3px;}" +
                ".wd-download:hover{color:#e5e7eb;border-color:#4b5563;}" +
                ".wd-table-wrapper{max-height:60vh;overflow:auto;border:1px solid #2b323b;border-radius:4px;}" +
                ".wd-tabs{margin:12px 0;display:flex;gap:8px;}" +
                ".wd-tab{background:#11161d;border:1px solid #374151;color:#e5e7eb;padding:4px 10px;border-radius:4px;cursor:pointer;}" +
                ".wd-tab-active{background:#2563eb;border-color:#3b82f6;}" +
                ".wd-panel{display:none;}" +
                ".wd-panel-active{display:block;}" +
                ".wd-replay-layout{display:flex;gap:16px;}" +
                ".wd-runs{flex:1;max-width:320px;}" +
                ".wd-replay-main{flex:2;}" +
                ".wd-replay-controls{display:flex;align-items:center;flex-wrap:wrap;gap:8px;margin-bottom:8px;}" +
                ".wd-player-controls{display:flex;align-items:center;gap:6px;margin-bottom:8px;}" +
                ".wd-player-controls input[type=range]{flex:1;}" +
                ".wd-replay-wrapper{position:relative;display:inline-block;border:1px solid #2b323b;border-radius:4px;overflow:hidden;}" +
                ".wd-field-img{display:block;max-width:600px;width:100%;height:auto;}" +
                ".wd-field-canvas{position:absolute;left:0;top:0;width:100%;height:100%;pointer-events:none;}";
    }

    static String indexScriptJs() {
        return "let poseFrames=[];let poseStart=0;let poseEnd=0;let poseTimer=null;let isPlaying=false;" +
                "function showTab(name){" +
                "document.getElementById('panel-logs').classList.remove('wd-panel-active');" +
                "document.getElementById('panel-replay').classList.remove('wd-panel-active');" +
                "document.getElementById('tab-logs').classList.remove('wd-tab-active');" +
                "document.getElementById('tab-replay').classList.remove('wd-tab-active');" +
                "if(name==='logs'){" +
                "document.getElementById('panel-logs').classList.add('wd-panel-active');" +
                "document.getElementById('tab-logs').classList.add('wd-tab-active');" +
                "}else{" +
                "document.getElementById('panel-replay').classList.add('wd-panel-active');" +
                "document.getElementById('tab-replay').classList.add('wd-tab-active');" +
                "loadRuns();" +
                "}" +
                "}" +
                "async function loadLogs(){" +
                "const channel=document.getElementById('channel').value;" +
                "const limit=document.getElementById('limit').value;" +
                "const params=new URLSearchParams();" +
                "if(channel)params.append('channel',channel);" +
                "if(limit)params.append('limit',limit);" +
                "const res=await fetch('/api/logs?'+params.toString());" +
                "const data=await res.json();" +
                "const rows=document.getElementById('rows');" +
                "rows.innerHTML='';" +
                "data.forEach(r=>{" +
                "const tr=document.createElement('tr');" +
                "tr.innerHTML=`<td>${r.id}</td><td>${new Date(r.timestamp).toLocaleTimeString()}</td><td>${r.channel}</td><td>${r.payload}</td><td>${r.tags||''}</td>`;" +
                "rows.appendChild(tr);" +
                "});" +
                "}" +
                "async function loadRuns(){" +
                "const res=await fetch('/api/pose/runs');" +
                "const data=await res.json();" +
                "const runRows=document.getElementById('runRows');" +
                "runRows.innerHTML='';" +
                "data.forEach(r=>{" +
                "const tr=document.createElement('tr');" +
                "const start=new Date(r.firstTimestamp).toLocaleTimeString();" +
                "const end=new Date(r.lastTimestamp).toLocaleTimeString();" +
                "tr.innerHTML=`<td>${r.runId}</td><td>${start}</td><td>${end}</td><td>${r.count}</td><td><button onclick=\"selectRun('${r.runId}')\">Select</button></td>`;" +
                "runRows.appendChild(tr);" +
                "});" +
                "}" +
                "function selectRun(id){document.getElementById('runId').value=id;}" +
                "async function loadReplay(){" +
                "const runId=document.getElementById('runId').value;" +
                "const poseLimit=document.getElementById('poseLimit').value||1000;" +
                "if(!runId){alert('Select a run first');return;}" +
                "const params=new URLSearchParams();" +
                "params.append('channel','pose');" +
                "params.append('limit',poseLimit);" +
                "const res=await fetch('/api/logs?'+params.toString());" +
                "const data=await res.json();" +
                "poseFrames=[];" +
                "data.forEach(r=>{" +
                "if(!r.payload)return;" +
                "const parts=r.payload.split(',');" +
                "if(parts.length<5)return;" +
                "const id=parts[0];" +
                "if(id!==runId)return;" +
                "const t=parseFloat(parts[1]);" +
                "const x=parseFloat(parts[2]);" +
                "const y=parseFloat(parts[3]);" +
                "poseFrames.push({t,x,y});" +
                "});" +
                "poseFrames.sort((a,b)=>a.t-b.t);" +
                "if(poseFrames.length===0){alert('No pose data for run');return;}" +
                "poseStart=poseFrames[0].t;poseEnd=poseFrames[poseFrames.length-1].t;" +
                "document.getElementById('timeSlider').value=0;" +
                "updateTimeLabel(poseStart);" +
                "drawPathAtTime(poseStart);" +
                "}" +
                "function updateTimeLabel(t){" +
                "const span=document.getElementById('timeLabel');" +
                "span.textContent=new Date(t).toLocaleTimeString();" +
                "}" +
                "function playerToggle(){" +
                "if(!poseFrames.length)return;" +
                "if(isPlaying){stopPlayer();}else{startPlayer();}" +
                "}" +
                "function startPlayer(){isPlaying=true;document.getElementById('btnPlay').textContent='Pause';let last=Date.now();poseTimer=setInterval(()=>{const now=Date.now();const dt=now-last;last=now;let slider=parseFloat(document.getElementById('timeSlider').value);slider+=dt/(poseEnd-poseStart);if(slider>1){slider=1;stopPlayer();}document.getElementById('timeSlider').value=slider;const t=poseStart+slider*(poseEnd-poseStart);updateTimeLabel(t);drawPathAtTime(t);},50);}" +
                "function stopPlayer(){isPlaying=false;document.getElementById('btnPlay').textContent='Play';if(poseTimer){clearInterval(poseTimer);poseTimer=null;}}" +
                "function playerSeek(v){if(!poseFrames.length)return;const slider=parseFloat(v);const t=poseStart+slider*(poseEnd-poseStart);updateTimeLabel(t);drawPathAtTime(t);}" +
                "function playerRewind(){document.getElementById('timeSlider').value=0;playerSeek(0);}" +
                "function playerFastForward(){if(!poseFrames.length)return;document.getElementById('timeSlider').value=1;playerSeek(1);}" +
                "function drawPathAtTime(t){if(!poseFrames.length)return;const img=document.getElementById('fieldImg');const canvas=document.getElementById('fieldCanvas');const rect=img.getBoundingClientRect();canvas.width=rect.width;canvas.height=rect.height;const ctx=canvas.getContext('2d');ctx.clearRect(0,0,canvas.width,canvas.height);const cx=canvas.width/2;const cy=canvas.height/2;ctx.strokeStyle='#10b981';ctx.lineWidth=2;ctx.beginPath();let started=false;poseFrames.forEach(p=>{if(p.t>t)return;const px=cx + p.x;const py=cy - p.y;if(!started){ctx.moveTo(px,py);started=true;}else{ctx.lineTo(px,py);}});ctx.stroke();}" +
                "function exportRun(){const runId=document.getElementById('runId').value;if(!runId){alert('Select a run first');return;}window.location='/api/pose/export?runId='+encodeURIComponent(runId);}" +
                "loadLogs();setInterval(loadLogs,2000);";
    }
}
