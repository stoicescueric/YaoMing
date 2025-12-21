package com.watchdog;

class WatchdogAssets {
    static String indexHtml() {
        return "<!DOCTYPE html><html><head><meta charset='utf-8'><title>Watchdog" +
                "</title><style>body{font-family:Arial;margin:40px;}table{border-collapse:collapse;width:100%;}" +
                "th,td{border:1px solid #ccc;padding:6px;}" +
                "</style></head><body><h1>Watchdog Logs</h1>" +
                "<label>Channel <input id='channel' placeholder='optional'></label>" +
                "<label>Limit <input id='limit' type='number' value='100'></label>" +
                "<button onclick='loadLogs()'>Refresh</button>" +
                "<table><thead><tr><th>ID</th><th>Timestamp</th><th>Channel</th><th>Payload</th><th>Tags</th></tr></thead><tbody id='rows'></tbody></table>" +
                "<p><a href='/download'>Download DB</a></p>" +
                "<script>async function loadLogs(){const channel=document.getElementById('channel').value;const limit=document.getElementById('limit').value;" +
                "const params=new URLSearchParams();if(channel)params.append('channel',channel);if(limit)params.append('limit',limit);" +
                "const res=await fetch('/api/logs?'+params.toString());const data=await res.json();const rows=document.getElementById('rows');" +
                "rows.innerHTML='';data.forEach(r=>{const tr=document.createElement('tr');tr.innerHTML=`<td>${r.id}</td><td>${new Date(r.timestamp).toLocaleTimeString()}</td><td>${r.channel}</td><td>${r.payload}</td><td>${r.tags||''}</td>`;rows.appendChild(tr);});}" +
                "loadLogs();setInterval(loadLogs,2000);</script></body></html>";
    }
}
