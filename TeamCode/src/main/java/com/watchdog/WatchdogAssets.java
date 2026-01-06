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
                "<label>Channel <input id='channel' placeholder='optional'></label>" +
                "<label>Limit <input id='limit' type='number' value='100'></label>" +
                "<button onclick='loadLogs()'>Refresh</button>" +
                "<a href='/download' class='wd-download'>Download DB</a>" +
                "</section>" +
                "<section class='wd-table-wrapper'>" +
                "<table><thead><tr><th>ID</th><th>Timestamp</th><th>Channel</th><th>Payload</th><th>Tags</th></tr></thead><tbody id='rows'></tbody></table>" +
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
                "button{padding:4px 10px;margin-right:12px;border-radius:3px;border:1px solid #3b82f6;background:#2563eb;color:white;cursor:pointer;}" +
                "button:hover{background:#1d4ed8;}" +
                ".wd-header{display:flex;justify-content:space-between;align-items:center;margin-bottom:16px;}" +
                ".wd-logo-title{display:flex;align-items:center;gap:10px;}" +
                ".wd-logo{height:40px;width:auto;display:block;}" +
                ".wd-subtitle{margin:0;font-size:12px;color:#9ca3af;}" +
                ".wd-controls{display:flex;align-items:center;flex-wrap:wrap;gap:8px;margin-bottom:12px;}" +
                ".wd-download{color:#9ca3af;text-decoration:none;border:1px solid #374151;padding:3px 8px;border-radius:3px;}" +
                ".wd-download:hover{color:#e5e7eb;border-color:#4b5563;}" +
                ".wd-table-wrapper{max-height:70vh;overflow:auto;border:1px solid #2b323b;border-radius:4px;}";
    }

    static String indexScriptJs() {
        return "async function loadLogs(){" +
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
                "loadLogs();" +
                "setInterval(loadLogs,2000);";
    }
}
