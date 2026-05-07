const char HTML_PAGE[] PROGMEM = R"__HTML_EOF__(<!DOCTYPE html>
<html lang="es">
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta charset="UTF-8">
  <title>ESP32 Motor Control</title>
  <link rel="icon" href="data:image/svg+xml,&lt;svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 100 100'&gt;&lt;text y='.9em' font-size='90'&gt;⚙&lt;/text&gt;&lt;/svg&gt;">
  <style>
    :root {
      --bg:       #0d0f14;
      --surface:  #13161e;
      --border:   #1e2330;
      --accent1:  #00e5ff;
      --accent2:  #ff6b35;
      --green:    #39ff6e;
      --red:      #ff3355;
      --text:     #c8d0e0;
      --dim:      #4a5268;
      --mono:     'Courier New', 'Lucida Console', monospace;
      --sans:     -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
    }

    *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }

    body {
      background: var(--bg);
      color: var(--text);
      font-family: var(--sans);
      font-weight: 300;
      min-height: 100vh;
      padding: 20px;
    }

    header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      margin-bottom: 24px;
      padding-bottom: 16px;
      border-bottom: 1px solid var(--border);
    }
    .logo {
      font-family: var(--mono);
      font-size: 13px;
      letter-spacing: 3px;
      color: var(--accent1);
      text-transform: uppercase;
    }
    .logo span { color: var(--dim); }

    #statusDot {
      width: 9px; height: 9px;
      border-radius: 50%;
      background: var(--red);
      box-shadow: 0 0 8px var(--red);
      display: inline-block;
      margin-right: 8px;
      transition: background .4s, box-shadow .4s;
    }
    #statusDot.on { background: var(--green); box-shadow: 0 0 10px var(--green); }
    #statusText {
      font-family: var(--mono);
      font-size: 12px;
      letter-spacing: 2px;
      color: var(--dim);
    }

    .grid {
      display: grid;
      grid-template-columns: 280px 1fr;
      grid-template-rows: auto auto;
      gap: 16px;
    }
    @media (max-width: 720px) {
      .grid { grid-template-columns: 1fr; }
    }

    .card {
      background: var(--surface);
      border: 1px solid var(--border);
      border-radius: 6px;
      padding: 20px;
    }
    .card-title {
      font-family: var(--mono);
      font-size: 10px;
      letter-spacing: 3px;
      color: var(--dim);
      text-transform: uppercase;
      margin-bottom: 18px;
    }

    .metrics { grid-row: 1 / 3; display: flex; flex-direction: column; gap: 16px; }

    .metric-block { margin-bottom: 14px; }
    .metric-label {
      font-family: var(--mono);
      font-size: 10px;
      letter-spacing: 2px;
      color: var(--dim);
      margin-bottom: 4px;
    }
    .metric-value {
      font-family: var(--mono);
      font-size: 36px;
      line-height: 1;
      font-weight: 400;
    }
    .metric-value.cyan  { color: var(--accent1); text-shadow: 0 0 20px rgba(0,229,255,.35); }
    .metric-value.amber { color: var(--accent2); text-shadow: 0 0 20px rgba(255,107,53,.35); }
    .metric-unit {
      font-family: var(--mono);
      font-size: 12px;
      color: var(--dim);
      margin-left: 6px;
    }

    hr.sep { border: none; border-top: 1px solid var(--border); margin: 4px 0 16px; }

    #btnLed {
      width: 100%;
      padding: 11px;
      font-family: var(--mono);
      font-size: 12px;
      letter-spacing: 3px;
      border: 1px solid var(--dim);
      border-radius: 4px;
      background: transparent;
      color: var(--dim);
      cursor: pointer;
      transition: all .25s;
      margin-bottom: 16px;
    }
    #btnLed.led-on {
      border-color: var(--green);
      color: var(--green);
      box-shadow: 0 0 12px rgba(57,255,110,.2);
    }

    .slider-label {
      font-family: var(--mono);
      font-size: 10px;
      letter-spacing: 2px;
      color: var(--dim);
      display: flex;
      justify-content: space-between;
      margin-bottom: 8px;
    }
    #sliderVel {
      -webkit-appearance: none;
      width: 100%;
      height: 3px;
      background: var(--border);
      border-radius: 2px;
      outline: none;
      margin-bottom: 6px;
    }
    #sliderVel::-webkit-slider-thumb {
      -webkit-appearance: none;
      width: 16px; height: 16px;
      border-radius: 50%;
      background: var(--accent1);
      box-shadow: 0 0 10px rgba(0,229,255,.5);
      cursor: pointer;
    }

    .dir-row { display: flex; gap: 8px; margin-top: 12px; }
    .btn-dir {
      flex: 1;
      padding: 9px;
      font-family: var(--mono);
      font-size: 11px;
      letter-spacing: 2px;
      border: 1px solid var(--border);
      border-radius: 4px;
      background: transparent;
      color: var(--dim);
      cursor: pointer;
      transition: all .2s;
    }
    .btn-dir.active {
      border-color: var(--accent1);
      color: var(--accent1);
      background: rgba(0,229,255,.06);
    }

    .chart-card { position: relative; }
    .chart-header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-bottom: 14px;
    }
    .chart-actions { display: flex; gap: 8px; }
    .btn-sm {
      font-family: var(--mono);
      font-size: 9px;
      letter-spacing: 2px;
      padding: 5px 10px;
      border-radius: 3px;
      border: 1px solid var(--border);
      background: transparent;
      color: var(--dim);
      cursor: pointer;
      transition: all .2s;
    }
    .btn-sm:hover { border-color: var(--accent1); color: var(--accent1); }
    .btn-sm.danger:hover { border-color: var(--red); color: var(--red); }

    .svg-chart {
      width: 100%;
      height: 160px;
      background: var(--surface);
    }
    .svg-chart path {
      fill: none;
      stroke-width: 1.5;
      stroke-linejoin: round;
      stroke-linecap: round;
    }
    .svg-chart .grid-line {
      stroke: var(--border);
      stroke-width: 0.5;
    }
    .svg-chart text {
      font-family: var(--mono);
      font-size: 9px;
      fill: var(--dim);
    }

    .rec-badge {
      display: inline-flex;
      align-items: center;
      gap: 5px;
      font-family: var(--mono);
      font-size: 9px;
      letter-spacing: 2px;
      color: var(--red);
      opacity: 0;
      transition: opacity .3s;
    }
    .rec-badge.active { opacity: 1; }
    .rec-dot {
      width: 6px; height: 6px;
      border-radius: 50%;
      background: var(--red);
      animation: blink 1s infinite;
    }
    @keyframes blink { 0%,100%{opacity:1} 50%{opacity:0} }

    .history-section { grid-column: 1 / -1; }
    .table-wrap { overflow-x: auto; }
    table {
      width: 100%;
      border-collapse: collapse;
      font-family: var(--mono);
      font-size: 12px;
    }
    thead th {
      text-align: left;
      padding: 8px 12px;
      color: var(--dim);
      font-size: 10px;
      letter-spacing: 2px;
      border-bottom: 1px solid var(--border);
    }
    tbody tr { border-bottom: 1px solid rgba(30,35,48,.6); }
    tbody tr:hover { background: rgba(0,229,255,.03); }
    tbody td { padding: 7px 12px; color: var(--text); }
    tbody td.cyan  { color: var(--accent1); }
    tbody td.amber { color: var(--accent2); }
    .empty-row td { text-align: center; color: var(--dim); padding: 24px; }
  </style>
</head>
<body>

<header>
  <div class="logo">ESP32 <span>//</span> Motor Control</div>
  <div>
    <span id="statusDot"></span>
    <span id="statusText">OFFLINE</span>
  </div>
</header>

<div class="grid">

  <div class="metrics">
    <div class="card">
      <div class="card-title">Telemetria en vivo</div>
      <div class="metric-block">
        <div class="metric-label">VELOCIDAD</div>
        <div class="metric-value cyan"><span id="mRpm">0.0</span><span class="metric-unit">RPM</span></div>
      </div>
      <div class="metric-block">
        <div class="metric-label">LINEAL</div>
        <div class="metric-value cyan" style="font-size:24px"><span id="mSpd">0.000</span><span class="metric-unit">m/s</span></div>
      </div>
      <div class="metric-block">
        <div class="metric-label">CORRIENTE</div>
        <div class="metric-value amber"><span id="mCur">0.00</span><span class="metric-unit">A</span></div>
      </div>
      <div class="metric-block">
        <div class="metric-label">PWM</div>
        <div class="metric-value" style="font-size:24px;color:var(--text)"><span id="mVel">0</span><span class="metric-unit">/ 1023</span></div>
      </div>
    </div>

    <div class="card">
      <div class="card-title">Controles</div>
      <button id="btnLed" onclick="toggleLed()">● LED: OFF</button>
      <hr class="sep">
      <div class="slider-label">
        <span>VELOCIDAD PWM</span>
        <span id="valVel">0</span>
      </div>
      <input type="range" min="0" max="1023" value="0" id="sliderVel" oninput="setVel(this.value)">
      <hr class="sep" style="margin-top:14px">
      <div class="slider-label"><span>DIRECCION</span></div>
      <div class="dir-row">
        <button class="btn-dir active" id="btnFwd" onclick="setDir(1)">▶ FWD</button>
        <button class="btn-dir"        id="btnRev" onclick="setDir(-1)">◀ REV</button>
      </div>
    </div>
  </div>

  <div class="card chart-card">
    <div class="chart-header">
      <div>
        <div class="card-title" style="margin-bottom:4px">RPM vs Tiempo</div>
        <span class="rec-badge" id="recBadge1"><span class="rec-dot"></span>REC</span>
      </div>
      <div class="chart-actions">
        <button class="btn-sm" onclick="exportCSV('rpm')">↓ CSV</button>
        <button class="btn-sm danger" onclick="clearData('rpm')">✕ Limpiar</button>
      </div>
    </div>
    <svg id="svgRpm" class="svg-chart" viewBox="0 0 800 160" preserveAspectRatio="none">
      <defs>
        <linearGradient id="gradRpm" x1="0" y1="0" x2="0" y2="1">
          <stop offset="0%" stop-color="#00e5ff" stop-opacity="0.3"/>
          <stop offset="100%" stop-color="#00e5ff" stop-opacity="0"/>
        </linearGradient>
      </defs>
      <g id="gridRpm"></g>
      <path id="pathRpm" stroke="#00e5ff" fill="url(#gradRpm)"/>
      <g id="labelsRpm"></g>
    </svg>
  </div>

  <div class="card chart-card" style="grid-column: 2;">
    <div class="chart-header">
      <div>
        <div class="card-title" style="margin-bottom:4px">Corriente vs Tiempo</div>
        <span class="rec-badge active" id="recBadge2"><span class="rec-dot"></span>REC</span>
      </div>
      <div class="chart-actions">
        <button class="btn-sm" onclick="exportCSV('cur')">↓ CSV</button>
        <button class="btn-sm danger" onclick="clearData('cur')">✕ Limpiar</button>
      </div>
    </div>
    <svg id="svgCur" class="svg-chart" viewBox="0 0 800 160" preserveAspectRatio="none">
      <defs>
        <linearGradient id="gradCur" x1="0" y1="0" x2="0" y2="1">
          <stop offset="0%" stop-color="#ff6b35" stop-opacity="0.3"/>
          <stop offset="100%" stop-color="#ff6b35" stop-opacity="0"/>
        </linearGradient>
      </defs>
      <g id="gridCur"></g>
      <path id="pathCur" stroke="#ff6b35" fill="url(#gradCur)"/>
      <g id="labelsCur"></g>
    </svg>
  </div>

  <div class="card history-section">
    <div class="chart-header">
      <div class="card-title">Historial de sesion</div>
      <div class="chart-actions">
        <button class="btn-sm" onclick="exportCSV('all')">↓ CSV completo</button>
        <button class="btn-sm danger" onclick="clearData('all')">✕ Limpiar todo</button>
      </div>
    </div>
    <div class="table-wrap">
      <table>
        <thead>
          <tr>
            <th>TIEMPO</th><th>RPM</th><th>m/s</th><th>CORRIENTE (A)</th><th>PWM</th><th>DIR</th>
          </tr>
        </thead>
        <tbody id="histBody">
          <tr class="empty-row"><td colspan="6">Sin datos — conecta el ESP32</td></tr>
        </tbody>
      </table>
    </div>
  </div>

</div>

<script>
// STORAGE
const STORAGE_KEY = 'esp32_log_v1';
const MAX_POINTS  = 300;
const MAX_ROWS    = 500;

function loadLog() {
  try { return JSON.parse(localStorage.getItem(STORAGE_KEY)) || []; }
  catch { return []; }
}
function saveLog(log) {
  try { localStorage.setItem(STORAGE_KEY, JSON.stringify(log.slice(-MAX_ROWS))); }
  catch {}
}

let sessionLog = loadLog();

// SVG CHART ENGINE
function updateSvgChart(svgId, data, color, unit) {
  const svg = document.getElementById(svgId);
  const path = svg.querySelector('path');
  const grid = svg.querySelector('g[id^="grid"]');
  const labels = svg.querySelector('g[id^="labels"]');
  const w = 800, h = 160, pad = 24;
  const plotW = w - pad * 2, plotH = h - pad * 2;

  if (data.length < 2) return;

  const maxVal = Math.max(...data.map(d => d.v), 0.001);
  const minVal = Math.min(...data.map(d => d.v), 0);
  const range = maxVal - minVal || 1;

  grid.innerHTML = '';
  for (let i = 0; i <= 4; i++) {
    const y = pad + (plotH * i / 4);
    grid.innerHTML += '<line x1="' + pad + '" y1="' + y + '" x2="' + (w-pad) + '" y2="' + y + '" class="grid-line"/>';
  }

  labels.innerHTML = '';
  for (let i = 0; i <= 4; i++) {
    const y = pad + (plotH * i / 4);
    const val = maxVal - (range * i / 4);
    labels.innerHTML += '<text x="' + (pad-4) + '" y="' + (y+3) + '" text-anchor="end">' + val.toFixed(2) + '</text>';
  }

  const stepX = plotW / (MAX_POINTS - 1);
  let d = 'M ' + pad + ' ' + (pad + plotH);
  data.forEach((pt, i) => {
    const x = pad + i * stepX;
    const y = pad + plotH - ((pt.v - minVal) / range) * plotH;
    d += ' L ' + x + ' ' + y;
  });
  d += ' L ' + (pad + (data.length-1) * stepX) + ' ' + (pad + plotH) + ' Z';
  path.setAttribute('d', d);
}

let rpmData = sessionLog.slice(-MAX_POINTS).map(r => ({t: r.t, v: r.rpm}));
let curData = sessionLog.slice(-MAX_POINTS).map(r => ({t: r.t, v: r.cur}));

function restoreCharts() {
  updateSvgChart('svgRpm', rpmData, '#00e5ff', 'RPM');
  updateSvgChart('svgCur', curData, '#ff6b35', 'A');
  renderTable();
}

// WEBSOCKET
let ws;
let currentDir = 1;

function init() {
  ws = new WebSocket('ws://' + window.location.hostname + '/ws');
  ws.onopen = () => setStatus(true);
  ws.onmessage = (event) => {
    try {
      const d = JSON.parse(event.data);
      updateUI(d);
      recordPoint(d);
    } catch(e) { console.error(e); }
  };
  ws.onclose = () => { setStatus(false); setTimeout(init, 2000); };
  ws.onerror = () => ws.close();
}

function setStatus(on) {
  document.getElementById('statusDot').className = on ? 'on' : '';
  document.getElementById('statusText').innerText = on ? 'ONLINE' : 'OFFLINE';
}

// UI UPDATE
function updateUI(d) {
  if (d.rpm       !== undefined) document.getElementById('mRpm').innerText = parseFloat(d.rpm).toFixed(1);
  if (d.speed_m_s !== undefined) document.getElementById('mSpd').innerText = parseFloat(d.speed_m_s).toFixed(3);
  if (d.current   !== undefined) document.getElementById('mCur').innerText = parseFloat(d.current).toFixed(2);

  if (d.velocity !== undefined) {
    document.getElementById('mVel').innerText = d.velocity;
    document.getElementById('valVel').innerText = d.velocity;
    const s = document.getElementById('sliderVel');
    if (String(s.value) !== String(d.velocity)) {
      s.value = d.velocity;
    }
  }
  if (d.ledState !== undefined) updateLedUI(d.ledState);
  if (d.direction !== undefined) updateDirUI(d.direction);
}

// RECORD
let lastRecord = 0;
const RECORD_INTERVAL = 1000;

function recordPoint(d) {
  const now = Date.now();
  if (now - lastRecord < RECORD_INTERVAL) return;
  lastRecord = now;

  const ts = new Date().toLocaleTimeString('es-CL');
  const row = {
    t: ts, rpm: parseFloat(d.rpm||0), spd: parseFloat(d.speed_m_s||0),
    cur: parseFloat(d.current||0), vel: d.velocity||0,
    dir: (d.direction >= 0) ? 'FWD' : 'REV'
  };

  sessionLog.push(row);
  saveLog(sessionLog);

  rpmData.push({t: ts, v: row.rpm});
  curData.push({t: ts, v: row.cur});
  if (rpmData.length > MAX_POINTS) rpmData.shift();
  if (curData.length > MAX_POINTS) curData.shift();

  updateSvgChart('svgRpm', rpmData, '#00e5ff', 'RPM');
  updateSvgChart('svgCur', curData, '#ff6b35', 'A');
  prependTableRow(row);
}

// TABLA
function renderTable() {
  const tbody = document.getElementById('histBody');
  if (sessionLog.length === 0) {
    tbody.innerHTML = '<tr class="empty-row"><td colspan="6">Sin datos — conecta el ESP32</td></tr>';
    return;
  }
  tbody.innerHTML = '';
  [...sessionLog].reverse().forEach(r => tbody.insertAdjacentHTML('beforeend', rowHTML(r)));
}

function prependTableRow(r) {
  const tbody = document.getElementById('histBody');
  tbody.querySelectorAll('.empty-row').forEach(e => e.remove());
  tbody.insertAdjacentHTML('afterbegin', rowHTML(r));
  while (tbody.rows.length > MAX_ROWS) tbody.deleteRow(tbody.rows.length - 1);
}

function rowHTML(r) {
  return '<tr><td>' + r.t + '</td><td class="cyan">' + r.rpm.toFixed(1) + '</td><td class="cyan">' + r.spd.toFixed(3) + '</td><td class="amber">' + r.cur.toFixed(2) + '</td><td>' + r.vel + '</td><td>' + r.dir + '</td></tr>';
}

// EXPORT CSV
function exportCSV(type) {
  if (sessionLog.length === 0) return;
  let header, rows;
  if (type === 'rpm') {
    header = 'Tiempo,RPM,m/s\n';
    rows = sessionLog.map(r => r.t + ',' + r.rpm + ',' + r.spd).join('\n');
  } else if (type === 'cur') {
    header = 'Tiempo,Corriente_A\n';
    rows = sessionLog.map(r => r.t + ',' + r.cur).join('\n');
  } else {
    header = 'Tiempo,RPM,m/s,Corriente_A,PWM,Direccion\n';
    rows = sessionLog.map(r => r.t + ',' + r.rpm + ',' + r.spd + ',' + r.cur + ',' + r.vel + ',' + r.dir).join('\n');
  }
  const blob = new Blob([header + rows], { type: 'text/csv' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url; a.download = 'motor_log_' + type + '_' + Date.now() + '.csv'; a.click();
  URL.revokeObjectURL(url);
}

// CLEAR
function clearData(type) {
  if (!confirm('Borrar los datos de ' + type + '?')) return;
  sessionLog = []; rpmData = []; curData = [];
  localStorage.removeItem(STORAGE_KEY);
  updateSvgChart('svgRpm', [], '#00e5ff', 'RPM');
  updateSvgChart('svgCur', [], '#ff6b35', 'A');
  renderTable();
}

// CONTROLES
function send(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
}

function toggleLed() {
  const btn = document.getElementById('btnLed');
  const isOn = btn.classList.contains('led-on');
  updateLedUI(!isOn);
  send({ led: !isOn });
}
function updateLedUI(on) {
  const btn = document.getElementById('btnLed');
  btn.innerText = on ? '● LED: ON' : '● LED: OFF';
  btn.className = on ? 'led-on' : '';
}

function setDir(dir) {
  currentDir = dir;
  updateDirUI(dir);
  send({ direction: dir });
}
function updateDirUI(dir) {
  document.getElementById('btnFwd').className = (dir >= 0) ? 'btn-dir active' : 'btn-dir';
  document.getElementById('btnRev').className = (dir <  0) ? 'btn-dir active' : 'btn-dir';
}

// SLIDER - Estilo simple como en el ejemplo
function setVel(v) {
  const valInt = parseInt(v);
  document.getElementById('valVel').innerText = valInt;
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ velocity: valInt }));
  }
}

// INIT
window.onload = () => { restoreCharts(); init(); };
</script>
</body>
</html>
)__HTML_EOF__";