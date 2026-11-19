// Smart insole gait monitor: Web Bluetooth + demo mode, IDW heatmap, traces, gait metrics.
// Packet (both boards): 20 bytes little-endian = uint32 ms, 4 x float32 ΔC/C0 (Heel, Instep, Toe-ball, Toes).
'use strict';

const SVC = '6e400001-b5a3-f393-e0a9-e50e24dcca9e';
const CHR = '6e400003-b5a3-f393-e0a9-e50e24dcca9e';
const NAMES = ['Heel', 'Instep', 'Toe-ball', 'Toes'];
const COLORS = ['#1f77b4', '#9467bd', '#d62728', '#2ca02c'];
// normalised positions (x across, y heel->toe); same as plan/build.py
const POS = [[0.50, 0.13], [0.38, 0.45], [0.45, 0.72], [0.33, 0.91]];
const OUTLINE = [[0.50, 0.00], [0.74, 0.05], [0.80, 0.20], [0.80, 0.42], [0.86, 0.62],
  [0.88, 0.76], [0.80, 0.90], [0.62, 0.98], [0.38, 1.00], [0.18, 0.94],
  [0.12, 0.78], [0.16, 0.62], [0.26, 0.45], [0.24, 0.28], [0.22, 0.12], [0.30, 0.03]];
const L_W = 2.6;                     // length / width of the drawing frame
const WINDOW_S = 10;                 // trace window
const DEFAULT_CAL = '0:0, 5:0.12, 40:1.80, 140:2.74';
// gait detection thresholds on ΔC/C0
const TH_ON = 0.30, TH_OFF = 0.15;

// ------------------------------------------------------------------ state
const buf = [];                      // {t, v[4]} raw ΔC/C0, last WINDOW_S + 2 s
let zero = [0, 0, 0, 0];
let rec = null;                      // array while recording
let source = '';                     // 'ble' | 'demo' | ''
let rateCount = 0, rateT0 = performance.now();
const cal = NAMES.map(() => parseCal(DEFAULT_CAL));
let showN = false;

const gait = {
  heelOn: false, toesOn: false, ballOn: false,
  strikes: [], lastToeOff: null, stance: null, stride: null,
  ballPeaksSinceStrike: 0, fog: false, count: 0, split: [0, 0, 0, 0],
  acc: [0, 0, 0, 0],
};

// ------------------------------------------------------------ calibration
function parseCal(s) {
  const pts = s.split(',').map(p => p.split(':').map(Number));
  if (pts.length < 2 || pts.some(p => p.length !== 2 || p.some(isNaN))) return null;
  pts.sort((a, b) => a[1] - b[1]);
  return pts;                        // [[F, y], ...] sorted by y
}
function toForce(y, i) {
  const p = cal[i];
  if (!p) return NaN;
  if (y <= p[0][1]) return p[0][0];
  for (let k = 1; k < p.length; k++) {
    if (y <= p[k][1]) {
      const [f0, y0] = p[k - 1], [f1, y1] = p[k];
      return f0 + (f1 - f0) * (y - y0) / (y1 - y0);
    }
  }
  const [f0, y0] = p[p.length - 2], [f1, y1] = p[p.length - 1];
  return f0 + (f1 - f0) * (y - y0) / (y1 - y0);   // extrapolate last segment
}
const disp = (y, i) => showN ? Math.max(0, toForce(y, i)) : y;
const dispMax = () => showN ? 150 : 2.8;

// ---------------------------------------------------------------- samples
function push(t, v) {
  const z = v.map((x, i) => Math.max(0, x - zero[i]));
  buf.push({ t, v: z });
  while (buf.length && buf[0].t < t - WINDOW_S - 2) buf.shift();
  rateCount++;
  if (rec) rec.push([t, ...v, ...z]);
  updateGait(t, z);
}

function updateGait(t, v) {
  const [heel, , ball, toes] = v;
  for (let i = 0; i < 4; i++) gait.acc[i] += v[i];

  if (!gait.heelOn && heel > TH_ON) {              // heel strike
    gait.heelOn = true;
    const s = gait.strikes;
    if (s.length) {
      const st = t - s[s.length - 1];
      if (st > 0.4 && st < 4) {
        gait.stride = st; gait.count++;
        const tot = gait.acc.reduce((a, b) => a + b, 0) || 1;
        gait.split = gait.acc.map(a => a / tot);
      }
    }
    gait.acc = [0, 0, 0, 0];
    s.push(t); if (s.length > 20) s.shift();
    gait.ballPeaksSinceStrike = 0;
    gait.fog = false;
  } else if (gait.heelOn && heel < TH_OFF) gait.heelOn = false;

  if (!gait.toesOn && toes > TH_ON) gait.toesOn = true;
  else if (gait.toesOn && toes < TH_OFF) {          // toe-off
    gait.toesOn = false;
    const last = gait.strikes[gait.strikes.length - 1];
    // only a real stance: shorter than the stride (standing still or FOG gives long false "stances")
    const dt = last === undefined ? 0 : t - last;
    if (dt > 0.2 && dt < (gait.stride ? 0.85 * gait.stride : 1.6)) gait.stance = dt;
  }

  if (!gait.ballOn && ball > TH_ON) { gait.ballOn = true; gait.ballPeaksSinceStrike++; }
  else if (gait.ballOn && ball < TH_OFF) gait.ballOn = false;

  // FOG (OUR pattern): forefoot loading keeps repeating but no heel strike for > 2 s, after walking
  const last = gait.strikes[gait.strikes.length - 1];
  if (last !== undefined && gait.strikes.length >= 2 && t - last > 2 && gait.ballPeaksSinceStrike >= 2) gait.fog = true;
}

// --------------------------------------------------------------- bluetooth
let bleDevice = null;
async function connect() {
  if (!navigator.bluetooth) {
    setStatus('Web Bluetooth is not available here. Use Chrome or Edge on a laptop or Android.');
    return;
  }
  try {
    setStatus('Choose the insole (GAIT-ESP32 or GAIT-FDC)...');
    bleDevice = await navigator.bluetooth.requestDevice({ filters: [{ namePrefix: 'GAIT-' }], optionalServices: [SVC] });
    bleDevice.addEventListener('gattserverdisconnected', () => { source = ''; setStatus('Disconnected'); btn('btnConnect').textContent = 'Connect insole'; });
    const server = await bleDevice.gatt.connect();
    const ch = await (await server.getPrimaryService(SVC)).getCharacteristic(CHR);
    ch.addEventListener('characteristicvaluechanged', e => {
      const d = e.target.value;
      if (d.byteLength < 20) return;
      const v = [0, 1, 2, 3].map(i => d.getFloat32(4 + 4 * i, true));
      push(d.getUint32(0, true) / 1000, v);
    });
    await ch.startNotifications();
    stopDemo(); source = 'ble'; resetBuffers();
    btn('btnConnect').textContent = 'Disconnect';
    setStatus(`Connected: ${bleDevice.name} (${bleDevice.name === 'GAIT-FDC' ? 'Method B, FDC1004' : 'Method A, ESP32 touch'})`);
  } catch (err) {
    setStatus('Connection failed: ' + err.message);
  }
}

// -------------------------------------------------------------------- demo
let demoTimer = null, demoT = 0, demoMode = '';
const bump = (ph, a, b, peak) => (ph <= a || ph >= b) ? 0 : peak * Math.sin(Math.PI * (ph - a) / (b - a)) ** 2;
function walkSample(t, T, amp) {
  const ph = (t % T) / T;           // 0 = heel strike; stance ~ 0..0.6
  return [bump(ph, 0, 0.32, 1.6 * amp), bump(ph, 0.1, 0.45, 0.08 * amp),
    bump(ph, 0.22, 0.56, 2.0 * amp), bump(ph, 0.4, 0.62, 0.85 * amp)];
}
function demoSample(t) {
  let v;
  if (demoMode === 'medium') v = walkSample(t, 2.0, 1);
  else if (demoMode === 'short') v = walkSample(t, 1.1, 0.8);
  else {                             // FOG: walk 8 s, stop 1 s, shuffle 5 s, repeat
    const c = t % 14;
    if (c < 8) v = walkSample(c, 2.0, 1);
    else if (c < 9) v = [0.6, 0.05, 0.7, 0.2];
    else { const s = (c - 9) % 0.45 / 0.45; v = [0.05, 0.05, bump(s, 0, 0.8, 1.1), bump(s, 0.1, 0.7, 0.35)]; }
  }
  return v.map(x => x + 0.015 * (Math.random() - 0.5) + 0.01);
}
function startDemo(mode) {
  stopDemo(); demoMode = mode; demoT = 0; source = 'demo'; resetBuffers();
  demoTimer = setInterval(() => { demoT += 0.02; push(demoT, demoSample(demoT)); }, 20);
  setStatus('Demo mode: synthetic ' + { medium: 'medium-pace walking', short: 'short strides', fog: 'walking with freezing of gait' }[mode]);
}
function stopDemo() { if (demoTimer) clearInterval(demoTimer); demoTimer = null; demoMode = ''; }

function resetBuffers() {
  buf.length = 0; zero = [0, 0, 0, 0];
  Object.assign(gait, { heelOn: false, toesOn: false, ballOn: false, strikes: [], stance: null, stride: null,
    ballPeaksSinceStrike: 0, fog: false, count: 0, split: [0, 0, 0, 0], acc: [0, 0, 0, 0] });
}

// ----------------------------------------------------------------- drawing
function turbo(t) {
  // skip the near-black start of turbo so unloaded areas read as blue, not black
  t = 0.1 + 0.9 * Math.min(1, Math.max(0, t));
  const r = 0.13572138 + t * (4.61539260 + t * (-42.66032258 + t * (132.13108234 + t * (-152.94239396 + t * 59.28637943))));
  const g = 0.09140261 + t * (2.19418839 + t * (4.84296658 + t * (-14.18503333 + t * (4.27729857 + t * 2.82956604))));
  const b = 0.10667330 + t * (12.64194608 + t * (-60.58204836 + t * (110.36276771 + t * (-89.90310912 + t * 27.34824973))));
  return [r, g, b].map(c => Math.round(255 * Math.min(1, Math.max(0, c))));
}

function outlinePts() {                // closed Catmull-Rom, same as build.py
  const P = OUTLINE, n = P.length, out = [];
  for (let i = 0; i < n; i++) {
    const p0 = P[(i - 1 + n) % n], p1 = P[i], p2 = P[(i + 1) % n], p3 = P[(i + 2) % n];
    for (let k = 0; k < 12; k++) {
      const t = k / 12, t2 = t * t, t3 = t2 * t;
      out.push([0, 1].map(j => 0.5 * (2 * p1[j] + (-p0[j] + p2[j]) * t + (2 * p0[j] - 5 * p1[j] + 4 * p2[j] - p3[j]) * t2
        + (-p0[j] + 3 * p1[j] - 3 * p2[j] + p3[j]) * t3)));
    }
  }
  return out;
}
const OUT = outlinePts();

const heat = document.getElementById('heat'), hctx = heat.getContext('2d');
const HW = heat.width, HH = heat.height;
// frame: x 0..1 -> 10..HW-10 ; y 0..1 -> HH-10..10 (toes up)
const fx = x => 10 + x * (HW - 20), fy = y => HH - 10 - y * (HH - 20);
const GW = 52, GH = Math.round(GW * (HH - 20) / (HW - 20));
const grid = document.createElement('canvas'); grid.width = GW; grid.height = GH;
const gctx = grid.getContext('2d'), gimg = gctx.createImageData(GW, GH);
const cop = [];

function drawHeat(v) {
  const vmax = dispMax(), d = v.map(disp);
  for (let j = 0; j < GH; j++) {
    const y = 1 - (j + 0.5) / GH;
    for (let i = 0; i < GW; i++) {
      const x = (i + 0.5) / GW;
      let num = 0, den = 0;
      for (let s = 0; s < 4; s++) {
        const dx = x - POS[s][0], dy = (y - POS[s][1]) * L_W;   // same metric as build.py
        const w = 1 / Math.pow(dx * dx + dy * dy + 0.004, 1.5);
        num += w * d[s]; den += w;
      }
      const [r, g, b] = turbo(num / den / vmax);
      const k = 4 * (j * GW + i);
      gimg.data[k] = r; gimg.data[k + 1] = g; gimg.data[k + 2] = b; gimg.data[k + 3] = 255;
    }
  }
  gctx.putImageData(gimg, 0, 0);
  hctx.clearRect(0, 0, HW, HH);
  hctx.save();
  hctx.beginPath();
  OUT.forEach(([x, y], i) => i ? hctx.lineTo(fx(x), fy(y)) : hctx.moveTo(fx(x), fy(y)));
  hctx.closePath();
  hctx.clip();
  hctx.imageSmoothingEnabled = true;
  hctx.drawImage(grid, 10, 10, HW - 20, HH - 20);
  hctx.restore();
  hctx.beginPath();
  OUT.forEach(([x, y], i) => i ? hctx.lineTo(fx(x), fy(y)) : hctx.moveTo(fx(x), fy(y)));
  hctx.closePath(); hctx.lineWidth = 1.5; hctx.strokeStyle = '#222'; hctx.stroke();

  // sensor squares (12 mm on a ~100 mm wide insole)
  hctx.strokeStyle = 'rgba(255,255,255,.9)'; hctx.lineWidth = 1;
  POS.forEach(([x, y]) => { const s = 0.12 * (HW - 20); hctx.strokeRect(fx(x) - s / 2, fy(y) - s / 2, s, s); });

  // centre of pressure trail
  const tot = v.reduce((a, b) => a + b, 0);
  if (tot > 0.2) {
    cop.push([POS.reduce((a, p, i) => a + p[0] * v[i], 0) / tot, POS.reduce((a, p, i) => a + p[1] * v[i], 0) / tot]);
  } else cop.push(null);
  while (cop.length > 90) cop.shift();
  hctx.lineWidth = 2; hctx.strokeStyle = 'rgba(255,255,255,.95)';
  hctx.beginPath(); let pen = false;
  for (const c of cop) { if (!c) { pen = false; continue; } pen ? hctx.lineTo(fx(c[0]), fy(c[1])) : hctx.moveTo(fx(c[0]), fy(c[1])); pen = true; }
  hctx.stroke();
  const lastC = cop[cop.length - 1];
  if (lastC) { hctx.fillStyle = '#fff'; hctx.beginPath(); hctx.arc(fx(lastC[0]), fy(lastC[1]), 5, 0, 7); hctx.fill(); hctx.strokeStyle = '#000'; hctx.lineWidth = 1; hctx.stroke(); }
}

const trace = document.getElementById('trace'), tctx = trace.getContext('2d');
function drawTrace() {
  // Size the canvas to its on-screen width so text and lines stay sharp and readable on phones.
  const dpr = window.devicePixelRatio || 1;
  const W = Math.max(240, trace.clientWidth || 900), H = Math.round(Math.min(300, Math.max(200, W * 0.4)));
  if (trace.width !== Math.round(W * dpr) || trace.height !== Math.round(H * dpr)) {
    trace.width = Math.round(W * dpr); trace.height = Math.round(H * dpr); trace.style.height = H + 'px';
  }
  tctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const L = 46, R = 10, T = 10, B = 26;
  tctx.clearRect(0, 0, W, H);
  tctx.font = '12px -apple-system, Helvetica, Arial'; tctx.fillStyle = '#5e6873'; tctx.strokeStyle = '#e3e7ec'; tctx.lineWidth = 1;
  const ymax = dispMax(), step = showN ? 50 : 0.5;
  for (let y = 0; y <= ymax + 1e-9; y += step) {
    const py = H - B - (y / ymax) * (H - T - B);
    tctx.beginPath(); tctx.moveTo(L, py); tctx.lineTo(W - R, py); tctx.stroke();
    tctx.fillText(showN ? y.toFixed(0) : y.toFixed(1), 6, py + 4);
  }
  const tNow = buf.length ? buf[buf.length - 1].t : 0;
  for (let s = 0; s <= WINDOW_S; s += 2) {
    const px = L + (s / WINDOW_S) * (W - L - R);
    tctx.fillText(`${s - WINDOW_S} s`, px - 10, H - 8);
  }
  for (let i = 0; i < 4; i++) {
    tctx.strokeStyle = COLORS[i]; tctx.lineWidth = 1.6; tctx.beginPath();
    let first = true;
    for (const p of buf) {
      const age = tNow - p.t; if (age > WINDOW_S) continue;
      const px = L + (1 - age / WINDOW_S) * (W - L - R);
      const py = H - B - Math.min(1, disp(p.v[i], i) / ymax) * (H - T - B);
      first ? tctx.moveTo(px, py) : tctx.lineTo(px, py); first = false;
    }
    tctx.stroke();
  }
}

// ----------------------------------------------------------------- metrics
const $ = id => document.getElementById(id);
const btn = $;
function setStatus(s) { $('status').textContent = s; }
function updateMetrics() {
  $('mStride').textContent = gait.stride ? gait.stride.toFixed(2) + ' s' : '-';
  $('mCad').textContent = gait.stride ? (120 / gait.stride).toFixed(0) + ' steps/min' : '-';
  $('mStance').textContent = gait.stance ? gait.stance.toFixed(2) + ' s' : '-';
  $('mCount').textContent = gait.count;
  $('mFog').textContent = gait.fog ? 'DETECTED' : 'no';
  $('fogBox').classList.toggle('alert', gait.fog);
  $('split').innerHTML = NAMES.map((n, i) => `<div class="row"><span>${n}</span><span class="bar"><b style="width:${(100 * gait.split[i]).toFixed(0)}%;background:${COLORS[i]}"></b></span><span>${(100 * gait.split[i]).toFixed(0)} %</span></div>`).join('');
}

function frame() {
  const v = buf.length ? buf[buf.length - 1].v : [0, 0, 0, 0];
  drawHeat(v); drawTrace();
  requestAnimationFrame(frame);
}
setInterval(() => {
  const now = performance.now();
  const r = rateCount / ((now - rateT0) / 1000); rateCount = 0; rateT0 = now;
  $('mRate').textContent = source ? r.toFixed(0) + ' Hz' : '-';
  updateMetrics();
}, 500);

// ----------------------------------------------------------------- ui wiring
function refreshUnits() {
  $('scaleMax').textContent = showN ? '150 N' : dispMax().toFixed(1);
  $('scaleUnit').textContent = showN ? 'force (N)' : 'ΔC/C0';
  $('traceUnit').textContent = showN ? '(N, last 10 s)' : '(ΔC/C0, last 10 s)';
}
$('scaleBar').style.background = `linear-gradient(to right, ${Array.from({ length: 11 }, (_, i) => `rgb(${turbo(i / 10)})`).join(',')})`;
$('legend').innerHTML = NAMES.map((n, i) => `<span><i style="background:${COLORS[i]}"></i>${n}</span>`).join('');
$('calib').innerHTML = NAMES.map((n, i) => `<label for="cal${i}">${n}</label><input id="cal${i}" value="${DEFAULT_CAL}">`).join('');
NAMES.forEach((_, i) => $('cal' + i).addEventListener('change', e => {
  const p = parseCal(e.target.value); e.target.classList.toggle('bad', !p); if (p) cal[i] = p;
}));
$('unitN').addEventListener('change', e => { showN = e.target.checked; refreshUnits(); });
$('btnConnect').addEventListener('click', () => {
  if (source === 'ble' && bleDevice) { bleDevice.gatt.disconnect(); return; }
  $('demoSel').value = ''; connect();
});
$('demoSel').addEventListener('change', e => {
  if (source === 'ble' && bleDevice) bleDevice.gatt.disconnect();
  e.target.value ? startDemo(e.target.value) : (stopDemo(), source = '', setStatus('Not connected'));
});
$('btnZero').addEventListener('click', () => {
  if (!buf.length) return;
  const last = buf[buf.length - 1].v; zero = zero.map((z, i) => z + last[i]);
});
$('btnRec').addEventListener('click', e => {
  if (!rec) { rec = []; e.target.textContent = 'Stop'; e.target.classList.add('on'); $('btnCsv').disabled = true; }
  else { e.target.textContent = 'Record'; e.target.classList.remove('on'); $('btnCsv').disabled = !rec.length; window._lastRec = rec; rec = null; }
});
$('btnCsv').addEventListener('click', () => {
  const r = window._lastRec; if (!r) return;
  const head = ['t_s', ...NAMES.map(n => 'raw_' + n), ...NAMES.map(n => 'dCC0_' + n), ...NAMES.map(n => 'N_' + n)];
  const lines = [head.join(',')].concat(r.map(row => {
    const z = row.slice(5, 9);
    return [row[0].toFixed(3), ...row.slice(1).map(x => x.toFixed(4)), ...z.map((y, i) => Math.max(0, toForce(y, i)).toFixed(2))].join(',');
  }));
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([lines.join('\n')], { type: 'text/csv' }));
  a.download = `gait_${source || 'data'}_${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}.csv`;
  a.click();
});
refreshUnits(); updateMetrics(); requestAnimationFrame(frame);

// Open straight into a demo with ?demo=medium | short | fog (used by the live demo link).
const urlDemo = new URLSearchParams(location.search).get('demo');
if (['medium', 'short', 'fog'].includes(urlDemo)) { $('demoSel').value = urlDemo; startDemo(urlDemo); }
