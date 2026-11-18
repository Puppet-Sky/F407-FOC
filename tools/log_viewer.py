#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
flog 日志解析 + 可视化

把串口日志(含 shell 回显等杂乱内容)里的 flog CSV 数据解析出来，
生成一个**单文件、离线可用**的交互式 HTML 报告(无需联网、无第三方依赖)。

用法:
    python tools/log_viewer.py log.txt                  # 生成 log.html
    python tools/log_viewer.py log.txt -o report.html   # 指定输出
    python tools/log_viewer.py log.txt --open           # 生成后用浏览器打开
    type log.txt | python tools/log_viewer.py -          # 从标准输入读取

支持的日志内容(其余行自动忽略):
    ms,target,planned,angle,vel,vel_sp,uq     flog 表头(出现即视为新的一次记录)
    1234,2.000,0.512,0.498,1.203,1.500,0.812  数据行(行首允许有时间戳等前缀)
    msh >angle 2                              命令回显 -> 图上标注事件线
    ap     = 5.0000   angle loop P ...        tune 输出 -> 参数快照
    traj on: vmax 3.000 rad/s, ...            traj 输出 -> 参数快照

HTML 里: 滚轮缩放、拖动平移、双击复位、悬停读数、点击图例开关曲线，
并显示 跟踪误差/振荡主频/饱和比例 等统计。
"""

import argparse
import cmath
import json
import math
import os
import re
import sys
import webbrowser

COLUMNS = ["ms", "target", "planned", "angle", "vel", "vel_sp", "uq"]

_NUM = r"-?\d+(?:\.\d+)?"
_ROW_RE = re.compile(
    r"(?<![\d.,\-])(\d+)" + "".join(r",(" + _NUM + r")" for _ in range(6)) + r"(?![\d.])"
)
_HEADER_RE = re.compile(r"ms,target,planned,angle,vel,vel_sp,uq")
_ANSI_RE = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")
_CMD_RE = re.compile(r"msh\s*>\s*(angle|traj|tune|flog|speed)\b\s*(.*)$")
_TUNE_RE = re.compile(r"^\s*(ap|vp|vi|vd|vtf|vlim|vramp)\s*=\s*(-?\d+(?:\.\d+)?)\s*(.*)$")
_TRAJ_RE = re.compile(
    r"traj\s+(on|off):\s*vmax\s+(-?[\d.]+)\s*rad/s,\s*amax\s+(-?[\d.]+)\s*rad/s\^2,"
    r"\s*smooth\s+(-?[\d.]+)\s*ms,\s*vff\s+(-?[\d.]+)"
)


# --------------------------------------------------------------------------- 解析
# 见文件末尾 parse_log_with_origin / _parse_all


# --------------------------------------------------------------------------- 统计
def _fft(x):
    n = len(x)
    if n == 1:
        return x
    even = _fft(x[0::2])
    odd = _fft(x[1::2])
    out = [0j] * n
    for k in range(n // 2):
        t = cmath.exp(-2j * math.pi * k / n) * odd[k]
        out[k] = even[k] + t
        out[k + n // 2] = even[k] - t
    return out


def dominant_freq(t, y):
    """对不等间隔序列线性重采样后做 FFT，返回 (主频Hz, 幅值, 奈奎斯特频率Hz)；数据不足返回 None。"""
    n = len(t)
    if n < 32:
        return None
    dts = sorted(t[i + 1] - t[i] for i in range(n - 1) if t[i + 1] > t[i])
    if not dts:
        return None
    dt = dts[len(dts) // 2]
    if dt <= 0:
        return None
    total = t[-1] - t[0]
    m = int(total / dt) + 1
    size = 1
    while size * 2 <= min(m, 8192):
        size *= 2
    if size < 32:
        return None
    # 取最后 size 个点对应的时间窗(更关注稳态/振荡段)
    t_start = t[-1] - (size - 1) * dt
    yy = []
    j = 0
    for i in range(size):
        ti = t_start + i * dt
        while j + 1 < n - 1 and t[j + 1] < ti:
            j += 1
        t0, t1 = t[j], t[j + 1]
        a = 0.0 if t1 == t0 else min(max((ti - t0) / (t1 - t0), 0.0), 1.0)
        yy.append(y[j] * (1 - a) + y[j + 1] * a)
    mean = sum(yy) / size
    win = [(v - mean) * (0.5 - 0.5 * math.cos(2 * math.pi * i / (size - 1))) for i, v in enumerate(yy)]
    spec = _fft([complex(v) for v in win])
    best_k, best_mag = 0, 0.0
    for k in range(1, size // 2):
        mag = abs(spec[k])
        if mag > best_mag:
            best_k, best_mag = k, mag
    if best_k == 0:
        return None
    freq = best_k / (size * dt)
    amp = 2.0 * best_mag / (size * 0.5)       # 汉宁窗相干增益 0.5
    return freq, amp, 0.5 / dt


def compute_stats(cols):
    t, planned, angle = cols["t"], cols["planned"], cols["angle"]
    n = len(t)
    err = [p - a for p, a in zip(planned, angle)]
    st = {"n": n, "duration": t[-1] - t[0]}

    dts = [t[i + 1] - t[i] for i in range(n - 1)]
    dts_sorted = sorted(dts)
    st["dt_median_ms"] = dts_sorted[len(dts) // 2] * 1000.0
    st["dt_max_ms"] = dts_sorted[-1] * 1000.0

    st["err_rms"] = math.sqrt(sum(e * e for e in err) / n)
    st["err_max"] = max(abs(e) for e in err)
    tail = err[int(n * 0.9):] or err[-1:]
    st["err_final"] = sum(tail) / len(tail)

    # 静止段(规划角度几乎不变)的误差，用来衡量"停下来以后抖不抖"
    still = []
    for i in range(3, n - 3):
        if abs(planned[i + 3] - planned[i - 3]) < 1e-3:
            still.append(err[i])
    if len(still) >= 5:
        m = sum(still) / len(still)
        st["still_n"] = len(still)
        st["still_rms"] = math.sqrt(sum((e - m) ** 2 for e in still) / len(still))
        st["still_p2p"] = max(still) - min(still)

    for key in ("vel_sp", "uq", "vel"):
        peak = max(abs(v) for v in cols[key])
        st[key + "_peak"] = peak
        if peak > 1e-6:
            near = sum(1 for v in cols[key] if abs(v) >= 0.98 * peak)
            st[key + "_sat_pct"] = 100.0 * near / n

    f = dominant_freq(t, err)
    if f:
        st["osc_freq"], st["osc_amp"], st["nyquist"] = f
    return st


# --------------------------------------------------------------------------- 生成数据
def build_payload(sessions, events, params, source, skipped):
    out_sessions = []
    for idx, s in enumerate(sessions):
        rows = s["rows"]
        ms0 = rows[0][0]
        cols = {"t": [round((r[0] - ms0) / 1000.0, 4) for r in rows]}
        for i, name in enumerate(COLUMNS[1:], start=1):
            cols[name] = [r[i] for r in rows]
        cols["stats"] = compute_stats(cols)
        cols["id"] = idx
        cols["label"] = "记录 %d  (%d 点, %.1f s)" % (idx + 1, len(rows), cols["t"][-1])
        cols["events"] = [
            {"t": cols["t"][min(e["row"], len(rows) - 1)], "text": e["text"]}
            for e in events
            if e["session"] == s["orig"]
        ]
        cols["params"] = [
            {"t": cols["t"][min(p["row"], len(rows) - 1)], "text": p["text"]}
            for p in params
            if p["session"] == s["orig"]
        ]
        out_sessions.append(cols)
    return {"source": source, "skipped": skipped, "sessions": out_sessions}


# --------------------------------------------------------------------------- HTML
HTML_TEMPLATE = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>FOC 日志分析</title>
<style>
  :root { --bg:#12151b; --panel:#1a1f28; --line:#2a313d; --text:#d7dde8; --dim:#8590a2; --grid:#262d38; }
  * { box-sizing: border-box; }
  body { margin:0; font:13px/1.5 "Segoe UI","Microsoft YaHei",sans-serif; background:var(--bg); color:var(--text); }
  header { padding:12px 18px; border-bottom:1px solid var(--line); display:flex; gap:16px; align-items:center; flex-wrap:wrap; }
  header h1 { font-size:16px; margin:0; font-weight:600; }
  header .src { color:var(--dim); }
  select, button { background:var(--panel); color:var(--text); border:1px solid var(--line); border-radius:4px; padding:3px 8px; font:inherit; }
  button { cursor:pointer; } button:hover { border-color:#4a5568; }
  main { padding:12px 18px 40px; max-width:1500px; margin:0 auto; }
  .cards { display:grid; grid-template-columns:repeat(auto-fill,minmax(190px,1fr)); gap:8px; margin-bottom:12px; }
  .card { background:var(--panel); border:1px solid var(--line); border-radius:6px; padding:8px 10px; }
  .card .k { color:var(--dim); font-size:12px; } .card .v { font-size:17px; font-weight:600; }
  .card .s { color:var(--dim); font-size:11px; }
  .card.warn { border-color:#b7791f; } .card.warn .v { color:#f6ad55; }
  .panel { background:var(--panel); border:1px solid var(--line); border-radius:6px; margin-bottom:10px; padding:6px 8px 4px; }
  .phead { display:flex; align-items:center; gap:14px; flex-wrap:wrap; padding:0 2px 2px; }
  .phead .title { font-weight:600; }
  .chip { display:inline-flex; align-items:center; gap:5px; cursor:pointer; user-select:none; color:var(--text); }
  .chip i { width:14px; height:3px; display:inline-block; border-radius:2px; }
  .chip.off { opacity:.35; text-decoration:line-through; }
  canvas { width:100%; display:block; cursor:crosshair; }
  #tip { position:fixed; pointer-events:none; background:rgba(10,12,16,.92); border:1px solid #3a4352; border-radius:5px;
         padding:6px 9px; font-size:12px; display:none; z-index:10; white-space:pre; font-family:Consolas,monospace; }
  table { border-collapse:collapse; width:100%; font-family:Consolas,monospace; font-size:12px; }
  th, td { text-align:right; padding:2px 10px; border-bottom:1px solid var(--line); } th:first-child, td:first-child { text-align:left; }
  th { color:var(--dim); font-weight:500; }
  .hint { color:var(--dim); font-size:12px; margin:6px 0 10px; }
  .plist { color:var(--dim); font-family:Consolas,monospace; font-size:12px; }
  .plist b { color:var(--text); font-weight:500; }
</style>
</head>
<body>
<header>
  <h1>FOC 日志分析</h1>
  <span class="src" id="src"></span>
  <label>记录 <select id="sess"></select></label>
  <button id="reset">复位视图</button>
</header>
<main>
  <div class="cards" id="cards"></div>
  <div class="hint">滚轮：以鼠标位置缩放时间轴　拖动：平移　双击：复位　点击图例：开关曲线　虚线竖线：命令/参数事件</div>
  <div id="panels"></div>
  <div class="panel"><div class="phead"><span class="title">可见区间统计</span><span class="chip" style="cursor:default" id="rangeLabel"></span></div>
    <table id="stats"></table></div>
  <div class="panel" id="paramBox"><div class="phead"><span class="title">日志中出现的命令 / 参数</span></div><div class="plist" id="plist"></div></div>
</main>
<div id="tip"></div>
<script>
const DATA = __DATA__;

// ---------------------------------------------------------------- 面板定义
const PANELS = [
  { title:"角度 (rad)", series:[
      {key:"target",  name:"target 目标",   color:"#a0aec0", dash:[6,4]},
      {key:"planned", name:"planned 规划",  color:"#63b3ed"},
      {key:"angle",   name:"angle 实际",    color:"#f6ad55"} ]},
  { title:"跟踪误差 planned - angle (rad)", series:[
      {key:"err",     name:"误差",          color:"#fc8181", derived:(d,i)=>d.planned[i]-d.angle[i]} ], zero:true },
  { title:"速度 (rad/s)", series:[
      {key:"vel",     name:"vel 实际",      color:"#68d391"},
      {key:"vel_sp",  name:"vel_sp 位置环输出", color:"#b794f4"} ], zero:true },
  { title:"q 轴电压 uq (V)", series:[
      {key:"uq",      name:"uq",            color:"#f687b3"} ], zero:true },
];

// ---------------------------------------------------------------- 状态
let sess = null;            // 当前记录
let view = {x0:0, x1:1};    // 可见时间范围
let hoverT = null;
const hidden = {};          // series key -> true
const canvases = [];
const MARGIN = {l:62, r:14, t:6, b:22};
const PANEL_H = 200;

function getSeries(p, s) {
  return s.derived ? sess.t.map((_,i)=>s.derived(sess,i)) : sess[s.key];
}

function lowerBound(arr, v) { let lo=0, hi=arr.length; while(lo<hi){const m=(lo+hi)>>1; if(arr[m]<v) lo=m+1; else hi=m;} return lo; }

function niceStep(range, target) {
  const raw = range/target, p = Math.pow(10, Math.floor(Math.log10(raw))), f = raw/p;
  return (f<1.5?1:f<3?2:f<7?5:10)*p;
}
function fmt(v, step) {
  const d = Math.max(0, Math.min(6, -Math.floor(Math.log10(step||1))+ (step<1?0:0)));
  return v.toFixed(d);
}

// ---------------------------------------------------------------- 绘图
function drawPanel(pi) {
  const P = PANELS[pi], cv = canvases[pi], dpr = window.devicePixelRatio||1;
  const W = cv.clientWidth, H = PANEL_H;
  if (cv.width !== W*dpr || cv.height !== H*dpr) { cv.width = W*dpr; cv.height = H*dpr; cv.style.height = H+"px"; }
  const g = cv.getContext("2d"); g.setTransform(dpr,0,0,dpr,0,0); g.clearRect(0,0,W,H);
  const pw = W-MARGIN.l-MARGIN.r, ph = H-MARGIN.t-MARGIN.b;
  const t = sess.t, i0 = Math.max(0, lowerBound(t, view.x0)-1), i1 = Math.min(t.length-1, lowerBound(t, view.x1));
  const X = x => MARGIN.l + (x-view.x0)/(view.x1-view.x0)*pw;

  // y 范围：取可见曲线的范围
  let ymin=Infinity, ymax=-Infinity; const vis = [];
  for (const s of P.series) { if (hidden[s.key]) continue; const a = getSeries(P,s); vis.push([s,a]);
    for (let i=i0;i<=i1;i++){ const v=a[i]; if(v<ymin)ymin=v; if(v>ymax)ymax=v; } }
  if (!isFinite(ymin)) { ymin=-1; ymax=1; }
  if (P.zero) { ymin=Math.min(ymin,0); ymax=Math.max(ymax,0); }
  if (ymax-ymin < 1e-9) { ymax+=0.5; ymin-=0.5; }
  const pad=(ymax-ymin)*0.08; ymin-=pad; ymax+=pad;
  const Y = y => MARGIN.t + (1-(y-ymin)/(ymax-ymin))*ph;

  g.font = "11px Consolas, monospace"; g.textBaseline="middle";
  // 网格 + y 刻度
  const ys = niceStep(ymax-ymin, 5);
  g.strokeStyle="#262d38"; g.fillStyle="#8590a2"; g.lineWidth=1; g.textAlign="right";
  for (let y=Math.ceil(ymin/ys)*ys; y<=ymax; y+=ys) { const py=Math.round(Y(y))+.5;
    g.beginPath(); g.moveTo(MARGIN.l,py); g.lineTo(W-MARGIN.r,py); g.stroke(); g.fillText(fmt(y,ys), MARGIN.l-6, py); }
  // x 刻度
  const xs = niceStep(view.x1-view.x0, Math.max(4, pw/110)); g.textAlign="center"; g.textBaseline="top";
  for (let x=Math.ceil(view.x0/xs)*xs; x<=view.x1; x+=xs) { const px=Math.round(X(x))+.5;
    g.beginPath(); g.moveTo(px,MARGIN.t); g.lineTo(px,MARGIN.t+ph); g.stroke(); g.fillText(fmt(x,xs)+"s", px, MARGIN.t+ph+4); }
  // 零线
  if (ymin<0 && ymax>0) { g.strokeStyle="#3b4454"; const py=Math.round(Y(0))+.5; g.beginPath(); g.moveTo(MARGIN.l,py); g.lineTo(W-MARGIN.r,py); g.stroke(); }

  g.save(); g.beginPath(); g.rect(MARGIN.l, MARGIN.t, pw, ph); g.clip();
  // 事件线
  for (const e of sess.events) { if (e.t<view.x0||e.t>view.x1) continue; const px=Math.round(X(e.t))+.5;
    g.strokeStyle="rgba(250,240,137,.45)"; g.setLineDash([2,3]); g.beginPath(); g.moveTo(px,MARGIN.t); g.lineTo(px,MARGIN.t+ph); g.stroke(); g.setLineDash([]);
    if (pi===0) { g.fillStyle="#faf089"; g.textAlign="left"; g.textBaseline="top"; g.fillText(e.text, px+3, MARGIN.t+2); } }
  // 曲线(点很多时按像素列取 min/max)
  for (const [s,a] of vis) {
    g.strokeStyle=s.color; g.lineWidth=1.4; g.setLineDash(s.dash||[]); g.beginPath();
    const n=i1-i0+1;
    if (n > pw*2) {
      let started=false;
      for (let px=0; px<pw; px++) {
        const ta=view.x0+px/pw*(view.x1-view.x0), tb=view.x0+(px+1)/pw*(view.x1-view.x0);
        const ja=Math.max(i0,lowerBound(t,ta)), jb=Math.min(i1,lowerBound(t,tb)); if (ja>jb) continue;
        let lo=a[ja], hi=a[ja]; for (let j=ja;j<=jb;j++){ if(a[j]<lo)lo=a[j]; if(a[j]>hi)hi=a[j]; }
        const x=MARGIN.l+px+.5; if(!started){g.moveTo(x,Y(lo));started=true;} else g.lineTo(x,Y(lo)); g.lineTo(x,Y(hi));
      }
    } else {
      for (let i=i0;i<=i1;i++){ const x=X(t[i]), y=Y(a[i]); i===i0?g.moveTo(x,y):g.lineTo(x,y); }
    }
    g.stroke(); g.setLineDash([]);
    if (n <= pw/4) { g.fillStyle=s.color; for (let i=i0;i<=i1;i++){ g.beginPath(); g.arc(X(t[i]),Y(a[i]),1.8,0,6.3); g.fill(); } }
  }
  g.restore();
  // 十字光标
  if (hoverT!==null && hoverT>=view.x0 && hoverT<=view.x1) { const px=Math.round(X(hoverT))+.5;
    g.strokeStyle="rgba(215,221,232,.5)"; g.beginPath(); g.moveTo(px,MARGIN.t); g.lineTo(px,MARGIN.t+ph); g.stroke(); }
  // 边框
  g.strokeStyle="#3b4454"; g.strokeRect(MARGIN.l+.5, MARGIN.t+.5, pw, ph);
}
function drawAll(){ PANELS.forEach((_,i)=>drawPanel(i)); updateStats(); }

// ---------------------------------------------------------------- 交互
function xToT(cv, clientX) { const r=cv.getBoundingClientRect(), pw=r.width-MARGIN.l-MARGIN.r;
  return view.x0 + (clientX-r.left-MARGIN.l)/pw*(view.x1-view.x0); }

function setupInteraction(cv, pi) {
  let drag=null;
  cv.addEventListener("wheel", ev => { ev.preventDefault();
    const tc=xToT(cv,ev.clientX), k=Math.exp(ev.deltaY*0.0015), full=[0,sess.t[sess.t.length-1]];
    let a=tc-(tc-view.x0)*k, b=tc+(view.x1-tc)*k;
    if (b-a > full[1]-full[0]) { a=full[0]; b=full[1]; }
    if (b-a < 0.02) return; view={x0:a,x1:b}; drawAll(); }, {passive:false});
  cv.addEventListener("mousedown", ev => { drag={x:ev.clientX, v:{...view}}; });
  window.addEventListener("mouseup", ()=>{ drag=null; });
  window.addEventListener("mousemove", ev => { if (!drag) return;
    const r=cv.getBoundingClientRect(), pw=r.width-MARGIN.l-MARGIN.r, dt=-(ev.clientX-drag.x)/pw*(drag.v.x1-drag.v.x0);
    view={x0:drag.v.x0+dt, x1:drag.v.x1+dt}; drawAll(); });
  cv.addEventListener("dblclick", resetView);
  cv.addEventListener("mousemove", ev => { hoverT=xToT(cv,ev.clientX); if(!drag) drawAll(); showTip(ev); });
  cv.addEventListener("mouseleave", ()=>{ hoverT=null; document.getElementById("tip").style.display="none"; drawAll(); });
}

function showTip(ev) {
  const tip=document.getElementById("tip"), t=sess.t; let i=lowerBound(t,hoverT);
  if (i>0 && (i>=t.length || hoverT-t[i-1] < t[i]-hoverT)) i--; i=Math.min(i,t.length-1);
  const lines=["t = "+t[i].toFixed(3)+" s"];
  for (const P of PANELS) for (const s of P.series) { if (hidden[s.key]) continue;
    lines.push((s.name.split(" ")[0]+"          ").slice(0,9)+" "+getSeries(P,s)[i].toFixed(3)); }
  tip.textContent=lines.join("\n"); tip.style.display="block";
  tip.style.left=Math.min(window.innerWidth-190, ev.clientX+14)+"px"; tip.style.top=(ev.clientY+14)+"px";
}

function resetView(){ view={x0:sess.t[0], x1:sess.t[sess.t.length-1]}; drawAll(); }

// ---------------------------------------------------------------- 统计
function updateStats() {
  const t=sess.t, i0=Math.max(0,lowerBound(t,view.x0)), i1=Math.min(t.length-1,lowerBound(t,view.x1)-0);
  document.getElementById("rangeLabel").textContent = "t = "+view.x0.toFixed(2)+" ~ "+view.x1.toFixed(2)+" s  ("+(i1-i0+1)+" 点)";
  let html="<tr><th>信号</th><th>最小</th><th>最大</th><th>峰峰</th><th>平均</th><th>RMS</th></tr>";
  for (const P of PANELS) for (const s of P.series) {
    const a=getSeries(P,s); let mn=Infinity,mx=-Infinity,sum=0,sq=0,n=0;
    for (let i=i0;i<=i1;i++){ const v=a[i]; if(v<mn)mn=v; if(v>mx)mx=v; sum+=v; sq+=v*v; n++; }
    if(!n) continue; const f=v=>v.toFixed(4);
    html+="<tr><td style='color:"+s.color+"'>"+s.name+"</td><td>"+f(mn)+"</td><td>"+f(mx)+"</td><td>"+f(mx-mn)+"</td><td>"+f(sum/n)+"</td><td>"+f(Math.sqrt(sq/n))+"</td></tr>";
  }
  document.getElementById("stats").innerHTML=html;
}

function renderCards() {
  const st=sess.stats, cards=[];
  const add=(k,v,s,warn)=>cards.push('<div class="card'+(warn?' warn':'')+'"><div class="k">'+k+'</div><div class="v">'+v+'</div><div class="s">'+(s||"&nbsp;")+'</div></div>');
  add("数据点 / 时长", st.n+" / "+st.duration.toFixed(1)+" s", "采样间隔中位 "+st.dt_median_ms.toFixed(1)+" ms, 最大 "+st.dt_max_ms.toFixed(0)+" ms", st.dt_max_ms>st.dt_median_ms*3);
  add("跟踪误差 RMS", st.err_rms.toFixed(4)+" rad", "最大 "+st.err_max.toFixed(4)+" rad　末段均值 "+st.err_final.toFixed(4));
  if (st.still_rms!==undefined)
    add("静止段抖动", st.still_p2p.toFixed(4)+" rad p-p", "RMS "+st.still_rms.toFixed(4)+" rad，"+st.still_n+" 点 (规划角度不变时)", st.still_p2p>0.02);
  if (st.osc_freq!==undefined)
    add("误差主频 (末段)", st.osc_freq.toFixed(2)+" Hz", "幅值约 "+st.osc_amp.toFixed(4)+" rad，奈奎斯特 "+st.nyquist.toFixed(1)+" Hz", st.osc_amp>0.02);
  add("速度给定峰值", st.vel_sp_peak.toFixed(3)+" rad/s", st.vel_sp_sat_pct!==undefined? "接近峰值的点占 "+st.vel_sp_sat_pct.toFixed(1)+"%  (高=位置环饱和)":"", (st.vel_sp_sat_pct||0)>20);
  add("实际速度峰值", st.vel_peak.toFixed(3)+" rad/s", "");
  add("电压 uq 峰值", st.uq_peak.toFixed(3)+" V", st.uq_sat_pct!==undefined? "接近峰值的点占 "+st.uq_sat_pct.toFixed(1)+"%  (高=电压饱和)":"", (st.uq_sat_pct||0)>20);
  document.getElementById("cards").innerHTML=cards.join("");
}

function renderParams() {
  const items=sess.params, box=document.getElementById("paramBox");
  if (!items.length && !sess.events.length) { box.style.display="none"; return; } box.style.display="";
  const all=[...sess.events.map(e=>({t:e.t,text:"命令  "+e.text})), ...items.map(p=>({t:p.t,text:"参数  "+p.text}))].sort((a,b)=>a.t-b.t);
  document.getElementById("plist").innerHTML=all.map(x=>"<div><b>"+x.t.toFixed(2)+" s</b>　"+x.text.replace(/</g,"&lt;")+"</div>").join("");
}

// ---------------------------------------------------------------- 初始化
function buildPanels() {
  const root=document.getElementById("panels"); root.innerHTML=""; canvases.length=0;
  PANELS.forEach((P,pi)=>{
    const box=document.createElement("div"); box.className="panel";
    const head=document.createElement("div"); head.className="phead";
    head.innerHTML='<span class="title">'+P.title+'</span>';
    for (const s of P.series) { const c=document.createElement("span"); c.className="chip"+(hidden[s.key]?" off":"");
      c.innerHTML='<i style="background:'+s.color+'"></i>'+s.name; c.onclick=()=>{ hidden[s.key]=!hidden[s.key]; c.classList.toggle("off"); drawAll(); }; head.appendChild(c); }
    const cv=document.createElement("canvas"); box.appendChild(head); box.appendChild(cv); root.appendChild(box);
    canvases.push(cv); setupInteraction(cv,pi);
  });
}

function selectSession(i) { sess=DATA.sessions[i]; renderCards(); renderParams(); resetView(); }

(function init(){
  document.getElementById("src").textContent = DATA.source + (DATA.skipped? "　(忽略 "+DATA.skipped+" 行非数据内容)":"");
  const sel=document.getElementById("sess");
  DATA.sessions.forEach((s,i)=>{ const o=document.createElement("option"); o.value=i; o.textContent=s.label; sel.appendChild(o); });
  let best=0; DATA.sessions.forEach((s,i)=>{ if (s.stats.n > DATA.sessions[best].stats.n) best=i; });   // 默认显示数据点最多的一段
  sel.value=best; sel.onchange=()=>selectSession(+sel.value);
  document.getElementById("reset").onclick=resetView;
  buildPanels(); selectSession(best);
  window.addEventListener("resize", drawAll);
})();
</script>
</body>
</html>
"""


# --------------------------------------------------------------------------- 入口
def main():
    ap = argparse.ArgumentParser(description="解析串口日志中的 flog CSV 数据并生成交互式 HTML 报告")
    ap.add_argument("log", help="日志文件路径，- 表示标准输入")
    ap.add_argument("-o", "--output", help="输出 HTML 路径，默认与日志同名 .html")
    ap.add_argument("--open", action="store_true", help="生成后用默认浏览器打开")
    ap.add_argument("--encoding", default="utf-8", help="日志文件编码，默认 utf-8 (遇到乱码可试 gbk)")
    args = ap.parse_args()

    if args.log == "-":
        lines = sys.stdin.read().splitlines()
        source = "stdin"
        out = args.output or "flog_report.html"
    else:
        with open(args.log, "r", encoding=args.encoding, errors="replace") as f:
            lines = f.read().splitlines()
        source = os.path.basename(args.log)
        out = args.output or os.path.splitext(args.log)[0] + ".html"

    sessions, events, params, skipped = parse_log_with_origin(lines)
    if not sessions:
        print("未在日志中找到有效的 flog 数据行 (期望格式: ms,target,planned,angle,vel,vel_sp,uq)", file=sys.stderr)
        return 1

    payload = build_payload(sessions, events, params, source, skipped)
    html = HTML_TEMPLATE.replace("__DATA__", json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
    with open(out, "w", encoding="utf-8") as f:
        f.write(html)

    total = sum(len(s["rows"]) for s in sessions)
    print("解析完成: %d 段记录, 共 %d 个数据点, 忽略 %d 行其他内容" % (len(sessions), total, skipped))
    for i, s in enumerate(payload["sessions"]):
        st = s["stats"]
        line = "  记录 %d: %d 点 %.1fs  误差RMS=%.4f rad  最大=%.4f rad" % (
            i + 1, st["n"], st["duration"], st["err_rms"], st["err_max"])
        if "still_p2p" in st:
            line += "  静止抖动p-p=%.4f rad" % st["still_p2p"]
        if "osc_freq" in st:
            line += "  误差主频=%.2fHz" % st["osc_freq"]
        print(line)
    print("已生成: " + os.path.abspath(out))
    if args.open:
        webbrowser.open("file:///" + os.path.abspath(out).replace("\\", "/"))
    return 0


def parse_log_with_origin(lines):
    """丢弃过短的记录(<2 点)，并给保留下来的记录保留原始编号(orig)，供事件/参数挂靠。"""
    sessions_all, events, params, skipped = _parse_all(lines)
    sessions = []
    for orig, s in enumerate(sessions_all):
        if len(s["rows"]) >= 2:
            s["orig"] = orig
            sessions.append(s)
    return sessions, events, params, skipped


def _parse_all(lines):
    """逐行解析，返回 (sessions, events, params, skipped)；事件/参数挂靠到其后的第一个数据行。"""
    sessions, events, params = [], [], []
    pending_events, pending_params = [], []
    cur = None
    skipped = 0
    for raw in lines:
        line = _ANSI_RE.sub("", raw).rstrip("\r\n")
        if _HEADER_RE.search(line):
            sessions.append({"rows": []})
            cur = len(sessions) - 1
            continue
        m = _ROW_RE.search(line)
        if m:
            vals = [float(x) for x in m.groups()]
            if cur is None or (sessions[cur]["rows"] and vals[0] < sessions[cur]["rows"][-1][0]):
                sessions.append({"rows": []})
                cur = len(sessions) - 1
            rows = sessions[cur]["rows"]
            for text in pending_events:
                events.append({"session": cur, "row": len(rows), "text": text})
            for text in pending_params:
                params.append({"session": cur, "row": len(rows), "text": text})
            pending_events, pending_params = [], []
            rows.append(vals)
            continue
        m = _CMD_RE.search(line)
        if m:
            pending_events.append((m.group(1) + " " + m.group(2)).strip())
            continue
        m = _TUNE_RE.match(line)
        if m:
            pending_params.append("%s=%s" % (m.group(1), m.group(2)))
            continue
        m = _TRAJ_RE.search(line)
        if m:
            on, vmax, amax, smooth, vff = m.groups()
            pending_params.append("traj %s vmax=%s amax=%s smooth=%sms vff=%s" % (on, vmax, amax, smooth, vff))
            continue
        if line.strip():
            skipped += 1
    return sessions, events, params, skipped


if __name__ == "__main__":
    sys.exit(main())
