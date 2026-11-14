#!/usr/bin/env python3
"""Roadmap 074, the strategy atlas: atlas.json -> one self-contained atlas.html.

The page has no dependency: plain SVG drawn by inline JavaScript, the data inlined as JSON.
Views: the progress grid (relations x tiers); per scenario (1) the context class map, one strip
per tier where tiers disagree, (2) the payoff scatter per context (Pareto front, Nash
equilibria, the optimum, the Autoware pair, welfare iso-lines, a context slider), (3) the
double oracle's meta-game matrix (best-response arrows, NE outlined, a round slider), (6) the
episode view (time-space diagram, 2D TTC, clearance and headway strips against their
references, tiers overlaid); shared (4) parallel coordinates of the rule parameters weighted by
the equilibrium mixture, fitted Autoware rules highlighted, and (5) the rule families' state
machines; the N-car residual maps, conflict-set histograms and N4's error against N.

Usage:
  scripts/coop/atlas/atlas_build.py --atlas <dir>/atlas.json --out <dir>/atlas.html
"""

from __future__ import annotations

import argparse
import json
import os

MAX_BYTES = 16 * 1024 * 1024

CSS = r"""
:root{
  --bg:#f7f7f5; --panel:#ffffff; --ink:#1d1f22; --muted:#5e646c; --line:#d9dcdf; --accent:#2457c5;
  --good:#1f8a4c; --bad:#c0392b; --warn:#b7791f; --grid:#eceef0;
  --c0:#4e79a7; --c1:#f28e2b; --c2:#59a14f; --c3:#e15759; --c4:#76b7b2; --c5:#edc948;
  --c6:#b07aa1; --c7:#9c755f; --c8:#ff9da7; --c9:#8c8c8c;
  --tA:#2457c5; --tB:#c0392b; --tier0:#4e79a7; --tier1:#f28e2b; --tier2:#59a14f; --tier3:#b07aa1; --tier4:#9c755f;
  --neg:#c0392b; --pos:#2457c5; --zero:#f2f2f2;
}
@media (prefers-color-scheme: dark){
  :root:not([data-theme="light"]){
    --bg:#15171a; --panel:#1e2125; --ink:#e7e9ec; --muted:#a0a6ae; --line:#3a3f45; --accent:#7aa2ff;
    --good:#4cc38a; --bad:#ff7a6b; --warn:#e3b341; --grid:#2a2e33;
    --c0:#7aa6d6; --c1:#ffad5c; --c2:#7fcf74; --c3:#ff8a8c; --c4:#93d6d0; --c5:#f5dc6e;
    --c6:#d1a0c6; --c7:#c39a82; --c8:#ffbdc4; --c9:#a8a8a8;
    --tA:#7aa2ff; --tB:#ff7a6b; --tier0:#7aa6d6; --tier1:#ffad5c; --tier2:#7fcf74; --tier3:#d1a0c6; --tier4:#c39a82;
    --neg:#ff7a6b; --pos:#7aa2ff; --zero:#2a2e33;
  }
}
:root[data-theme="dark"]{
  --bg:#15171a; --panel:#1e2125; --ink:#e7e9ec; --muted:#a0a6ae; --line:#3a3f45; --accent:#7aa2ff;
  --good:#4cc38a; --bad:#ff7a6b; --warn:#e3b341; --grid:#2a2e33;
  --c0:#7aa6d6; --c1:#ffad5c; --c2:#7fcf74; --c3:#ff8a8c; --c4:#93d6d0; --c5:#f5dc6e;
  --c6:#d1a0c6; --c7:#c39a82; --c8:#ffbdc4; --c9:#a8a8a8;
  --tA:#7aa2ff; --tB:#ff7a6b; --tier0:#7aa6d6; --tier1:#ffad5c; --tier2:#7fcf74; --tier3:#d1a0c6; --tier4:#c39a82;
  --neg:#ff7a6b; --pos:#7aa2ff; --zero:#2a2e33;
}
*{box-sizing:border-box}
html,body{margin:0;padding:0}
body{background:var(--bg);color:var(--ink);font:14px/1.45 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;overflow-x:hidden}
header{padding:16px;border-bottom:1px solid var(--line);background:var(--panel);display:flex;flex-wrap:wrap;gap:8px 16px;align-items:baseline}
header h1{font-size:20px;margin:0}
header .sub{color:var(--muted);font-size:12px}
header .sp{flex:1}
nav{display:flex;flex-wrap:wrap;gap:4px;padding:8px 16px;background:var(--panel);border-bottom:1px solid var(--line);position:sticky;top:0;z-index:5}
nav button,.btn{background:transparent;border:1px solid var(--line);color:var(--ink);border-radius:6px;padding:4px 10px;font:inherit;cursor:pointer}
nav button.on{background:var(--accent);border-color:var(--accent);color:var(--panel)}
main{max-width:1180px;margin:0 auto;padding:16px}
section.view{display:none}
section.view.on{display:block}
.card{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:12px 16px;margin:0 0 16px}
.card h2{font-size:16px;margin:0 0 8px}
.card h3{font-size:14px;margin:12px 0 6px}
.muted{color:var(--muted)}
.small{font-size:12px}
.row{display:flex;flex-wrap:wrap;gap:8px 16px;align-items:center;margin:4px 0 8px}
label{font-size:12px;color:var(--muted)}
select,input[type=range]{font:inherit;color:var(--ink);background:var(--panel);border:1px solid var(--line);border-radius:4px;max-width:100%}
input[type=range]{width:min(420px,100%)}
.scroll{overflow-x:auto;max-width:100%}
table{border-collapse:collapse;font-size:12px}
th,td{border:1px solid var(--line);padding:4px 6px;text-align:left;vertical-align:top}
th{background:var(--grid)}
.badge{display:inline-block;border-radius:10px;padding:0 8px;font-size:11px;border:1px solid var(--line)}
.st-done{background:var(--good);color:var(--panel);border-color:var(--good)}
.st-planned{background:var(--grid);color:var(--muted)}
.st-missing,.st-empty{background:var(--warn);color:var(--panel);border-color:var(--warn)}
svg{display:block;width:100%;height:auto}
svg text{fill:var(--ink);font-size:11px;font-family:inherit}
svg .ax{stroke:var(--muted);stroke-width:1}
svg .gl{stroke:var(--grid);stroke-width:1}
#tip{position:fixed;pointer-events:none;background:var(--panel);color:var(--ink);border:1px solid var(--line);border-radius:6px;padding:6px 8px;font-size:12px;max-width:min(360px,90vw);box-shadow:0 2px 8px rgba(0,0,0,.2);display:none;z-index:10;white-space:pre-wrap}
.legend{display:flex;flex-wrap:wrap;gap:4px 12px;font-size:12px}
.legend span.sw{display:inline-block;width:12px;height:12px;border-radius:2px;margin-right:4px;vertical-align:-2px}
.grid2{display:grid;grid-template-columns:1fr;gap:16px}
@media (min-width:900px){.grid2{grid-template-columns:1fr 1fr}}
.sm{display:grid;grid-template-columns:repeat(auto-fill,minmax(260px,1fr));gap:12px}
"""

JS = r"""
const A = JSON.parse(document.getElementById('atlas-data').textContent);
const NS = 'http://www.w3.org/2000/svg';
const TIERS = A.tiers;
const TIER_COL = t => `var(--tier${Math.max(0, TIERS.indexOf(t)) % 5})`;
const tip = document.getElementById('tip');
function showTip(ev, txt){ tip.textContent = txt; tip.style.display = 'block';
  const x = Math.min(ev.clientX + 12, window.innerWidth - tip.offsetWidth - 8), y = Math.min(ev.clientY + 12, window.innerHeight - tip.offsetHeight - 8);
  tip.style.left = x + 'px'; tip.style.top = y + 'px'; }
function hideTip(){ tip.style.display = 'none'; }
function el(tag, attrs, parent, text){ const e = document.createElementNS(NS, tag);
  for (const k in (attrs || {})) e.setAttribute(k, attrs[k]); if (text !== undefined) e.textContent = text; if (parent) parent.appendChild(e); return e; }
function h(tag, attrs, parent, text){ const e = document.createElement(tag);
  for (const k in (attrs || {})) { if (k === 'class') e.className = attrs[k]; else e.setAttribute(k, attrs[k]); }
  if (text !== undefined) e.textContent = text; if (parent) parent.appendChild(e); return e; }
function svg(parent, w, hgt){ const e = el('svg', {viewBox: `0 0 ${w} ${hgt}`, preserveAspectRatio: 'xMidYMin meet'}, parent); e.style.maxWidth = w + 'px'; return e; }
function tipOn(node, txt){ node.addEventListener('mousemove', e => showTip(e, typeof txt === 'function' ? txt() : txt));
  node.addEventListener('mouseleave', hideTip); node.addEventListener('click', e => showTip(e, typeof txt === 'function' ? txt() : txt)); }
const fmt = (v, d = 3) => v === null || v === undefined ? '-' : (typeof v === 'number' ? (+v.toFixed(d)).toString() : String(v));
const nk = v => typeof v === 'number' ? (+v.toFixed(3)).toString() : String(v);
function scale(d0, d1, r0, r1){ const k = d1 === d0 ? 0 : (r1 - r0) / (d1 - d0); return v => r0 + (v - d0) * k; }
function niceTicks(lo, hi, n = 5){ if (lo === hi) { lo -= 1; hi += 1; } const span = hi - lo, step0 = span / n, mag = Math.pow(10, Math.floor(Math.log10(step0)));
  const step = [1, 2, 5, 10].map(m => m * mag).find(s => span / s <= n) || 10 * mag; const out = [];
  for (let v = Math.ceil(lo / step) * step; v <= hi + 1e-9; v += step) out.push(+v.toFixed(10)); return out; }
function axes(g, x, y, xr, yr, xl, yl, W, H, m){
  for (const t of niceTicks(xr[0], xr[1])) { el('line', {x1: x(t), x2: x(t), y1: m.t, y2: H - m.b, class: 'gl'}, g); el('text', {x: x(t), y: H - m.b + 14, 'text-anchor': 'middle'}, g, fmt(t, 2)); }
  for (const t of niceTicks(yr[0], yr[1])) { el('line', {x1: m.l, x2: W - m.r, y1: y(t), y2: y(t), class: 'gl'}, g); el('text', {x: m.l - 4, y: y(t) + 4, 'text-anchor': 'end'}, g, fmt(t, 2)); }
  el('line', {x1: m.l, x2: W - m.r, y1: H - m.b, y2: H - m.b, class: 'ax'}, g); el('line', {x1: m.l, x2: m.l, y1: m.t, y2: H - m.b, class: 'ax'}, g);
  el('text', {x: (m.l + W - m.r) / 2, y: H - 4, 'text-anchor': 'middle'}, g, xl); const yt = el('text', {x: 12, y: (m.t + H - m.b) / 2, 'text-anchor': 'middle', transform: `rotate(-90 12 ${(m.t + H - m.b) / 2})`}, g, yl);
}
function classColor(legendKeys, c){ const i = legendKeys.indexOf(c); if (c === '!') return 'var(--bad)'; if (c === null || c === undefined || c === '?') return 'var(--c9)';
  return `var(--c${(i < 0 ? 8 : i) % 9})`; }

// ---------------------------------------------------------------- tabs
const nav = document.getElementById('nav'), main = document.getElementById('main');
const views = {};
function addView(id, title){ const b = h('button', {}, nav, title); const s = h('section', {class: 'view', id: 'v-' + id}, main);
  views[id] = {b, s}; b.onclick = () => show(id); return s; }
function show(id){ for (const k in views) { views[k].b.classList.toggle('on', k === id); views[k].s.classList.toggle('on', k === id); }
  try { localStorage.setItem('atlas-tab', id); } catch (e) {} if (views[id].render) { views[id].render(); views[id].render = null; } }

// ---------------------------------------------------------------- progress grid
(function progress(){
  const s = addView('progress', 'Progress');
  const c = h('div', {class: 'card'}, s); h('h2', {}, c, 'Progress: relations x tiers');
  h('p', {class: 'muted small'}, c, 'Status per tier, cells loaded, D2 share where a comparison exists (payoffs inside the reference tier\'s 95 % interval). Generated ' + A.generated + '.');
  const sc = h('div', {class: 'scroll'}, c); const t = h('table', {}, sc); const hr = h('tr', {}, t); h('th', {}, hr, 'relation');
  for (const tier of TIERS) h('th', {}, hr, tier.replace('_', ' '));
  for (const sname in A.scenarios) { const tr = h('tr', {}, t); const th = h('th', {}, tr); const a = h('a', {href: '#'}, th, A.scenarios[sname].title || sname);
    a.onclick = e => { e.preventDefault(); show(sname); };
    for (const tier of TIERS) { const td = h('td', {}, tr); const ps = A.progress.filter(p => p.scenario === sname && p.tier === tier);
      if (!ps.length) { h('span', {class: 'muted'}, td, '-'); continue; }
      for (const p of ps) { const b = h('span', {class: 'badge st-' + p.status}, td, p.status); h('br', {}, td);
        if (p.cells) h('span', {class: 'small'}, td, p.cells + ' cells ');
        if (p.d2) h('span', {class: 'small'}, td, `D2 ${p.d2.within}/${p.d2.n} (${Math.round(100 * p.d2.share)} %)`);
        if (p.label) { h('br', {}, td); h('span', {class: 'small muted'}, td, p.label); }
        if (p.link) { h('br', {}, td); h('span', {class: 'small muted'}, td, 'NAS: ' + p.link); } } } }
  const n = h('div', {class: 'card'}, s); h('h2', {}, n, 'Notes'); const ul = h('ul', {class: 'small'}, n); for (const x of A.notes || []) h('li', {}, ul, x);
})();

// ---------------------------------------------------------------- per scenario
function cellsMatch(ctx, sel){ for (const f in sel) { if (ctx[f] !== undefined && ctx[f] !== null && nk(ctx[f]) !== sel[f]) return false; } return true; }

function scenarioView(sname, S){
  const s = addView(sname, S.title || sname);
  views[sname].render = () => {
    if (S.ncar) return ncarView(s);
    classMap(s, sname, S); payoffView(s, sname, S); metaView(s, S); episodeView(s, S);
    if ((S.state_machines || []).length) { const c = h('div', {class: 'card'}, s); h('h2', {}, c, '(5) The rules as state machines'); const g = h('div', {class: 'sm'}, c);
      for (const k of S.state_machines) drawSM(g, k); }
  };
}

function classMap(s, sname, S){
  const c = h('div', {class: 'card'}, s); h('h2', {}, c, '(1) Context class map');
  const [ax, ay] = S.axes; const facets = (S.facets || []);
  const tiers = Object.keys(S.tiers).filter(t => S.tiers[t].cells.some(x => x.class !== undefined));
  const all = []; for (const t of tiers) for (const x of S.tiers[t].cells) all.push([t, x]);
  const row = h('div', {class: 'row'}, c); const sel = {};
  for (const f of facets) { const vals = [...new Set(all.map(([, x]) => x.ctx[f]).filter(v => v !== undefined && v !== null).map(nk))].sort((a, b) => isNaN(a) ? a.localeCompare(b) : a - b);
    if (!vals.length) continue; const lab = h('label', {}, row, (S.labels || {})[f] || f); const sl = h('select', {}, lab); for (const v of vals) h('option', {value: v}, sl, v);
    const cnt = v => all.filter(([, x]) => nk(x.ctx[f]) === v).length; sel[f] = vals.reduce((a, b) => cnt(b) > cnt(a) ? b : a, vals[0]); sl.value = sel[f]; sl.onchange = () => { sel[f] = sl.value; draw(); }; }
  const tsel = {}; for (const t of tiers) { const lab = h('label', {}, row); const cb = h('input', {type: 'checkbox'}, lab); cb.checked = true; tsel[t] = true;
    lab.append(' ' + t.replace('_', ' ')); cb.onchange = () => { tsel[t] = cb.checked; draw(); }; }
  const box = h('div', {}, c); const leg = h('div', {class: 'legend'}, c);
  const lk = Object.keys(S.legend || {});
  for (const k of lk) { const sp = h('span', {}, leg); const sw = h('span', {class: 'sw'}, sp); sw.style.background = classColor(lk, k); sp.append(`${k} ${S.legend[k]}`); }
  h('p', {class: 'small muted'}, c, 'A cell split into strips: the tiers disagree there (one strip per tier, in the order of the checkboxes). Hover or tap for the equilibria and the loss C_eq - C_opt.');
  function draw(){
    box.innerHTML = '';
    const use = all.filter(([t, x]) => tsel[t] && cellsMatch(x.ctx, sel) && x.ctx[ax] !== undefined && x.ctx[ay] !== undefined);
    const xs = [...new Set(use.map(([, x]) => x.ctx[ax]))].sort((a, b) => a - b), ys = [...new Set(use.map(([, x]) => x.ctx[ay]))].sort((a, b) => a - b);
    if (!xs.length) { h('p', {class: 'muted'}, box, 'No cells for this selection.'); return; }
    const cw = Math.max(10, Math.min(36, 900 / xs.length)), ch = Math.max(14, Math.min(30, 420 / ys.length));
    const m = {l: 60, r: 10, t: 10, b: 44}, W = m.l + m.r + cw * xs.length, H = m.t + m.b + ch * ys.length;
    const g = svg(box, W, H); const byKey = {};
    for (const [t, x] of use) { const k = nk(x.ctx[ax]) + '|' + nk(x.ctx[ay]); (byKey[k] = byKey[k] || []).push([t, x]); }
    xs.forEach((xv, i) => { if (xs.length < 30 || i % Math.ceil(xs.length / 20) === 0) el('text', {x: m.l + cw * (i + 0.5), y: H - m.b + 14, 'text-anchor': 'middle'}, g, fmt(xv, 2)); });
    ys.forEach((yv, j) => el('text', {x: m.l - 4, y: m.t + ch * (ys.length - j - 0.5) + 4, 'text-anchor': 'end'}, g, fmt(yv, 2)));
    el('text', {x: m.l + cw * xs.length / 2, y: H - 6, 'text-anchor': 'middle'}, g, (S.labels || {})[ax] || ax);
    el('text', {x: 12, y: m.t + ch * ys.length / 2, 'text-anchor': 'middle', transform: `rotate(-90 12 ${m.t + ch * ys.length / 2})`}, g, (S.labels || {})[ay] || ay);
    xs.forEach((xv, i) => ys.forEach((yv, j) => {
      const list = byKey[nk(xv) + '|' + nk(yv)]; if (!list) return;
      const per = {}; for (const [t, x] of list) if (!per[t]) per[t] = x; const ts = Object.keys(per);
      const agree = new Set(ts.map(t => per[t].class)).size === 1;
      const X = m.l + cw * i, Y = m.t + ch * (ys.length - j - 1);
      const parts = agree ? [ts[0]] : ts;
      parts.forEach((t, k) => { const r = el('rect', {x: X + cw * k / parts.length, y: Y, width: cw / parts.length, height: ch}, g); r.style.fill = classColor(lk, per[t].class); r.style.stroke = 'var(--panel)'; r.style.strokeWidth = 0.5; });
      if (cw >= 16 && agree) { const tt = el('text', {x: X + cw / 2, y: Y + ch / 2 + 4, 'text-anchor': 'middle'}, g, per[ts[0]].class || ''); tt.style.fill = '#111'; tt.style.fontSize = '10px'; }
      const hit = el('rect', {x: X, y: Y, width: cw, height: ch, fill: 'transparent'}, g);
      tipOn(hit, () => `${ax} ${fmt(xv)}  ${ay} ${fmt(yv)}\n` + ts.map(t => { const x = per[t];
        const eq = (x.eq || []).slice(0, 4).map(e => e.join('/')).join(' ') + ((x.eq || []).length > 4 ? ` (+${x.eq.length - 4})` : '');
        return `${t}: ${x.class}${eq ? '  NE ' + eq : ''}${x.loss !== undefined && x.loss !== null ? '  loss ' + fmt(x.loss) : ''}${x.outcomes ? '  runs ' + x.outcomes : ''}${x.eq_summary ? '  ' + x.eq_summary : ''}`; }).join('\n'));
    }));
  }
  draw();
}

function pareto(pts){ const s = [...pts].sort((a, b) => b[2] - a[2] || b[3] - a[3]); const f = []; let best = -Infinity;
  for (const p of s) if (p[3] > best + 1e-12) { f.push(p); best = p[3]; } return f; }

function payoffView(s, sname, S){
  const items = []; for (const t in S.tiers) for (const p of S.tiers[t].payoffs) if (p.pairs && p.pairs.length) items.push([t, p]);
  if (!items.length) return;
  const c = h('div', {class: 'card'}, s); h('h2', {}, c, '(2) Payoffs per context');
  h('p', {class: 'small muted'}, c, 'Each point a rule pair (U_A, U_B). Line: the Pareto front. Ringed: Nash equilibria. Star: the optimum (max U_A + U_B). Square: the Autoware pair (swb tiers). Dashed: welfare iso-lines U_A + U_B = const.');
  const row = h('div', {class: 'row'}, c); const tl = h('label', {}, row, 'tier '); const tsel = h('select', {}, tl);
  const tiers = [...new Set(items.map(i => i[0]))]; for (const t of tiers) h('option', {value: t}, tsel, t);
  const sl = h('label', {}, row, 'context '); const rng = h('input', {type: 'range', min: 0, value: 0, step: 1}, sl); const cap = h('span', {class: 'small'}, row);
  const box = h('div', {}, c);
  let list = [];
  function pick(){ list = items.filter(i => i[0] === tsel.value); rng.max = Math.max(0, list.length - 1);
    const d = S.tiers[tsel.value].payoffs; const want = list.findIndex(i => i[1].eq && i[1].eq.length > 1); rng.value = Math.max(0, want); draw(); }
  function draw(){
    box.innerHTML = ''; const it = list[+rng.value]; if (!it) return; const p = it[1];
    cap.textContent = Object.entries(p.ctx).map(([k, v]) => `${k} ${fmt(v)}`).join('  ') + (p.label ? '  (' + p.label + ')' : '');
    const pts = p.pairs.filter(q => q[2] !== null && q[3] !== null);
    const W = 640, H = 400, m = {l: 56, r: 14, t: 12, b: 40};
    let xr = [Math.min(...pts.map(q => q[2])), Math.max(...pts.map(q => q[2]))], yr = [Math.min(...pts.map(q => q[3])), Math.max(...pts.map(q => q[3]))];
    const pad = r => { const d = (r[1] - r[0]) || 0.1; return [r[0] - 0.08 * d, r[1] + 0.08 * d]; }; xr = pad(xr); yr = pad(yr);
    const x = scale(xr[0], xr[1], m.l, W - m.r), y = scale(yr[0], yr[1], H - m.b, m.t);
    const g = svg(box, W, H); axes(g, x, y, xr, yr, 'U_A', 'U_B', W, H, m);
    const clip = el('clipPath', {id: 'cp-' + sname}, g); el('rect', {x: m.l, y: m.t, width: W - m.l - m.r, height: H - m.t - m.b}, clip);
    const gi = el('g', {'clip-path': `url(#cp-${sname})`}, g);
    const smin = xr[0] + yr[0], smax = xr[1] + yr[1];
    for (const sv of niceTicks(smin, smax, 6)) { const ln = el('line', {x1: x(xr[0]), y1: y(sv - xr[0]), x2: x(xr[1]), y2: y(sv - xr[1])}, gi); ln.style.stroke = 'var(--muted)'; ln.style.strokeDasharray = '3 4'; ln.style.opacity = 0.5; }
    const f = pareto(pts).sort((a, b) => a[2] - b[2]); if (f.length > 1) { const pl = el('polyline', {points: f.map(q => `${x(q[2])},${y(q[3])}`).join(' '), fill: 'none'}, gi); pl.style.stroke = 'var(--accent)'; pl.style.strokeWidth = 1.5; }
    const eqs = new Set((p.eq || []).map(e => e.join('|'))); const opt = pts.reduce((a, b) => (b[2] + b[3] > a[2] + a[3] ? b : a), pts[0]);
    const aw = p.aw ? p.aw.join('|') : null;
    for (const q of pts) { const k = q[0] + '|' + q[1]; const cx = x(q[2]), cy = y(q[3]);
      const dot = el('circle', {cx, cy, r: 4}, gi); dot.style.fill = 'var(--c0)'; dot.style.opacity = 0.75;
      if (eqs.has(k)) { const r = el('circle', {cx, cy, r: 8, fill: 'none'}, gi); r.style.stroke = 'var(--bad)'; r.style.strokeWidth = 2; }
      if (aw === k) { const r = el('rect', {x: cx - 7, y: cy - 7, width: 14, height: 14, fill: 'none'}, gi); r.style.stroke = 'var(--good)'; r.style.strokeWidth = 2; }
      if (q === opt) { const st = el('text', {x: cx, y: cy - 8, 'text-anchor': 'middle'}, gi, '★'); st.style.fill = 'var(--warn)'; st.style.fontSize = '16px'; }
      const hit = el('circle', {cx, cy, r: 8, fill: 'transparent'}, gi);
      tipOn(hit, `${q[0]} / ${q[1]}\nU_A ${fmt(q[2])}  U_B ${fmt(q[3])}  W ${fmt(q[2] + q[3])}${eqs.has(k) ? '\nNash equilibrium' : ''}${q === opt ? '\noptimum' : ''}`); }
  }
  tsel.onchange = pick; rng.oninput = draw; pick();
}

function metaView(s, S){
  if (!S.meta || !S.meta.length) return;
  const c = h('div', {class: 'card'}, s); h('h2', {}, c, '(3) The meta-game (double oracle)');
  h('p', {class: 'small muted'}, c, 'Rows: A\'s rules in the population at the round; columns: B\'s. Cell colour: welfare U_A + U_B (darker: worse); text U_A / U_B. Arrows: the unilateral best-response move (vertical: A, horizontal: B). Outlined: pure Nash equilibria of the restricted game. Bars: the round\'s equilibrium mixture.');
  const row = h('div', {class: 'row'}, c); const ml = h('label', {}, row, 'run '); const ms = h('select', {}, ml); S.meta.forEach((m, i) => h('option', {value: i}, ms, m.label));
  const rl = h('label', {}, row, 'round '); const rng = h('input', {type: 'range', min: 0, step: 1, value: 0}, rl); const cap = h('span', {class: 'small'}, row);
  const box = h('div', {class: 'scroll'}, c);
  function setRun(){ const m = S.meta[+ms.value]; rng.max = m.rounds.length - 1; rng.value = m.rounds.length - 1; draw(); }
  function draw(){
    box.innerHTML = ''; const m = S.meta[+ms.value]; const r = m.rounds[+rng.value]; if (!r) return;
    cap.textContent = `round ${r.round}: v_A ${fmt(r.vA)}, v_B ${fmt(r.vB)}, BR_A ${r.BR_A || '-'} (+${fmt(r.gain_A)}), BR_B ${r.BR_B || '-'} (+${fmt(r.gain_B)}), exploitability >= ${fmt(r.exploitability_lb)}`;
    const As = r.A_pop, Bs = r.B_pop; const T = k => m.table[k];
    const cw = 92, ch = 34, lw = 130, th = 70, bw = 40; const W = lw + cw * Bs.length + bw + 10, H = th + ch * As.length + bw;
    const g = svg(box, W, H); g.style.minWidth = Math.min(W, 900) + 'px';
    const ws = []; for (const a of As) for (const b of Bs) { const v = T(a + '|' + b); if (v) ws.push(v[0] + v[1]); }
    const wmin = Math.min(...ws), wmax = Math.max(...ws);
    const brA = {}, brB = {};
    for (const b of Bs) { let best = null; for (const a of As) { const v = T(a + '|' + b); if (v && (best === null || v[0] > T(best + '|' + b)[0] + 1e-9)) best = a; } brA[b] = best; }
    for (const a of As) { let best = null; for (const b of Bs) { const v = T(a + '|' + b); if (v && (best === null || v[1] > T(a + '|' + best)[1] + 1e-9)) best = b; } brB[a] = best; }
    Bs.forEach((b, j) => { const t = el('text', {x: lw + cw * (j + 0.5), y: th - 6, 'text-anchor': 'start', transform: `rotate(-30 ${lw + cw * (j + 0.5)} ${th - 6})`}, g, b); });
    As.forEach((a, i) => el('text', {x: lw - 6, y: th + ch * (i + 0.5) + 4, 'text-anchor': 'end'}, g, a));
    As.forEach((a, i) => Bs.forEach((b, j) => {
      const v = T(a + '|' + b); const X = lw + cw * j, Y = th + ch * i;
      const rc = el('rect', {x: X, y: Y, width: cw, height: ch}, g); rc.style.stroke = 'var(--line)';
      if (!v) { rc.style.fill = 'var(--grid)'; return; }
      const k = wmax === wmin ? 0.5 : (v[0] + v[1] - wmin) / (wmax - wmin); rc.style.fill = 'var(--c0)'; rc.style.fillOpacity = (0.12 + 0.6 * (1 - k)).toFixed(2);
      el('text', {x: X + cw / 2, y: Y + ch / 2 + 4, 'text-anchor': 'middle'}, g, `${fmt(v[0], 2)} / ${fmt(v[1], 2)}`);
      const ia = As.indexOf(brA[b]), jb = Bs.indexOf(brB[a]);
      if (ia !== i) { const d = Math.sign(ia - i); const ar = el('path', {d: `M${X + 8},${Y + ch / 2 - 6 * d} l0,${12 * d} m-3,${-4 * d} l3,${4 * d} l3,${-4 * d}`, fill: 'none'}, g); ar.style.stroke = 'var(--tA)'; ar.style.strokeWidth = 1.5; }
      if (jb !== j) { const d = Math.sign(jb - j); const ar = el('path', {d: `M${X + cw / 2 - 8 * d},${Y + ch - 6} l${16 * d},0 m${-4 * d},-3 l${4 * d},3 l${-4 * d},3`, fill: 'none'}, g); ar.style.stroke = 'var(--tB)'; ar.style.strokeWidth = 1.5; }
      if (ia === i && jb === j) { const o = el('rect', {x: X + 2, y: Y + 2, width: cw - 4, height: ch - 4, fill: 'none'}, g); o.style.stroke = 'var(--bad)'; o.style.strokeWidth = 3; }
      const hit = el('rect', {x: X, y: Y, width: cw, height: ch, fill: 'transparent'}, g);
      tipOn(hit, `${a} vs ${b}\nU_A ${fmt(v[0])}  U_B ${fmt(v[1])}  contact ${fmt(v[2])}  A ahead ${fmt(v[3])}`);
    }));
    As.forEach((a, i) => { const w = (r.A_mix || {})[a] || 0; if (w > 0) { const b = el('rect', {x: lw + cw * Bs.length + 4, y: th + ch * i + 6, width: bw * w, height: ch - 12}, g); b.style.fill = 'var(--tA)'; } });
    Bs.forEach((b, j) => { const w = (r.B_mix || {})[b] || 0; if (w > 0) { const bb = el('rect', {x: lw + cw * j + 6, y: th + ch * As.length + 4, width: cw - 12, height: (bw - 8) * w}, g); bb.style.fill = 'var(--tB)'; } });
  }
  ms.onchange = setRun; rng.oninput = draw; setRun();
}

function episodeView(s, S){
  const eps = S.episodes || []; if (!eps.length) return;
  const c = h('div', {class: 'card'}, s); h('h2', {}, c, '(6) Episodes');
  h('p', {class: 'small muted'}, c, 'Time-space: each car\'s position along its initial heading (A solid, B dashed), one colour per episode. Strips: 2D TTC, clearance and time headway against their references (dashed); PET over the run in the list. Kinematic episodes are re-simulated; swb episodes come from the agent\'s log (positions in Town03\'s frame, so their offset differs).');
  const groups = [...new Set(eps.map(e => e.group || ''))];
  const row = h('div', {class: 'row'}, c); const gl = h('label', {}, row, 'context '); const gs = h('select', {}, gl); groups.forEach(g => h('option', {value: g}, gs, g || '(none)'));
  const lst = h('div', {class: 'row small'}, c); const box = h('div', {}, c);
  let on = {};
  function setGroup(){ lst.innerHTML = ''; on = {}; const es = eps.filter(e => (e.group || '') === gs.value);
    es.forEach((e, i) => { const lab = h('label', {}, lst); const cb = h('input', {type: 'checkbox'}, lab); cb.checked = i < 4; on[e.id] = cb.checked;
      const sw = h('span', {class: 'sw'}); sw.style.cssText = `display:inline-block;width:10px;height:10px;margin:0 4px;background:var(--c${i % 9})`; lab.append(sw, `${e.id}  PET ${fmt(e.pet, 2)}`);
      cb.onchange = () => { on[e.id] = cb.checked; draw(); }; }); draw(); }
  function draw(){
    box.innerHTML = ''; const es = eps.filter(e => (e.group || '') === gs.value); const sel = es.map((e, i) => [e, i]).filter(([e]) => on[e.id]);
    if (!sel.length) return;
    const W = 680, H1 = 260, m = {l: 56, r: 12, t: 10, b: 36};
    const tmax = Math.max(...sel.map(([e]) => e.t[e.t.length - 1] || 0));
    const sv = []; for (const [e] of sel) for (const k of ['A', 'B']) for (const p of e[k]) sv.push(p[3] - e.A[0][3]);
    const yr = [Math.min(...sv), Math.max(...sv)];
    const x = scale(0, tmax, m.l, W - m.r), y = scale(yr[0], yr[1], H1 - m.b, m.t);
    const g = svg(box, W, H1); axes(g, x, y, [0, tmax], yr, 't (s)', 's - s_A(0) (m)', W, H1, m);
    for (const [e, i] of sel) for (const k of ['A', 'B']) { const pl = el('polyline', {points: e.t.map((t, n) => `${x(t)},${y(e[k][n][3] - e.A[0][3])}`).join(' '), fill: 'none'}, g);
      pl.style.stroke = `var(--c${i % 9})`; pl.style.strokeWidth = 1.8; if (k === 'B') pl.style.strokeDasharray = '5 3'; tipOn(pl, `${e.id} (${k})`); }
    const strips = [['ttc', '2D TTC (s)', 'ttc', 6], ['clear', 'clearance (m)', 'clear', null], ['thw', 'headway (s)', 'thw', 4]];
    for (const [key, lab, ref, cap] of strips) {
      const H2 = 120, m2 = {l: 56, r: 12, t: 8, b: 28}; const vals = []; for (const [e] of sel) for (const v of e[key]) if (v !== null) vals.push(cap ? Math.min(v, cap) : v);
      const rv = (sel[0][0].refs || {})[ref]; const hi = Math.max(...vals, rv || 0, 0.1); const yy = scale(0, hi, H2 - m2.b, m2.t);
      const g2 = svg(box, W, H2); axes(g2, x, yy, [0, tmax], [0, hi], 't (s)', lab, W, H2, m2);
      if (rv) { const rl = el('line', {x1: m2.l, x2: W - m2.r, y1: yy(rv), y2: yy(rv)}, g2); rl.style.stroke = 'var(--bad)'; rl.style.strokeDasharray = '4 3'; }
      for (const [e, i] of sel) { let d = '', pen = false; e.t.forEach((t, n) => { const v = e[key][n]; if (v === null) { pen = false; return; } d += (pen ? 'L' : 'M') + x(t) + ',' + yy(cap ? Math.min(v, cap) : v); pen = true; });
        const p = el('path', {d, fill: 'none'}, g2); p.style.stroke = `var(--c${i % 9})`; p.style.strokeWidth = 1.5; }
    }
  }
  gs.onchange = setGroup; setGroup();
}

// ---------------------------------------------------------------- N cars
function ncarView(s){
  const N = A.ncar || {};
  for (const R of N.ncar_residuals || []) {
    const c = h('div', {class: 'card'}, s); h('h2', {}, c, 'Residual map: ' + R.label);
    h('p', {class: 'small muted'}, c, 'U measured minus the composed model\'s prediction per (d0, dv), for one set, scene, context x, v_B, profile and car. Blue: the model is pessimistic, red: optimistic. A heavy outline: outside the D2 tolerance.');
    const keys = ['set', 'scene', 'x', 'vb', 'a', 'car']; const sel = {}; const row = h('div', {class: 'row'}, c); const cells = R.cells;
    const sels = {};
    for (const k of keys) { const lab = h('label', {}, row, k + ' '); const sl = h('select', {}, lab); sels[k] = sl; sl.onchange = () => { sel[k] = sl.value; refill(keys.indexOf(k) + 1); draw(); }; }
    function refill(from){ for (let i = from; i < keys.length; i++) { const k = keys[i]; const ok = cells.filter(x => keys.slice(0, i).every(q => nk(x[q]) === sel[q]));
      const vals = [...new Set(ok.map(x => nk(x[k])))].sort(); const sl = sels[k]; sl.innerHTML = ''; for (const v of vals) h('option', {value: v}, sl, v); sel[k] = vals[0]; } }
    const box = h('div', {}, c); const stat = h('p', {class: 'small'}, c);
    function draw(){ box.innerHTML = ''; const use = cells.filter(x => keys.every(q => nk(x[q]) === sel[q]));
      const d0s = [...new Set(use.map(x => x.d0))].sort((a, b) => a - b), dvs = [...new Set(use.map(x => x.dv))].sort((a, b) => a - b);
      const cw = 70, ch = 36, m = {l: 60, t: 10, b: 40, r: 10}, W = m.l + m.r + cw * d0s.length, H = m.t + m.b + ch * dvs.length; const g = svg(box, W, H);
      const rmax = Math.max(0.05, ...use.map(x => Math.abs(x.r)));
      for (const x of use) { const i = d0s.indexOf(x.d0), j = dvs.indexOf(x.dv); const X = m.l + cw * i, Y = m.t + ch * (dvs.length - 1 - j);
        const rc = el('rect', {x: X, y: Y, width: cw, height: ch}, g); rc.style.fill = x.r > 0 ? 'var(--pos)' : 'var(--neg)'; rc.style.fillOpacity = (0.1 + 0.8 * Math.min(1, Math.abs(x.r) / rmax)).toFixed(2);
        rc.style.stroke = x.within ? 'var(--line)' : 'var(--ink)'; rc.style.strokeWidth = x.within ? 1 : 3;
        el('text', {x: X + cw / 2, y: Y + ch / 2 + 4, 'text-anchor': 'middle'}, g, (x.r >= 0 ? '+' : '') + x.r.toFixed(3));
        tipOn(rc, `d0 ${x.d0} dv ${x.dv}\nr ${fmt(x.r)} tol ${fmt(x.tol)} ${x.within ? 'within' : 'outside'}\nclass ${x.cls}`); }
      d0s.forEach((v, i) => el('text', {x: m.l + cw * (i + 0.5), y: H - m.b + 14, 'text-anchor': 'middle'}, g, fmt(v)));
      dvs.forEach((v, j) => el('text', {x: m.l - 4, y: m.t + ch * (dvs.length - 1 - j + 0.5) + 4, 'text-anchor': 'end'}, g, fmt(v)));
      el('text', {x: m.l + cw * d0s.length / 2, y: H - 6, 'text-anchor': 'middle'}, g, 'd0 (m)'); el('text', {x: 14, y: m.t + 10, 'text-anchor': 'start'}, g, 'dv');
      const inSet = cells.filter(x => nk(x.set) === sel.set); stat.textContent = `set ${sel.set}: ${inSet.filter(x => x.within).length}/${inSet.length} car-cells within tolerance`; }
    refill(0); draw();
  }
  if (N.ncar_n1) { const c = h('div', {class: 'card'}, s); h('h2', {}, c, 'Conflict-set sizes (N1)');
    h('p', {class: 'small muted'}, c, 'Per source: the share of car-slots by |K_i| (cars in the conflict set), and of events by size. Log scale.');
    const g2 = h('div', {class: 'grid2'}, c);
    for (const src of N.ncar_n1) { const d = h('div', {}, g2); h('h3', {}, d, src.source); for (const [key, lab] of [['K_hist', '|K_i| per car-slot'], ['event_hist', 'event size']]) {
      const hist = src[key] || {}; const ks = Object.keys(hist).map(Number).sort((a, b) => a - b); if (!ks.length) continue; const tot = ks.reduce((a, k) => a + hist[k], 0);
      const W = 320, H = 130, m = {l: 40, r: 8, t: 8, b: 30}; const g = svg(d, W, H); const bw = (W - m.l - m.r) / ks.length;
      const lv = k => Math.log10(1 + 1e4 * hist[k] / tot) / 4; ks.forEach((k, i) => { const hgt = (H - m.t - m.b) * lv(k); const r = el('rect', {x: m.l + bw * i + 2, y: H - m.b - hgt, width: bw - 4, height: hgt}, g); r.style.fill = 'var(--c0)';
        el('text', {x: m.l + bw * (i + 0.5), y: H - m.b + 12, 'text-anchor': 'middle'}, g, k); tipOn(r, `${lab} ${k}: ${hist[k]} (${(100 * hist[k] / tot).toFixed(2)} %)`); });
      el('text', {x: m.l, y: H - 4}, g, lab); } } }
  if (N.ncar_n4) { const c = h('div', {class: 'card'}, s); h('h2', {}, c, 'N4: the far field, error against N');
    h('p', {class: 'small muted'}, c, 'Mean |U_A explicit - predicted| over the contexts, per N: the mean field (near field exact, the rest as density) and the composition with every realized gap. Dashed: the D2 tolerance floor 0.02.');
    const rows = N.ncar_n4; const ns = [...new Set(rows.map(r => r.n))].sort((a, b) => a - b);
    const ser = {mf: ns.map(n => { const xs = rows.filter(r => r.n === n); return xs.reduce((a, r) => a + Math.abs(r.u - r.mf), 0) / xs.length; }),
                 comp: ns.map(n => { const xs = rows.filter(r => r.n === n); return xs.reduce((a, r) => a + Math.abs(r.u - r.comp), 0) / xs.length; })};
    const W = 560, H = 260, m = {l: 50, r: 10, t: 10, b: 36}; const hi = Math.max(0.03, ...ser.mf, ...ser.comp) * 1.1;
    const x = scale(ns[0], ns[ns.length - 1], m.l, W - m.r), y = scale(0, hi, H - m.b, m.t); const g = svg(c, W, H); axes(g, x, y, [ns[0], ns[ns.length - 1]], [0, hi], 'N (cars)', 'mean |error|', W, H, m);
    const tl = el('line', {x1: m.l, x2: W - m.r, y1: y(0.02), y2: y(0.02)}, g); tl.style.stroke = 'var(--bad)'; tl.style.strokeDasharray = '4 3';
    [['mf', 'var(--c0)', 'mean field'], ['comp', 'var(--c1)', 'composition']].forEach(([k, col, lab], i) => { const pl = el('polyline', {points: ns.map((n, j) => `${x(n)},${y(ser[k][j])}`).join(' '), fill: 'none'}, g); pl.style.stroke = col; pl.style.strokeWidth = 2;
      ns.forEach((n, j) => { const d = el('circle', {cx: x(n), cy: y(ser[k][j]), r: 4}, g); d.style.fill = col; tipOn(d, `${lab}, N ${n}: ${fmt(ser[k][j])}`); });
      const t = el('text', {x: W - m.r - 100, y: m.t + 14 + 14 * i}, g, lab); t.style.fill = col; }); }
  if (!N.ncar_residuals && !N.ncar_n1 && !N.ncar_n4) h('p', {class: 'muted'}, s, 'No N-car results registered.');
}

// ---------------------------------------------------------------- rules: parallel coordinates and state machines
function rulesView(){
  const s = addView('rules', 'Rules');
  views.rules.render = () => {
    const metas = []; for (const k in A.scenarios) for (const m of A.scenarios[k].meta || []) metas.push(m);
    if (metas.length) { const c = h('div', {class: 'card'}, s); h('h2', {}, c, '(4) Rule parameters (cut-in, arena rules)');
      h('p', {class: 'small muted'}, c, 'One line per rule in the double oracle\'s population; width and opacity: its weight in the final equilibrium mixture. Highlighted: Autoware-like rules (A_aw_*, B_aw*) and the B rules fitted to Autoware runs (C3).');
      const row = h('div', {class: 'row'}, c); const ml = h('label', {}, row, 'run '); const ms = h('select', {}, ml); metas.forEach((m, i) => h('option', {value: i}, ms, m.label));
      const box = h('div', {class: 'grid2'}, c);
      function draw(){ box.innerHTML = ''; const m = metas[+ms.value];
        for (const side of ['A', 'B']) { const d = h('div', {}, box); h('h3', {}, d, side === 'A' ? 'A (the merging car)' : 'B (the lane\'s car)');
          const rules = m[side].map(n => ({name: n, p: m.params[n] || {}, w: m.weights[n] || 0, hl: /_aw/.test(n)}));
          if (side === 'B') for (const f of A.fitted || []) if (f.side === 'B') rules.push({name: f.name, p: f.params, w: 0, hl: true, fit: true});
          parcoords(d, rules); } }
      ms.onchange = draw; draw(); }
    if ((A.e3_knobs || []).length) { const c = h('div', {class: 'card'}, s); h('h2', {}, c, 'E3: Autoware\'s cruise knobs as strategies (cell 10, 0)');
      h('p', {class: 'small muted'}, c, 'Autoware\'s parameter space, not the arena rule\'s: B\'s utility at seed 0 against A_aw_perm / A_eq. A stays ahead in every variant.');
      const sc = h('div', {class: 'scroll'}, c); const t = h('table', {}, sc); const hr = h('tr', {}, t); ['variant', 'overrides', 'uB vs A_aw_perm', 'uB vs A_eq'].forEach(x => h('th', {}, hr, x));
      for (const k of A.e3_knobs) { const tr = h('tr', {}, t); h('td', {}, tr, k.variant); h('td', {}, tr, k.overrides); h('td', {}, tr, fmt(k.uB[0])); h('td', {}, tr, fmt(k.uB[1])); }
      const withP = A.e3_knobs.filter(k => k.params); if (withP.length) { h('h3', {}, c, 'The variants as Autoware B rules (parallel coordinates; last axis: uB vs A_aw_perm, missing where within +-0.01 of stock)');
        const ub = withP.map(k => k.uB[0]).filter(v => v !== null); const mid = withP.find(k => k.variant === 'stock').uB[0];
        parcoords(h('div', {class: 'scroll'}, c), withP.map(k => ({name: 'Autoware B ' + k.variant, p: {...k.params, uB: k.uB[0] === null ? mid : k.uB[0]}, w: k.variant === 'stock' ? 0.5 : 0,
          hl: ['stock', 'yield', 'hold'].includes(k.variant), fit: k.variant !== 'stock'})), 760); } }
    const c = h('div', {class: 'card'}, s); h('h2', {}, c, '(5) Rule families as state machines'); const g = h('div', {class: 'sm'}, c); for (const k in SM) drawSM(g, k);
  };
}

function parcoords(d, rules, W = 520){
  const dims = [...new Set(rules.flatMap(r => Object.keys(r.p)))].filter(k => rules.some(r => typeof r.p[k] === 'number'));
  const H = 280, m2 = {l: 30, r: 30, t: 24, b: 30}; const g = svg(d, W, H); const xs = scale(0, Math.max(1, dims.length - 1), m2.l, W - m2.r);
  const rng = {}; for (const k of dims) { const v = rules.map(r => r.p[k]).filter(v => typeof v === 'number'); rng[k] = [Math.min(...v), Math.max(...v)]; }
  dims.forEach((k, i) => { el('line', {x1: xs(i), x2: xs(i), y1: m2.t, y2: H - m2.b, class: 'ax'}, g); el('text', {x: xs(i), y: 14, 'text-anchor': 'middle'}, g, k);
  el('text', {x: xs(i), y: H - m2.b + 12, 'text-anchor': 'middle'}, g, fmt(rng[k][0], 2)); el('text', {x: xs(i), y: m2.t - 2 + 0, 'text-anchor': 'middle', dy: '-0.1em'}, g, ''); el('text', {x: xs(i) + 2, y: m2.t + 10, 'text-anchor': 'start'}, g, fmt(rng[k][1], 2)); });
  const yv = (k, v) => { const [lo, hi] = rng[k]; return H - m2.b - (hi === lo ? 0.5 : (v - lo) / (hi - lo)) * (H - m2.t - m2.b); };
  for (const r of rules.sort((a, b) => a.hl - b.hl || a.w - b.w)) { const pts = dims.map((k, i) => typeof r.p[k] === 'number' ? `${xs(i)},${yv(k, r.p[k])}` : null).filter(Boolean);
  const pl = el('polyline', {points: pts.join(' '), fill: 'none'}, g); pl.style.stroke = r.hl ? (r.fit ? 'var(--good)' : 'var(--warn)') : 'var(--c0)';
  pl.style.strokeWidth = (1 + 6 * r.w).toFixed(1); pl.style.opacity = r.hl ? 0.95 : (0.25 + 0.75 * r.w).toFixed(2); if (r.fit) pl.style.strokeDasharray = '6 3';
  tipOn(pl, `${r.name}${r.w ? '  weight ' + fmt(r.w) : ''}\n` + Object.entries(r.p).map(([k, v]) => `${k} ${fmt(v)}`).join('  ')); }
}

const SM = {
  cutin_A: {title: 'Cut-in A (C1): go / wait / go_if_yield', nodes: [['cruise', 0, 0], ['go', 1, -1], ['watch B', 1, 0], ['drop back', 1, 1], ['change lane', 2, -1], ['behind B', 2, 1]],
    edges: [[0, 1, 'go'], [0, 2, 'go_if_yield'], [0, 3, 'wait'], [2, 4, 'B brakes'], [2, 3, 'no yield in WATCH'], [1, 4, ''], [3, 5, 'slot behind B']]},
  cutin_B: {title: 'Cut-in B (C1): pre_yield / react', nodes: [['cruise', 0, 0], ['open gap', 1, -1], ['see A move', 1, 1], ['yield', 2, 0], ['hold', 2, 1], ['safety brake', 3, 0.5]],
    edges: [[0, 1, 'pre_yield'], [0, 2, 'react'], [2, 3, 'best reply'], [2, 4, 'best reply'], [4, 5, 'gap short'], [3, 5, '']]},
  arena_A: {title: 'Arena A (tau, a_rear, buffer, patience, use_yield, t_prep)', nodes: [['gap check vs B', 0, 0], ['decide', 1, -1], ['drop', 1, 1], ['slot test k', 2, 1], ['prepare t_prep', 2, -1], ['lane change', 3, 0]],
    edges: [[0, 1, 'gap >= buffer + v tau + kin'], [0, 2, 't > patience'], [2, 3, ''], [3, 3, 'short: next car'], [3, 1, 'fits'], [1, 4, ''], [4, 5, '']]},
  arena_B: {title: 'Arena B (pre_band, react_gap, b_y)', nodes: [['cruise', 0, 0], ['latched', 1, 0], ['brake b_y', 2, -1], ['follow A', 2, 1]],
    edges: [[0, 1, 'A within pre_band / react_gap'], [1, 2, 'gap < S0 + v'], [1, 3, 'else IDM'], [0, 3, 'A in lane']]},
  crossing_A: {title: 'Crossing A: go / t<tau>[y]p<patience>', nodes: [['approach', 0, 0], ['go', 1, -1], ['compare arrival', 1, 0], ['stop at line', 2, 0.5], ['cross', 2, -1]],
    edges: [[0, 1, 'go'], [0, 2, 'tau rule'], [2, 4, 'ahead by tau'], [2, 3, 'else'], [3, 4, 'B yields (y) or patience']]},
  crossing_B: {title: 'Crossing B: hold / react / yb<b>p<p>', nodes: [['approach', 0, 0], ['hold', 1, -1], ['react', 1, 0], ['yield band', 1, 1], ['brake', 2, 0.5], ['cross', 2, -1]],
    edges: [[0, 1, ''], [0, 2, ''], [0, 3, 'A within b s'], [2, 4, 'sees A commit'], [3, 4, ''], [1, 5, ''], [4, 5, 'A clear or patience']]},
  cutout_L: {title: 'Cut-out L: leave at t_l before the obstacle', nodes: [['follow O', 0, 0], ['lane change', 1, 0], ['clear', 2, 0]], edges: [[0, 1, 'TTC to O = t_l'], [1, 2, 't_lc']]},
  cutout_B: {title: 'Cut-out B: keep gap t_b, look past L or not', nodes: [['follow L at t_b', 0, 0], ['O revealed', 1, 0], ['brake for O', 2, 0]], edges: [[0, 1, 'L leaves (or seen early: look)'], [1, 2, 'reaction']]},
  merge_A: {title: 'Lane drop A: early / zipper, tau, y, d/f', nodes: [['own lane', 0, 0], ['look for gap', 1, 0], ['merge', 2, -1], ['stop at end', 2, 1]],
    edges: [[0, 1, 'early: at once / zipper: ZIP_T before end'], [1, 2, 'gap >= tau (y: or B yields)'], [1, 3, 'no gap: d stop / f force']]},
  merge_B: {title: 'Lane drop B: hold / react / courtesy c<d>', nodes: [['cruise', 0, 0], ['courtesy', 1, -1], ['react', 1, 1], ['follow A', 2, 0]],
    edges: [[0, 1, 'A\'s lane ends within COURTESY_T'], [0, 2, 'sees A move'], [1, 3, ''], [2, 3, '']]},
  follower: {title: 'Queue follower (C, D, ...): cruise capped by IDM', nodes: [['cruise v0', 0, 0], ['follow lead', 1, -1], ['follow A', 1, 1]], edges: [[0, 1, 'gap < s*'], [0, 2, 'A in lane, after reaction']]},
};
function drawSM(parent, key){ const sm = SM[key]; if (!sm) return; const d = h('div', {}, parent); h('h3', {}, d, sm.title);
  const bw = 74, bh = 24, H = 170; const cols = Math.max(...sm.nodes.map(n => n[1])) + 1; const W = Math.max(300, 24 + bw + (cols - 1) * (bw + 40)); const g = svg(d, W, H);
  const defs = el('defs', {}, g); const mk = el('marker', {id: 'ar-' + key, viewBox: '0 0 8 8', refX: 7, refY: 4, markerWidth: 6, markerHeight: 6, orient: 'auto'}, defs); const mp = el('path', {d: 'M0,0 L8,4 L0,8 z'}, mk); mp.style.fill = 'var(--muted)';
  const pos = sm.nodes.map(n => [12 + bw / 2 + (W - 24 - bw) * (cols === 1 ? 0.5 : n[1] / (cols - 1)), H / 2 + n[2] * 52]);
  for (const [a, b, lab] of sm.edges) { const [x1, y1] = pos[a], [x2, y2] = pos[b];
    if (a === b) { const p = el('path', {d: `M${x1 + bw / 2},${y1 - 4} c18,-18 18,18 0,8`, fill: 'none', 'marker-end': `url(#ar-${key})`}, g); p.style.stroke = 'var(--muted)'; el('text', {x: x1 + bw / 2 + 4, y: y1 - 12, 'font-size': 9}, g, lab); continue; }
    const dx = x2 - x1, dy = y2 - y1, L = Math.hypot(dx, dy) || 1; const sx = x1 + dx / L * (bw / 2), sy = y1 + dy / L * (bh / 2), ex = x2 - dx / L * (bw / 2 + 2), ey = y2 - dy / L * (bh / 2 + 2);
    const ln = el('line', {x1: sx, y1: sy, x2: ex, y2: ey, 'marker-end': `url(#ar-${key})`}, g); ln.style.stroke = 'var(--muted)';
    if (lab) { const t = el('text', {x: (sx + ex) / 2, y: (sy + ey) / 2 - 3, 'text-anchor': 'middle'}, g, lab); t.style.fontSize = '8.5px'; t.style.fill = 'var(--muted)'; } }
  sm.nodes.forEach((n, i) => { const [x, y] = pos[i]; const r = el('rect', {x: x - bw / 2, y: y - bh / 2, width: bw, height: bh, rx: 5}, g); r.style.fill = 'var(--panel)'; r.style.stroke = 'var(--accent)';
    const t = el('text', {x, y: y + 4, 'text-anchor': 'middle'}, g, n[0]); t.style.fontSize = '9.5px'; }); }

// ---------------------------------------------------------------- boot
for (const sname in A.scenarios) scenarioView(sname, A.scenarios[sname]);
rulesView();
const th = document.getElementById('theme'); const root = document.documentElement;
function setTheme(v){ if (v === 'auto') root.removeAttribute('data-theme'); else root.setAttribute('data-theme', v); try { localStorage.setItem('atlas-theme', v); } catch (e) {} }
th.onchange = () => setTheme(th.value);
let saved = null; try { saved = localStorage.getItem('atlas-theme'); } catch (e) {} if (saved) { th.value = saved; setTheme(saved); }
let tab = (location.hash || '').slice(1); if (!views[tab]) { try { tab = localStorage.getItem('atlas-tab'); } catch (e) {} }
show(views[tab] ? tab : 'progress');
window.addEventListener('hashchange', () => { const t = location.hash.slice(1); if (views[t]) show(t); });
"""

HTML = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Strategy atlas</title>
<style>{css}</style>
</head>
<body>
<header><h1>Strategy atlas</h1><span class="sub">roadmap 074: strategy profiles over the context spaces, per tier ({generated})</span>
<span class="sp"></span><label>theme <select id="theme"><option value="auto">auto</option><option value="light">light</option><option value="dark">dark</option></select></label></header>
<nav id="nav"></nav>
<main id="main"></main>
<div id="tip"></div>
<script type="application/json" id="atlas-data">{data}</script>
<script>{js}</script>
</body>
</html>
"""


def build(atlas_path, out):
    atlas = json.load(open(atlas_path))
    data = json.dumps(atlas, separators=(",", ":")).replace("</", "<\\/")
    html = HTML.format(css=CSS, js=JS, data=data, generated=atlas.get("generated", ""))
    if len(html.encode()) > MAX_BYTES:
        raise SystemExit(f"atlas.html would be {len(html.encode()) / 1e6:.1f} MB, over the 16 MB limit: fewer episodes")
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    open(out, "w").write(html)
    return len(html.encode())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--atlas", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    n = build(a.atlas, a.out)
    print(f"{a.out}: {n / 1e6:.2f} MB")


if __name__ == "__main__":
    main()
