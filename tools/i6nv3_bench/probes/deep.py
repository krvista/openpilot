"""Deep route analysis (10-02): every judgement the i6nv3 stack makes vs what the car did, on route_extract v2 npz.
Usage: python deep.py "<glob of npz>" [--top N]"""
import glob, sys
import numpy as np
sys.path.insert(0, sys.argv[0].rsplit('/', 1)[0])
from hunt import band

def L(f):
  d = dict(np.load(f, allow_pickle=True)); d['r'] = f.split('/')[-1][:-4][-2:]; return d

fs = sorted(glob.glob(sys.argv[1])); TOP = int(sys.argv[sys.argv.index('--top') + 1]) if '--top' in sys.argv else 15
D = [L(f) for f in fs]
D = [d for d in D if len(d['T']) > 6000]

print("=== 1. routes")
from collections import Counter
evc = Counter()
for d in D:
  ev = d['events']; names = [e[1] for e in ev] if len(ev) else []
  evc.update(names)
  print(f"  {d['r']}: {len(d['T'])/6000:5.1f} min, commit {d['commit']}, lat {d['lat'].mean():.0%}, v p50 {np.median(d['v'][d['v']>1]):.0f} km/h, "
        f"MDPS fault frames {int((d['mdps_fault']>0).sum())}, tx rejects {int(d['tx_reject'].sum())}, events {len(names)}")
print("  events (all routes):", ", ".join(f"{k} {v}" for k, v in evc.most_common(25)))

print("\n=== 2. MDPS response to op's activation (lat active, tx LKAS_ANGLE_ACTIVE == 2)")
for d in D:
  m = d['lat'] & (d['tx_active'] == 2)
  if m.sum() < 100: continue
  na = m & (d['mdps_active'] != 2)
  # episodes of not-active while requested
  e = np.diff(np.concatenate([[0], na.astype(int), [0]])); st = np.where(e == 1)[0]; en = np.where(e == -1)[0]; dur = (en - st) / 100
  print(f"  {d['r']}: requested {m.sum()/6000:5.1f} min | MDPS not active {na.mean() if False else na.sum()/max(m.sum(),1):.2%} ({len(st)} episodes, >0.5 s: {(dur>0.5).sum()}, longest {dur.max() if len(dur) else 0:.1f} s) | fault frames {int((m & (d['mdps_fault']>0)).sum())}")

print("\n=== 3. hands-off authority windows: 1.5-4 Hz shake and fast-descent (43a)")
B = [(5, 20), (20, 40), (40, 60), (60, 90), (90, 200)]
acc = {b: [] for b in B}
for d in D:
  g = d['gain']; dg = np.zeros(len(g)); dg[1:] = np.diff(g); n = len(g)
  for i in range(0, n - 200, 100):
    s = slice(i, i + 200)
    if not d['lat'][s].all() or d['pressed'][s].any() or (d['lb'][s] | d['rb'][s]).any() or g[s].min() < 0.15: continue
    v = d['v'][s].mean()
    for b in B:
      if b[0] <= v < b[1]: acc[b].append((band(d['ang'][s]) >= 0.3, np.sum(dg[s] <= -0.02) > 0, band(d['wire'][s]) >= 0.18))
for b in B:
  A = np.array(acc[b]) if acc[b] else np.zeros((0, 3), bool)
  if not len(A): continue
  h, fd, cmd = A[:, 0], A[:, 1], A[:, 2]
  print(f"  {b[0]:>3}-{b[1]:<3}: n={len(A):4d} shake {h.mean():5.1%} | fast-descent windows {fd.mean():5.1%} | shake with/without fd {h[fd].mean() if fd.any() else float('nan'):.0%}/{h[~fd].mean():.0%} | request-driven share of shake {cmd[h].mean() if h.any() else 0:.0%}")

print("\n=== 4. acceleration (43b): driver pedal + aEgo")
rows = {}
for d in D:
  v, a, gas, pr, lat, g = d['v'], d['a_ego'], d['gas'], d['pressed'], d['lat'], d['gain']
  on = np.zeros(len(v), bool); on[1:] = pr[1:] & ~pr[:-1]
  bl = d['lb'] | d['rb']
  for lab, m0 in (("cruise |a|<0.8", np.abs(a) < 0.8), ("pedal a 1.5-2.25", gas & (a >= 1.5) & (a < 2.25)), ("pedal a>=2.25", gas & (a >= 2.25)), ("no-pedal a>=1.5", ~gas & (a >= 1.5))):
    for vb in ((20, 45), (45, 80), (80, 200)):
      m = lat & m0 & (v >= vb[0]) & (v < vb[1])
      r = rows.setdefault((lab, vb), [0, 0, [], [], 0, 0])
      r[0] += m.sum(); r[1] += (on & m).sum()
      ho = m & ~pr & ~bl
      if ho.any():
        r[2].append(g[ho][::5]); r[3].append(np.abs(d['k_meas'] - d['k_ctrl'])[ho][::5])
      r[4] += (on & m & bl).sum(); r[5] += (on & m & (d['lead_sign'] if 'lead_sign' in d else np.zeros(len(v), bool))).sum()
print("  condition | speed | active min | grabs/min | grabs with blinker | hands-off gain p50 | |k_meas-k_ctrl| p50/p90 (1e-3)")
for (lab, vb), r in rows.items():
  if r[0] < 300: continue
  G = np.concatenate(r[2]) if r[2] else np.array([np.nan]); E = np.concatenate(r[3]) * 1e3 if r[3] else np.array([np.nan])
  print(f"  {lab:17s} | {vb[0]:>3}-{vb[1]:<3} | {r[0]/6000:5.1f} | {r[1]/(r[0]/6000):5.1f} | {r[4]/max(r[1],1):4.0%} | {np.nanmedian(G):.2f} | {np.nanmedian(E):.2f}/{np.nanpercentile(E,90):.2f}")

print("\n=== 5. city grip yield (42): press onsets 20-45 km/h, >=400 Nm, gain>=0.4 at onset -> time to gain<=0.20")
x = []
for d in D:
  v, pr, tq, g, lat = d['v'], d['pressed'], np.abs(d['tq_s']), d['gain'], d['lat']
  for i in np.where(pr[1:] & ~pr[:-1])[0] + 1:
    if 20 <= v[i] < 45 and lat[i] and g[i] >= 0.4 and (tq[i:i+30] >= 400).any() and i + 200 < len(v):
      j = np.where(g[i:i+200] <= 0.20)[0]; x.append(j[0]/100 if len(j) else 2.0)
x = np.array(x); print(f"  n={len(x)}, p50 {np.median(x):.2f} s, never within 2 s {np.mean(x>=2):.0%}" if len(x) else "  none")

print("\n=== 6. low-confidence yank (41): swings >= 4 deg / 0.5 s, then driver grab against the swing within 2 s")
hrs = low_min = fight = low_fight = assist_low = 0
for d in D:
  n = len(d['T']); ap = d['apply']; lat = d['lat']; v = d['v']; bl = d['lb'] | d['rb']; lcs = d['lcs']; pr = d['pressed']; tq = d['tq_s']; lm = d['lane_min']
  el = lat & ~bl & (lcs < 0.5) & (v >= 30); hrs += el.sum() / 360000; low_min += (el & (lm < 0.3)).sum() / 6000
  k = 50; da = np.zeros(n); da[k:] = ap[k:] - ap[:-k]; cr = np.abs(da) >= 4; last = -9999
  for i in np.where(cr[1:] & ~cr[:-1])[0] + 1:
    if i - last < 200 or i < 200 or i + 200 >= n or not el[i] or pr[i-k:i+1].any(): continue
    last = i; gr = np.where(pr[i:i+200] & (np.abs(tq[i:i+200]) >= 150))[0]
    if len(gr):
      opp = np.sign(tq[i+gr[0]]) == -np.sign(da[i]); low = lm[i-100:i].min() < 0.3
      fight += opp; low_fight += opp and low; assist_low += (not opp) and low
print(f"  eligible {hrs:.2f} h, low-conf {low_min:.1f} min | fight {fight} ({fight/max(hrs,1e-6):.1f}/h) | low-conf fight {low_fight} ({low_fight/max(low_min,0.01):.2f}/low-conf min) | low-conf assist {assist_low}")

print("\n=== 7. speed bands (lat active)")
print("  band | active min | grabs/min | fight share | resist min (hold>=300, request >=3 deg behind, gain>=0.2) | push min | hands-off gain p50 | |req-wheel| p50/p90")
for lo, hi in ((3, 10), (10, 20), (20, 30), (30, 45), (45, 60), (60, 80), (80, 100), (100, 200)):
  act = on_n = fi = res = pu = 0; G = []; E = []
  for d in D:
    v, lat, pr, tq, g, ang, wire = d['v'], d['lat'], d['pressed'], d['tq_s'], d['gain'], d['ang'], d['wire']
    m = lat & (v >= lo) & (v < hi); act += m.sum()
    on = np.zeros(len(v), bool); on[1:] = pr[1:] & ~pr[:-1]; idx = np.where(on & m)[0]; on_n += len(idx)
    lead = (wire - ang) * np.sign(tq); fi += int(np.sum(lead[idx] <= -1))
    hold = m & pr & (np.abs(tq) >= 300); res += (hold & (lead <= -3) & (g >= 0.2)).sum(); pu += (hold & (lead >= 3) & (g >= 0.2)).sum()
    ho = m & ~pr & ~(d['lb'] | d['rb'])
    if ho.any(): G.append(g[ho][::10]); E.append(np.abs(wire - ang)[ho][::10])
  if act < 600: continue
  G = np.concatenate(G); E = np.concatenate(E)
  print(f"  {lo:>3}-{hi:<3} | {act/6000:6.1f} | {on_n/(act/6000):5.1f} | {fi/max(on_n,1):4.0%} | {res/6000:4.2f} | {pu/6000:4.2f} | {np.median(G):.2f} | {np.median(E):.2f}/{np.percentile(E,90):.2f}")

print(f"\n=== 8. top {TOP} conflict moments (driver torque against op's pending request, >= 300 Nm, request >= 3 deg the other way, gain >= 0.2)")
C = []
for d in D:
  v, lat, pr, tq, g, ang, wire = d['v'], d['lat'], d['pressed'], d['tq_s'], d['gain'], d['ang'], d['wire']
  lead = (wire - ang) * np.sign(tq)
  bad = lat & (np.abs(tq) >= 300) & (lead <= -3) & (g >= 0.2)
  e = np.diff(np.concatenate([[0], bad.astype(int), [0]])); st = np.where(e == 1)[0]; en = np.where(e == -1)[0]
  for a_, b_ in zip(st, en):
    if b_ - a_ < 5: continue
    w = slice(a_, b_)
    C.append(dict(r=d['r'], t=d['T'][a_], dur=(b_-a_)/100, v=v[w].mean(), a=d['a_ego'][w].mean(), gas=d['gas'][w].mean() > 0.5,
                  blink='L' if d['lb'][w].any() else ('R' if d['rb'][w].any() else '-'), lm=d['lane_min'][max(a_-100,0):b_].min(),
                  score=float(np.sum(g[w] * np.abs(tq[w]))) / 100, gmax=g[w].max(), tqmax=np.abs(tq[w]).max(), lead=lead[w].min(),
                  ang0=ang[a_], ang1=ang[b_-1], req0=wire[a_], kmod=d['k_model'][w].mean()*1e3, pr=pr[w].mean(),
                  gps=(d['gps_lat'][a_], d['gps_lon'][a_]), dg=(g[max(a_-20,0)], g[a_], g[min(b_,len(g)-1)])))
C.sort(key=lambda c: -c['score'])
print(f"  total episodes {len(C)} ({sum(c['dur'] for c in C)/60:.2f} min)")
for c in C[:TOP]:
  print(f"  r{c['r']} t={c['t']:7.1f} dur {c['dur']:.2f}s v={c['v']:3.0f} a={c['a']:+.1f}{' gas' if c['gas'] else ''} blink {c['blink']} lm {c['lm']:.2f} | wheel {c['ang0']:+6.1f}->{c['ang1']:+6.1f} req {c['req0']:+6.1f} lead {c['lead']:+5.1f} | gain {c['dg'][0]:.2f}/{c['dg'][1]:.2f}/{c['dg'][2]:.2f} max {c['gmax']:.2f} | tq max {c['tqmax']:.0f} pressed {c['pr']:.0%} | k_model {c['kmod']:+.1f}e-3 | gps {c['gps'][0]:.5f},{c['gps'][1]:.5f}")
