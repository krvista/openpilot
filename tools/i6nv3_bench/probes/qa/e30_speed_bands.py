"""E30: lane keeping by speed band per build (plan lanes 20 Hz + route 100 Hz): op active, both inner lines > 0.7, no
blinker/LC +-3 s, hands-off (|tq| < 150, not pressed, +-1 s). Bands 30-60 / 60-90 / 90-120 / 120+ km/h:
offset RMS about a 20 s block mean (centring), mean offset (bias, + = right), heading RMS, 0.1-0.6 Hz share of offset power
on >= 10 s straight stretches (weave), command 20 Hz change std (1e-3/m), |apply - wheel| p50, MDPS gain p10, 55 lead flag
share, driver grab (|tq| >= 225 onset after >= 2 s quiet) per hour (all frames, hands allowed)."""
import sys
import numpy as np
sys.path.insert(0, sys.argv[0].rsplit('/', 1)[0])
from common import routes, bfit, interp_rows

S = sys.argv[1]
BANDS = [(30, 60), (60, 90), (90, 120), (120, 250)]
agg = {}
for r, P, d, T, f in routes(S):
  n = len(T); b = str(d['commit']); Tr = d['T']; v = f['v'] * 3.6
  LX = P['LX'].astype(float); LC = 0.5 * (P['L1'].astype(float) + P['L2'].astype(float))
  xs = np.tile(np.linspace(0, 20, 9), (n, 1)); cf = bfit(xs, interp_rows(xs, LX, LC), 2)
  e = -cf[:, 2]; head = cf[:, 1]; kr = 2 * cf[:, 0]
  good = f['lat'] & (P['PR'][:, 1] > 0.7) & (P['PR'][:, 2] > 0.7) & ~f['busy'] & ~f['hands']
  blk = (T // 20).astype(int); em = np.zeros(n)
  for k in np.unique(blk[good]):
    m = good & (blk == k); em[m] = e[m] - e[m].mean()
  kc = np.interp(T, Tr, d['k_ctrl']); ap = np.interp(T, Tr, d['apply']); an = np.interp(T, Tr, d['ang']); g = np.interp(T, Tr, d['gain'])
  sf = np.interp(T, Tr, d['sf_ctl'].astype(float)).astype(int)
  dk = np.concatenate([[0], np.diff(kc)]); cons = np.concatenate([[False], np.diff(T) < 0.07])
  # grabs at 100 Hz
  tq = np.abs(d['tq_s']); quiet = tq < 130; qc = np.concatenate([[0], np.cumsum(quiet)])
  strong = d['pressed'] | (tq >= 225); on = np.flatnonzero(strong[1:] & ~strong[:-1]) + 1
  on = [i for i in on if i >= 200 and qc[i] - qc[i - 200] >= 195 and d['lat'][i]]
  vr = d['v']
  for lo, hi in BANDS:
    a = agg.setdefault(b, {}).setdefault((lo, hi), dict(e=[], bias=[], head=[], dk=[], trk=[], g=[], lead=[], wv=[], grabs=0, lat_s=0.0))
    m = good & (v >= lo) & (v < hi)
    a['e'].append(em[m]); a['bias'].append(e[m]); a['head'].append(head[m]); a['dk'].append(dk[m & cons])
    a['trk'].append(np.abs(ap - an)[m]); a['g'].append(g[m]); a['lead'].append(((sf >> 9) & 1)[m])
    a['grabs'] += sum(1 for i in on if lo <= vr[i] < hi)
    a['lat_s'] += np.sum(d['lat'] & (vr >= lo) & (vr < hi)) * 0.01
    st = m & (np.abs(kr) < 0.5e-3)
    i = 0
    while i < n:
      if st[i]:
        j = i
        while j + 1 < n and st[j + 1] and T[j + 1] - T[j] < 0.1:
          j += 1
        if T[j] - T[i] >= 10:
          seg = e[i:j + 1] - e[i:j + 1].mean()
          fr = np.fft.rfftfreq(len(seg), 0.05); Pw = np.abs(np.fft.rfft(seg * np.hanning(len(seg)))) ** 2
          if Pw[1:].sum() > 0:
            a['wv'].append(Pw[(fr >= 0.1) & (fr <= 0.6)].sum() / Pw[1:].sum())
        i = j + 1
      else:
        i += 1
  print(r, b, flush=True)
for b, bands in agg.items():
  print(f"== build {b}")
  for (lo, hi), a in bands.items():
    E = np.concatenate(a['e'])
    if len(E) < 400:
      print(f"   {lo:3}-{hi:<3} km/h: hands-off good-lane {len(E) / 1200:.1f} min (too little)"); continue
    print(f"   {lo:3}-{hi:<3} km/h: {len(E) / 1200:5.1f} min | offset RMS {np.sqrt(np.mean(E ** 2)):.3f} m bias {np.mean(np.concatenate(a['bias'])):+.3f} | "
          f"heading RMS {np.sqrt(np.mean(np.concatenate(a['head']) ** 2)) * 1e3:.1f} mrad | weave {np.median(a['wv']) if a['wv'] else float('nan'):.2f} (n {len(a['wv'])}) | "
          f"cmd chg std {np.std(np.concatenate(a['dk'])) * 1e3:.3f}e-3 | |cmd-wheel| p50 {np.median(np.concatenate(a['trk'])):.2f} deg | gain p10 {np.percentile(np.concatenate(a['g']), 10):.2f} | "
          f"lead {np.mean(np.concatenate(a['lead'])):.0%} | grabs {a['grabs'] / max(a['lat_s'] / 3600, 1e-9):.0f}/h")
