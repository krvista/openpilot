"""Usage: <dir with route_extract npz + lane_<route>.npz>; see ROUTE8_REPORT §39."""
"""Grab onsets at >= 60 km/h (lat active): why. Uses route npz (100 Hz) + lane npz (20 Hz)."""
import glob, sys
import numpy as np
S = sys.argv[1]
rows = []; act = 0
for f in sorted(glob.glob(f"{S}/0000*.npz")):
  r = f.split('/')[-1][6:8]; d = np.load(f, allow_pickle=True); L = np.load(f.replace('/0000', '/lane_0000'))['L']
  T = d['T']; idx = np.clip(np.searchsorted(L[:, 0], T, side='right') - 1, 0, len(L) - 1)
  yl, yr = L[idx, 10], L[idx, 11]           # inner lines y at x=0 (left > 0? sign check below)
  center = (yl + yr) / 2                      # lane centre relative to car
  pr = d['pressed']; on = np.zeros(len(T), bool); on[1:] = pr[1:] & ~pr[:-1]
  m60 = d['lat'] & (d['v'] >= 60)
  act += m60.sum() / 6000
  last = -1e9
  for i in np.where(on & m60)[0]:
    if T[i] - last < 1.0: continue           # merge re-presses within 1 s
    last = T[i]
    w = slice(i, min(i + 100, len(T)))
    blk = (d['lb'][max(i - 200, 0):i + 300] | d['rb'][max(i - 200, 0):i + 300]).any() or (d['lcs'][max(i-200,0):i+300] > 0).any()
    tq = d['tq_s'][w]; tqm = tq[np.argmax(np.abs(tq))]
    req = d['apply'][max(i - 50, 0):i + 1]; dreq = req[-1] - req[0]       # op request change in the 0.5 s before
    wheel = d['ang'][i]; lead = d['apply'][i] - wheel                       # request - wheel (deg)
    kmod = d['k_model'][i]; curve = abs(d['k_meas'][max(i-100,0):i].mean()) > 1.5e-3
    rows.append(dict(r=r, t=T[i], v=d['v'][i], blk=blk, tq=tqm, dreq=dreq, lead=lead, curve=curve, center=center[i],
                     lm=d['lane_min'][i], dur=(pr[i:i+500].argmin() if not pr[i:i+500].all() else 500) / 100, kmod=kmod))
print(f"lat active >= 60 km/h: {act:.1f} min, grab onsets {len(rows)} ({len(rows)/act:.1f}/min)")
nb = [x for x in rows if not x['blk']]
print(f"  with blinker/LC within -2..+3 s: {len(rows)-len(nb)}   without: {len(nb)} ({len(nb)/act:.1f}/min)")
# against op: driver torque sign vs (request - wheel) sign: tq>0 pushes ang up; fight if op wants ang the other way (lead opposite tq)
fight = [x for x in nb if abs(x['lead']) >= 1.0 and np.sign(x['lead']) != np.sign(x['tq'])]
assist = [x for x in nb if abs(x['lead']) >= 1.0 and np.sign(x['lead']) == np.sign(x['tq'])]
print(f"  no-blinker: against op request (|req-wheel|>=1 deg) {len(fight)}, same direction {len(assist)}, op quiet {len(nb)-len(fight)-len(assist)}")
print(f"  no-blinker in curves {sum(x['curve'] for x in nb)}, straights {sum(not x['curve'] for x in nb)}")
c = np.array([x['center'] for x in nb]); tqs = np.array([x['tq'] for x in nb])
print(f"  lane-centre offset at grab (m, + = centre to the left of car): p25/p50/p75 {np.percentile(c,25):.2f}/{np.median(c):.2f}/{np.percentile(c,75):.2f}")
print(f"  driver torque direction at grab: +(ang up) {np.sum(tqs>0)}  -(ang down) {np.sum(tqs<0)}")
print(f"  grab duration p50 {np.median([x['dur'] for x in nb]):.2f} s, < 0.3 s {np.mean([x['dur']<0.3 for x in nb]):.0%}")
print(f"  lane_min at grab p50 {np.median([x['lm'] for x in nb]):.2f}")
