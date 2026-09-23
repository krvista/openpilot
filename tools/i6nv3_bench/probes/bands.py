"""Whole speed range report (standstill -> highway) + acceleration split, on route npz files (route_extract or corner)."""
import numpy as np, glob, sys
sys.path.insert(0, sys.argv[0].rsplit('/', 1)[0]); from shake2 import load
BANDS = [(0, 3), (3, 10), (10, 20), (20, 30), (30, 45), (45, 60), (60, 80), (80, 100), (100, 200)]
def per_band(fs, accel_split=False):
  rows = {}
  for f in fs:
    d = load(f); n = len(d['T']); v = d['v']; lat = d['lat']; pr = d['pressed']; tq = d['tq_s']; g = d['gain']
    ang = d['ang']; wire = d['wire']; bl = d['lb'] | d['rb']; a = d.get('a_ego', np.zeros(n))
    on = np.zeros(n, bool); on[1:] = pr[1:] & ~pr[:-1]
    lead = (wire - ang) * np.sign(tq)                        # request ahead of the wheel in the driver's torque direction
    for lo, hi in BANDS:
      for acc_lab, am in ((('all', np.ones(n, bool)),) if not accel_split else
                          (('cruise', np.abs(a) < 0.8), ('accel>1.5', a >= 1.5), ('brake<-1.5', a <= -1.5))):
        m = lat & (v >= lo) & (v < hi) & am
        r = rows.setdefault((lo, hi, acc_lab), dict(act=0, ho=0, onsets=0, fight=0, hold=0, resist=0, push=0, g_ho=[], err=[], tq_ho=[], rest_hi=0, rest=0))
        r['act'] += m.sum(); ho = m & ~pr & ~bl; r['ho'] += ho.sum()
        idx = np.where(on & m)[0]
        r['onsets'] += len(idx)
        # fight onset: at press onset the driver's torque opposes op's pending request (request - wheel >= 1 deg against tq)
        r['fight'] += int(np.sum(lead[idx] <= -1.0))
        hold = m & pr & (np.abs(tq) >= 300); r['hold'] += hold.sum()
        r['resist'] += (hold & (lead <= -3) & (g >= 0.2)).sum(); r['push'] += (hold & (lead >= 3) & (g >= 0.2)).sum()
        if ho.any():
          r['g_ho'].append(g[ho][::10]); r['err'].append(np.abs(wire - ang)[ho][::10]); r['tq_ho'].append(np.abs(tq)[ho][::10])
        rest = ho & (np.abs(tq) >= 150); r['rest'] += rest.sum(); r['rest_hi'] += (rest & (g >= 0.9) & (np.abs(wire - ang) >= 2)).sum()
  return rows
def show(rows, title):
  print(f"=== {title}")
  print("band km/h | accel | active min | grabs/min | fight share of grabs | hold>=300 min | resist min | push min | hands-off gain p50 | |req-wheel| p50/p90 | resting-hand pulled (gain>=0.9, >=2 deg) share")
  for (lo, hi, al), r in rows.items():
    if r['act'] < 600: continue
    am = r['act'] / 6000
    G = np.concatenate(r['g_ho']) if r['g_ho'] else np.array([np.nan]); E = np.concatenate(r['err']) if r['err'] else np.array([np.nan])
    print(f"  {lo:3d}-{hi:<3d} | {al:10s} | {am:6.1f} | {r['onsets']/am:5.1f} | {r['fight']/max(r['onsets'],1):4.0%} | {r['hold']/6000:5.2f} | {r['resist']/6000:4.2f} | {r['push']/6000:4.2f} | {np.nanmedian(G):.2f} | {np.nanmedian(E):.2f}/{np.nanpercentile(E,90):.2f} | {r['rest_hi']/max(r['rest'],1):4.0%}")
if __name__ == "__main__":
  groups = {}
  for arg in sys.argv[1:]:
    if '=' in arg:
      lab, pat = arg.split('='); groups[lab] = sorted(glob.glob(pat))
  for lab, fs in groups.items():
    show(per_band(fs), f"{lab}: speed bands")
  if '--accel' in sys.argv:
    for lab, fs in groups.items():
      show(per_band(fs, accel_split=True), f"{lab}: speed x acceleration")
