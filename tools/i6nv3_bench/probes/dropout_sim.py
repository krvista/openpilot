"""Offline replay of the Phase 39-2 dropout latch state machine on route npz (100 Hz) + lane npz (20 Hz, held).
Variants: base (current), +both (enter only if the stronger inner line is also < LANE_HI), +press (no entry / release while
the driver holds the wheel), +v50 (MIN_SPEED 40 -> 50 km/h); P49 = both + press + the entry debounce restarts on a press
(0.3 s hands-off before a hold) = Phase 49 as shipped (§40); P49+exit also releases once the stronger line is back. Validates base against the logged STEER_FLAG_LANE_DROPOUT runs."""
import glob, sys
import numpy as np
S = sys.argv[1]
ENTRY_LM, ENTRY_N, MIN_V, EXIT_N, MAX_N, REARM_N, LC_N, HI = 0.15, 30, 40 / 3.6, 50, 300, 100, 100, 0.30
def sim(lmin, lmax, v, busy, press, lat, both=False, use_press=False, min_v=MIN_V, press_resets=False, exit_max=False):
  out = np.zeros(len(v), bool); ld = False; armed = True; low = clr = n = rearm = hold = 0
  for i in range(len(v)):
    if not lat[i]:
      ld = False; low = clr = n = 0; hold = 0; continue
    hold = LC_N if busy[i] else max(hold - 1, 0)
    lowc = lmin[i] < ENTRY_LM and (not both or lmax[i] < HI)
    low = low + 1 if lowc and not (press_resets and press[i]) else 0
    if not armed:
      rearm = rearm + 1 if lmin[i] > HI else 0
      if rearm >= REARM_N: armed = True
    stop = hold > 0 or v[i] < min_v or (use_press and press[i])
    if ld and stop:
      ld = False; armed = False; rearm = 0
    if not ld and armed and not stop and low >= ENTRY_N:
      ld = True; clr = 0; n = 0
    elif ld:
      clr = clr + 1 if (lmax[i] if exit_max else lmin[i]) > HI else 0
      if clr >= EXIT_N: ld = False
    if ld:
      n += 1
      if n > MAX_N: ld = False; armed = False; rearm = 0
    out[i] = ld
  return out
def runs(x):
  e = np.diff(np.concatenate([[0], x.astype(int), [0]])); return np.where(e == 1)[0], np.where(e == -1)[0]
tot = {}; hours = 0
for f in sorted(glob.glob(f"{S}/0000*.npz")):
  r = f.split('/')[-1][6:8]; d = np.load(f, allow_pickle=True); L = np.load(f"{S}/lane_{f.split('/')[-1]}")['L']
  T = d['T']; idx = np.clip(np.searchsorted(L[:, 0], T, side='right') - 1, 0, len(L) - 1)
  pl, pr = L[idx, 2], L[idx, 3]           # inner left / right (cols p1, p2)
  lmin, lmax = np.minimum(pl, pr), np.maximum(pl, pr)
  v = d['v'] / 3.6; busy = d['lb'] | d['rb'] | (d['lcs'] > 0); press = d['pressed']; lat = d['lat']
  logged = (d['sf_ctl'] & (1 << 5)) > 0
  hours += lat.sum() / 360000
  res = {"logged": logged, "base": sim(lmin, lmax, v, busy, press, lat), "both": sim(lmin, lmax, v, busy, press, lat, both=True),
         "press": sim(lmin, lmax, v, busy, press, lat, use_press=True), "both+press": sim(lmin, lmax, v, busy, press, lat, True, True),
         "both+v50": sim(lmin, lmax, v, busy, press, lat, both=True, min_v=50 / 3.6),
         "P49": sim(lmin, lmax, v, busy, press, lat, both=True, use_press=True, press_resets=True),
         "P49+exit": sim(lmin, lmax, v, busy, press, lat, both=True, use_press=True, press_resets=True, exit_max=True)}
  line = [r]
  for k, x in res.items():
    s, e = runs(x); tot.setdefault(k, [0, 0.0]); tot[k][0] += len(s); tot[k][1] += x.sum() / 100
    line.append(f"{k} {len(s):2d}/{x.sum()/100:5.1f}s")
  # base vs logged agreement
  agree = (res['base'] == logged).mean()
  print("  ".join(line), f" base~logged {agree:.4f}")
print(f"lat-active hours {hours:.2f}")
for k, (n, s) in tot.items(): print(f"  {k:11} {n:3d} latches ({n/hours:4.1f}/h), {s:5.1f} s")
# remaining latches under both+v50 (the proposal): what were they
if "--list" in sys.argv:
  for f in sorted(glob.glob(f"{S}/0000*.npz")):
    r = f.split('/')[-1][6:8]; d = np.load(f, allow_pickle=True); L = np.load(f"{S}/lane_{f.split('/')[-1]}")['L']
    T = d['T']; idx = np.clip(np.searchsorted(L[:, 0], T, side='right') - 1, 0, len(L) - 1)
    pl, pr = L[idx, 2], L[idx, 3]; lmin, lmax = np.minimum(pl, pr), np.maximum(pl, pr)
    x = sim(lmin, lmax, d['v'] / 3.6, d['lb'] | d['rb'] | (d['lcs'] > 0), d['pressed'], d['lat'], both=True, min_v=50 / 3.6)
    for a, b in zip(*runs(x)):
      kp = d['k_model'][a:b]; pa = slice(b, min(b + 300, len(T)))
      print(f"  {r} t {T[a]:5.0f} dur {(b-a)/100:3.1f} v {d['v'][a]:3.0f} pL/pR {pl[a]:.2f}/{pr[a]:.2f} hold {d['k_ctrl'][max(a-30,0)]*1e3:6.2f} "
            f"plan mean/max {kp.mean()*1e3:6.2f}/{kp[np.argmax(abs(kp))]*1e3:6.2f}  press in 3 s after {d['pressed'][pa].mean():.0%}  max|tq| {abs(d['tq_s'][a:pa.stop]).max():4.0f}")
