"""E14: lane-dropout holds (controlsState.steerFlags bit 5) on every Phase-46+ route. Per hold: held command vs the model
action at latch, the largest |action| in the 5 s before (a turn), the largest |held - action| during the hold and the
Phase 41 cap release after it (bit 1/2) — 'stale-turn holds' = held |k| > 2e-3 and the action unwinding toward straight.
Plus the simulated fix: hold min(|held|, |action now|) with the action's sign (0 if signs differ)."""
import glob
import sys
import numpy as np

S = sys.argv[1]
tot = {}
for f in sorted(glob.glob(f"{S}/r3d/0*.npz") + glob.glob(f"{S}/r43/0*.npz") + glob.glob(f"{S}/r46/0*.npz")):
  d = np.load(f, allow_pickle=True); r = f[-12:-4]; b = str(d['commit'])
  T = d['T']; sf = d['sf_ctl'].astype(int); K = d['k_model']; kc = d['k_ctrl']; v = d['v']
  drop = (sf >> 5) & 1
  st = tot.setdefault(b, dict(n=0, stale=0, mins=0.0, rows=[]))
  st['mins'] += np.sum(d['lat'] & (v >= 30)) * 0.01 / 60
  on = np.flatnonzero(np.diff(drop) == 1) + 1
  for i in on:
    j = i
    while j + 1 < len(drop) and drop[j + 1]:
      j += 1
    a0 = np.searchsorted(T, T[i] - 5.0)
    held = kc[i]
    kmax = np.max(np.abs(K[a0:i])) if i > a0 else 0
    dev = np.max(np.abs(kc[i:j + 1] - K[i:j + 1]))
    # time after release until |k_ctrl - K| < 0.5e-3
    e = j
    while e + 1 < len(K) and abs(kc[e + 1] - K[e + 1]) > 0.5e-3 and T[e + 1] - T[j] < 5:
      e += 1
    stale = abs(held) > 2e-3 and abs(K[i]) < abs(held) and np.sign(K[i]) == np.sign(held) or (abs(held) > 2e-3 and np.sign(K[i]) != np.sign(held))
    fix = np.sign(K[i:j + 1]) == np.sign(held)
    fixed = np.where(fix, np.sign(held) * np.minimum(abs(held), np.abs(K[i:j + 1])), 0.0)
    st['n'] += 1; st['stale'] += int(stale)
    st['rows'].append((r, T[i], T[j] - T[i], v[i], held * 1e3, K[i] * 1e3, kmax * 1e3, dev * 1e3, T[e] - T[j], stale,
                       np.max(np.abs(fixed - K[i:j + 1])) * 1e3))
for b, st in tot.items():
  print(f"== build {b}: op-active>=30 {st['mins']:.0f} min, dropout holds {st['n']} ({st['n'] / max(st['mins'], 1) * 60:.1f}/h), "
        f"stale-turn holds {st['stale']}")
  for rw in st['rows']:
    print(f"   {rw[0]} t {rw[1]:7.1f} {rw[2]:4.1f}s v {rw[3]:5.1f} held {rw[4]:+6.2f} action {rw[5]:+6.2f} |action| 5s-max {rw[6]:5.1f} "
          f"max|held-action| {rw[7]:5.2f} -> converge {rw[8]:4.1f}s after release {'STALE' if rw[9] else ''}  fix max dev {rw[10]:5.2f}")
