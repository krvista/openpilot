"""E152 (report §95): WHEN does the command move at a corner entry / exit, per controlsd variant - the real controlsd
replayed over every stretch op steered (>= 30 km/h, no blinker / lane change within 3 s; the driver's hands do not matter
for the command, so all entries count, not only the hands-off ones). Reference A0 = the action re-timed to T = 0.375 s.
  entry: |A0| < 0.7e-3 held 1 s, then a peak >= 2e-3 within 3 s          exit: |A0| >= 2e-3 held 1 s, <= 0.7e-3 within 3 s
For each variant: the time its command reaches 30 % / 50 % / 70 % of the A0 peak (entry) or has fallen to 50 % of the held
level (exit), relative to the moment A0 itself does (ms; negative = the command moves before A0, i.e. it is taken from
further ahead in the plan; A0 is the plan ~0.375 s ahead, so a command at -425 ms is 0.8 s ahead of the road). 30 % is the
lowest level counted: the 'straight' before an entry may already hold up to 0.7e-3 and a lower level would be met before
the entry began. Median [25-75 %] by speed band; the difference between two variants is how much earlier / later the car
is asked to turn - printed PAIRED as well: for every other variant, the per-event difference to the first variant
(median, and in how many events it moved the command earlier). Variants as E149 (name=CONST:value;... with the
pseudo-constant TOLD:<s> = replay as if the model had been told the delay <s>: the logged action re-timed by modeld's
formula; TOLD:0.30 is the configuration the chain sim validated). argv: S build variants [workers=4]"""
import glob
import multiprocessing as mp
import os
import sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "tools/i6nv3_bench/probes/qa"); sys.path.insert(0, "tools/i6nv3_bench/probes"); sys.path.insert(0, "opendbc_repo")
os.environ.setdefault("REPLAY_MODEL_DELAY", "log")
S = sys.argv[1]; BUILD = sys.argv[2]
VARS = {}
for tok in sys.argv[3].split(","):
  nm, _, body = tok.partition("=")
  VARS[nm] = [kv.split(":") for kv in body.split(";") if kv]
NW = int(sys.argv[4]) if len(sys.argv) > 4 else 4
BANDS = [(30, 40), (40, 55), (55, 70), (70, 200)]
TI = np.array([10.0 * (i / 32) ** 2 for i in range(33)])


def parse(cur, v):
  if isinstance(cur, bool):
    return v in ("1", "True", "true")
  if isinstance(cur, (list, tuple)):
    return [float(x) for x in v.split("|")]
  return float(v)


def stretches():
  out = []
  for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
    d = np.load(f, allow_pickle=True)
    if str(d['commit'])[:7] != BUILD:
      continue
    r = f[-12:-4]; n = len(d['T']); v = d['v']
    bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
    cs = np.concatenate([[0], np.cumsum(bl)]); idx = np.arange(n)
    busy = (cs[np.minimum(idx + 300, n)] - cs[np.maximum(idx - 300, 0)]) > 0
    ok = d['lat'].astype(bool) & (v >= 30) & ~busy; ok[:1200] = False
    e = np.flatnonzero(np.diff(np.concatenate([[0], ok.astype(int), [0]])))
    for a, b in zip(e[::2], e[1::2]):
      k = a
      while b - k >= 1000:
        k1 = min(k + 6000, b)
        if b - k1 < 1000:
          k1 = b
        out.append((r, k, k1)); k = k1
  return out


def run(args):
  r, a, b = args
  import replay_state_control as R
  import openpilot.selfdrive.controls.controlsd as cd
  R.load(S, r); d = R.d; P = R.P
  T = d['T']; res = {}
  for nm, kvs in VARS.items():
    told = next((v for k, v in kvs if k == "TOLD"), None)          # TOLD:<s> = replay as if the model had been told <s>
    kvs = [(k, v) for k, v in kvs if k != "TOLD"]
    saved = {k: getattr(cd, k) for k, _ in kvs}
    for k, v in kvs:
      setattr(cd, k, parse(saved[k], v))
    env_told = os.environ.pop("REPLAY_FORCE_TOLD", None)
    if told not in (None, "log"):
      os.environ["REPLAY_FORCE_TOLD"] = told
    elif told is None and env_told is not None:
      os.environ["REPLAY_FORCE_TOLD"] = env_told
    out = R.replay("p57", T[a], T[b - 1])
    os.environ.pop("REPLAY_FORCE_TOLD", None)
    if env_told is not None:
      os.environ["REPLAY_FORCE_TOLD"] = env_told
    for k, v in saved.items():
      setattr(cd, k, v)
    res[nm] = np.interp(T[a:b], [o[0] for o in out], [o[1] for o in out])
  j = np.clip(np.searchsorted(P['T'], T[a:b], side="right") - 1, 0, len(P['T']) - 1)
  v = d['v'][a:b]; vv = np.maximum(v / 3.6, 1.0)
  if 'PSI' in P and R.LD is not None:
    tt = np.array([R.told_at(t) for t in P['T'][j]]) + 0.075
    psi_t = np.array([np.interp(t_, TI, P['PSI'][i_]) for t_, i_ in zip(tt, j)]); psi_0 = np.array([np.interp(0.375, TI, P['PSI'][i_]) for i_ in j])
    K = P['K'][j] + (2.0 / vv) * (psi_0 / 0.375 - psi_t / tt)
  else:
    K = d['k_model'][a:b].astype(float)
  m = len(K); aK = np.abs(K); ent = []; ext = []
  k = 100
  while k < m - 300:
    if aK[k] < 0.7e-3 and (aK[k - 100:k] < 0.7e-3).all():
      pk = k + int(np.argmax(aK[k:k + 300]))
      if aK[pk] >= 2e-3 and pk + 100 < m:
        s = np.sign(K[pk]); kp = aK[pk]
        bi = next((i for i, (lo, hi) in enumerate(BANDS) if lo <= np.median(v[k:pk + 1]) < hi), None)
        aX = [next((t for t in range(k, pk + 1) if K[t] * s >= x * kp), None) for x in (0.3, 0.5, 0.7)]
        if bi is not None and None not in aX and 0.3 * kp >= 0.6e-3:
          row = {}
          for nm in VARS:
            c = res[nm] * s; lo0 = max(k - 150, 0)
            cX = [next((t for t in range(lo0, min(pk + 200, m)) if c[t] >= x * kp), None) for x in (0.3, 0.5, 0.7)]
            row[nm] = tuple((cx - ax) * 10 if cx is not None else np.nan for cx, ax in zip(cX, aX)) + (float(c[pk] / kp), float(np.max(c[k:min(pk + 100, m)]) / kp))
          ent.append((bi, row))
      k = pk + 300; continue
    k += 10
  k = 100
  while k < m - 450:
    if aK[k] >= 2e-3 and (aK[k - 100:k] >= 2e-3).all() and np.all(np.sign(K[k - 100:k + 1]) == np.sign(K[k])):
      s = np.sign(K[k]); k0 = np.median(aK[k - 100:k]); seg = K[k:k + 300] * s; lo_i = np.flatnonzero(seg <= 0.7e-3)
      if len(lo_i) and seg[:lo_i[0]].max() <= 1.25 * k0:
        end = k + lo_i[0]
        m50 = next((t for t in range(k, end + 1) if K[t] * s <= 0.5 * k0), None)
        bi = next((i for i, (lo, hi) in enumerate(BANDS) if lo <= np.median(v[k:end + 1]) < hi), None)
        if m50 is not None and bi is not None:
          row = {}
          for nm in VARS:
            c = res[nm] * s
            c50 = next((t for t in range(max(k - 150, 0), min(end + 200, m)) if c[t] <= 0.5 * k0), None)
            row[nm] = ((c50 - m50) * 10 if c50 is not None else np.nan,)
          ext.append((bi, row))
        k = end + 200; continue
    k += 10
  return ent, ext


if __name__ == "__main__":
  ST = stretches()
  print(f"{BUILD}: {len(ST)} op stretches, {sum(b - a for _, a, b in ST) / 6000:.0f} min", flush=True)
  with mp.Pool(NW) as pool:
    out = pool.map(run, ST, chunksize=1)
  ent = [x for o in out for x in o[0]]; ext = [x for o in out for x in o[1]]
  q = lambda x, p: np.nanpercentile(x, p)
  for i, (lo, hi) in enumerate(BANDS):
    e = [r_ for b_, r_ in ent if b_ == i]; x = [r_ for b_, r_ in ext if b_ == i]
    print(f"  [{lo}-{hi}] entries {len(e)}, exits {len(x)}   (command vs A0, ms; negative = earlier)")
    for nm in VARS:
      line = f"     {nm:6s}"
      if len(e) >= 3:
        t30 = [r_[nm][0] for r_ in e]; t50 = [r_[nm][1] for r_ in e]; t70 = [r_[nm][2] for r_ in e]; pk = [r_[nm][3] for r_ in e]; mx = [r_[nm][4] for r_ in e]
        line += (f" entry 30 %: {np.nanmedian(t30):+5.0f} [{q(t30, 25):+.0f} {q(t30, 75):+.0f}]  50 %: {np.nanmedian(t50):+5.0f} [{q(t50, 25):+.0f} {q(t50, 75):+.0f}]  70 %: {np.nanmedian(t70):+5.0f}"
                 f"  command / A0 peak: at the A0 peak {np.nanmedian(pk):.2f}, its own maximum {np.nanmedian(mx):.2f}")
      if len(x) >= 3:
        t = [r_[nm][0] for r_ in x]
        line += f" | exit 50 %: {np.nanmedian(t):+5.0f} [{q(t, 25):+.0f} {q(t, 75):+.0f}]"
      print(line)
    ref = next(iter(VARS))
    for nm in VARS:
      if nm == ref:
        continue
      line = f"       {nm} - {ref}, paired:"
      if len(e) >= 3:
        for lvl, qi in (("30 %", 0), ("50 %", 1), ("70 %", 2)):
          dd = np.array([r_[nm][qi] - r_[ref][qi] for r_ in e]); dd = dd[np.isfinite(dd)]
          line += f" entry {lvl} {np.median(dd):+5.0f} ms (earlier in {int((dd < -5).sum())}, later in {int((dd > 5).sum())} of {len(dd)})"
      if len(x) >= 3:
        dd = np.array([r_[nm][0] - r_[ref][0] for r_ in x]); dd = dd[np.isfinite(dd)]
        line += f" | exit 50 % {np.median(dd):+5.0f} ms (earlier in {int((dd < -5).sum())}, later in {int((dd > 5).sum())} of {len(dd)})"
      print(line)
