"""Low-speed shake v2: 1 s windows, lat active, not pressed, no blinker, plan QUIET (model curvature range < KQ over the
window) and |wheel| < 30 deg. Wheel and request (wire) are detrended with a quadratic fit; the residual is the shake.
Reversals = sign changes of the detrended wheel rate with |rate| > 3 deg/s. Command-driven = wire residual >= 0.6 x wheel
residual (op's own request oscillates); otherwise the wheel oscillates against a quiet request (MDPS/road)."""
import numpy as np, glob, sys
KQ = 1.0e-3
def load(f):
  d = dict(np.load(f, allow_pickle=True))
  if 'wire' not in d: d['wire'] = d['k_wire'] / float(d['ratio'])
  if 'lb' not in d: d['lb'] = d['blink']; d['rb'] = np.zeros_like(d['blink'])
  return d
def resid(x):
  t = np.arange(len(x)); p = np.polyfit(t, x, 2); return x - np.polyval(p, t)
def windows(d, vlo, vhi):
  n = len(d['T']); out = []
  for i in range(0, n - 100, 50):
    s = slice(i, i + 100)
    if not d['lat'][s].all() or d['pressed'][s].any() or d['lb'][s].any() or d['rb'][s].any(): continue
    v = d['v'][s].mean()
    if not (vlo <= v < vhi) or np.abs(d['ang'][s]).max() > 30: continue
    if np.ptp(d['k_model'][s]) > KQ: continue
    ra = resid(d['ang'][s]); rw = resid(d['wire'][s])
    rate = np.diff(ra) * 100; big = np.abs(rate) > 3
    rev = int(np.sum((np.sign(rate[1:]) != np.sign(rate[:-1])) & big[1:]))
    out.append((v, np.sqrt(np.mean(ra**2)), np.sqrt(np.mean(rw**2)), d['gain'][s].mean(), np.abs(d['tq_s'][s]).mean(), rev,
                d.get('a_ego', np.zeros(n))[s].mean()))
  return np.array(out)
def report(groups, bands):
  for vlo, vhi in bands:
    print(f"--- {vlo}-{vhi} km/h, quiet-plan hands-off windows (1 s, 50 % overlap)")
    for lab, fs in groups.items():
      Ws = [w for w in (windows(load(f), vlo, vhi) for f in fs) if len(w)]
      if not Ws: print(f"  {lab}: none"); continue
      W = np.concatenate(Ws); sh = W[:, 1] >= 0.25; cmd = W[:, 2] >= 0.6 * W[:, 1]
      print(f"  {lab:10s} n={len(W):5d} | wheel resid p50/p90 {np.median(W[:,1]):.2f}/{np.percentile(W[:,1],90):.2f} deg | reversals/s p50/p90 {np.median(W[:,5]):.0f}/{np.percentile(W[:,5],90):.0f} | shake(>=0.25) {sh.mean():.0%} (cmd-driven {cmd[sh].mean() if sh.any() else 0:.0%}) | wire resid p50 {np.median(W[:,2]):.2f} | gain p50 {np.median(W[:,3]):.2f}, in shake {np.median(W[sh,3]) if sh.any() else 0:.2f} | tq p50 {np.median(W[:,4]):.0f}")
if __name__ == "__main__":
  groups = {}
  for arg in sys.argv[1:]:
    lab, pat = arg.split('='); groups[lab] = sorted(glob.glob(pat))
  report(groups, ((3, 10), (10, 20), (20, 30), (30, 45)))
