"""Hunting (1.5-4 Hz) in 2 s windows, all lat-active not-pressed no-blinker windows (plan need not be quiet: deliberate
steering lives below ~1 Hz). Power of wheel and request in the band; 'hunt' = wheel band RMS >= 0.3 deg."""
import numpy as np, glob, sys
from numpy.fft import rfft, irfft
sys.path.insert(0, sys.argv[0].rsplit('/', 1)[0]); from shake2 import load
def band(x, lo=1.5, hi=4.0, fs=100.0):
  x = x - np.polyval(np.polyfit(np.arange(len(x)), x, 1), np.arange(len(x)))
  X = rfft(x * np.hanning(len(x))); f = np.fft.rfftfreq(len(x), 1 / fs); X[(f < lo) | (f > hi)] = 0
  return np.sqrt(np.mean(irfft(X, len(x)) ** 2)) * 1.63   # hann RMS correction
def run(fs, bands):
  res = {b: [] for b in bands}
  for f in fs:
    d = load(f); n = len(d['T'])
    for i in range(0, n - 200, 100):
      s = slice(i, i + 200)
      if not d['lat'][s].all() or d['pressed'][s].any() or (d['lb'][s] | d['rb'][s]).any(): continue
      v = d['v'][s].mean()
      for b in bands:
        if b[0] <= v < b[1]:
          res[b].append((band(d['ang'][s]), band(d['wire'][s]), d['gain'][s].mean(), np.abs(d['tq_s'][s]).mean(), np.abs(d['ang'][s]).mean(), f[-6:-4]))
  return res
if __name__ == "__main__":
  groups = {}
  for arg in sys.argv[1:]:
    lab, pat = arg.split('='); groups[lab] = sorted(glob.glob(pat))
  bands = [(0.5, 10), (10, 20), (20, 30), (30, 45), (45, 60), (60, 80), (80, 200)]
  for lab, fs in groups.items():
    res = run(fs, bands)
    print(f"=== {lab}")
    for b in bands:
      R = res[b]
      if not R: continue
      W = np.array([r[:5] for r in R]); h = W[:, 0] >= 0.3; cmd = W[:, 1] >= 0.6 * W[:, 0]
      g_bins = [(0, .25), (.25, .5), (.5, .8), (.8, 1.01)]
      gb = " ".join(f"{lo:.2f}-{hi:.2f}:{np.mean(h[(W[:,2]>=lo)&(W[:,2]<hi)]) if ((W[:,2]>=lo)&(W[:,2]<hi)).sum()>20 else float('nan'):.0%}" for lo, hi in g_bins)
      print(f"  {b[0]:>4}-{b[1]:<3} n={len(W):4d} ({len(W)/30:.1f} min) | wheel 1.5-4 Hz RMS p50/p90 {np.median(W[:,0]):.2f}/{np.percentile(W[:,0],90):.2f} | hunt>=0.3 {h.mean():5.1%} (request-driven {cmd[h].mean() if h.any() else 0:.0%}) | hunt share by gain {gb} | tq in hunt {np.median(W[h,3]) if h.any() else 0:.0f} vs {np.median(W[:,3]):.0f}")
