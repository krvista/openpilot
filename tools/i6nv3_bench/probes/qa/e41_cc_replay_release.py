"""E41: replay recorded driver-release windows through the REAL hyundai CarController (phase_tests harness Sim): inputs from
the 100 Hz route npz (speed, driver torque, wheel angle, controlsd angle command 'apply', EPS pressed, blinkers, aEgo, gas).
Per release: logged vs replayed TX gain and TX angle (fidelity), and the replayed gain at the release + 0.1 / 0.3 / 0.6 s.
Usage (repo root): PYTHONPATH=$PWD:$PWD/opendbc_repo python3 e41_cc_replay_release.py <scratch> [variant]"""
import glob
import sys
import numpy as np

sys.path.insert(0, ".")
from phase_tests.harness import Sim  # noqa: E402
from opendbc.car.hyundai.values import CarControllerParams as CCP  # noqa: E402

S = sys.argv[1]; VARIANT = sys.argv[2] if len(sys.argv) > 2 else "now"
if VARIANT == "now":
  CCP.RELEASE_CATCH_FRAMES = 0
elif VARIANT.startswith("catch"):
  for tok in VARIANT.split("_")[1:]:
    k, v = tok.split("=")
    setattr(CCP, {"g": "RELEASE_CATCH_GAIN", "dps": "RELEASE_CATCH_APPROACH_DPS", "f": "RELEASE_CATCH_FRAMES", "r": "RELEASE_CATCH_RATE_UP"}[k],
            float(v) if k != "f" else int(v))


def releases(d):
  T = d['T']; n = len(T); v = d['v']; tq = np.abs(d['tq_s'])
  bl = d['lb'] | d['rb'] | (d['lcs'] > 0)
  c = lambda m: np.concatenate([[0], np.cumsum(m)]); idx = np.arange(n)
  busy = (c(bl)[np.minimum(idx + 300, n)] - c(bl)[np.maximum(idx - 300, 0)]) > 0
  act = d['lat'] & (v >= 30) & ~busy
  hi = tq >= 150
  out = []; i = 30
  while i < n - 220:
    if hi[i - 1] and not hi[i] and hi[i - 30:i].all():
      j = i + int(np.argmax(tq[i:i + 30] < 60)) if (tq[i:i + 30] < 60).any() else None
      if j is not None and j > 800 and act[j:j + 200].all() and not hi[j:j + 150].any():
        out.append(j)
      i += 150
    else:
      i += 1
  return out


def replay(d, j0, j1):
  sim = Sim()
  rows = []
  for k in range(j0, j1):
    sim.step(v=float(d['v'][k]) / 3.6, tq=float(d['tq_s'][k]), wheel=float(d['ang'][k]), cmd=float(d['apply'][k]),
             lat_active=bool(d['lat'][k]), pressed=bool(d['pressed'][k]), blinker=bool(d['lb'][k]), blinker_right=bool(d['rb'][k]),
             a_ego=float(d['a_ego'][k]), gas=bool(d['gas'][k]), lane_prob_min=float(d['lane_min'][k]),
             bs_l=bool(d['bsl'][k]), bs_r=bool(d['bsr'][k]))
    m = sim.lkas_alt()
    rows.append((m["ADAS_ACIAnglTqRedcGainVal"] if m else np.nan, m["ADAS_StrAnglReqVal"] if m else np.nan))
  return np.array(rows)


res = []
for f in sorted(glob.glob(f"{S}/r50/0*.npz")):
  d = dict(np.load(f, allow_pickle=True))
  for j in releases(d):
    j0, j1 = j - 800, j + 150          # 8 s warm-up so the controller state settles before the window of interest
    R = replay(d, j0, j1)
    lg = d['gain'][j0:j1]; lw = d['wire'][j0:j1]
    w = slice(600, 950)                # last 3.5 s: 2 s before .. 1.5 s after the release
    eg = np.nanmedian(np.abs(R[w, 0] - lg[w])); ew = np.nanmedian(np.abs(R[w, 1] - lw[w]))
    gi = 800
    tx = R[gi:gi + 61, 1]
    txrate = np.max(np.abs(np.diff(tx[::5]))) / 0.05 if len(tx) > 6 else 0.0
    wheel_rate_logged = np.max(np.abs(np.diff(d['ang'][j:j + 61:10]))) / 0.1
    res.append((f[-12:-4], d['T'][j], lg[gi], R[gi, 0], R[gi + 10, 0], R[gi + 30, 0], R[gi + 60, 0], eg, ew,
                abs(R[gi + 1, 1] - d['ang'][j]), txrate, abs(d['apply'][j + 60] - R[gi + 60, 1]), wheel_rate_logged, lg[gi + 10]))
  print(f[-12:-4], len(res), flush=True)
A = np.array([r[2:] for r in res], dtype=float)
print(f"[{VARIANT}] releases {len(A)} | fidelity (2 s before .. 1.5 s after): |gain replay - logged| median p50 {np.median(A[:, 5]):.3f} p90 {np.percentile(A[:, 5], 90):.3f}; "
      f"|TX angle replay - logged| p50 {np.median(A[:, 6]):.2f} p90 {np.percentile(A[:, 6], 90):.2f} deg")
print(f"   gain at release logged p50 {np.median(A[:, 0]):.2f} | replay: at release {np.median(A[:, 1]):.2f}, +0.1 s {np.median(A[:, 2]):.2f}, +0.3 s {np.median(A[:, 3]):.2f}, +0.6 s {np.median(A[:, 4]):.2f}")
print(f"   TX angle: |TX - wheel| at the release p50 {np.median(A[:, 7]):.2f} p90 {np.percentile(A[:, 7], 90):.2f} deg | TX rate in 0.6 s p50 {np.median(A[:, 8]):.1f} p90 {np.percentile(A[:, 8], 90):.1f} max {A[:, 8].max():.0f} deg/s | "
      f"|plan cmd - TX| at +0.6 s p50 {np.median(A[:, 9]):.2f} p90 {np.percentile(A[:, 9], 90):.2f} deg")
# empirical snap risk vs gain at +0.1 s from the LOGGED releases (wheel peak rate >= 20 deg/s in the 0.6 s after)
lg01 = A[:, 11]; snap = A[:, 10] >= 20
bins = [0, 0.1, 0.3, 0.5, 1.01]; risk = []
for lo, hi in zip(bins[:-1], bins[1:]):
  m = (lg01 >= lo) & (lg01 < hi); risk.append(np.mean(snap[m]) if m.sum() else np.nan)
print("   logged snap risk by gain at +0.1 s: " + "  ".join(f"{lo:.1f}-{hi:.1f}: {r_:.0%} (n {int(((lg01 >= lo) & (lg01 < hi)).sum())})" for lo, hi, r_ in zip(bins[:-1], bins[1:], risk)))
exp = np.array([risk[min(np.searchsorted(bins, g, side='right') - 1, len(risk) - 1)] for g in A[:, 2]])
print(f"   expected snaps with THIS variant's gain at +0.1 s: {np.nansum(exp):.1f} of {len(A)} (logged: {snap.sum()})")
low = A[:, 1] < 0.3
print(f"   releases with replayed gain < 0.3 at the release: {low.sum()} -> gain +0.1/+0.3/+0.6 s p50 {np.median(A[low, 2]):.2f}/{np.median(A[low, 3]):.2f}/{np.median(A[low, 4]):.2f}")
np.save(f"{S}/qa/e41_{VARIANT}.npy", A)
