"""E2: straight-road steering wobble at 100 Hz. Windows of 5.12 s, lat active, >= 50 km/h, |k_model| and |k_ctrl| < 0.6e-3
throughout, no blinker/LC/BSM. Welch PSD of command angle (apply), measured angle (ang), model action (k_model, in deg via
the window's apply/k ratio is not needed: reported as curvature) and the controlsd command (k_ctrl). Band RMS, and the
command->measured transfer gain/coherence per band, hands-off vs hands-on (driver torque)."""
import glob
import sys
import numpy as np

S = sys.argv[1]
N = 512
BANDS = [(0.1, 0.5), (0.5, 1.5), (1.5, 3.0), (3.0, 6.0), (6.0, 12.0)]
fr = np.fft.rfftfreq(N, 0.01)
win = np.hanning(N)
acc = {}
for f in sorted(glob.glob(f"{S}/ropm/0*.npz") + glob.glob(f"{S}/r3d/0*.npz") + glob.glob(f"{S}/r43/0*.npz") + glob.glob(f"{S}/r46/0*.npz") + glob.glob(f"{S}/rx/0*.npz")):
  d = np.load(f, allow_pickle=True)
  r = f[-12:-4]; build = str(d['commit'])
  T = d['T']
  ok = (d['lat'] & (d['v'] >= 50) & (np.abs(d['k_model']) < 0.6e-3) & (np.abs(d['k_ctrl']) < 0.6e-3) & ~d['lb'] & ~d['rb']
        & (d['lcs'] == 0) & ~d['bsl'] & ~d['bsr'])
  dt_ok = np.concatenate([[True], np.diff(T) < 0.03])
  ok &= dt_ok
  i = 0
  n = len(T)
  while i + N <= n:
    if not ok[i:i + N].all():
      bad = np.flatnonzero(~ok[i:i + N])[-1]
      i += bad + 1
      continue
    sl = slice(i, i + N)
    hands = bool((d['pressed'][sl] | (np.abs(d['tq_s'][sl]) > 60)).any())
    key = (build, hands)
    X = {k: np.fft.rfft((d[k][sl] - np.polyval(np.polyfit(np.arange(N), d[k][sl], 1), np.arange(N))) * win)
         for k in ('apply', 'ang', 'k_model', 'k_ctrl')}
    a = acc.setdefault(key, {'n': 0, 'Pa': 0, 'Pm': 0, 'Pk': 0, 'Pc': 0, 'Cam': 0, 'v': []})
    a['n'] += 1
    a['Pa'] = a['Pa'] + np.abs(X['apply']) ** 2; a['Pm'] = a['Pm'] + np.abs(X['ang']) ** 2
    a['Pk'] = a['Pk'] + np.abs(X['k_model']) ** 2; a['Pc'] = a['Pc'] + np.abs(X['k_ctrl']) ** 2
    a['Cam'] = a['Cam'] + X['ang'] * np.conj(X['apply'])
    a['v'].append(d['v'][sl].mean())
    i += N // 2
  print(r, build, flush=True)

scale = 2.0 / (np.sum(win ** 2) * 100.0)      # one-sided PSD scaling (units^2/Hz)
df = fr[1] - fr[0]
for key in sorted(acc):
  a = acc[key]
  print(f"== build {key[0]} hands {'on ' if key[1] else 'off'} windows {a['n']} ({a['n'] * 2.56 / 60:.1f} min) v {np.mean(a['v']):.0f} km/h")
  for lo, hi in BANDS:
    m = (fr >= lo) & (fr < hi)
    rms = lambda P: np.sqrt(np.sum(P[m]) * scale / a['n'] * df)
    coh = np.abs(np.sum(a['Cam'][m])) ** 2 / (np.sum(a['Pa'][m]) * np.sum(a['Pm'][m]))
    gain = np.abs(np.sum(a['Cam'][m])) / np.sum(a['Pa'][m])
    print(f"   {lo:4.1f}-{hi:<4.1f} Hz  cmd {rms(a['Pa']):.3f} deg  meas {rms(a['Pm']):.3f} deg  model {rms(a['Pk']) * 1e3:.4f}e-3  "
          f"ctrl {rms(a['Pc']) * 1e3:.4f}e-3  meas/cmd gain {gain:.2f} coh {coh:.2f}")
  pk = (fr > 1.0) & (fr < 10)
  print(f"   peak meas {fr[pk][np.argmax(a['Pm'][pk])]:.2f} Hz  peak cmd {fr[pk][np.argmax(a['Pa'][pk])]:.2f} Hz")
