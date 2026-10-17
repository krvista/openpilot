"""Shared loader for the driving-quality exploration: 20 Hz model frames (la/plan_*.npz) aligned with 100 Hz route
signals (ropm / r43 npz). Frame convention (checked): plan y, lane y and action curvature share one frame in which
curvature > 0 bends toward +y; laneLines[1] (left near) sits at -y, so +y = right of the camera."""
import glob

import numpy as np

T_IDXS = np.array([10.0 * (i / 32) ** 2 for i in range(33)])


def bfit(X, Y, deg):
  V = np.stack([X ** p for p in range(deg, -1, -1)], axis=-1)
  A = np.einsum('nmi,nmj->nij', V, V)
  b = np.einsum('nmi,nm->ni', V, Y)
  return np.linalg.solve(A + 1e-12 * np.eye(deg + 1), b[..., None])[..., 0]


def interp_rows(xq, X, Y):
  out = np.empty(xq.shape)
  for i in range(X.shape[0]):
    out[i] = np.interp(xq[i], X[i], Y[i])
  return out


def window_any(T, Tr, mask, w):
  """mask (on Tr) true anywhere within +-w s of each T."""
  c = np.concatenate([[0], np.cumsum(mask.astype(int))])
  lo = np.searchsorted(Tr, T - w); hi = np.searchsorted(Tr, T + w)
  return (c[hi] - c[lo]) > 0


def routes(S):
  for pf in sorted(glob.glob(f"{S}/la/plan_*.npz")):
    r = pf[-12:-4]
    P = dict(np.load(pf))
    rf = (glob.glob(f"{S}/ropm/{r}.npz") + glob.glob(f"{S}/r43/{r}.npz") + glob.glob(f"{S}/r46/{r}.npz") + glob.glob(f"{S}/r3d/{r}.npz")
          + glob.glob(f"{S}/rx/{r}.npz") + glob.glob(f"{S}/r50/{r}.npz"))
    if not rf or len(np.load(pf)["T"]) < 2000:
      continue
    d = dict(np.load(rf[0], allow_pickle=True))
    T = P['T']
    f = {k: np.interp(T, d['T'], d[k].astype(float)) for k in ('v', 'ang', 'apply', 'k_ctrl', 'k_meas', 'yaw', 'gain', 'tq_s')}
    f['v'] = f['v'] / 3.6
    f['lat'] = np.interp(T, d['T'], d['lat'].astype(float)) > 0.5
    f['busy'] = window_any(T, d['T'], d['lb'] | d['rb'] | (d['lcs'] > 0), 1.0)
    f['hands'] = window_any(T, d['T'], d['pressed'] | (np.abs(d['tq_s']) >= 150), 1.0)
    LX = P['LX'].astype(float)
    LC = 0.5 * (P['L1'].astype(float) + P['L2'].astype(float))
    xs0 = np.tile(np.linspace(0, 20, 9), (len(T), 1))
    f['k_road'] = 2.0 * bfit(xs0, interp_rows(xs0, LX, LC), 2)[:, 0]
    f['c0'] = np.array([np.interp(0.0, LX[i], LC[i]) for i in range(len(T))])     # lane centre y at the car
    f['width'] = np.array([np.interp(0.0, LX[i], P['L2'][i] - P['L1'][i]) for i in range(len(T))])
    f['good'] = (P['PR'][:, 1] > 0.7) & (P['PR'][:, 2] > 0.7) & ~f['busy'] & (f['v'] * 3.6 >= 30) & (f['width'] > 2.6) & (f['width'] < 4.2)
    yield r, P, d, T, f
