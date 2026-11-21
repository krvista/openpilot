"""E160 (report §96): across builds - while openpilot lateral was active, who actually commanded the MDPS?
Route npz v4 (cb_* = ADAS_CMD_35_10ms on bus 1, lfa_sym = HDA_LFA_SymSta). Per route and pooled per build, by speed band:
minutes of openpilot-lateral time and the share in
  relay   the ADAS controller passes openpilot's request to the MDPS (factory LFA symbol != 2, MDPS in angle mode)
  mixed   the factory LFA is active (symbol 2): the controller sends its own request
  none    the MDPS is not in angle mode (openpilot passive: low-speed / parking latches, standstill)
and, per route: how often the factory LFA was switched on / off (symbol 0 <-> not 0), how many of those came with a
press of the LFA button seen by the camera (LKAS_ALT bit 56 on bus 2) and how many without one; and (route npz with
tx_b7) the button-bit pulses openpilot itself transmitted with no camera press: how many, and what each did to the factory
LFA (off -> on, on -> off, nothing).
argv: S route [route ...]"""
import sys
from collections import defaultdict
import numpy as np
S = sys.argv[1]
BANDS = [(15, 30), (30, 55), (55, 70), (70, 90), (90, 200)]
pool = defaultdict(lambda: np.zeros((len(BANDS), 3)))
print("route    build   op-lat min | " + " | ".join(f"{lo}-{hi}: min relay/mixed/none".rjust(30) for lo, hi in BANDS) + " | LFA on/off toggles (with button / without)")
for r in sys.argv[2:]:
  try:
    d = dict(np.load(f"{S}/r50/{r}.npz", allow_pickle=True))
  except Exception:  # noqa: BLE001
    continue
  if 'cb_req' not in d:
    print(f"{r}: route npz older than v4"); continue
  b = str(d['commit'])[:7]; v = d['v']; lat = d['lat'].astype(bool); sym = d['lfa_sym'].astype(int); n = len(v)
  if lat.sum() < 3000:
    continue
  cmd = (d['cb_lvl2'].astype(int) == 2) & (d['mdps_active'].astype(int) == 2)
  cells = []
  for bi, (lo, hi) in enumerate(BANDS):
    m = lat & (v >= lo) & (v < hi)
    x = np.array([(m & cmd & (sym != 2)).sum(), (m & cmd & (sym == 2)).sum(), (m & ~cmd).sum()], float)
    pool[b][bi] += x
    cells.append(f"{m.sum() / 6000:5.1f} {x[0] / max(m.sum(), 1):4.0%}/{x[1] / max(m.sum(), 1):4.0%}/{x[2] / max(m.sum(), 1):4.0%}".rjust(30) if m.sum() >= 600 else "-".rjust(30))
  on = sym != 0; ok = (sym[1:] >= 0) & (sym[:-1] >= 0)
  tog = np.flatnonzero((on[1:] != on[:-1]) & ok) + 1
  bit = d['cam_b7'].astype(int) & 1; rise = np.flatnonzero((bit[1:] == 1) & (bit[:-1] == 0)) + 1
  wb = sum(1 for j in tog if np.any((rise >= j - 300) & (rise <= j + 20)))
  pul = ""
  if 'tx_b7' in d:
    tx = d['tx_b7'].astype(int) & 1; pr = np.flatnonzero((tx[1:] == 1) & (tx[:-1] == 0)) + 1
    pr = [i for i in pr if not bit[max(0, i - 5):i + 5].any()]
    eff = [("on" if (not on[max(0, i - 5)] and on[min(n - 1, i + 150)]) else "off" if (on[max(0, i - 5)] and not on[min(n - 1, i + 150)]) else "none") for i in pr]
    pul = f" | op pulses {len(pr)}: off->on {eff.count('on')}, on->off {eff.count('off')}, nothing {eff.count('none')}"
  print(f"{r} {b} {lat.sum() / 6000:7.1f}    | " + " | ".join(cells) + f" | {len(tog)} ({wb} / {len(tog) - wb})" + pul)
print("\npooled per build: op-lateral minutes and share relay / mixed / none")
for b, A in pool.items():
  print(f"  {b}: " + " | ".join(f"{lo}-{hi}: {A[i].sum() / 6000:5.1f} min {A[i][0] / max(A[i].sum(), 1):4.0%}/{A[i][1] / max(A[i].sum(), 1):4.0%}/{A[i][2] / max(A[i].sum(), 1):4.0%}"
                             for i, (lo, hi) in enumerate(BANDS)))
