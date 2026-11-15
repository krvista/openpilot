"""E135 (report §92): what the factory LFA does with this MDPS — for a drive recorded in the stock-LFA capture mode
(Phase 70; openpilot passive, the camera's own 0x110 reaches the MDPS). Input: the per-segment npz files of
e121_angle_frames_extract.py. Looks only at frames where the camera's request is REALLY steering: LKAS_ANGLE_ACTIVE = 2,
the MDPS reports angle mode active, and op sent nothing in the segment.
 1. what the camera sends: ACI gain (ADAS_ACIAnglTqRedcGainVal), bytes 7 / 8 / 13 and the low nibble of byte 9;
 2. request vs wheel: request - MDPS angle (does the factory keep the request next to the wheel, or far ahead of it like
    op's 5-9 deg in a corner entry?), request rate;
 3. how the wheel follows: wheel rate / request rate by request rate (E117-A) and the start delay from rest (E128: time
    from the request having moved 0.5 deg to the wheel having moved 1 deg);
 4. steady ratio: wheel / request in held curves.
Run on a normal op drive it reports that no factory steering was found (the camera only echoes the wheel there).
argv: npz [npz ...]"""
import sys
from collections import Counter
import numpy as np
F = sys.argv[1:]
D = [dict(np.load(f)) for f in F]
print(f"segments {len(D)}; op 0x110 frames per segment: {[int(d.get('n_tx', -1)) for d in D][:12]}{' ...' if len(D) > 12 else ''}")
tot = 0; rows = []; onsets = []; starts = []; pat = Counter()
for d in D:
  n = len(d['T']); act = (d['cam_act'] == 2) & (d['mdps_act'] == 2)
  gap = d['cam'] - d['mdps1']
  # steering = active and the request is not a mere echo of the wheel: over the surrounding second |request - MDPS angle| > 0.3 somewhere
  g1 = np.convolve((np.abs(gap) > 0.3).astype(float), np.ones(101), mode="same") > 0
  steer = act & g1 & (d['pressed'] < 0.5)
  tot += steer.sum()
  if steer.sum() < 100: continue
  for k in np.flatnonzero(steer)[::10]:
    pat[(int(d['cam_b7'][k]), int(d.get('cam_b8', np.zeros(n))[k]), int(d.get('cam_b13', np.zeros(n))[k]), int(d.get('cam_b9lo', np.zeros(n))[k]))] += 1
  u = d['cam']; w = d['mdps1']; v = d['v'] * 3.6
  sc = np.concatenate([[0], np.cumsum(steer)])
  t = np.arange(50, n - 15); ok = (sc[t + 16] - sc[t - 50]) == 66
  r1 = (u[t - 25] - u[t - 50]) / 0.25; r2 = (u[t] - u[t - 25]) / 0.25; rtx = (u[t] - u[t - 50]) / 0.5; rw = (w[t + 15] - w[t - 35]) / 0.5
  st = ok & (np.sign(r1) == np.sign(r2)) & (np.abs(r1) <= 2 * np.abs(r2) + 0.2) & (np.abs(r2) <= 2 * np.abs(r1) + 0.2) & (np.abs(rtx) >= 0.3)
  rows.append(np.c_[d['cam_gain'][steer], gap[steer], np.abs(np.gradient(u))[steer] * 100, v[steer], np.abs(u[steer]), w[steer] / np.where(np.abs(u[steer]) > 1.5, u[steer], np.nan)])
  onsets.append(np.c_[np.abs(rtx[st]), rw[st] * np.sign(rtx[st]) / np.abs(rtx[st]), v[t][st]][::5])
  k = 150
  while k < n - 260:                                             # start delay from rest (E128 on the camera's request)
    mv = u[k] - u[k - 20]
    if abs(mv) >= 0.5 and abs(u[k - 1] - u[k - 21]) < 0.5 and np.max(np.abs(u[k - 120:k - 20] - u[k - 20])) <= 0.3 and abs(w[k - 20] - w[k - 120]) < 0.3 and sc[k + 251] - sc[k - 120] == 371:
      s = np.sign(mv)
      if (u[k + 100] - u[k - 20]) * s >= 2.0:
        j = np.flatnonzero((w[k:k + 250] - w[k]) * s >= 1.0)
        starts.append((j[0] / 100.0 if len(j) else np.nan, (u[k + 100] - u[k - 20]) * s, v[k]))
      k += 150; continue
    k += 1
print(f"factory steering found: {tot / 100:.0f} s (camera ACTIVE = 2, MDPS angle mode active, request not an echo of the wheel, no driver press)")
if tot < 500:
  print("-> nothing to analyse: on a normal op drive the camera only echoes the MDPS angle (gain 0). Record a drive with 'stock LFA capture' on.")
  sys.exit(0)
X = np.concatenate(rows); O = np.concatenate([o for o in onsets if len(o)]) if any(len(o) for o in onsets) else np.zeros((0, 3))
q = lambda a, p: np.nanpercentile(a, p)
print(f"1. camera frame while steering: ACI gain p1/p50/p99 {q(X[:,0],1):.3f}/{q(X[:,0],50):.3f}/{q(X[:,0],99):.3f}; (byte7, byte8, byte13, byte9 low nibble) patterns {pat.most_common(6)}")
print(f"2. request - MDPS angle: p5/p50/p95 {q(X[:,1],5):+.2f}/{q(X[:,1],50):+.2f}/{q(X[:,1],95):+.2f} deg, |gap| p50/p90/p99 {q(np.abs(X[:,1]),50):.2f}/{q(np.abs(X[:,1]),90):.2f}/{q(np.abs(X[:,1]),99):.2f}; request rate p50/p90/p99 {q(X[:,2],50):.1f}/{q(X[:,2],90):.1f}/{q(X[:,2],99):.1f} deg/s")
for lo, hi in ((0, 40), (40, 70), (70, 200)):
  m = (X[:, 3] >= lo) & (X[:, 3] < hi)
  if m.sum() < 300: continue
  line = f"   [{lo}-{hi} km/h, {m.sum() / 100:.0f} s] |gap| p50/p90 {q(np.abs(X[m,1]),50):.2f}/{q(np.abs(X[m,1]),90):.2f}; steady wheel/request (|request| > 1.5 deg) p50 {q(X[m,5],50):.2f}"
  mo = (O[:, 2] >= lo) & (O[:, 2] < hi) if len(O) else np.zeros(0, bool)
  for a, b in ((0.3, 2.5), (2.5, 8.0), (8.0, 40.0)):
    z = mo & (O[:, 0] >= a) & (O[:, 0] < b) if len(O) else mo
    line += f" | transfer at {a:g}-{b:g} deg/s {np.median(O[z, 1]):.2f} (n {z.sum()})" if z.sum() >= 10 else f" | {a:g}-{b:g} deg/s: -"
  print(line)
S_ = np.array(starts)
if len(S_) >= 3:
  print(f"3. start from rest (request +0.5 deg -> wheel +1 deg): n {len(S_)}, p25/p50/p75 {q(S_[:,0],25):.2f}/{q(S_[:,0],50):.2f}/{q(S_[:,0],75):.2f} s (op on this car: 0.9 s), request moved {np.median(S_[:,1]):.1f} deg in the first second")
else:
  print(f"3. start from rest: only {len(S_)} clean onsets (need a few corner entries with LFA on)")
