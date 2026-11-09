"""E116 (report §86): how far the Phase 63a curve trim would wind up on the real 7a-7f drives. Open-loop replay of the
current CarController (e93.run: logged wheel / speed / torque / command in, TX out) — the logged wheel does not react to
the replayed trim, so the trim here is an UPPER bound. Per speed band: trim size when on, trim vs |command|, frames where
the trim exceeds the candidate bound max(1.5 deg, |cmd|) (= what '63c: cap tied to the command' would remove), and TX/cmd
ratio. Also the same with the candidate bound applied after the fact to the trim (TX' = TX - trim + bounded trim).
argv: S"""
import multiprocessing as mp
import sys
import numpy as np
sys.path.insert(0, "."); sys.path.insert(0, "tools/i6nv3_bench/probes/qa")
import e93_openloop_gate_bf36 as e93
e93.S = sys.argv[1]
BANDS = [(55, 70), (70, 90), (90, 200)]

if __name__ == "__main__":
  segs = e93.segments()
  with mp.Pool(4) as pool:
    res = pool.map(e93.run, [(f, a, b, "") for f, a, b in segs], chunksize=1)
  X = np.concatenate([r for _, r in res if len(r) > 300])
  cmd, tx, wh, v, act, tr, tq = X[:, 0], X[:, 1], X[:, 2], X[:, 3], X[:, 4] > 0.5, X[:, 8], X[:, 9]
  hands_off = np.abs(tq) < 60
  print(f"op frames {act.sum() / 6000:.0f} min")
  for lo, hi in BANDS:
    m = act & (v >= lo) & (v < hi)
    if m.sum() < 500:
      continue
    on = m & (np.abs(tr) > 0.05)
    bound = np.maximum(1.5, np.abs(cmd))
    over = on & (np.abs(tr) > bound)
    small = on & (np.abs(cmd) < 4.0)
    ratio = np.abs(tx[on]) / np.maximum(np.abs(cmd[on]), 0.5)
    print(f"[{lo}-{hi} km/h, {m.sum() / 6000:.1f} min] trim on {on.sum() / m.sum():5.1%} of op (small cmd < 4 deg: {small.sum() / max(on.sum(), 1):4.0%} of trim time)")
    if on.any():
      print(f"   |trim| when on: p50 {np.median(np.abs(tr[on])):.2f} p90 {np.percentile(np.abs(tr[on]), 90):.2f} p99 {np.percentile(np.abs(tr[on]), 99):.2f} max {np.abs(tr[on]).max():.2f} deg")
      print(f"   |TX|/|cmd| when on: p50 {np.median(ratio):.2f} p90 {np.percentile(ratio, 90):.2f} max {ratio.max():.1f}; TX >= 2 x cmd on {np.mean(ratio >= 2):4.1%} of trim time")
      print(f"   trim > max(1.5, |cmd|): {over.sum() / 6000:.2f} min = {over.sum() / max(on.sum(), 1):4.1%} of trim time, {over.sum() / m.sum():5.2%} of op; hands off in {np.mean(hands_off[over]) if over.any() else 0:4.0%} of it; "
            f"excess over the bound p50 {np.median(np.abs(tr[over]) - bound[over]) if over.any() else 0:.2f} max {(np.abs(tr[over]) - bound[over]).max() if over.any() else 0:.2f} deg")
      # worst episodes
      idx = np.flatnonzero(over)
      if len(idx):
        e = np.flatnonzero(np.diff(np.concatenate([[0], over.astype(int), [0]])))
        eps = sorted([(b0 - a0, a0, b0) for a0, b0 in zip(e[::2], e[1::2])], reverse=True)[:3]
        for n_, a0, b0 in eps:
          print(f"      episode {n_ / 100:4.1f} s: v {v[a0:b0].mean():.0f} km/h, |cmd| p50 {np.median(np.abs(cmd[a0:b0])):.1f}, |wheel| p50 {np.median(np.abs(wh[a0:b0])):.1f}, trim max {np.abs(tr[a0:b0]).max():.2f}, TX max {np.abs(tx[a0:b0]).max():.1f}, |tq| p50 {np.median(np.abs(tq[a0:b0])):.0f}")
