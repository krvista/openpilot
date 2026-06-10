#!/usr/bin/env python3
"""Phase 6h pre-validation (ported 2026-06-10 offline session; the companion
extract/camref/la_smooth scripts from the handoff §6 were not attached to this
port). Set D (or PRECHECK_DIR env) to a directory of per-segment npz extracts:
ctl=[t, desiredCurvature, ., latActive, measuredCurvature], cs=[t, ., vEgo],
mdl=[t, fallbackCurv, probL, probR, ., ., yStd].

4.0-4: 6h-2' asymmetric tightening-rate limiter sweep.
  - measure natural corner-entry buildup rates (tightening dk/dt) on normal corners
  - candidate rate_up = K * MAX_LATERAL_JERK / v^2 with K in {0.4, 0.5, 0.6, 0.75, 1.0}
  - gates: (a) seg4 spike peak & buildup clipped meaningfully,
           (b) normal-corner 90%-of-peak arrival delay <= +10% frames

4.0-5: 6h-3 conf-fallback re-sim on model-frame rate.
  variants:
    6f5  : no floor,    fallback = prev blended state   (deployed on 0x40)
    6g1  : floor 0.5,   fallback = prev blended state   (deployed on 0x42)
    6h3  : floor 0.5 + 6g-2a taper, fallback = MEASURED curvature
  fixtures: 0x40 seg5 (freeze under-steer), 0x40 seg9 (lane reconnection),
            0x42 seg4 (over-steer spike)
"""
import numpy as np, os
D = os.environ.get('PRECHECK_DIR', '/home/claude/extracted')
J_MAX = 3.0 + 9.81*0.06   # 3.59 m/s^3, matches CarControllerParams

def load(n):
  z = np.load(os.path.join(D, n + '.npz')); return {k: z[k] for k in z.files}

# ---------- 4.0-4 ----------
def asym_limit(curv, v, dt, K):
  out = np.empty_like(curv); last = curv[0]
  for i in range(len(curv)):
    rate_up = K * J_MAX / max(v[i], 5.0)**2 * dt
    dk = curv[i] - last
    if abs(curv[i]) > abs(last):           # tightening only
      dk = float(np.clip(dk, -rate_up, rate_up))
    last = last + dk; out[i] = last
  return out

def corner_episodes(curv, v, t, min_peak=0.003):
  """contiguous |curv|>0.0015 stretches with peak>=min_peak, v>5."""
  m = (np.abs(curv) > 0.0015) & (v > 5.0)
  idx = np.where(m)[0]
  eps = []
  if len(idx) == 0: return eps
  for run in np.split(idx, np.where(np.diff(idx) > 5)[0]+1):
    if len(run) < 20: continue
    if np.abs(curv[run]).max() >= min_peak:
      eps.append(run)
  return eps

def t90(curv, run):
  c = np.abs(curv[run]); pk = c.max()
  i = np.argmax(c >= 0.9*pk)
  return i

def sweep_4_0_4():
  print('='*78)
  print("4.0-4  6h-2' asymmetric tightening-rate sweep  (rate_up = K*J/v^2)")
  SEGS = ['99b215d21bbf8735_00000040--eb2be2a919--4','99b215d21bbf8735_00000040--eb2be2a919--5',
          '99b215d21bbf8735_00000040--eb2be2a919--6','99b215d21bbf8735_00000040--eb2be2a919--9',
          '99b215d21bbf8735_00000042--8c1f634610--3','99b215d21bbf8735_00000042--8c1f634610--4',
          '99b215d21bbf8735_00000042--8c1f634610--5','99b215d21bbf8735_00000042--8c1f634610--12']
  # natural tightening rates on the deployed command
  rates = []
  for name in SEGS:
    o = load(name); ctl = o['ctl']; cs = o['cs']
    t = ctl[:,0]; dc = ctl[:,1]; act = ctl[:,3]
    v = np.interp(t, cs[:,0], cs[:,2])
    dt = np.median(np.diff(t))
    m = (act > 0.5) & (v > 5.0)
    dk = np.diff(dc); tight = (np.abs(dc[1:]) > np.abs(dc[:-1])) & m[1:]
    norm = dk[tight] * np.maximum(v[1:][tight],5.0)**2 / J_MAX / dt   # in units of K
    rates.append(np.abs(norm))
  r = np.concatenate(rates)
  print(f"  natural tightening |dk/dt| in K-units (n={len(r)}): "
        f"p50 {np.percentile(r,50):.2f}  p90 {np.percentile(r,90):.2f}  "
        f"p95 {np.percentile(r,95):.2f}  p99 {np.percentile(r,99):.2f}")

  Ks = [0.4, 0.5, 0.6, 0.75, 1.0]
  # (a) seg4 spike
  o = load('99b215d21bbf8735_00000042--8c1f634610--4')
  ctl = o['ctl']; cs = o['cs']; t = ctl[:,0]; dc = ctl[:,1]
  v = np.interp(t, cs[:,0], cs[:,2]); dt = np.median(np.diff(t))
  t0 = cs[0,0]; w = ((t-t0) >= 40) & ((t-t0) <= 54)
  print(f"  -- seg4 spike window: deployed peak {np.abs(dc[w]).max():.4f} 1/m")
  for K in Ks:
    out = asym_limit(dc[w], v[w], dt, K)
    # buildup time of the spike lobe (to 90% of its own peak)
    print(f"     K={K:.2f}: peak {np.abs(out).max():.4f}  "
          f"peak reduction {100*(1-np.abs(out).max()/np.abs(dc[w]).max()):4.1f}%")
  # (b) normal corner arrival delay
  print('  -- normal corner 90%-arrival delay (frames @100Hz), all segs, peak>=0.003 --')
  for K in Ks:
    delays = []
    for name in SEGS:
      o = load(name); ctl = o['ctl']; cs = o['cs']
      t = ctl[:,0]; dc = ctl[:,1]; act = ctl[:,3]
      v = np.interp(t, cs[:,0], cs[:,2]); dt = np.median(np.diff(t))
      for run in corner_episodes(dc, v, t):
        if not act[run].mean() > 0.5: continue
        base = t90(dc, run)
        out = asym_limit(dc[run], v[run], dt, K)
        cand = np.argmax(np.abs(out) >= 0.9*np.abs(out).max())
        delays.append(cand - base)
    d = np.array(delays)
    print(f"     K={K:.2f}: n={len(d)}  delay p50 {np.percentile(d,50):+.0f}  "
          f"p90 {np.percentile(d,90):+.0f}  p99 {np.percentile(d,99):+.0f} frames")

# ---------- 4.0-5 ----------
def conf_blend_sim(fallback, lane_min, ystd, meas, variant):
  out = np.empty_like(fallback); prev = fallback[0]
  for i in range(len(fallback)):
    conf_y = float(np.interp(ystd[i],   [0.05, 0.30], [1.0, 0.0]))
    conf_l = float(np.interp(lane_min[i],[0.05, 0.30], [0.0, 1.0]))
    base = min(conf_y, conf_l)
    if variant == '6f5':
      conf = base; ref = prev
    elif variant == '6g1':
      conf = max(base, 0.5); ref = prev
    else:  # 6h3
      taper = float(np.clip((lane_min[i]-0.20)/(0.30-0.20), 0.0, 1.0))
      conf = max(base, 0.5*taper); ref = meas[i]
    cur = conf*fallback[i] + (1.0-conf)*ref
    prev = cur; out[i] = cur
  return out

def sim_4_0_5():
  print(); print('='*78)
  print('4.0-5  6h-3 conf-fallback re-sim (model-frame 20 Hz)')
  FIX = [('99b215d21bbf8735_00000040--eb2be2a919--5', 33, 44, 'seg5 미추종(freeze)'),
         ('99b215d21bbf8735_00000040--eb2be2a919--9', 0, 60, 'seg9 차선 재연결'),
         ('99b215d21bbf8735_00000042--8c1f634610--4', 40, 54, 'seg4 과조향 스파이크')]
  for name, a, b, lbl in FIX:
    o = load(name); mdl = o['mdl']; cs = o['cs']; ctl = o['ctl']
    t0 = cs[0,0]; tm = mdl[:,0]-t0
    w = (tm >= a) & (tm <= b)
    fb = mdl[:,1][w]
    lane_min = np.minimum(mdl[:,2], mdl[:,3])[w]
    ystd = mdl[:,6][w]
    meas = np.interp(mdl[:,0][w], ctl[:,0], ctl[:,4])   # controlsState.curvature (실측)
    res = {k: conf_blend_sim(fb, lane_min, ystd, meas, k) for k in ['6f5','6g1','6h3']}
    print(f"  [{lbl}]  n={w.sum()}  lane_min p10 {np.percentile(lane_min,10):.2f}  "
          f"|fallback| max {np.abs(fb).max():.4f}")
    pk_fb = np.abs(fb).max()
    for k in ['6f5','6g1','6h3']:
      x = res[k]
      # pass-through of the corner command (how much of fb's peak survives)
      print(f"     {k}: out peak {np.abs(x).max():.4f} ({100*np.abs(x).max()/pk_fb:5.1f}% of plan)"
            f"   mean |out-fb| {np.mean(np.abs(x-fb)):.4f}")
    if 'seg5' in lbl:
      print('     (freeze 해소 = 6h3/6g1 peak이 6f5보다 plan에 근접해야 함)')
    if 'seg4' in lbl:
      print('     (스파이크 억제 = 6h3 peak이 6g1보다 낮아야 함; 실측 곡률로 끌림)')

if __name__ == '__main__':
  sweep_4_0_4(); sim_4_0_5()
