"""Usage: <dir with route_extract npz + lane_<route>.npz>; see ROUTE8_REPORT §39."""
"""Lane-dropout (Phase 39 latch) episodes: what the lines did, why, and what the hold did vs the road/driver."""
import glob, sys
import numpy as np
S = sys.argv[1]
LD = 1 << 5
rows = []
for f in sorted(glob.glob(f"{S}/0000*.npz")):
  r = f.split('/')[-1][6:8]
  d = np.load(f, allow_pickle=True); L = np.load(f.replace('/0000', '/lane_0000'))['L']
  T = d['T']; ld = (d['sf_ctl'] & LD) > 0
  lt = L[:, 0]
  def lane(t0, t1, col):
    m = (lt >= t0) & (lt <= t1); return L[m, col] if m.any() else np.array([np.nan])
  e = np.diff(np.concatenate([[0], ld.astype(int), [0]])); st = np.where(e == 1)[0]; en = np.where(e == -1)[0]
  for a, b in zip(st, en):
    t0, t1 = T[a], T[b - 1]; dur = t1 - t0
    pre = slice(max(a - 300, 0), a)            # 3 s before
    post = slice(b, min(b + 300, len(T)))      # 3 s after
    pl, pr = lane(t0 - 0.5, t1, 1), lane(t0 - 0.5, t1, 2)
    w = lane(t0 - 1, t0, 11) - lane(t0 - 1, t0, 10)   # lane width at x=0 (y2 - y1)
    w_pre = lane(t0 - 6, t0 - 3, 11) - lane(t0 - 6, t0 - 3, 10)
    el, er = lane(t0, t1, 14), lane(t0, t1, 15)
    exp_, gain = lane(t0, t1, 21), lane(t0, t1, 22)
    k_hold = d['k_ctrl'][a:b].mean(); k_plan = d['k_model'][a:b]; k_meas = d['k_meas'][a:b]
    # why it ended
    if b < len(T) and d['v'][b] < 40: why = "slow"
    elif d['lb'][b - 5:b + 5].any() or d['rb'][b - 5:b + 5].any() or (d['lcs'][b - 5:b + 5] > 0).any(): why = "LC/blinker"
    elif dur > 2.95: why = "3s cap"
    elif not d['lat'][min(b, len(T) - 1)]: why = "lat off"
    else: why = "lines back"
    blk_pre = (d['lb'][max(a - 500, 0):a] | d['rb'][max(a - 500, 0):a]).any()
    lc_pre = (d['lcs'][max(a - 500, 0):a] > 0).any()
    press = d['pressed'][a:min(b + 200, len(T))].any()
    tq = np.abs(d['tq_s'][a:min(b + 200, len(T))]).max()
    rows.append(dict(r=r, t=t0, dur=dur, v=d['v'][a], pl=np.nanmin(pl), pr=np.nanmin(pr), pl_med=np.nanmedian(pl), pr_med=np.nanmedian(pr),
                     w=np.nanmedian(w), w_pre=np.nanmedian(w_pre), el=np.nanmedian(el), er=np.nanmedian(er), exp=np.nanmedian(exp_), gain=np.nanmedian(gain),
                     k_hold=k_hold, k_plan=np.mean(k_plan), k_plan_max=k_plan[np.argmax(np.abs(k_plan))] if len(k_plan) else 0, k_meas=np.mean(k_meas),
                     why=why, blk_pre=blk_pre, lc_pre=lc_pre, press=press, tq=tq, ang=d['ang'][a],
                     lat=d['gps_lat'][a], lon=d['gps_lon'][a]))
print(f"{'r':2} {'t':>6} {'dur':>4} {'v':>4} {'pL':>4} {'pR':>4} {'pLm':>4} {'pRm':>4} {'wid':>4} {'wpre':>4} {'edgL':>5} {'edgR':>5} {'exp':>4} {'gain':>5} "
      f"{'kHold':>6} {'kPlan':>6} {'kMeas':>6} {'why':10} blk lc  press  tq   ang   gps")
for x in rows:
  print(f"{x['r']} {x['t']:6.0f} {x['dur']:4.1f} {x['v']:4.0f} {x['pl']:4.2f} {x['pr']:4.2f} {x['pl_med']:4.2f} {x['pr_med']:4.2f} {x['w']:4.2f} {x['w_pre']:4.2f} "
        f"{x['el']:5.2f} {x['er']:5.2f} {x['exp']:4.0f} {x['gain']:5.2f} {x['k_hold']*1e3:6.2f} {x['k_plan']*1e3:6.2f} {x['k_meas']*1e3:6.2f} {x['why']:10} "
        f"{int(x['blk_pre'])}   {int(x['lc_pre'])}   {int(x['press'])}  {x['tq']:5.0f} {x['ang']:5.1f} {x['lat']:.5f},{x['lon']:.5f}")
np.save(f"{S}/dropouts.npy", rows, allow_pickle=True)
print("n", len(rows), "total s", sum(x['dur'] for x in rows))
