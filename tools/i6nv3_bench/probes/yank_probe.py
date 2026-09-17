"""Lane-merge / lane-loss "yank" probe.

Question (09-17): when lane lines merge or vanish, does op swing the wheel to correct a plan jump while its lane
confidence is low, so that the driver has to grab? Two outputs per route:
  1. EVENTS: op command swings (|d apply| >= SWING_DEG within SWING_S, lat active, no blinker, no lane change, driver
     not pressing at onset) — each with the lane confidence / lane width / plan shift of the preceding second and
     whether the driver grabbed (pressed with |tq| >= GRAB_NM) within GRAB_S after the swing.
  2. EXPOSURE: active frames binned by lane_min (min of inner-left/right probs) — swing rate and grab-after-swing
     share per bin. If low-confidence bins do not carry more swings/grabs, a confidence gate would not help.
Usage: PYTHONPATH=... python probes/yank_probe.py <route> [--npz out.npz] [--list]
"""
import glob
import sys

import numpy as np

from openpilot.tools.lib.logreader import LogReader

SWING_DEG, SWING_S, GRAB_NM, GRAB_S, MIN_KPH = 4.0, 0.5, 150.0, 2.0, 30.0
r = sys.argv[1]
npz = sys.argv[sys.argv.index("--npz") + 1] if "--npz" in sys.argv else None
files = sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
cs = []      # t, v_kph, angle, pressed, tq, blinker
cc = []      # t, lat, apply_deg
md = []      # t, lane_min, lprob, rprob, width0, plan_y15, plan_y30, ystd30, lcs, action_k
t0 = None
for f in files:
  for m in LogReader(f):
    w = m.which()
    t = m.logMonoTime * 1e-9
    if t0 is None:
      t0 = t
    if w == "carState":
      s = m.carState
      cs.append((t, s.vEgo * 3.6, s.steeringAngleDeg, s.steeringPressed, s.steeringTorque, s.leftBlinker or s.rightBlinker))
    elif w == "carControl":
      c = m.carControl
      cc.append((t, c.latActive, c.actuators.steeringAngleDeg))
    elif w == "modelV2":
      d = m.modelV2
      if len(d.laneLineProbs) >= 4 and len(d.laneLines) >= 4 and len(d.position.y) > 12:
        lp, rp = d.laneLineProbs[1], d.laneLineProbs[2]
        width = d.laneLines[2].y[0] - d.laneLines[1].y[0]
        y15 = float(np.interp(15.0, d.position.x, d.position.y))
        y30 = float(np.interp(30.0, d.position.x, d.position.y))
        ystd30 = float(np.interp(30.0, d.position.x, d.position.yStd)) if len(d.position.yStd) == len(d.position.x) else 0.0
        md.append((t, min(lp, rp), lp, rp, width, y15, y30, ystd30, int(d.meta.laneChangeState.raw), d.action.desiredCurvature))
CS = np.array(cs, float); CC = np.array(cc, float); MD = np.array(md, float)
if not len(CS) or not len(CC) or not len(MD):
  print(f"== {r}: no data"); sys.exit(0)
T = CS[:, 0]
lat = np.interp(T, CC[:, 0], CC[:, 1]) > 0.5
apply = np.interp(T, CC[:, 0], CC[:, 2])
lane_min = np.interp(T, MD[:, 0], MD[:, 1]); lprob = np.interp(T, MD[:, 0], MD[:, 2]); rprob = np.interp(T, MD[:, 0], MD[:, 3])
width = np.interp(T, MD[:, 0], MD[:, 4])
y15 = np.interp(T, MD[:, 0], MD[:, 5])
ystd = np.interp(T, MD[:, 0], MD[:, 7])
lcs = np.interp(T, MD[:, 0], MD[:, 8])
v = CS[:, 1]; ang = CS[:, 2]; pressed = CS[:, 3] > 0.5; tq_s = CS[:, 4]; tq = np.abs(tq_s); blink = CS[:, 5] > 0.5
k_model = np.interp(T, MD[:, 0], MD[:, 9])
n = len(T); k = int(SWING_S * 100); g = int(GRAB_S * 100)
dapply = np.zeros(n); dapply[k:] = apply[k:] - apply[:-k]
elig = lat & ~blink & (lcs < 0.5) & (v >= MIN_KPH)
# swing onsets: |dapply| crosses SWING_DEG, driver not pressing in the SWING_S window, eligible
cross = (np.abs(dapply) >= SWING_DEG)
onset = np.where(cross[1:] & ~cross[:-1])[0] + 1
events = []
last = -10_000
for i in onset:
  if i - last < 200 or i < 200 or i + g >= n:
    continue
  if not elig[i] or pressed[i - k:i + 1].any():
    continue
  last = i
  pre = slice(i - 100, i)          # the second before the swing completed
  grab = np.where(pressed[i:i + g] & (tq[i:i + g] >= GRAB_NM))[0]
  grab_t = grab[0] / 100 if len(grab) else None
  sgn = np.sign(dapply[i])
  # fight: the driver's first grab torque opposes the swing direction (yank corrected); assist: same direction (op too slow)
  fight = None if grab_t is None else bool(np.sign(tq_s[i + grab[0]]) == -sgn)
  wheel_follow = bool(np.sign(ang[i] - ang[i - k]) == sgn and abs(ang[i] - ang[i - k]) >= 2.0)
  # transient: apply reverses within 2 s (relative to the pre-swing level) without a grab
  transient = bool(grab_t is None and (apply[i + 200] - apply[i - k]) * sgn < 0.5 * abs(dapply[i]))
  dk_model = -(k_model[i] - k_model[i - k]) * sgn      # curvature sign is opposite to the angle sign on this car; >0 = plan moved with the swing
  # which inner line is weaker over the prior second, and does the swing head toward it? (+angle = left = line 1)
  weak_left = lprob[pre].min() < rprob[pre].min()
  toward_weak = bool((sgn > 0) == weak_left)
  # 39-2 latch eligibility in the preceding 0.5 s (lane_min < 0.15 for 0.3 s, v >= 40)
  low = lane_min[i - 50:i] < 0.15
  latch_ok = v[i] >= 40 and low.sum() >= 30
  events.append(dict(t=T[i] - t0, v=v[i], dapply=dapply[i], dwheel=ang[i] - ang[i - k], lane_min_pre=lane_min[pre].min(),
                     lane_min_at=lane_min[i], dwidth=width[i] - width[i - 200], dy15=y15[i] - y15[i - 100], ystd=ystd[i],
                     grab_t=grab_t, grab_tq=tq[i:i + g].max(), latch_ok=latch_ok, seg=int((T[i] - t0) // 60),
                     fight=fight, wheel_follow=wheel_follow, transient=transient, dk_model=dk_model,
                     lprob_pre=lprob[pre].min(), rprob_pre=rprob[pre].min(), toward_weak=toward_weak))
# torque/angle sign convention check on hands-on frames with op inactive
_m = (~lat) & pressed & (v >= 10)
if _m.sum() > 500:
  _dang = np.zeros(n); _dang[10:] = ang[10:] - ang[:-10]
  _mm = _m & (np.abs(_dang) > 1.0) & (tq >= 100)
  print(f"  sign check (op off, hands on): sign(tq)==sign(d angle) in {np.mean(np.sign(tq_s[_mm]) == np.sign(_dang[_mm])) * 100:.0f}% of {_mm.sum()} frames")
# exposure by lane_min bin
bins = [0.0, 0.3, 0.5, 0.8, 1.01]
print(f"== {r}: {n / 6000:.1f} min, eligible {elig.mean() * 100:.0f}% | swings {len(events)}, with grab<={GRAB_S:.0f}s "
      f"{sum(e['grab_t'] is not None for e in events)}")
print("  lane_min bin | eligible min | swings/min | grab share | 39-2 latch-eligible swings")
for lo, hi in zip(bins[:-1], bins[1:]):
  mfr = elig & (lane_min >= lo) & (lane_min < hi)
  ev = [e for e in events if lo <= e["lane_min_pre"] < hi]
  mins = mfr.sum() / 6000
  gr = sum(e["grab_t"] is not None for e in ev)
  print(f"  [{lo:.1f},{hi:.1f}) | {mins:6.1f} | {len(ev) / mins if mins > 0.05 else 0:5.2f} | "
        f"{gr}/{len(ev)} | {sum(e['latch_ok'] for e in ev)}")
if "--list" in sys.argv:
  for e in sorted(events, key=lambda e: e["t"]):
    grab_s = "-" if e["grab_t"] is None else f"{e['grab_t']:.1f}s/{e['grab_tq']:.0f}Nm"
    print(f"  seg{e['seg']:2d} t={e['t']:7.1f} v={e['v']:3.0f} dapply={e['dapply']:+5.1f} dwheel={e['dwheel']:+5.1f} "
          f"lane_min pre/at {e['lane_min_pre']:.2f}/{e['lane_min_at']:.2f} dwidth2s={e['dwidth']:+.2f} dy15={e['dy15']:+.2f} "
          f"ystd30={e['ystd']:.2f} grab={grab_s} fight={e['fight']} follow={int(e['wheel_follow'])} trans={int(e['transient'])} dk={e['dk_model'] * 1e3:+.1f}e-3")
if npz:
  np.savez_compressed(npz, T=T - t0, v=v, ang=ang, pressed=pressed, tq=tq, blink=blink, lat=lat, apply=apply, lane_min=lane_min,
                      width=width, y15=y15, ystd=ystd, lcs=lcs, tq_s=tq_s, k_model=k_model, lprob=lprob, rprob=rprob, events=np.array([list(e.values()) for e in events], dtype=object))
