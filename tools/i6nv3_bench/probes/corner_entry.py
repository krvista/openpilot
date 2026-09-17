"""Corner-entry latency probe (09-17): "the wheel starts turning a touch late on corner entry".

For each hands-off corner entry (lat active, no blinker, v >= 30 km/h, |k_model| rising from < K_LO to >= K_HI within
ENTRY_S) the probe times when each stage crosses the half level K_MID, relative to the model plan:
  model  = modelV2.action.desiredCurvature          (the plan)
  ctrl   = controlsState.desiredCurvature           (after controlsd post-processing: LP smoothing, confidence blend, entry assist)
  wire   = LKAS_ALT ADAS_StrAnglReqVal (sendcan)    (after CarController: curve trim, sp_smooth, VM governor) converted to curvature
  wheel  = controlsState.curvature                  (measured vehicle curvature; MDPS + vehicle response)
Also: ACI gain on the wire during the entry, the wheel/plan delivery ratio at the first plan peak, and whether the driver
assisted (pressed with |tq| >= ASSIST_NM in the same direction within ASSIST_S). Wire angle -> curvature uses a per-route
linear fit of controlsState.curvature vs carState.steeringAngleDeg on |angle| >= 3 deg frames (speed-independent
approximation: good enough for crossing times, not for absolute gains).
Usage: PYTHONPATH=... python probes/corner_entry.py <route> [--list] [--npz out.npz]
"""
import glob
import sys

import numpy as np

from openpilot.tools.lib.logreader import LogReader

K_LO, K_MID, K_HI, ENTRY_S, ASSIST_NM, ASSIST_S, MIN_KPH = 0.3e-3, 0.9e-3, 1.5e-3, 2.0, 100.0, 3.0, 30.0
r = sys.argv[1]
npz = sys.argv[sys.argv.index("--npz") + 1] if "--npz" in sys.argv else None
files = sorted(glob.glob(f"/home/user/drivelog/drivelog/*_{r}--*--rlog.zst"), key=lambda f: int(f.split("--")[-2]))
cs, ct, md, cc, wr = [], [], [], [], []
t0 = None
for f in files:
  for m in LogReader(f):
    w = m.which(); t = m.logMonoTime * 1e-9
    if t0 is None:
      t0 = t
    if w == "carState":
      s = m.carState
      cs.append((t, s.vEgo * 3.6, s.steeringAngleDeg, s.steeringPressed, s.steeringTorque, s.leftBlinker or s.rightBlinker))
    elif w == "controlsState":
      ct.append((t, m.controlsState.desiredCurvature, m.controlsState.curvature))
    elif w == "modelV2":
      d = m.modelV2
      lm = min(d.laneLineProbs[1], d.laneLineProbs[2]) if len(d.laneLineProbs) >= 4 else 1.0
      md.append((t, d.action.desiredCurvature, lm, int(d.meta.laneChangeState.raw)))
    elif w == "carControl":
      cc.append((t, m.carControl.latActive))
    elif w == "sendcan":
      for c in m.sendcan:
        if c.address == 272 and len(c.dat) >= 13:
          d = bytes(c.dat); raw = ((d[10] >> 2) | (d[11] << 6)) & 0x3FFF
          if raw & 0x2000:
            raw -= 0x4000
          wr.append((t, raw * 0.1, d[12] * 0.004))
CS, CT, MD, CC, WR = (np.array(a, float) for a in (cs, ct, md, cc, wr))
if min(len(CS), len(CT), len(MD), len(CC), len(WR)) == 0:
  print(f"== {r}: no data"); sys.exit(0)
T = CS[:, 0]; v = CS[:, 1]; ang = CS[:, 2]; pressed = CS[:, 3] > 0.5; tq_s = CS[:, 4]; blink = CS[:, 5] > 0.5
k_ctrl = np.interp(T, CT[:, 0], CT[:, 1]); k_meas = np.interp(T, CT[:, 0], CT[:, 2])
k_model = np.interp(T, MD[:, 0], MD[:, 1]); lane_min = np.interp(T, MD[:, 0], MD[:, 2]); lcs = np.interp(T, MD[:, 0], MD[:, 3])
lat = np.interp(T, CC[:, 0], CC[:, 1]) > 0.5
wire = np.interp(T, WR[:, 0], WR[:, 1]); gain = np.interp(T, WR[:, 0], WR[:, 2])
# angle -> curvature ratio (per route, |angle| >= 3 deg, moving)
mfit = (np.abs(ang) >= 3.0) & (v >= 20)
ratio = float(np.sum(k_meas[mfit] * ang[mfit]) / np.sum(ang[mfit] ** 2)) if mfit.sum() > 100 else 1.0 / 900.0
k_wire = wire * ratio
n = len(T); e_frames = int(ENTRY_S * 100); a_frames = int(ASSIST_S * 100)
hands_off = ~pressed          # steeringPressed only: this driver's resting column torque sits at 60-180 Nm (EPS reaction)
elig = lat & ~blink & (lcs < 0.5) & (v >= MIN_KPH)
ak = np.abs(k_model)
events = []
i = 300
while i < n - 600:
  if elig[i] and ak[i] < K_LO and hands_off[i]:
    seg = ak[i:i + e_frames]
    hit = np.where(seg >= K_HI)[0]
    if len(hit) and elig[i:i + hit[0]].all() and hands_off[i:i + hit[0]].all():
      j = i + hit[0]                                   # plan reaches K_HI
      sgn = np.sign(k_model[j])
      win = slice(i, min(n, j + 300))
      def cross(sig):
        s = sig[win] * sgn
        c = np.where(s >= K_MID)[0]
        return (T[i + c[0]] - T[i]) if len(c) else None
      t_model = cross(k_model); t_ctrl = cross(k_ctrl); t_wire = cross(k_wire); t_meas = cross(k_meas)
      # first plan peak within 3 s after K_HI; delivery = meas/model there (same-direction)
      pk = j + int(np.argmax((k_model[j:j + 300] * sgn)))
      deliv = float((k_meas[pk] * sgn) / max(k_model[pk] * sgn, 1e-6))
      asst = np.where(pressed[j:j + a_frames] & (np.abs(tq_s[j:j + a_frames]) >= ASSIST_NM))[0]
      assist_t = asst[0] / 100 if len(asst) else None
      # curvature sign is opposite to the angle/torque sign on this car (fitted ratio < 0): map through sign(ratio)
      assist_same = bool(np.sign(tq_s[j + asst[0]]) == sgn * np.sign(ratio)) if len(asst) else None
      events.append(dict(t=T[i] - t0, seg=int((T[i] - t0) // 60), v=v[j], k_hi=k_model[pk] * sgn, lane_min=lane_min[i:j].min(),
                         t_model=t_model, t_ctrl=t_ctrl, t_wire=t_wire, t_meas=t_meas, gain=gain[i:j + 100].mean(),
                         gain_min=gain[i:j + 100].min(), deliv=deliv, assist_t=assist_t, assist_same=assist_same))
      i = j + 300
      continue
  i += 1
def lagstat(a, b):
  d = [e[b] - e[a] for e in events if e[a] is not None and e[b] is not None]
  return (f"{np.median(d):.2f}/{np.percentile(d, 90):.2f} s (n={len(d)})") if d else "-"
print(f"== {r}: entries {len(events)} (hands-off, >= {MIN_KPH:.0f} km/h) | angle->k ratio {ratio * 1e3:.3f}e-3/deg | "
      f"assist within {ASSIST_S:.0f} s: {sum(e['assist_t'] is not None for e in events)} "
      f"(same dir {sum(bool(e['assist_same']) for e in events)})")
print(f"   lag p50/p90  model->ctrl {lagstat('t_model', 't_ctrl')} | ctrl->wire {lagstat('t_ctrl', 't_wire')} | "
      f"wire->wheel {lagstat('t_wire', 't_meas')} | model->wheel {lagstat('t_model', 't_meas')}")
dl = [e['deliv'] for e in events]; g = [e['gain'] for e in events]
if events:
  print(f"   delivery at first plan peak p25/50/75 {np.percentile(dl, 25):.2f}/{np.median(dl):.2f}/{np.percentile(dl, 75):.2f} | "
        f"wire ACI gain during entry p25/50/75 {np.percentile(g, 25):.2f}/{np.median(g):.2f}/{np.percentile(g, 75):.2f} | v p50 {np.median([e['v'] for e in events]):.0f} km/h")
  # split: entries the driver assisted vs not
  for lab, sel in (("assisted", [e for e in events if e['assist_t'] is not None]), ("clean", [e for e in events if e['assist_t'] is None])):
    if sel:
      mw = [e['t_meas'] - e['t_model'] for e in sel if e['t_meas'] is not None and e['t_model'] is not None]
      print(f"   {lab:8s} n={len(sel):3d}: model->wheel p50 {np.median(mw) if mw else float('nan'):.2f} s | gain p50 {np.median([e['gain'] for e in sel]):.2f} | "
            f"deliv p50 {np.median([e['deliv'] for e in sel]):.2f} | lane_min p50 {np.median([e['lane_min'] for e in sel]):.2f} | k_hi p50 {np.median([e['k_hi'] for e in sel]) * 1e3:.1f}e-3 | v p50 {np.median([e['v'] for e in sel]):.0f}")
if "--list" in sys.argv:
  fmt = lambda x: "  -  " if x is None else f"{x:5.2f}"
  for e in events:
    assist_s = "-" if e['assist_t'] is None else f"{e['assist_t']:.1f}s " + ("same" if e['assist_same'] else "opp")
    print(f"  seg{e['seg']:2d} t={e['t']:6.1f} v={e['v']:3.0f} k={e['k_hi'] * 1e3:4.1f}e-3 lm={e['lane_min']:.2f} model {fmt(e['t_model'])} ctrl {fmt(e['t_ctrl'])} "
          f"wire {fmt(e['t_wire'])} wheel {fmt(e['t_meas'])} gain {e['gain']:.2f}(min {e['gain_min']:.2f}) deliv {e['deliv']:.2f} "
          f"assist {assist_s}")
if npz:
  np.savez_compressed(npz, T=T - t0, v=v, ang=ang, pressed=pressed, tq_s=tq_s, blink=blink, lat=lat, k_model=k_model, k_ctrl=k_ctrl,
                      k_wire=k_wire, k_meas=k_meas, gain=gain, lane_min=lane_min, ratio=ratio,
                      events=np.array([list(e.values()) for e in events], dtype=object), keys=np.array(list(events[0].keys())) if events else np.array([]))
