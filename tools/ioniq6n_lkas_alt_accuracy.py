#!/usr/bin/env python3
"""
Hyundai Ioniq 6 — Camera vs openpilot steering accuracy analysis.
Decodes LKAS_ALT (0x110) sendcan frames, carControl, and carState messages,
then buckets errors by speed to surface where camera drops out.
"""
import sys, glob, math
sys.path.insert(0, '/home/user/openpilot')
sys.path.insert(0, '/home/user/openpilot/opendbc_repo')

from openpilot.tools.lib.logreader import LogReader

# ── Signal decoders ──────────────────────────────────────────────────────────
def _le_signed(d, start_bit, length, factor):
    raw = 0
    for i in range(length):
        bp = start_bit + i
        if d[bp >> 3] & (1 << (bp & 7)):
            raw |= (1 << i)
    if raw & (1 << (length - 1)):
        raw -= (1 << length)
    return raw * factor

def _motorola(d, start_bit, length, factor=1.0):
    raw = 0
    for i in range(length):
        bp = start_bit - i
        if d[bp >> 3] & (1 << (bp & 7)):
            raw |= (1 << (length - 1 - i))
    return raw * factor

def decode_lkas_alt(dat):
    d = bytes(dat)
    # ADAS_StrAnglReqVal  : bit 82, 14b, LE signed, ×0.1 → deg
    # LKAS_ANGLE_ACTIVE   : bit 77, 2b,  Motorola (big-endian)
    # ADAS_ACIAnglTqRedcGainVal : bit 96, 8b, LE unsigned, ×0.004
    cam_angle = _le_signed(d, 82, 14, 0.1)
    active    = int(_motorola(d, 77, 2))
    gain      = _le_signed(d, 96, 8, 0.004)
    return cam_angle, active, gain

# ── Speed buckets ────────────────────────────────────────────────────────────
BUCKETS = [(0,5),(5,15),(15,30),(30,50),(50,80),(80,999)]
BUCKET_LABELS = ["0-5","5-15","15-30","30-50","50-80","80+"]

def bucket_idx(v_kmh):
    for i,(lo,hi) in enumerate(BUCKETS):
        if lo <= v_kmh < hi:
            return i
    return len(BUCKETS)-1

# ── Collection structures ────────────────────────────────────────────────────
# per bucket: list of (cam_err, op_err, cam_active, cam_angle_abs)
from collections import defaultdict
buckets = defaultdict(list)   # idx -> list of (cam_err, op_err, cam_active, op_active)

# ── Main pass: parse all segments ───────────────────────────────────────────
files = sorted(glob.glob('/home/user/openpilot/drivelog/*rlog.zst'))
print(f"Found {len(files)} segments. Parsing …", flush=True)

# State carried forward within each segment
for seg_i, fpath in enumerate(files):
    try:
        lr = LogReader(fpath)
    except Exception as e:
        print(f"  SKIP {fpath}: {e}")
        continue

    last_cc_angle  = None   # carControl.actuators.steeringAngleDeg
    last_lat_active = False
    last_cs_angle  = None   # carState.steeringAngleDeg (actual)
    last_speed_kmh = None   # carState.vEgoRaw * 3.6
    prev_cc_ts     = 0.0
    prev_cs_ts     = 0.0

    for msg in lr:
        w = msg.which()
        if w == 'carControl':
            cc = msg.carControl
            last_cc_angle   = cc.actuators.steeringAngleDeg
            last_lat_active = cc.latActive
        elif w == 'carState':
            cs = msg.carState
            last_cs_angle  = cs.steeringAngleDeg
            last_speed_kmh = cs.vEgoRaw * 3.6
        elif w == 'sendcan':
            if last_cs_angle is None or last_cc_angle is None or last_speed_kmh is None:
                continue
            for frame in msg.sendcan:
                if frame.address != 0x110 or frame.src != 0:
                    continue
                cam_angle, cam_active, cam_gain = decode_lkas_alt(frame.dat)
                bidx = bucket_idx(last_speed_kmh)
                cam_err = abs(cam_angle - last_cs_angle)
                op_err  = abs(last_cc_angle - last_cs_angle)
                buckets[bidx].append((cam_err, op_err, cam_active, last_lat_active))

    if (seg_i+1) % 50 == 0:
        print(f"  … {seg_i+1}/{len(files)} segments done", flush=True)

print(f"Done. Total matched frames: {sum(len(v) for v in buckets.values())}\n")

# ── Statistics ───────────────────────────────────────────────────────────────
def pct(lst, p):
    if not lst: return float('nan')
    s = sorted(lst)
    i = int(math.ceil(p/100.0 * len(s))) - 1
    return s[max(0,i)]

def mae(lst):
    return sum(lst)/len(lst) if lst else float('nan')

# Print header
W = 10
HDR = (f"{'Speed':>8} | {'N':>6} | "
       f"{'CamMAE':>7} {'Cmp95':>7} {'Cmp99':>7} | "
       f"{'OpMAE':>7} {'Op_p95':>7} {'Op_p99':>7} | "
       f"{'Cam==0':>7} {'Cam==1':>7} {'Cam==2':>7} | "
       f"{'DropRate%':>10}")
SEP = "-" * len(HDR)

print("Camera vs openpilot steering accuracy — Hyundai Ioniq 6 LKAS_ALT (0x110)")
print("Angle errors relative to actual wheel angle (carState.steeringAngleDeg)")
print(SEP)
print(HDR)
print(SEP)

for bidx, label in enumerate(BUCKET_LABELS):
    rows = buckets[bidx]
    if not rows:
        print(f"{label:>8} |{'(no data)':>70}")
        continue

    cam_errs  = [r[0] for r in rows]
    op_errs   = [r[1] for r in rows]
    actives   = [r[2] for r in rows]
    lat_acts  = [r[3] for r in rows]

    n_total   = len(rows)
    n_act0    = actives.count(0)
    n_act1    = actives.count(1)
    n_act2    = actives.count(2)

    # Dropout: camera passive (active<2) while openpilot is lat-active
    op_active_frames  = [i for i,r in enumerate(rows) if r[3]]
    dropout_frames    = [i for i in op_active_frames if rows[i][2] < 2]
    dropout_pct = 100.0 * len(dropout_frames) / len(op_active_frames) if op_active_frames else float('nan')

    print(f"{label+' km/h':>8} | {n_total:>6} | "
          f"{mae(cam_errs):>7.2f} {pct(cam_errs,95):>7.2f} {pct(cam_errs,99):>7.2f} | "
          f"{mae(op_errs):>7.2f} {pct(op_errs,95):>7.2f} {pct(op_errs,99):>7.2f} | "
          f"{100*n_act0/n_total:>7.1f} {100*n_act1/n_total:>7.1f} {100*n_act2/n_total:>7.1f} | "
          f"{dropout_pct:>10.1f}")

print(SEP)

# ── Dropout by angle magnitude ───────────────────────────────────────────────
all_rows = [r for rows in buckets.values() for r in rows]
op_rows  = [r for r in all_rows if r[3]]  # only when op is lat-active

ANG_BUCKETS = [(0,2),(2,5),(5,10),(10,20),(20,999)]
ANG_LABELS  = ["0-2","2-5","5-10","10-20","20+"]

print()
print("Camera dropout by |cam_angle| magnitude (when op lat-active)")
print("(dropout = camera ACTIVE < 2 while op is steering)")
print(f"{'|cam|° rng':>12} | {'N':>6} | {'Dropout%':>9} | {'MeanCamAng':>11}")
print("-"*50)
for (lo,hi), lbl in zip(ANG_BUCKETS, ANG_LABELS):
    subset = [r for r in op_rows if lo <= abs(r[0]) < hi]
    if not subset:
        continue
    drops = [r for r in subset if r[2] < 2]
    print(f"{lbl+' deg':>12} | {len(subset):>6} | {100*len(drops)/len(subset):>9.1f} | "
          f"{sum(abs(r[0]) for r in subset)/len(subset):>11.2f}")

# ── Overall summary ──────────────────────────────────────────────────────────
print()
print("OVERALL SUMMARY")
print("-"*50)
all_cam = [r[0] for r in all_rows]
all_op  = [r[1] for r in all_rows]
print(f"Total frames analysed : {len(all_rows):,}")
print(f"Camera MAE            : {mae(all_cam):.3f}°   p95={pct(all_cam,95):.2f}°  p99={pct(all_cam,99):.2f}°")
print(f"Op     MAE            : {mae(all_op):.3f}°   p95={pct(all_op,95):.2f}°  p99={pct(all_op,99):.2f}°")

op_active_all = [r for r in all_rows if r[3]]
drop_all = [r for r in op_active_all if r[2] < 2]
if op_active_all:
    print(f"Camera dropout (op active): {100*len(drop_all)/len(op_active_all):.1f}%  "
          f"({len(drop_all):,}/{len(op_active_all):,} frames)")
    act_dist = {v: sum(1 for r in op_active_all if r[2]==v) for v in [0,1,2]}
    print(f"LKAS_ANGLE_ACTIVE dist (op active): "
          f"0={act_dist[0]/len(op_active_all)*100:.1f}%  "
          f"1={act_dist[1]/len(op_active_all)*100:.1f}%  "
          f"2={act_dist[2]/len(op_active_all)*100:.1f}%")

print()
print("INTERPRETATION GUIDE")
print("  LKAS_ANGLE_ACTIVE=2 → camera fully engaged  (op blends at high α)")
print("  LKAS_ANGLE_ACTIVE=1 → camera tentative/transitioning")
print("  LKAS_ANGLE_ACTIVE=0 → camera disengaged     (op must dominate → low α)")
print("  High dropout% at a speed/angle bucket = α should be LOW there")
