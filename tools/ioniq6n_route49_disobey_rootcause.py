#!/usr/bin/env python3
"""Deep-dive on the 28 long (>=200ms) op=2 / mdps=1 clusters from route 0x49.

For each cluster, dump the surrounding state (authority, dtb, blinker,
mdpsCounter, steeringAngleDeg, vEgo, laneChangeState, cam_stale_frames)
then aggregate across clusters to identify the common trigger.

Adds vs ioniq6n_route49_mdps_disobey.py:
  - full per-frame state snapshot (not just active bits)
  - cluster-level feature rollup
  - LKAS_ALT camera-side COUNTER staleness tracking
"""

import bisect
import glob
import sys
from collections import Counter, defaultdict

import numpy as np

sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG_DIR = '/home/user/openpilot/drivelog'
ROUTE_HASH = '433dad5bb2'

# carcontroller constants (current code, torque-cal values — same as
# production during this drive; the new angle-cal fix wasn't live yet)
DZ = 25.0
FULL_LO_V, FULL_HI_V = 60.0, 120.0
LO_V, HI_V = 8.0, 15.0
ACI_ENTER, ACI_EXIT = 0.30, 0.05
ACI_SPEED_FULL = 3.0 / 3.6
ACI_SPEED_ZERO = 1.0 / 3.6


def parse_lkas_alt(dat):
    if len(dat) < 14:
        return None
    # LKAS_ANGLE_ACTIVE : 77|2@0+  → byte 9 bits 5..4
    active = (dat[9] >> 4) & 0x3
    # LKA_ASSIST        :  7|1@0+  → byte 0 bit 7  (wild guess, check DBC if
    #                                               we need this)
    # ADAS_ACIAnglTqRedcGainVal is roughly at byte 12 (from prior correlation
    # tool).  We care about active for this scan.
    return active


def compute_dtb(torque, v):
    at = abs(torque)
    full = float(np.interp(v, [LO_V, HI_V], [FULL_LO_V, FULL_HI_V]))
    if at < DZ:
        return 1.0
    if at >= full:
        return 0.0
    return float(np.clip(1.0 - (at - DZ) / max(full - DZ, 1.0), 0.0, 1.0))


def main():
    files = sorted(glob.glob(f'{DRIVELOG_DIR}/*{ROUTE_HASH}*rlog.zst'))
    print(f"loading {len(files)} segments...")

    tx_samples = []        # (t_s, op_active)
    cam_samples = []       # (t_s, cam_counter) — LKAS_ALT on bus 2
    cs_samples = []        # per-carState frame dict
    sp_samples = []        # per-carStateSP dict
    cc_samples = []        # per-carControl dict
    mv_samples = []        # per-modelV2 laneChangeState

    t0 = None
    for seg in files:
        for m in LogReader(seg):
            try:
                w = m.which()
            except Exception:
                continue
            if t0 is None:
                t0 = m.logMonoTime
            t = (m.logMonoTime - t0) / 1e9

            if w == 'sendcan':
                for c in m.sendcan:
                    if c.address == 0x110 and len(c.dat) >= 14:
                        op = parse_lkas_alt(bytes(c.dat))
                        if op is not None:
                            tx_samples.append((t, op))
            elif w == 'can':
                for c in m.can:
                    if c.address == 0x110 and c.src == 2 and len(c.dat) >= 14:
                        # Camera/ADAS-side LKAS_ALT, read COUNTER.
                        # COUNTER field assumed at byte 6 low nibble (best
                        # effort — we only track staleness via delta not abs).
                        counter = int(bytes(c.dat)[6]) & 0x0F
                        cam_samples.append((t, counter))
            elif w == 'carState':
                cs = m.carState
                cs_samples.append({
                    't': t,
                    'vEgo': cs.vEgo,
                    'steeringAngle': cs.steeringAngleDeg,
                    'steeringTorque': cs.steeringTorque,
                    'steeringPressed': cs.steeringPressed,
                    'left': cs.leftBlinker,
                    'right': cs.rightBlinker,
                    'cruiseEnabled': cs.cruiseState.enabled,
                })
            elif w == 'carStateSP':
                csp = m.carStateSP
                sp_samples.append({
                    't': t,
                    'mdpsActive': int(csp.mdpsLkaAngleActive),
                    'mdpsFault': bool(csp.mdpsLkaAngleFault),
                    'mdpsCounter': int(csp.mdpsCounter),
                    'mdpsAngle': float(getattr(csp, 'mdpsSteeringAngle', 0.0)),
                })
            elif w == 'carControl':
                cc_samples.append({
                    't': t,
                    'latActive': bool(m.carControl.latActive),
                    'actAngle': float(m.carControl.actuators.steeringAngleDeg),
                })
            elif w == 'modelV2':
                mv_samples.append({
                    't': t,
                    'lcState': str(m.modelV2.meta.laneChangeState),
                    'lcDir': str(m.modelV2.meta.laneChangeDirection),
                })

    print(f"  TX={len(tx_samples)}  cam={len(cam_samples)}  "
          f"CS={len(cs_samples)}  SP={len(sp_samples)}  "
          f"CC={len(cc_samples)}  MV={len(mv_samples)}")

    # ── Index by time for nearest-sample lookup ──
    def build_idx(samples, key='t'):
        return [s[key] if isinstance(s, dict) else s[0] for s in samples]

    sp_t = build_idx(sp_samples)
    cc_t = build_idx(cc_samples)
    cs_t = build_idx(cs_samples)
    mv_t = build_idx(mv_samples)
    cam_t = [c[0] for c in cam_samples]

    def nearest(tlist, samples, t, tol=0.06):
        if not tlist:
            return None
        i = bisect.bisect_left(tlist, t)
        cands = []
        if i < len(tlist):
            cands.append((abs(tlist[i] - t), i))
        if i > 0:
            cands.append((abs(tlist[i - 1] - t), i - 1))
        if not cands:
            return None
        dt, idx = min(cands)
        if dt > tol:
            return None
        return samples[idx]

    # ── Detect op=2 / mdps=1 frames with latActive ──
    disobey_frames = []    # list of (t, full_ctx_dict)
    for t, op in tx_samples:
        if op != 2:
            continue
        sp = nearest(sp_t, sp_samples, t)
        cc = nearest(cc_t, cc_samples, t)
        if sp is None or cc is None:
            continue
        if not cc['latActive']:
            continue
        if sp['mdpsActive'] != 1:
            continue
        cs = nearest(cs_t, cs_samples, t)
        mv = nearest(mv_t, mv_samples, t, tol=0.15)
        cam = nearest(cam_t, cam_samples, t, tol=0.1)
        if cs is None:
            continue
        dtb = compute_dtb(cs['steeringTorque'], cs['vEgo'])
        sb = float(np.clip((cs['vEgo'] - ACI_SPEED_ZERO) /
                           (ACI_SPEED_FULL - ACI_SPEED_ZERO), 0.0, 1.0))
        blink = cs['left'] or cs['right']
        authority = dtb * sb
        if blink:
            authority *= 0.2
        disobey_frames.append({
            't': t,
            'vEgo': cs['vEgo'],
            'angle': cs['steeringAngle'],
            'torque': cs['steeringTorque'],
            'steeringPressed': cs['steeringPressed'],
            'blink': blink,
            'dtb': dtb,
            'speed_blend': sb,
            'authority': authority,
            'mdpsCounter': sp['mdpsCounter'],
            'mdpsFault': sp['mdpsFault'],
            'mdpsAngle': sp['mdpsAngle'],
            'actAngle': cc['actAngle'],
            'lcState': mv['lcState'] if mv else '?',
            'camCounter': cam[1] if cam else None,
        })

    print(f"\ndisobey (op=2, mdps=1, latActive): {len(disobey_frames)} frames")
    if not disobey_frames:
        return

    # ── Cluster by 200 ms gap ──
    disobey_frames.sort(key=lambda f: f['t'])
    clusters = []
    cur = [disobey_frames[0]]
    for f in disobey_frames[1:]:
        if f['t'] - cur[-1]['t'] < 0.2:
            cur.append(f)
        else:
            clusters.append(cur)
            cur = [f]
    clusters.append(cur)
    long_clusters = [c for c in clusters
                     if (c[-1]['t'] - c[0]['t']) >= 0.2]
    print(f"total clusters: {len(clusters)}, long (>=200ms): {len(long_clusters)}")

    # ── Per-cluster summary ──
    print("\n" + "=" * 110)
    print(" Long clusters (duration >=200 ms) — context at onset")
    print("=" * 110)
    print(f"  {'t_start':>8s} {'dur_ms':>6s} {'N':>3s} "
          f"{'vEgo':>5s} {'angle':>7s} {'actAng':>7s} {'Δang':>6s} "
          f"{'torque':>7s} {'dtb':>5s} "
          f"{'auth':>5s} {'blink':>5s} {'press':>5s} "
          f"{'mdpsΔ':>6s} {'lcState':>20s}")
    features = defaultdict(list)
    for c in long_clusters:
        first, last = c[0], c[-1]
        dur = (last['t'] - first['t']) * 1000
        mdps_ct_delta = last['mdpsCounter'] - first['mdpsCounter']
        if mdps_ct_delta < 0:
            mdps_ct_delta += 256  # 8-bit counter wrap
        ang_err = first['actAngle'] - first['angle']
        # Peak angle error across the cluster
        peak_ang_err = max(abs(f['actAngle'] - f['angle']) for f in c)
        # Aggregate features at cluster onset
        features['dur_ms'].append(dur)
        features['vEgo'].append(first['vEgo'])
        features['angle'].append(first['angle'])
        features['torque'].append(first['torque'])
        features['dtb'].append(first['dtb'])
        features['authority'].append(first['authority'])
        features['blink'].append(int(first['blink']))
        features['pressed'].append(int(first['steeringPressed']))
        features['mdpsCounter_delta'].append(mdps_ct_delta)
        features['lcState'].append(first['lcState'])
        features['ang_err'].append(ang_err)
        features['peak_ang_err'].append(peak_ang_err)
        print(f"  {first['t']:8.1f} {dur:6.0f} {len(c):3d} "
              f"{first['vEgo']:5.1f} {first['angle']:7.1f} "
              f"{first['actAngle']:7.1f} {ang_err:6.1f} "
              f"{first['torque']:7.1f} "
              f"{first['dtb']:5.2f} {first['authority']:5.2f} "
              f"{str(first['blink']):>5s} {str(first['steeringPressed']):>5s} "
              f"{mdps_ct_delta:>6d} {first['lcState']:>20s}")

    # ── Aggregate: what's common across clusters? ──
    print("\n" + "=" * 110)
    print(" Feature distribution across the long clusters (onset values)")
    print("=" * 110)

    def pct(arr):
        a = np.array(arr, dtype=float)
        return f"min={a.min():.2f} p25={np.percentile(a,25):.2f} p50={np.percentile(a,50):.2f} p75={np.percentile(a,75):.2f} p90={np.percentile(a,90):.2f} max={a.max():.2f}"

    print(f"  vEgo (m/s):       {pct(features['vEgo'])}")
    print(f"  angle (deg):      {pct(features['angle'])}")
    print(f"  torque (Nm):      {pct(features['torque'])}")
    print(f"  dtb:              {pct(features['dtb'])}")
    print(f"  authority:        {pct(features['authority'])}")
    print(f"  duration (ms):    {pct(features['dur_ms'])}")
    print(f"  mdpsCounter Δ:    {pct(features['mdpsCounter_delta'])}")
    print(f"  onset ang_err:    {pct(features['ang_err'])}")
    print(f"  peak ang_err:     {pct(features['peak_ang_err'])}")

    blink_frac = sum(features['blink']) / len(features['blink'])
    press_frac = sum(features['pressed']) / len(features['pressed'])
    print(f"\n  blinker_on:       {sum(features['blink'])}/{len(features['blink'])} "
          f"({blink_frac*100:.0f}%)")
    print(f"  steeringPressed:  {sum(features['pressed'])}/{len(features['pressed'])} "
          f"({press_frac*100:.0f}%)")

    lc_counter = Counter(features['lcState'])
    print(f"\n  laneChangeState breakdown:")
    for k, v in lc_counter.most_common():
        print(f"    {k:30s} {v}")

    # ── Threshold checks against common triggers ──
    print("\n" + "=" * 110)
    print(" Trigger hypotheses")
    print("=" * 110)
    low_auth = sum(1 for a in features['authority'] if a < ACI_ENTER)
    zero_dtb = sum(1 for d in features['dtb'] if d < 0.05)
    pressed = sum(features['pressed'])
    blink = sum(features['blink'])
    lc_active = sum(1 for s in features['lcState']
                    if 'laneChange' in s and s != 'off')
    mdps_stalled = sum(1 for d in features['mdpsCounter_delta'] if d <= 1)
    mdps_normal = sum(1 for d in features['mdpsCounter_delta'] if d >= 3)
    N = len(long_clusters)
    print(f"  authority<ACI_ENTER at onset : {low_auth}/{N}  "
          f"(op sent 2 but authority would not re-latch)")
    print(f"  dtb<0.05 at onset (full ovr) : {zero_dtb}/{N}")
    print(f"  steeringPressed at onset     : {pressed}/{N}")
    print(f"  blinker_on at onset          : {blink}/{N}")
    print(f"  laneChange active at onset   : {lc_active}/{N}")
    print(f"  mdpsCounter Δ<=1 (stalled)   : {mdps_stalled}/{N}")
    print(f"  mdpsCounter Δ>=3 (healthy)   : {mdps_normal}/{N}")


if __name__ == '__main__':
    main()
