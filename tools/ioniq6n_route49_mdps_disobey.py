#!/usr/bin/env python3
"""Did ADAS DRV / MDPS ever ignore MADS commands on route 0x49?

Three patterns to detect:

  P1 (op→2, MDPS stays 1):
      TX LKAS_ANGLE_ACTIVE = 2 (op says "I'm driving")
      but carStateSP.mdpsLkaAngleActive = 1 (MDPS still in passive)
      → ADAS DRV REFUSED to hand over. This is the user's hypothesis.

  P2 (op→2, MDPS fault):
      TX LKAS_ANGLE_ACTIVE = 2
      but carStateSP.mdpsLkaAngleFault = True
      → MDPS rejected with fault.

  P3 (late handover):
      TX flips 1→2 at frame k, but mdpsLkaAngleActive doesn't follow 1→2
      within N frames (e.g., 10 frames / 200 ms).
      → measure handover latency distribution.

Matching TX and mdps fields by time: carStateSP publishes at same cadence
as carState (100 Hz), CAN TX at 50 Hz → for each TX LKAS_ALT sample, take
the nearest carStateSP (±20 ms).
"""

import bisect
import glob
import sys
from collections import Counter

sys.path.insert(0, '/home/user/openpilot')
from openpilot.tools.lib.logreader import LogReader

DRIVELOG_DIR = '/home/user/openpilot/drivelog'
ROUTE_HASH = '433dad5bb2'


def parse_lkas_alt_tx(dat_bytes):
    if len(dat_bytes) < 14:
        return None
    # DBC: LKAS_ANGLE_ACTIVE : 77|2@0+  → byte 9, bits 5..4 → shift 4
    lkas_angle_active = (dat_bytes[9] >> 4) & 0x3
    return lkas_angle_active


def main():
    files = sorted(glob.glob(f'{DRIVELOG_DIR}/*{ROUTE_HASH}*rlog.zst'))
    if not files:
        print("no rlogs")
        return
    print(f"loading {len(files)} segments...")

    # time-series for fast nearest-match lookup
    tx_samples = []       # (t_s, op_active)
    sp_samples = []       # (t_s, mdps_active, mdps_fault, mdps_counter)
    cs_samples = []       # (t_s, lat_active, v_ego)

    t0 = None
    for seg in files:
        for m in LogReader(seg):
            try:
                w = m.which()
            except Exception:
                continue
            if t0 is None:
                t0 = m.logMonoTime
            t_s = (m.logMonoTime - t0) / 1e9

            if w == 'sendcan':
                for c in m.sendcan:
                    if c.address == 0x110 and len(c.dat) >= 14:
                        op_active = parse_lkas_alt_tx(bytes(c.dat))
                        if op_active is not None:
                            tx_samples.append((t_s, op_active))

            elif w == 'carStateSP':
                csp = m.carStateSP
                sp_samples.append((t_s,
                                   int(csp.mdpsLkaAngleActive),
                                   bool(csp.mdpsLkaAngleFault),
                                   int(csp.mdpsCounter)))

            elif w == 'carControl':
                cs_samples.append((t_s, bool(m.carControl.latActive), 0.0))

    print(f"  TX LKAS_ALT: {len(tx_samples)}, carStateSP: {len(sp_samples)}, "
          f"carControl: {len(cs_samples)}")

    if not sp_samples:
        print("\nERROR: no carStateSP samples — need build with MDPS diag fields")
        return

    sp_t = [s[0] for s in sp_samples]
    cc_t = [c[0] for c in cs_samples]

    def nearest_sp(t):
        i = bisect.bisect_left(sp_t, t)
        cands = []
        if i < len(sp_t):
            cands.append((abs(sp_t[i] - t), i))
        if i > 0:
            cands.append((abs(sp_t[i - 1] - t), i - 1))
        if not cands:
            return None
        dt, idx = min(cands)
        if dt > 0.05:
            return None
        return sp_samples[idx]

    def nearest_cc(t):
        i = bisect.bisect_left(cc_t, t)
        cands = []
        if i < len(cc_t):
            cands.append((abs(cc_t[i] - t), i))
        if i > 0:
            cands.append((abs(cc_t[i - 1] - t), i - 1))
        if not cands:
            return None
        dt, idx = min(cands)
        if dt > 0.05:
            return None
        return cs_samples[idx]

    # ── Pattern 1 & 2: per-TX mismatch scan ──
    counts = Counter()
    p1_events = []      # op→2 but mdps=1
    p2_events = []      # op→2 but mdps fault
    prev_tx = None
    handover_lags = []  # frames from op 1→2 to mdps 1→2

    pending_handover = None

    for t, op in tx_samples:
        sp = nearest_sp(t)
        cc = nearest_cc(t)
        if sp is None or cc is None:
            continue
        _, mdps_active, mdps_fault, mdps_counter = sp
        lat_active = cc[1]

        counts[('op', op)] += 1
        counts[('mdps', mdps_active)] += 1
        counts[('pair', op, mdps_active)] += 1

        if lat_active and op == 2 and mdps_active == 1:
            p1_events.append(t)
        if lat_active and op == 2 and mdps_fault:
            p2_events.append(t)

        # track 1→2 handover latency on op TX
        if prev_tx is not None and prev_tx != 2 and op == 2 and lat_active:
            pending_handover = (t, 0)
        if pending_handover is not None:
            start_t, _ = pending_handover
            if mdps_active == 2:
                handover_lags.append(t - start_t)
                pending_handover = None
            elif t - start_t > 2.0:
                # gave up — count as "never handed over within 2s"
                handover_lags.append(float('inf'))
                pending_handover = None
        prev_tx = op

    total = sum(counts[('op', k)] for k in range(3))
    print("\n" + "=" * 72)
    print(" op TX LKAS_ANGLE_ACTIVE distribution (all TX samples)")
    print("=" * 72)
    for k in range(3):
        print(f"  op={k}: {counts[('op', k)]:>7d}  ({counts[('op', k)]/max(total,1)*100:5.1f}%)")

    print("\n" + "=" * 72)
    print(" MDPS (carStateSP.mdpsLkaAngleActive) at same instant")
    print("=" * 72)
    for k in range(3):
        print(f"  mdps={k}: {counts[('mdps', k)]:>7d}  ({counts[('mdps', k)]/max(total,1)*100:5.1f}%)")

    print("\n" + "=" * 72)
    print(" Pair table (rows=op TX, cols=mdps same instant)")
    print("=" * 72)
    print(f"  {'':>8s}  {'mdps=0':>8s} {'mdps=1':>8s} {'mdps=2':>8s}")
    for op_k in range(3):
        row = [counts[('pair', op_k, m)] for m in range(3)]
        print(f"  op={op_k}:    {row[0]:>8d} {row[1]:>8d} {row[2]:>8d}")

    print("\n" + "=" * 72)
    print(" Pattern 1: op→2 but MDPS=1 (with latActive)  ← disobey / refusal")
    print("=" * 72)
    print(f"  count: {len(p1_events)}")
    if p1_events:
        # Cluster: consecutive frames within 0.2 s = one event
        clusters = []
        cur = [p1_events[0]]
        for t in p1_events[1:]:
            if t - cur[-1] < 0.2:
                cur.append(t)
            else:
                clusters.append(cur)
                cur = [t]
        clusters.append(cur)
        print(f"  clusters (gap≥200ms split): {len(clusters)}")
        durations = [(c[-1] - c[0]) * 1000 for c in clusters]
        import numpy as np
        arr = np.array(durations)
        print(f"  duration ms: p50={np.percentile(arr,50):.0f} "
              f"p75={np.percentile(arr,75):.0f} "
              f"p90={np.percentile(arr,90):.0f} "
              f"p99={np.percentile(arr,99):.0f} "
              f"max={arr.max():.0f}")
        long_clusters = [c for c in clusters if (c[-1] - c[0]) >= 0.2]
        print(f"  clusters ≥200ms (real refusal candidates): {len(long_clusters)}")
        for c in long_clusters[:10]:
            print(f"    t={c[0]:7.1f}s  duration={((c[-1]-c[0])*1000):.0f}ms  "
                  f"samples={len(c)}")

    print("\n" + "=" * 72)
    print(" Pattern 2: op→2 but MDPS fault (with latActive)")
    print("=" * 72)
    print(f"  count: {len(p2_events)}")
    if p2_events:
        print(f"  first 5 timestamps (s): {[round(t,1) for t in p2_events[:5]]}")

    print("\n" + "=" * 72)
    print(" Pattern 3: op 1→2 handover latency (op decides to engage)")
    print("=" * 72)
    finite_lags = [l for l in handover_lags if l != float('inf')]
    timeouts = sum(1 for l in handover_lags if l == float('inf'))
    if finite_lags:
        import numpy as np
        arr = np.array(finite_lags)
        print(f"  handovers: {len(finite_lags)}  (never within 2s: {timeouts})")
        print(f"  latency p50/p75/p90/p99 (ms): "
              f"{arr.min()*1000:.0f} / "
              f"{np.percentile(arr,50)*1000:.0f} / "
              f"{np.percentile(arr,75)*1000:.0f} / "
              f"{np.percentile(arr,90)*1000:.0f} / "
              f"{np.percentile(arr,99)*1000:.0f} / "
              f"max {arr.max()*1000:.0f}")
    else:
        print("  no 1→2 handover transitions observed")


if __name__ == '__main__':
    main()
