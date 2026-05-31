#!/usr/bin/env python3
"""Measure the SCC-V predicted-lat-accel distribution on straight vs curved road,
recomputing exactly what the controller computes, straight from modelV2 + controlsState.

This lets us pick robust thresholds (anticipation percentile vs raw-max, entering
threshold) with data instead of guessing. Writes JSON (bash stdout unreliable).

Usage: PYTHONPATH=$PWD python3 tools/wk2_dist.py <out.json> <rlog glob...>
"""
import sys, glob, os, json
import numpy as np
import zstandard, capnp

capnp.remove_import_hook()
_CD = os.path.join(os.getcwd(), 'cereal')
_LOG = capnp.load(os.path.join(_CD, 'log.capnp'), imports=[_CD, os.getcwd()])

# Mirror vision_controller.py exactly.
_T_IDXS = np.array([10.0 * (i / 32.0) ** 2 for i in range(33)])
_ANTICIPATE_MASK = (_T_IDXS >= 3.0) & (_T_IDXS <= 5.0)
STRAIGHT_CURV = 0.002   # |controlsState.curvature| below this == straight (R > 500m)
CURVED_CURV = 0.006     # above this == genuine curve


def read_events(path):
    raw = open(path, 'rb').read()
    data = zstandard.ZstdDecompressor().decompress(raw, max_output_size=400 * 1024 * 1024)
    for ev in _LOG.Event.read_multiple_bytes(data):
        yield ev


def run(paths):
    # latest controlsState curvature + carState vEgo, paired with each modelV2
    cur_curv = 0.0
    v_ego = 0.0
    rows = []  # (vEgo, curv, p97_near, max_near, p90_anti, max_anti, mean_anti)
    for src in paths:
        try:
            it = read_events(src)
        except Exception:
            continue
        for ev in it:
            w = ev.which()
            if w == 'controlsState':
                cur_curv = abs(float(ev.controlsState.curvature))
            elif w == 'carState':
                v_ego = float(ev.carState.vEgo)
            elif w == 'modelV2':
                m = ev.modelV2
                z = np.abs(np.array(m.orientationRate.z, dtype=float))
                vx = np.array(m.velocity.x, dtype=float)
                n = min(len(z), len(vx))
                if n < 5:
                    continue
                pred = z[:n] * vx[:n]
                p97 = float(np.percentile(pred, 97))
                mx = float(pred.max())
                mask = _ANTICIPATE_MASK[:n]
                aw = pred[:n][mask]
                if aw.size == 0:
                    continue
                rows.append((v_ego, cur_curv, p97, mx,
                             float(np.percentile(aw, 90)), float(aw.max()), float(aw.mean())))
    arr = np.array(rows) if rows else np.zeros((0, 7))

    def stats(sub, col):
        if sub.shape[0] == 0:
            return None
        v = sub[:, col]
        return {'n': int(v.size), 'p50': round(float(np.percentile(v, 50)), 3),
                'p90': round(float(np.percentile(v, 90)), 3),
                'p99': round(float(np.percentile(v, 99)), 3),
                'max': round(float(v.max()), 3)}

    # only moving (>8 m/s)
    mv = arr[arr[:, 0] > 8] if arr.shape[0] else arr
    straight = mv[mv[:, 1] < STRAIGHT_CURV]
    curved = mv[mv[:, 1] > CURVED_CURV]
    cols = {2: 'near_p97', 3: 'near_max', 4: 'anti_p90', 5: 'anti_max', 6: 'anti_mean'}
    out = {
        'total_model_frames': int(arr.shape[0]),
        'moving_frames': int(mv.shape[0]),
        'straight_frames': int(straight.shape[0]),
        'curved_frames': int(curved.shape[0]),
        'straight': {name: stats(straight, c) for c, name in cols.items()},
        'curved': {name: stats(curved, c) for c, name in cols.items()},
    }
    return out


def main(argv):
    out_path, srcs = argv[0], argv[1:]
    ins = []
    for a in srcs:
        ins += sorted(glob.glob(a)) or [a]
    out = run(ins)
    with open(out_path, 'w') as f:
        json.dump(out, f, indent=2)
    print(f"wrote {out_path}: straight={out['straight_frames']} curved={out['curved_frames']} frames")


if __name__ == '__main__':
    main(sys.argv[1:])
