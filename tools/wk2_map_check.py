#!/usr/bin/env python3
"""For every published SCC-M (map) turning frame, recompute what the vision predicted
lat-acc would have been at that instant (from modelV2), to size a vision cross-check
gate that rejects map slowdowns the camera does not corroborate.

Writes JSON. Usage: PYTHONPATH=$PWD python3 tools/wk2_map_check.py <out.json> <rlog glob...>
"""
import sys, glob, os, json
import numpy as np
import zstandard, capnp

capnp.remove_import_hook()
_CD = os.path.join(os.getcwd(), 'cereal')
_LOG = capnp.load(os.path.join(_CD, 'log.capnp'), imports=[_CD, os.getcwd()])
_T_IDXS = np.array([10.0 * (i / 32.0) ** 2 for i in range(33)])
_ANTI_MASK = (_T_IDXS >= 3.0) & (_T_IDXS <= 5.0)


def read_events(path):
    raw = open(path, 'rb').read()
    data = zstandard.ZstdDecompressor().decompress(raw, max_output_size=400 * 1024 * 1024)
    for ev in _LOG.Event.read_multiple_bytes(data):
        yield ev


def run(paths):
    pred_now = 0.0      # latest vision near-horizon p97
    anti_now = 0.0      # latest vision 3-5s window max
    v_ego = 0.0
    turning_frames = []  # (map_vt_kph, v_cruise_kph, pred, anti)
    for src in paths:
        try:
            it = read_events(src)
        except Exception:
            continue
        for ev in it:
            w = ev.which()
            if w == 'carState':
                v_ego = float(ev.carState.vEgo)
            elif w == 'modelV2':
                m = ev.modelV2
                z = np.abs(np.array(m.orientationRate.z, dtype=float))
                vx = np.array(m.velocity.x, dtype=float)
                n = min(len(z), len(vx))
                if n >= 5:
                    pred = z[:n] * vx[:n]
                    pred_now = float(np.percentile(pred, 97))
                    aw = pred[:n][_ANTI_MASK[:n]]
                    anti_now = float(aw.max()) if aw.size else 0.0
            elif w == 'longitudinalPlanSP':
                s = ev.longitudinalPlanSP.smartCruiseControl
                if str(s.map.state) == 'turning':
                    turning_frames.append((float(s.map.vTarget) * 3.6,
                                           v_ego * 3.6,
                                           pred_now, anti_now))
    tf = turning_frames
    out = {'turning_frames': len(tf)}
    if tf:
        arr = np.array(tf)
        out['map_vt_kph'] = {'min': round(float(arr[:, 0].min()), 1), 'max': round(float(arr[:, 0].max()), 1)}
        out['drop_kph'] = {'min': round(float((arr[:, 1] - arr[:, 0]).min()), 1),
                           'max': round(float((arr[:, 1] - arr[:, 0]).max()), 1)}
        out['vision_pred_during_turning'] = {
            'p50': round(float(np.percentile(arr[:, 2], 50)), 3),
            'p90': round(float(np.percentile(arr[:, 2], 90)), 3),
            'max': round(float(arr[:, 2].max()), 3)}
        out['vision_anti_during_turning'] = {
            'p50': round(float(np.percentile(arr[:, 3], 50)), 3),
            'p90': round(float(np.percentile(arr[:, 3], 90)), 3),
            'max': round(float(arr[:, 3].max()), 3)}
        # gate: how many turning frames would survive if we require vision corroboration
        for floor in (0.5, 0.8, 1.0, 1.3):
            survive = int(np.sum(np.maximum(arr[:, 2], arr[:, 3]) >= floor))
            out[f'survive_vision_floor_{floor}'] = survive
    return out


def main(argv):
    out_path, srcs = argv[0], argv[1:]
    ins = []
    for a in srcs:
        ins += sorted(glob.glob(a)) or [a]
    out = run(ins)
    with open(out_path, 'w') as f:
        json.dump(out, f, indent=2)
    print(f"wrote {out_path}: turning_frames={out['turning_frames']}")


if __name__ == '__main__':
    main(sys.argv[1:])
