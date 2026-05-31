#!/usr/bin/env python3
"""Measure contiguous SCC-V entering and SCC-M turning episode durations from the
published longitudinalPlanSP states, split by straight vs curved road. Used to size
the entering/turning debounce: genuine-curve episodes last seconds; false positives
on straight road are brief. Writes JSON.

Usage: PYTHONPATH=$PWD python3 tools/wk2_episodes.py <out.json> <rlog glob...>
"""
import sys, glob, os, json
import zstandard, capnp

capnp.remove_import_hook()
_CD = os.path.join(os.getcwd(), 'cereal')
_LOG = capnp.load(os.path.join(_CD, 'log.capnp'), imports=[_CD, os.getcwd()])

STRAIGHT_LAT = 0.7  # measured lateral accel threshold


def read_events(path):
    raw = open(path, 'rb').read()
    data = zstandard.ZstdDecompressor().decompress(raw, max_output_size=400 * 1024 * 1024)
    for ev in _LOG.Event.read_multiple_bytes(data):
        yield ev


def run(paths):
    rows = {}
    def put(t, k, v): rows.setdefault(t, {})[k] = v
    for src in paths:
        try:
            it = read_events(src)
        except Exception:
            continue
        for ev in it:
            t = ev.logMonoTime * 1e-9
            w = ev.which()
            if w == 'longitudinalPlanSP':
                s = ev.longitudinalPlanSP.smartCruiseControl
                put(t, 'v', str(s.vision.state)); put(t, 'm', str(s.map.state))
            elif w == 'carState':
                put(t, 'vEgo', float(ev.carState.vEgo))
            elif w == 'controlsState':
                put(t, 'curv', abs(float(ev.controlsState.curvature)))
    seq = [(t, rows[t]) for t in sorted(rows)]
    # forward-fill vEgo, curv
    last = {}
    for _, r in seq:
        for k in ('vEgo', 'curv'):
            if k in r: last[k] = r[k]
            elif k in last: r[k] = last[k]

    def episodes(key, active_val):
        eps = []
        run_start = None; frames = 0; straight_frames = 0
        prev_t = None
        for t, r in seq:
            st = r.get(key)
            if st is None:
                continue
            if st == active_val:
                if run_start is None:
                    run_start = t; frames = 0; straight_frames = 0
                frames += 1
                vE = r.get('vEgo', 0.0); cu = r.get('curv', 0.0)
                if vE > 8 and vE * vE * cu < STRAIGHT_LAT:
                    straight_frames += 1
                prev_t = t
            else:
                if run_start is not None:
                    eps.append({'start': round(run_start, 1), 'dur_s': round(prev_t - run_start, 2),
                                'frames': frames, 'straight_frames': straight_frames})
                    run_start = None
        if run_start is not None:
            eps.append({'start': round(run_start, 1), 'dur_s': round(prev_t - run_start, 2),
                        'frames': frames, 'straight_frames': straight_frames})
        return eps

    v_eps = episodes('v', 'entering')
    m_eps = episodes('m', 'turning')

    def summarize(eps):
        straight = [e for e in eps if e['straight_frames'] >= max(1, e['frames'] // 2)]
        return {
            'count': len(eps),
            'straight_dominant_count': len(straight),
            'durations_s': sorted(round(e['dur_s'], 2) for e in eps),
            'straight_dominant': straight,
        }

    return {'vision_entering': summarize(v_eps), 'map_turning': summarize(m_eps)}


def main(argv):
    out_path, srcs = argv[0], argv[1:]
    ins = []
    for a in srcs:
        ins += sorted(glob.glob(a)) or [a]
    out = run(ins)
    with open(out_path, 'w') as f:
        json.dump(out, f, indent=2)
    print(f"wrote {out_path}")


if __name__ == '__main__':
    main(sys.argv[1:])
