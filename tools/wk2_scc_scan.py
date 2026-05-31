#!/usr/bin/env python3
"""WK2 SCC false-positive scanner (direct capnp decode; no openpilot hw stack).

Detects SCC-V / SCC-M going active while the road is effectively straight
(low *measured* lateral accel) and flags whether the driver re-accelerated
(gas) or cancelled ACC shortly after. Writes a JSON report (bash stdout is
unreliable in this sandbox; always read the JSON).

cereal Vision schema has NO anticipatedLateralAccel (internal-only). We infer
SCC-V false positives from state transitions + maxPredictedLateralAccel while
currentLateralAccel is near zero.

Usage: PYTHONPATH=$PWD python3 tools/wk2_scc_scan.py <out.json> <glob-or-path...>
"""
import sys, glob, os, json, collections
import zstandard, capnp

capnp.remove_import_hook()
_CD = os.path.join(os.getcwd(), 'cereal')
_LOG = capnp.load(os.path.join(_CD, 'log.capnp'), imports=[_CD, os.getcwd()])

STRAIGHT_LAT = 0.7   # m/s^2 measured lat accel below this == effectively straight
REACT_WIN    = 4.0   # s after engage to look for driver gas/cancel
MIN_SPEED    = 8.0   # m/s
ACTIVE_V = {'entering', 'turning', 'leaving'}
ACTIVE_M = {'turning'}


def read_events(path):
    raw = open(path, 'rb').read()
    data = zstandard.ZstdDecompressor().decompress(raw, max_output_size=400 * 1024 * 1024)
    for ev in _LOG.Event.read_multiple_bytes(data):
        yield ev


def collect(paths):
    rows = {}
    skipped = []
    def put(t, k, v): rows.setdefault(t, {})[k] = v
    for src in paths:
        try:
            it = read_events(src)
        except Exception as e:
            skipped.append(f"{src}: {e!r}"); continue
        try:
            for ev in it:
                t = ev.logMonoTime * 1e-9
                w = ev.which()
                if w == 'longitudinalPlanSP':
                    lp = ev.longitudinalPlanSP
                    try:
                        s = lp.smartCruiseControl
                        put(t, 'v_state', str(s.vision.state)); put(t, 'v_vt', float(s.vision.vTarget))
                        put(t, 'v_maxpred', float(s.vision.maxPredictedLateralAccel))
                        put(t, 'v_cur', float(s.vision.currentLateralAccel))
                        put(t, 'm_state', str(s.map.state)); put(t, 'm_vt', float(s.map.vTarget))
                        put(t, 'm_vcruise', float(s.map.vCruise))
                    except Exception:
                        pass
                    try:
                        put(t, 'src', str(lp.longitudinalPlanSource))
                    except Exception:
                        pass
                elif w == 'carState':
                    cs = ev.carState
                    put(t, 'vEgo', float(cs.vEgo)); put(t, 'aEgo', float(cs.aEgo))
                    put(t, 'gas', bool(cs.gasPressed)); put(t, 'cruise_en', bool(cs.cruiseState.enabled))
                elif w == 'controlsState':
                    put(t, 'curv', float(ev.controlsState.curvature))
        except Exception as e:
            skipped.append(f"{src} (mid-stream): {e!r}")
    return [(t, rows[t]) for t in sorted(rows)], skipped


def ffill(rows, keys):
    last = {}
    for _, r in rows:
        for k in keys:
            if k in r: last[k] = r[k]
            elif k in last: r[k] = last[k]
    return rows


def scan(rows):
    rows = ffill(rows, ['v_state', 'm_state', 'vEgo', 'curv', 'cruise_en',
                        'v_cur', 'v_maxpred', 'm_vt', 'm_vcruise', 'src'])
    ev = []
    pv = pm = 'disabled'
    for i, (t, r) in enumerate(rows):
        vEgo = r.get('vEgo', 0.0); curv = r.get('curv', 0.0)
        cur = vEgo * vEgo * abs(curv)
        vs = r.get('v_state', 'disabled'); ms = r.get('m_state', 'disabled')
        v_eng = vs in ACTIVE_V and pv not in ACTIVE_V
        m_eng = ms in ACTIVE_M and pm not in ACTIVE_M
        if (v_eng or m_eng) and cur < STRAIGHT_LAT and vEgo > MIN_SPEED:
            react = None
            for tt, rr in rows[i:]:
                if tt - t > REACT_WIN: break
                if rr.get('gas'): react = 'gas_tap'; break
                if rr.get('cruise_en') is False: react = 'acc_cancel'; break
            ev.append({
                't': round(t, 1), 'who': 'SCC-V' if v_eng else 'SCC-M',
                'state': vs if v_eng else ms, 'vkph': round(vEgo * 3.6, 0),
                'curLat': round(cur, 2), 'maxpred': round(r.get('v_maxpred', 0.0), 2),
                'm_vt': round(r.get('m_vt', 0.0), 1), 'src': r.get('src'),
                'react': react,
            })
        pv, pm = vs, ms
    return ev


def main(argv):
    out_path, srcs = argv[0], argv[1:]
    ins = []
    for a in srcs:
        ins += (sorted(glob.glob(os.path.join(a, '**', '*log*.zst'), recursive=True)) or [a]) \
               if os.path.isdir(a) else (sorted(glob.glob(a)) or [a])
    rows, skipped = collect(ins)
    ev = scan(rows)
    rep = {
        'sources': len(ins), 'samples': len(rows), 'skipped': skipped[:20],
        'candidates': len(ev), 'reacted': sum(1 for e in ev if e['react']),
        'by_who': dict(collections.Counter(e['who'] for e in ev)),
        'by_reaction': dict(collections.Counter(e['react'] for e in ev if e['react'])),
        'events': ev,
    }
    with open(out_path, 'w') as f:
        json.dump(rep, f, indent=2, default=str)
    print(f"wrote {out_path}: {len(ev)} candidates / {len(rows)} samples / {len(ins)} sources")


if __name__ == '__main__':
    main(sys.argv[1:])
