#!/usr/bin/env python3
"""Replay-validate SCC-V state-machine changes against real rlogs.

Faithfully reimplements the SCC-V vision state machine (both OLD and NEW
parameterizations) and replays it over modelV2 + controlsState + carState from
the rlogs. Reports entering engagements split by straight vs curved road, so we
can confirm the fix kills straight-road false positives while preserving genuine
curve behavior. Also sanity-checks the reimplementation by comparing the OLD
config's engagement count against what was actually published on-device.

Usage: PYTHONPATH=$PWD python3 tools/wk2_validate.py <out.json> <rlog glob...>
"""
import sys, glob, os, json
import numpy as np
import zstandard, capnp

capnp.remove_import_hook()
_CD = os.path.join(os.getcwd(), 'cereal')
_LOG = capnp.load(os.path.join(_CD, 'log.capnp'), imports=[_CD, os.getcwd()])

_T_IDXS = np.array([10.0 * (i / 32.0) ** 2 for i in range(33)])
_ANTICIPATE_MASK = (_T_IDXS >= 3.0) & (_T_IDXS <= 5.0)
MIN_V = 20 / 3.6
STRAIGHT_LAT = 0.7  # measured lat accel: straight if below


def read_events(path):
    raw = open(path, 'rb').read()
    data = zstandard.ZstdDecompressor().decompress(raw, max_output_size=400 * 1024 * 1024)
    for ev in _LOG.Event.read_multiple_bytes(data):
        yield ev


def frames(paths):
    """yield per-modelV2-frame dict with the inputs the controller sees."""
    cur_curv = 0.0
    v_ego = 0.0
    pub_v_state = 'disabled'
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
            elif w == 'longitudinalPlanSP':
                pub_v_state = str(ev.longitudinalPlanSP.smartCruiseControl.vision.state)
            elif w == 'modelV2':
                m = ev.modelV2
                z = np.abs(np.array(m.orientationRate.z, dtype=float))
                vx = np.array(m.velocity.x, dtype=float)
                n = min(len(z), len(vx))
                if n < 5:
                    continue
                pred = z[:n] * vx[:n]
                aw = pred[:n][_ANTICIPATE_MASK[:n]]
                if aw.size == 0:
                    continue
                yield {
                    'v_ego': v_ego, 'curv': cur_curv,
                    'cur_lat': v_ego * v_ego * cur_curv,
                    'p97': float(np.percentile(pred, 97)),
                    'anti_max': float(aw.max()),
                    'anti_p75': float(np.percentile(aw, 75)),
                    'pub': pub_v_state,
                }


class VisionSM:
    """Faithful reimplementation of vision_controller state machine (entry/abort only)."""
    def __init__(self, enter_th, abort_th, anti_th, anti_abort_th,
                 turning_th=2.0, leaving_pred_th=1.4, leaving_lat_th=1.5, finish_th=1.3,
                 debounce=0, use_anti_p75=False):
        self.enter_th = enter_th; self.abort_th = abort_th
        self.anti_th = anti_th; self.anti_abort_th = anti_abort_th
        self.turning_th = turning_th; self.leaving_pred_th = leaving_pred_th
        self.leaving_lat_th = leaving_lat_th; self.finish_th = finish_th
        self.debounce = debounce; self.use_anti_p75 = use_anti_p75
        self.state = 'enabled'   # assume feature+long enabled while we evaluate
        self.cand = 0

    def step(self, f):
        anti = f['anti_p75'] if self.use_anti_p75 else f['anti_max']
        mp = f['p97']; cur = f['cur_lat']
        s = self.state
        if s == 'enabled':
            if f['v_ego'] <= MIN_V:
                self.cand = 0
            else:
                cond = (mp >= self.enter_th) or (anti >= self.anti_th)
                if cond:
                    self.cand += 1
                    if self.cand > self.debounce:
                        self.state = 'entering'; self.cand = 0
                else:
                    self.cand = 0
        elif s == 'entering':
            if cur >= self.turning_th:
                self.state = 'turning'
            elif mp < self.abort_th and anti < self.anti_abort_th:
                self.state = 'enabled'
        elif s == 'turning':
            if mp <= self.leaving_pred_th or cur <= self.leaving_lat_th:
                self.state = 'leaving'
        elif s == 'leaving':
            if cur >= self.turning_th:
                self.state = 'turning'
            elif cur < self.finish_th:
                self.state = 'enabled'
        return self.state


def evaluate(fs, sm):
    """count entering ENGAGEMENTS (enabled->entering transitions) split straight/curve."""
    prev = 'enabled'
    straight_eng = 0; curve_eng = 0
    active_frames_straight = 0; active_frames_curve = 0
    for f in fs:
        st = sm.step(f)
        if st in ('entering', 'turning', 'leaving'):
            if f['cur_lat'] < STRAIGHT_LAT and f['v_ego'] > 8:
                active_frames_straight += 1
            else:
                active_frames_curve += 1
        if st == 'entering' and prev not in ('entering', 'turning', 'leaving'):
            if f['cur_lat'] < STRAIGHT_LAT and f['v_ego'] > 8:
                straight_eng += 1
            else:
                curve_eng += 1
        prev = st
    return {'straight_engagements': straight_eng, 'curve_engagements': curve_eng,
            'active_frames_straight': active_frames_straight,
            'active_frames_curve': active_frames_curve}


def run(paths):
    fs = list(frames(paths))
    # published (on-device OLD) active-state engagements for sanity
    pub_straight = pub_curve = 0
    prevp = 'disabled'
    for f in fs:
        p = f['pub']
        if p == 'entering' and prevp not in ('entering', 'turning', 'leaving'):
            if f['cur_lat'] < STRAIGHT_LAT and f['v_ego'] > 8:
                pub_straight += 1
            else:
                pub_curve += 1
        prevp = p

    configs = {
        'OLD':  dict(enter_th=2.0, abort_th=1.6, anti_th=1.3, anti_abort_th=1.1, debounce=0),
        'A_enter2.3_anti1.8_db5': dict(enter_th=2.3, abort_th=1.8, anti_th=1.8, anti_abort_th=1.5, debounce=5, use_anti_p75=True),
        'B_enter2.0_anti1.8_db3': dict(enter_th=2.0, abort_th=1.6, anti_th=1.8, anti_abort_th=1.4, debounce=3),
        'C_enter2.0_anti1.8_db5': dict(enter_th=2.0, abort_th=1.6, anti_th=1.8, anti_abort_th=1.4, debounce=5),
        'D_enter2.0_anti1.6_db3': dict(enter_th=2.0, abort_th=1.6, anti_th=1.6, anti_abort_th=1.3, debounce=3),
    }
    out = {'frames': len(fs),
           'published_old_engagements': {'straight': pub_straight, 'curve': pub_curve}}
    for name, kw in configs.items():
        out[name] = evaluate(fs, VisionSM(**kw))
    return out


def main(argv):
    out_path, srcs = argv[0], argv[1:]
    ins = []
    for a in srcs:
        ins += sorted(glob.glob(a)) or [a]
    out = run(ins)
    with open(out_path, 'w') as f:
        json.dump(out, f, indent=2)
    print(f"wrote {out_path}: frames={out['frames']}")


if __name__ == '__main__':
    main(sys.argv[1:])
