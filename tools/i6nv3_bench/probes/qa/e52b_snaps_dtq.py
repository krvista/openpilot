"""E52: Phase 58 hand-on gap cap through the REAL CarController (phase_tests harness, open loop: logged wheel / torque /
command). Replays every E50 gap episode of 98e912d (8 s warm-up .. close + 1 s) with the cap OFF and ON and reports:
fidelity (OFF vs logged TX), hold-compensated driver torque before the close (is the hand detected?), and the stored gap
|TX - wheel| at the release (max over the 0.5 s before the close) OFF vs ON, for snaps and all episodes.
Usage (repo root): PYTHONPATH=$PWD:$PWD/opendbc_repo python3 e52_p58_replay.py <scratch> [ON_NM]"""
import json
import sys
import numpy as np
sys.path.insert(0, ".")
from phase_tests.harness import Sim  # noqa: E402
from opendbc.car.hyundai.values import CarControllerParams as CCP  # noqa: E402

S = sys.argv[1]
if len(sys.argv) > 2:
  CCP.HAND_GAP_ON_NM = float(sys.argv[2]); CCP.HAND_GAP_OFF_NM = 0.67 * CCP.HAND_GAP_ON_NM
CAP = CCP.HAND_GAP_CAP_DEG
E = [e for e in json.load(open(f"{S}/qa/e50_events.json")) if e['snap']]
cache = {}


def replay(d, j0, j1, cap):
  CCP.HAND_GAP_CAP_DEG = cap
  sim = Sim(); tx = np.full(j1 - j0, np.nan); dtq = np.zeros(j1 - j0); on = np.zeros(j1 - j0, bool)
  for n, k in enumerate(range(j0, j1)):
    sim.step(v=float(d['v'][k]) / 3.6, tq=float(d['tq_s'][k]), wheel=float(d['ang'][k]), cmd=float(d['apply'][k]),
             lat_active=bool(d['lat'][k]), pressed=bool(d['pressed'][k]), blinker=bool(d['lb'][k]), blinker_right=bool(d['rb'][k]),
             a_ego=float(d['a_ego'][k]), gas=bool(d['gas'][k]), lane_prob_min=float(d['lane_min'][k]),
             bs_l=bool(d['bsl'][k]), bs_r=bool(d['bsr'][k]))
    m = sim.lkas_alt()
    if m:
      tx[n] = m["ADAS_StrAnglReqVal"]
    dtq[n] = max(0.0, abs(float(d['tq_s'][k])) - sim.cc.hold_comp_last)
    on[n] = sim.cc.hand_gap_cap is not None
  return tx, dtq, on


rows = []
for e in E:
  if e['r'] not in cache:
    cache[e['r']] = dict(np.load(f"{S}/r50/{e['r']}.npz", allow_pickle=True))
  d = cache[e['r']]; T = d['T']
  i = int(np.searchsorted(T, e['t'])); k = int(np.searchsorted(T, e['tc']))
  j0 = max(i - 800, 0); j1 = min(k + 100, len(T))
  off, dtq, _ = replay(d, j0, j1, 0.0)
  on_, _, act = replay(d, j0, j1, CAP)
  ang = d['ang'][j0:j1]; lg = d['wire'][j0:j1]
  w = slice(i - j0, k - j0 + 1); wr = slice(max(k - j0 - 50, i - j0), k - j0 + 1)
  fid = np.nanmedian(np.abs(off[w] - lg[w]))
  g_log = np.nanmax(np.abs(lg[wr] - ang[wr])); g_off = np.nanmax(np.abs(off[wr] - ang[wr])); g_on = np.nanmax(np.abs(on_[wr] - ang[wr]))
  pw = dtq[max(k - j0 - 150, 0):k - j0 + 1]
  print(f'   pre-close 1.5 s driver-domain tq: max {pw.max():.0f}, share >= 60 {np.mean(pw >= 60):.0%}, share >= 30 {np.mean(pw >= 30):.0%}')
  rows.append(dict(e=e, fid=fid, dtq=float(np.median(dtq[i - j0 - 50 if i - j0 >= 50 else 0:max(k - j0 - 40, i - j0 + 1)])), g_log=g_log, g_off=g_off, g_on=g_on,
                   act=float(np.mean(act[wr]))))
  print(e['r'], f"{e['t']:.1f}", 'SNAP' if e['snap'] else '', f"fid {fid:.2f} dtq {rows[-1]['dtq']:.0f} gap log/off/on {g_log:.1f}/{g_off:.1f}/{g_on:.1f}", flush=True)
for nm, R in (("all", rows), ("snaps", [x for x in rows if x['e']['snap']])):
  A = np.array([[x['fid'], x['dtq'], x['g_log'], x['g_off'], x['g_on'], x['act']] for x in R])
  print(f"== [{nm}] n {len(A)} | fidelity |TX off - logged| p50 {np.median(A[:, 0]):.2f} p90 {np.percentile(A[:, 0], 90):.2f} deg | driver-domain torque before p50 {np.median(A[:, 1]):.0f} "
        f"| cap active at the release {np.mean(A[:, 5] > 0.5):.0%} | stored gap at release p50 off {np.median(A[:, 3]):.1f} -> on {np.median(A[:, 4]):.1f} deg, "
        f">= 8 deg {np.mean(A[:, 3] >= 8):.0%} -> {np.mean(A[:, 4] >= 8):.0%}, >= 12 {np.mean(A[:, 3] >= 12):.0%} -> {np.mean(A[:, 4] >= 12):.0%}")
  # expected snaps by the E50 odds per gap band
  odds = lambda g: 0.0 if g < 5 else (0.07 if g < 8 else (0.24 if g < 12 else 0.48))
  print(f"   expected snaps by E50 gap-band odds: off {sum(odds(x['g_off']) for x in R):.1f} -> on {sum(odds(x['g_on']) for x in R):.1f}")
