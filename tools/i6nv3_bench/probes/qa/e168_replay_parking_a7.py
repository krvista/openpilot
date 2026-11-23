"""Open-loop replay of route a7 through the real car controller: Phase 75 rules (PARKING_V2 False) vs Phase 76."""
import sys
sys.path.insert(0, "/home/user/wt3/phase_tests")
import numpy as np
from harness import Sim
from opendbc.car.hyundai.values import CarControllerParams as P
S = sys.argv[1]; P.PARKING_V2 = sys.argv[2] == "new"
d = dict(np.load(f"{S}/r50/000000a7.npz", allow_pickle=True)); T = d['T']; n = len(T); v = d['v']; lat = d['lat'].astype(bool)
g = np.load(f"{S}/pk/000000a7.npy"); tg = g[:, 0] - g[0, 0] + T[0]; fixseen = np.maximum.accumulate(g[:, 3] > 0)
fs = np.interp(T, tg, fixseen.astype(float)) > 0.5
GEAR = {1: 'park', 2: 'drive', 3: 'neutral', 4: 'reverse'}
s = Sim(solo=True); park = np.zeros(n, bool); act = np.zeros(n, bool)
for i in range(n):
    s.cc._cc_sp.mads.enabled = bool(lat[i])
    s.step(v=v[i] / 3.6, tq=float(d['tq_s'][i]), wheel=float(d['ang'][i]), cmd=float(d['apply'][i]), lat_active=bool(lat[i]),
           pressed=bool(d['pressed'][i]), blinker=bool(d['lb'][i]), blinker_right=bool(d['rb'][i]), lane_prob_min=float(d['lane_min'][i]),
           lead_dist=(float(d['lead_d'][i]) if d['lead'][i] else None), lfa_state=int(d['lfa_sym'][i]), gw_req_angle=float(d['cb_req'][i]),
           gw_req_lvl2=int(d['cb_lvl2'][i]), a_ego=float(d['a_ego'][i]), mdps_angle_2=float(d['mdps_ang'][i]), wheel_rate=float(d['rate'][i]),
           gas=bool(d['gas'][i]), tx_rejected=bool(d['tx_reject'][i] > 0), gear=GEAR.get(int(d['gear'][i]), 'drive'),
           cruise_enabled=bool(d['cruise'][i]), gps_fix_seen=bool(fs[i]), lat_granted=bool(d['lat_granted'][i]))
    park[i] = s.cc.parking_mode_active; act[i] = s.lkas_alt()["LKAS_ANGLE_ACTIVE"] == 2
e = np.flatnonzero(np.diff(np.r_[0, park.astype(np.int8), 0])); mv = v > 5
print(sys.argv[2], "parking episodes:", [(round(float(T[a]), 1), round(float(T[min(b, n - 1)]), 1)) for a, b in zip(e[::2], e[1::2])])
print(f"  latched & moving & lateral on {np.sum(park & mv & lat)/100:.1f} s | requesting {np.sum(act & mv)/100:.1f} s | logged requesting {np.sum((d['tx_active'] == 2) & mv)/100:.1f} s | first request at {T[np.argmax(act)]:.1f} s")
