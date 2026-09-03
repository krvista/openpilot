# ccnc_analysis — i6nv2 drivelog analysis tools

Offline tools used to tune the CCNC angle-steering branch from `krvista/drivelog`
rlogs. Kept in-tree because the session scratchpad is ephemeral.

Pipeline:

1. `distill2.py <rows.tsv>` — rlog.zst → compact `.npz` per segment (100 Hz carState,
   carControl/carOutput angle, carStateSP pause, 10 Hz modelV2 summary, driverAssistance
   (op LDW), onroadEvents). Rows: `route--dongle\tseg\tpath-or-blob`. Env: `REPO`,
   `NPZ_OUT` (default `./npz2`). v4 adds per-side blinker, lane-line lateral positions,
   model lane-change state and op LDW flags (BSD prep).
2. `report.py <route-prefix>...` — unified defect detectors per route (shake bands,
   yank, regrab, hands-off divergence, LDW, events).
3. Replays (all take route prefixes, env `REPO` selects the tree whose CarController is
   replayed; `phase_tests/harness.Sim` is the driver):
   - `cc_ab_replay.py` — 0.8-2.5 Hz TX band RMS A/B between two trees.
   - `gate_metrics.py` — Phase 35a grip-descent gate metrics (`GATE_NM`, `KILL35A`).
   - `grip_dynamics.py`, `postgrip_exposure.py` — grip onset / post-grip exposure.
   - `curve_exposure.py` — Phase 36 curve ceiling exposure (`KILL36`).
   - `highspeed_scan.py`, `highspeed_scan2.py` — >= 60 km/h baseline: BSM presence,
     wheel rate / implied lateral jerk (op-driven subset vs all hands-off), plan step vs
     VM jerk cap, post-release correction peaks (clean vs re-grip).
   - `bsm_scan.py` — blindspot episodes at >= 40 km/h and what moved toward that side
     (needs v4 npz, env `NPZ`).

Conventions: "eff-active" = carControl.latActive and not carStateSP.lateralControlPaused;
"hands-off" = not steeringPressed; "op-driven" = hands-off, replayed driver_tq < 30 Nm,
no press within +/-2 s, no blinker. Implied lateral jerk = v^2 * wheel_rate / (SR * wb)
from a 100 ms-smoothed steering angle (steeringRateDeg is 4 deg/s-quantized).
