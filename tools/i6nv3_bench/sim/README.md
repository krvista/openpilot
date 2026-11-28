# roadsim - closed-loop lateral simulator (Ioniq 6 N, solo profile / direct path)

`roadsim.py` is the maintained copy of the simulator used for reports 105-122. It is numpy only, has no openpilot import
and runs 120 s at 100 Hz in about 0.1 s. `phase_tests/test_bench_sim.py` is its self-check.

Signs: curvature + = right, position `e` + = car right of the lane centre, `c0` = lane centre in the car frame = -e.

## What follows the driving code by itself, and what does not

- **Constants**: the 33 lane-term constants (`HEAD_NAMES`: lane-centre weight / P / KI / caps, KE table, entry lead, lane fit
  gain / T / range / arm / speed gate, avoidance release) are read from `openpilot/selfdrive/controls/controlsd.py` with `ast`
  when the module is imported. Only literal arithmetic is evaluated; nothing of controlsd is imported or run. `FALLBACK`
  (Phase 88 values) is used only when the file is missing, i.e. when `roadsim.py` was copied out of the repository.
  `HEAD_FROM_SOURCE` lists what was read.
- **Control law**: `head_controller()` is a hand port of `Controls._lane_center` (P, integral, offset part KE, lane fit of
  Phases 82 / 88, avoidance release of Phases 83 / 84) and `Controls._entry_lead` (Phase 78). The self-check feeds 4000 random
  frames to the port and to the real functions and requires the same curvature to 1e-12. A change of the law in controlsd
  makes that test fail; the port then has to be updated by hand.
- **Earlier builds**: `P_P79` (routes a8 / a9), `P_P84` (aa-b6: lane fit over 45-60 km/h), `P_P77`, `P_A7`, `P_A1` are the
  differences to HEAD, e.g. `rs.params(**rs.P_P79)`.

## Model

```
road        k_road(t), v(t) of the scenario (synthetic: straight | step | curve | s_curve | dropout; or log segments)
model       20 Hz, sees the car frame_age_s ago:
            K = a_ff * k_road_smooth(t + T_ff) - b_psi * psi - b_e(v) * (e - p*) + action noise
            p* = p0 + c_in * tanh(k_preview / k_in)            preferred position: its straight-line spot, inside in curves
            plan(L) - centre(L) = p* + (e - p*)(1 + w tL) exp(-w tL) + psi L exp(-w tL) + plan noise
controller  100 Hz, head_controller() (see above) or any factory(P) -> step(Obs)
chain       curvature jerk limit -> wheel angle -> 0.15 deg backlash -> 0.1 deg quantisation -> TX rate limit ->
            transport delay 0.14 s -> wheel lag 0.06 s -> vehicle lag 0.09 s -> x (1 - shortfall) + bias d(t)
car         psi' = v (k_car - k_road),  e' = v psi
```

`ENSEMBLE` (nominal, stiff, soft, lagged, eager, curvehold, quiet) spans the fitted range of the driving-model feedback;
`DRIVES` (a1, a7) carry the steering bias and the model's preferred straight-line position of two logged drives.

How the lane fit is fed: controlsd fits a quadratic to the lane centre over `clip(v * 1 s, 15, 45)` m. The simulator gives
the port the road curvature half that range ahead (`fit_lead_s`), the true heading at the car (the lever-arm correction
`LANE_FIT_ARM_M` is taken as exact; `FIT_ARM_ERR_M` is a residual error) and, with `FIT_NOISE=1`, the measured noise of the
fitted curvature (0.12e-3, tau 0.35 s) and heading (1.5 mrad, tau 0.55 s).

Avoidance release: ported in full (usual plan spot, warm-up, weight down at once / back in 1.5 s, held integral, reset after
5 s without a lane). It is driven by the simulated plan, so it acts on lane-centre steps and on plan noise; real triggers
(parked cars, cones, a model that swerves) are not in any scenario.

### Actuator error model (option, off by default)

From routes b4-b6 (report 121.2 / 121.4):

- `COMPLIANCE`: the wheel stays short of the request by this many degrees per m/s^2 of lateral acceleration.
  `rs.params(**rs.ACTUATOR_ERROR)` sets 0.39 (fit on the logs: 0.40).
- per-drive steering offset: `drive=rs.offset_drive(off_deg, v_kph)` (b4 +0.29, b5 -0.08, b6 +0.16 deg).
- `EXTRA_DELAY_S` adds transport delay.

With +0.30 deg and the compliance the simulator delivers 0.891 of the command in right curves and 1.009 in left ones at
80 km/h (logged b4: 0.86 / 1.00), and the curve position does not depend on the offset because the lane integral absorbs it
on the straight - both as in report 121.4.

## Use

```python
import sys; sys.path.insert(0, "tools/i6nv3_bench/sim")
import roadsim as rs

scn = rs.synthetic("curve", v_kph=70, k=1.8e-3)
r = rs.simulate(scn, P=rs.params(), ens="nominal", drive="a7", seed=1)     # HEAD; noise=False for clean event responses
rs.event_metrics(r)                       # entry / steady / exit position, wheel rate, jerk
rs.metrics(r)                             # by speed band: straight mean / scatter, curve positions, wheel activity
rs.steady_curve(70)                       # mean steady-curve position over left / right and both drives
rs.margins(60); rs.limit_cycle(60)        # gain / delay margin, noise-free weave
r["parts"]                                # n x 6: P, integral, KE part, entry lead, lane fit, avoidance weight
```

`python tools/i6nv3_bench/sim/roadsim.py` prints the table below.

`rs.log_scenarios()` (hands-off segments of routes a1 / a7 with their real speed and road curvature) needs `scen_logs.npz`
(5.9 MB), which is not in the repository; put it next to `roadsim.py` or point `ROADSIM_SCEN_LOGS` at it.

## How far it is from the logs

Steady curve, cm inside of the straight-line position, mean of left / right and of drives a1 / a7, nominal model, linear
chain, no noise (11-28, HEAD = Phase 88). Nothing in the simulator was tuned to these logged values.

| km/h (curvature) | HEAD | Phase 84 gates | Phase 79 set | HEAD + actuator error |
|---|---|---|---|---|
| 50 (3.0e-3) | +5.9 | +7.6 | +9.2 | +3.3 |
| 60 (2.5e-3) | +5.1 | +5.1 | +8.9 | +1.3 |
| 70 (1.8e-3) | +5.2 | +5.2 | +8.3 | +0.5 |
| 80 (1.4e-3) | +5.0 | +5.0 | +7.4 | -0.6 |
| 90 (1.1e-3) | +4.6 | +4.6 | +6.5 | -1.8 |
| 100 (0.9e-3) | +2.9 | +2.9 | +3.9 | -2.3 |

| 60-80 km/h | logged | simulator | difference |
|---|---|---|---|
| HEAD builds (b0-b6) | +3.5 cm (468 s, 97 curves) | +5.1 (perfect actuator), +0.4 (actuator error model) | +1.6 / -3.1 cm |
| Phase 79 (a8, a9) | +13.1 cm (23 s) | +8.2 | -4.9 cm |
| gain of Phase 82 | 9.6 cm (the report rates 5-10 cm as safe) | 3.1 cm | about a third |

Over the model ensemble the HEAD value spans +2.9 (soft) .. +9.1 (curvehold) and the Phase 79 value +5.6 .. +12.7; with the
non-linear chain, action / lane / fit noise and 3 seeds the means stay at +5.1 / +8.2 / +0.4 cm. The self-check allows
3 cm (HEAD), 4 cm (HEAD with the actuator option) and 6 cm (Phase 79) around the logged values and requires the Phase 79
set to sit at least 2 cm further inside. These tolerances state the error; they are not an accuracy claim.

Read the simulator for the direction and relative size of a change and for stability (margins, step response, weave), not
for absolute centimetres.

## Known gaps

- **Inside bias with a perfect actuator.** HEAD sits +5.1 cm inside (60-80 km/h; +5.6 cm in report 121.5 from the earlier
  harness). The logged 'centre of the curve' is therefore, in the simulator's view, the model's inside preference cancelled
  by the compliance; with the compliance switched on the simulated car ends at +0.4 cm, below the logged +3.5.
- **Left of centre in both curve directions.** The logs show the car 2-7 cm left of its straight-line position in left AND
  right curves (report 121.2). The simulator is symmetric and does not reproduce it. The model parameter `c_left` shifts the
  preferred position to one side in both directions but is 0 and unfitted.
- **Position sensitivity** to a wheel-angle error: about 19 cm/deg in the simulator, 7-12 cm/deg by regression on the logs
  (r 0.2), 20-30 cm/deg from the gains.
- **No wind, no road crown or bank.** The only disturbance is the curvature bias `d` (constant per drive plus a slow AR(1)
  wander of 0.04e-3). Gusts, crown changes and bank in curves are not modelled, so the slow weave on 80-110 km/h straights
  cannot be attributed with it.
- **Lane-line noise** is a fixed mix (slow AR(1) 1.5 cm, 0.08-0.45 Hz 2.5 cm, 0.6-5 Hz 0.8 cm) fitted on a1 / a7. It does not
  depend on line quality, speed, lighting or the model's own line std, has no dropouts except the `dropout` scenario, and
  the factory camera (the position the logs report) is not simulated: `e` is the true position, `e_meas` the model-line one.
- **Driving model** is a linear feedback law with a tanh inside preference, fitted on two drives (a1, a7) of an earlier
  build. Below about 35 km/h and above 100 km/h it is extrapolated.
- **Not modelled in the controller**: low-confidence blend and dropout hold, LAT_CMD smoothing and look-ahead, lane-width
  checks, the reset of the lane state on engage, lane changes / blinkers, driver torque, the car controller's yield /
  authority logic, `LatControlAngle` (its trim is off on the direct path; the Phase 87 study wrapped the real class around
  this simulator, report 121).
- **Entry lead** is fed the model's feed-forward difference over `ENTRY_LEAD_S`, not a plan yaw profile.
- **Speed** is constant in the synthetic scenarios.
