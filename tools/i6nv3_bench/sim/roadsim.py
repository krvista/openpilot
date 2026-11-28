"""roadsim - closed-loop lateral simulator for the Ioniq 6 N solo profile (openpilot commands the MDPS alone).
numpy only. READ README.md (same directory) for what is modelled, how far it is from the logs and what is missing.

The controller constants are READ FROM openpilot/selfdrive/controls/controlsd.py when this module is imported (HEAD,
HEAD_FROM_SOURCE), so the defaults cannot fall behind the driving code; the control LAW is a port (head_controller) and has
to follow by hand when controlsd's _lane_center / _entry_lead change (phase_tests/test_bench_sim.py compares the port with
the real functions on random inputs).

Signs (as in the logs): curvature + = RIGHT; lateral position e + = car RIGHT of the lane centre; heading psi + = car
pointing right of the lane; lane-centre y in the car frame c0 = -e.  (Steering angle + = left in the car; the simulator
keeps the wheel angle in 'curvature sign' degrees, wheel_deg = k * deg_per_k, i.e. + = right. Only magnitudes are used.)

Loop (100 Hz):
  road        k_road(t), v(t) from the scenario
  model       20 Hz, sees the car MODEL['frame_age_s'] ago:
              K = a_ff * k_road_smooth(t + T_ff) - b_psi * psi - b_e(v) * (e_seen - p*) + action noise
              p* = p0 + c_in * tanh(k_preview / k_in)           (preferred position: straight / inside of curves)
              plan(L) - centre(L) = p* + (e_seen - p*) (1 + w tL) exp(-w tL) + psi L exp(-w tL) + plan noise, tL = L / v
  controller  100 Hz, default = controlsd's direct path at HEAD: lane-centre P / I / E parts (Phases 73c / 77), lane fit
              (Phases 82 / 88), avoidance release (Phases 83 / 84), entry lead (Phase 78); replaceable
  chain       curvature jerk limit -> wheel angle (deg per 1e-3 by speed) -> 0.15 deg backlash -> 0.1 deg quantisation ->
              TX rate limit -> transport delay -> first-order wheel lag -> first-order vehicle lag -> x (1 - load-
              proportional shortfall, option ACTUATOR_ERROR) + bias d(t)
  car         psi' = v (k_car - k_road),  e' = v psi

Quick start (PYTHONPATH not needed):
  import sys; sys.path.insert(0, "tools/i6nv3_bench/sim"); import roadsim as rs
  scn = rs.synthetic("curve", v_kph=70, k=1.8e-3)       # or rs.log_scenarios("a7")[0] (needs scen_logs.npz, see README)
  res = rs.simulate(scn, P=rs.params(), drive="a7", ens="nominal", seed=1)          # HEAD
  res79 = rs.simulate(scn, P=rs.params(**rs.P_P79))                                 # Phase 79 parameter set
  resA = rs.simulate(scn, P=rs.params(**rs.ACTUATOR_ERROR), drive=rs.offset_drive(0.30, 70))   # with the actuator error model
  print(rs.event_metrics(res), rs.steady_curve(70))
  # own control law:
  def my_factory(P):
      def step(o):                      # o: Obs (see class Obs); return the curvature command (1/m) [, tuple of parts]
          return o.k_action + 0.4e-3 * (o.c0 - 0.034)
      return step
  res = rs.simulate(scn, controller=my_factory)
  python tools/i6nv3_bench/sim/roadsim.py            # the fidelity table of the README
"""
import ast
import math
import os

import numpy as np

DT = 0.01
HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------------------------------------------------------------
# controller constants: read from controlsd.py (literal top-level assignments only; nothing of controlsd is imported or run)
CONTROLSD = os.path.normpath(os.path.join(HERE, "..", "..", "..", "openpilot", "selfdrive", "controls", "controlsd.py"))
HEAD_NAMES = (
  "LANE_CENTER_WEIGHT", "LANE_CENTER_T_S", "LANE_CENTER_L_M", "LANE_CENTER_ACC", "LANE_CENTER_TAU_S", "LANE_CENTER_KI",
  "LANE_CENTER_I_CAP", "LANE_CENTER_I_MAX_K", "LANE_CENTER_I_BLEED_S", "LANE_CENTER_PROB", "LANE_CENTER_SPEED_MS",
  "LANE_CENTER_BIAS_M", "LANE_CENTER_KE_SPEED_MS", "LANE_CENTER_KE",
  "ENTRY_LEAD_S", "ENTRY_LEAD_CAP", "ENTRY_LEAD_K", "ENTRY_LEAD_WASHOUT_S", "ENTRY_LEAD_MIN_MS", "ENTRY_LEAD_TAU_S",
  "LANE_FIT_GAIN", "LANE_FIT_SPEED_MS", "LANE_FIT_T_S", "LANE_FIT_RANGE_S", "LANE_FIT_RANGE_M", "LANE_FIT_ARM_M",
  "LANE_AVOID_RELEASE", "LANE_AVOID_DEV_M", "LANE_AVOID_CURVE", "LANE_AVOID_USUAL_S", "LANE_AVOID_RECOVER_S",
  "LANE_AVOID_RESET_S", "LANE_AVOID_WARMUP_S",
)
# used only when controlsd.py is not there (the file copied out of the repository): the values of Phase 88 (b385a975)
FALLBACK = dict(
  LANE_CENTER_WEIGHT=0.5, LANE_CENTER_T_S=2.0, LANE_CENTER_L_M=(12.0, 80.0), LANE_CENTER_ACC=0.4, LANE_CENTER_TAU_S=0.3,
  LANE_CENTER_KI=0.06e-3, LANE_CENTER_I_CAP=0.30e-3, LANE_CENTER_I_MAX_K=1.0e-3, LANE_CENTER_I_BLEED_S=20.0,
  LANE_CENTER_PROB=(0.5, 0.8), LANE_CENTER_SPEED_MS=(30.0 / 3.6, 45.0 / 3.6), LANE_CENTER_BIAS_M=-0.034,
  LANE_CENTER_KE_SPEED_MS=(45.0 / 3.6, 60.0 / 3.6, 80.0 / 3.6, 110.0 / 3.6), LANE_CENTER_KE=(0.0, 0.0, 0.0, 0.0),
  ENTRY_LEAD_S=0.10, ENTRY_LEAD_CAP=0.15e-3, ENTRY_LEAD_K=(0.15e-3, 0.4e-3), ENTRY_LEAD_WASHOUT_S=0.6,
  ENTRY_LEAD_MIN_MS=30.0 / 3.6, ENTRY_LEAD_TAU_S=0.05,
  LANE_FIT_GAIN=0.35, LANE_FIT_SPEED_MS=(40.0 / 3.6, 55.0 / 3.6), LANE_FIT_T_S=1.5, LANE_FIT_RANGE_S=1.0,
  LANE_FIT_RANGE_M=(15.0, 45.0), LANE_FIT_ARM_M=1.8,
  LANE_AVOID_RELEASE=True, LANE_AVOID_DEV_M=(0.20, 0.45), LANE_AVOID_CURVE=0.25, LANE_AVOID_USUAL_S=10.0,
  LANE_AVOID_RECOVER_S=1.5, LANE_AVOID_RESET_S=5.0, LANE_AVOID_WARMUP_S=3.0,
)


def read_controlsd(path=CONTROLSD, names=HEAD_NAMES):
  """{NAME: value} of the module-level 'NAME = <literal arithmetic>' assignments of controlsd.py (lists become tuples).
  A name that is missing or is not plain arithmetic on literals is left out. {} when the file is not there."""
  try:
    with open(path, encoding="utf-8") as f:
      tree = ast.parse(f.read())
  except (OSError, SyntaxError):
    return {}
  out = {}
  for node in tree.body:
    if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name) and node.targets[0].id in names:
      if any(isinstance(x, (ast.Name, ast.Call, ast.Attribute)) for x in ast.walk(node.value)):
        continue
      try:
        v = eval(compile(ast.Expression(node.value), path, "eval"), {"__builtins__": {}}, {})  # noqa: S307 - literals only
      except Exception:  # noqa: BLE001
        continue
      out[node.targets[0].id] = tuple(v) if isinstance(v, (list, tuple)) else v
  return out


_SRC = read_controlsd()
HEAD_FROM_SOURCE = tuple(k for k in HEAD_NAMES if k in _SRC)      # all of HEAD_NAMES inside the repository
HEAD = {k: _SRC.get(k, FALLBACK[k]) for k in HEAD_NAMES}

# chain after controlsd + simulator-only knobs. Every constant can be overridden: params(NAME=..)
CHAIN = dict(
  CURV_JERK=5.0,                      # m/s^3, clip_curvature-type limit on the command (d k / dt <= CURV_JERK / v^2)
  DEG_PER_K_KPH=(40.0, 60.0, 80.0, 100.0), DEG_PER_K=(2.85, 3.12, 3.45, 3.74),     # wheel deg per 1e-3 1/m
  HYST_SPEED_MS=(7.0, 12.0), HYST_DEG=(0.50, 0.15),                                # car controller backlash band
  QUANT_DEG=0.1,                      # CAN resolution of the request
  TX_JERK=3.59,                       # m/s^3: TX governor / panda limit on the request rate (deg/s = TX_JERK / v^2 * deg_per_k)
  ACT_DELAY_S=0.14,                   # transport: controlsd -> CAN -> ADAS controller relay (~30 ms) -> MDPS starts moving
  WHEEL_TAU_S=0.06,                   # MDPS first-order lag (request -> wheel = ACT_DELAY - 0.02 + WHEEL_TAU ~ 0.18 s)
  VEH_TAU_S=0.09,                     # wheel -> path curvature (command -> curvature ~ 0.26-0.29 s in total)
  ACT_GAIN=1.0,                       # curvature delivered per curvature commanded
  LINEAR=False,                       # True: no backlash / quantisation / rate limits (used by margins())
  # actuator error model (report 121.2 / 121.4, routes b4-b6): the wheel (sensor) stays short of the request in proportion
  # to the lateral load. 0 = perfect actuator (default). ACTUATOR_ERROR below switches it on; the per-drive steering offset
  # is a curvature bias of the drive (offset_drive()).
  COMPLIANCE=0.0,                     # deg of shortfall per m/s^2 of lateral acceleration
  EXTRA_DELAY_S=0.0,                  # added to ACT_DELAY_S
  # lane fit as the camera delivers it: residual error of the lever-arm correction (m) and the scale of the measured
  # noise of the fitted curvature (0.12e-3, tau 0.35 s) and heading (1.5 mrad, tau 0.55 s); FIT_SEED seeds it
  FIT_ARM_ERR_M=0.0, FIT_NOISE=0.0, FIT_SEED=0,
)
DEFAULTS = dict(HEAD, **CHAIN)
ACTUATOR_ERROR = dict(COMPLIANCE=0.39)          # fitted 0.40 deg / (m/s^2) on b4-b6 (report 121.2), 0.39 used in 121.4

# ---------------------------------------------------------------------------------------------------------------------
# the driving model as a feedback law (README, calibration table). ENSEMBLE entries override MODEL.
MODEL = dict(
  b_psi=0.050,                                    # 1/m per rad of heading error (a1 0.048 +- 0.002, a7 0.046 +- 0.004)
  BE_KPH=(37.0, 52.0, 67.0, 85.0), BE=(1.20e-3, 0.75e-3, 0.62e-3, 0.54e-3),   # 1/m per m of offset, by speed
  be_scale=1.0,
  a_ff=1.0,                                       # feed-forward gain on the road curvature
  T_ff=0.55,                                      # s: the action follows the road curvature this far AHEAD of the car
  ff_smooth_s=0.5,                                # the model sees a smoothed road
  frame_age_s=0.10,                               # age of the car state the action is based on (frame + inference)
  c_in=0.20, k_in=1.5e-3, pin_lead_s=1.0,         # inside preference in curves: p* shift = c_in * tanh(k / k_in), m
  be_curve=2.0,                                   # b_e multiplier deep in a curve (1 + (be_curve - 1) |tanh(k / k_in)|)
  plan_wn=0.45,                                   # rad/s: the plan converges to p* like (1 + w t) exp(-w t)
  NACT_KPH=(37.0, 52.0, 67.0, 85.0), NACT=(0.20e-3, 0.14e-3, 0.11e-3, 0.085e-3),   # action noise std (pink 0.02-8 Hz)
  nact_scale=1.0,
  n_plan_std=0.05, n_plan_tau=0.7,                # plan(L) - centre(L) noise: AR(1), m / s
  n_lane_slow=0.015, n_lane_mid=0.025, n_lane_fast=0.008,   # lane-centre wander as the model sees it, m: slow (tau 10 s),
                                                  # 0.08-0.45 Hz (lane geometry: the lines move, the car does not), 0.6-5 Hz
  d_wander=0.04e-3, d_wander_tau=15.0,            # slow wander of the curvature bias (crown, roll estimate, offsets)
)
ENSEMBLE = dict(
  nominal={},
  stiff=dict(be_scale=1.5, b_psi=0.060),          # upper edge of the fits (curves included: b_e 0.7-1.0e-3, b_psi 0.06)
  soft=dict(be_scale=0.6, b_psi=0.042),           # lower edge (straights only, a7 75+: 0.040; b_e - 2 sd)
  lagged=dict(frame_age_s=0.20, T_ff=0.40),       # the model reacts 0.1 s later than assumed
  eager=dict(T_ff=0.65, c_in=0.26),               # turns in earlier and prefers the inside more
  curvehold=dict(be_curve=3.5, c_in=0.24),        # holds its inside line harder against the lane-centre term (a7 curves)
  quiet=dict(nact_scale=0.6),                     # less action noise (motorway-like)
)
# what differs between the two drives: steering bias d0 (curvature the car makes with a zero command; yaw rate / v minus
# the command on straights) and the model's preferred position on straights p0
DRIVES = dict(
  a1=dict(d0=-0.060e-3, p0=-0.08),
  a7=dict(d0=+0.14e-3, p0=0.04),
  none=dict(d0=0.0, p0=0.0),
)


def params(**kw):
  P = dict(DEFAULTS)
  for k, v in kw.items():
    if k not in P:
      raise KeyError(f"unknown parameter {k}")
    P[k] = v
  return P


# Parameter sets of earlier builds (what the routes were driven with). Each lists every lane constant that differs from HEAD.
_NO_NEW = dict(LANE_FIT_GAIN=0.0, LANE_AVOID_RELEASE=False)
# Phase 79 (routes a8 / a9, 19454511): offset part KE 0.3e-3 at 60-80 km/h, no lane fit, no avoidance release
P_P79 = dict(_NO_NEW, LANE_CENTER_KE=(0.0, 0.3e-3, 0.3e-3, 0.15e-3))
# Phase 77 (the scratch simulator's old defaults): KE 0.5 / 0.8e-3 on three speed points
P_P77 = dict(_NO_NEW, LANE_CENTER_KE_SPEED_MS=(45.0 / 3.6, 60.0 / 3.6, 80.0 / 3.6), LANE_CENTER_KE=(0.0, 0.5e-3, 0.8e-3))
# Phases 82-87 (routes aa-b6): as HEAD but the lane fit came in over 45-60 km/h (Phase 88: 40-55)
P_P84 = dict(LANE_FIT_SPEED_MS=(45.0 / 3.6, 60.0 / 3.6))
# the constants a7 was driven with (Phase 73c / 75): no KE, no entry lead
P_A7 = dict(_NO_NEW, LANE_CENTER_KI=0.03e-3, LANE_CENTER_I_CAP=0.15e-3, LANE_CENTER_KE=(0.0, 0.0, 0.0, 0.0), ENTRY_LEAD_S=0.0)
# a1 (Phase 72): no lane-centre term at all
P_A1 = dict(_NO_NEW, LANE_CENTER_WEIGHT=0.0, LANE_CENTER_KI=0.0, LANE_CENTER_KE=(0.0, 0.0, 0.0, 0.0), ENTRY_LEAD_S=0.0)


def offset_drive(off_deg, v_kph, P=None, p0=0.0):
  """Drive dict (for simulate(drive=...)) of a steering-angle offset of off_deg (the sensor reads this much left of the
  request on straights, as b4 +0.29 / b5 -0.08 / b6 +0.16 deg): a curvature bias at speed v_kph."""
  P = DEFAULTS if P is None else P
  return dict(d0=-off_deg / _interp(v_kph, P["DEG_PER_K_KPH"], P["DEG_PER_K"]) * 1e-3, p0=p0)


def model_params(ens="nominal", drive="none", **kw):
  M = dict(MODEL)
  M.update(ENSEMBLE[ens] if isinstance(ens, str) else ens)
  M.update(DRIVES[drive] if isinstance(drive, str) else drive)
  M.update(kw)
  return M


# ---------------------------------------------------------------------------------------------------------------------
def _interp(x, xp, fp):
  if x <= xp[0]:
    return fp[0]
  for k in range(1, len(xp)):
    if x <= xp[k]:
      return fp[k - 1] + (fp[k] - fp[k - 1]) * (x - xp[k - 1]) / (xp[k] - xp[k - 1])
  return fp[-1]


def _box(x, n):
  n = int(n)
  if n <= 1:
    return np.asarray(x, float)
  xp = np.r_[np.full(n // 2, x[0]), x, np.full(n - n // 2 - 1, x[-1])]
  return np.convolve(xp, np.ones(n) / n, 'valid')


def _shift(x, n):
  """x read n samples ahead (n > 0) or behind, ends held."""
  n = int(n)
  if n == 0:
    return x
  return np.r_[x[n:], np.full(n, x[-1])] if n > 0 else np.r_[np.full(-n, x[0]), x[:n]]


def _pink(n, rng, f_lo, f_hi, slope=1.0):
  """unit-variance noise with power ~ 1 / f^slope between f_lo and f_hi (Hz), 100 Hz."""
  m = max(int(2 ** math.ceil(math.log2(max(n, 16)))), 4096)
  f = np.fft.rfftfreq(m, DT)
  a = np.zeros(len(f)); k = (f >= f_lo) & (f <= f_hi)
  a[k] = f[k] ** (-slope / 2.0)
  x = np.fft.irfft(a * (rng.standard_normal(len(f)) + 1j * rng.standard_normal(len(f))), m)[:n]
  return x / (math.sqrt(np.sum(a ** 2) * 2) / m)


def _ar1(n, rng, tau, step=1):
  a = math.exp(-DT * step / tau); s = math.sqrt(1 - a * a)
  w = rng.standard_normal(n // step + 2); x = np.empty(len(w)); x[0] = w[0]
  for i in range(1, len(w)):
    x[i] = a * x[i - 1] + s * w[i]
  return np.repeat(x, step)[:n]


class Obs:
  """What a controller sees each 100 Hz frame (model values are held between the 20 Hz model frames).
    t, v            time (s), speed (m/s)
    new_frame       True on the frame a new model output arrived
    k_action        the model's action (desiredCurvature), 1/m
    k_lead          the action ENTRY_LEAD_S later minus the action now (controlsd's d in _entry_lead)
    c0              lane centre at the car in the car frame, m (+ = centre to the right = car LEFT of centre); model lines
    cL_minus_plan   centre(L) - plan(L) at L (m) - what the Phase 73c proportional part uses
    L               preview distance used for cL_minus_plan = clip(v * LANE_CENTER_T_S, LANE_CENTER_L_M)
    psi_lane        lane heading relative to the car (rad, + = lane bends away to the right) = -psi
    k_lane          road curvature the model's lane lines show under the car (smoothed road, no noise), 1/m
    k_lane_ahead    tuple: the same 0.5 / 1.0 / 2.0 s ahead
    lane_prob       min inner lane-line probability (scenario)
    usable          lane-centre context usable (no blinker / press / dropout); scenario
    k_meas          curvature from the steering-angle sensor as openpilot computes it (contains the steering bias d)
    wheel_deg       wheel angle in curvature-sign degrees
    k_prev          the previous frame's final command
    plan_off        plan(L) - centre(L), m (Phase 83 watches it against its usual value); = -cL_minus_plan
  """
  __slots__ = ("t", "v", "new_frame", "k_action", "k_lead", "c0", "cL_minus_plan", "L", "psi_lane", "k_lane",
               "k_lane_ahead", "lane_prob", "usable", "k_meas", "wheel_deg", "k_prev", "plan_off")


def fit_lead_s(v, P):
  """How far ahead of the car (s) the curvature of the Phase 82 quadratic sits: half its range clip(v * RANGE_S, RANGE_M)."""
  return min(max(v * P["LANE_FIT_RANGE_S"], P["LANE_FIT_RANGE_M"][0]), P["LANE_FIT_RANGE_M"][1]) / (2.0 * max(v, 1.0))


def head_controller(P):
  """controlsd direct path at HEAD: action + _lane_center (P / I / E parts, lane fit of Phases 82 / 88, avoidance release of
  Phases 83 / 84) + _entry_lead (Phase 78), in controlsd's order of operations. Not ported: the lane-width checks and the
  non-finite guards (the simulated lane is always readable), LANE_STATE_RESET on engage (a run is one engagement).
  step(o) -> (command, (P, I, E, entry lead, fit, avoidance weight))."""
  W = P["LANE_CENTER_WEIGHT"]; KI = P["LANE_CENTER_KI"]; ICAP = P["LANE_CENTER_I_CAP"]; IMAXK = P["LANE_CENTER_I_MAX_K"]
  BLEED = P["LANE_CENTER_I_BLEED_S"]; BIAS = P["LANE_CENTER_BIAS_M"]; ACC = P["LANE_CENTER_ACC"]; TAU = P["LANE_CENTER_TAU_S"]
  PROB = P["LANE_CENTER_PROB"]; SPD = P["LANE_CENTER_SPEED_MS"]; KES = P["LANE_CENTER_KE_SPEED_MS"]; KEV = P["LANE_CENTER_KE"]
  ELS = P["ENTRY_LEAD_S"]; ELCAP = P["ENTRY_LEAD_CAP"]; ELK = P["ENTRY_LEAD_K"]; ELWO = P["ENTRY_LEAD_WASHOUT_S"]
  ELMIN = P["ENTRY_LEAD_MIN_MS"]; ELTAU = P["ENTRY_LEAD_TAU_S"]
  FG = P["LANE_FIT_GAIN"]; FSPD = P["LANE_FIT_SPEED_MS"]; FT = P["LANE_FIT_T_S"]; DA = P["FIT_ARM_ERR_M"]; NZ = P["FIT_NOISE"]
  AV = bool(P["LANE_AVOID_RELEASE"]); ADEV = P["LANE_AVOID_DEV_M"]; ACRV = P["LANE_AVOID_CURVE"]; AUS = P["LANE_AVOID_USUAL_S"]
  AREC = P["LANE_AVOID_RECOVER_S"]; ARST = P["LANE_AVOID_RESET_S"]; AWARM = P["LANE_AVOID_WARMUP_S"]
  a_lp = DT / (TAU + DT); a_wo = DT / (ELWO + DT); a_el = DT / (ELTAU + DT); a_rec = DT / (AREC + DT)
  s = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]     # p, integ, pe, entry-lead lp, washout, fit
  av = [None, 1.0, 0, 0]                 # usual, wa, gap, n_us
  nz = [0.0, 0.0]; rng = np.random.default_rng(P["FIT_SEED"])

  def step(o):
    v = max(o.v, 1.0); w = 0.0; raw = 0.0; e0 = 0.0; raw_e = 0.0; raw_f = 0.0; seen = False; w_i = 0.0
    usual, wa, gap, n_us = av
    if o.new_frame and NZ > 0.0:         # camera fit noise: AR(1) at the model rate
      nz[0] += 0.05 / 0.35 * (-nz[0]) + rng.normal() * 0.12e-3 * NZ * math.sqrt(2 * 0.05 / 0.35)
      nz[1] += 0.05 / 0.55 * (-nz[1]) + rng.normal() * 1.5e-3 * NZ * math.sqrt(2 * 0.05 / 0.55)
    if o.usable and W > 0.0:
      dist = o.L
      w = _interp(o.lane_prob, PROB, (0.0, 1.0)) * _interp(v, SPD, (0.0, 1.0))
      if AV and w > 0.0:                                                         # Phase 83
        off = o.plan_off
        if usual is None:
          usual = off; n_us = 0
        warm = n_us * DT >= AWARM
        dev = abs(off - usual) - ACRV * abs(o.k_action) * dist * dist / 2.0
        wa_t = _interp(dev, ADEV, (1.0, 0.0)) if warm else 1.0
        wa = wa_t if wa_t < wa else wa + a_rec * (wa_t - wa)
        a_us = DT / (AUS * (1.0 if wa >= 1.0 - 1e-6 else 2.0) + DT)
        usual += max(a_us, 1.0 / (n_us + 1.0)) * (off - usual)
        n_us = min(n_us + 1, 10 ** 6)
        seen = True
        w *= wa
        w_i = w if wa >= 0.5 else 0.0
      else:
        w_i = w
      cap = ACC / (v * v)
      raw = W * w * 2.0 * (o.cL_minus_plan + BIAS) / (dist * dist)
      raw = -cap if raw < -cap else (cap if raw > cap else raw)
      e0 = o.c0 + BIAS
      ke = _interp(v, KES, KEV)
      if ke > 0.0:
        raw_e = ke * w * e0
        lo, hi = -cap - raw, cap - raw
        raw_e = lo if raw_e < lo else (hi if raw_e > hi else raw_e)
      wf = w * _interp(v, FSPD, (0.0, 1.0)) if FG > 0.0 else 0.0
      if wf > 0.0:                                                               # Phase 82 / 88
        tl = fit_lead_s(v, P)            # the quadratic's curvature is the lane's about half its range ahead
        f = min(max((tl - 0.5) / 0.5, 0.0), 1.0)
        k_l = (1.0 - f) * o.k_lane_ahead[0] + f * o.k_lane_ahead[1] + nz[0]
        psi_c = o.psi_lane + nz[1] + DA * k_l          # heading at the car = controlsd's (psi_l - LANE_FIT_ARM_M * k_l)
        lf = max(v * FT, 10.0)
        fit = 1.432 * e0 / (lf * lf) + 1.653 * psi_c / lf + k_l
        raw_f = FG * wf * (fit - o.k_action)
        lo, hi = -cap - raw - raw_e, cap - raw - raw_e
        raw_f = lo if raw_f < lo else (hi if raw_f > hi else raw_f)
    s[0] += a_lp * (raw - s[0]); s[2] += a_lp * (raw_e - s[2]); s[5] += a_lp * (raw_f - s[5])
    if w_i > 0.0:
      if abs(o.k_action) < IMAXK and KI > 0.0:
        x = s[1] + KI * w_i * e0 * DT
        s[1] = -ICAP if x < -ICAP else (ICAP if x > ICAP else x)
    elif not seen or wa >= 0.5:                        # released for an avoidance: held, not bled
      s[1] *= max(1.0 - DT / BLEED, 0.0)
    gap = 0 if seen else gap + 1
    if gap * DT >= ARST:
      usual = None
    if not seen:
      wa += a_rec * (1.0 - wa)
    av[0], av[1], av[2], av[3] = usual, wa, gap, n_us
    # Phase 78
    rawl = 0.0
    if o.usable and ELS > 0.0 and o.v >= ELMIN:
      d = o.k_lead; kf = o.k_action + d
      if d * kf > 0.0 and abs(kf) > abs(o.k_action):
        rawl = _interp(abs(kf), ELK, (0.0, 1.0)) * d
        rawl = -ELCAP if rawl < -ELCAP else (ELCAP if rawl > ELCAP else rawl)
    s[4] += a_wo * (rawl - s[4])
    pulse = rawl - s[4]
    if pulse * rawl <= 0.0:
      pulse = 0.0
    s[3] += a_el * (pulse - s[3])
    return o.k_action + s[0] + s[1] + s[2] + s[5] + s[3], (s[0], s[1], s[2], s[3], s[5], wa)

  step.state = s            # [P, I, E, entry lead, washout, fit] - may be preset (e.g. the integral at a segment start)
  step.avoid = av           # [usual, weight, gap frames, frames averaged]
  return step


# ---------------------------------------------------------------------------------------------------------------------
def simulate(scn, P=None, controller=None, ens="nominal", drive="none", seed=0, noise=True, model=None, init=None):
  """Run one scenario. scn: dict from synthetic() / log_scenarios() (keys v, k_road [, prob, usable, lane_shift, d,
  e0, psi0]). P: params(...) dict (controller + chain constants). controller: factory(P) -> step(o) -> k_cmd or
  (k_cmd, parts); default head_controller. ens / drive / model: the driving-model variant (ENSEMBLE, DRIVES, overrides).
  noise=False: no action / lane / plan noise and no bias wander. init: dict(I=..) presets the HEAD integral.
  Returns dict of 100 Hz arrays: t, v, k_road, e (true position), e_meas (model-line position, what the logs' lane
  position is), psi, k_action, k_cmd, k_term (= k_cmd - k_action), k_car, wheel (deg), req (deg, the transmitted
  request), parts (n x 6: P, I, E, entry lead, fit, avoidance weight for the HEAD controller)."""
  P = dict(DEFAULTS) if P is None else P
  M = model_params(ens, drive, **(model or {}))
  V = np.asarray(scn["v"], float); KR = np.asarray(scn["k_road"], float); n = len(V)
  rng = np.random.default_rng(seed)
  prob = np.asarray(scn.get("prob", np.ones(n)), float); usable = np.asarray(scn.get("usable", np.ones(n, bool)), bool)
  shift = np.asarray(scn.get("lane_shift", np.zeros(n)), float)
  vk = V * 3.6
  BE = np.interp(vk, M["BE_KPH"], M["BE"]) * M["be_scale"]
  DEGK = np.interp(vk, P["DEG_PER_K_KPH"], P["DEG_PER_K"])
  HYS = np.interp(V, P["HYST_SPEED_MS"], P["HYST_DEG"])
  GAINV = 1.0 - P["COMPLIANCE"] * V * V / (1e3 * DEGK); GVl = GAINV.tolist()   # delivered share under lateral load
  KS = _box(KR, M["ff_smooth_s"] / DT)
  n_age = int(round(M["frame_age_s"] / DT))
  KFF = M["a_ff"] * _shift(KS, int(round(M["T_ff"] / DT)))
  PIN = np.tanh(_shift(_box(KR, 100), int(round(M["pin_lead_s"] / DT))) / M["k_in"])
  BE = BE * (1.0 + (M["be_curve"] - 1.0) * np.abs(PIN))
  PIN = M["c_in"] * PIN + M.get("c_left", 0.0) * np.abs(PIN)      # c_left: same-side shift in both curve directions (m)
  n_lead = int(round(P["ENTRY_LEAD_S"] / DT))
  KLEAD = _shift(KFF, n_lead) - KFF
  KA = (_shift(KS, 50), _shift(KS, 100), _shift(KS, 200))
  D = np.full(n, float(M.get("d0", 0.0))) + np.asarray(scn.get("d", 0.0), float)
  if noise:
    NACT = _pink(n, rng, 0.02, 5.0) * np.interp(vk, M["NACT_KPH"], M["NACT"]) * M["nact_scale"]
    NPLAN = _ar1(n, rng, M["n_plan_tau"], 5) * M["n_plan_std"]
    NLANE = (_ar1(n, rng, 10.0, 5) * M["n_lane_slow"] + _pink(n, rng, 0.08, 0.45, 0.0) * M["n_lane_mid"]
             + _pink(n, rng, 0.6, 5.0, 0.0) * M["n_lane_fast"])
    D = D + _ar1(n, rng, M["d_wander_tau"], 5) * M["d_wander"]
  else:
    NACT = NPLAN = NLANE = np.zeros(n)
  Vl = V.tolist(); KRl = KR.tolist(); BEl = BE.tolist(); DEGl = DEGK.tolist(); HYl = HYS.tolist(); KFFl = KFF.tolist()
  PINl = PIN.tolist(); KLl = KLEAD.tolist(); Dl = D.tolist(); NAl = NACT.tolist(); NPl = NPLAN.tolist(); NLl = NLANE.tolist()
  KSl = KS.tolist(); KA0, KA1, KA2 = (a.tolist() for a in KA); probl = prob.tolist(); usl = usable.tolist(); shl = shift.tolist()
  ctrl = (controller or head_controller)(P)
  if init and hasattr(ctrl, "state") and "I" in init:
    ctrl.state[1] = float(init["I"])
  b_psi = M["b_psi"]; p0 = M.get("p0", 0.0); wn = M["plan_wn"]
  TS = P["LANE_CENTER_T_S"]; Lmin, Lmax = P["LANE_CENTER_L_M"]
  lin = bool(P["LINEAR"]); quant = P["QUANT_DEG"]; cj = P["CURV_JERK"]; txj = P["TX_JERK"]; gain = P["ACT_GAIN"]
  n_del = max(int(round((P["ACT_DELAY_S"] + P["EXTRA_DELAY_S"]) / DT)), 0)
  a_w = DT / (P["WHEEL_TAU_S"] + DT); a_v = DT / (P["VEH_TAU_S"] + DT)
  e = float(scn.get("e0", p0)); psi = float(scn.get("psi0", 0.0))
  k0 = KRl[0] - Dl[0]                       # start in equilibrium with the road
  buf = [k0 * DEGl[0]] * (n_del + 1); bi = 0
  wheel = k0 * DEGl[0]; kveh = k0; hyst = wheel; req = wheel; kprev = k0
  E = [0.0] * n; EM = [0.0] * n; PS = [0.0] * n; KM = [0.0] * n; KC = [0.0] * n; KCAR = [0.0] * n; WH = [0.0] * n; RQ = [0.0] * n
  PARTS = [None] * n
  o = Obs(); o.k_action = k0; o.k_lead = 0.0; o.c0 = -(e - shl[0]); o.cL_minus_plan = 0.0; o.plan_off = 0.0; o.L = 30.0; o.psi_lane = -psi
  o.k_prev = k0; o.k_lane = KSl[0]; o.k_lane_ahead = (KA0[0], KA1[0], KA2[0])
  em = e - shl[0]; kmodel = k0
  exp = math.exp
  for i in range(n):
    v = Vl[i]
    if v < 1.0:
      v = 1.0
    nf = (i % 5 == 0)
    if nf:
      j = i - n_age
      if j < 0:
        j = 0
      em = E[j] - shl[j] + NLl[i] if i > 0 else e - shl[0] + NLl[0]
      pm = PS[j] if i > 0 else psi
      pst = p0 + PINl[j]
      kmodel = KFFl[j] - b_psi * pm - BEl[i] * (em - pst) + NAl[i]
      L = v * TS
      L = Lmin if L < Lmin else (Lmax if L > Lmax else L)
      tL = L / v; ex = exp(-wn * tL)
      plan_rel = pst + (em - pst) * (1.0 + wn * tL) * ex + pm * L * ex + NPl[i]
      o.k_action = kmodel; o.k_lead = KLl[j]; o.c0 = -em; o.cL_minus_plan = -plan_rel; o.plan_off = plan_rel; o.L = L; o.psi_lane = -pm
      o.k_lane = KSl[j]; o.k_lane_ahead = (KA0[j], KA1[j], KA2[j])
    o.t = i * DT; o.v = v; o.new_frame = nf; o.lane_prob = probl[i]; o.usable = usl[i]
    o.k_meas = gain * GVl[i] * kveh + Dl[i]; o.wheel_deg = wheel; o.k_prev = kprev
    out = ctrl(o)
    if isinstance(out, tuple):
      kc, PARTS[i] = out
    else:
      kc = out
    dk = DEGl[i]
    if not lin:
      lim = cj / (v * v) * DT
      if kc > kprev + lim:
        kc = kprev + lim
      elif kc < kprev - lim:
        kc = kprev - lim
    kprev = kc
    ang = kc * 1e3 * dk
    if not lin:
      h = HYl[i]
      if hyst < ang - h:
        hyst = ang - h
      elif hyst > ang + h:
        hyst = ang + h
      a_q = round(hyst / quant) * quant if quant > 0.0 else hyst
      rl = txj / (v * v) * 1e3 * dk * DT
      if a_q > req + rl:
        req = req + rl
      elif a_q < req - rl:
        req = req - rl
      else:
        req = a_q
    else:
      req = ang
    buf[bi] = req; bi += 1
    if bi > n_del:
      bi = 0
    wheel += a_w * (buf[bi] - wheel)
    kveh += a_v * (wheel / dk * 1e-3 - kveh)
    kcar = gain * GVl[i] * kveh + Dl[i]
    psi += v * (kcar - KRl[i]) * DT
    e += v * psi * DT
    E[i] = e; EM[i] = em; PS[i] = psi; KM[i] = kmodel; KC[i] = kc; KCAR[i] = kcar; WH[i] = wheel; RQ[i] = req
  res = dict(t=np.arange(n) * DT, v=V, k_road=KR, e=np.array(E) - shift, e_meas=np.array(EM), psi=np.array(PS), k_action=np.array(KM),
             k_cmd=np.array(KC), k_car=np.array(KCAR), wheel=np.array(WH), req=np.array(RQ), name=scn.get("name", ""),
             events=scn.get("events", {}), prob=prob)
  res["k_term"] = res["k_cmd"] - res["k_action"]
  if PARTS[0] is not None:
    res["parts"] = np.array(PARTS)
  return res


# ---------------------------------------------------------------------------------------------------------------------
def synthetic(kind, v_kph=60.0, T=None, **kw):
  """Synthetic scenarios (constant speed). kind:
    'straight'   crown / steering bias as a curvature offset: d (default +0.12e-3, a7-like; the DRIVES d0 adds to it)
    'step'       the lane centre jumps by 'step' m (default 0.20) after the pre-roll   -> overshoot / settling
    (step / curve / s_curve / dropout start with a pre-roll 'pre_s' (default 60 s) of straight so that the integral and
     the car have settled before the event)
    'curve'      pre-roll, ramp of 'ramp_s' (default 2.5 s, measured entries 2-3 s) to k (default 2.5e-3, sign =
                 direction: + right), hold 'hold_s' (default 12 s), ramp out, 15 s straight
    's_curve'    right then left curve of +-k with ramps, 6 s holds
    'dropout'    straight then curve; lane lines unusable (prob 0) for 'drop_s' (default 3 s) from 'drop_t' (default 1 s into the entry)
  """
  v = v_kph / 3.6
  if kind == "straight":
    T = T or 120.0; n = int(T / DT); k = np.zeros(n); scn = dict(d=np.full(n, kw.get("d", 0.12e-3)))
  elif kind == "step":
    t0 = kw.get("pre_s", 60.0); T = T or t0 + 60.0; n = int(T / DT); k = np.zeros(n); sh = np.zeros(n); sh[int(t0 / DT):] = kw.get("step", 0.20)
    scn = dict(lane_shift=sh, events=dict(step_t=t0, step=kw.get("step", 0.20)))
  elif kind in ("curve", "dropout"):
    kk = kw.get("k", 2.5e-3); r = kw.get("ramp_s", 2.5); h = kw.get("hold_s", 12.0); t0 = kw.get("pre_s", 60.0)
    T = T or (t0 + 2 * r + h + 15.0); n = int(T / DT); t = np.arange(n) * DT
    k = kk * np.clip((t - t0) / r, 0, 1) * np.clip((t0 + 2 * r + h - t) / r, 0, 1)
    scn = dict(events=dict(entry_t=t0, exit_t=t0 + r + h, k=kk, ramp_s=r))
    if kind == "dropout":
      p = np.ones(n); a = kw.get("drop_t", t0 + 1.0); p[int(a / DT):int((a + kw.get("drop_s", 3.0)) / DT)] = 0.0; scn["prob"] = p
  elif kind == "s_curve":
    kk = kw.get("k", 2.0e-3); r = kw.get("ramp_s", 2.5); h = kw.get("hold_s", 6.0); t0 = kw.get("pre_s", 60.0)
    T = T or (t0 + 4 * r + 2 * h + 15.0); n = int(T / DT); t = np.arange(n) * DT
    up = np.clip((t - t0) / r, 0, 1); mid = np.clip((t - (t0 + r + h)) / (2 * r), 0, 1); dn = np.clip((t - (t0 + 3 * r + 2 * h)) / r, 0, 1)
    k = kk * (up - 2 * mid + dn)
    scn = dict(events=dict(entry_t=t0, reversal_t=t0 + r + h, exit_t=t0 + 3 * r + 2 * h, k=kk, ramp_s=r))
  else:
    raise KeyError(kind)
  scn.update(name=f"{kind}@{v_kph:.0f}", v=np.full(n, v), k_road=k)
  return scn


_LOGS = None
SCEN_LOGS = os.environ.get("ROADSIM_SCEN_LOGS", os.path.join(HERE, "scen_logs.npz"))   # 5.9 MB, not in the repository (README)


def log_scenarios(drive=None, min_s=12.0):
  """The hands-off segments of a1 (23, 971 s) and a7 (6, 223 s) as scenarios: real speed and road-curvature traces,
  the lane probability, the logged start state (e0, psi0) and, under 'log', what the car really did (off = model-line
  position, K action, k_ctrl, mdps_ang, k_meas, good mask, term = k_ctrl - k_model, rawP, plan_rel)."""
  global _LOGS
  if _LOGS is None:
    if not os.path.exists(SCEN_LOGS):
      raise FileNotFoundError(f"{SCEN_LOGS}: the a1 / a7 log scenarios are not in the repository - set ROADSIM_SCEN_LOGS (README)")
    z = dict(np.load(SCEN_LOGS, allow_pickle=True))
    cols = [str(c) for c in z["cols"]]
    _LOGS = [(str(nm), {c: z[str(nm)][k].astype(float) for k, c in enumerate(cols)}) for nm in z["names"]]
  out = []
  for nm, L in _LOGS:
    if drive and not nm.startswith(drive[-2:]):
      continue
    if len(L["t"]) * DT < min_s:
      continue
    out.append(dict(name=nm, v=L["v"], k_road=L["k_road"], prob=L["prob"], e0=float(L["off"][:10].mean()), psi0=float(L["psi"][:10].mean()), log=L))
  return out


# ---------------------------------------------------------------------------------------------------------------------
BANDS = ((30, 45), (45, 60), (60, 75), (75, 200))


def _hp(x, n):
  return x - _box(x, n)


def masks(k_action):
  """straight / left / right as in probe e169: |action| < 0.5e-3 / < -1e-3 / > 1e-3."""
  return np.abs(k_action) < 0.5e-3, k_action < -1e-3, k_action > 1e-3


def metrics(res, skip_s=3.0, target=-0.034, pos="e_meas"):
  """Metrics of one run or of a list of runs (pooled by speed band). pos: 'e_meas' (model-line position, comparable with
  the report's numbers) or 'e' (true position). Returns {band: {...}} with cm / deg / deg/s / m/s^3:
    straight_mean, straight_std, straight_s, left_mean, right_mean, inside_cut (= (right - left) / 2), curve_s,
    wheel_hp_std (3 s high-pass, straights), wheel_rate_rms (straights), jerk_rms (all), term_mean (1e-3, straights)"""
  runs_ = res if isinstance(res, (list, tuple)) else [res]
  acc = {b: {k: [] for k in ("st", "lc", "rc", "whp", "wr", "jerk", "term")} for b in BANDS}
  for r in runs_:
    s = int(skip_s / DT); n = len(r["t"])
    if n <= s + 100:
      continue
    x = r[pos] if not isinstance(pos, str) else r[pos]
    st, lc, rc = masks(r["k_action"] if "class_k" not in r else r["class_k"])
    whp = _hp(r["wheel"], 300); wr = np.gradient(r["wheel"], DT)
    jerk = np.gradient((r["v"] ** 2) * r["k_car"], DT) if "k_car" in r else np.zeros(n)
    jerk = _box(jerk, 20)
    ok = np.ones(n, bool); ok[:s] = False
    if "good" in r:
      ok &= r["good"] > 0.5
    vk = r["v"] * 3.6
    for b in BANDS:
      m = ok & (vk >= b[0]) & (vk < b[1]); a = acc[b]
      a["st"].append(x[m & st]); a["lc"].append(x[m & lc]); a["rc"].append(x[m & rc]); a["whp"].append(whp[m & st]); a["wr"].append(wr[m & st])
      a["jerk"].append(jerk[m]); a["term"].append(r["k_term"][m & st] if "k_term" in r else np.zeros(0))
  out = {}
  for b in BANDS:
    a = {k: np.concatenate(v) if v else np.zeros(0) for k, v in acc[b].items()}
    if len(a["st"]) < 500 and len(a["lc"]) + len(a["rc"]) < 300:
      continue
    o = dict(straight_s=len(a["st"]) * DT, curve_s=(len(a["lc"]) + len(a["rc"])) * DT)
    if len(a["st"]) >= 500:
      o.update(straight_mean=a["st"].mean() * 100, straight_std=a["st"].std() * 100, wheel_hp_std=a["whp"].std(),
               wheel_rate_rms=float(np.sqrt(np.mean(a["wr"] ** 2))), term_mean=a["term"].mean() * 1e3 if len(a["term"]) else 0.0)
    if len(a["lc"]) > 150:
      o["left_mean"] = a["lc"].mean() * 100
    if len(a["rc"]) > 150:
      o["right_mean"] = a["rc"].mean() * 100
    if "left_mean" in o and "right_mean" in o:
      o["inside_cut"] = (o["right_mean"] - o["left_mean"]) / 2
    if len(a["jerk"]):
      o["jerk_rms"] = float(np.sqrt(np.mean(a["jerk"] ** 2)))
    out[b] = o
  return out


def event_metrics(res, target=-0.034, pos="e"):
  """For synthetic scenarios (true position, cm, relative to the position just before the event):
    step:   overshoot_pct, settle_s (last time outside +-2 cm of the final value), final_cm (vs the new centre + target)
    curve:  entry_1s / entry_2s (offset change 1 / 2 s after the curve starts; + = to the OUTSIDE), curve_inside_cm (mean
            over the hold, + = inside, relative to the straight before), peak_inside_cm, exit_2s (outside +), exit_peak_cm,
            wheel_rate_max (deg/s over the entry), jerk_rms_entry (m/s^3)"""
  ev = res["events"]; x = res[pos]; t = res["t"]; o = {}
  ix = lambda tt: min(int(round(tt / DT)), len(t) - 1)   # noqa: E731
  if "step_t" in ev:
    i0 = ix(ev["step_t"]); before = x[i0 - 200:i0].mean() + ev["step"]; y = x[i0:]
    # the lane moved by step: the car (relative to the new centre) starts at before - step and has to come back to 'final'
    final = y[-500:].mean(); start = y[0]
    span = final - start
    o["overshoot_pct"] = float(max((y - final).max() if span > 0 else (final - y).max(), 0.0) / abs(span) * 100) if abs(span) > 1e-3 else float("nan")
    out_ = np.flatnonzero(np.abs(y - final) > 0.02)
    o["settle_s"] = float((out_[-1] + 1) * DT) if len(out_) else 0.0
    t90 = np.flatnonzero(np.abs(y - start) >= 0.9 * abs(span))
    o["rise90_s"] = float(t90[0] * DT) if len(t90) else float("nan")
    o["final_cm"] = float((final - target) * 100)
  if "entry_t" in ev:
    sg = 1.0 if ev["k"] > 0 else -1.0; i0 = ix(ev["entry_t"]); base = x[i0 - 300:i0].mean()
    rel = (x - base) * sg * 100            # + = toward the inside of the curve
    i1 = ix(ev["exit_t"]); r = ev["ramp_s"]
    o["entry_1s"] = float(-rel[ix(ev["entry_t"] + 1.0)]); o["entry_2s"] = float(-rel[ix(ev["entry_t"] + 2.0)])
    if "reversal_t" not in ev:
      hold = slice(ix(ev["entry_t"] + r + 1.0), i1)
      o["curve_inside_cm"] = float(rel[hold].mean()); o["peak_inside_cm"] = float(rel[i0:i1].max()); o["peak_outside_cm"] = float(-rel[i0:i1].min())
      o["exit_2s"] = float(-rel[ix(ev["exit_t"] + 2.0)]); o["exit_peak_out_cm"] = float(-rel[i1:i1 + 800].min())
      o["abs_inside_cm"] = float((x[hold].mean() - target) * sg * 100)
    else:
      o["peak_inside_first_cm"] = float(rel[i0:ix(ev["reversal_t"])].max()); o["peak_swing_cm"] = float(rel[i0:].max() - rel[i0:].min())
    ent = slice(i0, ix(ev["entry_t"] + r + 1.5))
    o["wheel_rate_max"] = float(np.abs(_box(np.gradient(res["wheel"], DT), 10))[ent].max())
    o["jerk_rms_entry"] = float(np.sqrt(np.mean(_box(np.gradient(res["v"] ** 2 * res["k_car"], DT), 20)[ent] ** 2)))
  return o


def _oscillates(v_kph, P, controller, ens, drive, model, T=90.0):
  n = int(T / DT)
  scn = dict(name="margin", v=np.full(n, v_kph / 3.6), k_road=np.zeros(n), e0=0.25 + DRIVES.get(drive, {}).get("p0", 0.0) if isinstance(drive, str) else 0.25)
  r = simulate(scn, P=P, controller=controller, ens=ens, drive=drive, noise=False, model=model)
  x = r["e"][-2500:]; y = r["e"][-6000:-3500]
  return (np.ptp(x) > 0.04 and np.ptp(x) > 0.7 * np.ptp(y)) or not np.isfinite(x).all() or np.abs(x).max() > 3.0


def margins(v_kph=60.0, P=None, controller=None, ens="nominal", drive="none", model=None, gmax=8.0, dmax=1.2):
  """Gain margin: factor on the delivered curvature (ACT_GAIN: scales the whole loop - model and controller) at which a
  sustained oscillation appears. Delay margin: extra transport delay (s) at which it appears. Linear chain (no backlash /
  quantisation / rate limit), no noise, start 25 cm off. Bisection; (gmax / dmax returned when never unstable)."""
  P = dict(DEFAULTS) if P is None else dict(P)
  P["LINEAR"] = True
  def bis(key, lo, hi, tol):
    base = P[key]
    def bad(x):
      Q = dict(P); Q[key] = base * x if key == "ACT_GAIN" else base + x
      return _oscillates(v_kph, Q, controller, ens, drive, model)
    if not bad(hi):
      return hi
    while hi - lo > tol:
      mid = 0.5 * (lo + hi)
      if bad(mid):
        hi = mid
      else:
        lo = mid
    return 0.5 * (lo + hi)
  return dict(gain_margin=bis("ACT_GAIN", 1.0, gmax, 0.1), delay_margin_s=bis("ACT_DELAY_S", 0.0, dmax, 0.02))


def limit_cycle(v_kph=60.0, P=None, controller=None, ens="nominal", drive="none", model=None, T=180.0):
  """Self-sustained weave with NO noise on a straight (backlash 0.15 deg + 0.1 deg quantisation against the model's weak
  spring and any integral): peak-to-peak (cm) and period (s) of the true position over the last 60 s."""
  n = int(T / DT)
  r = simulate(dict(name="lc", v=np.full(n, v_kph / 3.6), k_road=np.zeros(n)), P=P, controller=controller, ens=ens, drive=drive, noise=False, model=model)
  x = r["e"][-6000:]; y = x - x.mean(); zc = np.flatnonzero((y[:-1] < 0) & (y[1:] >= 0))
  return dict(ptp_cm=float(np.ptp(x) * 100), period_s=float(np.mean(np.diff(zc)) * DT) if len(zc) > 1 else float("nan"),
              mean_cm=float(x.mean() * 100), wheel_ptp_deg=float(np.ptp(r["wheel"][-6000:])))


def scenario_set(v_list=(50.0, 65.0, 85.0)):
  """The standard synthetic set by speed + the log segments. Returns (synthetic list, log list)."""
  syn = []
  for v in v_list:
    syn += [synthetic("straight", v), synthetic("straight", v, d=-0.06e-3), synthetic("step", v),
            synthetic("curve", v, k=2.5e-3), synthetic("curve", v, k=-2.5e-3), synthetic("curve", v, k=1.2e-3, hold_s=8.0),
            synthetic("s_curve", v), synthetic("dropout", v)]
  return syn, log_scenarios()


# ---------------------------------------------------------------------------------------------------------------------
# steady-curve position: the number the logs give most directly (E179 LT-03, report 118 / 119)
STEADY_K = {50: 3.0e-3, 60: 2.5e-3, 70: 1.8e-3, 80: 1.4e-3, 90: 1.1e-3, 100: 0.9e-3}   # typical logged curve by speed (1/m)


def steady_curve(v_kph, P=None, k=None, ens="nominal", drives=("a1", "a7"), hold_s=20.0, noise=False, seed=0, linear=True):
  """Mean steady-curve position (cm, + = inside of the curve, relative to the straight before; true position) of a
  constant-speed curve, averaged over left / right and the drives. Returns dict(inside, left, right, deliv = curvature
  delivered / commanded in the hold, fit = lane-fit part along the curve 1e-3, term = all lane parts along the curve 1e-3)."""
  P = dict(DEFAULTS if P is None else P, LINEAR=linear); kk = k or STEADY_K[int(v_kph)]; out = {1: [], -1: []}; dl = []; ft = []; tm = []
  for dr in drives:
    for sg in (1, -1):
      scn = synthetic("curve", v_kph=v_kph, k=sg * kk, hold_s=hold_s)
      r = simulate(scn, P=P, ens=ens, drive=dr, noise=noise, seed=seed)
      ev = scn["events"]; i0 = int(ev["entry_t"] / DT); h = slice(int((ev["entry_t"] + ev["ramp_s"] + 4.0) / DT), int(ev["exit_t"] / DT))
      out[sg].append((r["e"][h].mean() - r["e"][i0 - 300:i0].mean()) * sg * 100)
      dl.append(r["k_car"][h].mean() / r["k_cmd"][h].mean()); tm.append(r["k_term"][h].mean() * sg * 1e3)
      ft.append(r["parts"][h, 4].mean() * sg * 1e3 if "parts" in r and r["parts"].shape[1] > 4 else 0.0)
  return dict(inside=float(np.mean(out[1] + out[-1])), right=float(np.mean(out[1])), left=float(np.mean(out[-1])),
              deliv=float(np.mean(dl)), fit=float(np.mean(ft)), term=float(np.mean(tm)))


def fidelity_table(speeds=(50, 60, 70, 80, 90, 100)):
  """Rows (v, HEAD, Phase 84 gates, Phase 79, HEAD + actuator error model) of steady_curve()['inside'] - the README table."""
  sets = (("HEAD", params()), ("P84", params(**P_P84)), ("P79", params(**P_P79)), ("HEAD+act", params(**ACTUATOR_ERROR)))
  return [(v, *[steady_curve(v, P) for _, P in sets]) for v in speeds], [nm for nm, _ in sets]


if __name__ == "__main__":
  print(f"controller constants from {CONTROLSD}: {len(HEAD_FROM_SOURCE)} of {len(HEAD_NAMES)} read"
        + ("" if len(HEAD_FROM_SOURCE) == len(HEAD_NAMES) else f" (FALLBACK for {sorted(set(HEAD_NAMES) - set(HEAD_FROM_SOURCE))})"))
  rows, names = fidelity_table()
  print("steady curve, cm inside of the straight-line position (mean of left / right, drives a1 + a7, linear chain, no noise); "
        "R / L; lane parts along the curve 1e-3")
  for v, *rs_ in rows:
    print(f" {v:3d} km/h k {STEADY_K[v] * 1e3:.1f}e-3: " + " | ".join(
      f"{nm} {r['inside']:+5.1f} (R {r['right']:+5.1f} L {r['left']:+5.1f}, term {r['term']:+.3f})" for nm, r in zip(names, rs_, strict=True)))
  import time
  scn = synthetic("straight", 60.0)
  t0 = time.monotonic(); r = simulate(scn, seed=1); dt_ = time.monotonic() - t0
  print(f"120 s straight at 60 km/h: {dt_:.2f} s wall; e mean {r['e'][2000:].mean()*100:+.1f} cm, std {r['e'][2000:].std()*100:.1f} cm")
  print(metrics(r)); print(margins(60.0))
