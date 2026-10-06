"""Phase 46 (report §32): boot observability + boot alerts, and steering telemetry flags.

  46-1  (removed, report §37) modeld_v2 import-stage timing — found the cause (compile_modeld Device.DEFAULT, Phase 48).
  46-2  angle-control cars drop lateralTorqueParameters from selfdrived's checks (unused; held the boot engage ~1 s).
  46-3  selfdrived stays "initializing" while the driving model has not published yet (max 30 s), then gives the rest of
        the pipeline 3 s; REPLAY keeps the upstream 6 s.
  46-4  controlsState.steerFlags (controlsd guards) and carStateSP.steerFlags (CarController latches / yield paths).
"""
import os
import types

import pytest

from phase_tests.harness import Sim
import opendbc.car.hyundai.carcontroller as ccmod
from opendbc.car import structs
from opendbc.car.structs import car

KPH = 1 / 3.6
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


# ---------------------------------------------------------------- 46-3 initialization wait
def _sd():
  from phase_tests.harness_noncontrol import FakeParams  # noqa: F401 (stub install side effect)
  import openpilot.selfdrive.selfdrived.selfdrived as sd
  return sd


def _timeline(sd, model_at_s=None, until_s=40.0):
  """Return the first time init_timed_out() is True, stepping selfdrived's 100 Hz frame counter."""
  obj = types.SimpleNamespace(sm=types.SimpleNamespace(frame=0, seen={'modelV2': False}), init_model_seen_frame=None)
  for f in range(int(until_s * 100)):
    obj.sm.frame = f
    obj.sm.seen['modelV2'] = model_at_s is not None and f * 0.01 >= model_at_s
    if sd.SelfdriveD.init_timed_out(obj):
      return f * 0.01
  return None


class TestInitWait:
  def test_waits_for_the_model_then_three_seconds(self):
    sd = _sd()
    t = _timeline(sd, model_at_s=11.0)        # first modelV2 ~11 s after selfdrived starts (routes 2e-38: 10-11.6 s)
    assert t is not None and abs(t - 14.0) < 0.02

  def test_model_never_comes_times_out_at_30s(self):
    sd = _sd()
    t = _timeline(sd, model_at_s=None)
    assert t is not None and abs(t - sd.INIT_MODEL_WAIT_MAX_S) < 0.02

  def test_never_before_upstream_6s(self):
    sd = _sd()
    t = _timeline(sd, model_at_s=0.5)          # model already up: the upstream 6 s floor still applies
    assert t is not None and t > sd.INIT_TIMEOUT_S

  def test_replay_keeps_upstream(self, monkeypatch):
    sd = _sd()
    monkeypatch.setattr(sd, "REPLAY", True)
    assert abs(_timeline(sd, model_at_s=None) - 6.0) < 0.02

  def test_kill_restores_upstream(self, monkeypatch):
    sd = _sd()
    monkeypatch.setattr(sd, "INIT_MODEL_WAIT_MAX_S", 6.0)
    assert abs(_timeline(sd, model_at_s=None) - 6.0) < 0.02
    assert abs(_timeline(sd, model_at_s=3.0) - 6.0) < 0.02

  def test_shipped_defaults(self):
    sd = _sd()
    assert (sd.INIT_TIMEOUT_S, sd.INIT_MODEL_WAIT_MAX_S, sd.INIT_AFTER_MODEL_S) == (6.0, 30.0, 3.0)


class TestSafetyModeGrace:
  """Review fix: card writes ControlsReady (-> pandad leaves ELM327) only after selfdrived initializes, so the 10 s
  safety-mismatch grace must also run from initialization."""
  def _grace(self, sd, frame, init_frame):
    obj = types.SimpleNamespace(sm=types.SimpleNamespace(frame=frame), init_frame=init_frame)
    return sd.SelfdriveD.safety_mode_grace_over(obj)

  def test_late_init_gets_four_seconds(self):
    sd = _sd()
    assert not self._grace(sd, 1400, 1400)          # initialized at 14 s: no mismatch at once (the bug)
    assert not self._grace(sd, 1790, 1400)
    assert self._grace(sd, 1801, 1400)

  def test_upstream_10s_floor_kept(self):
    sd = _sd()
    assert not self._grace(sd, 900, 100)            # initialized early: still the upstream 10 s
    assert self._grace(sd, 1001, 100)

  def test_source_uses_the_helper(self):
    src = open(os.path.join(ROOT, "openpilot/selfdrive/selfdrived/selfdrived.py")).read()
    assert "(safety_mismatch and self.safety_mode_grace_over())" in src
    assert "safety_mismatch and self.sm.frame*DT_CTRL > 10." not in src
    assert "self.init_frame = self.sm.frame" in src


# ---------------------------------------------------------------- 46-2 torque parameters on angle cars
class TestUnusedLateralServices:
  def test_angle_car_drops_torque_params(self):
    sd = _sd()
    CP = types.SimpleNamespace(steerControlType=car.CarParams.SteerControlType.angle)
    assert sd.unused_lateral_services(CP) == ['lateralTorqueParameters']

  def test_torque_car_keeps_them(self):
    sd = _sd()
    CP = types.SimpleNamespace(steerControlType=car.CarParams.SteerControlType.torque)
    assert sd.unused_lateral_services(CP) == []

  def test_controlsd_reads_them_only_under_torque_tuning(self):
    """The premise of 46-2: controlsd's only use is inside `lateralTuning.which() == 'torque'`."""
    src = open(os.path.join(ROOT, "openpilot/selfdrive/controls/controlsd.py")).read()
    uses = [i for i in range(len(src)) if src.startswith("self.sm['lateralTorqueParameters']", i)]
    assert len(uses) == 1
    before = src[:uses[0]].rsplit("\n", 4)
    assert "if self.CP.lateralTuning.which() == 'torque':" in "\n".join(before)


# ---------------------------------------------------------------- 46-4 steering flags
def _drive(sim, n, v_kph, **kw):
  for _ in range(n):
    sim.step(v=v_kph * KPH, **{**dict(tq=0.0, wheel=0.0, cmd=0.0), **kw})
  return sim.cc.steer_debug_flags


class TestCarControllerFlags:
  def test_hands_off_cruise(self):
    sim = Sim()
    f = _drive(sim, 400, 60.0)
    assert f & ccmod.SF_EFF_ACTIVE and f & ccmod.SF_WIRE_ACTIVE
    for bit in (ccmod.SF_PARKING, ccmod.SF_PASSTHROUGH, ccmod.SF_DRIVER_PRESSED, ccmod.SF_FAST_FLOOR, ccmod.SF_ACCEL_YIELD):
      assert not f & bit

  def test_parking_latch_and_release_window(self):
    sim = Sim()
    f = _drive(sim, 500, 10.0)
    assert f & ccmod.SF_PARKING and not f & ccmod.SF_EFF_ACTIVE
    f = _drive(sim, 50, 20.0, lane_prob_min=0.9)
    assert f & ccmod.SF_PARKING and f & ccmod.SF_PARKING_RELEASING
    f = _drive(sim, ccmod.PARKING_RELEASE_FRAMES, 20.0, lane_prob_min=0.9)
    assert not f & ccmod.SF_PARKING and not f & ccmod.SF_PARKING_RELEASING

  def test_eps_press_opens_fast_floor(self):
    sim = Sim()
    _drive(sim, 300, 50.0)
    f = _drive(sim, 20, 50.0, tq=330.0, pressed=True)
    assert f & ccmod.SF_EPS_PRESSED and f & ccmod.SF_FAST_FLOOR

  def test_hard_acceleration(self):
    sim = Sim()
    _drive(sim, 300, 70.0)
    f = _drive(sim, 50, 70.0, a_ego=3.0, gas=True)
    assert f & ccmod.SF_ACCEL_YIELD

  def test_low_speed_grip_latch(self):
    sim = Sim()
    _drive(sim, 300, 40.0)                       # past the boot window
    f = _drive(sim, 200, 14.4, lane_prob_min=0.9)
    assert f & ccmod.SF_EFF_ACTIVE and not f & ccmod.SF_PASSTHROUGH
    f = _drive(sim, 30, 14.4, tq=420.0, pressed=True, cmd=5.0, lane_prob_min=0.9)   # a real grip at 4 m/s
    assert f & ccmod.SF_PASSTHROUGH and f & ccmod.SF_LOWSPEED_LATCH and not f & ccmod.SF_EFF_ACTIVE

  def test_bits_unique_and_fit_uint32(self):
    bits = [v for k, v in vars(ccmod).items() if k.startswith("SF_")]
    assert len(bits) == len(set(bits)) and all(b > 0 and (b & (b - 1)) == 0 and b < 2 ** 32 for b in bits)


class TestControlsdFlags:
  def test_bits_unique_and_fit_uint16(self):
    src = open(os.path.join(ROOT, "openpilot/selfdrive/controls/controlsd.py")).read()
    vals = [int(line.split("1 <<")[1].split()[0]) for line in src.splitlines() if line.startswith("STEER_FLAG_")]
    assert len(vals) == 9 and len(set(vals)) == len(vals) and max(vals) < 16

  def test_low_confidence_step_sets_gate_and_cap(self):
    try:
      from phase_tests.test_noncontrol_controlsd_state import mk_controls, run
      from phase_tests.test_phase41 import _model
      import openpilot.selfdrive.controls.controlsd as cd
    except Exception as e:  # noqa: BLE001 - environments without the controlsd import chain
      pytest.skip(f"controlsd unavailable: {e}")
    s = mk_controls(); s.sm['carState'].vEgo = 12.0
    run(s, 200, _model(0.0, [0.9, 0.95, 0.95, 0.9]))
    assert not s.steer_flags & (cd.STEER_FLAG_LOWCONF_GATE | cd.STEER_FLAG_LOWCONF_CAPPED)
    run(s, 20, _model(0.006, [0.9, 0.2, 0.95, 0.9]))
    assert s.steer_flags & cd.STEER_FLAG_LOWCONF_GATE and s.steer_flags & cd.STEER_FLAG_LOWCONF_CAPPED
    assert s.steer_flags & cd.STEER_FLAG_CONF_BLEND          # lane_min 0.2 < 0.30: the 6g blend acts too
    run(s, 20, _model(0.006, [0.9, 0.95, 0.95, 0.9]))
    assert s.steer_flags & cd.STEER_FLAG_LOWCONF_RELEASE and not s.steer_flags & cd.STEER_FLAG_LOWCONF_GATE
    n = 0
    for _ in range(200):                                 # the ramp is flagged on every one of its frames, then clears
      run(s, 1, _model(0.006, [0.9, 0.95, 0.95, 0.9]))
      n += bool(s.steer_flags & cd.STEER_FLAG_LOWCONF_RELEASE)
    assert n == int(cd.LowConfRateCap(0.01).release_frames) - 20 and not s.steer_flags & cd.STEER_FLAG_LOWCONF_RELEASE


class TestPlumbing:
  def test_struct_default(self):
    assert structs.CarStateSP().steerFlags == 0

  def test_capnp_roundtrip_and_size(self):
    try:
      from openpilot.cereal import log
    except Exception as e:  # noqa: BLE001
      pytest.skip(f"capnp unavailable: {e}")
    cs = log.Event.new_message(); cs.init('controlsState').steerFlags = 0x1FF
    sp = log.Event.new_message(); sp.init('carStateSP').steerFlags = 0x0FFFFFFF
    assert cs.controlsState.steerFlags == 0x1FF and sp.carStateSP.steerFlags == 0x0FFFFFFF
    # Load check: the new fields cost at most one 8-byte word per message
    cs0 = log.Event.new_message(); cs0.init('controlsState').steerCmdGapDeg = 1.0
    cs1 = log.Event.new_message(); cs1.init('controlsState').steerCmdGapDeg = 1.0; cs1.controlsState.steerFlags = 3
    assert len(cs1.to_bytes()) - len(cs0.to_bytes()) <= 8
    sp0 = log.Event.new_message(); sp0.init('carStateSP').lateralControlPaused = True
    sp1 = log.Event.new_message(); sp1.init('carStateSP').lateralControlPaused = True; sp1.carStateSP.steerFlags = 3
    assert len(sp1.to_bytes()) - len(sp0.to_bytes()) <= 8

  def test_card_mirrors_the_flags(self):
    src = open(os.path.join(ROOT, "openpilot/selfdrive/car/card.py")).read()
    assert "CS_SP.steerFlags = int(getattr(self.CI.CC, 'steer_debug_flags', 0))" in src
