"""Phase 85 (routes b4 / b6, report 118): a road impact with hands off must not make the frame passive in mid-curve."""
import pytest

from opendbc.car.hyundai.values import CarControllerParams as P
from phase_tests.harness import Sim, run_signal
from phase_tests.test_phase37a import settle
from phase_tests.test_phase38 import _frame_active, _panda_delta

V = 21.0                                                             # 76 km/h
IMPACT_TQ = (233.0, 428.0, 336.0, -25.0, -331.0, -351.0, -154.0, 50.0, 148.0, 78.0)   # route b6 4980.98 s on


@pytest.fixture
def keep():
  saved = {k: getattr(P, k) for k in ("WHEEL_OUTRUN_NEEDS_DRIVER", "WHEEL_OUTRUN_PASSIVE")}
  yield
  for k, v in saved.items():
    setattr(P, k, v)


def in_curve():
  sim = Sim(); settle(sim, v=V)
  run_signal(sim, 300, v=V, wheel=-17.3, cmd=-17.3, tq=0.0, mdps_angle_2=-17.3)
  a, g = _frame_active(sim); assert a and g >= 0.8, (a, g)
  return sim


def impact(sim):
  """MDPS angle jumps 0.3 deg per sample for three samples and comes back; column torque rings."""
  dmax = _panda_delta(sim, V); assert dmax < 0.3
  w = -17.3; out = []
  for i, tq in enumerate(IMPACT_TQ):
    w += (-0.3, -0.3, -0.3, 0.4, 0.4, 0.1, 0.0, 0.0, 0.0, 0.0)[i]
    sim.step(v=V, wheel=-17.3, cmd=-17.3, tq=tq, pressed=False, mdps_angle_2=w)
    out.append(_frame_active(sim))
  return out


def test_shipped_values():
  assert P.WHEEL_OUTRUN_NEEDS_DRIVER is True and P.WHEEL_OUTRUN_DRIVER_GAIN == 0.5
  assert (P.WHEEL_OUTRUN_DRIVER_RECENT_FRAMES, P.WHEEL_OUTRUN_DRIVER_NM, P.WHEEL_OUTRUN_DRIVER_NM_FRAMES) == (10, 150.0, 8)


def test_impact_with_hands_off_keeps_the_frame_active(keep):
  out = impact(in_curve())
  assert all(a for a, _ in out), out
  assert min(g for _, g in out) >= 0.6, out                          # authority stays: the wheel is held in the curve


def test_kill_switch_restores_the_passive_frames(keep):
  P.WHEEL_OUTRUN_NEEDS_DRIVER = False
  out = impact(in_curve())
  assert any(not a and g == 0.0 for a, g in out), out


def test_driver_torque_of_one_sign_qualifies_after_80_ms(keep):
  sim = in_curve(); dmax = _panda_delta(sim, V); w = -17.3; first = None
  for i in range(20):
    w += dmax + 0.35
    sim.step(v=V, wheel=w, cmd=-17.3, tq=190.0, pressed=False, mdps_angle_2=w)
    if first is None and not _frame_active(sim)[0]:
      first = i
  assert first is not None and 6 <= first <= 9, first


def test_pressed_flag_qualifies_at_once(keep):
  sim = in_curve(); dmax = _panda_delta(sim, V); w = -17.3; act = []
  for i in range(4):
    w += dmax + 0.35
    sim.step(v=V, wheel=w, cmd=-17.3, tq=400.0, pressed=True, mdps_angle_2=w)
    act.append(_frame_active(sim)[0])
  assert act[1:] == [False, False, False], act                       # second fast sample, as before


def test_yielded_authority_qualifies(keep):
  sim = in_curve()
  sim.cc.aci_gain_last = 0.3                                         # authority already given up (blinker, grip, ramp)
  sim.cc.wheel_outrun_driver_age = 0
  dmax = _panda_delta(sim, V); w = -17.3; act = []
  for i in range(3):
    w += dmax + 0.35
    sim.step(v=V, wheel=w, cmd=-17.3, tq=40.0, pressed=False, mdps_angle_2=w)
    act.append(_frame_active(sim)[0])
  assert act[1:] == [False, False], act


def test_source_gate_and_reset():
  import opendbc.car.hyundai.carcontroller as cc
  src = open(cc.__file__).read()
  assert "self.wheel_outrun_hot >= CarControllerParams.WHEEL_OUTRUN_ENTRY_FRAMES and _driver" in src
