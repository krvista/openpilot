"""Phase 39 (i6nv3 routes e/f, 09-11): city-speed post-release recovery (39a) and the
resting-hand yield start 30 -> 60 Nm below 40 km/h (39b). Pure-function tests on
compute_torque_reduction_gain plus the call-site gate."""
import numpy as np

from phase_tests.harness import make_cp, Sim, run_signal  # noqa: F401 (path setup side effect)
from opendbc.car.hyundai.carcontroller import compute_torque_reduction_gain, _interp
from opendbc.car.hyundai.values import CarControllerParams as P


def frames_to(gain_target, *, city_release, post_grip=True, err=3.5, v_kph=35.0, last=0.19, n=600):
  """Frames for the gain to climb from `last` to gain_target with a steady |steering_error| = err."""
  for i in range(n):
    last = compute_torque_reduction_gain(0.0, v_kph, True, last, err, post_grip=post_grip,
                                         suppress_error_boost=True, city_release=city_release)
    if last >= gain_target:
      return i + 1
  return n


class TestPhase39aCityRelease:
  def test_post_grip_tail_recovers_3x_faster_when_city_release(self):
    # 35 km/h hands-off ceiling without the boost is 0.64, so climb 0.19 -> 0.60
    base = frames_to(0.6, city_release=False)
    city = frames_to(0.6, city_release=True)
    # reference tail 0.004/frame -> ~(0.6-0.19)/0.004 = 103 frames; city 0.012 -> ~34
    assert 90 <= base <= 120, base
    assert 28 <= city <= 45, city
    assert city * 2.5 < base

  def test_city_release_only_changes_the_beyond_2deg_tail(self):
    # inside the fast 0.5-1.5 deg region both regimes climb at the same (already fast) rate
    for err in (0.3, 1.0, 1.5):
      a = frames_to(0.6, city_release=False, err=err)
      b = frames_to(0.6, city_release=True, err=err)
      assert a == b, (err, a, b)

  def test_city_release_is_inert_outside_post_grip(self):
    # hands-off drift recovery (post_grip False) keeps the legacy shape exactly
    for err in (0.3, 1.0, 2.5, 4.0):
      a = frames_to(0.6, city_release=False, post_grip=False, err=err)
      b = frames_to(0.6, city_release=True, post_grip=False, err=err)
      assert a == b, (err, a, b)

  def test_rate_matches_constant_and_quantizer(self):
    last = 0.19
    steps = []
    for _ in range(30):
      g = compute_torque_reduction_gain(0.0, 35.0, True, last, 4.0, post_grip=True,
                                        suppress_error_boost=True, city_release=True)
      steps.append(g - last)
      last = g
    assert max(steps) <= P.CITY_RELEASE_RATE_UP + 0.0021
    assert np.median(steps) >= P.CITY_RELEASE_RATE_UP - 0.0021

  def test_call_site_gate_constants(self):
    # the gate the carcontroller applies: city speed, hands really off, no real grip
    assert P.CITY_RELEASE_SPEED_KPH == 45.0
    assert P.CITY_RELEASE_HANDS_OFF_NM == 30.0
    assert P.CITY_RELEASE_RATE_UP == P.ANCHORED_RECOVERY_RATE_UP


class TestPhase39bGripStart:
  @staticmethod
  def steady(tq, v_kph, grip_start):
    last = 0.0
    for _ in range(2000):
      last = compute_torque_reduction_gain(tq, v_kph, True, last, 0.0, grip_start=grip_start,
                                           suppress_error_boost=True)
    return last

  def test_table_values(self):
    assert P.ACIGAIN_GRIP_START_V == [60.0, 50.0]
    assert float(_interp(35.0, P.ACIGAIN_GRIP_START_SPEEDS_KPH, P.ACIGAIN_GRIP_START_V)) == 60.0
    assert float(_interp(60.0, P.ACIGAIN_GRIP_START_SPEEDS_KPH, P.ACIGAIN_GRIP_START_V)) == 50.0

  def test_resting_hand_keeps_authority_at_city_speed(self):
    # 35 km/h, 50-100 Nm driver-domain: the new start keeps (much) more of the ceiling
    for tq in (40.0, 60.0, 80.0, 100.0):
      old = self.steady(tq, 35.0, 30.0)
      new = self.steady(tq, 35.0, 60.0)
      assert new >= old - 1e-9, (tq, old, new)
    # a hand under the new start does not yield at all; at the old start it already did
    assert self.steady(55.0, 35.0, 60.0) == self.steady(0.0, 35.0, 60.0)
    assert self.steady(55.0, 35.0, 30.0) < self.steady(0.0, 35.0, 30.0)

  def test_heavy_hold_still_yields_to_the_floor(self):
    # at ho_full (140 Nm) and beyond both starts land on the same floor
    assert abs(self.steady(140.0, 35.0, 60.0) - self.steady(140.0, 35.0, 30.0)) < 0.005
    assert self.steady(300.0, 35.0, 60.0) <= 0.16


def _release_recovery(kill=False):
  """Full CarController: press at 35 km/h, release with the plan 4 deg away from the wheel;
  frames after release until the sent gain reaches 0.6."""
  old = P.CITY_RELEASE_SPEED_KPH
  try:
    if kill:
      P.CITY_RELEASE_SPEED_KPH = 0.0
    sim = Sim()
    run_signal(sim, 120, v=9.7, wheel=0.0, cmd=0.0, tq=0.0)
    run_signal(sim, 200, v=9.7, wheel=0.0, cmd=0.0, tq=0.0)
    run_signal(sim, 100, v=9.7, wheel=0.0, cmd=0.0, tq=460.0)      # real grip (pressed)
    tr = run_signal(sim, 400, v=9.7, wheel=0.0, cmd=4.0, tq=0.0)   # release, plan 4 deg away
    g = tr['gain']
    for i, x in enumerate(g):
      if x >= 0.6:
        return i + 1, g
    return len(g), g
  finally:
    P.CITY_RELEASE_SPEED_KPH = old


class TestPhase39aIntegration:
  def test_release_recovery_faster_at_city_speed(self):
    n_on, g_on = _release_recovery()
    n_kill, g_kill = _release_recovery(kill=True)
    assert n_on < n_kill, (n_on, n_kill)
    assert n_on <= 0.6 * n_kill, (n_on, n_kill, g_on[:5], g_kill[:5])
    assert max(g_on) <= 1.0 and min(g_on) >= 0.0
