import math

from cereal import log
from openpilot.selfdrive.controls.lib.roll_bias import MAX_OFFSET, RollBias, RollCorrectedParams


def torque_params(offset, points=9900., live_valid=True, use_params=False):
  tp = log.LiveTorqueParametersData.new_message()
  tp.latAccelOffsetFiltered = offset
  tp.totalBucketPoints = points
  tp.liveValid = live_valid
  tp.useParams = use_params
  return tp


def settle(rb, tp, valid=True, n=10000):
  for _ in range(n):
    b = rb.update(tp, valid)
  return b


class TestRollBias:
  def test_wk2_offset_removes_mount_roll(self):
    # torqued -0.45 m/s^2 -> 2.6 deg of roll to take out of the estimate (liveParameters +2.3 deg)
    b = settle(RollBias(), torque_params(-0.45))
    assert abs(math.degrees(b) - 2.63) < 0.05

  def test_gates(self):
    for tp, valid in [(torque_params(-0.45, points=1000.), True),
                      (torque_params(-0.45, live_valid=False), True),
                      (torque_params(-0.45, use_params=True), True),  # latcontrol_torque already removes it
                      (torque_params(float('nan')), True),
                      (torque_params(-0.45), False)]:
      assert settle(RollBias(), tp, valid) == 0.
    assert settle(RollBias(enabled=False), torque_params(-0.45)) == 0.

  def test_clipped(self):
    b = settle(RollBias(), torque_params(-3.0))
    assert abs(b - MAX_OFFSET / 9.81) < 1e-4

  def test_slow(self):
    rb = RollBias()
    b = settle(rb, torque_params(-0.45), n=100)  # 1 s
    assert 0 < b < 0.15 * 0.45 / 9.81

  def test_params_wrapper(self):
    lp = log.LiveParametersData.new_message(roll=0.04, steerRatio=15.5, angleOffsetDeg=0.3)
    p = RollCorrectedParams(lp, 0.0)
    assert p.roll == 0.0 and p.steerRatio == 15.5 and abs(p.angleOffsetDeg - 0.3) < 1e-6
