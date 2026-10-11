import numpy as np

from cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.curve_exit_lead import CurveExitLead, EXIT_LEAD_T, MAX_REDUCTION
from openpilot.selfdrive.controls.lib.drive_helpers import get_curvature_from_plan
from openpilot.selfdrive.modeld.constants import ModelConstants

V = 25.
LAT_DELAY = 0.28
T = np.array(ModelConstants.T_IDXS)


def plan(curv_fn):
  """Model plan whose curvature at time t is curv_fn(t); returns (model, curvature the model would act on)."""
  md = log.ModelDataV2.new_message()
  k = np.array([curv_fn(t) for t in T])
  yaw_rate = k * V
  yaw = np.concatenate([[0.], np.cumsum((yaw_rate[1:] + yaw_rate[:-1]) / 2 * np.diff(T))])
  md.orientation.t = T.tolist()
  md.orientation.z = yaw.tolist()
  md.orientationRate.t = T.tolist()
  md.orientationRate.z = yaw_rate.tolist()
  action = get_curvature_from_plan(yaw, yaw_rate, T, V, LAT_DELAY + 1.5 * DT_MDL)
  return md, action


def lat(k):
  return k * V ** 2


class TestCurveExitLead:
  def test_exit_unwinds_earlier(self):
    md, k = plan(lambda t: max(0., 1.2 - 1.5 * t) / V ** 2)  # unwinding at 1.5 m/s^3
    out = CurveExitLead().update(md, V, k, LAT_DELAY)
    assert 0 < out < k
    assert abs(lat(k - out) - 1.5 * EXIT_LEAD_T) < 0.06

  def test_entry_and_steady_untouched(self):
    for fn in (lambda t: min(1.5, 0.6 + 1.5 * t) / V ** 2, lambda t: 1.0 / V ** 2):
      md, k = plan(fn)
      assert CurveExitLead().update(md, V, k, LAT_DELAY) == k

  def test_left_curve_exit(self):
    md, k = plan(lambda t: min(0., -1.2 + 1.5 * t) / V ** 2)
    out = CurveExitLead().update(md, V, k, LAT_DELAY)
    assert k < out < 0.

  def test_never_past_straight(self):
    # left curve whose plan is already into a right one 0.1 s later: stop at straight, never steer right
    md, k = plan(lambda t: (-1.7 + 4.0 * t) / V ** 2)
    assert lat(k) < -0.2
    out = CurveExitLead().update(md, V, k, LAT_DELAY)
    assert k < out <= 0.

  def test_bounded(self):
    md, k = plan(lambda t: max(0., 1.5 - 6.0 * t) / V ** 2)
    out = CurveExitLead().update(md, V, k, LAT_DELAY)
    assert lat(k - out) <= MAX_REDUCTION + 1e-6

  def test_gates(self):
    md, k = plan(lambda t: max(0., 1.2 - 1.5 * t) / V ** 2)
    assert CurveExitLead(enabled=False).update(md, V, k, LAT_DELAY) == k
    assert CurveExitLead().update(md, 10., k, LAT_DELAY) == k
    assert CurveExitLead().update(md, V, k + 0.5 / V ** 2, LAT_DELAY) == k + 0.5 / V ** 2  # plan disagrees with the action
    assert CurveExitLead().update(log.ModelDataV2.new_message(), V, k, LAT_DELAY) == k
    md, k = plan(lambda t: max(0., 0.15 - 0.5 * t) / V ** 2)  # straight-ish
    assert CurveExitLead().update(md, V, k, LAT_DELAY) == k
