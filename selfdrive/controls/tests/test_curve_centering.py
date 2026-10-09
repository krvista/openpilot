import numpy as np

from cereal import car, log
from openpilot.selfdrive.controls.lib.curve_centering import CurveCentering, MAX_LAT_ACCEL, PARAM, read_enabled
from openpilot.selfdrive.modeld.constants import ModelConstants


def make_model(v, lat_accel, offset, width=3.2, prob=0.9, lane_change=log.LaneChangeState.off):
  """Constant-curvature road; the planned path runs `offset` m right of the lane centre."""
  curv = lat_accel / v ** 2
  md = log.ModelDataV2.new_message()
  md.action.desiredCurvature = curv
  md.meta.laneChangeState = lane_change
  x = np.array(ModelConstants.X_IDXS)
  lines = md.init('laneLines', 4)
  for i, y0 in enumerate([-1.5 * width, -width / 2, width / 2, 1.5 * width]):
    lines[i].x = x.tolist()
    lines[i].y = (y0 + curv * x ** 2 / 2).tolist()
  md.laneLineProbs = [prob] * 4
  px = v * np.array(ModelConstants.T_IDXS)
  md.position.x = px.tolist()
  md.position.y = (offset + curv * px ** 2 / 2).tolist()
  return md


def make_cs(v, left_bsm=False, right_bsm=False, blinker=False):
  return car.CarState.new_message(vEgo=v, leftBlindspot=left_bsm, rightBlindspot=right_bsm, leftBlinker=blinker)


def settle(cc, md, cs, lat_active=True):
  k = 0.
  for i in range(500):
    k = cc.update(lat_active, md, i % 5 == 0, cs)
  return k * cs.vEgo ** 2


class TestCurveCentering:
  def test_straight_untouched(self):
    assert settle(CurveCentering(), make_model(25., 0.1, 0.3), make_cs(25.)) == 0.

  def test_right_curve_inside_pushes_left(self):
    cc = CurveCentering()
    out = settle(cc, make_model(25., 1.2, 0.3), make_cs(25.))
    assert abs(cc.inside - 0.3) < 0.02
    assert -MAX_LAT_ACCEL <= out < -0.2

  def test_left_curve_inside_pushes_right(self):
    assert 0.2 < settle(CurveCentering(), make_model(25., -1.2, -0.3), make_cs(25.)) <= MAX_LAT_ACCEL

  def test_outside_or_centred_untouched(self):
    assert settle(CurveCentering(), make_model(25., 1.2, -0.3), make_cs(25.)) == 0.
    assert settle(CurveCentering(), make_model(25., 1.2, 0.05), make_cs(25.)) == 0.

  def test_bsm_on_inside_aims_outward_of_centre(self):
    md = make_model(25., 1.0, 0.0)
    assert settle(CurveCentering(), md, make_cs(25.)) == 0.
    assert settle(CurveCentering(), md, make_cs(25., right_bsm=True)) < -0.05
    assert settle(CurveCentering(), md, make_cs(25., left_bsm=True)) == 0.

  def test_gates(self):
    for md, cs in [(make_model(25., 1.2, 0.3, prob=0.4), make_cs(25.)),
                   (make_model(25., 1.2, 0.3, width=5.0), make_cs(25.)),
                   (make_model(25., 1.2, 0.3, lane_change=log.LaneChangeState.laneChangeStarting), make_cs(25.)),
                   (make_model(25., 1.2, 0.3), make_cs(25., blinker=True)),
                   (make_model(10., 1.2, 0.3), make_cs(10.))]:
      assert settle(CurveCentering(), md, cs) == 0.
    assert settle(CurveCentering(), make_model(25., 1.2, 0.3), make_cs(25.), lat_active=False) == 0.
    assert settle(CurveCentering(enabled=False), make_model(25., 1.2, 0.3), make_cs(25.)) == 0.

  def test_bounded_by_model_lat_accel(self):
    # far inside on a gentle curve: limited to half of the model's own 0.5 m/s^2
    out = settle(CurveCentering(), make_model(25., 0.5, 1.0), make_cs(25.))
    assert -0.25 - 1e-3 < out < -0.24

  def test_smooth(self):
    cc = CurveCentering()
    md, cs = make_model(25., 1.2, 0.5), make_cs(25.)
    prev = 0.
    for i in range(300):
      out = cc.update(True, md, i % 5 == 0, cs) * 25. ** 2
      assert abs(out - prev) < 0.01  # per 10 ms
      prev = out

  def test_malformed_model_returns_zero(self):
    md = make_model(25., 1.2, 0.3)
    md.position.x = []
    assert settle(CurveCentering(), md, make_cs(25.)) == 0.

  def test_toggle_read_without_params_key(self, tmp_path):
    class PrebuiltParams:  # prebuilt params library: the key is unknown, only the path lookup works
      def get_param_path(self, key=""):
        return str(tmp_path / key)
      def get_bool(self, key, block=False):
        raise Exception(f"UnknownKeyName {key}")
    p = PrebuiltParams()
    assert read_enabled(p)
    (tmp_path / PARAM).write_text("0")
    assert not read_enabled(p)
    (tmp_path / PARAM).write_text("1")
    assert read_enabled(p)
