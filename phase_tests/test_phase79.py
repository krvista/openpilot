"""Phase 79 (report 110): the lane-centre offset gain is trimmed - flat 0.3e-3 from 60 km/h, fading above 80."""
import pytest

import openpilot.selfdrive.controls.controlsd as cd
from phase_tests.test_phase73 import model, term

KPH = 1 / 3.6


@pytest.fixture
def keep():
  saved = (cd.LANE_CENTER_KE, cd.LANE_CENTER_KI, cd.LANE_FIT_GAIN)
  cd.LANE_CENTER_KE = [0.0, 0.3e-3, 0.3e-3, 0.15e-3]; cd.LANE_FIT_GAIN = 0.0     # the Phase 79 schedule (Phase 82 replaced it)
  yield
  cd.LANE_CENTER_KE, cd.LANE_CENTER_KI, cd.LANE_FIT_GAIN = saved


def test_shipped_schedule():
  assert cd.LANE_CENTER_KE_SPEED_MS == pytest.approx([45 * KPH, 60 * KPH, 80 * KPH, 110 * KPH])
  assert cd.LANE_CENTER_KE == [0.0, 0.0, 0.0, 0.0]                             # Phase 82 (was [0, 0.3e-3, 0.3e-3, 0.15e-3])
  assert cd.LANE_CENTER_KI == 0.06e-3 and cd.LANE_CENTER_I_CAP == 0.30e-3     # the integral is not touched


def test_offset_gain_by_speed(keep):
  cd.LANE_CENTER_KI = 0.0
  e = 0.10 + cd.LANE_CENTER_BIAS_M
  for v_kph, ke in ((40, 0.0), (45, 0.0), (52.5, 0.15e-3), (60, 0.3e-3), (70, 0.3e-3), (80, 0.3e-3),
                    (95, 0.225e-3), (110, 0.15e-3), (130, 0.15e-3)):
    out, s = term(model(centre=0.10), v=v_kph * KPH, n=400)
    assert s._lc_e == pytest.approx(ke * e, rel=0.02, abs=1e-9), v_kph
    assert out == pytest.approx(s._lc_p + s._lc_e, rel=1e-6)


def test_never_stiffer_than_phase77_and_never_above_the_flat_value(keep):
  cd.LANE_CENTER_KI = 0.0
  old_v, old_k = [45 * KPH, 60 * KPH, 80 * KPH], [0.0, 0.5e-3, 0.8e-3]
  e = 0.10 + cd.LANE_CENTER_BIAS_M
  for v_kph in range(30, 140, 5):
    _, s = term(model(centre=0.10), v=v_kph * KPH, n=400)
    assert s._lc_e <= float(cd._interp(v_kph * KPH, old_v, old_k)) * e * 1.02 + 1e-9
    assert s._lc_e <= 0.3e-3 * e * 1.02


def test_kill_switch(keep):
  cd.LANE_CENTER_KI = 0.0
  cd.LANE_CENTER_KE = [0.0, 0.0, 0.0, 0.0]
  for v_kph in (50, 70, 100):
    out, s = term(model(centre=0.10), v=v_kph * KPH, n=400)
    assert s._lc_e == 0.0 and out == pytest.approx(s._lc_p)
