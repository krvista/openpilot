"""Phase 42 (i6nv3 0x25 Euljiro left turn, report §25): the driver was steering the turn with 300-1684 Nm while op held
its straight request at 0.79 -> 0.15 authority for 0.6 s (yield latency), then lost the wheel anchor for a few frames
when the pressed flags flickered. Three changes, all in the CarController:
  42a  city-speed fast yield  — ACIGAIN_GRIP_RATE_DN_FLOOR_V [0.0, 0.03] -> [0.03, 0.03] (same grip gate as 35a)
  42b  shove                  — driver_tq >= 700 empties authority at >= 0.10/frame
  42c  post-press anchor hold — apply stays on the wheel up to 0.3 s after a pressed-arm anchor ends, unless the
                                driver has really let go (driver_tq < 150 for 0.1 s)
"""
import numpy as np

from phase_tests.harness import Sim
import opendbc.car.hyundai.carcontroller as ccmod
from opendbc.car.hyundai.values import CarControllerParams as P


def press_run(v_kph, tq, frames=120, rate_dps=40.0, rest_tq=100.0):
  """3 s resting hand on a straight road, then a pressed turn-in at rate_dps with column torque tq."""
  sim = Sim(); v = v_kph / 3.6
  for _ in range(300):
    sim.step(v=v, tq=rest_tq, wheel=0.0, cmd=0.0)
  g0 = sim.tx_gain(); gains = []; wheel = 0.0
  for _ in range(frames):
    wheel += rate_dps * 0.01
    sim.step(v=v, tq=tq, wheel=wheel, cmd=0.0, pressed=True)
    gains.append(sim.tx_gain())
  return g0, np.array(gains)


def t_below(gains, level):
  idx = np.where(gains <= level)[0]
  return idx[0] / 100 if len(idx) else None


class TestPhase42aCityFastYield:
  def test_shipped_table(self):
    assert P.ACIGAIN_GRIP_RATE_DN_FLOOR_V == [0.03, 0.03]

  def test_city_press_reaches_floor_fast(self):
    g0, g = press_run(35.0, 450.0)
    assert g0 >= 0.5                                  # resting hand: op had real authority before the press
    t = t_below(g, 0.20)
    assert t is not None and t <= 0.25                # was 0.49 s with the 35a table

  def test_45kph_press_reaches_floor_fast(self):
    _, g = press_run(45.0, 450.0)
    assert t_below(g, 0.20) <= 0.25

  def test_highway_unchanged(self):
    old = P.ACIGAIN_GRIP_RATE_DN_FLOOR_V
    try:
      P.ACIGAIN_GRIP_RATE_DN_FLOOR_V = [0.0, 0.03]
      _, g_old = press_run(70.0, 450.0)
    finally:
      P.ACIGAIN_GRIP_RATE_DN_FLOOR_V = old
    _, g_new = press_run(70.0, 450.0)
    assert np.allclose(g_old, g_new)

  def test_resting_hand_not_affected(self):
    """A resting hand (below the 160 driver-Nm gate, not pressed) must see the identical slow curve."""
    def rest(v_kph):
      sim = Sim(); v = v_kph / 3.6
      for _ in range(300):
        sim.step(v=v, tq=100.0, wheel=0.0, cmd=0.0)
      out = []
      for i in range(200):
        sim.step(v=v, tq=180.0, wheel=0.5 * np.sin(i / 10.0), cmd=0.0)   # light hand, tiny wheel motion
        out.append(sim.tx_gain())
      return np.array(out)
    old = P.ACIGAIN_GRIP_RATE_DN_FLOOR_V
    try:
      P.ACIGAIN_GRIP_RATE_DN_FLOOR_V = [0.0, 0.03]
      ref = rest(35.0)
    finally:
      P.ACIGAIN_GRIP_RATE_DN_FLOOR_V = old
    assert np.allclose(ref, rest(35.0))

  def test_kill_restores_slow_city_drop(self):
    old = P.ACIGAIN_GRIP_RATE_DN_FLOOR_V
    try:
      P.ACIGAIN_GRIP_RATE_DN_FLOOR_V = [0.0, 0.03]
      _, g = press_run(35.0, 450.0)
    finally:
      P.ACIGAIN_GRIP_RATE_DN_FLOOR_V = old
    assert t_below(g, 0.20) > 0.35


class TestPhase42bShove:
  def _step(self, tq, last=0.60):
    return ccmod.compute_torque_reduction_gain(tq, 35.0, True, last, 0.0, grip_start=60.0, grip_full=110.0, grip_floor=0.08)

  def test_shove_drops_at_least_ten_quanta(self):
    g = self._step(800.0)
    assert g <= 0.60 - P.ACIGAIN_SHOVE_RATE_DN + 1e-9

  def test_below_shove_keeps_table_rate(self):
    g = self._step(600.0)
    assert 0.60 - 0.045 <= g <= 0.60 - 0.02            # table: 0.01 @300 -> 0.04 @700, ~0.033 @600

  def test_kill(self):
    old = P.ACIGAIN_SHOVE_RATE_DN
    try:
      P.ACIGAIN_SHOVE_RATE_DN = 0.0
      g = self._step(800.0)
    finally:
      P.ACIGAIN_SHOVE_RATE_DN = old
    assert g >= 0.60 - 0.045


class TestPhase42cAnchorHold:
  def _run(self, flicker_frames, flicker_tq=20.0, hold_frames=None):
    old = P.ANCHOR_HOLD_FRAMES
    if hold_frames is not None:
      P.ANCHOR_HOLD_FRAMES = hold_frames
    try:
      sim = Sim(); v = 35.0 / 3.6; wheel = 20.0
      for _ in range(300):
        sim.step(v=v, tq=100.0, wheel=0.0, cmd=0.0)
      for _ in range(100):
        sim.step(v=v, tq=450.0, wheel=wheel, cmd=0.0, pressed=True)      # anchored on the wheel at +20
      anchored = abs(sim.tx_angle() - wheel)
      if flicker_tq == "hold-band":
        # driver-domain torque between ANCHOR_HOLD_NM and the driver_pressed hysteresis threshold (0.8 x 250 = 200):
        # the debounced press decays, only the 42c hold keeps the anchor
        flicker_tq = sim.cc.hold_comp_last + 175.0
      dev = []
      for _ in range(flicker_frames):
        sim.step(v=v, tq=flicker_tq, wheel=wheel, cmd=0.0, pressed=False)  # flags drop, wheel held
        dev.append(abs(sim.tx_angle() - wheel))
      return anchored, np.array(dev), sim
    finally:
      P.ANCHOR_HOLD_FRAMES = old

  def test_anchored_before_flicker(self):
    anchored, _, _ = self._run(1)
    assert anchored <= 1.0

  def test_hold_bridges_a_short_flicker(self):
    _, dev_hold, sim = self._run(9)
    _, dev_kill, _ = self._run(9, hold_frames=0)
    assert dev_hold.max() <= 0.6
    assert dev_kill.max() > dev_hold.max() + 0.3      # without the hold the request leaves the wheel

  def test_real_let_go_ends_hold(self):
    _, dev, sim = self._run(60)                        # 0.6 s of 20 Nm: a real release
    assert sim.cc.anchor_hold_left == 0
    assert dev[-1] > 0.5                                # the request is free to move toward the plan again

  def test_sustained_light_torque_keeps_hold_through_window(self):
    _, dev, sim = self._run(25, flicker_tq="hold-band")  # flags off but 150-200 driver-Nm still on the wheel
    assert dev.max() <= 0.6
    _, dev_kill, _ = self._run(25, flicker_tq="hold-band", hold_frames=0)
    assert dev_kill.max() > 0.6                          # the test bites: without the hold the request walks off

  def test_shipped_defaults(self):
    assert P.ANCHOR_HOLD_FRAMES == 30 and P.ANCHOR_HOLD_NM == 150.0 and P.ANCHOR_HOLD_LOW_FRAMES == 10
