"""Phase 49 (i6nv3 routes 3d-42, report §39): the lane-dropout latch ("Lane Lines Lost") needs BOTH inner lines weak and
the driver's hands off the wheel. One clear line (Namsan tunnel: left 0.97 / right 0.14-0.19) no longer latches and a
running hold ends once one line is back; a driver press restarts the 0.3 s entry debounce and ends a running hold (one
hold per blackout, like a lane change). Everything else is the 39-2 latch (report §40)."""
import pytest

from phase_tests.test_noncontrol_controlsd_state import K, cd, mk_controls, mk_model, run


def mk_lanes(k_plan, left, right):
  m = mk_model(fallback=k_plan)
  m.laneLineProbs = [left, left, right, right]
  return m


class TestPhase49BothLines:
  def test_one_clear_line_does_not_latch(self):
    # tunnel: wall-side line ~0.2 / 0.13, the other one clear
    for left, right in ((0.97, 0.14), (0.71, 0.05), (0.10, 0.97), (0.10, 0.31), (0.10, 0.30)):   # 0.30 itself is not weak
      s = mk_controls(); run(s, 200, mk_model())
      run(s, 300, mk_lanes(K, left, right))
      assert not s.lane_dropout, (left, right)

  def test_both_weak_still_latches(self):
    # route 04 seg 8 numbers (0.14 / 0.09) and a stronger line just under LANE_HI
    for left, right in ((0.14, 0.09), (0.10, 0.29)):
      s = mk_controls(); run(s, 200, mk_model())
      run(s, 35, mk_lanes(K, left, right))
      assert s.lane_dropout, (left, right)

  def test_stronger_line_rising_past_hi_resets_the_entry_debounce(self):
    s = mk_controls(); run(s, 200, mk_model())
    run(s, 20, mk_lanes(K, 0.10, 0.10))
    run(s, 1, mk_lanes(K, 0.10, 0.50))            # one frame with a clear right line
    run(s, 20, mk_lanes(K, 0.10, 0.10))           # 0.2 s again: debounce restarted, not 0.4 s
    assert not s.lane_dropout
    run(s, 15, mk_lanes(K, 0.10, 0.10))
    assert s.lane_dropout


class TestPhase49HandsOff:
  def test_no_entry_while_driver_holds(self):
    s = mk_controls(); run(s, 200, mk_model())
    s.sm['carState'].steeringPressed = True
    run(s, 200, mk_lanes(K, 0.10, 0.10))
    assert not s.lane_dropout

  def test_letting_go_needs_0_3_s_hands_off_and_the_hold_is_continuous(self):
    # review: the debounce restarts on a press (0.3 s hands-off before a hold), and the hold value is op's own command
    # from after the release — the latch starts without a step
    s = mk_controls(); run(s, 200, mk_model())        # steady on K
    s.sm['carState'].steeringPressed = True
    run(s, 200, mk_lanes(4 * K, 0.10, 0.10))          # driver turns with a 4x plan for 2 s, lines gone
    assert not s.lane_dropout and s.lane_dropout_armed   # a press while not latched never disarms
    s.sm['carState'].steeringPressed = False
    run(s, 1, mk_lanes(4 * K, 0.10, 0.10))
    at_release = s.desired_curvature
    run(s, 24, mk_lanes(4 * K, 0.10, 0.10))
    assert not s.lane_dropout                         # 0.25 s hands-off: not yet
    before = s.desired_curvature
    out = run(s, 10, mk_lanes(4 * K, 0.10, 0.10))
    assert s.lane_dropout                             # 0.35 s: held
    assert out[0] == pytest.approx(at_release, abs=0.1 * K), (out[0], at_release)
    assert abs(out[0] - before) < 0.1 * K

  def test_press_frame_clears_flag_and_hands_back_to_the_blend(self):
    s = mk_controls(); run(s, 200, mk_model())
    run(s, 100, mk_lanes(4 * K, 0.10, 0.10))
    assert s.lane_dropout and s.steer_flags & cd.STEER_FLAG_LANE_DROPOUT
    held = s.desired_curvature
    s.sm['carState'].steeringPressed = True
    out = run(s, 1, mk_lanes(4 * K, 0.10, 0.10))
    assert not s.lane_dropout and not (s.steer_flags & cd.STEER_FLAG_LANE_DROPOUT)
    assert out[0] > held                              # the blend moves toward the plan again (no hold)

  def test_press_ends_a_hold_and_no_relatch_until_lines_return(self):
    s = mk_controls(); run(s, 200, mk_model())
    run(s, 100, mk_lanes(4 * K, 0.10, 0.10))
    assert s.lane_dropout
    s.sm['carState'].steeringPressed = True
    run(s, 1, mk_lanes(4 * K, 0.10, 0.10))
    assert not s.lane_dropout and not s.lane_dropout_armed
    s.sm['carState'].steeringPressed = False
    run(s, 300, mk_lanes(4 * K, 0.10, 0.10))     # same blackout, hands off again: no second hold
    assert not s.lane_dropout
    run(s, 110, mk_lanes(K, 0.9, 0.9))            # lines back > LANE_HI for 1.1 s: re-armed
    assert s.lane_dropout_armed
    run(s, 35, mk_lanes(K, 0.10, 0.10))
    assert s.lane_dropout

  def test_hold_value_is_unchanged_hands_off(self):
    # the 39-2 hold itself is untouched: pre-dropout command, not the 4x spike
    s = mk_controls(); run(s, 200, mk_model())
    steady = s.desired_curvature
    out = run(s, 200, mk_lanes(4 * K, 0.10, 0.10))
    assert s.lane_dropout
    assert out[0] == pytest.approx(steady, rel=0.05)


class TestPhase49Exit:
  def test_one_line_back_ends_the_hold(self):
    s = mk_controls(); run(s, 200, mk_model())
    run(s, 100, mk_lanes(K, 0.10, 0.10))
    assert s.lane_dropout
    run(s, 40, mk_lanes(K, 0.97, 0.10))               # 0.4 s with the left line back: still held
    assert s.lane_dropout
    run(s, 15, mk_lanes(K, 0.97, 0.10))               # 0.55 s: released, armed kept (a normal exit)
    assert not s.lane_dropout and s.lane_dropout_armed


class TestPhase49Kill:
  @pytest.fixture(autouse=True)
  def _restore(self):
    old = (cd.LANE_DROPOUT_MAX_LM, cd.LANE_DROPOUT_HANDS_OFF, cd.LANE_DROPOUT_EXIT_ON_ONE_LINE)
    yield
    cd.LANE_DROPOUT_MAX_LM, cd.LANE_DROPOUT_HANDS_OFF, cd.LANE_DROPOUT_EXIT_ON_ONE_LINE = old

  def test_kill_max_lm_alone(self):
    cd.LANE_DROPOUT_MAX_LM = 1.01
    s = mk_controls(); run(s, 200, mk_model())
    run(s, 35, mk_lanes(K, 0.97, 0.13))
    assert s.lane_dropout                         # one weak line latches again
    s2 = mk_controls(); run(s2, 200, mk_model())
    s2.sm['carState'].steeringPressed = True
    run(s2, 100, mk_lanes(K, 0.10, 0.10))
    assert not s2.lane_dropout                    # the hands-off gate is independent

  def test_kill_hands_off_alone(self):
    cd.LANE_DROPOUT_HANDS_OFF = False
    s = mk_controls(); run(s, 200, mk_model())
    s.sm['carState'].steeringPressed = True
    run(s, 35, mk_lanes(K, 0.10, 0.10))
    assert s.lane_dropout                         # a held wheel no longer blocks it
    run(s, 10, mk_lanes(K, 0.10, 0.10))
    assert s.lane_dropout                         # nor ends it
    s2 = mk_controls(); run(s2, 200, mk_model())
    run(s2, 300, mk_lanes(K, 0.97, 0.13))
    assert not s2.lane_dropout                    # the both-lines gate is independent

  def test_kill_exit_on_one_line_alone(self):
    cd.LANE_DROPOUT_EXIT_ON_ONE_LINE = False
    s = mk_controls(); run(s, 200, mk_model())
    run(s, 100, mk_lanes(K, 0.10, 0.10))
    run(s, 100, mk_lanes(K, 0.97, 0.10))          # one line back for 1 s: pre-49 exit needs both
    assert s.lane_dropout

  def test_defaults(self):
    assert cd.LANE_DROPOUT_MAX_LM == pytest.approx(0.30)
    assert cd.LANE_DROPOUT_HANDS_OFF is True
    assert cd.LANE_DROPOUT_EXIT_ON_ONE_LINE is True
    assert cd.LANE_DROPOUT_MIN_SPEED == pytest.approx(40.0 / 3.6)   # speed gate unchanged (user decision, §39)
