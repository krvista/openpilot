"""Phase 38: wire governor — the transmitted LKAS_ALT angle never exceeds the panda's VM per-frame
delta / max angle relative to the last transmitted value, even when op anchors apply_angle_last to the wheel."""
from phase_tests.harness import Sim, run_signal
from opendbc.car.hyundai.values import CarControllerParams as P
from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm
import contextlib


@contextlib.contextmanager
def no_outrun_passive():
  """Phase 38-3 sends PASSIVE frames while the wheel outruns the panda allowance; these governor tests
  drive the wheel 12 deg in one/ten frames, so run them with 38-3 off to test the governor's own contract."""
  old = P.WHEEL_OUTRUN_PASSIVE
  P.WHEEL_OUTRUN_PASSIVE = False
  try:
    yield
  finally:
    P.WHEEL_OUTRUN_PASSIVE = old


def settle(sim, n=120, **kw):
  kw.setdefault('v', 15.0); kw.setdefault('wheel', 0.0); kw.setdefault('cmd', 0.0); kw.setdefault('tq', 0.0)
  return run_signal(sim, n, **kw)


def _tx_angle(sim):
  for m in sim.last_msgs:
    if isinstance(m, tuple) and m[0] == "LKAS_ALT":
      return float(m[2]["ADAS_StrAnglReqVal"])
  raise AssertionError("no LKAS_ALT frame")


def _panda_delta(sim, v):
  return min(get_max_angle_delta_vm(v, sim.s.BASELINE_VM, sim.s.params), P.ANGLE_LIMITS.MAX_ANGLE_RATE)


class TestPhase38WireGovernor:
  def test_grip_anchor_jump_is_ramped_on_the_wire(self):
    with no_outrun_passive():
      # 53 km/h: driver grips and swings the wheel 12 deg in 0.1 s (120 deg/s); op anchors
      # apply_angle_last to the wheel internally, but the wire must move <= panda delta/frame
      v = 14.7; sim = Sim(); settle(sim)
      run_signal(sim, 200, v=v, wheel=0.0, cmd=0.0, tq=0.0)
      tx_prev = _tx_angle(sim); dmax = _panda_delta(sim, v); worst = 0.0; internal_jump = 0.0; resyncs = 0
      for i in range(60):
        w = min(12.0, 1.2 * (i + 1))
        sim.step(v=v, wheel=w, cmd=0.0, tq=460.0, pressed=True, mdps_angle_2=w)
        tx = _tx_angle(sim)
        if abs(tx - w) < 1e-6 and abs(tx - tx_prev) > dmax:   # saturation-watchdog resync frame: wire == measured
          resyncs += 1
        else:
          worst = max(worst, abs(tx - tx_prev))
        tx_prev = tx
        internal_jump = max(internal_jump, abs(sim.s.apply_angle_last - tx))
      assert worst <= dmax + 1e-9, (worst, dmax)
      assert internal_jump > 2.0, "test premise: the internal anchor did jump ahead of the wire"
      assert dmax < 1.0                                       # ~0.47 deg/frame at 53 km/h on the baseline model
      assert resyncs <= 1, resyncs                            # backup watchdog (100 frames); the echo detector is the primary realignment

  def test_wire_reference_resets_to_measured_on_passive_frames(self):
    v = 14.7; sim = Sim(); settle(sim)
    run_signal(sim, 100, v=v, wheel=0.0, cmd=0.0, tq=0.0)
    sim.step(v=v, wheel=9.0, cmd=0.0, tq=0.0, lat_active=False, mdps_angle_2=9.3)   # passive: wire == measured (MDPS)
    assert abs(_tx_angle(sim) - 9.3) < 0.06 and abs(sim.s.tx_angle_last - 9.3) < 1e-9
    sim.step(v=v, wheel=9.0, cmd=9.0, tq=0.0, lat_active=True)                        # re-activate: continues from 9.3
    assert abs(_tx_angle(sim) - 9.3) <= _panda_delta(sim, v) + 1e-9

  def test_max_angle_bound_at_speed(self):
    v = 27.8; sim = Sim(); settle(sim)
    run_signal(sim, 100, v=v, wheel=0.0, cmd=0.0, tq=0.0)
    amax = get_max_angle_vm(v, sim.s.BASELINE_VM, sim.s.params)
    for _ in range(400):
      sim.step(v=v, wheel=0.0, cmd=40.0, tq=0.0)
    assert abs(_tx_angle(sim)) <= (int(amax * 10) - 1) / 10.0 + 1e-6   # one CAN unit inside the panda's max

  def test_kill_sends_internal_value(self):
    old = P.TX_GOVERNOR
    try:
      P.TX_GOVERNOR = False
      v = 14.7; sim = Sim(); settle(sim)
      run_signal(sim, 200, v=v, wheel=0.0, cmd=0.0, tq=0.0)
      sim.step(v=v, wheel=12.0, cmd=0.0, tq=460.0, pressed=True)
      assert abs(_tx_angle(sim) - sim.s.apply_angle_last) < 1e-6
    finally:
      P.TX_GOVERNOR = old


class TestPhase38EchoResync:
  def test_rejected_echo_realigns_wire_reference_to_measured(self):
    with no_outrun_passive():
      # 38-2: the panda resets its reference to the measured angle on a rejection; carstate
      # reports the src-192 echo and the controller must restart from the measurement
      v = 14.7; sim = Sim(); settle(sim)
      run_signal(sim, 200, v=v, wheel=0.0, cmd=0.0, tq=0.0)
      for _ in range(20):                                   # governor saturating: wire lags a 12 deg anchor
        sim.step(v=v, wheel=12.0, cmd=0.0, tq=460.0, pressed=True, mdps_angle_2=12.0)
      assert abs(sim.s.tx_angle_last - 12.0) > 2.0
      sim.step(v=v, wheel=12.0, cmd=0.0, tq=460.0, pressed=True, mdps_angle_2=12.0, tx_rejected=True)
      assert abs(_tx_angle(sim) - 12.0) <= _panda_delta(sim, v) + 1e-6   # continues from the measured angle


def _frame_active(sim):
  for m in sim.last_msgs:
    if isinstance(m, tuple) and m[0] == "LKAS_ALT":
      return int(m[2]["LKAS_ANGLE_ACTIVE"]) == 2, float(m[2]["ADAS_ACIAnglTqRedcGainVal"])
  raise AssertionError("no LKAS_ALT frame")


class TestPhase38_3WheelOutrunPassive:
  # route 00000005 seg 18: 91 km/h, driver pushes 190 Nm through an ALC, wheel 35 deg/s (0.35 deg/frame)
  # vs a panda allowance of ~0.2 deg/frame -> every other active frame rejected. The frame must go
  # passive (active bit 1, gain 0, angle = wheel) while the wheel outruns the allowance.
  def test_fast_driver_wheel_makes_passive_frames_then_active_resumes_from_the_wheel(self):
    v = 25.3; sim = Sim(); settle(sim, v=v)
    run_signal(sim, 200, v=v, wheel=8.0, cmd=8.0, tq=0.0, mdps_angle_2=8.0)
    active0, _ = _frame_active(sim); assert active0
    dmax = _panda_delta(sim, v); step = dmax + 0.3 + 0.05          # clearly above allowance + 2 CAN margin
    passive_frames = 0; w = 8.0
    for i in range(12):                                            # wheel yanked back by the driver
      w -= step
      sim.step(v=v, wheel=w, cmd=8.0, tq=190.0, pressed=False, mdps_angle_2=w)
      active, gain = _frame_active(sim)
      if not active:
        passive_frames += 1
        assert gain == 0.0
        assert abs(_tx_angle(sim) - w) < 0.11                      # passive frame carries the wheel
    assert passive_frames >= 10, passive_frames                    # from the 2nd fast frame on
    # wheel settles: 5 quiet frames later active resumes, and the first active frame starts from the wheel
    for i in range(4):
      sim.step(v=v, wheel=w, cmd=8.0, tq=0.0, mdps_angle_2=w)
      assert not _frame_active(sim)[0]
    sim.step(v=v, wheel=w, cmd=8.0, tq=0.0, mdps_angle_2=w)
    assert _frame_active(sim)[0]
    assert abs(_tx_angle(sim) - w) <= dmax + 1e-6                  # governor steps from the wheel, not from a stale reference

  def test_op_driven_motion_at_max_rate_stays_active(self):
    # op's own command moves the wheel at (just under) the allowance: never passive
    v = 25.3; sim = Sim(); settle(sim, v=v)
    dmax = _panda_delta(sim, v); w = 0.0
    for i in range(40):
      w += dmax
      sim.step(v=v, wheel=w, cmd=w + dmax, tq=0.0, mdps_angle_2=w)
      assert _frame_active(sim)[0]

  def test_kill_switch(self):
    old = P.WHEEL_OUTRUN_PASSIVE
    try:
      P.WHEEL_OUTRUN_PASSIVE = False
      v = 25.3; sim = Sim(); settle(sim, v=v); w = 8.0
      for i in range(8):
        w -= 1.0
        sim.step(v=v, wheel=w, cmd=8.0, tq=190.0, mdps_angle_2=w)
        assert _frame_active(sim)[0]
    finally:
      P.WHEEL_OUTRUN_PASSIVE = old

  def test_coalesced_can_samples_do_not_trigger(self):
    # review: card's frame may swallow two 100 Hz MDPS samples; the per-SAMPLE step (carstate) stays within
    # the allowance while the frame-differenced angle doubles. op steering at its own rate must stay active.
    v = 25.3; sim = Sim(); settle(sim, v=v)
    dmax = _panda_delta(sim, v); step_can = int(dmax * 10) - 1; w = 0.0
    for i in range(60):
      two = (i % 3 == 0)
      w += (2 if two else 1) * step_can / 10.0
      sim.step(v=v, wheel=w, cmd=w + dmax, tq=0.0, mdps_angle_2=w, mdps_step_can=step_can)
      assert _frame_active(sim)[0], i

  def test_single_over_threshold_frame_does_not_trigger(self):
    v = 25.3; sim = Sim(); settle(sim, v=v)
    run_signal(sim, 50, v=v, wheel=5.0, cmd=5.0, tq=0.0, mdps_angle_2=5.0)
    sim.step(v=v, wheel=3.0, cmd=5.0, tq=150.0, mdps_angle_2=3.0)        # one 2 deg jump
    assert _frame_active(sim)[0]
    for _ in range(10):
      sim.step(v=v, wheel=3.0, cmd=5.0, tq=0.0, mdps_angle_2=3.0)
      assert _frame_active(sim)[0]

  def test_dwell_cap_returns_active_from_the_wheel(self):
    v = 25.3; sim = Sim(); settle(sim, v=v)
    dmax = _panda_delta(sim, v); step = dmax + 0.35; w = 0.0; states = []
    for i in range(130):                                                  # the driver keeps winding for 1.3 s
      w += step
      sim.step(v=v, wheel=w, cmd=0.0, tq=200.0, mdps_angle_2=w)
      states.append(_frame_active(sim)[0])
    assert not states[5] and any(states[100:104]), states[95:110]         # passive by frame 5, active again by the cap
    i_act = next(i for i in range(60, 130) if states[i])
    assert abs(sim.s.tx_angle_last) > 0                                    # and the active frame stepped from the wheel, not from 0
    assert i_act >= 100

  def test_passive_frame_keeps_icons_and_blocks_camera_warning(self):
    import phase_tests.harness as H
    old = dict(H.CAM_MSG_TEMPLATE)
    try:
      H.CAM_MSG_TEMPLATE["LKA_WARNING"] = 1; H.CAM_MSG_TEMPLATE["FCA_SYSWARN"] = 1
      v = 25.3; sim = Sim(); settle(sim, v=v)
      run_signal(sim, 50, v=v, wheel=8.0, cmd=8.0, tq=0.0, mdps_angle_2=8.0)
      w = 8.0
      for _ in range(4):
        w -= _panda_delta(sim, v) + 0.35
        sim.step(v=v, wheel=w, cmd=8.0, tq=190.0, mdps_angle_2=w)
      active, gain = _frame_active(sim); assert not active and gain == 0.0
      frame = next(m[2] for m in sim.last_msgs if isinstance(m, tuple) and m[0] == "LKAS_ALT")
      assert int(frame["LKA_ICON"]) == 2 and int(frame["LKA_WARNING"]) == 0 and int(frame["FCA_SYSWARN"]) == 0 and int(frame["LKA_ASSIST"]) == 1
    finally:
      H.CAM_MSG_TEMPLATE.clear(); H.CAM_MSG_TEMPLATE.update(old)

  def test_gain_reramps_under_the_37a_cap_after_passive_exit(self):
    # review: with the wire gain at 0 during passive frames the internal gain must not keep ramping;
    # the first active frames after the exit must climb at <= ACIGAIN_RATE_UP_CAP per frame from 0
    v = 25.3; sim = Sim(); settle(sim, v=v)
    run_signal(sim, 300, v=v, wheel=8.0, cmd=8.0, tq=0.0, mdps_angle_2=8.0)
    assert _frame_active(sim)[1] > 0.8                             # wire gain (0..1 in the harness) high before the episode
    w = 8.0; step = _panda_delta(sim, v) + 0.35
    for _ in range(8):                                             # driver yanks: passive
      w -= step; sim.step(v=v, wheel=w, cmd=8.0, tq=190.0, mdps_angle_2=w)
    assert not _frame_active(sim)[0]
    gains = []
    for _ in range(40):                                            # wheel quiet: active resumes, gain re-ramps
      sim.step(v=v, wheel=w, cmd=w, tq=0.0, mdps_angle_2=w)
      active, g = _frame_active(sim)
      if active: gains.append(g)
    assert gains and gains[0] <= 0.05, gains[:3]                   # no step on the exit frame
    steps = [b - a for a, b in zip(gains, gains[1:])]
    assert max(steps) <= max(P.ACIGAIN_RATE_UP_CAP_V) + 1e-6, max(steps)
