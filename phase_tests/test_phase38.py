"""Phase 38: wire governor — the transmitted LKAS_ALT angle never exceeds the panda's VM per-frame
delta / max angle relative to the last transmitted value, even when op anchors apply_angle_last to the wheel."""
from phase_tests.harness import Sim, run_signal
from opendbc.car.hyundai.values import CarControllerParams as P
from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm


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
    assert 1 <= resyncs <= 2, resyncs                       # one realignment per RESYNC_FRAMES of saturation

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
