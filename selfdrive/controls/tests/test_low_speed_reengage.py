import json
from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.low_speed_reengage import LowSpeedReengage, PARAM, STEP, GAP, ABS_FLOOR, CONFIRMS, MAX_FAILS, \
                                                                 ENTRY_LOCKOUT_WAIT, read_toggle

WK2 = SimpleNamespace(brand='chrysler', minSteerSpeed=17.5)
HOLD0, ENTRY0 = 17.5 - 3.9, 17.5 - 3.3   # 49.0 / 51.1 km/h
GAIN = 255.


def run(lsr, v, frames=1, lat=None, fault=False, torque=0.2, gain=GAIN, pressed=False, drv=0.):
  """Advance like controlsd: latActive = speed gate (unless forced) and no fault; the EPS motor follows with `gain`."""
  out = None
  for _ in range(frames):
    allowed = v > lsr.min_lat_speed(v)
    out = (allowed and not fault) if lat is None else lat
    lsr.update(out, v, fault, pressed, torque if out else 0., gain * torque if out else 0., drv)
  return out


def dip(lsr, v, frames=60, **kw):
  """Engage above minSteerSpeed, hold at v for a while, then drop out well below everything."""
  assert run(lsr, 17.6)
  held = run(lsr, v, frames=frames, **kw)
  run(lsr, 3.0, frames=150)
  return held


def low_entry(lsr, v, **kw):
  """Fresh engagement at v from below, held past the fault window, then back out."""
  run(lsr, 3.0, frames=150)
  entered = run(lsr, v, **kw)
  run(lsr, v, frames=350, **kw)
  run(lsr, 3.0, frames=150)
  return entered


class TestLowSpeedReengage:
  def test_off_is_the_old_fixed_window(self):
    lsr = LowSpeedReengage(WK2, enabled=False)
    assert not run(lsr, 16.0)            # no fresh entry below minSteerSpeed
    assert run(lsr, 17.6)
    assert run(lsr, 14.6)                # hold down to -3.0
    assert not run(lsr, 14.4)
    assert not run(lsr, 16.0)
    assert (lsr.hold, lsr.entry) == (HOLD0, ENTRY0)  # and nothing is learned

  def test_starts_one_step_below_the_confirmed_speeds(self):
    lsr = LowSpeedReengage(WK2)
    assert abs(lsr.hold_floor() - (HOLD0 - STEP)) < 1e-6
    # the entry trial may not come closer than GAP to the confirmed hold floor: 51.1 - 1 < 49.0 + 2
    assert abs(lsr.entry_floor() - ENTRY0) < 1e-6
    assert not run(lsr, ENTRY0 - 0.05)
    assert run(lsr, ENTRY0 + 0.05)
    assert run(lsr, HOLD0 - STEP + 0.05)
    assert not run(lsr, HOLD0 - STEP - 0.05)

  def test_hold_lowers_after_clean_dips_with_evidence(self):
    lsr = LowSpeedReengage(WK2)
    trial = HOLD0 - STEP + 0.05
    for _ in range(CONFIRMS):
      assert abs(lsr.hold - HOLD0) < 1e-6
      assert dip(lsr, trial)
    assert abs(lsr.hold - (HOLD0 - STEP)) < 1e-6       # confirmed one step lower
    assert abs(lsr.hold_floor() - (HOLD0 - 2 * STEP)) < 1e-6  # and the next trial is open
    # now the entry trial has room: confirmed hold + GAP <= entry - STEP
    assert abs(lsr.entry_floor() - (ENTRY0 - STEP)) < 1e-6

  def test_hold_needs_torque_evidence(self):
    lsr = LowSpeedReengage(WK2)
    for _ in range(CONFIRMS + 3):
      dip(lsr, HOLD0 - STEP + 0.05, torque=0.01)       # too little torque to tell whether the EPS follows
    assert abs(lsr.hold - HOLD0) < 1e-6
    dip(lsr, HOLD0 - STEP + 0.05, frames=120)
    assert abs(lsr.hold - (HOLD0 - STEP)) < 1e-6

  def test_hold_fault_on_trial_returns_to_confirmed_then_stops(self):
    lsr = LowSpeedReengage(WK2)
    trial = HOLD0 - STEP + 0.05
    assert run(lsr, 17.6)
    assert run(lsr, trial, frames=20)
    assert not run(lsr, trial, fault=True)
    run(lsr, trial, frames=300)
    assert lsr.hold_fails == 1 and abs(lsr.hold - HOLD0) < 1e-6
    assert abs(lsr.hold_floor() - HOLD0) < 1e-6        # no retry in this drive
    assert run(lsr, 17.6) and run(lsr, HOLD0 + 0.05) and not run(lsr, trial)
    # next drive retries once more; a second failure stops the search for good
    lsr2 = LowSpeedReengage(WK2)
    lsr2.hold_fails = 1
    lsr2.hold_trial_ok = True
    run(lsr2, 17.6)
    run(lsr2, trial, frames=20)
    run(lsr2, trial, fault=True)
    assert lsr2.hold_fails == MAX_FAILS and not lsr2.hold_trial_ok

  def test_silent_refusal_fails_the_trial(self):
    lsr = LowSpeedReengage(WK2)
    for _ in range(CONFIRMS):
      dip(lsr, HOLD0 - STEP + 0.05, gain=20.)          # no fault, but the motor does not follow
    assert abs(lsr.hold - HOLD0) < 1e-6 and lsr.hold_fails == 1 and not lsr.hold_trial_ok

  def test_fault_on_confirmed_hold_steps_back_up(self):
    lsr = LowSpeedReengage(WK2)
    lsr.hold_trial_ok = False                          # holding exactly at the confirmed floor
    run(lsr, 17.6)
    run(lsr, HOLD0 + 0.1, frames=50)
    run(lsr, HOLD0 + 0.1, fault=True)
    assert abs(lsr.hold - (HOLD0 + STEP)) < 1e-6 and lsr.hold_fails == MAX_FAILS
    assert lsr.entry >= lsr.hold + GAP - 1e-6
    # not when the driver was overriding, and not well above the floor
    for kw, v in (({'pressed': True}, HOLD0 + 0.1), ({}, HOLD0 + 2.0)):
      lsr = LowSpeedReengage(WK2)
      lsr.hold_trial_ok = False
      run(lsr, 17.6)
      run(lsr, v, frames=50, **kw)
      run(lsr, v, fault=True, **kw)
      assert abs(lsr.hold - HOLD0) < 1e-6 and lsr.hold_fails == 0

  def test_entry_lowers_and_fails_like_the_hold(self):
    lsr = LowSpeedReengage(WK2)
    lsr.hold = HOLD0 - 3 * STEP                        # room for entry trials
    trial = ENTRY0 - STEP + 0.05
    for _ in range(CONFIRMS):
      assert abs(lsr.entry - ENTRY0) < 1e-6
      assert low_entry(lsr, trial)
    assert abs(lsr.entry - (ENTRY0 - STEP)) < 1e-6
    # refused on the next trial: back to the confirmed entry for this drive
    trial2 = ENTRY0 - 2 * STEP + 0.05
    run(lsr, 3.0, frames=150)
    assert run(lsr, trial2)
    run(lsr, trial2, frames=50)
    assert not run(lsr, trial2, fault=True)
    run(lsr, 3.0, frames=400)
    assert lsr.entry_fails == 1 and abs(lsr.entry - (ENTRY0 - STEP)) < 1e-6
    assert not run(lsr, trial2) and run(lsr, ENTRY0 - STEP + 0.05)
    # a fault on the confirmed entry raises it and stops the search
    run(lsr, 3.0, frames=150)
    run(lsr, ENTRY0 - STEP + 0.05, frames=50)
    run(lsr, ENTRY0 - STEP + 0.05, fault=True)
    assert abs(lsr.entry - ENTRY0) < 1e-6 and lsr.entry_fails == MAX_FAILS

  def test_entry_above_min_steer_speed_and_late_faults_do_not_count(self):
    lsr = LowSpeedReengage(WK2)
    run(lsr, 17.6)
    run(lsr, 16.0, frames=400)
    run(lsr, 16.0, fault=True)                         # holding well above the floor, long after the entry
    assert (lsr.hold_fails, lsr.entry_fails) == (0, 0) and abs(lsr.hold - HOLD0) < 1e-6

  def test_entry_fault_far_above_the_entry_or_by_the_driver_is_no_verdict(self):
    # a fault soon after re-engaging at 55-63 km/h, or one the driver caused, says nothing about the confirmed entry
    for v, pressed in ((61. / 3.6, False), (61. / 3.6, True), (ENTRY0 + 0.1, True)):
      lsr = LowSpeedReengage(WK2)
      for _ in range(6):
        run(lsr, 3.0, frames=150)
        run(lsr, v, frames=100, pressed=pressed)
        run(lsr, v, fault=True, pressed=pressed)
      assert abs(lsr.entry - ENTRY0) < 1e-6 and lsr.entry_fails == 0
    # an unpressed fault right at the confirmed entry still raises it
    lsr = LowSpeedReengage(WK2)
    run(lsr, 3.0, frames=150)
    run(lsr, ENTRY0 + 0.1, frames=50)
    run(lsr, ENTRY0 + 0.1, fault=True)
    assert abs(lsr.entry - (ENTRY0 + STEP)) < 1e-6 and lsr.entry_fails == MAX_FAILS

  def test_trial_faults_count_even_with_the_driver_on_the_wheel(self):
    lsr = LowSpeedReengage(WK2)
    lsr.hold = HOLD0 - 3 * STEP                        # room for entry trials
    trial = ENTRY0 - STEP + 0.05
    run(lsr, 3.0, frames=150)
    assert run(lsr, trial)
    run(lsr, trial, frames=50, pressed=True)
    run(lsr, trial, fault=True, pressed=True)
    assert lsr.entry_fails == 1 and not lsr.entry_trial_ok and abs(lsr.entry - ENTRY0) < 1e-6
    lsr = LowSpeedReengage(WK2)
    htrial = HOLD0 - STEP + 0.05
    run(lsr, 17.6)
    run(lsr, htrial, frames=20, pressed=True)
    run(lsr, htrial, fault=True, pressed=True)
    assert lsr.hold_fails == 1 and not lsr.hold_trial_ok and abs(lsr.hold - HOLD0) < 1e-6

  def test_fault_in_an_entry_window_below_the_hold_floor_fails_the_hold_trial(self):
    # re-engage a little above the entry, slow into the hold trial band within the entry's fault window, fault
    lsr = LowSpeedReengage(WK2)
    v = ENTRY0 + 1.3 / 3.6
    run(lsr, 3.0, frames=150)
    assert run(lsr, v, frames=20)
    while v > HOLD0 - STEP + 0.1:
      v -= 2. / 3.6 * 0.01                             # 2 km/h per s
      run(lsr, v)
    assert lsr.entry_timer > 0. and lsr.visit_frames > 0
    run(lsr, v, fault=True)
    run(lsr, 3.0, frames=150)
    # a failed hold trial, not a clean use of it (and the entry, 1.3 km/h above its confirmed value, is no verdict)
    assert lsr.hold_fails == 1 and lsr.hold_ok == 0 and abs(lsr.hold - HOLD0) < 1e-6
    assert lsr.entry_fails == 0 and abs(lsr.entry - ENTRY0) < 1e-6

  def test_entry_counts_from_the_first_applied_torque(self):
    # the carcontroller holds the LKAS bit down for 2 s after it fell: the EPS sees the entry where torque starts
    lsr = LowSpeedReengage(WK2)
    lsr.hold = HOLD0 - 3 * STEP
    trial = ENTRY0 - STEP + 0.05
    for _ in range(CONFIRMS):
      run(lsr, 3.0, frames=150)
      assert run(lsr, trial)
      lsr_v = trial
      for _ in range(150):                             # locked out while accelerating 0.5 km/h per 0.1 s
        lsr_v += 0.5 / 3.6 / 10
        lsr.update(True, lsr_v, False, False, 0., 0., 0.)
      assert lsr.entry_speed > ENTRY0                  # no longer a trial entry
      run(lsr, lsr_v, frames=350)
    assert abs(lsr.entry - ENTRY0) < 1e-6 and lsr.entry_ok == 0
    # but the wait is bounded: a request that stays at zero torque does not hold the verdict off forever
    lsr = LowSpeedReengage(WK2)
    run(lsr, 3.0, frames=150)
    assert run(lsr, ENTRY0 + 0.1)
    for _ in range(int(ENTRY_LOCKOUT_WAIT * 100) + 5):
      lsr.update(True, ENTRY0 + 0.1, False, False, 0., 0., 0.)
    assert lsr.entry_torque_seen and lsr.entry_timer > 0.

  def test_moderate_hand_torque_still_counts_as_evidence(self):
    lsr = LowSpeedReengage(WK2)
    for _ in range(CONFIRMS):
      dip(lsr, HOLD0 - STEP + 0.05, frames=60, drv=-60.)  # resting against the request
    assert abs(lsr.hold - (HOLD0 - STEP)) < 1e-6
    # hand torque with the request (the motor assists it) or a driver steering hard is no evidence
    for drv in (60., -100.):
      lsr = LowSpeedReengage(WK2)
      for _ in range(CONFIRMS + 2):
        dip(lsr, HOLD0 - STEP + 0.05, frames=60, drv=drv)
      assert abs(lsr.hold - HOLD0) < 1e-6

  def test_never_below_abs_floor(self):
    lsr = LowSpeedReengage(WK2)
    lsr.hold = ABS_FLOOR
    assert lsr.hold_floor() == ABS_FLOOR

  def test_state_persists(self, tmp_path):
    path = str(tmp_path / "state.json")
    lsr = LowSpeedReengage(WK2, state_path=path)
    for _ in range(CONFIRMS):
      dip(lsr, HOLD0 - STEP + 0.05)
    dip(lsr, HOLD0 - 2 * STEP + 0.05)                  # one clean use of the next trial
    lsr2 = LowSpeedReengage(WK2, state_path=path)
    assert abs(lsr2.hold - (HOLD0 - STEP)) < 1e-6 and lsr2.hold_ok == 1 and lsr2.hold_ev.n > 0
    assert json.load(open(path))["hold_kph"] == round((HOLD0 - STEP) * 3.6, 1)
    # a file for another car or a corrupt one is ignored
    d = json.load(open(path))
    d["min_steer_speed"] = 14.5
    json.dump(d, open(path, "w"))
    assert abs(LowSpeedReengage(WK2, state_path=path).hold - HOLD0) < 1e-6
    open(path, "w").write("{")
    assert abs(LowSpeedReengage(WK2, state_path=path).hold - HOLD0) < 1e-6

  def test_torque_ramp_after_low_entry(self):
    lsr = LowSpeedReengage(WK2)
    assert lsr.torque_scale == 1.
    run(lsr, 15.5)                       # low-speed entry
    assert lsr.torque_scale < 0.05
    run(lsr, 15.5, frames=49)
    assert abs(lsr.torque_scale - 0.5) < 0.02
    run(lsr, 15.5, frames=60)
    assert lsr.torque_scale == 1.
    # entry above minSteerSpeed is not ramped, and the hold below it keeps full torque
    lsr = LowSpeedReengage(WK2)
    run(lsr, 17.6)
    assert lsr.torque_scale == 1.
    run(lsr, 15.0, frames=10)
    assert lsr.torque_scale == 1.
    # dropping out mid-ramp resets it
    lsr = LowSpeedReengage(WK2)
    run(lsr, 15.5, frames=20)
    run(lsr, 3.0)
    assert lsr.torque_scale == 1.

  def test_other_brands_untouched(self):
    lsr = LowSpeedReengage(SimpleNamespace(brand='toyota', minSteerSpeed=0.), enabled=True)
    assert lsr.min_lat_speed(5.) == 0.3
    lsr.update(True, 5., True)
    assert lsr.torque_scale == 1.

  def test_toggle_file(self, tmp_path):
    class P:
      def get_param_path(self, key=""):
        return str(tmp_path / key)
    assert read_toggle(P())              # missing = on
    (tmp_path / PARAM).write_text("1")
    assert read_toggle(P())
    (tmp_path / PARAM).write_text("0")
    assert not read_toggle(P())
