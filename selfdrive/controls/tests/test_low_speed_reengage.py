from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.low_speed_reengage import LowSpeedReengage, PARAM, read_toggle

WK2 = SimpleNamespace(brand='chrysler', minSteerSpeed=17.5)


def run(lsr, v, frames=1, lat=None, fault=False):
  """Advance like controlsd: latActive = speed gate (unless forced) and no fault."""
  out = None
  for _ in range(frames):
    allowed = v > lsr.min_lat_speed(v)
    out = (allowed and not fault) if lat is None else lat
    lsr.update(out, v, fault)
  return out


class TestLowSpeedReengage:
  def test_off_is_unchanged(self):
    lsr = LowSpeedReengage(WK2, enabled=False)
    assert not run(lsr, 16.0)            # no fresh entry below minSteerSpeed
    assert run(lsr, 17.6)                # entry above it
    assert run(lsr, 14.6)                # hold down to -3.0
    assert not run(lsr, 14.4)            # dropped below the hold floor
    assert not run(lsr, 16.0)            # and stays off until minSteerSpeed again
    assert run(lsr, 17.6)

  def test_on_reengages_from_entry_margin(self):
    lsr = LowSpeedReengage(WK2, enabled=True)
    assert not run(lsr, 14.9)            # below -2.5: still off
    assert run(lsr, 15.1)                # above -2.5: fresh engagement
    assert run(lsr, 14.6)                # then the -3.0 hold applies
    assert not run(lsr, 14.4)

  def test_fault_after_low_entry_locks_out_until_min_steer_speed(self):
    lsr = LowSpeedReengage(WK2, enabled=True)
    assert run(lsr, 15.5)                # low-speed entry
    run(lsr, 15.5, frames=50)
    assert not run(lsr, 15.5, fault=True)  # EPS refuses within the window
    run(lsr, 15.5, frames=300)           # fault clears
    assert not run(lsr, 15.5)            # locked out
    assert not run(lsr, 17.0)
    assert run(lsr, 17.6)                # above minSteerSpeed: normal entry, lockout cleared
    run(lsr, 14.0)                       # dip below the hold floor
    assert run(lsr, 15.5)                # allowed to try low again

  def test_late_fault_does_not_lock_out(self):
    lsr = LowSpeedReengage(WK2, enabled=True)
    assert run(lsr, 15.5)
    run(lsr, 15.5, frames=400)           # 4 s of accepted torque
    run(lsr, 15.5, fault=True)           # an unrelated fault later (e.g. override)
    run(lsr, 15.5, frames=10)
    assert run(lsr, 15.5)

  def test_other_brands_untouched(self):
    lsr = LowSpeedReengage(SimpleNamespace(brand='toyota', minSteerSpeed=0.), enabled=True)
    assert lsr.min_lat_speed(5.) == 0.3

  def test_toggle_file(self, tmp_path):
    class P:
      def get_param_path(self, key=""):
        return str(tmp_path / key)
    assert read_toggle(P())              # missing = on
    (tmp_path / PARAM).write_text("1")
    assert read_toggle(P())
    (tmp_path / PARAM).write_text("0")
    assert not read_toggle(P())

  def test_default_on(self):
    lsr = LowSpeedReengage(WK2)
    assert run(lsr, 15.1)
