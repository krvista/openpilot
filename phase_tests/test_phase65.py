"""Phase 65 (report §78-79): GNSS almanac backup made with the receiver stopped. pigeond's periodic UBX-UPD-SOS backup
while running was NACKed on every try, so every ignition started the GNSS from nothing (first fix 3-7 min). Now, once per
boot, after ALMANAC_FIX_MIN_S of fix in total, at a standstill (or anywhere after ALMANAC_FIX_ANYWAY_S of fix): controlled
GNSS stop -> create backup -> controlled GNSS start; retried after ALMANAC_RETRY_S up to ALMANAC_MAX_TRIES."""
import pytest

from phase_tests.harness_noncontrol import FakeParams  # noqa: F401 (stub install side effect)

import openpilot.system.ubloxd.pigeond as pg

FIX = pg.ALMANAC_FIX_MIN_S


def test_messages_are_valid_ubx():
  for msg in (pg.UBLOX_GNSS_STOP, pg.UBLOX_GNSS_START, pg.UBLOX_SOS_CREATE):
    assert pg.add_ubx_checksum(msg[:-2]) == msg
  assert pg.UBLOX_GNSS_STOP[6:10] == b"\x00\x00\x08\x00"     # CFG-RST hot, controlled GNSS stop
  assert pg.UBLOX_GNSS_START[6:10] == b"\x00\x00\x09\x00"    # CFG-RST hot, controlled GNSS start
  assert pg.ALMANAC_SAVE_STOPPED is True


def run(saver, t_end, fix=lambda t: True, v=lambda t: 0.0, dt=0.25, t0=0.0):
  """Feed the saver every dt; returns the times it asked for a backup (marks each as failed unless told otherwise)."""
  asks = []
  t = t0
  while t <= t_end:
    if saver.update(t, fix(t), v(t)):
      asks.append(t)
      saver.done(t, False)
    t += dt
  return asks


class TestWhen:
  def test_not_before_a_long_fix(self):
    assert run(pg.AlmanacSaver(), FIX - 1.0) == []

  def test_at_a_standstill_after_the_fix_time(self):
    asks = run(pg.AlmanacSaver(), FIX + 10.0)
    assert asks and asks[0] == pytest.approx(FIX, abs=0.5)

  def test_moving_waits_for_a_stop_then_goes_anyway(self):
    asks = run(pg.AlmanacSaver(), pg.ALMANAC_FIX_ANYWAY_S + 10.0, v=lambda t: 20.0)
    assert asks[0] == pytest.approx(pg.ALMANAC_FIX_ANYWAY_S, abs=0.5)

  def test_waits_for_the_standstill_to_settle(self):
    still_from = FIX + 100.0
    asks = run(pg.AlmanacSaver(), FIX + 200.0, v=lambda t: 0.0 if t >= still_from else 15.0)
    assert asks[0] == pytest.approx(still_from + pg.ALMANAC_STANDSTILL_S, abs=0.3)

  def test_fix_time_adds_up_across_gaps(self):
    gap = (600.0, 700.0)                                        # a 100 s tunnel pauses the count, does not restart it
    asks = run(pg.AlmanacSaver(), FIX + 200.0, fix=lambda t: not (gap[0] <= t < gap[1]))
    assert asks[0] == pytest.approx(FIX + (gap[1] - gap[0]), abs=0.5)

  def test_not_without_a_fix_now(self):
    asks = run(pg.AlmanacSaver(), FIX + 100.0, fix=lambda t: t < FIX + 1.0)
    assert asks == [] or asks[0] <= FIX + 1.0

  def test_missing_fix_messages_do_not_count(self):
    s = pg.AlmanacSaver()
    s.update(0.0, True, 0.0)
    assert s.update(FIX + 10.0, True, 0.0) is False             # one message 13 min later is not 13 min of fix
    assert s.fix_total == 0.0

  def test_retry_spacing_and_cap(self):
    asks = run(pg.AlmanacSaver(), FIX + 60 * 60.0)
    assert len(asks) == pg.ALMANAC_MAX_TRIES
    assert all(b - a >= pg.ALMANAC_RETRY_S - 0.3 for a, b in zip(asks, asks[1:]))

  def test_once_per_boot_after_success(self):
    s = pg.AlmanacSaver()
    t = 0.0
    while not s.update(t, True, 0.0):
      t += 0.25
    s.done(t, True)
    assert run(s, t + 3600.0, t0=t + 0.25) == []


class FakePigeon(pg.TTYPigeon):
  def __init__(self, reply):
    self.sent = []
    self.reply = reply
    self.pending = b""

  def send(self, dat):
    self.sent.append(dat)
    if dat == pg.UBLOX_SOS_CREATE and self.reply is not None:
      self.pending = self.reply

  def receive(self):
    d, self.pending = self.pending, b""
    return d


@pytest.fixture
def fast_timeout():
  old = pg.ALMANAC_SOS_TIMEOUT_S
  pg.ALMANAC_SOS_TIMEOUT_S = 0.05
  yield
  pg.ALMANAC_SOS_TIMEOUT_S = old


class TestSave:
  @pytest.mark.parametrize("reply, ok", [(pg.UBLOX_SOS_ACK, True), (pg.UBLOX_SOS_NACK, False), (None, False)])
  def test_stop_backup_start_in_order(self, fast_timeout, reply, ok):
    p = FakePigeon(reply)
    assert pg.save_almanac_stopped(p) is ok
    assert p.sent == [pg.UBLOX_GNSS_STOP, pg.UBLOX_SOS_CREATE, pg.UBLOX_GNSS_START]   # always restarted
