#!/usr/bin/env python3
import sys
import time
import signal
import struct
import threading
import requests
import urllib.parse
from datetime import datetime, UTC

from openpilot.cereal import log, messaging
from openpilot.common.api import Api
from openpilot.common.time_helpers import system_time_valid
from openpilot.common.params import Params
from openpilot.common.serial import Serial
from openpilot.common.swaglog import cloudlog
from openpilot.common.hardware import COMMA_HARDWARE
from openpilot.common.gpio import gpio_init, gpio_set
from openpilot.common.hardware.comma.pins import GPIO

UBLOX_TTY = "/dev/ttyHS0"

UBLOX_ACK = b"\xb5\x62\x05\x01\x02\x00"
UBLOX_NACK = b"\xb5\x62\x05\x00\x02\x00"
UBLOX_SOS_ACK = b"\xb5\x62\x09\x14\x08\x00\x02\x00\x00\x00\x01\x00\x00\x00"
UBLOX_SOS_NACK = b"\xb5\x62\x09\x14\x08\x00\x02\x00\x00\x00\x00\x00\x00\x00"
UBLOX_BACKUP_RESTORE_MSG = b"\xb5\x62\x09\x14\x08\x00\x03"
UBLOX_ASSIST_ACK = b"\xb5\x62\x13\x60\x08\x00"

def set_power(enabled: bool) -> None:
  gpio_init(GPIO.UBLOX_SAFEBOOT_N, True)
  gpio_init(GPIO.GNSS_PWR_EN, True)
  gpio_init(GPIO.UBLOX_RST_N, True)

  gpio_set(GPIO.UBLOX_SAFEBOOT_N, True)
  gpio_set(GPIO.GNSS_PWR_EN, enabled)
  gpio_set(GPIO.UBLOX_RST_N, enabled)

def add_ubx_checksum(msg: bytes) -> bytes:
  A = B = 0
  for b in msg[2:]:
    A = (A + b) % 256
    B = (B + A) % 256
  return msg + bytes([A, B])

def get_assistnow_messages() -> list[bytes]:
  params = Params()
  if token := params.get('AssistNowToken'):
    cloudlog.warning("Downloading AssistNow data directly from u-blox")
    r = requests.get("https://online-live2.services.u-blox.com/GetOnlineData.ashx", params=urllib.parse.urlencode({
      'token': token,
      'gnss': 'gps,glo',
      'datatype': 'eph,alm,aux',
    }, safe=':,'), timeout=5)
  elif dongle_id := params.get('DongleId'):
    cloudlog.warning("Downloading AssistNow data from comma's AGPS proxy")
    api = Api(dongle_id)
    r = api.get(f"v1/{dongle_id}/assist", access_token=api.get_token(), timeout=5)
  else:
    raise RuntimeError("Neither AssistNowToken nor DongleId is configured")

  r.raise_for_status()
  dat = r.content

  # split up messages
  msgs = []
  while len(dat) > 0:
    assert dat[:2] == b"\xB5\x62"
    msg_len = 6 + (dat[5] << 8 | dat[4]) + 2
    msgs.append(dat[:msg_len])
    dat = dat[msg_len:]
  return msgs


class TTYPigeon:
  def __init__(self):
    self.tty = Serial(UBLOX_TTY, baudrate=9600, timeout=0)

  def send(self, dat: bytes) -> None:
    self.tty.write(dat)

  def receive(self) -> bytes:
    dat = b''
    while len(dat) < 0x1000:
      d = self.tty.read(0x40)
      dat += d
      if len(d) == 0:
        break
    return dat

  def set_baud(self, baud: int) -> None:
    self.tty.baudrate = baud

  def wait_for_ack(self, ack: bytes = UBLOX_ACK, nack: bytes = UBLOX_NACK, timeout: float = 0.5) -> bool:
    dat = b''
    st = time.monotonic()
    while True:
      dat += self.receive()
      if ack in dat:
        cloudlog.debug("Received ACK from ublox")
        return True
      elif nack in dat:
        cloudlog.error("Received NACK from ublox")
        return False
      elif time.monotonic() - st > timeout:
        cloudlog.error("No response from ublox")
        raise TimeoutError('No response from ublox')
      time.sleep(0.001)

  def send_with_ack(self, dat: bytes, ack: bytes = UBLOX_ACK, nack: bytes = UBLOX_NACK) -> None:
    self.send(dat)
    self.wait_for_ack(ack, nack)

  def wait_for_backup_restore_status(self, timeout: float = 1.) -> int:
    dat = b''
    st = time.monotonic()
    while True:
      dat += self.receive()
      position = dat.find(UBLOX_BACKUP_RESTORE_MSG)
      if position >= 0 and len(dat) >= position + 11:
        return dat[position + 10]
      elif time.monotonic() - st > timeout:
        cloudlog.error("No backup restore response from ublox")
        raise TimeoutError('No response from ublox')
      time.sleep(0.001)

  def reset_device(self) -> bool:
    # deleting the backup does not always work on first try (mostly on second try)
    for _ in range(5):
      # device cold start
      self.send(b"\xb5\x62\x06\x04\x04\x00\xff\xff\x00\x00\x0c\x5d")
      time.sleep(1) # wait for cold start
      init_baudrate(self)

      # clear configuration
      self.send_with_ack(b"\xb5\x62\x06\x09\x0d\x00\x1f\x1f\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x17\x71\xd7")

      # clear flash memory (almanac backup)
      self.send_with_ack(b"\xB5\x62\x09\x14\x04\x00\x01\x00\x00\x00\x22\xf0")

      # try restoring backup to verify it got deleted
      self.send(b"\xB5\x62\x09\x14\x00\x00\x1D\x60")
      # 1: failed to restore, 2: could restore, 3: no backup
      status = self.wait_for_backup_restore_status()
      if status == 1 or status == 3:
        return True
    return False

def save_almanac(pigeon: TTYPigeon) -> None:
  # store almanac in flash
  pigeon.send(b"\xB5\x62\x09\x14\x04\x00\x00\x00\x00\x00\x21\xEC")
  try:
    if pigeon.wait_for_ack(ack=UBLOX_SOS_ACK, nack=UBLOX_SOS_NACK):
      cloudlog.info("Done storing almanac")
    else:
      cloudlog.error("Error storing almanac")
  except TimeoutError:
    pass

# i6n Phase 65 (report §78-79): the almanac backup never stored on this device — every 5-min UBX-UPD-SOS save while the
# receiver runs was NACKed (5-9 of 5-9 per drive), so each ignition (a cold boot, no network, clock at the OS build date)
# started the GNSS from nothing: first fix 3-7 min. u-blox expects the receiver stopped for the backup, so the backup is
# made the way the receiver accepts it: controlled GNSS stop (UBX-CFG-RST 0x08), create backup, controlled GNSS start
# (0x09; RAM data kept = hot start, ~1-2 s without GNSS — locationd keeps running on IMU / camera, GNSS is never used for
# steering). Once per boot, after ALMANAC_FIX_MIN_S of fix in total (a full almanac takes 12.5 min of tracking; tunnels
# only pause it), at a standstill — or anywhere once ALMANAC_FIX_ANYWAY_S of fix has passed without one. Retried after
# ALMANAC_RETRY_S, at most ALMANAC_MAX_TRIES per boot. Kill: ALMANAC_SAVE_STOPPED = False (periodic save while running).
ALMANAC_SAVE_STOPPED = True
ALMANAC_FIX_MIN_S = 12.5 * 60.    # total fix time before the almanac is complete enough to keep
ALMANAC_FIX_ANYWAY_S = 25 * 60.   # total fix time after which no standstill is waited for
ALMANAC_FIX_STALE_S = 2.          # fix time only counts while fix messages keep coming
ALMANAC_STANDSTILL_S = 2.         # stopped for this long (a red light) before the receiver is paused
ALMANAC_STANDSTILL_MS = 0.3
ALMANAC_RETRY_S = 5 * 60.
ALMANAC_MAX_TRIES = 3
ALMANAC_SOS_TIMEOUT_S = 2.        # flash write
UBLOX_GNSS_STOP = b"\xB5\x62\x06\x04\x04\x00\x00\x00\x08\x00\x16\x74"
UBLOX_GNSS_START = b"\xB5\x62\x06\x04\x04\x00\x00\x00\x09\x00\x17\x76"
UBLOX_SOS_CREATE = b"\xB5\x62\x09\x14\x04\x00\x00\x00\x00\x00\x21\xEC"


def save_almanac_stopped(pigeon) -> bool:
  """Phase 65: stop the receiver, store the almanac backup in flash, start the receiver again. Returns success."""
  ok = False
  pigeon.send(UBLOX_GNSS_STOP)
  time.sleep(0.1)
  pigeon.receive()   # drop what was in flight
  pigeon.send(UBLOX_SOS_CREATE)
  try:
    ok = pigeon.wait_for_ack(ack=UBLOX_SOS_ACK, nack=UBLOX_SOS_NACK, timeout=ALMANAC_SOS_TIMEOUT_S)
  except TimeoutError:
    ok = False
  finally:
    pigeon.send(UBLOX_GNSS_START)
  if ok:
    cloudlog.warning("Done storing almanac (receiver stopped)")
  else:
    cloudlog.error("Error storing almanac (receiver stopped)")
  return ok


class AlmanacSaver:
  """Phase 65: decides when to make the stopped-receiver backup (pure state machine, fed with monotonic time,
  GNSS fix and vehicle speed)."""
  def __init__(self):
    self.fix_total = 0.0
    self.last_fix_t: float | None = None
    self.has_fix = False
    self.still_since: float | None = None
    self.last_try_t: float | None = None
    self.tries = 0
    self.saved = False

  def update(self, now: float, has_fix: bool | None, v_ego: float | None) -> bool:
    """Feed the latest state (None = no new message); returns True when a backup should be made now."""
    if has_fix is not None:
      if has_fix and self.has_fix and self.last_fix_t is not None and now - self.last_fix_t <= ALMANAC_FIX_STALE_S:
        self.fix_total += now - self.last_fix_t
      self.has_fix = has_fix
      self.last_fix_t = now
    elif self.last_fix_t is not None and now - self.last_fix_t > ALMANAC_FIX_STALE_S:
      self.has_fix = False
    if v_ego is not None:
      if v_ego < ALMANAC_STANDSTILL_MS:
        self.still_since = now if self.still_since is None else self.still_since
      else:
        self.still_since = None
    if self.saved or self.tries >= ALMANAC_MAX_TRIES or not self.has_fix or self.fix_total < ALMANAC_FIX_MIN_S:
      return False
    still = self.still_since is not None and now - self.still_since >= ALMANAC_STANDSTILL_S
    spaced = self.last_try_t is None or now - self.last_try_t >= ALMANAC_RETRY_S
    return spaced and (still or self.fix_total >= ALMANAC_FIX_ANYWAY_S)

  def done(self, now: float, ok: bool) -> None:
    self.tries += 1
    self.last_try_t = now
    self.saved = ok


def init_baudrate(pigeon: TTYPigeon):
  # ublox default setting on startup is 9600 baudrate
  pigeon.set_baud(9600)

  # $PUBX,41,1,0007,0003,460800,0*15\r\n
  pigeon.send(b"\x24\x50\x55\x42\x58\x2C\x34\x31\x2C\x31\x2C\x30\x30\x30\x37\x2C\x30\x30\x30\x33\x2C\x34\x36\x30\x38\x30\x30\x2C\x30\x2A\x31\x35\x0D\x0A")
  time.sleep(0.1)
  pigeon.set_baud(460800)


def init_pigeon(pigeon: TTYPigeon) -> bool:
  # try initializing a few times
  for _ in range(10):
    try:

      # setup port config
      pigeon.send_with_ack(b"\xb5\x62\x06\x00\x14\x00\x03\xFF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\x00\x01\x00\x00\x00\x00\x00\x1E\x7F")
      pigeon.send_with_ack(b"\xb5\x62\x06\x00\x14\x00\x00\xFF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x19\x35")
      pigeon.send_with_ack(b"\xb5\x62\x06\x00\x14\x00\x01\x00\x00\x00\xC0\x08\x00\x00\x00\x08\x07\x00\x01\x00\x01\x00\x00\x00\x00\x00\xF4\x80")
      pigeon.send_with_ack(b"\xb5\x62\x06\x00\x14\x00\x04\xFF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x1D\x85")
      pigeon.send_with_ack(b"\xb5\x62\x06\x00\x00\x00\x06\x18")
      pigeon.send_with_ack(b"\xb5\x62\x06\x00\x01\x00\x01\x08\x22")
      pigeon.send_with_ack(b"\xb5\x62\x06\x00\x01\x00\x03\x0A\x24")

      # UBX-CFG-RATE (0x06 0x08)
      pigeon.send_with_ack(b"\xB5\x62\x06\x08\x06\x00\x64\x00\x01\x00\x00\x00\x79\x10")

      # UBX-CFG-NAV5 (0x06 0x24)
      pigeon.send_with_ack(b"\xB5\x62\x06\x24\x24\x00\x05\x00\x04\x03\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x5A\x63")

      # UBX-CFG-ODO (0x06 0x1E)
      pigeon.send_with_ack(b"\xB5\x62\x06\x1E\x14\x00\x00\x00\x00\x00\x01\x03\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x3C\x37")
      pigeon.send_with_ack(b"\xB5\x62\x06\x39\x08\x00\xFF\xAD\x62\xAD\x1E\x63\x00\x00\x83\x0C")
      pigeon.send_with_ack(b"\xB5\x62\x06\x23\x28\x00\x00\x00\x00\x04\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x56\x24")

      # UBX-CFG-NAV5 (0x06 0x24)
      pigeon.send_with_ack(b"\xB5\x62\x06\x24\x00\x00\x2A\x84")
      pigeon.send_with_ack(b"\xB5\x62\x06\x23\x00\x00\x29\x81")
      pigeon.send_with_ack(b"\xB5\x62\x06\x1E\x00\x00\x24\x72")
      pigeon.send_with_ack(b"\xB5\x62\x06\x39\x00\x00\x3F\xC3")

      # UBX-CFG-MSG (set message rate)
      pigeon.send_with_ack(b"\xB5\x62\x06\x01\x03\x00\x01\x07\x01\x13\x51")
      pigeon.send_with_ack(b"\xB5\x62\x06\x01\x03\x00\x02\x15\x01\x22\x70")
      pigeon.send_with_ack(b"\xB5\x62\x06\x01\x03\x00\x02\x13\x01\x20\x6C")
      pigeon.send_with_ack(b"\xB5\x62\x06\x01\x03\x00\x0A\x09\x01\x1E\x70")
      pigeon.send_with_ack(b"\xB5\x62\x06\x01\x03\x00\x0A\x0B\x01\x20\x74")
      pigeon.send_with_ack(b"\xB5\x62\x06\x01\x03\x00\x01\x35\x01\x41\xAD")
      cloudlog.debug("pigeon configured")

      # try restoring almanac backup
      pigeon.send(b"\xB5\x62\x09\x14\x00\x00\x1D\x60")
      restore_status = pigeon.wait_for_backup_restore_status()
      if restore_status == 2:
        cloudlog.warning("almanac backup restored")
      elif restore_status == 3:
        cloudlog.warning("no almanac backup found")
      else:
        cloudlog.error(f"failed to restore almanac backup, status: {restore_status}")

      # sending time to ublox
      if system_time_valid():
        t_now = datetime.now(UTC).replace(tzinfo=None)
        cloudlog.warning("Sending current time to ublox")

        # UBX-MGA-INI-TIME_UTC
        msg = add_ubx_checksum(b"\xB5\x62\x13\x40\x18\x00" + struct.pack("<BBBBHBBBBBxIHxxI",
          0x10,
          0x00,
          0x00,
          0x80,
          t_now.year,
          t_now.month,
          t_now.day,
          t_now.hour,
          t_now.minute,
          t_now.second,
          0,
          30,
          0
        ))
        pigeon.send_with_ack(msg, ack=UBLOX_ASSIST_ACK)

      cloudlog.warning("Pigeon GPS on!")
      break
    except TimeoutError:
      cloudlog.warning("Initialization failed, trying again!")
  else:
    cloudlog.warning("Failed to initialize pigeon")
    return False
  return True

def deinitialize_and_exit(pigeon: TTYPigeon | None):
  if pigeon is not None:
    # controlled GNSS stop
    pigeon.send(b"\xB5\x62\x06\x04\x04\x00\x00\x00\x08\x00\x16\x74")

  # turn off power and exit cleanly
  set_power(False)
  sys.exit(0)

def init(pigeon: TTYPigeon) -> None:
  # register exit handler
  signal.signal(signal.SIGINT, lambda sig, frame: deinitialize_and_exit(pigeon))

  # power cycle ublox
  set_power(False)
  time.sleep(0.1)
  set_power(True)
  time.sleep(0.5)

  init_baudrate(pigeon)
  init_pigeon(pigeon)

def run_receiving(duration: int = 0):
  pm = messaging.PubMaster(['ubloxRaw'])

  pigeon = TTYPigeon()
  init(pigeon)

  start_time = time.monotonic()
  last_almanac_save = time.monotonic()
  almanac = AlmanacSaver()                                          # Phase 65
  sm = messaging.SubMaster(['gpsLocationExternal', 'carState']) if ALMANAC_SAVE_STOPPED else None
  last_sm_t = 0.0
  assist_attempted = False
  assist_messages = None

  def download_assistnow() -> None:
    nonlocal assist_messages
    sm = messaging.SubMaster(['deviceState'])
    while assist_messages is None:
      sm.update(1000)
      if system_time_valid() and sm['deviceState'].networkType != log.DeviceState.NetworkType.none:
        try:
          assist_messages = get_assistnow_messages()
        except Exception:
          cloudlog.warning("failed to get AssistNow messages")
      time.sleep(10.)
  threading.Thread(target=download_assistnow, daemon=True).start()

  while (duration == 0) or (time.monotonic() - start_time < duration):
    if assist_messages is not None and not assist_attempted:
      assist_attempted = True
      try:
        for msg in assist_messages:
          pigeon.send_with_ack(msg, ack=UBLOX_ASSIST_ACK)
        cloudlog.warning("AssistNow messages sent")
      except Exception:
        cloudlog.warning("failed to send AssistNow messages")

    dat = pigeon.receive()
    if len(dat) > 0:
      if dat[0] == 0x00:
        cloudlog.warning("received invalid data from ublox, re-initing!")
        init(pigeon)
        assist_attempted = False
        continue

      # send out to socket
      msg = messaging.new_message('ubloxRaw', len(dat), valid=True)
      msg.ubloxRaw = dat[:]
      pm.send('ubloxRaw', msg)

      if ALMANAC_SAVE_STOPPED:
        # Phase 65: one stopped-receiver backup per boot, at a standstill after a long continuous fix
        now = time.monotonic()
        if now - last_sm_t >= 0.25:
          last_sm_t = now
          sm.update(0)
          has_fix = sm['gpsLocationExternal'].hasFix if sm.updated['gpsLocationExternal'] else None
          v_ego = sm['carState'].vEgo if sm.updated['carState'] else None
          if almanac.update(now, has_fix, v_ego):
            almanac.done(now, save_almanac_stopped(pigeon))
      # save almanac every 5 minutes (pre-Phase-65 path)
      elif (time.monotonic() - last_almanac_save) > 60*5:
        save_almanac(pigeon)
        last_almanac_save = time.monotonic()
    else:
      # prevent locking up a CPU core if ublox disconnects
      time.sleep(0.001)


def main():
  assert COMMA_HARDWARE, "unsupported hardware for pigeond"
  run_receiving()

if __name__ == "__main__":
  main()
