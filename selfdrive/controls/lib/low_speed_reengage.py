import json
import os

from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog

# Lateral below minSteerSpeed on Chrysler EPS units with a high minimum steer speed (WK2: 17.5 m/s = 63 km/h).
#
# History (WK2 drivelog): the EPS keeps tracking torque below minSteerSpeed once engaged (hold, dbb1463) and
# accepts a fresh LKAS request there (routes 7e-8e: 103 entries at 54.5 km/h, no fault; route 8f on 1af62ef: 10
# entries at 51 km/h and 3 drops at 49 km/h, no fault, motor torque following the applied torque as above 63 km/h).
# Where it stops accepting is not documented beyond opendbc's "13 m/s on the way down once engaged", so instead
# of fixed speeds this searches for the lowest ones that work, STEP at a time, and remembers them across drives:
#
#   hold   lowest speed lateral stays active at once engaged
#   entry  lowest speed a fresh engagement is requested at, always >= hold + GAP
#
# Each has a confirmed value and, one STEP below it, a trial. Lateral runs to the trial; the trial becomes the
# confirmed value after CONFIRMS clean uses with enough evidence that the EPS motor followed the applied torque
# (gain = sum(applied * motor) / sum(applied^2), ~255 on the WK2 at every speed measured). A steer fault on a
# trial, or a gain below MIN_GAIN (the EPS ignoring the request without a fault), sends that value back to the
# confirmed one for the rest of the drive; after MAX_FAILS such failures the search stops there for good. A
# fault on a confirmed value (not during a driver override, and only within STEP of it) raises it one STEP and stops
# the search too. A fault on a trial counts even with the driver overriding (most low entries are made with a hand on
# the wheel; a refusal there must not repeat every drive), it only costs the search, never the confirmed values.
# State lives in STATE_FILE (delete it to start over). Turn the whole thing off: echo 0 > /data/params/d/LowSpeedReengage
# (read every 5 s; missing = on; off = hold to minSteerSpeed - 3.0, entry at minSteerSpeed).
PARAM = "LowSpeedReengage"
STATE_FILE = "low_speed_lateral.json"
STATE_VERSION = 1

STEP = 1. / 3.6           # m/s per search step (1 km/h)
GAP = 2. / 3.6            # m/s the entry stays above the confirmed hold floor
ABS_FLOOR = 20. / 3.6     # m/s, the search never goes below this
START_HOLD_MARGIN = 3.9   # m/s below minSteerSpeed: confirmed hold floor to start from (WK2 49 km/h)
START_ENTRY_MARGIN = 3.3  # m/s below minSteerSpeed: confirmed entry to start from (WK2 51 km/h)
OFF_HOLD_MARGIN = 3.0     # m/s, hold floor with the toggle off (the floor since dbb1463)
CONFIRMS = 2              # clean uses of a trial before it is confirmed
MAX_FAILS = 2             # failures of a trial before the search stops there for good
HOLD_EVIDENCE_FRAMES = 100   # frames of usable torque evidence over the trial's uses (1 s)
ENTRY_EVIDENCE_FRAMES = 50   # per entry, within FAULT_WINDOW
MIN_VISIT_FRAMES = 10     # frames below the confirmed hold floor for a dip to count as a use
MIN_GAIN = 100.           # motor torque units per unit of applied torque; below = the EPS is not following
EVIDENCE_MIN_TORQUE = 0.05   # |applied torque| for a frame to be usable evidence
# |driver torque| (column units) for a frame to be usable evidence. WK2 drivelog G/H/I: at 40 only 38-49% of low
# entries got a verdict and the hold descended ~1.8 km/h per moving hour; at 80, 70-79% and ~3.5 km/h/h, gain still
# 243-257. Between the two only hand torque opposing the request counts (see Evidence.add).
EVIDENCE_QUIET_DRIVER = 40.
EVIDENCE_MAX_DRIVER = 80.
FAULT_WINDOW = 3.0        # s after a low-speed entry in which a steer fault counts as the EPS refusing it
HOLD_FAULT_WINDOW = 1.0   # s after lateral was last active (the wind-down) in which a fault still counts
PRESSED_WINDOW = 2.0      # s after steeringPressed in which a fault is the driver's, not the EPS refusing
# s an entry may wait for its first applied torque: the carcontroller holds the LKAS bit (and torque) down for 2 s
# after it fell, so a quick re-entry reaches the EPS later, and faster, than the latActive edge says.
ENTRY_LOCKOUT_WAIT = 2.5
# Torque is ramped in after a low-speed entry. Routes 7e-87 (77a1df4, 59 entries, no faults): the request reached
# 0.39 of full scale (p90 0.76) within 0.5 s of the entry and the car moved 0.12 m in 2 s, usually with the driver
# still holding the wheel from steering by hand (pressed within 2 s on 76%).
RAMP_SECONDS = 1.0
EPS_ = 1e-3


def read_toggle(params, key: str = PARAM) -> bool:
  # file read like curve_centering.read_enabled (prebuilt params library does not know new keys); missing = on
  try:
    with open(params.get_param_path(key)) as f:
      return f.read().strip() not in ("0", "false", "False")
  except Exception:
    return True


def default_state_path() -> str:
  # not in the params directory: Params.clearAll deletes files there that are not known keys
  try:
    from openpilot.system.hardware import PC
    from openpilot.system.hardware.hw import Paths
    return os.path.join(Paths.comma_home() if PC else "/data", STATE_FILE)
  except Exception:
    return os.path.join("/data", STATE_FILE)


class Evidence:
  """Does the EPS motor follow the applied torque? Least-squares gain through the origin."""
  def __init__(self):
    self.n, self.sxy, self.sxx = 0, 0., 0.

  def add(self, applied: float, motor: float, driver: float):
    # above EVIDENCE_QUIET_DRIVER only hand torque against the request counts: the motor also assists the driver,
    # so hand torque in the request's direction (G/H/I: +4..+9 motor units at 40-80) could fake a following EPS
    if abs(driver) > EVIDENCE_QUIET_DRIVER and driver * applied > 0.:
      return
    if abs(applied) > EVIDENCE_MIN_TORQUE and abs(driver) < EVIDENCE_MAX_DRIVER:
      self.n += 1
      self.sxy += applied * motor
      self.sxx += applied * applied

  @property
  def gain(self) -> float:
    return self.sxy / self.sxx if self.sxx > 0. else 0.


class LowSpeedReengage:
  def __init__(self, CP, enabled: bool = True, state_path: str | None = None):
    self.applies = CP.brand == 'chrysler' and CP.minSteerSpeed > 10.
    self.min_steer_speed = CP.minSteerSpeed
    self.enabled = enabled
    self.state_path = state_path

    # learned, persisted
    self.hold = self.min_steer_speed - START_HOLD_MARGIN     # confirmed (m/s)
    self.entry = self.min_steer_speed - START_ENTRY_MARGIN   # confirmed (m/s)
    self.hold_ok = 0          # clean uses of the hold trial so far
    self.entry_ok = 0
    self.hold_fails = 0       # failures of the current hold trial; MAX_FAILS = search stopped
    self.entry_fails = 0
    self.hold_ev = Evidence()  # accumulated over the uses of the current hold trial
    if self.applies:
      self._load()
    # a failed trial is not retried in the same drive
    self.hold_trial_ok = self.hold_fails < MAX_FAILS
    self.entry_trial_ok = self.entry_fails < MAX_FAILS

    self.entry_timer = 0.     # s since the last low-speed entry, while inside FAULT_WINDOW
    self.entry_speed = 0.
    self.entry_ev = Evidence()
    self.entry_wait = 0.      # s the current entry has waited for its first applied torque
    self.entry_torque_seen = False
    self.visit_frames = 0     # frames active below the confirmed hold floor in the current dip
    self.torque_scale = 1.    # multiplies the lateral torque request; ramps 0 -> 1 after a low-speed entry
    self.since_active = 1e3   # s since lateral was last active
    self.since_pressed = 1e3
    self.lat_active_prev = False
    self.fault_prev = False

  # ---- persistence

  def _load(self):
    if self.state_path is None:
      return
    try:
      with open(self.state_path) as f:
        d = json.load(f)
      if d.get("version") != STATE_VERSION or abs(d.get("min_steer_speed", 0.) - self.min_steer_speed) > 1e-3:
        return
      hold, entry = float(d["hold"]), float(d["entry"])
      if not (ABS_FLOOR - EPS_ <= hold <= self.min_steer_speed and hold + GAP - EPS_ <= entry <= self.min_steer_speed):
        return
      self.hold, self.entry = hold, entry
      self.hold_ok, self.entry_ok = int(d.get("hold_ok", 0)), int(d.get("entry_ok", 0))
      self.hold_fails, self.entry_fails = int(d.get("hold_fails", 0)), int(d.get("entry_fails", 0))
      self.hold_ev.n, self.hold_ev.sxy, self.hold_ev.sxx = int(d.get("ev_n", 0)), float(d.get("ev_sxy", 0.)), float(d.get("ev_sxx", 0.))
    except Exception:
      pass  # missing or unreadable: start from the defaults

  def _save(self):
    if self.state_path is None:
      return
    try:
      d = {"version": STATE_VERSION, "min_steer_speed": self.min_steer_speed, "hold": self.hold, "entry": self.entry,
           "hold_kph": round(self.hold * 3.6, 1), "entry_kph": round(self.entry * 3.6, 1),
           "hold_ok": self.hold_ok, "entry_ok": self.entry_ok, "hold_fails": self.hold_fails, "entry_fails": self.entry_fails,
           "ev_n": self.hold_ev.n, "ev_sxy": self.hold_ev.sxy, "ev_sxx": self.hold_ev.sxx}
      tmp = self.state_path + ".tmp"
      with open(tmp, "w") as f:
        json.dump(d, f)
      os.replace(tmp, self.state_path)
    except Exception:
      cloudlog.exception("low speed lateral: state not saved")

  # ---- speeds in use

  def hold_floor(self) -> float:
    if self.hold_trial_ok and self.hold - STEP >= ABS_FLOOR - EPS_:
      return self.hold - STEP
    return self.hold

  def entry_floor(self) -> float:
    # the entry trial never goes closer than GAP to the confirmed hold floor
    if self.entry_trial_ok and self.entry - STEP >= self.hold + GAP - EPS_:
      return self.entry - STEP
    return self.entry

  def min_lat_speed(self, v_ego: float) -> float:
    """Lowest speed at which lateral may be active this frame."""
    if not self.applies:
      return max(self.min_steer_speed, 0.3)
    if not self.enabled:
      return max(self.min_steer_speed - OFF_HOLD_MARGIN if self.lat_active_prev else self.min_steer_speed, 0.3)
    return max(self.hold_floor() if self.lat_active_prev else self.entry_floor(), 0.3)

  # ---- outcomes

  def _hold_trial_failed(self, why: str):
    self.hold_fails += 1
    self.hold_trial_ok = False
    self.hold_ok = 0
    self.hold_ev = Evidence()
    cloudlog.warning(f"low speed lateral: hold trial {(self.hold - STEP) * 3.6:.1f} km/h failed ({why}), {self.hold_fails}/{MAX_FAILS}")
    self._save()

  def _entry_trial_failed(self, why: str):
    self.entry_fails += 1
    self.entry_trial_ok = False
    self.entry_ok = 0
    cloudlog.warning(f"low speed lateral: entry trial {(self.entry - STEP) * 3.6:.1f} km/h failed ({why}), {self.entry_fails}/{MAX_FAILS}")
    self._save()

  def _hold_confirmed_failed(self):
    # a confirmed floor faulted: one step back up (not above the long-proven floor's worth of margin) and stop searching
    self.hold = min(self.hold + STEP, self.min_steer_speed)
    self.entry = min(max(self.entry, self.hold + GAP), self.min_steer_speed)
    self.hold_fails, self.hold_trial_ok, self.hold_ok, self.hold_ev = MAX_FAILS, False, 0, Evidence()
    cloudlog.warning(f"low speed lateral: fault on the confirmed hold floor, now {self.hold * 3.6:.1f} km/h")
    self._save()

  def _entry_confirmed_failed(self):
    self.entry = min(self.entry + STEP, self.min_steer_speed)
    self.entry_fails, self.entry_trial_ok, self.entry_ok = MAX_FAILS, False, 0
    cloudlog.warning(f"low speed lateral: fault on the confirmed entry, now {self.entry * 3.6:.1f} km/h")
    self._save()

  def _finish_entry(self):
    """FAULT_WINDOW after a low-speed entry passed without a fault."""
    if self.entry_speed >= self.entry - EPS_ or self.entry_ev.n < ENTRY_EVIDENCE_FRAMES:
      return  # not a trial, or not enough torque to tell: does not count either way
    if self.entry_ev.gain < MIN_GAIN:
      self._entry_trial_failed(f"motor gain {self.entry_ev.gain:.0f}")
      return
    self.entry_ok += 1
    if self.entry_ok >= CONFIRMS:
      self.entry, self.entry_ok, self.entry_fails = self.entry_floor(), 0, 0
      cloudlog.warning(f"low speed lateral: entry confirmed at {self.entry * 3.6:.1f} km/h")
    self._save()

  def _finish_visit(self):
    """A dip below the confirmed hold floor ended without a fault."""
    frames, self.visit_frames = self.visit_frames, 0
    if frames < MIN_VISIT_FRAMES:
      return
    self.hold_ok += 1
    if self.hold_ok >= CONFIRMS and self.hold_ev.n >= HOLD_EVIDENCE_FRAMES:
      if self.hold_ev.gain < MIN_GAIN:
        self._hold_trial_failed(f"motor gain {self.hold_ev.gain:.0f}")
        return
      self.hold, self.hold_ok, self.hold_fails, self.hold_ev = self.hold_floor(), 0, 0, Evidence()
      cloudlog.warning(f"low speed lateral: hold confirmed at {self.hold * 3.6:.1f} km/h")
    self._save()

  # ---- per frame

  def update(self, lat_active: bool, v_ego: float, steer_fault: bool, steering_pressed: bool = False,
             applied_torque: float = 0., motor_torque: float = 0., driver_torque: float = 0.):
    """Call once per controls frame after latActive is decided."""
    if not self.applies:
      return
    fault_edge = steer_fault and not self.fault_prev
    self.since_active = 0. if (lat_active or self.lat_active_prev) else self.since_active + DT_CTRL
    self.since_pressed = 0. if steering_pressed else self.since_pressed + DT_CTRL
    searching = self.enabled

    # fresh engagement below minSteerSpeed
    entry_fault = False
    if lat_active and not self.lat_active_prev and v_ego < self.min_steer_speed:
      self.entry_timer, self.entry_speed, self.entry_ev = DT_CTRL, v_ego, Evidence()
      self.entry_wait, self.entry_torque_seen = 0., False
      self.torque_scale = 0.
    elif self.entry_timer > 0.:
      if fault_edge:
        entry_fault = True
        self.entry_timer = 0.
        if searching and self.entry_speed < self.entry - EPS_:
          self._entry_trial_failed("steer fault")
        elif searching and self.entry_speed < self.entry + STEP and self.since_pressed > PRESSED_WINDOW:
          # only a fault right at the confirmed entry says something about it: one at 55-63 km/h, where the LKAS
          # bit usually never fell, or one the driver caused, must not ratchet the entry up for good
          self._entry_confirmed_failed()
      elif not lat_active:
        self.entry_timer = 0.  # dropped out by itself: no verdict
      elif not self.entry_torque_seen and abs(applied_torque) < EPS_ and self.entry_wait < ENTRY_LOCKOUT_WAIT:
        # nothing has reached the EPS yet (LKAS bit re-enable lockout): the entry happens where the torque starts
        self.entry_wait += DT_CTRL
        self.entry_speed = max(self.entry_speed, v_ego)
      else:
        self.entry_torque_seen = True
        self.entry_ev.add(applied_torque, motor_torque, driver_torque)
        self.entry_timer += DT_CTRL
        if self.entry_timer >= FAULT_WINDOW:
          self.entry_timer = 0.
          if searching:
            self._finish_entry()
    self.torque_scale = min(self.torque_scale + DT_CTRL / RAMP_SECONDS, 1.) if lat_active else 1.

    # holding below the confirmed floor (the hold trial)
    in_trial = searching and lat_active and v_ego < self.hold - EPS_
    if in_trial:
      self.visit_frames += 1
      self.hold_ev.add(applied_torque, motor_torque, driver_torque)
    # a fault inside an entry's window that lands below the confirmed hold floor still fails the hold trial,
    # otherwise the visit it ended would be counted as a clean use
    if fault_edge and (not entry_fault or self.visit_frames > 0) and searching and self.since_active < HOLD_FAULT_WINDOW and v_ego < self.min_steer_speed:
      if self.visit_frames > 0:
        self.visit_frames = 0
        self._hold_trial_failed("steer fault")
      elif v_ego < self.hold + STEP and self.since_pressed > PRESSED_WINDOW:
        self._hold_confirmed_failed()
    elif self.visit_frames > 0 and not in_trial and (self.since_active >= HOLD_FAULT_WINDOW or v_ego >= self.hold + STEP):
      self._finish_visit()  # wound down below the floor, or back up out of the trial band

    self.lat_active_prev = lat_active
    self.fault_prev = steer_fault
