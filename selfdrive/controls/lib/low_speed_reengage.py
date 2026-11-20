from openpilot.common.realtime import DT_CTRL

# On by default: let lateral re-engage below minSteerSpeed on Chrysler EPS units with a high minimum
# steer speed. Once engaged the WK2 EPS keeps tracking torque below minSteerSpeed (the hold, HOLD_MARGIN), but
# after a dip below that floor lateral only came back above minSteerSpeed (63 km/h): WK2 drivelog 6a-7c spent 22%
# of the time above 54 km/h (33.8 of 150 min, single stretches of 2 min) cruising with lateral off, mostly on
# 60 km/h roads. Whether the EPS accepts a fresh LKAS request below 17.5 m/s is unknown (openpilot never asked);
# this asks from ENTRY_MARGIN below it. If the EPS faults within FAULT_WINDOW of such an entry, stop asking until
# the car is above minSteerSpeed again, so a refusing EPS faults once per dip, not repeatedly.
# Turn off: echo 0 > /data/params/d/LowSpeedReengage (read every 5 s; missing = on).
PARAM = "LowSpeedReengage"
# Step 1 of lowering both (was entry -2.5 / hold -3.0 = 54.5 / 52 km/h on the WK2). Routes 7e-8e (77a1df4, 2a9aafd):
# 103 entries at 54-58 km/h and 13 drops at the hold floor without a steer fault, and EPS motor torque follows the
# applied torque the same at 50-54.5 km/h as above 63 (corr 0.97-0.98, gain ~255). opendbc documents assist down to
# 13 m/s (46.8 km/h) once engaged; the carcontroller's LKAS bit floor (-4.3 = 47.5 km/h) stays above that.
# UNVERIFIED: a fresh entry below 54 km/h and torque below 50 km/h have no data yet; the fault lockout covers a refusal.
ENTRY_MARGIN = 3.3   # m/s below minSteerSpeed where a fresh engagement is requested (WK2: 51 km/h)
HOLD_MARGIN = 3.9    # m/s below minSteerSpeed that lateral holds to once active (WK2: 49 km/h)
FAULT_WINDOW = 3.0   # s after a low-speed entry in which a steer fault counts as the EPS refusing it
# Torque is ramped in after a low-speed entry. Routes 7e-87 (77a1df4, 59 entries, no faults): the request reached
# 0.39 of full scale (p90 0.76) within 0.5 s of the entry and the car moved 0.12 m in 2 s, usually with the driver
# still holding the wheel from steering by hand (pressed within 2 s on 76%).
RAMP_SECONDS = 1.0


def read_toggle(params, key: str = PARAM) -> bool:
  # file read like curve_centering.read_enabled (prebuilt params library does not know new keys); missing = on
  try:
    with open(params.get_param_path(key)) as f:
      return f.read().strip() not in ("0", "false", "False")
  except Exception:
    return True


class LowSpeedReengage:
  def __init__(self, CP, enabled: bool = True):
    self.applies = CP.brand == 'chrysler' and CP.minSteerSpeed > 10.
    self.min_steer_speed = CP.minSteerSpeed
    self.enabled = enabled
    self.locked_out = False
    self.entry_timer = 0.   # s since the last low-speed entry, while inside FAULT_WINDOW
    self.torque_scale = 1.  # multiplies the lateral torque request; ramps 0 -> 1 after a low-speed entry
    self.lat_active_prev = False
    self.fault_prev = False

  def min_lat_speed(self, v_ego: float) -> float:
    """Lowest speed at which lateral may be active this frame."""
    if not self.applies:
      return max(self.min_steer_speed, 0.3)
    if self.lat_active_prev:
      return max(self.min_steer_speed - HOLD_MARGIN, 0.3)
    if self.enabled and not self.locked_out:
      return max(self.min_steer_speed - ENTRY_MARGIN, 0.3)
    return max(self.min_steer_speed, 0.3)

  def update(self, lat_active: bool, v_ego: float, steer_fault: bool):
    """Call once per controls frame after latActive is decided."""
    if lat_active and not self.lat_active_prev and v_ego < self.min_steer_speed:
      self.entry_timer = DT_CTRL  # fresh engagement below minSteerSpeed: watch for a refusal
      self.torque_scale = 0.
    elif self.entry_timer > 0.:
      self.entry_timer = self.entry_timer + DT_CTRL if self.entry_timer < FAULT_WINDOW else 0.

    self.torque_scale = min(self.torque_scale + DT_CTRL / RAMP_SECONDS, 1.) if lat_active else 1.

    if steer_fault and not self.fault_prev and self.entry_timer > 0.:
      self.locked_out = True
      self.entry_timer = 0.
    if v_ego > self.min_steer_speed:
      self.locked_out = False

    self.lat_active_prev = lat_active
    self.fault_prev = steer_fault
