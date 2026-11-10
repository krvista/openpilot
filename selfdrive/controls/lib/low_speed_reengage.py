from openpilot.common.realtime import DT_CTRL

# EXPERIMENT (off by default): let lateral re-engage below minSteerSpeed on Chrysler EPS units with a high minimum
# steer speed. Once engaged the WK2 EPS keeps tracking torque down to minSteerSpeed - 3.0 (controlsd's hold), but
# after a dip below that floor lateral only came back above minSteerSpeed (63 km/h): WK2 drivelog 6a-7c spent 22%
# of the time above 54 km/h (33.8 of 150 min, single stretches of 2 min) cruising with lateral off, mostly on
# 60 km/h roads. Whether the EPS accepts a fresh LKAS request below 17.5 m/s is unknown (openpilot never asked);
# this asks from ENTRY_MARGIN below it. If the EPS faults within FAULT_WINDOW of such an entry, stop asking until
# the car is above minSteerSpeed again, so a refusing EPS faults once per dip, not repeatedly.
# Turn on: echo 1 > /data/params/d/LowSpeedReengage (read every 5 s; missing or 0 = off).
PARAM = "LowSpeedReengage"
ENTRY_MARGIN = 2.5   # m/s below minSteerSpeed where a fresh engagement is requested (hold floor stays -3.0)
HOLD_MARGIN = 3.0    # m/s, controlsd's existing hold floor below minSteerSpeed once lateral is active
FAULT_WINDOW = 3.0   # s after a low-speed entry in which a steer fault counts as the EPS refusing it


def read_toggle(params, key: str = PARAM) -> bool:
  # file read like curve_centering.read_enabled (prebuilt params library does not know new keys); missing = off
  try:
    with open(params.get_param_path(key)) as f:
      return f.read().strip() in ("1", "true", "True")
  except Exception:
    return False


class LowSpeedReengage:
  def __init__(self, CP, enabled: bool = False):
    self.applies = CP.brand == 'chrysler' and CP.minSteerSpeed > 10.
    self.min_steer_speed = CP.minSteerSpeed
    self.enabled = enabled
    self.locked_out = False
    self.entry_timer = 0.   # s since the last low-speed entry, while inside FAULT_WINDOW
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
    elif self.entry_timer > 0.:
      self.entry_timer = self.entry_timer + DT_CTRL if self.entry_timer < FAULT_WINDOW else 0.

    if steer_fault and not self.fault_prev and self.entry_timer > 0.:
      self.locked_out = True
      self.entry_timer = 0.
    if v_ego > self.min_steer_speed:
      self.locked_out = False

    self.lat_active_prev = lat_active
    self.fault_prev = steer_fault
