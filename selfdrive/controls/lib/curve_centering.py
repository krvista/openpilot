import numpy as np

from cereal import log
from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.modeld.constants import ModelConstants

LaneChangeState = log.LaneChangeState

# The driving model plans a line inside of curves and holds it: WK2 drivelog routes 3a-4d (3.1 h
# active above 54 km/h), lane-centre offset of the planned path 1 s ahead vs model lateral accel:
#   0.3-0.6 m/s^2: 0.10 m inside, 0.6-1.0: 0.17 m, 1.0-1.5: 0.21-0.32 m (>0.3 m 47-64% of the time).
# Curvature tracking was 1.00 (actual / desired), so this is the plan, not the controller. On straights
# the car was centred (mean -0.03 m), and a car on the inside (BSM) did not move the line (0.19 m with
# and without). This nudges the desired curvature outward while the planned path is inside of the lane
# centre in a curve; it never steers inward and is gated off on straights, lane changes and blinkers.
LOOKAHEAD_T = 1.0           # s, point on the model plan whose lane-centre offset is corrected
MIN_SPEED = 12.             # m/s
CURVE_BP = [0.25, 0.6]      # m/s^2 |model lateral accel| -> fade in
DEADBAND = 0.08             # m inside of centre left alone
BSM_MARGIN = 0.10           # m outward of centre to aim for while a car is alongside on the inside
GAIN = 1.2                  # (m/s^2)/m of lateral accel per metre inside (pure pursuit over 1 s = 2.0)
MAX_LAT_ACCEL = 0.4         # m/s^2
MAX_FRACTION = 0.5          # of the model's own lateral accel
MIN_LANE_PROB = 0.7
LANE_WIDTH = (2.7, 4.3)     # m
TAU = 0.5                   # s, smoothing of the correction
PARAM = "CurveLaneCentering"


def read_enabled(params) -> bool:
  # Release branches run the prebuilt params library, which does not know keys added to params_keys.h
  # after it was built: Params.get_bool(PARAM) raised UnknownKeyName at controlsd start, so nothing
  # sent LKAS commands and the car showed its lane-sense fault. Read the file directly (the path lookup
  # does not check the key); missing or unreadable means on.
  try:
    with open(params.get_param_path(PARAM)) as f:
      return f.read().strip() not in ("0", "false", "False")
  except Exception:
    return True


class CurveCentering:
  def __init__(self, enabled: bool = True):
    self.enabled = enabled
    self.lat_accel = 0.0     # filtered outward correction, signed like curvature * v^2
    self.target = 0.0
    self.inside = 0.0        # last measured inside offset of the plan (m), for logging/tests
    self.alpha = DT_CTRL / (TAU + DT_CTRL)

  def reset(self):
    self.lat_accel = 0.0
    self.target = 0.0

  def _target(self, model, CS) -> float:
    v = CS.vEgo
    if v < MIN_SPEED or CS.leftBlinker or CS.rightBlinker or model.meta.laneChangeState != LaneChangeState.off:
      return 0.0

    probs = model.laneLineProbs
    if len(probs) < 4 or len(model.laneLines) < 4 or min(probs[1], probs[2]) < MIN_LANE_PROB:
      return 0.0
    left, right = model.laneLines[1], model.laneLines[2]
    if min(len(left.x), len(left.y), len(right.x), len(right.y), len(model.position.x), len(model.position.y)) < 2:
      return 0.0
    width = right.y[0] - left.y[0]
    if not LANE_WIDTH[0] < width < LANE_WIDTH[1]:
      return 0.0

    model_lat_accel = model.action.desiredCurvature * v ** 2
    curve = np.interp(abs(model_lat_accel), CURVE_BP, [0., 1.])
    if curve <= 0.:
      return 0.0
    direction = 1. if model_lat_accel > 0 else -1.  # + = right, as curvature and model y

    # lane-centre offset of the planned path at the lookahead point (+ = right of centre)
    px = np.interp(LOOKAHEAD_T, ModelConstants.T_IDXS, model.position.x)
    py = np.interp(LOOKAHEAD_T, ModelConstants.T_IDXS, model.position.y)
    centre = (np.interp(px, left.x, left.y) + np.interp(px, right.x, right.y)) / 2.
    self.inside = float((py - centre) * direction)

    bsm_inside = CS.rightBlindspot if direction > 0 else CS.leftBlindspot
    err = self.inside + BSM_MARGIN if bsm_inside else self.inside - DEADBAND
    if err <= 0.:
      return 0.0
    outward = min(GAIN * err * curve, MAX_LAT_ACCEL, MAX_FRACTION * abs(model_lat_accel))
    return -direction * outward

  def update(self, lat_active: bool, model, model_updated: bool, CS) -> float:
    """Returns the curvature (1/m) to add to the model's desired curvature."""
    if not self.enabled or not lat_active:
      self.reset()
      return 0.0
    if model_updated:
      try:
        self.target = self._target(model, CS)
      except Exception:
        # an aid on top of the model's curvature must never take controlsd down
        cloudlog.exception("curve centering failed")
        self.target = 0.0
    self.lat_accel += self.alpha * (self.target - self.lat_accel)
    return self.lat_accel / max(CS.vEgo, MIN_SPEED) ** 2
