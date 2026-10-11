import numpy as np

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.drive_helpers import get_curvature_from_plan
from openpilot.selfdrive.modeld.constants import ModelConstants

# Unwind a little earlier on curve exits. The model's curvature already looks lat_delay + 1.5 frames
# ahead (0.355 s), which matches the WK2 yaw response on average (best-fit 0.34-0.36 s on entry,
# exit and steady, routes 51-5b), but on exits the car still ends up closer to the inside: routes
# 43-4d (no centering) sat 0.32 m inside on exits vs 0.20 m mid-curve, and the target curvature on
# exits stayed 0.07-0.10 m/s^2 above the lane-line road curvature. While the plan is unwinding, take
# the curvature EXIT_LEAD_T further along the plan; entries, steady curves and straights are untouched
# and the result never goes past straight.
EXIT_LEAD_T = 0.1           # s
MIN_SPEED = 12.             # m/s
CURVE_BP = [0.2, 0.5]       # m/s^2 |desired lateral accel| -> fade in
MAX_MISMATCH = 0.15         # m/s^2: the plan must reproduce the model's curvature, else leave it alone
MAX_REDUCTION = 0.25        # m/s^2 (replay of 51-5b: exits mean 0.06, p90 0.13, max 0.49 unclipped)
PARAM = "CurveExitLead"


class CurveExitLead:
  def __init__(self, enabled: bool = True):
    self.enabled = enabled

  def update(self, model, v_ego: float, desired_curvature: float, lat_delay: float) -> float:
    if not self.enabled or v_ego < MIN_SPEED:
      return desired_curvature
    curve = np.interp(abs(desired_curvature) * v_ego ** 2, CURVE_BP, [0., 1.])
    if curve <= 0.:
      return desired_curvature
    yaws, yaw_rates = model.orientation.z, model.orientationRate.z
    if len(yaws) != len(ModelConstants.T_IDXS) or len(yaw_rates) < 1:
      return desired_curvature

    # same action time as modeld's plan-to-curvature (lat_delay + frame delay + half a frame)
    action_t = lat_delay + 1.5 * DT_MDL
    now = get_curvature_from_plan(yaws, yaw_rates, ModelConstants.T_IDXS, v_ego, action_t)
    if abs(now - desired_curvature) * v_ego ** 2 > MAX_MISMATCH:
      return desired_curvature
    ahead = get_curvature_from_plan(yaws, yaw_rates, ModelConstants.T_IDXS, v_ego, action_t + EXIT_LEAD_T)

    # only toward straight: keep the sign of the current request, never more than it
    direction = 1. if desired_curvature > 0 else -1.
    unwound = direction * max(0., min(abs(desired_curvature), direction * ahead))
    reduction = min(curve * abs(desired_curvature - unwound), MAX_REDUCTION / v_ego ** 2)
    return float(desired_curvature - direction * reduction)
