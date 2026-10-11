import math

import numpy as np

from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.realtime import DT_CTRL

# liveParameters.roll comes from the calibrated device pose, and calibration does not estimate device roll
# (rpyCalib roll -0.007 deg on routes 51-5b), so a tilted mount reads as road roll. WK2 drivelog 3a-5b:
# roll +2.3 deg on every route, torqued latAccelOffset converged to -0.43..-0.45 m/s^2 (~9900 points,
# = 2.6 deg of roll), and every symptom of a missing +0.45 m/s^2 in the feedforward: PID integrator
# +0.16 on straights, driver holding +18 on straights under openpilot (-22 when driving manually),
# left curves 0.91-0.95 of desired and right curves 1.03-1.06. latcontrol_torque already removes
# latAccelOffset from its feedforward, but only with useParams, which is off here, and the NNLC
# feedforward takes params.roll directly. Remove the bias from the roll that lateral control uses.
MIN_POINTS = 4000           # torqued's own minimum for a valid estimate
MAX_OFFSET = 0.5            # m/s^2 (2.9 deg)
TAU = 10.                   # s


class RollBias:
  def __init__(self, enabled: bool = True):
    self.enabled = enabled
    self.bias = 0.0          # rad, subtracted from liveParameters.roll
    self.alpha = DT_CTRL / (TAU + DT_CTRL)

  def update(self, torque_params, valid: bool) -> float:
    target = 0.0
    offset = torque_params.latAccelOffsetFiltered
    # with useParams latcontrol_torque subtracts latAccelOffset itself; never correct twice
    if self.enabled and valid and torque_params.liveValid and not torque_params.useParams and \
       torque_params.totalBucketPoints >= MIN_POINTS and math.isfinite(offset):
      # torqued fits lat_accel - g*sin(roll) = factor * torque + offset; a roll bias b gives offset = -g*b
      target = -float(np.clip(offset, -MAX_OFFSET, MAX_OFFSET)) / ACCELERATION_DUE_TO_GRAVITY
    self.bias += self.alpha * (target - self.bias)
    return self.bias


class RollCorrectedParams:
  """liveParameters with the roll used by lateral control replaced."""
  __slots__ = ('_lp', 'roll')

  def __init__(self, lp, roll: float):
    self._lp = lp
    self.roll = roll

  def __getattr__(self, name):
    return getattr(self._lp, name)
