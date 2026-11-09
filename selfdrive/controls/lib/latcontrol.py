import numpy as np
from abc import abstractmethod, ABC
from openpilot.selfdrive.locationd.helpers import Pose

# Hand torque above which the integrator holds, below the brand's steeringPressed threshold (brand torque units).
# Chrysler (WK2 drivelog 5f-69): resting hands read 20-50, steeringPressed only trips at 120, and in between the
# integrator wound against the driver (hand torque leads it by 1-2 s, corr 0.49; +4 units per 4.5 s hold) and the car
# swung 5-7 cm the other way on release. 40 is the level torqued already treats as the driver steering.
DRIVER_TORQUE_INTEGRATOR_HOLD = {'chrysler': 40.}


class LatControl(ABC):
  def __init__(self, CP, CP_SP, CI, dt):
    self.dt = dt
    self.sat_limit = CP.steerLimitTimer
    self.sat_time = 0.
    self.sat_check_min_speed = 10.
    self.driver_torque_hold = DRIVER_TORQUE_INTEGRATOR_HOLD.get(CP.brand, float('inf'))

    # we define the steer torque scale as [-1.0...1.0]
    self.steer_max = 1.0

  @abstractmethod
  def update(self, active: bool, CS, VM, params, steer_limited_by_safety: bool, desired_curvature: float, calibrated_pose: Pose,
             curvature_limited: bool, lat_delay: float):
    pass

  def reset(self):
    self.sat_time = 0.

  def driver_steering(self, CS) -> bool:
    """Hand torque the integrator should not learn against, even while it is below steeringPressed."""
    return abs(CS.steeringTorque) > self.driver_torque_hold

  def _check_saturation(self, saturated, CS, steer_limited_by_safety, curvature_limited):
    # Saturated only if control output is not being limited by car torque/angle rate limits
    if (saturated or curvature_limited) and CS.vEgo > self.sat_check_min_speed and not steer_limited_by_safety and not CS.steeringPressed:
      self.sat_time += self.dt
    else:
      self.sat_time -= self.dt
    self.sat_time = np.clip(self.sat_time, 0.0, self.sat_limit)
    return self.sat_time > (self.sat_limit - 1e-3)
