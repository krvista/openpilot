import numpy as np
from abc import abstractmethod, ABC
from openpilot.selfdrive.locationd.helpers import Pose

# Hand torque above which the integrator holds, below the brand's steeringPressed threshold (brand torque units).
# Chrysler (WK2 drivelog 5f-69): resting hands read 20-50, steeringPressed only trips at 120, and in between the
# integrator wound against the driver (hand torque leads it by 1-2 s, corr 0.49; +4 units per 4.5 s hold) and the car
# swung 5-7 cm the other way on release. 40 is the level torqued already treats as the driver steering.
DRIVER_TORQUE_INTEGRATOR_HOLD = {'chrysler': 40.}

# Error-term (P) blend around a driver override, shared by the torque controllers and NNLC.
RELEASE_BLEND_SECONDS = 0.4  # ramp the error term back in after the driver lets go of the wheel
# Brands whose error term also yields while the wheel is pressed, ramping out over PRESS_BLEND_SECONDS.
# Chrysler (WK2 drivelog 5f-7c, 23cf66e): during presses of 0.5 s or more the integrator held, but P alone raised
# the torque opposing the driver by 0.11-0.12 of full scale within 0.5 s (to 0.27-0.35 on average, > 0.8 at the end
# of 9% of curve presses), and 50-72% of all saturated-command time was op at full torque into a curve against a
# driver steering out of it. The feedforward keeps the model's curve; only the correction against the driver yields.
PRESS_YIELD_BRANDS = ('chrysler',)
PRESS_BLEND_SECONDS = 0.3


class LatControl(ABC):
  def __init__(self, CP, CP_SP, CI, dt):
    self.dt = dt
    self.sat_limit = CP.steerLimitTimer
    self.sat_time = 0.
    self.sat_check_min_speed = 10.
    self.driver_torque_hold = DRIVER_TORQUE_INTEGRATOR_HOLD.get(CP.brand, float('inf'))
    self.press_yield = CP.brand in PRESS_YIELD_BRANDS
    self.release_blend = 1.0  # multiplies the error term; 1 = full correction
    self.steering_pressed_prev = False

    # we define the steer torque scale as [-1.0...1.0]
    self.steer_max = 1.0

  @abstractmethod
  def update(self, active: bool, CS, VM, params, steer_limited_by_safety: bool, desired_curvature: float, calibrated_pose: Pose,
             curvature_limited: bool, lat_delay: float):
    pass

  def reset(self):
    self.sat_time = 0.

  def update_release_blend(self, CS) -> float:
    """Error-term weight around a driver override. Feedforward and the logged error are untouched.

    Release: while the driver holds the wheel the P term fights them, and on release the controller would snap to
    full correction at once (WK2 drivelog: requested-torque step within 300 ms of release p50 0.16, p90 0.52 of
    full scale), so the weight ramps back to 1 over RELEASE_BLEND_SECONDS.
    Press (PRESS_YIELD_BRANDS): the weight ramps down to 0 over PRESS_BLEND_SECONDS while pressed, and the release
    ramp starts from wherever it got to. Other brands keep full correction while pressed and restart from 0 on release.
    """
    pressed = bool(CS.steeringPressed)
    if pressed:
      self.release_blend = max(self.release_blend - self.dt / PRESS_BLEND_SECONDS, 0.) if self.press_yield else 1.
    elif self.steering_pressed_prev and not self.press_yield:
      self.release_blend = 0.
    else:
      self.release_blend = min(self.release_blend + self.dt / RELEASE_BLEND_SECONDS, 1.)
    self.steering_pressed_prev = pressed
    return self.release_blend

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
