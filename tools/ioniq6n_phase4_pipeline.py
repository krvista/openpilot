"""Pure-python mirror of the Phase 4 CCNC angle pipeline in carcontroller.py.

This module reimplements the exact logic of the CCNC angle block (lines
283-421 of opendbc_repo/opendbc/car/hyundai/carcontroller.py) so branch
tests can run without the full carcontroller dependency graph (CANPacker,
SunnyPilot mixins, etc.) that needs a running opendbc build.

KEEP IN SYNC with carcontroller.py. If a constant or branch changes
there, mirror it here.
"""
import os
import sys
import math
import numpy as np

sys.path.insert(0, '/home/user/openpilot/opendbc_repo')

from opendbc.car import DT_CTRL
from opendbc.car.structs import CarParams
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.hyundai.values import HyundaiFlags, CarControllerParams, CAR


# ---- mirror constants ----
ACI_GAIN_OP_FLOOR = 0.15
LOW_SPEED_PASSTHROUGH_ENTER_MS = 2.0 / 3.6
LOW_SPEED_PASSTHROUGH_EXIT_MS  = 3.0 / 3.6
LOWSPEED_LPF_TAU_BP = [0.0, 4.17]
LOWSPEED_LPF_TAU_V  = [0.16, 0.0]
LPF_DT = DT_CTRL * 2
JITTER_DEADBAND = 0.03
JITTER_FRAMES = 20
JITTER_STEP = 0.05
ACI_SPEED_FULL_MS = 3.0 / 3.6
ACI_SPEED_ZERO_MS = 1.0 / 3.6
DRIVER_TORQUE_DEADZONE = 30
DRIVER_TORQUE_FULL_OVERRIDE = 150
ACI_ENTER = 0.30
ACI_EXIT = 0.05
CAM_STALE_FRAMES = 25
ACI_GAIN_RAMP_TAU_FRAMES = 30.0


def make_ioniq6n_cp():
  cp = CarParams()
  cp.carFingerprint = str(CAR.HYUNDAI_IONIQ_6_N)
  cp.mass = 2175.0 + 136.0
  cp.wheelbase = 2.965
  cp.steerRatio = 14.26
  cp.centerToFront = 2.965 * 0.4
  cp.tireStiffnessFront = 1.1 * 192150
  cp.tireStiffnessRear = 1.1 * 202500
  cp.rotationalInertia = cp.mass * (cp.wheelbase ** 2) * 0.25 + 500
  cp.steerRatioRear = 0.0
  cp.flags = int(HyundaiFlags.EV | HyundaiFlags.CCNC |
                 HyundaiFlags.CANFD_LKA_STEERING_ALT |
                 HyundaiFlags.CANFD | HyundaiFlags.CANFD_ALT_BUTTONS)
  cp.steerControlType = CarParams.SteerControlType.angle
  return cp


class Phase4Sim:
  """Runs the CCNC angle block for one frame at a time; tracks every
  intermediate for assertions.

  Raises on any unhandled exception so pytest-style tests surface
  crashes immediately.
  """
  def __init__(self):
    self.CP = make_ioniq6n_cp()
    self.VM = VehicleModel(self.CP)
    self.params = CarControllerParams(self.CP)
    # state that the real carcontroller holds on self
    self.apply_angle_last = 0.0
    self.aci_active_latched = False
    self.passthrough_latched = False
    self.low_speed_cam_latched = False
    self.aci_gain_ramp = 0.0
    self.lpf_angle_last = 0.0
    self.jitter_counter = 0
    self.jitter_sign = 1
    self.cam_msg_last_frame = 0
    self.cam_msg_last_counter = -1
    self.frame = 0
    # trace ring for assertion
    self.trace = []

  def step(self, *, v_ego_raw, steering_angle_deg, steering_torque,
           blinker, lat_active, op_angle_cmd, cam_counter=None):
    """One 100 Hz frame. Returns dict with all outputs + decisions."""
    ccnc_lka_alt = True  # CP flags preconfigured

    # cam staleness
    cam_stale = False
    if cam_counter is not None:
      if cam_counter != self.cam_msg_last_counter:
        self.cam_msg_last_frame = self.frame
        self.cam_msg_last_counter = cam_counter
      if (self.frame - self.cam_msg_last_frame) > CAM_STALE_FRAMES:
        cam_stale = True

    # safety clamps
    v_ego_safe = float(np.clip(v_ego_raw, 0.0, 100.0)) if np.isfinite(v_ego_raw) else 0.0
    steer_angle_safe = float(steering_angle_deg) if np.isfinite(steering_angle_deg) else 0.0
    steer_torque_safe = float(steering_torque) if np.isfinite(steering_torque) else 0.0
    op_curv_raw = float(op_angle_cmd)
    op_curv_safe = op_curv_raw if np.isfinite(op_curv_raw) else steer_angle_safe

    # speed blend
    speed_blend = float(np.clip(
      (v_ego_safe - ACI_SPEED_ZERO_MS) / (ACI_SPEED_FULL_MS - ACI_SPEED_ZERO_MS),
      0.0, 1.0))
    driver_abs_torque = abs(steer_torque_safe)
    override_factor = float(np.clip(
      (driver_abs_torque - DRIVER_TORQUE_DEADZONE) /
      (DRIVER_TORQUE_FULL_OVERRIDE - DRIVER_TORQUE_DEADZONE),
      0.0, 1.0))
    driver_torque_blend = 1.0 - override_factor

    # ACI hysteresis
    authority = driver_torque_blend * speed_blend if lat_active else 0.0
    if blinker:
      authority *= 0.2
    if lat_active:
      if authority >= ACI_ENTER:
        self.aci_active_latched = True
      elif authority < ACI_EXIT:
        self.aci_active_latched = False
    else:
      self.aci_active_latched = False

    # passthrough latch
    if not lat_active and driver_torque_blend > 0.9:
      self.passthrough_latched = True
    elif lat_active or driver_torque_blend < 0.6:
      self.passthrough_latched = False

    # low-speed cam passthrough latch
    if v_ego_raw < LOW_SPEED_PASSTHROUGH_ENTER_MS:
      self.low_speed_cam_latched = True
    elif v_ego_raw > LOW_SPEED_PASSTHROUGH_EXIT_MS:
      self.low_speed_cam_latched = False

    in_passthrough = self.passthrough_latched or self.low_speed_cam_latched

    if self.aci_active_latched:
      self.aci_gain_ramp = min(1.0, self.aci_gain_ramp + 1.0 / ACI_GAIN_RAMP_TAU_FRAMES)
    else:
      self.aci_gain_ramp = 0.0

    # CCNC angle block — 50 Hz TX
    if self.frame % 2 == 0:
      desired_angle_deg = op_curv_safe
      if override_factor > 0:
        desired_angle_deg = (1.0 - override_factor) * desired_angle_deg + \
                            override_factor * steer_angle_safe
      tau_s = float(np.interp(v_ego_safe, LOWSPEED_LPF_TAU_BP, LOWSPEED_LPF_TAU_V))
      if tau_s > 0.001:
        alpha_lpf = LPF_DT / (tau_s + LPF_DT)
        desired_angle_deg = alpha_lpf * desired_angle_deg + (1.0 - alpha_lpf) * self.lpf_angle_last
      self.lpf_angle_last = desired_angle_deg

      rate_lat_active = bool(lat_active) and self.aci_active_latched

      self.apply_angle_last = apply_steer_angle_limits_vm(
        desired_angle_deg, self.apply_angle_last, v_ego_safe,
        steer_angle_safe, rate_lat_active, self.params, self.VM,
      )

      if rate_lat_active:
        if abs(self.apply_angle_last - self.lpf_angle_last) < JITTER_DEADBAND:
          self.jitter_counter += 1
        else:
          self.jitter_counter = 0
        if self.jitter_counter >= JITTER_FRAMES:
          self.apply_angle_last += self.jitter_sign * JITTER_STEP
          self.jitter_sign *= -1
          self.jitter_counter = 0
      else:
        self.jitter_counter = 0
        self.lpf_angle_last = steer_angle_safe

    out = dict(
      frame=self.frame,
      apply_angle=self.apply_angle_last,
      aci_active=self.aci_active_latched,
      aci_gain_ramp=self.aci_gain_ramp,
      passthrough=self.passthrough_latched,
      low_speed_cam=self.low_speed_cam_latched,
      in_passthrough=in_passthrough,
      cam_stale=cam_stale,
      authority=authority,
      driver_blend=driver_torque_blend,
      override_factor=override_factor,
      speed_blend=speed_blend,
      lpf_angle=self.lpf_angle_last,
      jitter_counter=self.jitter_counter,
    )
    self.trace.append(out)
    self.frame += 1
    return out


def assert_finite(out, label):
  for k, v in out.items():
    if isinstance(v, bool) or v is None:
      continue
    if isinstance(v, (int, float, np.floating)):
      assert math.isfinite(float(v)), f"{label}: {k}={v} not finite"


def assert_bounded_apply(out, max_deg=176.7, label=""):
  v = out["apply_angle"]
  assert abs(v) <= max_deg + 1e-3, f"{label}: apply_angle={v} exceeds {max_deg}"
