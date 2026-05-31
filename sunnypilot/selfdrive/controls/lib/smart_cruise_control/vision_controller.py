"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

import cereal.messaging as messaging
from cereal import custom
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import MIN_V

VisionState = custom.LongitudinalPlanSP.SmartCruiseControl.VisionState

ACTIVE_STATES = (VisionState.entering, VisionState.turning, VisionState.leaving)
ENABLED_STATES = (VisionState.enabled, VisionState.overriding, *ACTIVE_STATES)

# modelV2 trajectory time indices: 33 steps non-uniform over [0, 10s]
# (matches ModelConstants.T_IDXS — duplicated locally to avoid hardware-stack import).
_T_IDXS = np.array([10.0 * (i / 32.0) ** 2 for i in range(33)])
# Boolean mask for the 3-5s anticipation window (idx ~17..22, t≈2.82..4.73s).
_ANTICIPATE_MASK = (_T_IDXS >= 3.0) & (_T_IDXS <= 5.0)

_ENTERING_PRED_LAT_ACC_TH = 2.0  # Predicted Lat Acc threshold to trigger entering turn state.
_ABORT_ENTERING_PRED_LAT_ACC_TH = 1.6  # Predicted Lat Acc threshold to abort entering state if speed drops.

# 3-5s lookahead anticipation. Trigger entering early with a gentle drop when
# the modelV2 trajectory shows non-trivial lat acc in the 3..5s window.
# Raised 1.3 -> 1.8 from wk2 drivelog analysis (2020 Jeep GC, dongle 0fb02cc3a5abcc2f,
# build v6 644e7f06): on genuinely straight highway the 3-5s window lat-acc reaches p99
# ~1.4 with momentary peaks ~2.4, so a 1.3 threshold fired repeatedly on straight road
# and caused unwanted slow-downs (some escalating to driver ACC cancel). 1.8 sits above
# the straight p99 while genuine curves still cross it. Abort raised to keep hysteresis.
_ANTICIPATE_PRED_LAT_ACC_TH = 1.8        # 3-5s window peak threshold to start anticipating
_ANTICIPATE_ABORT_LAT_ACC_TH = 1.4       # drop back to enabled when window falls below this

# Consecutive-frame debounce for enabled->entering. The modelV2 predicted lat-acc has brief
# single-frame spikes on straight road (peaks ~2.4 even when straight); requiring the entry
# condition to hold for a few frames rejects those while genuine curves (which sustain the
# signal for ~0.5-2.5 s) still trigger.
_ENTERING_DEBOUNCE_FRAMES = 3            # require >3 i.e. 4 consecutive frames (~0.2 s at DT_MDL 20 Hz)

_TURNING_LAT_ACC_TH = 2.0  # Lat Acc threshold to trigger turning state.

_LEAVING_PRED_LAT_ACC_TH = 1.4  # Predicted Lat Acc threshold to anticipate curve straightening and switch to leaving early.
_LEAVING_LAT_ACC_TH = 1.5  # Lat Acc threshold to trigger leaving turn state.
_FINISH_LAT_ACC_TH = 1.3  # Lat Acc threshold to trigger the end of the turn cycle.

_A_LAT_REG_MAX = 2.8  # Maximum lateral acceleration (was 2.0; raised to 0.29g for Jeep GC on highway curves)

_NO_OVERSHOOT_TIME_HORIZON = 4.  # s. Time to use for velocity desired based on a_target when not overshooting.

# User-tuned ENTERING-state cruise drop curve.
# Map max_pred_lat_acc -> desired cruise speed drop (kph). The actual drop is
# delivered by a_target = -drop_ms / _NO_OVERSHOOT_TIME_HORIZON applied while
# v_target = v_ego, so output_v_target = v_ego - drop_ms. Capped at 10 kph
# per user intent: even strong highway curves should not drop more than 10 kph
# while v_ego <= ~120 kph. For deeper curves the lateral safety considerations
# in the rest of the long plan remain in effect.
_PRED_DROP_BP    = [1.6, 2.0, 3.0]    # max_pred_lat_acc breakpoints (m/s²)
_PRED_DROP_KPH_V = [0.0, 5.0, 10.0]   # desired cruise drop (kph)

# Gentle anticipatory drop curve for the 3-5s lookahead window.
# Max 2.5 kph drop while only the far horizon sees the curve — user wanted "살짝".
_ANTICIPATE_DROP_BP    = [1.3, 1.5, 2.0]
_ANTICIPATE_DROP_KPH_V = [0.0, 1.0, 2.5]

# Lookup table for the acceleration for the TURNING state
# depending on the current lateral acceleration of the vehicle.
_TURNING_ACC_V = [0.5, 0., -0.4]  # acc value
_TURNING_ACC_BP = [1.5, 2.3, 3.]  # absolute value of current lat acc

_LEAVING_ACC = 0.3  # Comfortable acceleration to regain speed while leaving a turn (gentle: 5 kph drop recovers in ~5s, 10 kph in ~9s).


class SmartCruiseControlVision:
  v_target: float = 0
  a_target: float = 0.
  v_ego: float = 0.
  a_ego: float = 0.
  output_v_target: float = V_CRUISE_UNSET
  output_a_target: float = 0.

  def __init__(self):
    self.params = Params()
    self.frame = -1
    self.long_enabled = False
    self.long_override = False
    self.is_enabled = False
    self.is_active = False
    self.enabled = self.params.get_bool("SmartCruiseControlVision")
    self.v_cruise_setpoint = 0.

    self.state = VisionState.disabled
    self.current_lat_acc = 0.
    self.max_pred_lat_acc = 0.
    self.anticipated_lat_acc = 0.  # peak of predicted lat acc in the 3-5s lookahead window
    self.entering_candidate_frames = 0  # consecutive frames the entry condition has held

  def get_a_target_from_control(self) -> float:
    return self.a_target

  def get_v_target_from_control(self) -> float:
    if self.is_active:
      return max(self.v_target, MIN_V) + self.a_target * _NO_OVERSHOOT_TIME_HORIZON

    return V_CRUISE_UNSET

  def _update_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enabled = self.params.get_bool("SmartCruiseControlVision")

  def _update_calculations(self, sm: messaging.SubMaster) -> None:
    if not self.long_enabled:
      return
    else:
      rate_plan = np.array(np.abs(sm['modelV2'].orientationRate.z))
      vel_plan = np.array(sm['modelV2'].velocity.x)

      self.current_lat_acc = self.v_ego ** 2 * abs(sm['controlsState'].curvature)

      # get the maximum lat accel from the model
      predicted_lat_accels = rate_plan * vel_plan
      self.max_pred_lat_acc = np.percentile(predicted_lat_accels, 97)

      # 3-5s lookahead: peak lat acc in the window. Used to trigger gentle anticipation
      # before the near-horizon p97 crosses _ENTERING_PRED_LAT_ACC_TH.
      n = min(len(predicted_lat_accels), len(_ANTICIPATE_MASK))
      self.anticipated_lat_acc = float(predicted_lat_accels[:n][_ANTICIPATE_MASK[:n]].max()) if n else 0.

      # v_target anchors the "now" velocity; a_target (computed later for ENTERING)
      # drives the actual cruise drop via output_v_target = v_target + a_target * _NO_OVERSHOOT_TIME_HORIZON.
      self.v_target = max(self.v_ego, MIN_V)

  def _update_state_machine(self) -> tuple[bool, bool]:
    # ENABLED, ENTERING, TURNING, LEAVING, OVERRIDING
    if self.state != VisionState.disabled:
      # longitudinal and feature disable always have priority in a non-disabled state
      if not self.long_enabled or not self.enabled:
        self.state = VisionState.disabled
      elif self.long_override:
        self.state = VisionState.overriding

      else:
        # ENABLED
        if self.state == VisionState.enabled:
          # Do not enter a turn control cycle if the speed is low.
          if self.v_ego <= MIN_V:
            self.entering_candidate_frames = 0
          # Enter on either the near-horizon trigger (curve ~1s away) OR the 3-5s
          # anticipation trigger (early gentle drop). a_target picks the larger drop.
          # Require the condition to hold for _ENTERING_DEBOUNCE_FRAMES consecutive frames
          # so single-frame predicted-lat-acc spikes on straight road don't trip it.
          elif (self.max_pred_lat_acc >= _ENTERING_PRED_LAT_ACC_TH or
                self.anticipated_lat_acc >= _ANTICIPATE_PRED_LAT_ACC_TH):
            self.entering_candidate_frames += 1
            if self.entering_candidate_frames > _ENTERING_DEBOUNCE_FRAMES:
              self.state = VisionState.entering
              self.entering_candidate_frames = 0
          else:
            self.entering_candidate_frames = 0

        # OVERRIDING
        elif self.state == VisionState.overriding:
          if not self.long_override:
            self.state = VisionState.enabled

        # ENTERING
        elif self.state == VisionState.entering:
          # Transition to Turning if current lateral acceleration is over the threshold.
          if self.current_lat_acc >= _TURNING_LAT_ACC_TH:
            self.state = VisionState.turning
          # Abort only when BOTH horizons drop below their abort thresholds —
          # otherwise we'd flap back to enabled while still anticipating.
          elif (self.max_pred_lat_acc < _ABORT_ENTERING_PRED_LAT_ACC_TH and
                self.anticipated_lat_acc < _ANTICIPATE_ABORT_LAT_ACC_TH):
            self.state = VisionState.enabled

        # TURNING
        elif self.state == VisionState.turning:
          # Anticipate curve straightening: if the model predicts the curve ending ahead
          # (max_pred drops), switch to leaving before current lat_acc actually falls so
          # v_target recovers sooner. Fall back to measured lat_acc otherwise.
          if self.max_pred_lat_acc <= _LEAVING_PRED_LAT_ACC_TH:
            self.state = VisionState.leaving
          elif self.current_lat_acc <= _LEAVING_LAT_ACC_TH:
            self.state = VisionState.leaving

        # LEAVING
        elif self.state == VisionState.leaving:
          # Transition back to Turning if current lateral acceleration goes back over the threshold.
          if self.current_lat_acc >= _TURNING_LAT_ACC_TH:
            self.state = VisionState.turning
          # Finish if current lateral acceleration goes below a threshold.
          elif self.current_lat_acc < _FINISH_LAT_ACC_TH:
            self.state = VisionState.enabled

    # DISABLED
    elif self.state == VisionState.disabled:
      if self.long_enabled and self.enabled:
        if self.long_override:
          self.state = VisionState.overriding
        else:
          self.state = VisionState.enabled

    enabled = self.state in ENABLED_STATES
    active = self.state in ACTIVE_STATES

    return enabled, active

  def _update_solution(self) -> float:
    # DISABLED, ENABLED, OVERRIDING
    if self.state not in ACTIVE_STATES:
      # when not overshooting, calculate v_turn as the speed at the prediction horizon when following
      # the smooth deceleration.
      a_target = self.a_ego
    # ENTERING
    elif self.state == VisionState.entering:
      # Two-horizon drop blend:
      #   near (p97 of 0..10s trajectory): user-tuned 0..10 kph as the curve closes in
      #   far  (peak in 3..5s window)    : gentle 0..2.5 kph for early anticipation
      # Use the larger of the two so anticipation kicks in first and gracefully ramps
      # up to the full near-horizon drop as the curve approaches.
      drop_near = float(np.interp(self.max_pred_lat_acc, _PRED_DROP_BP, _PRED_DROP_KPH_V))
      drop_far  = float(np.interp(self.anticipated_lat_acc, _ANTICIPATE_DROP_BP, _ANTICIPATE_DROP_KPH_V))
      desired_drop_ms = max(drop_near, drop_far) / 3.6
      a_target = -desired_drop_ms / _NO_OVERSHOOT_TIME_HORIZON
    # TURNING
    elif self.state == VisionState.turning:
      # When turning, we provide a target acceleration that is comfortable for the lateral acceleration felt.
      a_target = np.interp(self.current_lat_acc, _TURNING_ACC_BP, _TURNING_ACC_V)
    # LEAVING
    elif self.state == VisionState.leaving:
      # When leaving, we provide a comfortable acceleration to regain speed.
      a_target = _LEAVING_ACC
    else:
      raise NotImplementedError(f"SCC-V state not supported: {self.state}")

    return a_target

  def update(self, sm: messaging.SubMaster, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float,
             v_cruise_setpoint: float) -> None:
    self.long_enabled = long_enabled
    self.long_override = long_override
    self.v_ego = v_ego
    self.a_ego = a_ego
    self.v_cruise_setpoint = v_cruise_setpoint

    self._update_params()
    self._update_calculations(sm)

    self.is_enabled, self.is_active = self._update_state_machine()
    self.a_target = self._update_solution()

    self.output_v_target = self.get_v_target_from_control()
    self.output_a_target = self.get_a_target_from_control()

    self.frame += 1
