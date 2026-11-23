"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.cereal import log, custom

from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiFlags
from openpilot.common.params import Params
from openpilot.sunnypilot.mads.helpers import MadsSteeringModeOnBrake, read_steering_mode_param, MADS_NO_ACC_MAIN_BUTTON
from openpilot.sunnypilot.mads.state import StateMachine, GEARS_ALLOW_PAUSED_SILENT

State = custom.ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState
ButtonType = structs.CarState.ButtonEvent.Type
EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
GearShifter = structs.CarState.GearShifter
SafetyModel = structs.CarParams.SafetyModel

SET_SPEED_BUTTONS = (ButtonType.accelCruise, ButtonType.resumeCruise, ButtonType.decelCruise, ButtonType.setCruise)
IGNORED_SAFETY_MODES = (SafetyModel.silent, SafetyModel.noOutput)

# i6n Phase 75m (report §103.3-4), CCNC angle-steering platform only (self.main_gated). There the panda cannot see the
# LFA button (it reaches the car on the camera's LKAS_ALT only) and the cruise buttons are not decoded, so the panda
# grants lateral on the SCC main edge or through the MADS heartbeat and never without main.
#   LKAS_BUTTON_NEEDS_MAIN  the LFA button is not an enable source while cruiseState.available is False (it enabled
#                           MADS, the panda never granted, 2 s later 'Controls Mismatch: Lateral'); the press shows
#                           cruiseMainRequired instead. Switching off with the button is unchanged. Nor does a paused
#                           MADS resume while main is off. Kill: False.
#   CRUISE_EDGE_ENABLE      with Unified Engagement Mode on and MADS off, the rising edge of cruiseState.enabled raises
#                           lkasEnable (engage chime) - only with cruiseState.available, not while a panda reports
#                           invalid RX checks and not after a mismatch trip until the next main-on edge (the panda
#                           would not grant: 2 s later the mismatch alert). The usual no-entry conditions apply. It also
#                           brings lateral back at the next cruise engagement after the driver switched it off with
#                           the LFA button; MadsUnifiedEngagementMode off keeps 'cruise only'. Kill: False.
# i6n Phase 75l: the lateral mismatch counter only ever grew while MADS stayed active, so short panda gaps (each grant
# wait leaves 0-100) added up to the 200-frame trip. It now loses one count per LATERAL_MISMATCH_DECAY_FRAMES frames in
# which the panda agrees - a decay, not a reset: a grant that keeps flapping (more than 1 / (1 + N) of the time off)
# still trips. Kill: 0.
LKAS_BUTTON_NEEDS_MAIN = True
CRUISE_EDGE_ENABLE = True
LATERAL_MISMATCH_DECAY_FRAMES = 5


class ModularAssistiveDrivingSystem:
  def __init__(self, selfdrive):
    self.CP = selfdrive.CP
    self.CP_SP = selfdrive.CP_SP
    self.params = selfdrive.params

    self.enabled = False
    self.active = False
    self.available = False
    self.lateral_mismatch_counter = 0
    self.lateral_agree_frames = 0   # Phase 75l
    self.allow_always = False
    self.main_gated = False         # Phase 75m
    self.cruise_edge_blocked = False   # Phase 75m: a mismatch trip happened; cleared by the next main-on edge
    self.no_main_cruise = False
    self.selfdrive = selfdrive
    self.selfdrive.enabled_prev = False
    self.state_machine = StateMachine(self)
    self.events = self.selfdrive.events
    self.events_sp = self.selfdrive.events_sp
    self.disengage_on_accelerator = Params().get_bool("DisengageOnAccelerator")
    if self.CP.brand == "hyundai":
      if self.CP.flags & (HyundaiFlags.HAS_LDA_BUTTON | HyundaiFlags.CANFD):
        self.allow_always = True
      # Phase 75m: CCNC angle steering (LKAS_ALT) - see LKAS_BUTTON_NEEDS_MAIN
      self.main_gated = bool(self.CP.flags & HyundaiFlags.CCNC and self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG_ALT)
    if self.CP.brand == "tesla":
      self.allow_always = True

    if self.CP.brand in MADS_NO_ACC_MAIN_BUTTON:
      self.no_main_cruise = True

    # read params on init
    self.enabled_toggle = self.params.get_bool("Mads")
    self.main_enabled_toggle = self.params.get_bool("MadsMainCruiseAllowed")
    self.steering_mode_on_brake = read_steering_mode_param(self.CP, self.CP_SP, self.params)
    self.unified_engagement_mode = self.params.get_bool("MadsUnifiedEngagementMode")

    # Boot-time auto-enable retry: the initial cruiseState.available rising
    # edge often fires while selfdriveInitializing or commIssue (both
    # NO_ENTRY) are active, blocking the MADS state transition. This flag
    # keeps retrying silently until MADS successfully enables or the user
    # explicitly disables via LFA button.
    self._boot_enable_pending = True

  def read_params(self):
    self.main_enabled_toggle = self.params.get_bool("MadsMainCruiseAllowed")
    self.unified_engagement_mode = self.params.get_bool("MadsUnifiedEngagementMode")

  def pedal_pressed_non_gas_pressed(self, CS: structs.CarState) -> bool:
    # ignore `pedalPressed` events caused by gas presses
    if self.events.has(EventName.pedalPressed) and not (CS.gasPressed and not self.selfdrive.CS_prev.gasPressed and self.disengage_on_accelerator):
      return True

    return False

  def should_silent_lkas_enable(self, CS: structs.CarState) -> bool:
    if self.steering_mode_on_brake == MadsSteeringModeOnBrake.PAUSE and (CS.brakePressed or CS.regenBraking or self.pedal_pressed_non_gas_pressed(CS)):
      return False

    if self.events_sp.contains_in_list(GEARS_ALLOW_PAUSED_SILENT):
      return False

    return True

  def block_unified_engagement_mode(self) -> bool:
    # UEM disabled
    if not self.unified_engagement_mode:
      return True

    if self.enabled:
      return True

    if self.selfdrive.enabled and self.selfdrive.enabled_prev:
      return True

    return False

  def get_wrong_car_mode(self, alert_only: bool) -> None:
    if alert_only:
      if self.events.has(EventName.wrongCarMode):
        self.replace_event(EventName.wrongCarMode, EventNameSP.wrongCarModeAlertOnly)
    else:
      self.events.remove(EventName.wrongCarMode)

  def transition_paused_state(self):
    if self.state_machine.state != State.paused:
      self.events_sp.add(EventNameSP.silentLkasDisable)

  def replace_event(self, old_event: int, new_event: int):
    self.events.remove(old_event)
    self.events_sp.add(new_event)

  def data_sample(self):
    # When the safety and selfdrived do not agree on controls_allowed_lateral
    # we want to disengage sunnypilot. However the status from the panda goes through
    # another socket other than the CAN messages and one can arrive earlier than the other.
    # Therefore we allow a mismatch for two samples, then we trigger the disengagement.
    if not self.active or self.selfdrive.enabled:
      self.lateral_mismatch_counter = 0
      self.lateral_agree_frames = 0
    elif any(not ps.controlsAllowedLateral for ps in self.selfdrive.sm['pandaStates']
             if ps.safetyModel not in IGNORED_SAFETY_MODES):
      self.lateral_mismatch_counter += 1
      self.lateral_agree_frames = 0
    elif LATERAL_MISMATCH_DECAY_FRAMES > 0 and self.lateral_mismatch_counter > 0:
      # Phase 75l: the panda agrees -> slow decay
      self.lateral_agree_frames += 1
      if self.lateral_agree_frames >= LATERAL_MISMATCH_DECAY_FRAMES:
        self.lateral_agree_frames = 0
        self.lateral_mismatch_counter -= 1

  def main_off_blocks_enable(self, CS: structs.CarState) -> bool:
    """Phase 75m: on the main-gated platform lateral cannot be granted while SCC main is off."""
    return bool(LKAS_BUTTON_NEEDS_MAIN and self.main_gated and not CS.cruiseState.available)

  def cruise_edge_enable(self, CS: structs.CarState) -> bool:
    """Phase 75m: the driver engaged cruise with MADS off (main on) on a car whose cruise buttons cannot be read."""
    if not (CRUISE_EDGE_ENABLE and self.main_gated and self.unified_engagement_mode) or self.enabled:
      return False
    if not (CS.cruiseState.enabled and not self.selfdrive.CS_prev.cruiseState.enabled and CS.cruiseState.available):
      return False
    # a panda that dropped lateral for invalid RX checks re-arms on a main edge only: not while they are invalid, and
    # not after a mismatch trip until main has been switched on again
    if self.cruise_edge_blocked:
      return False
    return not any(ps.safetyRxChecksInvalid for ps in self.selfdrive.sm['pandaStates']
                   if ps.safetyModel not in IGNORED_SAFETY_MODES)

  def update_events(self, CS: structs.CarState):
    if not self.selfdrive.enabled and self.enabled:
      if CS.standstill:
        if self.events.has(EventName.doorOpen):
          self.replace_event(EventName.doorOpen, EventNameSP.silentDoorOpen)
          self.transition_paused_state()
        if self.events.has(EventName.seatbeltNotLatched):
          self.replace_event(EventName.seatbeltNotLatched, EventNameSP.silentSeatbeltNotLatched)
          self.transition_paused_state()
      if self.events.has(EventName.wrongGear) and (CS.vEgo < 2.5 or CS.gearShifter == GearShifter.reverse):
        self.replace_event(EventName.wrongGear, EventNameSP.silentWrongGear)
        self.transition_paused_state()
      if self.events.has(EventName.reverseGear):
        self.replace_event(EventName.reverseGear, EventNameSP.silentReverseGear)
        self.transition_paused_state()
      if self.events.has(EventName.brakeHold):
        self.replace_event(EventName.brakeHold, EventNameSP.silentBrakeHold)
        self.transition_paused_state()
      if self.events.has(EventName.parkBrake):
        self.replace_event(EventName.parkBrake, EventNameSP.silentParkBrake)
        self.transition_paused_state()

      if self.steering_mode_on_brake == MadsSteeringModeOnBrake.PAUSE:
        if self.pedal_pressed_non_gas_pressed(CS):
          self.transition_paused_state()

      self.events.remove(EventName.preEnableStandstill)
      self.events.remove(EventName.belowEngageSpeed)
      self.events.remove(EventName.speedTooLow)
      self.events.remove(EventName.cruiseDisabled)
      self.events.remove(EventName.manualRestart)
      self.events.remove(EventName.espActive)

    selfdrive_enable_events = self.events.has(EventName.pcmEnable) or self.events.has(EventName.buttonEnable)
    set_speed_btns_enable = any(be.type in SET_SPEED_BUTTONS for be in CS.buttonEvents)

    # wrongCarMode alert only or actively block control
    self.get_wrong_car_mode(selfdrive_enable_events or set_speed_btns_enable)

    if selfdrive_enable_events:
      if self.pedal_pressed_non_gas_pressed(CS):
        self.events_sp.add(EventNameSP.pedalPressedAlertOnly)

      if self.block_unified_engagement_mode():
        self.events.remove(EventName.pcmEnable)
        self.events.remove(EventName.buttonEnable)
    else:
      if self.main_enabled_toggle:
        if CS.cruiseState.available and not self.selfdrive.CS_prev.cruiseState.available:
          self.events_sp.add(EventNameSP.lkasEnable)
      # Boot-enable: retry silently every frame until MADS engages.
      # Separated from main_enabled_toggle so it fires regardless of
      # MadsMainCruiseAllowed — covers the race condition where vehicle
      # boots before comma (cruiseState.available already True, rising
      # edge fires during NO_ENTRY window and is never retried).
      # For allow_always platforms (CANFD Hyundai), this guarantees MADS
      # auto-enables on boot even if MadsMainCruiseAllowed=False.
      # Respect the configured brake-pedal behavior while retrying: the
      # PAUSE/DISENGAGE modes gate user-initiated enables on the pedal
      # (should_silent_lkas_enable / the lkasEnable removal below), and the
      # silent retry must not slip past that gate — otherwise MADS enables
      # against the pedal for one frame and immediately disengages again
      # (enable/disable flap + alert churn with the brake held at boot).
      # The retry simply resumes once the pedal is released.
      if self._boot_enable_pending and CS.cruiseState.available and not self.enabled and \
         (self.main_enabled_toggle or self.allow_always) and \
         self.should_silent_lkas_enable(CS) and \
         not (self.steering_mode_on_brake == MadsSteeringModeOnBrake.DISENGAGE and self.pedal_pressed_non_gas_pressed(CS)):
        self.events_sp.add(EventNameSP.silentLkasEnable)
      # Phase 75m: cruise engaged with MADS off
      if self.cruise_edge_enable(CS):
        self.events_sp.add(EventNameSP.lkasEnable)

    for be in CS.buttonEvents:
      if be.type == ButtonType.cancel:
        if not self.selfdrive.enabled and self.selfdrive.enabled_prev:
          self.events_sp.add(EventNameSP.manualLongitudinalRequired)
      if be.type == ButtonType.lkas and be.pressed and (CS.cruiseState.available or self.allow_always):
        if self.enabled:
          if self.selfdrive.enabled:
            self.events_sp.add(EventNameSP.manualSteeringRequired)
          else:
            self.events_sp.add(EventNameSP.lkasDisable)
            self._boot_enable_pending = False
        elif self.main_off_blocks_enable(CS):
          self.events_sp.add(EventNameSP.cruiseMainRequired)   # Phase 75m: no entry, tell the driver why
        else:
          self.events_sp.add(EventNameSP.lkasEnable)

    if not CS.cruiseState.available and not self.no_main_cruise:
      self.events.remove(EventName.buttonEnable)
      if self.selfdrive.CS_prev.cruiseState.available:
        self.events_sp.add(EventNameSP.lkasDisable)

    if self.steering_mode_on_brake == MadsSteeringModeOnBrake.DISENGAGE:
      if self.pedal_pressed_non_gas_pressed(CS):
        if self.enabled:
          self.events_sp.add(EventNameSP.lkasDisable)
        else:
          # block lkasEnable if being sent, then send pedalPressedAlertOnly event
          if self.events_sp.contains(EventNameSP.lkasEnable):
            self.events_sp.remove(EventNameSP.lkasEnable)
            self.events_sp.add(EventNameSP.pedalPressedAlertOnly)

    if self.should_silent_lkas_enable(CS):
      if self.state_machine.state == State.paused and not self.main_off_blocks_enable(CS):   # Phase 75m
        self.events_sp.add(EventNameSP.silentLkasEnable)

    if CS.cruiseState.available and not self.selfdrive.CS_prev.cruiseState.available:
      self.cruise_edge_blocked = False   # Phase 75m: the main edge re-arms the panda
    if self.lateral_mismatch_counter >= 200:
      self.events_sp.add(EventNameSP.controlsMismatchLateral)
      self.cruise_edge_blocked = True

    self.events.remove(EventName.pcmDisable)
    self.events.remove(EventName.buttonCancel)
    self.events.remove(EventName.pedalPressed)
    self.events.remove(EventName.wrongCruiseMode)

  def update(self, CS: structs.CarState):
    if not self.enabled_toggle:
      return

    self.data_sample()

    self.update_events(CS)

    if not self.CP.passive and self.selfdrive.initialized:
      self.enabled, self.active = self.state_machine.update()

    if self.enabled:
      self._boot_enable_pending = False

    # Copy of previous SelfdriveD states for MADS events handling
    self.selfdrive.enabled_prev = self.selfdrive.enabled
