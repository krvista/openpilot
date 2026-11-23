"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math
import time

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log, custom

from opendbc.car import structs
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.livedelay.helpers import get_lat_delay
from openpilot.sunnypilot.modeld_v2.modeld_base import ModelStateBase
from openpilot.sunnypilot.selfdrive.controls.lib.blinker_pause_lateral import BlinkerPauseLateral
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as LatControlTorqueV0

# Phase 75 (report §103):
# 75l  CarControlSP.latGranted = the panda allows lateral (pandaStates.controlsAllowedLateral, the field the MADS mismatch
#      check uses; pandas in silent / noOutput are skipped as there). After an LFA-button re-enable the panda grants at
#      its next 1 Hz heartbeat check, 0-1 s later, and rejected every active frame until then; the car controller now
#      sends passive frames meanwhile. pandaStates is 10 Hz, so a normal engage starts at most ~0.1 s later. Missing /
#      stale / empty pandaStates -> True (as before Phase 75; the panda enforces either way), so it cannot deadlock.
#      Kill: LAT_GRANT_FROM_PANDA = False.
# 75k  request paused: the car controller is not delivering the request by design (carStateSP.lateralControlPaused /
#      lateralPausedReason) or, in the solo profile, the factory LFA is steering (gatewayFlags) - the saturation alert
#      stays off then (latcontrol_angle.py). A missing / stale carStateSP reads as not paused.
LAT_GRANT_FROM_PANDA = True
IGNORED_SAFETY_MODES = (structs.CarParams.SafetyModel.silent, structs.CarParams.SafetyModel.noOutput)
GW_SOLO_LFA_ACTIVE = (1 << 4) | (1 << 2)   # carStateSP.gatewayFlags: GW_SOLO | GW_LFA_ACTIVE (opendbc hyundai carcontroller.py)
GW_SOLO_FOLLOW = (1 << 4) | (1 << 6)       # GW_SOLO | GW_FOLLOW: the follow band holds the request on the wheel (icon 2 / 3)
PAUSED_WHILE_FOLLOW_BAND = True            # Phase 75k kill switch for the follow-band term


def _fresh(sm, service: str) -> bool:
  """the service has been received and is alive (False for a SubMaster without it)"""
  alive = getattr(sm, "alive", None)
  seen = getattr(sm, "seen", None)
  return bool(alive is not None and seen is not None and alive.get(service, False) and seen.get(service, False))


class ControlsExt(ModelStateBase):
  def __init__(self, CP: structs.CarParams, params: Params):
    ModelStateBase.__init__(self)
    self.CP = CP
    self.params = params
    self._param_update_time: float = 0.0
    self.blinker_pause_lateral = BlinkerPauseLateral()

    cloudlog.info("controlsd_ext is waiting for CarParamsSP")
    self.CP_SP = messaging.log_from_bytes(params.get("CarParamsSP", block=True), custom.CarParamsSP)
    cloudlog.info("controlsd_ext got CarParamsSP")

    self.sm_services_ext = ['radarState', 'selfdriveStateSP', 'pandaStates', 'carStateSP',   # Phase 75: + pandaStates, carStateSP
                            'gpsLocationExternal']                                        # Phase 76
    self._gps_fix_seen = False
    self.pm_services_ext = ['carControlSP']

  def initialize_lateral_control(self, lac, CI, dt):
    enforce_torque_control = self.params.get_bool("EnforceTorqueControl")
    torque_versions = self.params.get("TorqueControlTune")
    if not enforce_torque_control:
      if self.CP.lateralTuning.which() == 'torque':
        return LatControlTorqueV0(self.CP, self.CP_SP, CI, dt)  # FIXME-SP: revert when upstream fixes tuning issues with v1
      return lac

    if torque_versions == 0.0:  # v0
      return LatControlTorqueV0(self.CP, self.CP_SP, CI, dt)
    else:
      return lac

  def get_params_sp(self, sm: messaging.SubMaster) -> None:
    if time.monotonic() - self._param_update_time > PARAMS_UPDATE_PERIOD:
      self.blinker_pause_lateral.get_params()

      if self.CP.lateralTuning.which() == 'torque':
        self.lat_delay = get_lat_delay(self.params, sm["lateralDelay"].lateralDelay)

      self._param_update_time = time.monotonic()

  def get_lat_active(self, sm: messaging.SubMaster) -> bool:
    if self.blinker_pause_lateral.update(sm['carState']):
      return False

    ss_sp = sm['selfdriveStateSP']
    if ss_sp.mads.available:
      return bool(ss_sp.mads.active)

    # MADS not available, use stock state to engage
    return bool(sm['selfdriveState'].active)

  @staticmethod
  def get_lat_granted(sm: messaging.SubMaster) -> bool:
    """Phase 75l: every panda in a car safety mode allows lateral; True when the panda state is not known."""
    if not LAT_GRANT_FROM_PANDA or not _fresh(sm, 'pandaStates'):
      return True
    return not any(not ps.controlsAllowedLateral for ps in sm['pandaStates']
                   if ps.safetyModel not in IGNORED_SAFETY_MODES)

  @staticmethod
  def get_request_paused(sm: messaging.SubMaster) -> bool:
    """Phase 75k: the car controller reports the request as passive, the factory LFA as steering, or the request as
    held on the wheel by the follow band while the factory LFA releases."""
    if not _fresh(sm, 'carStateSP'):
      return False
    cs_sp = sm['carStateSP']
    return bool(cs_sp.lateralControlPaused or cs_sp.lateralPausedReason != 0
                or (cs_sp.gatewayFlags & GW_SOLO_LFA_ACTIVE) == GW_SOLO_LFA_ACTIVE
                or (PAUSED_WHILE_FOLLOW_BAND and (cs_sp.gatewayFlags & GW_SOLO_FOLLOW) == GW_SOLO_FOLLOW))

  @staticmethod
  def get_lead_data(_lead, src: log.RadarState.LeadData) -> None:
    _lead.dRel = src.dRel
    _lead.yRel = src.yRel
    _lead.vRel = src.vRel
    _lead.aRel = src.deprecated.aRel
    _lead.vLead = src.vLead
    _lead.dPath = src.deprecated.dPath
    _lead.vLat = src.deprecated.vLat
    _lead.vLeadK = src.vLeadK
    _lead.aLeadK = src.aLeadK
    _lead.fcw = src.deprecated.fcw
    _lead.status = src.present
    _lead.aLeadTau = src.aLeadTau
    _lead.modelProb = src.modelProb
    _lead.radar = src.radar
    _lead.radarTrackId = src.radarTrackId

  def state_control_ext(self, sm: messaging.SubMaster) -> custom.CarControlSP:
    CC_SP = custom.CarControlSP.new_message()

    self.get_lead_data(CC_SP.leadOne, sm['radarState'].leadOne)
    self.get_lead_data(CC_SP.leadTwo, sm['radarState'].leadTwo)

    # MADS state
    mads_src = sm['selfdriveStateSP'].mads
    CC_SP.mads.state = mads_src.state
    CC_SP.mads.enabled = mads_src.enabled
    CC_SP.mads.active = mads_src.active
    CC_SP.mads.available = mads_src.available

    # ICBM state
    icbm_src = sm['selfdriveStateSP'].intelligentCruiseButtonManagement
    CC_SP.intelligentCruiseButtonManagement.state = icbm_src.state
    CC_SP.intelligentCruiseButtonManagement.sendButton = icbm_src.sendButton
    CC_SP.intelligentCruiseButtonManagement.vTarget = icbm_src.vTarget

    # Phase 37b-2: op is actively crossing (not merely queued in preLaneChange)
    CC_SP.lateralLaneChangeActive = bool(sm['modelV2'].meta.laneChangeState == log.LaneChangeState.laneChangeStarting)

    # Phase 45: lane-line confidence for the car controller's parking-latch release (0.0 if unavailable/non-finite)
    # (each prob is checked: min(0.9, nan) returns 0.9 in Python, so a NaN on one side must not slip through)
    probs = sm['modelV2'].laneLineProbs
    pl, pr = (float(probs[1]), float(probs[2])) if len(probs) >= 4 else (0.0, 0.0)
    ok = sm.valid['modelV2'] and sm.alive['modelV2'] and math.isfinite(pl) and math.isfinite(pr)
    CC_SP.laneLineProbMin = min(pl, pr) if ok else 0.0

    # Phase 72: the steering profile controlsd latched for this frame (common/steer_profile.py)
    CC_SP.stockLfaWithOp = bool(getattr(self, "_steer_legacy", False))

    # Phase 75l: the panda's lateral permission (see LAT_GRANT_FROM_PANDA)
    CC_SP.latGranted = self.get_lat_granted(sm)

    # Phase 76: latched 'GPS has had a fix since start' for the parking-mode signature
    try:
      if sm.seen['gpsLocationExternal'] and sm['gpsLocationExternal'].hasFix:
        self._gps_fix_seen = True
    except Exception:  # noqa: BLE001 - a missing service leaves the signature on until the latch's own exits
      pass
    CC_SP.gpsFixSeen = bool(getattr(self, '_gps_fix_seen', False))

    return CC_SP

  @staticmethod
  def publish_ext(CC_SP: custom.CarControlSP, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    cc_sp_send = messaging.new_message('carControlSP')
    cc_sp_send.valid = sm['carState'].canValid
    cc_sp_send.carControlSP = CC_SP

    pm.send('carControlSP', cc_sp_send)

  def run_ext(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    CC_SP = self.state_control_ext(sm)
    self.publish_ext(CC_SP, sm, pm)
