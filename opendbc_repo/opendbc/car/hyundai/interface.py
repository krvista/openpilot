from opendbc.car import Bus, get_safety_config, structs, uds
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import HyundaiFlags, CAR, DBC, \
                                                   CANFD_UNSUPPORTED_LONGITUDINAL_CAR, \
                                                   UNSUPPORTED_LONGITUDINAL_CAR, HyundaiSafetyFlags
from opendbc.car.hyundai.radar_interface import RADAR_START_ADDR
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.disable_ecu import disable_ecu
from opendbc.car.hyundai.carcontroller import CarController
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.radar_interface import RadarInterface

from opendbc.sunnypilot.car.hyundai.escc import ESCC_MSG
from opendbc.sunnypilot.car.hyundai.longitudinal.helpers import get_longitudinal_tune
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP, HyundaiSafetyFlagsSP

ButtonType = structs.CarState.ButtonEvent.Type
Ecu = structs.CarParams.Ecu

# Cancel button can sometimes be ACC pause/resume button, main button can also enable on some cars
ENABLE_BUTTONS = (ButtonType.accelCruise, ButtonType.decelCruise, ButtonType.cancel, ButtonType.mainCruise)


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  DRIVABLE_GEARS = (structs.CarState.GearShifter.sport, structs.CarState.GearShifter.manumatic)

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "hyundai"

    # "LKA steering" if LKAS or LKAS_ALT messages are seen coming from the camera.
    # Generally means our LKAS message is forwarded to another ECU (commonly ADAS ECU)
    # that finally retransmits our steering command in LFA or LFA_ALT to the MDPS.
    # "LFA steering" if camera directly sends LFA to the MDPS
    cam_can = CanBus(None, fingerprint).CAM
    lka_steering = 0x50 in fingerprint[cam_can] or 0x110 in fingerprint[cam_can]
    CAN = CanBus(None, fingerprint, lka_steering)

    if ret.flags & HyundaiFlags.CANFD:
      # Shared configuration for CAN-FD cars
      ret.alphaLongitudinalAvailable = candidate not in CANFD_UNSUPPORTED_LONGITUDINAL_CAR
      if lka_steering and Ecu.adas not in [fw.ecu for fw in car_fw]:
        # this needs to be figured out for cars without an ADAS ECU
        ret.alphaLongitudinalAvailable = False

      ret.enableBsm = 0x1e5 in fingerprint[CAN.ECAN]

      # Check if the car is hybrid. Only HEV/PHEV cars have 0xFA on E-CAN.
      if 0xFA in fingerprint[CAN.ECAN]:
        ret.flags |= HyundaiFlags.HYBRID.value

      if lka_steering:
          ret.flags |= HyundaiFlags.CANFD_LKA_STEERING.value
          # HDA2-ALT + CCNC angle-control platform auto-detection:
          # presence of LKAS_ALT (0x110) on the camera bus indicates the
          # ADAS architecture that commands MDPS via ADAS_StrAnglReqVal.
          # Combined with HyundaiFlags.CCNC from the car's static config
          # in values.py, this auto-enables angle-based steering, rate
          # limiter, hysteresis, camera-ref blend, low-speed camera
          # passthrough, ACI gain policy, and op-only alert suppression
          # — no per-car code is required; future 2025+ Hyundai/Kia/
          # Genesis cars sharing this ADAS architecture inherit all
          # behaviour automatically via this fingerprint-driven flag.
          if 0x110 in fingerprint[CAN.CAM]:
              ret.flags |= HyundaiFlags.CANFD_LKA_STEERING_ALT.value
      else:
          if not ret.flags & HyundaiFlags.RADAR_SCC:
              ret.flags |= HyundaiFlags.CANFD_CAMERA_SCC.value
      
      # CANFD_ALT_BUTTONS detection applies to both LKA and non-LKA variants.
      if 0x1cf not in fingerprint[CAN.ECAN]:
          ret.flags |= HyundaiFlags.CANFD_ALT_BUTTONS.value

      # Some LKA steering cars have alternative messages for gear checks
      # ICE cars do not have 0x130; GEARS message on 0x40 or 0x70 instead
      if 0x130 not in fingerprint[CAN.ECAN]:
        if 0x40 not in fingerprint[CAN.ECAN]:
          ret.flags |= HyundaiFlags.CANFD_ALT_GEARS_2.value
        else:
          ret.flags |= HyundaiFlags.CANFD_ALT_GEARS.value

      cfgs = [get_safety_config(structs.CarParams.SafetyModel.hyundaiCanfd), ]
      if CAN.ECAN >= 4:
        cfgs.insert(0, get_safety_config(structs.CarParams.SafetyModel.noOutput))
      ret.safetyConfigs = cfgs

      if ret.flags & HyundaiFlags.CANFD_LKA_STEERING:
        ret.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.CANFD_LKA_STEERING.value
        if ret.flags & HyundaiFlags.CANFD_LKA_STEERING_ALT:
          ret.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.CANFD_LKA_STEERING_ALT.value
      if ret.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
        ret.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.CANFD_ALT_BUTTONS.value
      if ret.flags & HyundaiFlags.CANFD_CAMERA_SCC:
        ret.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.CAMERA_SCC.value
      # CCNC safety flag: required for panda to allow 0x161/0x162 TX on
      # any member of the HDA2-ALT + CCNC angle-control platform (Ioniq
      # 6 N and future 2025+ Hyundai/Kia/Genesis cars that share the
      # CCNC | CANFD_LKA_STEERING_ALT flag combo), so we can suppress
      # spurious hands-on / HDP takeover alerts while openpilot steers.
      # Note: on HDA2-ALT the 0x161/0x162 publisher is native on bus 1
      # (not camera-forwarded), so panda cannot block the stock source;
      # suppression is best-effort (see c6a33de for the safety-side
      # check_relay=false requirement on this platform).
      if ret.flags & HyundaiFlags.CCNC and (
        not (ret.flags & HyundaiFlags.CANFD_LKA_STEERING) or (ret.flags & HyundaiFlags.CANFD_LKA_STEERING_ALT)
      ):
        ret.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.CCNC.value

    else:
      # Shared configuration for non CAN-FD cars
      ret.alphaLongitudinalAvailable = candidate not in UNSUPPORTED_LONGITUDINAL_CAR
      ret.enableBsm = 0x58b in fingerprint[0]

      # Send LFA message on cars with HDA
      if 0x485 in fingerprint[2]:
        ret.flags |= HyundaiFlags.SEND_LFA.value

      # These cars use the FCA11 message for the AEB and FCW signals, all others use SCC12
      if 0x38d in fingerprint[0] or 0x38d in fingerprint[2]:
        ret.flags |= HyundaiFlags.USE_FCA.value

      if ret.flags & HyundaiFlags.LEGACY:
        # these cars require a special panda safety mode due to missing counters and checksums in the messages
        ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.hyundaiLegacy)]
      else:
        ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.hyundai, 0)]

      if ret.flags & HyundaiFlags.CAMERA_SCC:
        ret.safetyConfigs[0].safetyParam |= HyundaiSafetyFlags.CAMERA_SCC.value

      # These cars have the LFA button on the steering wheel
      if 0x391 in fingerprint[0]:
        ret.flags |= HyundaiFlags.HAS_LDA_BUTTON.value

    # Common lateral control setup

    ret.centerToFront = ret.wheelbase * 0.4
    ret.steerActuatorDelay = 0.1
    ret.steerLimitTimer = 0.4

    # HDA2-ALT + CCNC angle-control platform (Ioniq 6 N 2026 and future
    # 2025+ Hyundai/Kia/Genesis cars with CCNC | CANFD_LKA_STEERING_ALT)
    # uses angle-based control via LKAS_ALT's ADAS_StrAnglReqVal field.
    # LatControlAngle provides live steerRatio, angleOffsetDeg, and roll
    # compensation from liveParameters, which are critical for accurate
    # angle commands.
    if ret.flags & HyundaiFlags.CCNC and ret.flags & HyundaiFlags.CANFD_LKA_STEERING_ALT:
      ret.steerControlType = structs.CarParams.SteerControlType.angle
      # Phase 11: 20 km/h -> 5 km/h. controlsd's standstill gate
      # (vEgo <= minSteerSpeed) silently killed CC.latActive below 20 km/h:
      # routes 0x10-0x28 measured 27% of MADS-enabled time (1.4 h) with the
      # UI green but no steering, 31 silent dropouts/h in city stop-and-go.
      # The carcontroller's own low-speed machinery (LOW_SPEED_PASSTHROUGH
      # 20/22 km/h latch, traffic_following hold, parking-mode latch) was
      # built to manage exactly this regime and never ran because this gate
      # sits upstream of it. 5 km/h keeps a floor at walking pace (angle
      # control while nearly stopped serves no purpose; resume is wheel-
      # anchored) and lets the low-speed latches own 5-20 km/h as designed.
      # Kill switch: 20.0 / 3.6 restores the pre-Phase-11 gate.
      ret.minSteerSpeed = 5.0 / 3.6
    else:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    if ret.flags & HyundaiFlags.ALT_LIMITS:
      ret.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.ALT_LIMITS.value

    if ret.flags & HyundaiFlags.ALT_LIMITS_2:
      ret.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.ALT_LIMITS_2.value

      # see https://github.com/commaai/opendbc/pull/1137/
      ret.dashcamOnly = True

    # Common longitudinal control setup

    ret.radarUnavailable = RADAR_START_ADDR not in fingerprint[1] or Bus.radar not in DBC[ret.carFingerprint]
    ret.openpilotLongitudinalControl = alpha_long and ret.alphaLongitudinalAvailable
    ret.pcmCruise = not ret.openpilotLongitudinalControl
    ret.startingState = True
    ret.vEgoStarting = 0.1
    ret.startAccel = 1.0
    ret.longitudinalActuatorDelay = 0.5

    if ret.openpilotLongitudinalControl:
      ret.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.LONG.value
    if ret.flags & HyundaiFlags.HYBRID:
      ret.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.HYBRID_GAS.value
    elif ret.flags & HyundaiFlags.EV:
      ret.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.EV_GAS.value
    elif ret.flags & HyundaiFlags.FCEV:
      ret.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.FCEV_GAS.value

    # Car specific configuration overrides

    if candidate == CAR.KIA_OPTIMA_G4_FL:
      ret.steerActuatorDelay = 0.2

    # Dashcam cars are missing a test route, or otherwise need validation
    # TODO: Optima Hybrid 2017 uses a different SCC12 checksum
    if candidate in (CAR.KIA_OPTIMA_H,):
      ret.dashcamOnly = True

    return ret

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                     car_fw: list[structs.CarParams.CarFw], alpha_long: bool, is_release_sp: bool, docs: bool) -> structs.CarParamsSP:
    # identical logic used in _get_params
    # "LKA steering" if LKAS or LKAS_ALT messages are seen coming from the camera.
    # Generally means our LKAS message is forwarded to another ECU (commonly ADAS ECU)
    # that finally retransmits our steering command in LFA or LFA_ALT to the MDPS.
    # "LFA steering" if camera directly sends LFA to the MDPS
    cam_can = CanBus(None, fingerprint).CAM
    lka_steering = 0x50 in fingerprint[cam_can] or 0x110 in fingerprint[cam_can]
    CAN = CanBus(None, fingerprint, lka_steering)

    if not stock_cp.flags & HyundaiFlags.CANFD:
      # TODO-SP: add route with ESCC message for process replay
      if ESCC_MSG in fingerprint[0]:
        ret.flags |= HyundaiFlagsSP.ENHANCED_SCC.value

    if ret.flags & HyundaiFlagsSP.ENHANCED_SCC:
      ret.safetyParam |= HyundaiSafetyFlagsSP.ESCC
      stock_cp.radarUnavailable = False

    if stock_cp.flags & HyundaiFlags.HAS_LDA_BUTTON:
      ret.safetyParam |= HyundaiSafetyFlagsSP.HAS_LDA_BUTTON

    if stock_cp.flags & (HyundaiFlags.CANFD_CAMERA_SCC | HyundaiFlags.CAMERA_SCC):
      stock_cp.radarUnavailable = False

    if stock_cp.flags & HyundaiFlags.ALT_LIMITS_2:
      stock_cp.dashcamOnly = False

    if ret.flags & HyundaiFlagsSP.NON_SCC:
      stock_cp.alphaLongitudinalAvailable = False
      stock_cp.openpilotLongitudinalControl = False
      stock_cp.pcmCruise = True
      ret.safetyParam |= HyundaiSafetyFlagsSP.NON_SCC

    # untested non-SCC platforms, need user validations
    if stock_cp.carFingerprint in (CAR.HYUNDAI_BAYON_1ST_GEN_NON_SCC, CAR.KIA_FORTE_2021_NON_SCC,
                                   CAR.KIA_SELTOS_2023_NON_SCC, CAR.GENESIS_G70_2021_NON_SCC):
      stock_cp.dashcamOnly = True

    if stock_cp.flags & HyundaiFlags.CANFD:
      if 0x1fa in fingerprint[CAN.ECAN]:
        ret.flags |= HyundaiFlagsSP.SPEED_LIMIT_AVAILABLE.value
    else:
      # Detect smartMDPS, which bypasses EPS low-speed lockout, allowing sunnypilot to send steering commands down to 0
      if 0x2AA in fingerprint[0]:
        stock_cp.minSteerSpeed = 0.0
        stock_cp.flags &= ~HyundaiFlags.MIN_STEER_32_MPH.value

      if 0x544 in fingerprint[0]:
        ret.flags |= HyundaiFlagsSP.SPEED_LIMIT_AVAILABLE.value

      if 0x53E in fingerprint[2]:
        ret.flags |= HyundaiFlagsSP.HAS_LKAS12.value

    ret.intelligentCruiseButtonManagementAvailable = not (stock_cp.flags & HyundaiFlags.CANFD_ALT_BUTTONS)

    return ret

  @staticmethod
  def _get_longitudinal_tuning_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP) -> structs.CarParamsSP:
    if ret.flags & (HyundaiFlagsSP.LONG_TUNING_DYNAMIC | HyundaiFlagsSP.LONG_TUNING_PREDICTIVE):
      get_longitudinal_tune(stock_cp)

    return ret

  @staticmethod
  def init(CP, CP_SP, can_recv, can_send, communication_control=None):
    # 0x80 silences response
    if communication_control is None:
      communication_control = bytes([uds.SERVICE_TYPE.COMMUNICATION_CONTROL, 0x80 | uds.CONTROL_TYPE.DISABLE_RX_DISABLE_TX, uds.MESSAGE_TYPE.NORMAL])

    if CP.openpilotLongitudinalControl and not ((CP.flags & (HyundaiFlags.CANFD_CAMERA_SCC | HyundaiFlags.CAMERA_SCC)) or
                                                (CP_SP.flags & HyundaiFlagsSP.ENHANCED_SCC)):
      addr, bus = 0x7d0, CanBus(CP).ECAN if CP.flags & HyundaiFlags.CANFD else 0
      if CP.flags & HyundaiFlags.CANFD_LKA_STEERING.value:
        addr, bus = 0x730, CanBus(CP).ECAN
      disable_ecu(can_recv, can_send, bus=bus, addr=addr, com_cont_req=communication_control)

    # for blinkers
    if CP.flags & HyundaiFlags.ENABLE_BLINKERS:
      disable_ecu(can_recv, can_send, bus=CanBus(CP).ECAN, addr=0x7B1, com_cont_req=communication_control)

  @staticmethod
  def deinit(CP, can_recv, can_send):
    communication_control = bytes([uds.SERVICE_TYPE.COMMUNICATION_CONTROL, 0x80 | uds.CONTROL_TYPE.ENABLE_RX_ENABLE_TX, uds.MESSAGE_TYPE.NORMAL])
    CarInterface.init(CP, can_recv, can_send, communication_control)
