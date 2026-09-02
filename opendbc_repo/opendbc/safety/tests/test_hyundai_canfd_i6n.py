"""i6n (Ioniq 6 N CCNC) angle-steering safety tests — i6nv3 bench stage 0.

Runs the upstream LKAS_ALT angle-steering suite under the REAL i6n safety
param (CCNC forces angle steering — no CANFD_ANGLE_STEERING bit), and adds
the model-id path falsification the port review asked for: model 11 carries
baseline-identical params, so a broken encode/decode would silently produce
baseline behaviour and nothing could tell. Proving that a DIFFERENT model id
(KIA_EV9, distinct physics) changes the enforced limits under the very same
CCNC config demonstrates the selector is live; proving id 11 resolves to the
baseline numbers pins the intended (bit-identical) envelope.
"""

from opendbc.car.hyundai.values import HyundaiSafetyFlags
from opendbc.car.lateral import get_max_angle_delta_vm
from opendbc.car.structs import CarParams
from opendbc.sunnypilot.car.hyundai.values import (ANGLE_STEERING_MODEL_BY_CAR, HyundaiAngleSteeringModel,
                                                    encode_angle_model_id)
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.test_hyundai_canfd import TestHyundaiCanfdLKASteeringAltAngle, ANGLE_SAFETY_BASELINE_MODEL, round_angle

I6N_CAR = "HYUNDAI_IONIQ_6_N"
# What interface.py produces for the Ioniq 6 N after fingerprinting:
# EV gas, LKA-msg steering + ALT (0x110 on cam bus), ALT buttons, CCNC.
I6N_SAFETY_PARAM = (HyundaiSafetyFlags.EV_GAS | HyundaiSafetyFlags.CANFD_LKA_STEER_MSG |
                    HyundaiSafetyFlags.CANFD_LKA_STEER_MSG_ALT | HyundaiSafetyFlags.CANFD_ALT_BUTTONS |
                    HyundaiSafetyFlags.CCNC)


class TestI6nCcncAngleSteering(TestHyundaiCanfdLKASteeringAltAngle):
  """Upstream LKAS_ALT angle suite + i6n additions, under the i6n CCNC param."""

  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")  # EV_GAS

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    # model id is resolved INSIDE set_safety_hooks (init), so the SP param must
    # be in place before the hooks are set — same ordering pandad guarantees
    self.safety.set_current_safety_param_sp(encode_angle_model_id(ANGLE_STEERING_MODEL_BY_CAR[I6N_CAR]))
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, int(I6N_SAFETY_PARAM))
    self.safety.init_tests()

  # ALT_BUTTONS platform: the panda samples CRUISE_BUTTONS_ALT (0x1aa), not 0x1cf
  def _button_msg(self, buttons, main_button=0, bus=1):
    values = {"CRUISE_BUTTONS": buttons, "ADAPTIVE_CRUISE_MAIN_BTN": main_button}
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS_ALT", self.PT_BUS, values)

  def _lkas_button_msg(self, enabled):
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS_ALT", self.PT_BUS, {"LDA_BTN": enabled})

  def test_button_sends(self):
    # ALT_BUTTONS car: our CarController never transmits cruise buttons for the
    # CCNC platform (factory SCC owns cancel/resume; see F3 in carcontroller.py)
    # and CRUISE_BUTTONS_ALT (0x1aa) is not on the TX whitelist. The
    # whitelisted 0x1cf is inherited from the upstream LKA_ALT config and is
    # never used by our code — pinned here so a future change is conscious.
    from opendbc.car.hyundai.values import Buttons
    for allowed in (0, 1):
      self.safety.set_controls_allowed(allowed)
      for btn in (Buttons.RES_ACCEL, Buttons.SET_DECEL, Buttons.CANCEL):
        self.assertFalse(self._tx(self._button_msg(btn, bus=self.BUTTONS_TX_BUS)))

  # --- i6n additions ---

  def test_i6n_param_has_no_upstream_angle_bit(self):
    # the whole point of this file: angle enforcement must come from CCNC alone
    assert not (I6N_SAFETY_PARAM & HyundaiSafetyFlags.CANFD_ANGLE_STEERING)
    assert I6N_SAFETY_PARAM & HyundaiSafetyFlags.CCNC == 2048

  def test_i6n_model_id_resolves_to_11(self):
    self.assertEqual(ANGLE_STEERING_MODEL_BY_CAR[I6N_CAR], HyundaiAngleSteeringModel.HYUNDAI_IONIQ_6_N)
    self.assertEqual(self.safety.get_hyundai_angle_model_id(), 11)

  def test_ccnc_forces_angle_enforcement(self):
    # an over-limit angle step must be BLOCKED even though CANFD_ANGLE_STEERING is unset
    speed = 20.0
    self.safety.set_controls_allowed(True)
    self._reset_speed_measurement(speed + 1)
    self._tx(self._angle_cmd_msg(0, True))
    vm = self.get_vm(ANGLE_SAFETY_BASELINE_MODEL)
    max_delta = get_max_angle_delta_vm(speed, vm, self.get_baseline_limits())
    self.assertTrue(self._tx(self._angle_cmd_msg(round_angle(max_delta), True)))
    self._tx(self._angle_cmd_msg(0, True))
    self.assertFalse(self._tx(self._angle_cmd_msg(round_angle(max_delta * 3.0), True)))

  def _delta_allowed(self, speed, delta):
    self.safety.set_controls_allowed(True)
    self._reset_speed_measurement(speed + 1)
    self._tx(self._angle_cmd_msg(0, True))
    return self._tx(self._angle_cmd_msg(delta, True))

  def test_model_11_enforces_baseline_envelope(self):
    # id 11 params are intentionally bit-identical to baseline: the accepted
    # jerk envelope must equal get_max_angle_delta_vm(baseline VM)
    vm = self.get_vm(ANGLE_SAFETY_BASELINE_MODEL)
    lim = self.get_baseline_limits()
    for speed in (5.0, 12.0, 20.0, 30.0):
      d = round_angle(get_max_angle_delta_vm(speed, vm, lim))
      self.assertTrue(self._delta_allowed(speed, d), f"baseline delta {d} rejected at {speed}")
      self.assertFalse(self._delta_allowed(speed, round_angle(d * 1.15, 6)), f"over-baseline accepted at {speed}")

  def test_model_selection_is_live_under_ccnc(self):
    # FALSIFICATION: same CCNC param, different model id -> different envelope.
    # KIA_EV9 (wb 3.1 / sr 16 / mass 2664) has distinct physics from baseline
    # (wb 2.756 / sr 13.7). If the encode->decode->table path were dead, the
    # panda would enforce baseline here and this test would fail.
    ev9 = "KIA_EV9"
    self.safety.set_current_safety_param_sp(encode_angle_model_id(ANGLE_STEERING_MODEL_BY_CAR[ev9]))
    self._reset_safety_hooks()
    self.assertEqual(self.safety.get_hyundai_angle_model_id(), ANGLE_STEERING_MODEL_BY_CAR[ev9])
    vm_ev9, vm_base = self.get_vm(ev9), self.get_vm(ANGLE_SAFETY_BASELINE_MODEL)
    lim = self.get_baseline_limits()
    checked = 0
    for speed in (8.0, 12.0, 20.0, 30.0):
      d_ev9 = get_max_angle_delta_vm(speed, vm_ev9, lim)
      d_base = get_max_angle_delta_vm(speed, vm_base, lim)
      if abs(d_ev9 - d_base) / max(d_base, 1e-9) < 0.03:
        continue  # envelopes too close to discriminate at this speed
      checked += 1
      d_ev9_r = round_angle(d_ev9)
      self.assertTrue(self._delta_allowed(speed, d_ev9_r), f"EV9 delta {d_ev9_r} rejected at {speed}")
      if d_ev9 > d_base:
        # a delta only legal under EV9 physics must pass -> baseline would have blocked it
        probe = round_angle((d_ev9 + d_base) / 2.0)
        self.assertTrue(self._delta_allowed(speed, probe), f"selector dead: baseline envelope enforced at {speed}")
      else:
        probe = round_angle((d_ev9 + d_base) / 2.0, 6)
        self.assertFalse(self._delta_allowed(speed, probe), f"selector dead: baseline envelope enforced at {speed}")
    self.assertGreater(checked, 0, "no speed discriminated EV9 from baseline — test is vacuous")
