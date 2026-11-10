from openpilot.common.parameterized import parameterized

from cereal import car, log
from opendbc.car.car_helpers import interfaces
from opendbc.car.chrysler.values import CAR as CHRYSLER
from opendbc.car.honda.values import CAR as HONDA
from opendbc.car.toyota.values import CAR as TOYOTA
from opendbc.car.nissan.values import CAR as NISSAN
from opendbc.car.gm.values import CAR as GM
from opendbc.car.vehicle_model import VehicleModel
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car.helpers import convert_to_capnp
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle
from openpilot.selfdrive.locationd.helpers import Pose
from openpilot.common.mock.generators import generate_livePose
from openpilot.sunnypilot.selfdrive.car import interfaces as sunnypilot_interfaces
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as LatControlTorqueV0


class TestLatControl:

  @parameterized.expand([(HONDA.HONDA_CIVIC, LatControlPID), (TOYOTA.TOYOTA_RAV4, LatControlTorque),
                         (NISSAN.NISSAN_LEAF, LatControlAngle), (GM.CHEVROLET_BOLT_EUV, LatControlTorque)])
  def test_saturation(self, car_name, controller):
    CarInterface = interfaces[car_name]
    CP = CarInterface.get_non_essential_params(car_name)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
    CI = CarInterface(CP, CP_SP)
    sunnypilot_interfaces.setup_interfaces(CI)
    CP_SP = convert_to_capnp(CP_SP)
    VM = VehicleModel(CP)

    controller = controller(CP.as_reader(), CP_SP.as_reader(), CI, DT_CTRL)

    CS = car.CarState.new_message()
    CS.vEgo = 30
    CS.steeringPressed = False

    params = log.LiveParametersData.new_message()

    lp = generate_livePose()
    pose = Pose.from_live_pose(lp.livePose)

    # Saturate for curvature limited and controller limited
    for _ in range(1000):
      _, _, lac_log = controller.update(True, CS, VM, params, False, 0, pose, True, 0.2)
    assert lac_log.saturated

    for _ in range(1000):
      _, _, lac_log = controller.update(True, CS, VM, params, False, 0, pose, False, 0.2)
    assert not lac_log.saturated

    for _ in range(1000):
      _, _, lac_log = controller.update(True, CS, VM, params, False, 1, pose, False, 0.2)
    assert lac_log.saturated

  @parameterized.expand([(LatControlTorque,), (LatControlTorqueV0,)])
  def test_press_yield_blend(self, controller_cls):
    def make(car_name):
      CarInterface = interfaces[car_name]
      CP = CarInterface.get_non_essential_params(car_name)
      CarInterface.configure_torque_tune(car_name, CP.lateralTuning)
      CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
      CI = CarInterface(CP, CP_SP)
      sunnypilot_interfaces.setup_interfaces(CI)
      return controller_cls(CP.as_reader(), convert_to_capnp(CP_SP).as_reader(), CI, DT_CTRL), VehicleModel(CP)

    params = log.LiveParametersData.new_message()
    pose = Pose.from_live_pose(generate_livePose().livePose)

    def run(controller, VM, pressed, frames):
      CS = car.CarState.new_message(vEgo=30, steeringPressed=pressed)
      blends = []
      for _ in range(frames):
        controller.update(True, CS, VM, params, False, 0.002, pose, False, 0.2)
        blends.append(controller.release_blend)
      return blends

    # Chrysler: the error term ramps out over 0.3 s while pressed and back in over 0.4 s from where it got to
    controller, VM = make(CHRYSLER.JEEP_GRAND_CHEROKEE_2019)
    assert run(controller, VM, False, 10)[-1] == 1.
    b = run(controller, VM, True, 40)
    assert abs(b[14] - 0.5) < 0.02 and b[-1] == 0.
    assert all(x >= y for x, y in zip(b, b[1:]))
    b = run(controller, VM, False, 50)
    assert abs(b[19] - 0.5) < 0.02 and b[-1] == 1.
    # a 0.1 s blip only dips the weight and recovers without a step to 0
    b = run(controller, VM, True, 10) + run(controller, VM, False, 5)
    assert min(b) > 0.6
    # other brands: full correction while pressed, restart from 0 on release (unchanged)
    controller, VM = make(TOYOTA.TOYOTA_RAV4)
    assert set(run(controller, VM, True, 40)) == {1.}
    b = run(controller, VM, False, 41)
    assert b[0] == 0. and b[-1] == 1.

  @parameterized.expand([(LatControlTorque,), (LatControlTorqueV0,)])
  def test_driver_torque_holds_integrator(self, controller_cls):
    def make(car_name):
      CarInterface = interfaces[car_name]
      CP = CarInterface.get_non_essential_params(car_name)
      CarInterface.configure_torque_tune(car_name, CP.lateralTuning)
      CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
      CI = CarInterface(CP, CP_SP)
      sunnypilot_interfaces.setup_interfaces(CI)
      return controller_cls(CP.as_reader(), convert_to_capnp(CP_SP).as_reader(), CI, DT_CTRL), VehicleModel(CP)

    def integrator_moves(controller, VM, steering_torque, pressed=False):
      controller.pid.reset()
      CS = car.CarState.new_message(vEgo=30, steeringTorque=steering_torque, steeringPressed=pressed)
      params = log.LiveParametersData.new_message()
      pose = Pose.from_live_pose(generate_livePose().livePose)
      # small desired curvature: a saturated command would stop the integrator by itself (anti-windup)
      _, _, first = controller.update(True, CS, VM, params, False, 0.0002, pose, False, 0.2)
      for _ in range(100):
        _, _, lac_log = controller.update(True, CS, VM, params, False, 0.0002, pose, False, 0.2)
      return abs(lac_log.i - first.i) > 1e-4

    # Chrysler: hand torque above 40 column-torque units holds the integrator, below steeringPressed's 120
    controller, VM = make(CHRYSLER.JEEP_GRAND_CHEROKEE_2019)
    assert integrator_moves(controller, VM, 0)
    assert integrator_moves(controller, VM, 30)
    assert not integrator_moves(controller, VM, 60)
    assert not integrator_moves(controller, VM, -60)
    assert not integrator_moves(controller, VM, 0, pressed=True)
    # other brands: only steeringPressed holds it
    controller, VM = make(TOYOTA.TOYOTA_RAV4)
    assert integrator_moves(controller, VM, 60)
    assert not integrator_moves(controller, VM, 60, pressed=True)
