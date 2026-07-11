#!/usr/bin/env python3
from types import SimpleNamespace
import random
import unittest
import numpy as np

from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm
from opendbc.car.tesla.teslacan import get_steer_ctrl_type
from opendbc.car.tesla.values import CarControllerParams, TeslaSafetyFlags, TeslaFlags, CANBUS
from opendbc.car.tesla.carcontroller import get_safety_CP
from opendbc.car.structs import CarParams
from opendbc.car.vehicle_model import VehicleModel
from opendbc.can import CANDefine
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety, MAX_SPEED_DELTA, MAX_WRONG_COUNTERS, away_round, round_speed

from opendbc.sunnypilot.car.tesla.speed_profile import (
  AP_FIRST_STABLE_FRAMES,
  SpeedProfileProtocol,
  TeslaSpeedProfileCarController,
)
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP, TeslaSafetyFlagsSP

MSG_DAS_steeringControl = 0x488
MSG_APS_eacMonitor = 0x27d
MSG_DAS_Control = 0x2b9
MSG_GTW_carConfig = 0x398
MSG_UI_driverAssistControl = 0x3f8
MSG_UI_autopilotControl = 0x3fd


def round_angle(apply_angle, can_offset=0):
  apply_angle_can = (apply_angle + 1638.35) / 0.1 + can_offset
  # 0.49999_ == 0.5
  rnd_offset = 1e-5 if apply_angle >= 0 else -1e-5
  return away_round(apply_angle_can + rnd_offset) * 0.1 - 1638.35


class TestTeslaSafetyBase(common.CarSafetyTest, common.AngleSteeringSafetyTest, common.LongitudinalAccelSafetyTest):
  SAFETY_PARAM = 0

  RELAY_MALFUNCTION_ADDRS = {0: (MSG_DAS_steeringControl, MSG_APS_eacMonitor)}
  FWD_BLACKLISTED_ADDRS = {2: [MSG_DAS_steeringControl, MSG_APS_eacMonitor]}
  TX_MSGS = [[MSG_DAS_steeringControl, 0], [MSG_APS_eacMonitor, 0], [MSG_DAS_Control, 0]]

  STANDSTILL_THRESHOLD = 0.1
  GAS_PRESSED_THRESHOLD = 3

  # Angle control limits
  STEER_ANGLE_MAX = 360  # deg
  DEG_TO_CAN = 10

  # Tesla uses get_max_angle_delta_vm and get_max_angle_vm for real lateral accel and jerk limits
  # TODO: integrate this into AngleSteeringSafetyTest
  ANGLE_RATE_BP = None
  ANGLE_RATE_UP = None
  ANGLE_RATE_DOWN = None

  # Real time limits
  LATERAL_FREQUENCY = 50  # Hz

  # Long control limits
  MAX_ACCEL = 2.0
  MIN_ACCEL = -3.48
  INACTIVE_ACCEL = 0.0

  cnt_epas = 0
  cnt_angle_cmd = 0

  packer: CANPackerSafety

  def _get_steer_cmd_angle_max(self, speed):
    return get_max_angle_vm(max(speed, 1), self.VM, CarControllerParams)

  def setUp(self):
    self.VM = VehicleModel(get_safety_CP())
    self.packer = CANPackerSafety("tesla_model3_party")
    self.define = CANDefine("tesla_model3_party")
    self.acc_states = {d: v for v, d in self.define.dv["DAS_control"]["DAS_accState"].items()}
    self.autopark_states = {d: v for v, d in self.define.dv["DI_state"]["DI_autoparkState"].items()}
    self.active_autopark_states = [self.autopark_states[s] for s in ('ACTIVE', 'COMPLETE', 'SELFPARK_STARTED')]

    self.steer_control_types = {d: v for v, d in self.define.dv["DAS_steeringControl"]["DAS_steeringControlType"].items()}

    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.tesla, self.SAFETY_PARAM)
    self.safety.init_tests()

  def _angle_cmd_msg(self, angle: float, state: bool | int, increment_timer: bool = True, bus: int = 0):
    # If FSD 14, translate steer control type to new flipped definition
    if self.safety.get_current_safety_param() & TeslaSafetyFlags.FSD_14:
      state = get_steer_ctrl_type(TeslaFlags.FSD_14, int(state))

    values = {"DAS_steeringAngleRequest": angle, "DAS_steeringControlType": state}
    if increment_timer:
      self.safety.set_timer(self.cnt_angle_cmd * int(1e6 / self.LATERAL_FREQUENCY))
      self.__class__.cnt_angle_cmd += 1
    return self.packer.make_can_msg_safety("DAS_steeringControl", bus, values)

  def _angle_meas_msg(self, angle: float, hands_on_level: int = 0, eac_status: int = 1, eac_error_code: int = 0):
    values = {"EPAS3S_internalSAS": angle, "EPAS3S_handsOnLevel": hands_on_level,
              "EPAS3S_eacStatus": eac_status, "EPAS3S_eacErrorCode": eac_error_code,
              "EPAS3S_sysStatusCounter": self.cnt_epas % 16}
    self.__class__.cnt_epas += 1
    return self.packer.make_can_msg_safety("EPAS3S_sysStatus", 0, values)

  def _user_brake_msg(self, brake, quality_flag: bool = True):
    values = {"ESP_driverBrakeApply": 2 if brake else 1}
    if not quality_flag:
      values["ESP_driverBrakeApply"] = random.choice((0, 3))  # NotInit_orOff, Faulty_SNA
    return self.packer.make_can_msg_safety("ESP_status", 0, values)

  def _speed_msg(self, speed):
    values = {"DI_vehicleSpeed": speed * 3.6}
    return self.packer.make_can_msg_safety("DI_speed", 0, values)

  def _speed_msg_2(self, speed, quality_flag=True):
    values = {"ESP_vehicleSpeed": speed * 3.6, "ESP_wheelSpeedsQF": quality_flag}
    return self.packer.make_can_msg_safety("ESP_B", 0, values)

  def _vehicle_moving_msg(self, speed: float, quality_flag=True):
    values = {"ESP_vehicleStandstillSts": 1 if speed <= self.STANDSTILL_THRESHOLD else 0,
              "ESP_wheelSpeedsQF": quality_flag}
    return self.packer.make_can_msg_safety("ESP_B", 0, values)

  def _user_gas_msg(self, gas):
    values = {"DI_accelPedalPos": gas}
    return self.packer.make_can_msg_safety("DI_systemStatus", 0, values)

  def _pcm_status_msg(self, enable, autopark_state=0):
    values = {
      "DI_cruiseState": 2 if enable else 0,
      "DI_autoparkState": autopark_state,
    }
    return self.packer.make_can_msg_safety("DI_state", 0, values)

  def _long_control_msg(self, set_speed, acc_state=0, jerk_limits=(0, 0), accel_limits=(0, 0), aeb_event=0, bus=0):
    values = {
      "DAS_setSpeed": set_speed,
      "DAS_accState": acc_state,
      "DAS_aebEvent": aeb_event,
      "DAS_jerkMin": jerk_limits[0],
      "DAS_jerkMax": jerk_limits[1],
      "DAS_accelMin": accel_limits[0],
      "DAS_accelMax": accel_limits[1],
    }
    return self.packer.make_can_msg_safety("DAS_control", bus, values)

  def _accel_msg(self, accel: float):
    # For common.LongitudinalAccelSafetyTest
    return self._long_control_msg(10, accel_limits=(accel, max(accel, 0)))

  def test_rx_hook(self):
    # counter check
    for msg_type in ("angle", "long", "speed", "speed_2"):
      # send multiple times to verify counter checks
      for i in range(10):
        if msg_type == "angle":
          msg = self._angle_cmd_msg(0, True, bus=2)
        elif msg_type == "long":
          msg = self._long_control_msg(0, bus=2)
        elif msg_type == "speed":
          msg = self._speed_msg(0)
        elif msg_type == "speed_2":
          msg = self._speed_msg_2(0)

        should_rx = i >= 5
        if not should_rx:
          # mess with checksums
          if msg_type == "angle":
            msg[0].data[3] = 0
          elif msg_type == "long":
            msg[0].data[7] = 0
          elif msg_type == "speed":
            msg[0].data[0] = 0
          elif msg_type == "speed_2":
            msg[0].data[7] = 0

        self.safety.set_controls_allowed(True)
        self.assertEqual(should_rx, self._rx(msg))
        self.assertEqual(should_rx, self.safety.get_controls_allowed())

      # Send static counters
      for i in range(MAX_WRONG_COUNTERS + 1):
        should_rx = i + 1 < MAX_WRONG_COUNTERS
        self.assertEqual(should_rx, self._rx(msg))
        self.assertEqual(should_rx, self.safety.get_controls_allowed())

  def test_vehicle_speed_measurements(self):
    # OVERRIDDEN: 79.1667 is the max speed in m/s
    self._common_measurement_test(self._speed_msg, 0, 285 / 3.6, 1,
                                  self.safety.get_vehicle_speed_min, self.safety.get_vehicle_speed_max)

  def test_rx_hook_speed_mismatch(self):
    # TODO: overridden because of custom rounding
    # Tesla relies on speed for lateral limits close to ISO 11270, so it checks two sources
    for speed in np.arange(0, 40, 0.5):
      # match signal rounding on CAN
      speed = away_round(speed / 0.08 * 3.6) * 0.08 / 3.6
      for speed_delta in np.arange(-5, 5, 0.1):
        speed_2 = max(speed + speed_delta, 0)
        speed_2 = away_round(speed_2 * 2 * 3.6) / 2 / 3.6

        # Set controls allowed in between rx since first message can reset it
        self.assertTrue(self._rx(self._speed_msg(speed)))
        self.safety.set_controls_allowed(True)
        self.assertTrue(self._rx(self._speed_msg_2(speed_2)))

        within_delta = abs(speed - speed_2) <= MAX_SPEED_DELTA
        self.assertEqual(self.safety.get_controls_allowed(), within_delta)

    # Test ESP_B quality flag
    for quality_flag in (True, False):
      self.safety.set_controls_allowed(True)
      self.assertTrue(self._rx(self._speed_msg(0)))
      self.assertEqual(quality_flag, self._rx(self._speed_msg_2(0, quality_flag=quality_flag)))
      self.assertEqual(quality_flag, self.safety.get_controls_allowed())

  def test_user_brake_quality_flag(self):
    for quality_flag in (True, False):
      msg = self._user_brake_msg(True, quality_flag=quality_flag)
      self.assertEqual(quality_flag, self._rx(msg))

  def test_steering_wheel_disengage(self):
    # Tesla disengages when the user forcibly overrides the locked-in angle steering control
    # Either when the hands on level is high, or if there is a high angle rate fault
    for hands_on_level in range(4):
      for eac_status in range(8):
        for eac_error_code in range(16):
          self.safety.set_controls_allowed(True)

          should_disengage = hands_on_level >= 3 or (eac_status == 0 and eac_error_code == 9)
          self.assertTrue(self._rx(self._angle_meas_msg(0, hands_on_level=hands_on_level, eac_status=eac_status,
                                                        eac_error_code=eac_error_code)))
          self.assertNotEqual(should_disengage, self.safety.get_controls_allowed())
          self.assertEqual(should_disengage, self.safety.get_steering_disengage_prev())

          # Should not recover
          self.assertTrue(self._rx(self._angle_meas_msg(0, hands_on_level=0, eac_status=1, eac_error_code=0)))
          self.assertNotEqual(should_disengage, self.safety.get_controls_allowed())
          self.assertFalse(self.safety.get_steering_disengage_prev())

  def test_autopark_summon_while_enabled(self):
    # We should not respect Autopark that activates while controls are allowed
    self._rx(self._pcm_status_msg(True, 0))

    self._rx(self._pcm_status_msg(True, self.autopark_states["SELFPARK_STARTED"]))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))
    self.assertTrue(self._tx(self._long_control_msg(0, acc_state=self.acc_states["ACC_CANCEL_GENERIC_SILENT"])))

    # We should still not respect Autopark if we disengage cruise
    self._rx(self._pcm_status_msg(False, self.autopark_states["SELFPARK_STARTED"]))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self._tx(self._angle_cmd_msg(0, False)))
    self.assertTrue(self._tx(self._long_control_msg(0, acc_state=self.acc_states["ACC_CANCEL_GENERIC_SILENT"])))

  def test_autopark_summon_behavior(self):
    for autopark_state in range(16):
      self._rx(self._pcm_status_msg(False, 0))

      # We shouldn't allow controls if Autopark is an active state
      autopark_active = autopark_state in self.active_autopark_states
      self._rx(self._pcm_status_msg(False, autopark_state))
      self._rx(self._pcm_status_msg(True, autopark_state))
      self.assertNotEqual(autopark_active, self.safety.get_controls_allowed())

      # We should also start blocking all inactive/active openpilot msgs
      self.assertNotEqual(autopark_active, self._tx(self._angle_cmd_msg(0, False)))
      self.assertNotEqual(autopark_active, self._tx(self._angle_cmd_msg(0, True)))
      self.assertNotEqual(autopark_active, self._tx(self._long_control_msg(0, acc_state=self.acc_states["ACC_CANCEL_GENERIC_SILENT"])))
      self.assertNotEqual(autopark_active or not self.LONGITUDINAL, self._tx(self._long_control_msg(0, acc_state=self.acc_states["ACC_ON"])))

      # Regain controls when Autopark disables
      self._rx(self._pcm_status_msg(True, 0))
      self.assertTrue(self.safety.get_controls_allowed())
      self.assertTrue(self._tx(self._angle_cmd_msg(0, False)))
      self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))
      self.assertTrue(self._tx(self._long_control_msg(0, acc_state=self.acc_states["ACC_CANCEL_GENERIC_SILENT"])))
      self.assertEqual(self.LONGITUDINAL, self._tx(self._long_control_msg(0, acc_state=self.acc_states["ACC_ON"])))

  def test_steering_control_type(self):
    # Only angle control is allowed (no LANE_KEEP_ASSIST or EMERGENCY_LANE_KEEP)
    self.safety.set_controls_allowed(True)
    for steer_control_type in range(4):
      should_tx = steer_control_type in (self.steer_control_types["NONE"],
                                         self.steer_control_types["ANGLE_CONTROL"],
                                         self.steer_control_types["LANE_KEEP_ASSIST"])
      self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(0, state=steer_control_type)))

  def test_stock_lkas_passthrough(self):
    # TODO: make these generic passthrough tests
    no_lkas_msg = self._angle_cmd_msg(0, state=False)
    no_lkas_msg_cam = self._angle_cmd_msg(0, state=True, bus=2)
    lkas_msg_cam = self._angle_cmd_msg(0, state=self.steer_control_types['LANE_KEEP_ASSIST'], bus=2)

    # stock system sends no LKAS -> no forwarding, and OP is allowed to TX
    self.assertEqual(1, self._rx(no_lkas_msg_cam))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, no_lkas_msg_cam.addr))
    self.assertTrue(self._tx(no_lkas_msg))

    # stock system sends LKAS -> forwarding, and OP is not allowed to TX
    self.assertEqual(1, self._rx(lkas_msg_cam))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, lkas_msg_cam.addr))
    self.assertFalse(self._tx(no_lkas_msg))

  def test_angle_cmd_when_enabled(self):
    # We properly test lateral acceleration and jerk below
    pass

  def test_lateral_accel_limit(self):
    for speed in np.linspace(0, 40, 100):
      speed = max(speed, 1)
      # match DI_vehicleSpeed rounding on CAN
      speed = round_speed(away_round(speed / 0.08 * 3.6) * 0.08 / 3.6)
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)  # safety fudges the speed

        # angle signal can't represent 0, so it biases one unit down
        angle_unit_offset = -1 if sign == -1 else 0

        # at limit (safety tolerance adds 1)
        max_angle = round_angle(get_max_angle_vm(speed, self.VM, CarControllerParams), angle_unit_offset + 1) * sign
        max_angle = np.clip(max_angle, -self.STEER_ANGLE_MAX, self.STEER_ANGLE_MAX)
        self.safety.set_desired_angle_last(round(max_angle * self.DEG_TO_CAN))

        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle, True)))

        # 1 unit above limit
        max_angle_raw = round_angle(get_max_angle_vm(speed, self.VM, CarControllerParams), angle_unit_offset + 2) * sign
        max_angle = np.clip(max_angle_raw, -self.STEER_ANGLE_MAX, self.STEER_ANGLE_MAX)
        self._tx(self._angle_cmd_msg(max_angle, True))

        # at low speeds max angle is above 360, so adding 1 has no effect
        should_tx = abs(max_angle_raw) >= self.STEER_ANGLE_MAX
        self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(max_angle, True)))

  def test_lateral_jerk_limit(self):
    for speed in np.linspace(0, 40, 100):
      speed = max(speed, 1)
      # match DI_vehicleSpeed rounding on CAN
      speed = round_speed(away_round(speed / 0.08 * 3.6) * 0.08 / 3.6)
      for sign in (-1, 1):  # (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)  # safety fudges the speed
        self._tx(self._angle_cmd_msg(0, True))

        # angle signal can't represent 0, so it biases one unit down
        angle_unit_offset = 1 if sign == -1 else 0

        # Stay within limits
        # Up
        max_angle_delta = round_angle(get_max_angle_delta_vm(speed, self.VM, CarControllerParams), angle_unit_offset) * sign
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Don't change
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Down
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))

        # Inject too high rates
        # Up
        max_angle_delta = round_angle(get_max_angle_delta_vm(speed, self.VM, CarControllerParams), angle_unit_offset + 1) * sign
        self.assertFalse(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Don't change
        self.safety.set_desired_angle_last(round(max_angle_delta * self.DEG_TO_CAN))
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Down
        self.assertFalse(self._tx(self._angle_cmd_msg(0, True)))

        # Recover
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))


class TestTeslaStockSafety(TestTeslaSafetyBase):

  LONGITUDINAL = False

  def test_cancel(self):
    for acc_state in range(16):
      self.safety.set_controls_allowed(True)
      should_tx = acc_state == self.acc_states["ACC_CANCEL_GENERIC_SILENT"]
      self.assertFalse(self._tx(self._long_control_msg(0, acc_state=acc_state, accel_limits=(self.MIN_ACCEL, self.MAX_ACCEL))))
      self.assertEqual(should_tx, self._tx(self._long_control_msg(0, acc_state=acc_state)))

  def test_no_aeb(self):
    for aeb_event in range(4):
      should_tx = aeb_event == 0
      ret = self._tx(self._long_control_msg(10, acc_state=self.acc_states["ACC_CANCEL_GENERIC_SILENT"], aeb_event=aeb_event))
      self.assertEqual(ret, should_tx)

  def test_stock_aeb_no_cancel(self):
    # No passthrough logic since we always forward DAS_control,
    # but ensure we can't send cancel cmd while stock AEB is active
    no_aeb_msg = self._long_control_msg(10, acc_state=self.acc_states["ACC_CANCEL_GENERIC_SILENT"], aeb_event=0)
    no_aeb_msg_cam = self._long_control_msg(10, aeb_event=0, bus=2)
    aeb_msg_cam = self._long_control_msg(10, aeb_event=1, bus=2)

    # stock system sends no AEB -> no forwarding, and OP is allowed to TX
    self.assertEqual(1, self._rx(no_aeb_msg_cam))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, no_aeb_msg_cam.addr))
    self.assertTrue(self._tx(no_aeb_msg))

    # stock system sends AEB -> forwarding, and OP is not allowed to TX
    self.assertEqual(1, self._rx(aeb_msg_cam))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, aeb_msg_cam.addr))
    self.assertFalse(self._tx(no_aeb_msg))


class TestTeslaFSD14StockSafety(TestTeslaStockSafety):
  SAFETY_PARAM = TeslaSafetyFlags.FSD_14


class TestTeslaLongitudinalSafety(TestTeslaSafetyBase):
  SAFETY_PARAM = TeslaSafetyFlags.LONG_CONTROL

  RELAY_MALFUNCTION_ADDRS = {0: (MSG_DAS_steeringControl, MSG_APS_eacMonitor, MSG_DAS_Control)}
  FWD_BLACKLISTED_ADDRS = {2: [MSG_DAS_steeringControl, MSG_APS_eacMonitor, MSG_DAS_Control]}

  def test_no_aeb(self):
    for aeb_event in range(4):
      self.assertEqual(self._tx(self._long_control_msg(10, aeb_event=aeb_event)), aeb_event == 0)

  def test_stock_aeb_passthrough(self):
    no_aeb_msg = self._long_control_msg(10, aeb_event=0)
    no_aeb_msg_cam = self._long_control_msg(10, aeb_event=0, bus=2)
    aeb_msg_cam = self._long_control_msg(10, aeb_event=1, bus=2)

    # stock system sends no AEB -> no forwarding, and OP is allowed to TX
    self.assertEqual(1, self._rx(no_aeb_msg_cam))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, no_aeb_msg_cam.addr))
    self.assertTrue(self._tx(no_aeb_msg))

    # stock system sends AEB -> forwarding, and OP is not allowed to TX
    self.assertEqual(1, self._rx(aeb_msg_cam))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, aeb_msg_cam.addr))
    self.assertFalse(self._tx(no_aeb_msg))

  def test_prevent_reverse(self):
    # Note: Tesla can reverse while at a standstill if both accel_min and accel_max are negative.
    self.safety.set_controls_allowed(True)

    # accel_min and accel_max are positive
    self.assertTrue(self._tx(self._long_control_msg(set_speed=10, accel_limits=(1.1, 0.8))))
    self.assertTrue(self._tx(self._long_control_msg(set_speed=0, accel_limits=(1.1, 0.8))))

    # accel_min and accel_max are both zero
    self.assertTrue(self._tx(self._long_control_msg(set_speed=10, accel_limits=(0, 0))))
    self.assertTrue(self._tx(self._long_control_msg(set_speed=0, accel_limits=(0, 0))))

    # accel_min and accel_max have opposing signs
    self.assertTrue(self._tx(self._long_control_msg(set_speed=10, accel_limits=(-0.8, 1.3))))
    self.assertTrue(self._tx(self._long_control_msg(set_speed=0, accel_limits=(0.8, -1.3))))
    self.assertTrue(self._tx(self._long_control_msg(set_speed=0, accel_limits=(0, -1.3))))

    # accel_min and accel_max are negative
    self.assertFalse(self._tx(self._long_control_msg(set_speed=10, accel_limits=(-1.1, -0.6))))
    self.assertFalse(self._tx(self._long_control_msg(set_speed=0, accel_limits=(-0.6, -1.1))))
    self.assertFalse(self._tx(self._long_control_msg(set_speed=0, accel_limits=(-0.1, -0.1))))


class TestTeslaFSD14LongitudinalSafety(TestTeslaLongitudinalSafety):
  SAFETY_PARAM = TeslaSafetyFlags.LONG_CONTROL | TeslaSafetyFlags.FSD_14


class TestTeslaSpeedProfileSafety(unittest.TestCase):
  """Security boundary for the opt-in, stock-frame-shadowed speed profile."""

  TX_MSGS = [[MSG_UI_autopilotControl, 2]]
  ENGAGED_MIN_US = 1_000_000
  BASELINE_MAX_AGE_US = 100_000

  HW3 = {
    "raw_hw": 2,
    "fsd14": False,
    "mux": 0,
    "target_byte": 6,
    "target_mask": 0x06,
    "shift": 1,
    "mapping": {1: 2, 2: 1, 3: 0},
  }
  HW4 = {
    "raw_hw": 3,
    "fsd14": True,
    "mux": 2,
    "target_byte": 7,
    "target_mask": 0xE0,
    "shift": 5,
    "mapping": {1: 3, 2: 2, 3: 1, 4: 0, 5: 4},
  }

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.packer = CANPackerSafety("tesla_model3_party")
    self._set_mode(speed_profile=True, fsd14=False)

  def _set_mode(self, speed_profile: bool, fsd14: bool, longitudinal: bool = False, vehicle_bus: bool = False):
    param_sp = 0
    if speed_profile:
      param_sp |= TeslaSafetyFlagsSP.SPEED_PROFILE
    if vehicle_bus:
      param_sp |= TeslaSafetyFlagsSP.HAS_VEHICLE_BUS

    param = 0
    if fsd14:
      param |= TeslaSafetyFlags.FSD_14
    if longitudinal:
      param |= TeslaSafetyFlags.LONG_CONTROL
    self.safety.set_current_safety_param_sp(param_sp)
    self.safety.set_safety_hooks(CarParams.SafetyModel.tesla, param)
    self.safety.init_tests()

  def _rx(self, msg):
    return self.safety.safety_rx_hook(msg)

  def _tx(self, msg):
    return self.safety.safety_tx_hook(msg)

  @staticmethod
  def _raw_msg(addr: int, bus: int, dat: bytes | bytearray):
    return common.make_msg(bus, addr, len(dat), bytes(dat))

  def _hw_msg(self, raw_hw: int, bus: int = 0, length: int = 8, all_zero: bool = False):
    dat = bytearray(8)
    if not all_zero:
      dat[0] = ((raw_hw & 0x3) << 6) | 0x15
      dat[1:] = b"\x21\x32\x43\x54\x65\x76\x87"
    return self._raw_msg(MSG_GTW_carConfig, bus, dat[:length])

  def _follow_distance_msg(self, follow_distance: int, bus: int = 0, length: int = 8):
    dat = bytearray(b"\x11\x22\x33\x44\x55\x1b\x77\x88")
    dat[5] = ((follow_distance & 0x7) << 5) | 0x1b
    return self._raw_msg(MSG_UI_driverAssistControl, bus, dat[:length])

  def _baseline_data(self, protocol, fsd_selected: bool = True, mux: int | None = None):
    mux = protocol["mux"] if mux is None else mux
    dat = bytearray(b"\xa8\x12\x34\x56\x18\x9a\xd1\x17")
    dat[0] = (dat[0] & 0xf8) | (mux & 0x7)
    if fsd_selected:
      dat[4] |= 0x40
    else:
      dat[4] &= ~0x40
    # Use an out-of-range stock value so every mapped target is a real mutation.
    byte = protocol["target_byte"]
    mask = protocol["target_mask"]
    dat[byte] = (dat[byte] & ~mask) | mask
    return dat

  def _baseline_msg(self, dat: bytes | bytearray, bus: int = 0, length: int = 8):
    return self._raw_msg(MSG_UI_autopilotControl, bus, dat[:length])

  def _candidate_msg(self, baseline: bytes | bytearray, protocol, profile: int, bus: int = 2, length: int = 8):
    dat = bytearray(baseline)
    byte = protocol["target_byte"]
    mask = protocol["target_mask"]
    dat[byte] = (dat[byte] & ~mask) | ((profile << protocol["shift"]) & mask)
    return self._raw_msg(MSG_UI_autopilotControl, bus, dat[:length])

  def _pcm_status_msg(self, enabled: bool):
    values = {"DI_cruiseState": 2 if enabled else 0, "DI_autoparkState": 0}
    return self.packer.make_can_msg_safety("DI_state", 0, values)

  def _prime_mandatory_rx_checks(self):
    msgs = (
      self.packer.make_can_msg_safety("DAS_control", 2, {}),
      self.packer.make_can_msg_safety("DAS_steeringControl", 2, {"DAS_steeringControlType": 0}),
      self.packer.make_can_msg_safety("DI_speed", 0, {"DI_vehicleSpeed": 0}),
      self.packer.make_can_msg_safety("ESP_B", 0, {"ESP_vehicleSpeed": 0, "ESP_wheelSpeedsQF": 1,
                                                    "ESP_vehicleStandstillSts": 1}),
      self.packer.make_can_msg_safety("EPAS3S_sysStatus", 0, {"EPAS3S_internalSAS": 0}),
      self.packer.make_can_msg_safety("DI_systemStatus", 0, {"DI_accelPedalPos": 0}),
      self.packer.make_can_msg_safety("ESP_status", 0, {"ESP_driverBrakeApply": 1}),
      self._pcm_status_msg(False),
      self.packer.make_can_msg_safety("UI_warning", 0, {}),
    )
    for msg in msgs:
      self.assertTrue(self._rx(msg))

  def _configure_protocol(self, protocol, follow_distance: int = 1):
    self._set_mode(speed_profile=True, fsd14=protocol["fsd14"])
    self.assertTrue(self._rx(self._hw_msg(protocol["raw_hw"])))
    self.assertTrue(self._rx(self._follow_distance_msg(follow_distance)))

  def _engage(self, elapsed_us: int = ENGAGED_MIN_US):
    self.safety.set_timer(0)
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self.safety.set_timer(elapsed_us)

  def _ready(self, protocol, follow_distance: int = 1, fsd_selected: bool = True):
    self._configure_protocol(protocol, follow_distance)
    self._engage()
    baseline = self._baseline_data(protocol, fsd_selected=fsd_selected)
    self.assertTrue(self._rx(self._baseline_msg(baseline)))
    return baseline, protocol["mapping"][follow_distance]

  def test_feature_is_off_by_default(self):
    self._set_mode(speed_profile=False, fsd14=False)
    baseline = self._baseline_data(self.HW3)
    candidate = self._candidate_msg(baseline, self.HW3, 2)
    self.assertFalse(self._tx(candidate))
    self.assertEqual(2, self.safety.safety_fwd_hook(0, MSG_UI_autopilotControl))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, MSG_UI_autopilotControl))

  def test_optional_sources_do_not_participate_in_rx_liveness(self):
    self._prime_mandatory_rx_checks()
    self.safety.set_timer(1_000_000)
    self.safety.set_controls_allowed(True)
    self.safety.safety_tick_current_safety_config()
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.safety_config_valid())

    # Mark every optional source seen, then let only those sources age past their old 10-second threshold.
    self._rx(self._hw_msg(self.HW3["raw_hw"]))
    self._rx(self._follow_distance_msg(1))
    self._rx(self._baseline_msg(self._baseline_data(self.HW3)))
    self.safety.set_timer(12_000_001)
    self._prime_mandatory_rx_checks()
    self.safety.set_controls_allowed(True)
    self.safety.safety_tick_current_safety_config()
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.safety_config_valid())

  def test_flag_composes_with_longitudinal_and_vehicle_bus(self):
    self._set_mode(speed_profile=True, fsd14=False, longitudinal=True, vehicle_bus=True)
    self._rx(self._hw_msg(self.HW3["raw_hw"]))
    self._rx(self._follow_distance_msg(1))
    self._engage()
    baseline = self._baseline_data(self.HW3)
    self._rx(self._baseline_msg(baseline))
    self.assertTrue(self._tx(self._candidate_msg(baseline, self.HW3, self.HW3["mapping"][1])))

  def test_exact_mapping_for_both_protocols(self):
    for protocol in (self.HW3, self.HW4):
      for follow_distance, profile in protocol["mapping"].items():
        with self.subTest(raw_hw=protocol["raw_hw"], follow_distance=follow_distance, profile=profile):
          baseline, mapped_profile = self._ready(protocol, follow_distance)
          self.assertEqual(profile, mapped_profile)

          wrong_profile = (profile + 1) % (3 if protocol is self.HW3 else 5)
          self.assertFalse(self._tx(self._candidate_msg(baseline, protocol, wrong_profile)))
          self.assertTrue(self._tx(self._candidate_msg(baseline, protocol, profile)))

  def test_actual_host_controller_output_is_accepted_once(self):
    host_protocols = {
      self.HW3["raw_hw"]: SpeedProfileProtocol.hw3,
      self.HW4["raw_hw"]: SpeedProfileProtocol.hw4_fsd14,
    }
    for protocol in (self.HW3, self.HW4):
      with self.subTest(protocol=protocol["raw_hw"]):
        baseline, profile = self._ready(protocol)
        controller = TeslaSpeedProfileCarController(SimpleNamespace(flags=TeslaFlagsSP.SPEED_PROFILE))
        state = SimpleNamespace(
          protocol=host_protocols[protocol["raw_hw"]],
          profile=profile,
          autopilot_control_frames=[bytes(baseline)],
        )
        cs = SimpleNamespace(speed_profile=state)
        enabled = SimpleNamespace(enabled=True)

        for _ in range(AP_FIRST_STABLE_FRAMES):
          self.assertEqual([], controller.update_speed_profile(enabled, cs))
        sends = controller.update_speed_profile(enabled, cs)
        self.assertEqual(1, len(sends))

        addr, dat, bus = sends[0]
        candidate = self._raw_msg(addr, bus, dat)
        self.assertTrue(self._tx(candidate))
        self.assertFalse(self._tx(candidate))

  def test_unknown_hardware_and_no_default_profile(self):
    for protocol in (self.HW3, self.HW4):
      with self.subTest(protocol=protocol["raw_hw"], case="no follow distance"):
        self._set_mode(speed_profile=True, fsd14=protocol["fsd14"])
        self._rx(self._hw_msg(protocol["raw_hw"]))
        self._engage()
        baseline = self._baseline_data(protocol)
        self._rx(self._baseline_msg(baseline))
        self.assertFalse(self._tx(self._candidate_msg(baseline, protocol, next(iter(protocol["mapping"].values())))))

      with self.subTest(protocol=protocol["raw_hw"], case="all-zero clears hardware"):
        follow_distance, profile = next(iter(protocol["mapping"].items()))
        self._configure_protocol(protocol, follow_distance)
        self._rx(self._hw_msg(0, all_zero=True))
        # An invalid source value leaves no default while hardware is unknown.
        self._rx(self._follow_distance_msg(0))
        self._engage()
        baseline = self._baseline_data(protocol)
        self._rx(self._baseline_msg(baseline))
        self.assertFalse(self._tx(self._candidate_msg(baseline, protocol, profile)))

        # Hardware recovery alone is insufficient: a new valid follow-distance source is required.
        self._rx(self._hw_msg(protocol["raw_hw"]))
        self._rx(self._baseline_msg(baseline))
        self.assertFalse(self._tx(self._candidate_msg(baseline, protocol, profile)))
        self._rx(self._follow_distance_msg(follow_distance))
        self._rx(self._baseline_msg(baseline))
        self.assertTrue(self._tx(self._candidate_msg(baseline, protocol, profile)))

  def test_invalid_follow_distance_does_not_replace_valid_value(self):
    for protocol in (self.HW3, self.HW4):
      follow_distance, profile = next(iter(protocol["mapping"].items()))
      for invalid_follow_distance in ({0, 4, 5, 6, 7} - set(protocol["mapping"])):
        with self.subTest(protocol=protocol["raw_hw"], follow_distance=invalid_follow_distance):
          self._configure_protocol(protocol, follow_distance)
          self._rx(self._follow_distance_msg(invalid_follow_distance))
          self._engage()
          baseline = self._baseline_data(protocol)
          self._rx(self._baseline_msg(baseline))
          self.assertTrue(self._tx(self._candidate_msg(baseline, protocol, profile)))

  def test_follow_distance_updates_only_invalidate_baseline_on_profile_change(self):
    for protocol in (self.HW3, self.HW4):
      with self.subTest(protocol=protocol["raw_hw"]):
        initial_profile = protocol["mapping"][1]
        changed_profile = protocol["mapping"][2]
        self._configure_protocol(protocol, follow_distance=1)
        self._engage()
        baseline = self._baseline_data(protocol)
        self._rx(self._baseline_msg(baseline))

        # Repeats and invalid source values preserve both the last valid profile and fresh baseline.
        self._rx(self._follow_distance_msg(1))
        for invalid_follow_distance in (0, 6, 7):
          self._rx(self._follow_distance_msg(invalid_follow_distance))
        self.assertTrue(self._tx(self._candidate_msg(baseline, protocol, initial_profile)))

        # A genuinely different mapped profile cannot shadow a baseline captured for the old value.
        self._rx(self._baseline_msg(baseline))
        self._rx(self._follow_distance_msg(2))
        self.assertFalse(self._tx(self._candidate_msg(baseline, protocol, changed_profile)))
        self._rx(self._baseline_msg(baseline))
        self.assertTrue(self._tx(self._candidate_msg(baseline, protocol, changed_profile)))

  def test_protocol_must_match_fsd14(self):
    mismatches = ((False, 0), (False, 1), (False, 3), (True, 0), (True, 1), (True, 2))
    for fsd14, raw_hw in mismatches:
      with self.subTest(fsd14=fsd14, raw_hw=raw_hw):
        protocol = self.HW4 if fsd14 else self.HW3
        self._set_mode(speed_profile=True, fsd14=fsd14)
        self._rx(self._hw_msg(raw_hw))
        self._rx(self._follow_distance_msg(1))
        self._engage()
        baseline = self._baseline_data(protocol)
        self._rx(self._baseline_msg(baseline))
        self.assertFalse(self._tx(self._candidate_msg(baseline, protocol, protocol["mapping"][1])))

  def test_source_and_tx_bus_and_dlc_are_exact(self):
    for protocol in (self.HW3, self.HW4):
      profile = protocol["mapping"][1]

      for bad_bus, bad_length in ((1, 8), (0, 7)):
        with self.subTest(protocol=protocol["raw_hw"], source="hardware", bus=bad_bus, length=bad_length):
          self._set_mode(speed_profile=True, fsd14=protocol["fsd14"])
          self._rx(self._hw_msg(protocol["raw_hw"], bus=bad_bus, length=bad_length))
          self._rx(self._follow_distance_msg(1))
          self._engage()
          baseline = self._baseline_data(protocol)
          self._rx(self._baseline_msg(baseline))
          self.assertFalse(self._tx(self._candidate_msg(baseline, protocol, profile)))

      for bad_bus, bad_length in ((1, 8), (0, 7)):
        with self.subTest(protocol=protocol["raw_hw"], source="follow distance", bus=bad_bus, length=bad_length):
          self._set_mode(speed_profile=True, fsd14=protocol["fsd14"])
          self._rx(self._hw_msg(protocol["raw_hw"]))
          self._rx(self._follow_distance_msg(1, bus=bad_bus, length=bad_length))
          self._engage()
          baseline = self._baseline_data(protocol)
          self._rx(self._baseline_msg(baseline))
          self.assertFalse(self._tx(self._candidate_msg(baseline, protocol, profile)))

      for bad_bus, bad_length in ((1, 8), (0, 7)):
        with self.subTest(protocol=protocol["raw_hw"], source="baseline", bus=bad_bus, length=bad_length):
          self._configure_protocol(protocol)
          self._engage()
          baseline = self._baseline_data(protocol)
          self._rx(self._baseline_msg(baseline, bus=bad_bus, length=bad_length))
          self.assertFalse(self._tx(self._candidate_msg(baseline, protocol, profile)))

      with self.subTest(protocol=protocol["raw_hw"], source="tx"):
        baseline, profile = self._ready(protocol)
        self.assertFalse(self._tx(self._candidate_msg(baseline, protocol, profile, bus=0)))
        self.assertFalse(self._tx(self._candidate_msg(baseline, protocol, profile, length=7)))
        # Wrongly routed/DLC'd attempts never consume the valid baseline.
        self.assertTrue(self._tx(self._candidate_msg(baseline, protocol, profile)))

  def test_requires_fresh_once_only_baseline_for_target_mux(self):
    for protocol in (self.HW3, self.HW4):
      with self.subTest(protocol=protocol["raw_hw"], case="no baseline"):
        self._configure_protocol(protocol)
        self._engage()
        baseline = self._baseline_data(protocol)
        self.assertFalse(self._tx(self._candidate_msg(baseline, protocol, protocol["mapping"][1])))

      with self.subTest(protocol=protocol["raw_hw"], case="wrong mux"):
        self._configure_protocol(protocol)
        self._engage()
        baseline = self._baseline_data(protocol)
        wrong_mux_baseline = self._baseline_data(protocol, mux=(protocol["mux"] + 1) % 8)
        self._rx(self._baseline_msg(wrong_mux_baseline))
        self.assertFalse(self._tx(self._candidate_msg(baseline, protocol, protocol["mapping"][1])))

      with self.subTest(protocol=protocol["raw_hw"], case="stale"):
        baseline, profile = self._ready(protocol)
        self.safety.set_timer(self.ENGAGED_MIN_US + self.BASELINE_MAX_AGE_US + 1)
        self.assertFalse(self._tx(self._candidate_msg(baseline, protocol, profile)))

      with self.subTest(protocol=protocol["raw_hw"], case="fresh boundary and replay"):
        baseline, profile = self._ready(protocol)
        self.safety.set_timer(self.ENGAGED_MIN_US + self.BASELINE_MAX_AGE_US)
        candidate = self._candidate_msg(baseline, protocol, profile)
        self.assertTrue(self._tx(candidate))
        self.assertFalse(self._tx(candidate))

  def test_unchanged_baseline_is_not_an_injection_and_does_not_consume_it(self):
    for protocol in (self.HW3, self.HW4):
      with self.subTest(protocol=protocol["raw_hw"]):
        baseline, profile = self._ready(protocol)
        self.assertFalse(self._tx(self._baseline_msg(baseline, bus=2)))
        self.assertTrue(self._tx(self._candidate_msg(baseline, protocol, profile)))

  def test_requires_stable_live_engagement(self):
    for protocol in (self.HW3, self.HW4):
      profile = protocol["mapping"][1]
      with self.subTest(protocol=protocol["raw_hw"], case="not engaged"):
        self._configure_protocol(protocol)
        baseline = self._baseline_data(protocol)
        self._rx(self._baseline_msg(baseline))
        self.assertFalse(self._tx(self._candidate_msg(baseline, protocol, profile)))

      with self.subTest(protocol=protocol["raw_hw"], case="unstable"):
        self._configure_protocol(protocol)
        self._engage(self.ENGAGED_MIN_US - 1)
        baseline = self._baseline_data(protocol)
        self._rx(self._baseline_msg(baseline))
        self.assertFalse(self._tx(self._candidate_msg(baseline, protocol, profile)))
        self.safety.set_timer(self.ENGAGED_MIN_US)
        self.assertTrue(self._tx(self._candidate_msg(baseline, protocol, profile)))

      with self.subTest(protocol=protocol["raw_hw"], case="disengaged"):
        baseline, profile = self._ready(protocol)
        self._rx(self._pcm_status_msg(False))
        self.assertFalse(self._tx(self._candidate_msg(baseline, protocol, profile)))

  def test_stock_aeb_rejects_without_consuming_baseline(self):
    for protocol in (self.HW3, self.HW4):
      with self.subTest(protocol=protocol["raw_hw"]):
        baseline, profile = self._ready(protocol)
        candidate = self._candidate_msg(baseline, protocol, profile)

        self.assertTrue(self._rx(self.packer.make_can_msg_safety("DAS_control", 2, {"DAS_aebEvent": 1})))
        self.assertFalse(self._tx(candidate))

        self.assertTrue(self._rx(self.packer.make_can_msg_safety("DAS_control", 2, {"DAS_aebEvent": 0})))
        self.assertTrue(self._tx(candidate))

  def test_hw3_requires_live_fsd_selection(self):
    baseline, profile = self._ready(self.HW3, fsd_selected=False)
    self.assertFalse(self._tx(self._candidate_msg(baseline, self.HW3, profile)))

    baseline = self._baseline_data(self.HW3, fsd_selected=True)
    self._rx(self._baseline_msg(baseline))
    self.assertTrue(self._tx(self._candidate_msg(baseline, self.HW3, profile)))

  def test_every_non_target_bit_is_immutable_and_forwarding_remains(self):
    for protocol in (self.HW3, self.HW4):
      baseline, profile = self._ready(protocol)
      target_bits = {protocol["target_byte"] * 8 + bit for bit in range(8) if protocol["target_mask"] & (1 << bit)}

      for bit in range(64):
        if bit in target_bits:
          continue
        with self.subTest(protocol=protocol["raw_hw"], bit=bit):
          self._rx(self._baseline_msg(baseline))
          candidate = self._candidate_msg(baseline, protocol, profile)
          candidate[0].data[bit // 8] ^= 1 << (bit % 8)
          self.assertFalse(self._tx(candidate))

      self._rx(self._baseline_msg(baseline))
      self.assertTrue(self._tx(self._candidate_msg(baseline, protocol, profile)))
      self.assertEqual(2, self.safety.safety_fwd_hook(0, MSG_UI_autopilotControl))
      self.assertEqual(0, self.safety.safety_fwd_hook(2, MSG_UI_autopilotControl))


class TestTeslaIgnition(unittest.TestCase):
  TX_MSGS: list = []

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()
    self.packer = CANPackerSafety("tesla_model3_party")

  def _msg(self, counter, state):
    return self.packer.make_can_msg_safety("VCFRONT_LVPowerState", 0,
                                           {"VCFRONT_LVPowerStateCounter": counter,
                                            "VCFRONT_vehiclePowerState": state})

  # VEHICLE_POWER_STATE_DRIVE=3 (counter-gated)
  def test_ignition_on(self):
    for i in range(16):
      self.safety.init_tests()
      self.safety.ignition_can_hook(self._msg(i, 3))
      self.assertFalse(self.safety.get_ignition_can())
      self.safety.ignition_can_hook(self._msg((i + 1) % 16, 3))
      self.assertTrue(self.safety.get_ignition_can())

  def test_ignition_off(self):
    self.safety.ignition_can_hook(self._msg(0, 3))
    self.safety.ignition_can_hook(self._msg(1, 3))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._msg(2, 2))
    self.safety.ignition_can_hook(self._msg(3, 2))
    self.assertFalse(self.safety.get_ignition_can())


class TestTeslaVehicleBusSafety(TestTeslaSafetyBase):

  LONGITUDINAL = False

  def setUp(self):
    super().setUp()
    self.safety = libsafety_py.libsafety
    self.packer_adas = CANPackerSafety("tesla_model3_vehicle")
    self.safety.set_current_safety_param_sp(TeslaSafetyFlagsSP.HAS_VEHICLE_BUS)
    self.safety.set_safety_hooks(CarParams.SafetyModel.tesla, 0)
    self.safety.init_tests()

  def _lkas_button_msg(self, enabled):
    values = {"UI_activeTouchPoints": 3 if enabled else 0}
    return self.packer_adas.make_can_msg_safety("UI_status2", CANBUS.vehicle, values)


if __name__ == "__main__":
  unittest.main()
