from math import radians

import pytest

from openpilot.cereal import log
from opendbc.car.structs import car
from openpilot.common.params import Params
from openpilot.common.realtime import DT_DMON
from openpilot.selfdrive.monitoring.policy import DriverMonitoring, DRIVER_MONITOR_SETTINGS

EventName = log.OnroadEvent.EventName
dm_settings = DRIVER_MONITOR_SETTINGS()

TEST_TIMESPAN = 120  # seconds
DISTRACTED_SECONDS_TO_ORANGE = dm_settings._VISION_POLICY_ALERT_2_TIMEOUT + 1
DISTRACTED_SECONDS_TO_RED = dm_settings._VISION_POLICY_ALERT_3_TIMEOUT + 1
INVISIBLE_SECONDS_TO_ORANGE = dm_settings._WHEELTOUCH_POLICY_ALERT_2_TIMEOUT + 1
INVISIBLE_SECONDS_TO_RED = dm_settings._WHEELTOUCH_POLICY_ALERT_3_TIMEOUT + 1

def make_msg(face_detected, distracted=False, model_uncertain=False):
  ds = log.DriverStateV2.new_message()
  ds.leftDriverData.faceOrientation = [0., 0., 0.]
  ds.leftDriverData.facePosition = [0., 0.]
  ds.leftDriverData.faceProb = 1. * face_detected
  ds.leftDriverData.leftEyeProb = 1.
  ds.leftDriverData.rightEyeProb = 1.
  ds.leftDriverData.leftBlinkProb = 1. * distracted
  ds.leftDriverData.rightBlinkProb = 1. * distracted
  ds.leftDriverData.faceOrientationStd = [1.*model_uncertain, 1.*model_uncertain, 1.*model_uncertain]
  ds.leftDriverData.facePositionStd = [1.*model_uncertain, 1.*model_uncertain]
  # TODO: test both separately when e2e is used
  ds.leftDriverData.phoneProb = 0.
  return ds


# driver state from neural net, 10Hz
msg_NO_FACE_DETECTED = make_msg(False)
msg_ATTENTIVE = make_msg(True)
msg_DISTRACTED = make_msg(True, distracted=True)
msg_ATTENTIVE_UNCERTAIN = make_msg(True, model_uncertain=True)
msg_DISTRACTED_UNCERTAIN = make_msg(True, distracted=True, model_uncertain=True)
msg_DISTRACTED_BUT_SOMEHOW_UNCERTAIN = make_msg(True, distracted=True, model_uncertain=dm_settings._HI_STD_THRESHOLD*1.5)


DRIVER_MONITORING_YAW_MODE_PARAM = "DriverMonitoringYawMode"
LEFT_MODEL_YAW = 0.2
RIGHT_MODEL_YAW = 0.6
LEFT_PHONE_PROB = 0.11
RIGHT_PHONE_PROB = 0.22
TEST_STEERING_ANGLE_DEG = 40.
LHD_STEER_YAW_OFFSET = -radians(TEST_STEERING_ANGLE_DEG - dm_settings._POSE_YAW_MIN_STEER_DEG) * dm_settings._POSE_YAW_STEER_FACTOR
RHD_STEER_YAW_OFFSET = -LHD_STEER_YAW_OFFSET

def make_yaw_mode_msg():
  ds = log.DriverStateV2.new_message()
  for driver_data, yaw, phone_prob in ((ds.leftDriverData, LEFT_MODEL_YAW, LEFT_PHONE_PROB),
                                       (ds.rightDriverData, RIGHT_MODEL_YAW, RIGHT_PHONE_PROB)):
    driver_data.faceOrientation = [0., yaw, 0.]
    driver_data.facePosition = [0., 0.]
    driver_data.faceOrientationStd = [0., 0., 0.]
    driver_data.facePositionStd = [0., 0.]
    driver_data.faceProb = 1.
    driver_data.leftEyeProb = 1.
    driver_data.rightEyeProb = 1.
    driver_data.phoneProb = phone_prob
  return ds


@pytest.fixture
def driver_monitoring_yaw_mode():
  params = Params()
  previous_value = params.get(DRIVER_MONITORING_YAW_MODE_PARAM)
  params.remove(DRIVER_MONITORING_YAW_MODE_PARAM)
  try:
    yield params
  finally:
    if previous_value is None:
      params.remove(DRIVER_MONITORING_YAW_MODE_PARAM)
    else:
      params.put(DRIVER_MONITORING_YAW_MODE_PARAM, previous_value, block=True)

# driver interaction with car
car_interaction_DETECTED = True
car_interaction_NOT_DETECTED = False

# some common state vectors
always_no_face = [msg_NO_FACE_DETECTED] * int(TEST_TIMESPAN / DT_DMON)
always_attentive = [msg_ATTENTIVE] * int(TEST_TIMESPAN / DT_DMON)
always_distracted = [msg_DISTRACTED] * int(TEST_TIMESPAN / DT_DMON)
always_true = [True] * int(TEST_TIMESPAN / DT_DMON)
always_false = [False] * int(TEST_TIMESPAN / DT_DMON)

class TestMonitoring:
  def _run_seq(self, msgs, interaction, engaged, lowspeed):
    DM = DriverMonitoring()
    alert_lvls = []
    for idx in range(len(msgs)):
      DM._update_states(msgs[idx], [0, 0, 0], 0, engaged[idx], lowspeed[idx])
      # cal_rpy and car_speed don't matter here

      # evaluate events at 10Hz for tests
      DM._update_events(interaction[idx], engaged[idx], lowspeed[idx], 0)
      alert_lvls.append(DM.alert_level)
    assert len(alert_lvls) == len(msgs), f"got {len(alert_lvls)} for {len(msgs)} driverState input msgs"
    return alert_lvls, DM


  # engaged, driver is attentive all the time
  def test_fully_aware_driver(self):
    alert_lvls, d_status = self._run_seq(always_attentive, always_false, always_true, always_false)
    assert all(a == 0 for a in alert_lvls)
    assert d_status.active_policy == log.DriverMonitoringState.MonitoringPolicy.vision

  # engaged, driver is distracted and does nothing
  def test_fully_distracted_driver(self):
    alert_lvls, d_status = self._run_seq(always_distracted, always_false, always_true, always_false)
    s = d_status.settings
    assert alert_lvls[int(s._VISION_POLICY_ALERT_1_TIMEOUT / 2 / DT_DMON)] == 0
    assert alert_lvls[int((s._VISION_POLICY_ALERT_1_TIMEOUT + \
                    (s._VISION_POLICY_ALERT_2_TIMEOUT - s._VISION_POLICY_ALERT_1_TIMEOUT) / 2) / DT_DMON)] == 1
    assert alert_lvls[int((s._VISION_POLICY_ALERT_2_TIMEOUT + \
                    (s._VISION_POLICY_ALERT_3_TIMEOUT - s._VISION_POLICY_ALERT_2_TIMEOUT) / 2) / DT_DMON)] == 2
    assert alert_lvls[int((s._VISION_POLICY_ALERT_3_TIMEOUT + \
                    (TEST_TIMESPAN - 10 - s._VISION_POLICY_ALERT_3_TIMEOUT) / 2) / DT_DMON)] == 3
    assert isinstance(d_status.awareness, float)

  # engaged, distracted past red and beyond the no-response window -> unavailability response + lockout
  def test_distracted_lockout(self):
    alert_lvls, d_status = self._run_seq(always_distracted, always_false, always_true, always_false)
    assert alert_lvls[int(DISTRACTED_SECONDS_TO_RED / DT_DMON)] == 3
    assert d_status.lockout_active
    assert d_status.lockout_time_elapsed > 0
    assert d_status.lockout_count >= 1

  # no face -> wheeltouch red, sustained past the no-response timeout -> unavailability response + lockout
  def test_invisible_lockout(self):
    _, d_status = self._run_seq(always_no_face, always_false, always_true, always_false)
    assert d_status.active_policy == log.DriverMonitoringState.MonitoringPolicy.wheeltouch
    assert d_status.lockout_active
    assert d_status.lockout_count >= 1

  # engaged, no face detected the whole time, no action
  def test_fully_invisible_driver(self):
    alert_lvls, d_status = self._run_seq(always_no_face, always_false, always_true, always_false)
    s = d_status.settings
    assert alert_lvls[int(s._WHEELTOUCH_POLICY_ALERT_1_TIMEOUT / 2 / DT_DMON)] == 0
    assert alert_lvls[int((s._WHEELTOUCH_POLICY_ALERT_1_TIMEOUT + \
                    (s._WHEELTOUCH_POLICY_ALERT_2_TIMEOUT - s._WHEELTOUCH_POLICY_ALERT_1_TIMEOUT) / 2) / DT_DMON)] == 1
    assert alert_lvls[int((s._WHEELTOUCH_POLICY_ALERT_2_TIMEOUT + \
                    (s._WHEELTOUCH_POLICY_ALERT_3_TIMEOUT - s._WHEELTOUCH_POLICY_ALERT_2_TIMEOUT) / 2) / DT_DMON)] == 2
    assert alert_lvls[int((s._WHEELTOUCH_POLICY_ALERT_3_TIMEOUT + \
                    (TEST_TIMESPAN - 10 - s._WHEELTOUCH_POLICY_ALERT_3_TIMEOUT) / 2) / DT_DMON)] == 3
    assert d_status.active_policy == log.DriverMonitoringState.MonitoringPolicy.wheeltouch

  # engaged, down to orange, driver pays attention, back to normal; then down to orange, driver touches wheel
  #  - should have short orange recovery time and no green afterwards; wheel touch only recovers when paying attention
  def test_normal_driver(self):
    ds_vector = [msg_DISTRACTED] * int(DISTRACTED_SECONDS_TO_ORANGE/DT_DMON) + \
                [msg_ATTENTIVE] * int(DISTRACTED_SECONDS_TO_ORANGE/DT_DMON) + \
                [msg_DISTRACTED] * int((DISTRACTED_SECONDS_TO_ORANGE+2)/DT_DMON) + \
                [msg_ATTENTIVE] * (int(TEST_TIMESPAN/DT_DMON)-int((DISTRACTED_SECONDS_TO_ORANGE*3+2)/DT_DMON))
    interaction_vector = [car_interaction_NOT_DETECTED] * int(DISTRACTED_SECONDS_TO_ORANGE*3/DT_DMON) + \
                         [car_interaction_DETECTED] * (int(TEST_TIMESPAN/DT_DMON)-int(DISTRACTED_SECONDS_TO_ORANGE*3/DT_DMON))
    alert_lvls, _ = self._run_seq(ds_vector, interaction_vector, always_true, always_false)
    assert alert_lvls[int(DISTRACTED_SECONDS_TO_ORANGE*0.5/DT_DMON)] == 0
    assert alert_lvls[int((DISTRACTED_SECONDS_TO_ORANGE-0.1)/DT_DMON)] == 2
    assert alert_lvls[int(DISTRACTED_SECONDS_TO_ORANGE*1.5/DT_DMON)] == 0
    assert alert_lvls[int((DISTRACTED_SECONDS_TO_ORANGE*3-0.1)/DT_DMON)] == 2
    assert alert_lvls[int((DISTRACTED_SECONDS_TO_ORANGE*3+0.1)/DT_DMON)] == 2
    assert alert_lvls[int((DISTRACTED_SECONDS_TO_ORANGE*3+2.5)/DT_DMON)] == 0

  # engaged, down to orange, driver dodges camera, then comes back still distracted, down to red, \
  #                          driver dodges, and then touches wheel to no avail, disengages and reengages
  #  - orange/red alert should remain after disappearance, and only disengaging clears red
  def test_biggest_comma_fan(self):
    _invisible_time = 2  # seconds
    ds_vector = always_distracted[:]
    interaction_vector = always_false[:]
    op_vector = always_true[:]
    ds_vector[int(DISTRACTED_SECONDS_TO_ORANGE/DT_DMON):int((DISTRACTED_SECONDS_TO_ORANGE+_invisible_time)/DT_DMON)] \
                                                        = [msg_NO_FACE_DETECTED] * int(_invisible_time/DT_DMON)
    ds_vector[int((DISTRACTED_SECONDS_TO_RED+_invisible_time)/DT_DMON):int((DISTRACTED_SECONDS_TO_RED+2*_invisible_time)/DT_DMON)] \
                                                        = [msg_NO_FACE_DETECTED] * int(_invisible_time/DT_DMON)
    interaction_vector[int((DISTRACTED_SECONDS_TO_RED+2*_invisible_time+0.5)/DT_DMON):int((DISTRACTED_SECONDS_TO_RED+2*_invisible_time+1.5)/DT_DMON)] \
                                                        = [True] * int(1/DT_DMON)
    op_vector[int((DISTRACTED_SECONDS_TO_RED+2*_invisible_time+2.5)/DT_DMON):int((DISTRACTED_SECONDS_TO_RED+2*_invisible_time+3)/DT_DMON)] \
                                                        = [False] * int(0.5/DT_DMON)
    alert_lvls, _ = self._run_seq(ds_vector, interaction_vector, op_vector, always_false)
    assert alert_lvls[int((DISTRACTED_SECONDS_TO_ORANGE+0.5*_invisible_time)/DT_DMON)] == 2
    assert alert_lvls[int((DISTRACTED_SECONDS_TO_RED+1.5*_invisible_time)/DT_DMON)] == 3
    assert alert_lvls[int((DISTRACTED_SECONDS_TO_RED+2*_invisible_time+1.5)/DT_DMON)] == 3
    assert alert_lvls[int((DISTRACTED_SECONDS_TO_RED+2*_invisible_time+3.5)/DT_DMON)] == 0

  # engaged, invisible driver, down to orange, driver touches wheel; then down to orange again, driver appears
  #  - both actions should clear the alert, but momentary appearance should not
  def test_sometimes_transparent_commuter(self):
    for _visible_time in (0.5, 10):
      ds_vector = always_no_face[:]*2
      interaction_vector = always_false[:]*2
      ds_vector[int((2*INVISIBLE_SECONDS_TO_ORANGE+1)/DT_DMON):int((2*INVISIBLE_SECONDS_TO_ORANGE+1+_visible_time)/DT_DMON)] = \
                                                                                               [msg_ATTENTIVE] * int(_visible_time/DT_DMON)
      interaction_vector[int((INVISIBLE_SECONDS_TO_ORANGE)/DT_DMON):int((INVISIBLE_SECONDS_TO_ORANGE+1)/DT_DMON)] = [True] * int(1/DT_DMON)
      alert_lvls, _ = self._run_seq(ds_vector, interaction_vector, 2*always_true, 2*always_false)
      assert alert_lvls[int(dm_settings._WHEELTOUCH_POLICY_ALERT_1_TIMEOUT/2/DT_DMON)] == 0
      assert alert_lvls[int((INVISIBLE_SECONDS_TO_ORANGE-0.1)/DT_DMON)] == 2
      assert alert_lvls[int((INVISIBLE_SECONDS_TO_ORANGE+0.1)/DT_DMON)] == 0
      if _visible_time == 0.5:
        assert alert_lvls[int((INVISIBLE_SECONDS_TO_ORANGE*2+1-0.1)/DT_DMON)] == 2
        assert alert_lvls[int((INVISIBLE_SECONDS_TO_ORANGE*2+1+0.1+_visible_time)/DT_DMON)] == 2
      elif _visible_time == 10:
        assert alert_lvls[int((INVISIBLE_SECONDS_TO_ORANGE*2+1-0.1)/DT_DMON)] == 2
        assert alert_lvls[int((INVISIBLE_SECONDS_TO_ORANGE*2+1+0.1+_visible_time)/DT_DMON)] == 0

  # engaged, invisible driver, down to red, driver appears and then touches wheel, then disengages/reengages
  #  - only disengage will clear the alert
  def test_last_second_responder(self):
    _visible_time = 2  # seconds
    ds_vector = always_no_face[:]
    interaction_vector = always_false[:]
    op_vector = always_true[:]
    ds_vector[int(INVISIBLE_SECONDS_TO_RED/DT_DMON):int((INVISIBLE_SECONDS_TO_RED+_visible_time)/DT_DMON)] = [msg_ATTENTIVE] * int(_visible_time/DT_DMON)
    interaction_vector[int((INVISIBLE_SECONDS_TO_RED+_visible_time)/DT_DMON):int((INVISIBLE_SECONDS_TO_RED+_visible_time+1)/DT_DMON)] = [True] * int(1/DT_DMON)
    op_vector[int((INVISIBLE_SECONDS_TO_RED+_visible_time+1)/DT_DMON):int((INVISIBLE_SECONDS_TO_RED+_visible_time+0.5)/DT_DMON)] = [False] * int(0.5/DT_DMON)
    alert_lvls, _ = self._run_seq(ds_vector, interaction_vector, op_vector, always_false)
    assert alert_lvls[int(dm_settings._WHEELTOUCH_POLICY_ALERT_1_TIMEOUT/2/DT_DMON)] == 0
    assert alert_lvls[int((INVISIBLE_SECONDS_TO_ORANGE-0.1)/DT_DMON)] == 2
    assert alert_lvls[int((INVISIBLE_SECONDS_TO_RED-0.1)/DT_DMON)] == 3
    assert alert_lvls[int((INVISIBLE_SECONDS_TO_RED+0.5*_visible_time)/DT_DMON)] == 3
    assert alert_lvls[int((INVISIBLE_SECONDS_TO_RED+_visible_time+0.5)/DT_DMON)] == 3
    assert alert_lvls[int((INVISIBLE_SECONDS_TO_RED+_visible_time+1+0.1)/DT_DMON)] == 0

  # disengaged, always distracted driver
  #  - dm should stay quiet when not engaged
  def test_pure_dashcam_user(self):
    alert_lvls, _ = self._run_seq(always_distracted, always_false, always_false, always_false)
    assert all(a == 0 for a in alert_lvls)

  # engaged, car stops at traffic light, down to orange, no action, then car starts moving
  #  - should only reach green when stopped, but continues counting down on launch
  def test_long_traffic_light_victim(self):
    _redlight_time = 60  # seconds
    lowspeed_vector = always_true[:]
    lowspeed_vector[int(_redlight_time/DT_DMON):] = [False] * int((TEST_TIMESPAN-_redlight_time)/DT_DMON)
    alert_lvls, d_status = self._run_seq(always_distracted, always_false, always_true, lowspeed_vector)
    s = d_status.settings
    assert alert_lvls[int((_redlight_time-0.1)/DT_DMON)] == 0
    _alert_1_to_2 = s._VISION_POLICY_ALERT_2_TIMEOUT - s._VISION_POLICY_ALERT_1_TIMEOUT
    assert alert_lvls[int((_redlight_time+0.5)/DT_DMON)] == 1
    assert alert_lvls[int((_redlight_time+_alert_1_to_2+0.5)/DT_DMON)] == 2

  # engaged, distracted while moving, then car stops after reaching orange
  #  - should reset timer to pre green at low speed
  def test_distracted_then_stops(self):
    _stop_time = DISTRACTED_SECONDS_TO_ORANGE + 1  # stop 1 second after reaching orange
    lowspeed_vector = always_false[:]
    lowspeed_vector[int(_stop_time/DT_DMON):] = [True] * int((TEST_TIMESPAN-_stop_time)/DT_DMON)
    alert_lvls, _ = self._run_seq(always_distracted, always_false, always_true, lowspeed_vector)
    # just before and briefly after stopping: orange alert; goes away quickly after stopped
    assert alert_lvls[int((_stop_time+0.1)/DT_DMON)] == 2
    assert alert_lvls[int((_stop_time+0.5)/DT_DMON)] == 0

  # engaged, model is somehow uncertain and driver is distracted
  #  - should fall back to wheel touch after uncertain alert
  def test_somehow_indecisive_model(self):
    ds_vector = [msg_DISTRACTED_BUT_SOMEHOW_UNCERTAIN] * int(TEST_TIMESPAN/DT_DMON)
    interaction_vector = always_false[:]
    alert_lvls, d_status = self._run_seq(ds_vector, interaction_vector, always_true, always_false)
    s = d_status.settings
    assert alert_lvls[int((INVISIBLE_SECONDS_TO_ORANGE-1+DT_DMON*s._HI_STD_FALLBACK_TIME-0.1)/DT_DMON)] == 1
    assert alert_lvls[int((INVISIBLE_SECONDS_TO_ORANGE-1+DT_DMON*s._HI_STD_FALLBACK_TIME+0.1)/DT_DMON)] == 2
    assert alert_lvls[int((INVISIBLE_SECONDS_TO_RED-1+DT_DMON*s._HI_STD_FALLBACK_TIME+0.1)/DT_DMON)] == 3


@pytest.mark.parametrize(("wheel_on_right", "expected_yaw", "expected_phone_prob"), [
  (False, -LEFT_MODEL_YAW, LEFT_PHONE_PROB),
  (True, RIGHT_MODEL_YAW, RIGHT_PHONE_PROB),
])
def test_standard_yaw_mode_uses_detected_wheel_side(driver_monitoring_yaw_mode, wheel_on_right, expected_yaw, expected_phone_prob):
  driver_monitoring_yaw_mode.put(DRIVER_MONITORING_YAW_MODE_PARAM, "standard", block=True)
  dm = DriverMonitoring(rhd_saved=wheel_on_right)

  dm._update_states(make_yaw_mode_msg(), [0., 0., 0.], 0., False, False, steering_angle_deg=TEST_STEERING_ANGLE_DEG)

  assert dm.wheel_on_right == wheel_on_right
  assert dm.phone_prob == pytest.approx(expected_phone_prob)
  assert dm.pose.yaw == pytest.approx(expected_yaw)
  assert dm.pose.steer_yaw_offset == pytest.approx(RHD_STEER_YAW_OFFSET if wheel_on_right else LHD_STEER_YAW_OFFSET)


@pytest.mark.parametrize("wheel_on_right", [False, True])
def test_rhd_head_invert_lhd_mode_uses_right_head_and_normalizes_yaw(driver_monitoring_yaw_mode, wheel_on_right):
  driver_monitoring_yaw_mode.put(DRIVER_MONITORING_YAW_MODE_PARAM, "rhd_head_invert_lhd", block=True)
  dm = DriverMonitoring(rhd_saved=wheel_on_right)

  dm._update_states(make_yaw_mode_msg(), [0., 0., 0.], 0., False, False, steering_angle_deg=TEST_STEERING_ANGLE_DEG)

  assert dm.wheel_on_right == wheel_on_right
  assert dm.phone_prob == pytest.approx(RIGHT_PHONE_PROB)
  assert dm.pose.yaw == pytest.approx(RIGHT_MODEL_YAW)
  # Special mode mirrors only head yaw; steering compensation stays wheel-side based.
  assert dm.pose.steer_yaw_offset == pytest.approx(RHD_STEER_YAW_OFFSET if wheel_on_right else LHD_STEER_YAW_OFFSET)


@pytest.mark.parametrize(("wheel_on_right", "yaw_mode", "expected_uses_rhd_head"), [
  (False, "standard", False),
  (True, "standard", True),
  (False, "rhd_head_invert_lhd", True),
  (True, "rhd_head_invert_lhd", True),
])
def test_state_packet_publishes_effective_driver_head(
    driver_monitoring_yaw_mode, wheel_on_right, yaw_mode, expected_uses_rhd_head):
  driver_monitoring_yaw_mode.put(DRIVER_MONITORING_YAW_MODE_PARAM, yaw_mode, block=True)
  dm = DriverMonitoring(rhd_saved=wheel_on_right)

  dm._update_states(make_yaw_mode_msg(), [0., 0., 0.], 0., False, False)

  assert dm.get_state_packet().driverMonitoringState.visionPolicyState.usesRhdHead is expected_uses_rhd_head


def test_state_packet_uses_cached_yaw_mode_not_changed_param(driver_monitoring_yaw_mode):
  driver_monitoring_yaw_mode.put(DRIVER_MONITORING_YAW_MODE_PARAM, "standard", block=True)
  cached_mode_dm = DriverMonitoring(rhd_saved=False)

  # Changing the desired offroad preference must not change a running policy.
  driver_monitoring_yaw_mode.put(DRIVER_MONITORING_YAW_MODE_PARAM, "rhd_head_invert_lhd", block=True)
  cached_mode_dm._update_states(make_yaw_mode_msg(), [0., 0., 0.], 0., False, False)
  assert cached_mode_dm.get_state_packet().driverMonitoringState.visionPolicyState.usesRhdHead is False

  # A fresh policy reads the new preference and publishes its new effective head.
  new_mode_dm = DriverMonitoring(rhd_saved=False)
  new_mode_dm._update_states(make_yaw_mode_msg(), [0., 0., 0.], 0., False, False)
  assert new_mode_dm.get_state_packet().driverMonitoringState.visionPolicyState.usesRhdHead is True


@pytest.mark.parametrize("mode", [None, "not-a-yaw-mode"])
def test_missing_or_unrecognized_yaw_mode_falls_back_to_standard(
    driver_monitoring_yaw_mode, mode):
  if mode is not None:
    driver_monitoring_yaw_mode.put(DRIVER_MONITORING_YAW_MODE_PARAM, mode, block=True)
  dm = DriverMonitoring(rhd_saved=False)

  dm._update_states(make_yaw_mode_msg(), [0., 0., 0.], 0., False, False)

  assert dm.phone_prob == pytest.approx(LEFT_PHONE_PROB)
  assert dm.pose.yaw == pytest.approx(-LEFT_MODEL_YAW)


def _build_sm(selfdrive_enabled, lat_active, steering_pressed, gas_pressed):
  cs = car.CarState.new_message()
  cs.vEgo = 30.0
  cs.gearShifter = car.CarState.GearShifter.drive
  cs.steeringPressed = steering_pressed
  cs.gasPressed = gas_pressed
  ss = log.SelfdriveState.new_message()
  ss.enabled = selfdrive_enabled
  cc = car.CarControl.new_message()
  cc.latActive = lat_active
  mv2 = log.ModelDataV2.new_message()
  mv2.meta.disengagePredictions.brakeDisengageProbs = [0.0]
  lc = log.LiveCalibrationData.new_message()
  lc.rpyCalib = [0.0, 0.0, 0.0]
  return {
    'carState': cs, 'selfdriveState': ss, 'carControl': cc,
    'modelV2': mv2, 'liveCalibration': lc, 'driverStateV2': make_msg(False),
  }


@pytest.mark.parametrize("selfdrive_enabled, lat_active, steering, gas, expected_op_engaged, expected_driver_engaged", [
  (False, False, False, False, False, False),  # disabled
  (True,  False, False, False, True,  False),  # OP enabled
  (False, True,  False, False, True,  False),  # MADS lat-only
  (True,  True,  False, False, True,  False),  # both active
  (False, True,  False, True,  True,  False),  # MADS lat-only + gas
  (True,  True,  False, True,  True,  True),   # full op + gas: override
  (False, True,  True,  False, True,  True),   # MADS lat-only + wheel touch: override
])
def test_run_step_engagement(selfdrive_enabled, lat_active, steering, gas,
                             expected_op_engaged, expected_driver_engaged):
  sm = _build_sm(selfdrive_enabled, lat_active, steering, gas)
  dm = DriverMonitoring()
  captured = {}
  orig = dm._update_events

  def spy(driver_engaged, op_engaged, lowspeed, wrong_gear):
    captured['driver_engaged'] = driver_engaged
    captured['op_engaged'] = op_engaged
    return orig(driver_engaged, op_engaged, lowspeed, wrong_gear)

  object.__setattr__(dm, '_update_events', spy)
  dm.run_step(sm, demo=False)
  assert captured['op_engaged'] == expected_op_engaged
  assert captured['driver_engaged'] == expected_driver_engaged
