from types import SimpleNamespace

import pytest

from openpilot.selfdrive.monitoring.yaw import (
  DRIVER_MONITORING_YAW_MODE_RHD_HEAD_INVERT_LHD,
  DRIVER_MONITORING_YAW_MODE_STANDARD,
  get_driver_monitoring_yaw_mode,
  normalize_driver_yaw,
  policy_yaw_to_ui_yaw,
  select_driver_data,
  select_driver_data_for_head,
  uses_rhd_head,
  uses_rhd_head_from_state,
)


@pytest.mark.parametrize(("wheel_on_right", "yaw_mode", "selected_side", "face_orientation_yaw", "normalized_yaw", "ui_yaw"), [
  (False, DRIVER_MONITORING_YAW_MODE_STANDARD, "left", 0.25, 0.25, -0.25),
  (False, DRIVER_MONITORING_YAW_MODE_STANDARD, "left", -0.25, -0.25, 0.25),
  (True, DRIVER_MONITORING_YAW_MODE_STANDARD, "right", 0.25, -0.25, -0.25),
  (True, DRIVER_MONITORING_YAW_MODE_STANDARD, "right", -0.25, 0.25, 0.25),
  (False, DRIVER_MONITORING_YAW_MODE_RHD_HEAD_INVERT_LHD, "right", 0.25, -0.25, -0.25),
  (False, DRIVER_MONITORING_YAW_MODE_RHD_HEAD_INVERT_LHD, "right", -0.25, 0.25, 0.25),
  (True, DRIVER_MONITORING_YAW_MODE_RHD_HEAD_INVERT_LHD, "right", 0.25, -0.25, -0.25),
  (True, DRIVER_MONITORING_YAW_MODE_RHD_HEAD_INVERT_LHD, "right", -0.25, 0.25, 0.25),
])
def test_yaw_mode_head_selection_and_coordinate_transforms(
    wheel_on_right, yaw_mode, selected_side, face_orientation_yaw, normalized_yaw, ui_yaw):
  """`face_orientation_yaw` is the yaw returned by face_orientation_from_model."""
  driver_state = SimpleNamespace(leftDriverData="left", rightDriverData="right")
  use_rhd_head = uses_rhd_head(wheel_on_right, yaw_mode)

  assert select_driver_data(driver_state, wheel_on_right, yaw_mode) == selected_side
  assert select_driver_data_for_head(driver_state, use_rhd_head) == selected_side
  policy_yaw = normalize_driver_yaw(face_orientation_yaw, wheel_on_right, yaw_mode)
  assert policy_yaw == pytest.approx(normalized_yaw)
  assert policy_yaw_to_ui_yaw(policy_yaw, use_rhd_head) == pytest.approx(ui_yaw)


@pytest.mark.parametrize(("value", "expected"), [
  (None, DRIVER_MONITORING_YAW_MODE_STANDARD),
  (DRIVER_MONITORING_YAW_MODE_STANDARD, DRIVER_MONITORING_YAW_MODE_STANDARD),
  (DRIVER_MONITORING_YAW_MODE_RHD_HEAD_INVERT_LHD, DRIVER_MONITORING_YAW_MODE_RHD_HEAD_INVERT_LHD),
  ("invalid", DRIVER_MONITORING_YAW_MODE_STANDARD),
])
def test_invalid_yaw_mode_falls_back_to_standard(value, expected):
  assert get_driver_monitoring_yaw_mode(SimpleNamespace(get=lambda _: value)) == expected


@pytest.mark.parametrize(("is_rhd", "uses_rhd_head", "expected"), [
  (False, False, False),
  (False, True, True),
  (True, False, True),
  (True, True, True),
])
def test_uses_rhd_head_from_state_falls_back_to_old_rhd_field(is_rhd, uses_rhd_head, expected):
  dm_state = SimpleNamespace(
    isRHD=is_rhd,
    visionPolicyState=SimpleNamespace(usesRhdHead=uses_rhd_head),
  )
  assert uses_rhd_head_from_state(dm_state) is expected
