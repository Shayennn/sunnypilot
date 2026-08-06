from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.ui.onroad import driver_state as driver_state_module


@pytest.mark.parametrize(("is_rhd", "uses_rhd_head", "expected_head"), [
  (False, False, "left"),   # Standard LHD
  (True, True, "right"),    # Standard RHD
  (False, True, "right"),   # Special LHD
  (True, True, "right"),    # Special RHD
])
def test_get_driver_data_selects_policy_head_before_update_state(monkeypatch, is_rhd, uses_rhd_head, expected_head):
  """The base renderer chooses the policy head even before its first update."""
  left_head = SimpleNamespace(name="left")
  right_head = SimpleNamespace(name="right")
  monkeypatch.setattr(driver_state_module.ui_state, "sm", {
    "driverMonitoringState": SimpleNamespace(
      isRHD=is_rhd,
      visionPolicyState=SimpleNamespace(usesRhdHead=uses_rhd_head),
    ),
    "driverStateV2": SimpleNamespace(leftDriverData=left_head, rightDriverData=right_head),
  })

  # Avoid widget/texture setup: get_driver_data must be safe for the camera
  # preview to call before the renderer receives _update_state.
  renderer = object.__new__(driver_state_module.DriverStateRenderer)

  driver_data = renderer.get_driver_data()

  assert driver_data is (right_head if expected_head == "right" else left_head)
  assert renderer.is_rhd is is_rhd


def test_get_driver_data_reflects_published_head_changes_without_reconstructing_renderer(monkeypatch):
  left_head = SimpleNamespace(name="left")
  right_head = SimpleNamespace(name="right")
  dm_state = SimpleNamespace(
    isRHD=False,
    visionPolicyState=SimpleNamespace(usesRhdHead=False),
  )
  monkeypatch.setattr(driver_state_module.ui_state, "sm", {
    "driverMonitoringState": dm_state,
    "driverStateV2": SimpleNamespace(leftDriverData=left_head, rightDriverData=right_head),
  })

  renderer = object.__new__(driver_state_module.DriverStateRenderer)

  assert renderer.get_driver_data() is left_head

  dm_state.visionPolicyState.usesRhdHead = True

  assert renderer.get_driver_data() is right_head


@pytest.mark.parametrize(("is_rhd", "uses_rhd_head", "expected_head"), [
  (False, False, "left"),   # Standard LHD
  (True, True, "right"),    # Standard RHD
  (False, True, "right"),   # Special LHD
  (True, True, "right"),    # Special RHD
])
def test_physical_right_turn_draws_horizontal_arc_right_for_selected_policy_head(monkeypatch, is_rhd, uses_rhd_head, expected_head):
  """Every mode renders a selected raw right turn on the right of the icon."""
  left_head = SimpleNamespace(name="left", faceOrientation=(0.0, -0.69, 0.0))
  right_head = SimpleNamespace(name="right", faceOrientation=(0.0, -0.69, 0.0))
  unselected_head = right_head if expected_head == "left" else left_head
  unselected_head.faceOrientation = (0.0, 0.69, 0.0)
  monkeypatch.setattr(driver_state_module.ui_state, "sm", {
    "driverMonitoringState": SimpleNamespace(
      isRHD=is_rhd,
      visionPolicyState=SimpleNamespace(usesRhdHead=uses_rhd_head),
    ),
    "driverStateV2": SimpleNamespace(leftDriverData=left_head, rightDriverData=right_head),
  })

  renderer = object.__new__(driver_state_module.DriverStateRenderer)
  renderer.h_arc_lines = [driver_state_module.rl.Vector2(0, 0) for _ in range(driver_state_module.ARC_POINT_COUNT)]

  driver_data = renderer.get_driver_data()
  orientation = np.asarray(driver_data.faceOrientation)
  scales = np.where(orientation < 0, driver_state_module.SCALES_NEG, driver_state_module.SCALES_POS)
  renderer.driver_pose_vals = 0.8 * orientation * scales  # first _update_state from zero
  renderer.driver_pose_sins = np.sin(renderer.driver_pose_vals)
  renderer.driver_pose_diff = np.abs(renderer.driver_pose_vals)

  icon_x = 1000.0
  delta_x = -renderer.driver_pose_sins[1] * driver_state_module.ARC_LENGTH / 2.0
  arc_data = renderer._calculate_arc_data(
    delta_x, abs(delta_x), icon_x, 0.0,
    renderer.driver_pose_sins[1], renderer.driver_pose_diff[1], is_horizontal=True,
  )

  assert driver_data.name == expected_head
  assert delta_x > 0
  assert arc_data is not None
  assert arc_data.x + arc_data.width > icon_x
  assert min(point.x for point in renderer.h_arc_lines) > icon_x
