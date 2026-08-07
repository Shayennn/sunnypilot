from types import SimpleNamespace

import pytest

from openpilot.cereal import log
from openpilot.selfdrive.ui.mici.onroad import driver_state as driver_state_module


@pytest.mark.parametrize(("is_rhd", "policy_yaw", "expected_visual_yaw", "expected_head"), [
  # physical right: Mici visual yaw is negative
  (False, 0.2, -0.2, "left"),
  (True, -0.2, -0.2, "right"),
  # physical left: Mici visual yaw is positive
  (False, -0.2, 0.2, "left"),
  (True, 0.2, 0.2, "right"),
])
def test_physical_turn_renders_correct_ring_and_uses_wheel_side_head(
  monkeypatch, is_rhd, policy_yaw, expected_visual_yaw, expected_head,
):
  """Policy-normalized yaw still renders the physical turn direction."""
  dm_state = SimpleNamespace(
    activePolicy=log.DriverMonitoringState.MonitoringPolicy.vision,
    isRHD=is_rhd,
    visionPolicyState=SimpleNamespace(
      faceDetected=True,
      awarenessPercent=100,
      pose=SimpleNamespace(pitch=0.0, yaw=policy_yaw),
    ),
  )
  left_head = SimpleNamespace(name="left")
  right_head = SimpleNamespace(name="right")
  driver_state = SimpleNamespace(leftDriverData=left_head, rightDriverData=right_head)
  fake_ui_state = SimpleNamespace(sm={
    "driverMonitoringState": dm_state,
    "driverStateV2": driver_state,
  })
  monkeypatch.setattr(driver_state_module, "get_ui_state", lambda: fake_ui_state)
  renderer = object.__new__(driver_state_module.DriverStateRenderer)
  renderer._force_active = False

  driver_data = renderer.get_driver_data()

  assert driver_data is (right_head if expected_head == "right" else left_head)
  # Mici's visual convention uses negative yaw for right and positive for left.
  assert renderer._face_yaw == pytest.approx(expected_visual_yaw)
