from types import SimpleNamespace

import pytest

from openpilot.cereal import log
from openpilot.selfdrive.ui.mici.onroad import driver_state as driver_state_module


@pytest.mark.parametrize(("is_rhd", "uses_rhd_head", "policy_yaw", "expected_visual_yaw", "expected_head"), [
  # physical right: Mici visual yaw is negative
  (False, False, 0.2, -0.2, "left"),   # Standard LHD
  (True, True, -0.2, -0.2, "right"),   # Standard RHD
  (True, False, -0.2, -0.2, "right"),  # Old-log Standard RHD fallback
  (False, True, -0.2, -0.2, "right"),  # RHD / Invert LHD
  (True, True, -0.2, -0.2, "right"),   # RHD / Invert RHD
  # physical left: Mici visual yaw is positive
  (False, False, -0.2, 0.2, "left"),   # Standard LHD
  (True, True, 0.2, 0.2, "right"),     # Standard RHD
  (False, True, 0.2, 0.2, "right"),    # RHD / Invert LHD
  (True, True, 0.2, 0.2, "right"),     # RHD / Invert RHD
])
def test_physical_turn_renders_correct_ring_and_uses_policy_head(
  monkeypatch, is_rhd, uses_rhd_head, policy_yaw, expected_visual_yaw, expected_head,
):
  """Physical turns retain their Mici ring direction in every side/mode pair.

  Monitoring normalizes standard LHD and RHD yaw differently. The special
  mode normalizes LHD as the RHD head too, so both the ring and camera preview
  must use that same selected head.
  """
  dm_state = SimpleNamespace(
    activePolicy=log.DriverMonitoringState.MonitoringPolicy.vision,
    isRHD=is_rhd,
    visionPolicyState=SimpleNamespace(
      faceDetected=True,
      awarenessPercent=100,
      pose=SimpleNamespace(pitch=0.0, yaw=policy_yaw),
      usesRhdHead=uses_rhd_head,
    ),
  )
  left_head = SimpleNamespace(name="left")
  right_head = SimpleNamespace(name="right")
  driver_state = SimpleNamespace(leftDriverData=left_head, rightDriverData=right_head)
  monkeypatch.setattr(driver_state_module.ui_state, "sm", {
    "driverMonitoringState": dm_state,
    "driverStateV2": driver_state,
  })
  renderer = object.__new__(driver_state_module.DriverStateRenderer)
  renderer._force_active = False

  driver_data = renderer.get_driver_data()

  assert driver_data is (right_head if expected_head == "right" else left_head)
  # With zero pitch, negative visual yaw rotates right and positive rotates left.
  assert renderer._face_yaw == pytest.approx(expected_visual_yaw)


def test_same_renderer_tracks_published_effective_head(monkeypatch):
  """A policy state transition updates the effective head without recreating the UI."""
  dm_state = SimpleNamespace(
    activePolicy=log.DriverMonitoringState.MonitoringPolicy.vision,
    isRHD=False,
    visionPolicyState=SimpleNamespace(
      faceDetected=True,
      awarenessPercent=100,
      pose=SimpleNamespace(pitch=0.0, yaw=0.2),
      usesRhdHead=False,
    ),
  )
  left_head = SimpleNamespace(name="left")
  right_head = SimpleNamespace(name="right")
  monkeypatch.setattr(driver_state_module.ui_state, "sm", {
    "driverMonitoringState": dm_state,
    "driverStateV2": SimpleNamespace(leftDriverData=left_head, rightDriverData=right_head),
  })

  renderer = object.__new__(driver_state_module.DriverStateRenderer)
  renderer._force_active = False

  assert renderer.get_driver_data() is left_head
  assert renderer._face_yaw == pytest.approx(-0.2)

  # The next policy publication selects and normalizes the RHD head.
  dm_state.visionPolicyState.pose.yaw = -0.2
  dm_state.visionPolicyState.usesRhdHead = True
  assert renderer.get_driver_data() is right_head
  assert renderer._face_yaw == pytest.approx(-0.2)
