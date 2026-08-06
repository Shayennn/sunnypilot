"""Shared driver-monitoring yaw conventions.

The policy consumes the model's driver head while the UI needs a yaw expressed
in its own visual coordinate system. Keep the mode and side transforms here so
the two paths cannot develop independent sign rules.
"""

DRIVER_MONITORING_YAW_MODE_PARAM = "DriverMonitoringYawMode"
DRIVER_MONITORING_YAW_MODE_STANDARD = "standard"
DRIVER_MONITORING_YAW_MODE_RHD_HEAD_INVERT_LHD = "rhd_head_invert_lhd"


def get_driver_monitoring_yaw_mode(params):
  yaw_mode = params.get(DRIVER_MONITORING_YAW_MODE_PARAM)
  return yaw_mode if yaw_mode == DRIVER_MONITORING_YAW_MODE_RHD_HEAD_INVERT_LHD else DRIVER_MONITORING_YAW_MODE_STANDARD


def uses_rhd_head(wheel_on_right, yaw_mode):
  """Whether driver-monitoring must use and normalize the RHD model head."""
  return wheel_on_right or yaw_mode == DRIVER_MONITORING_YAW_MODE_RHD_HEAD_INVERT_LHD


def select_driver_data_for_head(driver_state, use_rhd_head):
  return driver_state.rightDriverData if use_rhd_head else driver_state.leftDriverData


def select_driver_data(driver_state, wheel_on_right, yaw_mode):
  return select_driver_data_for_head(driver_state, uses_rhd_head(wheel_on_right, yaw_mode))


def normalize_driver_yaw(yaw, wheel_on_right, yaw_mode):
  """Normalize face_orientation_from_model yaw into policy coordinates."""
  return -yaw if uses_rhd_head(wheel_on_right, yaw_mode) else yaw


def policy_yaw_to_ui_yaw(yaw, use_rhd_head):
  """Convert already-normalized policy yaw into Mici's visual coordinate system.

  Mici's native yaw convention matches the normalized RHD-head policy
  convention. Only Standard LHD reaches the policy without RHD-head
  normalization, so only that case needs a visual sign flip.
  """
  return yaw if use_rhd_head else -yaw


def uses_rhd_head_from_state(dm_state):
  """Read the policy-selected head, with old-log fallback to the RHD side."""
  return dm_state.isRHD or dm_state.visionPolicyState.usesRhdHead
