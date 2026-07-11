"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from enum import IntEnum
import math

from opendbc.car import DT_CTRL, CanData, structs
from opendbc.car.tesla.values import CANBUS, TeslaFlags
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP


GTW_CAR_CONFIG = "GTW_carConfig"
DRIVER_ASSIST_CONTROL = "UI_driverAssistControl"
AUTOPILOT_CONTROL = "UI_autopilotControl"

GTW_CAR_CONFIG_BYTES = tuple(f"GTW_carConfigByte{i}" for i in range(8))
DRIVER_ASSIST_CONTROL_BYTES = tuple(f"UI_driverAssistControlByte{i}" for i in range(8))
AUTOPILOT_CONTROL_BYTES = tuple(f"UI_autopilotControlByte{i}" for i in range(8))

AP_FIRST_STABLE_FRAMES = round(1.0 / DT_CTRL)


class SpeedProfileProtocol(IntEnum):
  unknown = 0
  hw3 = 1
  hw4_fsd14 = 2


FOLLOW_DISTANCE_PROFILES = {
  SpeedProfileProtocol.hw3: {1: 2, 2: 1, 3: 0},
  SpeedProfileProtocol.hw4_fsd14: {1: 3, 2: 2, 3: 1, 4: 0, 5: 4},
}


def raw_can_values(dat: bytes, signal_names: tuple[str, ...]) -> dict[str, int]:
  if len(dat) != len(signal_names):
    raise ValueError(f"expected {len(signal_names)} bytes, got {len(dat)}")
  return dict(zip(signal_names, dat, strict=True))


def _raw_can_frames(vl_all: dict[str, list[float]], signal_names: tuple[str, ...]) -> list[bytes]:
  values = [vl_all.get(name, []) for name in signal_names]
  if not values or not any(values):
    return []
  if len({len(v) for v in values}) != 1:
    return []
  return [bytes(int(v) for v in frame) for frame in zip(*values, strict=True)]


def detect_das_hardware(dat: bytes) -> int:
  """Return Tesla's raw dasHw value, treating empty gateway stubs as unknown."""
  if len(dat) != 8 or not any(dat):
    return 0
  das_hw = (dat[0] >> 6) & 0x03
  return das_hw if das_hw in (2, 3) else 0


def select_protocol(das_hw: int, fsd_14: bool) -> SpeedProfileProtocol:
  """Select only profile layouts constrained by both hardware and firmware."""
  if das_hw == 2 and not fsd_14:
    return SpeedProfileProtocol.hw3
  if das_hw == 3 and fsd_14:
    return SpeedProfileProtocol.hw4_fsd14
  return SpeedProfileProtocol.unknown


def profile_for_follow_distance(protocol: SpeedProfileProtocol, follow_distance: int) -> int | None:
  return FOLLOW_DISTANCE_PROFILES.get(protocol, {}).get(follow_distance)


def apply_speed_profile(dat: bytes, protocol: SpeedProfileProtocol, profile: int | None) -> bytes | None:
  """Clone a live 0x3FD payload and change only its protocol-specific profile field."""
  if len(dat) != 8 or profile is None:
    return None

  mux = dat[0] & 0x07
  modified = bytearray(dat)
  if protocol == SpeedProfileProtocol.hw3:
    # HW3 only applies the profile when FSD is selected in the mux-0 frame.
    if mux != 0 or not (dat[4] & 0x40) or not 0 <= profile <= 2:
      return None
    modified[6] = (modified[6] & ~0x06) | (profile << 1)
  elif protocol == SpeedProfileProtocol.hw4_fsd14:
    if mux != 2 or not 0 <= profile <= 4:
      return None
    modified[7] = (modified[7] & ~0xE0) | (profile << 5)
  else:
    return None

  return bytes(modified) if modified != dat else None


class TeslaSpeedProfileState:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.enabled = bool(CP_SP.flags & TeslaFlagsSP.SPEED_PROFILE)
    self.fsd_14 = bool(CP.flags & TeslaFlags.FSD_14)
    self.das_hw = 0
    self.protocol = SpeedProfileProtocol.unknown
    self.follow_distance: int | None = None
    self.profile: int | None = None
    self.autopilot_control_frames: list[bytes] = []

  @staticmethod
  def parser_messages(enabled: bool) -> list[tuple[str, float]]:
    if not enabled:
      return []
    # These inputs gate an optional feature and must not invalidate base car state.
    return [(GTW_CAR_CONFIG, math.nan), (DRIVER_ASSIST_CONTROL, math.nan), (AUTOPILOT_CONTROL, math.nan)]

  def _update_protocol(self, das_hw: int) -> bool:
    protocol = select_protocol(das_hw, self.fsd_14)
    changed = das_hw != self.das_hw or protocol != self.protocol
    self.das_hw = das_hw
    self.protocol = protocol
    if changed:
      # Require a new source selection after hardware/layout changes. Panda applies
      # the same rule, and it avoids inventing ordering across CAN IDs in vl_all.
      self.follow_distance = None
      self.profile = None
    return changed

  def update(self, cp_party) -> None:
    self.autopilot_control_frames = []
    if not self.enabled:
      return

    hardware_frames = _raw_can_frames(cp_party.vl_all[GTW_CAR_CONFIG], GTW_CAR_CONFIG_BYTES)
    protocol_changed = False
    for dat in hardware_frames:
      protocol_changed |= self._update_protocol(detect_das_hardware(dat))

    follow_frames = _raw_can_frames(cp_party.vl_all[DRIVER_ASSIST_CONTROL], DRIVER_ASSIST_CONTROL_BYTES)
    profile_changed = False
    if not protocol_changed:
      for dat in follow_frames:
        follow_distance = (dat[5] >> 5) & 0x07
        profile = profile_for_follow_distance(self.protocol, follow_distance)
        if profile is not None:
          profile_changed |= profile != self.profile
          self.follow_distance = follow_distance
          self.profile = profile

    # Keep only the newest live frame for each mux in this parser drain. Never replay
    # a value retained in CANParser.vl from a previous control cycle.
    # Cross-ID ordering is not represented in vl_all, so a new protocol/profile must
    # be followed by a 0x3FD from a later drain before it can authorize a shadow.
    if protocol_changed or profile_changed:
      return

    frames_by_mux: dict[int, bytes] = {}
    for dat in _raw_can_frames(cp_party.vl_all[AUTOPILOT_CONTROL], AUTOPILOT_CONTROL_BYTES):
      frames_by_mux[dat[0] & 0x07] = dat
    self.autopilot_control_frames = list(frames_by_mux.values())


class TeslaSpeedProfileCarController:
  def __init__(self, CP_SP: structs.CarParamsSP):
    self.speed_profile_enabled = bool(CP_SP.flags & TeslaFlagsSP.SPEED_PROFILE)
    self.speed_profile_engaged_frames = 0

  def update_speed_profile(self, CC: structs.CarControl, CS) -> list[CanData]:
    if not self.speed_profile_enabled:
      return []

    if CC.enabled:
      self.speed_profile_engaged_frames += 1
    else:
      self.speed_profile_engaged_frames = 0
      return []

    # Match Flipper's AP-First gate: do not shadow 0x3FD on the engagement edge.
    if self.speed_profile_engaged_frames <= AP_FIRST_STABLE_FRAMES:
      return []

    can_sends = []
    state = CS.speed_profile
    for dat in state.autopilot_control_frames:
      modified = apply_speed_profile(dat, state.protocol, state.profile)
      if modified is not None:
        can_sends.append(CanData(0x3FD, modified, CANBUS.autopilot_party))
    return can_sends
