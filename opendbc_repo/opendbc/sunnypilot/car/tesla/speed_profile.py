"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math
from enum import IntEnum

from opendbc.car import structs
from opendbc.car.tesla.values import TeslaFlags


GTW_CAR_CONFIG = "GTW_carConfig"
DRIVER_ASSIST_CONTROL = "UI_driverAssistControl"

GTW_CAR_CONFIG_BYTES = tuple(f"GTW_carConfigByte{i}" for i in range(8))
DRIVER_ASSIST_CONTROL_BYTES = tuple(f"UI_driverAssistControlByte{i}" for i in range(8))

SELECTOR_CONFIRMATION_SAMPLES = 2


class SpeedProfileProtocol(IntEnum):
  unknown = 0
  hw3 = 1
  hw4_fsd14 = 2


VALID_SELECTORS = {
  SpeedProfileProtocol.hw3: frozenset((1, 2, 3)),
  SpeedProfileProtocol.hw4_fsd14: frozenset((1, 2, 3, 4, 5)),
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


def detect_das_hardware(dat: bytes) -> int | None:
  """Return Tesla's raw dasHw value, treating empty gateway stubs as unknown."""
  if len(dat) != 8 or not any(dat):
    return None
  return (dat[0] >> 6) & 0x03


def select_protocol(das_hw: int | None, fsd_14: bool) -> SpeedProfileProtocol:
  """Select only profile layouts constrained by both hardware and firmware."""
  if das_hw == 2 and not fsd_14:
    return SpeedProfileProtocol.hw3
  # FSD_14 is derived from an exact HW4 EPS firmware match, so it is a safe
  # fallback when some Juniper/Giga gateways forward an all-zero 0x398 stub.
  # A populated, conflicting hardware value still fails closed.
  if fsd_14 and das_hw in (None, 3):
    return SpeedProfileProtocol.hw4_fsd14
  return SpeedProfileProtocol.unknown


class TeslaSpeedProfileInputState:
  def __init__(self, CP: structs.CarParams):
    self.enabled = bool(CP.openpilotLongitudinalControl)
    self.fsd_14 = bool(CP.flags & TeslaFlags.FSD_14)
    self.das_hw: int | None = None
    self.protocol = select_protocol(self.das_hw, self.fsd_14)

    self.selector: int | None = None
    self._candidate_selector: int | None = None
    self._candidate_samples = 0
    self._pending_gap_adjust_events = 0

  @staticmethod
  def parser_messages(enabled: bool) -> list[tuple[str, float]]:
    if not enabled:
      return []
    # These optional inputs must not invalidate base Tesla car state when absent.
    return [(GTW_CAR_CONFIG, math.nan), (DRIVER_ASSIST_CONTROL, math.nan)]

  def _reset_source(self) -> None:
    self.selector = None
    self._candidate_selector = None
    self._candidate_samples = 0
    self._pending_gap_adjust_events = 0

  def _update_protocol(self, das_hw: int | None) -> bool:
    protocol = select_protocol(das_hw, self.fsd_14)
    changed = das_hw != self.das_hw or protocol != self.protocol
    self.das_hw = das_hw
    self.protocol = protocol
    if changed:
      self._reset_source()
    return changed

  def _observe_selector(self, selector: int) -> None:
    if selector not in VALID_SELECTORS.get(self.protocol, ()):
      # Invalid source values break an in-progress confirmation. The last
      # confirmed selector remains the baseline and never creates an event.
      self._candidate_selector = None
      self._candidate_samples = 0
      return

    if selector == self._candidate_selector:
      self._candidate_samples = min(self._candidate_samples + 1, SELECTOR_CONFIRMATION_SAMPLES)
    else:
      self._candidate_selector = selector
      self._candidate_samples = 1

    if self._candidate_samples >= SELECTOR_CONFIRMATION_SAMPLES and selector != self.selector:
      # The first confirmed value establishes a baseline. Every later raw
      # selector change represents one Tesla profile adjustment, including
      # HW4 transitions within the former personality groups (1<->2, 4<->5).
      if self.selector is not None:
        self._pending_gap_adjust_events += 1
      self.selector = selector

  def update(self, cp_party) -> list[structs.CarState.ButtonEvent]:
    if self.enabled:
      hardware_frames = _raw_can_frames(cp_party.vl_all[GTW_CAR_CONFIG], GTW_CAR_CONFIG_BYTES)
      protocol_changed = False
      for dat in hardware_frames:
        protocol_changed |= self._update_protocol(detect_das_hardware(dat))

      # CANParser.vl_all does not preserve ordering between different message IDs.
      # A protocol observation and selector observation in the same drain therefore
      # cannot prove that the selector used the newly recognized layout.
      if not protocol_changed and self.protocol != SpeedProfileProtocol.unknown:
        follow_frames = _raw_can_frames(cp_party.vl_all[DRIVER_ASSIST_CONTROL], DRIVER_ASSIST_CONTROL_BYTES)
        for dat in follow_frames:
          self._observe_selector((dat[5] >> 5) & 0x07)

    # CarState consumers treat gapAdjustCruise as an edge. Drain at most one
    # queued edge per update so multiple confirmed changes in one CAN batch are
    # not collapsed by consumers that use any(...) over buttonEvents.
    if self._pending_gap_adjust_events:
      self._pending_gap_adjust_events -= 1
      return [structs.CarState.ButtonEvent(pressed=False, type=structs.CarState.ButtonEvent.Type.gapAdjustCruise)]
    return []
