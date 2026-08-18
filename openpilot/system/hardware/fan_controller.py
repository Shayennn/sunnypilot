#!/usr/bin/env python3
import numpy as np
from openpilot.common.hardware import HARDWARE
from openpilot.common.pid import PIDController

# raise fan setpoint on tici/tizi to reduce noise
# after raising LMH threshold in AGNOS 18.1 to prevent CPU throttling
OFFSET = 0 if HARDWARE.get_device_type() == "mici" else 5

# The stock curve is treated as calibrated for San Diego's roughly 18 C annual
# average. Climate compensation only changes fan feedforward and the idle fan
# ceiling; absolute device-temperature targets and thermal limits stay fixed.
SAN_DIEGO_AVERAGE_TEMP_C = 18.0
MIN_AVERAGE_TEMP_C = -10.0
MAX_AVERAGE_TEMP_C = 40.0
MIN_CLIMATE_ADJUSTMENT_C = -10.0
MAX_CLIMATE_ADJUSTMENT_C = 20.0
BASE_IDLE_FAN_LIMIT = 30
MAX_IDLE_FAN_LIMIT = 50


def get_climate_adjustment(average_temp_c: float) -> float:
  """Return the bounded feedforward temperature offset from the stock climate."""
  average_temp_c = float(average_temp_c)
  if not np.isfinite(average_temp_c):
    average_temp_c = SAN_DIEGO_AVERAGE_TEMP_C

  average_temp_c = float(np.clip(average_temp_c, MIN_AVERAGE_TEMP_C, MAX_AVERAGE_TEMP_C))
  return float(np.clip(average_temp_c - SAN_DIEGO_AVERAGE_TEMP_C,
                       MIN_CLIMATE_ADJUSTMENT_C, MAX_CLIMATE_ADJUSTMENT_C))


def get_idle_fan_limit(average_temp_c: float) -> int:
  """Raise the ignition-off ceiling in hotter climates without lowering stock safety."""
  climate_adjustment = max(0.0, get_climate_adjustment(average_temp_c))
  return int(np.clip(BASE_IDLE_FAN_LIMIT + round(climate_adjustment),
                     BASE_IDLE_FAN_LIMIT, MAX_IDLE_FAN_LIMIT))


class FanController:
  def __init__(self, rate: int) -> None:
    self.last_ignition = False
    self.controller = PIDController(k_p=0, k_i=4e-3, rate=rate)

  def update(self, cur_temp: float, ignition: bool,
             average_temp_c: float = SAN_DIEGO_AVERAGE_TEMP_C) -> int:
    climate_adjustment = get_climate_adjustment(average_temp_c)
    self.controller.pos_limit = 100 if ignition else get_idle_fan_limit(average_temp_c)
    self.controller.neg_limit = 30 if ignition else 0

    if ignition != self.last_ignition:
      self.controller.reset()
    self.last_ignition = ignition

    return int(self.controller.update(
                 error=(cur_temp - (75 + OFFSET)),  # temperature setpoint in C
                 feedforward=np.interp(cur_temp + climate_adjustment,
                                       [60.0 + OFFSET, 100.0 + OFFSET], [0, 100])
              ))
