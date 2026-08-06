import math
import numbers
from collections.abc import Mapping
from collections import defaultdict, deque
from dataclasses import dataclass, field

from opendbc.car.carlog import carlog
from opendbc.can.dbc import DBC, Signal


MAX_BAD_COUNTER = 5
CAN_INVALID_CNT = 5


@dataclass(frozen=True)
class CounterPolicy:
  name: str
  allowed_deltas: frozenset[int] = frozenset({1})
  expected_period_nanos: int | None = None
  tolerance_nanos: int = 0
  max_elapsed_nanos: int | None = None

  def __post_init__(self) -> None:
    object.__setattr__(self, "allowed_deltas", frozenset(self.allowed_deltas))
    if not isinstance(self.name, str) or not self.name:
      raise ValueError("counter policy name must not be empty")
    if not self.allowed_deltas or any(not isinstance(delta, int) or isinstance(delta, bool) or delta <= 0
                                      for delta in self.allowed_deltas):
      raise ValueError("counter policy deltas must be positive integers")
    alternate_deltas = self.allowed_deltas - {1}
    if alternate_deltas and self.expected_period_nanos is None and self.max_elapsed_nanos is None:
      raise ValueError("alternate counter policy deltas require a timing bound")
    if not alternate_deltas and (self.expected_period_nanos is not None or self.max_elapsed_nanos is not None):
      raise ValueError("counter policy timing bound requires an alternate delta")
    if self.expected_period_nanos is not None and self.max_elapsed_nanos is not None:
      raise ValueError("counter policy exact period and maximum elapsed time are mutually exclusive")
    if self.expected_period_nanos is not None and (
      not isinstance(self.expected_period_nanos, int) or isinstance(self.expected_period_nanos, bool) or self.expected_period_nanos <= 0
    ):
      raise ValueError("counter policy period must be positive")
    if self.max_elapsed_nanos is not None and (
      not isinstance(self.max_elapsed_nanos, int) or isinstance(self.max_elapsed_nanos, bool) or self.max_elapsed_nanos <= 0
    ):
      raise ValueError("counter policy maximum elapsed time must be positive")
    if not isinstance(self.tolerance_nanos, int) or isinstance(self.tolerance_nanos, bool) or self.tolerance_nanos < 0:
      raise ValueError("counter policy tolerance must not be negative")
    if self.expected_period_nanos is None and self.tolerance_nanos != 0:
      raise ValueError("counter policy tolerance requires an expected period")


def get_raw_value(dat: bytes | bytearray, sig: Signal) -> int:
  ret = 0
  i = sig.msb // 8
  bits = sig.size
  while 0 <= i < len(dat) and bits > 0:
    lsb = sig.lsb if (sig.lsb // 8) == i else i * 8
    msb = sig.msb if (sig.msb // 8) == i else (i + 1) * 8 - 1
    size = msb - lsb + 1
    d = (dat[i] >> (lsb - (i * 8))) & ((1 << size) - 1)
    ret |= d << (bits - size)
    bits -= size
    i = i - 1 if sig.is_little_endian else i + 1
  return ret


@dataclass
class MessageState:
  address: int
  name: str
  size: int
  signals: list[Signal]
  ignore_alive: bool = False
  ignore_checksum: bool = False
  ignore_counter: bool = False
  frequency: float = 0.0
  timeout_threshold: float = 1e5  # default to 1Hz threshold
  vals: list[float] = field(default_factory=list)
  all_vals: list[list[float]] = field(default_factory=list)
  timestamps: deque[int] = field(default_factory=lambda: deque(maxlen=500))
  counter: int = 0
  counter_fail: int = 0
  first_seen_nanos: int = 0
  last_warning_log_nanos: int = 0
  counter_policy: CounterPolicy | None = None
  counter_initialized: bool = False
  counter_nanos: int = 0
  counter_policy_accepted_alternate: int = 0
  counter_policy_rejected: int = 0
  counter_policy_last_delta: int | None = None
  counter_policy_last_elapsed_nanos: int | None = None
  counter_policy_last_reject_reason: str | None = None
  counter_policy_last_accept_log_nanos: int | None = None
  counter_policy_last_reject_log_nanos: int | None = None
  counter_policy_last_invalid_log_nanos: int | None = None
  counter_policy_invalid_logged: bool = False
  counter_policy_invalid_latched: bool = False

  def rate_limited_log(self, last_update_nanos: int, msg: str) -> None:
    if (last_update_nanos - self.last_warning_log_nanos) >= 1_000_000_000:
      carlog.warning(f"CANParser: {hex(self.address)} {self.name} {msg}")
      self.last_warning_log_nanos = last_update_nanos

  def counter_policy_log(self, nanos: int, event: str, msg: str, warning: bool = False) -> None:
    assert self.counter_policy is not None
    if event == "reject":
      attr = "counter_policy_last_reject_log_nanos"
      interval_nanos = 1_000_000_000
    elif event == "invalid":
      attr = "counter_policy_last_invalid_log_nanos"
      interval_nanos = 60_000_000_000
    else:
      attr = "counter_policy_last_accept_log_nanos"
      interval_nanos = 60_000_000_000
    last_log_nanos = getattr(self, attr)
    if last_log_nanos is None or (nanos - last_log_nanos) >= interval_nanos:
      log = carlog.warning if warning else carlog.info
      log(f"CANParser: counter_policy_event={event} policy={self.counter_policy.name} " +
          f"address={hex(self.address)} message={self.name} {msg}")
      setattr(self, attr, nanos)

  def reject_counter_policy(self, nanos: int, reason: str, details: str) -> bool:
    assert self.counter_policy is not None
    self.counter_fail = min(self.counter_fail + 1, MAX_BAD_COUNTER)
    self.counter_policy_rejected += 1
    self.counter_policy_last_reject_reason = reason
    grace = self.counter_fail < MAX_BAD_COUNTER
    details = f"reason={reason} {details} counter_fail={self.counter_fail} grace={str(grace).lower()}"
    self.counter_policy_log(nanos, "reject", details, warning=True)
    if not grace and not self.counter_policy_invalid_logged:
      self.counter_policy_log(nanos, "invalid", details, warning=True)
      self.counter_policy_invalid_logged = True
    return grace

  def parse(self, nanos: int, dat: bytes) -> bool:
    if self.counter_policy is not None and len(dat) != self.size:
      self.counter_policy_last_delta = None
      self.counter_policy_last_elapsed_nanos = None
      self.reject_counter_policy(nanos, "size", f"expected_size={self.size} size={len(dat)}")
      return False

    tmp_vals: list[float] = [0.0] * len(self.signals)
    checksum_failed = False
    counter_failed = False
    policy_counter: tuple[int, int] | None = None

    if self.first_seen_nanos == 0:
      self.first_seen_nanos = nanos

    for i, sig in enumerate(self.signals):
      tmp = get_raw_value(dat, sig)
      if sig.is_signed:
        tmp -= ((tmp >> (sig.size - 1)) & 0x1) * (1 << sig.size)

      if not self.ignore_checksum and sig.calc_checksum is not None:
        expected_checksum = sig.calc_checksum(self.address, sig, bytearray(dat))
        if tmp != expected_checksum:
          checksum_failed = True
          self.rate_limited_log(nanos, f"checksum failed: received {hex(tmp)}, calculated {hex(expected_checksum)}")

      if not self.ignore_counter and sig.type == 1:  # COUNTER
        if self.counter_policy is None:
          if not self.update_counter(tmp, sig.size):
            counter_failed = True
        else:
          policy_counter = (tmp, sig.size)

      tmp_vals[i] = tmp * sig.factor + sig.offset

    # A policy counter is updated only after the entire frame passes checksum validation.
    # Legacy messages intentionally keep their original counter/checksum update ordering.
    if checksum_failed:
      return False
    if self.counter_policy is not None and not self.ignore_counter:
      assert policy_counter is not None
      if not self.update_counter_policy(nanos, *policy_counter):
        return False
    elif counter_failed:
      return False

    if not self.vals:
      self.vals = [0.0] * len(self.signals)
      self.all_vals = [[] for _ in self.signals]

    for i, v in enumerate(tmp_vals):
      self.vals[i] = v
      self.all_vals[i].append(v)

    self.timestamps.append(nanos)

    if self.frequency < 1e-5 and len(self.timestamps) >= 3:
      dt = (self.timestamps[-1] - self.timestamps[0]) * 1e-9
      if (dt > 1.0 or (self.timestamps.maxlen is not None and len(self.timestamps) >= self.timestamps.maxlen)) and dt != 0:
        self.frequency = min(len(self.timestamps) / dt, 100.0)
        self.timeout_threshold = (1_000_000_000 / self.frequency) * 10
    return True

  def update_counter(self, cur_count: int, cnt_size: int) -> bool:
    if ((self.counter + 1) & ((1 << cnt_size) - 1)) != cur_count:
      self.counter_fail = min(self.counter_fail + 1, MAX_BAD_COUNTER)
    elif self.counter_fail > 0:
      self.counter_fail -= 1
    self.counter = cur_count
    return self.counter_fail < MAX_BAD_COUNTER

  def update_counter_policy(self, nanos: int, cur_count: int, cnt_size: int) -> bool:
    assert self.counter_policy is not None

    if not self.counter_initialized:
      self.counter = cur_count
      self.counter_nanos = nanos
      self.counter_initialized = True
      carlog.info(f"CANParser: counter_policy_event=seed policy={self.counter_policy.name} " +
                  f"address={hex(self.address)} message={self.name} counter={cur_count}")
      return True

    counter_mask = (1 << cnt_size) - 1
    delta = (cur_count - self.counter) & counter_mask
    elapsed_nanos = nanos - self.counter_nanos
    self.counter_policy_last_delta = delta
    self.counter_policy_last_elapsed_nanos = elapsed_nanos

    reject_reason = None
    if delta not in self.counter_policy.allowed_deltas:
      reject_reason = "delta"
    elif delta != 1:
      if elapsed_nanos <= 0:
        reject_reason = "non_monotonic_time"
      elif self.counter_policy.max_elapsed_nanos is not None:
        if elapsed_nanos > self.counter_policy.max_elapsed_nanos:
          reject_reason = "timing"
      else:
        assert self.counter_policy.expected_period_nanos is not None
        expected_nanos = delta * self.counter_policy.expected_period_nanos
        if abs(elapsed_nanos - expected_nanos) > self.counter_policy.tolerance_nanos:
          reject_reason = "timing"

    if reject_reason is not None:
      details = (f"previous_counter={self.counter} counter={cur_count} delta={delta} " +
                 f"elapsed_nanos={elapsed_nanos}")
      grace = self.reject_counter_policy(nanos, reject_reason, details)
      # Preserve the established rolling-counter recovery model: a
      # checksum-valid anomaly becomes the next observed baseline, but still
      # accumulates a failure and crosses the sticky threshold at five.
      self.counter = cur_count
      self.counter_nanos = nanos
      return grace

    if self.counter_fail > 0:
      self.counter_fail -= 1
    self.counter_policy_invalid_logged = False
    self.counter = cur_count
    self.counter_nanos = nanos
    self.counter_policy_last_reject_reason = None

    if delta != 1:
      self.counter_policy_accepted_alternate += 1
      self.counter_policy_log(nanos, "accept_alternate",
                              f"counter={cur_count} delta={delta} elapsed_nanos={elapsed_nanos} " +
                              f"accepted_alternate={self.counter_policy_accepted_alternate}")
    return True

  def valid(self, current_nanos: int, bus_timeout: bool) -> bool:
    if self.ignore_alive:
      return True
    if not self.timestamps:
      return False
    if (current_nanos - self.timestamps[-1]) > self.timeout_threshold:
      return False
    return True


class VLDict(dict):
  def __init__(self, parser):
    super().__init__()
    self.parser = parser

  def __getitem__(self, key):
    if key not in self:
      self.parser._add_message(key)
    return super().__getitem__(key)


class CANParser:
  def __init__(self, dbc_name: str, messages: list[tuple[str | int, int]], bus: int, *,
               counter_policies: Mapping[str | int, CounterPolicy] | None = None):
    self.dbc_name: str = dbc_name
    self.bus: int = bus
    self.dbc = DBC(dbc_name)

    self.vl: dict[int | str, dict[str, float]] = VLDict(self)
    self.vl_all: dict[int | str, dict[str, list[float]]] = {}
    self.ts_nanos: dict[int | str, dict[str, int]] = {}
    self.addresses: set[int] = set()
    self.message_states: dict[int, MessageState] = {}
    self.counter_policies: dict[int, CounterPolicy] = {}

    if counter_policies is not None and not isinstance(counter_policies, Mapping):
      raise TypeError("counter_policies must be a mapping")

    for name_or_addr, policy in counter_policies.items() if counter_policies is not None else ():
      if isinstance(name_or_addr, numbers.Number):
        msg = self.dbc.addr_to_msg.get(int(name_or_addr))
      else:
        msg = self.dbc.name_to_msg.get(name_or_addr)
      if msg is None:
        raise RuntimeError(f"could not find counter policy message {name_or_addr!r} in DBC {dbc_name}")
      if msg.address in self.counter_policies:
        raise RuntimeError(f"duplicate counter policy for message {msg.name}")
      if not isinstance(policy, CounterPolicy):
        raise TypeError(f"counter policy for message {msg.name} must be a CounterPolicy")
      counter_signals = [sig for sig in msg.sigs.values() if sig.type == 1]
      if len(counter_signals) != 1:
        raise RuntimeError(f"counter policy message {msg.name} must have exactly one counter signal")
      if not any(sig.calc_checksum is not None for sig in msg.sigs.values()):
        raise RuntimeError(f"counter policy message {msg.name} must have a checksum signal")
      counter_max = (1 << counter_signals[0].size) - 1
      if any(delta > counter_max for delta in policy.allowed_deltas):
        raise RuntimeError(f"counter policy delta exceeds the counter width for message {msg.name}")
      self.counter_policies[msg.address] = policy

    for name_or_addr, freq in messages:
      if isinstance(name_or_addr, numbers.Number):
        msg = self.dbc.addr_to_msg.get(int(name_or_addr))
      else:
        msg = self.dbc.name_to_msg.get(name_or_addr)
      if msg is None:
        raise RuntimeError(f"could not find message {name_or_addr!r} in DBC {dbc_name}")
      if msg.address in self.addresses:
        raise RuntimeError("Duplicate Message Check: %d" % msg.address)

      self._add_message(name_or_addr, freq)

    self.can_invalid_cnt: int = CAN_INVALID_CNT
    self.last_nonempty_nanos: int = 0
    self._last_update_nanos: int = 0

  def _add_message(self, name_or_addr: str | int, freq: int | None = None) -> None:
    if isinstance(name_or_addr, numbers.Number):
      msg = self.dbc.addr_to_msg.get(int(name_or_addr))
    else:
      msg = self.dbc.name_to_msg.get(name_or_addr)
    assert msg is not None
    assert msg.address not in self.addresses

    self.addresses.add(msg.address)
    signal_names = list(msg.sigs.keys())
    signals_dict = {s: 0.0 for s in signal_names}
    dict.__setitem__(self.vl, msg.address, signals_dict)
    dict.__setitem__(self.vl, msg.name, signals_dict)
    self.vl_all[msg.address] = defaultdict(list)
    self.vl_all[msg.name] = self.vl_all[msg.address]
    self.ts_nanos[msg.address] = {s: 0 for s in signal_names}
    self.ts_nanos[msg.name] = self.ts_nanos[msg.address]

    state = MessageState(
      address=msg.address,
      name=msg.name,
      size=msg.size,
      signals=list(msg.sigs.values()),
      ignore_alive=freq is not None and math.isnan(freq),
      counter_policy=self.counter_policies.get(msg.address),
    )
    if freq is not None and freq > 0:
      state.frequency = freq
    else:
      # if frequency not specified, assume 1Hz until we learn it
      freq = 1
    state.timeout_threshold = (1_000_000_000 / freq) * 10

    self.message_states[msg.address] = state

  @property
  def bus_timeout(self) -> bool:
    ignore_alive = all(s.ignore_alive for s in self.message_states.values())
    bus_timeout_threshold = 500 * 1_000_000
    for st in self.message_states.values():
      if st.timeout_threshold > 0:
        bus_timeout_threshold = min(bus_timeout_threshold, st.timeout_threshold)
    return ((self._last_update_nanos - self.last_nonempty_nanos) > bus_timeout_threshold) and not ignore_alive

  @property
  def can_valid(self) -> bool:
    valid = True
    counters_valid = True
    bus_timeout = self.bus_timeout
    for state in self.message_states.values():
      if state.counter_fail >= MAX_BAD_COUNTER or state.counter_policy_invalid_latched:
        counters_valid = False
        state.rate_limited_log(self._last_update_nanos,
                               f"counter invalid, {state.counter_fail=} latched={state.counter_policy_invalid_latched} {MAX_BAD_COUNTER=}")
      if not state.valid(self._last_update_nanos, bus_timeout):
        valid = False
        state.rate_limited_log(self._last_update_nanos, "not valid (timeout or missing)")

    # TODO: probably only want to increment this once per update() call
    self.can_invalid_cnt = 0 if valid else min(self.can_invalid_cnt + 1, CAN_INVALID_CNT)
    return self.can_invalid_cnt < CAN_INVALID_CNT and counters_valid

  def update(self, strings, sendcan: bool = False):
    if strings and not isinstance(strings[0], list | tuple):
      strings = [strings]

    for addr, state in self.message_states.items():
      state.counter_policy_invalid_latched = False
      for k in self.vl_all[addr]:
        self.vl_all[addr][k].clear()

    updated_addrs: set[int] = set()
    for entry in strings:
      t = entry[0]
      frames = entry[1]
      bus_empty = True
      for address, dat, src in frames:
        if src != self.bus:
          continue
        bus_empty = False
        state = self.message_states.get(address)
        if state is None or len(dat) > 64:
          continue
        parsed = state.parse(t, dat)
        if state.counter_policy is not None and state.counter_fail >= MAX_BAD_COUNTER:
          state.counter_policy_invalid_latched = True
        if parsed:
          updated_addrs.add(address)

          vl_addr = self.vl[address]
          vl_all_addr = self.vl_all[address]
          ts_addr = self.ts_nanos[address]

          for i, sig in enumerate(state.signals):
            vl_addr[sig.name] = state.vals[i]
            vl_all_addr[sig.name] = state.all_vals[i]
            ts_addr[sig.name] = state.timestamps[-1]

      if not bus_empty:
        self.last_nonempty_nanos = t

      self._last_update_nanos = t

    return updated_addrs


class CANDefine:
  def __init__(self, dbc_name: str):
    dbc = DBC(dbc_name)

    dv = defaultdict(dict)
    for val in dbc.vals:
      sgname = val.name
      address = val.address
      msg = dbc.addr_to_msg.get(address)
      if msg is None:
        raise KeyError(address)
      msgname = msg.name
      parts = val.def_val.split()
      values = [int(v) for v in parts[::2]]
      defs = parts[1::2]
      dv[address][sgname] = dict(zip(values, defs, strict=True))
      dv[msgname][sgname] = dv[address][sgname]

    self.dv = dict(dv)
