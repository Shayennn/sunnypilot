import unittest
import random

from opendbc.can import CANPacker, CANParser, CounterPolicy
from opendbc.can.parser import CAN_INVALID_CNT
from opendbc.can.tests import TEST_DBC

MAX_BAD_COUNTER = 5


class TestCanParserPacker(unittest.TestCase):
  COUNTER_DBC = "honda_civic_touring_2016_can_generated"
  COUNTER_MSG = "STEERING_CONTROL"
  COUNTER_ADDR = 0xE4
  COUNTER_PERIOD_NANOS = 500_000_000

  @classmethod
  def counter_policy(cls):
    return CounterPolicy(
      name="test_counter_policy",
      allowed_deltas=frozenset({1, 2}),
      expected_period_nanos=cls.COUNTER_PERIOD_NANOS,
      tolerance_nanos=25_000_000,
    )

  @classmethod
  def counter_parser(cls, policy=True, messages=None):
    counter_policies = {cls.COUNTER_MSG: cls.counter_policy()} if policy else None
    return CANParser(cls.COUNTER_DBC, messages if messages is not None else [(cls.COUNTER_MSG, 0)], 0,
                     counter_policies=counter_policies)

  @classmethod
  def counter_msg(cls, counter, steer_torque=0, bad_checksum=False):
    msg = CANPacker(cls.COUNTER_DBC).make_can_msg(cls.COUNTER_MSG, 0, {
      "COUNTER": counter,
      "STEER_TORQUE": steer_torque,
    })
    if bad_checksum:
      dat = bytearray(msg[1])
      dat[4] ^= 0x1
      msg = (msg[0], bytes(dat), msg[2])
    return msg

  def test_packer(self):
    packer = CANPacker(TEST_DBC)

    for b in range(6):
      for i in range(256):
        values = {"COUNTER": i}
        addr, dat, bus = packer.make_can_msg("CAN_FD_MESSAGE", b, values)
        assert addr == 245
        assert bus == b
        assert dat[0] == i

  def test_packer_counter(self):
    msgs = [("CAN_FD_MESSAGE", 0), ]
    packer = CANPacker(TEST_DBC)
    parser = CANParser(TEST_DBC, msgs, 0)

    # packer should increment the counter
    for i in range(1000):
      msg = packer.make_can_msg("CAN_FD_MESSAGE", 0, {})
      parser.update([0, [msg]])
      assert parser.vl["CAN_FD_MESSAGE"]["COUNTER"] == (i % 256)

    # setting COUNTER should override
    for _ in range(100):
      cnt = random.randint(0, 255)
      msg = packer.make_can_msg("CAN_FD_MESSAGE", 0, {
        "COUNTER": cnt,
        "SIGNED": 0
      })
      parser.update([0, [msg]])
      assert parser.vl["CAN_FD_MESSAGE"]["COUNTER"] == cnt

    # then, should resume counting from the override value
    cnt = parser.vl["CAN_FD_MESSAGE"]["COUNTER"]
    for i in range(100):
      msg = packer.make_can_msg("CAN_FD_MESSAGE", 0, {})
      parser.update([0, [msg]])
      assert parser.vl["CAN_FD_MESSAGE"]["COUNTER"] == ((cnt + i) % 256)

  def test_parser_can_valid(self):
    msgs = [("CAN_FD_MESSAGE", 10), ]
    packer = CANPacker(TEST_DBC)
    parser = CANParser(TEST_DBC, msgs, 0)

    # shouldn't be valid initially
    assert not parser.can_valid

    # not valid until the message is seen
    for _ in range(100):
      parser.update([0, []])
      assert not parser.can_valid

    # valid once seen
    for i in range(1, 100):
      t = int(0.01 * i * 1e9)
      msg = packer.make_can_msg("CAN_FD_MESSAGE", 0, {})
      parser.update([t, [msg]])
      assert parser.can_valid

  def test_parser_updated_list(self):
    msgs = [("CAN_FD_MESSAGE", 10), ]
    parser = CANParser(TEST_DBC, msgs, 0)
    packer = CANPacker(TEST_DBC)

    msg = packer.make_can_msg("CAN_FD_MESSAGE", 0, {})
    ret = parser.update([0, [msg]])
    assert ret == {245}

    ret = parser.update([])
    assert len(ret) == 0

  def test_parser_counter_can_valid(self):
    """
    Tests number of allowed bad counters + ensures CAN stays invalid
    while receiving invalid messages + that we can recover
    """
    msgs = [
      ("STEERING_CONTROL", 0),
    ]
    packer = CANPacker("honda_civic_touring_2016_can_generated")
    parser = CANParser("honda_civic_touring_2016_can_generated", msgs, 0)

    msg = packer.make_can_msg("STEERING_CONTROL", 0, {"COUNTER": 0})

    # bad static counter, invalid once it's seen MAX_BAD_COUNTER messages
    for idx in range(0x1000):
      parser.update([0, [msg]])
      assert ((idx + 1) < MAX_BAD_COUNTER) == parser.can_valid

    # one to recover
    msg = packer.make_can_msg("STEERING_CONTROL", 0, {"COUNTER": 1})
    parser.update([0, [msg]])
    assert parser.can_valid

  def test_counter_policy_configuration(self):
    policy = self.counter_policy()
    parser = CANParser(self.COUNTER_DBC, [], 0, counter_policies={self.COUNTER_MSG: policy})

    # Policy configuration must also work with dynamic message registration.
    assert self.COUNTER_ADDR not in parser.message_states
    parser.vl[self.COUNTER_MSG]
    assert parser.message_states[self.COUNTER_ADDR].counter_policy == policy

    parser = CANParser(self.COUNTER_DBC, [], 0, counter_policies={self.COUNTER_ADDR: policy})
    parser.vl[self.COUNTER_ADDR]
    assert parser.message_states[self.COUNTER_ADDR].counter_policy == policy

    with self.assertRaises(RuntimeError):
      CANParser(self.COUNTER_DBC, [], 0, counter_policies={"NOT_A_MESSAGE": policy})
    with self.assertRaises(RuntimeError):
      CANParser(TEST_DBC, [], 0, counter_policies={"Brake_Status": policy})
    with self.assertRaises(RuntimeError):
      CANParser("tesla_model3_party", [], 0, counter_policies={"SCCM_leftStalk": policy})
    with self.assertRaises(RuntimeError):
      wide_policy = CounterPolicy("too_wide", frozenset({1, 4}), self.COUNTER_PERIOD_NANOS)
      CANParser(self.COUNTER_DBC, [], 0, counter_policies={self.COUNTER_MSG: wide_policy})
    with self.assertRaises(RuntimeError):
      CANParser(self.COUNTER_DBC, [], 0, counter_policies={self.COUNTER_MSG: policy, self.COUNTER_ADDR: policy})
    with self.assertRaises(TypeError):
      CANParser(self.COUNTER_DBC, [], 0, counter_policies={self.COUNTER_MSG: object()})
    with self.assertRaises(TypeError):
      CANParser(self.COUNTER_DBC, [], 0, counter_policies=[])

  def test_counter_policy_validation(self):
    invalid_policies = (
      {"name": ""},
      {"name": "test", "allowed_deltas": frozenset()},
      {"name": "test", "allowed_deltas": frozenset({0, 1})},
      {"name": "test", "allowed_deltas": frozenset({1.0})},
      {"name": "test", "allowed_deltas": frozenset({1, 2})},
      {"name": "test", "allowed_deltas": frozenset({1}), "expected_period_nanos": 500_000_000},
      {"name": "test", "expected_period_nanos": 0},
      {"name": "test", "tolerance_nanos": -1},
      {"name": "test", "tolerance_nanos": 1},
    )
    for kwargs in invalid_policies:
      with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
        CounterPolicy(**kwargs)

  def test_counter_policy_accepts_cadence_and_rollover(self):
    sequences = (
      ((0, 1, 3), (1_000_000_000, 1_500_000_000, 2_500_000_000)),
      ((2, 0, 1), (1_000_000_000, 2_000_000_000, 2_500_000_000)),
      ((3, 1), (1_000_000_000, 2_000_000_000)),
    )

    for counters, timestamps in sequences:
      with self.subTest(counters=counters):
        parser = self.counter_parser()
        for counter, nanos in zip(counters, timestamps, strict=True):
          assert parser.update([nanos, [self.counter_msg(counter)]]) == {self.COUNTER_ADDR}
          assert parser.can_valid

        state = parser.message_states[self.COUNTER_ADDR]
        assert state.counter == counters[-1]
        assert state.counter_fail == 0

  def test_counter_policy_preserves_untimed_plus_one(self):
    for elapsed_nanos in (0, 29_510_464, 500_000_000, 1_000_000_000):
      with self.subTest(elapsed_nanos=elapsed_nanos):
        parser = self.counter_parser()
        assert parser.update([1_000_000_000, [self.counter_msg(0)]]) == {self.COUNTER_ADDR}
        assert parser.update([1_000_000_000 + elapsed_nanos, [self.counter_msg(1)]]) == {self.COUNTER_ADDR}
        assert parser.message_states[self.COUNTER_ADDR].counter_fail == 0
        assert parser.can_valid

  def test_counter_policy_rejects_wrong_delta_or_timing(self):
    cases = (
      (0, 1_500_000_000, "delta"),               # repeated
      (3, 2_500_000_000, "delta"),               # +3/backwards on a 2-bit counter
      (2, 1_000_000_000, "non_monotonic_time"),  # +2 with no elapsed time
      (2, 1_500_000_000, "timing"),              # +2 at 0.5 seconds
      (2, 2_026_000_000, "timing"),              # just outside tolerance
    )

    for counter, nanos, reason in cases:
      with self.subTest(counter=counter, nanos=nanos):
        parser = self.counter_parser()
        assert parser.update([1_000_000_000, [self.counter_msg(0, 100)]]) == {self.COUNTER_ADDR}
        # Preserve legacy grace and rephase semantics for checksum-valid values.
        assert parser.update([nanos, [self.counter_msg(counter, 200)]]) == {self.COUNTER_ADDR}

        state = parser.message_states[self.COUNTER_ADDR]
        assert state.counter == counter
        assert state.counter_fail == 1
        assert state.counter_policy_last_reject_reason == reason
        assert list(state.timestamps) == [1_000_000_000, nanos]
        assert parser.vl[self.COUNTER_MSG]["STEER_TORQUE"] == 200
        assert parser.can_valid

  def test_counter_policy_timing_tolerance_is_inclusive(self):
    for elapsed_nanos in (975_000_000, 1_025_000_000):
      with self.subTest(elapsed_nanos=elapsed_nanos):
        parser = self.counter_parser()
        parser.update([1_000_000_000, [self.counter_msg(0)]])
        assert parser.update([1_000_000_000 + elapsed_nanos, [self.counter_msg(2)]]) == {self.COUNTER_ADDR}
        assert parser.message_states[self.COUNTER_ADDR].counter_fail == 0

  def test_counter_policy_checksum_first_and_recovery(self):
    parser = self.counter_parser()
    state = parser.message_states[self.COUNTER_ADDR]

    # A checksum-invalid first frame cannot seed trusted state.
    assert parser.update([500_000_000, [self.counter_msg(3, 50, bad_checksum=True)]]) == set()
    assert not state.counter_initialized
    assert state.counter_fail == 0
    assert not state.timestamps

    assert parser.update([1_000_000_000, [self.counter_msg(3, 100)]]) == {self.COUNTER_ADDR}
    assert state.counter_initialized

    # A bad checksum cannot rephase state or update values.
    assert parser.update([1_500_000_000, [self.counter_msg(0, 200, bad_checksum=True)]]) == set()
    assert state.counter == 3
    assert state.counter_fail == 0
    assert parser.vl[self.COUNTER_MSG]["STEER_TORQUE"] == 100

    # A valid-checksum timing rejection preserves legacy decoded-value grace
    # and becomes the observed baseline for bounded recovery.
    assert parser.update([1_700_000_000, [self.counter_msg(1, 300)]]) == {self.COUNTER_ADDR}
    assert state.counter == 1
    assert state.counter_fail == 1
    assert list(state.timestamps) == [1_000_000_000, 1_700_000_000]
    assert parser.vl[self.COUNTER_MSG]["STEER_TORQUE"] == 300

    # Relative to the last checksum-valid frame, +2 at one second is valid and heals one failure.
    assert parser.update([2_700_000_000, [self.counter_msg(3, 400)]]) == {self.COUNTER_ADDR}
    assert state.counter == 3
    assert state.counter_fail == 0
    assert state.counter_policy_accepted_alternate == 1
    assert parser.vl[self.COUNTER_MSG]["STEER_TORQUE"] == 400

  def test_counter_policy_rejects_wrong_message_size(self):
    policy = self.counter_policy()
    parser = CANParser("tesla_model3_party", [], 2, counter_policies={"DAS_settings": policy})
    parser.vl["DAS_settings"]
    state = parser.message_states[0x293]

    # This seven-byte checksum collision seeded the policy before exact DLC validation.
    short_msg = (0x293, bytes.fromhex("6b000000000000"), 2)
    assert parser.update([1_000_000_000, [short_msg]]) == set()
    assert not state.counter_initialized
    assert state.counter_fail == 1
    assert state.counter_policy_last_reject_reason == "size"
    assert not state.timestamps

    valid_msg = CANPacker("tesla_model3_party").make_can_msg("DAS_settings", 2, {"DAS_settingCounter": 0})
    long_msg = (valid_msg[0], valid_msg[1] + b"\x00", valid_msg[2])
    assert parser.update([1_500_000_000, [long_msg]]) == set()
    assert not state.counter_initialized
    assert state.counter_fail == 2
    assert not state.timestamps

  def test_counter_policy_ignore_counter(self):
    parser = self.counter_parser()
    state = parser.message_states[self.COUNTER_ADDR]
    state.ignore_counter = True

    assert parser.update([1_000_000_000, [self.counter_msg(3)]]) == {self.COUNTER_ADDR}
    assert not state.counter_initialized
    assert state.counter_fail == 0

  def test_counter_policy_threshold_is_sticky_within_update(self):
    parser = self.counter_parser()
    state = parser.message_states[self.COUNTER_ADDR]
    parser.update([1_000_000_000, [self.counter_msg(0)]])

    # The fifth rejection crosses the threshold. A later +1 in the same CAN
    # event can heal the numeric failure count, but not hide that crossing.
    frames = [self.counter_msg(0) for _ in range(MAX_BAD_COUNTER)]
    frames.append(self.counter_msg(1))
    parser.update([1_500_000_000, frames])
    assert state.counter_fail == MAX_BAD_COUNTER - 1
    assert state.counter_policy_invalid_latched
    assert not parser.can_valid

    parser.update([1_600_000_000, [self.counter_msg(2)]])
    assert not state.counter_policy_invalid_latched
    assert state.counter_fail == MAX_BAD_COUNTER - 2
    assert parser.can_valid

  def test_counter_policy_timeout_still_invalidates(self):
    parser = self.counter_parser()
    parser.update([1_000_000_000, [self.counter_msg(0)]])
    assert parser.can_valid

    parser.update([12_000_000_000, []])
    for _ in range(CAN_INVALID_CNT - 1):
      assert parser.can_valid
    assert not parser.can_valid

  def test_default_counter_policy_remains_strict(self):
    parser = self.counter_parser(policy=False)

    assert parser.update([1_000_000_000, [self.counter_msg(1)]]) == {self.COUNTER_ADDR}
    for i, counter in enumerate((3, 1, 3, 1, 3), start=1):
      parser.update([1_000_000_000 + i * 1_000_000_000, [self.counter_msg(counter)]])

    state = parser.message_states[self.COUNTER_ADDR]
    assert state.counter_fail == MAX_BAD_COUNTER
    assert not parser.can_valid

  def test_parser_no_partial_update(self):
    """
    Ensure that the CANParser doesn't partially update messages with invalid signals (COUNTER/CHECKSUM).
    Previously, the signal update loop would only break once it got to one of these invalid signals,
    after already updating most/all of the signals.
    """
    msgs = [
      ("STEERING_CONTROL", 0),
    ]
    packer = CANPacker("honda_civic_touring_2016_can_generated")
    parser = CANParser("honda_civic_touring_2016_can_generated", msgs, 0)

    def rx_steering_msg(values, bad_checksum=False):
      msg = packer.make_can_msg("STEERING_CONTROL", 0, values)
      if bad_checksum:
        # add 1 to checksum
        dat = bytearray(msg[1])
        dat[4] = (dat[4] & 0xF0) | ((dat[4] & 0x0F) + 1)
        msg = (msg[0], bytes(dat), msg[2])

      parser.update([0, [msg]])

    rx_steering_msg({"STEER_TORQUE": 100}, bad_checksum=False)
    assert parser.vl["STEERING_CONTROL"]["STEER_TORQUE"] == 100
    assert parser.vl_all["STEERING_CONTROL"]["STEER_TORQUE"] == [100]

    for _ in range(5):
      rx_steering_msg({"STEER_TORQUE": 200}, bad_checksum=True)
      assert parser.vl["STEERING_CONTROL"]["STEER_TORQUE"] == 100
      assert parser.vl_all["STEERING_CONTROL"]["STEER_TORQUE"] == []

    # Even if CANParser doesn't update instantaneous vl, make sure it didn't add invalid values to vl_all
    rx_steering_msg({"STEER_TORQUE": 300}, bad_checksum=False)
    assert parser.vl["STEERING_CONTROL"]["STEER_TORQUE"] == 300
    assert parser.vl_all["STEERING_CONTROL"]["STEER_TORQUE"] == [300]

  def test_packer_parser(self):
    msgs = [
      ("Brake_Status", 0),
      ("CAN_FD_MESSAGE", 0),
      ("STEERING_CONTROL", 0),
    ]
    packer = CANPacker(TEST_DBC)
    parser = CANParser(TEST_DBC, msgs, 0)

    for steer in range(-256, 255):
      for active in (1, 0):
        values = {
          "STEERING_CONTROL": {
            "STEER_TORQUE": steer,
            "STEER_TORQUE_REQUEST": active,
          },
          "Brake_Status": {
            "Signal1": 61042322657536.0,
          },
          "CAN_FD_MESSAGE": {
            "SIGNED": steer,
            "64_BIT_LE": random.randint(0, 100),
            "64_BIT_BE": random.randint(0, 100),
          },
        }

        msgs = [packer.make_can_msg(k, 0, v) for k, v in values.items()]
        parser.update([0, msgs])

        for k, v in values.items():
          for key, val in v.items():
            self.assertAlmostEqual(parser.vl[k][key], val)

        # also check address
        for sig in ("STEER_TORQUE", "STEER_TORQUE_REQUEST", "COUNTER", "CHECKSUM"):
          assert parser.vl["STEERING_CONTROL"][sig] == parser.vl[228][sig]

  def test_scale_offset(self):
    """Test that both scale and offset are correctly preserved"""
    dbc_file = "honda_civic_touring_2016_can_generated"
    msgs = [("VSA_STATUS", 50)]
    parser = CANParser(dbc_file, msgs, 0)
    packer = CANPacker(dbc_file)

    for brake in range(100):
      values = {"USER_BRAKE": brake}
      msgs = packer.make_can_msg("VSA_STATUS", 0, values)
      parser.update([0, [msgs]])

      self.assertAlmostEqual(parser.vl["VSA_STATUS"]["USER_BRAKE"], brake)

  def test_subaru(self):
    # Subaru is little endian

    dbc_file = "subaru_global_2017_generated"

    msgs = [("ES_LKAS", 50)]

    parser = CANParser(dbc_file, msgs, 0)
    packer = CANPacker(dbc_file)

    idx = 0
    for steer in range(-256, 255):
      for active in [1, 0]:
        values = {
          "LKAS_Output": steer,
          "LKAS_Request": active,
          "SET_1": 1
        }

        msgs = packer.make_can_msg("ES_LKAS", 0, values)
        parser.update([0, [msgs]])

        self.assertAlmostEqual(parser.vl["ES_LKAS"]["LKAS_Output"], steer)
        self.assertAlmostEqual(parser.vl["ES_LKAS"]["LKAS_Request"], active)
        self.assertAlmostEqual(parser.vl["ES_LKAS"]["SET_1"], 1)
        self.assertAlmostEqual(parser.vl["ES_LKAS"]["COUNTER"], idx % 16)
        idx += 1

  def test_bus_timeout(self):
    """Test CAN bus timeout detection"""
    dbc_file = "honda_civic_touring_2016_can_generated"

    freq = 100
    msgs = [("VSA_STATUS", freq), ("STEER_MOTOR_TORQUE", freq/2)]

    parser = CANParser(dbc_file, msgs, 0)
    packer = CANPacker(dbc_file)

    i = 0

    def send_msg(blank=False):
      nonlocal i
      i += 1
      t = i*((1 / freq) * 1e9)

      if blank:
        msgs = []
      else:
        msgs = [packer.make_can_msg("VSA_STATUS", 0, {}), ]

      parser.update([t, msgs])

    # all good, no timeout
    for _ in range(1000):
      send_msg()
      assert not parser.bus_timeout, str(_)

    # timeout after 10 blank msgs
    for n in range(200):
      send_msg(blank=True)
      assert (n >= 10) == parser.bus_timeout

    # no timeout immediately after seen again
    send_msg()
    assert not parser.bus_timeout

  def test_updated(self):
    """Test updated value dict"""
    dbc_file = "honda_civic_touring_2016_can_generated"
    msgs = [("VSA_STATUS", 50)]
    parser = CANParser(dbc_file, msgs, 0)
    packer = CANPacker(dbc_file)

    # Make sure nothing is updated
    assert len(parser.vl_all["VSA_STATUS"]["USER_BRAKE"]) == 0

    idx = 0
    for _ in range(10):
      # Ensure CANParser holds the values of any duplicate messages over multiple frames
      user_brake_vals = [random.randrange(100) for _ in range(random.randrange(5, 10))]
      half_idx = len(user_brake_vals) // 2
      can_msgs = [[], []]
      for frame, brake_vals in enumerate((user_brake_vals[:half_idx], user_brake_vals[half_idx:])):
        for user_brake in brake_vals:
          values = {"USER_BRAKE": user_brake}
          can_msgs[frame].append(packer.make_can_msg("VSA_STATUS", 0, values))
          idx += 1

      parser.update([[0, m] for m in can_msgs])
      vl_all = parser.vl_all["VSA_STATUS"]["USER_BRAKE"]

      assert vl_all == user_brake_vals
      if len(user_brake_vals):
        assert vl_all[-1] == parser.vl["VSA_STATUS"]["USER_BRAKE"]

  def test_timestamp_nanos(self):
    """Test message timestamp dict"""
    dbc_file = "honda_civic_touring_2016_can_generated"

    msgs = [
      ("VSA_STATUS", 50),
      ("POWERTRAIN_DATA", 100),
    ]

    parser = CANParser(dbc_file, msgs, 0)
    packer = CANPacker(dbc_file)

    # Check the default timestamp is zero
    for msg in ("VSA_STATUS", "POWERTRAIN_DATA"):
      ts_nanos = parser.ts_nanos[msg].values()
      assert set(ts_nanos) == {0}

    # Check:
    # - timestamp is only updated for correct messages
    # - timestamp is correct for multiple runs
    # - timestamp is from the latest message if updating multiple strings
    for _ in range(10):
      can_strings = []
      log_mono_time = 0
      for i in range(10):
        log_mono_time = int(0.01 * i * 1e+9)
        can_msg = packer.make_can_msg("VSA_STATUS", 0, {})
        can_strings.append((log_mono_time, [can_msg]))
      parser.update(can_strings)

      ts_nanos = parser.ts_nanos["VSA_STATUS"].values()
      assert set(ts_nanos) == {log_mono_time}
      ts_nanos = parser.ts_nanos["POWERTRAIN_DATA"].values()
      assert set(ts_nanos) == {0}

  def test_nonexistent_messages(self):
    # Ensure we don't allow messages not in the DBC
    existing_messages = ("STEERING_CONTROL", 228, "CAN_FD_MESSAGE", 245)

    for msg in existing_messages:
      CANParser(TEST_DBC, [(msg, 0)], 0)
      with self.assertRaises(RuntimeError):
        new_msg = msg + "1" if isinstance(msg, str) else msg + 1
        CANParser(TEST_DBC, [(new_msg, 0)], 0)

  def test_track_all_signals(self):
    parser = CANParser("toyota_nodsu_pt_generated", [("ACC_CONTROL", 0)], 0)
    assert parser.vl["ACC_CONTROL"] == {
      "ACCEL_CMD": 0,
      "ALLOW_LONG_PRESS": 0,
      "ACC_MALFUNCTION": 0,
      "RADAR_DIRTY": 0,
      "DISTANCE": 0,
      "MINI_CAR": 0,
      "ACC_TYPE": 0,
      "CANCEL_REQ": 0,
      "ACC_CUT_IN": 0,
      "LEAD_VEHICLE_STOPPED": 0,
      "PERMIT_BRAKING": 0,
      "RELEASE_STANDSTILL": 0,
      "ITS_CONNECT_LEAD": 0,
      "ACCEL_CMD_ALT": 0,
      "CHECKSUM": 0,
    }

  def test_disallow_duplicate_messages(self):
    CANParser("toyota_nodsu_pt_generated", [("ACC_CONTROL", 5)], 0)

    with self.assertRaises(RuntimeError):
      CANParser("toyota_nodsu_pt_generated", [("ACC_CONTROL", 5), ("ACC_CONTROL", 10)], 0)

    with self.assertRaises(RuntimeError):
      CANParser("toyota_nodsu_pt_generated", [("ACC_CONTROL", 10), ("ACC_CONTROL", 10)], 0)

  def test_allow_undefined_msgs(self):
    # TODO: we should throw an exception for these, but we need good
    #  discovery tests in openpilot first
    packer = CANPacker("toyota_nodsu_pt_generated")

    assert packer.make_can_msg("ACC_CONTROL", 0, {"UNKNOWN_SIGNAL": 0}) == (835, b'\x00\x00\x00\x00\x00\x00\x00N', 0)
    assert packer.make_can_msg("UNKNOWN_MESSAGE", 0, {"UNKNOWN_SIGNAL": 0}) == (0, b'', 0)
    assert packer.make_can_msg(0, 0, {"UNKNOWN_SIGNAL": 0}) == (0, b'', 0)
