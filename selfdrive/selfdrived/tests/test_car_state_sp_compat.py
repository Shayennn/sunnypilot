from dataclasses import asdict

from cereal import custom
from opendbc.car import structs


def test_deprecated_personality_request_fields_preserve_wire_compatibility():
  assert custom.CarStateSP.schema.fields["longitudinalPersonalityRequestDEPRECATED"].proto.ordinal.explicit == 1
  assert custom.CarStateSP.schema.fields["longitudinalPersonalityRequestValidDEPRECATED"].proto.ordinal.explicit == 2

  state = structs.CarStateSP(
    speedLimit=12.5,
    longitudinalPersonalityRequestDEPRECATED=2,
    longitudinalPersonalityRequestValidDEPRECATED=True,
  )
  encoded = custom.CarStateSP.new_message(**asdict(state)).to_bytes()

  with custom.CarStateSP.from_bytes(encoded) as decoded:
    assert decoded.speedLimit == 12.5
    assert decoded.longitudinalPersonalityRequestDEPRECATED == 2
    assert decoded.longitudinalPersonalityRequestValidDEPRECATED

  legacy = custom.CarStateSP.new_message(speedLimit=4.0)
  assert legacy.longitudinalPersonalityRequestDEPRECATED == 0
  assert not legacy.longitudinalPersonalityRequestValidDEPRECATED
