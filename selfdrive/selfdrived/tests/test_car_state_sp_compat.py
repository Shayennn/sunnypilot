from cereal import custom
from opendbc.car import structs
from openpilot.selfdrive.car.helpers import convert_to_capnp


def test_deprecated_personality_request_fields_preserve_wire_compatibility():
  assert custom.CarStateSP.schema.fields["longitudinalPersonalityRequestDEPRECATED"].proto.ordinal.explicit == 1
  assert custom.CarStateSP.schema.fields["longitudinalPersonalityRequestValidDEPRECATED"].proto.ordinal.explicit == 2

  state = custom.CarStateSP.new_message(
    speedLimit=12.5,
    longitudinalPersonalityRequestDEPRECATED=2,
    longitudinalPersonalityRequestValidDEPRECATED=True,
  )
  encoded = state.to_bytes()

  with custom.CarStateSP.from_bytes(encoded) as decoded:
    assert decoded.speedLimit == 12.5
    assert decoded.longitudinalPersonalityRequestDEPRECATED == 2
    assert decoded.longitudinalPersonalityRequestValidDEPRECATED

  current = convert_to_capnp(structs.CarStateSP(speedLimit=4.0))
  assert current.longitudinalPersonalityRequestDEPRECATED == 0
  assert not current.longitudinalPersonalityRequestValidDEPRECATED
