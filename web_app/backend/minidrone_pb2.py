# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: minidrone.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='minidrone.proto',
  package='minidrone',
  syntax='proto3',
  serialized_options=None,
  serialized_pb=_b('\n\x0fminidrone.proto\x12\tminidrone\"\x13\n\x03TX2\x12\x0c\n\x04temp\x18\x01 \x01(\x02\"K\n\x0b\x43omputeUnit\x12\x0c\n\x04temp\x18\x01 \x01(\x02\x12\x11\n\tfan_speed\x18\x02 \x01(\x05\x12\x1b\n\x03tx2\x18\x03 \x01(\x0b\x32\x0e.minidrone.TX2\"?\n\x0b\x42\x61tteryUnit\x12\x0c\n\x04temp\x18\x01 \x01(\x02\x12\x11\n\tfan_speed\x18\x02 \x01(\x02\x12\x0f\n\x07voltage\x18\x03 \x01(\x02\"`\n\x04QDES\x12+\n\x0b\x63omputeUnit\x18\x01 \x01(\x0b\x32\x16.minidrone.ComputeUnit\x12+\n\x0b\x62\x61tteryUnit\x18\x02 \x01(\x0b\x32\x16.minidrone.BatteryUnit\"\x16\n\x05Servo\x12\r\n\x05\x61ngle\x18\x01 \x01(\x02\"-\n\tLeftWheel\x12\r\n\x05speed\x18\x01 \x01(\x02\x12\x11\n\tdirection\x18\x02 \x01(\t\".\n\nRightWheel\x12\r\n\x05speed\x18\x01 \x01(\x02\x12\x11\n\tdirection\x18\x02 \x01(\t\"R\n\x06Wheels\x12\"\n\x04left\x18\x01 \x01(\x0b\x32\x14.minidrone.LeftWheel\x12$\n\x05right\x18\x02 \x01(\x0b\x32\x15.minidrone.RightWheel\"P\n\nDriveTrain\x12\x1f\n\x05servo\x18\x01 \x01(\x0b\x32\x10.minidrone.Servo\x12!\n\x06wheels\x18\x02 \x01(\x0b\x32\x11.minidrone.Wheelsb\x06proto3')
)




_TX2 = _descriptor.Descriptor(
  name='TX2',
  full_name='minidrone.TX2',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='temp', full_name='minidrone.TX2.temp', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=30,
  serialized_end=49,
)


_COMPUTEUNIT = _descriptor.Descriptor(
  name='ComputeUnit',
  full_name='minidrone.ComputeUnit',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='temp', full_name='minidrone.ComputeUnit.temp', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='fan_speed', full_name='minidrone.ComputeUnit.fan_speed', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='tx2', full_name='minidrone.ComputeUnit.tx2', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=51,
  serialized_end=126,
)


_BATTERYUNIT = _descriptor.Descriptor(
  name='BatteryUnit',
  full_name='minidrone.BatteryUnit',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='temp', full_name='minidrone.BatteryUnit.temp', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='fan_speed', full_name='minidrone.BatteryUnit.fan_speed', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='voltage', full_name='minidrone.BatteryUnit.voltage', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=128,
  serialized_end=191,
)


_QDES = _descriptor.Descriptor(
  name='QDES',
  full_name='minidrone.QDES',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='computeUnit', full_name='minidrone.QDES.computeUnit', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='batteryUnit', full_name='minidrone.QDES.batteryUnit', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=193,
  serialized_end=289,
)


_SERVO = _descriptor.Descriptor(
  name='Servo',
  full_name='minidrone.Servo',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='angle', full_name='minidrone.Servo.angle', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=291,
  serialized_end=313,
)


_LEFTWHEEL = _descriptor.Descriptor(
  name='LeftWheel',
  full_name='minidrone.LeftWheel',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='speed', full_name='minidrone.LeftWheel.speed', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='direction', full_name='minidrone.LeftWheel.direction', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=315,
  serialized_end=360,
)


_RIGHTWHEEL = _descriptor.Descriptor(
  name='RightWheel',
  full_name='minidrone.RightWheel',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='speed', full_name='minidrone.RightWheel.speed', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='direction', full_name='minidrone.RightWheel.direction', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=362,
  serialized_end=408,
)


_WHEELS = _descriptor.Descriptor(
  name='Wheels',
  full_name='minidrone.Wheels',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='left', full_name='minidrone.Wheels.left', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='right', full_name='minidrone.Wheels.right', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=410,
  serialized_end=492,
)


_DRIVETRAIN = _descriptor.Descriptor(
  name='DriveTrain',
  full_name='minidrone.DriveTrain',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='servo', full_name='minidrone.DriveTrain.servo', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='wheels', full_name='minidrone.DriveTrain.wheels', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=494,
  serialized_end=574,
)

_COMPUTEUNIT.fields_by_name['tx2'].message_type = _TX2
_QDES.fields_by_name['computeUnit'].message_type = _COMPUTEUNIT
_QDES.fields_by_name['batteryUnit'].message_type = _BATTERYUNIT
_WHEELS.fields_by_name['left'].message_type = _LEFTWHEEL
_WHEELS.fields_by_name['right'].message_type = _RIGHTWHEEL
_DRIVETRAIN.fields_by_name['servo'].message_type = _SERVO
_DRIVETRAIN.fields_by_name['wheels'].message_type = _WHEELS
DESCRIPTOR.message_types_by_name['TX2'] = _TX2
DESCRIPTOR.message_types_by_name['ComputeUnit'] = _COMPUTEUNIT
DESCRIPTOR.message_types_by_name['BatteryUnit'] = _BATTERYUNIT
DESCRIPTOR.message_types_by_name['QDES'] = _QDES
DESCRIPTOR.message_types_by_name['Servo'] = _SERVO
DESCRIPTOR.message_types_by_name['LeftWheel'] = _LEFTWHEEL
DESCRIPTOR.message_types_by_name['RightWheel'] = _RIGHTWHEEL
DESCRIPTOR.message_types_by_name['Wheels'] = _WHEELS
DESCRIPTOR.message_types_by_name['DriveTrain'] = _DRIVETRAIN
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

TX2 = _reflection.GeneratedProtocolMessageType('TX2', (_message.Message,), dict(
  DESCRIPTOR = _TX2,
  __module__ = 'minidrone_pb2'
  # @@protoc_insertion_point(class_scope:minidrone.TX2)
  ))
_sym_db.RegisterMessage(TX2)

ComputeUnit = _reflection.GeneratedProtocolMessageType('ComputeUnit', (_message.Message,), dict(
  DESCRIPTOR = _COMPUTEUNIT,
  __module__ = 'minidrone_pb2'
  # @@protoc_insertion_point(class_scope:minidrone.ComputeUnit)
  ))
_sym_db.RegisterMessage(ComputeUnit)

BatteryUnit = _reflection.GeneratedProtocolMessageType('BatteryUnit', (_message.Message,), dict(
  DESCRIPTOR = _BATTERYUNIT,
  __module__ = 'minidrone_pb2'
  # @@protoc_insertion_point(class_scope:minidrone.BatteryUnit)
  ))
_sym_db.RegisterMessage(BatteryUnit)

QDES = _reflection.GeneratedProtocolMessageType('QDES', (_message.Message,), dict(
  DESCRIPTOR = _QDES,
  __module__ = 'minidrone_pb2'
  # @@protoc_insertion_point(class_scope:minidrone.QDES)
  ))
_sym_db.RegisterMessage(QDES)

Servo = _reflection.GeneratedProtocolMessageType('Servo', (_message.Message,), dict(
  DESCRIPTOR = _SERVO,
  __module__ = 'minidrone_pb2'
  # @@protoc_insertion_point(class_scope:minidrone.Servo)
  ))
_sym_db.RegisterMessage(Servo)

LeftWheel = _reflection.GeneratedProtocolMessageType('LeftWheel', (_message.Message,), dict(
  DESCRIPTOR = _LEFTWHEEL,
  __module__ = 'minidrone_pb2'
  # @@protoc_insertion_point(class_scope:minidrone.LeftWheel)
  ))
_sym_db.RegisterMessage(LeftWheel)

RightWheel = _reflection.GeneratedProtocolMessageType('RightWheel', (_message.Message,), dict(
  DESCRIPTOR = _RIGHTWHEEL,
  __module__ = 'minidrone_pb2'
  # @@protoc_insertion_point(class_scope:minidrone.RightWheel)
  ))
_sym_db.RegisterMessage(RightWheel)

Wheels = _reflection.GeneratedProtocolMessageType('Wheels', (_message.Message,), dict(
  DESCRIPTOR = _WHEELS,
  __module__ = 'minidrone_pb2'
  # @@protoc_insertion_point(class_scope:minidrone.Wheels)
  ))
_sym_db.RegisterMessage(Wheels)

DriveTrain = _reflection.GeneratedProtocolMessageType('DriveTrain', (_message.Message,), dict(
  DESCRIPTOR = _DRIVETRAIN,
  __module__ = 'minidrone_pb2'
  # @@protoc_insertion_point(class_scope:minidrone.DriveTrain)
  ))
_sym_db.RegisterMessage(DriveTrain)


# @@protoc_insertion_point(module_scope)