#include <Wire.h>
#include <Servo.h>
#include <string.h>
#include <stdio.h>
#include "minidrone.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"

int i2cAddress = 0x50;

void setup() {
  // put your setup code here, to run once:
  Wire.begin(i2cAddress);
  // Wire.onReceive(receiveEvent);
  Serial.begin(115200);
  Wire.onRequest(requestEvent);

  Serial.println("Setup complete.");

  minidrone_MinidroneMessage message = minidrone_MinidroneMessage_init_zero;
  // message.QDES.ComputeUnit.temp = 25;

  minidrone_MinidroneMessage_QDES_ComputeUnit computeUnitMsg = minidrone_MinidroneMessage_QDES_ComputeUnit_init_zero;
  computeUnitMsg.temp = 25;

  // Serial.println(message);

}

void requestEvent() {
  uint8_t buffer[128];
  size_t message_length;
  bool status;

  //encode the message
  minidrone_MinidroneMessage message = minidrone_MinidroneMessage_init_zero;

  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  // message.QDES.ComputeUnit.temp = 25;
  // message.minidrone_MinidroneMessage_QDES.minidrone_MinidroneMessage_QDES_ComputeUnit.temp = 25;
  // minidrone_MinidroneMessage_QDES qdes_message = message.minidrone_MinidroneMessage_QDES;

  Serial.println(message.dummy_field);

  // status = pb_encode(&stream, MinidroneMessage_fields, &message);
  // message_length = stream.bytes_written;

  // if (!status) {
  //   printf("Encoding failed: %s\n", PB_GET_ERROR(&stream));
  //   return 1;
  // }

  // Wire.write((char *)&stream, buffer);
}

void loop() {

}
