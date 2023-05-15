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
}

void requestEvent() {
  uint8_t buffer[128];
  size_t message_length;
  bool status;

  //encode the message
  MinidroneMessage message = MinidroneMessage_init_zero;

  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  message.QDES.ComputeUnit.temp = 25;

  status = pb_encode(&stream, MinidroneMessage_fields, &message);
  message_length = stream.bytes_written;

  if (!status) {
    printf("Encoding failed: %s\n", PB_GET_ERROR(&stream));
    return 1;
  }

  Wire.write((char *)&stream, buffer);
}
