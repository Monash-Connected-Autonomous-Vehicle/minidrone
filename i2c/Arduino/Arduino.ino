#include <Wire.h>
#include <Servo.h>
#include <string.h>
#include <stdio.h>
#include "minidrone.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "pb_common.h"
#include "pb.h"

int i2cAddress = 0x50;

bool sendOveri2c = true;

void setup() {
  Serial.println("---------------------------------------------");

  uint8_t buffer[128];
  size_t message_length;
  bool status;

  if (sendOveri2c) {
    Wire.begin(i2cAddress);
    Serial.begin(115200);

    Serial.println("Sending over i2c...");
    Wire.onRequest(requestEvent);
  } else {
    // Wire.onReceive(receiveEvent);
    Serial.println("NOT sending over i2c");

    Serial.println("Setup complete.");

    minidrone_MinidroneMessage message = minidrone_MinidroneMessage_init_zero;
    minidrone_Test testmessage = minidrone_Test_init_zero;

    message.has_test = true;
    testmessage.value = 123;
    message.test = testmessage;

    // if (message.has_test) {
    //   Serial.println("Has field");
    //   message.test = testmessage;
    // } else {
    //   Serial.println("does not have field");
    // }
    
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    status = pb_encode(&stream, minidrone_MinidroneMessage_fields, &message);

    if (!status) {
      Serial.println("Failed to encode");
      return;
    }

    Serial.print("Message length: ");
    Serial.println(stream.bytes_written);

    Serial.print("Message: ");
    char print_buffer[40];
    for (int i = 0; i < stream.bytes_written; i++) {
      sprintf(print_buffer, "%02X", buffer[i]);
      Serial.print(print_buffer);
    }
  }



}

void requestEvent() {
  uint8_t buffer[128];
  size_t message_length;
  bool status;

  minidrone_MinidroneMessage message = minidrone_MinidroneMessage_init_zero;
  minidrone_Test testmessage = minidrone_Test_init_zero;

  message.has_test = true;
  testmessage.value = 123;
  message.test = testmessage;
  
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
  status = pb_encode(&stream, minidrone_MinidroneMessage_fields, &message);

  if (!status) {
    Serial.println("Failed to encode");
    return;
  }

  Serial.print("Message length: ");
  Serial.println(stream.bytes_written);

  Serial.print("Message: ");
  char print_buffer[40];
  for (int i = 0; i < stream.bytes_written; i++) {
    sprintf(print_buffer, "%02X", buffer[i]);
    Serial.print(print_buffer);
  }

  //send the message over i2c
  Wire.write((char *) &stream);

}

void loop() {

}
