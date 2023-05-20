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
int FAN_SPEED = 20;
int * fanPtr = &FAN_SPEED;
float TEMP = 30;
float * tempPtr = &TEMP;

void setup() {
  Serial.begin(115200);

  if (sendOveri2c) {
    Wire.begin(i2cAddress);
    // Wire.begin();

    Serial.println("Sending over i2c...");
    Wire.onRequest(requestEvent);
  } else {
    // Wire.onReceive(receiveEvent);
  //   Serial.println("NOT sending over i2c");

  //   Serial.println("Setup complete.");

  //   //ENCODING

  //   minidrone_MinidroneMessage message = minidrone_MinidroneMessage_init_zero;
  //   minidrone_Test testmessage = minidrone_Test_init_zero;

  //   message.has_test = true;
  //   testmessage.value = 123;
  //   message.test = testmessage;

  //   // if (message.has_test) {
  //   //   Serial.println("Has field");
  //   //   message.test = testmessage;
  //   // } else {
  //   //   Serial.println("does not have field");
  //   // }
    
  //   pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
  //   // status = pb_encode_ex(&stream, minidrone_MinidroneMessage_fields, &message, 0);
  //   status = pb_encode(&stream, minidrone_MinidroneMessage_fields, &message);
  //   message_length = stream.bytes_written;

  //   if (!status) {
  //     Serial.println("Failed to encode");
  //     return;
  //   }

  //   Serial.print("Message length: ");
  //   Serial.println(stream.bytes_written);

  //   Serial.print("Message: ");
  //   char print_buffer[40];
  //   for (int i = 0; i < stream.bytes_written; i++) {
  //     sprintf(print_buffer, "%02X", buffer[i]);
  //     Serial.print(print_buffer);
  //   }
   }

  //DECODING
  // minidrone_MinidroneMessage message = minidrone_MinidroneMessage_init_zero;

  // pb_istream_t stream = pb_istream_from_buffer(buffer, );
  // // pb_istream_t stream = 

  // status = pb_decode(&stream, minidrone_MinidroneMessage_fields, &message);

  // if (!status) {
  //   Serial.println("Decoding failed");
  //   Serial.println(PB_GET_ERROR(&stream));
  //   return 1;
  // }

  // Serial.println("\nDECODING: Message: ");
  // Serial.println(message.test.value);


}

// void encode() {
//   uint8_t buffer[128];
//   size_t message_length;
//   bool status;

//   minidrone_MinidroneMessage message = minidrone_MinidroneMessage_init_zero;
//   minidrone_Test testmessage = minidrone_Test_init_zero;

//   message.has_test = true;
//   testmessage.value = 123;
//   message.test = testmessage;
  
//   pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
//   status = pb_encode(&stream, minidrone_MinidroneMessage_fields, &message); //encodes to buffer

//   if (!status) {
//     Serial.println("Failed to encode");
//     return;
//   }

//   Serial.print("Message length: ");
//   Serial.println(stream.bytes_written);

//   Serial.print("Message: ");
//   char print_buffer[40];
//   for (int i = 0; i < stream.bytes_written; i++) {
//     sprintf(print_buffer, "%02X", buffer[i]);
//     Serial.print(print_buffer);
//   }
// }



void requestEvent() {
  Serial.println("\nREQUEST EVENT");
  uint8_t buffer[128];
  size_t message_length;
  bool status;

  // minidrone_MinidroneMessage message = minidrone_MinidroneMessage_init_zero;
  // minidrone_Test testmessage = minidrone_Test_init_zero;
  minidrone_QDES msg_QDES = minidrone_QDES_init_zero;
  minidrone_ComputeUnit msg_ComputeUnit = minidrone_ComputeUnit_init_zero;

  msg_ComputeUnit.fan_speed = *(fanPtr);
  msg_ComputeUnit.temp = *tempPtr;

  msg_QDES.has_computeUnit = true;
  msg_QDES.computeUnit = msg_ComputeUnit;

  // minidrone_MinidroneMessage_QDES QDES = minidrone_MinidroneMessage_QDES_init_zero;
  // minidrone_MinidroneMessage_QDES_ComputeUnit computeUnit = minidrone_MinidroneMessage_QDES_ComputeUnit_init_zero;
  // message.has_test = true;
  // testmessage.value = 69;
  // message.test = testmessage;
  // message.has_QDES = true;
  // computeUnit.temp = 20;
  // computeUnit.fan_speed = 30;
  // QDES.computeunit = computeUnit;
  // message. = QDES;

  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
  // status = pb_encode(&stream, minidrone_MinidroneMessage_fields, &message);
  status = pb_encode(&stream, minidrone_QDES_fields, &msg_QDES);
  
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

  int msg_len = stream.bytes_written;

  //send the message over i2c
  // Wire.write((char *) &buffer, stream.bytes_written);
  // Wire.beginTransmission(i2cAddress);
  Wire.write((char *) &buffer, stream.bytes_written);
  // if (Wire.endTransmission(true) != 0) {
  //   Serial.println("WIRE ERROR");
  // }


    //DECODING
  minidrone_QDES message = minidrone_QDES_init_zero;
  // minidrone_ComputeUnit message = minidrone_ComputeUnit_init_zero;

  pb_istream_t stream2 = pb_istream_from_buffer(buffer, msg_len);
  // pb_istream_t stream = 

  status = pb_decode(&stream2, minidrone_QDES_fields, &message);

  if (!status) {
    Serial.println("Decoding failed");
    Serial.println(PB_GET_ERROR(&stream2));
    return 1;
  }

  Serial.println("\nDECODING: Message: ");
  Serial.println(message.computeUnit.fan_speed);
  Serial.println(message.computeUnit.temp);
  // Serial.println(message.computeUnit.temp);

}

void loop() {
  
  FAN_SPEED += 1;
  TEMP += 0.5;
  // Serial.print("FAN SPEED: ");
  // Serial.println(FAN_SPEED);
  delay(500);
}
