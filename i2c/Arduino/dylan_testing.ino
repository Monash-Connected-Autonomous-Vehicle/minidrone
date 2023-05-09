#include <Wire.h>
#include <Servo.h>
#include <string.h>
//#include <cstring.h>

// Configure settings here
// Wait time for reverse (ms)
int REV_WAIT = 4000;
// Wait time for forward (ms)
int FWD_WAIT = 3000;
// Arduinos i2C address  
int i2cAddress = 0x50;
//Make sure the python program follows the rules and sends char arrays that are 20 values long, arduino will overflow if the lengths are mismatched and may crash if too high
const unsigned int MAX_MESSAGE_LENGTH = 20;
//Starting PWM values, these are floats in order to perform very small iterations at the milisecond level. these are rounded out at output.
float leftOut = 1500;
int leftInInt = 1500;
float lDeltaMax = 0;
float rightOut = 1500;
int rightInInt = 1500;
float rDeltaMax = 0;
//Max time (ms) elapsed without a command issued before the TX2 is presumed dead and motors are shut down
int msgTimeout = 1000; 
//Keeps track of time between incoming messages, dont change this unless needed for something else idk
long int LastMsg = 0; 
//NOTE: Configure acceleration settings in respective function
Servo motorLeft;
Servo motorRight;
Servo servoSteering;



//struct {
//  float QDES_temp; //4bytes
//  char drive_mode[2]; //2bytes
//  float wheel_speed[2]; //8bytes
//} data_dump;


struct {
  char wheel_speed[2]; //8bytes
} data_dump;



void setup() {
  // put your setup code here, to run once:
Wire.begin(i2cAddress);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  //Wire.onRequest(sendData);
  Serial.begin(115200);
  motorLeft.attach(9);
  motorRight.attach(10);
  motorRight.write(1500);
  motorLeft.write(1500);

  Serial.print("startup complete");
}
// Maximum PWM change per milisecond calculator function based on speed and other factors to be added
float maxChangeInPWM(int current,int goal){
  if (current < goal && current > 1510){ //Accelerating
    //Very simplistic function that can be replaced later, plot these in desmos to see the curves, substituting 'current' with x + 1500
    float DeltaPWM =  -sqrt((current-1500)/5)+10;
    } else if(current > goal && current > 1510) { //Breaking  
    float DeltaPWM = -(sqrt((current-1500)/5)+10);
    } else if(current > goal && current < 1490) { //Reversing Accel
    float DeltaPWM = (-3*(-sqrt((-(current-1500))/5)+10));
    } else if(current < goal && current < 1490) { //Reversing Decel
    float DeltaPWM = (3*(-sqrt((-(current-1500))/5)+10));
    } else if(current < goal && (1490 <= current <= 1510)) { //Accel from neutral towards forward includes wait time
    float DeltaPWM = (20/FWD_WAIT);
    } else if(current > goal && (1490 <= current <= 1510)) { //Accel from neutral towards reverse includes wait time
    return -(20/REV_WAIT);
    } else {
    Serial.print("how did you get here? [Critical] ");   
    float DeltaPWM = 0;
    }
   
  
  
}

//void requestEvent() {
//  float test_qdes_temp = 45.7;
//  char test_drive_mode[2] = {'D', 'R'};
//  float test_wheel_speed[2] = {25.7, 12.4};
//  
//  data_dump.QDES_temp = test_qdes_temp;
//  strcpy(data_dump.drive_mode, test_drive_mode);
//
//  for (int i = 0; i < 2; i++) {
//    data_dump.wheel_speed[i] = test_wheel_speed[i];
//  }
//
//  Wire.write((byte *) &data_dump, sizeof(data_dump));
//}

void requestEvent() {
  char test_wheel_speed[2] = {'D', 'R'};
  
//  data_dump.QDES_temp = test_qdes_temp;
  strcpy(data_dump.wheel_speed, test_wheel_speed);

//  for (int i = 0; i < 2; i++) {
//    data_dump.wheel_speed[i] = test_wheel_speed[i];
//  }

  Serial.println((char * ) &data_dump);
  Serial.println(sizeof(data_dump));

  Wire.write((char *) &data_dump, 100);
}


void receiveEvent(int bytes){
  Serial.print("recieved");
  
  static char message[MAX_MESSAGE_LENGTH];
  static String incomingMsg = "";
  unsigned int message_pos = 0;
  while (Wire.available() > 0){
    char inByte = Wire.read();
    Serial.println(inByte);
    
      Serial.print("Message pos: ");
      Serial.println(message_pos);

    //Are not we at the end of the incoming string and not at the incoming message limit?
    if (inByte != '\n' && message_pos < MAX_MESSAGE_LENGTH - 1) {

      message[message_pos] = inByte;
      message_pos++;
    }
    //The full message is here
    else {
      message[message_pos] = '\0';
      
      incomingMsg = message;
       Serial.println();
      Serial.print("Message: ");
      Serial.println(incomingMsg);

     while(Wire.available()){
      Wire.read();
      }

      
      //Decode - check flags
      String leftIn = "";
      for (int i = incomingMsg.indexOf('L')+1;i<incomingMsg.indexOf('R');i++){
        leftIn += incomingMsg[i];
      }
      leftInInt = leftIn.toFloat();
      Serial.print("Decoded Left motor request of: ");
      Serial.println(leftInInt);
      String rightIn = "";
            for (int i = incomingMsg.indexOf('R')+1;i<incomingMsg.indexOf('C');i++){
        rightIn += incomingMsg[i];
      }
      rightInInt = rightIn.toFloat();
      Serial.print("Decoded Right motor request of: ");
      Serial.println(rightInInt);
      String checksumIn = "";
            for (int i = incomingMsg.indexOf('C')+1;i<incomingMsg.indexOf('E');i++){
       checksumIn += incomingMsg[i];
      }
      int checksumInInt = checksumIn.toFloat();
      Serial.print("Decoded Checksum Value: ");
      Serial.println(checksumInInt); 


      //TODO VALIDATE CHECKSUM

        leftInInt = map(leftInInt,-500,500,1000,2000);
        rightInInt = map(rightInInt,-500,500,1000,2000);
        
      
    }




  
  }
}
  
  
  
  
  
  


void loop() {
  // put your main code here, to run repeatedly:
  delay(1);
  
      if(leftInInt != round(leftOut)){

        if (leftOut < leftInInt && leftOut > 1510){ //Accelerating
        //Very simplistic function that can be replaced later, plot these in desmos to see the curves, substituting 'current' with x + 1500
        lDeltaMax =  -sqrt((leftOut-1500)/5)+10;
        Serial.println("a");
        } else if(leftOut > leftInInt && leftOut > 1510) { //Breaking  
        lDeltaMax = -(sqrt((leftOut-1500)/5)+10);
        Serial.println("b");
        } else if(leftOut > leftInInt && leftOut < 1490) { //Reversing Accel
        lDeltaMax = (-3*(-sqrt((-(leftOut-1500))/5)+10));
                Serial.println("c");
        } else if(leftOut < leftInInt && leftOut < 1490) { //Reversing Decel
        lDeltaMax = (3*(-sqrt((-(leftOut-1500))/5)+10));
        Serial.println("d");
        } else if(leftOut < leftInInt && (1490 <= leftOut <= 1510)) { //Accel from neutral towards forward includes wait time
        //lDeltaMax = (20/FWD_WAIT);
        lDeltaMax = 0.01;
        Serial.println("e");
        } else if(leftOut > leftInInt && (1490 <= leftOut <= 1510)) { //Accel from neutral towards reverse includes wait time
        lDeltaMax = -0.01;
        //lDeltaMax -(20/REV_WAIT);
        Serial.println("f");
        } else {
        Serial.print("how did you get here? [Critical] ");   
        lDeltaMax = 0;
        }

        

      
      if (abs(leftInInt - leftOut) > abs(lDeltaMax)) {
        leftOut += lDeltaMax;
      }else{
        leftOut = leftInInt;
      }
      Serial.print("Leftdelta = ");
      Serial.println(lDeltaMax,5);
      Serial.print("LeftIn = ");
      Serial.println(leftInInt);
      Serial.print("LeftPWM = ");
      Serial.println(leftOut);

      
      }
     if(rightInInt != round(rightOut)){
      float rDeltaMax = maxChangeInPWM(round(rightOut),rightInInt);
      if (abs(rightInInt - rightOut) > abs(rDeltaMax)) {
        rightOut += rDeltaMax;










      //incomplete








        
      }else{
        rightOut = rightInInt;
      }
   
      }
  
  motorRight.write(round(rightOut));
  motorLeft.write(round(leftOut));

}
