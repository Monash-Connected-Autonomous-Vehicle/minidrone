#include <PID_v2.h>


// Specify the links and initial tuning parameters
// PID controller gains (lookup their meanings)
double Kp = 20, Ki = 5, Kd = 1;

// PID controller obj
PID_v2 myPID(Kp, Ki, Kd, PID::Direct);

const int ticksPerRevolution = 1024; //ticks of the encoder for 1 revolution
float prevTime = 0; //time in microseconds
float prevVelocity = 0;

/*******Interrupt-based Rotary Encoder Sketch*******
stolen from https://www.instructables.com/Improved-Arduino-Rotary-Encoder-pinValues/
*/

static int pinA = 2; // Our first hardware interrupt pin is digital pin 2
static int pinB = 3; // Our second hardware interrupt pin is digital pin 3

volatile byte aFlag = 0; // let's us know when we're expecting a rising edge on ISRPinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0; // let's us know when we're expecting a rising edge on ISRPinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile long int encoderPos = 0; //Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
volatile long int prevEncPos = 0; 
volatile byte pinValues = 0; 

void setup() {
  myPID.Start(
     0,    // input
     0,    // current outputPWM
     0.1   // setpoint
  ); 
  // set pin A & B as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pinA, INPUT_PULLUP); 
  pinMode(pinB, INPUT_PULLUP); 
  // execute ISR when rising on both pins
  attachInterrupt(digitalPinToInterrupt(pinA), PinAISR, RISING); 
  attachInterrupt(digitalPinToInterrupt(pinB), PinBISR, RISING); 
  
  Serial.begin(115200); 
}

void PinAISR(){
  PinISR(pinA);
}

void PinBISR(){
  PinISR(pinB);
}


void PinISR(int pin){
  cli(); //stop interrupts happening before we read pin values
  volatile byte thisFlag = ( pin == pinA ) ? aFlag : bFlag;

  pinValues = PIND & 0xC; // read all eight pin values then strip away all but ISRPinA and ISRPinB's values
  if(pinValues == B00001100 && thisFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    ( pin == pinA ) ? encoderPos-- : encoderPos++;    
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (pinValues == B00000100) bFlag = 1; //signal that we're expecting ISRPinB to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

void loop(){
  if(prevEncPos != encoderPos) {
      // single time for the whole iteration. 
      long long int timeNow = micros();
      
      //Updating deltas
      float delta_pos = ((encoderPos - prevEncPos)); // Ticks
      float delta_time = (timeNow - prevTime) / pow(10, 6); // Seconds
      float revPerSec =  ( (delta_pos / delta_time) / float(ticksPerRevolution)); 

      prevTime = timeNow;
      prevEncPos = encoderPos;

      Serial.print(revPerSec,9);
      

      int outputPWM = myPID.Run(revPerSec); 
      Serial.print(outputPWM);
      Serial.println();
    }
}
