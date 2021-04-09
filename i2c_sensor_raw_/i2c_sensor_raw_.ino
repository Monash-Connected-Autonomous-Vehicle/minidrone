#include <Wire.h>

int i2cAddress = 0x40;
char test;

void setup() {
  Wire.begin(i2cAddress);                 
}

void loop() {
  test = 'a';
  Wire.write(test);
}
