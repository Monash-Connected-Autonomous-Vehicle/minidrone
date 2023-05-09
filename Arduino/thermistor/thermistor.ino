int sensorPin = A0;
float Vout = 0;
float Vin = 5;
int R = 100000;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
float logRt, Rt, T;

void setup() {
  
  Serial.begin(9600);
}

void loop() {
  Vout = analogRead(sensorPin);

  Rt = R * ((Vin/Vout) - 1);
  
  logRt = log(Rt);
  T = (1.0 / (c1 + c2*logRt + c3*logRt*logRt*logRt));
  T = T - 273.15 + 23.85; //celsius 

  Serial.print("Temperature: "); 
  Serial.print(T);
  Serial.println(" C"); 

}
