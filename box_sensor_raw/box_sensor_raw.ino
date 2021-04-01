/* MCAV- Box
 *  
 * - Sends data from IMU and GPS modules via serial port
 * 
 * - Information sent is used to reconstruct ROS sensor messages (/NavSatFix and /Imu)
 * 
 * - Data is packaged into "frames". Frames start with and end with any single alphabetical character (a-z). 
 *   IMU and GPS information is packaged into two different frames. Precision of incoming and transmitted data must be known at 
 *   either end. 
 * 
 * WIRING UP:
 * 
 * |5V| -> VCC for IMU
 * |GND| -> GND for IMU
 * |A4| -> SDA for IMU
 * |A5| -> SCL for IMU
 *  
 * |3.3V| -> VCC for GPS
 * |GND| -> GND for GPS
 * |8| -> TX for GPS
 * |9| -> RX for GPS
 * 
 * 
 */ 
 
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

static const int RXpin = 8, TXpin = 9; 
static const uint32_t GPSbaud = 9600; 
static const uint32_t SERIAL_BAUD = 115200; // Serial port receiving must match this 

static const int GPS_PRECISION = 6;
static const int IMU_PRECISION = 4;

TinyGPSPlus gps; // TinyGPS++ object

SoftwareSerial ss(RXpin,TXpin);// serial connection to GPS module

#define SENSOR_DELAY_MS (100) // delay needed so that input buffer does not overflow

Adafruit_BNO055 bno = Adafruit_BNO055(55); // instantiate IMU class 

void setup() {
  Serial.begin(SERIAL_BAUD); // start serial port communication
  if(!bno.begin()) Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");   
  bno.setExtCrystalUse(true); // use the crystal on the development board
  
  ss.begin(GPSbaud); // recieve serial GPS data 
}

void loop() {
  
  // imu sensor events data
  sensors_event_t angVelocityData , linearAccelData, magnetometerData; 
  
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);

  // gps
  while (ss.available() > 0) {
    if (gps.encode(ss.read())) {
      
      send_gps_info();
    }
  }

  // data frame for IMU
  Serial.print("imu,");  
  send_imu_quat();
  send_imu_info(&angVelocityData);
  send_imu_info(&linearAccelData);
  Serial.print("end");
  Serial.println();

  // data frame for magnetometer
  Serial.print("mag,"); 
  send_imu_info(&magnetometerData);
  Serial.print("end");
  Serial.println();

  // data for calibration
  Serial.print("cal,");
  send_calibration_info();
  Serial.print("end");
  Serial.println();
  
  delay(SENSOR_DELAY_MS); // Pause before capturing new data and sending data
}

// calculates and sends quaternions from IMU data 
void send_imu_quat(){
  imu::Quaternion quat = bno.getQuat(); // Request quaternion data from BNO055
  Serial.print(quat.x(), IMU_PRECISION);
  Serial.print(",");  
  Serial.print(quat.y(), IMU_PRECISION);  
  Serial.print(",");
  Serial.print(quat.z(), IMU_PRECISION);
  Serial.print(",");
  Serial.print(quat.w(), IMU_PRECISION);
  Serial.print(","); 
}

// sends linear acceleration, angular velocity, and magnetometer data via serial 
void send_imu_info(sensors_event_t *event){
  
  if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION){
    // - VECTOR_LINEARACCEL   - m/s^2
    Serial.print(event->acceleration.x, IMU_PRECISION);
    Serial.print(",");  
    Serial.print(event->acceleration.y, IMU_PRECISION);
    Serial.print(",");  
    Serial.print(event->acceleration.z, IMU_PRECISION); 
    Serial.print(",");
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR){
    // - VECTOR_GYROSCOPE     - rad/s
    Serial.print(event->gyro.x, IMU_PRECISION);  
    Serial.print(",");
    Serial.print(event->gyro.y, IMU_PRECISION);  
    Serial.print(",");
    Serial.print(event->gyro.z, IMU_PRECISION); 
    Serial.print(",");
  }   
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    // - VECTOR_MAGNETOMETER  - uT
    Serial.print(event->magnetic.x);
    Serial.print(",");
    Serial.print(event->magnetic.y);
    Serial.print(",");
    Serial.print(event->magnetic.z);
    Serial.print(",");
  }
}

  // TODO: ACCELEROMETER and GRAVITY
  // - VECTOR_EULER         - degrees
  // - VECTOR_GRAVITY       - m/s^2
  // - VECTOR_ACCELEROMETER - m/s^2

  // bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  // bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

void send_calibration_info(void)
{
  // imu calibration events data
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  
  // imu temperature data degC
  int8_t temp = bno.getTemp();

  Serial.print(system, DEC);
  Serial.print(",");
  Serial.print(gyro, DEC);
  Serial.print(",");
  Serial.print(accel, DEC);
  Serial.print(",");
  Serial.print(mag, DEC);
  Serial.print(",");
  Serial.print(temp, DEC);
  Serial.print(",");
   
}

// sends entire GPS "data frame" 
void send_gps_info(){
  Serial.print("gps,");
  Serial.print(gps.location.lat(), GPS_PRECISION);
  Serial.print(",");
  Serial.print(gps.location.lng(), GPS_PRECISION);
  Serial.print(",");
  Serial.print(gps.altitude.meters(), GPS_PRECISION);
  Serial.print(",");
  Serial.print(gps.satellites.value());
  Serial.print(",");
  Serial.print("end");
  Serial.println();
}
