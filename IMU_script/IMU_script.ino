#include "Wire.h" // This library allows you to communicate with I2C devices.
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include <utility/imumaths.h>
#include <SoftwareSerial.h>

#define PI 3.1415926535897932384626433832795

Adafruit_BNO055 bno = Adafruit_BNO055(55);


  
int led = 11;     // LED pin
int button = 12; // push button is connected
int temp = 0;    // temporary variable for reading the button pin status

double quant_vector[4] = {0, 0, 0}; //w, x, y, z coordinates
double pos_vec[3] = {0, 0, 0}; 
double acc_mag = 0; 
int calibration = 0; 


// https://conservancy.umn.edu/bitstream/handle/11299/199815/Real-Time%20Position%20Tracking%20Using%20IMU%20Data.pdf?sequence=1&isAllowed=y
void setup() {
  Serial.begin(9600);
  pinMode(led, OUTPUT);   // declare LED as output
  pinMode(button, INPUT); // declare push button as input
  
  // Initialise the sensor 
  if(!bno.begin())
  {
    // There was a problem detecting the BNO055 ... check your connections 
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  uint8_t system, gyro, accel, mag = 0;
  while(system != 3)
  {
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.print("CALIBRATION: Sys=");
    Serial.print(system, DEC);
    Serial.print(" Gyro=");
    Serial.print(gyro, DEC);
    Serial.print(" Accel=");
    Serial.print(accel, DEC);
    Serial.print(" Mag=");
    Serial.println(mag, DEC);
    delay(100);
  }

  Serial.println(""); Serial.println("Calibrated");
  
  delay(1000);
  bno.setExtCrystalUse(true);
}

void quat_prod(double q1[4], double q2[4], double q3[4]) {
  q3[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
  q3[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
  q3[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
  q3[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

void quat_conj(double q1[4]) {
  q1[1] = q1[1] * -1; 
  q1[2] = q1[2] * -1;
  q1[3] = q1[3] * -1;; 
}

void quat_rotate(double v[4], double q[4], double res[4]) {
  double temp[4] = {0, 0, 0, 0}; 
  double conj_q[4] = {q[0], q[1], q[2], q[3]};
  quat_conj(conj_q);
  quat_prod(q, v, temp);
  quat_prod(temp, conj_q, res);
}


void loop() {
  sensors_event_t event; 
  temp = digitalRead(button);
     if (temp == HIGH) {
        digitalWrite(led, HIGH);
        imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        imu::Quaternion quat = bno.getQuat(); 

        double acc_vect[4] = {0, acc.x(), acc.y(), acc.z()}; 
        double quat_vect[4] = {quat.w(), quat.x(), quat.y(), quat.z()};
        double res[4] = {0, 0, 0, 0}; 
        
        acc_mag = (sqrt(pow(acc.x(),2) + pow(acc.y(),2) + pow(acc.z(),2)));
        //ignore if acceleration too low
        if (acc_mag > 0) {
          double conj_q[4] = {quat_vect[0], quat_vect[1], quat_vect[2], quat_vect[3]};
          quat_conj(conj_q);
          quat_rotate(acc_vect, conj_q, res); 

          //convert into m/s/s
          res[1] *= 9.81/1000000L;
          res[2] *= 9.81/1000000L;
          res[3] *= 9.81/1000000L;

          //get position by double integrating
          pos_vec[0] += (res[1]) * 100 * 100;
          pos_vec[1] += (res[2]) * 100 * 100;
          pos_vec[2] += (res[3]) * 100 * 100;
        }

        Serial.print(pos_vec[0], 4);
        Serial.print("_");
        Serial.print(pos_vec[2],4);
        Serial.print("_");
        Serial.print(pos_vec[1],4);
        Serial.print("\n");

     } else { 
       digitalWrite(led, LOW);
       pos_vec[0] = 0;
       pos_vec[1] = 0;
       pos_vec[2] = 0;
     }
  delay(100); //how long IMU delay is
}
