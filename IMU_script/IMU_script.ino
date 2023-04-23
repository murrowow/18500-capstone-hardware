#include "Wire.h" // This library allows you to communicate with I2C devices.
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include <utility/imumaths.h>
#include <SoftwareSerial.h>

#define PI 3.1415926535897932384626433832795

Adafruit_BNO055 bno = Adafruit_BNO055(55);


  
int led = 11;     // LED pin
int button = 12; // push button is connected
int bState = 0;     // temporary variable for reading the button pin status
int prevBState = 0; // temporary variable for reading the button pin status (the previous status)

float quant_vector[4] = {0, 0, 0}; //w, x, y, z coordinates
float pos_vec[3] = {0, 0, 0}; 
float acc_mag = 0; 
int calibration = 0; 
float accel_error[3] = {0};

// Values used to track time
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
int interval = 100; //ms

void setup() {
  Serial.begin(9600);
  Serial.println("AccX,AccY,AccZ");
  pinMode(led, OUTPUT);   // declare LED as output
  pinMode(button, INPUT); // declare push button as input
  
  // Initialise the sensor 
  if(!bno.begin())
  {
    // There was a problem detecting the BNO055 ... check your connections 
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  int time = 50; 
  Serial.println("Calibrating");
  while (time > 0) {
    imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    accel_error[0] += acc.x();
    accel_error[1] += acc.y();
    accel_error[2] += acc.z(); 
    Serial.print(".");
    time -= 1; 
  }
  Serial.println("");
  Serial.print("Done"); 
  
  delay(1000);
  bno.setExtCrystalUse(true);
}

//** QUATERNION FUNCTIONS **//
void quat_prod(float q1[4], float q2[4], float q3[4]) {
  q3[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
  q3[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
  q3[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
  q3[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

void quat_conj(float q1[4]) {
  q1[1] = q1[1] * -1; 
  q1[2] = q1[2] * -1;
  q1[3] = q1[3] * -1;; 
}

void quat_rotate(float v[4], float q[4], float res[4]) {
  float temp[4] = {0, 0, 0, 0}; 
  float conj_q[4] = {q[0], q[1], q[2], q[3]};
  quat_conj(conj_q);
  quat_prod(q, v, temp);
  quat_prod(temp, conj_q, res);
}
//** QUATERNION FUNCTIONS **//
//** TIME FUNCTIONS **//
bool hit_time_interval(){
  bool hasHit = currentMillis - previousMillis >= interval;
  if (hasHit){
    previousMillis = currentMillis;
  }
  return hasHit;
}

void update_time(){
  currentMillis = millis();
}
//** TIME FUNCTIONS **//

void loop() {
  sensors_event_t event; 
  prevBState = bState;
  bState = digitalRead(button);
  
  if (prevBState == LOW && bState == HIGH){
//      Serial.print("Bon\n");
  }else if (prevBState == HIGH && bState == LOW){
//      Serial.print("Boff\n");

  }

  static unsigned long pastTime = millis();
  static float velocity[3] = { 0 };
  static float pastAccel[4] = { 0 };

  if (bState == HIGH) {
    digitalWrite(led, HIGH);
    imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY); 
    imu::Quaternion quat = bno.getQuat(); 

    unsigned long currentTime = millis();
    float acc_vect[4] = {0, acc.x(), acc.y(), acc.z()}; 
    float quat_vect[4] = {quat.w(), quat.x(), quat.y(), quat.z()};
    float res[4] = {0, 0, 0, 0}; 

    acc_mag = (sqrt(pow(acc.x(),2) + pow(acc.y(),2) + pow(acc.z(),2)));
    //ignore if acceleration too low
    if (acc_mag > 0.05) {
      float conj_q[4] = {quat_vect[0], quat_vect[1], quat_vect[2], quat_vect[3]};
      quat_conj(conj_q);
      quat_rotate(acc_vect, conj_q, res);

      //convert into m/s/s
      res[1] /= 9.81f*1000000.f;
      res[2] /= 9.81f*1000000.f;
      res[3] /= 9.81f*1000000.f;

      float dt = (float) (currentTime - pastTime);
      float dv[3] = {0.5f * (pastAccel[1] + res[1]) * dt, 0.5f * (pastAccel[2] + res[2]) * dt, 0.5f * (pastAccel[3] + res[3]) * dt};

      velocity[0] += dv[0];
      velocity[1] += dv[1];
      velocity[2] += dv[2];

      //get position by float integrating
      pos_vec[0] += 0.5f * dv[0] * dt + velocity[0] * dt;
      pos_vec[1] += 0.5f * dv[1] * dt + velocity[1] * dt;
      pos_vec[2] += 0.5f * dv[2] * dt + velocity[2] * dt;

      // Update past accel data
      memcpy(pastAccel, res, 4*sizeof(float));
    }

    pastTime = millis();
    update_time();
    if (hit_time_interval()) {
//      Serial.print(pos_vec[0], 2);
//      Serial.print("_");
//      Serial.print(pos_vec[2],2);
//      Serial.print("_");
//      Serial.print(pos_vec[1],2);
//      Serial.print("\n");
       Serial.print(pos_vec[0], 2);
       Serial.print(",");
       Serial.print(pos_vec[1], 2);
       Serial.print(",");
       Serial.println(pos_vec[2], 2);
    }

  } else { 

    digitalWrite(led, LOW);
    pos_vec[0] = 0;
    pos_vec[1] = 0;
    pos_vec[2] = 0;
  }
  delay(100); //how long IMU delay is
}
