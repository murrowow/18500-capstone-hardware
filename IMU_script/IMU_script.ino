#include "Wire.h" // This library allows you to communicate with I2C devices.
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include <utility/imumaths.h> 
#include <SoftwareSerial.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

int led = 7;     // draw LED pin
int button = 2; // push button is connected
int c_led = 6; //calibration led pin
int cal_button = 4; // calibration button is connected

int bState = 0;     // temporary variable for reading the button pin status
int prevBState = 0; // temporary variable for reading the button pin status (the previous status)

int cState = 0;      // temporary variable
int cal_time = 1;    // length of calibration

float x_distance = 0; 
float y_distance = 0; 

//time tracking
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
int interval = 0; //ms

// orientation data
float thetaM;
float phiM;
float thetaFold=0;
float thetaFnew;
float phiFold=0;
float phiFnew;
 
float thetaG=0;
float phiG=0;
 
float theta;
float phi;
 
float dt;
unsigned long millisOld;
 
#define BNO055_SAMPLERATE_DELAY_MS (100)

void setup() {
  Serial.begin(9600);
  pinMode(led, OUTPUT);   // declare LED as output
  pinMode(button, INPUT); // declare push button as input
  pinMode(c_led, OUTPUT); 
  
  // Initialise the sensor 
  if(!bno.begin())
  {
    // There was a problem detecting the BNO055 ... check your connections 
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
  Serial.println(""); Serial.println("Calibrated");
  
  delay(1000);
  bno.setExtCrystalUse(true);
} 

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
  cState = digitalRead(cal_button); 

  // collect and calculate the roll and pitch data
  uint8_t system, gyro, accel, mg = 0;
  bno.getCalibration(&system, &gyro, &accel, &mg);
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyr = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
 
  thetaM=-atan2(acc.x()/9.8,acc.z()/9.8)/2/3.141592654*360;
  phiM=-atan2(acc.y()/9.8,acc.z()/9.8)/2/3.141592654*360;
  phiFnew=.95*phiFold+.05*phiM;
  thetaFnew=.95*thetaFold+.05*thetaM;
 
  dt=(millis()-millisOld)/1000.;
  millisOld=millis();
  theta=(theta+gyr.y()*dt)*.95+thetaM*.05;
  phi=(phi-gyr.x()*dt)*.95+ phiM*.05;
  thetaG=thetaG+gyr.y()*dt;
  phiG=phiG-gyr.x()*dt;
  
  //bluetooth signals
  if (cState == LOW && prevBState == LOW && bState == HIGH){
      Serial.print("Bon\n");
  }else if (cState == LOW && prevBState == HIGH && bState == LOW){
      Serial.print("Boff\n");
  }
  
  //trying to calibrate the pen
  if (cState == HIGH) {
    digitalWrite(c_led, HIGH);
    thetaG=0;
    thetaFold=0;
    phiG=0;
    phiFold=0;
    x_distance = 0; 
    y_distance = 0; 
  } else { 
    digitalWrite(c_led, LOW);
  }

  // when drawing button is pressed and not calibrating
  if (bState == HIGH && cState == LOW) {
    digitalWrite(led, HIGH);
    update_time();
    if (hit_time_interval()) {
      x_distance += -phi * dt * dt / 100;
      y_distance += theta * dt * dt / 100;
      Serial.print(x_distance, 2); 
      Serial.print("_"); 
      Serial.print(y_distance, 2);
      Serial.print("_"); 
      Serial.print(0);
      Serial.print("\n"); 
    }
    phiFold=phiFnew;
    thetaFold=thetaFnew;
  } else { 
    digitalWrite(led, LOW);
    x_distance = 0; 
    y_distance = 0; 
  }
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
