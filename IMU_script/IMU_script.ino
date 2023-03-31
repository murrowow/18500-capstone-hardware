#include "Wire.h" // This library allows you to communicate with I2C devices.
#include "ESP8266WiFi.h" // Wifi library 
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"

Adafruit_BNO055 bno = Adafruit_BNO055(55);

int led = 0;     // LED pin
int button = 16; // push button is connected
int temp = 0;    // temporary variable for reading the button pin status

double pos_vector[3] = {0, 0, 0}; 

// imu data
 
void setup() {
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  pinMode(led, OUTPUT);   // declare LED as output
  pinMode(button, INPUT); // declare push button as input
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
  bno.setExtCrystalUse(true);
}

void loop() {
  sensors_event_t event; 
  temp = digitalRead(button);
     if (temp == HIGH) {
        digitalWrite(led, HIGH);
        bno.getEvent(&event);

        double time_step = event.timestamp * 0.00001;
        double dist_x = event.acceleration.x * time_step * time_step;
        double dist_y = event.acceleration.y * time_step * time_step; 
        double dist_z = event.acceleration.z * time_step * time_step; 

        /* Display the floating point data */
        pos_vector[0] += dist_x; 
        pos_vector[1] += dist_y; 
        pos_vector[2] += dist_z; 
        Serial.print("X: ");
        Serial.print(pos_vector[0], 4);
        Serial.print("\tY: ");
        Serial.print(pos_vector[1], 4);
        Serial.print("\tZ: ");
        Serial.print(pos_vector[2], 4);
        Serial.println("");
        delay(100); 
     } else { 
       digitalWrite(led, LOW);
       pos_vector[0] = 0; 
       pos_vector[1] = 1; 
       pos_vector[2] = 2; 
     }
}
