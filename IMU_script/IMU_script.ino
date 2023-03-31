#include "Wire.h" // This library allows you to communicate with I2C devices.
#include "ESP8266WiFi.h" // Wifi library 
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "utility/imumaths.h"

Adafruit_BNO055 bno = Adafruit_BNO055(55);

int led = 0;     // LED pin
int button = 16; // push button is connected
int temp = 0;    // temporary variable for reading the button pin status
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

        /* Display the floating point data */
        Serial.print("X: ");
        Serial.print(event.orientation.x, 4);
        Serial.print("\tY: ");
        Serial.print(event.orientation.y, 4);
        Serial.print("\tZ: ");
        Serial.print(event.orientation.z, 4);
        Serial.print("\tdistance: ");
        Serial.print(event.distance, 4);
        Serial.println("");
        delay(100);
     } else { 
       digitalWrite(led, LOW);
     }
}
