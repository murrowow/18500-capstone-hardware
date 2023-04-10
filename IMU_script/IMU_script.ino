#include "Wire.h" // This library allows you to communicate with I2C devices.
#include "ESP8266WiFi.h" // Wifi library 
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include <SoftwareSerial.h>
SoftwareSerial BTserial(8, 9); // RX | TX

Adafruit_BNO055 bno = Adafruit_BNO055(55);

int led = 0;     // LED pin
int button = 16; // push button is connected
int temp = 0;    // temporary variable for reading the button pin status

double pos_vector[3] = {0, 0, 0};
double old_timestamp = 0; 
double timestamp = 0;  

const long baudRate = 38400; 
char c=' ';
boolean NL = true;

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
  
   BTserial.begin(baudRate);  
   Serial.print("BTserial started at "); Serial.println(baudRate);
   Serial.println(" ");
  
  delay(1000);
  bno.setExtCrystalUse(true);
}

void loop() {
  // Read from the Bluetooth module and send to the Arduino Serial Monitor
    if (BTserial.available())
    {
        c = BTserial.read();
        Serial.write(c);
    }
 
 
    // Read from the Serial Monitor and send to the Bluetooth module
    if (Serial.available())
    {
        c = Serial.read();
        BTserial.write(c);   
 
        // Echo the user input to the main window. The ">" character indicates the user entered text.
        if (NL) { Serial.print(">");  NL = false; }
        Serial.write(c);
        if (c==10) { NL = true; }
    }
    
  sensors_event_t event; 
  temp = digitalRead(button);
     if (temp == HIGH) {
        digitalWrite(led, HIGH);
        bno.getEvent(&event);

        double timestamp = event.timestamp;
        double time_step = (timestamp - old_timestamp)/1000000L; //convert to s
        double dist_x = event.acceleration.x * time_step * time_step;
        double dist_y = event.acceleration.y * time_step * time_step; 
        double dist_z = event.acceleration.z * time_step * time_step; 

        /* Display the floating point data */
        pos_vector[0] += dist_x; 
        pos_vector[1] += dist_y; 
        pos_vector[2] += dist_z; 
        delay(100); 
     } else { 
       digitalWrite(led, LOW);
       pos_vector[0] = 0; 
       pos_vector[1] = 0; 
       pos_vector[2] = 0; 
     }
     old_timestamp = timestamp; 
}
