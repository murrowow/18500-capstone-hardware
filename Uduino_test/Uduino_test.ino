#include<Uduino.h>

void setup()
{
  Serial.begin(9600);
}

void loop() {
  uduino.update();  //Mandatory part, to maintain the connection between Arduino and Unity
  uduino.println("Hello!\n"); 
}
