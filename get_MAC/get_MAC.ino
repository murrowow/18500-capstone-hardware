#include <ESP8266WiFi.h>

void setup(){
  Serial.begin(115200);
  Serial.println();
}
 
void loop(){
  Serial.print("ESP Board MAC Address:  "); //2C:F4:32:7C:31:87
  Serial.println(WiFi.macAddress());
}
