#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <SoftwareSerial.h> 
SoftwareSerial s(D4,D8); //RX,TX

#define BLYNK_AUTH_TOKEN "iXYGxQOWkLKhe_Eziyo2ON9jQoT78pji" // Enter your Blynk auth token

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "vivo"; // Enter your WiFi name
char pass[] = "123456789"; // Enter your WiFi password

void setup() {
  Serial.begin(9600);
  s.begin(9600);
  pinMode(D0, OUTPUT);
  Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);
}

void loop() {
  Blynk.run();
}

BLYNK_WRITE(V0) {
int sliderValue = param.asInt();
sliderValue=sliderValue/2.83;
Serial.print("Slider Value: ");
Serial.println(sliderValue);
s.write(sliderValue);
}








