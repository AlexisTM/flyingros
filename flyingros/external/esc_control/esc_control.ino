/*
 * Esc control programmer for Arduino
 * made with love by AlexisTM
 */
#include <Servo.h>
int value = 0;

Servo esc;
void setup() {
  esc.attach(9);    
  Serial.begin(115200);    
  Serial.println(F("Welcome to the ESC programmer.\n"));
  Serial.println(F("Send an int which is the time in µS the PWM will be up."));
  Serial.println(F("0 is to stop it, 900 is the minimum, 2000 is the maximum"));
  Serial.println(F("To start the programmation, connect pin 9 to ESC signal (orange or yellow)"));
  Serial.println(F("                                      GND to esc GND (brown)"));
  Serial.println(F("Then, send a PWM of 2000 and power on the ESC. "));
  Serial.println(F("It will not rotate and you will hear bips according to your ESC.\n\n"));
  Serial.println(F("Waiting your command (send an int though serial from 900 to 2000).\n\n"));
}

void loop() {
  esc.writeMicroseconds(value);
  if(Serial.available()){
    value = Serial.parseInt();
    Serial.print(F("Writing: "));
    Serial.print(value);
    Serial.println(F("us"));
  }
}
