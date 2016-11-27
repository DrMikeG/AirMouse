

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

void setup()   {                
  Serial.begin(38400);
  
  pinMode(8, INPUT_PULLUP);
  
      // configure LED for output
    pinMode(LED_PIN, OUTPUT);

}

void loop()                     
{
  if (digitalRead(8) == HIGH) {
    Serial.println("Button is not pressed...");
    digitalWrite(LED_PIN, false);

  } else {
    Serial.println("Button pressed!!!");
    digitalWrite(LED_PIN, true);
  }
  delay(250);
}
