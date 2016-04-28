/*
 Software serial multple serial test

*/
void setup() {
    Serial.begin(9600);
    pinMode(3, INPUT);
    pinMode(4, INPUT);
    pinMode(5, INPUT);
}

void loop() {
    bool alarm= alarm_is_triggered();
    bool dev = device_is_on();
    bool secs = last_two_seconds();
//    Serial.write("Alarm: ");
//    Serial.print(alarm);
//    Serial.write("\n");
//
//    Serial.write("On: ");
//    Serial.print(dev);
//    Serial.write("\n");
    if (secs) {
      Serial.write("Secs: ");
      Serial.print(secs);
      Serial.write("\n");
    }

//    Serial.write("--------\n");
}

bool alarm_is_triggered() {
  return !digitalRead(3);
}

bool device_is_on() {
  return digitalRead(4);
}

bool last_two_seconds() {
  return analogRead(0) > 350;
}

