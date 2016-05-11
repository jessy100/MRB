unsigned long last_time = millis();
bool still_triggered = false;
int counter = 0;

void setup() {
    Serial.begin(9600);
    pinMode(3, INPUT);
    pinMode(4, INPUT);
    pinMode(5, INPUT);
}

void loop() {
    if (!device_is_on()) {
        last_time = millis();
        return;
    }
    if ((millis() - last_time) > 2000) { // 2 seconds
        bool alarm = last_two_seconds();(); //Tumblr in action x)
        if (alarm) {
            if (!still_triggered) {
                counter++;
                still_triggered = true;
                if (counter > 10) {
                    send_sms();
                }
            }
        }
        else {
            still_triggered = true;
        }
        last_time = millis();
    }
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

void send_sms() {
    //SMS stuff here
}
