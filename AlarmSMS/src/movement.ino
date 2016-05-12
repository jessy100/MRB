unsigned long last_time = millis();
unsigned long last_minute = millis();
unsigned long last_alarm = millis();
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

    if ((millis() - last_alarm) > 20000) { // 1 minuut
        send_sms("No movement in last 10 seconds");
    }

    if ((millis() - last_time) > 60000) { // 1 minuut
        counter = 0;
        last_minute = millis();
    }
    if ((millis() - last_time) > 2000) { // 2 seconds
        bool alarm = last_two_seconds(); //Tumblr in action x)
        if (alarm) {
            last_alarm = millis();
            counter++;
            if (counter > 10) {
                send_sms("10 movements in 1 minute!");
                counter = 0;
            }
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

void send_sms(char text[]) {
    //SMS stuff here
}
