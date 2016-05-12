unsigned long last_time = millis();
unsigned long last_minute = millis();
unsigned long last_alarm = millis();
int counter = 0;

#include <SoftwareSerial.h>

SoftwareSerial gsm(10,11); // RX, TX
char phoneNumber[] = "0646618884";
void setup() {
    Serial.begin(9600);
    pinMode(3, INPUT);
    pinMode(4, INPUT);
    pinMode(5, INPUT);
    gsm.begin(9600);
    delay(3500);
}

void loop() {
    Serial.print(counter);
    Serial.write("\n");
    if ((millis() - last_alarm) > 20000) { // 20 seconden
        Serial.write("No movement in last 20 seconds \n");
    }

    if ((millis() - last_minute) > 60000) { // 1 minuut
        counter = 0;
        last_minute = millis();
    }
    if ((millis() - last_time) > 2000) { // 2 seconds
        bool alarm = last_two_seconds();
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
   // Serial.write(text);
    gsm.println("AT+CPIN=0905"); // set SMS mode to text
    while(gsm.available())
        Serial.write((byte)gsm.read());
    
    delay(500);
    
    gsm.println("AT+CMGF=1"); // set SMS mode to text
    while(gsm.available())
        Serial.write((byte)gsm.read());
    
    delay(500);
    Serial.println();
    
    gsm.print("AT+CMGS="); // now send message...
    
    delay(500);
    
    gsm.write((byte)34);// ASCII equivalent of "
    
    delay(500);
    
    gsm.print(phoneNumber);
    gsm.write((byte)34); // ASCII equivalent of "
    
    delay(500);
    
    gsm.println();
    while(gsm.available())
        Serial.write((byte)gsm.read());
    
    delay(500);
    Serial.println();
    
    gsm.print(text); // our message to send
    gsm.write((byte)26); // ASCII equivalent of Ctrl-Z
    // this will send the following to the GSM module
    // on the Cellular Shield: AT+CMGS=”phonenumber”<CR>
    // message<CTRL+Z><CR>
    gsm.println();
    while(gsm.available())
        Serial.write((byte)gsm.read());
    delay(500);
}
