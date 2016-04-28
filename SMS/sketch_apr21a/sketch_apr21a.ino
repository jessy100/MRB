 /*
  Software serial multple serial test

 Receives from the hardware serial, sends to software serial.
 Receives from software serial, sends to hardware serial.

 The circuit:
 * RX is digital pin 10 (connect to TX of other device)
 * TX is digital pin 11 (connect to RX of other device)

 Note:
 Not all pins on the Mega and Mega 2560 support change interrupts,
 so only the following can be used for RX:
 10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69

 Not all pins on the Leonardo support change interrupts,
 so only the following can be used for RX:
 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).

 created back in the mists of time
 modified 25 May 2012
 by Tom Igoe
 based on Mikal Hart's example

 This example code is in the public domain.

 */
#include <SoftwareSerial.h>

SoftwareSerial gsm(10,11); // RX, TX
char phoneNumber[] = "0646618884";
void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  //while (!Serial) {
  //  ; // wait for serial port to connect. Needed for native USB port only
  gsm.begin(9600);
  delay(3500);
}



void loop()
{ 

gsm.println("AT");
while(gsm.available())
    Serial.write((byte)gsm.read());
delay(500);
Serial.println();
  

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
gsm.print("Hello, This is your Arduino-Mini"); // our message to send
gsm.write((byte)26); // ASCII equivalent of Ctrl-Z
// this will send the following to the GSM module
// on the Cellular Shield: AT+CMGS=”phonenumber”<CR>
// message<CTRL+Z><CR>
gsm.println();
while(gsm.available())
    Serial.write((byte)gsm.read());
delay(500);
Serial.println();
delay(15000); // The GSM module needs to return to an OK status
}
