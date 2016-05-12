#include <Arduino.h>

// Quadcopter workshop for HU by Alten 18-03-2015

#include "PID_v1.h"

//PID Parameters
const double  Kp = 60;
const double  Ki = 2*Kp / 10;
const double  Kd = Kp * 10 / 10;

// Interval (milliseconds) between sending analog data
const int SampleTime = 50; // [ms]
int LastSample;

// define the revolutions counter int volatile
volatile int half_revolutions = 0;// count the half revolutions of the fan with the tacho  meter

// define digital pins
int HeightSensor = A0;
int MotoroutPin = 3; // PWM output
int Tacho = 2;

// Define the control input,output and setpoint global, so the PID controller can use them
double ActualHeightSensor;
double ActualHeightFan;
double DesiredHeightFan = 11; // measure the height which you think the fan should be to balance [cm]
double Motorout; // PID output

/*Specify the PID input, setpoint and output
* The input is the actual height of the fan [cm]
* The setpoint is the desired height of the fan [cm]. The controller should get the actual height as close as possible to desired height.
* The output is mapped to the pwm Motorout [0-255]
*/
PID PID_controller(&ActualHeightFan, &Motorout, &DesiredHeightFan,Kp,Ki,Kd, DIRECT);


void setup()
{
   // configure motor output pin
   pinMode(MotoroutPin, OUTPUT);
   // set the PWM frequency , do not change!
   TCCR2B = TCCR2B & 0b11111000 | 0x01;

   //configure tacho input to pin2, interrupt 0
   // Board	     int.0	int.1
   // Uno, Ethernet	2	3
   pinMode(Tacho,INPUT_PULLUP);
   attachInterrupt(0, rpm_fan, RISING);

  // Open serial communications and wait for port to open:
  Serial.begin(115200);

  //turn the PID on
  PID_controller.SetSampleTime(SampleTime); // config sampletime
  PID_controller.SetOutputLimits(0,255);    // Analogwrite PWM function is mapped between 0 and 255
  PID_controller.SetMode(AUTOMATIC);

}

void rpm_fan(){ /* this code will be executed every time the interrupt 0 (pin2) gets high.*/
  half_revolutions++;
}

// function that sends data with corresponding label to the serial port for visualisation in format of Megunolink
void sendPlotData(String seriesName, float data){
  Serial.print("{TIMEPLOT|data|");
  Serial.print(seriesName);
  Serial.print("|T|");
  Serial.print(data);
  Serial.println("}");
}

void loop() {
  if ((millis() - LastSample) > SampleTime){
    LastSample = millis(); //current time

     int val =  analogRead(HeightSensor);

     // the Adc value first needs to  be mapped from 0..1023 to 0..5v.
     double HeightSensorVoltage = (5.0/1023.0) * (double) val;

     /* Calibration voltage v [V] to distance L [cm]
     *
     *       The inverse distance / voltage characteristic as on the last page of http://docs-europe.electrocomponents.com/webdocs/0d1b/0900766b80d1bdcc.pdf is used.
     *       Calibration data:
     *
     *       L =  5 [cm] ; v = 2.85 [V] ;  invL = 1 / (L+0.42) = 0.185 [1/cm]
     *       L =  20 [cm] ; v = 0.62 [V] ; invL = 1 / (L+0.42) = 0.049 [1/cm]
     *
     *       linear fit: v = a + b*invL
     *        -> v = -0.184 + 16.4*invL
     *        -> v = -0.184 + 16.4/(L + 0.42)
     *        -> L = 16.4 / (v+0.184) - 0.42
     */

     ActualHeightSensor  = 16.4 / (HeightSensorVoltage + 0.184) - 0.42; // in cm

     //from the height of the sensor, compute the height of the fan (will be more convenient for control later on):
     ActualHeightFan     = 20 - ActualHeightSensor;

     // check the diffrence between the actual heigh and the desired heght to v
     float Error = DesiredHeightFan - ActualHeightFan;

     /*
      * 2: Read out the Tacho meter and calibrate the pwm range.
      *
      * First Convert the half_revolutions to frecuency for current sample moment
      */
     // float rpm = 30*1000/(millis() - LastSample)*half_revolutions;
      float rpm = 0.15 * (30000* half_revolutions /(SampleTime)) + 0.85 * rpm; // Digital Low pass filter with wcutoff ~ 1 hz.
      half_revolutions = 0;

      /*
      *    Compute the new PID controller output
      */
      PID_controller.Compute();


      //write PWM to MotoroutPin to set the fan speed.
      //analogWrite(MotoroutPin,255);
      analogWrite(MotoroutPin,Motorout);

//    sendPlotData("rpm",rpm);
      sendPlotData("ActualHeightFan",ActualHeightFan);
      sendPlotData("Motorout",Motorout);
      sendPlotData("Error",Error);

  }

}
