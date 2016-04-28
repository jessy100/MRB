#include <Arduino.h>

// Quadcopter workshop for HU by Alten 18-03-2015
#include "PID_v1.h"

class FuzzyTriangle {
public:
    FuzzyTriangle(double begin, double end) :
    begin(begin),
    end(end),
    middle((end-begin) / 2)
    {}
    double get_percentage(double fan_height) {
        if (fan_height > end || fan_height < begin) {
            return 0;
        }
        if (fan_height > middle) {
            double delta = (100/(middle - end));
            return 100 + ((fan_height - middle) * delta);
        } else {
            double delta = (100/(middle - begin));
            return 0 + ((fan_height - middle) * delta);
        }
    }
private:
    double begin, end, middle;
}

//Fuzzy variable sets
FuzzyTriangle low = FuzzyTriangle(3, 11);
FuzzyTriangle middle = FuzzyTriangle(7, 15);
FuzzyTriangle height = FuzzyTriangle(11, 19);

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

void setup(){
   pinMode(MotoroutPin, OUTPUT);          // configure motor output pin
   TCCR2B = TCCR2B & 0b11111000 | 0x01;   // set the PWM frequency , do not change!

   //configure tacho input to pin2, interrupt 0
   // Board       int.0  int.1
   // Uno, Ethernet 22 3
   pinMode(Tacho,INPUT_PULLUP);
   attachInterrupt(0, rpm_fan, RISING);

  Serial.begin(115200);                   // Open serial communications and wait for port to open:

}

/* this code will be executed every time the interrupt 0 (pin2) gets high.*/
void rpm_fan(){ half_revolutions++; }

// function that sends data with corresponding label to the serial port for visualisation in format of Megunolink
void sendPlotData(String seriesName, float data){
  Serial.print("{TIMEPLOT|data|");
  Serial.print(seriesName);
  Serial.print("|T|");
  Serial.print(data);
  Serial.println("}");
}

void loop(){
  if ((millis() - LastSample) > SampleTime){
    LastSample = millis(); //current time
    int val =  analogRead(HeightSensor);

    // the Adc value first needs to  be mapped from 0..1023 to 0..5v.
    double HeightSensorVoltage = (5.0/1023.0) * (double) val;

    ActualHeightSensor  = 16.4 / (HeightSensorVoltage + 0.184) - 0.42; // in cm

    //from the height of the sensor, compute the height of the fan (will be more convenient for control later on):
    ActualHeightFan     = 20 - ActualHeightSensor;

    // check the diffrence between the actual heigh and the desired heght to v
    float Error = DesiredHeightFan - ActualHeightFan;

    float rpm = 0.15 * (30000* half_revolutions /(SampleTime)) + 0.85 * rpm; // Digital Low pass filter with wcutoff ~ 1 hz.
    half_revolutions = 0;

    //write PWM to MotoroutPin to set the fan speed.
    //analogWrite(MotoroutPin,255);
    analogWrite(MotoroutPin,Motorout);

    sendPlotData("ActualHeightFan",ActualHeightFan);
    sendPlotData("Motorout",Motorout);
    sendPlotData("Error",Error);
  }
}
