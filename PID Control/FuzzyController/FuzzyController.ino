#include <Arduino.h>

// Quadcopter workshop for HU by Alten 18-03-2015
#include "PID_v1.h"

enum HeightState {
  L,
  M,
  H,
};

class FuzzyTriangle {
public:
    FuzzyTriangle(double begin, double end) :
    begin(begin),
    end(end),
    middle(((end-begin) / 2) + begin)
    {}
    double get_percentage(double fan_height) {
        double delta;
      
        if (fan_height > middle) 
             delta = (100/(middle - end));
        else
             delta = (100/(middle + begin));

        float returnPer = 100 + ((fan_height - middle) * delta);
        if(returnPer < 0)
          return 0;
          
        return returnPer;
    }
private:
    double begin, end, middle;
};

//Fuzzy variable set
FuzzyTriangle low = FuzzyTriangle(1, 9);
FuzzyTriangle middle = FuzzyTriangle(5, 11);
FuzzyTriangle high = FuzzyTriangle(7, 15);

double prev = 0;
double time_elapsed = 1000;

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
   // Board int.0  int.1
   // Uno, Ethernet 22 3
   pinMode(Tacho,INPUT_PULLUP);
   attachInterrupt(0, rpm_fan, RISING);

  Serial.begin(115200);                   // Open serial communications and wait for port to open:

}

/* this code will be executed every time the interrupt 0 (pin2) gets high.*/
void rpm_fan(){ half_revolutions++; }

void loop(){
  int delta_time = millis() - LastSample;
  if (delta_time > SampleTime){
    LastSample = millis(); //current time
    int val =  analogRead(HeightSensor);

    // the Adc value first needs to  be mapped from 0..1023 to 0..5v.
    double HeightSensorVoltage = (5.0/1023.0) * (double) val;

    ActualHeightSensor  = 16.4 / (HeightSensorVoltage + 0.184) - 0.42; // in cm

    //from the height of the sensor, compute the height of the fan (will be more convenient for control later on):
    ActualHeightFan     = 20 - ActualHeightSensor;

    //write the fan height to the screen:
   // Serial.println(ActualHeightFan);
      Serial.print("Low : "); Serial.print(low.get_percentage(ActualHeightSensor)); Serial.println("%");
      Serial.print("Middle : "); Serial.print(middle.get_percentage(ActualHeightSensor)); Serial.println("%");
      Serial.print("High : "); Serial.print(high.get_percentage(ActualHeightSensor)); Serial.println("%");
      Serial.println();
      
      double low_value = low.get_percentage(ActualHeightSensor);
      double mid_value = middle.get_percentage(ActualHeightSensor);
      double high_value = high.get_percentage(ActualHeightSensor);

      HeightState highest = HeightState::L;
      HeightState second = HeightState::L;
      
      if (mid_value > high_value && mid_value > low_value) {
        highest = HeightState::M;
      }
      else if (high_value > mid_value && high_value > low_value) {
        highest = HeightState::H;
      }


      //Second
      if (mid_value > high_value && high_value > low_value) {
        second = HeightState::H;
      }
      if (low_value > high_value && high_value > mid_value) {
        second = HeightState::H;
      }

      //Middle
      if (high_value > mid_value && mid_value > low_value) {
        second = HeightState::M;
      }
      if (low_value > mid_value && mid_value > high_value) {
        second = HeightState::M;
      }

      //Low
      if (mid_value > low_value && low_value > high_value) {
        second = HeightState::L;
      }
      if (high_value > low_value && low_value > mid_value) {
        second = HeightState::L;
      }


      if (highest == HeightState::M) {
        if (mid_value > 95) {
          analogWrite(MotoroutPin,175);
        }
        else if (second == HeightState::L) {
          analogWrite(MotoroutPin,145);
        }
        else {
          analogWrite(MotoroutPin,240);
        }
      }
      else if (highest == HeightState::L) {
          analogWrite(MotoroutPin, 100);
      }
      else {
        analogWrite(MotoroutPin, 255);
      }

      Serial.print("Highest: "); Serial.print(highest); Serial.println("");
      Serial.print("Second : "); Serial.print(second); Serial.println("");
      Serial.println();
 
      
      
    //  delay(1000);
      prev = ActualHeightSensor;
      
    // check the diffrence between the actual heigh and the desired heght to v
    float Error = DesiredHeightFan - ActualHeightFan;

    float rpm = 0.15 * (30000* half_revolutions /(SampleTime)) + 0.85 * rpm; // Digital Low pass filter with wcutoff ~ 1 hz.
    half_revolutions = 0;

    //write PWM to MotoroutPin to set the fan speed.
    //analogWrite(MotoroutPin,1.1);
    //analogWrite(MotoroutPin,Motorout);

    //sendPlotData("ActualHeightFan",ActualHeightFan);
    //sendPlotData("Motorout",Motorout);
    //sendPlotData("Error",Error);
  }
}
