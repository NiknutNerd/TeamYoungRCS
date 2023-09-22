#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

const int I2CDataPin = 0;
const int I2CClockPin = 1;

const int GPSResetPin = 11;

const int UARTTXPin = 12;
const int UARTRXPin = 13;

const int solenoidAPin = 2;
const int solenoidBPin = 3;

const int servoFeedbackPin = 4;
const int servoPWMPin = 5;

const int regulator6VPin = 10;

const int powerGood6VPin = 14;
const int powerGood3VPin = 15;

const int switchPin = A0;

const int buzzerPin = 17;

const int brightLEDPin = 9;
const int debugLED1 = 18;
const int debugLED2 = 19;
const int debugLED3 = 20;
const int debugLED4 = 21;

unsigned long currentTime = millis();
int loops;

class Timer{
  private:
    unsigned long startTime;
    unsigned long timeSinceStart;
  public:
    Timer(unsigned long initTime){
      startTime = initTime;
    }
    unsigned long getTime(){
      unsigned long timeSinceStart = currentTime - startTime;
      return timeSinceStart;
    }
    void resetTime(){
      startTime = currentTime;
      timeSinceStart = 0;
    }
};

Timer LEDTimer(currentTime);
//Timer testTimer(currentTime);
String color = "None";
int switchState;
int lastSwitchState = switchState;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(brightLEDPin, OUTPUT);
  pinMode(debugLED1, OUTPUT);
  pinMode(debugLED2, OUTPUT);
  pinMode(debugLED3, OUTPUT);
  pinMode(debugLED4, OUTPUT);

  pinMode(switchPin, INPUT);
  currentTime = millis();
  LEDTimer.resetTime();
  //testTimer.resetTime();
  loops = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  currentTime = millis(); 
  String lastColor = color;
  loops++;
  switchState = digitalRead(switchPin);
  if(switchState == HIGH && switchState != lastSwitchState){
    LEDTimer.resetTime();
  }
  if(true){
    if(LEDTimer.getTime() < 1000){
      digitalWrite(debugLED1, HIGH);
      color = "Red";
    }else if(LEDTimer.getTime() < 2000){
      digitalWrite(debugLED1, LOW);
      digitalWrite(debugLED2, HIGH);
      color = "Green";
    }else if(LEDTimer.getTime() < 3000){
      digitalWrite(debugLED2, LOW);
      digitalWrite(debugLED3, HIGH);
      color = "Blue";
    }else if(LEDTimer.getTime() < 4000){
      digitalWrite(debugLED3, LOW);
      digitalWrite(debugLED4, HIGH);
      color = "White";
    }else{
      digitalWrite(debugLED1, LOW);
      digitalWrite(debugLED2, LOW);
      digitalWrite(debugLED3, LOW);
      digitalWrite(debugLED4, LOW);
      color = "\n";
      LEDTimer.resetTime();
    } 
  }else{
    digitalWrite(debugLED1, LOW);
      digitalWrite(debugLED2, LOW);
      digitalWrite(debugLED3, LOW);
      digitalWrite(debugLED4, LOW);
      color = "\n";
  }
  
  if(!color.equals(lastColor)){
    //Serial.print(loops);
    loops = 0;
    Serial.print(color);
    Serial.print(switchState);
  }
  //Serial.print(switchState);
  if(switchState != lastSwitchState){
    Serial.print(switchState);
  } 
  lastSwitchState = switchState;
}