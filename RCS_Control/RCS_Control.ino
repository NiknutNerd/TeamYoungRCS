#include <math.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME680.h>

/*
#define BME_SCK 13;
#define BME_MISO 12;
#define BME_MOSI 11;
#define BME_CS 10;

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme;
*/

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

const int switchPin = 16;

const int buzzerPin = 17;

const int brightLEDPin = 9;
const int debugLED1 = 18;
const int debugLED2 = 19;
const int debugLED3 = 20;
const int debugLED4 = 21;

const double minCycleTime = 1.0/20.0;
const double minCycleTimeMillis = minCycleTime * 1000;

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
Timer bmeTimer(currentTime);
Timer solenoidTimer(currentTime);
//Timer testTimer(currentTime);

String color = "None";
int switchState;
int lastSwitchState = switchState;
int solenoidState = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //BME Stuff
  /*
  while(!Serial);
  if(!bme.begin()){
    Serial.println(F("Can't find BME"));
    while(1);
  }
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2x);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);
  */

  pinMode(brightLEDPin, OUTPUT);
  pinMode(debugLED1, OUTPUT);
  pinMode(debugLED2, OUTPUT);
  pinMode(debugLED3, OUTPUT);
  pinMode(debugLED4, OUTPUT);

  pinMode(solenoidAPin, OUTPUT);
  pinMode(solenoidBPin, OUTPUT);

  pinMode(switchPin, INPUT);

  currentTime = millis();
  LEDTimer.resetTime();
  bmeTimer.resetTime();
  solenoidTimer.resetTime();
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
    //Serial.print(switchState);
    //Serial.print(solenoidState);
  }
  //Serial.print(switchState);
  if(switchState != lastSwitchState){
    Serial.print(switchState);
  } 
  lastSwitchState = switchState;

  if(false){
    if(solenoidTimer.getTime() < 50){
      digitalWrite(solenoidAPin, HIGH);
      digitalWrite(solenoidBPin, LOW);
      solenoidState = 0;
    }else if(solenoidTimer.getTime() < 100){
      digitalWrite(solenoidAPin, LOW);
      digitalWrite(solenoidBPin, HIGH);
      solenoidState = 1;
    }
    /*
    else if(solenoidTimer.getTime() < 150){
      digitalWrite(solenoidBPin, HIGH);
      digitalWrite(solenoidAPin, LOW);
      solenoidState = 2;
    }
    */
    else{
      solenoidTimer.resetTime();
    }
  }

  //BME Stuff
  /*
  if(! bme.performReading()){
    Serial.println("Reading Failed");
    return;
  }
  if(bmeTimer.getTime() > 1000){
    Serial.print("Temperature: ");
    Serial.println(bme.temperature);
    Serial.print("Pressure: ");
    Serial.println(bme.pressure / 100.0);
    Serial.print("Humidity: ");
    Serial.println(bme.humidity);
    Serial.print("Gas: ");
    Serial.println(bme.gas_resistance / 1000.0);
    Serial.print("Approx. Altitude");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");
    Serial.println();
    bmeTimer.resetTime();
  }
  */
}