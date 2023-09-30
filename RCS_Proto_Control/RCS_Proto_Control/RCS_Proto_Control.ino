#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME680.h>
#include <utility/imumaths.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_BME680 bme;
sensors_event_t event;

const int UART_TX = 0;
const int UART_RX = 1;

const int I2C_DATA = 4;
const int I2C_CLOCK = 5;

const int SWITCH_PIN = 6;

const int DEBUG_LED_1 = 8;
const int DEBUG_LED_2 = 9;
const int DEBUG_LED_3 = 25;
const int BRIGHT_LED = 19;

const int SENSOR_RESET = 11;
const int SERVO_FEEDBACK= 16;
const int SERVO_CONTROL = 17;

const int SOLENOID_CW = 20;
const int SOLENOID_CCW = 21;

const double MIN_CYCLE = 1.0/20.0;
const double MIN_CYCLE_MILLIS = MIN_CYCLE * 1000;
double onPercent;
double offPercent
long onTime;
long offTime;

double targetX;
double currentX;
double errorX;

class Timer{
  private:
    long startTime;
    long timeSinceStart;
  public:
    Timer(){
      startTime = (long)millis();
    }
    long getTime(){
      long timeSinceStart = (long)millis() - startTime;
      return timeSinceStart;
    }
    void resetTime(){
      startTime = (long)millis();
      timeSinceStart = 0;
    }
};

class CountdownTimer{
  private:
    long targetTime;
    long initialTime;
  public:
    CountdownTimer(long time){
      initialTime = time;
      targetTime = (long)millis() + time; 
    }
    long getTimeLeft(){
      long timeLeft = targetTime - (long)millis();
      if(timeLeft < 0){
        timeLeft = -1;
      }
      return timeLeft;
    }
    void changeTimer(long newTime){
      initialTime = newTime;
      targetTime = (long)millis() + newTime;
    }
};

Timer sensorSetup;
Timer printTimer;
Timer solenoidTimer;

CountdownTimer aCountdown(0);
CountdownTimer bCountdown(0);
CountdownTimer aOnCountdown(0);
CountdownTimer aOffCountdown(0);
CountdownTimer bOnCountdown(0);
CountdownTimer bOffCountdown(0);

void PWMSetup(double percent){
  if(percent > 0){
    bCountdown.changeTimer(0);
    bOnCountdown.changeTimer(0);
    bOffCountdown.changeTimer(0);

    onPercent = percent;
    offPercent = 1 - percent;
    if(onPercent >= offPercent){
      offTime = MIN_CYCLE_MILLIS;
      onTime = (onPercent/offPercent) * MIN_CYCLE_MILLIS;
    }else if(offPercent > onPercent){
      //Change to else
      onTime = MIN_CYCLE_MILLIS;
      offTime = (offPercent/onPercent) * MIN_CYCLE_MILLIS;
    }
    aCountdown.changeTimer(onTime + offTime);
    aOnCountdown.changeTimer(onTime);
    aOffCountdown.changeTimer(onTime+offTime);
  }else if(percent < 0){
    aCountdown.changeTimer(0);
    aOnCountdown.changeTimer(0);
    aOffCountdown.changeTimer(0);
    
    onPercent = abs(percent);
    offPercent = 1 - abs(percent);
    if(onPercent >= offPercent){
      offTime = MIN_CYCLE_MILLIS;
      onTime = (onPercent/offPercent) * MIN_CYCLE_MILLIS;
    }else if(offPercent > onPercent){
      onTime = MIN_CYCLE_MILLIS;
      offTime = (offPercent/onPercent) * MIN_CYCLE_MILLIS;
    }
    bCountdown.changeTimer(onTime + offTime);
    bOnCountdown.changeTimer(onTime);
    bOffCountdown.changeTimer(onTime+offTime);
  }else if(percent == 0){
    aCountdown.changeTimer(0);
    aOnCountdown.changeTimer(0);
    aOffCountdown.changeTimer(0);

    bCountdown.changeTimer(0);
    bOnCountdown.changeTimer(0);
    bOffCountdown.changeTimer(0);
  }
}
void PWMLoop(){
  if(aOnCountdown.getTimeLeft() > 0){
    digitalWrite(SOLENOID_CW, HIGH);
  }else if(aOffCountdown.getTimeLeft() > 0){
    digitalWrite(SOLENOID_CW, LOW);
  }else if(bOnCountdown.getTimeLeft() > 0){
    digitalWrite(SOLENOID_CCW, HIGH);
  }else if(bOffCountdown.getTimeLeft() > 0){
    digitalWrite(SOLENOID_CCW, LOW);
  }else{
    digitalWrite(SOLENOID_CW, LOW);
    digitalWrite(SOLENOID_CCW, LOW);
  }
}

void setup() {
  Serial.begin(9600);
  while(!Serial){
  }
  Serial.println("Control Program Starting");

  printTimer.resetTime();
  sensorSetup.resetTime();
  solenoidTimer.resetTime();

  Serial.println("Starting Sensors");

  while(sensorSetup.getTime() < 5000){
    
    if(bno.begin() && bme.begin()){
      break;
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
