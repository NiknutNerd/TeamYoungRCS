#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);
sensors_event_t event;

const int I2C_DATA = 4;
const int I2C_CLOCK = 5;

const int GPS_RESET = 11;

const int UART_TX = 12;
const int UART_RX = 13;

const int SOLENOID_CW = 18;
const int SOLENOID_CCW = 19;

//const int SERVO_FEEDBACK = 4;
//const int SERVO_PWM = 5;

const int REGULATOR_6V = 10;

const int POWER_GOOD_6V = 14;
const int POWER_GOOD_3V = 15;

const int SWITCH_PIN = 16;

const int BUZZER_PIN = 17;

const int BRIGHT_LED = 9;
const int DEBUG_LED_1 = 18;
const int DEBUG_LED_2 = 19;
const int DEBUG_LED_3 = 20;
const int DEBUG_LED_4 = 21;

const double MIN_CYCLE_TIME = 1.0/20.0;
const double MIN_CYCLE_TIME_MILLIS = MIN_CYCLE_TIME * 1000;
double onPercent;
double offPercent;
long onTime;
long offTime;

long currentTime = millis();
int loops;

class Timer{
  private:
    long startTime;
    long timeSinceStart;
  public:
    Timer(long initTime){
      startTime = initTime;
    }
    long getTime(){
      long timeSinceStart = currentTime - startTime;
      return timeSinceStart;
    }
    void resetTime(){
      startTime = currentTime;
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
      targetTime = currentTime + time; 
    }
    long getTimeLeft(){
      long timeLeft = targetTime - currentTime;
      if(timeLeft < 0){
        timeLeft = -1;
      }
      return timeLeft;
    }
    void changeTimer(long newTime){
      initialTime = newTime;
      targetTime = currentTime + newTime;
    }
};

Timer LEDTimer(currentTime);
Timer bmeTimer(currentTime);
Timer solenoidTimer(currentTime);
Timer PWMTimer(currentTime);
Timer printTimer(currentTime);
//Timer testTimer(currentTime);

CountdownTimer aCountdown(0);
CountdownTimer bCountdown(0);
CountdownTimer aOnCountdown(0);
CountdownTimer aOffCountdown(0);
CountdownTimer bOnCountdown(0);
CountdownTimer bOffCountdown(0);
CountdownTimer testCountdown(0);

String color = "None";
int switchState;
int lastSwitchState = switchState;
int solenoidState = 0;

void PWMSetup(double percent){
  if(percent > 0){
    bCountdown.changeTimer(0);
    bOnCountdown.changeTimer(0);
    bOffCountdown.changeTimer(0);

    onPercent = percent;
    offPercent = 1 - percent;
    if(onPercent >= offPercent){
      offTime = MIN_CYCLE_TIME_MILLIS;
      onTime = (offPercent/onPercent) * MIN_CYCLE_TIME_MILLIS;
    }else if(offPercent > onPercent){
      //Change to else
      onTime = MIN_CYCLE_TIME_MILLIS;
      offTime = (onPercent/offPercent) * MIN_CYCLE_TIME_MILLIS;
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
      offTime = MIN_CYCLE_TIME_MILLIS;
      onTime = (offPercent/onPercent) * MIN_CYCLE_TIME_MILLIS;
    }else if(offPercent > onPercent){
      onTime = MIN_CYCLE_TIME_MILLIS;
      offTime = (onPercent/offPercent) * MIN_CYCLE_TIME_MILLIS;
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
  Serial.println("Control Program Starting");

  //IMU Setup
  if(!bno.begin()){
    Serial.println("No IMU Detected");
    delay(5000);
  }
  //Makes IMU more accurate
  bno.setExtCrystalUse(true);

  //Define all pin modes
  pinMode(BRIGHT_LED, OUTPUT);
  pinMode(DEBUG_LED_1, OUTPUT);
  pinMode(DEBUG_LED_2, OUTPUT);
  pinMode(DEBUG_LED_3, OUTPUT);
  pinMode(DEBUG_LED_4, OUTPUT);

  pinMode(SOLENOID_CW, OUTPUT);
  pinMode(SOLENOID_CCW, OUTPUT);

  pinMode(SWITCH_PIN, INPUT);

  //Create Timers
  currentTime = millis();
  LEDTimer.resetTime();
  bmeTimer.resetTime();
  solenoidTimer.resetTime();
  PWMTimer.resetTime();
  printTimer.resetTime();
  //testTimer.resetTime();
  testCountdown.changeTimer(5000);
  Serial.println(testCountdown.getTimeLeft());
  loops = 0;
}

//Functions Here
void limitedPrint(long frequency){
  if(printTimer.getTime() > frequency){
    Serial.println("Telemetry: ");
    Serial.print("A Countdowns: ");
    Serial.print(aCountdown.getTimeLeft());
    Serial.print(aOnCountdown.getTimeLeft());
    Serial.println(aOffCountdown.getTimeLeft());

    Serial.print("B Countdowns: ");
    Serial.print(bCountdown.getTimeLeft());
    Serial.print(bOnCountdown.getTimeLeft());
    Serial.println(bOffCountdown.getTimeLeft());
    Serial.println("");

    Serial.print("IMU X: ");
    Serial.println(event.orientation.x);
    Serial.print("IMU Y: ");
    Serial.println(event.orientation.y);
    Serial.print("IMU Z: ");
    Serial.println(event.orientation.z);
    Serial.println("");

    Serial.print("Color: ");
    Serial.println(color);
    Serial.println("");

    Serial.print("Switch State: ");
    Serial.println(switchState);
    Serial.println("");

    printTimer.resetTime();
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  limitedPrint(500);

  //IMU Loop Setup
  //sensors_event_t event;
  bno.getEvent(&event);

  currentTime = millis(); 
  String lastColor = color;
  loops++;
  switchState = digitalRead(SWITCH_PIN);
  if(switchState == HIGH && switchState != lastSwitchState){
    LEDTimer.resetTime();
  }

  //5 second PWM 2 solenoid / LED test
  if(true){
    if(solenoidTimer.getTime() < 1000){
      if(aCountdown.getTimeLeft() <= 0 && bCountdown.getTimeLeft() <= 0){
        PWMSetup(.75);
      }
    }else if(solenoidTimer.getTime() < 2000){
      if(aCountdown.getTimeLeft() <= 0 && bCountdown.getTimeLeft() <= 0){
        PWMSetup(.5);
      }
    }else if(solenoidTimer.getTime() < 3000){
      if(aCountdown.getTimeLeft() < 0 && bCountdown.getTimeLeft() < 0){
        PWMSetup(-.75);
      }
    }else if(solenoidTimer.getTime() < 4000){
      if(aCountdown.getTimeLeft() < 0 && bCountdown.getTimeLeft() < 0){
        PWMSetup(-.5);
      }
    }else if(solenoidTimer.getTime() < 5000){
      if(aCountdown.getTimeLeft() < 0 && bCountdown.getTimeLeft() < 0){
        PWMSetup(0.0);
      }
    }else{
      solenoidTimer.resetTime();
    }
    PWMLoop();
  }

  //LED Cycler
  if(false){
    if(LEDTimer.getTime() < 1000){
      digitalWrite(DEBUG_LED_1, HIGH);
      color = "Red";
    }else if(LEDTimer.getTime() < 2000){
      digitalWrite(DEBUG_LED_1, LOW);
      digitalWrite(DEBUG_LED_2, HIGH);
      color = "Green";
    }else if(LEDTimer.getTime() < 3000){
      digitalWrite(DEBUG_LED_2, LOW);
      digitalWrite(DEBUG_LED_3, HIGH);
      color = "Blue";
    }else if(LEDTimer.getTime() < 4000){
      digitalWrite(DEBUG_LED_3, LOW);
      digitalWrite(DEBUG_LED_4, HIGH);
      color = "White";
    }else{
      digitalWrite(DEBUG_LED_1, LOW);
      digitalWrite(DEBUG_LED_2, LOW);
      digitalWrite(DEBUG_LED_3, LOW);
      digitalWrite(DEBUG_LED_4, LOW);
      color = "\n";
      LEDTimer.resetTime();
    } 
  }else{
    digitalWrite(DEBUG_LED_1, LOW);
      digitalWrite(DEBUG_LED_2, LOW);
      digitalWrite(DEBUG_LED_3, LOW);
      digitalWrite(DEBUG_LED_4, LOW);
      color = "\n";
  }

  lastSwitchState = switchState;
}