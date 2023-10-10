#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);
imu::Vector<3> orientation;
imu::Vector<3> gyro;
imu::Vector<3> accel;
imu::Vector<3> linAccel;

const int I2C_DATA = 4;
const int I2C_CLOCK = 5;

const int SWITCH_PIN = 6;

const int SOLENOID_CW = 20;
const int SOLENOID_CCW = 21;

const double MIN_CYCLE = 1.0/10.0;
const double MIN_CYCLE_MILLIS = MIN_CYCLE * 1000;
double onPercent;
double offPercent;
long onTime;
long offTime;

int switchState;

//PID Variables
float pidError;
float p;
float kp;
float i;
float ki;
float d;
float kd;

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

Timer printTimer;

CountdownTimer aCountdown(0);
CountdownTimer bCountdown(0);
CountdownTimer aOnCountdown(0);
CountdownTimer aOffCountdown(0);
CountdownTimer bOnCountdown(0);
CountdownTimer bOffCountdown(0);

void imuStuff(){
  orientation = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
}
/*
float PID(float target){
  //pidError = target - 
}
*/

void PWMSetup(double percent){
  if(aCountdown.getTimeLeft() > 0 || bCountdown.getTimeLeft() > 0){
    return;
  }
  if(percent < .05 && percent > -.05){
    aCountdown.changeTimer(0);
    aOnCountdown.changeTimer(0);
    aOffCountdown.changeTimer(0);

    bCountdown.changeTimer(0);
    bOnCountdown.changeTimer(0);
    bOffCountdown.changeTimer(0);
  }else if(percent > 0){
    bCountdown.changeTimer(0);
    bOnCountdown.changeTimer(0);
    bOffCountdown.changeTimer(0);
    digitalWrite(SOLENOID_CW, LOW);

    onPercent = percent;
    offPercent = 1 - percent;
    if(onPercent > 0.9){
      onTime = MIN_CYCLE_MILLIS;
      offTime = 0;
    }else if(onPercent >= offPercent){
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
    digitalWrite(SOLENOID_CCW, LOW);
    
    onPercent = abs(percent);
    offPercent = 1 - abs(percent);
    if(onPercent > 0.9){
      onTime = MIN_CYCLE_MILLIS;
      offTime = 0;
    }else if(onPercent >= offPercent){
      offTime = MIN_CYCLE_MILLIS;
      onTime = (onPercent/offPercent) * MIN_CYCLE_MILLIS;
    }else if(offPercent > onPercent){
      onTime = MIN_CYCLE_MILLIS;
      offTime = (offPercent/onPercent) * MIN_CYCLE_MILLIS;
    }
    bCountdown.changeTimer(onTime + offTime);
    bOnCountdown.changeTimer(onTime);
    bOffCountdown.changeTimer(onTime+offTime);
  }
}
void PWMLoop(){
  if(aOnCountdown.getTimeLeft() > 0){
    digitalWrite(SOLENOID_CW, HIGH);
    digitalWrite(SOLENOID_CCW, LOW);
  }else if(aOffCountdown.getTimeLeft() > 0){
    digitalWrite(SOLENOID_CW, LOW);
  }else if(bOnCountdown.getTimeLeft() > 0){
    digitalWrite(SOLENOID_CCW, HIGH);
    digitalWrite(SOLENOID_CW, LOW);
  }else if(bOffCountdown.getTimeLeft() > 0){
    digitalWrite(SOLENOID_CCW, LOW);
  }else{
    digitalWrite(SOLENOID_CW, LOW);
    digitalWrite(SOLENOID_CCW, LOW);
  }
}

void setup() {
  pinMode(SOLENOID_CW, OUTPUT);
  pinMode(SOLENOID_CCW, OUTPUT);
  pinMode(SWITCH_PIN, INPUT);

  switchState = digitalRead(SWITCH_PIN);
  if(switchState == HIGH){
    Serial.begin(9600);
    while(!Serial){
    }
    Serial.println("PID Program Starting");
    Serial.println("Starting Sensors");
  }

  while(!bno.begin()){
  }

  uint8_t system, gyro, accel, mag = 0;
  bno.setExtCrystalUse(true);
}


void limitedPrint(long frequency){
  if(printTimer.getTime() > frequency){
    imuStuff();
    Serial.println("Telemetry: ");

    Serial.print("A Countdowns: ");
    Serial.print(aCountdown.getTimeLeft());
    Serial.print(aOnCountdown.getTimeLeft());
    Serial.println(aOffCountdown.getTimeLeft());

    Serial.print("B Countdowns: ");
    Serial.print(bCountdown.getTimeLeft());
    Serial.print(bOnCountdown.getTimeLeft());
    Serial.println(bOffCountdown.getTimeLeft());

    Serial.println("Orientation: ");
    Serial.print("X: ");
    Serial.println(orientation.x());
    Serial.print("Y: ");
    Serial.println(orientation.y());
    Serial.print("Z: ");
    Serial.println(orientation.z());

    Serial.println("Gyroscope Radians: ");
    gyro.toRadians();
    Serial.print("X: ");
    Serial.println(gyro.x());
    Serial.print("Y: ");
    Serial.println(gyro.y());
    Serial.print("Z: ");
    Serial.println(gyro.z());

    Serial.println("Gyroscope Degrees: ");
    gyro.toDegrees();
    Serial.print("X: ");
    Serial.println(gyro.x());
    Serial.print("Y: ");
    Serial.println(gyro.y());
    Serial.print("Z: ");
    Serial.println(gyro.z());

    Serial.println("Acceleration: ");
    Serial.print("X: ");
    Serial.println(accel.x());
    Serial.print("Y: ");
    Serial.println(accel.y());
    Serial.print("Z: ");
    Serial.println(accel.z());

    Serial.println("Linear Acceleration:");
    Serial.print("X: ");
    Serial.println(linAccel.x());
    Serial.print("Y: ");
    Serial.println(linAccel.y());
    Serial.print("Z: ");
    Serial.println(linAccel.z());
    printTimer.resetTime();
  }
}
void loop() {
  switchState = digitalRead(SWITCH_PIN);
  if(switchState == HIGH){
    limitedPrint(1000);
  }
  //bno.getEvent(&event);

}
