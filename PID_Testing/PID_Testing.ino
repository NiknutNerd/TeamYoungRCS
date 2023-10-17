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

const int DEBUG_LED_1 = 8;
const int DEBUG_LED_2 = 9;

const int SOLENOID_CW = 21;
const int SOLENOID_CCW = 20;

const double MIN_CYCLE = 1.0/25.0;
const double MIN_CYCLE_MILLIS = MIN_CYCLE * 1000;
double onPercent;
double offPercent;
long onTime;
long offTime;

int switchState;

//PID Variables
float oPIDError;
float oLastTarget = 0.0;
float op = 0.0;
float okp = 0.1;
float oi = 0.0;
float oki = 0.0;
float od = 0.0;
float okd = 0.0;
float oPIDOutput;

float vPIDError;
float vLastTarget = 0.0;
float vp = 0.0;
float vkp = 0.035;
float vi = 0.0;
float vki = 0.0;
float vd = 0.0;
float vkd = 0.0;
float vPIDOutput;

float inputBeingUsed;

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
    void reset(){
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
    long getTime(){
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
Timer oPIDTimer;
Timer vPIDTimer;
Timer solenoidTimer;

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

float oPID(float target){
  imuStuff();
  float current = orientation.x();
  if((target - current) > 180){
    oPIDError = (target - current) - 360;
  }else if((target - current) < -180){
    oPIDError = (target - current) + 360;
  }else{
    oPIDError = target - current;
  }
  od = (oPIDError - op) / oPIDTimer.getTime();
  oi = oi + (oPIDError * oPIDTimer.getTime());
  op = oPIDError;
  
  oPIDTimer.reset();
  //add way to reset i if target is changed
  if(target != oLastTarget){
    oi = 0;
  }
  if(oki * oi > .2){
    oi = .2 / oki;
  }else if(oki * oi < -.2){
    oi = (-1) * (.2 / oki);
  }
  oPIDOutput = (okp * op) + (oki * oi) + (okd * od);
  oLastTarget = target;
  
  return (-1) * oPIDOutput;
}

float vPID(float target){
  imuStuff();
  float current = gyro.z();
  vPIDError = (current - target);
  //vPIDError = (-1) * (target - current);
  vd = (vPIDError - vp) / vPIDTimer.getTime();
  vi = vi + (vPIDError * vPIDTimer.getTime());
  vp = vPIDError;
  
  vPIDTimer.reset();
  //add way to reset i if target is changed
  
  if(target != vLastTarget){
    vi = 0;
  }
  
  if(vki * vi > .2){
    vi = .2 / vki;
  }else if(vki * vi < -.2){
    vi = (-1) * (.2 / vki);
  }
  vLastTarget = target;
  vPIDOutput = (vkp * vp) + (vki * vi) + (vkd * vd);
  
  return vPIDOutput;
}


void PWMSetup(double percent){
  if(aCountdown.getTime() > 0 || bCountdown.getTime() > 0){
    return;
  }
  inputBeingUsed = percent;
  if(percent < .075 && percent > -.075){
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
    if(percent > .8){
      onPercent = .8;
      offPercent = .2;
    }
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
    digitalWrite(SOLENOID_CCW, LOW);
    
    onPercent = abs(percent);
    offPercent = 1 - abs(percent);
    if(abs(percent) > .8){
      onPercent = .8;
      offPercent = .1;
    }
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
  }
}
void PWMLoop(){
  if(aOnCountdown.getTime() > 0){
    digitalWrite(SOLENOID_CW, HIGH);
    digitalWrite(SOLENOID_CCW, LOW);
  }else if(aOffCountdown.getTime() > 0){
    digitalWrite(SOLENOID_CW, LOW);
  }else if(bOnCountdown.getTime() > 0){
    digitalWrite(SOLENOID_CCW, HIGH);
    digitalWrite(SOLENOID_CW, LOW);
  }else if(bOffCountdown.getTime() > 0){
    digitalWrite(SOLENOID_CCW, LOW);
  }else{
    digitalWrite(SOLENOID_CW, LOW);
    digitalWrite(SOLENOID_CCW, LOW);
  }
}

void setup() {
  pinMode(SOLENOID_CW, OUTPUT);
  pinMode(SOLENOID_CCW, OUTPUT);
  pinMode(DEBUG_LED_1, OUTPUT);
  pinMode(DEBUG_LED_2, OUTPUT);
  pinMode(SWITCH_PIN, INPUT);

  switchState = digitalRead(SWITCH_PIN);
  if(switchState == HIGH){
    Serial.begin(115200);
    while(!Serial){
    }
    Serial.println("PID Program Starting");
    Serial.println("Starting Sensors");
  }

  while(!bno.begin()){
  }

  uint8_t system, gyroo, accel, mag = 0;
  bno.setExtCrystalUse(true);

  while(gyroo < 3 && mag < 3){
    bno.getCalibration(&system, &gyroo, &accel, &mag);
    digitalWrite(DEBUG_LED_2, HIGH);
  }
  digitalWrite(DEBUG_LED_2, LOW);
  imuStuff();
  gyro.toDegrees();

  printTimer.reset();
  oPIDTimer.reset();
  vPIDTimer.reset();
  solenoidTimer.reset();
}


void limitedPrint(long frequency){
  orientation = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  if(printTimer.getTime() > frequency){
    //imuStuff();
    Serial.println("Telemetry: ");

    Serial.print("A Countdowns: ");
    Serial.print(aCountdown.getTime());
    Serial.print(aOnCountdown.getTime());
    Serial.println(aOffCountdown.getTime());

    Serial.print("B Countdowns: ");
    Serial.print(bCountdown.getTime());
    Serial.print(bOnCountdown.getTime());
    Serial.println(bOffCountdown.getTime());

    Serial.println("Orientation: ");
    Serial.print("X: ");
    Serial.print(orientation.x());
    Serial.println(" degrees");
    Serial.print("Y: ");
    Serial.print(orientation.y());
    Serial.println(" degrees");
    Serial.print("Z: ");
    Serial.print(orientation.z());
    Serial.println(" degrees");

    Serial.println("Gyroscope Degrees: ");
    Serial.print("X: ");
    Serial.print(gyro.x());
    Serial.println(" degrees per second");
    Serial.print("Y: ");
    Serial.print(gyro.y());
    Serial.println(" degrees per second");
    Serial.print("Z: ");
    Serial.print(gyro.z());
    Serial.println(" degrees per second");

    Serial.println("vPID STUFFS: ");
    Serial.print("oPIDOutput: ");
    Serial.println(oPIDOutput);
    Serial.print("oPIDError: ");
    Serial.println(oPIDError);
    Serial.print("vPIDOutput: ");
    Serial.println(vPIDOutput);
    Serial.print("vPIDError: ");
    Serial.println(vPIDError);
    Serial.print("PWM Input Being Used: ");
    Serial.println(inputBeingUsed);
    printTimer.reset();
  }
}
void loop() {
  imuStuff();
  /*
  if(gyro.z() > 10.0){
    float input = (.02) * gyro.z();
    if(input > .75){
      input = .75;
    }
    PWMSetup(input);
  }else if(gyro.z() < -10.0){
    float input = (.02) * gyro.z();
    if(input < -.75){
      input = -.75;
    }
    PWMSetup(input);
  }
  */
  
  switchState = digitalRead(SWITCH_PIN);
  if(switchState == HIGH){
    limitedPrint(1000);
  }
  /*
  if(solenoidTimer.getTime() < 10000){
    PWMSetup(vPID(15));
  }else if(solenoidTimer.getTime() < 20000){
    PWMSetup(vPID(-10));
  }
  */
  PWMSetup(vPID(oPID(90)));
  PWMLoop();
  
}

void loop1(){
  //limitedPrint(1000);
  /*
  switchState = digitalRead(SWITCH_PIN);
  if(switchState == HIGH){
    limitedPrint(1000);
  }
  */
}