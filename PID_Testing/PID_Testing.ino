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
float okp = 0.15;
float oi = 0.0;
float oki = 0.0;
float od = 0.0;
float okd = 0.0;
float oPIDOutput;

float vPIDError;
float vLastTarget = 0.0;
float vp = 0.0;
float vkp = 0.01;
//float vkp = 0.02;
float vi = 0.0;
float vki = 0.00001;
float vd = 0.0;
float vkd = 0.00;
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

    bool isDone(){
      if((targetTime - (long)millis()) <= 0){
        return true;
      }else{
        return false;
      }
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
  if(abs(oPIDError) < 5){
    return 0;
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
  
  //return (-1) * oPIDOutput;
  return oPIDOutput;
}

float vPID(float target){
  imuStuff();
  float current = gyro.z();
  vPIDError = (target - current);
  if(abs(vPIDError) < 5){
    return 0;
  }
  //vPIDError = (-1) * (target - current);
  vd = (vPIDError - vp) / vPIDTimer.getTime();
  vi = vi + (vPIDError * vPIDTimer.getTime());
  vp = vPIDError;
  
  vPIDTimer.reset();
  //add way to reset i if target is changed
  
  if(target != vLastTarget){
    vi = 0;
  }
  
  if(vki * vi > .05){
    vi = 0;
  }else if(vki * vi < -.05){
    vi = 0;
  }
  vLastTarget = target;
  vPIDOutput = (vkp * vp) + (vki * vi) + (vkd * vd);
  
  //return (-1) * vPIDOutput;
  return vPIDOutput;
}


void PWMSetup(float percent){
  if(!aCountdown.isDone() || !bCountdown.isDone()){
    return;
  }
  if(abs(percent) < 0.075){
    aCountdown.changeTimer(0);
    aOnCountdown.changeTimer(0);
    bCountdown.changeTimer(0);
    bOnCountdown.changeTimer(0);
  }else if(percent > 0){
    if(percent > .9){
      onPercent = .9;
      offPercent = .1;
    }else{
      onPercent = percent;
      offPercent = 1 - percent;
    }

    if(onPercent >= offPercent){
      offTime = MIN_CYCLE_MILLIS;
      onTime = (onPercent / offPercent) * MIN_CYCLE_MILLIS;
    }else if(offPercent > onPercent){
      onTime = MIN_CYCLE_MILLIS;
      offTime = (offPercent / onPercent) * MIN_CYCLE_MILLIS;
    }

    aOnCountdown.changeTimer(onTime);
    aCountdown.changeTimer(onTime + offTime);
  }else if(percent < 0){
    if(abs(percent) > .9){
      onPercent = .9;
      offPercent = .1;
    }else{
      onPercent = percent;
      offPercent = 1 - percent;
    }

    if(onPercent >= offPercent){
      offTime = MIN_CYCLE_MILLIS;
      onTime = (onPercent / offPercent) * MIN_CYCLE_MILLIS;
    }else if(offPercent > onPercent){
      onTime = MIN_CYCLE_MILLIS;
      offTime = (offPercent / onPercent) * MIN_CYCLE_MILLIS;
    }

    bOnCountdown.changeTimer(onTime);
    bCountdown.changeTimer(onTime + offTime);
  }
}
void PWMLoop(){
  if(aOnCountdown.getTime() > 0){
    digitalWrite(SOLENOID_CW, HIGH);
    digitalWrite(SOLENOID_CCW, LOW);
  }else if(bOnCountdown.getTime() > 0){
    digitalWrite(SOLENOID_CCW, HIGH);
    digitalWrite(SOLENOID_CW, LOW);
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

  Serial1.begin(9600);

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

    Serial.println("Acceleration: ");
    Serial.print("X: ");
    Serial.println(accel.x());
    Serial.print("Y: ");
    Serial.println(accel.y());
    Serial.print("Z: ");
    Serial.println(accel.z());

    Serial.println("Linear Acceleration: ");
    Serial.print("X: ");
    Serial.println(linAccel.x());
    Serial.print("Y: ");
    Serial.println(linAccel.y());
    Serial.print("Z: ");
    Serial.println(linAccel.z());

    sensors_event_t event;
    bno.getEvent(&event);
    Serial.println("Heading: ");
    Serial.print(event.orientation.roll);

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
void loggerPrint(long frequency){
  orientation = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  if(printTimer.getTime() > frequency){
    //imuStuff();
    Serial1.println("Telemetry: ");

    /*
    Serial1.println("Acceleration: ");
    Serial1.print("X: ");
    Serial1.println(accel.x());
    Serial1.print("Y: ");
    Serial1.println(accel.y());
    Serial1.print("Z: ");
    Serial1.println(accel.z());

    Serial1.println("Linear Acceleration: ");
    Serial1.print("X: ");
    Serial1.println(linAccel.x());
    Serial1.print("Y: ");
    Serial1.println(linAccel.y());
    Serial1.print("Z: ");
    Serial1.println(linAccel.z());
    
    sensors_event_t event;
    bno.getEvent(&event);
    Serial1.println("Heading: ");
    Serial1.print(event.orientation.roll);
    */

    Serial1.println("Orientation: ");
    Serial1.print("X: ");
    Serial1.print(orientation.x());
    Serial1.println(" degrees");

    /*
    Serial1.println("Gyroscope Degrees: ");
    Serial1.print("Z: ");
    Serial1.print(gyro.z());
    Serial1.println(" degrees per second");
    */

    Serial1.println("vPID STUFFS: ");
    Serial1.print("oPIDError: ");
    Serial1.println(oPIDError);

    Serial1.print("oPIDOutput: ");
    Serial1.println(oPIDOutput);
    Serial1.print("Gyro");
    Serial1.println(gyro.z());
    
    Serial1.print("vPIDOutput: ");
    Serial1.println(vPIDOutput);
    Serial1.print("vPIDError: ");
    Serial1.println(vPIDError);
    Serial1.print("PWM Input Being Used: ");
    Serial1.println(inputBeingUsed);
    printTimer.reset();
  }
}
void loop() {
  imuStuff();
  //loggerPrint(1000);

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
    
  }
  /*
  if(solenoidTimer.getTime() < 10000){
    PWMSetup(vPID(15));
  }else if(solenoidTimer.getTime() < 20000){
    PWMSetup(vPID(-10));
  }
  */
  PWMSetup(vPID(oPID(90)));
  //PWMSetup(vPID(0));
  //PWMSetup(.5);
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