#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Zanshin_BME680.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <utility/imumaths.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BNO055 bno = Adafruit_BNO055(55);
BME680_Class bme;
SFE_UBLOX_GNSS gps;

//Flight state enum
enum FlightState{
  INIT,
  FLIGHT_READY,
  ASCENT,
  CONTROLLED,
  FREEFALL,
  LANDED
};
//Initialize the flight state to init
FlightState flightState = INIT;
FlightState currentFlightState = flightState;

//Semaphores
bool switchSem = false;
bool brightLEDSem = false;
bool i2cSem = false;
bool missionTimeSem = false;
bool flightStateSem = false;
bool bmeAltSem = false;
bool bmeTempSem = false;
bool imuAccelSem = false;
bool imuGyroSem = false;
bool imuOrientSem = false;
bool gpsLatSem = false;
bool gpsLongSem = false;
bool gpsAltSem = false;

//Initializing raw data values for the imu
imu::Vector<3> orientation;
imu::Vector<3> gyro;
imu::Vector<3> accel;
imu::Vector<3> linAccel;

//Function to call to make sure the imu data is accessable
void imuStuff(){
  while(i2cSem);
  i2cSem = true;
  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  orientation = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  i2cSem = false;
}

//Assign pins for everything that is needed
const int SWITCH_PIN = 6;
const int SENSOR_RESET = 11;
const int DEBUG_LED_1 = 8;
const int DEBUG_LED_2 = 9;
const int DEBUG_LED_3 = 25;
const int BRIGHT_LED = 19;
const int SOLENOID_CCW = 20;
const int SOLENOID_CW = 21;

//Random Variables
float gpsAltitude;
float gpsLatitude;
float gpsLongitude;
float bmeAltitude;
int32_t bmeTemperature;
int32_t bmePressure;
int packetCount = 0;
int lastLoop = 0;
int lastLoop1 = 0;

float vPIDError;
float vLastTarget = 0.0;
float vp = 0.0;
float vkp = 0.035;
float vi = 0.0;
float vki = 0.0;
float vd = 0.0;
float vkd = 0.0;
float vPIDOutput;

//PWM Variables
const float MIN_CYCLE = 1.0/25.0;
const float MIN_CYCLE_MILLIS = MIN_CYCLE * 1000.0;

//TODO: Tune PID and put variables here

class Timer{
  private:
    long startTime;
    long timeSinceStart;
  public:
    //Constructor
    Timer(){
      startTime = (long)millis();
    }
    void reset(){
      startTime = (long)millis();
      timeSinceStart = 0;
    }
    long getTime(){
      timeSinceStart = (long)millis() - startTime;
      return timeSinceStart;
    }
    long getTimeMillis(){
      long milliseconds = (long)millis() - startTime;
      return milliseconds;
    }
    long getTimeSeconds(){
      long seconds = getTimeMillis() / 1000;
      return seconds;
    }
    long getTimeMinutes(){
      long minutes = getTimeMillis() / 60000;
      return minutes;
    }
    long getTimeHours(){
      long hours = getTimeMillis() / 3600000;
      return hours;
    }
    long timerHours(){
      return getTimeHours();
    }
    long timerMinutes(){
      long minutes = getTimeMinutes() % 60;
      return minutes;
    }
    long timerSeconds(){
      long seconds = getTimeSeconds() % 60;
      return seconds;
    }
    long timerMillis(){
      long millis = getTimeMillis() % 1000;
      return millis;
    }
    String printableTimer(){
      /*
      Returns a string of the time the timer has been running
      in the form hh:mm:ss.sss
      */
      String timer;
      if(timerHours() < 10){
        timer += "0";
      }
      timer += String(timerHours());
      timer += ":";

      if(timerMinutes() < 10){
        timer += "0";
      }
      timer += String(timerMinutes());
      timer += ":";

      if(timerSeconds() < 10){
        timer += "0";
      }
      timer += String(timerSeconds());
      timer += ".";

      if(timerMillis() < 100){
        timer += "0";
      }
      if(timerMillis() < 10){
        timer += "0";
      }
      timer += String(timerMillis());
      return timer;
    }
};

class CountdownTimer: public Timer{
  private:
    long startTime;
    long initTime;
    long targetTime;
    long timeLeft;
    long timeSinceStart;
  public:
    CountdownTimer(long time){
      startTime = (long)millis();
      initTime = time;
      targetTime = (long)millis() + time;
    }
    CountdownTimer(){
      startTime = (long)millis();
      initTime = 0;
      targetTime = (long)millis();
    }
    void reset(long newTime){
      startTime = (long)millis();
      initTime = newTime;
      targetTime = (long)millis() + newTime;
    }
    bool isDone(){
      if((targetTime - (long)millis()) <= 0){
        return true;
      }else{
        return false;
      }
    }
    long getTime(){
      timeLeft = targetTime - (long)millis();
      if(timeLeft < 0){
        timeLeft = -1;
      }
      return timeLeft;
    }
    long getTimeMillis(){
      long milliseconds = targetTime - (long)millis();
      if(timeLeft < 0){
        timeLeft = -1;
      }
      return timeLeft;
    }
};
//Timers
Timer missionTime;
Timer hazardTimer;
Timer errorTimer;
//Timer logTimer;
//Timer oPIDTimer;
Timer vPIDTimer;

//Countdowns
CountdownTimer sensorSetup;
//PWM Countdowns
CountdownTimer aOnCountdown;
CountdownTimer aCountdown;
CountdownTimer bOnCountdown;
CountdownTimer bCountdown;

float vPID(float target){
  imuStuff();
  while(i2cSem);
  i2cSem = true;
  float current = gyro.z();
  i2cSem = false;
  vPIDError = (current - target);
  if(abs(vPIDError) < 3){
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
  
  if(vki * vi > .2){
    vi = .2 / vki;
  }else if(vki * vi < -.2){
    vi = (-1) * (.2 / vki);
  }
  vLastTarget = target;
  vPIDOutput = (vkp * vp) + (vki * vi) + (vkd * vd);
  return (-1) * vPIDOutput;
}

void PWMSetup(float percent){
  //These variables are only used here so can be local
  float onPercent;
  float offPercent;
  long onTime;
  long offTime;
  if(!aCountdown.isDone() || !bCountdown.isDone()){
    return;
  }
  if(abs(percent) < 0.1){
    aCountdown.reset(0);
    aOnCountdown.reset(0);
    bCountdown.reset(0);
    bOnCountdown.reset(0);
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

    aOnCountdown.reset(onTime);
    aCountdown.reset(onTime + offTime);
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

    bOnCountdown.reset(onTime);
    bCountdown.reset(onTime + offTime);
  }
}

void PWMLoop(){
  if(!aOnCountdown.isDone()){
    digitalWrite(SOLENOID_CW, HIGH);
    digitalWrite(SOLENOID_CCW, LOW);
  }else if(!bOnCountdown.isDone()){
    digitalWrite(SOLENOID_CW, LOW);
    digitalWrite(SOLENOID_CCW, HIGH);
  }else{
    digitalWrite(SOLENOID_CW, LOW);
    digitalWrite(SOLENOID_CCW, LOW);
  }
}

float readAltitude(){
  float atmospheric = bmePressure / 100.0;
  return 44330.0 * (1.0 - pow(atmospheric / SEALEVELPRESSURE_HPA, 0.1903));
}

void errorCycle(long frequency){
  if(errorTimer.getTime() < frequency){
    digitalWrite(DEBUG_LED_1, HIGH);
    digitalWrite(DEBUG_LED_2, LOW);
    digitalWrite(DEBUG_LED_3, LOW);
    //digitalWrite(BRIGHT_LED, LOW);
  }else if(errorTimer.getTime() < (2 * frequency)){
    digitalWrite(DEBUG_LED_1, LOW);
    digitalWrite(DEBUG_LED_2, HIGH);
    digitalWrite(DEBUG_LED_3, LOW);
  }else if(errorTimer.getTime() < (3 * frequency)){
    digitalWrite(DEBUG_LED_1, LOW);
    digitalWrite(DEBUG_LED_2, LOW);
    digitalWrite(DEBUG_LED_3, HIGH);
  }else if(errorTimer.getTime() < (4 * frequency)){
    digitalWrite(DEBUG_LED_1, LOW);
    digitalWrite(DEBUG_LED_2, LOW);
    digitalWrite(DEBUG_LED_3, LOW);
    //digitalWrite(BRIGHT_LED, LOW);
  }else{
    errorTimer.reset();
  }
}

void loggerHeader(){
  Serial1.print("TEAM_ID,");
  Serial1.print("MISSION_TIME,");
  Serial1.print("PACKET_COUNT,");
  Serial1.print("SW_STATE,");
  Serial1.print("BME_ALTITUDE,");
  Serial1.print("BME_TEMP,");
  Serial1.print("ACC_X,");
  Serial1.print("ACC_Y,");
  Serial1.print("ACC_Z,");
  Serial1.print("GYRO_X,");
  Serial1.print("GYRO_Y,");
  Serial1.print("GYRO_Z,");
  Serial1.print("ORIENT_X,");
  Serial1.print("ORIENT_Y,");
  Serial1.print("ORIENT_Z,");
  Serial1.print("GPS_LAT,");
  Serial1.print("GPS_LONG,");
  Serial1.println("GPS_ALT,");
}

void loggerPrint(){
  packetCount++;
  Serial1.print("MOAB");
  Serial1.print(",");

  while(missionTimeSem);
  missionTimeSem = true;
  Serial1.print(missionTime.printableTimer());
  missionTimeSem = false;

  Serial1.print(",");
  Serial1.print(packetCount);
  Serial1.print(",");

  while(flightStateSem);
  flightStateSem = true;
  Serial1.print(flightState);
  flightStateSem = false;

  Serial1.print(",");

  int32_t bmeHumidity;
  int32_t bmeGas;

  while(i2cSem);
  i2cSem = true;
  bme.getSensorData(bmeTemperature, bmeHumidity, bmePressure, bmeGas, false);
  i2cSem = false;

  while(bmeAltSem);
  bmeAltSem = true;
  bmeAltitude = readAltitude();
  Serial1.print(bmeAltitude);
  bmeAltSem = false;

  Serial1.print(",");

  while(bmeTempSem);
  bmeTempSem = true;
  float bmeTemp = bmeTemperature / 100.0;
  Serial1.print(bmeTemp);
  bmeTempSem = false;

  Serial1.print(",");

  imuStuff();
  
  while(i2cSem);
  i2cSem = true;
  Serial1.print(accel.x());
  i2cSem = false;
  Serial1.print(",");
  while(i2cSem);
  i2cSem = true;
  Serial1.print(accel.y());
  i2cSem = false;
  Serial1.print(",");
  while(i2cSem);
  i2cSem = true;
  Serial1.print(accel.z());
  i2cSem = false;
  

  Serial1.print(",");

  while(i2cSem);
  i2cSem = true;
  Serial1.print(gyro.x());
  i2cSem = false;
  Serial1.print(",");
  while(i2cSem);
  i2cSem = true;
  Serial1.print(gyro.y());
  i2cSem = false;
  Serial1.print(",");
  while(i2cSem);
  i2cSem = true;
  Serial1.print(gyro.z());
  i2cSem = false;

  Serial1.print(",");

  while(i2cSem);
  i2cSem = true;
  Serial1.print(orientation.x());
  i2cSem = false;
  Serial1.print(",");
  while(i2cSem);
  i2cSem = true;
  Serial1.print(orientation.y());
  i2cSem = false;
  Serial1.print(",");
  while(i2cSem);
  i2cSem = true;
  Serial1.print(orientation.z());
  i2cSem = false;

  Serial1.print(",");

  while(gpsLatSem);
  gpsLatSem = true;
  while(i2cSem);
  i2cSem = true;
  gpsLatitude = (float)gps.getLatitude() * 0.0000001;
  i2cSem = false;
  Serial1.print(gpsLatitude);
  gpsLatSem = false;

  Serial1.print(",");

  while(gpsLongSem);
  gpsLongSem = true;
  while(i2cSem);
  i2cSem = true;
  gpsLongitude = (float)gps.getLongitude() * 0.0000001;
  i2cSem = false;
  Serial1.print(gpsLongitude);
  gpsLongSem = false;
  Serial1.print(",");

  while(gpsAltSem);
  gpsAltSem = true;
  while(i2cSem);
  i2cSem = true;
  gpsAltitude = (float)gps.getAltitudeMSL() / 1000.0;
  i2cSem = false;
  Serial1.println(gpsAltitude);
  gpsAltSem = false;
}

void setup() {
  //Set pinmode for all pins
  pinMode(SWITCH_PIN, INPUT);
  pinMode(SENSOR_RESET, OUTPUT);
  pinMode(DEBUG_LED_1, OUTPUT);
  pinMode(DEBUG_LED_2, OUTPUT);
  pinMode(DEBUG_LED_3, OUTPUT);
  pinMode(BRIGHT_LED, OUTPUT);
  pinMode(SOLENOID_CW, OUTPUT);
  pinMode(SOLENOID_CCW, OUTPUT);
  digitalWrite(SENSOR_RESET, HIGH);

  Wire.begin();

  Serial1.begin(9600);

  while(switchSem);
  switchSem = true;
  if(digitalRead(SWITCH_PIN) == HIGH){
    switchSem = false;
    Serial.begin(9600);
  }
  switchSem = false;
  loggerHeader();

  //Try to start sensors for 5 seconds, if all on continue
  sensorSetup.reset(5000);
  while(!sensorSetup.isDone()){
    digitalWrite(DEBUG_LED_1, HIGH);
    if(bno.begin() && bme.begin() && gps.begin()){
      break;
    }
  }
  digitalWrite(DEBUG_LED_1, LOW);
  
  if(!bno.begin() || !bme.begin() || !gps.begin()){
    /*
      If one or more of the sensors don't start, blink for error
      If it does start working it will continue
    */
    while(1){
      errorCycle(250);
    }
    digitalWrite(DEBUG_LED_1, LOW);
    digitalWrite(DEBUG_LED_2, LOW);
    digitalWrite(DEBUG_LED_3, LOW);
  }

  //Imu callibration
  while(i2cSem);
  i2cSem = true;
  uint8_t system, gyroscope, accel, mag = 0;
  bno.setExtCrystalUse(true);
  while(gyroscope < 3 && mag < 3){
    bno.getCalibration(&system, &gyroscope, &accel, &mag);
    digitalWrite(DEBUG_LED_2, HIGH);
  }
  i2cSem = false;
  digitalWrite(DEBUG_LED_2, LOW);
  imuStuff();
  //Set the gyro data to be in degrees
  //DON'T SET TO DEGREES TWICE, WILL BREAK EVERYTHING
  while(i2cSem);
  i2cSem = true;
  gyro.toDegrees();
  i2cSem = false;

  //BME Stuff
  while(i2cSem);
  i2cSem = true;
  bme.setOversampling(TemperatureSensor, Oversample16);
  bme.setOversampling(HumiditySensor, Oversample16);
  bme.setOversampling(PressureSensor, Oversample16);
  bme.setIIRFilter(IIR4);
  bme.setGas(0,0);
  while(bmeAltSem);
  bmeAltSem = true;
  bmeAltitude = readAltitude();
  i2cSem = false;
  bmeAltSem = false;

  //GPS Stuff
  while(i2cSem);
  i2cSem = true;
  gps.setI2COutput(COM_TYPE_UBX, 250);
  gps.setNavigationFrequency(5);
  gps.setI2CpollingWait(250);
  if(!gps.setDynamicModel(DYN_MODEL_AIRBORNE4g));
  while(gpsLatSem);
  gpsLatSem = true;
  gpsLatitude = (float)gps.getLatitude() * 0.0000001;
  gpsLatSem = false;

  while(gpsLongSem);
  gpsLongSem = true;
  gpsLongitude = (float)gps.getLongitude() * 0.0000001;
  gpsLongSem = false;

  while(gpsAltSem);
  gpsAltSem = true;
  gpsAltitude = (float)gps.getAltitudeMSL() / 1000.0;
  i2cSem = false;
  gpsAltSem = false;

  //Reset All Timers
  hazardTimer.reset();
  errorTimer.reset();
  //logTimer.reset();
  //oPIDTimer.reset();
  vPIDTimer.reset();
  while(flightStateSem);
  flightStateSem = true;
  flightState = FLIGHT_READY;
  flightStateSem = false;
}

void loop() {
  while(switchSem);
  switchSem = true;
  if(digitalRead(SWITCH_PIN) == HIGH){
    switchSem = false;
    int loopTime = millis() - lastLoop;
    lastLoop = millis();
    Serial.begin(9600);
    Serial.print("Loop Time: ");
    Serial.println(loopTime);
  }
  switchSem = false;

  if(hazardTimer.getTime() < 100){
    digitalWrite(BRIGHT_LED, HIGH);
  }else if(hazardTimer.getTime() < 1000){
    digitalWrite(BRIGHT_LED, LOW);
  }else{
    hazardTimer.reset();
  }

  PWMSetup(vPID(0));
  PWMLoop();
  
  
  while(flightStateSem);
  flightStateSem = true;
  currentFlightState = flightState;
  flightStateSem = false;
  
  //Serial.println(millis());
  switch(currentFlightState){
    case FLIGHT_READY:
      /*
      Exit Condition:
      When acceleration upwards is greater than 3 m/s^2
      or
      When altitude is greater than 300m
      */
      break;
    case ASCENT:
      /*
      Exit Condition:
      When BME Altitude hits 18km (18000m)
      or
      When GPS Altitude hits 18km (18000m)
      */
      {
        /*
        while(gpsAltSem);
        gpsAltSem = true;
        float gpsAlt = gpsAltitude;
        gpsAltSem = false;

        while(bmeAltSem);
        bmeAltSem = true;
        float bmeAlt = bmeAltitude;
        bmeAltSem = false;

        if(gpsAlt > 18000 || bmeAlt > 18000){
          while(flightStateSem);
          flightStateSem = true;
          flightState = CONTROLLED;
          flightStateSem = false;
        }
        */
      }      
      break;
    case CONTROLLED:
      /*
      Exit Condition:
      When vertical velocity is a sustained negative 3 m/s
      or
      Altitude is less than 17km 
      */
      break;
    case FREEFALL:
      /*
      Exit Condition:
      Vertical Velocity is less than 0.5 m/s
      or
      Altitude is less than 300m
      */
      break;
    case LANDED:
      //Do landed stuff
      break;
    default:
      //errorCycle(500);
      break;
  }
}

void setup1(){
  //Serial1.begin(9600);
  while(1){
    //Don't exit setup1 until setup is done
    digitalWrite(DEBUG_LED_3, HIGH);
    while(flightStateSem);
    flightStateSem = true;
    if(flightState != INIT){
      digitalWrite(DEBUG_LED_3, LOW);
      flightStateSem = false;
      break;
    }
    flightStateSem = false;
  }
}

void loop1(){
  loggerPrint();
  while(switchSem);
  switchSem = true;
  if(digitalRead(SWITCH_PIN) == HIGH){
    switchSem = false;
    int loopTime = millis() - lastLoop1;
    lastLoop1 = millis();
    Serial.begin(9600);
    Serial.print("Loop 1 Time: ");
    Serial.println(loopTime);
  }
  switchSem = false;
}