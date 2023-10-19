#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME680.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <utility/imumaths.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_BME680 bme;
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

imu::Vector<3> orientation;
imu::Vector<3> gyro;
imu::Vector<3> accel;
imu::Vector<3> linAccel;

void imuStuff(){
  orientation = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
}

//Assign pins for everything that is needed
const int I2C_DATA = 4;
const int I2C_CLOCK = 5;
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
float bmeAltitude;
int packetCount = 0;

//PWM Variables
const float MIN_CYCLE = 1.0/25.0;
const float MIN_CYCLE_MILLIS = MIN_CYCLE * 1000.0;

//PID Variables
float vPIDError;
float vLastTarget = 0.0;
float vp = 0.0;
float vkp = 0.035;
float vi = 0.0;
float vki = 0.0;
float vd = 0.0;
float vkd = 0.0;
float vPIDOutput;

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
Timer logTimer;
//Timer oPIDTimer;
Timer vPIDTimer;

//Countdowns
CountdownTimer sensorSetup;
//PWM Countdowns
CountdownTimer aOnCountdown;
CountdownTimer aCountdown;
CountdownTimer bOnCountdown;
CountdownTimer bCountdown;

void imuStuff(){
  orientation = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
}

float vPID(float target){
  //imuStuff();
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
  if(abs(percent) < 0.075){
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

void errorCycle(long frequency){
  if(errorTimer.getTime() < frequency){
    digitalWrite(DEBUG_LED_1, HIGH);
    digitalWrite(DEBUG_LED_2, LOW);
    digitalWrite(DEBUG_LED_3, LOW);
    digitalWrite(BRIGHT_LED, LOW);
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
    digitalWrite(BRIGHT_LED, HIGH);
  }else{
    errorTimer.reset();
  }
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

  Serial1.begin(9600);
  Wire.begin();

  //Try to start sensors for 5 seconds, if all on continue
  /*
  sensorSetup.reset(5000);
  while(!sensorSetup.isDone()){
    digitalWrite(DEBUG_LED_1, HIGH);
    if(bno.begin() && bme.begin() && gps.begin()){
      break;
    }
  }
  */

  sensorSetup.reset(5000);
  while(!sensorSetup.isDone()){
    digitalWrite(DEBUG_LED_1, HIGH);
    if(bno.begin() && bme.begin() && gps.begin()){
      break;
    }
  }
  sensorSetup.reset();
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
  
  uint8_t system, gyroscope, accel, mag = 0;
  bno.setExtCrystalUse(true);
  while(gyroscope < 3 && mag < 3){
    bno.getCalibration(&system, &gyroscope, &accel, &mag);
    digitalWrite(DEBUG_LED_2, HIGH);
  }
  digitalWrite(DEBUG_LED_2, LOW);
  //imuStuff();
  gyro.toDegrees();

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320,150);

  //GPS Stuff
  gps.setI2COutput(COM_TYPE_UBX, 250);
  gps.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);

  //Reset All Timers
  hazardTimer.reset();
  errorTimer.reset();
  logTimer.reset();
  vPIDTimer.reset();
}

void loggerPrint(long frequency){
  if(logTimer.getTime() > frequency){
    logTimer.reset();
    packetCount++;
    Serial1.print("MOAB");
    Serial1.print(",");
    Serial1.print(missionTime.printableTimer());
    Serial1.print(",");
    Serial1.print(packetCount);
    Serial1.print(",");
    Serial1.print(flightState);
    Serial1.print(",");
    if(!bme.performReading()){
      Serial1.print("error");
    }else{
      Serial1.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    }
    Serial1.print(",");
    if(!bme.performReading()){
      Serial1.print("error");
    }else{
      Serial1.print(bme.temperature);
    }
    Serial1.print(",");
    //imuStuff();
    Serial1.print(accel.x());
    Serial1.print(",");
    Serial1.print(accel.y());
    Serial1.print(",");
    Serial1.print(accel.z());
    Serial1.print(",");
    Serial1.print(gyro.x());
    Serial1.print(",");
    Serial1.print(gyro.y());
    Serial1.print(",");
    Serial1.print(gyro.z());
    Serial1.print(",");
    Serial1.print(orientation.x());
    Serial1.print(",");
    Serial1.print(orientation.y());
    Serial1.print(",");
    Serial1.print(orientation.z());
    Serial1.print(",");
    float gpsLat = (float)gps.getLatitude() * 0.0000001;
    Serial1.print(gpsLat);
    Serial1.print(",");
    float gpsLong = (float)gps.getLongitude() * 0.0000001;
    Serial1.print(gpsLong);
    Serial1.print(",");
    float gpsAlt = gpsAltitude;
    Serial1.println(gpsAlt);
  }
}

void loop() {
  loggerPrint(250);

  //Hazard Light Blinking, should always be happening
  if(hazardTimer.getTime() < 500){
    digitalWrite(BRIGHT_LED, HIGH);
  }else if(hazardTimer.getTime() < 1000){
    digitalWrite(BRIGHT_LED, LOW);
  }else{
    hazardTimer.reset();
  }
  gpsAltitude = (float)gps.getAltitudeMSL() / 1000.0;
  bmeAltitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

  PWMSetup(vPID(0));
  PWMLoop();
  
  currentFlightState = flightState;
  switch(currentFlightState){
    case INIT:
      flightState = FLIGHT_READY;
      break;
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
        float gpsAlt = gpsAltitude;
        float bmeAlt = bmeAltitude;

        if(gpsAlt > 18000 || bmeAlt > 18000){
          flightState = CONTROLLED;
        }
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
