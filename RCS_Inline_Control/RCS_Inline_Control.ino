#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
//#include <Adafruit_BME680.h>
#include <Zanshin_BME680.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <utility/imumaths.h>
#include <Servo.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BNO055 bno = Adafruit_BNO055(55);
//Adafruit_BME680 bme;
BME680_Class bme;
SFE_UBLOX_GNSS gps;
Servo fucker;

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
const int SERVO_PWM = 18;

//Random Variables
float gpsAltitude;
float gpsLatitude;
float gpsLongitude;
float bmeAltitude;
int32_t bmeTemperature;
int32_t bmePressure;
int lastLoop = 0;
int packetCount = 0;

//PWM Variables
const float MIN_CYCLE = 1.0/25.0;
const float MIN_CYCLE_MILLIS = MIN_CYCLE * 1000.0;

//PID Variables
float oPIDError;
float oLastTarget = 0.0;
float op = 0.0;
float okp = 0.2;
float oi = 0.0;
float oki = 0.0;
float od = 0.0;
float okd = 0.0;
float oPIDOutput;

float vPIDError;
float vLastTarget = 0.0;
float vp = 0.0;
float vkp = 0.035;
//float vkp = 0.02;
float vi = 0.0;
float vki = 0.00001;
float vd = 0.0;
float vkd = 0.00;
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
Timer servoTimer;
Timer missionTime;
Timer hazardTimer;
Timer errorTimer;
Timer logTimer;
Timer oPIDTimer;
Timer vPIDTimer;

//Countdowns
CountdownTimer sensorSetup;
CountdownTimer gpsTimer;
//PWM Countdowns
CountdownTimer aOnCountdown;
CountdownTimer aCountdown;
CountdownTimer bOnCountdown;
CountdownTimer bCountdown;


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
  
  return oPIDOutput;
}

float vPID(float target){
  imuStuff();
  float current = gyro.z();
  vPIDError = (target - current);
  if(abs(vPIDError) < 5){
    return 0;
  }
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
  
  return vPIDOutput;
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

float readAltitude(){
  float atmospheric = bmePressure / 100.0;
  return 44330.0 * (1.0 - pow(atmospheric / SEALEVELPRESSURE_HPA, 0.1903));
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

void loggerPrint(long frequency){
  if(logTimer.getTime() > frequency){
    if(digitalRead(SWITCH_PIN) == HIGH){
      Serial.println("Log Start");
    }
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

    if(digitalRead(SWITCH_PIN) == HIGH){
      Serial.println("Sensor Reading Start");
    }

    int32_t bmeHumidity;
    int32_t bmeGas;
    //int bmePressure;
    bme.getSensorData(bmeTemperature, bmeHumidity, bmePressure, bmeGas, false);
    
    bmeAltitude = readAltitude();
    Serial1.print(bmeAltitude);
    Serial1.print(",");

    float bmeTemp = bmeTemperature / 100.0;
    Serial1.print(bmeTemp);
    if(digitalRead(SWITCH_PIN) == HIGH){
      Serial.println("BME Reading Done");
    }

    Serial1.print(",");

    imuStuff();
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

    if(digitalRead(SWITCH_PIN) == HIGH){
      Serial.println("IMU Reading Done");
    }

    gpsLatitude = (float)gps.getLatitude() * 0.0000001;
    Serial1.print(gpsLatitude, 6);
    Serial1.print(",");
    gpsLongitude = (float)gps.getLongitude() * 0.0000001;
    Serial1.print(gpsLongitude, 6);
    Serial1.print(",");
    gpsAltitude = (float)gps.getAltitudeMSL() / 1000.0;
    Serial1.println(gpsAltitude, 6);
    if(digitalRead(SWITCH_PIN) == HIGH){
      Serial.println("GPS Reading Done");
    }
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
  pinMode(SERVO_PWM, OUTPUT);
  digitalWrite(SENSOR_RESET, HIGH);

  fucker.attach(SERVO_PWM);
  fucker.write(150); //Initialize level, range 130 (back) to 170 (forward)

  Serial1.begin(9600);
  if(digitalRead(SWITCH_PIN) == HIGH){
    Serial.begin(9600);
  }
  Wire.begin();
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

  uint8_t system, gyroscope, accel, mag = 0;
  bno.setExtCrystalUse(true);
  while(gyroscope < 3 && mag < 3){
    bno.getCalibration(&system, &gyroscope, &accel, &mag);
    digitalWrite(DEBUG_LED_2, HIGH);
  }
  digitalWrite(DEBUG_LED_2, LOW);
  imuStuff();
  gyro.toDegrees();

  /*
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_4);
  */
  bme.setOversampling(TemperatureSensor, Oversample16);
  bme.setOversampling(HumiditySensor, Oversample16);
  bme.setOversampling(PressureSensor, Oversample16);
  bme.setIIRFilter(IIR4);
  bme.setGas(0,0);


  //bme.setGasHeater(20,0);

  bmeAltitude = readAltitude();

  //GPS Stuff
  gps.setI2COutput(COM_TYPE_UBX, 250);
  gps.setNavigationFrequency(4);
  gps.setI2CpollingWait(250);
  if(!gps.setDynamicModel(DYN_MODEL_AIRBORNE4g));
  //gps.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);

  gpsLatitude = (float)gps.getLatitude() * 0.0000001;
  gpsLongitude = (float)gps.getLongitude() * 0.0000001;
  gpsAltitude = (float)gps.getAltitudeMSL() / 1000.0;

  //Reset All Timers
  servoTimer.reset();
  hazardTimer.reset();
  errorTimer.reset();
  logTimer.reset();
  oPIDTimer.reset();
  vPIDTimer.reset();
}

void loop() {
  if(digitalRead(SWITCH_PIN) == HIGH){
    int loopTime = millis() - lastLoop;
    lastLoop = millis();
    Serial.println(loopTime);
  }

  //Hazard Light Blinking, should always be happening
  if(hazardTimer.getTime() < 350){
    digitalWrite(BRIGHT_LED, HIGH);
  }else if(hazardTimer.getTime() < 1000){
    digitalWrite(BRIGHT_LED, LOW);
  }else{
    hazardTimer.reset();
  }

  //Only try to call gps 4 times a second, might decrease
  
  switch(flightState){
    case INIT:
      flightState = FLIGHT_READY;
      break;
    case FLIGHT_READY:
      loggerPrint(1000);
      digitalWrite(DEBUG_LED_3, )
      /*
      Exit Condition:
      When acceleration upwards is greater than 3 m/s^2
      or
      When altitude is greater than 300m
      */
      
      {
        float vertAccel = linAccel.z();
        float gpsAlt = gpsAltitude;
        if(vertAccel >= 3.0 || gpsAlt >= 400.0){
          flightState = ASCENT;
        }
      }

      break;
    case ASCENT:
      loggerPrint(1000);
      /*
      Exit Condition:
      When BME Altitude hits 18km (18000m)
      or
      When GPS Altitude hits 18km (18000m)
      */
      {
        float gpsAlt = gpsAltitude;
        float bmeAlt = bmeAltitude;

        if(gpsAltitude > 20000 || bmeAltitude > 20000){
          servoTimer.reset();
          flightState = CONTROLLED;
        }
      }      
      break;
    case CONTROLLED:
      loggerPrint(1000);
      if(servoTimer.getTime() < 5000){
        fucker.write(130);
      }else if(servoTimer.getTime() < 10000){
        fucker.write(170);
      }else if(servoTimer.getTime() < 15000){
        fucker.write(150);
      }
      PWMSetup(vPID(oPID(90)));
      PWMLoop();
      /*
      Exit Condition:
      When vertical velocity is a sustained negative 3 m/s
      or
      Altitude is less than 17km 
      */{
        float vertAccel = linAccel.z();
        float bmeAlt = bmeAltitude;
        float gpsAlt = gpsAltitude;
        if(vertAccel <= -5.0 || bmeAlt <= 19000 || gpsAlt <= 19000){
          flightState = FREEFALL;
        }
      }
      break;
    case FREEFALL:
      loggerPrint(1000);
      PWMLoop();
      /*
      Exit Condition:
      Vertical Velocity is less than 0.5 m/s
      or
      Altitude is less than 300m
      */
      {
        float vertAccel = linAccel.z();
        float bmeAlt = bmeAltitude;
        float gpsAlt = gpsAltitude;
        if(vertAccel >= -0.5 || bmeAlt <= 400.0 || gpsAlt <= 400.0){
          flightState = LANDED;
        }
      }

      break;
    case LANDED:
      //Do landed stuff
      //DONT DO ANYTHING BUT BLINK LIGHTS
      break;
    default:
      errorCycle(500);
      break;
  }
}
