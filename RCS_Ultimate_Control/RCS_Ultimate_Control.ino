#include <math.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME680.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <utility/imumaths.h>

#define SEALEVELPRESSURE_HPA (1013.25)
//#define defaultMaxWait 250

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_BME680 bme;
SFE_UBLOX_GNSS gps;
Servo camServo;

imu::Vector<3> orientation;
imu::Vector<3> gyro;
imu::Vector<3> accel;
imu::Vector<3> linAccel;

const int UART_TX = 0;
const int UART_RX = 1;

const int I2C_DATA = 4;
const int I2C_CLOCK = 5;

const int SWITCH_PIN = 6;

const int DEBUG_LED_1 = 8;
const int DEBUG_LED_2 = 9;
//const int DEBUG_LED_1 = 20;
//const int DEBUG_LED_2 = 21;
const int DEBUG_LED_3 = 25;
const int BRIGHT_LED = 19;

const int SENSOR_RESET = 11;
const int SERVO_FEEDBACK= A0;
const int SERVO_CONTROL = 18;

const int SOLENOID_CCW = 20;
const int SOLENOID_CW = 21;
//const int SOLENOID_CCW = 8;
//const int SOLENOID_CW = 9;
const double MIN_CYCLE = 1.0/20.0;
const double MIN_CYCLE_MILLIS = MIN_CYCLE * 1000;
double onPercent;
double offPercent;
long onTime;
long offTime;

int switchState;

double targetX;
double currentX;
double errorX;
double inputPower;

enum FlightState{
  INIT, FLIGHT_READY, ASCENT, CONTROLLED, FREEFALL, LANDED
};

FlightState flightState = INIT;

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
    long getTotalSeconds(){
      long seconds = getTime() / 1000;
      return seconds;
    }
    long getTotalMinutes(){
      long minutes = getTotalSeconds() / 60;
      return minutes;
    }
    long getHours(){
      long hours = getTotalMinutes() / 60;
      return hours;
    }
    long getTimerMinutes(){
      long minutes = getTotalMinutes() - (getHours() * 60);
      return minutes;
    }
    long getTimerSeconds(){
      long seconds = getTotalSeconds() - (getTotalMinutes() * 60);
      return seconds;
    }
    long getTimerMillis(){
      long mil = getTime() - (getTotalSeconds() * 1000);
      return mil;
    }
    String printableTimer(){
      String timer = (String(getHours()) + ":");
      timer += (String(getTimerMinutes()) + ":");
      timer += (String(getTimerSeconds()) + ".");
      timer += getTimerMillis();
      return timer;
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

Timer sensorSetup;
Timer printTimer;
Timer solenoidTimer;
Timer LEDTimer;
Timer servoTimer;

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

void PWMSetup(double percent){
  if(aCountdown.getTime() > 0 || bCountdown.getTime() > 0){
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
  pinMode(BRIGHT_LED, OUTPUT);
  pinMode(DEBUG_LED_1, OUTPUT);
  pinMode(DEBUG_LED_2, OUTPUT);
  pinMode(DEBUG_LED_3, OUTPUT);
  pinMode(SOLENOID_CW, OUTPUT);
  pinMode(SOLENOID_CCW, OUTPUT);
  pinMode(SWITCH_PIN, INPUT);

  switchState = digitalRead(SWITCH_PIN);
  if(switchState == HIGH){
    Serial.begin(9600);
    while(!Serial){
    }
    Serial.println("Control Program Starting");
    Serial.println("Starting Sensors");
  }
  Wire.begin();

  while(sensorSetup.getTime() < 5000){
    digitalWrite(DEBUG_LED_1, HIGH);
    if(bno.begin() && bme.begin() && gps.begin()){
      break;
    }
  }
  
  sensorSetup.reset();
  digitalWrite(DEBUG_LED_1, LOW);
  if(!bno.begin() || !bme.begin() || !gps.begin()){
    //Serial.println("No IMU or BME Detected");
    while(1){
      if(LEDTimer.getTime() < 250){
        digitalWrite(DEBUG_LED_1, HIGH);
        digitalWrite(DEBUG_LED_2, LOW);
        digitalWrite(DEBUG_LED_3, LOW);
        digitalWrite(BRIGHT_LED, LOW);
      }else if(LEDTimer.getTime() < 500){
        digitalWrite(DEBUG_LED_1, LOW);
        digitalWrite(DEBUG_LED_2, HIGH);
        digitalWrite(DEBUG_LED_3, LOW);
        digitalWrite(BRIGHT_LED, LOW);
      }else if(LEDTimer.getTime() < 750){
        digitalWrite(DEBUG_LED_1, LOW);
        digitalWrite(DEBUG_LED_2, LOW);
        digitalWrite(DEBUG_LED_3, HIGH);
        digitalWrite(BRIGHT_LED, LOW);
      }else if(LEDTimer.getTime() < 1000){
        digitalWrite(DEBUG_LED_1, LOW);
        digitalWrite(DEBUG_LED_2, LOW);
        digitalWrite(DEBUG_LED_3, LOW);
        digitalWrite(BRIGHT_LED, HIGH);
      }else{
        LEDTimer.reset();
      }
    }
  }

  //IMU Stuff
  uint8_t system, gyroo, accel, mag = 0;
  bno.setExtCrystalUse(true);
  //IMU Callibration
  while(gyroo < 3 && mag < 3){
    bno.getCalibration(&system, &gyroo, &accel, &mag);
    digitalWrite(DEBUG_LED_2, HIGH);
  }
  digitalWrite(DEBUG_LED_2, LOW);
  imuStuff();
  gyro.toDegrees();

  //BME Stuff
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320,150);

  //GPS Stuff
  digitalWrite(SENSOR_RESET, HIGH);
  //I think 1000 will work for now, should try to change to 250 later
  gps.setI2COutput(COM_TYPE_UBX, 1000);
  gps.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);

  printTimer.reset();
  sensorSetup.reset();
  solenoidTimer.reset();
  LEDTimer.reset();
  servoTimer.reset();
}

void setup1(){
  Serial.begin(9600);
}

void limitedPrint(long frequency){
  if(printTimer.getTime() > frequency){
    imuStuff();
    Serial.println("Telemetry: ");

    Serial.print("A Countdowns: ");
    Serial.print(aCountdown.getTime());
    Serial.print(aOnCountdown.getTime());
    Serial.println(aOffCountdown.getTime());

    Serial.print("B Countdowns: ");
    Serial.print(bCountdown.getTime());
    Serial.print(bOnCountdown.getTime());
    Serial.println(bOffCountdown.getTime());

    Serial.print("Error: ");
    Serial.print(errorX);
    Serial.print(" Input: ");
    Serial.println(inputPower);

    Serial.println(analogRead(SERVO_FEEDBACK));

    if(!bme.performReading()){
      Serial.println("BME Temp Failed");
    }else{
      Serial.print("BME Temperature: ");
      Serial.print(bme.temperature);
      Serial.println(" *C");
    }
    //Serial.print("Pressure = ");
    //Serial.print(bme.pressure / 100.0);
    //Serial.println(" hPa");

    /*
    Serial.print("Approximate Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("BNO Temperature: ");
    Serial.print(bno.getTemp());
    Serial.println("*C");

    Serial.print("Sattelites In View: ");
    Serial.println(gps.getSIV());
    Serial.print("GPS Horizontal Accuracy: ");
    Serial.println(gps.getHorizontalAccEst());
    Serial.print("GPS Vertical Accuracy: ");
    Serial.println(gps.getVerticalAccEst());
    Serial.print("GPS Ground Speed: ");
    Serial.println(gps.getGroundSpeed());
    Serial.print("GPS Heading: ");
    Serial.println(gps.getHeading());

    Serial.print("Latitude: ");
    Serial.print(gps.getLatitude());
    Serial.println(" Degrees * 10^-7");
    Serial.print("Longitute: ");
    Serial.print(gps.getLongitude());
    Serial.println(" Degrees * 10^-7");
    Serial.print("Altitude Above Eliptic: ");
    Serial.print(gps.getAltitude());
    Serial.println("mm");
    Serial.print("Altitude Above Sea Level: ");
    Serial.print(gps.getAltitudeMSL());
    Serial.println("mm");
    */

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

    printTimer.reset();
  }
}

void loop() {
  imuStuff();
  if(false){
    if(solenoidTimer.getTime() < 5000){
      digitalWrite(DEBUG_LED_1, HIGH);
      digitalWrite(DEBUG_LED_2, HIGH);
    }else if(solenoidTimer.getTime() < 6000){
      digitalWrite(DEBUG_LED_1, LOW);
      digitalWrite(DEBUG_LED_2, LOW);
      digitalWrite(SOLENOID_CCW, HIGH);
    }else{
      digitalWrite(SOLENOID_CCW, LOW);
    }
  }
  
  if(false){
    if(servoTimer.getTime() < 1000){
      analogWrite(SERVO_CONTROL, 200);
    }else if(servoTimer.getTime() < 2000){
      analogWrite(SERVO_CONTROL, 100);
    }else{
      servoTimer.reset();
    }
  }

  if(false){
    if(LEDTimer.getTime() < 250){
      digitalWrite(DEBUG_LED_1, HIGH);
      digitalWrite(DEBUG_LED_2, LOW);
      digitalWrite(DEBUG_LED_3, LOW);
      digitalWrite(BRIGHT_LED, LOW);
    }else if(LEDTimer.getTime() < 500){
      digitalWrite(DEBUG_LED_1, LOW);
      digitalWrite(DEBUG_LED_2, HIGH);
      digitalWrite(DEBUG_LED_3, LOW);
      digitalWrite(BRIGHT_LED, LOW);
    }else if(LEDTimer.getTime() < 750){
      digitalWrite(DEBUG_LED_1, LOW);
      digitalWrite(DEBUG_LED_2, LOW);
      digitalWrite(DEBUG_LED_3, HIGH);
      digitalWrite(BRIGHT_LED, LOW);
    }else if(LEDTimer.getTime() < 1000){
      digitalWrite(DEBUG_LED_1, LOW);
      digitalWrite(DEBUG_LED_2, LOW);
      digitalWrite(DEBUG_LED_3, LOW);
      digitalWrite(BRIGHT_LED, HIGH);
    }else{
      LEDTimer.reset();
    }
  }
  if(true){
    if(solenoidTimer.getTime() < 1000){
      PWMSetup(.75);
    }else if(solenoidTimer.getTime() < 2000){
      PWMSetup(.5);
    }else if(solenoidTimer.getTime() < 3000){
      PWMSetup(-.75);
    }else if(solenoidTimer.getTime() < 4000){
      PWMSetup(-.5);
    }else if(solenoidTimer.getTime() < 5000){
      PWMSetup(0.0);
    }else{
      solenoidTimer.reset();
    }
    PWMLoop();
  }

  if(false){
    currentX = orientation.x();
    targetX = 180;
    errorX = targetX - currentX;
    inputPower = errorX / 180.0;
    PWMSetup(inputPower);
    PWMLoop();
  }

  switch(flightState){
    case INIT:
      flightState = FLIGHT_READY;
      break;
    case FLIGHT_READY:
      //flight ready stuff
      break;
    case ASCENT:
      //Ascent
      break;
    case CONTROLLED:
      //Controlled Stuff
      break;
    case FREEFALL:
      //Freefall Stuff
      break;
    case LANDED:
      //Landed Stuff
      break;
    default:
      if(LEDTimer.getTime() < 250){
        digitalWrite(DEBUG_LED_1, HIGH);
        digitalWrite(DEBUG_LED_2, LOW);
        digitalWrite(DEBUG_LED_3, LOW);
        digitalWrite(BRIGHT_LED, LOW);
      }else if(LEDTimer.getTime() < 500){
        digitalWrite(DEBUG_LED_1, LOW);
        digitalWrite(DEBUG_LED_2, HIGH);
        digitalWrite(DEBUG_LED_3, LOW);
        digitalWrite(BRIGHT_LED, LOW);
      }else if(LEDTimer.getTime() < 750){
        digitalWrite(DEBUG_LED_1, LOW);
        digitalWrite(DEBUG_LED_2, LOW);
        digitalWrite(DEBUG_LED_3, HIGH);
        digitalWrite(BRIGHT_LED, LOW);
      }else if(LEDTimer.getTime() < 1000){
        digitalWrite(DEBUG_LED_1, LOW);
        digitalWrite(DEBUG_LED_2, LOW);
        digitalWrite(DEBUG_LED_3, LOW);
        digitalWrite(BRIGHT_LED, HIGH);
      }else{
        LEDTimer.reset();
      }
      break;
  }
}

void loop1() {
  //If in init this loop shouldn't run
  while(flightState == INIT);

}