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

const int DEBUG_LED_1 = 8;
const int DEBUG_LED_2 = 9;

const int SOLENOID_CW = 21;
const int SOLENOID_CCW = 20;

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

Timer printTimer;

//Semaphore Stuff
bool osem = false;
bool gsem = false


void imuStuff(){
  osem = true;
  orientation = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  osem = false;

  gsem = true;
  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  gsem = false;
  accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
}

void setup(){
  pinMode(SOLENOID_CW, OUTPUT);
  pinMode(SOLENOID_CCW, OUTPUT);
  pinMode(DEBUG_LED_1, OUTPUT);
  pinMode(DEBUG_LED_2, OUTPUT);
  Serial.begin(115200);

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
}

void limitedPrint(long frequency){
  if(printTimer.getTime() > frequency){
    while(osem){}
    osem = true;
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
    osem = false;
    printTimer.reset();
  }

}

void loop(){
  imuStuff();
  osem = true;
  if(orientation.x() < 135){
    digitalWrite(DEBUG_LED_1, HIGH);
    digitalWrite(DEBUG_LED_2, LOW);
  }else if(orientation.x() > 225){
    digitalWrite(DEBUG_LED_1, LOW);
    digitalWrite(DEBUG_LED_2, HIGH);
  }else{
    digitalWrite(DEBUG_LED_1, LOW);
    digitalWrite(DEBUG_LED_2, LOW);
  }
  osem = false;
}

void loop1(){
  limitedPrint(1000);
}