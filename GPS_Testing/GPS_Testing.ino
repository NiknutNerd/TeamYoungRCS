#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <utility/imumaths.h>

SFE_UBLOX_GNSS gps;

const int DEBUG_LED_1 = 8;
const int DEBUG_LED_2 = 9;
const int DEBUG_LED_3 = 25;
const int BRIGHT_LED = 19;

const int SENSOR_RESET = 11;

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

Timer printTimer;
void setup() {
  Serial.begin(9600);
  Wire.begin();
  while(!gps.begin()){
    digitalWrite(DEBUG_LED_1, HIGH);
  }
  digitalWrite(DEBUG_LED_1, LOW);

  //GPS Stuff
  digitalWrite(SENSOR_RESET, HIGH);
  //I think 1000 will work for now, should try to change to 250 later
  gps.setI2COutput(COM_TYPE_UBX, 1000);
  gps.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);

  printTimer.resetTime();
}

void limitedPrint(long frequency){
  if(printTimer.getTime() > frequency){
    Serial.print("GPS Telemetry: ");

    Serial.print("Sattelites In View: ");
    Serial.println(gps.getSIV());
    Serial.print("Horizontal Accuracy: ");
    Serial.println(gps.getHorizontalAccEst());
    Serial.print("Vertical Accuracy: ");
    Serial.println(gps.getVerticalAccEst());
    Serial.print("Speed Accuracy: ");
    Serial.println(gps.getSpeedAccEst());
    Serial.print("Heading Accuracy: ");
    Serial.println(gps.getHeadingAccEst());
    //Serial.print("Position Accuracy: ");
    //Serial.println(gps.getPositionAccuracy());
    Serial.println("");

    Serial.print("Ground Speed: ");
    Serial.println(gps.getGroundSpeed());
    Serial.print("Heading: ");
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


  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
