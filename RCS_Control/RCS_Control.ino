#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

int brightLEDPin = 9;
int switchPin = 16;

unsigned long currentTime;

void setup() {
  // put your setup code here, to run once:
  pinMode(9,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  currentTime = millis(); 
  
}

class Timer{
  private:
    unsigned long startTime;
    unsigned long timeSinceStart;
  public:
    Timer(unsigned long initTime){
      startTime = initTime;
    }
    unsigned long getTime(){
      unsigned long timeSinceStart = currentTime - startTime;
      return timeSinceStart;
    }
    void resetTime(){
      startTime = currentTime;
      timeSinceStart = 0;
    }
};