//#include "SoftwareSerialTX.h"
#include <SoftwareSerial.h>
const int UART_TX = 0;
const int UART_RX = 1;


//SoftwareSerialTX openLog(UART_TX);

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

Timer telemTimer;

void setup() {
  openLog.begin(9600);
  /*
  while(!openLog){
  }
  */

  telemTimer.resetTime();

}

String missionTime(long milliseconds){
  String output;
  long hours = milliseconds/3600000;
  long minutes = (milliseconds%3600000)/60000;
  long seconds = ((milliseconds%3600000)%60000)/1000;
  long millisecs = ((milliseconds%3600000)%60000)%1000;
  output = (String)hours;
  output += ":";
  output += (String)minutes;
  output += ":";
  output += (String)seconds;
  output += ".";
  output += (String)millisecs;
  return(output);
}


void telemetry(long frequency){
  if(telemTimer.getTime() > frequency){
    openLog.write("MOAB");
    openLog.write(",");
    openLog.write(missionTime(millis()));
    openLog.write(",");
  }
}

void loop() {
  telemetry(250);
}
