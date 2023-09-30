#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME680.h>
#include <utility/imumaths.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_BME680 bme;
sensors_event_t event;

const int I2C_DATA = 4;
const int I2C_CLOCK = 5;

const int GPS_RESET = 11;

const int UART_TX = 0;
const int UART_RX = 1;

const int SOLENOID_CW = 2;
const int SOLENOID_CCW = 3;

//const int SERVO_FEEDBACK = 4;
//const int SERVO_PWM = 5;

const int REGULATOR_6V = 10;

const int POWER_GOOD_6V = 14;
const int POWER_GOOD_3V = 15;

const int SWITCH_PIN = 16;

const int BUZZER_PIN = 17;

const int BRIGHT_LED = 9;
const int DEBUG_LED_1 = 18;
const int DEBUG_LED_2 = 19;
const int DEBUG_LED_3 = 20;
const int DEBUG_LED_4 = 21;

const double MIN_CYCLE_TIME = 1.0/20.0;
const double MIN_CYCLE_TIME_MILLIS = MIN_CYCLE_TIME * 1000;
double onPercent;
double offPercent;
long onTime;
long offTime;

double targetX;
double currentX;
double errorX;


//long currentTime = millis();
int loops;

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

class CountdownTimer{
  private:
    long targetTime;
    long initialTime;
  public:
    CountdownTimer(long time){
      initialTime = time;
      targetTime = (long)millis() + time; 
    }
    long getTimeLeft(){
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

Timer LEDTimer;
Timer bmeTimer;
Timer solenoidTimer;
Timer PWMTimer;
Timer printTimer;
Timer sensorSetup;
Timer telemTimer;
//Timer testTimer;

CountdownTimer aCountdown(0);
CountdownTimer bCountdown(0);
CountdownTimer aOnCountdown(0);
CountdownTimer aOffCountdown(0);
CountdownTimer bOnCountdown(0);
CountdownTimer bOffCountdown(0);
CountdownTimer testCountdown(0);

String color = "None";
int switchState;
int lastSwitchState = switchState;
int solenoidState = 0;

void PWMSetup(double percent){
  if(percent > 0){
    bCountdown.changeTimer(0);
    bOnCountdown.changeTimer(0);
    bOffCountdown.changeTimer(0);

    onPercent = percent;
    offPercent = 1 - percent;
    if(onPercent >= offPercent){
      offTime = MIN_CYCLE_TIME_MILLIS;
      onTime = (onPercent/offPercent) * MIN_CYCLE_TIME_MILLIS;
    }else if(offPercent > onPercent){
      //Change to else
      onTime = MIN_CYCLE_TIME_MILLIS;
      offTime = (offPercent/onPercent) * MIN_CYCLE_TIME_MILLIS;
    }
    aCountdown.changeTimer(onTime + offTime);
    aOnCountdown.changeTimer(onTime);
    aOffCountdown.changeTimer(onTime+offTime);
  }else if(percent < 0){
    aCountdown.changeTimer(0);
    aOnCountdown.changeTimer(0);
    aOffCountdown.changeTimer(0);
    
    onPercent = abs(percent);
    offPercent = 1 - abs(percent);
    if(onPercent >= offPercent){
      offTime = MIN_CYCLE_TIME_MILLIS;
      onTime = (onPercent/offPercent) * MIN_CYCLE_TIME_MILLIS;
    }else if(offPercent > onPercent){
      onTime = MIN_CYCLE_TIME_MILLIS;
      offTime = (offPercent/onPercent) * MIN_CYCLE_TIME_MILLIS;
    }
    bCountdown.changeTimer(onTime + offTime);
    bOnCountdown.changeTimer(onTime);
    bOffCountdown.changeTimer(onTime+offTime);
  }else if(percent == 0){
    aCountdown.changeTimer(0);
    aOnCountdown.changeTimer(0);
    aOffCountdown.changeTimer(0);

    bCountdown.changeTimer(0);
    bOnCountdown.changeTimer(0);
    bOffCountdown.changeTimer(0);
  }
}
void PWMLoop(){
  if(aOnCountdown.getTimeLeft() > 0){
    digitalWrite(SOLENOID_CW, HIGH);
  }else if(aOffCountdown.getTimeLeft() > 0){
    digitalWrite(SOLENOID_CW, LOW);
  }else if(bOnCountdown.getTimeLeft() > 0){
    digitalWrite(SOLENOID_CCW, HIGH);
  }else if(bOffCountdown.getTimeLeft() > 0){
    digitalWrite(SOLENOID_CCW, LOW);
  }else{
    digitalWrite(SOLENOID_CW, LOW);
    digitalWrite(SOLENOID_CCW, LOW);
  }
}

void setup() {
  Serial1.setRX(UART_RX);
  Serial1.setTX(UART_TX);
  Serial1.begin(9600);
  Serial.begin(9600);
  while(!Serial){
  }
  Serial.println("Control Program Starting");

  //Create Timers
  LEDTimer.resetTime();
  bmeTimer.resetTime();
  solenoidTimer.resetTime();
  PWMTimer.resetTime();
  printTimer.resetTime();
  sensorSetup.resetTime();
  telemTimer.resetTime();
  //testTimer.resetTime();
  testCountdown.changeTimer(5000);
  //Serial.println(testCountdown.getTimeLeft());
  loops = 0;

  
  Serial.println("Starting Sensors");
  Serial.println(sensorSetup.getTime());
  Serial.println(sensorSetup.getTime() < 5000);
  
  /*
  while(sensorSetup.getTime() < 5000){
    digitalWrite(DEBUG_LED_3, HIGH);
    if(bno.begin() && bme.begin()){
      break;
    }
    if(printTimer.getTime() > 500){
      Serial.print("Trying to start sensors");
      Serial.print(sensorSetup.getTime());
    }
  }
  */
  printTimer.resetTime();
  digitalWrite(DEBUG_LED_3, LOW);
  //IMU Setup
  Serial.println("");
  Serial.println("Setup Timer Complete");
  /*
  if(!bno.begin() || !bme.begin()){
    Serial.println("No IMU or BME Detected");
    while(1){
      if(LEDTimer.getTime() < 500){
        digitalWrite(DEBUG_LED_3, HIGH);
        digitalWrite(DEBUG_LED_4, LOW);
      }else if(LEDTimer.getTime() < 1000){
        digitalWrite(DEBUG_LED_3, LOW);
        digitalWrite(DEBUG_LED_4, HIGH);
      }else{
        LEDTimer.resetTime();
      }
    }
  }
  */
  Serial.println("Past BNO Begin Check");

  //IMU Stuff
  /*
  uint8_t system, gyro, accel, mag = 0;
  bno.setExtCrystalUse(true);

  //BME Stuff
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320,150);
  */
  

  //Define all pin modes
  pinMode(BRIGHT_LED, OUTPUT);
  pinMode(DEBUG_LED_1, OUTPUT);
  pinMode(DEBUG_LED_2, OUTPUT);
  pinMode(DEBUG_LED_3, OUTPUT);
  pinMode(DEBUG_LED_4, OUTPUT);

  pinMode(SOLENOID_CW, OUTPUT);
  pinMode(SOLENOID_CCW, OUTPUT);

  pinMode(SWITCH_PIN, INPUT);  

  /*
  while(gyro < 3 && mag < 3){
    bno.getCalibration(&system, &gyro, &accel, &mag);
    if(LEDTimer.getTime() < 250){
      digitalWrite(DEBUG_LED_4, HIGH);
    }else if(LEDTimer.getTime() < 250){
      digitalWrite(DEBUG_LED_4, LOW);
    }else{
      digitalWrite(DEBUG_LED_4, LOW);
      LEDTimer.resetTime();
    }
  }
  digitalWrite(DEBUG_LED_4, LOW);
  */
  LEDTimer.resetTime();
}
String missionTime(long milliseconds){
  String output;
  long hours = milliseconds/3600000;
  long minutes = (milliseconds%3600000)/60000;
  long seconds = ((milliseconds%3600000)%60000)/1000;
  long millisecs = ((milliseconds%3600000)%60000)%1000;
  output = hours;
  output += ":";
  output += minutes;
  output += ":";
  output += seconds;
  output += ".";
  outpt += millisecs;
  return(output);
}

void telemetry(long frequency){
  if(telemTimer.getTime() > frequency){
    Serial1.print("MOAB");
    Serial1.print(",");
    Serlial1.print(missionTime(millis()));
    Serial1.print(",");
  }
}

//Functions Here
void limitedPrint(long frequency){
  if(printTimer.getTime() > frequency){
    Serial.println("Telemetry: ");
    Serial.print("A Countdowns: ");
    Serial.print(aCountdown.getTimeLeft());
    Serial.print(aOnCountdown.getTimeLeft());
    Serial.println(aOffCountdown.getTimeLeft());

    Serial.print("B Countdowns: ");
    Serial.print(bCountdown.getTimeLeft());
    Serial.print(bOnCountdown.getTimeLeft());
    Serial.println(bOffCountdown.getTimeLeft());
    
    Serial.print("Color: ");
    Serial.println(color);

    Serial.print("Switch State: ");
    Serial.println(switchState);

    /*
    Serial.print("Temperature = ");
    Serial.print(bme.temperature);
    Serial.println(" *C");
    Serial.print("Pressure = ");
    Serial.print(bme.pressure / 100.0);
    Serial.println(" hPa");
    Serial.print("Humidity = ");
    Serial.print(bme.humidity);
    Serial.println(" %");
    Serial.print("Gas = ");
    Serial.print(bme.gas_resistance / 1000.0);
    Serial.println(" KOhms");
    Serial.print("Approximate Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");
    Serial.println("");

    Serial.print("IMU X: ");
    Serial.println(event.orientation.x);
    Serial.print("IMU Y: ");
    Serial.println(event.orientation.y);
    Serial.print("IMU Z: ");
    Serial.println(event.orientation.z);
    Serial.println("");
    */

    printTimer.resetTime();
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  limitedPrint(500);
  telemetry(250);

  //IMU Loop Setup
  //sensors_event_t event;
  //bno.getEvent(&event);

  String lastColor = color;
  loops++;
  switchState = digitalRead(SWITCH_PIN);
  if(switchState == HIGH && switchState != lastSwitchState){
    LEDTimer.resetTime();
  }

  //5 second PWM 2 solenoid / LED test
  if(false){
    if(solenoidTimer.getTime() < 1000){
      if(aCountdown.getTimeLeft() <= 0 && bCountdown.getTimeLeft() <= 0){
        PWMSetup(.75);
      }
    }else if(solenoidTimer.getTime() < 2000){
      if(aCountdown.getTimeLeft() <= 0 && bCountdown.getTimeLeft() <= 0){
        PWMSetup(.5);
      }
    }else if(solenoidTimer.getTime() < 3000){
      if(aCountdown.getTimeLeft() < 0 && bCountdown.getTimeLeft() < 0){
        PWMSetup(-.75);
      }
    }else if(solenoidTimer.getTime() < 4000){
      if(aCountdown.getTimeLeft() < 0 && bCountdown.getTimeLeft() < 0){
        PWMSetup(-.5);
      }
    }else if(solenoidTimer.getTime() < 5000){
      if(aCountdown.getTimeLeft() < 0 && bCountdown.getTimeLeft() < 0){
        PWMSetup(0.0);
      }
    }else{
      solenoidTimer.resetTime();
    }
    PWMLoop();
  }

  //LED Cycler
  if(false){
    if(LEDTimer.getTime() < 1000){
      digitalWrite(DEBUG_LED_1, HIGH);
      color = "Red";
    }else if(LEDTimer.getTime() < 2000){
      digitalWrite(DEBUG_LED_1, LOW);
      digitalWrite(DEBUG_LED_2, HIGH);
      color = "Green";
    }else if(LEDTimer.getTime() < 3000){
      digitalWrite(DEBUG_LED_2, LOW);
      digitalWrite(DEBUG_LED_3, HIGH);
      color = "Blue";
    }else if(LEDTimer.getTime() < 4000){
      digitalWrite(DEBUG_LED_3, LOW);
      digitalWrite(DEBUG_LED_4, HIGH);
      color = "White";
    }else{
      digitalWrite(DEBUG_LED_1, LOW);
      digitalWrite(DEBUG_LED_2, LOW);
      digitalWrite(DEBUG_LED_3, LOW);
      digitalWrite(DEBUG_LED_4, LOW);
      color = "\n";
      LEDTimer.resetTime();
    } 
  }else{
    digitalWrite(DEBUG_LED_1, LOW);
      digitalWrite(DEBUG_LED_2, LOW);
      digitalWrite(DEBUG_LED_3, LOW);
      digitalWrite(DEBUG_LED_4, LOW);
      color = "";
  }

  if(true){
    /*
    if(event.orientation.x > 45 && event.orientation.x < 315){
      digitalWrite(DEBUG_LED_1, HIGH);
    }else{
      digitalWrite(DEBUG_LED_1, LOW);
    }
    if(event.orientation.y > 15 || event.orientation.y < -15){
      digitalWrite(DEBUG_LED_2, HIGH);
    }else{
      digitalWrite(DEBUG_LED_2, LOW);
    }
    if(event.orientation.z > 15 || event.orientation.z < -15){
      digitalWrite(DEBUG_LED_3, HIGH);
    }else{
      digitalWrite(DEBUG_LED_3, LOW);
    }
    */
  }

  if(false){
    /*
    currentX = event.orientation.x;
    targetX = 180;
    errorX = targetX - currentX;
    double inputPower = errorX * (1.0/180.0);
    PWMSetup(inputPower);
    PWMLoop();
    */
  }

  lastSwitchState = switchState;
}