#RCS Team Young Code Documentation
##Error Codes:
###Setup:
####Blue LED Only:
Starting the sensors (IMU, BME, and GPS)
Should turn on and turn off quickly, if it lasts 5 seconds something is probably wrong. Likely the power is not on.
###White LED Only:
The IMU needs to be calibrated. Move board/payload around until it goes off.
###All Board Lights in 1 Second Cycle:
One or more of the sensors (IMU, BME, or GPS) is not working.
Payload needs to be power cycled to try again. Likely cause is the power is not on.
###Loop:
####All Board Lights In 2 Second Cycle:
Something happened with the enum and it got set to a state that does not exist. Almost definitely a code problem

##Timer
###Constructor:
Timer
If you use parenthesis it gets mad
###Methods:
####void reset()
Resets start of the timer to current time
####long getTime()
Returns time since start or last reset of timer in millis
####long getTimeMillis()
Returns time since start or last reset of timer in millis
####long getTimeSeconds()
Returns time since start or last reset of timer in seconds
####long getTimeMinutes()
Returns time since start or last reset in minutes
####long getTimeHours()
Returns time since start or last reset in hours
####long timerHours()
Returns time since start or last reset in hours
To be used in timer string
####long timerMinutes()
Returns the minutes of the time not including hours
To be used in timer string
####long timerSeconds()
Returns the seconds of the time not accounted for by hours or minutes
To be used in timer string
####long timerMillis()
Returns the milliseconds of the time not accounted for by hours, minutes, or seconds
To be used in timer string
####String printableTimer(){
Returns a string of the time in the format hh:mm:ss.sss
Includes padding zeros
###Instances:
missionTime;
hazardTimer;
Only use in loop1 or things could break
errorTimer;
Used to control LEDs when they are showing an error
logTimer;
Timer for the data logging
oPIDTimer;
Timer for the orientation PID
vPIDTimer;
Timer for the velocity PID

##CountdownTimer
Child of Timer
If creating but using later, input 0
###Constructor:
CountdownTimer(long time)
CountdownTimer()
If no time is given, lets it not crash
###Methods:
####void reset(long newTime)
Changes the countdown to whatever the newTime is
If you call CountdownTimer.reset(5000), it will 
####bool isDone(){
If timer is less than 0, returns true
If timer is not done yet, return false
####long getTime()
Returns time left on timer in milliseconds
####long getTimeMillis()
Returns time left on timer in milliseconds
You might be able to use the other types of get time from Timer, but I have no clue if that will work.
###Instances:
sensorSetup;
aOnCountdown;
aCountdown;
bOnCountdown;
bCountdown;

##Functions:
###IMUStuff();
Makes sure that the imu data is up to date

###PWMSetup(float percent)
Takes in percentage for how strong you want the signal to be
Positive for Solenoid CW and negative for Solenoid CCW
Starts timers for PWMLoop to use to control solenoids
If either timer is running, function will return
Deadzone from -.075 to .075, if between these values all timers will be set to 0 and the solenoids will be off
If value is greater than .9 or less than -.9, value will be set to .9 or -.9 to save gas
The smaller of either on or off will be set to the min cycle time, while the larger will be set to the ratio of the larger value and the smaller value times the minimum time

###PWMLoop()
To be used on every loop to act on the timers that are started in PWMSetup
If aOnTimer is still going it will open up the CW Solenoid, if bOnTimer is going it will open the CCW Solenoid, and if neither timer is going it will close both

###errorBlink(long frequency)
Blinks the two debug leds and the integrated LED, with each being on as long as is inputted to frequency.

###loggerPrint(long frequency)
Prints the required telemetry to the data logger as often as is passed into the frequency

###limitedPrint(long frequency)
Prints everything inside it only as often as is passed into frequency
Checks if the printTimer.getTime() > frequency, if true will print everything and then reset the timer.

##Need To Know:
###setup()
Always reset all regular timers at the end of Setup()
Make sure to set the gyro to degrees and then DON”T DO IT AGAIN
###setup1()
Don’t exit setup1() until setup is complete
###loop()
###loop1()

##Random Stuff:
###Flight State Enum
Enum that holds the current flight state
####INIT
Mission State during Setup
Continues to FLIGHT_READY as soon as the setup function is over
####FLIGHT_READY
Exit Condition to ASCENT:
Upwards velocity is greater than 3m/s or altitude is greater than 1000m
####ASCENT
Exit Condition to CONTROLLED:
BME altitude is greater than 18km or GPS altitude is greater than 18km
####CONTROLLED
Exit Condition to FREEFALL:
Altitude is less than 18 km or vertical velocity is negative
####FREEFALL
Exit Condition to LANDED:
Vertical Velocity is 0 (or close to it, likely 0.5m/s threshold) or altitude is less than 200m
####LANDED
Never exits, should blink lights until found and powered off

###Semaphores
Use to make sure loop and loop1 don’t access the same thing at the same time.
####Use:
while(semaphore);
semaphore = true;
Use the thing;
semaphore = false;
####Ones I’m Using:
bool missionTimeSem;
Use when getting total mission time
bool flightStateSem;
Use when getting the flightState
bool bmeAltSem;
Use when getting altitude from bme
bool bmeTempSem;
Use when getting temp from bme
bool imuAccelSem;
Use when getting acceleration from imu
bool imuGyroSem;
Use when getting gyroscope data from imu
bool imuOrientSem;
bool gpsLatSem;
Use when getting latitude from gps
bool gpsLongSem;
Use when getting longitude from gps
bool gpsAltSem;
Use when getting altitude from gps
