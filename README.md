# TeamYoungRCSTimer
Constructor:
Timer(unsigned long initTime)
Methods:
getTime()
Returns time since start or last reset of timer
resetTime()
Resets start of the timer to current time
Instances:
LEDTimer(currentTime);
bmeTimer(currentTime);
solenoidTimer(currentTime);
PWMTimer(currentTime);

CountdownTimer
Constructor:
CountdownTimer(unsigned long time)
Methods:
getTimeLeft()
	Returns time left on timer
changeTimer(unsigned long newTime)
	Changes how long the timer is
Instances:
aCountdown();
bCountdown();
aOnCountdown();
aOffCountdown();
bOnCountdown();
bOffCountdown();

Functions:
PWMSetup(double percent)
Takes in percentage for how strong you want the signal to be
Positive for solenoid A and negative for solenoid B
Starts timers for PWM
Use in conjunction with PWMLoop
Only use if previous countdown is over
Add else statement


PWMLoop()
To be used on every loop to act on the timers that are started in PWMSetup
