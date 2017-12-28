/*  Mobile Inverted Pendulum Robot
 *  
 *  This project is inspired by JJROBOTS' B-ROBOT, which is a self balanced Arduino robot with stepper motors.
 *  One can refer to their project at http://jjrobots.com/b-robot 
 *  
 *  Author: Bang-Yu Liu
 *  Date: 31/10/2017
 *  Version: 0.1
 *  License: GPL v2
 *
 *  The MIP robot can raise up itself with the help of two arms, front and back. Throttle and direction commands are
 *  given via an RC transmitter and receiver. The width of PWM signals from the receiver is determined by the capture function of encoder
 *  modules. Then the signals are converted into throttle and direction commands, respectively.
 *  The MIP robot is controlled with a cascade control scheme. The outer loop takes the throttle command as its input
 *  then calculates a demanding tilting angle and feeds the angle command into the inner loop. 
 *  The inner loop stablizes the tilting angle of the robot by adjusting the rotational speed of two stepper motors.
 *  The tilting angle is obtained from the built-in IMU on a 86Duino One. One can access the angle information with 
 *  the FreeIMU1 library.
 */

#include <EEPROM.h>
#include <Encoder.h>
#include <Wire.h>
#include "FreeIMU1.h"
#include "TimerOne.h"
 
/* stepper section
*
*/
// define pins numbers
const int enPin_1 = 27; 
const int dirPin_1 = 28;
const int stepPin_1 = 29;
const int enPin_2 = 30;
const int dirPin_2 = 31;
const int stepPin_2 = 32;
// define variables
int dir = 0;  
int timerPeriod;
int speedM = 0;
int disabler = 1100;

/* stability controller
*
*/
const int MAX_ACCEL = 200;
float kpStab = 8.0;
float kdStab = 5000.0;
long timeOld;
long timeValue;
float dt;
float ypr[3];
float angle = 0.0;
float angleOld = 0.0;
float angleF = 0.0;
float tangle = 0.0;
float tangleOld = 0.0;
float setPointOld = 0.0;
float PIDErrorOld = 0.0;
float PIDErrorOld2 = 0.0;
float ctrlOutput = 0.0;

/* speed controller
*
*/
const int ITERM_MAX_ERROR = 80;
const int ITERM_MAX = 8000;
float kpSpd = 0.04;
float kiSpd = 0.08;
float targetSpd = 0.0;
float targetSpdOld = 0.0;
float fbkSpd = 0.0;
float estSpd = 0.0;
float PIDSpdErrorSum = 0.0;
float errorOld = 0.0;

/* throttle and steering input command
*
*/
const int UPPER_BOUND = 1700;
const int LOWER_BOUND = 1300;
int direction = 0; // 0: straight, -1: left, 1: right.
int steering = 1500;
int throttle = 1100;

void setMotorSpeed(int tspeed)
{ 
 
	int speed;
		
	speedM = tspeed;
	speed = speedM * 13; // rpm * (1600/60) = rpm * 26, 26/2 = 13 due to 2 pulses in one interrupt 

	if (speed==0) {
		dir = 0;
	}
	else if (speed>0)
	{
		timerPeriod = 1000000 / speed;   
		dir = 1;
		digitalWrite(dirPin_1, LOW); // Enables the motor to move in a particular direction
		digitalWrite(dirPin_2, HIGH);
	}
	else
	{
		timerPeriod = 1000000 / (-speed);
		dir = -1;
		digitalWrite(dirPin_1, HIGH); // Enables the motor to move in a particular direction
		digitalWrite(dirPin_2, LOW);
	}
	Timer1.setPeriod(timerPeriod);
}

void ISR_motor()
{
	if (dir == 0) return;
	
	if (direction <= 0) {
		digitalWrite(stepPin_1,HIGH);
	}
	if (direction >= 0) {
		digitalWrite(stepPin_2,HIGH);
	}
	delayMicroseconds(2);
	if (direction <= 0) {
		digitalWrite(stepPin_1,LOW);
	}
	if (direction >= 0) {
		digitalWrite(stepPin_2,LOW);
	}
	
	digitalWrite(stepPin_1,HIGH);
	digitalWrite(stepPin_2,HIGH);
	delayMicroseconds(2); 
	digitalWrite(stepPin_1,LOW);
	digitalWrite(stepPin_2,LOW);
}

// PD controller implementation(Proportional, derivative). DT is in miliseconds
float stabilityPDControl(float DT, float input, float setPoint, float Kp, float Kd)
{
  float error;
  float output;

  error = setPoint - input;

  // Kd is implemented in two parts
  //    The biggest one using only the input (sensor) part not the SetPoint input-input(t-2)
  //    And the second using the setpoint to make it a bit more agressive   setPoint-setPoint(t-1)
  output = Kp * error + (Kd * (setPoint - setPointOld) - Kd * (input - PIDErrorOld)) / DT;
  
  PIDErrorOld2 = PIDErrorOld;
  PIDErrorOld = input;  // error for Kd is only the input component
  setPointOld = setPoint;
  return (output);
}

float speedPIControl(float DT, float input, float setPoint, float Kp, float Ki)
{
  float error;
  float output;

  error = setPoint - input;
  PIDSpdErrorSum += constrain(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
  PIDSpdErrorSum = constrain(PIDSpdErrorSum, -ITERM_MAX, ITERM_MAX);

  output = Kp * error + Ki * PIDSpdErrorSum * DT * 0.001; // DT is in miliseconds
  errorOld = error;
  return (output);
}

// Set the FreeIMU object
FreeIMU1 my3IMU = FreeIMU1();

void setup() {
	Serial.begin(9600);
	
	// Initialize encoder 1 in capture mode
	Enc1.begin(MODE_CAPTURE);
	
	// Initialize the onboard IMU
	Wire.begin();
  
	delay(5);
	my3IMU.init(1); // the parameter enable or disable fast mode
	delay(5);
	
	// Sets the six pins as Outputs
	pinMode(enPin_1, OUTPUT); 
	pinMode(dirPin_1, OUTPUT);
	pinMode(stepPin_1, OUTPUT);  
	digitalWrite(enPin_1, HIGH); // Diarm the coils of the first stepper

	pinMode(enPin_2, OUTPUT); 
	pinMode(dirPin_2, OUTPUT);
	pinMode(stepPin_2, OUTPUT);  
	digitalWrite(enPin_2, HIGH); // Diarm the coils of the second stepper
	
	// Initialize timer1 and attach an ISR to it
	Timer1.initialize(500000);         // initialize timer1, and set a 1/2 second period by default
	Timer1.attachInterrupt(ISR_motor);  // attaches ISR_motor() as a timer overflow interrupt
	
	// get current time with millis()
	timeOld = millis();
}

void loop() {
	
	// System on/off
	disabler = Enc1.pulseIn(2, HIGH); // pin 20 of 86duino One
	
	if (disabler > 1300) {
		digitalWrite(enPin_1, LOW); // Arm the coils of the first stepper
		digitalWrite(enPin_2, LOW); // Arm the coils of the second stepper
	}
	else {
		digitalWrite(enPin_1, HIGH); // Disarm the coils of the first stepper
		digitalWrite(enPin_2, HIGH); // Disarm the coils of the second stepper
		PIDSpdErrorSum = 0.0; // reset I speed controller
	}
	
	// Steering cmd
	steering = Enc1.pulseIn(1, HIGH); // pin 19 of 86duino One
	
	if (steering > UPPER_BOUND) {
		direction = 1; // turn right
	}
	else if (steering < LOWER_BOUND) {
		direction = -1; // turn left
	}
	else {
		direction = 0; // go straight
	}
	
	
	// Throttle cmd
	throttle = Enc1.pulseIn(0, HIGH); // pin 18 of 86duino One
		
	if (1400 < throttle && throttle < 1600) {
		throttle = 1500;
	}
	else {
		throttle = throttle;
	}
	
	if (1400 < throttle && throttle < 1600) {
		targetSpd = 0.0;
	}
	else if (throttle <= 1400) {
		targetSpd = (float)((throttle - 1400) / 2);
	}
	else {
		targetSpd = (float)((throttle - 1600) / 2);
	}
	targetSpdOld = 0.2 * targetSpd + 0.8 * targetSpdOld;
	targetSpdOld = constrain(targetSpdOld, -100.0, 100.0);
	
	// Update time
	timeValue = millis();
	dt = (float)(timeValue - timeOld);
	timeOld = timeValue;
	
	// Tilting angle raw and filtered inputs
	my3IMU.getYawPitchRoll(ypr);
	angle = ypr[1];
	angleF = 0.025 * angle + 0.975 * angleF;
	angleOld = angleF;
	
	// Speed raw and filtered inputs
	fbkSpd = -1.2 * ctrlOutput;
	estSpd = 0.1 * fbkSpd + 0.9 * estSpd;
	
	// Outter loop controller for the speed
	tangle = speedPIControl(dt, estSpd, targetSpdOld, kpSpd, kiSpd);
	tangleOld = 0.1 * tangle + 0.9 * tangleOld;
	tangleOld = constrain(tangleOld, -30.0, 30.0);
	
	// Inner loop controller for the stability of the tilting
	ctrlOutput = stabilityPDControl(dt, angleF, tangleOld, kpStab, kdStab);
	ctrlOutput = constrain(ctrlOutput, -300.0, 300.0);
	
	// Control output
	setMotorSpeed((int)ctrlOutput);
}
