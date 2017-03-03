#pragma config(Sensor, in1,    liftPoten,      sensorNone)
#pragma config(Sensor, in2,    leftClawPoten,  sensorPotentiometer)
#pragma config(Sensor, in3,    rightClawPoten, sensorPotentiometer)
#pragma config(Sensor, dgtl1,  leftEncoder,    sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  rightEncoder,   sensorQuadEncoder)
#pragma config(Motor,  port1,           rightPincer,   tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           liftRightExtremes, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           liftRightMid,  tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           driveRightFront, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           driveRightBack, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           driveLeftFront, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           driveLeftBack, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           liftLeftExtremes, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           liftLeftMid,   tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port10,          leftPincer,    tmotorVex393_HBridge, openLoop, reversed)

#pragma platform(VEX2)

#pragma competitionControl(Competition)

#include "Vex_Competition_Includes.c"

void setLiftPower(int power)
{
	motor[liftRightExtremes] = power;
	motor[liftRightMid] = power;
	motor[liftLeftExtremes] = power;
	motor[liftLeftMid] = power;
}
void liftToPos(int angle)
{
	int currAngle = SensorValue[liftPoten];
	float dif = currAngle - angle;
	int power = (int) (0.15 * dif);
	if(abs(power) >= 5)
		setLiftPower(power);
}
void setLeftDrivePower(int power)
{
	motor[driveLeftBack] = power;
	motor[driveLeftFront] = power;
}
void setRightDrivePower(int power)
{
	motor[driveRightBack] = power;
	motor[driveRightFront] = power;
}
void driveForTime(int time, int power)
{
	setLeftDrivePower(power);
	setRightDrivePower(power);
	wait1Msec(time);

	setLeftDrivePower(0);
	setRightDrivePower(0);
}
void driveForDistance(int numClicks)
{
	SensorValue[rightEncoder] = 0;
	SensorValue[leftEncoder] = 0;
	int leftError = numClicks - SensorValue(leftEncoder);
	float leftConst = 0.9;
	int rightError = numClicks - SensorValue(rightEncoder);
	float rightConst = 0.75;
	while(fabs(leftError * leftConst) > 0 && fabs(rightError * rightConst) > 0)
	{
		leftError = numClicks - SensorValue(leftEncoder);
		rightError = numClicks - SensorValue(rightEncoder);
		setLeftDrivePower((int)(leftConst * leftError) + 63 *(sgn(numClicks)));
		setRightDrivePower((int)(rightConst * rightError) + 63 *(sgn(numClicks)));
	}
	//clean
	setRightDrivePower(0);
	setLeftDrivePower(0);
}

void turnClockwise(int angle)
{
	SensorValue(leftEncoder) = 0;
	SensorValue(rightEncoder) = 0;

	float convAngle = (angle * 232) / 90;
	int leftEn = 0;
	int rightEn = 0;

	while(leftEn < convAngle || rightEn < convAngle)
	{
		leftEn = SensorValue(leftEncoder);
		rightEn = SensorValue(rightEncoder);

		setLeftDrivePower(127);
		setRightDrivePower(-127);

		if(leftEn >= convAngle)
			setLeftDrivePower(0);
		if(rightEn <= -convAngle)
			setRightDrivePower(0);
	}
	setLeftDrivePower(-10);
	setRightDrivePower(10);
}

void turnCounterClockwise(int angle)
{
	SensorValue(leftEncoder) = 0;
	SensorValue(rightEncoder) = 0;

	int leftEn = 0;
	int rightEn = 0;

	float convAngle = (angle * 190) / 90;

	while(leftEn > -convAngle || rightEn > -convAngle)
	{
		leftEn = SensorValue(leftEncoder); //going to negative
		rightEn = SensorValue(rightEncoder); //going to negative as well

		setLeftDrivePower(-127);
		setRightDrivePower(127);

		if(leftEn <= -convAngle)
			setLeftDrivePower(0);
		if(rightEn <= -convAngle)
			setRightDrivePower(0);
	}
	setLeftDrivePower(10);
	setRightDrivePower(-10);
}

//Helper - doen't have limits, don't use
//Negatives close pincers, positives open
void setPincerPower(int power)
{
	motor[leftPincer] = power;
	motor[rightPincer] = power;
}
//pincerToPos() checks how close the pincer is to the desired angle and sets it to a proportional power
//as it approaches the angle, the power decreases
//when it is at that angle, the power will be 0
void pincerToPos(int angle)
{
	int currRightAngle = SensorValue[rightClawPoten];
	float rightDif = currRightAngle - angle;
	//if(rightDif <= 10)//Buffer - should reduce bouncing
	//	rightDif = 0;
	int rightPower = (int) -(0.25 * rightDif);
	if(abs(rightPower) >= 5)
		motor[rightPincer] = rightPower;

	int currLeftAngle = SensorValue[leftClawPoten];
	float leftDif = currLeftAngle - angle; //POSSIBLE ERROR
	//if(leftDif <= 10)//Buffer - should reduce bouncing
	//	leftDif = 0;
	int leftPower = (int) -(0.25 * leftDif);
	if(abs(leftPower) >= 5)
		motor[leftPincer] = leftPower;

	//writeDebugStreamLine("left power: %d", leftPower);
	//if(angle == 1030)
		//writeDebugStreamLine("right power: %d, rightPot: %d, left power: %d, leftPot: %d", rightPower, rightClawPoten, leftPower, leftClawPoten);
}

//distance can be negative to drive backwards
void driveWithPincerCont(int distance, int power, int angle, int maxTime )
{
	SensorValue[leftEncoder] = 0;
	SensorValue[rightEncoder] = 0;
	int leftEn = 0;
	int rightEn = 0;
	clearTimer(T2);

	while( fabs(leftEn) < fabs(distance) || fabs(rightEn) < fabs(distance) ||
			 ( fabs(SensorValue[leftClawPoten] - angle) > 5 || fabs(SensorValue[rightClawPoten] - angle) > 5) )
	{
		if(fabs(leftEn) < fabs(distance) || fabs(rightEn) < fabs(distance))
		{
			setLeftDrivePower(power * sgn(distance));
			setRightDrivePower(power * sgn(distance));
			leftEn = SensorValue[leftEncoder];
			rightEn = -SensorValue[rightEncoder];
		}
		else
		{
			setLeftDrivePower(0);
			setRightDrivePower(0);
		}
		if(fabs(SensorValue[leftClawPoten] - angle) > 5 || fabs(SensorValue[rightClawPoten] - angle) > 5)
		{
			motor[leftPincer] = -127 * (SensorValue[leftClawPoten] - angle);
			motor[rightPincer] = -127 * (SensorValue[rightClawPoten] - angle);
		}
		else
		{
			motor[leftPincer] = 0;
			motor[rightPincer] = 0;
		}

		if(time1[T2] > maxTime)
			break;
	}
}

void launch()
{
	int currAngle = SensorValue[liftPoten];
	while(currAngle > 600) //Lift to drop pos - FIX NUMBER
	{
		setLiftPower(127);
		currAngle = SensorValue[liftPoten];
		wait1Msec(10);
	}
	setLiftPower(-10);
	wait1Msec(250);
	setLiftPower(0);
	for(int i = 0; i < 1500; i++) //Open claw
	{
		pincerToPos(1030);
		wait1Msec(1);
	}
	setPincerPower(0);
	while(currAngle < 3200) //Lift down - FIX NUMBER
	{
		setLiftPower(-127);
		currAngle = SensorValue[liftPoten];
		wait1Msec(10);
	}
	setLiftPower(0);
}
void fastLaunch()
{
	int currAngle = SensorValue[liftPoten];
	while(currAngle > 600) //Lift to drop pos - FIX NUMBER
	{
		setLiftPower(127);
		currAngle = SensorValue[liftPoten];
		wait1Msec(10);
	}
	setLiftPower(-10);
	wait1Msec(250);
	setLiftPower(0);
	for(int i = 0; i < 1100; i++) //Open claw
	{
		pincerToPos(1030);
		wait1Msec(1);
	}
	setPincerPower(0);
	while(currAngle < 3200) //Lift down - FIX NUMBER
	{
		setLiftPower(-127);
		currAngle = SensorValue[liftPoten];
		wait1Msec(10);
	}
	setLiftPower(0);
}

//Programming Skills - broken into 4 "phases"
//phase descriptions in method body
void runProgSkills(string side)
{
	//phase I : preloads (order: 2 stars, cube, 2 stars, cube)
	driveForDistance(-70); //drive up a little to give space

	//grab
	//setPincerPower(-127);
	//wait1Msec(1500);
	for(int i = 0; i < 1500; i++)
	{
		pincerToPos(25);
		wait1Msec(1);
	}

	driveWithPincerCont(-500, 127, 0, 1500); //drives back to fence
	setPincerPower(-127);
	setLeftDrivePower(0);
	setRightDrivePower(0);
	launch();

	//Do for rest of preloads
	for(int i = 0; i < 3; i++)
	{
		driveForDistance(500);
		for(int i = 0; i < 1000; i++)
		{
			pincerToPos(75);
			wait1Msec(1);
		}

		driveWithPincerCont(-500, 127, 0, 1500);
		setPincerPower(-127);
		setLeftDrivePower(0);
		setRightDrivePower(0);
		launch();
	}

	//phase II : get cube in the middle and launch

	driveForDistance(350);
	wait1Msec(250);

	//turn
	if(side == "right")
		turnClockwise(45);
	else if(side == "left")
		turnCounterClockwise(45);

	wait1Msec(250);

	driveForDistance(300); //moves down center toward cube

	for(int i = 0; i < 1500; i++)
	{
		pincerToPos(75);
		wait1Msec(1);
	}

	setPincerPower(0); //relax pincer motors b/c we are pushing

	setLiftPower(100);

	if(side == "right")
		turnCounterClockwise(60);
	else if(side == "left")
		turnClockwise(60);

	setLiftPower(127);
	driveForDistance(-100);

	launch();
	/*
	//turn
	if(side == "right")
		turnCounterClockwise(140);
	else if(side == "left")
		turnClockwise(140);

	wait1Msec(250);

	//drives toward fence and launches
	driveForDistance(-450);

	launch(); */

	//phase III : move back and get last cube and launches
	driveForDistance(700);
	wait1Msec(250);

	setPincerPower(-127);
	wait1Msec(250);

	driveForDistance(-650);

	launch();

	//phase IV : spin and grab star
	//sets pincers to be on sides of robot
	for(int i = 0; i < 1500; i++)
	{
		pincerToPos(3100);
		wait1Msec(1);
	}

	//move away from fence a little bit
	driveForDistance(250);
	wait1Msec(250);

	//turn
	if(side == "right")
		turnClockwise(115);
	else if(side == "left")
		turnCounterClockwise(115);

	wait1Msec(250);

	for(int i = 0; i < 1000; i++)
	{
		pincerToPos(1030);
		wait1Msec(1);
	}

	driveForDistance(100);
	wait1Msec(250);

	setPincerPower(-127); //grab star - usually not very well
	wait1Msec(250);

	for(int i = 0; i < 1000; i++) //open claw again
	{
		pincerToPos(1030);
		wait1Msec(1);
	}

	driveForDistance(50); //move forward a little
	wait1Msec(250);

	setPincerPower(-127); //grab cube again
	wait1Msec(500);

	//turn - may need to change
	if(side == "right")
		turnCounterClockwise(115);
	else if(side == "left")
		turnClockwise(115);

	wait1Msec(250);

	driveForDistance(-250); //may need changing
	launch();
}

//focus - limits wait times and use of pincer power
void runMainCompAuton(string side)
{
	clearTimer(T1);

	if(side != "right" && side != "left")
		return;

	driveWithPincerCont(550, 127, 0, 1500);

	setLiftPower(100);

	if(side == "right")
		turnCounterClockwise(120);
	else if(side == "left")
		turnClockwise(120);

	setLiftPower(127);
	driveForDistance(-400); //holding cube, drag to launch

	fastLaunch();

	if(side == "right")
		turnClockwise(10);
	else if(side == "left")
		turnCounterClockwise(5);

	driveForDistance(500);
	setPincerPower(-127);
	wait1Msec(500);
	driveForDistance(-500);

	int currAngle = SensorValue[liftPoten];
	while(currAngle > 800) //Lift to drop pos - FIX NUMBER
	{
		setLiftPower(127);
		currAngle = SensorValue[liftPoten];
		wait1Msec(10);
	}
	setLiftPower(-10);
	wait1Msec(250);
	setLiftPower(0);
	for(int i = 0; i < 1100; i++) //Open claw
	{
		pincerToPos(1030);
		wait1Msec(1);
	}

	setPincerPower(0);

	writeDebugStreamLine("%i", time1[T1]);
}

//drive to knock off stars - no string
void runBasicCompAuton()
{
	driveWithPincerCont(470, 127, 2000, 1500);
	setPincerPower(0);
	for(int i = 0; i < 2000; i++)
	{
		liftToPos(1300);
		wait1Msec(1);
	}
	driveForDistance(-100);
	for(int i = 0; i < 750; i++)
	{
		liftToPos(1500);
		wait1Msec(1);
	}
	setLiftPower(0);
}


void pre_auton()
{

}

task autonomous()

{
	string side = "left";
	runMainCompAuton(side);
	//runBasicCompAuton();
	//runProgSkills(side);
}

task usercontrol()
{
	bool pinpointDrive = false;
	while(true)
	{
		int setPoint = 3000;

		//Buttons and Joysticks
		int  rightJoy = vexRT[Ch2];
		int  leftJoy = vexRT[Ch3];
		word rightTriggerUp = vexRT[Btn6U]; //for up lift
		word rightTriggerDown = vexRT[Btn6D]; //for down lift
		word leftTriggerUp = vexRT[Btn5U]; //for pincer close
		word leftTriggerDown = vexRT[Btn5D]; //for pincer open
		word btnEightDown = vexRT[Btn8D]; //for lift to set point
		word btnSevenUp = vexRT[Btn7U]; //for folding claws
		word btnSevenD = vexRT[Btn7D]; //180 degrees

		//Drive Motors
		if(leftJoy > 15 || leftJoy < -15) //dead zones
		{
			if(pinpointDrive)
				setLeftDrivePower(2 * leftJoy / 3);
			else
				setLeftDrivePower(leftJoy);
		}
		else
			setLeftDrivePower(0);
		if(rightJoy > 15 || rightJoy < -15) //dead zones
		{
			if(pinpointDrive)
				setRightDrivePower(2 * rightJoy / 3);
			else
				setRightDrivePower(rightJoy);
		}
		else
			setRightDrivePower(0);

		//Lift Motors

		if(rightTriggerUp == 1)
			setLiftPower(127);
		else if(rightTriggerDown == 1)
			setLiftPower(-127);
		else if(btnEightDown == 1)
			liftToPos(setPoint);
		else setLiftPower(0);

		//pincer
		if(leftTriggerDown == 1)
			pincerToPos(0);
		else if(leftTriggerUp == 1)
			pincerToPos(1030);
		else if(btnSevenUp == 1)
			pincerToPos(3100);
		else if(btnSevenD == 1)
			pincerToPos(1500);
		else
			setPincerPower(0);
	}
}
