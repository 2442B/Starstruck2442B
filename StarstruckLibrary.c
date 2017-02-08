#pragma config(Sensor, in1,    liftPoten,      sensorPotentiometer)
#pragma config(Sensor, in3,    rightClawPoten, sensorPotentiometer)
#pragma config(Sensor, in2,    leftClawPoten,  sensorPotentiometer)
#pragma config(Sensor, dgtl1,  leftEncoder,    sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  rightEncoder,   sensorQuadEncoder)
#pragma config(Motor,  port1,           rightPincer,   tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           liftRightExtremes, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           liftRightMid,  tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           driveRightFront, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           driveRightBack, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           driveLeftFront, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           driveLeftBack, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           liftLeftExtremes, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           liftLeftMid,   tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port10,          leftPincer,    tmotorVex393_HBridge, openLoop, reversed)

#define BCI_USE_TIMER
#define BCI_USE_POS_PID

#include "..\BCI\BCI.h"
#include "..\Starstruck2442B\drivingFunctions.c"
#include "..\Starstruck2442B\turningFunctions.c"

#pragma systemFile

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

/*
______   __                               ______                   __                 ______     __                           __                    __    __
/      \ |  \                             /      \                 |  \               /      \   |  \                         |  \                  |  \  |  \
|  $$$$$$\| $$  ______   __   __   __     |  $$$$$$\  ______    ____| $$  ______      |  $$$$$$\ _| $$_     ______    ______  _| $$_     _______     | $$  | $$  ______    ______    ______
| $$   \$$| $$ |      \ |  \ |  \ |  \    | $$   \$$ /      \  /      $$ /      \     | $$___\$$|   $$ \   |      \  /      \|   $$ \   /       \    | $$__| $$ /      \  /      \  /      \
| $$      | $$  \$$$$$$\| $$ | $$ | $$    | $$      |  $$$$$$\|  $$$$$$$|  $$$$$$\     \$$    \  \$$$$$$    \$$$$$$\|  $$$$$$\\$$$$$$  |  $$$$$$$    | $$    $$|  $$$$$$\|  $$$$$$\|  $$$$$$\
| $$   __ | $$ /      $$| $$ | $$ | $$    | $$   __ | $$  | $$| $$  | $$| $$    $$     _\$$$$$$\  | $$ __  /      $$| $$   \$$ | $$ __  \$$    \     | $$$$$$$$| $$    $$| $$   \$$| $$    $$
| $$__/  \| $$|  $$$$$$$| $$_/ $$_/ $$    | $$__/  \| $$__/ $$| $$__| $$| $$$$$$$$    |  \__| $$  | $$|  \|  $$$$$$$| $$       | $$|  \ _\$$$$$$\    | $$  | $$| $$$$$$$$| $$      | $$$$$$$$
\$$    $$| $$ \$$    $$ \$$   $$   $$     \$$    $$ \$$    $$ \$$    $$ \$$     \     \$$    $$   \$$  $$ \$$    $$| $$        \$$  $$|       $$    | $$  | $$ \$$     \| $$       \$$     \
\$$$$$$  \$$  \$$$$$$$  \$$$$$\$$$$       \$$$$$$   \$$$$$$   \$$$$$$$  \$$$$$$$      \$$$$$$     \$$$$   \$$$$$$$ \$$         \$$$$  \$$$$$$$      \$$   \$$  \$$$$$$$ \$$        \$$$$$$$

*/


//Helper - doen't have limits, don't use
//Negatives close pincers, positives open
void setPincerPower(int power)
{
	motor[leftPincer] = power;
	motor[rightPincer] = power;
}


void pincerToPos(int angle)
{
	int currRightAngle = SensorValue[rightClawPoten];
	float rightDif = currRightAngle - angle;
	int rightPower = (int) -(0.25 * rightDif);
	if(abs(rightPower) >= 5)
		motor[rightPincer] = rightPower;

	int currLeftAngle = SensorValue[leftClawPoten];
	float leftDif = currLeftAngle - angle + 300; //POSSIBLE ERROR
	int leftPower = (int) -(0.25 * leftDif);
	if(abs(leftPower) >= 5)
		motor[leftPincer] = leftPower;
}

void launch()
{
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
/*
void runProgSkills(string side)
{

	//phase I : preloads
	//claw starts on sides
	driveStraight(-300);
	//program edit - NEEDS TESTING
	//driveStraight(100, 127, -6); //drives back
	wait1Msec(500);
	setPincerPower(-127); //hopefully grabs star + fallen preload
	wait1Msec(1000);
	setLiftPower(-100);
	wait1Msec(1000);  //waits
	setLiftPower(0);
	driveStraight(-900); //drives back to fence
	launch(); //launch() leaves claw open


	//Do it again!
	for(int i = 0; i < 3; i++) //2 cubes and 2 groups of stars
	{
		driveStraight(900);
		setPincerPower(-127);
		wait1Msec(1000);
		driveStraight(-900);
		launch();
	}

	//phase II : get cube in the middle and launch

	//drive to line up with middle
	driveStraight(550, 127, -6);
	wait1Msec(250);

	//turn
	if(side == "right")
		turnClockwise(90);
	else if(side == "left")
		turnCounterClockwise(90);

	wait1Msec(250);

	driveStraight(400, 127, -6); //moves down center toward cube
	wait1Msec(750);
	setPincerPower(-127); //grabs cube
	wait1Msec(1500); //place to possible change wait time
	setPincerPower(0); //relax pincer motors b/c we are pushing

	driveStraight(1750, 127, -6); //drives to other end of field
	setPincerPower(-127); //grabs cube again
	wait1Msec(750); //longer wait time - turning with a cube

	//turn
	if(side == "right")
		turnCounterClockwise(140);
	else if(side == "left")
		turnClockwise(140);

	wait1Msec(250);

	//drives toward fence and launches
	driveBackForDistance(-450, -127,  -6);
	launch();

	//phase III : move back and get last cube and launches

	driveStraight(700, 127, -6);
	wait1Msec(250);
	setPincerPower(-127);
	wait1Msec(250);
	driveBackForDistance(-650, -127, 6);
	launch();

	//phase IV : spin and grab star

	//sets pincers to be on sides of robot

	for(int i = 0; i < 1500; i++)
	{
		pincerToPos(3200);
		wait1Msec(1);
	}

	//move away from fence a little bit
	driveStraight(250, 127, -6);
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

	driveStraight(100, 127, -6);
	wait1Msec(250);
	setPincerPower(-127); //grab star - usually not very well
	wait1Msec(250);
	for(int i = 0; i < 1000; i++) //open claw again
	{
		pincerToPos(1030);
		wait1Msec(1);
	}
	driveStraight(50, 127, -6); //move forward a little
	wait1Msec(250);
	setPincerPower(-127); //grab cube again
	wait1Msec(500);

	//turn - may need to change
	if(side == "right")
		turnCounterClockwise(115);
	else if(side == "left")
		turnClockwise(115);

	wait1Msec(250);
	driveBackForDistance(-250, -127, 6); //may need changing
	launch();
}


void runCompAuton(string side, int autonNum)
{
	if(side != "right" && side != "left") return;

	driveStraight(500);
	for(int i = 0; i < 1000; i++)
	{
		pincerToPos(1900); //claw to 90
		wait1Msec(1);
	}
	setPincerPower(0);
	driveStraight(365); //drive to fence

	for(int i = 0; i < 750; i++)
	{
		liftToPos(1900); //lift up to knock  -- NEEDS NEW VALUES
		wait1Msec(1);
	}
	setLiftPower(0); //relax lift motors
	wait1Msec(1000);

	//puts pincers against sides
	for(int i = 0; i < 1000; i++)
	{
		pincerToPos(3200);
		wait1Msec(1);
	} //close claw to get out of way


	if(autonNum == 1) //main auton - will only run if running auton "1"
	{
		if(side == "left")
			turnClockwise(100); //90 deg, but kinda guess and check - turn right
		else if(side == "right")
			turnCounterClockwise(100); //turn left
		wait1Msec(500);

		liftToPos(3200);
		driveStraight(900); //parallel to fence

		if(side == "left")
			turnClockwise(125); //turn to face cube
		else if(side == "right")
			turnCounterClockwise(125); //turn to face cube

		wait1Msec(750);
		setLiftPower(0);
		driveStraight(250); //forward to snag cube
		setPincerPower(-127); //maybe works, hopefully holds pincer shut
		wait1Msec(2000);
		driveStraight(-300); //back to fence
		setLiftPower(127);

		//CHANGED during competition
		wait1Msec(1500);
		setLiftPower(0);

		for(int i = 0; i < 1000; i++)
		{
			pincerToPos(1030); //open pincer (should drop cube)
			wait1Msec(1);
		}
		wait1Msec(250);
		for(int i = 0; i < 250; i++)
		{
			pincerToPos(0);
			wait1Msec(1);
		}
		//starts to drive back
		driveStraight(100);

	} //end of section included for auton "1"

	else if(autonNum == 2) //will only run if running auton "2"
	{
		//spin 180 to prepare for driver control only when commenting out the following code is commented,
		if(side == "right")
			turnCounterClockwise(180);
		else if(side == "left")
			turnClockwise(180);
	} //end of section included for auton "2"

	//lowers lift
	for(int i = 0; i < 750; i++)
	{
		liftToPos(3200);
		wait1Msec(1);
	}

	setLiftPower(0); //stops lift motors
	setPincerPower(0); //stops lift motors

	//Clean
	setLeftDrivePower(0);
	setRightDrivePower(0);
} */

task usercontrol()
{
	// User control code here, inside the loop

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
			pincerToPos(3200);
		else if(btnSevenD == 1)
			pincerToPos(1700);
		else
			setPincerPower(0);
	}
}
