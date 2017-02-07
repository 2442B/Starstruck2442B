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
#pragma config(UART_Usage, UART2, uartVEXLCD, baudRate19200, IOPins, None, None)

#define BCI_USE_TIMER
#define BCI_USE_PID_OPT
#define BCI_USE_POS_PID
#define BCI_USE_MOTORCONTROL

#include "..\BCI\BCI.h"

#pragma systemFile

int lcdButton2;

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

//holdLiftPos() deprecated - removed

void setLeftDrivePower(int power)
{
	addMotor(driveLeftFront);
	addMotor(driveLeftBack);
	startTask(motorSlewRateTask);
	setMotorSpeed(driveLeftFront, power);
	setMotorSpeed(driveLeftBack, power);
}
void setRightDrivePower(int power)
{
	addMotor(driveRightFront);
	addMotor(driveRightBack);
	startTask(motorSlewRateTask);
	setMotorSpeed(driveRightFront, power);
	setMotorSpeed(driveRightBack, power);
}

void driveForTime(int time, int power)
{
	setLeftDrivePower(power);
	setRightDrivePower(power);
	wait1Msec(time);

	setLeftDrivePower(0);
	setRightDrivePower(0);
}

bool PID_Opt_DriveStraight(const int distance, tMotor *leftMotors, tMotor *rightMotors, const unsigned int numMotors, const tSensors leftSensor, const tSensors rightSensor, pos_PID *distancePID, pos_PID *anglePID)
{
	//Save left and right quad values instead of setting them to zero
	const float encoderLeft = SensorValue[leftSensor], encoderRight = SensorValue[rightSensor];

	//Total distance elapsed since start and total angle change since start
	float distanceElapsed = 0, angleChange = 0;
	pos_PID_ChangeSensor(distancePID, &distanceElapsed);
	pos_PID_ChangeSensor(anglePID, &angleChange);

	//Target distance for the distance PID controller
	//Angle PID controller's target is 0
	const int targetDistance = distance;
	pos_PID_SetTargetPosition(distancePID, targetDistance);
	pos_PID_SetTargetPosition(anglePID, 0);

	//If distance PID controller is at target
	bool atTarget = false;

	//Distance that is "close enough" to target
	const int atTargetDistance = 5;

	//Timer for being at target
	timer atTargetTimer;
	timer_Initialize(&atTargetTimer);

	//Timeout period (ms)
	const int timeoutPeriod = 250;

	//Current left and right quad displacements
	float currentLeft, currentRight;

	//Distance and angle PID output
	int distOutput, angleOutput;

	//Loop index
	unsigned int i = 0;

	while (!atTarget)
	{
		//Calculate distance displacement
		currentLeft = SensorValue[leftSensor] - encoderLeft;
		currentRight = SensorValue[rightSensor] - encoderRight;

		//Overall displacement is the average of left and right displacements
		distanceElapsed = (currentLeft + currentRight) / 2.0;

		//Angle change doesn't need to be a real angle, just the difference in displacements
		angleChange = currentRight - currentLeft;

		//Get output from both PID's
		distOutput = pos_PID_StepController(distancePID);
		angleOutput = pos_PID_StepController(anglePID);

		//Set motors to distance PID output with correction from angle PID
		for (i = 0; i < numMotors; i++)
		{
			writeDebugStreamLine("distOutput: %i", distOutput);
			//writeDebugStreamLine("angleOutput: %i", angleOutput);
			motor[*(leftMotors + i)] = distOutput + angleOutput;
			motor[*(rightMotors + i)] = distOutput - angleOutput;
		}

		//Place mark if we're close enough to the target distance
		if (fabs(targetDistance - distanceElapsed) <= atTargetDistance)
		{
			timer_PlaceHardMarker(&atTargetTimer);
		}
		else
		{
			timer_ClearHardMarker(&atTargetTimer);
		}

		//If we've been close enough for long enough, we're there
		if (timer_GetDTFromHardMarker(&atTargetTimer) >= timeoutPeriod)
		{
			atTarget = true;
		}
	}

	for (i = 0; i < numMotors; i++)
		{
			motor[*(leftMotors + i)] = 0;
			motor[*(rightMotors + i)] = 0;
		}

	return true;
}

void driveForDistance(int distance)
{
	tMotor leftMotors[2] = {driveLeftFront, driveLeftBack};
	tMotor rightMotors[2] = {driveRightFront, driveRightBack};

	pos_PID distancePID, anglePID;
	pos_PID_InitController(&distancePID, NULL, 0.4, 0.5, 0.5); //NEED NUMBERS
 	pos_PID_InitController(&anglePID, NULL, 0, 0, 0); //NEED NUMBERS

 	PID_Opt_DriveStraight(distance, leftMotors, rightMotors, 2, leftEncoder, rightEncoder, &distancePID, &anglePID); //TEST FOR NEGATIVE NUMS

 	}

void turnCounterClockwise(int angle)
{
	float conv_angle = angle * (2); //NEED NUMS

	pos_PID leftPID;
	pos_PID rightPID;

	pos_PID_InitController(&leftPID, leftEncoder, 1, 2, 3); //NEED VALUES
	pos_PID_InitController(&rightPID, rightEncoder, 1, 2, 3); //NEED VALUES

	pos_PID_SetTargetPosition(&leftPID, -conv_angle);
	pos_PID_SetTargetPosition(&rightPID, conv_angle);

	int leftPower, rightPower;
	do
	{
		leftPower = pos_PID_StepController(&leftPID);
		rightPower = pos_PID_StepController(&rightPID);
		setLeftDrivePower(leftPower);
		setRightDrivePower(rightPower);
	} while(abs(leftPower) <= 5 && abs(rightPower) <= 5);
}

void turnClockwise(int angle)
{
	float conv_angle = angle * (2); //NEED NUMS

	pos_PID leftPID;
	pos_PID rightPID;

	pos_PID_InitController(&leftPID, leftEncoder, 1, 2, 3); //NEED VALUES
	pos_PID_InitController(&rightPID, rightEncoder, 1, 2, 3); //NEED VALUES

	pos_PID_SetTargetPosition(&leftPID, conv_angle);
	pos_PID_SetTargetPosition(&rightPID, -conv_angle);

	int leftPower;
	int rightPower;
	do
	{
		leftPower = pos_PID_StepController(&leftPID);
		rightPower = pos_PID_StepController(&rightPID);
		setLeftDrivePower(leftPower);
		setRightDrivePower(rightPower);
	} while(abs(leftPower) <= 5 && abs(rightPower) <= 5);
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

void runProgSkills(string side)
{

	//phase I : preloads
	//claw starts on sides
	driveForDistance(-300);
	//program edit - NEEDS TESTING
	//driveForDistance(100, 127, -6); //drives back
	wait1Msec(500);
	setPincerPower(-127); //hopefully grabs star + fallen preload
	wait1Msec(1000);
	setLiftPower(-100);
	wait1Msec(1000);  //waits
	setLiftPower(0);
	driveForDistance(-900); //drives back to fence
	launch(); //launch() leaves claw open


	//Do it again!
	for(int i = 0; i < 3; i++) //2 cubes and 2 groups of stars
	{
		driveForDistance(900);
		setPincerPower(-127);
		wait1Msec(1000);
		driveForDistance(-900);
		launch();
	}

	//phase II : get cube in the middle and launch

	//drive to line up with middle
	/*
	driveForDistance(550, 127, -6);
	wait1Msec(250);

	//turn
	if(side == "right")
	turnClockwise(90);
	else if(side == "left")
	turnCounterClockwise(90);

	wait1Msec(250);

	driveForDistance(400, 127, -6); //moves down center toward cube
	wait1Msec(750);
	setPincerPower(-127); //grabs cube
	wait1Msec(1500); //place to possible change wait time
	setPincerPower(0); //relax pincer motors b/c we are pushing

	driveForDistance(1750, 127, -6); //drives to other end of field
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

	driveForDistance(700, 127, -6);
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
	driveForDistance(250, 127, -6);
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

	driveForDistance(100, 127, -6);
	wait1Msec(250);
	setPincerPower(-127); //grab star - usually not very well
	wait1Msec(250);
	for(int i = 0; i < 1000; i++) //open claw again
	{
	pincerToPos(1030);
	wait1Msec(1);
	}
	driveForDistance(50, 127, -6); //move forward a little
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
	launch(); */
}

/*
auton ideas:

- two seperate autons
- one independent
- one (or more?) to work alongside 2442A, 2442C
- getting stars on the field [ideas below]
- 3 stars in the back?
- stars in the corners
*other teams don't do this*
- move toward stars on the fence holding claw up (possible application of tasks
-

*/
//sides - auton
//format: side; results
//for 2, 4 - see markers in code
//qualifying
//1: right; got cube, not stars
//2: [REDOWNLOAD] right* - comment out for 2442C; got stars [cube: N/A]
//3: right; got stars, not cube
//4: right; got stars, not cube
//Think* we have fixed it
//5: right; got cube, not stars
//6: [REDOWNLOAD] left* - comment out for 2442A; got stars [cube: N/A]
//Skills break
//7: [REDOWNLOAD] left* - comment out for 1615A; missed stars [cube: N/A]
//8: [REDOWNLOAD] right;
//quarterfinals
//1: right; got stars
//semifinals
//1: right; got stars - lost auton
//finals
//1: don't remember


void runCompAuton(string side, int autonNum)
{
	if(side != "right" && side != "left") return;

	driveForDistance(500);
	for(int i = 0; i < 1000; i++)
	{
		pincerToPos(1900); //claw to 90
		wait1Msec(1);
	}
	setPincerPower(0);
	driveForDistance(365); //drive to fence

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
		driveForDistance(900); //parallel to fence

		if(side == "left")
			turnClockwise(125); //turn to face cube
		else if(side == "right")
			turnCounterClockwise(125); //turn to face cube

		wait1Msec(750);
		setLiftPower(0);
		driveForDistance(250); //forward to snag cube
		setPincerPower(-127); //maybe works, hopefully holds pincer shut
		wait1Msec(2000);
		driveForDistance(-300); //back to fence
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
		driveForDistance(100);

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
}

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
