
// This code is for the VEX cortex platform
#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"

// Our Pragmas
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

//Our Functions

void pre_auton()
{

}

task autonomous()
{

}

task usercontrol()
{

	// User control code here, inside the loop

	while (true)
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
				pincerToPos(1700);
			else
				setPincerPower(0);
		}
	}
}
