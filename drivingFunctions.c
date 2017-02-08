#ifndef DRIVINGFUNCTIONS_C_INCLUDED

#define DRIVINGFUNCTIONS_C_INCLUDED


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
void setAllMotorPower(int power)
{
	setLeftDrivePower(power);
	setRightDrivePower(power);
}

/**

 * Drives in a straight line for a distance in mm

 * @param  distance Distance to drive for (mm)

 */

void driveStraight(const long distance)

{

	//Save left and right quad values instead of setting them to zero

	const long encoderLeft = SensorValue[leftEncoder], encoderRight = SensorValue[rightEncoder];



	//Total distance elapsed since start and total angle change since start

	float distanceElapsed = 0, angleChange = 0;

	float lastDistance = 0;



  //Conversion between encoder degrees and base_link mm

  //const float conv = 1.311250;
	const float conv = 1;



	//Target distance for the distance PID controller

	//Angle PID controller's target is 0

	int targetDistance = distance * conv;



	pos_PID distancePID, anglePID;



	if (distance <= 800)

	{

		pos_PID_InitController(&distancePID, &distanceElapsed, 0.25, 0.0, 0.0);

		pos_PID_InitController(&anglePID, &angleChange, 0, 0, 0);

	}

	else

	{

		pos_PID_InitController(&distancePID, &distanceElapsed, 0.25, 0.0, 0.0);

		pos_PID_InitController(&anglePID, &angleChange, 0, 0, 0);

	}



	pos_PID_SetTargetPosition(&distancePID, targetDistance);

	pos_PID_SetTargetPosition(&anglePID, 0);



	//If distance PID controller is at target

	bool atTarget = false;



	//Distance that is "close enough" to target

	const int atTargetDistance = 15;



	//Threshold for not moving

	const int threshold = 2;



	//Timer for being at target

	timer atTargetTimer;

	timer_Initialize(&atTargetTimer);



	//Timeout period (ms)

	const int timeoutPeriod = 250;



	//Current left and right quad displacements

	long currentLeft, currentRight;



	//Distance and angle PID output

	int distOutput, angleOutput;



	while (!atTarget)

	{

		//Calculate distance displacement

		currentLeft = SensorValue[leftEncoder] - encoderLeft;

		currentRight = SensorValue[rightEncoder] - encoderRight;



		//Overall displacement is the average of left and right displacements

		distanceElapsed = (currentLeft + currentRight) / 2.0;



		//Angle change doesn't need to be a real angle, just the difference in

		//displacements

		angleChange = currentLeft - currentRight;



		//Get output from both PID's

		distOutput = pos_PID_StepController(&distancePID);

		angleOutput = pos_PID_StepController(&anglePID);



		//Set motors to distance PID output with correction from angle PID

		setLeftDrivePower(distOutput + angleOutput);

		setRightDrivePower(distOutput - angleOutput);



		//Place mark if we're close enough to the target distance

		if (fabs(targetDistance - distanceElapsed) <= atTargetDistance)

		{

			timer_PlaceHardMarker(&atTargetTimer);

		}

		//Place mark if we haven't moved much

		else if (fabs(distanceElapsed - lastDistance) <= threshold)

		{

			timer_PlaceHardMarker(&atTargetTimer);

		}

		else

		{

			timer_ClearHardMarker(&atTargetTimer);

		}



		lastDistance = distanceElapsed;



		//If we've been close enough for long enough, we're there

		if (timer_GetDTFromHardMarker(&atTargetTimer) >= timeoutPeriod)

		{

			atTarget = true;

		}



		wait1Msec(15);

	}


	setAllMotorPower(0);

	//writeDebugStreamLine("target: %1.2f", targetDistance);

}

void driveForTime(int time, int power)
{
	setLeftDrivePower(power);
	setRightDrivePower(power);
	wait1Msec(time);

	setLeftDrivePower(0);
	setRightDrivePower(0);
}



#endif //DRIVINGFUNCTIONS_C_INCLUDED
