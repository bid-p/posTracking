task trackPos(){

	float chassisWidth = 5;
	float dR;
	float dL;
	float dM;
	float dS;
	float dTheta;
	float avgTheta;
	float dX;
	float dY;

	float lastEncoderValueL = 0;
	float lastEncoderValueR = 0;
	float lastEncoderValueM = 0;
	float currentEncoderValueL = 0;
	float currentEncoderValueR = 0;
	float currentEncoderValueM = 0;

	while(true){

		currentEncoderValueR = SensorValue[rTracker];
		currentEncoderValueL = SensorValue[lTracker];

		// Can multiply by a scalar to get the values in inches, or whatever unit I want x and y coordinates to be in.
		dR = currentEncoderValueR - lastEncoderValueR;
		dL = currentEncoderValueL - lastEncoderValueL;
		dM = currentEncoderValueM - lastEncoderValueM;

		dR *= ((2.875 * PI) / 360);
		dL *= ((2.875 * PI) / 360);
		dM *= ((2.875 * PI) / 360);

		dTheta = (dR - dL) / chassisWidth;  // Change in angle robot turned since last checked.
		dS *= 2.0 * sin(dTheta / 2.0) / dTheta;  // arc based distance robot traveled since last checked.

		avgTheta = theta + dTheta / 2.0;  // Angle robot is assumed to have been facing when moving dS.

		// Change in x and y position since last checked.
		dX = dS * cos(avgTheta) + dM * sin(avgTheta);
		dY = dS * sin(avgTheta) - dM * cos(avgTheta);

		// Update current robot position.
		xPos += dX;
		yPos += dY;
		theta += dTheta;

		lastEncoderValueL = currentEncoderValueL;
		lastEncoderValueR = currentEncoderValueR;
		lastEncoderValueM = currentEncoderValueM;

		writeDebugStreamLine("X = %f   Y = %f   Heading = %f", xPos, yPos, theta);

		delay(5);
	}

}

void moveTo(int targetX, int targetY, float breakThreshold){

	float distFromTarget; //current distanct from target
	float angleFromTarget; //current angle distance from target (in rads)

	float drivePower = 0; //amount of power to be supplied to drive in the Y direction
	float turnPower = 0; //amount of power to be supplied to drive to turn

	float highPass = 50; //constant used to limit turnPower in the high pass filter

	struct PID drivePID;
	initPIDStruct(&drivePID, 0, 0, 0);
	struct PID turnPID;
	initPIDStruct(&turnPID, 0, 0, 0);

	do {

		angleFromTarget = atan2((targetY - yPos),(targetX - xPos)) - theta; //initializes angleFromTarget using the inverse tangent, updates every 10ms
		distFromTarget = sqrt( pow((targetX - xPos),2) + pow((targetY - yPos),2) ); //initializes distFromTarget using distance formula, updates every 10ms

		drivePower = calcPID(&drivePID, distFromTarget, 0); //runs the calcPID function to calc how much power to move by when PID is fed distFromTarget and 0, the goal for distFromTarget
		turnPower = calcPID(&turnPID, angleFromTarget, 0); //runs the calcPID function to calc how much power to move by when PID is fed angleFromTarget and 0, the goal for angleFromTarget

		if(turnPower > highPass){ //high pass filter biases the drive so that it turns and drive to target at the same time (curved movement)
			turnPower = highPass;
		}

		powerDrive(drivePower + turnPower, drivePower - turnPower);

		delay(10);

	}
	while (fabs(distFromTarget) < breakThreshold); //while the absolute value of 0 - distFromTarget is greater than the threshold...
	//threshold is value (in ticks) used to cushion how far away from the target is enough to break from the loop
}
