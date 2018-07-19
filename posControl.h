task trackPos(){

	float chassisWidth /* = */ ;
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

void moveTo(int targetX, int targetY){

	float distFromTarget;
	float angleFromTarget;

	float drivePower = 0;
	float turnPower = 0;

	float highPass = 50;
	float breakThreshold = 5;

	struct PID drivePID;
	initPIDStruct(&drivePID, 0, 0, 0);
	struct PID turnPID;
	initPIDStruct(&turnPID, 0, 0, 0);

	do {

		angleFromTarget = atan2((targetY - yPos),(targetX - xPos)) - theta;
		distFromTarget = sqrt( pow((targetX - xPos),2) + pow((targetY - yPos),2) );

		drivePower = calcPID(&drivePID, distFromTarget, 0);
		turnPower = calcPID(&turnPID, angleFromTarget, 0);

		if(turnPower > highPass){
			turnPower = highPass;
		}

		powerDrive(drivePower + turnPower, drivePower - turnPower);

		delay(10);

	}
	while (fabs(0 - distFromTarget) > breakThreshold);

}
